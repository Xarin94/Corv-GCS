/**
 * TerrainManager.js - Terrain Loading and Chunk Management
 * Handles HGT file loading, terrain chunk generation, and elevation queries
 */

import { VISIBILITY_RADIUS, RELOAD_DISTANCE, CAMERA_FOV } from '../core/constants.js';
import { decodeHgtCooperatively } from './HgtDecoder.js';

// ============== SATELLITE TEXTURE RESOLUTION BANDS ==============
// Texture detail follows distance instead of a single HD/standard split. Each band
// pairs a tile zoom with the largest canvas that band is allowed to allocate, and
// the two are chosen to match: a chunk spans ~1/CHUNKS_PER_TILE_AXIS of a degree,
// so the pixels a band needs stay just under its cap and the fallback loop in
// createChunkTexture() almost never has to step the zoom down.
//
// Why the cap matters more than the zoom: before this table every chunk could
// allocate up to 8192², i.e. 256 MB of RGBA plus mips, and four of them were
// enough to put ~284 MB of texture in VRAM. Now a chunk at 20 km costs 256².
const ZOOM_BANDS = [
    // A near chunk needs ~4900 px at zoom 17 and steps down to 16 (1.6 m/px);
    // SatelliteDetail.js draws zoom 17 to 20 around the aircraft over it
    { maxDist: 3000,     zoom: 17, maxDim: 4096 },
    { maxDist: 6000,     zoom: 16, maxDim: 2048 },
    { maxDist: 12000,    zoom: 15, maxDim: 1024 },
    { maxDist: 22000,    zoom: 14, maxDim: 512 },
    { maxDist: Infinity, zoom: 13, maxDim: 256 },
];
const BASE_BAND = 2;        // zoom used for the first pass, before the aircraft has a position
// Terrain radius of the schematic view (satellite imagery off). Chunks exist
// out to VISIBILITY_RADIUS around the aircraft, which leaves room for the
// chase camera to sit up to 2.5 km away from it.
export const SCHEMATIC_RADIUS = 30000;
const TILE_ZOOM = ZOOM_BANDS[BASE_BAND].zoom;
// Absolute ceiling, clamped to the GPU's real maxTextureSize in initTerrain() so
// weak GPUs (4096 limit) degrade instead of crashing.
let MAX_CANVAS_DIM = 4096;
const SATELLITE_RADIUS = 10000; // 10km - raggio della mappa satellitare (in metri)

/** Band index for a distance in metres. */
function bandForDistance(dist) {
    for (let i = 0; i < ZOOM_BANDS.length; i++) {
        if (dist <= ZOOM_BANDS[i].maxDist) return i;
    }
    return ZOOM_BANDS.length - 1;
}
import { STATE } from '../core/state.js';
import { latLonToMeters, calculateDistance, latLonToTile, tileToBounds } from '../core/utils.js';
import { LRUCache } from '../core/LRUCache.js';
import { getTile as getCachedTile, putTile as putCachedTile } from '../maps/TileCache.js';
import { renderWorld } from '../render/RenderState.js';
import { TERRAIN_GRID } from '../render/TerrainData.js';

// ============== MEMORY TRACKING ==============
let texturesCreated = 0;
let texturesDisposed = 0;
let canvasesCreated = 0;
let canvasesReleased = 0;
let chunksCreated = 0;
let chunksDisposed = 0;

export function getMemoryStats() {
    const gpuStats = terrainView?.getStats();
    return {
        texturesCreated,
        texturesDisposed,
        texturesActive: texturesCreated - texturesDisposed,
        canvasesCreated,
        canvasesReleased,
        canvasesActive: canvasesCreated - canvasesReleased,
        chunksCreated,
        chunksDisposed,
        chunksActive: chunksCreated - chunksDisposed,
        imageLRUSize: imageLRU.size(),
        tileDrawQueueLen: tileDrawQueue.length,
        textureApplyQueueLen: textureApplyQueue.length,
        activeChunkJobsCount: activeChunkJobs.size,
        pendingTileCallbacksCount: pendingTileCallbacks.size,
        heightLayers: gpuStats?.heightLayers || 0,
        heightMB: gpuStats?.heightMB || 0,
        renderData: renderWorld.getStats(),
        compressedTextures: compressedTexturesBuilt,
        compressedMB: +(compressedBytes / 1048576).toFixed(1),
        compressionActive: compressAvailable,
        hgt: {
            tiles: Object.keys(hgtElevationData).length,
            queryMB: +(Object.values(hgtElevationData).reduce((n, t) => n + t.data.byteLength, 0) / 1048576).toFixed(2),
            workerTiles: hgtRegisteredInWorker.size,
            pending: hgtPreparations.size,
            workerDecodes: hgtWorkerDecodes,
            fallbackDecodes: hgtFallbackDecodes,
            lastPrepareMs: +hgtLastPrepareMs.toFixed(2)
        },
        satelliteDetail: gpuStats?.satelliteDetail || null
    };
}

// Terrain data storage
const hgtFiles = {};
const hgtElevationData = {};
const activeChunks = {};
const runwayObjects = [];
let cleanupIntervalId = null;

// Set of HGT filenames available on disk (populated at startup, lazy-loaded on demand)
const availableHgtFiles = new Set();
const hgtLoadingInProgress = new Map(); // filename → promise of the load in flight
const hgtPreparations = new Map(); // tile key → shared decode/registration promise
const hgtWorkerRequests = new Map(); // request id → worker reply callbacks
const hgtReadInProgress = new Set(); // one chunk-enqueue continuation per tile
let hgtRequestId = 0;
let hgtWorkerDecodes = 0;
let hgtFallbackDecodes = 0;
let hgtLastPrepareMs = 0;
let terrainWorkerInitAttempted = false;

// The disk list arrives over IPC after the first terrain updates have run. Until
// then a tile that is on disk looks missing: downloading it then fetched it again
// on every start and overwrote the copy on disk. Auto-download waits for the list.
let hgtListReady = false;
let resolveHgtListReady;
const hgtListReadyPromise = new Promise(r => { resolveHgtListReady = r; });

/**
 * Register which HGT files are available on disk without loading them. Called
 * once at startup, with an empty list when there are none or the lookup failed:
 * that call is what lets missing tiles be downloaded.
 */
export function setAvailableHgtFiles(names) {
    (names || []).forEach(n => availableHgtFiles.add(n.toUpperCase()));
    console.log(`[terrain] ${availableHgtFiles.size} HGT files available on disk (lazy)`);
    hgtListReady = true;
    resolveHgtListReady();
}

/**
 * Lazy-load a single HGT file from disk via IPC if not already loaded. A second
 * caller while the file loads shares that load: answering false made
 * getTerrainElevationAsync take the tile for missing and download it again.
 */
function ensureHgtLoaded(filename) {
    if (hgtFiles[filename]) return Promise.resolve(true);
    if (!availableHgtFiles.has(filename)) return Promise.resolve(false);
    if (!window.topography || !window.topography.loadOne) return Promise.resolve(false);
    let load = hgtLoadingInProgress.get(filename);
    if (!load) {
        load = loadHgtFromDisk(filename);
        hgtLoadingInProgress.set(filename, load);
    }
    return load;
}

async function loadHgtFromDisk(filename) {
    try {
        let ab = await window.topography.loadOne(filename);
        if (!ab) return false;
        if (ab.buffer) ab = ab.buffer; // unwrap if needed
        const file = new File([ab], filename, { type: 'application/octet-stream' });
        if (!await addHGTFile(filename, file)) return false;
        console.log(`[terrain] Lazy-loaded ${filename}`);
        return true;
    } catch (e) {
        console.warn(`[terrain] Failed to lazy-load ${filename}`, e);
        return false;
    } finally {
        hgtLoadingInProgress.delete(filename);
    }
}

// Track tiles that failed auto-download to avoid retrying within a session burst.
// Call resetAutoDownloadFailures() before critical operations (SITL launch,
// mission upload) to allow one more attempt after transient network errors.
const _autoDownloadFailed = new Set();
const _autoDownloadInProgress = new Set();

export function resetAutoDownloadFailures() {
    _autoDownloadFailed.clear();
}

/**
 * Auto-download a single SRTM tile from AWS Mapzen (free, no auth).
 * Downloads gzipped HGT, decompresses, saves to disk via IPC, and registers it.
 */
async function autoDownloadSRTM(filename, latBase, lonBase) {
    if (_autoDownloadInProgress.has(filename)) return null;
    if (!navigator.onLine) return null;

    _autoDownloadInProgress.add(filename);
    try {
        const latPre = latBase >= 0 ? 'N' : 'S';
        const lonPre = lonBase >= 0 ? 'E' : 'W';
        const latNum = String(Math.abs(latBase)).padStart(2, '0');
        const lonNum = String(Math.abs(lonBase)).padStart(3, '0');
        const tileName = `${latPre}${latNum}${lonPre}${lonNum}`;
        const url = `https://elevation-tiles-prod.s3.amazonaws.com/skadi/${tileName.substring(0, 3)}/${tileName}.hgt.gz`;

        console.log(`[terrain] Auto-downloading ${filename} from AWS...`);
        const resp = await fetch(url);
        if (!resp.ok) {
            console.warn(`[terrain] Auto-download failed for ${filename}: ${resp.status}`);
            _autoDownloadFailed.add(filename);
            return null;
        }

        const gzBuf = await resp.arrayBuffer();
        // Decompress gzip in renderer via DecompressionStream
        const ds = new DecompressionStream('gzip');
        const writer = ds.writable.getWriter();
        writer.write(new Uint8Array(gzBuf));
        writer.close();
        const reader = ds.readable.getReader();
        const chunks = [];
        while (true) {
            const { done, value } = await reader.read();
            if (done) break;
            chunks.push(value);
        }
        const totalLen = chunks.reduce((s, c) => s + c.length, 0);
        const hgtBuf = new Uint8Array(totalLen);
        let offset = 0;
        for (const c of chunks) { hgtBuf.set(c, offset); offset += c.length; }

        // Save to disk via IPC
        if (window.topography && window.topography.save) {
            if (await window.topography.save(filename, hgtBuf.buffer)) availableHgtFiles.add(filename);
            else console.warn(`[terrain] ${filename} could not be saved to disk: it will be downloaded again next time`);
        }

        // Register in memory
        const file = new File([hgtBuf.buffer], filename, { type: 'application/octet-stream' });
        if (!await addHGTFile(filename, file)) return null;
        console.log(`[terrain] Auto-downloaded and registered ${filename} (${(totalLen / 1024 / 1024).toFixed(1)} MB)`);
        return file;
    } catch (e) {
        console.warn(`[terrain] Auto-download error for ${filename}:`, e.message);
        _autoDownloadFailed.add(filename);
        return null;
    } finally {
        _autoDownloadInProgress.delete(filename);
    }
}

// Caching
let lastTerrainQuery = { lat: null, lon: null, height: null };

// Texture/Image caches - capacità ridotta per liberare memoria più aggressivamente
const imageLRU = new LRUCache(1500, (img) => {
    // Force garbage collection of image data
    if (img && img.close) {
        try { img.close(); } catch (e) {}
    } else if (img && img.src) {
        img.src = '';
    }
});

// Loading queue system per limitare caricamenti concorrenti
const tileLoadQueue = [];
const MAX_CONCURRENT_TILE_LOADS = 24; // Max tile in download contemporaneo
let currentTileLoads = 0;
let isProcessingTileQueue = false;

// Texture apply queue to avoid main-thread spikes
const textureApplyQueue = [];
let isProcessingTextureQueue = false;
const MAX_TEXTURE_APPLIES_PER_FRAME = 1;
const TEXTURE_APPLY_BUDGET_MS = 3;

// Tile draw queue to avoid main-thread spikes
const tileDrawQueue = [];
let isProcessingTileDrawQueue = false;
const MAX_TILE_DRAWS_PER_FRAME = 6;
const TILE_DRAW_BUDGET_MS = 2;
const MAX_TILE_DRAW_RETRIES = 2;

// Chunk texture creation queue (spread canvas + tile enqueue work)
const chunkTextureQueue = [];
let isProcessingChunkTextureQueue = false;
// Finer chunks mean more of them to texture; 3 per frame keeps the first pass
// under ~8 s without making any single frame expensive.
const MAX_CHUNK_TEXTURES_PER_FRAME = 3;

// Track active chunk jobs for cleanup
const activeChunkJobs = new Map(); // mesh.uuid -> job

// Contatori tile per tracking progresso
let totalTilesToLoad = 0;  // Tile totali da caricare
let tilesLoaded = 0;       // Tile caricate con successo

// Consecutive tile error tracking for connection-loss detection
let consecutiveTileErrors = 0;
const CONSECUTIVE_ERROR_THRESHOLD = 15;
let connectionLostNotified = false;

// Cache-only mode: when the network is down, satellite stays ON so the tiles
// already stored in IndexedDB are used; only the network fetch is skipped.
// Cache misses resolve to null right away and don't count as errors.
let tileNetworkEnabled = true;

export function setTileNetworkEnabled(enabled) {
    tileNetworkEnabled = !!enabled;
    if (tileNetworkEnabled) {
        consecutiveTileErrors = 0;
        connectionLostNotified = false;
        return;
    }
    // Drop anything waiting for the network: nothing will answer it.
    while (tileLoadQueue.length > 0) {
        const item = tileLoadQueue.shift();
        resolveTileCallbacks(item.key, null);
    }
}

export function isTileNetworkEnabled() { return tileNetworkEnabled; }

// Set once the first satellite pass has been scheduled after the base terrain
// is ready; the loading overlay uses it to know the texture phase has begun.
let firstTexturePassStarted = false;

// Chunk creation queue
const CHUNKS_PER_FRAME = 5; // Aumentato per velocizzare
const chunkCreationQueue = [];
let isProcessingChunks = false;

// ============== CHUNK GRANULARITY ==============
// Chunks per axis of a 1° HGT tile. Must divide (size - 1) = 3600 exactly.
//
// This was 10, giving ~11 x 7.5 km chunks. That is coarser than every decision
// made about a chunk: a single distance is used to pick its geometry LOD and its
// texture resolution, yet its near edge could be 11 km closer than its far edge.
// The consequences were an over-detailed geometry band (a chunk touching the 12 km
// ring rendered its whole 11 km at full SRTM1 density) and a texture that had to
// cover 11 km in one image, which is what pushed single textures to 8192² / 256 MB.
// At 30 a chunk is ~3.7 x 2.5 km: LOD and resolution decisions become ~3x sharper,
// and a high-zoom texture for it fits in 4096².
const CHUNKS_PER_TILE_AXIS = 30;

// ============== GEOMETRY LOD ==============
// Screen-space error, not fixed distance rings. A chunk is decimated until the
// spacing between its vertices projects to about LOD_TARGET_PIXELS on screen, so
// triangle density follows apparent size instead of the source data grid.
//
// The near-field floor is set by the data: SRTM1 is a 30 m grid, so rendering it
// undecimated over a 7 km radius is ~360 k triangles no matter how it is chunked.
// Going below that needs a roughness-aware error bound (flat valleys need far fewer
// vertices than ridges), which is the next step and is not implemented here.
const LOD_TARGET_PIXELS = 6;     // allowed screen-space error, in pixels
const SRTM1_SPACING_M = 30.9;    // ground distance between adjacent SRTM1 samples
const LOD_REBUILDS_PER_PASS = 10; // max chunk rebuilds per cleanup pass (5s)

// Updated from the camera on init/resize so the error metric follows the real
// viewport instead of an assumed one.
let lodPixelScale = 60 * Math.PI / 180 / 1080; // ≈ tan(fov/2)*2 / viewportHeight

/**
 * Feed the LOD metric the camera geometry it needs.
 * @param {number} fovDeg vertical field of view
 * @param {number} viewportHeight in device pixels
 */
export function setLodViewParams(fovDeg, viewportHeight) {
    if (!(fovDeg > 0) || !(viewportHeight > 0)) return;
    lodPixelScale = (2 * Math.tan(fovDeg * Math.PI / 360)) / viewportHeight;
}

/**
 * Decimation step for a chunk at `dist` metres.
 * Returns a power of two so it always divides the vertex count evenly.
 */
function lodStepForDistance(dist) {
    // Ground size of one screen pixel at this distance
    const metresPerPixel = Math.max(1e-3, dist) * lodPixelScale;
    const ideal = (LOD_TARGET_PIXELS * metresPerPixel) / SRTM1_SPACING_M;
    if (ideal <= 1) return 1;
    // Round down to a power of two: never coarser than the error budget allows
    const step = 1 << Math.floor(Math.log2(ideal));
    return Math.min(8, step);
}

function sanitizeLodStep(step, vertsPerChunk) {
    return (step > 1 && vertsPerChunk % step === 0) ? step : 1;
}

// Cleanup settings (più aggressivi)
const CLEANUP_RADIUS = VISIBILITY_RADIUS * 1.05; // poco oltre la visibilità
// Chunks resident at once. At 30 per tile axis a chunk is ~9 km², and the
// 35 km visibility disc holds roughly 460 of them, so this is the disc plus
// headroom for the cleanup hysteresis rather than an arbitrary cap.
const MAX_ACTIVE_CHUNKS = 550;
const HGT_CACHE_RADIUS = CLEANUP_RADIUS * 1.2; // raggio cache HGT
// Tiles acknowledged by the terrain worker, which owns its native Int16 grid.
const hgtRegisteredInWorker = new Set();
const WORKER_STALE_MS = 30000; // 30s: worker can be slow on large HGT tiles
const BASE_READY_FORCE_MS = 10000;
let lastChunkActivityTime = performance.now();

// Flag per sapere quando il terreno base è pronto
let terrainBaseReady = false;
// Flag: initial base textures (zoom 15) loaded, HD upgrades now allowed
let initialTexturesLoaded = false;

// Terrain brightness while the sunlight is off (the MAP BRIGHTNESS slider)
let mapBrightness = 0.85;

// Scene reference (set during init)
let sceneRef = null;
let terrainView = null;
let chunksVisible = true;
const TRI_GRID_CELL_M = TERRAIN_GRID.cellM;
const TRI_GRID_RADIUS_M = TERRAIN_GRID.radiusM;
const TRI_GRID_FULL_AGL = TERRAIN_GRID.fullAgl;
const TRI_GRID_MAX_AGL = TERRAIN_GRID.maxAgl;
let rendererRef = null;

// Worker-based chunk generation (optional)
const USE_TERRAIN_WORKER = true;
const MAX_WORKER_INFLIGHT = 8;
let terrainWorker = null;
let workerAvailable = false;
let workerInflight = 0;
const workerPending = new Map();

// Worker-based tile streaming (optional)
const USE_TILE_WORKER = true;
let tileWorker = null;
let tileWorkerAvailable = false;
const pendingTileCallbacks = new Map();

// Worker-based texture culling (optional)
const USE_TEXTURE_CULL_WORKER = true;
let textureCullWorker = null;
let textureCullWorkerAvailable = false;
let textureCullInFlight = false;

function markChunkActivity() {
    lastChunkActivityTime = performance.now();
}

function getChunkDistanceToPlayer(item) {
    const chunksPerAxis = CHUNKS_PER_TILE_AXIS;
    const centerLat = item.latBase + 1 - ((item.cy + 0.5) / chunksPerAxis);
    const centerLon = item.lonBase + ((item.cx + 0.5) / chunksPerAxis);
    const centerWorld = latLonToMeters(centerLat, centerLon);
    const playerPos = latLonToMeters(STATE.lat, STATE.lon);
    const dx = centerWorld.x - playerPos.x;
    const dz = centerWorld.z - playerPos.z;
    return Math.sqrt(dx * dx + dz * dz);
}

function isChunkInRange(item, radius = VISIBILITY_RADIUS) {
    return getChunkDistanceToPlayer(item) <= radius;
}

/** Int16 elevation samples of one chunk from the parsed HGT tile. */
function sampleChunkHeights(item, step) {
    const { cx, cy, size, vertsPerChunk, hgtKey } = item;
    const cache = hgtElevationData[hgtKey];
    const geoW = vertsPerChunk / step + 1;
    const heights = new Int16Array(geoW * geoW);
    let minH = Infinity, maxH = -Infinity;
    if (cache) {
        const startRow = cy * vertsPerChunk;
        const startCol = cx * vertsPerChunk;
        for (let r = 0; r < geoW; r++) {
            const row = (startRow + r * step) * size;
            for (let c = 0; c < geoW; c++) {
                const h = cache.data[row + startCol + c * step];
                heights[r * geoW + c] = h;
                if (h < minH) minH = h;
                if (h > maxH) maxH = h;
            }
        }
    }
    if (minH > maxH) { minH = 0; maxH = 0; }
    return { heights, minH, maxH };
}

// GPU resource creation and batching belong to the selected backend.
function setChunkHeights(mesh, item, step, heights, minH, maxH) {
    const record = renderWorld.terrain.put(item, step, heights, minH, maxH);
    terrainView.updateHeights(mesh, record);
}
export function updateTerrainInstances(camera) {
    terrainView?.updateFrame(renderWorld.camera, { camera, aircraft: STATE,
        pixelAngle: lodPixelScale, satelliteEnabled: !!window.satelliteEnabled });
}

/**
 * Initialize terrain manager
 * @param {THREE.Scene} scene
 * @param {THREE.WebGLRenderer} renderer
 * @param {THREE.Vector3} sunDirection
 */
export function initTerrain(scene, renderer, sunDirection, backend) {
    sceneRef = scene;
    rendererRef = renderer;

    // Clamp the texture cap to the GPU's real limit so we never allocate a
    // canvas larger than the hardware can upload as a texture.
    const gpuMaxTexture = renderer?.capabilities?.maxTextureSize;
    if (gpuMaxTexture > 0) {
        MAX_CANVAS_DIM = Math.min(MAX_CANVAS_DIM, gpuMaxTexture);
    }
    if (!backend?.createTerrainView) throw new Error('Terrain requires a rendering backend');
    terrainView?.dispose();
    terrainView = backend.createTerrainView({ scene, world: renderWorld, loadTileImage,
        onTextureDisposed: () => { texturesDisposed++; } });
    if (sunDirection) renderWorld.terrainStyle.sunDirection.splice(0, 3, sunDirection.x, sunDirection.y, sunDirection.z);

    initTerrainWorker();
    initTileWorker();
    initTextureCullWorker();
    initCompressWorkers();

    // Feed the LOD metric the real camera geometry (falls back to a 60° / 1080p
    // assumption if either is missing).
    setLodViewParams(CAMERA_FOV, renderer?.domElement?.height);

    // Start cleanup interval (store handle for potential cleanup)
    if (cleanupIntervalId) clearInterval(cleanupIntervalId);
    cleanupIntervalId = setInterval(cleanupDistantChunks, 5000);
}

function initTerrainWorker() {
    if (terrainWorkerInitAttempted) return;
    terrainWorkerInitAttempted = true;
    if (!USE_TERRAIN_WORKER || typeof Worker === 'undefined') return;

    try {
        terrainWorker = new Worker(new URL('./TerrainWorker.js', import.meta.url), { type: 'module' });
        workerAvailable = true;
        hgtRegisteredInWorker.clear(); // fresh worker holds no tiles

        terrainWorker.onmessage = (e) => {
            const data = e.data || {};
            if (data.type === 'hgtReady' || data.type === 'hgtFailed') {
                const request = hgtWorkerRequests.get(data.id);
                if (!request || request.key !== data.key) return;
                hgtWorkerRequests.delete(data.id);
                clearTimeout(request.timer);
                if (data.type === 'hgtFailed') {
                    request.reject(new Error(data.reason));
                } else {
                    // Registration is acknowledged before any buildChunk is sent.
                    hgtRegisteredInWorker.add(data.key);
                    hgtWorkerDecodes++;
                    hgtLastPrepareMs = data.prepareMs;
                    request.resolve({ data: data.elevations, size: data.size });
                }
                return;
            }
            if (data.type === 'chunkBuilt') {
                const item = workerPending.get(data.chunkKey);
                if (!item) return;
                workerPending.delete(data.chunkKey);
                workerInflight = Math.max(0, workerInflight - 1);
                markChunkActivity();

                const existing = activeChunks[data.chunkKey];
                if (item.lodRebuild) {
                    // LOD rebuild: the chunk keeps its mesh and its satellite
                    // texture (same area, same UV layout); only its elevation
                    // layer and grid change.
                    if (existing) {
                        setChunkHeights(existing, item, data.step, data.heights, data.minH, data.maxH);
                        existing.data.lodRebuildQueued = false;
                    }
                    return;
                }
                if (!existing && isChunkInRange(item)) {
                    addChunkMesh(item, data.step, data.heights, data.minH, data.maxH);
                    noteChunkCreated(true);
                }
                return;
            }

            if (data.type === 'chunkFailed') {
                const item = workerPending.get(data.chunkKey);
                if (!item) return;
                workerPending.delete(data.chunkKey);
                workerInflight = Math.max(0, workerInflight - 1);
                markChunkActivity();
                if (item.lodRebuild) {
                    // Keep the current grid; a later pass may retry
                    if (activeChunks[data.chunkKey]) activeChunks[data.chunkKey].data.lodRebuildQueued = false;
                    return;
                }
                if (!activeChunks[data.chunkKey] && isChunkInRange(item)) {
                    createSingleChunk(item);
                }
            }
        };

        terrainWorker.onerror = (e) => disableTerrainWorker(e?.message || 'unknown error');
        terrainWorker.onmessageerror = () => disableTerrainWorker('Unreadable worker reply');
    } catch (err) {
        workerAvailable = false;
        terrainWorker = null;
    }
}

function disableTerrainWorker(reason) {
    console.warn(`[terrain] TerrainWorker unavailable: ${reason}; using cooperative HGT decoding`);
    workerAvailable = false;
    terrainWorker?.terminate();
    terrainWorker = null;
    hgtRegisteredInWorker.clear();
    for (const request of hgtWorkerRequests.values()) {
        clearTimeout(request.timer);
        request.reject(Object.assign(new Error(reason), { workerUnavailable: true }));
    }
    hgtWorkerRequests.clear();
    for (const item of workerPending.values()) {
        if (!activeChunks[item.chunkKey] || item.lodRebuild) chunkCreationQueue.unshift(item);
    }
    workerPending.clear();
    workerInflight = 0;
    if (!isProcessingChunks && chunkCreationQueue.length > 0) processChunkQueue();
}

/** All import, disk, download and elevation-query paths share one preparation. */
function prepareHgt(file, key) {
    initTerrainWorker();
    const cached = hgtElevationData[key];
    if (cached && (!workerAvailable || hgtRegisteredInWorker.has(key))) return Promise.resolve(cached);
    if (hgtPreparations.has(key)) return hgtPreparations.get(key);
    const job = (async () => {
        let tile;
        if (workerAvailable && terrainWorker) {
            try {
                tile = await new Promise((resolve, reject) => {
                    const id = ++hgtRequestId;
                    const timer = setTimeout(() => disableTerrainWorker('HGT preparation timed out'), 60000);
                    hgtWorkerRequests.set(id, { key, resolve, reject, timer });
                    try {
                        terrainWorker.postMessage({ type: 'prepareHgt', id, key, file });
                    } catch (error) {
                        disableTerrainWorker(error.message);
                    }
                });
            } catch (error) {
                if (!error.workerUnavailable) throw error; // invalid HGT is not retried
            }
        }
        if (!tile) {
            const start = performance.now();
            tile = cached || await decodeHgtCooperatively(await file.arrayBuffer());
            if (!cached) hgtFallbackDecodes++;
            hgtLastPrepareMs = performance.now() - start;
        }
        hgtElevationData[key] = tile;
        lastTerrainQuery = { lat: null, lon: null, height: null };
        return tile;
    })().finally(() => hgtPreparations.delete(key));
    hgtPreparations.set(key, job);
    return job;
}

function initTileWorker() {
    if (!USE_TILE_WORKER || typeof Worker === 'undefined') return;

    try {
        tileWorker = new Worker(new URL('./TileWorker.js', import.meta.url), { type: 'module' });
        tileWorkerAvailable = true;

        tileWorker.onmessage = (e) => {
            const data = e.data || {};
            if (data.type === 'tileLoaded') {
                if (data.bitmap) {
                    imageLRU.set(data.key, data.bitmap);
                    consecutiveTileErrors = 0; // Reset on success
                }
                // Opportunistic cache: store blob in IndexedDB for offline use
                if (data.blob && data.key) {
                    const parts = data.key.split('/');
                    if (parts.length === 3) {
                        putCachedTile('esri', parseInt(parts[0]), parseInt(parts[1]), parseInt(parts[2]), data.blob).catch(() => {});
                    }
                }
                resolveTileCallbacks(data.key, data.bitmap || null);
                currentTileLoads = Math.max(0, currentTileLoads - 1);
                processTileLoadQueue();
                return;
            }

            if (data.type === 'tileError') {
                consecutiveTileErrors++;
                if (consecutiveTileErrors >= CONSECUTIVE_ERROR_THRESHOLD && !connectionLostNotified) {
                    connectionLostNotified = true;
                    console.warn(`${CONSECUTIVE_ERROR_THRESHOLD} consecutive tile errors — connection lost, satellite from cache only`);
                    setTileNetworkEnabled(false);
                    window.dispatchEvent(new CustomEvent('connectionLost'));
                }
                resolveTileCallbacks(data.key, null);
                currentTileLoads = Math.max(0, currentTileLoads - 1);
                processTileLoadQueue();
            }
        };

        tileWorker.onerror = (e) => {
            console.error(`[terrain] TileWorker failed: ${e?.message || 'unknown'} ${e?.filename ? `(${e.filename}:${e.lineno})` : ''}`);
            tileWorkerAvailable = false;
            tileWorker = null;
            pendingTileCallbacks.clear();
        };
    } catch (err) {
        tileWorkerAvailable = false;
        tileWorker = null;
    }
}

function initTextureCullWorker() {
    if (!USE_TEXTURE_CULL_WORKER || typeof Worker === 'undefined') return;

    try {
        textureCullWorker = new Worker(new URL('./TextureCullWorker.js', import.meta.url), { type: 'module' });
        textureCullWorkerAvailable = true;

        textureCullWorker.onmessage = (e) => {
            const data = e.data || {};
            if (data.type !== 'texturesToUnload') return;

            textureCullInFlight = false;
            const keys = data.keys || [];
            for (const key of keys) {
                const mesh = activeChunks[key];
                if (mesh) {
                    unloadChunkTexture(mesh);
                }
            }
        };

        textureCullWorker.onerror = (e) => {
            console.error(`[terrain] TextureCullWorker failed: ${e?.message || 'unknown'} ${e?.filename ? `(${e.filename}:${e.lineno})` : ''}`);
            textureCullWorkerAvailable = false;
            textureCullWorker = null;
            textureCullInFlight = false;
        };
    } catch (err) {
        textureCullWorkerAvailable = false;
        textureCullWorker = null;
    }
}

// ============== TEXTURE COMPRESSION (BC1) ==============
// Satellite chunk textures are compressed to BC1 before they reach the GPU. An
// RGBA8 chunk texture with mips costs 5.33 bytes per pixel; BC1 costs 0.67. With
// ~460 chunks resident the uncompressed path would need several hundred MB of
// VRAM, which is what the resolution bands alone could not fix.
const COMPRESS_WORKER_COUNT = 2;
const compressWorkers = [];
let compressAvailable = false;
let compressSeq = 0;
let compressRoundRobin = 0;
const compressPending = new Map(); // id -> { mesh, canvas }
let compressedTexturesBuilt = 0;
let compressedBytes = 0;

function initCompressWorkers() {
    if (typeof Worker === 'undefined' || !terrainView) return;

    // BC1 needs the S3TC extension. Everything desktop has it; if it is missing
    // the RGBA path still works, just with the old memory cost.
    const ext = terrainView.capabilities.bc1;
    if (!ext) {
        console.warn('[terrain] S3TC unavailable — terrain textures stay uncompressed');
        return;
    }

    try {
        for (let i = 0; i < COMPRESS_WORKER_COUNT; i++) {
            const w = new Worker(new URL('./TextureCompressWorker.js', import.meta.url), { type: 'module' });
            w.onmessage = (e) => onCompressedTexture(e.data || {});
            w.onerror = (e) => {
                console.error(`[terrain] TextureCompressWorker failed: ${e?.message || 'unknown'} — textures uncompressed`);
                compressAvailable = false;
            };
            compressWorkers.push(w);
        }
        compressAvailable = true;
    } catch (err) {
        compressAvailable = false;
        console.warn('[terrain] Texture compression worker unavailable:', err.message);
    }
}

/**
 * Hand a finished chunk canvas to a compression worker.
 *
 * The canvas is a CPU-backed OffscreenCanvas (see createChunkTexture), so
 * transferToImageBitmap() hands its pixels over without a copy and the worker
 * reads them back itself. getImageData() here used to do that readback on the
 * main thread — from a GPU-backed canvas, a synchronous GPU→CPU copy of up to
 * 64 MB per chunk.
 * @returns {boolean} false if the caller should fall back to an RGBA texture
 */
function requestCompressedTexture(mesh, canvas) {
    if (!compressAvailable || !compressWorkers.length) return false;
    // Below one block per axis there is nothing to gain and the padding would
    // dominate; those textures are negligible anyway.
    if (canvas.width < 8 || canvas.height < 8) return false;
    // A DOM canvas (built while compression was unavailable) takes the RGBA path
    if (typeof canvas.transferToImageBitmap !== 'function') return false;

    const width = canvas.width, height = canvas.height;
    let bitmap;
    try {
        bitmap = canvas.transferToImageBitmap();
    } catch (e) {
        return false;
    }
    // Unlike a CanvasTexture, a compressed texture owns its pixels: the staging
    // canvas is done.
    releaseCanvas(canvas);

    const id = ++compressSeq;
    compressPending.set(id, { mesh });
    const worker = compressWorkers[compressRoundRobin++ % compressWorkers.length];
    worker.postMessage({ id, bitmap, width, height }, [bitmap]);
    return true;
}

function onCompressedTexture(msg) {
    const pending = compressPending.get(msg.id);
    if (!pending) return;
    compressPending.delete(msg.id);

    const { mesh } = pending;
    const width = msg.width, height = msg.height;

    if (!msg.ok || !mesh || (mesh.data && mesh.data.disposed) || !window.satelliteEnabled) {
        if (!msg.ok) console.warn('[terrain] Texture compression failed:', msg.error);
        if (mesh && mesh.data && !msg.ok) mesh.data.textureLoaded = false;
        return;
    }

    const texture = terrainView.makeCompressedTexture(msg);

    texturesCreated++;
    compressedTexturesBuilt++;
    for (const m of msg.mips) compressedBytes += m.data.length;

    attachTextureToMesh(mesh, texture);
}

function releaseCanvas(canvas) {
    if (!canvas) return;
    canvas.width = 1;
    canvas.height = 1;
    canvasesReleased++;
}

/**
 * Get terrain elevation from HGT data
 * @param {number} lat - Latitude
 * @param {number} lon - Longitude
 * @returns {number|null} Elevation in meters or null
 */
const HGT_VOID = -12000;   // at or below: no data (SRTM voids are -32768)

export function getTerrainElevationFromHGT(lat, lon) {
    return sampleHgt(lat, lon, false);
}

/**
 * Elevation for the vehicle's own terrain database (MAVLink terrain feed):
 * null unless the tile is loaded and all four samples around the point are
 * real. The vehicle flies on what it is sent, so "unknown" must stay unknown
 * instead of reading as sea level.
 * @returns {number|null} metres AMSL
 */
export function getTerrainElevationChecked(lat, lon) {
    return sampleHgt(lat, lon, true);
}

function sampleHgt(lat, lon, strict) {
    const latBase = Math.floor(lat);
    const lonBase = Math.floor(lon);
    const key = `${latBase}_${lonBase}`;
    const cached = hgtElevationData[key];

    if (!cached) return null;

    const { data, size } = cached;
    const latFrac = lat - latBase;
    const lonFrac = lon - lonBase;
    const row = (1.0 - latFrac) * (size - 1);
    const col = lonFrac * (size - 1);
    const r0 = Math.floor(row);
    const r1 = Math.min(r0 + 1, size - 1);
    const c0 = Math.floor(col);
    const c1 = Math.min(c0 + 1, size - 1);
    const fr = row - r0;
    const fc = col - c0;

    const h00 = data[r0 * size + c0];
    const h01 = data[r0 * size + c1];
    const h10 = data[r1 * size + c0];
    const h11 = data[r1 * size + c1];

    // Voids are SRTM's -32768; anything above is real, down to ocean trench depths
    if (strict && (h00 <= HGT_VOID || h01 <= HGT_VOID || h10 <= HGT_VOID || h11 <= HGT_VOID)) return null;
    if (h00 <= HGT_VOID) return 0;

    const h0 = h00 * (1 - fc) + h01 * fc;
    const h1 = h10 * (1 - fc) + h11 * fc;
    return h0 * (1 - fr) + h1 * fr;
}

// Samples around a point that must all equal it for the point to be water:
// the 3 x 3 block and the four at distance 2 (the terrain shader tests the same).
const WATER_FLAT_OFFSETS = [[0, 1], [0, -1], [1, 0], [-1, 0], [1, 1], [1, -1], [-1, 1], [-1, -1], [0, 2], [0, -2], [2, 0], [-2, 0]];

/**
 * Water under a point, from the elevation data alone. SRTM flattens every lake
 * to a single height and the sea to 0 — ground is never flat to the metre
 * across a dozen samples, so that marks a water surface. Below 0 the tiles
 * carry bathymetry: the sea bed, with the surface at 0.
 * Not handled: land below sea level (Dead Sea shore, polders) reads as sea,
 * and the Caspian Sea as a surface at 0 instead of −28 m.
 * @returns {{level:number, bed:number|null}|null} surface height (MSL) and the
 *   bed under it when the data has one; null over land or without data
 */
export function getWaterSurfaceAt(lat, lon) {
    const latBase = Math.floor(lat);
    const lonBase = Math.floor(lon);
    const cached = hgtElevationData[`${latBase}_${lonBase}`];
    if (!cached) return null;
    const { data, size } = cached;
    const r = Math.round((1 - (lat - latBase)) * (size - 1));
    const c = Math.round((lon - lonBase) * (size - 1));
    const at = (dr, dc) => data[Math.min(size - 1, Math.max(0, r + dr)) * size + Math.min(size - 1, Math.max(0, c + dc))];
    const h = at(0, 0);
    if (h <= HGT_VOID) return null;
    if (h < 0) return { level: 0, bed: getTerrainElevationFromHGT(lat, lon) };
    for (const [dr, dc] of WATER_FLAT_OFFSETS) if (at(dr, dc) !== h) return null;
    return { level: h, bed: null };
}

/**
 * A sub in water the elevation data does not show — a flooded quarry, a lake
 * SRTM leaves unflattened: around it, ground at its surface height (within
 * 1.5 m) is drawn and treated as that water, so it does not hide the vehicle.
 * @param {{x:number, z:number, level:number}|null} water world position and
 *   surface height, null to switch it off
 */
export function setTerrainSubWater(water) {
    const u = renderWorld.terrainStyle.subWater;
    if (water) { u[0] = water.x; u[1] = water.z; u[2] = water.level; u[3] = TRI_GRID_RADIUS_M; }
    else u[3] = 0;
}

/** The triangle grid's cell and radius, for the water plane drawn at the same lattice (Water3D). */
export const LOW_ALT_GRID = { cellM: TRI_GRID_CELL_M, radiusM: TRI_GRID_RADIUS_M };

/** Current strength (0..1) of the triangle grid and the point it is centred on. */
export function getLowAltGridState() {
    return { strength: renderWorld.terrainStyle.gridStrength, center: renderWorld.terrainStyle.gridCenter };
}

/**
 * Async version: ensures HGT data is parsed before querying elevation.
 * Use this when you need a guaranteed result (e.g. SITL launch).
 */
export async function getTerrainElevationAsync(lat, lon) {
    const latBase = Math.floor(lat);
    const lonBase = Math.floor(lon);
    const key = `${latBase}_${lonBase}`;

    // If not in cache, try to parse from loaded HGT files
    if (!hgtElevationData[key]) {
        const latPre = lat >= 0 ? 'N' : 'S';
        const lonPre = lon >= 0 ? 'E' : 'W';
        const latNum = String(Math.abs(latBase)).padStart(2, '0');
        const lonNum = String(Math.abs(lonBase)).padStart(3, '0');
        const filename = `${latPre}${latNum}${lonPre}${lonNum}.HGT`;
        // Try from already-loaded files first
        let file = hgtFiles[filename];
        // If not loaded yet, try lazy-load from disk
        if (!file && availableHgtFiles.has(filename)) {
            await ensureHgtLoaded(filename);
            file = hgtFiles[filename];
        }
        // If still not available, auto-download from AWS Mapzen — once the disk
        // list is known, or a tile already on disk would be fetched again
        if (!file && !hgtListReady) {
            await hgtListReadyPromise;
            if (availableHgtFiles.has(filename)) {
                await ensureHgtLoaded(filename);
                file = hgtFiles[filename];
            }
        }
        if (!file && !_autoDownloadFailed.has(filename)) {
            file = await autoDownloadSRTM(filename, latBase, lonBase);
        }
        if (file) {
            await prepareHgt(file, key);
        }
    }

    return getTerrainElevationFromHGT(lat, lon);
}

/**
 * Get terrain elevation with caching
 * @param {number} lat - Latitude
 * @param {number} lon - Longitude
 * @returns {number|null} Elevation
 */
export function getTerrainElevationCached(lat, lon) {
    if (lastTerrainQuery.lat !== null &&
        Math.abs(lat - lastTerrainQuery.lat) < 0.00001 &&
        Math.abs(lon - lastTerrainQuery.lon) < 0.00001) {
        return lastTerrainQuery.height;
    }

    const height = getTerrainElevationFromHGT(lat, lon);
    // If tile is missing, trigger background auto-download for next frame
    if (height === null) {
        getTerrainElevationAsync(lat, lon).catch(() => {});
    }
    lastTerrainQuery = { lat, lon, height };
    return height;
}

/**
 * Add HGT file to storage. Resolves true only after elevation data is ready.
 * @param {string} filename 
 * @param {File} file 
 */
export async function addHGTFile(filename, file) {
    const match = String(filename).toUpperCase().match(/^([NS])(\d{1,2})([EW])(\d{1,3})/);
    if (!match) return false;
    const name = filename.toUpperCase();
    hgtFiles[name] = file;
    try {
        const latSign = match[1] === 'S' ? -1 : 1;
        const lonSign = match[3] === 'W' ? -1 : 1;
        const latBase = latSign * Number(match[2]);
        const lonBase = lonSign * Number(match[4]);
        const key = `${latBase}_${lonBase}`;
        const tile = await prepareHgt(file, key);
        console.log(`[terrain] HGT ready: ${filename} (${tile.size}x${tile.size})`);
        return true;
    } catch (error) {
        if (hgtFiles[name] === file) delete hgtFiles[name];
        console.warn(`[terrain] Failed to prepare ${filename}: ${error.message}`);
        return false;
    }
}

/**
 * Get count of loaded HGT files
 * @returns {number}
 */
export function getHGTFileCount() {
    return Object.keys(hgtFiles).length;
}

/**
 * Get bounds for loaded HGT files (1° x 1° tiles)
 * @returns {Array<{key:string, latTop:number, latBottom:number, lonLeft:number, lonRight:number}>}
 */
export function getHgtFileBounds() {
    const out = [];
    const keys = Object.keys(hgtFiles);
    for (const filename of keys) {
        const match = String(filename).toUpperCase().match(/^([NS])(\d{1,2})([EW])(\d{1,3})/);
        if (!match) continue;
        const latSign = match[1] === 'S' ? -1 : 1;
        const lonSign = match[3] === 'W' ? -1 : 1;
        const latBase = latSign * Number(match[2]);
        const lonBase = lonSign * Number(match[4]);
        if (!Number.isFinite(latBase) || !Number.isFinite(lonBase)) continue;
        out.push({
            key: `${latBase}_${lonBase}`,
            latTop: latBase + 1,
            latBottom: latBase,
            lonLeft: lonBase,
            lonRight: lonBase + 1
        });
    }
    return out;
}

/**
 * Update terrain chunks based on current position
 */
/**
 * Distance (m) from a position to the nearest point of the 1° HGT cell
 * [la, la+1] x [lo, lo+1]. Zero when the position is inside the cell.
 */
function hgtTileDistance(lat, lon, la, lo) {
    const dLat = lat < la ? la - lat : (lat > la + 1 ? lat - (la + 1) : 0);
    const dLon = lon < lo ? lo - lon : (lon > lo + 1 ? lon - (lo + 1) : 0);
    const mLat = dLat * 111320;
    const mLon = dLon * 111320 * Math.cos(lat * Math.PI / 180);
    return Math.sqrt(mLat * mLat + mLon * mLon);
}

export async function updateTerrainChunks() {
    const currentLat = STATE.lat;
    const currentLon = STATE.lon;

    // Only the HGT cells that reach into the chunk visibility radius matter:
    // chunks beyond it are never built, so reading (or worse, downloading)
    // the old ±2° window — up to 25 x 25 MB — just delayed the first terrain.
    // Nearest cell first, so the ground under the aircraft appears first.
    const HGT_MARGIN_M = 5000;
    const candidates = [];
    for (let la = Math.floor(currentLat - 1); la <= Math.floor(currentLat + 1); la++) {
        for (let lo = Math.floor(currentLon - 1); lo <= Math.floor(currentLon + 1); lo++) {
            const dist = hgtTileDistance(currentLat, currentLon, la, lo);
            if (dist <= VISIBILITY_RADIUS + HGT_MARGIN_M) candidates.push({ la, lo, dist });
        }
    }
    candidates.sort((a, b) => a.dist - b.dist);

    let loaded = 0, lazy = 0, missing = 0;
    const missingFiles = [];
    for (const { la, lo } of candidates) {
        const latStr = (la >= 0 ? 'N' : 'S') + Math.abs(la).toString().padStart(2, '0');
        const lonStr = (lo >= 0 ? 'E' : 'W') + Math.abs(lo).toString().padStart(3, '0');
        const filename = `${latStr}${lonStr}.HGT`;
        if (hgtFiles[filename]) {
            loaded++;
            processHGTFile(hgtFiles[filename], la, lo);
        } else if (availableHgtFiles.has(filename)) {
            lazy++;
            // Lazy-load from disk, then process
            ensureHgtLoaded(filename).then(ok => {
                if (ok && hgtFiles[filename]) {
                    processHGTFile(hgtFiles[filename], la, lo);
                }
            });
        } else {
            missing++;
            missingFiles.push({ filename, latBase: la, lonBase: lo });
        }
    }

    // Auto-download missing HGT tiles that cover the visible area.
    // Rate-limit to avoid saturating the network: max 2 downloads in flight.
    const MAX_DOWNLOADS_PER_CALL = 2;
    let started = 0;
    // Before the disk list arrives every tile looks missing: wait for it
    // (loadTopographyAtStart runs this again once it is in)
    if (!hgtListReady) missingFiles.length = 0;
    for (const { filename, latBase, lonBase } of missingFiles) {
        if (started >= MAX_DOWNLOADS_PER_CALL) break;
        if (_autoDownloadInProgress.has(filename)) continue;
        if (_autoDownloadFailed.has(filename)) continue; // already failed once this session
        started++;
        autoDownloadSRTM(filename, latBase, lonBase).then(file => {
            if (file && hgtFiles[filename]) {
                processHGTFile(hgtFiles[filename], latBase, lonBase);
            }
        }).catch(() => {});
    }

    console.debug(`[terrain] updateTerrainChunks: loaded=${loaded} lazy=${lazy} missing=${missing} downloadsStarted=${started} active=${Object.keys(activeChunks).length} queue=${chunkCreationQueue.length}`);
}

/**
 * Process HGT file and generate chunks
 * @param {File} file 
 * @param {number} latBase 
 * @param {number} lonBase 
 */
function processHGTFile(file, latBase, lonBase) {
    const key = `${latBase}_${lonBase}`;
    const cached = hgtElevationData[key];

    // Fast path. Once a tile is parsed and handed to the worker there is nothing
    // left to extract from the file, only chunks left to enqueue. Re-reading a
    // 25 MB HGT through FileReader — for all 25 resident tiles, every refresh —
    // is what made the periodic pass hitch once a second.
    if (cached && (!workerAvailable || hgtRegisteredInWorker.has(key))) {
        enqueueChunksForTile(latBase, lonBase, cached.size);
        return;
    }

    // Share the preparation with imports and elevation queries. Enqueue only
    // after the worker has acknowledged that its native grid is available.
    if (hgtReadInProgress.has(key)) return;
    hgtReadInProgress.add(key);

    prepareHgt(file, key).then(tile => enqueueChunksForTile(latBase, lonBase, tile.size))
        .catch(error => console.warn(`[terrain] HGT ${key}: ${error.message}`))
        .finally(() => hgtReadInProgress.delete(key));
}

/**
 * Queue every not-yet-built chunk of one 1°x1° tile that falls inside the
 * visibility disc. Safe to call repeatedly: it reads the cached elevation data
 * and touches neither the file nor the worker registration.
 * @param {number} latBase
 * @param {number} lonBase
 * @param {number} size samples per tile axis
 */
function enqueueChunksForTile(latBase, lonBase, size) {
    const key = `${latBase}_${lonBase}`;
    const chunksPerAxis = CHUNKS_PER_TILE_AXIS;
    const vertsPerChunk = Math.floor((size - 1) / chunksPerAxis);
    const playerPos = latLonToMeters(STATE.lat, STATE.lon);

    // The tile scan reaches +-2 degrees but the visibility disc is ~0.3 degrees, so
    // most tiles cannot contribute a single chunk. Reject them on their nearest
    // corner instead of testing 900 chunk centres each.
    const nearLat = Math.min(Math.max(STATE.lat, latBase), latBase + 1);
    const nearLon = Math.min(Math.max(STATE.lon, lonBase), lonBase + 1);
    const nearWorld = latLonToMeters(nearLat, nearLon);
    if (Math.hypot(nearWorld.x - playerPos.x, nearWorld.z - playerPos.z) > VISIBILITY_RADIUS) return;

    const chunksList = [];
    for (let cx = 0; cx < chunksPerAxis; cx++) {
        for (let cy = 0; cy < chunksPerAxis; cy++) {
            const chunkKey = `${latBase}_${lonBase}_${cx}_${cy}`;
            if (activeChunks[chunkKey]) continue;

            const chunkLatCenter = latBase + 1 - ((cy + 0.5) / chunksPerAxis);
            const chunkLonCenter = lonBase + (cx + 0.5) / chunksPerAxis;
            const centerWorld = latLonToMeters(chunkLatCenter, chunkLonCenter);
            const dist = Math.sqrt(
                (centerWorld.x - playerPos.x) ** 2 + 
                (centerWorld.z - playerPos.z) ** 2
            );
            
            if (dist <= VISIBILITY_RADIUS) {
                chunksList.push({
                    cx, cy, dist, chunkKey,
                    latBase, lonBase, size, vertsPerChunk,
                    lodStep: sanitizeLodStep(lodStepForDistance(dist), vertsPerChunk),
                    hgtKey: key,
                    // The worker reads its own registered copy and createSingleChunk()
                    // falls back to hgtElevationData[hgtKey], which is populated before
                    // we get here — so neither path needs a per-chunk buffer copy.
                    dataView: null
                });
            }
        }
    }

    chunksList.sort((a, b) => a.dist - b.dist);

    for (const chunkData of chunksList) {
        if (!chunkCreationQueue.some(q => q.chunkKey === chunkData.chunkKey)) {
            chunkCreationQueue.push(chunkData);
        }
    }

    if (!isProcessingChunks && chunkCreationQueue.length > 0) {
        processChunkQueue();
    }
}

/**
 * Process chunk creation queue progressively
 */
function processChunkQueue() {
    const now = performance.now();
    if (workerAvailable && workerPending.size > 0) {
        for (const [chunkKey, item] of workerPending) {
            const requestedAt = item.requestedAt || 0;
            if (requestedAt && now - requestedAt > WORKER_STALE_MS) {
                console.warn(`[terrain] Worker stale for chunk ${chunkKey}, re-queueing`);
                workerPending.delete(chunkKey);
                workerInflight = Math.max(0, workerInflight - 1);
                if ((!activeChunks[chunkKey] || item.lodRebuild) && isChunkInRange(item)) {
                    chunkCreationQueue.unshift(item);
                } else if (item.lodRebuild && activeChunks[chunkKey]) {
                    // Rebuild dropped — release the flag so a later pass can retry
                    activeChunks[chunkKey].data.lodRebuildQueued = false;
                }
            }
        }
    }

    if (workerInflight > workerPending.size) {
        workerInflight = workerPending.size;
    }

    if (chunkCreationQueue.length === 0) {
        if (workerAvailable && workerInflight > 0) {
            requestAnimationFrame(processChunkQueue);
            return;
        }
        isProcessingChunks = false;
        
        // Terreno base completato - ora carica satellite se abilitato
        if (!terrainBaseReady && Object.keys(activeChunks).length > 0) {
            terrainBaseReady = true;
            console.log(`[terrain] Base terrain ready: ${Object.keys(activeChunks).length} chunks active`);
            
            // Avvia caricamento satellite dopo un breve delay
            if (window.satelliteEnabled) {
                setTimeout(() => {
                    resetTextureRefreshPosition();
                    refreshNearbyChunkTextures();
                    firstTexturePassStarted = true;
                }, 100);
            } else {
                firstTexturePassStarted = true;
            }
        } else if (terrainBaseReady && window.satelliteEnabled) {
            // A later batch drained (HGT tiles arrive one at a time at
            // startup): texture the new chunks now instead of waiting for the
            // movement/30 s gate of the periodic refresh.
            resetTextureRefreshPosition();
            refreshNearbyChunkTextures();
        }
        return;
    }

    isProcessingChunks = true;

    if (workerAvailable && terrainWorker) {
        let scheduled = 0;
        while (scheduled < CHUNKS_PER_FRAME && chunkCreationQueue.length > 0 && workerInflight < MAX_WORKER_INFLIGHT) {
            const item = chunkCreationQueue.shift();
            if (!activeChunks[item.chunkKey] || item.lodRebuild) {
                if (!isChunkInRange(item)) {
                    continue;
                }
                item.requestedAt = performance.now();
                workerPending.set(item.chunkKey, item);
                workerInflight++;
                terrainWorker.postMessage({
                    type: 'buildChunk',
                    chunkKey: item.chunkKey,
                    hgtKey: item.hgtKey,
                    latBase: item.latBase,
                    lonBase: item.lonBase,
                    size: item.size,
                    vertsPerChunk: item.vertsPerChunk,
                    step: item.lodStep || 1,
                    cx: item.cx,
                    cy: item.cy
                });
                scheduled++;
            }
        }
    } else {
        for (let i = 0; i < CHUNKS_PER_FRAME && chunkCreationQueue.length > 0; i++) {
            const item = chunkCreationQueue.shift();
            if (!activeChunks[item.chunkKey] || item.lodRebuild) {
                if (!isChunkInRange(item)) {
                    continue;
                }
                const existing = activeChunks[item.chunkKey];
                if (item.lodRebuild && existing) {
                    const step = sanitizeLodStep(item.lodStep || 1, item.vertsPerChunk);
                    const { heights, minH, maxH } = sampleChunkHeights(item, step);
                    setChunkHeights(existing, item, step, heights, minH, maxH);
                    existing.data.lodRebuildQueued = false;
                } else {
                    createSingleChunk(item);
                }
            }
        }
    }

    requestAnimationFrame(processChunkQueue);
}

/**
 * Create a single terrain chunk
 * @param {Object} item - Chunk creation parameters
 */
function createSingleChunk(item) {
    const step = sanitizeLodStep(item.lodStep || 1, item.vertsPerChunk);
    const { heights, minH, maxH } = sampleChunkHeights(item, step);
    // NON caricare satellite qui - verrà fatto da refreshNearbyChunkTextures
    // dopo che il terreno base è completamente caricato
    const mesh = addChunkMesh(item, step, heights, minH, maxH);
    noteChunkCreated(false);
    return mesh;
}

// Chunks come by the hundred at startup and on every move: one summary line
// per CHUNK_LOG_MS instead of one per chunk, which would flood the debug log.
const CHUNK_LOG_MS = 2000;
let chunkLog = { worker: 0, main: 0, timer: null };
function noteChunkCreated(fromWorker) {
    chunkLog[fromWorker ? 'worker' : 'main']++;
    if (chunkLog.timer) return;
    chunkLog.timer = setTimeout(() => {
        const { worker, main } = chunkLog;
        console.debug(`[terrain] ${worker + main} chunks created (worker ${worker}, main thread ${main}), ${Object.keys(activeChunks).length} active`);
        chunkLog = { worker: 0, main: 0, timer: null };
    }, CHUNK_LOG_MS);
}

/**
 * Register a chunk. It is drawn through its grid's instanced batch until it
 * gets a satellite map (setChunkMap), so it only joins the scene then.
 */
function addChunkMesh(item, step, heights, minH, maxH) {
    const { cx, cy, chunkKey, latBase, lonBase, vertsPerChunk } = item;
    const chunksPerAxis = CHUNKS_PER_TILE_AXIS;

    const record = renderWorld.terrain.put(item, step, heights, minH, maxH);
    const mesh = terrainView.createChunk(record, {
        chunkLatTop: latBase + 1 - cy / chunksPerAxis,
        chunkLatBottom: latBase + 1 - (cy + 1) / chunksPerAxis,
        chunkLonLeft: lonBase + cx / chunksPerAxis,
        chunkLonRight: lonBase + (cx + 1) / chunksPerAxis,
        vertsPerChunk
    });
    activeChunks[chunkKey] = mesh;
    chunksCreated++;
    markChunkActivity();
    return mesh;
}

/**
 * Resolution band for a chunk, from its centre distance to the aircraft.
 * Until the first pass of base textures has landed everything uses BASE_BAND, so
 * the whole visible area gets covered quickly before any chunk spends time on a
 * 4096² texture.
 * @returns {number} index into ZOOM_BANDS
 */
function getBandForChunk(latTop, latBottom, lonLeft, lonRight) {
    if (!initialTexturesLoaded) return BASE_BAND;
    const centerLat = (latTop + latBottom) / 2;
    const centerLon = (lonLeft + lonRight) / 2;
    const dist = calculateDistance(STATE.lat, STATE.lon, centerLat, centerLon);
    return bandForDistance(dist);
}

/**
 * Create composite texture for a terrain chunk
 */
function createChunkTexture(mesh, latTop, latBottom, lonLeft, lonRight) {
    if (!window.satelliteEnabled) {
        mesh.data.textureLoaded = false;
        return;
    }

    // Resolution follows distance. The band fixes both the zoom and the canvas it
    // may allocate; the loop below is the safety net for geometry the table cannot
    // predict — chunks are taller in latitude than wide in longitude at high
    // latitude, so the same zoom needs more pixels the further north you fly.
    const band = getBandForChunk(latTop, latBottom, lonLeft, lonRight);
    const bandCap = Math.min(ZOOM_BANDS[band].maxDim, MAX_CANVAS_DIM);
    let zoomLevel = ZOOM_BANDS[band].zoom;

    const TILE_SIZE = 256;
    const MIN_ZOOM = ZOOM_BANDS[ZOOM_BANDS.length - 1].zoom;
    while (zoomLevel > MIN_ZOOM) {
        const tl = latLonToTile(latTop, lonLeft, zoomLevel);
        const br = latLonToTile(latBottom, lonRight, zoomLevel);
        const w = (br.x - tl.x + 1) * TILE_SIZE;
        const h = (br.y - tl.y + 1) * TILE_SIZE;
        if (w <= bandCap && h <= bandCap) break;
        zoomLevel--;
    }

    const tileTopLeft = latLonToTile(latTop, lonLeft, zoomLevel);
    const tileBottomRight = latLonToTile(latBottom, lonRight, zoomLevel);

    const tilesX = tileBottomRight.x - tileTopLeft.x + 1;
    const tilesY = tileBottomRight.y - tileTopLeft.y + 1;

    // Compute the chunk's crop rectangle in mosaic pixel space up front and
    // allocate ONLY the cropped canvas: tiles are drawn directly at negative
    // offsets (canvas clips them). This avoids a transient full-mosaic canvas
    // (up to 8192² = 268 MB) plus a full-size copy per chunk.
    const mosaicWidth = tilesX * TILE_SIZE;
    const mosaicHeight = tilesY * TILE_SIZE;

    const topLeftBounds = tileToBounds(tileTopLeft.x, tileTopLeft.y, zoomLevel);
    const bottomRightBounds = tileToBounds(tileBottomRight.x, tileBottomRight.y, zoomLevel);
    const tilesLatTop = topLeftBounds.latTop;
    const tilesLatBottom = bottomRightBounds.latBottom;
    const tilesLonLeft = topLeftBounds.lonLeft;
    const tilesLonRight = bottomRightBounds.lonRight;

    const uMin = (lonLeft - tilesLonLeft) / (tilesLonRight - tilesLonLeft);
    const uMax = (lonRight - tilesLonLeft) / (tilesLonRight - tilesLonLeft);
    const vMin = (tilesLatTop - latTop) / (tilesLatTop - tilesLatBottom);
    const vMax = (tilesLatTop - latBottom) / (tilesLatTop - tilesLatBottom);

    const cropX = Math.floor(uMin * mosaicWidth);
    const cropY = Math.floor(vMin * mosaicHeight);
    const cropW = Math.max(1, Math.floor((uMax - uMin) * mosaicWidth));
    const cropH = Math.max(1, Math.floor((vMax - vMin) * mosaicHeight));

    // With BC1 compression the canvas is only a staging buffer for the worker:
    // an OffscreenCanvas with willReadFrequently stays in CPU memory, so the tiles
    // (CPU ImageBitmaps) are blitted without a GPU round trip and the result can be
    // transferred to the worker as-is. The RGBA fallback uploads the canvas itself
    // as a texture, so it keeps a regular (GPU) canvas.
    const staging = compressAvailable && typeof OffscreenCanvas !== 'undefined';
    const canvas = staging ? new OffscreenCanvas(cropW, cropH) : document.createElement('canvas');
    canvas.width = cropW;
    canvas.height = cropH;
    const ctx = canvas.getContext('2d', staging ? { willReadFrequently: true } : undefined);
    // Fill with a neutral fallback so failed/missing satellite tiles show a solid
    // patch instead of black holes in the texture.
    ctx.fillStyle = '#808080';
    ctx.fillRect(0, 0, cropW, cropH);
    canvasesCreated++;

    const totalTilesForChunk = tilesX * tilesY;
    const chunkJob = {
        mesh,
        canvas,
        ctx,
        cropX,
        cropY,
        zoomLevel,
        totalTiles: totalTilesForChunk,
        tilesDrawn: 0,
        tilesHit: 0,   // tiles that really landed on the canvas (cache or network)
        aborted: false
    };
    
    // Track active job for cleanup
    activeChunkJobs.set(mesh.uuid, chunkJob);
    
    // Aggiungi al contatore globale delle tile da caricare
    totalTilesToLoad += totalTilesForChunk;

    // Record the band this texture was built for, not the effective zoom: the cap
    // loop above may have stepped the zoom down, and keying the re-texture decision
    // off the zoom would make such a chunk look permanently out of date and get
    // rebuilt on every pass.
    mesh.data.textureZoom = zoomLevel;
    mesh.data.textureBand = band;

    for (let ty = tileTopLeft.y; ty <= tileBottomRight.y; ty++) {
        for (let tx = tileTopLeft.x; tx <= tileBottomRight.x; tx++) {
            const localX = tx - tileTopLeft.x;
            const localY = ty - tileTopLeft.y;

            loadTileImage(tx, ty, zoomLevel, (img) => {
                tilesLoaded++; // Incrementa contatore globale (loading overlay)
                enqueueTileDraw(chunkJob, img, localX, localY, TILE_SIZE, tx, ty, zoomLevel);
            });
        }
    }
}

function enqueueTileDraw(job, img, localX, localY, tileSize, tileX, tileY, tileZ, retryCount = 0) {
    if (!window.satelliteEnabled || !job || !job.ctx || job.aborted) return;
    tileDrawQueue.push({ job, img, localX, localY, tileSize, tileX, tileY, tileZ, retryCount });
    if (!isProcessingTileDrawQueue) {
        requestAnimationFrame(processTileDrawQueue);
    }
}

function processTileDrawQueue() {
    if (tileDrawQueue.length === 0) {
        isProcessingTileDrawQueue = false;
        return;
    }

    isProcessingTileDrawQueue = true;
    const start = performance.now();
    let processed = 0;

    while (tileDrawQueue.length > 0) {
        const { job, img, localX, localY, tileSize, tileX, tileY, tileZ, retryCount } = tileDrawQueue.shift();

        // Skip aborted jobs
        if (!job || job.aborted || !job.ctx) {
            continue;
        }

        // The ImageBitmap may have been evicted + closed by imageLRU while
        // queued (detached → width/height become 0). Drawing it throws an
        // uncaught InvalidStateError, which would kill the queue loop and
        // leave isProcessingTileDrawQueue stuck true, stalling all future
        // tile draws. Guard against it and retry once; after retries, the
        // neutral canvas fill covers the gap instead of a black hole.
        if (!img || img.width === 0 || img.height === 0) {
            if (retryCount < MAX_TILE_DRAW_RETRIES) {
                loadTileImage(tileX, tileY, tileZ, (img2) => {
                    enqueueTileDraw(job, img2, localX, localY, tileSize, tileX, tileY, tileZ, retryCount + 1);
                });
            } else {
                // No usable tile after retries — leave the fallback fill and move on.
                job.tilesDrawn++;
            }
        } else {
            try {
                // Draw in mosaic space shifted by the crop origin — out-of-bounds
                // portions are clipped by the canvas for free.
                job.ctx.drawImage(img, localX * tileSize - job.cropX, localY * tileSize - job.cropY, tileSize, tileSize);
                job.tilesHit++;
            } catch (e) {
                // Detached/invalid image source — leave fallback fill.
            }
            job.tilesDrawn++;
        }

        if (job.tilesDrawn >= job.totalTiles) {
            // Remove from active jobs tracking
            activeChunkJobs.delete(job.mesh.uuid);
            // Nullify ctx to prevent further draws
            const canvas = job.canvas;
            job.ctx = null;
            job.canvas = null;
            if (job.tilesHit === 0) {
                // Not a single tile (typically offline with nothing cached for
                // this area): keep the height-tinted chunk rather than a grey slab.
                releaseCanvas(canvas);
                if (job.mesh && job.mesh.data) job.mesh.data.textureLoaded = false;
            } else {
                enqueueCompositeTexture(job.mesh, canvas);
            }
        }

        processed++;
        const elapsed = performance.now() - start;
        if (processed >= MAX_TILE_DRAWS_PER_FRAME || elapsed > TILE_DRAW_BUDGET_MS) {
            break;
        }
    }

    if (tileDrawQueue.length > 0) {
        requestAnimationFrame(processTileDrawQueue);
    } else {
        isProcessingTileDrawQueue = false;
    }
}

function enqueueCompositeTexture(mesh, canvas) {
    if (!mesh || (mesh.data && mesh.data.disposed)) {
        if (canvas) {
            canvas.width = 1;
            canvas.height = 1;
            canvasesReleased++;
        }
        return;
    }
    textureApplyQueue.push({ mesh, canvas });
    if (!isProcessingTextureQueue) {
        requestAnimationFrame(processTextureApplyQueue);
    }
}

function enqueueChunkTexture(mesh, ud, dist, forceReload = false) {
    if (!mesh || !ud || ud.textureQueued) return;
    if (!forceReload && ud.textureLoaded) return;
    if (dist > SATELLITE_RADIUS) return;
    ud.textureQueued = true;
    chunkTextureQueue.push({ mesh, ud, dist });
    if (!isProcessingChunkTextureQueue) {
        requestAnimationFrame(processChunkTextureQueue);
    }
}

function processChunkTextureQueue() {
    if (chunkTextureQueue.length === 0) {
        isProcessingChunkTextureQueue = false;
        return;
    }

    isProcessingChunkTextureQueue = true;
    let processed = 0;
    while (chunkTextureQueue.length > 0 && processed < MAX_CHUNK_TEXTURES_PER_FRAME) {
        const item = chunkTextureQueue.shift();
        const mesh = item.mesh;
        const ud = item.ud;
        if (!mesh || !ud) {
            processed++;
            continue;
        }
        ud.textureQueued = false;
        if (!window.satelliteEnabled) {
            processed++;
            continue;
        }

        // Recheck distance to avoid work for out-of-range chunks
        const centerLat = (ud.chunkLatTop + ud.chunkLatBottom) / 2;
        const centerLon = (ud.chunkLonLeft + ud.chunkLonRight) / 2;
        const centerWorld = latLonToMeters(centerLat, centerLon);
        const playerPos = latLonToMeters(STATE.lat, STATE.lon);
        const dx = centerWorld.x - playerPos.x;
        const dz = centerWorld.z - playerPos.z;
        const dist = Math.sqrt(dx * dx + dz * dz);
        if (dist <= SATELLITE_RADIUS) {
            // LOD swap: keep the current texture visible until the new one is
            // ready (applyCompositeTexture disposes it on swap). Unloading
            // first left the chunk white for the whole tile download.
            if (ud.textureLoaded) {
                abortChunkJob(mesh);
            }
            createChunkTexture(mesh, ud.chunkLatTop, ud.chunkLatBottom, ud.chunkLonLeft, ud.chunkLonRight);
        }
        processed++;
    }

    if (chunkTextureQueue.length > 0) {
        requestAnimationFrame(processChunkTextureQueue);
    } else {
        isProcessingChunkTextureQueue = false;
    }
}

function processTextureApplyQueue() {
    if (textureApplyQueue.length === 0) {
        isProcessingTextureQueue = false;
        return;
    }

    isProcessingTextureQueue = true;
    const start = performance.now();
    let processed = 0;

    while (textureApplyQueue.length > 0) {
        const job = textureApplyQueue.shift();
        applyCompositeTexture(job.mesh, job.canvas);
        processed++;

        const elapsed = performance.now() - start;
        if (processed >= MAX_TEXTURE_APPLIES_PER_FRAME || elapsed > TEXTURE_APPLY_BUDGET_MS) {
            break;
        }
    }

    if (textureApplyQueue.length > 0) {
        requestAnimationFrame(processTextureApplyQueue);
    } else {
        isProcessingTextureQueue = false;
    }
}

/**
 * Load tile image with caching and queue system
 */
function loadTileImage(tileX, tileY, tileZ, callback) {
    const key = `${tileZ}/${tileX}/${tileY}`;

    // Check in-memory LRU cache first
    const cached = imageLRU.get(key);
    if (cached) {
        // An evicted+closed ImageBitmap reports zero size; using it leaves a hole.
        if (cached.width > 0 && cached.height > 0) {
            callback(cached);
            return;
        }
        // Stale/closed bitmap — remove it and reload from persistent/network.
        imageLRU.delete(key);
    }

    if (!enqueueTileCallback(key, callback)) return;

    // Check IndexedDB persistent cache before network
    getCachedTile('esri', tileZ, tileX, tileY).then(blob => {
        if (blob) {
            createImageBitmap(blob).then(bitmap => {
                imageLRU.set(key, bitmap);
                resolveTileCallbacks(key, bitmap);
            }).catch(() => {
                // Corrupted blob, fall through to network
                enqueueForNetwork(tileX, tileY, tileZ, key);
            });
            return;
        }
        // Cache miss — fetch from network
        enqueueForNetwork(tileX, tileY, tileZ, key);
    }).catch(() => {
        enqueueForNetwork(tileX, tileY, tileZ, key);
    });
}

function enqueueForNetwork(tileX, tileY, tileZ, key) {
    if (!tileNetworkEnabled) {
        // Cache-only mode: not in IndexedDB → no tile, no error accounting.
        resolveTileCallbacks(key, null);
        return;
    }
    tileLoadQueue.push({ tileX, tileY, tileZ, key });
    if (!isProcessingTileQueue) {
        processTileLoadQueue();
    }
}

function enqueueTileCallback(key, callback) {
    const list = pendingTileCallbacks.get(key);
    if (list) {
        list.push(callback);
        return false;
    }
    pendingTileCallbacks.set(key, [callback]);
    return true;
}

function resolveTileCallbacks(key, img) {
    const list = pendingTileCallbacks.get(key);
    if (!list) return;
    pendingTileCallbacks.delete(key);
    for (const cb of list) {
        try { cb(img); } catch (e) {}
    }
}

/**
 * Process tile load queue with concurrency limit
 */
function processTileLoadQueue() {
    if (tileLoadQueue.length === 0) {
        isProcessingTileQueue = false;
        return;
    }

    isProcessingTileQueue = true;

    // Load tiles up to the concurrent limit
    while (currentTileLoads < MAX_CONCURRENT_TILE_LOADS && tileLoadQueue.length > 0) {
        const item = tileLoadQueue.shift();
        
        // Double-check cache (might have been loaded while in queue)
        const cached = imageLRU.get(item.key);
        if (cached) {
            resolveTileCallbacks(item.key, cached);
            continue;
        }

        currentTileLoads++;
        
        if (tileWorkerAvailable && tileWorker) {
            tileWorker.postMessage({
                type: 'loadTile',
                key: item.key,
                url: `https://mt${item.tileX % 4}.google.com/vt/lyrs=s&x=${item.tileX}&y=${item.tileY}&z=${item.tileZ}`
            });
        } else {
            const tileUrl = `https://mt${item.tileX % 4}.google.com/vt/lyrs=s&x=${item.tileX}&y=${item.tileY}&z=${item.tileZ}`;
            fetch(tileUrl).then(res => {
                if (!res.ok) throw new Error(res.status);
                return res.blob();
            }).then(blob => {
                // Store in IndexedDB for offline use
                putCachedTile('esri', item.tileZ, item.tileX, item.tileY, blob).catch(() => {});
                return createImageBitmap(blob);
            }).then(bitmap => {
                imageLRU.set(item.key, bitmap);
                consecutiveTileErrors = 0;
                resolveTileCallbacks(item.key, bitmap);
                currentTileLoads--;
                processTileLoadQueue();
            }).catch(() => {
                consecutiveTileErrors++;
                if (consecutiveTileErrors >= CONSECUTIVE_ERROR_THRESHOLD && !connectionLostNotified) {
                    connectionLostNotified = true;
                    console.warn(`${CONSECUTIVE_ERROR_THRESHOLD} consecutive tile errors — connection lost, satellite from cache only`);
                    setTileNetworkEnabled(false);
                    window.dispatchEvent(new CustomEvent('connectionLost'));
                }
                resolveTileCallbacks(item.key, null);
                currentTileLoads--;
                processTileLoadQueue();
            });
        }
    }
}

/**
 * Apply composite texture to mesh.
 * The canvas arrives already cropped to the chunk bounds (tiles are drawn
 * directly at the crop offset in processTileDrawQueue), so no copy happens here.
 */
function applyCompositeTexture(mesh, canvas) {
    // If satellite got disabled while tiles were loading, don't apply.
    if (!window.satelliteEnabled) {
        if (mesh && mesh.data) mesh.data.textureLoaded = false;
        return;
    }

    if (!mesh || (mesh.data && mesh.data.disposed)) {
        releaseCanvas(canvas);
        return;
    }

    // Preferred path: compress to BC1 off-thread. The texture is attached when
    // the worker answers; until then the chunk keeps whatever it already had.
    if (requestCompressedTexture(mesh, canvas)) return;

    const texture = terrainView.makeCanvasTexture(canvas);
    texturesCreated++;

    attachTextureToMesh(mesh, texture);
}

/**
 * Put a finished texture (compressed or not) on a chunk.
 */
function attachTextureToMesh(mesh, texture) {
    if (mesh && terrainView.hasChunk(mesh)) {
        terrainView.setMap(mesh, texture);
        mesh.data.textureLoaded = true;
    }
}

/**
 * Abort the in-flight texture composition job for a mesh (if any) without
 * touching the material's current map — used by LOD swaps to keep the old
 * texture visible until the replacement is ready.
 */
function abortChunkJob(mesh) {
    const job = activeChunkJobs.get(mesh.uuid);
    if (!job) return;
    job.aborted = true;
    // Release canvas memory
    if (job.canvas) {
        job.canvas.width = 1;
        job.canvas.height = 1;
        job.canvas = null;
        canvasesReleased++;
    }
    job.ctx = null;
    activeChunkJobs.delete(mesh.uuid);
}

function unloadChunkTexture(mesh) {
    if (!mesh || !terrainView.hasChunk(mesh)) return;

    abortChunkJob(mesh);

    const mapDescriptor = renderWorld.terrain.chunks.get(mesh.id)?.appearance;
    if (mapDescriptor) {
        // A CanvasTexture's backing canvas is released along with it (a
        // compressed texture's staging canvas was released when it was built)
        if (mapDescriptor.kind === 'canvas-reference') canvasesReleased++;
        terrainView.setMap(mesh, null);
    }
    if (mesh.data) {
        mesh.data.textureLoaded = false;
        mesh.data.textureQueued = false;
        mesh.data.textureZoom = 0;
        mesh.data.textureBand = undefined;
    }
}

/**
 * Clear all pending texture operations (call when satellite is disabled)
 */
function clearPendingTextureOperations() {
    // Abort all active chunk jobs and release their canvases
    let releasedCount = 0;
    for (const [uuid, job] of activeChunkJobs) {
        job.aborted = true;
        if (job.canvas) {
            job.canvas.width = 1;
            job.canvas.height = 1;
            job.canvas = null;
            releasedCount++;
        }
        job.ctx = null;
    }
    canvasesReleased += releasedCount;
    activeChunkJobs.clear();
    
    // Clear tile draw queue
    tileDrawQueue.length = 0;
    
    // Clear texture apply queue and release canvases
    for (const item of textureApplyQueue) {
        if (item.canvas) {
            item.canvas.width = 1;
            item.canvas.height = 1;
            canvasesReleased++;
        }
    }
    textureApplyQueue.length = 0;
}

export function setTerrainChunksVisible(visible) {
    chunksVisible = !!visible;
    renderWorld.terrainStyle.visible = chunksVisible;
}

/**
 * Palette of the schematic terrain: green ground with dark lines (light UI
 * theme) or dark ground with light lines.
 * @param {boolean} light
 */
export function setTerrainSchematicLight(light) {
    renderWorld.terrainStyle.light = !!light;
}

/**
 * Strength (0..1) of the low-altitude grid at a height above the surface
 * under it: 1 up to TRI_GRID_FULL_AGL, 0 from TRI_GRID_MAX_AGL, null → 0.
 */
export function lowAltGridStrength(height) {
    if (!Number.isFinite(height)) return 0;
    const t = Math.max(0, Math.min(1, (height - TRI_GRID_FULL_AGL) / (TRI_GRID_MAX_AGL - TRI_GRID_FULL_AGL)));
    return 1 - t * t * (3 - 2 * t);
}

/**
 * Aircraft position for the triangle grid of the schematic view: centred on
 * it, at full strength up to TRI_GRID_FULL_AGL above the ground, gone at
 * TRI_GRID_MAX_AGL.
 * @param {number|null} agl metres above the ground, null when unknown (no grid)
 * @param {number} x world x of the aircraft
 * @param {number} z world z of the aircraft
 */
export function setTerrainLowAltGrid(agl, x, z) {
    const s = lowAltGridStrength(agl);
    renderWorld.terrainStyle.gridStrength = s;
    if (s > 0) { renderWorld.terrainStyle.gridCenter.x = x; renderWorld.terrainStyle.gridCenter.y = z; }
}

/**
 * Enable/disable satellite textures on existing terrain chunks.
 * When disabling, removes any already-applied textures so the overlay actually disappears,
 * and the bare terrain switches to the schematic isoline style.
 * When enabling, schedules texture generation for chunks that don't have it yet.
 * @param {boolean} enabled
 */
export function setTerrainSatelliteEnabled(enabled) {
    const on = !!enabled;
    if (!activeChunks) return;

    // Chunks beyond the satellite radius keep the height palette while the
    // imagery is on; with it off every chunk is drawn schematic.
    renderWorld.terrainStyle.schematic = !on;

    console.log(`[terrain] Satellite ${on ? 'ENABLED' : 'DISABLED'} — activeChunks=${Object.keys(activeChunks).length}, queue=${chunkCreationQueue.length}, workerPending=${workerPending.size}`);

    // Enabling needs no per-chunk pass: refreshNearbyChunkTextures() below
    // queues the chunks inside SATELLITE_RADIUS, nearest first. Texturing every
    // resident chunk here (as this loop used to) built ~400 textures for chunks
    // beyond the radius — tile loads, compositing and BC1 compression — only for
    // the texture cull to throw them away, every time the satellite came back on
    // (including on leaving the FPV AR mode).
    if (!on) {
        for (const key in activeChunks) {
            const mesh = activeChunks[key];
            if (mesh && terrainView.hasChunk(mesh)) unloadChunkTexture(mesh);
        }
        clearPendingTextureOperations();
        try { imageLRU.clear(); } catch (e) {}
        tileLoadQueue.length = 0;
        pendingTileCallbacks.clear();
        currentTileLoads = 0;
    }

    // Force a terrain refresh: if chunks were never created (lazy HGT load,
    // slow worker, or stale queue), this re-runs updateTerrainChunks() so the
    // missing rectangular patches get rebuilt.
    resetTextureRefreshPosition();
    updateTerrainChunks();
    refreshNearbyChunkTextures();
}

/**
 * Cleanup distant chunks
 * NON esegue durante il caricamento iniziale
 */
function cleanupDistantChunks() {
    // Non pulire durante il caricamento iniziale
    if (!terrainBaseReady) {
        const now = performance.now();
        if (Object.keys(activeChunks).length > 0 && (now - lastChunkActivityTime) > BASE_READY_FORCE_MS) {
            terrainBaseReady = true;
            return;
        }
        return;
    }
    
    const playerPos = latLonToMeters(STATE.lat, STATE.lon);
    const chunkEntries = Object.entries(activeChunks);

    let removed = 0;
    for (const [key, mesh] of chunkEntries) {
        const ud = mesh.data;
        if (!ud.chunkLatTop) continue;

        const centerLat = (ud.chunkLatTop + ud.chunkLatBottom) / 2;
        const centerLon = (ud.chunkLonLeft + ud.chunkLonRight) / 2;
        const centerWorld = latLonToMeters(centerLat, centerLon);

        const dist = Math.sqrt(
            (centerWorld.x - playerPos.x) ** 2 +
            (centerWorld.z - playerPos.z) ** 2
        );

        if (dist > CLEANUP_RADIUS) {
            disposeChunk(key, mesh);
            removed++;
        }
    }

    if (Object.keys(activeChunks).length > MAX_ACTIVE_CHUNKS) {
        const sorted = Object.entries(activeChunks)
            .map(([key, mesh]) => {
                const ud = mesh.data;
                if (!ud.chunkLatTop) return { key, dist: 0 };
                const centerLat = (ud.chunkLatTop + ud.chunkLatBottom) / 2;
                const centerLon = (ud.chunkLonLeft + ud.chunkLonRight) / 2;
                const centerWorld = latLonToMeters(centerLat, centerLon);
                const dist = Math.sqrt(
                    (centerWorld.x - playerPos.x) ** 2 +
                    (centerWorld.z - playerPos.z) ** 2
                );
                return { key, mesh, dist };
            })
            .sort((a, b) => b.dist - a.dist);

        const toRemove = sorted.slice(0, sorted.length - MAX_ACTIVE_CHUNKS);
        for (const item of toRemove) {
            if (item.mesh) {
                disposeChunk(item.key, item.mesh);
                removed++;
            }
        }
    }

    // LOD maintenance: rebuild chunks whose distance band changed
    updateChunkLods(playerPos);

    // Prune HGT cache far from player to free memory
    cleanupHgtCache();

}

/**
 * Rebuild chunks whose LOD band no longer matches their distance.
 * Hysteresis (±15%) prevents rebuild ping-pong at band boundaries; the old
 * mesh stays in the scene until the worker delivers the replacement.
 */
function updateChunkLods(playerPos) {
    const rebuilds = [];

    for (const [key, mesh] of Object.entries(activeChunks)) {
        const ud = mesh.data;
        if (!ud || ud.chunkLatTop == null || !ud.lodStep || ud.lodRebuildQueued) continue;
        if (workerPending.has(key)) continue;

        const centerLat = (ud.chunkLatTop + ud.chunkLatBottom) / 2;
        const centerLon = (ud.chunkLonLeft + ud.chunkLonRight) / 2;
        const centerWorld = latLonToMeters(centerLat, centerLon);
        const dist = Math.sqrt(
            (centerWorld.x - playerPos.x) ** 2 +
            (centerWorld.z - playerPos.z) ** 2
        );

        // Compare against the step a rebuild would actually apply: an unsanitised
        // target the chunk can never reach would requeue it on every pass forever.
        const desired = sanitizeLodStep(lodStepForDistance(dist), ud.vertsPerChunk || 0);
        if (desired === ud.lodStep) continue;

        const vpc = ud.vertsPerChunk || 0;
        if (desired < ud.lodStep) {
            // Upgrade only when firmly inside the finer band
            if (sanitizeLodStep(lodStepForDistance(dist * 1.15), vpc) < ud.lodStep) {
                rebuilds.push({ key, mesh, dist, upgrade: 1 });
            }
        } else {
            // Downgrade only when firmly outside the current band
            if (sanitizeLodStep(lodStepForDistance(dist * 0.85), vpc) > ud.lodStep) {
                rebuilds.push({ key, mesh, dist, upgrade: 0 });
            }
        }
    }

    if (rebuilds.length === 0) return;

    // Upgrades first, nearest first
    rebuilds.sort((a, b) => (b.upgrade - a.upgrade) || (a.dist - b.dist));

    for (const rb of rebuilds.slice(0, LOD_REBUILDS_PER_PASS)) {
        requeueChunkForLod(rb.key, rb.mesh, rb.dist);
    }

    if (!isProcessingChunks && chunkCreationQueue.length > 0) {
        processChunkQueue();
    }
}

function requeueChunkForLod(chunkKey, mesh, dist) {
    const parts = chunkKey.split('_');
    if (parts.length !== 4) return;
    const latBase = parseInt(parts[0], 10);
    const lonBase = parseInt(parts[1], 10);
    const cx = parseInt(parts[2], 10);
    const cy = parseInt(parts[3], 10);
    if (!Number.isFinite(latBase) || !Number.isFinite(lonBase)) return;

    const hgtKey = `${latBase}_${lonBase}`;
    const hgt = hgtElevationData[hgtKey];
    if (!hgt) return; // elevation data no longer in memory — skip

    const size = hgt.size;
    // Must match the grid processHGTFile() used to create the chunk: cx/cy index
    // a CHUNKS_PER_TILE_AXIS grid, so a different divisor here puts startRow/startCol
    // outside the tile, the worker clamps every sample to the tile edge, and the
    // rebuilt chunk collapses to a zero-area mesh — a hole in the terrain.
    const vertsPerChunk = Math.floor((size - 1) / CHUNKS_PER_TILE_AXIS);

    mesh.data.lodRebuildQueued = true;
    chunkCreationQueue.push({
        cx, cy, dist, chunkKey,
        latBase, lonBase, size, vertsPerChunk,
        lodStep: sanitizeLodStep(lodStepForDistance(dist), vertsPerChunk),
        hgtKey,
        dataView: null,
        lodRebuild: true
    });
}

/**
 * Cleanup HGT elevation cache far from player.
 * Parsed elevation arrays are kept permanently in memory to avoid
 * re-parsing or re-downloading the same tile repeatedly.
 */
function cleanupHgtCache() {
    // Intentionally left empty: hgtElevationData entries are retained for the
    // full session lifetime. SRTM3 uses 2.75 MiB, SRTM1 uses 24.73 MiB per
    // native grid; the worker retains another grid and the File is also kept.
    // A bounded cache requires coordinated eviction with pending chunk jobs.
}

/**
 * Dispose a single chunk
 */
function disposeChunk(key, mesh) {
    if (!mesh) return;

    if (mesh.data) mesh.data.disposed = true;

    // Release any pending texture work and map
    unloadChunkTexture(mesh);

    // Remove queued work items for this mesh and release canvases
    if (textureApplyQueue.length > 0) {
        for (let i = textureApplyQueue.length - 1; i >= 0; i--) {
            const item = textureApplyQueue[i];
            if (item.mesh === mesh) {
                if (item.canvas) {
                    item.canvas.width = 1;
                    item.canvas.height = 1;
                    canvasesReleased++;
                }
                textureApplyQueue.splice(i, 1);
            }
        }
    }

    if (tileDrawQueue.length > 0) {
        for (let i = tileDrawQueue.length - 1; i >= 0; i--) {
            const item = tileDrawQueue[i];
            if (item.job && item.job.mesh === mesh) {
                tileDrawQueue.splice(i, 1);
            }
        }
    }

    if (chunkTextureQueue.length > 0) {
        for (let i = chunkTextureQueue.length - 1; i >= 0; i--) {
            const item = chunkTextureQueue[i];
            if (item.mesh === mesh) {
                chunkTextureQueue.splice(i, 1);
            }
        }
    }

    terrainView.removeChunk(mesh);
    renderWorld.terrain.remove(key);
    delete activeChunks[key];
    chunksDisposed++;
}

/**
 * Push the sunlight switch to the terrain shader. The sun direction needs no
 * call: the shader reads the scene's sun vector directly (see initTerrain).
 */
export function updateTerrainHillshading() {
    renderWorld.terrainStyle.sunlight = window.sunlightEnabled !== false;
}

/**
 * Terrain brightness while the sunlight is off.
 * @param {number} value 0.3 .. 1.6
 */
export function setMapBrightness(value) {
    const v = Number(value);
    if (!Number.isFinite(v)) return;
    mapBrightness = Math.max(0.3, Math.min(1.6, v));
    renderWorld.terrainStyle.brightness = mapBrightness;
}

// Getters
/**
 * Everything the loading overlay needs to decide whether the initial load
 * is really over. Each *Pending counter covers a stage the older per-queue
 * getters missed: HGT reads/downloads in flight, chunks handed to the worker,
 * textures being composed/compressed off the tile queue (cached tiles never
 * enter tileLoadQueue at all).
 */
export function getInitialLoadStatus() {
    let texturedChunks = 0;
    let chunksActive = 0;
    for (const key in activeChunks) {
        chunksActive++;
        const ud = activeChunks[key] && activeChunks[key].data;
        if (ud && ud.textureLoaded) texturedChunks++;
    }
    return {
        hgtAvailable: availableHgtFiles.size,
        hgtLoaded: Object.keys(hgtFiles).length,
        hgtPending: hgtLoadingInProgress.size + _autoDownloadInProgress.size + hgtPreparations.size,
        chunksActive,
        chunksPending: chunkCreationQueue.length + workerPending.size,
        terrainBaseReady,
        firstTexturePassStarted,
        texturedChunks,
        texturesPending: chunkTextureQueue.length + activeChunkJobs.size +
            tileDrawQueue.length + textureApplyQueue.length + compressPending.size,
        tilesQueued: tileLoadQueue.length + currentTileLoads,
        tilesTotal: totalTilesToLoad,
        tilesLoaded
    };
}

export function getActiveChunks() { return activeChunks; }
export function getHgtElevationData() { return hgtElevationData; }
export function getChunkCreationQueue() { return chunkCreationQueue; }
export function getTileLoadQueue() { return tileLoadQueue; }
export function getCurrentTileLoads() { return currentTileLoads; }
export function getRunwayObjects() { return runwayObjects; }
export function getTotalTilesToLoad() { return totalTilesToLoad; }
export function getTilesLoaded() { return tilesLoaded; }

// Track last texture refresh position and time
let lastTextureRefreshPos = { x: null, z: null };
let lastTextureRefreshTime = 0;

// Calcola distanza di refresh in base alla velocità
// A bassa velocità refresh frequente, ad alta velocità refresh anticipato
function getRefreshDistance() {
    const gs = STATE.gs || 0; // ground speed in knots
    const gsMs = gs * 0.514444; // converti in m/s
    
    // Refresh ogni ~10 secondi di volo, minimo 500m, massimo 5000m
    const refreshDist = Math.max(500, Math.min(5000, gsMs * 10));
    return refreshDist;
}

/**
 * Reset texture refresh position to force immediate refresh on next call
 */
export function resetTextureRefreshPosition() {
    lastTextureRefreshPos = { x: null, z: null };
    lastTextureRefreshTime = 0;
}

/**
 * Check if nearby chunks need texture refresh based on position and speed
 * Carica texture HD per tutti i chunk entro SATELLITE_RADIUS (10km)
 */
export function refreshNearbyChunkTextures() {
    if (!window.satelliteEnabled) return;

    const playerPos = latLonToMeters(STATE.lat, STATE.lon);
    const now = performance.now();
    const refreshDistance = getRefreshDistance();

    // Force refresh if position was reset (null) or moved enough
    const needsRefresh = lastTextureRefreshPos.x === null || (() => {
        const distFromLastRefresh = Math.sqrt(
            (playerPos.x - lastTextureRefreshPos.x) ** 2 +
            (playerPos.z - lastTextureRefreshPos.z) ** 2
        );
        // Anche refresh ogni 30 secondi minimo per sicurezza
        const timeElapsed = now - lastTextureRefreshTime;
        return distFromLastRefresh >= refreshDistance || timeElapsed > 30000;
    })();

    if (!needsRefresh) return;

    lastTextureRefreshPos = { x: playerPos.x, z: playerPos.z };
    lastTextureRefreshTime = now;

    // Find chunks that need textures or LOD swap
    let chunksToLoad = [];
    let chunksToUpgrade = [];
    let chunksToDowngrade = [];
    let cullCandidates = [];
    // Band hysteresis: a chunk must be 10% inside the next band before it is
    // re-textured at higher resolution, and 10% outside before it drops back.
    // Without it, a chunk parked on a band boundary re-composes its texture on
    // every pass — which is the most expensive thing the terrain does.
    const HYSTERESIS = 0.1;

    for (const [key, mesh] of Object.entries(activeChunks)) {
        if (!mesh || !mesh.data) continue;

        const ud = mesh.data;
        if (ud.chunkLatTop == null) continue;
        // Skip chunks already being processed
        if (ud.textureQueued || activeChunkJobs.has(mesh.uuid)) continue;

        const centerLat = (ud.chunkLatTop + ud.chunkLatBottom) / 2;
        const centerLon = (ud.chunkLonLeft + ud.chunkLonRight) / 2;
        const centerWorld = latLonToMeters(centerLat, centerLon);

        const dist = Math.sqrt(
            (centerWorld.x - playerPos.x) ** 2 +
            (centerWorld.z - playerPos.z) ** 2
        );

        if (dist > SATELLITE_RADIUS) {
            if (ud.textureLoaded) {
                cullCandidates.push({ key, centerX: centerWorld.x, centerZ: centerWorld.z });
            }
            continue;
        }

        if (!ud.textureLoaded) {
            // No texture yet — load at appropriate zoom
            chunksToLoad.push({ mesh, ud, dist });
        } else if (initialTexturesLoaded) {
            // Band swap only after initial base textures are loaded
            const current = ud.textureBand ?? BASE_BAND;
            const bandIfCloser = bandForDistance(dist * (1 + HYSTERESIS));
            const bandIfFarther = bandForDistance(dist * (1 - HYSTERESIS));
            if (bandIfCloser < current) {
                // Closer than its texture assumes — re-texture at higher resolution
                chunksToUpgrade.push({ mesh, ud, dist });
            } else if (bandIfFarther > current) {
                // Further away than its texture assumes — drop resolution, free VRAM
                chunksToDowngrade.push({ mesh, ud, dist });
            }
        }
    }

    // Mark initial load complete when all base textures are loaded
    if (!initialTexturesLoaded && chunksToLoad.length === 0 && activeChunkJobs.size === 0) {
        initialTexturesLoaded = true;
    }

    // Sort by distance: load closest first, downgrade farthest first
    chunksToLoad.sort((a, b) => a.dist - b.dist);
    chunksToUpgrade.sort((a, b) => a.dist - b.dist);
    chunksToDowngrade.sort((a, b) => b.dist - a.dist);

    for (const { mesh, ud, dist } of chunksToLoad) {
        enqueueChunkTexture(mesh, ud, dist);
    }
    for (const { mesh, ud, dist } of chunksToUpgrade) {
        enqueueChunkTexture(mesh, ud, dist, true);
    }
    for (const { mesh, ud, dist } of chunksToDowngrade) {
        enqueueChunkTexture(mesh, ud, dist, true);
    }

    // Off-thread selection of textures to unload outside satellite radius
    if (cullCandidates.length > 0) {
        if (textureCullWorkerAvailable && textureCullWorker && !textureCullInFlight) {
            textureCullInFlight = true;
            textureCullWorker.postMessage({
                type: 'cullTextures',
                playerPos: { x: playerPos.x, z: playerPos.z },
                radius: SATELLITE_RADIUS,
                chunks: cullCandidates
            });
        } else if (!textureCullWorkerAvailable) {
            for (const item of cullCandidates) {
                const mesh = activeChunks[item.key];
                if (mesh) unloadChunkTexture(mesh);
            }
        }
    }
}

// Export for runway drawing
export { sceneRef as getSceneRef };
