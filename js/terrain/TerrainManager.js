/**
 * TerrainManager.js - Terrain Loading and Chunk Management
 * Handles HGT file loading, terrain chunk generation, and elevation queries
 */

import { VISIBILITY_RADIUS, RELOAD_DISTANCE, CAMERA_FOV } from '../core/constants.js';

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
    { maxDist: 3000,     zoom: 17, maxDim: 4096 }, // max practical detail, ~1.2 m/px
    { maxDist: 6000,     zoom: 16, maxDim: 2048 },
    { maxDist: 12000,    zoom: 15, maxDim: 1024 },
    { maxDist: 22000,    zoom: 14, maxDim: 512 },
    { maxDist: Infinity, zoom: 13, maxDim: 256 },
];
const BASE_BAND = 2;        // zoom used for the first pass, before the aircraft has a position
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
import { latLonToMeters, calculateDistance, getHeightColor, latLonToTile, tileToBounds } from '../core/utils.js';
import { LRUCache } from '../core/LRUCache.js';
import { getTile as getCachedTile, putTile as putCachedTile } from '../maps/TileCache.js';

// ============== MEMORY TRACKING ==============
let texturesCreated = 0;
let texturesDisposed = 0;
let canvasesCreated = 0;
let canvasesReleased = 0;
let chunksCreated = 0;
let chunksDisposed = 0;

export function getMemoryStats() {
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
        compressedTextures: compressedTexturesBuilt,
        compressedMB: +(compressedBytes / 1048576).toFixed(1),
        compressionActive: compressAvailable
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
const hgtLoadingInProgress = new Set(); // prevent duplicate loads

/** Register which HGT files are available on disk without loading them */
export function setAvailableHgtFiles(names) {
    names.forEach(n => availableHgtFiles.add(n.toUpperCase()));
    console.log(`[terrain] ${availableHgtFiles.size} HGT files available on disk (lazy)`);
}

/** Lazy-load a single HGT file from disk via IPC if not already loaded */
async function ensureHgtLoaded(filename) {
    if (hgtFiles[filename]) return true;
    if (!availableHgtFiles.has(filename)) return false;
    if (hgtLoadingInProgress.has(filename)) return false; // already loading
    if (!window.topography || !window.topography.loadOne) return false;

    hgtLoadingInProgress.add(filename);
    try {
        let ab = await window.topography.loadOne(filename);
        if (!ab) return false;
        if (ab.buffer) ab = ab.buffer; // unwrap if needed
        const file = new File([ab], filename, { type: 'application/octet-stream' });
        addHGTFile(filename, file);
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
            await window.topography.save(filename, hgtBuf.buffer);
            availableHgtFiles.add(filename);
        }

        // Register in memory
        const file = new File([hgtBuf.buffer], filename, { type: 'application/octet-stream' });
        addHGTFile(filename, file);
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
const WORKER_STALE_MS = 10000;
const BASE_READY_FORCE_MS = 10000;
let lastChunkActivityTime = performance.now();

// Flag per sapere quando il terreno base è pronto
let terrainBaseReady = false;
// Flag: initial base textures (zoom 15) loaded, HD upgrades now allowed
let initialTexturesLoaded = false;

// Hillshading state
let hillshadeNeedsFullUpdate = true;
let hillshadeUpdatePending = false;
let cachedSunDir = null;
let lastSunDirX = NaN, lastSunDirY = NaN, lastSunDirZ = NaN;
let mapBrightness = 0.85;

// Scene reference (set during init)
let sceneRef = null;
let rendererRef = null;
let currentSunDirectionRef = null;

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

// Worker-based hillshade (optional)
const USE_HILLSHADE_WORKER = true;
let hillshadeWorker = null;
let hillshadeWorkerAvailable = false;
const hillshadePending = new Map();

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

// Shared wireframe material (single instance for all chunks — saves draw-call state switches)
const sharedWireframeMaterial = new THREE.MeshBasicMaterial({
    color: 0x000000,
    wireframe: true,
    transparent: true,
    opacity: 0.06,
    depthWrite: false
});

// Track which chunk currently has the wireframe (only one at a time)
let wireframeChunkKey = null;

/**
 * Add wireframe overlay to a terrain mesh (triangle grid lines).
 * Shares the same geometry as the solid mesh (zero extra geometry cost).
 * Uses a single shared material for all wireframes (reduces GPU state changes).
 */
function addWireframeOverlay(mesh) {
    if (mesh.userData._wireframe) return; // already has one
    const wire = new THREE.Mesh(mesh.geometry, sharedWireframeMaterial);
    wire.renderOrder = 1;
    mesh.add(wire);
    mesh.userData._wireframe = wire;
}

/**
 * Remove wireframe overlay from a mesh.
 */
function removeWireframeOverlay(mesh) {
    const w = mesh.userData._wireframe;
    if (!w) return;
    mesh.remove(w);
    mesh.userData._wireframe = null;
}

/**
 * Update wireframe: only the single closest chunk (without satellite texture)
 * gets the wireframe overlay. Called from the render loop.
 */
export function updateWireframeProximity() {
    if (window.satelliteEnabled) {
        // Remove any lingering wireframe when satellite is on
        if (wireframeChunkKey && activeChunks[wireframeChunkKey]) {
            removeWireframeOverlay(activeChunks[wireframeChunkKey]);
        }
        wireframeChunkKey = null;
        return;
    }

    const playerPos = latLonToMeters(STATE.lat, STATE.lon);
    let bestKey = null;
    let bestDist = Infinity;

    for (const key in activeChunks) {
        const mesh = activeChunks[key];
        if (!mesh || !mesh.userData) continue;
        const ud = mesh.userData;
        if (ud.textureLoaded) continue; // satellite chunk, no wireframe needed

        const centerLat = (ud.chunkLatTop + ud.chunkLatBottom) / 2;
        const centerLon = (ud.chunkLonLeft + ud.chunkLonRight) / 2;
        const cw = latLonToMeters(centerLat, centerLon);
        const dx = cw.x - playerPos.x;
        const dz = cw.z - playerPos.z;
        const dist = dx * dx + dz * dz; // no sqrt needed for comparison
        if (dist < bestDist) {
            bestDist = dist;
            bestKey = key;
        }
    }

    // Nothing changed
    if (bestKey === wireframeChunkKey) return;

    // Remove old wireframe
    if (wireframeChunkKey && activeChunks[wireframeChunkKey]) {
        removeWireframeOverlay(activeChunks[wireframeChunkKey]);
    }

    // Add wireframe to closest chunk
    wireframeChunkKey = bestKey;
    if (bestKey && activeChunks[bestKey]) {
        addWireframeOverlay(activeChunks[bestKey]);
    }
}

/**
 * Initialize terrain manager
 * @param {THREE.Scene} scene
 * @param {THREE.WebGLRenderer} renderer
 * @param {THREE.Vector3} sunDirection
 */
export function initTerrain(scene, renderer, sunDirection) {
    sceneRef = scene;
    rendererRef = renderer;
    currentSunDirectionRef = sunDirection;

    // Clamp the texture cap to the GPU's real limit so we never allocate a
    // canvas larger than the hardware can upload as a texture.
    const gpuMaxTexture = renderer?.capabilities?.maxTextureSize;
    if (gpuMaxTexture > 0) {
        MAX_CANVAS_DIM = Math.min(MAX_CANVAS_DIM, gpuMaxTexture);
    }
    cachedSunDir = new THREE.Vector3(0, 1, 0);

    initTerrainWorker();
    initTileWorker();
    initHillshadeWorker();
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
    if (!USE_TERRAIN_WORKER || typeof Worker === 'undefined') return;

    try {
        terrainWorker = new Worker(new URL('./TerrainWorker.js', import.meta.url), { type: 'module' });
        workerAvailable = true;

        terrainWorker.onmessage = (e) => {
            const data = e.data || {};
            if (data.type === 'chunkBuilt') {
                const item = workerPending.get(data.chunkKey);
                if (!item) return;
                workerPending.delete(data.chunkKey);
                workerInflight = Math.max(0, workerInflight - 1);
                markChunkActivity();

                // LOD rebuild: swap the old mesh only now that the new one is
                // ready, so the chunk never disappears for a few frames.
                if (item.lodRebuild && activeChunks[data.chunkKey]) {
                    const oldMesh = activeChunks[data.chunkKey];
                    // Hand the satellite texture over to the rebuilt mesh —
                    // same geographic area and UV layout, so it's still valid.
                    // Re-compositing it from tiles made the chunk visibly
                    // "reload" (texture → green → texture) on every LOD change.
                    if (oldMesh.material && oldMesh.material.map &&
                        oldMesh.userData && oldMesh.userData.textureLoaded) {
                        item.inheritedMap = oldMesh.material.map;
                        item.inheritedZoom = oldMesh.userData.textureZoom;
                        item.inheritedBand = oldMesh.userData.textureBand;
                        oldMesh.material.map = null; // detach so dispose doesn't kill it
                    }
                    disposeChunk(data.chunkKey, oldMesh);
                }

                if (!activeChunks[data.chunkKey] && isChunkInRange(item)) {
                    createSingleChunkFromBuffers(item, data.positions, data.uvs, data.colors, data.normals);
                }
                // Rebuilt mesh never got created (chunk left range between
                // dispose and rebuild) — don't orphan the detached texture.
                if (item.inheritedMap) {
                    try { item.inheritedMap.dispose(); texturesDisposed++; } catch (e) {}
                    item.inheritedMap = null;
                }
                return;
            }

            if (data.type === 'chunkFailed') {
                const item = workerPending.get(data.chunkKey);
                if (!item) return;
                workerPending.delete(data.chunkKey);
                workerInflight = Math.max(0, workerInflight - 1);
                markChunkActivity();
                if (!activeChunks[data.chunkKey] && isChunkInRange(item)) {
                    createSingleChunk(item);
                }
            }
        };

        terrainWorker.onerror = () => {
            workerAvailable = false;
            terrainWorker = null;
            for (const item of workerPending.values()) {
                if (!activeChunks[item.chunkKey] || item.lodRebuild) {
                    chunkCreationQueue.unshift(item);
                }
            }
            workerPending.clear();
            workerInflight = 0;
            if (!isProcessingChunks && chunkCreationQueue.length > 0) {
                processChunkQueue();
            }
        };
    } catch (err) {
        workerAvailable = false;
        terrainWorker = null;
    }
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
                    console.warn(`${CONSECUTIVE_ERROR_THRESHOLD} consecutive tile errors — connection lost, disabling satellite`);
                    window.satelliteEnabled = false;
                    window.dispatchEvent(new CustomEvent('connectionLost'));
                }
                resolveTileCallbacks(data.key, null);
                currentTileLoads = Math.max(0, currentTileLoads - 1);
                processTileLoadQueue();
            }
        };

        tileWorker.onerror = () => {
            tileWorkerAvailable = false;
            tileWorker = null;
            pendingTileCallbacks.clear();
        };
    } catch (err) {
        tileWorkerAvailable = false;
        tileWorker = null;
    }
}

function initHillshadeWorker() {
    if (!USE_HILLSHADE_WORKER || typeof Worker === 'undefined') return;

    try {
        hillshadeWorker = new Worker(new URL('./HillshadeWorker.js', import.meta.url), { type: 'module' });
        hillshadeWorkerAvailable = true;

        hillshadeWorker.onmessage = (e) => {
            const data = e.data || {};
            if (data.type !== 'hillshadeComputed') return;

            const mesh = hillshadePending.get(data.meshId);
            if (!mesh) return;
            // Stale response: a newer request for this mesh is in flight —
            // keep the pending entry (it belongs to the newer request) and
            // drop this result, otherwise old colors overwrite fresh ones
            // and the fresh response gets discarded.
            if (data.seq !== undefined && mesh.userData &&
                mesh.userData.hillshadeSeq !== data.seq) return;
            hillshadePending.delete(data.meshId);
            if (!mesh.geometry || !mesh.geometry.attributes || !mesh.geometry.attributes.color) return;

            const colorAttr = mesh.geometry.attributes.color;
            if (data.colors && data.colors.length === colorAttr.count * 3) {
                const hasTexture = mesh.userData && mesh.userData.textureLoaded;
                if (hasTexture) {
                    // Textured chunk: grayscale intensity lets the texture show through
                    colorAttr.array.set(data.colors);
                } else {
                    // Un-textured chunk: tint the shading with the height-based
                    // terrain color so it stays green instead of showing white
                    // before the satellite texture loads.
                    const posAttr = mesh.geometry.attributes.position;
                    const arr = colorAttr.array;
                    for (let i = 0; i < colorAttr.count; i++) {
                        const intensity = data.colors[i * 3];
                        const c = getHeightColor(posAttr.getY(i));
                        arr[i * 3]     = c.r * intensity;
                        arr[i * 3 + 1] = c.g * intensity;
                        arr[i * 3 + 2] = c.b * intensity;
                    }
                }
                colorAttr.needsUpdate = true;
            }
        };

        hillshadeWorker.onerror = () => {
            hillshadeWorkerAvailable = false;
            hillshadeWorker = null;
            hillshadePending.clear();
        };
    } catch (err) {
        hillshadeWorkerAvailable = false;
        hillshadeWorker = null;
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

        textureCullWorker.onerror = () => {
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
    if (typeof Worker === 'undefined' || !rendererRef) return;

    // BC1 needs the S3TC extension. Everything desktop has it; if it is missing
    // the RGBA path still works, just with the old memory cost.
    let ext = null;
    try {
        ext = rendererRef.getContext().getExtension('WEBGL_compressed_texture_s3tc');
    } catch (e) { /* fall through to the uncompressed path */ }
    if (!ext) {
        console.warn('[terrain] S3TC unavailable — terrain textures stay uncompressed');
        return;
    }

    try {
        for (let i = 0; i < COMPRESS_WORKER_COUNT; i++) {
            const w = new Worker(new URL('./TextureCompressWorker.js', import.meta.url), { type: 'module' });
            w.onmessage = (e) => onCompressedTexture(e.data || {});
            w.onerror = () => { compressAvailable = false; };
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
 * @returns {boolean} false if the caller should fall back to an RGBA texture
 */
function requestCompressedTexture(mesh, canvas) {
    if (!compressAvailable || !compressWorkers.length) return false;
    // Below one block per axis there is nothing to gain and the padding would
    // dominate; those textures are negligible anyway.
    if (canvas.width < 8 || canvas.height < 8) return false;

    let imageData;
    try {
        imageData = canvas.getContext('2d').getImageData(0, 0, canvas.width, canvas.height);
    } catch (e) {
        return false;
    }

    const id = ++compressSeq;
    compressPending.set(id, { mesh, canvas });
    const worker = compressWorkers[compressRoundRobin++ % compressWorkers.length];
    worker.postMessage({
        id,
        rgba: imageData.data.buffer,
        width: canvas.width,
        height: canvas.height
    }, [imageData.data.buffer]);
    return true;
}

function onCompressedTexture(msg) {
    const pending = compressPending.get(msg.id);
    if (!pending) return;
    compressPending.delete(msg.id);

    const { mesh, canvas } = pending;
    const width = msg.width, height = msg.height;

    // The canvas has served its purpose: unlike a CanvasTexture, a compressed
    // texture owns its pixels, so the backing canvas can be released immediately.
    releaseCanvas(canvas);

    if (!msg.ok || !mesh || (mesh.userData && mesh.userData.disposed) || !window.satelliteEnabled) {
        if (!msg.ok) console.warn('[terrain] Texture compression failed:', msg.error);
        if (mesh && mesh.userData && !msg.ok) mesh.userData.textureLoaded = false;
        return;
    }

    const texture = new THREE.CompressedTexture(
        msg.mips, msg.padWidth, msg.padHeight, THREE.RGB_S3TC_DXT1_Format
    );
    texture.wrapS = THREE.ClampToEdgeWrapping;
    texture.wrapT = THREE.ClampToEdgeWrapping;
    texture.minFilter = THREE.LinearMipmapLinearFilter;
    texture.magFilter = THREE.LinearFilter;
    if (rendererRef) texture.anisotropy = rendererRef.capabilities.getMaxAnisotropy();
    // Level 0 was padded up to a multiple of 4; map UV 0..1 onto the real image.
    // The worker flips vertically before padding, so the padding ends up at the
    // top and right and a plain scale (no offset) is the exact correction.
    texture.repeat.set(width / msg.padWidth, height / msg.padHeight);
    texture.needsUpdate = true;

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
export function getTerrainElevationFromHGT(lat, lon) {
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
    
    if (h00 < -1000) return 0; // Filter voids
    
    const h0 = h00 * (1 - fc) + h01 * fc;
    const h1 = h10 * (1 - fc) + h11 * fc;
    return h0 * (1 - fr) + h1 * fr;
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
        // If still not available, auto-download from AWS Mapzen
        if (!file && !_autoDownloadFailed.has(filename)) {
            file = await autoDownloadSRTM(filename, latBase, lonBase);
        }
        if (file) {
            const buf = await file.arrayBuffer();
            const len = buf.byteLength;
            const size = (len === 1201 * 1201 * 2) ? 1201 : (len === 3601 * 3601 * 2 ? 3601 : 0);
            if (size && !hgtElevationData[key]) {
                const dataView = new DataView(buf);
                const elevationArray = new Int16Array(size * size);
                for (let i = 0; i < size * size; i++) {
                    elevationArray[i] = dataView.getInt16(i * 2, false);
                }
                hgtElevationData[key] = { data: elevationArray, size };
                console.log(`[terrain] Parsed elevation data for ${filename} on demand (${size}x${size})`);
            }
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
 * Add HGT file to storage
 * @param {string} filename 
 * @param {File} file 
 */
export function addHGTFile(filename, file) {
    hgtFiles[filename.toUpperCase()] = file;

    // Pre-parse elevation data so getTerrainElevationFromHGT works immediately
    const match = String(filename).toUpperCase().match(/^([NS])(\d{1,2})([EW])(\d{1,3})/);
    if (match) {
        const latSign = match[1] === 'S' ? -1 : 1;
        const lonSign = match[3] === 'W' ? -1 : 1;
        const latBase = latSign * Number(match[2]);
        const lonBase = lonSign * Number(match[4]);
        const key = `${latBase}_${lonBase}`;
        if (!hgtElevationData[key]) {
            file.arrayBuffer().then(buf => {
                const len = buf.byteLength;
                const size = (len === 1201 * 1201 * 2) ? 1201 : (len === 3601 * 3601 * 2 ? 3601 : 0);
                if (size && !hgtElevationData[key]) {
                    const dataView = new DataView(buf);
                    const elevationArray = new Int16Array(size * size);
                    for (let i = 0; i < size * size; i++) {
                        elevationArray[i] = dataView.getInt16(i * 2, false);
                    }
                    hgtElevationData[key] = { data: elevationArray, size };
                    // Invalidate cached query so next frame picks up new data
                    lastTerrainQuery = { lat: null, lon: null, height: null };
                    console.log(`[terrain] Pre-parsed elevation data for ${filename} (${size}x${size})`);
                }
            }).catch(() => {});
        }
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
export async function updateTerrainChunks() {
    const currentLat = STATE.lat;
    const currentLon = STATE.lon;

    for (let la = Math.floor(currentLat - 2); la <= Math.floor(currentLat + 2); la++) {
        for (let lo = Math.floor(currentLon - 2); lo <= Math.floor(currentLon + 2); lo++) {
            const latStr = (la >= 0 ? 'N' : 'S') + Math.abs(la).toString().padStart(2, '0');
            const lonStr = (lo >= 0 ? 'E' : 'W') + Math.abs(lo).toString().padStart(3, '0');
            const filename = `${latStr}${lonStr}.HGT`;
            if (hgtFiles[filename]) {
                processHGTFile(hgtFiles[filename], la, lo);
            } else if (availableHgtFiles.has(filename)) {
                // Lazy-load from disk, then process
                ensureHgtLoaded(filename).then(ok => {
                    if (ok && hgtFiles[filename]) {
                        processHGTFile(hgtFiles[filename], la, lo);
                    }
                });
            }
        }
    }
}

/**
 * Process HGT file and generate chunks
 * @param {File} file 
 * @param {number} latBase 
 * @param {number} lonBase 
 */
function processHGTFile(file, latBase, lonBase) {
    const reader = new FileReader();
    reader.onload = (e) => generateChunksFromBuffer(e.target.result, latBase, lonBase);
    reader.readAsArrayBuffer(file);
}

/**
 * Generate terrain chunks from HGT buffer
 * @param {ArrayBuffer} buffer 
 * @param {number} latBase 
 * @param {number} lonBase 
 */
function generateChunksFromBuffer(buffer, latBase, lonBase) {
    const len = buffer.byteLength;
    let size = (len === 1201 * 1201 * 2) ? 1201 : (len === 3601 * 3601 * 2 ? 3601 : 0);
    if (!size) return;
    
    const dataView = new DataView(buffer);
    const key = `${latBase}_${lonBase}`;
    
    if (!hgtElevationData[key]) {
        const elevationArray = new Int16Array(size * size);
        for (let i = 0; i < size * size; i++) {
            elevationArray[i] = dataView.getInt16(i * 2, false);
        }
        hgtElevationData[key] = { data: elevationArray, size: size };
    }

    const chunksPerAxis = CHUNKS_PER_TILE_AXIS;
    const vertsPerChunk = Math.floor((size - 1) / chunksPerAxis);
    const playerPos = latLonToMeters(STATE.lat, STATE.lon);

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
                    dataView: workerAvailable ? null : new DataView(buffer.slice(0))
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

    if (workerAvailable && terrainWorker) {
        try {
            terrainWorker.postMessage({ type: 'registerHgt', key, size, buffer }, [buffer]);
        } catch (err) {
            workerAvailable = false;
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
                workerPending.delete(chunkKey);
                workerInflight = Math.max(0, workerInflight - 1);
                if ((!activeChunks[chunkKey] || item.lodRebuild) && isChunkInRange(item)) {
                    chunkCreationQueue.unshift(item);
                } else if (item.lodRebuild && activeChunks[chunkKey]) {
                    // Rebuild dropped — release the flag so a later pass can retry
                    activeChunks[chunkKey].userData.lodRebuildQueued = false;
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
            
            // Avvia caricamento satellite dopo un breve delay
            if (window.satelliteEnabled) {
                setTimeout(() => {
                    resetTextureRefreshPosition();
                    refreshNearbyChunkTextures();
                }, 100);
            }
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
                if (item.lodRebuild && activeChunks[item.chunkKey]) {
                    disposeChunk(item.chunkKey, activeChunks[item.chunkKey]);
                }
                createSingleChunk(item);
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
    const { cx, cy, chunkKey, latBase, lonBase, size, vertsPerChunk, dataView, hgtKey } = item;
    const chunksPerAxis = CHUNKS_PER_TILE_AXIS;
    const step = sanitizeLodStep(item.lodStep || 1, vertsPerChunk);

    const chunkLatTop = latBase + 1 - (cy / chunksPerAxis);
    const chunkLatBottom = latBase + 1 - ((cy + 1) / chunksPerAxis);
    const chunkLonLeft = lonBase + (cx / chunksPerAxis);
    const chunkLonRight = lonBase + ((cx + 1) / chunksPerAxis);

    const geoW = vertsPerChunk / step + 1;
    const geometry = new THREE.PlaneGeometry(1, 1, geoW - 1, geoW - 1);
    const posAttr = geometry.attributes.position;
    const uvAttr = new THREE.BufferAttribute(new Float32Array(posAttr.count * 2), 2);
    const colAttr = new THREE.BufferAttribute(new Float32Array(posAttr.count * 3), 3);
    geometry.setAttribute('uv', uvAttr);
    geometry.setAttribute('color', colAttr);

    const startRow = cy * vertsPerChunk;
    const startCol = cx * vertsPerChunk;

    const hgtCache = !dataView && hgtKey ? hgtElevationData[hgtKey] : null;
    for (let r = 0; r < geoW; r++) {
        for (let c = 0; c < geoW; c++) {
            const hgtRow = Math.min(startRow + r * step, size - 1);
            const hgtCol = Math.min(startCol + c * step, size - 1);
            const height = dataView
                ? dataView.getInt16((hgtRow * size + hgtCol) * 2, false)
                : (hgtCache ? hgtCache.data[hgtRow * size + hgtCol] : 0);
            const nLat = 1.0 - (hgtRow / (size - 1));
            const nLon = hgtCol / (size - 1);
            const vertLat = latBase + nLat;
            const vertLon = lonBase + nLon;
            const wPos = latLonToMeters(vertLat, vertLon);
            const vertIdx = r * geoW + c;

            posAttr.setXYZ(vertIdx, wPos.x, height, wPos.z);

            const u = c / (geoW - 1);
            const v = 1 - (r / (geoW - 1));
            uvAttr.setXY(vertIdx, u, v);

            const col = getHeightColor(height);
            colAttr.setXYZ(vertIdx, col.r, col.g, col.b);
        }
    }

    geometry.computeVertexNormals();

    const material = new THREE.MeshLambertMaterial({
        vertexColors: true,
        side: THREE.FrontSide  // heightfield seen from above: backface culling halves rasterization
    });

    const mesh = new THREE.Mesh(geometry, material);
    mesh.castShadow = false;  // terrain doesn't need to cast shadows (saves shadow-map draw calls)
    mesh.receiveShadow = true;
    mesh.userData = {
        chunkLatTop, chunkLatBottom, chunkLonLeft, chunkLonRight,
        lodStep: step,
        textureLoaded: false
    };

    sceneRef.add(mesh);
    activeChunks[chunkKey] = mesh;
    chunksCreated++;
    markChunkActivity();

    applyInheritedTexture(item, mesh);
    applyHillshadeToMesh(mesh);
    requeueTextureAfterLodRebuild(item, mesh);

    // NON caricare satellite qui - verrà fatto da refreshNearbyChunkTextures
    // dopo che il terreno base è completamente caricato
}

/**
 * Attach the satellite texture inherited from the pre-LOD-rebuild mesh (see
 * the terrain worker chunkReady handler). Vertex colors are flattened to a
 * neutral light map right away — the height-tinted greens they start with
 * would tint the satellite texture until the async hillshade result lands.
 */
function applyInheritedTexture(item, mesh) {
    if (!item.inheritedMap) return;
    mesh.material.map = item.inheritedMap;
    mesh.material.needsUpdate = true;
    mesh.userData.textureLoaded = true;
    mesh.userData.textureZoom = item.inheritedZoom || 0;
    mesh.userData.textureBand = item.inheritedBand ?? BASE_BAND;
    item.inheritedMap = null;

    const colorAttr = mesh.geometry.attributes.color;
    if (colorAttr) {
        const neutral = (window.sunlightEnabled !== false) ? 0.85 : mapBrightness;
        colorAttr.array.fill(neutral);
        colorAttr.needsUpdate = true;
    }
}

/**
 * After a LOD rebuild, the fresh mesh lost its satellite texture — re-enqueue
 * it right away instead of waiting for the next movement-based refresh.
 * (No-op when the texture was inherited: textureLoaded is already true.)
 */
function requeueTextureAfterLodRebuild(item, mesh) {
    if (!item.lodRebuild || !initialTexturesLoaded || !window.satelliteEnabled) return;
    const dist = getChunkDistanceToPlayer(item);
    if (dist <= SATELLITE_RADIUS) {
        enqueueChunkTexture(mesh, mesh.userData, dist);
    }
}

function createSingleChunkFromBuffers(item, positions, uvs, colors, normals) {
    const { cx, cy, chunkKey, latBase, lonBase, vertsPerChunk } = item;
    const chunksPerAxis = CHUNKS_PER_TILE_AXIS;
    const step = sanitizeLodStep(item.lodStep || 1, vertsPerChunk);

    const chunkLatTop = latBase + 1 - (cy / chunksPerAxis);
    const chunkLatBottom = latBase + 1 - ((cy + 1) / chunksPerAxis);
    const chunkLonLeft = lonBase + (cx / chunksPerAxis);
    const chunkLonRight = lonBase + ((cx + 1) / chunksPerAxis);

    const geoW = vertsPerChunk / step + 1;
    const geometry = new THREE.PlaneGeometry(1, 1, geoW - 1, geoW - 1);
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geometry.setAttribute('uv', new THREE.BufferAttribute(uvs, 2));
    geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    if (normals && normals.length === positions.length) {
        // Normals computed in the worker — avoids a 10-30ms main-thread stall per chunk
        geometry.setAttribute('normal', new THREE.BufferAttribute(normals, 3));
    } else {
        geometry.computeVertexNormals();
    }

    const material = new THREE.MeshLambertMaterial({
        vertexColors: true,
        side: THREE.FrontSide  // heightfield seen from above: backface culling halves rasterization
    });

    const mesh = new THREE.Mesh(geometry, material);
    mesh.castShadow = false;  // terrain doesn't need to cast shadows (saves shadow-map draw calls)
    mesh.receiveShadow = true;
    mesh.userData = {
        chunkLatTop, chunkLatBottom, chunkLonLeft, chunkLonRight,
        lodStep: step,
        textureLoaded: false
    };

    sceneRef.add(mesh);
    activeChunks[chunkKey] = mesh;
    chunksCreated++;
    markChunkActivity();

    applyInheritedTexture(item, mesh);
    applyHillshadeToMesh(mesh);
    requeueTextureAfterLodRebuild(item, mesh);
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
        mesh.userData.textureLoaded = false;
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

    const canvas = document.createElement('canvas');
    canvas.width = cropW;
    canvas.height = cropH;
    const ctx = canvas.getContext('2d');
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
    mesh.userData.textureZoom = zoomLevel;
    mesh.userData.textureBand = band;

    for (let ty = tileTopLeft.y; ty <= tileBottomRight.y; ty++) {
        for (let tx = tileTopLeft.x; tx <= tileBottomRight.x; tx++) {
            const localX = tx - tileTopLeft.x;
            const localY = ty - tileTopLeft.y;

            loadTileImage(tx, ty, zoomLevel, (img) => {
                tilesLoaded++; // Incrementa contatore globale (loading overlay)
                enqueueTileDraw(chunkJob, img, localX, localY, TILE_SIZE);
            });
        }
    }
}

function enqueueTileDraw(job, img, localX, localY, tileSize) {
    if (!window.satelliteEnabled || !job || !job.ctx || job.aborted) return;
    tileDrawQueue.push({ job, img, localX, localY, tileSize });
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
        const { job, img, localX, localY, tileSize } = tileDrawQueue.shift();

        // Skip aborted jobs
        if (!job || job.aborted || !job.ctx) {
            continue;
        }

        // The ImageBitmap may have been evicted + closed by imageLRU while
        // queued (detached → width/height become 0). Drawing it throws an
        // uncaught InvalidStateError, which would kill the queue loop and
        // leave isProcessingTileDrawQueue stuck true, stalling all future
        // tile draws. Guard against it and just skip the tile.
        if (img && img.width > 0 && img.height > 0) {
            try {
                // Draw in mosaic space shifted by the crop origin — out-of-bounds
                // portions are clipped by the canvas for free.
                job.ctx.drawImage(img, localX * tileSize - job.cropX, localY * tileSize - job.cropY, tileSize, tileSize);
            } catch (e) {
                // Detached/invalid image source — skip; chunk re-textures on next refresh
            }
        }
        job.tilesDrawn++;
        if (job.tilesDrawn >= job.totalTiles) {
            // Remove from active jobs tracking
            activeChunkJobs.delete(job.mesh.uuid);
            // Nullify ctx to prevent further draws
            const canvas = job.canvas;
            job.ctx = null;
            job.canvas = null;
            enqueueCompositeTexture(job.mesh, canvas);
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
    if (!mesh || (mesh.userData && mesh.userData.disposed)) {
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
        callback(cached);
        return;
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
                    console.warn(`${CONSECUTIVE_ERROR_THRESHOLD} consecutive tile errors — connection lost, disabling satellite`);
                    window.satelliteEnabled = false;
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
        if (mesh && mesh.userData) mesh.userData.textureLoaded = false;
        return;
    }

    if (!mesh || (mesh.userData && mesh.userData.disposed)) {
        releaseCanvas(canvas);
        return;
    }

    // Preferred path: compress to BC1 off-thread. The texture is attached when
    // the worker answers; until then the chunk keeps whatever it already had.
    if (requestCompressedTexture(mesh, canvas)) return;

    const texture = new THREE.CanvasTexture(canvas);
    texturesCreated++;

    // Force an early GPU upload. IMPORTANT: the canvas must stay alive — it
    // is texture.image, the source THREE re-reads on every re-upload (e.g.
    // after a WebGL context loss/restore). Shrinking it here permanently
    // blanked the texture to white. It is GC'd naturally when the texture is
    // disposed (unloadChunkTexture / LOD swap), so there is no leak.
    if (rendererRef) {
        rendererRef.initTexture(texture);
    }
    texture.wrapS = THREE.ClampToEdgeWrapping;
    texture.wrapT = THREE.ClampToEdgeWrapping;
    texture.minFilter = THREE.LinearMipmapLinearFilter;
    texture.magFilter = THREE.LinearFilter;
    if (rendererRef) {
        texture.anisotropy = rendererRef.capabilities.getMaxAnisotropy();
    }

    attachTextureToMesh(mesh, texture);
}

/**
 * Put a finished texture (compressed or not) on a chunk.
 */
function attachTextureToMesh(mesh, texture) {
    if (mesh && mesh.material) {
        // First texture on a green (height-tinted) chunk: flatten the vertex
        // colors to a neutral light map right away, otherwise the satellite
        // imagery shows green-tinted until the async hillshade result lands.
        // On re-texture (LOD swap) the colors are already a valid light map.
        if (!mesh.userData.textureLoaded && mesh.geometry &&
            mesh.geometry.attributes && mesh.geometry.attributes.color) {
            const colorAttr = mesh.geometry.attributes.color;
            const neutral = (window.sunlightEnabled !== false) ? 0.85 : mapBrightness;
            colorAttr.array.fill(neutral);
            colorAttr.needsUpdate = true;
        }

        const prevMaterial = mesh.material;
        if (prevMaterial.map) {
            try { prevMaterial.map.dispose(); texturesDisposed++; } catch (e) {}
            prevMaterial.map = null;
        }
        // Reuse material, just update the texture map (avoid allocation + GPU rebind)
        prevMaterial.map = texture;
        prevMaterial.needsUpdate = true;
        mesh.castShadow = false;
        mesh.receiveShadow = true;
        mesh.userData.textureLoaded = true;

        // Apply hillshading
        applyHillshadeToMesh(mesh);
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
    if (!mesh || !mesh.material) return;

    abortChunkJob(mesh);

    const hadTexture = !!mesh.material.map;
    if (mesh.material.map) {
        // The texture's backing canvas is released along with it
        if (mesh.material.map.image) canvasesReleased++;
        try { mesh.material.map.dispose(); texturesDisposed++; } catch (e) {}
        mesh.material.map = null;
        mesh.material.needsUpdate = true;
    }
    if (mesh.userData) {
        mesh.userData.textureLoaded = false;
        mesh.userData.textureQueued = false;
        mesh.userData.textureZoom = 0;
        mesh.userData.textureBand = undefined;
    }

    // The vertex colors are still the grayscale "light map" computed for the
    // textured state — without re-shading, the naked chunk renders white
    // (visible as white far chunks after the texture cull at high altitude).
    // Re-apply hillshade so it goes back to the green height-tinted look.
    if (hadTexture && !(mesh.userData && mesh.userData.disposed)) {
        applyHillshadeToMesh(mesh);
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

/**
 * Enable/disable satellite textures on existing terrain chunks.
 * When disabling, removes any already-applied textures so the overlay actually disappears.
 * When enabling, schedules texture generation for chunks that don't have it yet.
 * @param {boolean} enabled
 */
export function setTerrainChunksVisible(visible) {
    for (const key in activeChunks) {
        if (activeChunks[key]) activeChunks[key].visible = visible;
    }
}

export function setTerrainSatelliteEnabled(enabled) {
    const on = !!enabled;
    if (!activeChunks) return;

    for (const key in activeChunks) {
        const mesh = activeChunks[key];
        if (!mesh || !mesh.material) continue;

        if (!on) {
            unloadChunkTexture(mesh);
            continue;
        }

        // Enabling: schedule satellite textures (wireframe managed by updateWireframeProximity)
        const ud = mesh.userData || {};
        if (!ud.textureLoaded && ud.chunkLatTop != null) {
            createChunkTexture(mesh, ud.chunkLatTop, ud.chunkLatBottom, ud.chunkLonLeft, ud.chunkLonRight);
        }
    }

    if (!on) {
        clearPendingTextureOperations();
        try { imageLRU.clear(); } catch (e) {}
        tileLoadQueue.length = 0;
        pendingTileCallbacks.clear();
        currentTileLoads = 0;
    }
}

/**
 * Apply hillshade to a single mesh
 */
function applyHillshadeToMesh(mesh) {
    const geometry = mesh.geometry;
    const posAttr = geometry.attributes.position;
    const normalAttr = geometry.attributes.normal;
    const colorAttr = geometry.attributes.color;

    if (!posAttr || !normalAttr || !colorAttr) return;

    const sunlightEnabled = window.sunlightEnabled !== false;
    const sunDir = currentSunDirectionRef || new THREE.Vector3(0, 1, 0);

    if (hillshadeWorkerAvailable && hillshadeWorker) {
        const normalsCopy = new Float32Array(normalAttr.array);
        // Monotonic per-mesh sequence: lets the response handler drop stale
        // results when a newer request superseded this one (e.g. texture
        // applied/unloaded while the previous compute was in flight).
        const seq = (mesh.userData.hillshadeSeq = (mesh.userData.hillshadeSeq || 0) + 1);
        hillshadePending.set(mesh.uuid, mesh);
        hillshadeWorker.postMessage({
            type: 'computeHillshade',
            meshId: mesh.uuid,
            seq,
            normals: normalsCopy,
            sunDir: { x: sunDir.x, y: sunDir.y, z: sunDir.z },
            sunlightEnabled,
            brightness: mapBrightness
        }, [normalsCopy.buffer]);
        return;
    }

    const hasTexture = mesh.userData && mesh.userData.textureLoaded;
    const normal = new THREE.Vector3();
    for (let i = 0; i < posAttr.count; i++) {
        normal.set(normalAttr.getX(i), normalAttr.getY(i), normalAttr.getZ(i));
        let intensity = normal.dot(sunDir);
        if (sunlightEnabled) {
            intensity = Math.max(0, intensity);
            intensity = 0.45 + intensity * 1.05;
        } else {
            intensity = mapBrightness;
        }
        if (hasTexture) {
            colorAttr.setXYZ(i, intensity, intensity, intensity);
        } else {
            // Un-textured chunk: keep the standard green terrain color
            const c = getHeightColor(posAttr.getY(i));
            colorAttr.setXYZ(i, c.r * intensity, c.g * intensity, c.b * intensity);
        }
    }
    colorAttr.needsUpdate = true;
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
        const ud = mesh.userData;
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
                const ud = mesh.userData;
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
        const ud = mesh.userData;
        if (!ud || ud.chunkLatTop == null || !ud.lodStep || ud.lodRebuildQueued) continue;
        if (workerPending.has(key)) continue;

        const centerLat = (ud.chunkLatTop + ud.chunkLatBottom) / 2;
        const centerLon = (ud.chunkLonLeft + ud.chunkLonRight) / 2;
        const centerWorld = latLonToMeters(centerLat, centerLon);
        const dist = Math.sqrt(
            (centerWorld.x - playerPos.x) ** 2 +
            (centerWorld.z - playerPos.z) ** 2
        );

        const desired = lodStepForDistance(dist);
        if (desired === ud.lodStep) continue;

        if (desired < ud.lodStep) {
            // Upgrade only when firmly inside the finer band
            if (lodStepForDistance(dist * 1.15) < ud.lodStep) {
                rebuilds.push({ key, mesh, dist, upgrade: 1 });
            }
        } else {
            // Downgrade only when firmly outside the current band
            if (lodStepForDistance(dist * 0.85) > ud.lodStep) {
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
    const vertsPerChunk = Math.floor((size - 1) / 10);

    mesh.userData.lodRebuildQueued = true;
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
    // full session lifetime. Each 1°×1° SRTM tile is ~2.9 MB parsed; keeping
    // them avoids repeated AWS downloads and re-parse overhead.
}

/**
 * Dispose a single chunk
 */
function disposeChunk(key, mesh) {
    if (!mesh) return;

    if (mesh.userData) mesh.userData.disposed = true;
    hillshadePending.delete(mesh.uuid);

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

    // Remove wireframe overlay (geometry & material are shared — don't dispose them)
    if (mesh.userData._wireframe) {
        mesh.remove(mesh.userData._wireframe);
        mesh.userData._wireframe = null;
    }
    if (wireframeChunkKey === key) wireframeChunkKey = null;

    if (mesh.material) {
        if (mesh.material.map) {
            try { mesh.material.map.dispose(); texturesDisposed++; } catch (e) {}
        }
        try { mesh.material.dispose(); } catch (e) {}
    }
    if (mesh.geometry) {
        try { mesh.geometry.dispose(); } catch (e) {}
    }
    if (sceneRef) {
        sceneRef.remove(mesh);
    }
    delete activeChunks[key];
    chunksDisposed++;
}

/**
 * Update terrain hillshading
 * @param {boolean} forceUpdate - Force full update
 */
export function updateTerrainHillshading(forceUpdate = false) {
    if (!activeChunks) return;

    if (currentSunDirectionRef) {
        cachedSunDir.copy(currentSunDirectionRef);
    } else {
        cachedSunDir.set(0, 1, 0);
    }

    if (!forceUpdate && !hillshadeNeedsFullUpdate) {
        const dx = cachedSunDir.x - lastSunDirX;
        const dy = cachedSunDir.y - lastSunDirY;
        const dz = cachedSunDir.z - lastSunDirZ;
        if (dx * dx + dy * dy + dz * dz < 0.0001) return;
    }

    lastSunDirX = cachedSunDir.x;
    lastSunDirY = cachedSunDir.y;
    lastSunDirZ = cachedSunDir.z;
    hillshadeNeedsFullUpdate = false;

    if (hillshadeUpdatePending) return;
    hillshadeUpdatePending = true;

    requestAnimationFrame(() => {
        hillshadeUpdatePending = false;
        performHillshadeUpdate();
    });
}

/**
 * Perform the actual hillshade update.
 * Instead of recomputing every vertex of every chunk synchronously (a single
 * multi-hundred-ms main-thread stall when the sun moves), chunks are queued
 * and dispatched a few per frame to the HillshadeWorker via
 * applyHillshadeToMesh(). A new sun update simply refills the queue.
 */
const hillshadeChunkQueue = [];
let isProcessingHillshadeQueue = false;
const MAX_HILLSHADE_CHUNKS_PER_FRAME = 3;

function performHillshadeUpdate() {
    hillshadeChunkQueue.length = 0;
    for (const key in activeChunks) {
        hillshadeChunkQueue.push(key);
    }

    if (!isProcessingHillshadeQueue && hillshadeChunkQueue.length > 0) {
        isProcessingHillshadeQueue = true;
        requestAnimationFrame(processHillshadeQueue);
    }
}

function processHillshadeQueue() {
    const sunlightEnabled = window.sunlightEnabled !== false;
    let processed = 0;

    while (hillshadeChunkQueue.length > 0 && processed < MAX_HILLSHADE_CHUNKS_PER_FRAME) {
        const key = hillshadeChunkQueue.shift();
        const mesh = activeChunks[key];
        if (!mesh || !mesh.geometry) continue;

        if (sunlightEnabled && mesh.material && mesh.material.color) {
            mesh.material.color.setRGB(1, 1, 1);
        }
        applyHillshadeToMesh(mesh);
        processed++;
    }

    if (hillshadeChunkQueue.length > 0) {
        requestAnimationFrame(processHillshadeQueue);
    } else {
        isProcessingHillshadeQueue = false;
    }
}

/**
 * Set hillshade needs full update flag
 */
export function setHillshadeNeedsUpdate() {
    hillshadeNeedsFullUpdate = true;
}

export function setMapBrightness(value) {
    const v = Number(value);
    if (!Number.isFinite(v)) return;
    mapBrightness = Math.max(0.3, Math.min(1.6, v));
    if (window.sunlightEnabled === false) {
        applyMaterialBrightness(mapBrightness);
    }
}

function applyMaterialBrightness(scale) {
    const s = Math.max(0.1, Math.min(2.0, scale));
    for (const key in activeChunks) {
        const mesh = activeChunks[key];
        if (!mesh || !mesh.material || !mesh.material.color) continue;
        mesh.material.color.setRGB(s, s, s);
        mesh.material.needsUpdate = true;
    }
}

// Getters
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
        if (!mesh || !mesh.userData) continue;

        const ud = mesh.userData;
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
