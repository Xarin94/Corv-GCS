/**
 * TerrainManager.js - Terrain Loading and Chunk Management
 * Handles HGT file loading, terrain chunk generation, and elevation queries
 */

import { VISIBILITY_RADIUS, RELOAD_DISTANCE } from '../core/constants.js';

// Tile zoom levels
const TILE_ZOOM = 15;       // ~5m/pixel - standard definition for all chunks
const HD_TILE_ZOOM = 17;    // ~1.2m/pixel - high definition for nearby chunks
const HD_RADIUS = 2000;     // 2km - radius for high-res satellite textures
const MAX_CANVAS_DIM = 4096; // Max texture dimension (GPU limit)
const SATELLITE_RADIUS = 10000; // 10km - raggio della mappa satellitare (in metri)
import { STATE } from '../core/state.js';
import { latLonToMeters, calculateDistance, getHeightColor, latLonToTile, tileToBounds } from '../core/utils.js';
import { LRUCache } from '../core/LRUCache.js';

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
        pendingTileCallbacksCount: pendingTileCallbacks.size
    };
}

// Terrain data storage
const hgtFiles = {};
const hgtElevationData = {};
const activeChunks = {};
const runwayObjects = [];

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
const MAX_CHUNK_TEXTURES_PER_FRAME = 2;

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

// Cleanup settings (più aggressivi)
const CLEANUP_RADIUS = VISIBILITY_RADIUS * 1.05; // poco oltre la visibilità
const MAX_ACTIVE_CHUNKS = 240; // limite più basso per liberare memoria
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
    const chunksPerAxis = 10;
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
    cachedSunDir = new THREE.Vector3(0, 1, 0);

    initTerrainWorker();
    initTileWorker();
    initHillshadeWorker();
    initTextureCullWorker();
    
    // Start cleanup interval
    setInterval(cleanupDistantChunks, 5000);
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

                if (!activeChunks[data.chunkKey] && isChunkInRange(item)) {
                    createSingleChunkFromBuffers(item, data.positions, data.uvs, data.colors);
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
                if (!activeChunks[item.chunkKey]) {
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
            hillshadePending.delete(data.meshId);
            if (!mesh || !mesh.geometry || !mesh.geometry.attributes || !mesh.geometry.attributes.color) return;

            const colorAttr = mesh.geometry.attributes.color;
            if (data.colors && data.colors.length === colorAttr.count * 3) {
                colorAttr.array.set(data.colors);
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
        const file = hgtFiles[filename];
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
    
    for (let la = Math.floor(currentLat - 1); la <= Math.floor(currentLat + 1); la++) {
        for (let lo = Math.floor(currentLon - 1); lo <= Math.floor(currentLon + 1); lo++) {
            const latStr = (la >= 0 ? 'N' : 'S') + Math.abs(la).toString().padStart(2, '0');
            const lonStr = (lo >= 0 ? 'E' : 'W') + Math.abs(lo).toString().padStart(3, '0');
            const filename = `${latStr}${lonStr}.HGT`;
            if (hgtFiles[filename]) {
                processHGTFile(hgtFiles[filename], la, lo);
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

    const chunksPerAxis = 10;
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
                if (!activeChunks[chunkKey] && isChunkInRange(item)) {
                    chunkCreationQueue.unshift(item);
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
            if (!activeChunks[item.chunkKey]) {
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
                    cx: item.cx,
                    cy: item.cy
                });
                scheduled++;
            }
        }
    } else {
        for (let i = 0; i < CHUNKS_PER_FRAME && chunkCreationQueue.length > 0; i++) {
            const item = chunkCreationQueue.shift();
            if (!activeChunks[item.chunkKey]) {
                if (!isChunkInRange(item)) {
                    continue;
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
    const chunksPerAxis = 10;

    const chunkLatTop = latBase + 1 - (cy / chunksPerAxis);
    const chunkLatBottom = latBase + 1 - ((cy + 1) / chunksPerAxis);
    const chunkLonLeft = lonBase + (cx / chunksPerAxis);
    const chunkLonRight = lonBase + ((cx + 1) / chunksPerAxis);

    const geoW = vertsPerChunk + 1;
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
            const hgtRow = Math.min(startRow + r, size - 1);
            const hgtCol = Math.min(startCol + c, size - 1);
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
        side: THREE.DoubleSide
    });

    const mesh = new THREE.Mesh(geometry, material);
    mesh.castShadow = false;  // terrain doesn't need to cast shadows (saves shadow-map draw calls)
    mesh.receiveShadow = true;
    mesh.userData = { 
        chunkLatTop, chunkLatBottom, chunkLonLeft, chunkLonRight, 
        textureLoaded: false 
    };
    
    sceneRef.add(mesh);
    activeChunks[chunkKey] = mesh;
    chunksCreated++;
    markChunkActivity();

    applyHillshadeToMesh(mesh);

    // NON caricare satellite qui - verrà fatto da refreshNearbyChunkTextures
    // dopo che il terreno base è completamente caricato
}

function createSingleChunkFromBuffers(item, positions, uvs, colors) {
    const { cx, cy, chunkKey, latBase, lonBase, vertsPerChunk } = item;
    const chunksPerAxis = 10;

    const chunkLatTop = latBase + 1 - (cy / chunksPerAxis);
    const chunkLatBottom = latBase + 1 - ((cy + 1) / chunksPerAxis);
    const chunkLonLeft = lonBase + (cx / chunksPerAxis);
    const chunkLonRight = lonBase + ((cx + 1) / chunksPerAxis);

    const geoW = vertsPerChunk + 1;
    const geometry = new THREE.PlaneGeometry(1, 1, geoW - 1, geoW - 1);
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geometry.setAttribute('uv', new THREE.BufferAttribute(uvs, 2));
    geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    geometry.computeVertexNormals();

    const material = new THREE.MeshLambertMaterial({
        vertexColors: true,
        side: THREE.DoubleSide
    });

    const mesh = new THREE.Mesh(geometry, material);
    mesh.castShadow = false;  // terrain doesn't need to cast shadows (saves shadow-map draw calls)
    mesh.receiveShadow = true;
    mesh.userData = {
        chunkLatTop, chunkLatBottom, chunkLonLeft, chunkLonRight,
        textureLoaded: false
    };

    sceneRef.add(mesh);
    activeChunks[chunkKey] = mesh;
    chunksCreated++;
    markChunkActivity();

    applyHillshadeToMesh(mesh);
}

/**
 * Get zoom level for chunk based on distance from aircraft
 * Returns TILE_ZOOM until initial base textures are loaded, then HD for nearby chunks
 */
function getZoomForChunk(latTop, latBottom, lonLeft, lonRight) {
    if (!initialTexturesLoaded) return TILE_ZOOM;
    const centerLat = (latTop + latBottom) / 2;
    const centerLon = (lonLeft + lonRight) / 2;
    const dist = calculateDistance(STATE.lat, STATE.lon, centerLat, centerLon);
    return dist <= HD_RADIUS ? HD_TILE_ZOOM : TILE_ZOOM;
}

/**
 * Create composite texture for a terrain chunk
 */
function createChunkTexture(mesh, latTop, latBottom, lonLeft, lonRight) {
    if (!window.satelliteEnabled) {
        mesh.userData.textureLoaded = false;
        return;
    }

    // Use appropriate zoom level based on distance, with canvas size cap
    let zoomLevel = getZoomForChunk(latTop, latBottom, lonLeft, lonRight);

    // Reduce zoom if canvas would exceed GPU texture limits
    const TILE_SIZE = 256;
    while (zoomLevel > TILE_ZOOM) {
        const tl = latLonToTile(latTop, lonLeft, zoomLevel);
        const br = latLonToTile(latBottom, lonRight, zoomLevel);
        const w = (br.x - tl.x + 1) * TILE_SIZE;
        const h = (br.y - tl.y + 1) * TILE_SIZE;
        if (w <= MAX_CANVAS_DIM && h <= MAX_CANVAS_DIM) break;
        zoomLevel--;
    }

    const tileTopLeft = latLonToTile(latTop, lonLeft, zoomLevel);
    const tileBottomRight = latLonToTile(latBottom, lonRight, zoomLevel);

    const tilesX = tileBottomRight.x - tileTopLeft.x + 1;
    const tilesY = tileBottomRight.y - tileTopLeft.y + 1;

    const canvasWidth = tilesX * TILE_SIZE;
    const canvasHeight = tilesY * TILE_SIZE;
    const canvas = document.createElement('canvas');
    canvas.width = canvasWidth;
    canvas.height = canvasHeight;
    const ctx = canvas.getContext('2d');
    canvasesCreated++;

    const totalTilesForChunk = tilesX * tilesY;
    const chunkJob = {
        mesh,
        canvas,
        ctx,
        latTop,
        latBottom,
        lonLeft,
        lonRight,
        tileTopLeft,
        tileBottomRight,
        zoomLevel,
        totalTiles: totalTilesForChunk,
        tilesDrawn: 0,
        aborted: false
    };
    
    // Track active job for cleanup
    activeChunkJobs.set(mesh.uuid, chunkJob);
    
    // Aggiungi al contatore globale delle tile da caricare
    totalTilesToLoad += totalTilesForChunk;

    // Store zoom level used for this texture
    mesh.userData.textureZoom = zoomLevel;

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

        if (img) {
            job.ctx.drawImage(img, localX * tileSize, localY * tileSize, tileSize, tileSize);
        }
        job.tilesDrawn++;
        if (job.tilesDrawn >= job.totalTiles) {
            // Remove from active jobs tracking
            activeChunkJobs.delete(job.mesh.uuid);
            // Nullify ctx to prevent further draws
            const canvas = job.canvas;
            job.ctx = null;
            job.canvas = null;
            enqueueCompositeTexture(job.mesh, canvas, job.latTop, job.latBottom, job.lonLeft, job.lonRight, job.tileTopLeft, job.tileBottomRight, job.zoomLevel);
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

function enqueueCompositeTexture(mesh, canvas, latTop, latBottom, lonLeft, lonRight, tileTopLeft, tileBottomRight, zoomLevel) {
    if (!mesh || (mesh.userData && mesh.userData.disposed)) {
        if (canvas) {
            canvas.width = 1;
            canvas.height = 1;
            canvasesReleased++;
        }
        return;
    }
    textureApplyQueue.push({ mesh, canvas, latTop, latBottom, lonLeft, lonRight, tileTopLeft, tileBottomRight, zoomLevel });
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
            // Unload existing texture if this is a LOD swap
            if (ud.textureLoaded) {
                unloadChunkTexture(mesh);
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
        applyCompositeTexture(
            job.mesh,
            job.canvas,
            job.latTop,
            job.latBottom,
            job.lonLeft,
            job.lonRight,
            job.tileTopLeft,
            job.tileBottomRight,
            job.zoomLevel
        );
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

    // Check cache first
    const cached = imageLRU.get(key);
    if (cached) {
        callback(cached);
        return;
    }

    if (!enqueueTileCallback(key, callback)) return;

    // Add to queue
    tileLoadQueue.push({ tileX, tileY, tileZ, key, callback });
    
    // Start processing queue if not already running
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
                url: `https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/${item.tileZ}/${item.tileY}/${item.tileX}`
            });
        } else {
            const img = new Image();
            img.crossOrigin = 'anonymous';
            
            img.onload = () => {
                imageLRU.set(item.key, img);
                consecutiveTileErrors = 0;
                resolveTileCallbacks(item.key, img);
                currentTileLoads--;
                processTileLoadQueue();
            };

            img.onerror = () => {
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
            };
            
            img.src = `https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/${item.tileZ}/${item.tileY}/${item.tileX}`;
        }
    }
}

/**
 * Apply composite texture to mesh
 */
function applyCompositeTexture(mesh, canvas, latTop, latBottom, lonLeft, lonRight, tileTopLeft, tileBottomRight, zoomLevel) {
    // If satellite got disabled while tiles were loading, don't apply.
    if (!window.satelliteEnabled) {
        if (mesh && mesh.userData) mesh.userData.textureLoaded = false;
        return;
    }

    if (!mesh || (mesh.userData && mesh.userData.disposed)) {
        if (canvas) {
            canvas.width = 1;
            canvas.height = 1;
            canvasesReleased++;
        }
        return;
    }

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

    const cropX = Math.floor(uMin * canvas.width);
    const cropY = Math.floor(vMin * canvas.height);
    const cropW = Math.floor((uMax - uMin) * canvas.width);
    const cropH = Math.floor((vMax - vMin) * canvas.height);

    const croppedCanvas = document.createElement('canvas');
    croppedCanvas.width = Math.max(1, cropW);
    croppedCanvas.height = Math.max(1, cropH);
    const croppedCtx = croppedCanvas.getContext('2d');

    if (cropW > 0 && cropH > 0) {
        croppedCtx.drawImage(canvas, cropX, cropY, cropW, cropH, 0, 0, cropW, cropH);
    }

    // Release source canvas memory immediately
    canvas.width = 1;
    canvas.height = 1;
    canvasesReleased++;

    const texture = new THREE.CanvasTexture(croppedCanvas);
    texturesCreated++;
    
    // Force GPU upload and release cropped canvas memory
    if (rendererRef) {
        rendererRef.initTexture(texture);
    }
    croppedCanvas.width = 1;
    croppedCanvas.height = 1;
    texture.wrapS = THREE.ClampToEdgeWrapping;
    texture.wrapT = THREE.ClampToEdgeWrapping;
    texture.minFilter = THREE.LinearMipmapLinearFilter;
    texture.magFilter = THREE.LinearFilter;
    if (rendererRef) {
        texture.anisotropy = rendererRef.capabilities.getMaxAnisotropy();
    }

    if (mesh && mesh.material) {
        const prevMaterial = mesh.material;
        if (prevMaterial.map) {
            try { prevMaterial.map.dispose(); texturesDisposed++; } catch (e) {}
            prevMaterial.map = null;
        }
        prevMaterial.dispose();
        mesh.material = new THREE.MeshLambertMaterial({
            map: texture,
            side: THREE.DoubleSide,
            vertexColors: true
        });
        mesh.castShadow = false;
        mesh.receiveShadow = true;
        mesh.material.needsUpdate = true;
        mesh.userData.textureLoaded = true;

        // Apply hillshading
        applyHillshadeToMesh(mesh);
    }
}

function unloadChunkTexture(mesh) {
    if (!mesh || !mesh.material) return;
    
    // Abort any pending job for this mesh
    const job = activeChunkJobs.get(mesh.uuid);
    if (job) {
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
    
    if (mesh.material.map) {
        try { mesh.material.map.dispose(); texturesDisposed++; } catch (e) {}
        mesh.material.map = null;
        mesh.material.needsUpdate = true;
    }
    if (mesh.userData) {
        mesh.userData.textureLoaded = false;
        mesh.userData.textureQueued = false;
        mesh.userData.textureZoom = 0;
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
        hillshadePending.set(mesh.uuid, mesh);
        hillshadeWorker.postMessage({
            type: 'computeHillshade',
            meshId: mesh.uuid,
            normals: normalsCopy,
            sunDir: { x: sunDir.x, y: sunDir.y, z: sunDir.z },
            sunlightEnabled,
            brightness: mapBrightness
        }, [normalsCopy.buffer]);
        return;
    }

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
        colorAttr.setXYZ(i, intensity, intensity, intensity);
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

    // Prune HGT cache far from player to free memory
    cleanupHgtCache();
    
}

/**
 * Cleanup HGT elevation cache far from player
 */
function cleanupHgtCache() {
    const playerPos = latLonToMeters(STATE.lat, STATE.lon);
    const keys = Object.keys(hgtElevationData);
    for (const key of keys) {
        const parts = key.split('_');
        if (parts.length < 2) continue;
        const latBase = Number(parts[0]);
        const lonBase = Number(parts[1]);
        if (!Number.isFinite(latBase) || !Number.isFinite(lonBase)) continue;
        const centerWorld = latLonToMeters(latBase + 0.5, lonBase + 0.5);
        const dx = centerWorld.x - playerPos.x;
        const dz = centerWorld.z - playerPos.z;
        const dist = Math.sqrt(dx * dx + dz * dz);
        if (dist > HGT_CACHE_RADIUS) {
            delete hgtElevationData[key];
        }
    }
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
 * Perform the actual hillshade update
 */
function performHillshadeUpdate() {
    const sunDir = cachedSunDir;
    const normal = new THREE.Vector3();
    const sunlightEnabled = window.sunlightEnabled !== false;

    for (const key in activeChunks) {
        const mesh = activeChunks[key];
        if (!mesh || !mesh.geometry) continue;

        const geometry = mesh.geometry;
        const posAttr = geometry.attributes.position;
        const normalAttr = geometry.attributes.normal;
        const colorAttr = geometry.attributes.color;

        if (!posAttr || !normalAttr || !colorAttr) continue;

        const hasTexture = mesh.userData && mesh.userData.textureLoaded;
        const count = posAttr.count;

        for (let i = 0; i < count; i++) {
            normal.set(normalAttr.getX(i), normalAttr.getY(i), normalAttr.getZ(i));

            let intensity = normal.dot(sunDir);

            if (sunlightEnabled) {
                intensity = Math.max(0, intensity);
                intensity = 0.6 + intensity * 0.8;
                if (mesh.material && mesh.material.color) {
                    mesh.material.color.setRGB(1, 1, 1);
                }
            } else {
                intensity = mapBrightness;
            }

            if (hasTexture) {
                colorAttr.setXYZ(i, intensity, intensity, intensity);
            } else {
                const height = posAttr.getY(i);
                const baseColor = getHeightColor(height);
                colorAttr.setXYZ(i,
                    baseColor.r * intensity,
                    baseColor.g * intensity,
                    baseColor.b * intensity
                );
            }
        }

        colorAttr.needsUpdate = true;
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
    const hdUpgradeRadius = HD_RADIUS * 0.9;     // hysteresis: upgrade inside this
    const hdDowngradeRadius = HD_RADIUS * 1.1;   // hysteresis: downgrade outside this

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
            // LOD swap only after initial base textures are loaded
            if (dist <= hdUpgradeRadius && ud.textureZoom !== HD_TILE_ZOOM) {
                // Close chunk with low-res texture — upgrade to HD
                chunksToUpgrade.push({ mesh, ud, dist });
            } else if (dist > hdDowngradeRadius && ud.textureZoom === HD_TILE_ZOOM) {
                // Far chunk with HD texture — downgrade to save memory
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
