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
import { latLonToMeters, calculateDistance, latLonToTile, tileToBounds } from '../core/utils.js';
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
        heightLayers: [...heightStores.values()].reduce((n, st) => n + st.layers.size, 0),
        heightMB: +([...heightStores.values()].reduce((n, st) => n + st.capacity * st.geoW * st.geoW * 2, 0) / 1048576).toFixed(1),
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
let hgtParsing = 0; // FileReader passes in flight (HGT → chunk queue)
const hgtReadInProgress = new Set(); // tile keys with a FileReader pass in flight

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
// Tiles whose buffer has been transferred to the terrain worker. The transfer
// detaches the buffer, so re-sending one is both wasteful and impossible.
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

// ============== GPU TERRAIN ==============
// Chunk geometry lives on the GPU as elevation only. A chunk's samples (Int16,
// straight from the HGT grid) are one layer of a texture array per grid size,
// and the vertex shader rebuilds each vertex position and its normal from them
// with texelFetch. Every chunk built on the same grid shares one triangle list
// and one UV set.
//
// Chunks without a satellite map (everything beyond SATELLITE_RADIUS: ~390 of
// ~420 resident) are drawn as instances of one InstancedMesh per grid — a
// handful of draw calls — after a per-chunk frustum test in
// updateTerrainInstances(). Chunks with a map keep a mesh of their own (the map
// differs per chunk), on the same shared grid geometry. This replaced ~420
// meshes, each with its own position and normal buffers, which cost three.js a
// culling test, a matrix update and a draw call per chunk on every frame.

// Hillshade, and the height palette of chunks without a satellite map, are
// computed in the same vertex shader. The formula is the one the CPU used to
// bake into vertex colours:
//   sunlight on:  0.45 + 1.05 * max(0, N·sun)      sunlight off: map brightness
// times the height palette when the chunk has no map. It scales the diffuse
// colour, and MeshLambertMaterial then applies the scene lights.
const terrainShadingUniforms = {
    uSunDir: { value: new THREE.Vector3(0, 1, 0) }, // replaced by the scene's sun vector in initTerrain()
    uSunlightOn: { value: 1 },
    uBrightness: { value: mapBrightness }
};

const TERRAIN_VERTEX_PARS = `
uniform vec3 uSunDir;
uniform float uSunlightOn;
uniform float uBrightness;
uniform highp isampler2DArray uHeights;
uniform int uGridMax;                 // vertices per side - 1
#ifdef USE_INSTANCING
attribute vec4 aChunk;                // x0, z0, dx, dz: world position of grid vertex (0,0), spacing
attribute float aLayer;
#else
uniform vec4 uChunk;
uniform float uLayer;
#endif
varying vec3 vTerrainShade;
// Same palette as getHeightColor() in core/utils.js
vec3 terrainHeightColor(float h) {
    const vec3 G = vec3(0.0431372549, 0.4, 0.137254902);
    const vec3 Y = vec3(0.902, 0.7647058824, 0.3529411765);
    const vec3 O = vec3(0.902, 0.494, 0.133);
    const vec3 R = vec3(0.906, 0.298, 0.235);
    const vec3 P = vec3(0.608, 0.349, 0.713);
    const vec3 B = vec3(0.204, 0.596, 0.858);
    if (h <= -100.0) return vec3(0.0);
    if (h <= 700.0) return G;
    if (h <= 1400.0) return mix(G, Y, (h - 700.0) / 700.0);
    if (h <= 2100.0) return mix(Y, O, (h - 1400.0) / 700.0);
    if (h <= 2800.0) return mix(O, R, (h - 2100.0) / 700.0);
    if (h <= 3500.0) return mix(R, P, (h - 2800.0) / 700.0);
    if (h <= 4000.0) return mix(P, B, (h - 3500.0) / 500.0);
    return B;
}
float terrainHeight(ivec2 g, int layer) {
    return float(texelFetch(uHeights, ivec3(g, layer), 0).r);
}
`;

// Replaces <beginnormal_vertex>. position.xy is the grid vertex (column, row);
// rows run south, columns east. The normal is the central difference the
// terrain worker used to compute (one-sided on the chunk edge): n = S × E.
const TERRAIN_BEGINNORMAL = `
#ifdef USE_INSTANCING
vec4 tChunk = aChunk;
int tLayer = int(aLayer + 0.5);
#else
vec4 tChunk = uChunk;
int tLayer = int(uLayer + 0.5);
#endif
ivec2 tG = ivec2(position.xy + 0.5);
float tH = terrainHeight(tG, tLayer);
ivec2 tE = ivec2(min(tG.x + 1, uGridMax), tG.y);
ivec2 tW = ivec2(max(tG.x - 1, 0), tG.y);
ivec2 tS = ivec2(tG.x, min(tG.y + 1, uGridMax));
ivec2 tN = ivec2(tG.x, max(tG.y - 1, 0));
float tEx = float(tE.x - tW.x) * tChunk.z;
float tEy = terrainHeight(tE, tLayer) - terrainHeight(tW, tLayer);
float tSz = float(tS.y - tN.y) * tChunk.w;
float tSy = terrainHeight(tS, tLayer) - terrainHeight(tN, tLayer);
vec3 objectNormal = normalize(vec3(-tSz * tEy, tSz * tEx, -tSy * tEx));
`;

// Replaces <begin_vertex>
const TERRAIN_BEGIN_VERTEX = `
vec3 transformed = vec3(tChunk.x + float(tG.x) * tChunk.z, tH, tChunk.y + float(tG.y) * tChunk.w);
float terrainLight = uSunlightOn > 0.5
    ? 0.45 + 1.05 * max(0.0, dot(objectNormal, uSunDir))
    : uBrightness;
#ifdef USE_MAP
vTerrainShade = vec3(terrainLight);
#else
vTerrainShade = terrainHeightColor(tH) * terrainLight;
#endif
`;

function applyTerrainShading(shader) {
    // Shading uniforms are shared by every chunk; the chunk uniforms belong to
    // this material (a chunk's own mesh, or an instanced batch's grid).
    Object.assign(shader.uniforms, terrainShadingUniforms, this.userData.terrain);
    shader.vertexShader = shader.vertexShader
        .replace('#include <common>', '#include <common>\n' + TERRAIN_VERTEX_PARS)
        .replace('#include <beginnormal_vertex>', TERRAIN_BEGINNORMAL)
        .replace('#include <begin_vertex>', TERRAIN_BEGIN_VERTEX);
    shader.fragmentShader = shader.fragmentShader
        .replace('#include <common>', '#include <common>\nvarying vec3 vTerrainShade;')
        .replace('#include <color_fragment>', '#include <color_fragment>\n\tdiffuseColor.rgb *= vTerrainShade;');
}

function createTerrainMaterial(params) {
    const material = new THREE.MeshLambertMaterial(Object.assign({
        side: THREE.FrontSide  // heightfield seen from above: backface culling halves rasterization
    }, params));
    material.userData.terrain = {
        uHeights: { value: null },
        uGridMax: { value: 0 },
        uChunk: { value: new THREE.Vector4() },
        uLayer: { value: 0 }
    };
    // A single shared function (it reads its uniforms from `this`): three.js keys
    // its program cache on the callback's source, so every terrain material
    // compiles to the same few programs.
    material.onBeforeCompile = applyTerrainShading;
    return material;
}

// Material slot of chunks drawn through an instanced batch (no map). It is
// never rendered itself; a chunk only gets a material of its own while it
// carries a map. Never disposed.
const untexturedTerrainMaterial = createTerrainMaterial();

/** Point a non-instanced terrain material at a chunk's elevation layer. */
function syncChunkUniforms(material, ud) {
    const t = material.userData.terrain;
    t.uHeights.value = ud.heightStore.texture;
    t.uGridMax.value = ud.geoW - 1;
    t.uChunk.value.set(ud.chunkVec[0], ud.chunkVec[1], ud.chunkVec[2], ud.chunkVec[3]);
    t.uLayer.value = ud.heightLayer;
}

/**
 * Put a satellite map on a chunk, or remove it (texture = null). The chunk's
 * previous map is disposed. With a map the chunk is drawn as a mesh of its own;
 * without one it goes back to its grid's instanced batch.
 */
function setChunkMap(mesh, texture) {
    const own = mesh.material !== untexturedTerrainMaterial ? mesh.material : null;
    if (own && own.map) {
        try { own.map.dispose(); texturesDisposed++; } catch (e) {}
        own.map = null;
    }
    if (texture) {
        const material = own || createTerrainMaterial();
        material.map = texture;
        material.needsUpdate = true;
        if (!own) {
            syncChunkUniforms(material, mesh.userData);
            mesh.material = material;
            if (sceneRef) sceneRef.add(mesh);
        }
    } else if (own) {
        own.dispose();
        mesh.material = untexturedTerrainMaterial;
        if (mesh.parent) mesh.parent.remove(mesh);
    }
}

// ---- Shared grid geometry -------------------------------------------------
const gridGeometries = new Map(); // geoW -> BufferGeometry

/**
 * Grid of geoW x geoW vertices: position = (column, row, 0), same UVs and same
 * triangle order as the THREE.PlaneGeometry the chunks used to be built from,
 * so the winding FrontSide culling depends on is unchanged. Never disposed.
 */
function getGridGeometry(geoW) {
    let geometry = gridGeometries.get(geoW);
    if (geometry) return geometry;

    const seg = geoW - 1;
    const IndexArray = geoW * geoW > 65535 ? Uint32Array : Uint16Array;
    const index = new IndexArray(seg * seg * 6);
    let k = 0;
    for (let r = 0; r < seg; r++) {
        for (let c = 0; c < seg; c++) {
            const a = r * geoW + c;
            const b = (r + 1) * geoW + c;
            const cc = (r + 1) * geoW + c + 1;
            const d = r * geoW + c + 1;
            index[k++] = a; index[k++] = b; index[k++] = d;
            index[k++] = b; index[k++] = cc; index[k++] = d;
        }
    }

    const position = new Float32Array(geoW * geoW * 3);
    const uv = new Float32Array(geoW * geoW * 2);
    for (let r = 0; r < geoW; r++) {
        for (let c = 0; c < geoW; c++) {
            const i = r * geoW + c;
            position[i * 3] = c;
            position[i * 3 + 1] = r;
            uv[i * 2] = c / seg;
            uv[i * 2 + 1] = 1 - r / seg;
        }
    }

    geometry = new THREE.BufferGeometry();
    geometry.setIndex(new THREE.BufferAttribute(index, 1));
    geometry.setAttribute('position', new THREE.BufferAttribute(position, 3));
    geometry.setAttribute('uv', new THREE.BufferAttribute(uv, 2));
    // Grid space, not world space: meshes on it are culled per chunk instead
    geometry.boundingSphere = new THREE.Sphere(new THREE.Vector3(), Infinity);
    gridGeometries.set(geoW, geometry);
    return geometry;
}

// ---- Elevation texture arrays ----------------------------------------------
// One R16I 2D-array texture per grid size, one layer per chunk. three.js r128
// cannot update a single layer of an array texture, so the WebGL texture is
// managed here and handed to three through its texture properties; layers are
// written with texSubImage3D. A CPU copy of each layer (2 bytes per sample,
// ~2 MB in total) lets the array grow by re-uploading.
const heightStores = new Map(); // geoW -> store
const HEIGHT_STORE_INITIAL_LAYERS = 32;

function createHeightTexture(geoW, capacity) {
    const gl = rendererRef.getContext();
    const glTex = gl.createTexture();
    rendererRef.state.bindTexture(gl.TEXTURE_2D_ARRAY, glTex);
    gl.texStorage3D(gl.TEXTURE_2D_ARRAY, 1, gl.R16I, geoW, geoW, capacity);
    gl.texParameteri(gl.TEXTURE_2D_ARRAY, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
    gl.texParameteri(gl.TEXTURE_2D_ARRAY, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
    gl.texParameteri(gl.TEXTURE_2D_ARRAY, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D_ARRAY, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    return glTex;
}

/**
 * Hand three.js our WebGL texture for the store's placeholder: with a matching
 * version three binds it as a sampler2DArray and never tries to upload it.
 */
function adoptHeightTexture(store) {
    const props = rendererRef.properties.get(store.texture);
    props.__webglInit = true;
    props.__webglTexture = store.glTex;
    props.__version = store.texture.version;
}

function getHeightStore(geoW) {
    let store = heightStores.get(geoW);
    if (store) return store;
    const capacity = HEIGHT_STORE_INITIAL_LAYERS;
    store = {
        geoW, capacity,
        glTex: createHeightTexture(geoW, capacity),
        texture: new THREE.DataTexture2DArray(null, geoW, geoW, capacity),
        layers: new Map(), free: [], next: 0
    };
    adoptHeightTexture(store);
    heightStores.set(geoW, store);
    return store;
}

/**
 * After a WebGL context loss and restore, three.js rebuilds its own resources
 * from their sources, but these textures are ours: recreate them from the CPU
 * copies. Runs after three's own handler (registered when the renderer was
 * created), so the renderer state and properties are already the new ones.
 */
function restoreHeightStores() {
    for (const store of heightStores.values()) {
        store.glTex = createHeightTexture(store.geoW, store.capacity);
        for (const [layer, heights] of store.layers) uploadHeightLayer(store, layer, heights);
        adoptHeightTexture(store);
    }
}

function uploadHeightLayer(store, layer, heights) {
    const gl = rendererRef.getContext();
    rendererRef.state.bindTexture(gl.TEXTURE_2D_ARRAY, store.glTex);
    // three sets these per upload of its own; a 3D upload rejects FLIP_Y, and
    // rows of 2-byte samples with odd widths are not 4-byte aligned.
    gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL, false);
    gl.pixelStorei(gl.UNPACK_PREMULTIPLY_ALPHA_WEBGL, false);
    gl.pixelStorei(gl.UNPACK_ALIGNMENT, 2);
    gl.texSubImage3D(gl.TEXTURE_2D_ARRAY, 0, 0, 0, layer, store.geoW, store.geoW, 1,
        gl.RED_INTEGER, gl.SHORT, heights);
}

function allocHeightLayer(store, heights) {
    const layer = store.free.length ? store.free.pop() : store.next++;
    if (layer >= store.capacity) {
        // Grow: a new, larger array filled from the CPU copies
        const old = store.glTex;
        store.capacity *= 2;
        store.glTex = createHeightTexture(store.geoW, store.capacity);
        for (const [l, h] of store.layers) uploadHeightLayer(store, l, h);
        adoptHeightTexture(store);
        rendererRef.getContext().deleteTexture(old);
    }
    store.layers.set(layer, heights);
    uploadHeightLayer(store, layer, heights);
    return layer;
}

function freeHeightLayer(store, layer) {
    if (store && store.layers.delete(layer)) store.free.push(layer);
}

/**
 * Store a chunk's elevation samples and derive what the shader and the
 * culling need: grid origin and spacing in world metres, bounding sphere.
 * Replaces the chunk's previous layer (LOD rebuild).
 */
function setChunkHeights(mesh, item, step, heights, minH, maxH) {
    const ud = mesh.userData;
    const { cx, cy, latBase, lonBase, size, vertsPerChunk } = item;
    const geoW = vertsPerChunk / step + 1;

    if (ud.heightStore) freeHeightLayer(ud.heightStore, ud.heightLayer);
    const store = getHeightStore(geoW);
    ud.heightStore = store;
    ud.heightLayer = allocHeightLayer(store, heights);
    ud.geoW = geoW;
    ud.lodStep = step;

    // latLonToMeters() is linear in lat/lon, so the grid maps to world space as
    // an origin plus a constant spacing per row and per column.
    const startRow = cy * vertsPerChunk;
    const startCol = cx * vertsPerChunk;
    const latTop = latBase + 1 - startRow / (size - 1);
    const lonLeft = lonBase + startCol / (size - 1);
    const o = latLonToMeters(latTop, lonLeft);
    const e = latLonToMeters(latTop, lonLeft + step / (size - 1));
    const s = latLonToMeters(latTop - step / (size - 1), lonLeft);
    const dx = e.x - o.x, dz = s.z - o.z;
    ud.chunkVec = [o.x, o.z, dx, dz];

    const halfW = (geoW - 1) * dx / 2;
    const halfD = (geoW - 1) * dz / 2;
    const halfH = (maxH - minH) / 2;
    ud.sphere = {
        x: o.x + halfW, y: (minH + maxH) / 2, z: o.z + halfD,
        r: Math.sqrt(halfW * halfW + halfD * halfD + halfH * halfH)
    };

    mesh.geometry = getGridGeometry(geoW);
    ud.batch = getInstanceBatch(geoW);
    if (mesh.material !== untexturedTerrainMaterial) syncChunkUniforms(mesh.material, ud);
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

// ---- Instanced batches (chunks without a map) ------------------------------
const instanceBatches = new Map(); // geoW -> batch
const INSTANCE_CAPACITY = 1024;    // > MAX_ACTIVE_CHUNKS
let chunksVisible = true;

function getInstanceBatch(geoW) {
    let batch = instanceBatches.get(geoW);
    if (batch) return batch;

    const grid = getGridGeometry(geoW);
    const chunkAttr = new THREE.InstancedBufferAttribute(new Float32Array(INSTANCE_CAPACITY * 4), 4);
    const layerAttr = new THREE.InstancedBufferAttribute(new Float32Array(INSTANCE_CAPACITY), 1);
    chunkAttr.setUsage(THREE.DynamicDrawUsage);
    layerAttr.setUsage(THREE.DynamicDrawUsage);
    // Its own geometry object (the instance attributes are per batch) on the
    // grid's shared index / position / uv buffers
    const geometry = new THREE.BufferGeometry();
    geometry.setIndex(grid.index);
    geometry.setAttribute('position', grid.attributes.position);
    geometry.setAttribute('uv', grid.attributes.uv);
    geometry.setAttribute('aChunk', chunkAttr);
    geometry.setAttribute('aLayer', layerAttr);
    geometry.boundingSphere = grid.boundingSphere;

    const material = createTerrainMaterial();
    material.userData.terrain.uHeights.value = getHeightStore(geoW).texture;
    material.userData.terrain.uGridMax.value = geoW - 1;

    const mesh = new THREE.InstancedMesh(geometry, material, INSTANCE_CAPACITY);
    // Vertices come out of the shader in world space: identity instance matrices
    const m = mesh.instanceMatrix.array;
    for (let i = 0; i < INSTANCE_CAPACITY; i++) {
        m[i * 16] = m[i * 16 + 5] = m[i * 16 + 10] = m[i * 16 + 15] = 1;
    }
    mesh.frustumCulled = false;   // culled per chunk in updateTerrainInstances()
    mesh.matrixAutoUpdate = false;
    mesh.count = 0;
    mesh.visible = false;
    if (sceneRef) sceneRef.add(mesh);

    batch = { mesh, chunkAttr, layerAttr, count: 0 };
    instanceBatches.set(geoW, batch);
    return batch;
}

const _frustum = new THREE.Frustum();
const _projScreenMatrix = new THREE.Matrix4();
const _planes = new Float64Array(24);   // 6 x (nx, ny, nz, constant)

// Resident chunks as an array for the per-frame pass: iterating activeChunks
// with for...in (an object whose keys come and go) cost several times more.
const chunkList = [];

function listChunk(mesh) {
    mesh.userData.listIndex = chunkList.length;
    chunkList.push(mesh);
}

function unlistChunk(mesh) {
    const i = mesh.userData.listIndex;
    if (i === undefined || chunkList[i] !== mesh) return;
    const last = chunkList.pop();
    if (last !== mesh) {
        chunkList[i] = last;
        last.userData.listIndex = i;
    }
    mesh.userData.listIndex = undefined;
}

/**
 * Per-frame terrain culling, called right before rendering: chunks with a map
 * get their mesh shown or hidden, chunks without one are packed into their
 * grid's instanced batch.
 * @param {THREE.Camera} camera
 */
export function updateTerrainInstances(camera) {
    if (!camera) return;
    camera.updateMatrixWorld();
    _projScreenMatrix.multiplyMatrices(camera.projectionMatrix, camera.matrixWorldInverse);
    _frustum.setFromProjectionMatrix(_projScreenMatrix);
    for (let p = 0; p < 6; p++) {
        const plane = _frustum.planes[p];
        _planes[p * 4] = plane.normal.x;
        _planes[p * 4 + 1] = plane.normal.y;
        _planes[p * 4 + 2] = plane.normal.z;
        _planes[p * 4 + 3] = plane.constant;
    }

    for (const batch of instanceBatches.values()) batch.count = 0;

    for (let n = 0; n < chunkList.length; n++) {
        const mesh = chunkList[n];
        const ud = mesh.userData;
        const s = ud.sphere;
        // Sphere against the six frustum planes (same test as Frustum.intersectsSphere)
        let visible = chunksVisible;
        for (let p = 0; visible && p < 24; p += 4) {
            if (_planes[p] * s.x + _planes[p + 1] * s.y + _planes[p + 2] * s.z + _planes[p + 3] < -s.r) visible = false;
        }

        if (mesh.material !== untexturedTerrainMaterial) {   // own mesh (has a map)
            mesh.visible = visible;
            continue;
        }
        if (!visible) continue;
        const batch = ud.batch;
        if (batch.count >= INSTANCE_CAPACITY) continue;
        const i = batch.count++;
        const c = batch.chunkAttr.array;
        c[i * 4] = ud.chunkVec[0];
        c[i * 4 + 1] = ud.chunkVec[1];
        c[i * 4 + 2] = ud.chunkVec[2];
        c[i * 4 + 3] = ud.chunkVec[3];
        batch.layerAttr.array[i] = ud.heightLayer;
    }

    for (const batch of instanceBatches.values()) {
        batch.mesh.count = batch.count;
        batch.mesh.visible = batch.count > 0;
        if (batch.count === 0) continue;
        batch.chunkAttr.updateRange.offset = 0;
        batch.chunkAttr.updateRange.count = batch.count * 4;
        batch.chunkAttr.needsUpdate = true;
        batch.layerAttr.updateRange.offset = 0;
        batch.layerAttr.updateRange.count = batch.count;
        batch.layerAttr.needsUpdate = true;
    }
}

// ---- Wireframe overlay --------------------------------------------------------
// Triangle grid lines on the chunk under the aircraft while satellite imagery is
// off. One mesh, re-pointed at whichever chunk is nearest.
let wireframeChunkKey = null;
let wireframeMesh = null;

function getWireframeMesh() {
    if (!wireframeMesh) {
        // Black diffuse stays black whatever the lights: same look as the basic
        // wireframe material it replaces, with the terrain vertex shader
        const material = createTerrainMaterial({
            color: 0x000000,
            wireframe: true,
            transparent: true,
            opacity: 0.06,
            depthWrite: false
        });
        wireframeMesh = new THREE.Mesh(getGridGeometry(2), material);
        wireframeMesh.renderOrder = 1;
        wireframeMesh.frustumCulled = false;
        wireframeMesh.matrixAutoUpdate = false;
        wireframeMesh.visible = false;
        if (sceneRef) sceneRef.add(wireframeMesh);
    }
    return wireframeMesh;
}

/**
 * Update wireframe: only the single closest chunk (without satellite texture)
 * gets the wireframe overlay. Called from the render loop.
 */
export function updateWireframeProximity() {
    if (window.satelliteEnabled) {
        if (wireframeMesh) wireframeMesh.visible = false;
        wireframeChunkKey = null;
        return;
    }

    const playerPos = latLonToMeters(STATE.lat, STATE.lon);
    let bestKey = null;
    let bestDist = Infinity;

    for (const key in activeChunks) {
        const ud = activeChunks[key].userData;
        if (ud.textureLoaded || !ud.sphere) continue; // satellite chunk, no wireframe needed
        const dx = ud.sphere.x - playerPos.x;
        const dz = ud.sphere.z - playerPos.z;
        const dist = dx * dx + dz * dz; // no sqrt needed for comparison
        if (dist < bestDist) {
            bestDist = dist;
            bestKey = key;
        }
    }

    wireframeChunkKey = bestKey;
    const wire = getWireframeMesh();
    if (!bestKey) {
        wire.visible = false;
        return;
    }
    // Re-pointed every frame: cheap, and follows LOD rebuilds of that chunk
    const ud = activeChunks[bestKey].userData;
    wire.geometry = getGridGeometry(ud.geoW);
    syncChunkUniforms(wire.material, ud);
    wire.visible = true;
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

    // Clamp the texture cap to the GPU's real limit so we never allocate a
    // canvas larger than the hardware can upload as a texture.
    const gpuMaxTexture = renderer?.capabilities?.maxTextureSize;
    if (gpuMaxTexture > 0) {
        MAX_CANVAS_DIM = Math.min(MAX_CANVAS_DIM, gpuMaxTexture);
    }
    // The scene mutates this vector in place as the sun moves, so the terrain
    // shader follows the sun without any per-chunk work.
    if (sunDirection) terrainShadingUniforms.uSunDir.value = sunDirection;
    renderer.domElement.addEventListener('webglcontextrestored', restoreHeightStores);

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
    if (!USE_TERRAIN_WORKER || typeof Worker === 'undefined') return;

    try {
        terrainWorker = new Worker(new URL('./TerrainWorker.js', import.meta.url), { type: 'module' });
        workerAvailable = true;
        hgtRegisteredInWorker.clear(); // fresh worker holds no tiles

        terrainWorker.onmessage = (e) => {
            const data = e.data || {};
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
                        existing.userData.lodRebuildQueued = false;
                    }
                    return;
                }
                if (!existing && isChunkInRange(item)) {
                    addChunkMesh(item, data.step, data.heights, data.minH, data.maxH);
                    console.debug(`[terrain] Chunk created from worker: ${data.chunkKey} (total=${Object.keys(activeChunks).length})`);
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
                    if (activeChunks[data.chunkKey]) activeChunks[data.chunkKey].userData.lodRebuildQueued = false;
                    return;
                }
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
                    console.warn(`${CONSECUTIVE_ERROR_THRESHOLD} consecutive tile errors — connection lost, satellite from cache only`);
                    setTileNetworkEnabled(false);
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

    // One read per tile at a time: updateTerrainChunks() is re-run every
    // second (and every frame while no chunk exists yet), and each call used
    // to start another 25 MB FileReader for the same file — hundreds in
    // flight during startup, all producing the same chunks.
    if (hgtReadInProgress.has(key)) return;
    hgtReadInProgress.add(key);

    const reader = new FileReader();
    hgtParsing++;
    const done = () => {
        hgtParsing = Math.max(0, hgtParsing - 1);
        hgtReadInProgress.delete(key);
    };
    reader.onload = (e) => {
        done();
        generateChunksFromBuffer(e.target.result, latBase, lonBase);
    };
    reader.onerror = done;
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
    
    const key = `${latBase}_${lonBase}`;

    if (!hgtElevationData[key]) {
        const dataView = new DataView(buffer);
        const elevationArray = new Int16Array(size * size);
        for (let i = 0; i < size * size; i++) {
            elevationArray[i] = dataView.getInt16(i * 2, false);
        }
        hgtElevationData[key] = { data: elevationArray, size: size };
    }

    enqueueChunksForTile(latBase, lonBase, size);

    // Transferring detaches the buffer, so this may only happen once per tile.
    if (workerAvailable && terrainWorker && !hgtRegisteredInWorker.has(key)) {
        try {
            terrainWorker.postMessage({ type: 'registerHgt', key, size, buffer }, [buffer]);
            hgtRegisteredInWorker.add(key);
        } catch (err) {
            workerAvailable = false;
        }
    }
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
                    existing.userData.lodRebuildQueued = false;
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
    console.debug(`[terrain] Chunk created: ${item.chunkKey} (total=${Object.keys(activeChunks).length})`);
    return mesh;
}

/**
 * Register a chunk. It is drawn through its grid's instanced batch until it
 * gets a satellite map (setChunkMap), so it only joins the scene then.
 */
function addChunkMesh(item, step, heights, minH, maxH) {
    const { cx, cy, chunkKey, latBase, lonBase, vertsPerChunk } = item;
    const chunksPerAxis = CHUNKS_PER_TILE_AXIS;

    const mesh = new THREE.Mesh(getGridGeometry(vertsPerChunk / step + 1), untexturedTerrainMaterial);
    mesh.frustumCulled = false;     // culled per chunk in updateTerrainInstances()
    mesh.matrixAutoUpdate = false;  // vertices come out of the shader in world space
    mesh.userData = {
        chunkLatTop: latBase + 1 - (cy / chunksPerAxis),
        chunkLatBottom: latBase + 1 - ((cy + 1) / chunksPerAxis),
        chunkLonLeft: lonBase + (cx / chunksPerAxis),
        chunkLonRight: lonBase + ((cx + 1) / chunksPerAxis),
        vertsPerChunk,
        textureLoaded: false
    };
    setChunkHeights(mesh, item, step, heights, minH, maxH);

    activeChunks[chunkKey] = mesh;
    listChunk(mesh);
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
    mesh.userData.textureZoom = zoomLevel;
    mesh.userData.textureBand = band;

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
                if (job.mesh && job.mesh.userData) job.mesh.userData.textureLoaded = false;
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
        setChunkMap(mesh, texture);
        mesh.userData.textureLoaded = true;
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

    if (mesh.material.map) {
        // A CanvasTexture's backing canvas is released along with it (a
        // compressed texture's staging canvas was released when it was built)
        if (mesh.material.map.image && !mesh.material.map.isCompressedTexture) canvasesReleased++;
        setChunkMap(mesh, null);
    }
    if (mesh.userData) {
        mesh.userData.textureLoaded = false;
        mesh.userData.textureQueued = false;
        mesh.userData.textureZoom = 0;
        mesh.userData.textureBand = undefined;
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
    chunksVisible = !!visible;
}

export function setTerrainSatelliteEnabled(enabled) {
    const on = !!enabled;
    if (!activeChunks) return;

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
            if (mesh && mesh.material) unloadChunkTexture(mesh);
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

    if (wireframeChunkKey === key) wireframeChunkKey = null;

    // unloadChunkTexture() above already put the chunk back on the shared
    // material slot; this only catches a material left without its map
    if (mesh.material && mesh.material !== untexturedTerrainMaterial) {
        if (mesh.material.map) {
            try { mesh.material.map.dispose(); texturesDisposed++; } catch (e) {}
        }
        try { mesh.material.dispose(); } catch (e) {}
    }
    // The grid geometry is shared; the chunk's own data is its elevation layer
    freeHeightLayer(mesh.userData.heightStore, mesh.userData.heightLayer);
    mesh.userData.heightStore = null;
    if (mesh.parent) mesh.parent.remove(mesh);
    delete activeChunks[key];
    unlistChunk(mesh);
    chunksDisposed++;
}

/**
 * Push the sunlight switch to the terrain shader. The sun direction needs no
 * call: the shader reads the scene's sun vector directly (see initTerrain).
 */
export function updateTerrainHillshading() {
    terrainShadingUniforms.uSunlightOn.value = window.sunlightEnabled !== false ? 1 : 0;
}

/**
 * Terrain brightness while the sunlight is off.
 * @param {number} value 0.3 .. 1.6
 */
export function setMapBrightness(value) {
    const v = Number(value);
    if (!Number.isFinite(v)) return;
    mapBrightness = Math.max(0.3, Math.min(1.6, v));
    terrainShadingUniforms.uBrightness.value = mapBrightness;
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
        const ud = activeChunks[key] && activeChunks[key].userData;
        if (ud && ud.textureLoaded) texturedChunks++;
    }
    return {
        hgtAvailable: availableHgtFiles.size,
        hgtLoaded: Object.keys(hgtFiles).length,
        hgtPending: hgtLoadingInProgress.size + _autoDownloadInProgress.size + hgtParsing,
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
