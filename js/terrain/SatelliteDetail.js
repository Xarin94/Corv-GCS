/**
 * SatelliteDetail.js - Satellite imagery at full resolution around the aircraft
 *
 * The chunk textures (TerrainManager) end at zoom 16, 1.6 m per pixel, next to
 * the aircraft: a chunk is 3.7 × 2.5 km, and at the zoom the imagery really
 * has (20, 0.1 m) it would need a 37000 × 50000 texture. Flying 50 m above
 * the ground, a screen pixel covers 6 cm there, and the ground was a blur
 * that the 2D map, at zoom 20, showed sharp.
 *
 * Here four square textures follow the aircraft, one per zoom (17, 18, 19,
 * 20), each 8 × 8 tiles of 256 px: 1.7 km, 830 m, 420 m and 210 m across at
 * mid latitudes. The terrain shader draws them over the chunk textures, a
 * finer one over a coarser one, each fading out over its outer half tile — a
 * clipmap. A level
 * is used only while the camera is close enough for it to add detail: a
 * screen pixel there must cover less than two of its texels. At 1000 m above
 * the ground only zoom 17 and 18 are fetched. SYS CONFIG → 3D SATELLITE DETAIL
 * sets the finest level used: 18 by default (0.4 m), up to 20 (0.1 m), or 16
 * for the chunk textures alone.
 *
 * Each texture is toroidal: tile (x, y) lives in slot (x mod 8, y mod 8), so
 * when the window moves only the tiles entering it are loaded, into the slots
 * of the ones that left; the shader samples with repeat wrapping. A slot is
 * cleared to transparent the moment it changes tile, so the chunk texture
 * shows until the new tile arrives, and a tile that never comes (offline,
 * no imagery) leaves it showing for good. Tiles come through TerrainManager's
 * loader: memory cache, IndexedDB, then the network, like the chunk tiles.
 *
 * World x/z map to Mercator pixels through a linear map around each window's
 * centre; across a window Mercator's curvature moves a point by less than a
 * fifth of a texel. The textures are RGBA, not BC1: 21 MB of VRAM per level
 * with its mipmaps, allocated when the level is first used.
 */

import { latLonToMeters } from '../core/utils.js';

const LEVEL_ZOOMS = [17, 18, 19, 20];   // coarse to fine; Google's imagery goes no further than 20
const TILE = 256;
const TILES = 8;                        // tiles across a window
const SIZE = TILE * TILES;              // texture size, px
const RECENTER = 0.75;                  // window moves once the aircraft is this many tiles off its centre
const DEACTIVATE = 1.25;                // hysteresis on the distance a level is used to
const MAX_COPIES_PER_FRAME = 8;         // tile uploads per frame (each is a 256 × 256 texSubImage2D)
const M_PER_PX_Z0 = 156543.03392;       // Web Mercator metres per pixel at zoom 0 on the equator
const RAD = Math.PI / 180;

/** Uniforms of the terrain shader (merged into its own by TerrainManager). */
export const satelliteDetailUniforms = {
    uDetail0: { value: null },
    uDetail1: { value: null },
    uDetail2: { value: null },
    uDetail3: { value: null },
    // Per level: window centre in world x, z; texture pixels per metre in x, z
    uDetailMap: { value: LEVEL_ZOOMS.map(() => new THREE.Vector4()) },
    // Per level: the centre's pixel in the window; the window's origin in the texture
    uDetailWin: { value: LEVEL_ZOOMS.map(() => new THREE.Vector4()) },
    uDetailOn: { value: new THREE.Vector4() }     // 1 for a level in use
};

/**
 * Terrain fragment code: satelliteDetail(base, xz) returns the colour at world
 * x/z with the levels drawn over base. Texels are premultiplied (a cleared
 * slot is 0,0,0,0), which keeps the mipmaps right next to a missing tile.
 *
 * The levels are composited finest first, each under the ones before it, and
 * a fragment stops as soon as it is covered: in the zoom 20 window one texture
 * read, not four, and outside every window none. Reads in a branch that
 * differs between neighbouring fragments cannot take their mip level from
 * implicit derivatives, so textureGrad() gets the world x/z derivatives,
 * taken once before any branch, scaled to each level.
 */
export const SATELLITE_DETAIL_GLSL = `
uniform sampler2D uDetail0;
uniform sampler2D uDetail1;
uniform sampler2D uDetail2;
uniform sampler2D uDetail3;
uniform vec4 uDetailMap[4];
uniform vec4 uDetailWin[4];
uniform vec4 uDetailOn;

// acc: premultiplied colour and coverage of the finer levels
vec4 detailUnder(vec4 acc, sampler2D tex, vec4 map, vec4 win, vec2 xz, vec2 dx, vec2 dy) {
    vec2 p = (xz - map.xy) * map.zw + win.xy;           // pixel in the window, 0..${SIZE}
    vec2 edge = min(p, ${SIZE.toFixed(1)} - p);
    float w = smoothstep(0.0, ${(TILE / 2).toFixed(1)}, min(edge.x, edge.y));
    if (w <= 0.0) return acc;
    vec2 k = map.zw / ${SIZE.toFixed(1)};
    vec4 t = textureGrad(tex, (p + win.zw) / ${SIZE.toFixed(1)}, dx * k, dy * k) * w;
    return acc + t * (1.0 - acc.a);
}

vec3 satelliteDetail(vec3 base, vec2 xz) {
    vec2 dx = dFdx(xz), dy = dFdy(xz);
    vec4 acc = vec4(0.0);
    if (uDetailOn.w > 0.5) acc = detailUnder(acc, uDetail3, uDetailMap[3], uDetailWin[3], xz, dx, dy);
    if (uDetailOn.z > 0.5 && acc.a < 0.999) acc = detailUnder(acc, uDetail2, uDetailMap[2], uDetailWin[2], xz, dx, dy);
    if (uDetailOn.y > 0.5 && acc.a < 0.999) acc = detailUnder(acc, uDetail1, uDetailMap[1], uDetailWin[1], xz, dx, dy);
    if (uDetailOn.x > 0.5 && acc.a < 0.999) acc = detailUnder(acc, uDetail0, uDetailMap[0], uDetailWin[0], xz, dx, dy);
    return base * (1.0 - acc.a) + acc.rgb;
}
`;

let renderer = null;
let loadTile = null;      // (x, y, z, callback(ImageBitmap|null)) from TerrainManager
let contextLost = false;
let onContextLost = null, onContextRestored = null;
// Highest zoom drawn (SYS CONFIG → 3D SATELLITE DETAIL); 16 leaves the chunk textures alone
export const SATELLITE_DETAIL_DEFAULT_ZOOM = 18;
let maxZoom = SATELLITE_DETAIL_DEFAULT_ZOOM;

const levels = LEVEL_ZOOMS.map((zoom, i) => ({
    zoom, index: i,
    target: null,         // WebGLRenderTarget holding the tiles, created on first use
    active: false,
    tx0: null, ty0: null, // window origin, in tiles
    slots: new Array(TILES * TILES).fill(null)   // { tx, ty, state: 'loading' | 'ready' | 'missing' }
}));

const copyQueue = [];     // { level, slot, src (ImageBitmap | null to clear) }
const tileTexture = new THREE.Texture();           // carries one ImageBitmap to copyTextureToTexture
const clearTexture = new THREE.DataTexture(new Uint8Array(TILE * TILE * 4), TILE, TILE);
const _pos = new THREE.Vector2();
const _clearColor = new THREE.Color();

/**
 * @param {THREE.WebGLRenderer} r
 * @param {Function} loader TerrainManager's tile loader: (x, y, z, callback)
 */
export function initSatelliteDetail(r, loader) {
    disposeSatelliteDetail();
    renderer = r;
    loadTile = loader;
    const canvas = renderer.domElement;
    onContextLost = () => { contextLost = true; };
    // three re-creates its render targets empty: clear them and load every tile again
    onContextRestored = () => {
        contextLost = false;
        copyQueue.length = 0;
        for (const level of levels) {
            if (level.target) clearTarget(level);
            level.tx0 = level.ty0 = null;
            level.slots.fill(null);
        }
    };
    canvas.addEventListener('webglcontextlost', onContextLost);
    canvas.addEventListener('webglcontextrestored', onContextRestored);
    contextLost = false;
}

/** Release the active view's clipmaps and listeners when its backend closes. */
export function disposeSatelliteDetail() {
    if (renderer) {
        renderer.domElement.removeEventListener('webglcontextlost', onContextLost);
        renderer.domElement.removeEventListener('webglcontextrestored', onContextRestored);
    }
    for (const level of levels) releaseLevel(level);
    copyQueue.length = 0;
    renderer = loadTile = null;
    onContextLost = onContextRestored = null;
    contextLost = true;
}

/**
 * The sharpest zoom drawn: each step doubles the detail next to the aircraft,
 * and the tiles to download flying low. Levels above it stop loading and give
 * their VRAM back.
 * @param {number} zoom 16 (chunk textures only) to 20
 */
export function setSatelliteDetailMaxZoom(zoom) {
    const z = Math.round(Number(zoom));
    if (!Number.isFinite(z)) return;
    maxZoom = Math.max(16, Math.min(LEVEL_ZOOMS[LEVEL_ZOOMS.length - 1], z));
    for (const level of levels) if (level.zoom > maxZoom) releaseLevel(level);
}

function releaseLevel(level) {
    level.active = false;
    level.tx0 = level.ty0 = null;
    level.slots.fill(null);           // queued copies and late tiles no longer match a slot
    satelliteDetailUniforms.uDetailOn.value.setComponent(level.index, 0);
    if (level.target) {
        satelliteDetailUniforms[`uDetail${level.index}`].value = null;
        level.target.dispose();
        level.target = null;
    }
}

function createTarget() {
    const target = new THREE.WebGLRenderTarget(SIZE, SIZE, {
        depthBuffer: false,
        stencilBuffer: false,
        generateMipmaps: true,
        minFilter: THREE.LinearMipmapLinearFilter,
        magFilter: THREE.LinearFilter
    });
    target.texture.wrapS = THREE.RepeatWrapping;
    target.texture.wrapT = THREE.RepeatWrapping;
    target.texture.flipY = false;   // tiles are uploaded with the target's unpack flip: rows north to south
    target.texture.anisotropy = renderer.capabilities.getMaxAnisotropy();
    return target;
}

/** Whole texture to transparent, mipmaps included. */
function clearTarget(level) {
    const previous = renderer.getRenderTarget();
    renderer.getClearColor(_clearColor);
    const alpha = renderer.getClearAlpha();
    renderer.setRenderTarget(level.target);
    renderer.setClearColor(0x000000, 0);
    renderer.clear(true, false, false);
    renderer.setRenderTarget(previous);
    renderer.setClearColor(_clearColor, alpha);
    // A clear writes level 0 only: one copy rebuilds the mipmaps from it
    copy(level, 0, null, true);
}

/** Upload one tile (or transparency) into a slot. */
function copy(level, slot, bitmap, mipmaps) {
    const texture = level.target.texture;
    texture.generateMipmaps = mipmaps;
    _pos.set((slot % TILES) * TILE, Math.floor(slot / TILES) * TILE);
    if (bitmap) {
        tileTexture.image = bitmap;
        renderer.copyTextureToTexture(tileTexture, texture, null, _pos);
        tileTexture.image = null;
    } else {
        renderer.copyTextureToTexture(clearTexture, texture, null, _pos);
    }
    texture.generateMipmaps = true;
}

const mod = (a, n) => ((a % n) + n) % n;

/**
 * Centre the level's window on the aircraft when it has drifted off it, and
 * give the slots that change tile their new one.
 */
function placeWindow(level, lat, lon) {
    const n = 2 ** level.zoom;
    const fx = (lon + 180) / 360 * n;
    const fy = (1 - Math.asinh(Math.tan(lat * RAD)) / Math.PI) / 2 * n;
    const half = TILES / 2;
    if (level.tx0 !== null && Math.abs(fx - (level.tx0 + half)) < RECENTER && Math.abs(fy - (level.ty0 + half)) < RECENTER) return;

    const tx0 = Math.round(fx) - half, ty0 = Math.round(fy) - half;
    const jump = level.tx0 === null || Math.abs(tx0 - level.tx0) >= TILES || Math.abs(ty0 - level.ty0) >= TILES;
    level.tx0 = tx0;
    level.ty0 = ty0;
    if (jump) {
        // Nothing to keep: one clear instead of 64 slot clears (queued copies
        // of the old tiles no longer match their slot and are dropped)
        level.slots.fill(null);
        clearTarget(level);
    }

    const incoming = [];
    for (let ty = ty0; ty < ty0 + TILES; ty++) {
        for (let tx = tx0; tx < tx0 + TILES; tx++) {
            const slot = mod(ty, TILES) * TILES + mod(tx, TILES);
            const held = level.slots[slot];
            if (held && held.tx === tx && held.ty === ty) continue;
            level.slots[slot] = { tx, ty, state: 'loading' };
            incoming.push({ slot, tx, ty, d: Math.hypot(tx + 0.5 - fx, ty + 0.5 - fy) });
        }
    }
    if (!incoming.length) return;

    // The old tiles must not show at the new place, even for a frame
    if (!jump) incoming.forEach((t, i) => copy(level, t.slot, null, i === incoming.length - 1));
    // Nearest first
    incoming.sort((a, b) => a.d - b.d);
    for (const t of incoming) {
        loadTile(t.tx, t.ty, level.zoom, (bitmap) => {
            const held = level.slots[t.slot];
            if (!held || held.tx !== t.tx || held.ty !== t.ty) return;   // the window moved on
            if (!bitmap) { held.state = 'missing'; return; }
            copyQueue.push({ level, slot: t.slot, tx: t.tx, ty: t.ty, bitmap });
        });
    }
}

/** Uniforms of one level: the map from world x/z to its window. */
function writeUniforms(level) {
    const n = 2 ** level.zoom;
    const cx = level.tx0 + TILES / 2, cy = level.ty0 + TILES / 2;   // window centre, a tile corner
    const lon = cx / n * 360 - 180;
    const lat = Math.atan(Math.sinh(Math.PI * (1 - 2 * cy / n))) / RAD;
    const c = latLonToMeters(lat, lon);
    // World metres per degree, as latLonToMeters() has them
    const mPerLon = latLonToMeters(lat, lon + 1).x - c.x;
    const mPerLat = c.z - latLonToMeters(lat + 1, lon).z;
    const pxPerDeg = TILE * n / 360;
    satelliteDetailUniforms.uDetailMap.value[level.index].set(
        c.x, c.z, pxPerDeg / mPerLon, pxPerDeg / Math.cos(lat * RAD) / mPerLat);
    satelliteDetailUniforms.uDetailWin.value[level.index].set(
        SIZE / 2, SIZE / 2, mod(level.tx0, TILES) * TILE, mod(level.ty0, TILES) * TILE);
}

/** Upload the tiles that arrived, a few per frame; one mipmap rebuild per level. */
function flushCopies() {
    if (!copyQueue.length) return;
    const batch = [];
    for (const c of copyQueue.splice(0, MAX_COPIES_PER_FRAME)) {
        const held = c.level.slots[c.slot];
        if (!held || held.tx !== c.tx || held.ty !== c.ty) continue;   // the window moved on
        if (!c.bitmap.width) {
            // Evicted (closed) from the image cache while queued: ask again
            loadTile(c.tx, c.ty, c.level.zoom, (bitmap) => {
                if (c.level.slots[c.slot] !== held) return;
                if (bitmap && bitmap.width) copyQueue.push({ ...c, bitmap });
                else held.state = 'missing';
            });
            continue;
        }
        batch.push(c);
    }
    const last = new Map();
    batch.forEach((c, i) => last.set(c.level, i));
    batch.forEach((c, i) => {
        copy(c.level, c.slot, c.bitmap, last.get(c.level) === i);
        c.level.slots[c.slot].state = 'ready';
    });
}

/**
 * Once per rendered frame, before the render.
 * @param {object} p
 * @param {THREE.Camera} p.camera
 * @param {number} p.lat aircraft position
 * @param {number} p.lon
 * @param {number|null} p.groundY terrain height under the aircraft (world y), null if unknown
 * @param {number} p.pixelAngle screen pixel size in radians (vertical FOV / viewport height)
 * @param {boolean} p.enabled satellite imagery on and the terrain drawn with it
 */
export function updateSatelliteDetail({ camera, lat, lon, groundY, pixelAngle, enabled }) {
    const on = satelliteDetailUniforms.uDetailOn.value;
    if (!renderer || contextLost || !enabled || !Number.isFinite(lat) || !Number.isFinite(lon)) {
        on.set(0, 0, 0, 0);
        return;
    }
    const air = latLonToMeters(lat, lon);
    const p = camera.position;
    const dy = Number.isFinite(groundY) ? p.y - groundY : 0;
    const camDist = Math.hypot(p.x - air.x, dy, p.z - air.z);

    for (const level of levels) {
        if (level.zoom > maxZoom) {
            on.setComponent(level.index, 0);
            continue;
        }
        const res = M_PER_PX_Z0 * Math.cos(lat * RAD) / 2 ** level.zoom;    // metres per texel
        // Farthest a pixel can be and still cover less than two texels, and
        // how far from the camera the window's nearest edge is (at most)
        const useDist = 2 * res / pixelAngle;
        const reach = camDist - res * SIZE / 2;
        if (!level.active && reach < useDist) level.active = true;
        else if (level.active && reach > useDist * DEACTIVATE) level.active = false;
        if (level.active) {
            if (!level.target) {
                level.target = createTarget();
                clearTarget(level);
                satelliteDetailUniforms[`uDetail${level.index}`].value = level.target.texture;
            }
            placeWindow(level, lat, lon);
            writeUniforms(level);
        }
        on.setComponent(level.index, level.active ? 1 : 0);
    }
    flushCopies();
}

/** Levels in use and their tiles, for getMemoryStats(). */
export function getSatelliteDetailStats() {
    return levels.map(l => {
        const count = (state) => l.slots.filter(s => s && s.state === state).length;
        return { zoom: l.zoom, active: l.active, ready: count('ready'), loading: count('loading'), missing: count('missing'), allocated: !!l.target };
    });
}
