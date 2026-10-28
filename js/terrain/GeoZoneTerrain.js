/**
 * GeoZoneTerrain.js - UAS geographical zones painted on the 3D terrain
 *
 * The same colours as the 2D maps (safety/GeoZoneStyle.js): no-fly red with
 * black bands, conditions violet with bands, a ceiling coloured by the height
 * that may be flown. Zones come from safety/GeoZones.js (setGeoZoneTerrain);
 * the terrain shader (engine/three/TerrainRenderer.js) reads them from two
 * textures over a WINDOW_KM square that follows the aircraft, rasterised here
 * on the CPU when the zones change or the aircraft nears the window's edge.
 *
 * The textures hold numbers, not colours:
 *   coverage (RGBA8): R no-fly zones from the surface, G conditional zones
 *     from the surface, B zones that start above the ground — the fraction
 *     of each texel inside them, from SUB scanlines per texel with the exact
 *     span across each. Filtered and cut at one half, antialiased, the
 *     outlines run along the polygons' and not along the texels' staircase,
 *     and two zones that share an edge meet without a gap.
 *   floors (RG half float): R the lowest floor AGL, G the lowest floor MSL
 *     (NONE: none), spread one texel past every texel a zone touches so the
 *     filtered value is a real floor wherever the coverage says there is one.
 * The shader turns the floors into a height above its own ground, min(R,
 * G - h), so a floor given above sea level (a German control zone's 1569 ft)
 * is right on every slope; one at or under the ground there is a no-fly
 * zone. The bands are drawn in world metres, fading to their average where
 * they get finer than a few pixels.
 *
 * The window is laid out in lat/lon, which latLonToMeters() maps linearly to
 * world x/z, so a new scene origin only changes the uniform, never the raster.
 */

import { latLonToMeters } from '../core/utils.js';
import { GEOZONE_STYLE_GLSL, CEILING_MAX_M } from '../safety/GeoZoneStyle.js';

const SIZE = 1024;                 // texels per side (~20 m)
const SUB = 4;                     // scanlines per texel row
const WINDOW_KM = 20;
const RECENTER_KM = 4;             // re-rasterised once the aircraft is this far from the window's centre
const MIN_RASTER_MS = 500;
const NONE = 60000;                // no floor (half float max is 65504)
const BAND_M = 40;                 // one red and one black band

export const geoZoneTerrainUniforms = {
    uZoneCov: { value: null },
    uZoneFloor: { value: null },
    uZoneMap: { value: new THREE.Vector4() },   // world x, z of the window's north-west corner; 1 / width, 1 / height
    uZoneOn: { value: 0 }
};

/** Terrain fragment code: geoZoneTint(xz, h) → zone colour and its opacity over the ground. */
export const GEOZONE_TERRAIN_GLSL = `
uniform sampler2D uZoneCov;
uniform sampler2D uZoneFloor;
uniform vec4 uZoneMap;
uniform float uZoneOn;
${GEOZONE_STYLE_GLSL}
vec4 geoZoneTint(vec2 xz, float h) {
    if (uZoneOn < 0.5) return vec4(0.0);
    // Textures and derivatives before any branch that differs between fragments
    vec2 uv = (xz - uZoneMap.xy) * uZoneMap.zw;
    vec2 uvc = clamp(uv, 0.0, 1.0);
    vec3 c = texture(uZoneCov, uvc).rgb;
    vec2 f = texture(uZoneFloor, uvc).rg;
    float s = (xz.x + xz.y) * 0.70710678 / ${BAND_M.toFixed(1)};
    float sw = fwidth(s);
    vec3 cw = max(fwidth(c), vec3(0.02));
    if (uv.x < 0.0 || uv.y < 0.0 || uv.x > 1.0 || uv.y > 1.0) return vec4(0.0);
    vec3 cov = smoothstep(0.5 - cw, 0.5 + cw, c);
    float ceil = min(f.r, f.g - h);
    float noFly = max(cov.r, cov.b * step(ceil, 0.5));
    float tri = abs(fract(s) - 0.5);
    float w = max(sw, 1e-4);
    float dark = mix(smoothstep(0.25 - w, 0.25 + w, tri), 0.5, smoothstep(0.12, 0.3, sw));
    vec4 col = vec4(geoZoneCeilingColor(ceil), 0.42 * cov.b * (1.0 - smoothstep(GZ_CEILING_MAX - 30.0, GZ_CEILING_MAX, ceil)));
    col = mix(col, vec4(mix(GZ_CONDITIONAL, vec3(0.02), dark * 0.8), 0.5), cov.g);
    return mix(col, vec4(mix(GZ_PROHIBITED, vec3(0.02), dark), 0.62), noFly);
}
`;

let zones = [];
let zonesVersion = -1, enabled = false;
let covTex = null, floorTex = null;
let cov = null, cov8 = null;       // coverage: accumulated (3 per texel), uploaded (RGBA8)
let floors = null, spread = null;  // floors (2 half floats per texel); one channel half-spread
let win = null;                    // { lat0, lat1, lon0, lon1, lat, lon } of the raster
let rasterVersion = -1, lastRasterAt = -Infinity;
let noneHalf = 0;

/** The zones to paint (GeoZones.js), a version that changes with them, and whether to paint at all. */
export function setGeoZoneTerrain(list, version, on) {
    zones = list || [];
    zonesVersion = version;
    enabled = !!on;
    if (!enabled) geoZoneTerrainUniforms.uZoneOn.value = 0;
}

function kmBetween(aLat, aLon, bLat, bLon) {
    const k = Math.cos(aLat * Math.PI / 180);
    return Math.hypot((aLat - bLat) * 111.32, (aLon - bLon) * 111.32 * k);
}

/**
 * Add the polygon (even-odd over all its rings) to coverage channel covCh,
 * and, for floorCh >= 0, keep the lower of floorV (a half float) and the
 * floor already there in every texel it touches.
 */
function fillPolygon(rings, covCh, floorCh, floorV, toPx) {
    let yMin = Infinity, yMax = -Infinity;
    const px = rings.map((ring) => {
        const out = new Float64Array(ring.length);
        for (let i = 0; i < ring.length; i += 2) {
            const p = toPx(ring[i], ring[i + 1]);
            out[i] = p[0]; out[i + 1] = p[1];
            if (p[1] < yMin) yMin = p[1];
            if (p[1] > yMax) yMax = p[1];
        }
        return out;
    });
    // Scanline k runs through y = (k + 0.5) / SUB, in texel rows
    const k0 = Math.max(0, Math.ceil(yMin * SUB - 0.5)), k1 = Math.min(SIZE * SUB - 1, Math.ceil(yMax * SUB - 0.5) - 1);
    if (k1 < k0) return;
    const lines = new Array(k1 - k0 + 1);
    for (const ring of px) {
        for (let i = 0, j = ring.length - 2; i < ring.length; j = i, i += 2) {
            const x0 = ring[j], y0 = ring[j + 1], x1 = ring[i], y1 = ring[i + 1];
            if (y0 === y1) continue;
            const ya = Math.min(y0, y1) * SUB, yb = Math.max(y0, y1) * SUB;
            const a = Math.max(k0, Math.ceil(ya - 0.5)), b = Math.min(k1, Math.ceil(yb - 0.5) - 1);
            for (let k = a; k <= b; k++) {
                const yc = (k + 0.5) / SUB;
                (lines[k - k0] || (lines[k - k0] = [])).push(x0 + (yc - y0) * (x1 - x0) / (y1 - y0));
            }
        }
    }
    for (let n = 0; n < lines.length; n++) {
        const xs = lines[n];
        if (!xs || xs.length < 2) continue;
        xs.sort((p, q) => p - q);
        const row = Math.floor((k0 + n) / SUB) * SIZE;
        for (let m = 0; m + 1 < xs.length; m += 2) {
            const xa = Math.max(0, xs[m]), xb = Math.min(SIZE, xs[m + 1]);
            if (xb <= xa) continue;
            for (let c = Math.floor(xa), last = Math.min(SIZE - 1, Math.ceil(xb) - 1); c <= last; c++) {
                const part = (Math.min(xb, c + 1) - Math.max(xa, c)) / SUB;
                if (part <= 0) continue;
                cov[(row + c) * 3 + covCh] += part;
                // Positive half floats sort like their bit patterns
                if (floorCh >= 0 && floorV < floors[(row + c) * 2 + floorCh]) floors[(row + c) * 2 + floorCh] = floorV;
            }
        }
    }
}

/**
 * Spread the floors of channel ch one texel (8 neighbours), lower first: a
 * 3-wide minimum along the rows into spread, then down the columns back.
 */
function spreadFloors(ch) {
    for (let r = 0; r < SIZE; r++) {
        const base = r * SIZE;
        for (let c = 0; c < SIZE; c++) {
            let v = floors[(base + c) * 2 + ch];
            if (c > 0) { const u = floors[(base + c - 1) * 2 + ch]; if (u < v) v = u; }
            if (c < SIZE - 1) { const u = floors[(base + c + 1) * 2 + ch]; if (u < v) v = u; }
            spread[base + c] = v;
        }
    }
    for (let r = 0; r < SIZE; r++) {
        const base = r * SIZE;
        for (let c = 0; c < SIZE; c++) {
            let v = spread[base + c];
            if (r > 0) { const u = spread[base - SIZE + c]; if (u < v) v = u; }
            if (r < SIZE - 1) { const u = spread[base + SIZE + c]; if (u < v) v = u; }
            floors[(base + c) * 2 + ch] = v;
        }
    }
}

function dataTexture(array, format, type) {
    const t = new THREE.DataTexture(array, SIZE, SIZE, format, type);
    t.minFilter = THREE.LinearFilter;
    t.magFilter = THREE.LinearFilter;
    t.generateMipmaps = false;
    t.wrapS = t.wrapT = THREE.ClampToEdgeWrapping;
    return t;
}

function raster(lat, lon) {
    const dLat = WINDOW_KM / 2 / 111.32;
    const dLon = dLat / Math.max(0.05, Math.cos(lat * Math.PI / 180));
    win = { lat0: lat - dLat, lat1: lat + dLat, lon0: lon - dLon, lon1: lon + dLon, lat, lon };
    if (!cov) {
        cov = new Float32Array(SIZE * SIZE * 3);
        cov8 = new Uint8Array(SIZE * SIZE * 4);
        floors = new Uint16Array(SIZE * SIZE * 2);
        spread = new Uint16Array(SIZE * SIZE);
        noneHalf = THREE.DataUtils.toHalfFloat(NONE);
    }
    cov.fill(0);
    floors.fill(noneHalf);
    const sx = SIZE / (win.lon1 - win.lon0), sy = SIZE / (win.lat1 - win.lat0);
    const toPx = (x, y) => [(x - win.lon0) * sx, (win.lat1 - y) * sy];
    const box = [win.lon0, win.lat0, win.lon1, win.lat1];
    let drawn = 0;
    const floored = [false, false];
    for (const z of zones) {
        const b = z.bbox;
        if (b[0] > box[2] || b[2] < box[0] || b[1] > box[3] || b[3] < box[1]) continue;
        if (z.floorRef === 'AGL' && z.floor >= CEILING_MAX_M) continue;
        const surface = z.floorRef === 'AGL' && z.floor <= 0.5;
        const covCh = surface ? (z.kind === 'prohibited' ? 0 : 1) : 2;
        const floorCh = surface ? -1 : (z.floorRef === 'AGL' ? 0 : 1);
        const floorV = surface ? 0 : THREE.DataUtils.toHalfFloat(Math.max(0, Math.min(NONE - 1, z.floor)));
        for (const rings of z.polys) fillPolygon(rings, covCh, floorCh, floorV, toPx);
        drawn++;
        if (!surface) floored[floorCh] = true;
    }
    for (let i = 0, j = 0; i < cov.length; i += 3, j += 4) {
        cov8[j] = Math.min(255, Math.round(cov[i] * 255));
        cov8[j + 1] = Math.min(255, Math.round(cov[i + 1] * 255));
        cov8[j + 2] = Math.min(255, Math.round(cov[i + 2] * 255));
    }
    for (let ch = 0; ch < 2; ch++) if (floored[ch]) spreadFloors(ch);
    if (!covTex) {
        covTex = dataTexture(cov8, THREE.RGBAFormat, THREE.UnsignedByteType);
        floorTex = dataTexture(floors, THREE.RGFormat, THREE.HalfFloatType);
        geoZoneTerrainUniforms.uZoneCov.value = covTex;
        geoZoneTerrainUniforms.uZoneFloor.value = floorTex;
    }
    covTex.needsUpdate = true;
    floorTex.needsUpdate = true;
    return drawn;
}

/** Every frame, from the terrain renderer: the aircraft's position. */
export function updateGeoZoneTerrain(lat, lon) {
    const u = geoZoneTerrainUniforms;
    if (!enabled || !zones.length || !isFinite(lat) || !isFinite(lon)) { u.uZoneOn.value = 0; return; }
    const now = performance.now();
    const due = !win || rasterVersion !== zonesVersion || kmBetween(lat, lon, win.lat, win.lon) > RECENTER_KM;
    if (due && now - lastRasterAt >= MIN_RASTER_MS) {
        lastRasterAt = now;
        rasterVersion = zonesVersion;
        const t0 = performance.now();
        const drawn = raster(lat, lon);
        console.log(`[geozones] terrain: ${drawn} zones in the ${WINDOW_KM} km window, ${(performance.now() - t0).toFixed(0)} ms`);
    }
    if (!win) { u.uZoneOn.value = 0; return; }
    const nw = latLonToMeters(win.lat1, win.lon0), se = latLonToMeters(win.lat0, win.lon1);
    u.uZoneMap.value.set(nw.x, nw.z, 1 / (se.x - nw.x), 1 / (se.z - nw.z));
    u.uZoneOn.value = 1;
}

export function disposeGeoZoneTerrain() {
    if (covTex) covTex.dispose();
    if (floorTex) floorTex.dispose();
    covTex = floorTex = null;
    cov = cov8 = floors = spread = null;
    win = null;
    rasterVersion = -1;
    geoZoneTerrainUniforms.uZoneCov.value = null;
    geoZoneTerrainUniforms.uZoneFloor.value = null;
    geoZoneTerrainUniforms.uZoneOn.value = 0;
}
