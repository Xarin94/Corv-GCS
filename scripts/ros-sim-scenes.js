/**
 * ros-sim-scenes.js - Synthetic surfaces for the rosbridge emulator
 *
 * Height in metres, up, as a function of east / north metres from the scene
 * anchor (the vehicle's start position). Shared by scripts/rosbridge-sim.js,
 * which ray-casts its sensors against them, and by tests that compare the
 * GCS's averaged mesh with the truth.
 *
 *   terrain  land under an aircraft: a 14 m hill to the north-east, an 8 m
 *            hollow to the south-west, a 3 m undulation and a 4 % slope, 0 at
 *            the start position; and details only a fine mesh keeps: rocks
 *            (0.8 m, 1.2 m across), a 6 × 4 m slab 0.6 m high, a 1 m ditch
 *   seabed   lake bed under a boat or a ROV, the water surface at 0: 14 m
 *            deep at the start, a trench to the north-west, a mound to the
 *            south-east, shoaling to the east, never shallower than 5 m; a
 *            wreck 12 × 3.5 m standing 1.2 m proud and a field of rocks
 *   cave     an underwater cave in steps, under 2 m of lake: a shaft 3 m
 *            across from the lake bed down to 10 m, a level passage 3 m long
 *            (wall to wall) to the east, a second shaft to 20 m, another
 *            passage, a third shaft to 30 m with a closed bottom (CAVE_PATH:
 *            the centre line). Not a height field: a signed distance
 *            (negative in the water), ray-cast by sphere tracing; the water
 *            surface gives no echo.
 *   garda    5 km² of Lake Garda's southern basin, between the Sirmione
 *            peninsula and Lazise (GARDA: a 2.5 × 2 km rectangle, its
 *            south-west corner the scene origin, fixed to the map rather than
 *            to the vehicle). An approximation of the bed, not survey data:
 *            10 m deep in the south-west, 65–80 m in the north-east basin, a
 *            moraine ridge 14 m high running NW–SE with a boulder field on
 *            it, a meandering channel 8 m deep, sand ripples (0.35 m, 22 m)
 *            in the shallows, pockmarks, a rock shoal coming up to 6 m and a
 *            wreck 28 × 7 m standing 3.5 m proud. Beyond 90 m of range an
 *            imaging sonar sees none of the deepest part from the surface.
 *
 * The details have soft edges (slopes ≤ 1.5) so castRay's step stays safe.
 */

function gauss(e, n, ce, cn, s) {
    return Math.exp(-((e - ce) ** 2 + (n - cn) ** 2) / (2 * s * s));
}

// 1 inside, 0 outside, a ramp `soft` metres wide across the edge
function plateau(x, half, soft) {
    const d = Math.abs(x) - half;
    return d <= 0 ? 1 : d >= soft ? 0 : 0.5 + 0.5 * Math.cos(Math.PI * d / soft);
}

// Rotated rectangle (centre, half sizes, heading from north, soft edge)
function slab(e, n, ce, cn, halfAlong, halfAcross, headingDeg, soft) {
    const a = headingDeg * Math.PI / 180, de = e - ce, dn = n - cn;
    const along = de * Math.sin(a) + dn * Math.cos(a), across = de * Math.cos(a) - dn * Math.sin(a);
    return plateau(along, halfAlong, soft) * plateau(across, halfAcross, soft);
}

// Rocks on a regular jittered pattern, fixed by a seed
function rocks(e, n, list) {
    let h = 0;
    for (const [ce, cn, height, sigma] of list) {
        const d2 = (e - ce) ** 2 + (n - cn) ** 2;
        if (d2 < 16 * sigma * sigma) h += height * Math.exp(-d2 / (2 * sigma * sigma));
    }
    return h;
}

function rockField(seed, count, e0, n0, size, height, sigma) {
    let s = seed;
    const rnd = () => { s = (s * 1103515245 + 12345) % 2147483648; return s / 2147483648; };
    const out = [];
    for (let k = 0; k < count; k++) out.push([e0 + rnd() * size, n0 + rnd() * size, height * (0.6 + 0.4 * rnd()), sigma * (0.8 + 0.4 * rnd())]);
    return out;
}

const LAND_ROCKS = rockField(7, 40, -40, -40, 80, 0.8, 0.6);
const BED_ROCKS = rockField(11, 30, -30, -30, 60, 0.7, 0.5);

function terrainRaw(e, n) {
    return 14 * gauss(e, n, 70, 50, 40) - 8 * gauss(e, n, -60, -40, 30)
        + 3 * Math.sin(e / 35) * Math.cos(n / 45) + 0.04 * n
        + rocks(e, n, LAND_ROCKS)
        + 0.6 * slab(e, n, 25, -15, 3, 2, 20, 0.5)
        - 1.0 * slab(e, n, -20, 20, 30, 0.6, 90, 0.8);
}
const TERRAIN_ZERO = terrainRaw(0, 0);

function seabedDepth(e, n) {
    const d = 14 + 0.06 * n - 0.04 * e + 7 * gauss(e, n, -40, 60, 30) - 6 * gauss(e, n, 50, -30, 20)
        + 1.5 * Math.sin(e / 25) * Math.sin(n / 30);
    return Math.max(5, d) - 1.2 * slab(e, n, 15, 10, 6, 1.75, 35, 0.8) - rocks(e, n, BED_ROCKS);
}

// The cave's centre line, ENU metres from the start (up negative: depth):
// shafts 3 m across, passages 3 m between the shaft walls (6 m axis to axis)
const CAVE_RADIUS = 1.5;
const CAVE_PATH = [[0, 0, -2], [0, 0, -10], [6, 0, -10], [6, 0, -20], [12, 0, -20], [12, 0, -28.5]];
const LAKE_DEPTH = 2;

// Distance to the centre line
function tube(e, n, u) {
    let best = Infinity;
    for (let k = 0; k < CAVE_PATH.length - 1; k++) {
        const a = CAVE_PATH[k], b = CAVE_PATH[k + 1];
        const de = b[0] - a[0], dn = b[1] - a[1], du = b[2] - a[2];
        const len2 = de * de + dn * dn + du * du;
        const t = Math.max(0, Math.min(1, ((e - a[0]) * de + (n - a[1]) * dn + (u - a[2]) * du) / len2));
        best = Math.min(best, Math.hypot(e - a[0] - t * de, n - a[1] - t * dn, u - a[2] - t * du));
    }
    return best;
}

// Signed distance from the rock (negative in the water): the lake above the
// bed at −2 m, the shafts and passages below it, with 5 cm of roughness
function caveSdf(e, n, u) {
    const rough = 0.05 * Math.sin(3.1 * e) * Math.sin(2.7 * n) * Math.sin(2.3 * u);
    const lake = -LAKE_DEPTH - u;                    // < 0 above the lake bed
    return Math.min(tube(e, n, u) - CAVE_RADIUS + rough, lake);
}

// ── Lake Garda, southern basin ──────────────────────────────────────────────
const GARDA = {
    origin: { lat: 45.4960, lon: 10.6490 },          // south-west corner of the survey area
    size: [2500, 2000],                              // m east × north: 5 km²
};
{
    const mLat = 111320, mLon = 111320 * Math.cos(GARDA.origin.lat * Math.PI / 180);
    const [w, h] = GARDA.size;
    GARDA.area = [[0, 0], [w, 0], [w, h], [0, h]].map(([e, n]) => ({ lat: GARDA.origin.lat + n / mLat, lng: GARDA.origin.lon + e / mLon }));
}

// A reproducible value in [0, 1) per grid cell
function hash01(i, j, seed) {
    let h = (Math.imul(i, 374761393) + Math.imul(j, 668265263) + Math.imul(seed, 1442695041)) | 0;
    h = Math.imul(h ^ (h >>> 13), 1274126177);
    h ^= h >>> 16;
    return (h >>> 0) / 4294967296;
}

// Distance from (e, n) to the segment a–b
function segDist(e, n, a, b) {
    const de = b[0] - a[0], dn = b[1] - a[1];
    const t = Math.max(0, Math.min(1, ((e - a[0]) * de + (n - a[1]) * dn) / (de * de + dn * dn)));
    return Math.hypot(e - a[0] - t * de, n - a[1] - t * dn);
}

const MORAINE = [[900, 2050], [1750, 50]];
const WRECK = { e: 1250, n: 1150, half: [14, 3.5], heading: 65, height: 3.5 };

// Bumps hashed on a grid: within ±1 cell of (e, n), each cell may hold one
function hashedBumps(e, n, cell, seed, chance, hMin, hMax, sMin, sMax, sign) {
    const ci = Math.floor(e / cell), cj = Math.floor(n / cell);
    let h = 0;
    for (let dj = -1; dj <= 1; dj++) for (let di = -1; di <= 1; di++) {
        const i = ci + di, j = cj + dj;
        if (hash01(i, j, seed) > chance) continue;
        const be = (i + hash01(i, j, seed + 1)) * cell, bn = (j + hash01(i, j, seed + 2)) * cell;
        const sg = sMin + (sMax - sMin) * hash01(i, j, seed + 3);
        const d2 = (e - be) ** 2 + (n - bn) ** 2;
        if (d2 > 16 * sg * sg) continue;
        h += sign * (hMin + (hMax - hMin) * hash01(i, j, seed + 4)) * Math.exp(-d2 / (2 * sg * sg));
    }
    return h;
}

function gardaDepth(e, n) {
    const [w, h] = GARDA.size;
    // South-west shallows to the north-east basin
    const t = Math.max(0, Math.min(1, 0.7 * e / w + 0.3 * n / h));
    let d = 10 + 55 * t * t * (3 - 2 * t) + 13 * gauss(e, n, 1900, 1500, 600);
    // Moraine ridge, with boulders on and around it
    const dm = segDist(e, n, MORAINE[0], MORAINE[1]);
    d -= 14 * Math.exp(-(dm * dm) / (2 * 120 * 120));
    if (dm < 300) d -= hashedBumps(e, n, 12, 11, 0.35, 0.6, 2.0, 0.9, 1.8, 1);
    // Meandering channel
    const nc = 600 + 150 * Math.sin(e / 350);
    d += 8 * Math.exp(-((n - nc) ** 2) / (2 * 40 * 40));
    // Rock shoal up to 6 m in the west
    d = Math.min(d, 6 + 0.02 * ((e - 300) ** 2 + (n - 1500) ** 2) ** 0.5 * 4);
    // Sand ripples in the shallows, pockmarks in the mud
    const shallow = Math.max(0, Math.min(1, (35 - d) / 10));
    if (shallow > 0) d -= 0.35 * shallow * Math.sin(2 * Math.PI * (e * 0.866 + n * 0.5) / 22);
    if (d > 30) d += hashedBumps(e, n, 60, 23, 0.15, 0.8, 1.2, 3, 5, 1);
    // Wreck
    d -= WRECK.height * slab(e, n, WRECK.e, WRECK.n, WRECK.half[0], WRECK.half[1], WRECK.heading, 2.5);
    return Math.max(3, d);
}

// max: the highest the surface gets within 500 m of the anchor
const SCENES = {
    terrain: { height: (e, n) => terrainRaw(e, n) - TERRAIN_ZERO, max: 42 },
    seabed: { height: (e, n) => -seabedDepth(e, n), max: -3 },
    cave: { sdf: caveSdf, path: CAVE_PATH, radius: CAVE_RADIUS },
    garda: { height: (e, n) => -gardaDepth(e, n), max: -2.5, origin: GARDA.origin, area: GARDA.area }
};

// Sphere tracing through the water of an SDF scene: 0 when the ray leaves by
// the surface, starts in the rock or goes beyond maxRange
function castSdf(scene, oe, on, ou, de, dn, du, maxRange) {
    if (scene.sdf(oe, on, ou) >= 0) return 0;
    let t = 0;
    for (let i = 0; i < 400; i++) {
        const e = oe + t * de, n = on + t * dn, u = ou + t * du;
        if (u > 0) return 0;
        const s = scene.sdf(e, n, u);
        if (s > -0.005) return t;
        t += Math.max(0.01, -0.7 * s);
        if (t > maxRange) return 0;
    }
    return 0;
}

/**
 * Distance along a ray (origin o, unit direction d, ENU) to the scene
 * surface, or 0 when it does not hit within maxRange. Marches by 0.4 of the
 * vertical clearance (slopes ≤ 1.5 are never stepped through), then bisects.
 */
function castRay(scene, oe, on, ou, de, dn, du, maxRange) {
    if (scene.sdf) return castSdf(scene, oe, on, ou, de, dn, du, maxRange);
    const h = scene.height;
    const clear = (t) => ou + t * du - h(oe + t * de, on + t * dn);
    let t = 0, c = clear(0);
    if (c <= 0) return 0;                   // the sensor is under the surface
    // Nothing above the scene's top can be hit: skip to it on downward rays
    if (du < 0 && ou > scene.max) t = Math.min(maxRange, (ou - scene.max) / -du);
    c = clear(t);
    let prev = t;
    for (let i = 0; i < 600 && t < maxRange; i++) {
        if (c <= 0) break;
        prev = t;
        t += Math.max(0.05, Math.min(8, 0.4 * c));
        c = clear(t);
    }
    if (c > 0) return 0;
    let a = prev, b = t;
    for (let i = 0; i < 14; i++) {
        const m = (a + b) / 2;
        if (clear(m) > 0) a = m; else b = m;
    }
    const r = (a + b) / 2;
    return r <= maxRange ? r : 0;
}

module.exports = { SCENES, castRay, CAVE_PATH, GARDA };
