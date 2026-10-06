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
 *            in the shallows, boulders left by the glacier on the plain,
 *            pockmarks, a rock shoal coming up to 6 m, a slump on the slope
 *            into the basin and a wreck 28 × 7 m standing 3.5 m proud.
 *            Natural relief at every scale from 8 to 420 m (gradient noise),
 *            rougher on the moraine and the shoal than on the mud of the
 *            basin. Beyond 90 m of range an imaging sonar sees none of the
 *            deepest part from the surface.
 *            On the bed of the shallow south-west (11–20 m), the OBJECTS:
 *            a motorboat wreck, a car, a 20 ft container, a light aircraft, a
 *            pipeline crossing, oil drums, mooring blocks, an artificial reef
 *            and an upturned rowing boat — sharp-edged boxes and cylinders
 *            ray-cast exactly, so a sonar sees their shadows and the widening
 *            its beam gives them.
 *   garda-complex  the same lake and objects, with the ground around them
 *            made hard to survey (COMPLEX: a rocky ridge with a gully, a
 *            scarp, a pinnacle, a boulder field, sand waves the pipeline
 *            spans, pockmarks, a channel, rock texture).
 *
 * The height-field details have soft edges (slopes ≤ 1.5) so castRay's step
 * stays safe; objects with vertical walls are not marched but intersected.
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

// Rotated ellipse (centre, semi-axes along / across the heading from north,
// soft edge), its rim pushed in and out by `wobble` (0..1 of the soft width)
function oval(e, n, ce, cn, a, b, headingDeg, soft, wobble = 0) {
    const h = headingDeg * Math.PI / 180, de = e - ce, dn = n - cn;
    const along = de * Math.sin(h) + dn * Math.cos(h), across = de * Math.cos(h) - dn * Math.sin(h);
    const r = Math.hypot(along / a, across / b), ang = Math.atan2(across / b, along / a);
    const rim = 1 + wobble * (soft / Math.min(a, b)) * (0.6 * Math.sin(3 * ang + 1.3) + 0.4 * Math.sin(7 * ang + 0.4));
    const out = (r - rim) * Math.min(a, b);            // metres outside the rim
    return out <= 0 ? 1 : out >= soft ? 0 : 0.5 + 0.5 * Math.cos(Math.PI * out / soft);
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
    // Metres per degree as rosbridge-sim.js anchors a scene (WGS84 series):
    // scene metres ↔ lat/lon the same way the emulator converts the vehicle
    const f = GARDA.origin.lat * Math.PI / 180;
    const mLat = 111132.954 - 559.822 * Math.cos(2 * f) + 1.175 * Math.cos(4 * f);
    const mLon = 111412.84 * Math.cos(f) - 93.5 * Math.cos(3 * f);
    GARDA.toLatLon = (e, n) => ({ lat: GARDA.origin.lat + n / mLat, lng: GARDA.origin.lon + e / mLon });
    GARDA.toEN = (lat, lon) => [(lon - GARDA.origin.lon) * mLon, (lat - GARDA.origin.lat) * mLat];
    const [w, h] = GARDA.size;
    GARDA.area = [[0, 0], [w, 0], [w, h], [0, h]].map(([e, n]) => GARDA.toLatLon(e, n));
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

// Gradient noise (Perlin's, quintic fade) on a unit grid, about ±0.7
const GRAD = [[1, 0], [0.7071, 0.7071], [0, 1], [-0.7071, 0.7071], [-1, 0], [-0.7071, -0.7071], [0, -1], [0.7071, -0.7071]];
function gdot(i, j, seed, dx, dy) {
    const g = GRAD[(hash01(i, j, seed) * 8) | 0];
    return g[0] * dx + g[1] * dy;
}
function fade(t) { return t * t * t * (t * (t * 6 - 15) + 10); }
function gradNoise(x, y, seed) {
    const i = Math.floor(x), j = Math.floor(y), fx = x - i, fy = y - j;
    const n00 = gdot(i, j, seed, fx, fy), n10 = gdot(i + 1, j, seed, fx - 1, fy);
    const n01 = gdot(i, j + 1, seed, fx, fy - 1), n11 = gdot(i + 1, j + 1, seed, fx - 1, fy - 1);
    const u = fade(fx), v = fade(fy);
    const a = n00 + u * (n10 - n00), b = n01 + u * (n11 - n01);
    return a + v * (b - a);
}

// Relief at every scale: [wavelength m, amplitude m]. The two long ones
// everywhere, the short ones scaled by how rough the bed is there (slopes of
// each octave ≤ 0.07)
const RELIEF_LONG = [[420, 1.4], [160, 0.7]];
const RELIEF_SHORT = [[60, 0.35], [22, 0.16], [8, 0.06]];
function relief(e, n, rough) {
    let h = 0, k = 0;
    for (const [w, a] of RELIEF_LONG) h += a * gradNoise(e / w + 17.3 * k, n / w - 9.1 * k, 101 + k++);
    for (const [w, a] of RELIEF_SHORT) h += rough * a * gradNoise(e / w + 17.3 * k, n / w - 9.1 * k, 101 + k++);
    return h;
}

// The lake bed alone (no objects), depth in metres
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
    const shoal = 6 + 0.02 * ((e - 300) ** 2 + (n - 1500) ** 2) ** 0.5 * 4;
    d = Math.min(d, shoal);
    // A slump on the slope into the basin: the scar where the sediment left
    // (3 m deeper, a steep head wall upslope), the lobe where it came to rest
    // downslope (2 m proud, spreading out)
    d += 3 * oval(e, n, 1425, 1300, 75, 48, 80, 22, 0.8) - 2 * oval(e, n, 1590, 1320, 60, 75, 80, 45, 0.6);
    // Natural relief: sand in the shallows to mud in the basin, rougher where
    // there is rock (the moraine, the shoal)
    const byDepth = Math.max(0.4, Math.min(1, (45 - d) / 20));
    const rock = 0.6 * Math.exp(-(dm * dm) / (2 * 150 * 150)) + (shoal - d < 4 ? 0.6 : 0);
    d -= relief(e, n, byDepth + rock);
    // Glacial erratics scattered on the plain, sand ripples in the
    // shallows, pockmarks in the mud
    if (d < 30) d -= hashedBumps(e, n, 45, 31, 0.07, 0.5, 1.4, 0.7, 1.5, 1);
    const shallow = Math.max(0, Math.min(1, (35 - d) / 10));
    if (shallow > 0) d -= 0.35 * shallow * Math.sin(2 * Math.PI * (e * 0.866 + n * 0.5) / 22);
    if (d > 30) d += hashedBumps(e, n, 60, 23, 0.15, 0.8, 1.2, 3, 5, 1);
    // Wreck
    d -= WRECK.height * slab(e, n, WRECK.e, WRECK.n, WRECK.half[0], WRECK.half[1], WRECK.heading, 2.5);
    return Math.max(3, d);
}
const gardaBed = (e, n) => -gardaDepth(e, n);

// ── Objects on the bed ──────────────────────────────────────────────────────
// Parts are boxes turned about the vertical (centre, half sizes along / across
// the heading / up, heading) and cylinders between two points; ENU metres
// from the scene origin. Each part and each object carries a bounding sphere.

// Ray (unit d) into a box: the entry distance, or Infinity
function hitBox(p, oe, on, ou, de, dn, du) {
    const s = p.sin, c = p.cos;
    const pe = oe - p.c[0], pn = on - p.c[1], pu = ou - p.c[2];
    const o = [pe * s + pn * c, pe * c - pn * s, pu], d = [de * s + dn * c, de * c - dn * s, du];
    let t0 = -Infinity, t1 = Infinity;
    for (let a = 0; a < 3; a++) {
        const h = p.h[a];
        if (Math.abs(d[a]) < 1e-12) { if (Math.abs(o[a]) > h) return Infinity; continue; }
        let ta = (-h - o[a]) / d[a], tb = (h - o[a]) / d[a];
        if (ta > tb) { const x = ta; ta = tb; tb = x; }
        if (ta > t0) t0 = ta;
        if (tb < t1) t1 = tb;
        if (t0 > t1) return Infinity;
    }
    return t0 > 0 ? t0 : Infinity;
}

// Ray into a capped cylinder (axis a → b, radius r): the entry distance, or Infinity
function hitCyl(p, oe, on, ou, de, dn, du) {
    const [bx, by, bz] = p.ba, baba = p.baba, r = p.r;
    const ox = oe - p.a[0], oy = on - p.a[1], oz = ou - p.a[2];
    const bard = bx * de + by * dn + bz * du, baoc = bx * ox + by * oy + bz * oz;
    const k2 = baba - bard * bard;
    if (k2 > 1e-9 * baba) {
        const k1 = baba * (ox * de + oy * dn + oz * du) - baoc * bard;
        const k0 = baba * (ox * ox + oy * oy + oz * oz) - baoc * baoc - r * r * baba;
        const h = k1 * k1 - k2 * k0;
        if (h < 0) return Infinity;
        const t = (-k1 - Math.sqrt(h)) / k2, y = baoc + t * bard;
        if (t > 0 && y > 0 && y < baba) return t;
    }
    // Through a cap (also a ray along the axis)
    if (Math.abs(bard) < 1e-12) return Infinity;
    let best = Infinity;
    for (const cap of [0, baba]) {
        const t = (cap - baoc) / bard;
        if (!(t > 0) || t >= best) continue;
        const f = cap / baba;
        const qx = ox + t * de - f * bx, qy = oy + t * dn - f * by, qz = oz + t * du - f * bz;
        if (qx * qx + qy * qy + qz * qz <= r * r) best = t;
    }
    return best;
}

// The ray passes within the sphere before maxRange
function nearSphere(s, oe, on, ou, de, dn, du, maxRange) {
    const ce = s[0] - oe, cn = s[1] - on, cu = s[2] - ou;
    const t = Math.max(0, Math.min(maxRange, ce * de + cn * dn + cu * du));
    const x = ce - t * de, y = cn - t * dn, z = cu - t * du;
    return x * x + y * y + z * z <= s[3] * s[3];
}

/** Nearest object along a ray within maxRange, or 0. */
function castObjects(objects, oe, on, ou, de, dn, du, maxRange) {
    let best = Infinity;
    for (const ob of objects) {
        if (!nearSphere(ob.sphere, oe, on, ou, de, dn, du, Math.min(best, maxRange))) continue;
        for (const p of ob.parts) {
            if (!nearSphere(p.sphere, oe, on, ou, de, dn, du, Math.min(best, maxRange))) continue;
            const t = p.type === 'box' ? hitBox(p, oe, on, ou, de, dn, du) : hitCyl(p, oe, on, ou, de, dn, du);
            if (t < best) best = t;
        }
    }
    return best <= maxRange ? best : 0;
}

// Top of the objects at (e, n), or -Infinity
function objectsTop(objects, e, n) {
    const t = castObjects(objects, e, n, 50, 0, 0, -1, 500);
    return t ? 50 - t : -Infinity;
}

// An object at (e, n) heading `heading` (° from north), sunk `bury` m into the
// bed: parts in its own frame (x along the heading, y to starboard, z up from
// the bed under its centre)
function placeObject(name, e, n, heading, bury, parts, bed) {
    const a = heading * Math.PI / 180, s = Math.sin(a), c = Math.cos(a);
    const base = bed(e, n) - bury;
    const world = (x, y, z) => [e + x * s + y * c, n + x * c - y * s, base + z];
    const out = [];
    for (const q of parts) {
        if (q.box) {
            const [x, y, z0, len, wid, hgt] = q.box;
            const cc = world(x, y, z0 + hgt / 2), h = [len / 2, wid / 2, hgt / 2];
            out.push({ type: 'box', c: cc, h, sin: s, cos: c, sphere: [...cc, Math.hypot(...h)] });
        } else {
            const A = world(...q.cyl[0]), B = world(...q.cyl[1]), r = q.cyl[2];
            out.push(cylPart(A, B, r));
        }
    }
    return withSphere({ name, e, n, heading, depth: -bed(e, n), parts: out });
}

function cylPart(A, B, r) {
    const ba = [B[0] - A[0], B[1] - A[1], B[2] - A[2]], baba = ba[0] ** 2 + ba[1] ** 2 + ba[2] ** 2;
    const mid = [(A[0] + B[0]) / 2, (A[1] + B[1]) / 2, (A[2] + B[2]) / 2];
    return { type: 'cyl', a: A, ba, baba, r, sphere: [...mid, Math.sqrt(baba) / 2 + r] };
}

// Bounding sphere of an object around its parts', and its height above the bed
function withSphere(ob) {
    let ce = 0, cn = 0, cu = 0;
    for (const p of ob.parts) { ce += p.sphere[0]; cn += p.sphere[1]; cu += p.sphere[2]; }
    const k = ob.parts.length;
    ce /= k; cn /= k; cu /= k;
    let r = 0, top = -Infinity;
    for (const p of ob.parts) {
        r = Math.max(r, Math.hypot(p.sphere[0] - ce, p.sphere[1] - cn, p.sphere[2] - cu) + p.sphere[3]);
        top = Math.max(top, p.type === 'box' ? p.c[2] + p.h[2] : Math.max(p.a[2], p.a[2] + p.ba[2]) + p.r);
    }
    ob.sphere = [ce, cn, cu, r];
    ob.proud = top + ob.depth;
    return ob;
}

function gardaObjects(bed) {
    const obs = [];
    obs.push(placeObject('Motorboat wreck 9 m', 255, 305, 40, 0.3, [
        { box: [0, 0, 0, 9, 3, 1.6] },                       // hull
        { box: [0.8, 0, 1.6, 3.5, 2.2, 1.3] }                // wheelhouse
    ], bed));
    obs.push(placeObject('Car', 425, 265, 110, 0.1, [
        { box: [0, 0, 0.1, 4.4, 1.8, 0.75] },
        { box: [-0.2, 0, 0.85, 2.3, 1.6, 0.55] }
    ], bed));
    obs.push(placeObject('Container 20 ft', 335, 455, 15, 0.2, [{ box: [0, 0, 0, 6.06, 2.44, 2.59] }], bed));
    obs.push(placeObject('Light aircraft', 485, 415, 200, 0.2, [
        { cyl: [[-4, 0, 0.65], [4, 0, 0.65], 0.65] },        // fuselage
        { box: [0.8, 0, 0.5, 1.6, 11, 0.25] },               // wing
        { box: [-3.7, 0, 0.9, 0.9, 3.4, 0.15] },             // tailplane
        { box: [-3.7, 0, 1.0, 1.0, 0.15, 1.4] }              // fin
    ], bed));
    obs.push(placeObject('Upturned rowing boat', 300, 215, 160, 0.1, [{ box: [0, 0, 0, 4.8, 1.5, 0.6] }], bed));
    // Artificial reef: nine 2 m cubes, 4 m apart
    const reef = [];
    for (let i = -1; i <= 1; i++) for (let j = -1; j <= 1; j++) reef.push({ box: [4 * i, 4 * j, 0, 2, 2, 2] });
    obs.push(placeObject('Artificial reef', 505, 235, 30, 0.2, reef, bed));
    // Four mooring blocks 20 m apart along 70°
    for (let k = 0; k < 4; k++) {
        const s = 20 * k - 30;
        obs.push(placeObject(`Mooring block ${k + 1}`, 375 + s * Math.sin(70 * Math.PI / 180), 565 + s * Math.cos(70 * Math.PI / 180), 70 + 13 * k, 0.15,
            [{ box: [0, 0, 0, 1.5, 1.5, 1.0] }], bed));
    }
    // Oil drums: twelve upright, two lying, within 6 m
    const drums = [];
    for (let k = 0; k < 14; k++) {
        const x = (hash01(k, 1, 77) - 0.5) * 12, y = (hash01(k, 2, 77) - 0.5) * 12;
        if (k < 12) drums.push({ cyl: [[x, y, 0], [x, y, 0.9], 0.3] });
        else drums.push({ cyl: [[x - 0.45, y, 0.3], [x + 0.45, y, 0.3], 0.3] });
    }
    obs.push(placeObject('Oil drums', 205, 525, 0, 0.05, drums, bed));
    // A pipeline Ø 0.8 m across the field, laid on the bed in 25 m lengths,
    // a quarter of it in the sediment
    const P0 = [150, 395], P1 = [550, 545], len = Math.hypot(P1[0] - P0[0], P1[1] - P0[1]), nSeg = Math.round(len / 25);
    const at = (k) => {
        const f = k / nSeg, e = P0[0] + f * (P1[0] - P0[0]), n = P0[1] + f * (P1[1] - P0[1]);
        return [e, n, bed(e, n) + 0.2];
    };
    for (let k = 0; k < nSeg; k++) {
        obs.push(withSphere({ name: `Pipeline ${k + 1}/${nSeg}`, e: at(k + 0.5)[0], n: at(k + 0.5)[1], heading: 0, depth: -at(k + 0.5)[2] + 0.2,
            parts: [cylPart(at(k), at(k + 1), 0.4)] }));
    }
    return obs;
}
const GARDA_OBJECTS = gardaObjects(gardaBed);
GARDA.objects = GARDA_OBJECTS;

// ── A complex bed over the survey area (scene 'garda-complex') ──────────────
// Around the objects (south-west plain, 11–17 m), the ground that makes a
// survey hard, on top of the same lake bed: a rocky ridge running east–west,
// 3–4 m high with an uneven crest and a gully cut through it; a scarp where
// the bed steps 2.6 m down to the east; a rock pinnacle rising 5.5 m; a field
// of boulders up to 1.8 m; sand waves 0.8 m high, 14 m apart, which the
// pipeline spans; pockmarks with raised rims; a channel through the ridge's
// west end; and a 2–4 m rock texture on the rocky parts. Height up, m, added
// to the bed. Slopes ≤ 1.5, like the rest (castRay).
const COMPLEX = {
    ridge: { n: 365, e0: 225, e1: 435, gully: 332 },
    scarp: { e: 380, n0: 202, n1: 342, drop: 2.6 },
    pinnacle: { e: 405, n: 310, height: 5.5 },
    boulders: { e0: 227, e1: 317, n0: 236, n1: 290 },
    sandWaves: { e0: 353, e1: 443, n0: 397, n1: 473, height: 0.8, length: 14 },
    pockmarks: [[265, 425], [292, 447], [248, 452]],
    channel: [[238, 485], [232, 405], [246, 330]]
};
function complexRelief(e, n) {
    let h = 0, rocky = 0;
    // Ridge, wavy, its crest from 2.8 to 4.4 m; a gully 6 m wide through it
    const R = COMPLEX.ridge;
    if (e > R.e0 - 30 && e < R.e1 + 30 && n > R.n - 35 && n < R.n + 35) {
        const nr = R.n + 6 * Math.sin(e / 23);
        const H = (3.6 + 0.8 * gradNoise(e / 30, 3.3, 201)) * plateau(e - (R.e0 + R.e1) / 2, (R.e1 - R.e0) / 2, 20);
        const r = H * plateau(n - nr, 7, 7) * (1 - 0.85 * plateau(e - R.gully, 3, 5));
        h += r;
        rocky = Math.max(rocky, Math.min(1, r / 2));
    }
    // Scarp: the bed east of it 2.6 m deeper, over 4 m
    const S = COMPLEX.scarp;
    if (e > S.e - 10 && n > S.n0 - 25 && n < S.n1 + 25) {
        const x = Math.max(-1, Math.min(1, (e - S.e - 4 * Math.sin(n / 17)) / 2));
        h -= S.drop * (0.5 + 0.5 * Math.sin(x * Math.PI / 2)) * plateau(n - (S.n0 + S.n1) / 2, (S.n1 - S.n0) / 2, 20);
    }
    // Pinnacle on a broad base
    const P = COMPLEX.pinnacle, dp2 = (e - P.e) ** 2 + (n - P.n) ** 2;
    if (dp2 < 900) {
        const p = P.height * Math.exp(-dp2 / (2 * 2.3 * 2.3)) + 2 * Math.exp(-dp2 / (2 * 6 * 6));
        h += p;
        rocky = Math.max(rocky, Math.min(1, p / 2));
    }
    // Boulder field
    const B = COMPLEX.boulders;
    if (e > B.e0 - 10 && e < B.e1 + 10 && n > B.n0 - 10 && n < B.n1 + 10) {
        const z = plateau(e - (B.e0 + B.e1) / 2, (B.e1 - B.e0) / 2, 8) * plateau(n - (B.n0 + B.n1) / 2, (B.n1 - B.n0) / 2, 8);
        if (z > 0) {
            h += z * hashedBumps(e, n, 6, 41, 0.45, 0.4, 1.8, 0.8, 1.6, 1);
            rocky = Math.max(rocky, z);
        }
    }
    // Sand waves: crests running NNW–SSE, wandering, steeper on the lee side
    const W = COMPLEX.sandWaves;
    if (e > W.e0 - 20 && e < W.e1 + 20 && n > W.n0 - 20 && n < W.n1 + 20) {
        const z = plateau(e - (W.e0 + W.e1) / 2, (W.e1 - W.e0) / 2, 15) * plateau(n - (W.n0 + W.n1) / 2, (W.n1 - W.n0) / 2, 15);
        if (z > 0) {
            const ph = 2 * Math.PI * (e * 0.94 + n * 0.342 + 6 * gradNoise(e / 60, n / 60, 211)) / W.length;
            h += z * W.height * (Math.sin(ph) + 0.3 * Math.sin(2 * ph)) / 1.15;
        }
    }
    // Pockmarks: a crater 1.8 m deep and its rim
    for (const [pe, pn] of COMPLEX.pockmarks) {
        const r = Math.hypot(e - pe, n - pn);
        if (r < 14) h += -1.8 * Math.exp(-r * r / (2 * 3.5 * 3.5)) + 0.35 * Math.exp(-((r - 7) ** 2) / (2 * 1.5 * 1.5));
    }
    // Channel 1.2 m deep, 3 m wide at the bottom
    const C = COMPLEX.channel;
    const dc = Math.min(segDist(e, n, C[0], C[1]), segDist(e, n, C[1], C[2]));
    if (dc < 6) h -= 1.2 * plateau(dc, 1.5, 3) * plateau(n - 407, 62, 15);
    // Rock texture
    if (rocky > 0) h += rocky * (0.18 * gradNoise(e / 4, n / 4, 301) + 0.07 * gradNoise(e / 1.8, n / 1.8, 302));
    return h;
}
const gardaComplexBed = (e, n) => -Math.max(3, gardaDepth(e, n) - complexRelief(e, n));
const GARDA_COMPLEX_OBJECTS = gardaObjects(gardaComplexBed);

// max: the highest the surface gets within 500 m of the anchor. A scene with
// `bed` and `objects` is marched over the bed and intersects the objects;
// `height` is the top of either (for the tests and the map).
const SCENES = {
    terrain: { height: (e, n) => terrainRaw(e, n) - TERRAIN_ZERO, max: 42 },
    seabed: { height: (e, n) => -seabedDepth(e, n), max: -3 },
    cave: { sdf: caveSdf, path: CAVE_PATH, radius: CAVE_RADIUS },
    garda: {
        bed: gardaBed, objects: GARDA_OBJECTS,
        height: (e, n) => Math.max(gardaBed(e, n), objectsTop(GARDA_OBJECTS, e, n)),
        max: -2.5, origin: GARDA.origin, area: GARDA.area
    },
    'garda-complex': {
        bed: gardaComplexBed, objects: GARDA_COMPLEX_OBJECTS,
        height: (e, n) => Math.max(gardaComplexBed(e, n), objectsTop(GARDA_COMPLEX_OBJECTS, e, n)),
        max: -2.5, origin: GARDA.origin, area: GARDA.area
    }
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
    if (!scene.objects) return castHeight(scene.height, scene.max, oe, on, ou, de, dn, du, maxRange);
    const tb = castHeight(scene.bed, scene.max, oe, on, ou, de, dn, du, maxRange);
    const to = castObjects(scene.objects, oe, on, ou, de, dn, du, tb || maxRange);
    return to && (!tb || to < tb) ? to : tb;
}

function castHeight(h, top, oe, on, ou, de, dn, du, maxRange) {
    const clear = (t) => ou + t * du - h(oe + t * de, on + t * dn);
    let t = 0, c = clear(0);
    if (c <= 0) return 0;                   // the sensor is under the surface
    // Nothing above the scene's top can be hit: skip to it on downward rays
    if (du < 0 && ou > top) t = Math.min(maxRange, (ou - top) / -du);
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

module.exports = { SCENES, castRay, castObjects, CAVE_PATH, GARDA, COMPLEX };
