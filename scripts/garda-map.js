#!/usr/bin/env node
/**
 * garda-map.js - a bathymetric map of the Garda scene, or of a surveyed surface
 *
 * Renders the lake bed of scripts/ros-sim-scenes.js (the 'garda' scene: the
 * bed, its relief and the objects on it) as a PNG: depth colours, hill shading
 * from the north-west, isobaths, the objects numbered (the legend on stdout),
 * a scale bar and north. The truth the rosbridge emulator's sonars see.
 *
 * With --tiles it draws a SURVEYED surface instead — the ROS surface as the
 * GCS averaged it (JSON: { anchor, cell, tiles: [{ tx, ty, h, w }] }, as
 * getRosSurface() holds it, the arrays as plain lists) — drawn as the GCS's
 * mesh draws it (js/ros/SurfaceRaster.js: each cell at the finest level with
 * MIN SAMPLES, holes filled up to --fill metres), in the same frame and
 * colours, undrawn cells grey, so the survey and the truth compare side by
 * side; --diff colours the difference from the truth instead (blue: survey
 * too deep, red: too shallow). It also prints how far the survey is from the
 * scene, and what it made of each object. --against an earlier snapshot of
 * the same surface colours what changed since (red: the surface rose, blue:
 * it sank) and tells how much the passes in between changed the cells drawn
 * in both, and whether they moved them towards the scene.
 *
 *   node scripts/garda-map.js [--scene garda|garda-complex] [--out garda.png] [--area e0,n0,e1,n1] [--res 2]
 *        [--contour 5] [--exaggerate 3] [--box e0,n0,e1,n1]
 *        [--tiles surface.json [--fill 8] [--min-samples 1] [--diff | --against earlier.json]]
 *
 * --area   what to draw, scene metres east / north of the scene origin
 *          (45.4960 N, 10.6490 E); default the whole 2.5 × 2 km
 * --res    metres per pixel (default: the area in at most 1600 px across)
 * --box    a rectangle to outline (a survey area)
 */

const fs = require('fs');
const path = require('path');
const zlib = require('zlib');
const { SCENES, GARDA } = require('./ros-sim-scenes');

async function main() {

const argv = process.argv.slice(2);
function arg(name, def) {
    const i = argv.indexOf(`--${name}`);
    return i >= 0 && argv[i + 1] !== undefined && !argv[i + 1].startsWith('--') ? argv[i + 1] : def;
}
const nums = (s) => s.split(',').map(Number);
const AREA = nums(arg('area', `0,0,${GARDA.size[0]},${GARDA.size[1]}`));
const W_M = AREA[2] - AREA[0], H_M = AREA[3] - AREA[1];
const RES = parseFloat(arg('res', String(Math.max(W_M, H_M) / 1600)));
const W = Math.round(W_M / RES), H = Math.round(H_M / RES);
const CONTOUR = parseFloat(arg('contour', W_M > 1000 ? '5' : '1'));
const EXAG = parseFloat(arg('exaggerate', W_M > 1000 ? '3' : '1.5'));
const BOX = arg('box', null) ? nums(arg('box')) : null;
const OUT = arg('out', 'garda-bed.png');
const TILES = arg('tiles', null);
const DIFF = argv.includes('--diff');
const AGAINST = arg('against', null);
const FILL = parseFloat(arg('fill', '8'));
const MIN_SAMPLES = parseFloat(arg('min-samples', '1'));
const SCENE_NAME = arg('scene', 'garda');
const scene = SCENES[SCENE_NAME];
if (!scene || !scene.objects) { console.error(`--scene: garda or garda-complex`); process.exit(1); }

// ---------- heights ----------
// The scene's, or the surveyed surface's as the GCS draws it (NaN where it
// draws nothing), per pixel centre
// The renderer's own module (ES, no imports), as the tests load it
const R = await import('data:text/javascript,' + encodeURIComponent(fs.readFileSync(path.join(__dirname, '..', 'js/ros/SurfaceRaster.js'), 'utf8')));
function loadSurvey(file) {
    const s = JSON.parse(fs.readFileSync(file, 'utf8'));
    const a = s.anchor, c = s.cell;
    // scene metres → lat/lon → the surface's own anchor (spherical, as the GCS places it)
    const toXY = (e, n) => { const p = GARDA.toLatLon(e, n); return [(p.lng - a.lon) * a.mPerLon, (p.lat - a.lat) * a.mPerLat]; };
    // One grid over the area (and a margin for the coarse levels), the cells the tiles hold
    const corners = [toXY(AREA[0] - 20, AREA[1] - 20), toXY(AREA[2] + 20, AREA[3] + 20)];
    const ci0 = Math.floor(corners[0][0] / c), cj0 = Math.floor(corners[0][1] / c);
    const gw = Math.ceil(corners[1][0] / c) - ci0, gh = Math.ceil(corners[1][1] / c) - cj0;
    const data = new Float32Array(gw * gh).fill(R.EMPTY), wts = new Float32Array(gw * gh), drawn = new Float32Array(gw * gh * 2);
    for (const t of s.tiles) {
        for (let k = 0; k < 900; k++) {
            if (!(t.w[k] > 0) || t.h[k] === null) continue;
            const i = t.tx * 30 + (k % 30) - ci0, j = t.ty * 30 + Math.floor(k / 30) - cj0;
            if (i < 0 || j < 0 || i >= gw || j >= gh) continue;
            data[j * gw + i] = t.h[k]; wts[j * gw + i] = t.w[k];
        }
    }
    const fillLevels = R.fillLevelsFor(FILL, c);
    R.rasterize(data, wts, gw, gh, drawn, { cell: c, minSamples: MIN_SAMPLES, fillLevels });
    console.log(`${path.basename(file)}: ${s.tiles.length} tiles of ${c} m cells, drawn with holes filled up to ${(2 ** fillLevels) * c} m, MIN SAMPLES ${MIN_SAMPLES}`);
    const cellOf = (e, n) => {
        const [x, y] = toXY(e, n), i = Math.floor(x / c) - ci0, j = Math.floor(y / c) - cj0;
        return i < 0 || j < 0 || i >= gw || j >= gh ? -1 : j * gw + i;
    };
    // Heights relative to the anchor: the water surface the boat started on, the scene's 0
    return {
        surveyed: (e, n) => { const p = cellOf(e, n); return p < 0 || !(drawn[p * 2] > R.EMPTY) ? NaN : drawn[p * 2]; },
        measured: (e, n) => { const p = cellOf(e, n); return p >= 0 && wts[p] > 0; }
    };
}
let surveyed = null, measured = null, earlier = null;
if (TILES) ({ surveyed, measured } = loadSurvey(TILES));
if (TILES && AGAINST) earlier = loadSurvey(AGAINST);
const t0 = Date.now();
const Z = new Float32Array(W * H), TRUTH = new Float32Array(W * H), OBJ = new Uint8Array(W * H), MEAS = new Uint8Array(W * H);
const PREV = earlier ? new Float32Array(W * H) : null, PMEAS = earlier ? new Uint8Array(W * H) : null;
for (let j = 0; j < H; j++) {
    const n = AREA[3] - (j + 0.5) * RES;
    for (let i = 0; i < W; i++) {
        const e = AREA[0] + (i + 0.5) * RES, k = j * W + i;
        const h = scene.height(e, n);
        TRUTH[k] = h;
        Z[k] = surveyed ? surveyed(e, n) : h;
        if (measured && measured(e, n)) MEAS[k] = 1;
        if (earlier) { PREV[k] = earlier.surveyed(e, n); if (earlier.measured(e, n)) PMEAS[k] = 1; }
        if (!surveyed && h - scene.bed(e, n) > 0.05) OBJ[k] = 1;
    }
}
console.log(`${W} × ${H} px at ${RES} m/px, heights in ${((Date.now() - t0) / 1000).toFixed(1)} s`);

// ---------- colours ----------
const RAMP = [[0, [214, 241, 238]], [8, [150, 214, 214]], [15, [88, 178, 196]], [25, [52, 132, 176]], [40, [33, 92, 150]], [60, [20, 58, 112]], [85, [10, 30, 70]]];
function ramp(d) {
    if (d <= RAMP[0][0]) return RAMP[0][1];
    for (let k = 1; k < RAMP.length; k++) {
        if (d <= RAMP[k][0]) {
            const [d0, c0] = RAMP[k - 1], [d1, c1] = RAMP[k], f = (d - d0) / (d1 - d0);
            return c0.map((v, c) => v + f * (c1[c] - v));
        }
    }
    return RAMP[RAMP.length - 1][1];
}
function diverge(x) {
    // survey − truth: red too shallow (positive), blue too deep, white within 2 cm
    const f = Math.max(-1, Math.min(1, x / 0.5));
    return f >= 0 ? [255, 255 - 200 * f, 255 - 220 * f] : [255 + 220 * f, 255 + 160 * f, 255];
}

const img = new Uint8Array(W * H * 3);
const L = (() => { const az = 315 * Math.PI / 180, el = 40 * Math.PI / 180; return [Math.sin(az) * Math.cos(el), Math.cos(az) * Math.cos(el), Math.sin(el)]; })();
const at = (i, j) => Z[Math.min(H - 1, Math.max(0, j)) * W + Math.min(W - 1, Math.max(0, i))];
for (let j = 0; j < H; j++) {
    for (let i = 0; i < W; i++) {
        const k = j * W + i, z = Z[k];
        let c;
        if (Number.isNaN(z) || (PREV && Number.isNaN(PREV[k]))) c = PREV && !Number.isNaN(z) ? [150, 150, 150] : [62, 66, 72];
        else if (PREV) c = diverge(z - PREV[k]);
        else if (DIFF) c = diverge(z - TRUTH[k]);
        else {
            // hill shade from the neighbours (NaN neighbours: this pixel's height)
            const zr = at(i + 1, j), zl = at(i - 1, j), zu = at(i, j - 1), zd = at(i, j + 1);
            const dzx = ((Number.isNaN(zr) ? z : zr) - (Number.isNaN(zl) ? z : zl)) / (2 * RES) * EXAG;
            const dzy = ((Number.isNaN(zu) ? z : zu) - (Number.isNaN(zd) ? z : zd)) / (2 * RES) * EXAG;
            const len = Math.hypot(dzx, dzy, 1);
            const shade = Math.max(0, (-dzx * L[0] - dzy * L[1] + L[2]) / len);
            const base = OBJ[k] ? [236, 170, 70] : ramp(-z);
            c = base.map(v => v * (0.45 + 0.75 * shade));
            // isobaths: where the contour band changes to the right or below
            const band = Math.floor(-z / CONTOUR);
            const br = Math.floor(-at(i + 1, j) / CONTOUR), bd = Math.floor(-at(i, j + 1) / CONTOUR);
            if (!OBJ[k] && ((Number.isFinite(br) && br !== band) || (Number.isFinite(bd) && bd !== band))) {
                const major = Math.max(band, br, bd) % 5 === 0;
                c = c.map(v => v * (major ? 0.35 : 0.65));
            }
        }
        img[k * 3] = Math.max(0, Math.min(255, c[0]));
        img[k * 3 + 1] = Math.max(0, Math.min(255, c[1]));
        img[k * 3 + 2] = Math.max(0, Math.min(255, c[2]));
    }
}

// ---------- overlays ----------
function px(i, j, c) {
    if (i < 0 || j < 0 || i >= W || j >= H) return;
    const k = (j * W + i) * 3;
    img[k] = c[0]; img[k + 1] = c[1]; img[k + 2] = c[2];
}
function rect(i0, j0, w, h, c) { for (let j = j0; j < j0 + h; j++) for (let i = i0; i < i0 + w; i++) px(i, j, c); }
const toPx = (e, n) => [Math.round((e - AREA[0]) / RES), Math.round((AREA[3] - n) / RES)];

// A small bitmap font: digits, the few letters the overlays use
const GLYPHS = {
    0: ['111', '101', '101', '101', '111'], 1: ['010', '110', '010', '010', '111'], 2: ['111', '001', '111', '100', '111'],
    3: ['111', '001', '111', '001', '111'], 4: ['101', '101', '111', '001', '001'], 5: ['111', '100', '111', '001', '111'],
    6: ['111', '100', '111', '101', '111'], 7: ['111', '001', '001', '001', '001'], 8: ['111', '101', '111', '101', '111'],
    9: ['111', '101', '111', '001', '111'], ' ': ['0', '0', '0', '0', '0'], m: ['00000', '11110', '10101', '10101', '10101'],
    k: ['100', '101', '110', '101', '101'], N: ['10001', '11001', '10101', '10011', '10001']
};
const SCALE = Math.max(2, Math.round(W / 500));
function text(s, i, j, c, halo = [255, 255, 255]) {
    let x = i;
    for (const ch of String(s)) {
        const g = GLYPHS[ch] || GLYPHS[' '];
        for (const pass of [0, 1]) {
            for (let r = 0; r < 5; r++) for (let q = 0; q < g[r].length; q++) {
                if (g[r][q] !== '1') continue;
                if (pass === 0) rect(x + q * SCALE - 1, j + r * SCALE - 1, SCALE + 2, SCALE + 2, halo);
                else rect(x + q * SCALE, j + r * SCALE, SCALE, SCALE, c);
            }
        }
        x += (g[0].length + 1) * SCALE;
    }
}

if (BOX) {
    const [a0, b0] = toPx(BOX[0], BOX[3]), [a1, b1] = toPx(BOX[2], BOX[1]);
    for (let i = a0; i <= a1; i++) if ((i >> 3) & 1) { rect(i, b0 - 1, 1, 3, [255, 255, 255]); rect(i, b1 - 1, 1, 3, [255, 255, 255]); }
    for (let j = b0; j <= b1; j++) if ((j >> 3) & 1) { rect(a0 - 1, j, 3, 1, [255, 255, 255]); rect(a1 - 1, j, 3, 1, [255, 255, 255]); }
}

// The objects, numbered (a pipeline once, at its middle)
const legend = [];
const seen = new Set();
for (const o of scene.objects) {
    const key = o.name.startsWith('Pipeline') ? 'Pipeline' : o.name;
    if (seen.has(key)) continue;
    let ref = o;
    if (key === 'Pipeline') {
        const all = scene.objects.filter(x => x.name.startsWith('Pipeline'));
        ref = all[Math.floor(all.length / 2)];
    }
    seen.add(key);
    const [i, j] = toPx(ref.e, ref.n);
    const inside = i >= 0 && j >= 0 && i < W && j < H;
    const ll = GARDA.toLatLon(ref.e, ref.n);
    legend.push({ k: legend.length + 1, name: key === 'Pipeline' ? 'Pipeline Ø 0.8 m, 425 m' : o.name,
        depth: ref.depth, proud: ref.proud, lat: ll.lat, lon: ll.lng, inside });
    if (inside) text(legend.length, i + 4 * SCALE, j - 3 * SCALE, [20, 20, 20]);
}

// Scale bar (a round length near a fifth of the width) and north
const target = W_M / 5, steps = [5, 10, 20, 25, 50, 100, 200, 250, 500, 1000];
const bar = steps.reduce((b, s) => (Math.abs(s - target) < Math.abs(b - target) ? s : b), steps[0]);
const barPx = Math.round(bar / RES), bx = 6 * SCALE, by = H - 9 * SCALE;
for (let s = 0; s < 4; s++) rect(bx + Math.round(s * barPx / 4), by, Math.round(barPx / 4), 2 * SCALE, s % 2 ? [255, 255, 255] : [20, 20, 20]);
rect(bx - 1, by - 1, barPx + 2, 1, [20, 20, 20]); rect(bx - 1, by + 2 * SCALE, barPx + 2, 1, [20, 20, 20]);
text(bar >= 1000 ? `${bar / 1000} km` : `${bar} m`, bx, by - 7 * SCALE, [20, 20, 20]);
const nx = W - 10 * SCALE, ny = 4 * SCALE;
for (let r = 0; r < 8 * SCALE; r++) rect(nx - Math.floor(r / 3), ny + 6 * SCALE + r, 2 * Math.floor(r / 3) + 1, 1, [20, 20, 20]);
text('N', nx - 2 * SCALE, ny, [20, 20, 20]);

// ---------- PNG ----------
function crc32(buf) {
    let c, crc = 0xffffffff;
    for (let n = 0; n < buf.length; n++) {
        c = (crc ^ buf[n]) & 0xff;
        for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1;
        crc = (crc >>> 8) ^ c;
    }
    return (crc ^ 0xffffffff) >>> 0;
}
function chunk(type, data) {
    const len = Buffer.alloc(4); len.writeUInt32BE(data.length);
    const td = Buffer.concat([Buffer.from(type, 'ascii'), data]);
    const crc = Buffer.alloc(4); crc.writeUInt32BE(crc32(td));
    return Buffer.concat([len, td, crc]);
}
const raw = Buffer.alloc((W * 3 + 1) * H);
for (let j = 0; j < H; j++) { raw[j * (W * 3 + 1)] = 0; Buffer.from(img.buffer, j * W * 3, W * 3).copy(raw, j * (W * 3 + 1) + 1); }
const ihdr = Buffer.alloc(13);
ihdr.writeUInt32BE(W, 0); ihdr.writeUInt32BE(H, 4); ihdr[8] = 8; ihdr[9] = 2; ihdr[10] = 0; ihdr[11] = 0; ihdr[12] = 0;
fs.writeFileSync(OUT, Buffer.concat([Buffer.from([137, 80, 78, 71, 13, 10, 26, 10]), chunk('IHDR', ihdr), chunk('IDAT', zlib.deflateSync(raw, { level: 9 })), chunk('IEND', Buffer.alloc(0))]));

// ---------- legend ----------
let lo = Infinity, hi = -Infinity;
for (const z of Z) if (!Number.isNaN(z)) { lo = Math.min(lo, -z); hi = Math.max(hi, -z); }
console.log(`${OUT}: ${TILES ? (DIFF ? 'survey − truth' : 'surveyed surface') : `scene ${SCENE_NAME}`} · ${W_M} × ${H_M} m from ${GARDA.origin.lat} N ${GARDA.origin.lon} E + (${AREA[0]}, ${AREA[1]}) m · depth ${lo.toFixed(1)}–${hi.toFixed(1)} m · isobaths every ${CONTOUR} m (bold every ${5 * CONTOUR}) · scale bar ${bar} m`);
for (const l of legend) {
    console.log(`  ${String(l.k).padStart(2)}  ${l.name.padEnd(30)} ${l.depth.toFixed(1).padStart(5)} m deep · ${l.proud.toFixed(1)} m proud · ${l.lat.toFixed(6)} N ${l.lon.toFixed(6)} E${l.inside ? '' : ' (outside)'}`);
}

// ---------- the survey against the scene ----------
if (surveyed) {
    const stats = (sel) => {
        const e = [];
        for (let k = 0; k < Z.length; k++) if (!Number.isNaN(Z[k]) && sel(k)) e.push(Z[k] - TRUTH[k]);
        if (!e.length) return null;
        const a = e.map(Math.abs).sort((x, y) => x - y), q = (f) => a[Math.floor(f * (a.length - 1))];
        const mean = e.reduce((x, y) => x + y, 0) / e.length;
        return { n: e.length, mean, rms: Math.sqrt(e.reduce((x, y) => x + y * y, 0) / e.length), p50: q(0.5), p95: q(0.95) };
    };
    const inBox = (k) => {
        if (!BOX) return true;
        const i = k % W, j = (k - i) / W, e = AREA[0] + (i + 0.5) * RES, n = AREA[3] - (j + 0.5) * RES;
        return e >= BOX[0] && e <= BOX[2] && n >= BOX[1] && n <= BOX[3];
    };
    let boxPx = 0, drawnPx = 0;
    for (let k = 0; k < Z.length; k++) if (inBox(k)) { boxPx++; if (!Number.isNaN(Z[k])) drawnPx++; }
    const f = (s) => s ? `${s.n * RES * RES >= 1e4 ? (s.n * RES * RES / 1e4).toFixed(2) + ' ha' : (s.n * RES * RES).toFixed(0) + ' m²'} · mean ${s.mean >= 0 ? '+' : ''}${s.mean.toFixed(3)} m · |error| median ${s.p50.toFixed(3)} m, 95 % ${s.p95.toFixed(2)} m · RMS ${s.rms.toFixed(2)} m` : '—';
    console.log(`survey vs scene${BOX ? ' (in the box)' : ''}: drawn over ${(100 * drawnPx / boxPx).toFixed(1)} %`);
    console.log(`  measured cells      ${f(stats(k => inBox(k) && MEAS[k]))}`);
    console.log(`  filled between them ${f(stats(k => inBox(k) && !MEAS[k]))}`);
    console.log(`  all drawn           ${f(stats(inBox))}`);
    // Each object: the highest the survey stands above the bed over it, against its own
    console.log('objects (highest drawn above the bed within 1 m of it / its own height):');
    for (const l of legend) {
        const parts = l.name.startsWith('Pipeline') ? scene.objects.filter(o => o.name.startsWith('Pipeline')) : [scene.objects.find(o => o.name === l.name)];
        let best = -Infinity, meas = 0;
        for (const o of parts) {
            const r = o.sphere[3] + 1;
            for (let n = o.sphere[1] - r; n <= o.sphere[1] + r; n += RES) for (let e = o.sphere[0] - r; e <= o.sphere[0] + r; e += RES) {
                if (scene.height(e, n) - scene.bed(e, n) < 0.05) continue;     // not over the object
                const z = surveyed(e, n);
                if (Number.isNaN(z)) continue;
                best = Math.max(best, z - scene.bed(e, n));
                if (measured(e, n)) meas++;
            }
        }
        console.log(`  ${String(l.k).padStart(2)}  ${l.name.padEnd(30)} ${Number.isFinite(best) ? best.toFixed(2).padStart(5) + ' m' : '   — '} / ${l.proud.toFixed(2)} m · ${(meas * RES * RES).toFixed(1)} m² of it measured`);
    }
}

// ---------- what the passes since the earlier snapshot changed ----------
if (PREV) {
    const inBox = (k) => {
        if (!BOX) return true;
        const i = k % W, j = (k - i) / W, e = AREA[0] + (i + 0.5) * RES, n = AREA[3] - (j + 0.5) * RES;
        return e >= BOX[0] && e <= BOX[2] && n >= BOX[1] && n <= BOX[3];
    };
    const q = (a, f) => a[Math.floor(f * (a.length - 1))];
    const ha = (px) => `${(px * RES * RES / 1e4).toFixed(2)} ha`;
    let box = 0, before = 0, after = 0, measBefore = 0, measAfter = 0;
    const groups = { remeasured: [], filledThenMeasured: [], filledBoth: [] };
    for (let k = 0; k < Z.length; k++) {
        if (!inBox(k)) continue;
        box++;
        const b = !Number.isNaN(PREV[k]), a = !Number.isNaN(Z[k]);
        if (b) before++;
        if (a) after++;
        if (PMEAS[k]) measBefore++;
        if (MEAS[k]) measAfter++;
        if (!a || !b) continue;
        const g = PMEAS[k] && MEAS[k] ? 'remeasured' : MEAS[k] ? 'filledThenMeasured' : 'filledBoth';
        groups[g].push([Z[k] - PREV[k], PREV[k] - TRUTH[k], Z[k] - TRUTH[k]]);
    }
    console.log(`since ${path.basename(AGAINST)}${BOX ? ' (in the box)' : ''}: drawn over ${(100 * before / box).toFixed(1)} % → ${(100 * after / box).toFixed(1)} %, measured cells ${ha(measBefore)} → ${ha(measAfter)}`);
    const label = { remeasured: 'measured, measured again', filledThenMeasured: 'filled in, now measured', filledBoth: 'filled in, still filled in' };
    for (const [g, v] of Object.entries(groups)) {
        if (!v.length) continue;
        const ch = v.map(x => Math.abs(x[0])).sort((a, b) => a - b);
        const eb = v.map(x => Math.abs(x[1])).sort((a, b) => a - b), ea = v.map(x => Math.abs(x[2])).sort((a, b) => a - b);
        const rms = (i) => Math.sqrt(v.reduce((s, x) => s + x[i] * x[i], 0) / v.length);
        console.log(`  ${label[g].padEnd(28)} ${ha(v.length)} · change median ${q(ch, 0.5).toFixed(3)} m, 95 % ${q(ch, 0.95).toFixed(2)} m · `
            + `|error| median ${q(eb, 0.5).toFixed(3)} → ${q(ea, 0.5).toFixed(3)} m, 95 % ${q(eb, 0.95).toFixed(2)} → ${q(ea, 0.95).toFixed(2)} m, RMS ${rms(1).toFixed(2)} → ${rms(2).toFixed(2)} m`);
    }
}
}

main();
