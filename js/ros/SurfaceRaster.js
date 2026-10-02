/**
 * SurfaceRaster.js - From averaged cells to the heights and triangles drawn
 *
 * Pure functions (no three.js, no DOM) used by RosMesh3D on each block, and by
 * the offline tests. A grid is w × h cells, row-major: `data` the measured
 * height of each cell (EMPTY where none), `wts` its samples.
 *
 * Adaptive resolution (rasterize). The cells are also averaged into levels of
 * 2, 4, 8, 16 cells (pull, weighted by samples), and each cell is drawn at the
 * finest level with MIN SAMPLES (push, bilinear): its own height where the
 * sensor saw it well, a wider average where it did not, so one stray return is
 * never a vertex on its own.
 *
 * Discontinuities. A height field of a cave or of a terrain with a cliff has
 * jumps: the lake bed at −2 m next to the bottom of a shaft at −11.5 m, a near
 * wall next to a far one. Averaging or interpolating across a jump draws a
 * surface where there is only water or air — a funnel instead of a shaft. So:
 *   - a coarse cell whose cells span more than JUMP (max(0.4 m, 1.5 × its
 *     size): steeper than 56°) is not an average of anything;
 *   - a cell with data takes a coarser level only if that level agrees with
 *     its own height within JUMP;
 *   - an empty cell is filled only by interpolation — the four coarse cells
 *     around it valid and agreeing, at most FILL_LEVELS (1.2 m at 30 cm) —
 *     never extended past the edge of the data;
 *   - a triangle whose corners span more than JUMP over its size is not drawn.
 * `jumps: false` turns these off (the behaviour before them; tests compare).
 */

export const EMPTY = -1e9;
export const LEVELS = 4;              // coarsest: 2^4 = 16 cells
const FILL_LEVELS = 2;                // empty cells: interpolated from 2 or 4-cell levels only
const JUMP_MIN = 0.4;                 // m
const JUMP_SLOPE = 1.5;               // tan 56°

export function jumpTolerance(spanCells, cell) {
    return Math.max(JUMP_MIN, JUMP_SLOPE * spanCells * cell);
}

// Scratch levels per grid size: sum, weight, min, max of the cells below
const levelCache = new Map();
function levelsFor(w, h) {
    const key = w * 65536 + h;
    let L = levelCache.get(key);
    if (!L) {
        L = [];
        for (let k = 1, lw = w, lh = h; k <= LEVELS; k++) {
            lw = Math.ceil(lw / 2); lh = Math.ceil(lh / 2);
            const n = lw * lh;
            L.push({ w: lw, h: lh, sum: new Float64Array(n), wt: new Float64Array(n), lo: new Float32Array(n), hi: new Float32Array(n), ok: new Uint8Array(n) });
        }
        levelCache.set(key, L);
    }
    return L;
}

/**
 * Heights to draw: out[2p] = height (EMPTY: none), out[2p + 1] = level used.
 * @param {Float32Array} data  measured heights (EMPTY where none)
 * @param {Float32Array} wts   samples per cell
 * @param {number} w           cells per row
 * @param {number} h           rows
 * @param {Float32Array} out   2 × w × h
 * @param {{minSamples?: number, cell?: number, jumps?: boolean}} [opts]
 * @returns {Uint32Array} cells drawn per level
 */
export function rasterize(data, wts, w, h, out, opts = {}) {
    const minW = opts.minSamples ?? 1, cell = opts.cell ?? 0.3, guard = opts.jumps !== false;
    const levels = levelsFor(w, h);
    // Pull
    let fine = null, fw = w, fh = h;
    for (let k = 0; k < LEVELS; k++) {
        const L = levels[k], tol = jumpTolerance(2 << k, cell);
        L.sum.fill(0); L.wt.fill(0); L.lo.fill(Infinity); L.hi.fill(-Infinity);
        for (let j = 0; j < fh; j++) {
            for (let i = 0; i < fw; i++) {
                const p = j * fw + i;
                let s, wt, lo, hi;
                if (fine) { wt = fine.wt[p]; if (!wt) continue; s = fine.sum[p]; lo = fine.lo[p]; hi = fine.hi[p]; }
                else { wt = wts[p]; if (!wt) continue; const v = data[p]; s = v * wt; lo = hi = v; }
                const q = (j >> 1) * L.w + (i >> 1);
                L.sum[q] += s; L.wt[q] += wt;
                if (lo < L.lo[q]) L.lo[q] = lo;
                if (hi > L.hi[q]) L.hi[q] = hi;
            }
        }
        for (let q = 0; q < L.ok.length; q++) L.ok[q] = L.wt[q] > 0 && (!guard || L.hi[q] - L.lo[q] <= tol) ? 1 : 0;
        fine = L; fw = L.w; fh = L.h;
    }
    // Push
    const count = new Uint32Array(LEVELS + 1);
    for (let j = 0; j < h; j++) {
        for (let i = 0; i < w; i++) {
            const p = j * w + i, own = wts[p] > 0, v = data[p];
            out[p * 2] = EMPTY; out[p * 2 + 1] = 0;
            if (own && wts[p] >= minW) { out[p * 2] = v; count[0]++; continue; }
            const top = own || !guard ? LEVELS : FILL_LEVELS;
            for (let k = 0; k < top; k++) {
                const L = levels[k], f = 2 << k, tol = jumpTolerance(f, cell);
                const u = (i + 0.5) / f - 0.5, t = (j + 0.5) / f - 0.5;
                const i0 = Math.floor(u), j0 = Math.floor(t), fu = u - i0, ft = t - j0;
                let s = 0, sw = 0, corners = 0, lo = Infinity, hi = -Infinity;
                for (let dj = 0; dj < 2; dj++) {
                    const jj = j0 + dj;
                    if (jj < 0 || jj >= L.h) continue;
                    for (let di = 0; di < 2; di++) {
                        const ii = i0 + di;
                        if (ii < 0 || ii >= L.w) continue;
                        const q = jj * L.w + ii;
                        if (!(guard ? L.ok[q] : L.wt[q] > 0)) continue;
                        const bw = (di ? fu : 1 - fu) * (dj ? ft : 1 - ft);
                        const m = L.sum[q] / L.wt[q];
                        s += bw * L.sum[q];
                        sw += bw * L.wt[q];
                        corners++;
                        if (m < lo) lo = m;
                        if (m > hi) hi = m;
                    }
                }
                if (sw < minW) continue;
                const val = s / sw;
                if (guard) {
                    if (!own && corners < 4) continue;          // interpolate holes, never extend edges
                    if (hi - lo > jumpTolerance(2 * f, cell)) continue;
                    if (own && Math.abs(val - v) > tol) continue;
                }
                out[p * 2] = val; out[p * 2 + 1] = k + 1; count[k + 1]++;
                break;
            }
            // Under-sampled, and no coarser level agrees with it: its own height, still real
            if (own && out[p * 2] === EMPTY && guard) { out[p * 2] = v; count[0]++; }
        }
    }
    return count;
}

/**
 * Triangles over a w × h raster at vertex spacing s (cells): two per quad with
 * four corners drawn, one where three are; vertex ids are row-major on the
 * (w − 1) / s + 1 grid. With jumps on, a triangle spanning more than JUMP over
 * its size is left out.
 * @returns {number} indices written to out
 */
export function triangulate(view, w, h, s, out, opts = {}) {
    const cell = opts.cell ?? 0.3, guard = opts.jumps !== false;
    const W = Math.floor((w - 1) / s) + 1, Hh = Math.floor((h - 1) / s) + 1;
    const tol = jumpTolerance(s * Math.SQRT2, cell);
    const z = (vi, vj) => view[(vj * s * w + vi * s) * 2];
    const ok3 = (x, y, q) => !guard || Math.max(x, y, q) - Math.min(x, y, q) <= tol;
    let n = 0;
    for (let vj = 0; vj < Hh - 1; vj++) {
        for (let vi = 0; vi < W - 1; vi++) {
            const za = z(vi, vj), zb = z(vi + 1, vj), zc = z(vi, vj + 1), zd = z(vi + 1, vj + 1);
            const ha = za > EMPTY, hb = zb > EMPTY, hc = zc > EMPTY, hd = zd > EMPTY;
            if (ha + hb + hc + hd < 3) continue;
            const a = vj * W + vi, b = a + 1, c = a + W, d = c + 1;
            if (ha && hb && hd && ok3(za, zb, zd)) { out[n++] = a; out[n++] = b; out[n++] = d; }
            if (ha && hd && hc && ok3(za, zd, zc)) { out[n++] = a; out[n++] = d; out[n++] = c; }
            if (!hd && ha && hb && hc && ok3(za, zb, zc)) { out[n++] = a; out[n++] = b; out[n++] = c; }
            if (!ha && hb && hd && hc && ok3(zb, zd, zc)) { out[n++] = b; out[n++] = d; out[n++] = c; }
        }
    }
    return n;
}
