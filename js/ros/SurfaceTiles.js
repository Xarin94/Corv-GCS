/**
 * SurfaceTiles.js - Averaged height cells behind the ROS surface mesh
 *
 * Points from a ROS sensor (terrain under an aircraft, the sea bed under a
 * boat or a ROV) are not kept: each one only moves the mean height of the cell
 * it falls in. Cells are small and fixed (CELL, 30 cm by default), so a rock,
 * a step or a wreck survives the averaging; they are grouped in tiles of
 * 30 × 30 cells (9 × 9 m at 30 cm), created only where data arrives, so the
 * surface covers whatever is surveyed without a grid over the empty rest.
 * A tile is the unit of the mesh: at most 29 × 29 quads, 1 682 triangles,
 * and the unit sent to the renderer when something in it changes.
 *
 * Averaging. Every message gives each cell it touches one sample: the mean
 * height of its points in that cell, weighted by how many there are (1 point
 * = 1/3 of a sample, 10 points ≈ 5/6), so a cell grazed by one return moves
 * less than a cell the sensor saw well. A cell keeps a weighted running mean
 * of at most MEMORY samples: a cumulative mean while it fills, a moving
 * average after that, so a surface that changes (tide, a moved object, a bad
 * first pass) is followed instead of being frozen by its history.
 *
 * Budget. Past MAX TILES the tiles farthest from the vehicle are dropped
 * (2 048 tiles ≈ 16 ha at 30 cm, ~16 MB here).
 *
 * Coordinates: metres east (x), north (y), up (heights) from the caller's
 * anchor. Cell (i, j) of tile (tx, ty) spans x from (tx·30 + i)·CELL.
 */

export const TILE = 30;                // cells per tile side
const BIAS = 32768;                    // tile coordinates ±32 768 tiles (±295 km at 30 cm)

export function tileKey(tx, ty) { return (tx + BIAS) * 65536 + (ty + BIAS); }

function inPolygon(x, y, poly) {
    let inside = false;
    for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
        const [xi, yi] = poly[i], [xj, yj] = poly[j];
        if ((yi > y) !== (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi) inside = !inside;
    }
    return inside;
}

export function polygonAreaXY(poly) {
    let a = 0;
    for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) a += poly[j][0] * poly[i][1] - poly[i][0] * poly[j][1];
    return Math.abs(a) / 2;
}

/**
 * Area of a polygon (x east, y north, metres from the anchor) the surface
 * covers: filled cells whose centre is inside, m². A tile wholly inside counts
 * its filled cells at once; only the tiles on the edge are looked at cell by cell.
 * With `block` > 1 the area is counted in blocks of block × block cells — the
 * holes the mesh fills (SurfaceRaster.fillLevelsFor) — a block covered when
 * any of its cells has data and its centre is inside: a sector sonar's sweeps
 * metres apart cover the area they are drawn over.
 */
export function coveredArea(surface, poly, block = 1) {
    const c = surface.cell, span = TILE * c, cell2 = c * c;
    let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
    for (const [x, y] of poly) { x0 = Math.min(x0, x); y0 = Math.min(y0, y); x1 = Math.max(x1, x); y1 = Math.max(y1, y); }
    if (block > 1) {
        const seen = new Set(), bs = block * c;
        let n = 0;
        for (const t of surface.tiles.values()) {
            if (!t.filled) continue;
            const tx0 = t.tx * span, ty0 = t.ty * span;
            if (tx0 + span < x0 - bs || tx0 > x1 + bs || ty0 + span < y0 - bs || ty0 > y1 + bs) continue;
            for (let k = 0; k < TILE * TILE; k++) {
                if (!(t.w[k] > 0)) continue;
                const i = k % TILE, j = (k - i) / TILE;
                const bi = Math.floor((t.tx * TILE + i) / block), bj = Math.floor((t.ty * TILE + j) / block);
                const key = bi * 1048576 + bj;
                if (seen.has(key)) continue;
                seen.add(key);
                if (inPolygon((bi + 0.5) * bs, (bj + 0.5) * bs, poly)) n++;
            }
        }
        return n * bs * bs;
    }
    let covered = 0;
    for (const t of surface.tiles.values()) {
        if (!t.filled) continue;
        const tx0 = t.tx * span, ty0 = t.ty * span, tx1 = tx0 + span, ty1 = ty0 + span;
        if (tx1 < x0 || tx0 > x1 || ty1 < y0 || ty0 > y1) continue;
        const corners = inPolygon(tx0, ty0, poly) && inPolygon(tx1, ty0, poly) && inPolygon(tx1, ty1, poly) && inPolygon(tx0, ty1, poly);
        const vertexInside = poly.some(([x, y]) => x > tx0 && x < tx1 && y > ty0 && y < ty1);
        if (corners && !vertexInside) { covered += t.filled * cell2; continue; }
        for (let k = 0; k < TILE * TILE; k++) {
            if (!(t.w[k] > 0)) continue;
            const i = k % TILE, j = (k - i) / TILE;
            if (inPolygon(tx0 + (i + 0.5) * c, ty0 + (j + 0.5) * c, poly)) covered += cell2;
        }
    }
    return covered;
}

export class SurfaceTiles {
    /**
     * @param {object} [opts]
     * @param {number} [opts.cell=0.3]       cell size, m
     * @param {number} [opts.memory=20]      samples a cell averages over
     * @param {number} [opts.maxTiles=2048]  tiles kept
     */
    constructor(opts = {}) {
        this.cell = 0.3;
        this.memory = 20;
        this.maxTiles = 2048;
        this.version = 0;
        this.configure(opts);
        this.reset();
        // Per-message aggregation, reused
        this._slots = new Map();
        this._sum = new Float64Array(1024);
        this._cnt = new Uint32Array(1024);
        this._tile = [];
        this._idx = new Int32Array(1024);
    }

    /** Apply settings; a new cell size clears the surface. Returns true if it did. */
    configure({ cell, memory, maxTiles } = {}) {
        let relayout = false;
        if (Number.isFinite(cell) && cell > 0 && cell !== this.cell) { this.cell = cell; relayout = true; }
        if (Number.isFinite(memory) && memory >= 1) this.memory = memory;
        if (Number.isFinite(maxTiles) && maxTiles >= 1) this.maxTiles = Math.round(maxTiles);
        if (relayout) this.reset();
        return relayout;
    }

    reset() {
        this.tiles = new Map();
        this.dirty = new Set();
        this.removed = [];
        this.filled = 0;
        this.version++;
    }

    /**
     * Average one message's points.
     * @param {Float64Array|Float32Array} enu  x, y, up triples
     * @param {number} count                   points in use
     * @param {number} [vx] [vy]               vehicle position, for the budget
     * @returns {number} points used
     */
    add(enu, count, vx = 0, vy = 0) {
        const c = this.cell, slots = this._slots;
        if (this._sum.length < count) {
            this._sum = new Float64Array(count);
            this._cnt = new Uint32Array(count);
            this._idx = new Int32Array(count);
        }
        const sum = this._sum, cnt = this._cnt, idxs = this._idx, tiles = this._tile;
        let ns = 0, used = 0;
        slots.clear();
        tiles.length = 0;
        for (let k = 0; k < count; k++) {
            const u = enu[k * 3 + 2];
            if (!Number.isFinite(u)) continue;
            const gi = Math.floor(enu[k * 3] / c), gj = Math.floor(enu[k * 3 + 1] / c);
            const tx = Math.floor(gi / TILE), ty = Math.floor(gj / TILE);
            const idx = (gj - ty * TILE) * TILE + (gi - tx * TILE);
            const key = tileKey(tx, ty) * 1024 + idx;
            let s = slots.get(key);
            if (s === undefined) {
                s = ns++;
                slots.set(key, s);
                sum[s] = 0; cnt[s] = 0; idxs[s] = idx;
                tiles[s] = this._tileAt(tx, ty);
            }
            sum[s] += u;
            cnt[s]++;
            used++;
        }
        const mem = this.memory;
        for (let s = 0; s < ns; s++) {
            const t = tiles[s], i = idxs[s], n = cnt[s];
            const wObs = n / (n + 2);
            const W = t.w[i] + wObs;
            if (t.w[i] === 0) { t.h[i] = 0; t.filled++; this.filled++; }
            t.h[i] += (sum[s] / n - t.h[i]) * wObs / W;
            t.w[i] = Math.min(W, mem);
            this.dirty.add(t.key);
        }
        if (ns) this.version++;
        if (this.tiles.size > this.maxTiles) this._evict(vx, vy);
        return used;
    }

    /** Tiles changed since the last call (heights, NaN where empty; samples) and tiles dropped. */
    takeChanges() {
        const tiles = [];
        for (const key of this.dirty) {
            const t = this.tiles.get(key);
            if (!t) continue;
            const h = new Float32Array(TILE * TILE);
            for (let k = 0; k < h.length; k++) h[k] = t.w[k] > 0 ? t.h[k] : NaN;
            tiles.push({ tx: t.tx, ty: t.ty, h, w: Float32Array.from(t.w) });
        }
        const removed = this.removed;
        this.dirty.clear();
        this.removed = [];
        return { tiles, removed };
    }

    /** Every tile, for a renderer that starts over. */
    takeAll() {
        for (const key of this.tiles.keys()) this.dirty.add(key);
        this.removed = [];
        return this.takeChanges();
    }

    get size() { return this.tiles.size; }

    _tileAt(tx, ty) {
        const key = tileKey(tx, ty);
        let t = this.tiles.get(key);
        if (!t) {
            t = { key, tx, ty, h: new Float64Array(TILE * TILE), w: new Float32Array(TILE * TILE), filled: 0 };
            this.tiles.set(key, t);
        }
        return t;
    }

    // Over budget: drop the tiles farthest from the vehicle, 5 % below the
    // budget so this does not run on every message
    _evict(vx, vy) {
        const span = TILE * this.cell;
        const list = [...this.tiles.values()].map(t => [t, ((t.tx + 0.5) * span - vx) ** 2 + ((t.ty + 0.5) * span - vy) ** 2]);
        list.sort((a, b) => a[1] - b[1]);
        const keep = Math.floor(this.maxTiles * 0.95);
        for (let k = keep; k < list.length; k++) {
            const t = list[k][0];
            this.tiles.delete(t.key);
            this.dirty.delete(t.key);
            this.filled -= t.filled;
            this.removed.push([t.tx, t.ty]);
        }
    }
}
