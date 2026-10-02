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
