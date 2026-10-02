/**
 * SurfaceVolume.js - 3D surface of a cave from sonar / LiDAR returns (TSDF)
 *
 * A height field holds one surface per column: right for terrain and sea bed,
 * wrong in a cave, a shaft or under an overhang, where the floor, the walls
 * and the ceiling share columns. Here the space is cut into voxels (CELL, 30 cm
 * by default) holding a truncated signed distance to the surface, averaged
 * over the returns — positive on the open side, negative behind the rock — and
 * the mesh is where it changes sign. Every wall comes out with its own
 * orientation, nothing has to be told which way it faces.
 *
 * Integration (KinectFusion's TSDF, as cave and ship-hull sonar mappers use
 * it). For a return at range r from the sensor, the voxels the ray crosses
 * from r − TRUNC to r + TRUNC get the sample clamp((r − t) / TRUNC, −1, 1),
 * t their distance along the ray; each voxel keeps a weighted running mean of
 * at most MEMORY samples, like the 2D cells (SurfaceTiles.js). Free water far
 * in front of the surface is not carved: only the band around it matters.
 *
 * Meshing (surface nets). In each cube of 8 neighbouring voxels, all of them
 * observed, whose signs differ, one vertex at the mean of its edges' zero
 * crossings; each voxel edge that changes sign joins the four cubes around it
 * into a quad, two triangles. Voxels are grouped in chunks of 16³ (4.8 m at
 * 30 cm); a chunk is meshed again when it or a neighbour it borders changes,
 * and only those meshes travel to the renderer.
 *
 * Coordinates: metres east, north, up from the caller's anchor.
 */

export const CHUNK = 16;
const N3 = CHUNK * CHUNK * CHUNK;
const BIAS = 32768;
const TRUNC_CELLS = 3;                 // truncation band: ± 3 voxels

export function chunkKey(cx, cy, cz) { return ((cx + BIAS) * 65536 + (cy + BIAS)) * 65536 + (cz + BIAS); }

export class SurfaceVolume {
    /**
     * @param {object} [opts]
     * @param {number} [opts.cell=0.3]        voxel size, m
     * @param {number} [opts.memory=20]       samples a voxel averages over (≤ 255)
     * @param {number} [opts.maxChunks=1024]  chunks kept (16³ voxels each, 20 KB)
     */
    constructor(opts = {}) {
        this.cell = 0.3;
        this.memory = 20;
        this.maxChunks = 1024;
        this.configure(opts);
        this.reset();
    }

    configure({ cell, memory, maxChunks } = {}) {
        let relayout = false;
        if (Number.isFinite(cell) && cell > 0 && cell !== this.cell) { this.cell = cell; relayout = true; }
        if (Number.isFinite(memory) && memory >= 1) this.memory = Math.min(255, Math.round(memory));
        if (Number.isFinite(maxChunks) && maxChunks >= 8) this.maxChunks = Math.round(maxChunks);
        if (relayout) this.reset();
        return relayout;
    }

    reset() {
        this.chunks = new Map();          // key → { cx, cy, cz, d: Float32Array, w: Float32Array }
        this.dirty = new Set();
        this.removed = [];
    }

    get size() { return this.chunks.size; }

    _chunk(cx, cy, cz, create) {
        const key = chunkKey(cx, cy, cz);
        let c = this.chunks.get(key);
        if (!c && create) {
            c = { key, cx, cy, cz, d: new Float32Array(N3), w: new Uint8Array(N3) };
            this.chunks.set(key, c);
        }
        return c;
    }

    // Distance and weight of voxel (i, j, k), null if never observed
    _voxel(i, j, k) {
        const cx = Math.floor(i / CHUNK), cy = Math.floor(j / CHUNK), cz = Math.floor(k / CHUNK);
        const c = this.chunks.get(chunkKey(cx, cy, cz));
        if (!c) return null;
        const p = ((k - cz * CHUNK) * CHUNK + (j - cy * CHUNK)) * CHUNK + (i - cx * CHUNK);
        return c.w[p] > 0 ? c.d[p] : null;
    }

    /**
     * Fuse the returns of one message.
     * @param {number[]} o                  sensor position (e, n, u)
     * @param {Float64Array|Float32Array} enu  returns, x y z triples
     * @param {number} count
     * @returns {number} returns used
     */
    integrate(o, enu, count) {
        const v = this.cell, mu = TRUNC_CELLS * v, step = v / 2, mem = this.memory;
        let used = 0;
        for (let q = 0; q < count; q++) {
            const dx = enu[q * 3] - o[0], dy = enu[q * 3 + 1] - o[1], dz = enu[q * 3 + 2] - o[2];
            const r = Math.hypot(dx, dy, dz);
            if (!(r > 1e-3)) continue;
            const ux = dx / r, uy = dy / r, uz = dz / r;
            let lastKey = -1, lastP = -1;
            for (let t = Math.max(0, r - mu); t <= r + mu; t += step) {
                const i = Math.floor((o[0] + ux * t) / v), j = Math.floor((o[1] + uy * t) / v), k = Math.floor((o[2] + uz * t) / v);
                const cx = Math.floor(i / CHUNK), cy = Math.floor(j / CHUNK), cz = Math.floor(k / CHUNK);
                const c = this._chunk(cx, cy, cz, true);
                const li = i - cx * CHUNK, lj = j - cy * CHUNK, lk = k - cz * CHUNK;
                const p = (lk * CHUNK + lj) * CHUNK + li;
                if (c.key === lastKey && p === lastP) continue;
                lastKey = c.key; lastP = p;
                // Signed distance of the voxel centre along the ray
                const along = ((i + 0.5) * v - o[0]) * ux + ((j + 0.5) * v - o[1]) * uy + ((k + 0.5) * v - o[2]) * uz;
                const sdf = Math.max(-1, Math.min(1, (r - along) / mu));
                const W = c.w[p] + 1;
                c.d[p] += (sdf - c.d[p]) / W;
                c.w[p] = Math.min(W, mem);
                this._touch(c, li, lj, lk);
            }
            used++;
        }
        if (this.chunks.size > this.maxChunks) this._evict(o);
        return used;
    }

    // A voxel on a chunk's face also changes the mesh of the chunk beyond that
    // face: the cubes straddling the two chunks use voxels of both
    _touch(c, li, lj, lk) {
        this.dirty.add(c.key);
        const side = (l) => l === 0 ? [0, -1] : l === CHUNK - 1 ? [0, 1] : [0];
        if (side(li).length + side(lj).length + side(lk).length === 3) return;
        for (const dx of side(li)) for (const dy of side(lj)) for (const dz of side(lk)) {
            if (dx || dy || dz) this.dirty.add(chunkKey(c.cx + dx, c.cy + dy, c.cz + dz));
        }
    }

    _evict(o) {
        const span = CHUNK * this.cell;
        const list = [...this.chunks.values()].map(c => [c, ((c.cx + 0.5) * span - o[0]) ** 2 + ((c.cy + 0.5) * span - o[1]) ** 2 + ((c.cz + 0.5) * span - o[2]) ** 2]);
        list.sort((a, b) => a[1] - b[1]);
        for (let k = Math.floor(this.maxChunks * 0.95); k < list.length; k++) {
            const c = list[k][0];
            this.chunks.delete(c.key);
            this.dirty.delete(c.key);
            this.removed.push(c.key);
        }
    }

    /**
     * Surface nets mesh of one chunk: triangles, x y z per vertex (metres from
     * the anchor), three vertices a triangle, not indexed.
     */
    meshChunk(c) {
        const v = this.cell, i0 = c.cx * CHUNK, j0 = c.cy * CHUNK, k0 = c.cz * CHUNK;
        const verts = new Map();
        const corner = new Float32Array(8);
        // Vertex of the cube whose lowest voxel is (i, j, k), or null
        const cubeVertex = (i, j, k) => {
            const key = ((i - i0 + 1) * (CHUNK + 2) + (j - j0 + 1)) * (CHUNK + 2) + (k - k0 + 1);
            if (verts.has(key)) return verts.get(key);
            let mask = 0, ok = true;
            for (let n = 0; n < 8; n++) {
                const d = this._voxel(i + (n & 1), j + ((n >> 1) & 1), k + ((n >> 2) & 1));
                if (d === null) { ok = false; break; }
                corner[n] = d;
                if (d < 0) mask |= 1 << n;
            }
            let out = null;
            if (ok && mask !== 0 && mask !== 255) {
                let sx = 0, sy = 0, sz = 0, m = 0;
                for (const [a, b] of EDGES) {
                    const da = corner[a], db = corner[b];
                    if ((da < 0) === (db < 0)) continue;
                    const t = da / (da - db);
                    sx += (a & 1) + t * ((b & 1) - (a & 1));
                    sy += ((a >> 1) & 1) + t * (((b >> 1) & 1) - ((a >> 1) & 1));
                    sz += ((a >> 2) & 1) + t * (((b >> 2) & 1) - ((a >> 2) & 1));
                    m++;
                }
                out = [(i + 0.5 + sx / m) * v, (j + 0.5 + sy / m) * v, (k + 0.5 + sz / m) * v];
            }
            verts.set(key, out);
            return out;
        };
        const tris = [];
        const quad = (a, b, cc, d) => {
            if (!a || !b || !cc || !d) return;
            tris.push(...a, ...b, ...cc, ...a, ...cc, ...d);
        };
        // Every voxel edge of this chunk along +x, +y, +z that changes sign
        for (let k = k0; k < k0 + CHUNK; k++) for (let j = j0; j < j0 + CHUNK; j++) for (let i = i0; i < i0 + CHUNK; i++) {
            const d0 = this._voxel(i, j, k);
            if (d0 === null) continue;
            const dxv = this._voxel(i + 1, j, k), dyv = this._voxel(i, j + 1, k), dzv = this._voxel(i, j, k + 1);
            if (dxv !== null && (d0 < 0) !== (dxv < 0)) quad(cubeVertex(i, j - 1, k - 1), cubeVertex(i, j, k - 1), cubeVertex(i, j, k), cubeVertex(i, j - 1, k));
            if (dyv !== null && (d0 < 0) !== (dyv < 0)) quad(cubeVertex(i - 1, j, k - 1), cubeVertex(i, j, k - 1), cubeVertex(i, j, k), cubeVertex(i - 1, j, k));
            if (dzv !== null && (d0 < 0) !== (dzv < 0)) quad(cubeVertex(i - 1, j - 1, k), cubeVertex(i, j - 1, k), cubeVertex(i, j, k), cubeVertex(i - 1, j, k));
        }
        return Float32Array.from(tris);
    }

    /** Meshes of the chunks changed since the last call, and the chunks dropped. */
    takeChanges() {
        const meshes = [];
        for (const key of this.dirty) {
            const c = this.chunks.get(key);
            if (c) meshes.push({ key, cx: c.cx, cy: c.cy, cz: c.cz, pos: this.meshChunk(c) });
        }
        const removed = this.removed;
        this.dirty.clear();
        this.removed = [];
        return { meshes, removed };
    }

    takeAll() {
        for (const key of this.chunks.keys()) this.dirty.add(key);
        this.removed = [];
        return this.takeChanges();
    }
}

// The 12 edges of a cube, corners numbered n = x + 2y + 4z
const EDGES = [[0, 1], [2, 3], [4, 5], [6, 7], [0, 2], [1, 3], [4, 6], [5, 7], [0, 4], [1, 5], [2, 6], [3, 7]];
