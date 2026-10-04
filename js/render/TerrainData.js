export const TERRAIN_GRID = Object.freeze({ cellM: 25, radiusM: 500, fullAgl: 150, maxAgl: 200 });

/** Authoritative heightfield records. No mesh, material or GPU handle belongs here. */
export class TerrainData {
    constructor(origin) { this.origin = { ...origin }; this.chunks = new Map(); this.revision = 0; }
    put(item, step, heights, minH, maxH) {
        const { cx, cy, latBase, lonBase, size, vertsPerChunk, chunkKey } = item;
        const width = vertsPerChunk / step + 1;
        if (!Number.isInteger(width) || width < 2 || !(heights instanceof Int16Array) || heights.length !== width * width) {
            throw new Error('Invalid terrain heightfield');
        }
        const latTop = latBase + 1 - cy * vertsPerChunk / (size - 1);
        const lonLeft = lonBase + cx * vertsPerChunk / (size - 1);
        const latBottom = latTop - vertsPerChunk / (size - 1);
        const lonRight = lonLeft + vertsPerChunk / (size - 1);
        const metersPerLon = 111320 * Math.cos(this.origin.lat * Math.PI / 180);
        const x = (lonLeft - this.origin.lon) * metersPerLon;
        const z = -(latTop - this.origin.lat) * 111320;
        // Preserve the existing coordinate conversion's rounding at tile seams.
        const dx = (lonLeft + step / (size - 1) - this.origin.lon) * metersPerLon - x;
        const dz = -(latTop - step / (size - 1) - this.origin.lat) * 111320 - z;
        const halfW = (width - 1) * dx / 2, halfD = (width - 1) * dz / 2, halfH = (maxH - minH) / 2;
        const previous = this.chunks.get(chunkKey);
        const record = {
            id: chunkKey, revision: ++this.revision, width, step, heights,
            source: { cx, cy, latBase, lonBase, size, vertsPerChunk },
            bounds: [latTop, latBottom, lonLeft, lonRight], grid: [x, z, dx, dz], minH, maxH,
            sphere: { x: x + halfW, y: (minH + maxH) / 2, z: z + halfD, r: Math.hypot(halfW, halfD, halfH) },
            appearance: previous?.appearance || null
        };
        this.chunks.set(chunkKey, record);
        return record;
    }
    setAppearance(id, appearance) {
        const record = this.chunks.get(id);
        if (record) { record.appearance = appearance; record.revision = ++this.revision; }
    }
    remove(id) { if (this.chunks.delete(id)) this.revision++; }
    getStats() {
        let bytes = 0;
        for (const c of this.chunks.values()) bytes += c.heights.byteLength;
        return { chunks: this.chunks.size, heightBytes: bytes, revision: this.revision };
    }
}
