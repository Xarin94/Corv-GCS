/** Persistent point buffers owned by the domain, shared with a rendering adapter. */
export class PointCloudData {
    constructor({ chunkSize = 262144, liveCapacity = 262144, maxPoints = 3000000 } = {}) {
        this.chunkSize = chunkSize; this.maxPoints = maxPoints;
        this.chunks = []; this.total = 0; this.origin = null; this.generation = 0; this.revision = 0;
        this.visible = false; this.colorMode = 'height'; this.pointSize = 2; this.overTerrain = false;
        this.mapTransform = { position: [0, 0, 0], scale: [1, 1, 1] };
        this.liveTransform = { position: [0, 0, 0], quaternion: [0, 0, 0, 1] };
        this.histogram = new Int32Array(4000); this.histogramDirty = false;
        this.heightRange = [-10, 50]; this.lastRange = 0;
        this.live = { capacity: liveCapacity, positions: new Float32Array(liveCapacity * 3),
            intensity: new Uint8Array(liveCapacity), birth: new Float32Array(liveCapacity),
            head: 0, count: 0, frame: 'ned', ttl: 3, revision: 0 };
    }
    setOrigin(origin) {
        if (!origin) return;
        if (this.origin && this.origin.epoch !== origin.epoch) this.clear();
        this.origin = { ...origin }; this.revision++;
    }
    appendMapped({ enu, intensity, epoch }) {
        const updates = [];
        if (!this.origin || !enu || (epoch !== undefined && epoch !== this.origin.epoch)) return updates;
        const count = Math.floor(enu.length / 3);
        let k = 0;
        while (k < count && this.total < this.maxPoints) {
            let block = this.chunks[this.chunks.length - 1];
            if (!block || block.count >= this.chunkSize) {
                block = { id: this.chunks.length, count: 0, revision: 0,
                    positions: new Float32Array(this.chunkSize * 3), intensity: new Uint8Array(this.chunkSize) };
                this.chunks.push(block);
            }
            const start = block.count, room = Math.min(this.chunkSize - start, count - k, this.maxPoints - this.total);
            for (let i = 0; i < room; i++) {
                const src = (k + i) * 3, dst = (start + i) * 3;
                const up = enu[src + 2];
                block.positions[dst] = enu[src]; block.positions[dst + 1] = up; block.positions[dst + 2] = -enu[src + 1];
                block.intensity[start + i] = intensity ? intensity[k + i] : 128;
                const bin = Math.floor(up + 1000);
                if (bin >= 0 && bin < this.histogram.length) this.histogram[bin]++;
            }
            block.count += room; block.revision++;
            this.total += room; k += room;
            updates.push({ block, start, count: room });
        }
        if (updates.length) { this.histogramDirty = true; this.revision++; }
        return updates;
    }
    appendLive({ xyz, intensity, frame, ttl }, nowSeconds) {
        const updates = [], live = this.live;
        if (!xyz) return updates;
        if (frame) live.frame = frame;
        if (ttl) live.ttl = ttl;
        const count = Math.floor(xyz.length / 3);
        let k = 0;
        while (k < count) {
            const start = live.head, room = Math.min(live.capacity - start, count - k);
            for (let i = 0; i < room; i++) {
                const src = (k + i) * 3, dst = (start + i) * 3;
                live.positions[dst] = xyz[src + 1]; live.positions[dst + 1] = -xyz[src + 2]; live.positions[dst + 2] = -xyz[src];
                live.intensity[start + i] = intensity ? intensity[k + i] : 128;
                live.birth[start + i] = nowSeconds;
            }
            updates.push({ start, count: room });
            live.head = (live.head + room) % live.capacity;
            live.count = Math.min(live.capacity, live.count + room); k += room;
        }
        if (updates.length) { live.revision++; this.revision++; }
        return updates;
    }
    clearLive() { this.live.head = 0; this.live.count = 0; this.live.revision++; this.revision++; }
    clear() {
        this.chunks = []; this.total = 0; this.histogram.fill(0); this.histogramDirty = true;
        this.generation++; this.revision++;
    }
    updateRange(nowMs) {
        if (!this.histogramDirty || nowMs - this.lastRange <= 500) return;
        this.histogramDirty = false; this.lastRange = nowMs;
        const lo = this.percentile(0.02), hi = this.percentile(0.98);
        if (hi > lo) { this.heightRange[0] = lo; this.heightRange[1] = hi; }
    }
    percentile(q) {
        if (!this.total) return 0;
        const target = q * this.total;
        let sum = 0;
        for (let i = 0; i < this.histogram.length; i++) {
            sum += this.histogram[i];
            if (sum >= target) return i - 1000;
        }
        return 3000;
    }
}

/** Same YXZ convention as the aircraft and the former Three Euler. Output XYZW. */
export function setYXZQuaternion(target, x, y, z) {
    const c1 = Math.cos(x / 2), c2 = Math.cos(y / 2), c3 = Math.cos(z / 2);
    const s1 = Math.sin(x / 2), s2 = Math.sin(y / 2), s3 = Math.sin(z / 2);
    target[0] = s1 * c2 * c3 + c1 * s2 * s3;
    target[1] = c1 * s2 * c3 - s1 * c2 * s3;
    target[2] = c1 * c2 * s3 - s1 * s2 * c3;
    target[3] = c1 * c2 * c3 + s1 * s2 * s3;
}
