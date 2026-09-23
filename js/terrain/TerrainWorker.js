/**
 * TerrainWorker.js - Off-thread terrain chunk data preparation
 *
 * Extracts a chunk's elevation samples straight from the HGT grid. Positions
 * and normals are rebuilt from them in the terrain vertex shader (see the GPU
 * TERRAIN section of TerrainManager.js).
 */

const hgtBuffers = new Map();

function buildChunk(data) {
    const { chunkKey, hgtKey, size, vertsPerChunk, cx, cy } = data;
    const entry = hgtBuffers.get(hgtKey);
    if (!entry || !entry.buffer) {
        return { type: 'chunkFailed', chunkKey, reason: 'missing-hgt' };
    }

    // LOD decimation: sample every `step` HGT cells (1 = full res).
    const step = (data.step && vertsPerChunk % data.step === 0) ? data.step : 1;

    const dataView = new DataView(entry.buffer);
    const geoW = vertsPerChunk / step + 1;
    const vertCount = geoW * geoW;

    const heights = new Int16Array(vertCount);
    let minH = Infinity, maxH = -Infinity;

    const startRow = cy * vertsPerChunk;
    const startCol = cx * vertsPerChunk;

    // A chunk grid that disagrees with the caller's puts these past the tile and
    // would read samples of other rows. Fail loudly instead.
    if (startRow + vertsPerChunk > size - 1 || startCol + vertsPerChunk > size - 1) {
        return { type: 'chunkFailed', chunkKey, reason: 'chunk-out-of-tile' };
    }

    let i = 0;
    for (let r = 0; r < geoW; r++) {
        const row = (startRow + r * step) * size;
        for (let col = 0; col < geoW; col++) {
            const h = dataView.getInt16((row + startCol + col * step) * 2, false);
            heights[i++] = h;
            if (h < minH) minH = h;
            if (h > maxH) maxH = h;
        }
    }

    return {
        type: 'chunkBuilt',
        chunkKey,
        step,
        heights,
        minH,
        maxH
    };
}

self.onmessage = (e) => {
    const data = e.data || {};

    if (data.type === 'registerHgt') {
        hgtBuffers.set(data.key, { buffer: data.buffer, size: data.size });
        return;
    }

    if (data.type === 'buildChunk') {
        const result = buildChunk(data);
        if (result.type === 'chunkBuilt') {
            self.postMessage(result, [result.heights.buffer]);
        } else {
            self.postMessage(result);
        }
    }
};
