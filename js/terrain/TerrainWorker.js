/**
 * TerrainWorker.js - Off-thread terrain chunk data preparation
 *
 * Decodes HGT tiles and extracts a chunk's elevation samples straight from the
 * HGT grid. Positions and normals are rebuilt from them in the terrain vertex
 * shader (see the GPU TERRAIN section of TerrainManager.js).
 *
 * One of a pool when SharedArrayBuffer is available: the worker that decodes
 * a tile writes it into shared memory, the UI thread registers that same grid
 * in the other workers (registerHgt), and any of them can build any chunk.
 * Without it there is a single worker, holding its own copy of every grid.
 */

import { decodeHgt, readHgtShared } from './HgtDecoder.js';

const hgtBuffers = new Map();

function buildChunk(data) {
    const { chunkKey, hgtKey, size, vertsPerChunk, cx, cy } = data;
    const entry = hgtBuffers.get(hgtKey);
    if (!entry || !entry.data) {
        return { type: 'chunkFailed', chunkKey, reason: 'missing-hgt' };
    }

    // LOD decimation: sample every `step` HGT cells (1 = full res).
    const step = (data.step && vertsPerChunk % data.step === 0) ? data.step : 1;

    const geoW = vertsPerChunk / step + 1;
    const vertCount = geoW * geoW;

    const heights = new Int16Array(vertCount);
    let minH = Infinity, maxH = -Infinity;

    const startRow = cy * vertsPerChunk;
    const startCol = cx * vertsPerChunk;

    // A chunk grid that disagrees with the caller's puts these past the tile and
    // would read samples of other rows. Fail loudly instead.
    if (size !== entry.size || startRow < 0 || startCol < 0 ||
        startRow + vertsPerChunk > size - 1 || startCol + vertsPerChunk > size - 1) {
        return { type: 'chunkFailed', chunkKey, reason: 'chunk-out-of-tile' };
    }

    let i = 0;
    for (let r = 0; r < geoW; r++) {
        const row = (startRow + r * step) * size;
        for (let col = 0; col < geoW; col++) {
            const h = entry.data[row + startCol + col * step];
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

self.onmessage = async (e) => {
    const data = e.data || {};

    if (data.type === 'prepareHgt') {
        try {
            const start = performance.now();
            // Blob/File structured cloning does not copy a 25 MB JS buffer on
            // the UI thread. Read and convert here.
            if (data.shared && typeof SharedArrayBuffer === 'function') {
                // Posting shared memory hands over the grid itself, not a copy
                const tile = await readHgtShared(data.file);
                hgtBuffers.set(data.key, tile);
                self.postMessage({ type: 'hgtReady', id: data.id, key: data.key, size: tile.size,
                    elevations: tile.data, shared: true, prepareMs: performance.now() - start });
                return;
            }
            // Keep the decoded grid; the UI thread gets a copy for elevation queries
            const tile = decodeHgt(await data.file.arrayBuffer());
            hgtBuffers.set(data.key, tile);
            const elevations = tile.data.slice();
            self.postMessage({ type: 'hgtReady', id: data.id, key: data.key, size: tile.size,
                elevations, shared: false, prepareMs: performance.now() - start }, [elevations.buffer]);
        } catch (error) {
            self.postMessage({ type: 'hgtFailed', id: data.id, key: data.key, reason: error.message });
        }
        return;
    }

    if (data.type === 'registerHgt') {
        // A grid another worker of the pool decoded into shared memory. Stored
        // before this worker reads its next message: the chunk jobs the UI
        // thread sends after this one find the tile.
        hgtBuffers.set(data.key, { size: data.size, data: data.elevations });
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
