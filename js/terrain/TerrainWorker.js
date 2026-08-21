/**
 * TerrainWorker.js - Off-thread terrain chunk data preparation
 */

import { ORIGIN } from '../core/constants.js';

const hgtBuffers = new Map();

function latLonToMeters(lat, lon) {
    const metersPerLat = 111320;
    const metersPerLon = 111320 * Math.cos(ORIGIN.lat * (Math.PI / 180));
    const z = -(lat - ORIGIN.lat) * metersPerLat;
    const x = (lon - ORIGIN.lon) * metersPerLon;
    return { x, z };
}

function getHeightColor(height) {
    if (height <= -100) return { r: 0, g: 0, b: 0 };

    const G = [0.0431372549, 0.4, 0.137254902];
    const Y = [0.902, 0.7647058824, 0.3529411765];
    const O = [0.902, 0.494, 0.133];
    const R = [0.906, 0.298, 0.235];
    const P = [0.608, 0.349, 0.713];
    const B = [0.204, 0.596, 0.858];

    const lerp = (a, b, t) => a + (b - a) * t;
    const lerpColorArr = (c1, c2, t) => ({
        r: lerp(c1[0], c2[0], t),
        g: lerp(c1[1], c2[1], t),
        b: lerp(c1[2], c2[2], t)
    });

    if (height <= 700) return { r: G[0], g: G[1], b: G[2] };
    if (height <= 1400) return lerpColorArr(G, Y, (height - 700) / (1400 - 700));
    if (height <= 2100) return lerpColorArr(Y, O, (height - 1400) / (2100 - 1400));
    if (height <= 2800) return lerpColorArr(O, R, (height - 2100) / (2800 - 2100));
    if (height <= 3500) return lerpColorArr(R, P, (height - 2800) / (3500 - 2800));
    if (height <= 4000) return lerpColorArr(P, B, (height - 3500) / (4000 - 3500));
    return { r: B[0], g: B[1], b: B[2] };
}

/**
 * Compute smooth vertex normals for a regular grid via central differences.
 * Much faster than face-normal accumulation and runs off the main thread.
 */
function computeGridNormals(positions, geoW) {
    const normals = new Float32Array(positions.length);

    for (let r = 0; r < geoW; r++) {
        for (let c = 0; c < geoW; c++) {
            // East tangent: P(r, c+1) - P(r, c-1) (one-sided at edges)
            const cE = Math.min(c + 1, geoW - 1) ;
            const cW = Math.max(c - 1, 0);
            const iE = (r * geoW + cE) * 3;
            const iW = (r * geoW + cW) * 3;
            const ex = positions[iE] - positions[iW];
            const ey = positions[iE + 1] - positions[iW + 1];
            const ez = positions[iE + 2] - positions[iW + 2];

            // South tangent: P(r+1, c) - P(r-1, c)
            const rS = Math.min(r + 1, geoW - 1);
            const rN = Math.max(r - 1, 0);
            const iS = (rS * geoW + c) * 3;
            const iN = (rN * geoW + c) * 3;
            const sx = positions[iS] - positions[iN];
            const sy = positions[iS + 1] - positions[iN + 1];
            const sz = positions[iS + 2] - positions[iN + 2];

            // n = S × E (points +Y up with this grid orientation)
            let nx = sy * ez - sz * ey;
            let ny = sz * ex - sx * ez;
            let nz = sx * ey - sy * ex;
            const len = Math.sqrt(nx * nx + ny * ny + nz * nz) || 1;
            nx /= len; ny /= len; nz /= len;

            const o = (r * geoW + c) * 3;
            normals[o] = nx;
            normals[o + 1] = ny;
            normals[o + 2] = nz;
        }
    }

    return normals;
}

function buildChunk(data) {
    const { chunkKey, hgtKey, latBase, lonBase, size, vertsPerChunk, cx, cy } = data;
    const entry = hgtBuffers.get(hgtKey);
    if (!entry || !entry.buffer) {
        return { type: 'chunkFailed', chunkKey, reason: 'missing-hgt' };
    }

    // LOD decimation: sample every `step` HGT cells (1 = full res).
    const step = (data.step && vertsPerChunk % data.step === 0) ? data.step : 1;

    const dataView = new DataView(entry.buffer);
    const geoW = vertsPerChunk / step + 1;
    const vertCount = geoW * geoW;

    const positions = new Float32Array(vertCount * 3);
    const uvs = new Float32Array(vertCount * 2);
    const colors = new Float32Array(vertCount * 3);

    const startRow = cy * vertsPerChunk;
    const startCol = cx * vertsPerChunk;

    // A chunk grid that disagrees with the caller's puts these past the tile, and the
    // per-sample clamp below then collapses every vertex onto the tile edge — an
    // invisible chunk rather than an error. Fail loudly instead.
    if (startRow + vertsPerChunk > size - 1 || startCol + vertsPerChunk > size - 1) {
        return { type: 'chunkFailed', chunkKey, reason: 'chunk-out-of-tile' };
    }

    let p = 0;
    let u = 0;
    let c = 0;

    for (let r = 0; r < geoW; r++) {
        for (let col = 0; col < geoW; col++) {
            const hgtRow = Math.min(startRow + r * step, size - 1);
            const hgtCol = Math.min(startCol + col * step, size - 1);
            const height = dataView.getInt16((hgtRow * size + hgtCol) * 2, false);
            const nLat = 1.0 - (hgtRow / (size - 1));
            const nLon = hgtCol / (size - 1);
            const vertLat = latBase + nLat;
            const vertLon = lonBase + nLon;
            const wPos = latLonToMeters(vertLat, vertLon);

            positions[p++] = wPos.x;
            positions[p++] = height;
            positions[p++] = wPos.z;

            const uu = col / (geoW - 1);
            const vv = 1 - (r / (geoW - 1));
            uvs[u++] = uu;
            uvs[u++] = vv;

            const color = getHeightColor(height);
            colors[c++] = color.r;
            colors[c++] = color.g;
            colors[c++] = color.b;
        }
    }

    const normals = computeGridNormals(positions, geoW);

    return {
        type: 'chunkBuilt',
        chunkKey,
        step,
        positions,
        uvs,
        colors,
        normals
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
            self.postMessage(result, [result.positions.buffer, result.uvs.buffer, result.colors.buffer, result.normals.buffer]);
        } else {
            self.postMessage(result);
        }
    }
};
