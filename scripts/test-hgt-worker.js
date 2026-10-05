#!/usr/bin/env node
// Run the production worker protocol on a real Node worker thread, with a
// browser Worker shim. No Electron, network or application data is required.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const { pathToFileURL } = require('node:url');
const { Worker } = require('node:worker_threads');
const tmp = fs.mkdtempSync(path.join(os.tmpdir(), 'corv-hgt-test-'));
const root = path.join(__dirname, '..');
for (const name of ['HgtDecoder', 'TerrainWorker']) {
    const source = fs.readFileSync(path.join(root, 'js/terrain', name + '.js'), 'utf8')
        .replaceAll("'./HgtDecoder.js'", "'./HgtDecoder.mjs'");
    fs.writeFileSync(path.join(tmp, name + '.mjs'), source);
}
fs.writeFileSync(path.join(tmp, 'runner.mjs'), `
import { parentPort } from 'node:worker_threads';
globalThis.self = { postMessage: (data, transfers) => parentPort.postMessage(data, transfers) };
await import('./TerrainWorker.mjs');
parentPort.on('message', data => self.onmessage({ data }));
parentPort.postMessage({ type: 'started' });
`);

function sample(i) { return i % 71 === 0 ? -32768 : (i * 13 % 8000) - 2000; }
function fixture(size) {
    const buffer = new ArrayBuffer(size * size * 2);
    const view = new DataView(buffer);
    for (let i = 0; i < size * size; i++) view.setInt16(i * 2, sample(i), false);
    return buffer;
}
function reply(worker, send) {
    return new Promise((resolve, reject) => {
        const timer = setTimeout(() => reject(new Error('Worker reply timed out')), 10000);
        worker.once('message', data => { clearTimeout(timer); resolve(data); });
        worker.once('error', reject);
        if (send) worker.postMessage(send);
    });
}

(async () => {
    let worker, second;
    try {
        const decoder = await import(pathToFileURL(path.join(tmp, 'HgtDecoder.mjs')));
        assert.throws(() => decoder.decodeHgt(new ArrayBuffer(12)), /Invalid HGT/);
        let yields = 0;
        const fallback = await decoder.decodeHgtCooperatively(fixture(1201), async () => { yields++; });
        assert(yields > 1, 'fallback must yield between batches');
        for (let i = 0; i < fallback.data.length; i++) assert.equal(fallback.data[i], sample(i));
        console.log('PASS cooperative fallback, signed samples and voids');

        worker = new Worker(path.join(tmp, 'runner.mjs'));
        assert.equal((await reply(worker)).type, 'started');
        const invalid = await reply(worker, { type: 'prepareHgt', id: 0, key: 'bad', file: new Blob([new Uint8Array(12)]) });
        assert.equal(invalid.type, 'hgtFailed');
        assert.match(invalid.reason, /Invalid HGT/);
        for (const size of [1201, 3601]) {
            let uiTicks = 0;
            const timer = setInterval(() => uiTicks++, 1);
            const tile = await reply(worker, { type: 'prepareHgt', id: size, key: String(size), file: new Blob([fixture(size)]) });
            clearInterval(timer);
            assert.equal(tile.type, 'hgtReady');
            assert.equal(tile.size, size);
            assert.equal(tile.elevations.length, size * size);
            for (let i = 0; i < tile.elevations.length; i++) assert.equal(tile.elevations[i], sample(i));
            assert(uiTicks > 0, 'calling thread must remain able to run timers');
            // A chunk requested immediately after the acknowledgement must work.
            // Its copy also proves that transferring elevations did not detach
            // the worker's retained tile grid.
            const cells = (size - 1) / 30;
            const step = cells % 8 === 0 ? 8 : 1;
            const chunk = await reply(worker, { type: 'buildChunk', chunkKey: 'edge', hgtKey: String(size),
                size, vertsPerChunk: cells, cx: 29, cy: 29, step });
            assert.equal(chunk.type, 'chunkBuilt');
            const width = cells / step + 1;
            for (let row = 0; row < width; row++) for (let col = 0; col < width; col++) {
                assert.equal(chunk.heights[row * width + col], sample((29 * cells + row * step) * size + 29 * cells + col * step));
            }
            assert.equal(chunk.minH, Math.min(...chunk.heights));
            assert.equal(chunk.maxH, Math.max(...chunk.heights));
            console.log(`PASS SRTM ${size}: full decode + edge/LOD chunk, ${tile.prepareMs.toFixed(1)} ms in worker, ${uiTicks} calling-thread ticks`);
        }
        const missing = await reply(worker, { type: 'buildChunk', hgtKey: 'missing', chunkKey: 'none' });
        assert.equal(missing.reason, 'missing-hgt');
        console.log('PASS worker stays usable after invalid input');

        // Pool with shared memory: one worker decodes, another registers the
        // same grid, and both build chunks from it without any copy.
        const shared = decoder.decodeHgtShared(fixture(1201));
        assert(shared.data.buffer instanceof SharedArrayBuffer);
        for (let i = 0; i < shared.data.length; i++) assert.equal(shared.data[i], sample(i));
        assert.throws(() => decoder.decodeHgtShared(new ArrayBuffer(12)), /Invalid HGT/);
        // Streamed read: reads of odd sizes split samples between two buffers
        const bytes = new Uint8Array(fixture(1201)), sizes = [1, 3, 4096, 7, 65535, 2];
        const byteStream = (length) => () => {
            let pos = 0, k = 0;
            return new ReadableStream({ type: 'bytes', pull(c) {
                // A byte source ends a pending BYOB read with respond(0) after close()
                if (pos >= length) { c.close(); c.byobRequest?.respond(0); return; }
                const n = Math.min(sizes[k++ % sizes.length], length - pos);
                c.enqueue(bytes.slice(pos, pos + n)); pos += n;
            } });
        };
        const streamed = await decoder.readHgtShared({ size: bytes.length, stream: byteStream(bytes.length) }, 4096);
        assert(streamed.data.buffer instanceof SharedArrayBuffer);
        for (let i = 0; i < streamed.data.length; i++) assert.equal(streamed.data[i], sample(i));
        await assert.rejects(decoder.readHgtShared({ size: bytes.length, stream: byteStream(1001) }), /HGT read 1001 of/);
        await assert.rejects(decoder.readHgtShared(new Blob([new Uint8Array(12)])), /Invalid HGT/);
        const whole = await decoder.readHgtShared({ size: bytes.length, arrayBuffer: async () => fixture(1201),
            stream: () => new ReadableStream({ start(c) { c.close(); } }) });   // not a byte stream
        assert(whole.data.buffer instanceof SharedArrayBuffer);
        for (let i = 0; i < whole.data.length; i++) assert.equal(whole.data[i], sample(i));
        console.log('PASS streamed shared decode: split samples, truncated file, whole-read fallback');
        second = new Worker(path.join(tmp, 'runner.mjs'));
        assert.equal((await reply(second)).type, 'started');
        const tile = await reply(worker, { type: 'prepareHgt', id: 1, key: 'S', file: new Blob([fixture(1201)]), shared: true });
        assert.equal(tile.type, 'hgtReady');
        assert.equal(tile.shared, true);
        assert(tile.elevations.buffer instanceof SharedArrayBuffer, 'grid must come back as shared memory');
        for (let i = 0; i < tile.elevations.length; i++) assert.equal(tile.elevations[i], sample(i));
        second.postMessage({ type: 'registerHgt', key: 'S', size: 1201, elevations: tile.elevations });
        // A sample written here after registration is what both workers read: one grid, three threads
        const cells = 40, probe = (5 * cells) * 1201 + 7 * cells;
        tile.elevations[probe] = 4321;
        for (const [name, w] of [['decoding', worker], ['registered', second]]) {
            const chunk = await reply(w, { type: 'buildChunk', chunkKey: name, hgtKey: 'S', size: 1201, vertsPerChunk: cells, cx: 7, cy: 5, step: 1 });
            assert.equal(chunk.type, 'chunkBuilt');
            assert.equal(chunk.heights[0], 4321, `${name} worker must read the shared grid, not a copy`);
            for (let i = 1; i < chunk.heights.length; i++) {
                const row = Math.floor(i / (cells + 1)), col = i % (cells + 1);
                assert.equal(chunk.heights[i], sample((5 * cells + row) * 1201 + 7 * cells + col));
            }
        }
        console.log('PASS shared grid: decoded once, registered in a second worker, read by both without a copy');
        const copy = await reply(worker, { type: 'prepareHgt', id: 2, key: 'C', file: new Blob([fixture(1201)]), shared: false });
        assert.equal(copy.shared, false);
        assert(!(copy.elevations.buffer instanceof SharedArrayBuffer), 'copy mode transfers a private grid');
        console.log('PASS copy mode without shared memory');
    } finally {
        if (worker) await worker.terminate();
        if (second) await second.terminate();
        fs.rmSync(tmp, { recursive: true, force: true });
    }
})().catch(error => { console.error(error); process.exitCode = 1; });
