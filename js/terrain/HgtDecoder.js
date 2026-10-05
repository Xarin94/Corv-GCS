/** HGT is signed, big-endian Int16. Keep void samples intact for the callers. */
export function hgtSize(byteLength) {
    if (byteLength === 1201 * 1201 * 2) return 1201;
    if (byteLength === 3601 * 3601 * 2) return 3601;
    throw new Error(`Invalid HGT length: ${byteLength} bytes (expected SRTM1 or SRTM3)`);
}

// Reuse the input allocation: each big-endian sample is read before overwriting
// it in native byte order. This function belongs on the terrain worker.
export function decodeHgt(buffer) {
    const size = hgtSize(buffer.byteLength);
    const view = new DataView(buffer);
    const data = new Int16Array(buffer);
    for (let i = 0; i < data.length; i++) data[i] = view.getInt16(i * 2, false);
    return { size, data };
}

// Same decode, into shared memory: the one grid of a tile that the UI thread
// and every terrain worker read, with no per-thread copy. Needs SharedArrayBuffer.
export function decodeHgtShared(buffer) {
    const size = hgtSize(buffer.byteLength);
    const view = new DataView(buffer);
    const data = new Int16Array(new SharedArrayBuffer(buffer.byteLength));
    for (let i = 0; i < data.length; i++) data[i] = view.getInt16(i * 2, false);
    return { size, data };
}

// Read a File/Blob straight into a shared grid, through one small reused
// buffer. Reading the whole file first left a dead 25 MB buffer in every
// worker, and an idle worker does not garbage-collect it. Falls back to that
// whole read where the stream is not a byte stream.
export async function readHgtShared(blob, chunkBytes = 1 << 20) {
    const size = hgtSize(blob.size);
    let reader;
    try {
        reader = blob.stream().getReader({ mode: 'byob' });
    } catch (_) {
        return decodeHgtShared(await blob.arrayBuffer());
    }
    const data = new Int16Array(new SharedArrayBuffer(blob.size));
    let buffer = new ArrayBuffer(chunkBytes);
    let carry = 0;   // first byte of a sample split between two reads
    let n = 0;
    for (;;) {
        const { done, value } = await reader.read(new Uint8Array(buffer, carry, chunkBytes - carry));
        if (done) break;
        buffer = value.buffer;
        const bytes = new Uint8Array(buffer, 0, carry + value.byteLength);
        const end = bytes.length & ~1;
        // Big-endian pairs; the Int16Array store wraps 0..65535 to the signed value
        for (let i = 0; i < end; i += 2) data[n++] = (bytes[i] << 8) | bytes[i + 1];
        carry = bytes.length - end;
        if (carry) bytes[0] = bytes[end];
    }
    if (carry || n !== data.length) throw new Error(`HGT read ${n * 2 + carry} of ${blob.size} bytes`);
    return { size, data };
}

// Emergency path for hosts where Worker creation fails. Yield between small
// batches instead of performing a complete SRTM1 conversion on the UI thread.
export async function decodeHgtCooperatively(buffer, yieldTask = () => new Promise(r => setTimeout(r, 0))) {
    const size = hgtSize(buffer.byteLength);
    const view = new DataView(buffer);
    const data = new Int16Array(buffer);
    for (let start = 0; start < data.length; start += 65536) {
        const end = Math.min(start + 65536, data.length);
        for (let i = start; i < end; i++) data[i] = view.getInt16(i * 2, false);
        if (end < data.length) await yieldTask();
    }
    return { size, data };
}
