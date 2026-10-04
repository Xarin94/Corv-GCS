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
