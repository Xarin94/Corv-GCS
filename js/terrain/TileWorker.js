/**
 * TileWorker.js - Off-thread tile fetch + decode
 * Returns both ImageBitmap (for rendering) and Blob (for IndexedDB caching).
 */

self.onmessage = async (e) => {
    const data = e.data || {};
    if (data.type !== 'loadTile') return;

    try {
        const res = await fetch(data.url);
        if (!res.ok) {
            self.postMessage({ type: 'tileError', key: data.key });
            return;
        }

        const blob = await res.blob();
        const bitmap = await createImageBitmap(blob);
        // Transfer bitmap, send blob as cloneable (for IndexedDB caching on main thread)
        self.postMessage({ type: 'tileLoaded', key: data.key, bitmap, blob }, [bitmap]);
    } catch (err) {
        self.postMessage({ type: 'tileError', key: data.key });
    }
};
