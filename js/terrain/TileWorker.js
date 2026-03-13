/**
 * TileWorker.js - Off-thread tile fetch + decode
 */

self.onmessage = async (e) => {
    const data = e.data || {};
    if (data.type !== 'loadTile') return;

    try {
        const res = await fetch(data.url, { mode: 'cors' });
        if (!res.ok) {
            self.postMessage({ type: 'tileError', key: data.key });
            return;
        }

        const blob = await res.blob();
        const bitmap = await createImageBitmap(blob);
        self.postMessage({ type: 'tileLoaded', key: data.key, bitmap }, [bitmap]);
    } catch (err) {
        self.postMessage({ type: 'tileError', key: data.key });
    }
};
