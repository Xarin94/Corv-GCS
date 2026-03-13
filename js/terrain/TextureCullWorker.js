/**
 * TextureCullWorker.js - Off-thread selection of satellite textures to unload
 */

self.onmessage = (e) => {
    const data = e.data || {};
    if (data.type !== 'cullTextures') return;

    const { playerPos, radius, chunks } = data;
    if (!playerPos || !chunks || !Array.isArray(chunks)) {
        self.postMessage({ type: 'texturesToUnload', keys: [] });
        return;
    }

    const r2 = radius * radius;
    const keys = [];

    for (const item of chunks) {
        const dx = item.centerX - playerPos.x;
        const dz = item.centerZ - playerPos.z;
        if ((dx * dx + dz * dz) > r2) {
            keys.push(item.key);
        }
    }

    self.postMessage({ type: 'texturesToUnload', keys });
};
