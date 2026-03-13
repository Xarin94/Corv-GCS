/**
 * HillshadeWorker.js - Off-thread hillshade computation
 */

self.onmessage = (e) => {
    const data = e.data || {};
    if (data.type !== 'computeHillshade') return;

    const normals = data.normals;
    if (!normals || normals.length % 3 !== 0) {
        self.postMessage({ type: 'hillshadeComputed', meshId: data.meshId, colors: null });
        return;
    }

    const sunDir = data.sunDir || { x: 0, y: 1, z: 0 };
    const sunlightEnabled = data.sunlightEnabled !== false;
    const brightness = typeof data.brightness === 'number' ? data.brightness : 0.85;

    const colors = new Float32Array(normals.length);
    for (let i = 0; i < normals.length; i += 3) {
        const nx = normals[i];
        const ny = normals[i + 1];
        const nz = normals[i + 2];
        let intensity = nx * sunDir.x + ny * sunDir.y + nz * sunDir.z;
        if (sunlightEnabled) {
            intensity = Math.max(0, intensity);
            intensity = 0.45 + intensity * 1.05;
        } else {
            intensity = brightness;
        }
        colors[i] = intensity;
        colors[i + 1] = intensity;
        colors[i + 2] = intensity;
    }

    self.postMessage({ type: 'hillshadeComputed', meshId: data.meshId, colors }, [colors.buffer]);
};
