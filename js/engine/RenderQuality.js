export const RENDER_QUALITY_KEY = 'renderQuality';
export const DEFAULT_RENDER_QUALITY = 'balanced';
const CAPS = Object.freeze({ native: Infinity, balanced: 1.5, eco: 1 });

export function normalizeRenderQuality(value) {
    return Object.hasOwn(CAPS, value) ? value : DEFAULT_RENDER_QUALITY;
}

export function renderPixelRatio(quality, displayRatio = 1) {
    const ratio = Number.isFinite(displayRatio) && displayRatio > 0 ? displayRatio : 1;
    return Math.min(ratio, CAPS[normalizeRenderQuality(quality)]);
}

export function readRenderQuality(storage) {
    try { return normalizeRenderQuality(storage.getItem(RENDER_QUALITY_KEY)); }
    catch (_) { return DEFAULT_RENDER_QUALITY; }
}
