import { normalizeRenderQuality, renderPixelRatio } from './RenderQuality.js';

/**
 * Owns renderer creation, resolution, frame passes and GPU resource adapters.
 * Terrain and Livox adapters consume neutral data. Scene overlays, ROS and the
 * compatibility camera/renderer still need porting for another graphics API.
 */
export class ThreeWebGLBackend {
    constructor(three, { quality = 'balanced', renderer, terrainFactory, pointCloudFactory } = {}) {
        this.three = three;
        this.terrainFactory = terrainFactory;
        this.pointCloudFactory = pointCloudFactory;
        this.renderer = renderer || new three.WebGLRenderer({
            antialias: true, alpha: false, powerPreference: 'high-performance'
        });
        this.renderer.outputColorSpace = three.LinearSRGBColorSpace;
        // Three normally resets after EACH pass, hiding world + outline costs.
        this.renderer.info.autoReset = false;
        this.quality = normalizeRenderQuality(quality);
        this.submissions = new Float64Array(120);
        this.frames = 0;
        this.lastFrame = { passes: 0, calls: 0, triangles: 0, points: 0, lines: 0, submitMs: 0 };
    }

    resize(width, height, displayRatio = 1) {
        this.renderer.setPixelRatio(renderPixelRatio(this.quality, displayRatio));
        this.renderer.setSize(width, height);
    }

    setQuality(quality) { this.quality = normalizeRenderQuality(quality); }
    createTerrainView(options) {
        if (!this.terrainFactory) throw new Error('Terrain adapter is not configured');
        this.terrainView = this.terrainFactory({ ...options, three: this.three, renderer: this.renderer });
        return this.terrainView;
    }
    createPointCloudView(options) {
        if (!this.pointCloudFactory) throw new Error('Point cloud adapter is not configured');
        this.pointCloudView = this.pointCloudFactory({ ...options, three: this.three, renderer: this.renderer });
        return this.pointCloudView;
    }
    beginFrame() {
        this.renderer.info.reset();
        this.passes = 0;
        this.frameStart = performance.now();
    }
    render(scene, camera) {
        this.passes++;
        this.renderer.render(scene, camera);
    }
    setRenderTarget(target) { this.renderer.setRenderTarget(target); }
    endFrame() {
        const submitMs = performance.now() - this.frameStart;
        this.submissions[this.frames % this.submissions.length] = submitMs;
        this.frames++;
        const { calls, triangles, points, lines } = this.renderer.info.render;
        this.lastFrame = { passes: this.passes, calls, triangles, points, lines, submitMs };
    }
    getStats() {
        const n = Math.min(this.frames, this.submissions.length);
        const sorted = Array.from(this.submissions.subarray(0, n)).sort((a, b) => a - b);
        return {
            backend: 'three-webgl2', quality: this.quality, frames: this.frames,
            pixelRatio: this.renderer.getPixelRatio(),
            width: this.renderer.domElement.width, height: this.renderer.domElement.height,
            ...this.lastFrame,
            // CPU wall time for frame submission, NOT GPU duration or full UI time.
            submitP95Ms: n ? sorted[Math.ceil(n * 0.95) - 1] : 0
        };
    }
    dispose() { this.terrainView?.dispose(); this.pointCloudView?.dispose(); this.renderer.dispose(); }
}
