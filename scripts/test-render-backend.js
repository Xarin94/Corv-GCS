#!/usr/bin/env node
const assert = require('node:assert/strict');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const { pathToFileURL } = require('node:url');
const tmp = fs.mkdtempSync(path.join(os.tmpdir(), 'corv-render-test-'));
for (const name of ['RenderBackend', 'RenderQuality']) {
    fs.writeFileSync(path.join(tmp, name + '.mjs'), fs.readFileSync(path.join(__dirname, '../js/engine', name + '.js'), 'utf8')
        .replaceAll("'./RenderQuality.js'", "'./RenderQuality.mjs'"));
}
(async () => {
    try {
        const { ThreeWebGLBackend } = await import(pathToFileURL(path.join(tmp, 'RenderBackend.mjs')));
        const { renderPixelRatio, readRenderQuality } = await import(pathToFileURL(path.join(tmp, 'RenderQuality.mjs')));
        assert.equal(renderPixelRatio('native', 2), 2);
        assert.equal(renderPixelRatio('balanced', 2), 1.5);
        assert.equal(renderPixelRatio('eco', 2), 1);
        assert.equal(renderPixelRatio('eco', 0.75), 0.75);
        assert.equal(readRenderQuality({ getItem() { throw Error('blocked'); } }), 'balanced');
        const renderer = {
            domElement: {}, info: { render: {}, reset() { this.render = { calls: 0, triangles: 0, points: 0, lines: 0 }; } },
            setPixelRatio(v) { this.ratio = v; }, getPixelRatio() { return this.ratio; },
            setSize(w, h) { this.domElement.width = w * this.ratio; this.domElement.height = h * this.ratio; },
            render() { if (this.info.autoReset) this.info.reset(); this.info.render.calls += 7; this.info.render.triangles += 100; },
            setRenderTarget() {}
        };
        const backend = new ThreeWebGLBackend({}, { renderer });
        backend.resize(1200, 800, 2);
        assert.equal(backend.getStats().height, 1200);
        backend.beginFrame();
        backend.render({}, {}); backend.render({}, {}); backend.render({}, {});
        backend.endFrame();
        assert.equal(backend.getStats().calls, 21, 'counts include ALL three passes');
        assert.equal(backend.getStats().passes, 3);
        backend.beginFrame(); backend.render({}, {}); backend.endFrame();
        assert.equal(backend.getStats().calls, 7, 'does not accumulate preceding frames');
        backend.setQuality('eco'); backend.resize(1200, 800, 2);
        assert.equal(backend.getStats().height, 800);
        assert(backend.getStats().submitP95Ms >= 0);
        console.log('PASS multipass totals, per-frame reset, resolution profiles and resize');
    } finally { fs.rmSync(tmp, { recursive: true, force: true }); }
})().catch(error => { console.error(error); process.exitCode = 1; });
