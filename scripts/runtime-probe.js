// Loaded only by the measurement harness. All hooks are removed at completion.
import * as scene from '../js/engine/Scene3D.js';
import * as terrain from '../js/terrain/TerrainManager.js';
import { STATE } from '../js/core/state.js';

export function start() {
    const backend = scene.getRenderBackend(), renderer = scene.getRenderer();
    const gl = renderer.getContext(), ext = gl.getExtension('EXT_disjoint_timer_query_webgl2');
    const begin = backend.beginFrame, end = backend.endFrame;
    const originalQuality = backend.quality, originalSatellite = window.satelliteEnabled;
    let frames = [], gpu = [], longTasks = [], count = 0, lastTime = null, active = null;
    let pending = [], disjoint = 0;
    const observer = new PerformanceObserver(list => {
        for (const task of list.getEntries()) longTasks.push({ time: performance.timeOrigin + task.startTime, ms: task.duration });
    });
    observer.observe({ type: 'longtask', buffered: true });
    backend.beginFrame = function() {
        if (ext) {
            const invalid = gl.getParameter(ext.GPU_DISJOINT_EXT);
            while (pending.length && (invalid || gl.getQueryParameter(pending[0].query, gl.QUERY_RESULT_AVAILABLE))) {
                const item = pending.shift();
                if (invalid) disjoint++;
                else gpu.push({ time: item.time, ms: gl.getQueryParameter(item.query, gl.QUERY_RESULT) / 1e6 });
                gl.deleteQuery(item.query);
            }
            if (++count % 15 === 0 && pending.length < 8 && !gl.getQuery(ext.TIME_ELAPSED_EXT, gl.CURRENT_QUERY)) {
                active = { query: gl.createQuery(), time: performance.timeOrigin + performance.now() };
                gl.beginQuery(ext.TIME_ELAPSED_EXT, active.query);
            }
        }
        return begin.call(this);
    };
    backend.endFrame = function() {
        end.call(this);
        if (active) { gl.endQuery(ext.TIME_ELAPSED_EXT); pending.push(active); active = null; }
        const time = performance.timeOrigin + performance.now();
        frames.push({ time, intervalMs: lastTime === null ? null : time - lastTime,
            ...this.lastFrame });
        lastTime = time;
    };
    function configure(satellite, quality) {
        if (window.satelliteEnabled !== satellite) window.toggleSatellite();
        scene.setRenderQuality(quality);
    }
    return {
        info: { gpuTimerSupported: !!ext, originalQuality, originalSatellite },
        configure,
        snapshot() {
            const result = {
                time: Date.now(), hidden: document.hidden, focused: document.hasFocus(),
                viewport: [innerWidth, innerHeight, devicePixelRatio], frames, gpu, longTasks, disjoint,
                backend: scene.getRenderPerformanceStats(), memory: terrain.getMemoryStats(),
                jsHeapBytes: performance.memory?.usedJSHeapSize,
                workers: window.__runtimeWorkers || [],
                state: { mode: STATE.mode, connected: STATE.connected, connectionType: STATE.connectionType,
                    lat: STATE.lat, lon: STATE.lon, rawAlt: STATE.rawAlt, satellite: window.satelliteEnabled,
                    camera: document.getElementById('btn-cam')?.textContent, fpsCap: document.getElementById('render-fps-select')?.value,
                    lidarPoints: scene.getRenderWorld().pointClouds.get('livox')?.total || 0 },
                gl: { ...renderer.info.memory, programs: renderer.info.programs.length }
            };
            frames = []; gpu = []; longTasks = [];
            return result;
        },
        stop() {
            configure(originalSatellite, originalQuality);
            backend.beginFrame = begin; backend.endFrame = end;
            observer.disconnect();
            for (const item of pending) gl.deleteQuery(item.query);
            pending = [];
            if (window.__runtimeOriginalWorker) window.Worker = window.__runtimeOriginalWorker;
            for (const cleanup of window.__runtimeWorkerCleanups || []) cleanup();
            window.__runtimeWorkerCleanups = [];
            return this.snapshot();
        }
    };
}
