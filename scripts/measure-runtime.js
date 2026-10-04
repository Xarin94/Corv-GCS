#!/usr/bin/env node
// Visible GCS, normal profile and network. Read-only sampling; no vehicle commands.
// Usage: node scripts/measure-runtime.js [--output file.json]
// Leaves the normal application open after removing the probes.
const fs = require('node:fs');
const path = require('node:path');
const os = require('node:os');
const { spawn } = require('node:child_process');
const root = path.resolve(__dirname, '..');
const outputIndex = process.argv.indexOf('--output');
const output = path.resolve(outputIndex >= 0 ? process.argv[outputIndex + 1] : path.join(root, 'docs/audits/2026-10-04/real-load.json'));

if (!process.versions.electron) {
    const env = { ...process.env }; delete env.ELECTRON_RUN_AS_NODE;
    const child = spawn(require('electron'), [__filename, '--output', output], {
        cwd: root, env, windowsHide: true, stdio: ['ignore', 'pipe', 'pipe']
    });
    let pending = '';
    child.stdout.on('data', data => {
        pending += data.toString();
        const lines = pending.split(/\r?\n/); pending = lines.pop();
        for (const line of lines) if (line.includes('[MEASURE]')) console.log(line);
    });
    child.stderr.on('data', () => {}); // Normal app log still receives all diagnostics.
    child.on('error', error => { console.error(error); process.exitCode = 1; });
    child.on('exit', code => { console.log('[MEASURE] Application exited:', code); process.exitCode = code || 0; });
} else {
    runElectron();
}

function instrumentWorkers() {
    const Original = window.Worker;
    window.__runtimeOriginalWorker = Original;
    window.__runtimeWorkers = [];
    window.Worker = class extends Original {
        constructor(url, options) {
            super(url, options);
            const entry = { file: String(url).split('/').pop(), sent: 0, received: 0, types: {}, errors: [] };
            window.__runtimeWorkers.push(entry);
            const onMessage = event => {
                entry.received++;
                const type = event.data?.type || (event.data?.ok === undefined ? 'other' : 'compressed');
                entry.types[type] = (entry.types[type] || 0) + 1;
            };
            const onError = event => entry.errors.push(event.message);
            this.addEventListener('message', onMessage);
            this.addEventListener('error', onError);
            const send = this.postMessage.bind(this);
            this.postMessage = (...args) => { entry.sent++; return send(...args); };
            (window.__runtimeWorkerCleanups ||= []).push(() => {
                this.removeEventListener('message', onMessage);
                this.removeEventListener('error', onError);
                this.postMessage = send;
            });
        }
    };
}

function runElectron() {
    const electron = require('electron'), Module = require('node:module');
    const { app } = electron, pkg = require(path.join(root, 'package.json'));
    app.setName(pkg.name); app.setAppPath(root);
    app.setPath('userData', path.join(app.getPath('appData'), pkg.name));
    fs.mkdirSync(path.dirname(output), { recursive: true });
    const result = {
        startedAt: new Date().toISOString(), version: pkg.version,
        runtime: { electron: process.versions.electron, chromium: process.versions.chrome, node: process.versions.node },
        host: { cpu: os.cpus()[0].model.trim(), logicalCpus: os.cpus().length, ramBytes: os.totalmem() },
        mode: 'visible window, normal user profile, real network, built-in demo flight',
        samples: [], osSamples: [], phases: [], errors: [], finished: false
    };
    let win, probeReady = false, busy = false, ticks = 0, finishing = false;
    let planIndex = 0, planStarted = 0, initialQuality;
    const plan = [
        { name: 'satellite', satellite: true, quality: null, seconds: 85, warmup: 35 },
        { name: 'schematic', satellite: false, quality: null, seconds: 55, warmup: 10 },
        { name: 'satellite-eco', satellite: true, quality: 'eco', seconds: 55, warmup: 10 },
        { name: 'satellite-restored-profile', satellite: true, quality: null, seconds: 25, warmup: 5, profile: true }
    ];
    const osProbe = process.platform === 'win32' ? spawn('powershell.exe', [
        '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', path.join(__dirname, 'measure-runtime-os.ps1')
    ], { windowsHide: true, stdio: ['pipe', 'pipe', 'pipe'] }) : null;
    let osText = '';
    osProbe?.stdout.on('data', chunk => {
        osText += chunk.toString(); const lines = osText.split(/\r?\n/); osText = lines.pop();
        for (const line of lines) if (line.trim()) {
            try { result.osSamples.push(JSON.parse(line)); } catch { result.errors.push('OS sampler: ' + line); }
        }
    });
    osProbe?.stderr.on('data', chunk => result.errors.push('OS sampler: ' + chunk.toString()));
    osProbe?.on('error', error => result.errors.push(error.message));
    osProbe?.stdin.on('error', error => result.errors.push(error.message));
    function save() {
        const document = result.finished ? result : {
            startedAt: result.startedAt, finished: false, phase: plan[planIndex]?.name,
            sampleCount: result.samples.length, osSampleCount: result.osSamples.length,
            latest: result.samples[result.samples.length - 1], probe: result.probe, errors: result.errors
        };
        fs.writeFileSync(output, JSON.stringify(document, null, 2));
    }
    async function beginPhase() {
        const phase = plan[planIndex];
        await win.webContents.executeJavaScript(`window.__runtimeProbe.configure(${phase.satellite}, ${JSON.stringify(phase.quality || initialQuality)})`);
        planStarted = Date.now();
        result.phases.push({ ...phase, quality: phase.quality || initialQuality, start: planStarted });
        if (phase.profile) {
            await win.webContents.debugger.sendCommand('Profiler.enable');
            await win.webContents.debugger.sendCommand('Profiler.setSamplingInterval', { interval: 1000 });
            await win.webContents.debugger.sendCommand('Profiler.start');
        }
        console.log('[MEASURE] Phase ' + phase.name + ', ' + phase.seconds + ' seconds');
        save();
    }
    class MeasuredWindow extends electron.BrowserWindow {
        constructor(options) { super(options); win = this; this.webContents.debugger.attach('1.3'); }
        async loadFile(...args) {
            // Start the renderer before awaiting Page commands; an un-navigated
            // webContents has no renderer to answer them yet.
            await super.loadURL('about:blank');
            await this.webContents.debugger.sendCommand('Page.enable');
            await this.webContents.debugger.sendCommand('Page.addScriptToEvaluateOnNewDocument', { source: '(' + instrumentWorkers.toString() + ')()' });
            this.webContents.once('did-finish-load', async () => {
                try {
                    result.probe = await this.webContents.executeJavaScript(`(async () => {
                        const module = await import('../scripts/runtime-probe.js');
                        window.__runtimeProbe = module.start(); return window.__runtimeProbe.info;
                    })()`);
                    initialQuality = result.probe.originalQuality;
                    this.show(); this.focus();
                    result.gpuFeatures = app.getGPUFeatureStatus();
                    result.gpuInfo = await app.getGPUInfo('basic');
                    result.onBatteryPower = electron.powerMonitor.isOnBatteryPower();
                    probeReady = true; await beginPhase();
                } catch (error) { result.errors.push(error.stack); save(); console.log('[MEASURE] Setup failed:', error.message); }
            });
            return super.loadFile(...args);
        }
    }
    const originalLoad = Module._load;
    Module._load = function(request, parent, isMain) {
        if (request === 'electron') return { ...electron, BrowserWindow: MeasuredWindow };
        return originalLoad.apply(this, arguments);
    };
    require(path.join(root, 'main.js'));
    Module._load = originalLoad;
    const timer = setInterval(async () => {
        if (finishing || busy || !win || win.isDestroyed()) return;
        busy = true;
        try {
            const sample = { time: Date.now(), metrics: app.getAppMetrics(),
                window: { visible: win.isVisible(), minimized: win.isMinimized(), focused: win.isFocused() } };
            if (++ticks % 2 === 0 && osProbe?.stdin.writable) {
                osProbe.stdin.write(JSON.stringify({ pids: sample.metrics.map(m => m.pid) }) + '\n');
            }
            if (probeReady) sample.renderer = await win.webContents.executeJavaScript('window.__runtimeProbe.snapshot()');
            result.samples.push(sample);
            if (ticks % 10 === 0) {
                const r = sample.renderer;
                console.log('[MEASURE] ' + plan[planIndex].name + ' frames=' + (r?.frames.length || 0) + ' chunks=' + (r?.memory.chunksActive || 0) + ' tex=' + (r?.gl.textures || 0) + ' heapMiB=' + Math.round((r?.jsHeapBytes || 0) / 1048576));
                save();
            }
            if (probeReady && Date.now() - planStarted >= plan[planIndex].seconds * 1000) {
                result.phases[result.phases.length - 1].end = Date.now();
                if (plan[planIndex].profile) {
                    const { profile } = await win.webContents.debugger.sendCommand('Profiler.stop');
                    result.profile = summarizeProfile(profile);
                }
                if (++planIndex < plan.length) await beginPhase();
                else {
                    finishing = true; clearInterval(timer);
                    result.finalRenderer = await win.webContents.executeJavaScript('window.__runtimeProbe.stop()');
                    win.webContents.debugger.detach();
                    osProbe?.stdin.end();
                    if (osProbe && osProbe.exitCode === null) await new Promise(resolve => osProbe.once('close', resolve));
                    result.finished = true; result.finishedAt = new Date().toISOString();
                    result.summary = summarize(result);
                    result.osSampler = {
                        samples: result.osSamples.length,
                        collectionMs: stats(result.osSamples.map(s => s.samplerMs)),
                        cpuSeconds: result.osSamples[result.osSamples.length - 1]?.samplerCpuSeconds,
                        errors: result.osSamples.filter(s => s.error).map(s => s.error)
                    };
                    delete result.osSamples;
                    for (const sample of result.samples) if (sample.renderer) {
                        const r = sample.renderer;
                        r.frameIntervalsMs = stats(r.frames.map(f => f.intervalMs));
                        r.submitCpuMs = stats(r.frames.map(f => f.submitMs));
                        r.gpuDrawMs = stats(r.gpu.map(g => g.ms));
                        delete r.frames; delete r.gpu;
                    }
                    save();
                    console.log('[MEASURE] DONE ' + output);
                }
            }
        } catch (error) { result.errors.push(error.stack); save(); console.log('[MEASURE] Error:', error.message); }
        finally { busy = false; }
    }, 1000);
    app.on('before-quit', () => { clearInterval(timer); osProbe?.stdin.end(); if (!result.finished) save(); });
}

function stats(values) {
    const data = values.filter(Number.isFinite).sort((a, b) => a - b);
    if (!data.length) return null;
    const quantile = p => data[Math.min(data.length - 1, Math.ceil(data.length * p) - 1)];
    return { count: data.length, mean: data.reduce((a, b) => a + b, 0) / data.length,
        p50: quantile(.5), p95: quantile(.95), p99: quantile(.99), max: data[data.length - 1] };
}

function summarizeProfile(profile) {
    const nodes = new Map(profile.nodes.map(node => [node.id, node]));
    const costs = new Map();
    profile.samples.forEach((id, i) => {
        const frame = nodes.get(id)?.callFrame; if (!frame) return;
        const key = frame.functionName + '|' + frame.url + ':' + (frame.lineNumber + 1);
        costs.set(key, (costs.get(key) || 0) + (profile.timeDeltas[i] || 0));
    });
    return { elapsedMs: (profile.endTime - profile.startTime) / 1000,
        topSelf: [...costs].sort((a, b) => b[1] - a[1]).slice(0, 25).map(([location, us]) => ({ location, ms: us / 1000 })) };
}

function summarize(result) {
    return result.phases.map(phase => {
        const start = phase.start + phase.warmup * 1000, end = phase.end;
        const samples = result.samples.filter(s => s.time >= start && s.time <= end);
        const inRange = item => item.time >= start && item.time <= end;
        const frames = result.samples.flatMap(s => s.renderer?.frames || []).filter(inRange);
        const gpu = result.samples.flatMap(s => s.renderer?.gpu || []).filter(inRange);
        const longTasks = result.samples.flatMap(s => s.renderer?.longTasks || []).filter(inRange);
        const cpu = [], processCpu = {};
        for (let i = 1; i < samples.length; i++) {
            const previous = samples[i - 1], current = samples[i], dt = (current.time - previous.time) / 1000;
            let total = 0;
            for (const m of current.metrics) {
                const prev = previous.metrics.find(v => v.pid === m.pid);
                if (!prev) continue;
                const cores = (m.cpu.cumulativeCPUUsage - prev.cpu.cumulativeCPUUsage) / dt;
                total += cores;
                const key = m.type + (m.serviceName ? ':' + m.serviceName : '');
                (processCpu[key] ||= []).push(cores);
            }
            cpu.push(total);
        }
        const osSamples = result.osSamples.filter(inRange), threadCpu = new Map(), hardware = [];
        for (let i = 1; i < osSamples.length; i++) {
            const prev = osSamples[i - 1], current = osSamples[i], dt = (current.time - prev.time) / 1000;
            const previousThreads = new Map((prev.threads || []).map(t => [t.processId + ':' + t.threadId, t]));
            for (const t of current.threads || []) {
                const key = t.processId + ':' + t.threadId, old = previousThreads.get(key); if (!old) continue;
                const item = threadCpu.get(key) || { processId: t.processId, threadId: t.threadId, name: t.name, cpuSeconds: 0, spanSeconds: 0, peakCorePercent: 0 };
                const delta = Math.max(0, t.cpuSeconds - old.cpuSeconds);
                item.cpuSeconds += delta; item.spanSeconds += dt; item.peakCorePercent = Math.max(item.peakCorePercent, delta / dt * 100);
                threadCpu.set(key, item);
            }
            const oldEngines = new Map((prev.gpu || []).map(g => [g.name, g]));
            const engines = new Map();
            for (const g of current.gpu || []) {
                const old = oldEngines.get(g.name); if (!old) continue;
                const elapsed = Number(BigInt(g.timestamp100ns) - BigInt(old.timestamp100ns));
                if (elapsed <= 0) continue;
                const used = Number(BigInt(g.runningTime) - BigInt(old.runningTime)) / elapsed * 100;
                const key = g.name.replace(/^pid_\d+_/, '');
                engines.set(key, (engines.get(key) || 0) + Math.max(0, used));
            }
            hardware.push(Math.max(0, ...engines.values()));
        }
        const totalCpu = stats(cpu), first = frames[0]?.time, last = frames[frames.length - 1]?.time;
        return { phase: phase.name, quality: phase.quality, measuredSeconds: (end - start) / 1000,
            profilerEnabled: !!phase.profile,
            cpuCoreEquivalent: totalCpu,
            cpuMachinePercent: totalCpu ? totalCpu.mean / result.host.logicalCpus * 100 : null,
            cpuByProcess: Object.fromEntries(Object.entries(processCpu).map(([key, values]) => [key, stats(values)])),
            fps: frames.length > 1 ? (frames.length - 1) * 1000 / (last - first) : null,
            frameIntervalMs: stats(frames.map(f => f.intervalMs)), submitCpuMs: stats(frames.map(f => f.submitMs)),
            gpuDrawMs: stats(gpu.map(g => g.ms)), gpuBusiestEnginePercent: stats(hardware),
            workingSetSumMiB: stats(samples.map(s => s.metrics.reduce((n, m) => n + m.memory.workingSetSize, 0) / 1024)),
            privateBytesSumMiB: stats(samples.map(s => s.metrics.reduce((n, m) => n + (m.memory.privateBytes || 0), 0) / 1024)),
            jsHeapMiB: stats(samples.map(s => s.renderer?.jsHeapBytes / 1048576)),
            longTasks: { count: longTasks.length, totalMs: longTasks.reduce((n, t) => n + t.ms, 0), maxMs: Math.max(0, ...longTasks.map(t => t.ms)) },
            drawCalls: stats(frames.map(f => f.calls)), triangles: stats(frames.map(f => f.triangles)),
            windowVisibleAllSamples: samples.every(s => s.window.visible && !s.window.minimized),
            documentVisibleAllSamples: samples.every(s => !s.renderer?.hidden),
            focusedSamples: samples.filter(s => s.window.focused).length, samples: samples.length,
            topThreads: [...threadCpu.values()].map(t => ({ ...t, meanCorePercent: t.cpuSeconds / t.spanSeconds * 100 })).sort((a, b) => b.cpuSeconds - a.cpuSeconds).slice(0, 18),
            gpuMemoryLast: osSamples[osSamples.length - 1]?.gpuMemory,
            memoryLast: samples[samples.length - 1]?.renderer?.memory,
            stateLast: samples[samples.length - 1]?.renderer?.state
        };
    });
}
