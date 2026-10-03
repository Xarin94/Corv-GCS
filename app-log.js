/**
 * app-log.js - Rolling debug log: the last five minutes of the app, on disk
 *
 * Every console.* line of the main process, every console message of the
 * renderer (forwarded by attachWindow) and the periodic status lines go into an
 * in-memory window. Every FLUSH_MS the whole window is written over
 *
 *   <data root>/debug/corv-gcs-debug.log
 *
 * through a temp file + rename, so a reader never sees half a file. Lines older
 * than WINDOW_MS fall off the top; past MAX_BYTES the oldest go even inside the
 * window, so the file stays small enough to mail. At startup the previous
 * session's file becomes corv-gcs-debug.prev.log: the minutes before a crash
 * survive the restart the user makes before asking for help.
 *
 * The file opens with a header that is always current (app, system, GPU,
 * displays, connection, vehicle, resources) followed by the session's last
 * errors and warnings older than the window — the context a five-minute cut
 * would otherwise lose.
 *
 * Guards: identical consecutive lines collapse into one with a count, a flood
 * guard caps the lines per second, and each line has a length cap.
 */

const { app, ipcMain, dialog, shell, BrowserWindow } = require('electron');
const fs = require('fs');
const os = require('os');
const path = require('path');
const util = require('util');

const WINDOW_MS = 5 * 60 * 1000;      // what the file keeps
const MAX_BYTES = 3 * 1024 * 1024;    // hard cap: past it the oldest lines go even inside the window
const FLUSH_MS = 3000;                // rewrite period
const FLUSH_SOON_MS = 400;            // after an error: on disk before anything can take the process down
const MAX_LINES_PER_SEC = 200;        // flood guard: a loop logging every frame cannot push out the rest
const MAX_MSG = 1500;                 // characters per line…
const MAX_ERR_MSG = 6000;             // …errors keep their stack
const OLD_ISSUES = 40;                // errors/warnings older than the window kept in the header
const STATUS_MS = 30000;              // main-process resource line
const GPU_DESCRIBE_DELAY_MS = 8000;   // after ready: the GPU process has started and reported
const LOOP_PROBE_MS = 500;            // event-loop lag probe
const FILE_NAME = 'corv-gcs-debug.log';
const PREV_NAME = 'corv-gcs-debug.prev.log';

// The console as it was before the hook: lets this module print without
// feeding its own lines back into the window.
const raw = {};

let entries = [];        // { t, tLast, lvl, src, msg, n, line, bytes }
let head = 0;            // index of the oldest live entry
let bytes = 0;
let last = null;         // last entry, for the repeat collapse
let dirty = false;
let writing = false;
let soonTimer = null;
let started = false;
const sessionStart = Date.now();

let floodSecond = 0, floodCount = 0, floodDropped = 0, droppedTotal = 0;
let trimmedBySize = 0;
const oldIssues = [];    // rendered lines of errors/warnings that left the window

const header = new Map();
const throttles = new Map();

let logDir = null;

function pad(n, w = 2) { return String(n).padStart(w, '0'); }

function clock(t) {
    const d = new Date(t);
    return `${pad(d.getHours())}:${pad(d.getMinutes())}:${pad(d.getSeconds())}.${pad(d.getMilliseconds(), 3)}`;
}

function stamp(t) {
    const d = new Date(t);
    return `${d.getFullYear()}-${pad(d.getMonth() + 1)}-${pad(d.getDate())} ${clock(t)}`;
}

function mb(b) { return (b / 1048576).toFixed(1); }

function getDir() {
    if (logDir) return logDir;
    try {
        logDir = path.join(require('./mission-store').getRoot(), 'debug');
    } catch (e) {
        logDir = path.join(app.getPath('userData'), 'debug');
    }
    fs.mkdirSync(logDir, { recursive: true });
    return logDir;
}

function filePath() { return path.join(getDir(), FILE_NAME); }
function prevPath() { return path.join(getDir(), PREV_NAME); }

// ── Collecting ───────────────────────────────────────────────────────────────

/**
 * Add one line to the window.
 * @param {'E'|'W'|'I'|'D'} lvl
 * @param {string} src  'main' | 'rend' | …
 * @param {string} msg
 */
function write(lvl, src, msg) {
    const now = Date.now();
    msg = String(msg);
    const cap = lvl === 'E' ? MAX_ERR_MSG : MAX_MSG;
    if (msg.length > cap) msg = `${msg.slice(0, cap)} …[+${msg.length - cap} chars]`;

    if (last && last.msg === msg && last.lvl === lvl && last.src === src
        && entries.length - 1 >= head && entries[entries.length - 1] === last) {
        last.n++;
        last.tLast = now;
        last.line = null;
        dirty = true;
        return;
    }

    const sec = Math.floor(now / 1000);
    if (sec !== floodSecond) {
        if (floodDropped) {
            const n = floodDropped;
            floodDropped = 0;
            floodSecond = sec;
            floodCount = 0;
            write('W', 'log', `[log] ${n} lines dropped in one second (flood guard, ${MAX_LINES_PER_SEC}/s)`);
        }
        floodSecond = sec;
        floodCount = 0;
    }
    if (++floodCount > MAX_LINES_PER_SEC && lvl !== 'E') {
        floodDropped++;
        droppedTotal++;
        return;
    }

    const e = { t: now, tLast: now, lvl, src, msg, n: 1, line: null, bytes: msg.length + 24 };
    entries.push(e);
    bytes += e.bytes;
    last = e;
    dirty = true;

    if (bytes > MAX_BYTES * 1.25) trim(now);
    if (lvl === 'E') flushSoon();
}

function render(e) {
    if (e.line) return e.line;
    let s = `${clock(e.t)} ${e.lvl} ${e.src.padEnd(4)} ${e.msg.replace(/\n/g, '\n                    ')}`;
    if (e.n > 1) s += `  [x${e.n}, last ${clock(e.tLast)}]`;
    e.line = s;
    return s;
}

/** Drop what left the window, and the oldest lines past the size cap. */
function trim(now) {
    const cutoff = now - WINDOW_MS;
    while (head < entries.length) {
        const e = entries[head];
        const old = e.tLast < cutoff;
        const tooBig = bytes > MAX_BYTES;
        if (!old && !tooBig) break;
        if (!old) trimmedBySize++;
        if (e.lvl === 'E' || e.lvl === 'W') {
            oldIssues.push(render(e));
            if (oldIssues.length > OLD_ISSUES) oldIssues.shift();
        }
        bytes -= e.bytes;
        head++;
    }
    if (head > 2000 && head > entries.length / 2) {
        entries = entries.slice(head);
        head = 0;
    }
}

/**
 * Rate limit for noisy call sites.
 * @returns {number} -1 when this call must stay quiet, otherwise how many
 *   calls were kept quiet since the last one that went through.
 */
function throttle(key, ms) {
    const now = Date.now();
    const t = throttles.get(key);
    if (t && now - t.at < ms) { t.skipped++; return -1; }
    const skipped = t ? t.skipped : 0;
    throttles.set(key, { at: now, skipped: 0 });
    return skipped;
}

/** A header line, always current (multi-line values are indented). */
function setHeader(key, value) {
    key = String(key).slice(0, 24);
    if (value === null || value === undefined || value === '') header.delete(key);
    else header.set(key, String(value).slice(0, 3000));
    dirty = true;
}

// ── Writing ──────────────────────────────────────────────────────────────────

function buildText() {
    const now = Date.now();
    trim(now);
    const live = entries.length - head;
    const oldest = live ? entries[head].t : now;
    const out = [];
    out.push('==== CORV GCS debug log ====');
    out.push(`Written     ${stamp(now)}  (session started ${stamp(sessionStart)}, up ${fmtDuration(now - sessionStart)})`);
    out.push(`Covers      last ${WINDOW_MS / 60000} min: ${live} lines from ${clock(oldest)}, ${mb(bytes)} MB`
        + (trimmedBySize ? `; ${trimmedBySize} lines cut early by the ${mb(MAX_BYTES)} MB cap` : '')
        + (droppedTotal ? `; ${droppedTotal} dropped by the flood guard` : ''));
    for (const [k, v] of header) out.push(`${k.padEnd(11)} ${v.replace(/\n/g, '\n            ')}`);
    if (oldIssues.length) {
        out.push('');
        out.push(`---- Errors and warnings older than the window (last ${OLD_ISSUES}) ----`);
        for (const l of oldIssues) out.push(l);
    }
    out.push('');
    out.push('---- Log (oldest first; E error, W warning, I info, D debug; main = main process, rend = window) ----');
    for (let i = head; i < entries.length; i++) out.push(render(entries[i]));
    out.push('');
    return out.join('\n');
}

function fmtDuration(ms) {
    const s = Math.floor(ms / 1000);
    const h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60);
    return h ? `${h}h${pad(m)}m` : `${m}m${pad(s % 60)}s`;
}

async function flush() {
    if (!dirty || writing) return;
    writing = true;
    dirty = false;
    try {
        const text = buildText();
        const file = filePath();
        const tmp = `${file}.tmp`;
        await fs.promises.writeFile(tmp, text);
        try {
            await fs.promises.rename(tmp, file);
        } catch (e) {
            // Windows refuses the rename while another program holds the file open
            await fs.promises.writeFile(file, text);
            fs.promises.unlink(tmp).catch(() => {});
        }
    } catch (e) {
        if (throttle('log-write-fail', 60000) >= 0) raw.error(`[log] cannot write the debug log: ${e.message}`);
    } finally {
        writing = false;
    }
}

function flushSoon() {
    if (soonTimer || !started) return;
    soonTimer = setTimeout(() => { soonTimer = null; flush(); }, FLUSH_SOON_MS);
}

/** Synchronous write: crash and quit paths, where an await may never resume. */
function flushSync() {
    if (!started) return;
    try {
        fs.writeFileSync(filePath(), buildText());
        dirty = false;
    } catch (e) {
        raw.error(`[log] cannot write the debug log: ${e.message}`);
    }
}

// ── Console hook ─────────────────────────────────────────────────────────────

const INSPECT = { depth: 3, maxArrayLength: 20, maxStringLength: 2000, breakLength: Infinity };

function hookConsole() {
    const levels = { log: 'I', info: 'I', warn: 'W', error: 'E', debug: 'D' };
    for (const [name, lvl] of Object.entries(levels)) {
        const orig = console[name];
        raw[name] = orig.bind(console);
        console[name] = (...args) => {
            try { orig.apply(console, args); } catch (_) { /* closed stdout */ }
            let msg;
            try { msg = util.formatWithOptions(INSPECT, ...args); } catch (_) { msg = '[unformattable]'; }
            write(lvl, 'main', msg);
        };
    }
}

// Chromium console levels: strings since Electron 35, numbers before
// (0 verbose, 1 info, 2 warning, 3 error).
const RENDER_LEVELS = { debug: 'D', info: 'I', warning: 'W', error: 'E', 0: 'D', 1: 'I', 2: 'W', 3: 'E' };

function shortSource(sourceId, line) {
    if (!sourceId) return '';
    const name = sourceId.split(/[?#]/)[0].split('/').pop();
    return line ? `${name}:${line}` : name;
}

// ── System description ───────────────────────────────────────────────────────

function describeSystem() {
    const cpus = os.cpus() || [];
    setHeader('App', `${app.getName()} ${app.getVersion()}${app.isPackaged ? '' : ' (development)'}, `
        + `electron ${process.versions.electron}, chrome ${process.versions.chrome}, node ${process.versions.node}, pid ${process.pid}`);
    setHeader('System', `${process.platform} ${process.arch} ${os.release()} (${os.version ? os.version() : ''}), `
        + `${cpus.length} x ${cpus[0] ? cpus[0].model.trim() : 'cpu'}, RAM ${(os.totalmem() / 1073741824).toFixed(1)} GB, `
        + `locale ${app.getLocale() || '?'}`);
    setHeader('Paths', `exe ${app.getPath('exe')}\nlog ${filePath()}`);
}

async function describeGpu() {
    try {
        const status = app.getGPUFeatureStatus();
        const feat = Object.entries(status).map(([k, v]) => `${k}=${v}`).join(' ');
        const info = await app.getGPUInfo('basic');
        const devs = (info.gpuDevice || []).map(d =>
            `${d.active ? '*' : ' '}${(d.vendorId || 0).toString(16)}:${(d.deviceId || 0).toString(16)}`
            + `${d.deviceString ? ' ' + d.deviceString : ''}${d.driverVersion ? ' drv ' + d.driverVersion : ''}`).join('; ');
        setHeader('GPU', `${devs || 'unknown'}\n${feat}`);
        console.log(`[sys] GPU ${devs || 'unknown'} | ${feat}`);
        if (/software|disabled/.test(status.webgl2 || '') || /software|disabled/.test(status.gpu_compositing || '')) {
            console.warn('[sys] WebGL2 or GPU compositing is not hardware accelerated: the 3D view will be slow or blank');
        }
    } catch (e) {
        console.warn(`[sys] GPU info unavailable: ${e.message}`);
    }
}

function describeDisplays() {
    try {
        const { screen } = require('electron');
        const primary = screen.getPrimaryDisplay().id;
        const list = screen.getAllDisplays().map(d =>
            `${d.size.width}x${d.size.height}@${d.scaleFactor}${d.id === primary ? ' primary' : ''}${d.internal ? ' internal' : ''}`).join(', ');
        setHeader('Displays', list);
        return list;
    } catch (e) {
        return '?';
    }
}

// ── Resources ────────────────────────────────────────────────────────────────

let loopLagMax = 0;

function startLoopProbe() {
    let expected = Date.now() + LOOP_PROBE_MS;
    setInterval(() => {
        const now = Date.now();
        const lag = now - expected;
        expected = now + LOOP_PROBE_MS;
        if (lag > loopLagMax) loopLagMax = lag;
        if (lag > 1000) console.warn(`[perf] main process event loop blocked for ${lag} ms`);
    }, LOOP_PROBE_MS).unref();
}

function statusLine() {
    const byType = {};
    try {
        for (const m of app.getAppMetrics()) {
            const k = m.type === 'Browser' ? 'main' : m.type === 'Tab' ? 'window' : m.type === 'GPU' ? 'gpu' : 'other';
            const b = byType[k] || (byType[k] = { cpu: 0, mem: 0, n: 0 });
            b.cpu += (m.cpu && m.cpu.percentCPUUsage) || 0;
            b.mem += ((m.memory && m.memory.workingSetSize) || 0) / 1024;
            b.n++;
        }
    } catch (_) { /* before ready */ }
    const order = ['main', 'window', 'gpu', 'other'].filter(k => byType[k]);
    const cpu = order.map(k => `${k} ${byType[k].cpu.toFixed(0)}%`).join(' ');
    const mem = order.map(k => `${k} ${byType[k].mem.toFixed(0)}`).join(' ');
    const heap = process.memoryUsage().heapUsed;
    const s = `cpu ${cpu} | mem MB ${mem} | main heap ${mb(heap)} MB | free RAM ${(os.freemem() / 1073741824).toFixed(1)}/${(os.totalmem() / 1073741824).toFixed(1)} GB | main loop lag max ${loopLagMax} ms`;
    loopLagMax = 0;
    return s;
}

function startStatus() {
    setInterval(() => {
        const s = statusLine();
        setHeader('Resources', `${s}  (at ${clock(Date.now())})`);
        console.log(`[sys] ${s}`);
    }, STATUS_MS).unref();
}

// ── Electron events ──────────────────────────────────────────────────────────

function watchApp() {
    process.on('uncaughtExceptionMonitor', (err, origin) => {
        write('E', 'main', `[crash] ${origin} in the main process: ${err && err.stack ? err.stack : err}`);
        flushSync();
    });
    process.on('unhandledRejection', (reason) => {
        console.error(`[crash] unhandled promise rejection in the main process: ${reason && reason.stack ? reason.stack : reason}`);
    });
    process.on('warning', (w) => console.warn(`[node] ${w.name}: ${w.message}`));
    process.on('exit', (code) => { write('I', 'main', `[app] exit, code ${code}`); flushSync(); });

    app.on('render-process-gone', (_e, wc, d) => {
        console.error(`[crash] window renderer gone: reason=${d.reason} exitCode=${d.exitCode}`);
        flushSync();
    });
    app.on('child-process-gone', (_e, d) => {
        const msg = `[crash] ${d.type} process gone: reason=${d.reason} exitCode=${d.exitCode}${d.serviceName ? ' service=' + d.serviceName : ''}${d.name ? ' name=' + d.name : ''}`;
        if (d.reason === 'clean-exit') console.log(msg); else console.error(msg);
    });
    app.on('before-quit', () => console.log('[app] quit requested'));
    app.on('will-quit', () => { console.log('[app] quitting'); flushSync(); });

    app.whenReady().then(() => {
        console.log(`[app] ready (${Date.now() - sessionStart} ms after start)`);
        describeSystem();   // the locale is only known now
        // Read before the GPU process is up, the feature status says "disabled" for everything
        setTimeout(describeGpu, GPU_DESCRIBE_DELAY_MS);
        const displays = describeDisplays();
        console.log(`[sys] displays: ${displays}`);
        try {
            const { screen, powerMonitor } = require('electron');
            for (const ev of ['display-added', 'display-removed', 'display-metrics-changed']) {
                screen.on(ev, () => console.log(`[sys] ${ev}: ${describeDisplays()}`));
            }
            for (const ev of ['suspend', 'resume', 'on-ac', 'on-battery', 'lock-screen', 'unlock-screen', 'shutdown']) {
                powerMonitor.on(ev, () => console.log(`[power] ${ev}`));
            }
            powerMonitor.on('thermal-state-change', (s) => console.warn(`[power] thermal state: ${s && s.state !== undefined ? s.state : s}`));
            powerMonitor.on('speed-limit-change', (s) => console.warn(`[power] CPU speed limit: ${s && s.limit !== undefined ? s.limit : s}%`));
            setHeader('Power', powerMonitor.isOnBatteryPower() ? 'on battery' : 'on AC');
            powerMonitor.on('on-ac', () => setHeader('Power', 'on AC'));
            powerMonitor.on('on-battery', () => setHeader('Power', 'on battery'));
        } catch (e) {
            console.warn(`[sys] screen/power monitor unavailable: ${e.message}`);
        }
    });
}

/**
 * Window events and the renderer's console. Replaces a plain console forward:
 * the renderer's lines go into the window as 'rend', with their source file
 * for warnings and errors.
 */
function attachWindow(win) {
    const wc = win.webContents;
    wc.on('console-message', (event) => {
        const { level, message, lineNumber: line, sourceId: source } = event;
        const lvl = RENDER_LEVELS[level] || 'I';
        const where = shortSource(source, line);
        const text = `[renderer]${where ? ' ' + where : ''} ${message}`;
        if (lvl === 'E') raw.error(text);
        else if (lvl === 'W') raw.warn(text);
        else if (lvl === 'D') raw.debug(text);
        else raw.log(text);
        write(lvl, 'rend', (lvl === 'E' || lvl === 'W') && where ? `${message}  (${where})` : message);
    });
    wc.on('did-start-loading', () => console.log('[window] loading'));
    wc.on('did-finish-load', () => console.log(`[window] loaded (${Date.now() - sessionStart} ms after start)`));
    wc.on('did-fail-load', (_e, code, desc, url, isMain) => console.error(`[window] load failed: ${code} ${desc} ${url}${isMain ? '' : ' (subframe)'}`));
    wc.on('preload-error', (_e, p, err) => console.error(`[window] preload error in ${p}: ${err && err.stack ? err.stack : err}`));
    wc.on('unresponsive', () => { console.error('[window] UNRESPONSIVE: the page stopped answering'); flushSync(); });
    wc.on('responsive', () => console.warn('[window] responsive again'));
    win.on('minimize', () => console.log('[window] minimized'));
    win.on('restore', () => console.log('[window] restored'));
    win.on('maximize', () => console.log('[window] maximized'));
    win.on('unmaximize', () => console.log('[window] unmaximized'));
    win.on('enter-full-screen', () => console.log('[window] full screen'));
    win.on('leave-full-screen', () => console.log('[window] left full screen'));
    win.on('resized', () => {
        const [w, h] = win.getSize();
        console.log(`[window] resized to ${w}x${h}`);
    });
    win.on('closed', () => console.log('[window] closed'));
}

// ── IPC ──────────────────────────────────────────────────────────────────────

// Polled or high-rate: their calls are not logged, their failures still are.
const QUIET_IPC = new Set([
    'adsb-fetch', 'applog-info', 'fpv-status', 'lidar-get-status', 'msp-status', 'rtk-get-stats',
    'sitl-status', 'sitl-get-options', 'telfwd-feed-state', 'telfwd-get-outputs', 'topography-exists',
    'topography-load-one', 'topography-save', 'serial-list-ports', 'rtk-list-ports', 'telfwd-list-ports',
    'mavlink-send-command', 'mavlink-send-message', 'log-replay-seek', 'missions-list', 'store-get-root',
    'tlog-get-logs-dir', 'lidar-list-interfaces', 'rtk-get-type-names', 'lte-key-available', 'lte-key-load'
]);
// Arguments never written, whatever their shape.
const SECRET_IPC = new Set(['lte-key-save']);
const SECRET_FIELD = /key|pass|secret|token|auth|cred|user/i;

/** One-line, size-capped, secret-free picture of an IPC argument. */
function brief(v, depth = 0) {
    if (v === null || v === undefined) return String(v);
    if (typeof v === 'string') return v.length > 80 ? `${JSON.stringify(v.slice(0, 80))}…` : JSON.stringify(v);
    if (typeof v === 'number' || typeof v === 'boolean') return String(v);
    if (v instanceof ArrayBuffer || ArrayBuffer.isView(v)) return `<${v.byteLength} bytes>`;
    if (Array.isArray(v)) return depth > 0 || v.length > 8 ? `[${v.length} items]` : `[${v.map(x => brief(x, depth + 1)).join(', ')}]`;
    if (typeof v === 'object') {
        if (depth > 1) return '{…}';
        const keys = Object.keys(v);
        const parts = keys.slice(0, 14).map(k => `${k}: ${SECRET_FIELD.test(k) ? '***' : brief(v[k], depth + 1)}`);
        return `{${parts.join(', ')}${keys.length > 14 ? ', …' : ''}}`;
    }
    return typeof v;
}

/**
 * Every ipcMain.handle registered after init() goes through here: the call
 * with its arguments, slow answers, answers that report { success: false },
 * and thrown errors — which would otherwise only reach the renderer.
 */
function wrapIpc() {
    const orig = ipcMain.handle.bind(ipcMain);
    ipcMain.handle = (channel, fn) => orig(channel, async (event, ...args) => {
        const t0 = Date.now();
        const quiet = QUIET_IPC.has(channel);
        if (!quiet) write('D', 'main', `[ipc] ${channel}(${SECRET_IPC.has(channel) ? '***' : args.map(a => brief(a)).join(', ')})`);
        try {
            const r = await fn(event, ...args);
            const ms = Date.now() - t0;
            if (r && typeof r === 'object' && r.success === false) {
                console.warn(`[ipc] ${channel} → not done after ${ms} ms: ${r.error || r.message || 'no reason given'}`);
            } else if (ms > 5000) {
                console.warn(`[ipc] ${channel} took ${(ms / 1000).toFixed(1)} s`);
            } else if (!quiet && ms > 200) {
                write('D', 'main', `[ipc] ${channel} done in ${ms} ms`);
            }
            return r;
        } catch (e) {
            console.error(`[ipc] ${channel} failed after ${Date.now() - t0} ms: ${e && e.message ? e.message : e}`);
            throw e;
        }
    });
}

function initIpc() {
    ipcMain.handle('applog-info', () => ({
        file: filePath(),
        previous: prevPath(),
        dir: getDir(),
        windowMin: WINDOW_MS / 60000,
        maxMB: MAX_BYTES / 1048576
    }));

    ipcMain.handle('applog-reveal', () => {
        flushSync();
        shell.showItemInFolder(filePath());
        return filePath();
    });

    // One file for support: this session's window, then the previous session's
    ipcMain.handle('applog-save', async (e) => {
        flushSync();
        const d = new Date();
        const name = `corv-gcs-debug-${d.getFullYear()}${pad(d.getMonth() + 1)}${pad(d.getDate())}-${pad(d.getHours())}${pad(d.getMinutes())}${pad(d.getSeconds())}.log`;
        const res = await dialog.showSaveDialog(BrowserWindow.fromWebContents(e.sender), {
            title: 'Save debug log',
            defaultPath: path.join(app.getPath('desktop'), name),
            filters: [{ name: 'Log', extensions: ['log', 'txt'] }]
        });
        if (res.canceled || !res.filePath) return { saved: false };
        let text = buildText();
        try {
            text += `\n\n==== PREVIOUS SESSION (${PREV_NAME}) ====\n\n${fs.readFileSync(prevPath(), 'utf8')}`;
        } catch (_) { /* first session */ }
        await fs.promises.writeFile(res.filePath, text);
        console.log(`[log] debug log saved to ${res.filePath} (${mb(text.length)} MB)`);
        return { saved: true, filePath: res.filePath };
    });

    ipcMain.on('applog-header', (_e, key, value) => setHeader(key, value));
}

// ── Start ────────────────────────────────────────────────────────────────────

/** Call first thing in main.js: everything logged before it is not captured. */
function init() {
    if (started) return;
    hookConsole();
    try {
        if (fs.existsSync(filePath())) fs.renameSync(filePath(), prevPath());
    } catch (e) {
        raw.warn(`[log] could not keep the previous session's log: ${e.message}`);
    }
    started = true;
    describeSystem();
    watchApp();
    wrapIpc();
    initIpc();
    startLoopProbe();
    startStatus();
    setInterval(flush, FLUSH_MS).unref();
    console.log(`[log] debug log: ${filePath()} (last ${WINDOW_MS / 60000} min, max ${mb(MAX_BYTES)} MB, rewritten every ${FLUSH_MS / 1000} s)`);
    console.log(`[app] ${app.getName()} ${app.getVersion()} starting, args: ${process.argv.slice(1).join(' ') || '-'}`);
}

module.exports = { init, attachWindow, write, setHeader, throttle, flushSync, raw };
