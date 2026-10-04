/**
 * DebugLog.js - What the window did, for the rolling debug log (app-log.js)
 *
 * The main process already writes every console line of this window into the
 * last-five-minutes file. This module adds what nobody prints:
 *   - a snapshot every SNAPSHOT_MS: page, frame rate, JS heap, WebGL load, and
 *     the vehicle state as the operator sees it (mode name, position, battery…)
 *   - state changes, polled at CHANGE_MS: connection, mode, arming, home,
 *     mission, parameters, joystick, replay, a link that went silent
 *   - the operator's trail: clicks, changed fields and selects, page switches
 *   - the browser side of trouble: long tasks, lost WebGL context, failed
 *     resource loads, going offline, the window hidden or shown
 * and keeps two header lines of the file current ('Window', 'State').
 */

import { STATE } from './state.js';

const SNAPSHOT_MS = 10000;
const CHANGE_MS = 500;
const LONG_TASK_MS = 250;        // main-thread tasks worth a line
const LINK_SILENT_MS = 3000;     // connected, nothing received for this long

let providers = {};
const fps = { min: Infinity, sum: 0, n: 0 };
const prev = {};
let longTasks = { n: 0, max: 0, total: 0 };
const quiet = new Map();

/** At most one line per `ms` for `key`; returns -1 to stay quiet, else how many were skipped. */
function throttle(key, ms) {
    const now = Date.now();
    const t = quiet.get(key);
    if (t && now - t.at < ms) { t.skipped++; return -1; }
    const skipped = t ? t.skipped : 0;
    quiet.set(key, { at: now, skipped: 0 });
    return skipped;
}

function setHeader(key, value) {
    try { window.appLog?.setHeader(key, value); } catch (e) { /* preload without appLog */ }
}

const f = (v, d = 1) => (Number.isFinite(v) ? v.toFixed(d) : '-');

/** Called once a second by the frame counter in main.js. */
export function noteFps(value) {
    if (document.hidden) return;   // a hidden window does not draw: not a slow one
    fps.min = Math.min(fps.min, value);
    fps.sum += value;
    fps.n++;
}

// ── Snapshot ─────────────────────────────────────────────────────────────────

function glInfo() {
    const r = providers.getRenderer?.();
    if (!r || !r.info) return '';
    const { memory, render, programs } = r.info;
    const frame = providers.getRenderPerformanceStats?.();
    const detail = frame ? ` passes ${frame.passes} dpr ${frame.pixelRatio} submit95 ${frame.submitP95Ms.toFixed(1)}ms` : '';
    return `gl calls ${render.calls} tris ${(render.triangles / 1e6).toFixed(2)}M geo ${memory.geometries} tex ${memory.textures} prog ${programs ? programs.length : '-'}${detail}`;
}

function heapInfo() {
    const m = performance.memory;
    return m ? `heap ${(m.usedJSHeapSize / 1048576).toFixed(0)}/${(m.jsHeapSizeLimit / 1048576).toFixed(0)} MB` : '';
}

function vehicleLine() {
    if (!STATE.connected && STATE.mode !== 'REPLAY') return `no vehicle (${STATE.mode === 'LIVE' ? 'demo' : STATE.mode})`;
    const hbAge = STATE.lastHeartbeatTime ? ((Date.now() - STATE.lastHeartbeatTime) / 1000).toFixed(1) : '-';
    const agl = STATE.terrainHeight !== null ? STATE.rawAlt + STATE.offsetAlt - STATE.terrainHeight : NaN;
    return [
        `${STATE.mode} ${STATE.connectionType} ${STATE.linkType} ${f(STATE.linkKbps)} kbps hb ${hbAge} s`,
        `${STATE.flightMode} ${STATE.armed ? 'ARMED' : 'disarmed'}`,
        `pos ${f(STATE.lat, 6)},${f(STATE.lon, 6)} alt ${f(STATE.rawAlt)} m agl ${f(agl)} terrain ${f(STATE.terrainHeight)}`,
        `as ${f(STATE.as)} gs ${f(STATE.gs)} vs ${f(STATE.vs)} hdg ${f(STATE.yaw * 180 / Math.PI, 0)}`,
        `bat ${f(STATE.batteryVoltage, 2)} V ${f(STATE.batteryCurrent)} A ${STATE.batteryRemaining}%`,
        `gps fix ${STATE.gpsFix} sats ${STATE.gpsNumSat} hdop ${f(STATE.gpsHdop)}`,
        STATE.rssi !== null ? `rssi ${STATE.rssi}/${STATE.remRssi}` : '',
        STATE.homeLat !== null ? 'home set' : 'no home',
        `mission ${STATE.missionItems.length} items, current ${STATE.missionCurrentSeq}`,
        STATE.parameterCount ? `params ${STATE.parametersReceived}/${STATE.parameterCount}` : '',
        STATE.terrainFeedSent || STATE.terrainPending ? `terrain feed sent ${STATE.terrainFeedSent} err ${STATE.terrainFeedErrors} pending ${STATE.terrainPending}` : '',
        STATE.joystickConnected ? `joystick ${STATE.rcOverrideActive ? 'OVERRIDE' : 'connected'}` : '',
        STATE.traffic?.length ? `traffic ${STATE.traffic.length}` : ''
    ].filter(Boolean).join(' | ');
}

function snapshot() {
    const avg = fps.n ? fps.sum / fps.n : NaN;
    const fpsText = fps.n ? `fps avg ${avg.toFixed(0)} min ${fps.min}` : 'fps - (hidden)';
    fps.min = Infinity; fps.sum = 0; fps.n = 0;
    const lt = longTasks.n ? `long tasks ${longTasks.n} (max ${longTasks.max.toFixed(0)} ms, ${longTasks.total.toFixed(0)} ms total)` : '';
    longTasks = { n: 0, max: 0, total: 0 };

    const win = [`page ${providers.getTab?.() || '?'}`, fpsText, heapInfo(), glInfo(), lt,
        document.hidden ? 'HIDDEN' : '', `${window.innerWidth}x${window.innerHeight}@${window.devicePixelRatio}`]
        .filter(Boolean).join(' | ');
    const veh = vehicleLine();
    console.log(`[snap] ${win}`);
    console.log(`[state] ${veh}`);
    setHeader('Window', `${win}  (at ${new Date().toLocaleTimeString()})`);
    setHeader('State', `${veh}  (at ${new Date().toLocaleTimeString()})`);

    if (Number.isFinite(avg) && avg < 15 && throttle('low-fps', 60000) >= 0) {
        console.warn(`[perf] low frame rate: ${avg.toFixed(0)} fps average over the last ${SNAPSHOT_MS / 1000} s`);
    }
}

// ── Change detection ─────────────────────────────────────────────────────────

function changed(key, value) {
    if (prev[key] === value) return false;
    const first = !(key in prev);
    prev[key] = value;
    return !first;
}

function watchState() {
    const s = STATE;
    if (changed('connected', s.connected)) console.log(`[state] ${s.connected ? 'CONNECTED' : 'DISCONNECTED'} (${s.connectionType})`);
    if (changed('connectionType', s.connectionType)) console.log(`[state] connection type ${s.connectionType}`);
    if (changed('mode', s.mode)) console.log(`[state] app mode ${s.mode}`);
    if (changed('flightMode', s.flightMode)) console.log(`[state] flight mode ${s.flightMode} (${s.flightModeNum})`);
    if (changed('armed', s.armed)) console.warn(`[state] ${s.armed ? 'ARMED' : 'DISARMED'} in ${s.flightMode}`);
    if (changed('vehicleType', s.vehicleType)) console.log(`[state] vehicle type ${s.vehicleType} autopilot ${s.autopilotType}`);
    if (changed('firmware', s.firmwareVersion)) console.log(`[state] firmware ${s.firmwareVersion}`);
    if (changed('home', s.homeLat === null ? null : `${f(s.homeLat, 6)},${f(s.homeLon, 6)}`)) {
        console.log(s.homeLat === null ? '[state] home cleared' : `[state] home ${f(s.homeLat, 6)},${f(s.homeLon, 6)} alt ${f(s.homeAlt)}`);
    }
    if (changed('missionLen', s.missionItems.length)) console.log(`[state] mission now ${s.missionItems.length} items`);
    const paramsDone = s.parameterCount > 0 && s.parametersReceived >= s.parameterCount;
    if (changed('paramsDone', paramsDone) && paramsDone) console.log(`[state] parameters loaded: ${s.parametersReceived}`);
    if (changed('joystick', s.joystickConnected)) console.log(`[state] joystick ${s.joystickConnected ? 'connected' : 'disconnected'}`);
    if (changed('rcOverride', s.rcOverrideActive)) console.warn(`[state] RC override ${s.rcOverrideActive ? 'ACTIVE' : 'off'}`);
    if (changed('gpsFix', s.gpsFix)) console.log(`[state] GPS fix ${s.gpsFix}, ${s.gpsNumSat} sats`);

    // A link that is up but carries nothing: wrong baud rate, wrong port, radio out of range
    const silent = s.connected && s.lastLinkStatsTime && s.linkKbps === 0
        && Date.now() - (prev.lastRxAt || Date.now()) > LINK_SILENT_MS;
    if (s.linkKbps > 0 || !s.connected) prev.lastRxAt = Date.now();
    if (changed('linkSilent', !!silent)) {
        if (silent) console.warn(`[state] link ${s.linkType} up but nothing received for ${LINK_SILENT_MS / 1000} s (wrong baud rate or port, radio out of range?)`);
        else console.log('[state] link data flowing again');
    }
}

// ── Operator trail ───────────────────────────────────────────────────────────

const SECRET = /key|pass|secret|token|auth|user/i;

function describeEl(el) {
    const id = el.id ? `#${el.id}` : '';
    const cls = !id && el.classList?.length ? `.${[...el.classList].slice(0, 2).join('.')}` : '';
    const data = el.dataset?.section || el.dataset?.tab || el.dataset?.page || el.dataset?.action || '';
    const text = (el.getAttribute?.('title') && !el.textContent.trim() ? el.getAttribute('title') : el.textContent || el.value || '')
        .replace(/\s+/g, ' ').trim().slice(0, 40);
    return `${el.tagName.toLowerCase()}${id}${cls}${data ? `[${data}]` : ''}${text ? ` "${text}"` : ''}`;
}

function isSecretField(el) {
    return el.type === 'password' || SECRET.test(el.id || '') || SECRET.test(el.name || '');
}

function watchUi() {
    document.addEventListener('click', (e) => {
        const el = e.target.closest?.('button, a, [role="button"], [onclick], [data-section], [data-tab], select, input[type="checkbox"], input[type="radio"], .tab, .menu-item, label');
        if (!el || el.tagName === 'SELECT') return;   // a select reports on change
        if (el.tagName === 'INPUT') return;           // checkboxes report on change
        console.log(`[ui] click ${describeEl(el)}`);
    }, true);

    document.addEventListener('change', (e) => {
        const el = e.target;
        if (!el || !el.tagName) return;
        let value;
        if (el.type === 'checkbox' || el.type === 'radio') value = el.checked ? 'on' : 'off';
        else if (isSecretField(el)) value = '***';
        else if (el.type === 'file') value = `${el.files?.length || 0} file(s)`;
        else value = String(el.value ?? '').slice(0, 60);
        const id = el.id ? `#${el.id}` : el.name ? `[name=${el.name}]` : '';
        console.log(`[ui] change ${el.tagName.toLowerCase()}${id}${el.type && el.tagName === 'INPUT' ? `(${el.type})` : ''} = ${value}`);
    }, true);

    // Pages: polled, so every way of switching (click, shortcut, code) is seen
    setInterval(() => {
        const tab = providers.getTab?.();
        if (changed('tab', tab)) console.log(`[ui] page ${tab}`);
    }, CHANGE_MS);
}

// ── Browser-side trouble ─────────────────────────────────────────────────────

function watchBrowser() {
    document.addEventListener('visibilitychange', () => console.log(`[window] ${document.hidden ? 'hidden' : 'visible'}`));
    window.addEventListener('focus', () => { if (throttle('focus', 5000) >= 0) console.log('[window] focus'); });
    window.addEventListener('blur', () => { if (throttle('blur', 5000) >= 0) console.log('[window] blur'); });
    window.addEventListener('online', () => console.log('[net] online'));
    window.addEventListener('offline', () => console.warn('[net] offline'));
    console.log(`[net] ${navigator.onLine ? 'online' : 'OFFLINE'} at start`);

    // Failed <img>/<script>/<link> loads do not bubble: capture them
    window.addEventListener('error', (e) => {
        const t = e.target;
        if (!t || t === window || !t.tagName) return;
        const src = (t.src || t.href || '').slice(0, 160);
        const skipped = throttle(`res-${t.tagName}`, 10000);
        if (skipped >= 0) console.warn(`[res] ${t.tagName.toLowerCase()} failed to load: ${src}${skipped ? ` (+${skipped} more)` : ''}`);
    }, true);

    try {
        new PerformanceObserver((list) => {
            for (const entry of list.getEntries()) {
                if (entry.duration < LONG_TASK_MS) continue;
                longTasks.n++;
                longTasks.total += entry.duration;
                longTasks.max = Math.max(longTasks.max, entry.duration);
                if (entry.duration > 1000) console.warn(`[perf] window blocked for ${entry.duration.toFixed(0)} ms`);
            }
        }).observe({ entryTypes: ['longtask'] });
    } catch (e) { /* no long-task API */ }

    const r = providers.getRenderer?.();
    const canvas = r?.domElement;
    if (canvas) {
        canvas.addEventListener('webglcontextlost', () => console.error('[gl] WebGL context LOST: the 3D view stops until the GPU recovers'));
        canvas.addEventListener('webglcontextrestored', () => console.warn('[gl] WebGL context restored'));
        try {
            const gl = r.getContext();
            const ext = gl.getExtension('WEBGL_debug_renderer_info');
            const gpu = ext ? gl.getParameter(ext.UNMASKED_RENDERER_WEBGL) : gl.getParameter(gl.RENDERER);
            const desc = `${gpu} | WebGL2 three r${THREE.REVISION} max texture ${r.capabilities?.maxTextureSize}`;
            if (/swiftshader|basic render|llvmpipe|software/i.test(gpu)) console.warn(`[gl] SOFTWARE rendering: ${desc}`);
            else console.log(`[gl] ${desc}`);
            setHeader('WebGL', desc);
        } catch (e) { /* context not ready */ }
    }
}

// ── App events ───────────────────────────────────────────────────────────────

function watchAppEvents() {
    window.addEventListener('vehicleConnected', (e) => console.log(`[state] vehicle link opened (${e.detail?.type})`));
    window.addEventListener('mavlinkConnectionState', (e) => console.log(`[state] link state ${e.detail?.state}`));
    window.addEventListener('connectionLost', () => console.warn('[net] map tiles: internet connection lost, cache only'));
    window.addEventListener('relNavChanged', () => console.log('[state] relative navigation toggled'));
    window.addEventListener('platformChanged', (e) => console.log(`[state] platform ${e.detail?.platform}`));
    window.addEventListener('languageChanged', (e) => console.log(`[ui] language ${e.detail?.lang || e.detail?.language || ''}`));
    window.addEventListener('missionUpdated', () => {
        const skipped = throttle('missionUpdated', 5000);
        if (skipped >= 0) console.log(`[mission] route updated: ${STATE.missionItems.length} items${skipped ? ` (+${skipped} more edits)` : ''}`);
    });
}

/**
 * @param {object} p
 * @param {() => string} p.getTab        current page name
 * @param {() => object} p.getRenderer   THREE.WebGLRenderer
 */
export function initDebugLog(p = {}) {
    providers = p;
    const ua = navigator.userAgent.match(/Chrome\/[\d.]+/)?.[0] || '';
    console.log(`[window] debug log on: ${ua}, ${navigator.hardwareConcurrency} threads, ${navigator.deviceMemory || '?'} GB, screen ${screen.width}x${screen.height}@${window.devicePixelRatio}, lang ${navigator.language}`);
    watchUi();
    watchBrowser();
    watchAppEvents();
    watchState();
    setInterval(watchState, CHANGE_MS);
    setInterval(snapshot, SNAPSHOT_MS);
}
