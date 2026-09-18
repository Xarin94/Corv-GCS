/**
 * LidarController.js - Livox Mid-360 point cloud: settings + flight strip
 *
 * Wires the LIDAR section (SETUP → TOOLS) — connection, mount attitude
 * relative to the autopilot IMU, filters, navigation-quality gate, display,
 * recording) to the main-process client, and the strip on the flight screen
 * (state LED, point count, CLEAR MAP / SAVE) that is only shown while the
 * cloud is enabled. Points and origin events go straight to LidarCloud.
 *
 * Settings persist in localStorage under 'lidar-config'; the ENABLE state
 * does not — a LiDAR link is started deliberately, per flight.
 *
 * In the demo flight (no link) enabling the cloud starts LidarDemo instead
 * of the network client: a synthetic scan of the real SRTM terrain under the
 * demo aircraft, through the same cloud, strip and CLEAR MAP.
 */

import {
    setLidarOrigin, appendLidarPoints, appendLiveLidarPoints, clearLidarCloud, clearLiveLidarPoints,
    setLidarCloudVisible, setLidarColorMode, setLidarPointSize, setLidarMaxPoints, setLidarOverTerrain,
    getLidarPointCount
} from '../lidar/LidarCloud.js';
import { startLidarDemo, stopLidarDemo, resetLidarDemo, isLidarDemoRunning, configureLidarDemo } from '../lidar/LidarDemo.js';
import { isDemoMode } from '../core/state.js';
import { pushHudMessage } from '../hud/HUDRenderer.js';
import { setNavDot } from './TabController.js';

const STORAGE_KEY = 'lidar-config';

// id → config key, with the parser used on read.
const FIELDS = {
    'lidar-ip':          { key: 'lidarIp',   parse: v => String(v).trim() },
    'lidar-host':        { key: 'hostIp',    parse: v => String(v) },
    'lidar-datatype':    { key: 'dataType',  parse: v => parseInt(v, 10) || 1 },
    'lidar-mount-roll':  { key: 'mountRoll', parse: num(0) },
    'lidar-mount-pitch': { key: 'mountPitch', parse: num(0) },
    'lidar-mount-yaw':   { key: 'mountYaw',  parse: num(0) },
    'lidar-lever-x':     { key: 'leverX',    parse: num(0) },
    'lidar-lever-y':     { key: 'leverY',    parse: num(0) },
    'lidar-lever-z':     { key: 'leverZ',    parse: num(0) },
    'lidar-lag':         { key: 'lagMs',     parse: num(0) },
    'lidar-min-range':   { key: 'minRange',  parse: num(2.5) },
    'lidar-max-range':   { key: 'maxRange',  parse: num(70) },
    'lidar-voxel':       { key: 'voxel',     parse: num(0.25) },
    'lidar-max-points':  { key: 'maxPoints', parse: num(3000000) },
    'lidar-drop-noise':  { key: 'dropNoise', parse: v => !!v, checkbox: true },
    'lidar-min-fix':     { key: 'minFix',    parse: v => parseInt(v, 10) || 3 },
    'lidar-min-sats':    { key: 'minSats',   parse: num(8) },
    'lidar-max-hdop':    { key: 'maxHdop',   parse: num(2) },
    'lidar-require-ekf': { key: 'requireEkf', parse: v => !!v, checkbox: true },
    'lidar-live-ttl':    { key: 'liveSeconds', parse: num(3) },
    'lidar-color-mode':  { key: 'colorMode', parse: v => String(v), local: true },
    'lidar-point-size':  { key: 'pointSize', parse: num(2), local: true },
    'lidar-over-terrain': { key: 'overTerrain', parse: v => !!v, checkbox: true, local: true },
    'lidar-record-raw':  { key: 'recordRaw', parse: v => !!v, checkbox: true }
};

function num(def) {
    return v => { const n = parseFloat(v); return Number.isFinite(n) ? n : def; };
}

let cfg = {};
let enabled = false;
let lastStatus = null;
let lastGateOk = null;
let els = {};
let demoTimer = null;

function $(id) { return document.getElementById(id); }

// ============== SETTINGS ==============
function loadConfig() {
    try {
        const saved = localStorage.getItem(STORAGE_KEY);
        if (saved) return JSON.parse(saved);
    } catch (_) {}
    return {};
}

function saveConfig() {
    try { localStorage.setItem(STORAGE_KEY, JSON.stringify(cfg)); } catch (_) {}
}

function readInputs() {
    for (const [id, f] of Object.entries(FIELDS)) {
        const el = $(id);
        if (!el) continue;
        cfg[f.key] = f.parse(f.checkbox ? el.checked : el.value);
    }
}

function populateInputs() {
    for (const [id, f] of Object.entries(FIELDS)) {
        const el = $(id);
        if (!el || cfg[f.key] === undefined) continue;
        if (f.checkbox) el.checked = !!cfg[f.key];
        else el.value = cfg[f.key];
    }
}

// Only the keys the main process knows; display-only ones stay local.
function mainConfig() {
    const out = {};
    for (const f of Object.values(FIELDS)) if (!f.local) out[f.key] = cfg[f.key];
    return out;
}

function applyDisplayConfig() {
    setLidarColorMode(cfg.colorMode || 'height');
    setLidarPointSize(cfg.pointSize || 2);
    setLidarMaxPoints(cfg.maxPoints || 3000000);
    setLidarOverTerrain(cfg.overTerrain !== false);
    configureLidarDemo(cfg);
}

// ============== CONNECTION ==============
async function setEnabled(on) {
    if (!window.lidar) return;
    if (on && isDemoMode()) {
        readInputs();
        saveConfig();
        applyDisplayConfig();
        startDemo();
        return;
    }
    if (on) {
        readInputs();
        saveConfig();
        applyDisplayConfig();
        const res = await window.lidar.connect(mainConfig());
        if (!res || !res.success) {
            enabled = false;
            if (els.enable) els.enable.checked = false;
            pushHudMessage(`LIDAR: ${res && res.error ? res.error : 'connection failed'}`, 'error');
            renderStatus({ connected: false, link: 'OFF', error: res && res.error });
            return;
        }
        enabled = true;
        lastGateOk = null;
        setLidarCloudVisible(true);
        els.strip.classList.remove('hidden');
        pushHudMessage(`LIDAR: connecting to ${cfg.lidarIp}`, 'info');
    } else {
        enabled = false;
        if (isLidarDemoRunning()) stopDemo(); else await window.lidar.disconnect();
        setLidarCloudVisible(false);
        clearLiveLidarPoints();
        els.strip.classList.add('hidden');
    }
}

// ============== DEMO (no link) ==============
function startDemo() {
    enabled = true;
    clearLidarCloud();
    resetLidarDemo();
    startLidarDemo();
    setLidarCloudVisible(true);
    els.strip.classList.remove('hidden');
    els.strip.className = 'lidar-strip state-accum';
    els.stripState.textContent = 'DEMO · SYNTHETIC SCAN';
    if (els.statusText) els.statusText.innerHTML = '<span class="ok">DEMO</span> synthetic scan of the SRTM terrain under the demo flight — no LiDAR link.\nConnect a vehicle and enable again for the real sensor.';
    pushHudMessage('LIDAR: demo — synthetic terrain scan', 'info');
    clearInterval(demoTimer);
    demoTimer = setInterval(() => { if (els.stripCount) els.stripCount.textContent = `${fmt(getLidarPointCount())} pts`; }, 250);
}

function stopDemo() {
    clearInterval(demoTimer);
    demoTimer = null;
    stopLidarDemo();
    resetLidarDemo();
    clearLidarCloud();
    if (els.statusText) els.statusText.textContent = 'OFF';
}

async function clearMap() {
    if (!window.lidar) return;
    if (isLidarDemoRunning()) resetLidarDemo(); else await window.lidar.clear();
    clearLidarCloud();
    clearLiveLidarPoints();
    pushHudMessage('LIDAR: map cleared', 'info');
    updateStrip(lastStatus);
}

async function saveMap() {
    if (!window.lidar) return;
    if (isLidarDemoRunning()) { pushHudMessage('LIDAR: demo scan — nothing to save', 'warning'); return; }
    const res = await window.lidar.saveMap();
    if (res && res.success) pushHudMessage(`LIDAR: ${fmt(res.points)} pts saved → ${res.path}`, 'info');
    else pushHudMessage(`LIDAR: save failed — ${res && res.error ? res.error : 'unknown error'}`, 'error');
}

// ============== STATUS ==============
function fmt(n) {
    return (n || 0).toLocaleString('en-US');
}

function renderStatus(st) {
    const el = els.statusText;
    if (!el) return;
    if (!st || !st.connected) {
        el.innerHTML = st && st.error ? `<span class="err">${esc(st.error)}</span>` : 'OFF';
        return;
    }
    const lines = [];
    const linkCls = st.streaming ? 'ok' : 'warn';
    lines.push(`<span class="${linkCls}">${st.link}</span> ${st.model ? esc(st.model) : ''} ${st.sn ? 'S/N ' + esc(st.sn) : ''}`.trim());
    lines.push(`LiDAR ${esc(st.lidarIp)} → host ${esc(st.hostIp || '?')}`);
    lines.push(`${fmt(st.pps)} pkt/s · ${fmt(st.pointsPs)} pts/s · ${fmt(st.kbps)} kbit/s`);
    lines.push(`<span class="${st.gateOk ? 'ok' : 'warn'}">${esc(st.gateReason)}</span> · fix ${st.nav.fix} · ${st.nav.sats} sats · HDOP ${st.nav.hdop.toFixed(1)} · EKF var ${st.nav.ekfPosVar.toFixed(2)}`);
    lines.push(`map ${fmt(st.mapPoints)} / ${fmt(st.maxPoints)} pts · +${fmt(st.acceptedPs)}/s · filtered ${fmt(st.filteredPs)}/s · live ${fmt(st.livePs)}/s`);
    if (st.recording) lines.push(`<span class="ok">REC</span> ${fmt(st.recordedPoints)} pts → ${esc(st.recordPath)}`);
    if (st.error) lines.push(`<span class="err">${esc(st.error)}</span>`);
    el.innerHTML = lines.join('\n');
}

function esc(s) {
    return String(s).replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
}

function updateStrip(st) {
    const strip = els.strip;
    if (!strip || !enabled || isLidarDemoRunning()) return;
    let state = 'search', label = 'SEARCHING';
    if (st) {
        if (st.error && !st.streaming) { state = 'error'; label = 'ERROR'; }
        else if (!st.streaming) { state = 'search'; label = st.link; }
        else if (st.gateOk) { state = 'accum'; label = 'ACCUMULATING'; }
        else if (st.gateReason === 'MAP FULL') { state = 'full'; label = 'MAP FULL'; }
        else { state = 'hold'; label = `LIVE ONLY · ${st.gateReason}`; }
    }
    strip.className = `lidar-strip state-${state}`;
    els.stripState.textContent = label;
    els.stripCount.textContent = `${fmt(getLidarPointCount())} pts`;
}

function onStatus(st) {
    lastStatus = st;
    if (isLidarDemoRunning()) { setNavDot('lidar', true); return; }
    renderStatus(st);
    updateStrip(st);
    setNavDot('lidar', !!(st && st.connected));
    if (!enabled || !st || !st.streaming) return;
    // Announce gate transitions on the HUD, once per change.
    if (lastGateOk !== st.gateOk) {
        if (lastGateOk !== null || st.gateOk) {
            pushHudMessage(st.gateOk ? 'LIDAR: georeferenced — accumulating' : `LIDAR: live only, not kept — ${st.gateReason}`, st.gateOk ? 'info' : 'warning');
        }
        lastGateOk = st.gateOk;
    }
}

// ============== INIT ==============
export function initLidarController() {
    els = {
        enable: $('lidar-enable'),
        strip: $('lidar-strip'),
        stripState: $('lidar-strip-state'),
        stripCount: $('lidar-strip-count'),
        statusText: $('lidar-status-text'),
        host: $('lidar-host')
    };
    if (!els.enable || !els.strip) {
        console.warn('[LiDAR] panel not found, controller disabled');
        return;
    }

    cfg = loadConfig();
    populateInputs();
    readInputs();          // fills defaults from the DOM for keys never saved
    applyDisplayConfig();

    // Host interface list (AUTO + every IPv4 on this machine)
    if (window.lidar && els.host) {
        window.lidar.listInterfaces().then((list) => {
            for (const ni of list || []) {
                const opt = document.createElement('option');
                opt.value = ni.address;
                opt.textContent = `${ni.address} (${ni.name})`;
                els.host.appendChild(opt);
            }
            if (cfg.hostIp) els.host.value = cfg.hostIp;
            if (els.host.value !== cfg.hostIp) els.host.value = 'auto';
        }).catch(() => {});
    }

    // Every input: persist + push to the main process live (mount, filters,
    // gate thresholds take effect on the next packet; IP / format on reconnect).
    for (const [id, f] of Object.entries(FIELDS)) {
        const el = $(id);
        if (!el) continue;
        el.addEventListener('change', () => {
            readInputs();
            saveConfig();
            applyDisplayConfig();
            if (!f.local && window.lidar) window.lidar.setConfig(mainConfig());
        });
    }

    const preset = $('lidar-mount-preset');
    if (preset) {
        preset.addEventListener('change', () => {
            if (!preset.value) return;
            const [r, p, y] = preset.value.split(',').map(Number);
            $('lidar-mount-roll').value = r;
            $('lidar-mount-pitch').value = p;
            $('lidar-mount-yaw').value = y;
            preset.value = '';
            readInputs();
            saveConfig();
            if (window.lidar) window.lidar.setConfig(mainConfig());
        });
    }

    els.enable.addEventListener('change', () => setEnabled(els.enable.checked));

    // A vehicle link ends the demo flight, and with it the synthetic scan.
    window.addEventListener('mavlinkConnectionState', (ev) => {
        const state = ev.detail && ev.detail.state;
        if (state !== 'DISCONNECTED' && isLidarDemoRunning()) {
            stopDemo();
            enabled = false;
            els.enable.checked = false;
            setLidarCloudVisible(false);
            els.strip.classList.add('hidden');
            setNavDot('lidar', false);
            pushHudMessage('LIDAR: demo scan stopped — link up, enable again for the sensor', 'info');
        }
    });
    $('lidar-clear-btn')?.addEventListener('click', clearMap);
    $('lidar-save-btn')?.addEventListener('click', saveMap);
    $('lidar-strip-clear')?.addEventListener('click', clearMap);
    $('lidar-strip-save')?.addEventListener('click', saveMap);

    if (window.lidar) {
        window.lidar.onOrigin((o) => setLidarOrigin(o));
        window.lidar.onPoints((batch) => {
            appendLidarPoints(batch);
            if (els.stripCount) els.stripCount.textContent = `${fmt(getLidarPointCount())} pts`;
        });
        window.lidar.onLive((batch) => { if (enabled) appendLiveLidarPoints(batch); });
        window.lidar.onStatus(onStatus);
        // The main process keeps the link and the map across a renderer
        // restart: pick the session back up instead of showing OFF.
        window.lidar.getStatus().then((st) => {
            if (!st || !st.connected) return;
            enabled = true;
            els.enable.checked = true;
            setLidarCloudVisible(true);
            els.strip.classList.remove('hidden');
            clearLidarCloud();
            window.lidar.resync();
            onStatus(st);
        }).catch(() => {});
    } else {
        renderStatus({ connected: false, error: 'LiDAR bridge unavailable (preload)' });
    }
}

export function isLidarEnabled() { return enabled; }
