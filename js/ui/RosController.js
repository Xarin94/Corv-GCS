/**
 * RosController.js - ROS surface: settings, rosbridge worker, pose feed, strip
 *
 * Wires SETUP → TOOLS → ROS to RosWorker (the rosbridge client, sampling and
 * averaging, in a module Web Worker) and RosMesh3D (the wireframe). Feeds the
 * worker the vehicle pose from STATE at 20 Hz — the absolute position, or the
 * relative navigation frame: the same position the 3D view draws the vehicle
 * at — and clears the surface whenever that frame changes (relative mode
 * switched, a vehicle connecting, which restarts the relative frame).
 *
 * Origin of world-frame points (map, odom): home in absolute mode — a sub's
 * surface for ArduSub, whose EKF origin is at 0 m — and zero of the local
 * frame in relative mode.
 *
 * The strip on the flight screen switches the colour (grey, or red → blue by
 * distance from the vehicle) and shows / hides the surface; accumulation goes
 * on while it is hidden.
 *
 * Settings persist in localStorage under 'ros-config' (the strip's two
 * switches too); the ENABLE state does not — like the LiDAR, a link is
 * started deliberately, per flight.
 */

import { STATE, isDemoMode, POSE_HISTORY } from '../core/state.js';
import { ORIGIN } from '../core/constants.js';
import { isRelativeMode } from '../core/RelativeNav.js';
import { pushHudMessage } from '../hud/HUDRenderer.js';
import { setNavDot } from './TabController.js';
import { drawMountPreview } from './MountPreview.js';
import { getRoute } from '../mission/RouteModel.js';
import {
    applyRosTiles, clearRosMesh, setRosMeshVisible, setRosMeshShown, setRosMeshFillOpacity,
    setRosMeshColorMode, setRosMeshOverTerrain, setRosMeshMinSamples, setRosMeshFill, getRosColorRange, getRosMeshStats, getRosRoughScale
} from '../ros/RosMesh3D.js';
import { applyRosVolume, clearRosVolume, getRosVolumeStats, getRosVolumeColorRange, getRosVolumeRoughScale } from '../ros/RosVolume3D.js';

const STORAGE_KEY = 'ros-config';
const POSE_MS = 50;

// id → config key, with the parser used on read. `local` keys stay in the renderer.
const FIELDS = {
    'ros-url':          { key: 'url',        parse: v => String(v).trim(), local: true },
    'ros-transport':    { key: 'transport',  parse: v => String(v) },
    'ros-profile':      { key: 'profile',    parse: v => String(v), local: true },
    'ros-frame':        { key: 'frame',      parse: v => String(v) },
    'ros-rate':         { key: 'rateHz',     parse: num(5) },
    'ros-max-points':   { key: 'maxPoints',  parse: num(400) },
    'ros-mount-roll':   { key: 'mountRoll',  parse: num(0) },
    'ros-mount-pitch':  { key: 'mountPitch', parse: num(0) },
    'ros-mount-yaw':    { key: 'mountYaw',   parse: num(0) },
    'ros-lever-x':      { key: 'leverX',     parse: num(0) },
    'ros-lever-y':      { key: 'leverY',     parse: num(0) },
    'ros-lever-z':      { key: 'leverZ',     parse: num(0) },
    'ros-min-range':    { key: 'minRange',   parse: num(2.5) },
    'ros-max-range':    { key: 'maxRange',   parse: num(100) },
    'ros-despike':      { key: 'despike',    parse: v => (v === 'off' ? 'off' : 'on') },
    'ros-lag':          { key: 'lagMs',      parse: num(0) },
    'ros-time-base':    { key: 'timeBase',   parse: v => String(v) },
    'ros-vertical':     { key: 'vertical',   parse: v => (v === 'water' ? 'water' : 'vehicle') },
    'ros-cell':         { key: 'cell',       parse: num(0.3) },
    'ros-grid-memory':  { key: 'memory',     parse: num(20) },
    'ros-max-tiles':    { key: 'maxTiles',   parse: num(2048) },
    'ros-min-samples':  { key: 'minSamples', parse: num(1), local: true },
    'ros-fill':         { key: 'fillM',      parse: num(0) },
    'ros-layers':       { key: 'layers',     parse: v => String(v) },
    'ros-fill-opacity': { key: 'fillPct',    parse: num(10), local: true },
    'ros-over-terrain': { key: 'overTerrain', parse: v => !!v, checkbox: true, local: true }
};

// What choosing a sensor fills in: an aerial Livox inverted under the belly;
// a down-looking echo sounder or sonar (Range / LaserScan measure along x); a
// mechanical sector-scanning sonar, head down and sweeping across the track,
// one beam per ping (its LaserScans carry time_increment: each beam is placed
// at its own time). 30 cm cells need points: ~10 000 a second to fill a
// 100 m swath at 8 m/s.
const PROFILES = {
    aerial: { mountRoll: 180, mountPitch: 0, mountYaw: 0, minRange: 2.5, maxRange: 100, rateHz: 10, maxPoints: 5000, fillM: 0 },
    seabed: { mountRoll: 0, mountPitch: -90, mountYaw: 0, minRange: 0.5, maxRange: 100, rateHz: 10, maxPoints: 1000, fillM: 0 },
    // its sweeps 5–10 m apart along the track: holes filled up to 8 m
    sector: { mountRoll: 0, mountPitch: -90, mountYaw: 0, minRange: 0.75, maxRange: 100, rateHz: 10, maxPoints: 1000, fillM: 8 }
};
const KEY_TO_ID = Object.fromEntries(Object.entries(FIELDS).map(([id, f]) => [f.key, id]));

// The strip's colour button: grey → distance → roughness → grey
const COLOR_LABEL = { gray: 'GRAY', distance: 'DIST', rough: 'ROUGH' };
const COLOR_NEXT = { gray: 'distance', distance: 'rough', rough: 'gray' };

function num(def) {
    return v => { const n = parseFloat(v); return Number.isFinite(n) ? n : def; };
}

let cfg = {};
let enabled = false;
let worker = null;
let poseTimer = null;
let els = {};
let lastStatus = null;
let lastState = null;
let topicList = [];

function $(id) { return document.getElementById(id); }

// ============== SETTINGS ==============
function loadConfig() {
    try { return JSON.parse(localStorage.getItem(STORAGE_KEY) || '{}'); } catch (_) { return {}; }
}

function saveConfig() {
    try { localStorage.setItem(STORAGE_KEY, JSON.stringify(cfg)); } catch (_) { /* not persisted */ }
}

function readInputs() {
    for (const [id, f] of Object.entries(FIELDS)) {
        const el = $(id);
        if (el) cfg[f.key] = f.parse(f.checkbox ? el.checked : el.value);
    }
}

function populateInputs() {
    for (const [id, f] of Object.entries(FIELDS)) {
        const el = $(id);
        if (!el || cfg[f.key] === undefined) continue;
        if (f.checkbox) el.checked = !!cfg[f.key]; else el.value = cfg[f.key];
    }
}

function workerConfig() {
    const out = {};
    for (const f of Object.values(FIELDS)) if (!f.local) out[f.key] = cfg[f.key];
    return out;
}

function applyView() {
    setRosMeshFillOpacity((cfg.fillPct ?? 10) / 100);
    setRosMeshOverTerrain(cfg.overTerrain !== false);
    setRosMeshMinSamples(cfg.minSamples ?? 1);
    setRosMeshFill(cfg.fillM ?? 0);
    setRosMeshColorMode(cfg.colorMode);
    setRosMeshShown(cfg.shown !== false);
    if (els.colorBtn) {
        els.colorBtn.textContent = COLOR_LABEL[cfg.colorMode] || 'GRAY';
        els.colorBtn.classList.toggle('on', cfg.colorMode === 'distance');
        els.colorBtn.classList.toggle('rough', cfg.colorMode === 'rough');
    }
    if (els.viewBtn) {
        els.viewBtn.textContent = cfg.shown !== false ? 'VIEW ON' : 'VIEW OFF';
        els.viewBtn.classList.toggle('off', cfg.shown === false);
    }
}

function pushConfig() {
    applyView();
    if (worker) worker.postMessage({ op: 'config', cfg: workerConfig() });
}

function applyProfile(name) {
    const p = PROFILES[name];
    if (!p) return;
    for (const [key, value] of Object.entries(p)) {
        const el = $(KEY_TO_ID[key]);
        if (el) el.value = value;
    }
}

// ============== WORKER ==============
function ensureWorker() {
    if (worker) return worker;
    worker = new Worker(new URL('../ros/RosWorker.js', import.meta.url), { type: 'module' });
    worker.onmessage = (e) => {
        const m = e.data || {};
        if (m.op === 'status') onStatus(m.st);
        else if (m.op === 'topics') onTopics(m.list || []);
        else if (m.op === 'tiles') { if (enabled) applyRosTiles(m); }
        else if (m.op === 'volume') { if (enabled) applyRosVolume(m); }
    };
    worker.onerror = (e) => {
        console.error('[ROS] worker error:', e.message);
        renderStatus({ link: 'OFF', error: `worker: ${e.message}` });
    };
    worker.postMessage({ op: 'config', cfg: workerConfig() });
    return worker;
}

// Vehicle pose for the worker; null with the reason when there is none to place points with
function poseSample() {
    if (isDemoMode()) return { reason: 'NO VEHICLE' };
    const rel = isRelativeMode();
    if (!Number.isFinite(STATE.lat) || !Number.isFinite(STATE.lon) || (!rel && STATE.lat === 0 && STATE.lon === 0)) {
        return { reason: 'NO POSITION' };
    }
    let home = null;
    if (rel) home = { lat: ORIGIN.lat, lon: ORIGIN.lon, alt: 0 };
    else if (Number.isFinite(STATE.homeLat) && Number.isFinite(STATE.homeLon)) {
        const alt = Number.isFinite(STATE.subSurfaceAlt) ? STATE.subSurfaceAlt : (Number.isFinite(STATE.homeAlt) ? STATE.homeAlt : 0);
        home = { lat: STATE.homeLat, lon: STATE.homeLon, alt };
    }
    // Each value with the arrival time of the message it came from, the body
    // rates and the velocity, for the worker to project to the points' time.
    // The relative position is recomputed every frame: it is as of now.
    const now = Date.now();
    return {
        p: {
            tA: STATE.attTime || now, tP: rel ? now : (STATE.posTime || now),
            lat: STATE.lat, lon: STATE.lon, alt: STATE.rawAlt,
            roll: STATE.roll || 0, pitch: STATE.pitch || 0, yaw: STATE.yaw || 0,
            p: STATE.rollRate || 0, q: STATE.pitchRate || 0, r: STATE.yawRate || 0,
            vn: STATE.vn || 0, ve: STATE.ve || 0, vd: STATE.vd || 0, rel, home
        }
    };
}

// The planner's survey areas, for the coverage the worker measures against them
let sentAreas = '';
function sendAreas() {
    if (!worker) return;
    let areas = [];
    try {
        areas = getRoute().segments
            .filter(s => s.type === 'area' && s.points.length >= 3)
            .map((s, i) => ({ name: `area ${i + 1}`, points: s.points.map(p => ({ lat: p.lat, lng: p.lng })) }));
    } catch (_) { /* no route yet */ }
    const key = JSON.stringify(areas);
    if (key === sentAreas) return;
    sentAreas = key;
    worker.postMessage({ op: 'areas', areas });
}

// Every ATTITUDE / GLOBAL_POSITION_INT since the last call goes along: at 25 Hz
// they are one or two, in a SITL at speedup 20 a few dozen
let sentSeq = 0;
function sendPose() {
    if (!worker) return;
    const s = poseSample();
    const att = [], pos = [];
    if (s.p) {
        for (const x of POSE_HISTORY.att) if (x.seq > sentSeq) att.push(x);
        if (!s.p.rel) for (const x of POSE_HISTORY.pos) if (x.seq > sentSeq) pos.push(x);
    }
    sentSeq = POSE_HISTORY.seq;
    worker.postMessage({ op: 'pose', p: s.p || null, reason: s.reason, att, pos });
}

// ============== CONNECTION ==============
function setEnabled(on) {
    readInputs();
    saveConfig();
    if (on) {
        if (!/^wss?:\/\//i.test(cfg.url || '')) {
            els.enable.checked = false;
            pushHudMessage('ROS: the rosbridge URL must start with ws:// or wss://', 'error');
            return;
        }
        enabled = true;
        ensureWorker();
        pushConfig();
        worker.postMessage({ op: 'clear' });
        worker.postMessage({ op: 'connect', url: cfg.url });
        if (cfg.topic) worker.postMessage({ op: 'select', topic: cfg.topic, type: cfg.topicType });
        clearInterval(poseTimer);
        poseTimer = setInterval(sendPose, POSE_MS);
        sentAreas = '';
        sendAreas();
        setRosMeshVisible(true);
        els.strip.classList.remove('hidden');
        lastState = null;
        pushHudMessage(`ROS: connecting to ${cfg.url}`, 'info');
    } else {
        enabled = false;
        clearInterval(poseTimer);
        poseTimer = null;
        if (worker) worker.postMessage({ op: 'disconnect' });
        setRosMeshVisible(false);
        clearRosMesh();
        clearRosVolume();
        els.strip.classList.add('hidden');
        setNavDot('ros', false);
    }
}

function clearSurface(announce) {
    if (worker) worker.postMessage({ op: 'clear' });
    clearRosMesh();
    clearRosVolume();
    if (announce) pushHudMessage('ROS: surface cleared', 'info');
}

// ============== TOPICS ==============
function shortType(t) {
    return String(t || '').replace('/msg/', '/').replace(/^sensor_msgs\//, '');
}

function onTopics(list) {
    topicList = list;
    const sel = els.topic;
    if (!sel) return;
    const usable = list.filter(t => t.kind).sort((a, b) => a.name.localeCompare(b.name));
    const others = list.length - usable.length;
    sel.innerHTML = '';
    const none = document.createElement('option');
    none.value = '';
    none.textContent = usable.length ? '— choose a topic —' : '— no PointCloud2 / LaserScan / Range topic —';
    sel.appendChild(none);
    for (const t of usable) {
        const opt = document.createElement('option');
        opt.value = t.name;
        opt.dataset.type = t.type;
        opt.textContent = `${t.name} · ${shortType(t.type)}`;
        sel.appendChild(opt);
    }
    // A saved topic nobody publishes yet stays selected (rosbridge subscribes ahead of the publisher)
    if (cfg.topic && !usable.some(t => t.name === cfg.topic)) {
        const opt = document.createElement('option');
        opt.value = cfg.topic;
        opt.dataset.type = cfg.topicType || '';
        opt.textContent = `${cfg.topic} · ${shortType(cfg.topicType)} (not published now)`;
        sel.appendChild(opt);
    }
    sel.value = cfg.topic || '';
    sel.title = others ? `${others} other topics of types this view does not draw` : '';
}

function onTopicChange() {
    const opt = els.topic.selectedOptions[0];
    cfg.topic = els.topic.value || null;
    cfg.topicType = opt ? opt.dataset.type || null : null;
    saveConfig();
    clearSurface(false);
    if (worker) worker.postMessage({ op: 'select', topic: cfg.topic, type: cfg.topicType });
}

// ============== MOUNT PREVIEW ==============
const DEG = Math.PI / 180;

// The zone the selected topic collects from: a Range's cone, a LaserScan's
// fan, a cloud's directions over the last 10 s (from the worker)
function previewZone() {
    const st = lastStatus, z = st && st.zone, kind = st && st.kind;
    if (!kind) return { zone: null, caption: 'choose a topic: its zone shows here' };
    if (st.frame && st.frame !== 'sensor') return { zone: null, caption: `points in the ${st.frameId} frame: no sensor zone` };
    if (kind === 'range') {
        const fov = z && z.fov > 0 ? z.fov / DEG : 30;
        return { zone: { type: 'cone', half: fov / 2 }, caption: `Range · ${fmt(fov)}° cone along X` };
    }
    if (kind === 'scan') {
        const [a0, a1] = z && Number.isFinite(z.min) ? [z.min / DEG, z.max / DEG] : [-135, 135];
        return { zone: { type: 'fan', min: a0, max: a1 }, caption: `LaserScan · ${fmt(a0)}…${fmt(a1)}° in the X–Y plane` };
    }
    if (z && z.dirs && z.dirs.bins.length) return { zone: { type: 'dirs', ...z.dirs }, caption: 'PointCloud2 · where the points came from, last 10 s' };
    return cfg.profile === 'aerial'
        ? { zone: { type: 'band', elMin: -7, elMax: 52 }, caption: 'PointCloud2 · Mid-360 band until points arrive' }
        : { zone: null, caption: 'PointCloud2 · the zone shows when points arrive' };
}

// From the inputs as they are typed, before they are applied
function drawPreview() {
    const canvas = $('ros-mount-preview');
    if (!canvas) return;
    const v = (id) => parseFloat($(id)?.value) || 0;
    drawMountPreview(canvas, {
        mount: [v('ros-mount-roll'), v('ros-mount-pitch'), v('ros-mount-yaw')],
        lever: [v('ros-lever-x'), v('ros-lever-y'), v('ros-lever-z')],
        ...previewZone()
    });
}

// ============== STATUS ==============
function fmt(n, d = 0) {
    return (n || 0).toLocaleString('en-US', { maximumFractionDigits: d });
}

function esc(s) {
    return String(s).replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
}

const FRAME_LABEL = { sensor: 'SENSOR · placed with the telemetry pose', 'world-enu': 'WORLD ENU', 'world-ned': 'WORLD NED' };

function renderStatus(st) {
    const el = els.statusText;
    if (!el) return;
    if (!st || st.link === 'OFF') {
        el.innerHTML = st && st.error ? `<span class="err">${esc(st.error)}</span>` : 'OFF';
        return;
    }
    const ok = st.state === 'ACCUMULATING';
    const lines = [];
    lines.push(`<span class="${st.link === 'CONNECTED' ? 'ok' : 'warn'}">${esc(st.link)}</span> ${esc(st.url || '')}`);
    if (st.topic) lines.push(`${esc(st.topic)} · ${esc(shortType(st.type))}${st.frameId ? ` · frame ${esc(st.frameId)} → ${FRAME_LABEL[st.frame] || ''}` : ''}`);
    if (st.timedSpan > 0 && st.frame === 'sensor') lines.push(`beams measured over ${fmt(st.timedSpan, 2)} s a message (time_increment): each placed with the pose of its own time`);
    if (st.stampOff !== null && st.stampOff !== undefined) lines.push(`<span class="warn">header.stamp ${fmt(st.stampOff / 1000, 1)} s from this clock: not synced, arrival time used</span>`);
    const r = st.rate || {};
    lines.push(`${fmt(r.msgs)} msg/s · ${fmt(r.pointsIn)} pts/s in · ${fmt(r.sampled)} sampled · ${fmt(r.used)} averaged`
        + (st.despike === 'on' ? ` · ${fmt(r.spikes || 0)} spikes dropped (${fmt(100 * (r.spikes || 0) / Math.max(1, r.sampled), 1)} %)`
            : st.despike === 'unordered' ? ' · spike filter idle: the scan has no order' : ''));
    const drops = [];
    if (r.noPose) {
        const m = st.poseMiss;
        // (points later than the newest telemetry: the telemetry lags; earlier than the oldest kept: the points do)
        drops.push(`${fmt(r.noPose)} msg/s without pose` + (m && m.att !== null
            ? ` (points ${m.att >= 0 ? `${fmt(m.att)} ms after the newest attitude` : `${fmt(-m.att)} ms before it`}, ${fmt(m.oldest)} ms after the oldest kept, clock ×${fmt(m.clockRate, 1)})` : ''));
    }
    if (r.noHome) drops.push(`${fmt(r.noHome)} msg/s without home`);
    if (drops.length) lines.push(`<span class="warn">${drops.join(' · ')}</span>`);
    const g = st.surface || {};
    if (g.layers === 'cave') {
        const v = getRosVolumeStats();
        lines.push(`3D surface: ${fmt(g.chunks)} / ${fmt(g.maxChunks)} chunks of 16³ voxels of ${fmt(g.cell, 2)} m · ${fmt(v.triangles)} triangles`);
    } else {
        const m = getRosMeshStats();
        lines.push(`surface ${fmt(g.tiles)} / ${fmt(g.maxTiles)} tiles of ${g.tileCells}×${g.tileCells} · cell ${fmt(g.cell, 2)} m · ${fmt(g.filled)} cells (${fmt(g.filled * g.cell * g.cell)} m²)`);
        for (const c of g.coverage || []) {
            lines.push(`planned ${esc(c.name)}: ${fmt(c.area / 1e6, 2)} km² · <span class="${c.covered / c.area > 0.9 ? 'ok' : 'warn'}">${fmt(100 * c.covered / c.area, 1)} % covered</span>`);
        }
        const lv = m.levels, lvTotal = lv.reduce((x, y) => x + y, 0);
        const res = lvTotal ? lv.map((n, k) => n ? `${fmt(g.cell * 100 * 2 ** k)} cm ${Math.round(100 * n / lvTotal)}%` : null).filter(Boolean).join(' · ') : '—';
        lines.push(`drawn ${fmt(m.blocksDrawn)} / ${fmt(m.blocks)} blocks · ${fmt(m.triangles)} triangles · resolution ${res}`);
    }
    lines.push(`<span class="${ok ? 'ok' : 'warn'}">${esc(st.state)}</span> · pose ${isRelativeMode() ? 'relative (local frame)' : 'absolute (GPS)'}`
        + (st.vertical === 'water' ? ` · height: the water surface at home${st.waterAlt !== null && st.waterAlt !== undefined ? ` (${fmt(st.waterAlt, 1)} m)` : ', no home yet'}` : '')
        + (st.clockRate && Math.abs(st.clockRate - 1) > 0.1 ? ` · autopilot clock ×${fmt(st.clockRate, 1)} (SITL speedup)` : ''));
    if (st.error && st.link !== 'CONNECTED') lines.push(`<span class="err">${esc(st.error)}</span>`);
    el.innerHTML = lines.join('\n');
}

function updateStrip(st) {
    if (!els.strip || !enabled || !st) return;
    const s = st.state;
    const cls = s === 'ACCUMULATING' ? 'accum' : (s === 'CONNECTING' || s === 'NO ROSBRIDGE') ? 'search' : 'hold';
    els.strip.className = `lidar-strip ros-strip state-${cls}`;
    els.stripState.textContent = s;
    const g = st.surface || {}, cave = g.layers === 'cave';
    const have = cave ? g.chunks : g.filled;
    let text = !have ? '—' : cave ? `3D · ${fmt(getRosVolumeStats().triangles)} triangles · ${fmt(g.cell * 100)} cm`
        : `${fmt(g.filled)} cells · ${fmt(g.cell * 100)} cm`;
    if (cfg.colorMode === 'distance' && have) {
        const r = cave ? getRosVolumeColorRange() : getRosColorRange();
        text += ` · ${fmt(r.near)}–${fmt(r.far)} m`;
    } else if (cfg.colorMode === 'rough' && have) {
        const r = cave ? getRosVolumeRoughScale() : getRosRoughScale();
        text += cave ? ` · blue ≤ ${fmt(r.mean)}° · red ≥ ${fmt(r.full)}° of spread`
            : ` · blue ≤ ${fmt(r.mean * 100)} cm · red ≥ ${fmt(r.full * 100)} cm off-plane`;
    }
    if (!cave && g.coverage && g.coverage.length) {
        const a = g.coverage.reduce((s, c) => s + c.area, 0), c = g.coverage.reduce((s, x) => s + x.covered, 0);
        text += ` · ${fmt(100 * c / a)} % of the area`;
    }
    els.stripGrid.textContent = text;
}

function onStatus(st) {
    lastStatus = st;
    drawPreview();
    sendAreas();
    renderStatus(st);
    updateStrip(st);
    setNavDot('ros', enabled && st && st.link === 'CONNECTED');
    if (!enabled || !st || st.state === lastState) return;
    if (st.state === 'ACCUMULATING') pushHudMessage(`ROS: averaging ${st.topic}`, 'info');
    else if (lastState === 'ACCUMULATING') pushHudMessage(`ROS: ${st.state.toLowerCase()}`, 'warning');
    else if (st.state === 'NO ROSBRIDGE' && lastState === 'CONNECTING') pushHudMessage(`ROS: no rosbridge at ${st.url}`, 'warning');
    lastState = st.state;
}

// ============== INIT ==============
export function initRosController() {
    els = {
        enable: $('ros-enable'),
        topic: $('ros-topic'),
        strip: $('ros-strip'),
        stripState: $('ros-strip-state'),
        stripGrid: $('ros-strip-grid'),
        colorBtn: $('ros-strip-color'),
        viewBtn: $('ros-strip-view'),
        statusText: $('ros-status-text')
    };
    if (!els.enable || !els.strip) {
        console.warn('[ROS] panel not found, controller disabled');
        return;
    }

    cfg = loadConfig();
    if (!cfg.profile) applyProfile('aerial');     // first run: the aerial defaults
    populateInputs();
    readInputs();
    applyView();
    if (cfg.topic) onTopics([]);

    for (const [id, f] of Object.entries(FIELDS)) {
        const el = $(id);
        if (!el) continue;
        el.addEventListener('change', () => {
            if (f.key === 'profile') applyProfile(el.value);
            readInputs();
            saveConfig();
            pushConfig();
            drawPreview();
            if (f.key === 'url' && enabled) worker.postMessage({ op: 'connect', url: cfg.url });
        });
    }

    $('ros-mount-preset')?.addEventListener('change', (e) => {
        const v = e.target.value;
        if (!v) return;
        const [r, p, y] = v.split(',').map(Number);
        $('ros-mount-roll').value = r;
        $('ros-mount-pitch').value = p;
        $('ros-mount-yaw').value = y;
        e.target.value = '';
        readInputs();
        saveConfig();
        pushConfig();
        drawPreview();
    });

    for (const id of ['ros-mount-roll', 'ros-mount-pitch', 'ros-mount-yaw', 'ros-lever-x', 'ros-lever-y', 'ros-lever-z']) {
        $(id)?.addEventListener('input', drawPreview);
    }
    const previewCanvas = $('ros-mount-preview');
    if (previewCanvas && window.ResizeObserver) new ResizeObserver(drawPreview).observe(previewCanvas);   // drawn when the sub-tab shows

    els.enable.addEventListener('change', () => setEnabled(els.enable.checked));
    els.topic.addEventListener('change', onTopicChange);
    $('ros-topics-refresh')?.addEventListener('click', () => { if (worker) worker.postMessage({ op: 'topics' }); });
    $('ros-clear-btn')?.addEventListener('click', () => clearSurface(true));
    $('ros-strip-clear')?.addEventListener('click', () => clearSurface(true));
    els.colorBtn?.addEventListener('click', () => {
        cfg.colorMode = COLOR_NEXT[cfg.colorMode] || 'distance';
        saveConfig();
        applyView();
        updateStrip(lastStatus);
    });
    els.viewBtn?.addEventListener('click', () => {
        cfg.shown = cfg.shown === false;
        saveConfig();
        applyView();
    });

    // The frame the points were placed in is gone: start the surface again
    window.addEventListener('relNavChanged', () => { if (enabled) clearSurface(false); });
    window.addEventListener('mavlinkConnectionState', (e) => {
        if (enabled && e.detail && e.detail.state === 'CONNECTED') clearSurface(false);
    });
}

export function isRosEnabled() { return enabled; }

/** Last status from the worker (tests, diagnostics). */
export function getRosStatus() { return lastStatus; }
