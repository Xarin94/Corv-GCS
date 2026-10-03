/**
 * RosWorker.js - rosbridge client, sampling and averaging off the render thread
 *
 * Module Web Worker started by RosController. roslib (vendored, see
 * vendor/roslib/README.md) holds the WebSocket to rosbridge_server here, so
 * decoding a PointCloud2 never costs the render thread a frame: all that
 * leaves this worker are the averaged tiles that changed (30 × 30 heights
 * each, see SurfaceTiles.js), at most four times a second.
 *
 * Load on the link is cut at the source: the subscription asks rosbridge for
 * at most MAX RATE messages a second (throttle_rate) and keeps only the newest
 * (queue_length 1), in CBOR so binary arrays are not base64-inflated. Each
 * message then contributes at most POINTS PER MESSAGE sampled points.
 *
 * The pose comes from the controller (STATE, ~20 Hz), not from ROS: it is the
 * position the 3D view already draws the vehicle at, absolute (GPS) or in the
 * relative navigation frame, so the mesh lands where the vehicle is in either
 * mode. Each sample carries the arrival time of the ATTITUDE and of the
 * position it comes from, the body rates and the NED velocity; the pose at the
 * time of a message is PROJECTED from the last attitude before it with the
 * gyro rates (Euler angle kinematics) and from the last position with the
 * velocity. Without that, a copter yawing 90°/s with an attitude 50 ms old
 * places a return 80 m away 6 m off: spikes wherever the scan crosses a slope.
 *
 * Time of a message: its arrival minus LAG (how much later than the
 * telemetry the points reach the GCS), or — TIME BASE 'stamp', when the
 * companion computer's clock is synced with this one (chrony / NTP / GPS) —
 * its header.stamp plus LAG (then: how late the telemetry arrives). A stamp
 * more than 2 s from this clock is not trusted: arrival is used and the
 * status says so.
 *
 * Surfaces. Over open ground or water the points average into a height per
 * cell (SurfaceTiles.js). In a cave, a shaft, under a pier (SURFACES 'cave')
 * a height per cell cannot hold floor, walls and ceiling together: there the
 * returns are fused into a 3D volume of signed distances along their rays and
 * meshed where it changes sign (SurfaceVolume.js).
 *
 * Protocol with the controller:
 *   in   { op: 'connect', url }   { op: 'disconnect' }   { op: 'topics' }
 *        { op: 'select', topic, type }                   { op: 'config', cfg }
 *        { op: 'clear' }          { op: 'pose', p | null, reason }
 *   out  { op: 'status', st }     { op: 'topics', list: [{ name, type, kind }] }
 *        { op: 'tiles', reset, anchor, cell, tiles: [{ tx, ty, h, w }], removed: [[tx, ty]] }
 *                                 changed tiles (transferred arrays); reset: start over
 *        { op: 'volume', reset, anchor, cell, meshes: [{ key, pos }], removed: [key] }
 *                                 changed chunk meshes of the 3D surface (triangles, metres from the anchor)
 */

import { Ros, Topic } from '../../vendor/roslib/roslib.esm.min.js';
import { SurfaceTiles, TILE, coveredArea, polygonAreaXY } from './SurfaceTiles.js';
import {
    kindOf, frameOf, decodePoints, despike, eulerToMatrix, makeAnchor, toEnu, sensorToEnu, worldToEnu, projectPose,
} from './RosPoints.js';
import { SurfaceVolume } from './SurfaceVolume.js';

const DEG = Math.PI / 180;
const RECONNECT_MS = 2000;
const TILES_MS = 250;
const POSE_CAP = 1024;             // samples kept: 40 s at 25 Hz, 2 s of a SITL at speedup 20
const CLOCK_SPAN_MS = 300;         // samples spanning at least this give the autopilot's clock rate
const MAX_PROJECT_MS = 500;       // a pose is projected at most this far from its sample
const STAMP_TRUST_MS = 2000;
const MAX_POINTS = 20000;

const cfg = {
    transport: 'cbor', rateHz: 5, maxPoints: 400, frame: 'auto', lagMs: 0, timeBase: 'arrival',
    minRange: 0.5, maxRange: 100, despike: 'on',
    mountRoll: 0, mountPitch: 0, mountYaw: 0, leverX: 0, leverY: 0, leverZ: 0,
    cell: 0.3, memory: 20, maxTiles: 2048, layers: 'floor'
};

let ros = null;
let url = null;
let link = 'OFF';
let lastError = null;
let reconnectTimer = null;
let topics = [];
let sel = null;                   // { topic, type, kind }
let sub = null;                   // roslib Topic
let lastMsgAt = 0;
let lastFrameId = '';
let lastFrame = '';

const surface = new SurfaceTiles();
const volume = new SurfaceVolume();
let anchor = null;                // makeAnchor() + rel, epoch
let epoch = 0;
let resetPending = true;          // the renderer must drop what it has

const attRing = [];               // { t, b, roll, pitch, yaw, p, q, r }     t: ATTITUDE arrival, b: time_boot_ms
const posRing = [];               // { t, b, lat, lon, alt, vn, ve, vd, rel, home }
let clockRate = 1;                // autopilot seconds per second here (SITL: its speedup)
let poseReason = 'NO VEHICLE';
let stampOff = null;              // header.stamp − this clock, ms, when it was not trusted
const Rm = new Float64Array(9);
const Ra = new Float64Array(9);
const lever = [0, 0, 0];
const veh = [0, 0, 0];
const org = [0, 0, 0];
const pts = new Float32Array(MAX_POINTS * 3);
const enu = new Float64Array(MAX_POINTS * 3);
const sensorPos = [0, 0, 0];

// Per-second counters, and totals since the last clear
const rate = { msgs: 0, pointsIn: 0, sampled: 0, used: 0, spikes: 0, noPose: 0, noHome: 0 };
let unordered = false;            // the last scan had no order: the spike filter left it alone
let lastRate = { ...rate };
let totalMsgs = 0;
let blocked = null;               // why the last message was not used

// Where the sensor collects from, for the mount preview: a Range's cone, a
// LaserScan's fan, and for clouds in the sensor frame the directions (10° ×
// 10° bins of azimuth and elevation) points came from in the last 10 s
const DIR_AZ = 36, DIR_EL = 18, DIR_KEEP_MS = 10000, DIR_SAMPLES = 200;
const dirSeen = new Float64Array(DIR_AZ * DIR_EL);
let zoneMeta = null;

function post(m, transfer) { self.postMessage(m, transfer || []); }

// ============== CONNECTION ==============
function connect(newUrl) {
    disconnect();
    url = newUrl;
    link = 'CONNECTING';
    open();
    postStatus();
}

function open() {
    clearTimeout(reconnectTimer);
    const r = new Ros();
    ros = r;
    r.on('connection', () => {
        if (ros !== r) return;
        link = 'CONNECTED';
        lastError = null;
        listTopics();
        subscribe();
        postStatus();
    });
    r.on('error', () => {
        if (ros !== r) return;
        lastError = `cannot reach ${url}`;
    });
    r.on('close', () => {
        if (ros !== r) return;
        sub = null;
        link = 'RECONNECTING';
        reconnectTimer = setTimeout(open, RECONNECT_MS);
        postStatus();
    });
    r.connect(url).catch((e) => {
        if (ros !== r) return;
        lastError = e && e.message ? e.message : String(e);
        link = 'RECONNECTING';
        reconnectTimer = setTimeout(open, RECONNECT_MS);
        postStatus();
    });
}

function disconnect() {
    clearTimeout(reconnectTimer);
    unsubscribe();
    const r = ros;
    ros = null;
    if (r) { try { r.close(); } catch (_) { /* already closed */ } }
    link = 'OFF';
    topics = [];
}

function listTopics() {
    if (!ros || !ros.isConnected) return;
    const r = ros;
    r.getTopics((res) => {
        if (ros !== r) return;
        const names = (res && res.topics) || [], types = (res && res.types) || [];
        topics = names.map((name, i) => ({ name, type: types[i] || '', kind: kindOf(types[i]) }));
        post({ op: 'topics', list: topics });
    }, (err) => {
        lastError = `rosapi: ${err} — is rosapi running (rosbridge_websocket_launch)?`;
        postStatus();
    });
}

function subscribe() {
    unsubscribe();
    if (!ros || !ros.isConnected || !sel || !sel.kind) return;
    sub = new Topic({
        ros,
        name: sel.topic,
        messageType: sel.type,
        compression: cfg.transport === 'json' ? 'none' : 'cbor',
        throttle_rate: Math.round(1000 / Math.max(0.1, cfg.rateHz)),
        queue_length: 1
    });
    sub.subscribe(onMessage);
}

function unsubscribe() {
    if (!sub) return;
    try { sub.unsubscribe(); } catch (_) { /* connection gone */ }
    sub = null;
}

// ============== POSE ==============
// The snapshot of STATE (p), and every ATTITUDE / GLOBAL_POSITION_INT that
// arrived since the last one (att, pos: absolute navigation only); a snapshot
// value is used where there are none (the relative position, a replay)
function pushPose(p, att = [], pos = []) {
    const push = (ring, s) => {
        const last = ring[ring.length - 1];
        // The autopilot rebooted (a SITL relaunched): its clock starts over
        if (last && s.b !== null && last.b !== null && s.b < last.b) ring.length = 0;
        ring.push(s);
        if (ring.length > POSE_CAP) ring.shift();
    };
    if (att.length) for (const s of att) push(attRing, s);
    else {
        const a = attRing[attRing.length - 1];
        if (!a || p.tA !== a.t) push(attRing, { t: p.tA, b: null, roll: p.roll, pitch: p.pitch, yaw: p.yaw, p: p.p, q: p.q, r: p.r });
    }
    const q = posRing[posRing.length - 1];
    if (pos.length) for (const s of pos) push(posRing, { ...s, rel: p.rel, home: p.home });
    else if (!q || p.tP !== q.t || p.rel !== q.rel) {
        push(posRing, { t: p.tP, b: null, lat: p.lat, lon: p.lon, alt: p.alt, vn: p.vn, ve: p.ve, vd: p.vd, rel: p.rel, home: p.home });
    }
    clockRate = autopilotClockRate();
}

// Autopilot time per time here, from the attitude samples' time_boot_ms over
// their arrival: 1 on a real vehicle, SIM_SPEEDUP in a SITL
function autopilotClockRate() {
    let first = null;
    const last = attRing[attRing.length - 1];
    if (!last || last.b === null) return 1;
    for (let i = attRing.length - 1; i >= 0 && attRing[i].b !== null; i--) first = attRing[i];
    const dt = last.t - first.t, db = last.b - first.b;
    if (dt < CLOCK_SPAN_MS || db <= 0) return clockRate;
    return Math.max(0.05, Math.min(200, db / dt));
}

// Last sample at or before t (the first one if t is older than all)
function sampleAt(ring, t) {
    for (let i = ring.length - 1; i >= 0; i--) if (ring[i].t <= t) return ring[i];
    return ring[0];
}

// Pose at time t (ms, this clock), projected from the telemetry samples
// (projectPose); null when the nearest sample is more than MAX_PROJECT_MS away
function poseAt(t) {
    if (!attRing.length || !posRing.length) return null;
    const a = sampleAt(attRing, t), q = sampleAt(posRing, t);
    if (Math.abs(t - a.t) > MAX_PROJECT_MS || Math.abs(t - q.t) > MAX_PROJECT_MS) return null;
    return { ...projectPose(a, q, t, clockRate), rel: q.rel, home: q.home };
}

// Time of a message on this clock (see the header)
function messageTime(msg, arrival) {
    const st = cfg.timeBase === 'stamp' && msg && msg.header && msg.header.stamp;
    if (st) {
        const sec = st.sec ?? st.secs, ns = st.nanosec ?? st.nsecs ?? 0;
        if (Number.isFinite(sec)) {
            const ts = sec * 1000 + ns / 1e6;
            if (Math.abs(ts - arrival) <= STAMP_TRUST_MS) { stampOff = null; return ts + cfg.lagMs; }
            stampOff = ts - arrival;
        }
    }
    return arrival - cfg.lagMs;
}

// ============== MESSAGES ==============
function onMessage(msg) {
    const now = Date.now();
    lastMsgAt = now;
    totalMsgs++;
    rate.msgs++;
    const frameId = (msg && msg.header && msg.header.frame_id) || '';
    const frame = frameOf(frameId, cfg.frame);
    lastFrameId = frameId;
    lastFrame = frame;

    const sensor = frame === 'sensor';
    const decoded = decodePoints(sel.kind, msg, pts, Math.min(cfg.maxPoints, MAX_POINTS),
        sensor ? cfg.minRange : 0, sensor ? cfg.maxRange : Infinity);
    const total = decoded.total;
    let count = decoded.count;
    rate.pointsIn += total;
    rate.sampled += count;
    // Spikes along the scan, in the sensor frame where the order is the beams'
    if (sensor && cfg.despike !== 'off' && sel.kind !== 'range') {
        const d = despike(pts, count);
        count = d.count;
        rate.spikes += d.removed;
        unordered = !d.ordered;
    }
    if (sel.kind === 'range') zoneMeta = { fov: msg.field_of_view };
    else if (sel.kind === 'scan') zoneMeta = { min: msg.angle_min, max: msg.angle_max };
    if (sensor && count) {
        const step = Math.max(1, Math.floor(count / DIR_SAMPLES));
        for (let k = 0; k < count; k += step) {
            const x = pts[k * 3], y = pts[k * 3 + 1], z = pts[k * 3 + 2];
            const az = Math.min(DIR_AZ - 1, Math.floor((Math.atan2(y, x) + Math.PI) / (2 * Math.PI) * DIR_AZ));
            const el = Math.min(DIR_EL - 1, Math.floor((Math.atan2(z, Math.hypot(x, y)) + Math.PI / 2) / Math.PI * DIR_EL));
            dirSeen[el * DIR_AZ + az] = now;
        }
    }
    // (decoded first: the zone shows on the bench, with no vehicle yet)
    const pose = poseAt(messageTime(msg, now));
    if (!pose) { rate.noPose++; blocked = poseReason; return; }
    if (!count) { blocked = total ? 'NO VALID POINTS' : 'EMPTY MESSAGES'; return; }

    if (!anchor) {
        anchor = { ...makeAnchor(pose.lat, pose.lon, pose.alt), rel: pose.rel, epoch: ++epoch };
    } else if (anchor.rel !== pose.rel) {
        rate.noPose++;
        blocked = 'FRAME CHANGED · CLEAR';
        return;
    }

    toEnu(anchor, pose.lat, pose.lon, pose.alt, veh);
    if (sensor) {
        eulerToMatrix(pose.roll, pose.pitch, pose.yaw, Ra);
        sensorToEnu(pts, count, Rm, lever, Ra, veh, enu);
    } else {
        const h = pose.home;
        if (!h) { rate.noHome++; blocked = 'NO HOME (world frame)'; return; }
        toEnu(anchor, h.lat, h.lon, h.alt, org);
        worldToEnu(pts, count, frame === 'world-ned', org, enu);
    }
    if (cfg.layers === 'cave') {
        // Rays from the sensor: the vehicle plus the lever arm (FRD → NED → ENU)
        eulerToMatrix(pose.roll, pose.pitch, pose.yaw, Ra);
        const ln = Ra[0] * lever[0] + Ra[1] * lever[1] + Ra[2] * lever[2];
        const le = Ra[3] * lever[0] + Ra[4] * lever[1] + Ra[5] * lever[2];
        const ld = Ra[6] * lever[0] + Ra[7] * lever[1] + Ra[8] * lever[2];
        sensorPos[0] = veh[0] + le; sensorPos[1] = veh[1] + ln; sensorPos[2] = veh[2] - ld;
        rate.used += volume.integrate(sensorPos, enu, count);
    } else {
        rate.used += surface.add(enu, count, veh[0], veh[1]);
    }
    blocked = null;
}

// ============== OUTPUT ==============
function postTiles() {
    const t = resetPending ? surface.takeAll() : surface.takeChanges();
    if (resetPending || t.tiles.length || t.removed.length) {
        post({ op: 'tiles', reset: resetPending, anchor, cell: surface.cell, tiles: t.tiles, removed: t.removed },
            t.tiles.flatMap(x => [x.h.buffer, x.w.buffer]));
    }
    const v = resetPending ? volume.takeAll() : volume.takeChanges();
    if (resetPending || v.meshes.length || v.removed.length) {
        post({ op: 'volume', reset: resetPending, anchor, cell: volume.cell, meshes: v.meshes, removed: v.removed },
            v.meshes.map(m => m.pos.buffer));
    }
    resetPending = false;
}

function state() {
    if (link === 'OFF') return 'OFF';
    if (link !== 'CONNECTED') return lastError ? 'NO ROSBRIDGE' : 'CONNECTING';
    if (!sel || !sel.kind) return 'NO TOPIC';
    if (Date.now() - lastMsgAt > 3000) return 'NO DATA';
    return blocked || 'ACCUMULATING';
}

// The mission's survey areas (lat/lng polygons from the planner) and how much
// of each the surface covers — absolute navigation only: a relative frame has
// no place on the map
let areas = [];
let coverage = [];
const COVERAGE_MS = 2000;
setInterval(() => {
    if (!anchor || anchor.rel || !areas.length) { coverage = []; return; }
    coverage = areas.map(a => {
        const poly = a.points.map(p => {
            const v = toEnu(anchor, p.lat, p.lng, anchor.alt, [0, 0, 0]);
            return [v[0], v[1]];
        });
        return { name: a.name, area: polygonAreaXY(poly), covered: coveredArea(surface, poly) };
    });
}, COVERAGE_MS);

function zoneStatus() {
    const now = Date.now(), bins = [];
    for (let b = 0; b < dirSeen.length; b++) if (now - dirSeen[b] < DIR_KEEP_MS) bins.push(b);
    return { ...zoneMeta, dirs: { az: DIR_AZ, el: DIR_EL, bins } };
}

function postStatus() {
    post({
        op: 'status',
        st: {
            link, url, error: lastError, state: state(),
            topic: sel && sel.topic, type: sel && sel.type, kind: sel && sel.kind,
            frameId: lastFrameId, frame: lastFrame, timeBase: cfg.timeBase, stampOff, clockRate,
            despike: cfg.despike !== 'off' ? (unordered ? 'unordered' : 'on') : 'off',
            zone: zoneStatus(),
            rate: lastRate, totalMsgs,
            surface: {
                cell: surface.cell, maxTiles: surface.maxTiles, tileCells: TILE, layers: cfg.layers,
                tiles: surface.size, filled: surface.filled,
                chunks: volume.size, maxChunks: volume.maxChunks,
                coverage
            },
            anchor
        }
    });
}

setInterval(postTiles, TILES_MS);
setInterval(() => {
    lastRate = { ...rate };
    for (const k of Object.keys(rate)) rate[k] = 0;
    postStatus();
}, 1000);

// ============== CONFIG ==============
function applyConfig(c) {
    const resub = c.transport !== undefined && c.transport !== cfg.transport
        || c.rateHz !== undefined && c.rateHz !== cfg.rateHz;
    Object.assign(cfg, c);
    eulerToMatrix(cfg.mountRoll * DEG, cfg.mountPitch * DEG, cfg.mountYaw * DEG, Rm);
    lever[0] = cfg.leverX; lever[1] = cfg.leverY; lever[2] = cfg.leverZ;
    if (surface.configure({ cell: cfg.cell, memory: cfg.memory, maxTiles: cfg.maxTiles })) resetPending = true;
    // A chunk (16³ voxels) costs as much memory as ~2 tiles
    if (volume.configure({ cell: cfg.cell, memory: cfg.memory, maxChunks: Math.max(64, Math.round(cfg.maxTiles / 2)) })) resetPending = true;
    if (resub && sub) subscribe();
}
applyConfig({});

function clear() {
    surface.reset();
    volume.reset();
    anchor = null;
    blocked = null;
    resetPending = true;
    postTiles();
}

self.onmessage = (e) => {
    const m = e.data || {};
    switch (m.op) {
        case 'pose':
            if (m.p) pushPose(m.p, m.att, m.pos); else poseReason = m.reason || 'NO POSE';
            break;
        case 'connect': connect(m.url); break;
        case 'disconnect': disconnect(); postStatus(); break;
        case 'topics': listTopics(); break;
        case 'select':
            sel = m.topic ? { topic: m.topic, type: m.type, kind: kindOf(m.type) } : null;
            zoneMeta = null;
            dirSeen.fill(0);
            subscribe();
            postStatus();
            break;
        case 'config': applyConfig(m.cfg || {}); break;
        case 'clear': clear(); break;
        case 'areas': areas = Array.isArray(m.areas) ? m.areas : []; break;
    }
};
