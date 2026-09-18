/**
 * lidar-core.js - Livox Mid-360 / Mid-360S point-cloud client (worker thread)
 *
 * Runs inside lidar-worker.js, off the Electron main thread: that thread is
 * also Chromium's browser process, and 2 000 UDP callbacks per second plus
 * the per-packet maths there starved the compositor (renderer dropped to
 * ~1 FPS). Everything network- and CPU-bound lives here; lidar-manager.js on
 * the main thread only relays IPC. No electron import in this file.
 *
 * The GCS talks to the LiDAR directly over the IP link (a LAN bridge to the
 * aircraft): MAVLink telemetry keeps flowing on its own channel, this module
 * opens the Livox SDK2 UDP protocol next to it and georeferences every point
 * with the vehicle pose from that telemetry.
 *
 *   LiDAR (192.168.1.1xx)                 host (this PC)
 *     56000  ◀── search (cmd 0x0000) ──   56000   discovery, unicast + broadcast
 *     56100  ◀── config (cmd 0x0100) ──   56101   host IP/ports, work mode, data type
 *     56200  ── push msg (0x0102) ──▶     56201   lidar state (ignored)
 *     56300  ── point packets ──▶         56301   ~2100 packets/s × 96 points
 *     56400  ── IMU packets ──▶           56401   200 Hz (received, ignored)
 *
 * Point packets carry points in the LiDAR frame (X forward, Y left, Z up,
 * origin at the optical centre). Each packet is turned into world points with
 * the chain
 *
 *   p_frd  = (x, -y, -z)                       Livox FLU → aircraft FRD
 *   p_body = R_mount · p_frd + lever_arm       mount attitude + offset from the IMU
 *   p_ned  = R_att(roll, pitch, yaw) · p_body  ATTITUDE (30) at the packet time
 *   ENU    = pos(t) + (p_ned.e, p_ned.n, -p_ned.d)   GLOBAL_POSITION_INT (33)
 *
 * where pos(t) and att(t) are interpolated in time from a pose ring buffer,
 * because telemetry runs at 3-10 Hz and packets at 2 kHz. This is the direct
 * georeferencing every airborne LiDAR pipeline uses (DJI L1/L2 live preview,
 * YellowScan LiveStation); the difference from a survey product is the time
 * base: with no PPS into the Mid-360 both streams are aligned on GCS arrival
 * time, so a configurable "telemetry lag" absorbs the radio delay.
 *
 * Accumulation is gated on navigation quality (GPS fix / sats / HDOP and the
 * ArduPilot EKF flags + variances) and decimated through a voxel grid — one
 * point per voxel, like the map downsampling in FAST-LIO — so a long flight
 * stays within a few million points for the renderer and for the .ply export.
 *
 * Packets that cannot be georeferenced (gate closed: no GPS, EKF not
 * converged, map full…) are not thrown away: a subsample goes to the renderer
 * as LIVE points, relative to the vehicle (level NED frame when attitude is
 * known, body frame otherwise), where they fade out after a few seconds. The
 * operator always sees what the sensor sees; only the georeferenced points
 * are kept.
 */

const dgram = require('dgram');
const fs = require('fs');
const os = require('os');
const path = require('path');

// ============== LIVOX SDK2 PROTOCOL ==============
const LIDAR_PORT = { search: 56000, cmd: 56100, push: 56200, point: 56300, imu: 56400 };
const HOST_PORT  = { search: 56000, cmd: 56101, push: 56201, point: 56301, imu: 56401 };

const CMD_SEARCH       = 0x0000;
const CMD_WORK_MODE    = 0x0100;   // "LidarWorkModeControl": key/value parameter set

const KEY_PCL_DATA_TYPE     = 0x0000;
const KEY_STATE_HOST_IP     = 0x0005;
const KEY_POINT_HOST_IP     = 0x0006;
const KEY_IMU_HOST_IP       = 0x0007;
const KEY_WORK_MODE         = 0x001A;

const WORK_MODE_NORMAL = 0x01;

const DEV_TYPE_NAMES = { 9: 'Mid-360', 35: 'Mid-360S', 15: 'HAP', 40: 'Avia2' };

const PKT_HEADER = 36;              // LivoxLidarEthernetPacket header
const SDK_HEADER = 24;              // SdkPacket header

// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over the first 18 header
// bytes, CRC-32 (IEEE, reflected) over the payload — the two FastCRC routines
// SDK2 uses.
const CRC16_TABLE = new Uint16Array(256);
for (let i = 0; i < 256; i++) {
    let c = i << 8;
    for (let k = 0; k < 8; k++) c = (c & 0x8000) ? ((c << 1) ^ 0x1021) & 0xFFFF : (c << 1) & 0xFFFF;
    CRC16_TABLE[i] = c;
}
function crc16(buf, len) {
    let crc = 0xFFFF;
    for (let i = 0; i < len; i++) crc = ((crc << 8) & 0xFFFF) ^ CRC16_TABLE[((crc >> 8) ^ buf[i]) & 0xFF];
    return crc;
}
const CRC32_TABLE = new Uint32Array(256);
for (let i = 0; i < 256; i++) {
    let c = i;
    for (let k = 0; k < 8; k++) c = (c & 1) ? (0xEDB88320 ^ (c >>> 1)) : (c >>> 1);
    CRC32_TABLE[i] = c >>> 0;
}
function crc32(buf, off = 0, len = buf.length - off) {
    let crc = 0xFFFFFFFF;
    for (let i = off; i < off + len; i++) crc = CRC32_TABLE[(crc ^ buf[i]) & 0xFF] ^ (crc >>> 8);
    return (crc ^ 0xFFFFFFFF) >>> 0;
}

let seqCounter = 1;
function packCommand(cmdId, data = Buffer.alloc(0)) {
    const buf = Buffer.alloc(SDK_HEADER + data.length);
    buf[0] = 0xAA;                                   // sof
    buf[1] = 0;                                      // protocol version
    buf.writeUInt16LE(SDK_HEADER + data.length, 2);  // whole frame length
    buf.writeUInt32LE((seqCounter++) & 0xFFFF, 4);   // seq_num
    buf.writeUInt16LE(cmdId, 8);
    buf[10] = 0;                                     // cmd_type: request
    buf[11] = 0;                                     // sender: host
    // rsvd[6] @12
    buf.writeUInt16LE(crc16(buf, 18), 18);
    buf.writeUInt32LE(data.length ? crc32(data) : 0, 20);
    data.copy(buf, SDK_HEADER);
    return buf;
}

function parseCommand(buf) {
    if (buf.length < SDK_HEADER || buf[0] !== 0xAA) return null;
    const length = buf.readUInt16LE(2);
    if (length > buf.length) return null;
    if (crc16(buf, 18) !== buf.readUInt16LE(18)) return null;
    const data = buf.subarray(SDK_HEADER, length);
    if (data.length && crc32(data) !== buf.readUInt32LE(20)) return null;
    return {
        seq: buf.readUInt32LE(4),
        cmdId: buf.readUInt16LE(8),
        cmdType: buf[10],      // 0 request, 1 ack
        senderType: buf[11],   // 0 host, 1 lidar
        data
    };
}

// Key/value parameter block: u16 key_num, u16 rsvd, then {u16 key, u16 len, bytes}*
function buildKvRequest(entries) {
    let size = 4;
    for (const e of entries) size += 4 + e.value.length;
    const buf = Buffer.alloc(size);
    buf.writeUInt16LE(entries.length, 0);
    let o = 4;
    for (const e of entries) {
        buf.writeUInt16LE(e.key, o);
        buf.writeUInt16LE(e.value.length, o + 2);
        e.value.copy(buf, o + 4);
        o += 4 + e.value.length;
    }
    return buf;
}

function hostIpValue(ip, hostPort, lidarPort) {
    const v = Buffer.alloc(8);
    const parts = ip.split('.').map(n => parseInt(n, 10));
    for (let i = 0; i < 4; i++) v[i] = parts[i] & 0xFF;
    v.writeUInt16LE(hostPort, 4);
    v.writeUInt16LE(lidarPort, 6);
    return v;
}

// ============== CONFIG ==============
const DEFAULT_CONFIG = {
    lidarIp: '192.168.1.12',
    hostIp: 'auto',        // IPv4 the LiDAR must send to; 'auto' = interface on the LiDAR's /24
    dataType: 1,           // 1 = Cartesian 32-bit (mm), 2 = Cartesian 16-bit (cm) — halves the bandwidth
    // Mount attitude of the LiDAR relative to the autopilot body frame (FRD),
    // ZYX Euler in degrees, and the lever arm IMU → LiDAR optical centre (m, FRD).
    mountRoll: 0, mountPitch: 0, mountYaw: 0,
    leverX: 0, leverY: 0, leverZ: 0,
    lagMs: 0,              // telemetry arrives this much later than the point packets
    minRange: 2.5,         // m — rejects the airframe: rudder, gear, struts, antennas in the FOV
    maxRange: 70,          // m — Mid-360 rated range
    dropNoise: true,       // tag bits 0-3: spatial / intensity noise flags
    voxel: 0.25,           // m — one point kept per voxel
    maxPoints: 3000000,
    // Navigation-quality gate
    minFix: 3,             // GPS_FIX_TYPE: 3 = 3D, 4 = DGPS, 5 = RTK float, 6 = RTK fixed
    maxHdop: 2.0,
    minSats: 8,
    requireEkf: true,      // EKF_STATUS_REPORT flags + variances (ArduPilot)
    liveSeconds: 3,        // non-georeferenced points stay on screen this long
    recordRaw: false       // stream every accepted point (pre-voxel) to a .ply
};
let config = { ...DEFAULT_CONFIG };

// ============== RUNTIME STATE ==============
let sockets = null;        // { search, cmd, push, point, imu }
let connected = false;     // sockets open, handshake running
let lidar = { ip: null, sn: null, devType: 0, configured: false, lastAck: 0, lastPacket: 0 };
let handshakeTimer = null;
let processTimer = null;
let statusTimer = null;

// Packets are held ~lag + 60 ms before georeferencing so the pose buffer has
// samples on both sides of the measurement time (interpolation, not extrapolation).
let packetQueue = [];
const MAX_QUEUE = 6000;

// Pose ring buffers (host arrival time in ms).
const POSE_CAP = 1024;
const att = { t: new Float64Array(POSE_CAP), r: new Float32Array(POSE_CAP), p: new Float32Array(POSE_CAP), y: new Float32Array(POSE_CAP), n: 0, head: 0 };
const pos = { t: new Float64Array(POSE_CAP), lat: new Float64Array(POSE_CAP), lon: new Float64Array(POSE_CAP), alt: new Float64Array(POSE_CAP),
              vn: new Float32Array(POSE_CAP), ve: new Float32Array(POSE_CAP), vd: new Float32Array(POSE_CAP), n: 0, head: 0 };
const nav = { fix: 0, sats: 0, hdop: 99, gpsTime: 0, ekfFlags: 0, ekfPosVar: 0, ekfVelVar: 0, ekfTime: 0 };

// Cloud origin — set at the first accepted point, cleared with the map. The
// epoch stamps every origin/points event so the renderer can drop a batch
// that was already in flight when the map was cleared.
let origin = null;         // { lat, lon, alt, mPerLat, mPerLon, epoch }
let epoch = 0;

// Voxel occupancy: open-addressing hash of packed voxel indices (17 bits per axis).
let voxelKeys = null;      // Float64Array, 0 = empty, stored key + 1
let voxelMask = 0;
let voxelCount = 0;

// Accumulated map (ENU metres from origin + intensity), chunked so it can
// grow without copying; this is what SAVE MAP writes out.
const MAP_CHUNK = 262144;
let mapChunks = [];        // [{ xyz: Float32Array, i: Uint8Array, n }]
let mapCount = 0;

// Batch towards the renderer
let batchXyz = new Float32Array(32768 * 3);
let batchInt = new Uint8Array(32768);
let batchN = 0;
let lastFlush = 0;

// Live (non-georeferenced) batch: vehicle-relative points, subsampled.
const LIVE_PER_PKT = 16;   // of 96 → ~33 kpts/s at the full rate
let liveXyz = new Float32Array(16384 * 3);
let liveInt = new Uint8Array(16384);
let liveN = 0;
let liveFrame = 'body';    // 'ned' when attitude is fresh, 'body' otherwise

// Stats (per second)
let stats = { packets: 0, points: 0, accepted: 0, gated: 0, filtered: 0, live: 0, bytes: 0 };
let statsShown = { pps: 0, pointsPs: 0, acceptedPs: 0, gatedPs: 0, filteredPs: 0, livePs: 0, kbps: 0 };
let statsT0 = 0;
let gate = { ok: false, reason: 'NO LINK' };
let lastError = '';

// Raw recording
let rec = null;            // { fd, path, count, buf, off, countPos }

// ============== SMALL MATH ==============
const DEG = Math.PI / 180;

// ZYX Euler → rotation matrix (row-major 3×3), maps the rotated frame into the
// parent frame: body→NED for the attitude, lidar→body for the mount.
function eulerToMatrix(roll, pitch, yaw, out) {
    const cr = Math.cos(roll), sr = Math.sin(roll);
    const cp = Math.cos(pitch), sp = Math.sin(pitch);
    const cy = Math.cos(yaw), sy = Math.sin(yaw);
    out[0] = cy * cp; out[1] = cy * sp * sr - sy * cr; out[2] = cy * sp * cr + sy * sr;
    out[3] = sy * cp; out[4] = sy * sp * sr + cy * cr; out[5] = sy * sp * cr - cy * sr;
    out[6] = -sp;     out[7] = cp * sr;                out[8] = cp * cr;
    return out;
}
const R_mount = new Float64Array(9);
const R_att = new Float64Array(9);
let mountDirty = true;

function metersPerDegree(latDeg) {
    const f = latDeg * DEG;
    return {
        mPerLat: 111132.954 - 559.822 * Math.cos(2 * f) + 1.175 * Math.cos(4 * f),
        mPerLon: 111412.84 * Math.cos(f) - 93.5 * Math.cos(3 * f) + 0.118 * Math.cos(5 * f)
    };
}

function lerpAngle(a, b, f) {
    let d = b - a;
    while (d > Math.PI) d -= 2 * Math.PI;
    while (d < -Math.PI) d += 2 * Math.PI;
    return a + d * f;
}

// ============== POSE BUFFER ==============
function pushAtt(t, r, p, y) {
    const i = att.head;
    att.t[i] = t; att.r[i] = r; att.p[i] = p; att.y[i] = y;
    att.head = (i + 1) % POSE_CAP;
    if (att.n < POSE_CAP) att.n++;
}
function pushPos(t, lat, lon, alt, vn, ve, vd) {
    const i = pos.head;
    pos.t[i] = t; pos.lat[i] = lat; pos.lon[i] = lon; pos.alt[i] = alt;
    pos.vn[i] = vn; pos.ve[i] = ve; pos.vd[i] = vd;
    pos.head = (i + 1) % POSE_CAP;
    if (pos.n < POSE_CAP) pos.n++;
}

// Index of the newest sample with t <= time, walking back from head. Returns
// -1 when the buffer is empty or every sample is newer than `time`.
function findBefore(buf, time) {
    for (let k = 1; k <= buf.n; k++) {
        const i = (buf.head - k + POSE_CAP) % POSE_CAP;
        if (buf.t[i] <= time) return i;
    }
    return -1;
}

const MAX_EXTRAP_MS = 400;
const poseOut = { lat: 0, lon: 0, alt: 0, roll: 0, pitch: 0, yaw: 0 };

function poseAt(time) {
    if (att.n === 0 || pos.n === 0) return null;
    // Attitude
    let i = findBefore(att, time);
    if (i < 0) return null;
    const j = (i + 1) % POSE_CAP;
    const hasNext = att.n > 1 && j !== att.head && att.t[j] > att.t[i];
    if (hasNext) {
        const f = (time - att.t[i]) / (att.t[j] - att.t[i]);
        poseOut.roll = lerpAngle(att.r[i], att.r[j], f);
        poseOut.pitch = lerpAngle(att.p[i], att.p[j], f);
        poseOut.yaw = lerpAngle(att.y[i], att.y[j], f);
    } else {
        if (time - att.t[i] > MAX_EXTRAP_MS) return null;
        poseOut.roll = att.r[i]; poseOut.pitch = att.p[i]; poseOut.yaw = att.y[i];
    }
    // Position
    i = findBefore(pos, time);
    if (i < 0) return null;
    const jp = (i + 1) % POSE_CAP;
    const hasNextP = pos.n > 1 && jp !== pos.head && pos.t[jp] > pos.t[i];
    if (hasNextP) {
        const f = (time - pos.t[i]) / (pos.t[jp] - pos.t[i]);
        poseOut.lat = pos.lat[i] + (pos.lat[jp] - pos.lat[i]) * f;
        poseOut.lon = pos.lon[i] + (pos.lon[jp] - pos.lon[i]) * f;
        poseOut.alt = pos.alt[i] + (pos.alt[jp] - pos.alt[i]) * f;
    } else {
        const dt = (time - pos.t[i]) / 1000;
        if (dt * 1000 > MAX_EXTRAP_MS) return null;
        // Dead-reckon on the EKF velocity for the gap after the last sample.
        const mpd = metersPerDegree(pos.lat[i]);
        poseOut.lat = pos.lat[i] + (pos.vn[i] * dt) / mpd.mPerLat;
        poseOut.lon = pos.lon[i] + (pos.ve[i] * dt) / mpd.mPerLon;
        poseOut.alt = pos.alt[i] - pos.vd[i] * dt;
    }
    return poseOut;
}

// ============== MAVLINK TAP ==============
// Called by main-mavlink.js for every decoded message (live link only).
const EKF_ATTITUDE = 1, EKF_VELOCITY_HORIZ = 2, EKF_POS_HORIZ_ABS = 16, EKF_POS_VERT_ABS = 32;
const EKF_REQUIRED = EKF_ATTITUDE | EKF_VELOCITY_HORIZ | EKF_POS_HORIZ_ABS | EKF_POS_VERT_ABS;

function onMavlinkMessage(msgId, data) {
    if (!connected) return;
    const now = Date.now();
    switch (msgId) {
        case 30: // ATTITUDE
            if (Number.isFinite(data.roll) && Number.isFinite(data.pitch) && Number.isFinite(data.yaw))
                pushAtt(now, data.roll, data.pitch, data.yaw);
            break;
        case 33: // GLOBAL_POSITION_INT
            if (Number.isFinite(data.lat) && Number.isFinite(data.lon) && data.lat !== 0 && data.lon !== 0)
                pushPos(now, data.lat / 1e7, data.lon / 1e7, (data.alt || 0) / 1000,
                        (data.vx || 0) / 100, (data.vy || 0) / 100, (data.vz || 0) / 100);
            break;
        case 24: // GPS_RAW_INT
            if (Number.isFinite(data.fixType)) {
                nav.fix = data.fixType;
                nav.sats = data.satellitesVisible || 0;
                nav.hdop = (Number.isFinite(data.eph) && data.eph !== 65535) ? data.eph / 100 : 99;
                nav.gpsTime = now;
            }
            break;
        case 193: // EKF_STATUS_REPORT
            if (Number.isFinite(data.flags)) {
                nav.ekfFlags = data.flags;
                nav.ekfPosVar = Math.max(data.posHorizVariance || 0, data.posVertVariance || 0);
                nav.ekfVelVar = data.velocityVariance || 0;
                nav.ekfTime = now;
            }
            break;
    }
}

function evaluateGate(now) {
    const FRESH = 2000;
    if (!connected) return { ok: false, reason: 'NO LINK' };
    if (pos.n === 0 || now - pos.t[(pos.head - 1 + POSE_CAP) % POSE_CAP] > FRESH) return { ok: false, reason: 'NO POSITION' };
    if (att.n === 0 || now - att.t[(att.head - 1 + POSE_CAP) % POSE_CAP] > FRESH) return { ok: false, reason: 'NO ATTITUDE' };
    if (now - nav.gpsTime > 5000) return { ok: false, reason: 'NO GPS' };
    if (nav.fix < config.minFix) return { ok: false, reason: `GPS FIX ${nav.fix} < ${config.minFix}` };
    if (nav.sats < config.minSats) return { ok: false, reason: `SATS ${nav.sats} < ${config.minSats}` };
    if (nav.hdop > config.maxHdop) return { ok: false, reason: `HDOP ${nav.hdop.toFixed(1)} > ${config.maxHdop}` };
    if (config.requireEkf) {
        if (nav.ekfTime === 0) return { ok: false, reason: 'NO EKF REPORT' };
        if (now - nav.ekfTime > 5000) return { ok: false, reason: 'EKF STALE' };
        if ((nav.ekfFlags & EKF_REQUIRED) !== EKF_REQUIRED) return { ok: false, reason: 'EKF NOT CONVERGED' };
        if (nav.ekfPosVar > 0.5 || nav.ekfVelVar > 0.5) return { ok: false, reason: 'EKF VARIANCE HIGH' };
    }
    if (voxelCount >= config.maxPoints) return { ok: false, reason: 'MAP FULL' };
    return { ok: true, reason: 'ACCUMULATING' };
}

// ============== VOXEL MAP ==============
function resetMap() {
    voxelKeys = null;
    voxelCount = 0;
    mapChunks = [];
    mapCount = 0;
    origin = null;
    batchN = 0;
}

function ensureVoxelTable() {
    if (voxelKeys) return;
    let size = 1 << 16;
    while (size < config.maxPoints * 2) size <<= 1;
    voxelKeys = new Float64Array(size);
    voxelMask = size - 1;
}

// Returns true when the voxel was free (and marks it). Key packs three 17-bit
// indices (±16 km at 0.25 m) into 51 bits, safely below 2^53.
function claimVoxel(e, n, u) {
    if (!voxelKeys) ensureVoxelTable();
    const inv = 1 / config.voxel;
    const ix = Math.floor(e * inv) + 65536;
    const iy = Math.floor(n * inv) + 65536;
    const iz = Math.floor(u * inv) + 65536;
    if (ix < 0 || iy < 0 || iz < 0 || ix >= 131072 || iy >= 131072 || iz >= 131072) return false;
    const key = (ix * 131072 + iy) * 131072 + iz + 1;
    // Multiplicative hash on the low bits, linear probing.
    let h = ((ix * 73856093) ^ (iy * 19349663) ^ (iz * 83492791)) & voxelMask;
    for (;;) {
        const k = voxelKeys[h];
        if (k === 0) { voxelKeys[h] = key; voxelCount++; return true; }
        if (k === key) return false;
        h = (h + 1) & voxelMask;
    }
}

function mapPush(e, n, u, intensity) {
    let c = mapChunks[mapChunks.length - 1];
    if (!c || c.n >= MAP_CHUNK) {
        c = { xyz: new Float32Array(MAP_CHUNK * 3), i: new Uint8Array(MAP_CHUNK), n: 0 };
        mapChunks.push(c);
    }
    const o = c.n * 3;
    c.xyz[o] = e; c.xyz[o + 1] = n; c.xyz[o + 2] = u;
    c.i[c.n] = intensity;
    c.n++;
    mapCount++;
}

function batchPush(e, n, u, intensity) {
    if (batchN >= batchInt.length) flushBatch();
    const o = batchN * 3;
    batchXyz[o] = e; batchXyz[o + 1] = n; batchXyz[o + 2] = u;
    batchInt[batchN] = intensity;
    batchN++;
}

function flushBatch() {
    lastFlush = Date.now();
    if (batchN > 0) {
        const enu = batchXyz.slice(0, batchN * 3);
        const intensity = batchInt.slice(0, batchN);
        emit('lidar-points', { epoch, enu, intensity }, [enu.buffer, intensity.buffer]);
        batchN = 0;
    }
    if (liveN > 0) {
        const xyz = liveXyz.slice(0, liveN * 3);
        const intensity = liveInt.slice(0, liveN);
        emit('lidar-live', { frame: liveFrame, ttl: config.liveSeconds, xyz, intensity }, [xyz.buffer, intensity.buffer]);
        liveN = 0;
    }
}

// ============== POINT PACKETS ==============
function onPointPacket(buf, rxTime) {
    if (buf.length < PKT_HEADER) return;
    const dataType = buf[10];
    if (dataType === 0) return;                    // IMU packet on the wrong port
    stats.packets++;
    stats.bytes += buf.length;
    lidar.lastPacket = rxTime;
    if (packetQueue.length >= MAX_QUEUE) packetQueue.shift();
    packetQueue.push({ buf, t: rxTime });
}

function processQueue() {
    const now = Date.now();
    const hold = config.lagMs + 60;
    const g = evaluateGate(now);
    if (g.ok !== gate.ok || g.reason !== gate.reason) {
        gate = g;
        sendStatus();
    }
    if (mountDirty) {
        eulerToMatrix(config.mountRoll * DEG, config.mountPitch * DEG, config.mountYaw * DEG, R_mount);
        mountDirty = false;
    }

    let processed = 0;
    while (packetQueue.length && packetQueue[0].t + hold <= now && processed < 600) {
        const { buf, t } = packetQueue.shift();
        processed++;
        const dotNum = buf.readUInt16LE(5);
        const dataType = buf[10];
        stats.points += dotNum;
        if (!gate.ok) { stats.gated += dotNum; livePacket(buf, dotNum, dataType); continue; }

        // Pose at the packet's (corrected) measurement time.
        const pose = poseAt(t + config.lagMs);
        if (!pose) { stats.gated += dotNum; livePacket(buf, dotNum, dataType); continue; }

        if (!origin) {
            const mpd = metersPerDegree(pose.lat);
            origin = { lat: pose.lat, lon: pose.lon, alt: pose.alt, mPerLat: mpd.mPerLat, mPerLon: mpd.mPerLon, epoch: ++epoch };
            ensureVoxelTable();
            sendOrigin();
        }
        eulerToMatrix(pose.roll, pose.pitch, pose.yaw, R_att);
        const vehE = (pose.lon - origin.lon) * origin.mPerLon;
        const vehN = (pose.lat - origin.lat) * origin.mPerLat;
        const vehU = pose.alt - origin.alt;

        georeferencePacket(buf, dotNum, dataType, vehE, vehN, vehU, t);
    }

    if ((batchN > 0 || liveN > 0) && now - lastFlush >= 50) flushBatch();
}

// Point decoding shared by the georeferenced and the live paths.
const pt = { x: 0, y: 0, z: 0, refl: 0, tag: 0 };
function readPoint(buf, o, dataType) {
    if (dataType === 1) {
        pt.x = buf.readInt32LE(o) * 0.001; pt.y = buf.readInt32LE(o + 4) * 0.001; pt.z = buf.readInt32LE(o + 8) * 0.001;
        pt.refl = buf[o + 12]; pt.tag = buf[o + 13];
    } else if (dataType === 2) {
        pt.x = buf.readInt16LE(o) * 0.01; pt.y = buf.readInt16LE(o + 2) * 0.01; pt.z = buf.readInt16LE(o + 4) * 0.01;
        pt.refl = buf[o + 6]; pt.tag = buf[o + 7];
    } else {
        const d = buf.readUInt32LE(o) * 0.001;
        const theta = buf.readUInt16LE(o + 4) * 0.01 * DEG;   // zenith
        const phi = buf.readUInt16LE(o + 6) * 0.01 * DEG;     // azimuth
        pt.x = d * Math.sin(theta) * Math.cos(phi); pt.y = d * Math.sin(theta) * Math.sin(phi); pt.z = d * Math.cos(theta);
        pt.refl = buf[o + 8]; pt.tag = buf[o + 9];
    }
}

// Gate closed: a subsample of the packet, vehicle-relative. With a fresh
// attitude the points are levelled into NED around the vehicle (the picture
// stays upright while the aircraft banks); without one they stay in the body
// frame and the renderer rotates them with the model.
function livePacket(buf, dotNum, dataType) {
    const stride = dataType === 1 ? 14 : dataType === 2 ? 8 : dataType === 3 ? 10 : 0;
    if (!stride) return;
    const n = Math.min(dotNum, Math.floor((buf.length - PKT_HEADER) / stride));
    const step = Math.max(1, Math.floor(n / LIVE_PER_PKT));
    const rMin2 = minR2(), rMax2 = maxR2();
    const lx = config.leverX, ly = config.leverY, lz = config.leverZ;
    const m = R_mount;
    const now = Date.now();
    const iAtt = att.n ? (att.head - 1 + POSE_CAP) % POSE_CAP : -1;
    const levelled = iAtt >= 0 && now - att.t[iAtt] < 2000;
    if (levelled) eulerToMatrix(att.r[iAtt], att.p[iAtt], att.y[iAtt], R_att);
    const frame = levelled ? 'ned' : 'body';
    if (frame !== liveFrame && liveN > 0) flushBatch();
    liveFrame = frame;
    const a = R_att;

    for (let k = 0; k < n; k += step) {
        readPoint(buf, PKT_HEADER + k * stride, dataType);
        const r2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
        if (r2 < rMin2 || r2 > rMax2) continue;
        if (config.dropNoise && (pt.tag & 0x0F) !== 0) continue;
        const fx = pt.x, fy = -pt.y, fz = -pt.z;
        let bx = m[0] * fx + m[1] * fy + m[2] * fz + lx;
        let by = m[3] * fx + m[4] * fy + m[5] * fz + ly;
        let bz = m[6] * fx + m[7] * fy + m[8] * fz + lz;
        if (levelled) {
            const pn = a[0] * bx + a[1] * by + a[2] * bz;
            const pe = a[3] * bx + a[4] * by + a[5] * bz;
            const pd = a[6] * bx + a[7] * by + a[8] * bz;
            bx = pn; by = pe; bz = pd;
        }
        if (liveN >= liveInt.length) flushBatch();
        const o = liveN * 3;
        liveXyz[o] = bx; liveXyz[o + 1] = by; liveXyz[o + 2] = bz;
        liveInt[liveN] = pt.refl;
        liveN++;
        stats.live++;
    }
}

const minR2 = () => config.minRange * config.minRange;
const maxR2 = () => config.maxRange * config.maxRange;

function georeferencePacket(buf, dotNum, dataType, vehE, vehN, vehU, rxTime) {
    const stride = dataType === 1 ? 14 : dataType === 2 ? 8 : dataType === 3 ? 10 : 0;
    if (!stride) return;
    const n = Math.min(dotNum, Math.floor((buf.length - PKT_HEADER) / stride));
    const rMin2 = minR2(), rMax2 = maxR2();
    const lx = config.leverX, ly = config.leverY, lz = config.leverZ;
    const m = R_mount, a = R_att;
    const tSec = rxTime / 1000;

    for (let k = 0; k < n; k++) {
        readPoint(buf, PKT_HEADER + k * stride, dataType);
        const x = pt.x, y = pt.y, z = pt.z, refl = pt.refl, tag = pt.tag;
        const r2 = x * x + y * y + z * z;
        if (r2 < rMin2 || r2 > rMax2) { stats.filtered++; continue; }      // also drops the (0,0,0) no-return points
        if (config.dropNoise && (tag & 0x0F) !== 0) { stats.filtered++; continue; }

        // Livox FLU → FRD, mount rotation, lever arm → body FRD
        const fx = x, fy = -y, fz = -z;
        const bx = m[0] * fx + m[1] * fy + m[2] * fz + lx;
        const by = m[3] * fx + m[4] * fy + m[5] * fz + ly;
        const bz = m[6] * fx + m[7] * fy + m[8] * fz + lz;
        // body → NED, then ENU from the vehicle position
        const pn = a[0] * bx + a[1] * by + a[2] * bz;
        const pe = a[3] * bx + a[4] * by + a[5] * bz;
        const pd = a[6] * bx + a[7] * by + a[8] * bz;
        const E = vehE + pe, N = vehN + pn, U = vehU - pd;

        if (rec) recordPoint(E, N, U, refl, tSec);
        if (!claimVoxel(E, N, U)) continue;
        stats.accepted++;
        mapPush(E, N, U, refl);
        batchPush(E, N, U, refl);
    }
}

// ============== PLY WRITER ==============
// Binary little-endian PLY, count patched in place at close. Points are ENU
// metres from the origin recorded in the header — CloudCompare / QGIS turn
// that into a projected CRS with a global shift.
const COUNT_FIELD = '0000000000';

function openPly(filePath, withTime) {
    const header = [
        'ply',
        'format binary_little_endian 1.0',
        'comment Corv-GCS Livox point cloud',
        'comment frame ENU metres, x=east y=north z=up, relative to origin',
        `comment origin_lat ${origin ? origin.lat.toFixed(8) : 'unset'} origin_lon ${origin ? origin.lon.toFixed(8) : 'unset'} origin_alt_msl ${origin ? origin.alt.toFixed(3) : 'unset'}`,
        `comment created ${new Date().toISOString()}`,
        `comment lidar ${lidar.sn || 'unknown'} ${DEV_TYPE_NAMES[lidar.devType] || ''}`,
        `element vertex ${COUNT_FIELD}`,
        'property float x',
        'property float y',
        'property float z',
        'property uchar intensity'
    ];
    if (withTime) header.push('property double time');
    header.push('end_header');
    const text = header.join('\n') + '\n';
    const countPos = text.indexOf(COUNT_FIELD);
    const fd = fs.openSync(filePath, 'w');
    fs.writeSync(fd, text);
    return { fd, path: filePath, count: 0, countPos, buf: Buffer.alloc(1 << 16), off: 0, withTime };
}

function plyWrite(w, E, N, U, intensity, tSec) {
    const need = w.withTime ? 21 : 13;
    if (w.off + need > w.buf.length) plyFlush(w);
    w.buf.writeFloatLE(E, w.off); w.buf.writeFloatLE(N, w.off + 4); w.buf.writeFloatLE(U, w.off + 8);
    w.buf[w.off + 12] = intensity;
    if (w.withTime) w.buf.writeDoubleLE(tSec, w.off + 13);
    w.off += need;
    w.count++;
}

function plyFlush(w) {
    if (w.off > 0) { fs.writeSync(w.fd, w.buf, 0, w.off); w.off = 0; }
}

function closePly(w) {
    plyFlush(w);
    fs.writeSync(w.fd, String(w.count).padStart(COUNT_FIELD.length, '0'), w.countPos);
    fs.closeSync(w.fd);
}

function recordPoint(E, N, U, refl, tSec) {
    plyWrite(rec, E, N, U, refl, tSec);
}

let dataRoot = null;   // set by the host (worker data / test): <data root>/lidar/ holds the files
function lidarDir() {
    const dir = path.join(dataRoot || path.join(os.homedir(), 'corv-gcs-data'), 'lidar');
    fs.mkdirSync(dir, { recursive: true });
    return dir;
}

function stamp() {
    return new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
}

function startRecording() {
    if (rec) return rec.path;
    const filePath = path.join(lidarDir(), `raw-${stamp()}.ply`);
    rec = openPly(filePath, true);
    return filePath;
}

function stopRecording() {
    if (!rec) return null;
    const p = rec.path;
    closePly(rec);
    rec = null;
    return p;
}

function saveMap() {
    if (!origin || mapCount === 0) return { success: false, error: 'Map is empty' };
    const filePath = path.join(lidarDir(), `map-${stamp()}.ply`);
    const w = openPly(filePath, false);
    for (const c of mapChunks) {
        for (let k = 0; k < c.n; k++) plyWrite(w, c.xyz[k * 3], c.xyz[k * 3 + 1], c.xyz[k * 3 + 2], c.i[k], 0);
    }
    closePly(w);
    return { success: true, path: filePath, points: mapCount };
}

// ============== NETWORK ==============
function pickHostIp(lidarIp) {
    if (config.hostIp && config.hostIp !== 'auto') return config.hostIp;
    const prefix = lidarIp.split('.').slice(0, 3).join('.') + '.';
    let fallback = null;
    for (const list of Object.values(os.networkInterfaces())) {
        for (const ni of list) {
            if (ni.family !== 'IPv4' || ni.internal) continue;
            if (ni.address.startsWith(prefix)) return ni.address;
            if (!fallback) fallback = ni.address;
        }
    }
    return fallback || '0.0.0.0';
}

function listInterfaces() {
    const out = [];
    for (const [name, list] of Object.entries(os.networkInterfaces())) {
        for (const ni of list) {
            if (ni.family === 'IPv4' && !ni.internal) out.push({ name, address: ni.address });
        }
    }
    return out;
}

function bindUdp(port, onMessage) {
    return new Promise((resolve, reject) => {
        const s = dgram.createSocket({ type: 'udp4', reuseAddr: true });
        s.on('error', (e) => { lastError = `${port}: ${e.message}`; reject(e); });
        s.on('message', onMessage);
        s.bind(port, '0.0.0.0', () => {
            try { s.setRecvBufferSize(4 * 1024 * 1024); } catch (_) {}
            resolve(s);
        });
    });
}

async function connect() {
    if (connected) return { success: true };
    resetPose();
    packetQueue = [];
    stats = { packets: 0, points: 0, accepted: 0, gated: 0, filtered: 0, live: 0, bytes: 0 };
    statsT0 = Date.now();
    lidar = { ip: config.lidarIp, sn: null, devType: 0, configured: false, lastAck: 0, lastPacket: 0 };
    lastError = '';

    try {
        const search = await bindUdp(HOST_PORT.search, onSearchReply).catch(async (e) => {
            // Livox Viewer or a second GCS may hold 56000 — any port works for
            // the reply, which comes back to the sender's address.
            console.warn('[lidar] port 56000 busy, using ephemeral search socket:', e.message);
            return bindUdp(0, onSearchReply);
        });
        const cmd = await bindUdp(HOST_PORT.cmd, onCmdReply);
        const push = await bindUdp(HOST_PORT.push, () => {});
        const point = await bindUdp(HOST_PORT.point, (msg) => onPointPacket(msg, Date.now()));
        const imu = await bindUdp(HOST_PORT.imu, () => {});
        try { search.setBroadcast(true); } catch (_) {}
        sockets = { search, cmd, push, point, imu };
    } catch (e) {
        closeSockets();
        return { success: false, error: `UDP bind failed (${e.message}) — is Livox Viewer or another GCS running?` };
    }

    connected = true;
    mountDirty = true;
    if (config.recordRaw) startRecording();
    handshakeTimer = setInterval(handshakeTick, 1000);
    processTimer = setInterval(processQueue, 20);
    statusTimer = setInterval(() => { rollStats(); sendStatus(); }, 500);
    handshakeTick();
    sendStatus();
    console.log(`[lidar] connecting to ${config.lidarIp}, host ${pickHostIp(config.lidarIp)}`);
    return { success: true };
}

function closeSockets() {
    if (!sockets) return;
    for (const s of Object.values(sockets)) { try { s.close(); } catch (_) {} }
    sockets = null;
}

function disconnect() {
    if (!connected) return { success: true };
    connected = false;
    clearInterval(handshakeTimer); handshakeTimer = null;
    clearInterval(processTimer); processTimer = null;
    clearInterval(statusTimer); statusTimer = null;
    flushBatch();
    stopRecording();
    closeSockets();
    packetQueue = [];
    gate = { ok: false, reason: 'NO LINK' };
    sendStatus();
    console.log('[lidar] disconnected');
    return { success: true };
}

function resetPose() {
    att.n = 0; att.head = 0;
    pos.n = 0; pos.head = 0;
    nav.fix = 0; nav.sats = 0; nav.hdop = 99; nav.gpsTime = 0;
    nav.ekfFlags = 0; nav.ekfPosVar = 0; nav.ekfVelVar = 0; nav.ekfTime = 0;
}

// Discovery + configuration retried every second until the LiDAR acks and
// packets flow; re-sent if packets stop (LiDAR rebooted, link dropped).
function handshakeTick() {
    if (!sockets) return;
    const now = Date.now();
    const streaming = lidar.lastPacket && now - lidar.lastPacket < 2000;
    if (lidar.configured && streaming) return;
    if (lidar.configured && !streaming && now - lidar.lastAck > 5000) lidar.configured = false;

    const searchPkt = packCommand(CMD_SEARCH);
    sockets.search.send(searchPkt, LIDAR_PORT.search, config.lidarIp, noop);
    sockets.search.send(searchPkt, LIDAR_PORT.search, '255.255.255.255', noop);
    if (!lidar.configured) sendConfig();
}

const noop = () => {};

function sendConfig() {
    const host = pickHostIp(config.lidarIp);
    if (host === '0.0.0.0') lastError = 'no local IPv4 interface found — set HOST IP manually';
    const req = buildKvRequest([
        { key: KEY_STATE_HOST_IP, value: hostIpValue(host, HOST_PORT.push, LIDAR_PORT.push) },
        { key: KEY_POINT_HOST_IP, value: hostIpValue(host, HOST_PORT.point, LIDAR_PORT.point) },
        { key: KEY_IMU_HOST_IP,   value: hostIpValue(host, HOST_PORT.imu, LIDAR_PORT.imu) }
    ]);
    sockets.cmd.send(packCommand(CMD_WORK_MODE, req), LIDAR_PORT.cmd, config.lidarIp, noop);
    sockets.cmd.send(packCommand(CMD_WORK_MODE, buildKvRequest([
        { key: KEY_PCL_DATA_TYPE, value: Buffer.from([config.dataType === 2 ? 2 : 1]) }
    ])), LIDAR_PORT.cmd, config.lidarIp, noop);
    sockets.cmd.send(packCommand(CMD_WORK_MODE, buildKvRequest([
        { key: KEY_WORK_MODE, value: Buffer.from([WORK_MODE_NORMAL]) }
    ])), LIDAR_PORT.cmd, config.lidarIp, noop);
}

function onSearchReply(msg, rinfo) {
    const pkt = parseCommand(msg);
    if (!pkt || pkt.cmdId !== CMD_SEARCH || pkt.cmdType !== 1 || pkt.data.length < 24) return;
    // DetectionData: ret_code u8, dev_type u8, sn[16], lidar_ip[4], cmd_port u16
    if (pkt.data[0] !== 0) return;
    lidar.devType = pkt.data[1];
    lidar.sn = pkt.data.subarray(2, 18).toString('ascii').replace(/\0.*$/, '');
    lidar.ip = `${pkt.data[18]}.${pkt.data[19]}.${pkt.data[20]}.${pkt.data[21]}`;
    if (lidar.ip !== config.lidarIp && rinfo.address !== config.lidarIp) {
        // Another unit answered the broadcast — only the configured one is used.
        return;
    }
}

function onCmdReply(msg) {
    const pkt = parseCommand(msg);
    if (!pkt || pkt.cmdType !== 1) return;
    if (pkt.cmdId === CMD_WORK_MODE && pkt.data.length >= 3) {
        const ret = pkt.data[0];
        const errKey = pkt.data.readUInt16LE(1);
        lidar.lastAck = Date.now();
        if (ret === 0) lidar.configured = true;
        else lastError = `config rejected (ret ${ret}, key 0x${errKey.toString(16)})`;
    }
}

// ============== STATUS ==============
function rollStats() {
    const now = Date.now();
    const dt = (now - statsT0) / 1000;
    if (dt < 0.4) return;
    statsShown = {
        pps: Math.round(stats.packets / dt),
        pointsPs: Math.round(stats.points / dt),
        acceptedPs: Math.round(stats.accepted / dt),
        gatedPs: Math.round(stats.gated / dt),
        filteredPs: Math.round(stats.filtered / dt),
        livePs: Math.round(stats.live / dt),
        kbps: Math.round(stats.bytes * 8 / dt / 1000)
    };
    stats = { packets: 0, points: 0, accepted: 0, gated: 0, filtered: 0, live: 0, bytes: 0 };
    statsT0 = now;
}

function getStatus() {
    const now = Date.now();
    const streaming = !!(lidar.lastPacket && now - lidar.lastPacket < 2000);
    let link = 'OFF';
    if (connected) link = streaming ? 'STREAMING' : lidar.configured ? 'CONFIGURED' : lidar.sn ? 'FOUND' : 'SEARCHING';
    return {
        connected, link, streaming,
        lidarIp: config.lidarIp, hostIp: connected ? pickHostIp(config.lidarIp) : null,
        sn: lidar.sn, model: DEV_TYPE_NAMES[lidar.devType] || (lidar.devType ? `type ${lidar.devType}` : null),
        gateOk: gate.ok, gateReason: gate.reason,
        nav: { fix: nav.fix, sats: nav.sats, hdop: nav.hdop, ekfFlags: nav.ekfFlags, ekfPosVar: nav.ekfPosVar },
        mapPoints: mapCount, maxPoints: config.maxPoints,
        queue: packetQueue.length,
        ...statsShown,
        recording: !!rec, recordPath: rec ? rec.path : null, recordedPoints: rec ? rec.count : 0,
        origin,
        error: lastError
    };
}

function sendStatus() { emit('lidar-status', getStatus()); }
function sendOrigin() { emit('lidar-origin', origin); }

// ============== COMMANDS ==============
function applyConfig(patch) {
    const prevVoxel = config.voxel, prevMax = config.maxPoints;
    for (const [k, v] of Object.entries(patch || {})) {
        if (!(k in DEFAULT_CONFIG)) continue;
        if (typeof DEFAULT_CONFIG[k] === 'number') { const n = Number(v); if (Number.isFinite(n)) config[k] = n; }
        else if (typeof DEFAULT_CONFIG[k] === 'boolean') config[k] = !!v;
        else config[k] = String(v);
    }
    mountDirty = true;
    if (config.voxel !== prevVoxel || config.maxPoints !== prevMax) {
        // The occupancy table is keyed on the voxel size: rebuild it from the map.
        voxelKeys = null; voxelCount = 0;
        if (mapCount > 0) {
            ensureVoxelTable();
            for (const c of mapChunks) for (let k = 0; k < c.n; k++) claimVoxel(c.xyz[k * 3], c.xyz[k * 3 + 1], c.xyz[k * 3 + 2]);
        }
    }
    if (connected) {
        if (config.recordRaw && !rec) startRecording();
        if (!config.recordRaw && rec) stopRecording();
    }
}

// Renderer (re)started while the link was up: replay the whole map so the
// scene matches what this thread holds.
function resync() {
    if (!origin) return { success: true, points: 0 };
    sendOrigin();
    for (const c of mapChunks) {
        emit('lidar-points', { epoch, enu: c.xyz.slice(0, c.n * 3), intensity: c.i.slice(0, c.n) });
    }
    return { success: true, points: mapCount };
}

// Request/response commands, by name. The worker and the tests call these
// directly; lidar-manager.js maps each ipcMain channel onto one of them.
const commands = {
    connect: async (cfg) => { applyConfig(cfg); return connect(); },
    disconnect: async () => disconnect(),
    setConfig: async (cfg) => { applyConfig(cfg); return { success: true }; },
    getStatus: async () => getStatus(),
    listInterfaces: async () => listInterfaces(),
    clear: async () => { resetMap(); sendStatus(); return { success: true }; },
    saveMap: async () => { try { return saveMap(); } catch (e) { return { success: false, error: e.message }; } },
    getDir: async () => lidarDir(),
    resync: async () => resync()
};

// ============== HOST BINDING ==============
// emit(channel, data, transferList?) delivers an event towards the renderer.
let emit = () => {};

function bind(host) {
    if (host && typeof host.emit === 'function') emit = host.emit;
    if (host && host.dataRoot) dataRoot = host.dataRoot;
}

module.exports = {
    bind, commands, onMavlinkMessage, cleanup: () => disconnect(),
    // Offline tests (scripts/test-lidar-math.js) drive the pipeline without sockets.
    _test: {
        packCommand, parseCommand, buildKvRequest, crc16, crc32,
        applyConfig, onPointPacket, processQueue, evaluateGate, resetMap,
        setConnected: (v) => { connected = !!v; if (v) { resetPose(); mountDirty = true; } },
        getMap: () => ({ chunks: mapChunks, count: mapCount, origin })
    }
};
