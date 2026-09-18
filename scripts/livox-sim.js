#!/usr/bin/env node
/**
 * livox-sim.js - Livox Mid-360 emulator for end-to-end tests without hardware
 *
 * Speaks the SDK2 UDP protocol the GCS implements in lidar-manager.js
 * (search / config / point packets) and synthesises returns from a known
 * scene — a ground plane plus a few boxes — using the TRUE vehicle pose read
 * from a second SITL MAVLink port. If the GCS reconstructs the boxes sharp
 * and the ground flat, the whole frame chain (Livox FLU → mount → body →
 * NED → ENU, pose interpolation, lag) is right; any mistake smears them.
 *
 *   node scripts/livox-sim.js [--bind 127.0.0.2] [--mav tcp:127.0.0.1:5762]
 *                             [--pps 1000] [--mount 180,0,0] [--lever 0,0,0.1]
 *                             [--type 1|2]
 *
 * In the GCS set LIDAR IP = 127.0.0.2, HOST IP = 127.0.0.1, and the same
 * mount / lever arm you pass here (the emulator "installs" the sensor that
 * way, the GCS has to undo it).
 *
 * SITL exposes serial1 on TCP 5762 next to the GCS link on 5760, so both
 * see the same vehicle. Points stream at --pps packets/s × 96 points; the
 * real sensor does ~2083 pps (200 kpts/s), 1000 is plenty for a test.
 */

const dgram = require('dgram');
const net = require('net');
const path = require('path');

// node-mavlink lives in the project's node_modules
const mavlink = require(path.join(__dirname, '..', 'node_modules', 'node-mavlink'));
const { MavLinkPacketSplitter, MavLinkPacketParser, MavLinkProtocolV2, minimal, common, ardupilotmega } = mavlink;

// ============== ARGS ==============
const argv = process.argv.slice(2);
function arg(name, def) {
    const i = argv.indexOf(`--${name}`);
    return i >= 0 && argv[i + 1] !== undefined ? argv[i + 1] : def;
}
const BIND = arg('bind', '127.0.0.2');
const MAV = arg('mav', 'tcp:127.0.0.1:5762');
const PPS = parseInt(arg('pps', '1000'), 10);
const DATA_TYPE_DEFAULT = parseInt(arg('type', '1'), 10);
const [MR, MP, MY] = arg('mount', '180,0,0').split(',').map(Number);
const [LX, LY, LZ] = arg('lever', '0,0,0').split(',').map(Number);
const SN = 'SIM360S000000001';
const DEV_TYPE = 35;             // Mid-360S

const DEG = Math.PI / 180;
const POINTS_PER_PKT = 96;
const FOV_EL_MIN = -7 * DEG, FOV_EL_MAX = 52 * DEG;
const MAX_RANGE = 70;

// ============== CRC (same as the GCS side) ==============
const CRC16_TABLE = new Uint16Array(256);
for (let i = 0; i < 256; i++) { let c = i << 8; for (let k = 0; k < 8; k++) c = (c & 0x8000) ? ((c << 1) ^ 0x1021) & 0xFFFF : (c << 1) & 0xFFFF; CRC16_TABLE[i] = c; }
function crc16(buf, len) { let crc = 0xFFFF; for (let i = 0; i < len; i++) crc = ((crc << 8) & 0xFFFF) ^ CRC16_TABLE[((crc >> 8) ^ buf[i]) & 0xFF]; return crc; }
const CRC32_TABLE = new Uint32Array(256);
for (let i = 0; i < 256; i++) { let c = i; for (let k = 0; k < 8; k++) c = (c & 1) ? (0xEDB88320 ^ (c >>> 1)) : (c >>> 1); CRC32_TABLE[i] = c >>> 0; }
function crc32(buf, off = 0, len = buf.length - off) { let crc = 0xFFFFFFFF; for (let i = off; i < off + len; i++) crc = CRC32_TABLE[(crc ^ buf[i]) & 0xFF] ^ (crc >>> 8); return (crc ^ 0xFFFFFFFF) >>> 0; }

function parseCommand(buf) {
    if (buf.length < 24 || buf[0] !== 0xAA) return null;
    const length = buf.readUInt16LE(2);
    if (length > buf.length || crc16(buf, 18) !== buf.readUInt16LE(18)) return null;
    const data = buf.subarray(24, length);
    if (data.length && crc32(data) !== buf.readUInt32LE(20)) return null;
    return { seq: buf.readUInt32LE(4), cmdId: buf.readUInt16LE(8), cmdType: buf[10], data };
}

function packAck(seq, cmdId, data) {
    const buf = Buffer.alloc(24 + data.length);
    buf[0] = 0xAA; buf[1] = 0;
    buf.writeUInt16LE(24 + data.length, 2);
    buf.writeUInt32LE(seq, 4);
    buf.writeUInt16LE(cmdId, 8);
    buf[10] = 1;      // ack
    buf[11] = 1;      // lidar
    buf.writeUInt16LE(crc16(buf, 18), 18);
    buf.writeUInt32LE(data.length ? crc32(data) : 0, 20);
    data.copy(buf, 24);
    return buf;
}

// ============== POSE FROM SITL ==============
const pose = { t: 0, lat: 0, lon: 0, alt: 0, relAlt: 0, vn: 0, ve: 0, vd: 0, roll: 0, pitch: 0, yaw: 0, have: false };
let ground = null;      // { lat0, lon0, alt0, mPerLat, mPerLon }

const registry = new Map();
for (const dialect of [minimal, common, ardupilotmega]) {
    if (dialect && dialect.REGISTRY) for (const [id, clazz] of Object.entries(dialect.REGISTRY)) registry.set(Number(id), clazz);
}
const protocol = new MavLinkProtocolV2(254, 191);
let seq = 0;

function connectMavlink() {
    const m = MAV.match(/^tcp:([^:]+):(\d+)$/);
    if (!m) { console.error('only tcp:host:port is supported for --mav'); process.exit(1); }
    const sock = net.createConnection({ host: m[1], port: parseInt(m[2], 10) }, () => {
        console.log(`[sim] MAVLink connected ${MAV}`);
        // Ask SITL for position + attitude at 10 Hz on this port.
        for (const [id, rate] of [[6, 10], [10, 10], [2, 2]]) {
            const req = new common.RequestDataStream();
            req.targetSystem = 1; req.targetComponent = 1;
            req.reqStreamId = id; req.reqMessageRate = rate; req.startStop = 1;
            sock.write(protocol.serialize(req, seq++ & 0xFF));
        }
    });
    // 'error' is always followed by 'close': schedule the retry once, from 'close'.
    sock.on('error', (e) => { console.error('[sim] MAVLink error:', e.message); });
    sock.on('close', () => { pose.have = false; setTimeout(connectMavlink, 2000); });
    const parser = sock.pipe(new MavLinkPacketSplitter()).pipe(new MavLinkPacketParser());
    parser.on('data', (packet) => {
        const clazz = registry.get(packet.header.msgid);
        if (!clazz) return;
        let msg;
        try { msg = packet.protocol.data(packet.payload, clazz); } catch (_) { return; }
        const now = Date.now();
        if (packet.header.msgid === 33) {
            pose.lat = msg.lat / 1e7; pose.lon = msg.lon / 1e7; pose.alt = msg.alt / 1000; pose.relAlt = msg.relativeAlt / 1000;
            pose.vn = msg.vx / 100; pose.ve = msg.vy / 100; pose.vd = msg.vz / 100;
            pose.t = now; pose.have = true;
            if (!ground && pose.lat !== 0) {
                const f = pose.lat * DEG;
                ground = {
                    lat0: pose.lat, lon0: pose.lon, alt0: pose.alt - pose.relAlt,
                    mPerLat: 111132.954 - 559.822 * Math.cos(2 * f) + 1.175 * Math.cos(4 * f),
                    mPerLon: 111412.84 * Math.cos(f) - 93.5 * Math.cos(3 * f)
                };
                console.log(`[sim] scene anchored at ${ground.lat0.toFixed(6)}, ${ground.lon0.toFixed(6)}, ground ${ground.alt0.toFixed(1)} m MSL`);
            }
        } else if (packet.header.msgid === 30) {
            pose.roll = msg.roll; pose.pitch = msg.pitch; pose.yaw = msg.yaw;
        }
    });
}

// ============== SCENE ==============
// Boxes in ENU metres from the anchor: [e0, n0, e1, n1, height]
const BOXES = [
    [40, 30, 60, 50, 15],
    [-80, 20, -50, 35, 8],
    [10, -70, 70, -60, 25],
    [-30, -30, -20, -20, 40]
];

function eulerToMatrix(roll, pitch, yaw, out) {
    const cr = Math.cos(roll), sr = Math.sin(roll), cp = Math.cos(pitch), sp = Math.sin(pitch), cy = Math.cos(yaw), sy = Math.sin(yaw);
    out[0] = cy * cp; out[1] = cy * sp * sr - sy * cr; out[2] = cy * sp * cr + sy * sr;
    out[3] = sy * cp; out[4] = sy * sp * sr + cy * cr; out[5] = sy * sp * cr - cy * sr;
    out[6] = -sp;     out[7] = cp * sr;                out[8] = cp * cr;
    return out;
}
const Rm = eulerToMatrix(MR * DEG, MP * DEG, MY * DEG, new Float64Array(9));
const Ra = new Float64Array(9);

// Ray from O along D (ENU); returns the nearest hit distance below MAX_RANGE or 0.
function castRay(oe, on, ou, de, dn, du) {
    let best = MAX_RANGE;
    // Ground plane u = 0
    if (du < -1e-6) { const t = -ou / du; if (t > 0 && t < best) best = t; }
    // Boxes (slab test)
    for (const b of BOXES) {
        let tmin = 0, tmax = best;
        const lo = [b[0], b[1], 0], hi = [b[2], b[3], b[4]];
        const o = [oe, on, ou], d = [de, dn, du];
        let ok = true;
        for (let k = 0; k < 3; k++) {
            if (Math.abs(d[k]) < 1e-9) { if (o[k] < lo[k] || o[k] > hi[k]) { ok = false; break; } continue; }
            let t1 = (lo[k] - o[k]) / d[k], t2 = (hi[k] - o[k]) / d[k];
            if (t1 > t2) { const s = t1; t1 = t2; t2 = s; }
            if (t1 > tmin) tmin = t1;
            if (t2 < tmax) tmax = t2;
            if (tmin > tmax) { ok = false; break; }
        }
        if (ok && tmin > 0.05 && tmin < best) best = tmin;
    }
    return best < MAX_RANGE ? best : 0;
}

// ============== LIVOX SERVER ==============
let host = null;           // { ip, port } for point packets
let dataType = DATA_TYPE_DEFAULT;
let udpCnt = 0, frameCnt = 0;
const t0 = process.hrtime.bigint();

const search = dgram.createSocket({ type: 'udp4', reuseAddr: true });
const cmd = dgram.createSocket({ type: 'udp4', reuseAddr: true });
const point = dgram.createSocket({ type: 'udp4', reuseAddr: true });

search.on('message', (msg, rinfo) => {
    const pkt = parseCommand(msg);
    if (!pkt || pkt.cmdId !== 0x0000 || pkt.cmdType !== 0) return;
    const data = Buffer.alloc(24);
    data[0] = 0; data[1] = DEV_TYPE;
    data.write(SN, 2, 'ascii');
    BIND.split('.').forEach((n, i) => { data[18 + i] = parseInt(n, 10); });
    data.writeUInt16LE(56100, 22);
    search.send(packAck(pkt.seq, 0x0000, data), rinfo.port, rinfo.address);
});

cmd.on('message', (msg, rinfo) => {
    const pkt = parseCommand(msg);
    if (!pkt || pkt.cmdType !== 0) return;
    if (pkt.cmdId === 0x0100) {
        const n = pkt.data.readUInt16LE(0);
        let o = 4;
        for (let i = 0; i < n && o + 4 <= pkt.data.length; i++) {
            const key = pkt.data.readUInt16LE(o), len = pkt.data.readUInt16LE(o + 2);
            const v = pkt.data.subarray(o + 4, o + 4 + len);
            if (key === 0x0006 && len >= 8) {
                host = { ip: `${v[0]}.${v[1]}.${v[2]}.${v[3]}`, port: v.readUInt16LE(4) };
                console.log(`[sim] point destination ${host.ip}:${host.port}`);
            } else if (key === 0x0000 && len >= 1) {
                dataType = v[0] === 2 ? 2 : 1;
                console.log(`[sim] data type ${dataType}`);
            } else if (key === 0x001A && len >= 1) {
                console.log(`[sim] work mode ${v[0]}`);
            }
            o += 4 + len;
        }
        const ack = Buffer.alloc(3); ack[0] = 0; ack.writeUInt16LE(0, 1);
        cmd.send(packAck(pkt.seq, 0x0100, ack), rinfo.port, rinfo.address);
    }
});

search.bind(56000, BIND);
cmd.bind(56100, BIND);
point.bind(56300, BIND, () => console.log(`[sim] Livox emulator on ${BIND} (search 56000, cmd 56100, points from 56300)`));

// ============== POINT STREAM ==============
let sentPkts = 0, sentPts = 0, lastLog = Date.now();
let carry = 0;

function buildPacket() {
    const stride = dataType === 2 ? 8 : 14;
    const buf = Buffer.alloc(36 + POINTS_PER_PKT * stride);
    buf[0] = 0;
    buf.writeUInt16LE(buf.length, 1);
    buf.writeUInt16LE(Math.round(POINTS_PER_PKT / 200000 * 1e7), 3);    // 0.1 µs units
    buf.writeUInt16LE(POINTS_PER_PKT, 5);
    buf.writeUInt16LE(udpCnt++ & 0xFFFF, 7);
    buf[9] = frameCnt & 0xFF;
    buf[10] = dataType;
    buf[11] = 0;                                                         // no time sync
    const ns = process.hrtime.bigint() - t0;
    buf.writeBigUInt64LE(ns, 28);

    // Pose now (dead-reckoned from the last sample on velocity)
    const dt = (Date.now() - pose.t) / 1000;
    const lat = pose.lat + (pose.vn * dt) / ground.mPerLat;
    const lon = pose.lon + (pose.ve * dt) / ground.mPerLon;
    const alt = pose.alt - pose.vd * dt;
    const vehE = (lon - ground.lon0) * ground.mPerLon, vehN = (lat - ground.lat0) * ground.mPerLat, vehU = alt - ground.alt0;
    eulerToMatrix(pose.roll, pose.pitch, pose.yaw, Ra);
    // Lever arm (body FRD) → NED → ENU
    const ln = Ra[0] * LX + Ra[1] * LY + Ra[2] * LZ, le = Ra[3] * LX + Ra[4] * LY + Ra[5] * LZ, ld = Ra[6] * LX + Ra[7] * LY + Ra[8] * LZ;
    const oe = vehE + le, on = vehN + ln, ou = vehU - ld;

    for (let k = 0; k < POINTS_PER_PKT; k++) {
        const az = Math.random() * 2 * Math.PI;
        const el = FOV_EL_MIN + Math.random() * (FOV_EL_MAX - FOV_EL_MIN);
        const ce = Math.cos(el);
        const lx = ce * Math.cos(az), ly = ce * Math.sin(az), lz = Math.sin(el);   // lidar FLU
        const fx = lx, fy = -ly, fz = -lz;                                          // FRD
        const bx = Rm[0] * fx + Rm[1] * fy + Rm[2] * fz, by = Rm[3] * fx + Rm[4] * fy + Rm[5] * fz, bz = Rm[6] * fx + Rm[7] * fy + Rm[8] * fz;
        const dn = Ra[0] * bx + Ra[1] * by + Ra[2] * bz, de = Ra[3] * bx + Ra[4] * by + Ra[5] * bz, dd = Ra[6] * bx + Ra[7] * by + Ra[8] * bz;
        const t = castRay(oe, on, ou, de, dn, -dd);
        const o = 36 + k * stride;
        if (t === 0) continue;                                    // no return → zeros
        const r = t + (Math.random() - 0.5) * 0.04;
        const hitU = ou - dd * t;
        const refl = hitU > 0.5 ? 110 + Math.random() * 40 : 25 + Math.random() * 20;
        if (dataType === 2) {
            buf.writeInt16LE(Math.round(r * lx * 100), o); buf.writeInt16LE(Math.round(r * ly * 100), o + 2); buf.writeInt16LE(Math.round(r * lz * 100), o + 4);
            buf[o + 6] = refl; buf[o + 7] = 0;
        } else {
            buf.writeInt32LE(Math.round(r * lx * 1000), o); buf.writeInt32LE(Math.round(r * ly * 1000), o + 4); buf.writeInt32LE(Math.round(r * lz * 1000), o + 8);
            buf[o + 12] = refl; buf[o + 13] = 0;
        }
        sentPts++;
    }
    buf.writeUInt32LE(crc32(buf, 36), 24);
    return buf;
}

setInterval(() => {
    if (!host || !pose.have || !ground) return;
    carry += PPS * 0.005;
    let n = Math.floor(carry);
    carry -= n;
    while (n-- > 0) {
        point.send(buildPacket(), host.port, host.ip);
        sentPkts++;
    }
    if (++frameCnt % 20 === 0) frameCnt++;
    const now = Date.now();
    if (now - lastLog > 5000) {
        console.log(`[sim] ${(sentPkts / ((now - lastLog) / 1000)).toFixed(0)} pkt/s, ${(sentPts / ((now - lastLog) / 1000)).toFixed(0)} returns/s, alt ${pose.relAlt.toFixed(0)} m AGL`);
        sentPkts = 0; sentPts = 0; lastLog = now;
    }
}, 5);

connectMavlink();
