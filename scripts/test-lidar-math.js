#!/usr/bin/env node
/**
 * test-lidar-math.js - offline checks for lidar-core.js
 *
 * Runs the georeferencing pipeline without sockets: fakes the MAVLink pose
 * tap and the point-packet input, then verifies that known LiDAR-frame
 * points land where the frame chain says they should. Also round-trips the
 * SDK2 command framing (CRC-16/CCITT-FALSE header, CRC-32 payload).
 *
 *   node scripts/test-lidar-math.js
 */

const path = require('path');

const lm = require(path.join(__dirname, '..', 'lidar-core.js'));
const T = lm._test;

let failures = 0;
function check(name, cond, detail = '') {
    console.log(`${cond ? 'PASS' : 'FAIL'}  ${name}${detail ? '  ' + detail : ''}`);
    if (!cond) failures++;
}
function near(a, b, tol) { return Math.abs(a - b) <= tol; }

// ---------- 1. Protocol framing ----------
{
    const kv = T.buildKvRequest([{ key: 0x001A, value: Buffer.from([1]) }]);
    check('kv request layout', kv.equals(Buffer.from([1, 0, 0, 0, 0x1A, 0x00, 1, 0, 1])), kv.toString('hex'));
    const frame = T.packCommand(0x0100, kv);
    check('frame length field', frame.readUInt16LE(2) === frame.length && frame.length === 24 + kv.length);
    const parsed = T.parseCommand(frame);
    check('frame round-trip', parsed && parsed.cmdId === 0x0100 && parsed.data.equals(kv));
    const bad = Buffer.from(frame); bad[26] ^= 0xFF;
    check('payload CRC rejects corruption', T.parseCommand(bad) === null);
    check('crc16 check value', T.crc16(Buffer.from('123456789'), 9) === 0x29B1, T.crc16(Buffer.from('123456789'), 9).toString(16));
    check('crc32 check value', T.crc32(Buffer.from('123456789')) === 0xCBF43926, T.crc32(Buffer.from('123456789')).toString(16));
}

// ---------- 2. Georeferencing ----------
// Catch the events the core would send towards the renderer.
const sent = [];
lm.bind({ emit: (ch, d) => sent.push({ ch, d }) });

function makePacket(pointsMm, dataType = 1) {
    const stride = 14;
    const buf = Buffer.alloc(36 + pointsMm.length * stride);
    buf.writeUInt16LE(buf.length, 1);
    buf.writeUInt16LE(pointsMm.length, 5);
    buf[10] = dataType;
    pointsMm.forEach(([x, y, z, r], k) => {
        const o = 36 + k * stride;
        buf.writeInt32LE(x, o); buf.writeInt32LE(y, o + 4); buf.writeInt32LE(z, o + 8);
        buf[o + 12] = r === undefined ? 100 : r; buf[o + 13] = 0;
    });
    return buf;
}

// Freeze time so the hold/lag logic is deterministic.
let fakeNow = 1_000_000;
const realNow = Date.now;
Date.now = () => fakeNow;

function feedPose({ lat, lon, alt, roll = 0, pitch = 0, yaw = 0 }) {
    lm.onMavlinkMessage(30, { roll, pitch, yaw });
    lm.onMavlinkMessage(33, { lat: Math.round(lat * 1e7), lon: Math.round(lon * 1e7), alt: Math.round(alt * 1000), vx: 0, vy: 0, vz: 0 });
    lm.onMavlinkMessage(24, { fixType: 3, satellitesVisible: 12, eph: 90 });
    lm.onMavlinkMessage(193, { flags: 0x1FF & ~0x80, velocityVariance: 0.1, posHorizVariance: 0.1, posVertVariance: 0.1 });
}

function run(cfg, pose, pointsMm) {
    T.setConnected(true);
    T.resetMap();
    sent.length = 0;
    T.applyConfig({ ...cfg });
    // Two pose samples around the packet time so interpolation is exercised.
    feedPose(pose);
    fakeNow += 100;
    feedPose(pose);
    fakeNow += 50;
    T.onPointPacket(makePacket(pointsMm), fakeNow);
    fakeNow += 100;
    feedPose(pose);
    fakeNow += 200;
    T.processQueue();
    const map = T.getMap();
    const pts = [];
    for (const c of map.chunks) for (let k = 0; k < c.n; k++) pts.push([c.xyz[k * 3], c.xyz[k * 3 + 1], c.xyz[k * 3 + 2], c.i[k]]);
    return { map, pts };
}

const base = { voxel: 0.1, minRange: 0.5, maxRange: 100, dropNoise: true, requireEkf: true, minFix: 3, minSats: 8, maxHdop: 2, lagMs: 0,
               mountRoll: 0, mountPitch: 0, mountYaw: 0, leverX: 0, leverY: 0, leverZ: 0 };
const P0 = { lat: 45.0, lon: 9.0, alt: 200 };

// 2a. Level, heading north, upright mount: lidar +X → north, +Y → west, +Z → up
{
    const { pts } = run(base, P0, [[10000, 0, 0], [0, 10000, 0], [0, 0, 10000]]);
    check('north/west/up (identity)', pts.length === 3
        && near(pts[0][1], 10, 0.01) && near(pts[0][0], 0, 0.01) && near(pts[0][2], 0, 0.01)
        && near(pts[1][0], -10, 0.01) && near(pts[1][1], 0, 0.01)
        && near(pts[2][2], 10, 0.01), JSON.stringify(pts.map(p => p.slice(0, 3).map(v => +v.toFixed(2)))));
}

// 2b. Heading east (yaw 90°): lidar +X → east
{
    const { pts } = run(base, { ...P0, yaw: Math.PI / 2 }, [[10000, 0, 0]]);
    check('yaw 90 → +X east', pts.length === 1 && near(pts[0][0], 10, 0.01) && near(pts[0][1], 0, 0.01), JSON.stringify(pts));
}

// 2c. Inverted mount (roll 180): lidar +Z → down, +Y → east (right)
{
    const { pts } = run({ ...base, mountRoll: 180 }, P0, [[0, 0, 10000], [0, 10000, 0]]);
    check('inverted mount → +Z down, +Y right', pts.length === 2 && near(pts[0][2], -10, 0.01) && near(pts[1][0], 10, 0.01), JSON.stringify(pts));
}

// 2d. Vehicle rolled 30° right (right wing down), inverted mount, point straight
//     "up" the lidar (= body down). Body down in NED = (0, -sin30, cos30): the belly
//     faces left, so the point is 5 m WEST and 8.66 m below.
{
    const { pts } = run({ ...base, mountRoll: 180 }, { ...P0, roll: 30 * Math.PI / 180 }, [[0, 0, 10000]]);
    check('roll 30 + inverted', pts.length === 1 && near(pts[0][0], -5, 0.02) && near(pts[0][2], -8.66, 0.02), JSON.stringify(pts));
}

// 2e. Lever arm 1 m forward, heading north → shifts the point 1 m north
{
    const { pts } = run({ ...base, leverX: 1 }, P0, [[10000, 0, 0]]);
    check('lever arm forward', pts.length === 1 && near(pts[0][1], 11, 0.01), JSON.stringify(pts));
}

// 2f. Filters: range gate, noise tag, voxel dedup
{
    T.setConnected(true); T.resetMap(); sent.length = 0;
    T.applyConfig({ ...base, minRange: 2, maxRange: 20 });
    feedPose(P0); fakeNow += 100; feedPose(P0); fakeNow += 50;
    const buf = makePacket([[1000, 0, 0], [30000, 0, 0], [5000, 0, 0], [5010, 0, 0], [0, 0, 0]]);
    buf[36 + 2 * 14 + 13] = 0x00;          // normal
    buf[36 + 3 * 14 + 13] = 0x00;          // same voxel as previous → dropped by dedup
    T.onPointPacket(buf, fakeNow);
    fakeNow += 300; feedPose(P0); T.processQueue();
    const m = T.getMap();
    check('range + zero + voxel filtering', m.count === 1, `count=${m.count}`);
    const buf2 = makePacket([[8000, 0, 0]]);
    buf2[36 + 13] = 0x01;                  // spatial noise tag
    T.onPointPacket(buf2, fakeNow); fakeNow += 300; feedPose(P0); T.processQueue();
    check('noise tag dropped', T.getMap().count === 1);
}

// 2g. Gate: RTK-fixed required but only 3D fix → nothing accumulates, reason reported
{
    T.setConnected(true); T.resetMap();
    T.applyConfig({ ...base, minFix: 6 });
    feedPose(P0); fakeNow += 100; feedPose(P0); fakeNow += 50;
    T.onPointPacket(makePacket([[10000, 0, 0]]), fakeNow);
    fakeNow += 300; feedPose(P0); T.processQueue();
    const g = T.evaluateGate(fakeNow);
    check('gate blocks on fix type', T.getMap().count === 0 && !g.ok && /GPS FIX/.test(g.reason), g.reason);
}

// 2g-bis. Gate closed → the packet is still shown: live batch, vehicle-relative,
//         levelled (frame 'ned') because attitude is fresh. Heading east, +X → east.
{
    T.setConnected(true); T.resetMap(); sent.length = 0;
    T.applyConfig({ ...base, minFix: 6 });
    feedPose({ ...P0, yaw: Math.PI / 2 }); fakeNow += 100; feedPose({ ...P0, yaw: Math.PI / 2 }); fakeNow += 50;
    T.onPointPacket(makePacket([[10000, 0, 0]]), fakeNow);
    fakeNow += 300; feedPose({ ...P0, yaw: Math.PI / 2 }); T.processQueue();
    const live = sent.find(s => s.ch === 'lidar-live');
    check('live batch when not georeferenceable', !!live && live.d.frame === 'ned' && live.d.xyz.length === 3
        && near(live.d.xyz[1], 10, 0.01) && near(live.d.xyz[0], 0, 0.01), live ? JSON.stringify([live.d.frame, Array.from(live.d.xyz)]) : 'no live batch');
    check('map untouched by live points', T.getMap().count === 0);
}

// 2h. Renderer batch carries the same points as the map
{
    const { pts } = run(base, P0, [[10000, 0, 0], [0, 10000, 0]]);
    const batch = sent.find(s => s.ch === 'lidar-points');
    const originEv = sent.find(s => s.ch === 'lidar-origin');
    check('origin event + batch emitted', !!batch && !!originEv && batch.d.enu.length === pts.length * 3 && batch.d.epoch === originEv.d.epoch);
}

// 2i. Lag: pose changes between samples; with lagMs the packet is matched to the later pose
{
    T.setConnected(true); T.resetMap();
    T.applyConfig({ ...base, lagMs: 200, voxel: 0.05 });
    feedPose({ ...P0, yaw: 0 });
    fakeNow += 50;
    const tPkt = fakeNow;
    T.onPointPacket(makePacket([[10000, 0, 0]]), tPkt);
    fakeNow += 200;
    feedPose({ ...P0, yaw: Math.PI / 2 });     // this is the pose that "belongs" to the packet
    fakeNow += 100;
    feedPose({ ...P0, yaw: Math.PI / 2 });
    fakeNow += 100;
    T.processQueue();
    const m = T.getMap();
    const p = m.chunks[0] && [m.chunks[0].xyz[0], m.chunks[0].xyz[1]];
    check('telemetry lag matches later pose', m.count === 1 && p && near(p[0], 10, 0.05), JSON.stringify(p));
}

Date.now = realNow;
console.log(failures ? `\n${failures} check(s) FAILED` : '\nall checks passed');
process.exit(failures ? 1 : 0);
