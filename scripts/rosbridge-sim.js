#!/usr/bin/env node
/**
 * rosbridge-sim.js - rosbridge_server emulator with synthetic sensors, for SITL tests
 *
 * Speaks the rosbridge v2 protocol over WebSocket the way rosbridge_server
 * does (subscribe with throttle_rate / queue_length / compression, JSON or
 * CBOR, the rosapi services roslib uses to list topics) and publishes what
 * the GCS's ROS surface reads, synthesised from a known scene using the TRUE
 * vehicle pose read from a second SITL MAVLink port:
 *
 *   /livox/lidar        sensor_msgs/PointCloud2  livox_frame  Mid-360 pattern, livox_ros_driver2 layout
 *   /cloud_registered   sensor_msgs/PointCloud2  map          the same scan in the world frame (FAST-LIO style)
 *   /scan               sensor_msgs/LaserScan    laser        270° push-broom, 0.5°
 *   /sonar/multibeam    sensor_msgs/PointCloud2  sonar        256-beam swath, ±65°
 *   /ping1d/range       sensor_msgs/Range        ping1d       single-beam echo sounder (0.5–100 m, Ping2)
 *   /sonar/imaging      sensor_msgs/PointCloud2  imaging_sonar  imaging sonar looking down: a fan of
 *                                                --sonar-aperture (90°) across the vehicle, --sonar-beams
 *                                                (256), --sonar-range (90 m), --sonar-rate (20 Hz), with
 *                                                the disturbances of a real one (below)
 *   /sonar/profiler     sensor_msgs/PointCloud2  sonar_profiler  dual 360° profiling sonar: one fan
 *                                                across the vehicle (y–z plane: a level passage's
 *                                                cross-section), one level (x–y: a shaft's), 2 × 400
 *                                                beams, 40 m
 *   /sonar/sector       sensor_msgs/LaserScan    sector_sonar  mechanical sector-scanning sonar (below)
 * plus a few topics of other types, listed by rosapi and never published.
 *
 * Sector-scanning sonar (a Ping360, Imagenex 881, Tritech Micron class head):
 * ONE beam per ping, the head stepping --sector-step (1.8°) between pings,
 * back and forth over --sector-aperture (120°) in the sensor's x–y plane —
 * mounted with --sector-mount (0,-90,0: x down, the sweep across the track).
 * A ping lasts the echo's travel to --sector-range and back (2 R / 1480 m/s)
 * plus 12 ms to step the head: 12–17 pings a second, a sweep every 4–6 s,
 * so each beam is measured from a different pose. The bottom is where the
 * echo crosses a threshold: rays across the beam — --sector-beam (2°) in the
 * sweep plane by --sector-fan (2°; 25° for a Ping360) across it — weighted by
 * its pattern, and the range by which a quarter of the echo's energy has come
 * back, reported on the beam's axis as a sector sonar cannot tell: objects
 * come out wider by part of the beam's footprint, and an oblique bed a little
 * shallow (5–20 cm at 45–60° off nadir in 15 m of water). The pings are
 * published as LaserScans of about --sector-batch (1 s) of the sweep — each
 * sweep cut in equal parts, never across a turn of the head — with
 * header.stamp the first ping and time_increment the ping period
 * (sensor_msgs/LaserScan): the GCS places every beam at its own time. The head steps on
 * the SITL's clock (its speed-up from ATTITUDE.time_boot_ms), so a survey run
 * faster than real time keeps the sonar's sampling along the track.
 *
 * Scenes (scripts/ros-sim-scenes.js): 'terrain' (hills under an aircraft),
 * 'seabed' (a lake bed 5–25 m under the surface the vehicle started on),
 * 'cave' (an underwater cave from a basin at the start, for a ROV) or
 * 'garda' (5 km² of Lake Garda's southern basin, fixed to the map: its origin
 * is the survey area's corner, not the vehicle's start; the water surface is
 * still the home altitude) or 'garda-complex' (the same, with a ridge, a
 * scarp, a pinnacle, boulders and sand waves around the objects).
 *
 * Imaging sonar disturbances (--sonar-clean turns them off): the return comes
 * from anywhere in the beam's vertical aperture (--sonar-elevation, 10°) but
 * is reported on its axis, as an imaging sonar cannot tell; range noise of
 * 3 cm + 0.2 % of the range; 3 % of beams with no return; targets in the water
 * column (fish, particles); and aeration — near-field echoes from bubbles,
 * more when the hull rolls and pitches in the waves and when it goes fast.
 * The waves themselves are SITL's (SIM_WAVE_*, see the boat's defaults): they
 * move the true pose this emulator ray-casts from, so the sonar and the
 * telemetry see the same sea.
 * Every sensor is "installed" with the mount and lever arm given here; the
 * GCS has to undo it. A mesh that matches the scene means the whole chain —
 * decoding, sampling, mount, attitude, position, frames, averaging — is right.
 *
 *   node scripts/rosbridge-sim.js [--port 9090] [--mav tcp:127.0.0.1:5762]
 *        [--scene terrain|seabed|cave|garda|garda-complex] [--ros1] [--lidar-mount 180,0,0]
 *        [--down-mount 0,-90,0] [--profiler-mount 0,0,0] [--lever 0,0,0] [--lidar-points 10000]
 *        [--sector-mount 0,-90,0] [--sector-aperture 120] [--sector-step 1.8] [--sector-range 40]
 *        [--sector-beam 2] [--sector-fan 2] [--sector-batch 1]
 *        [--anchor lat,lon,alt] [--pose e,n,up,yaw]
 *
 * --pose holds the vehicle still at east / north / up metres from the anchor
 * (default 0,0,0), level, heading yaw degrees, with no SITL at all: for
 * offline tests (scripts/test-ros-surface.js). With --speed v it moves along
 * that heading at v m/s, level, from --pose-t0 (epoch ms; default: start), so
 * a test can feed the GCS the same track (scripts/test-ros-sector.js).
 * --sector-untimed publishes the sector sonar with time_increment 0, as a
 * republisher that forgot it: for tests, to see what per-beam time is worth.
 *
 * Start it before the vehicle moves: the scene is anchored at the first
 * position (home), with the ground at the home altitude (MAVLink
 * GLOBAL_POSITION_INT alt − relative_alt, or the first true altitude without
 * GPS). The pose is SITL's truth: SIMSTATE (lat/lon, attitude) and SIM_STATE
 * (altitude, velocity) when SITL sends them — also without GPS — else
 * GLOBAL_POSITION_INT and ATTITUDE.
 */

const http = require('http');
const net = require('net');
const path = require('path');
const crypto = require('crypto');
const { SCENES, castRay } = require('./ros-sim-scenes');

const mavlink = require(path.join(__dirname, '..', 'node_modules', 'node-mavlink'));
const { MavLinkPacketSplitter, MavLinkPacketParser, MavLinkProtocolV2, minimal, common, ardupilotmega } = mavlink;

// ============== ARGS ==============
const argv = process.argv.slice(2);
function arg(name, def) {
    const i = argv.indexOf(`--${name}`);
    return i >= 0 && argv[i + 1] !== undefined && !argv[i + 1].startsWith('--') ? argv[i + 1] : def;
}
const PORT = parseInt(arg('port', '9090'), 10);
const MAV = arg('mav', 'tcp:127.0.0.1:5762');
const SCENE_NAME = arg('scene', 'terrain');
const ROS1 = argv.includes('--ros1');
const LIDAR_MOUNT = arg('lidar-mount', '180,0,0').split(',').map(Number);
const DOWN_MOUNT = arg('down-mount', '0,-90,0').split(',').map(Number);
const PROFILER_MOUNT = arg('profiler-mount', '0,0,0').split(',').map(Number);
const LEVER = arg('lever', '0,0,0').split(',').map(Number);
const LIDAR_POINTS = parseInt(arg('lidar-points', '10000'), 10);
const ANCHOR_ARG = arg('anchor', null);
const SONAR = {
    aperture: parseFloat(arg('sonar-aperture', '90')),
    beams: parseInt(arg('sonar-beams', '256'), 10),
    range: parseFloat(arg('sonar-range', '90')),
    rate: parseFloat(arg('sonar-rate', '20')),
    elevation: parseFloat(arg('sonar-elevation', '10')),
    clean: argv.includes('--sonar-clean')
};
const SOUND_SPEED = 1480;          // m/s in fresh water
const SECTOR = {
    mount: arg('sector-mount', '0,-90,0').split(',').map(Number),
    aperture: parseFloat(arg('sector-aperture', '120')),
    step: parseFloat(arg('sector-step', '1.8')),
    range: parseFloat(arg('sector-range', '40')),
    beam: parseFloat(arg('sector-beam', '2')),
    fan: parseFloat(arg('sector-fan', '2')),
    batch: parseFloat(arg('sector-batch', '1')),
    untimed: argv.includes('--sector-untimed'),
    minRange: 0.75
};
// The echo's travel to the range and back, plus stepping the head
SECTOR.period = 2 * SECTOR.range / SOUND_SPEED + 0.012;
SECTOR.sweepPings = Math.floor(SECTOR.aperture / SECTOR.step + 1e-9) + 1;
SECTOR.half = (SECTOR.sweepPings - 1) * SECTOR.step / 2;       // the steps centred on the axis
// Pings per message: the sweep in equal parts of about --sector-batch
SECTOR.perBatch = Math.ceil(SECTOR.sweepPings / Math.max(1, Math.round(SECTOR.sweepPings * SECTOR.period / SECTOR.batch)));
const POSE_ARG = arg('pose', null);
const SCENE = SCENES[SCENE_NAME];
if (!SCENE) { console.error(`unknown scene ${SCENE_NAME} (${Object.keys(SCENES).join(' | ')})`); process.exit(1); }

const DEG = Math.PI / 180;
const T = (pkg, name) => ROS1 ? `${pkg}/${name}` : `${pkg}/msg/${name}`;

// ============== CBOR (RFC 8949) ==============
// What rosbridge's cbor_conversion produces: uint8[] as a byte string,
// float32[] as an RFC 8746 typed array (tag 85, little endian), floats as
// float64, maps with text keys.
function cborEncode(value) {
    const parts = [];
    let size = 0;
    const push = (b) => { parts.push(b); size += b.length; };
    const head = (major, n) => {
        if (n < 24) push(Buffer.from([(major << 5) | n]));
        else if (n < 0x100) push(Buffer.from([(major << 5) | 24, n]));
        else if (n < 0x10000) { const b = Buffer.alloc(3); b[0] = (major << 5) | 25; b.writeUInt16BE(n, 1); push(b); }
        else if (n < 0x100000000) { const b = Buffer.alloc(5); b[0] = (major << 5) | 26; b.writeUInt32BE(n, 1); push(b); }
        else { const b = Buffer.alloc(9); b[0] = (major << 5) | 27; b.writeBigUInt64BE(BigInt(n), 1); push(b); }
    };
    const enc = (v) => {
        if (v === null || v === undefined) push(Buffer.from([0xf6]));
        else if (v === true) push(Buffer.from([0xf5]));
        else if (v === false) push(Buffer.from([0xf4]));
        else if (typeof v === 'number') {
            if (Number.isInteger(v) && Math.abs(v) <= Number.MAX_SAFE_INTEGER) {
                if (v >= 0) head(0, v); else head(1, -1 - v);
            } else {
                const b = Buffer.alloc(9); b[0] = 0xfb; b.writeDoubleBE(v, 1); push(b);
            }
        } else if (typeof v === 'string') {
            const b = Buffer.from(v, 'utf8'); head(3, b.length); push(b);
        } else if (v instanceof Float32Array) {
            head(6, 85);
            const b = Buffer.from(v.buffer, v.byteOffset, v.byteLength); head(2, b.length); push(b);
        } else if (v instanceof Uint8Array) {
            head(2, v.length); push(Buffer.from(v.buffer, v.byteOffset, v.byteLength));
        } else if (Array.isArray(v)) {
            head(4, v.length); for (const x of v) enc(x);
        } else {
            const keys = Object.keys(v);
            head(5, keys.length);
            for (const k of keys) { enc(k); enc(v[k]); }
        }
    };
    enc(value);
    return Buffer.concat(parts, size);
}

// JSON as rosbridge sends it: uint8[] base64, non-finite floats as null
function jsonEncode(value) {
    return JSON.stringify(value, (k, v) => {
        if (v instanceof Float32Array) return Array.from(v, x => (Number.isFinite(x) ? x : null));
        if (v instanceof Uint8Array) return Buffer.from(v.buffer, v.byteOffset, v.byteLength).toString('base64');
        if (typeof v === 'number' && !Number.isFinite(v)) return null;
        return v;
    });
}

// ============== WEBSOCKET (RFC 6455, server side) ==============
const WS_GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11';

function wsFrame(opcode, payload) {
    const len = payload.length;
    let header;
    if (len < 126) header = Buffer.from([0x80 | opcode, len]);
    else if (len < 0x10000) { header = Buffer.alloc(4); header[0] = 0x80 | opcode; header[1] = 126; header.writeUInt16BE(len, 2); }
    else { header = Buffer.alloc(10); header[0] = 0x80 | opcode; header[1] = 127; header.writeBigUInt64BE(BigInt(len), 2); }
    return Buffer.concat([header, payload]);
}

class Client {
    constructor(socket, id) {
        this.socket = socket;
        this.id = id;
        this.buf = Buffer.alloc(0);
        this.frag = [];
        this.subs = new Map();        // topic → { throttle, compression, last }
        this.closed = false;
        socket.on('data', (d) => this.onData(d));
        socket.on('close', () => { this.closed = true; clients.delete(this); log(`client ${id} disconnected`); });
        socket.on('error', () => { /* followed by close */ });
    }

    send(text) { this.write(wsFrame(1, Buffer.from(text, 'utf8'))); }
    sendBinary(buf) { this.write(wsFrame(2, buf)); }
    write(frame) {
        if (this.closed) return;
        // Like a full rosbridge queue: drop rather than buffer without bound
        if (this.socket.writableLength > 8 * 1024 * 1024) { stats.dropped++; return; }
        this.socket.write(frame);
    }

    onData(d) {
        this.buf = Buffer.concat([this.buf, d]);
        for (;;) {
            if (this.buf.length < 2) return;
            const fin = this.buf[0] & 0x80, opcode = this.buf[0] & 0x0f;
            const masked = this.buf[1] & 0x80;
            let len = this.buf[1] & 0x7f, off = 2;
            if (len === 126) { if (this.buf.length < 4) return; len = this.buf.readUInt16BE(2); off = 4; }
            else if (len === 127) { if (this.buf.length < 10) return; len = Number(this.buf.readBigUInt64BE(2)); off = 10; }
            const maskOff = off;
            if (masked) off += 4;
            if (this.buf.length < off + len) return;
            const payload = Buffer.from(this.buf.subarray(off, off + len));
            if (masked) for (let i = 0; i < len; i++) payload[i] ^= this.buf[maskOff + (i & 3)];
            this.buf = this.buf.subarray(off + len);
            if (opcode === 8) { this.write(wsFrame(8, Buffer.alloc(0))); this.socket.end(); return; }
            if (opcode === 9) { this.write(wsFrame(10, payload)); continue; }
            if (opcode === 10) continue;
            this.frag.push(payload);
            if (!fin) continue;
            const msg = Buffer.concat(this.frag).toString('utf8');
            this.frag = [];
            try { this.onMessage(JSON.parse(msg)); } catch (e) { log(`client ${this.id}: bad message (${e.message})`); }
        }
    }

    onMessage(m) {
        switch (m.op) {
            case 'subscribe': {
                const topic = TOPICS.get(m.topic);
                this.subs.set(m.topic, { throttle: m.throttle_rate || 0, compression: m.compression || 'none', last: 0 });
                log(`client ${this.id} subscribed ${m.topic} (${m.type || '?'}, throttle ${m.throttle_rate || 0} ms, queue ${m.queue_length || 1}, ${m.compression || 'none'})${topic && topic.gen ? '' : ' — never published here'}`);
                break;
            }
            case 'unsubscribe':
                this.subs.delete(m.topic);
                log(`client ${this.id} unsubscribed ${m.topic}`);
                break;
            case 'call_service': this.callService(m); break;
            default: break;    // advertise / publish / set_level: nothing to do here
        }
    }

    callService(m) {
        const name = String(m.service || '').replace(/^\//, '');
        const args = m.args || {};
        const all = [...TOPICS.values()];
        let values = null;
        if (name === 'rosapi/topics') values = { topics: all.map(t => t.name), types: all.map(t => t.type) };
        else if (name === 'rosapi/topics_and_raw_types') values = { topics: all.map(t => t.name), types: all.map(t => t.type), typedefs_full_text: all.map(() => '') };
        else if (name === 'rosapi/topic_type') values = { type: (TOPICS.get(args.topic) || {}).type || '' };
        else if (name === 'rosapi/topics_for_type') values = { topics: all.filter(t => t.type === args.type).map(t => t.name) };
        const res = { op: 'service_response', service: m.service, id: m.id, result: !!values, values: values || `service ${m.service} not emulated` };
        this.send(JSON.stringify(res));
    }
}

const clients = new Set();
let nextClient = 1;
const server = http.createServer((req, res) => { res.writeHead(426); res.end('rosbridge emulator: WebSocket only\n'); });
server.on('upgrade', (req, socket) => {
    const key = req.headers['sec-websocket-key'];
    if (!key) { socket.destroy(); return; }
    const accept = crypto.createHash('sha1').update(key + WS_GUID).digest('base64');
    socket.write('HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n'
        + `Sec-WebSocket-Accept: ${accept}\r\n\r\n`);
    socket.setNoDelay(true);
    const c = new Client(socket, nextClient++);
    clients.add(c);
    log(`client ${c.id} connected from ${req.socket.remoteAddress}`);
});

// ============== POSE FROM SITL ==============
const pose = { t: 0, tAtt: 0, lat: 0, lon: 0, alt: 0, vn: 0, ve: 0, vd: 0, roll: 0, pitch: 0, yaw: 0, p: 0, q: 0, r: 0, have: false, src: '' };
let homeAlt = null;          // GLOBAL_POSITION_INT alt − relative_alt
let truthLatLonAt = 0, truthAltAt = 0;
let anchor = null;           // { lat, lon, alt, mPerLat, mPerLon }

const registry = new Map();
for (const dialect of [minimal, common, ardupilotmega]) {
    if (dialect && dialect.REGISTRY) for (const [id, clazz] of Object.entries(dialect.REGISTRY)) registry.set(Number(id), clazz);
}
const protocol = new MavLinkProtocolV2(254, 192);
let mavSeq = 0;

function makeAnchor(lat, lon, alt) {
    const f = lat * DEG;
    return {
        lat, lon, alt,
        mPerLat: 111132.954 - 559.822 * Math.cos(2 * f) + 1.175 * Math.cos(4 * f),
        mPerLon: 111412.84 * Math.cos(f) - 93.5 * Math.cos(3 * f)
    };
}
if (ANCHOR_ARG || POSE_ARG) {
    const [la, lo, al] = (ANCHOR_ARG || '0,0,0').split(',').map(Number);
    anchor = makeAnchor(la, lo, al);
}
if (POSE_ARG) {
    const [e, n, u, yaw] = POSE_ARG.split(',').map(Number);
    const v = parseFloat(arg('speed', '0')), t0 = parseFloat(arg('pose-t0', String(Date.now())));
    const ya = (yaw || 0) * DEG, vn = v * Math.cos(ya), ve = v * Math.sin(ya);
    const place = () => {
        const now = Date.now(), dt = (now - t0) / 1000;
        Object.assign(pose, {
            lat: anchor.lat + (n + vn * dt) / anchor.mPerLat, lon: anchor.lon + (e + ve * dt) / anchor.mPerLon, alt: anchor.alt + u,
            yaw: ya, vn, ve, vd: 0, t: now, tAtt: now, have: true, src: v ? `moving ${v} m/s (--pose --speed)` : 'fixed (--pose)'
        });
    };
    place();
    setInterval(place, 50);
}

function tryAnchor() {
    if (anchor || !pose.have || (pose.lat === 0 && pose.lon === 0)) return;
    const alt = homeAlt !== null ? homeAlt : pose.alt;
    // A scene fixed to the map keeps its own origin; the surface is still home's
    const o = SCENE.origin || { lat: pose.lat, lon: pose.lon };
    anchor = makeAnchor(o.lat, o.lon, alt);
    log(`scene '${SCENE_NAME}' anchored at ${anchor.lat.toFixed(7)}, ${anchor.lon.toFixed(7)}, ${alt.toFixed(2)} m (${homeAlt !== null ? 'home' : 'first true altitude'})`);
}

function connectMavlink() {
    const m = MAV.match(/^tcp:([^:]+):(\d+)$/);
    if (!m) { console.error('only tcp:host:port is supported for --mav'); process.exit(1); }
    const sock = net.createConnection({ host: m[1], port: parseInt(m[2], 10) }, () => {
        log(`MAVLink connected ${MAV}`);
        const send = (msg) => sock.write(protocol.serialize(msg, mavSeq++ & 0xff));
        for (const [id, rate] of [[6, 10], [10, 20], [12, 10]]) {     // POSITION, EXTRA1, EXTRA3
            const req = new common.RequestDataStream();
            req.targetSystem = 1; req.targetComponent = 1;
            req.reqStreamId = id; req.reqMessageRate = rate; req.startStop = 1;
            send(req);
        }
        for (const id of [164, 108]) {           // SIMSTATE, SIM_STATE at 20 Hz
            const cmd = new common.CommandLong();
            cmd.targetSystem = 1; cmd.targetComponent = 1;
            cmd.command = common.MavCmd.SET_MESSAGE_INTERVAL;
            cmd._param1 = id; cmd._param2 = 50000;
            send(cmd);
        }
    });
    sock.on('error', (e) => { log(`MAVLink error: ${e.message}`); });
    sock.on('close', () => { pose.have = false; setTimeout(connectMavlink, 2000); });
    const parser = sock.pipe(new MavLinkPacketSplitter()).pipe(new MavLinkPacketParser());
    parser.on('data', (packet) => {
        const clazz = registry.get(packet.header.msgid);
        if (!clazz) return;
        let msg;
        try { msg = packet.protocol.data(packet.payload, clazz); } catch (_) { return; }
        const now = Date.now();
        const truthLL = now - truthLatLonAt < 1000, truthAlt = now - truthAltAt < 1000;
        switch (packet.header.msgid) {
            case 164:      // SIMSTATE: true lat/lon (degE7) and attitude
                stats.poses++;
                if (msg.lat === 0 && msg.lng === 0) break;
                pose.lat = msg.lat / 1e7; pose.lon = msg.lng / 1e7;
                pose.roll = msg.roll; pose.pitch = msg.pitch; pose.yaw = msg.yaw;
                pose.p = msg.xgyro; pose.q = msg.ygyro; pose.r = msg.zgyro; pose.tAtt = now;
                truthLatLonAt = now; pose.t = now; pose.have = true; pose.src = 'SITL truth';
                break;
            case 108:      // SIM_STATE: true altitude and velocity
                if (!Number.isFinite(msg.alt)) break;
                pose.alt = msg.alt; pose.vn = msg.vn; pose.ve = msg.ve; pose.vd = msg.vd;
                truthAltAt = now;
                if (!truthLL) {
                    pose.roll = msg.roll; pose.pitch = msg.pitch; pose.yaw = msg.yaw;
                }
                break;
            case 33:       // GLOBAL_POSITION_INT: fallback, and the home altitude
                if (msg.lat !== 0 || msg.lon !== 0) {
                    if (homeAlt === null) homeAlt = (msg.alt - msg.relativeAlt) / 1000;
                    if (!truthLL) { pose.lat = msg.lat / 1e7; pose.lon = msg.lon / 1e7; pose.t = now; pose.have = true; pose.src = 'EKF'; }
                    if (!truthAlt) { pose.alt = msg.alt / 1000; pose.vn = msg.vx / 100; pose.ve = msg.vy / 100; pose.vd = msg.vz / 100; }
                }
                break;
            case 30:       // ATTITUDE: the autopilot's clock; attitude fallback
                onBootTime(msg.timeBootMs, now);
                if (!truthLL) {
                    pose.roll = msg.roll; pose.pitch = msg.pitch; pose.yaw = msg.yaw;
                    pose.p = msg.rollspeed; pose.q = msg.pitchspeed; pose.r = msg.yawspeed; pose.tAtt = now;
                }
                break;
        }
        // Without GPS, wait for a true altitude before anchoring
        if (pose.have && (homeAlt !== null || truthAlt)) tryAnchor();
    });
}

// ============== AUTOPILOT CLOCK ==============
// A SITL runs at its speed-up: ATTITUDE.time_boot_ms against arrival over the
// last 3 s gives the rate (1 with --pose, where there is no autopilot). The
// pose is projected and the sector sonar's head steps on this clock.
const simClock = { rate: 1, ring: [] };          // ring: [wall ms, boot ms]
function onBootTime(boot, now) {
    const r = simClock.ring;
    if (r.length && boot < r[r.length - 1][1]) r.length = 0;     // SITL restarted
    r.push([now, boot]);
    while (r.length > 2 && now - r[0][0] > 3000) r.shift();
    const [w0, b0] = r[0];
    if (now - w0 >= 500 && boot > b0) simClock.rate = Math.max(0.05, Math.min(100, (boot - b0) / (now - w0)));
}
function simRate() { return POSE_ARG ? 1 : simClock.rate; }
// Sensor time (s) at a wall time, and back
function sensorTime(wall) {
    const r = simClock.ring;
    if (POSE_ARG || !r.length) return wall / 1000;
    const [w, b] = r[r.length - 1];
    return (b + (wall - w) * simClock.rate) / 1000;
}
function wallTime(s) {
    const r = simClock.ring;
    if (POSE_ARG || !r.length) return s * 1000;
    const [w, b] = r[r.length - 1];
    return w + (s * 1000 - b) / simClock.rate;
}

// ============== SENSORS ==============
function eulerToMatrix(roll, pitch, yaw, out = new Float64Array(9)) {
    const cr = Math.cos(roll), sr = Math.sin(roll), cp = Math.cos(pitch), sp = Math.sin(pitch), cy = Math.cos(yaw), sy = Math.sin(yaw);
    out[0] = cy * cp; out[1] = cy * sp * sr - sy * cr; out[2] = cy * sp * cr + sy * sr;
    out[3] = sy * cp; out[4] = sy * sp * sr + cy * cr; out[5] = sy * sp * cr - cy * sr;
    out[6] = -sp;     out[7] = cp * sr;                out[8] = cp * cr;
    return out;
}
const R_LIDAR = eulerToMatrix(LIDAR_MOUNT[0] * DEG, LIDAR_MOUNT[1] * DEG, LIDAR_MOUNT[2] * DEG);
const R_DOWN = eulerToMatrix(DOWN_MOUNT[0] * DEG, DOWN_MOUNT[1] * DEG, DOWN_MOUNT[2] * DEG);
const R_PROFILER = eulerToMatrix(PROFILER_MOUNT[0] * DEG, PROFILER_MOUNT[1] * DEG, PROFILER_MOUNT[2] * DEG);
const R_SECTOR = eulerToMatrix(SECTOR.mount[0] * DEG, SECTOR.mount[1] * DEG, SECTOR.mount[2] * DEG);
const Ra = new Float64Array(9);

// Sensor origin and the mapping of sensor-frame directions to ENU, at the pose
// projected to `at` (wall ms; velocity; body rates through the Euler
// kinematics, so a scan taken in a turn is not drawn with the attitude of up
// to 50 ms ago), on the autopilot's clock. The inverse of what the GCS does
// with the points.
function rig(Rm, at = Date.now()) {
    if (!anchor || !pose.have) return null;
    const k = simRate();
    const dt = Math.min(0.5, k * (at - pose.t) / 1000);
    const lat = pose.lat + pose.vn * dt / anchor.mPerLat;
    const lon = pose.lon + pose.ve * dt / anchor.mPerLon;
    const alt = pose.alt - pose.vd * dt;
    let roll = pose.roll, pitch = pose.pitch, yaw = pose.yaw;
    const da = Math.min(0.5, k * (at - pose.tAtt) / 1000), n = Math.max(1, Math.ceil(Math.abs(da) / 0.02)), h = da / n;
    for (let k = 0; k < n; k++) {
        const sr = Math.sin(roll), cr = Math.cos(roll), qr = pose.q * sr + pose.r * cr;
        roll += h * (pose.p + qr * Math.tan(pitch));
        pitch += h * (pose.q * cr - pose.r * sr);
        yaw += h * qr / Math.max(1e-3, Math.cos(pitch));
    }
    eulerToMatrix(roll, pitch, yaw, Ra);
    const ln = Ra[0] * LEVER[0] + Ra[1] * LEVER[1] + Ra[2] * LEVER[2];
    const le = Ra[3] * LEVER[0] + Ra[4] * LEVER[1] + Ra[5] * LEVER[2];
    const ld = Ra[6] * LEVER[0] + Ra[7] * LEVER[1] + Ra[8] * LEVER[2];
    const o = [(lon - anchor.lon) * anchor.mPerLon + le, (lat - anchor.lat) * anchor.mPerLat + ln, alt - anchor.alt - ld];
    // sensor FLU (x, y, z) → FRD (x, -y, -z) → body (Rm) → NED (Ra) → ENU
    const dir = (x, y, z) => {
        const fx = x, fy = -y, fz = -z;
        const bx = Rm[0] * fx + Rm[1] * fy + Rm[2] * fz, by = Rm[3] * fx + Rm[4] * fy + Rm[5] * fz, bz = Rm[6] * fx + Rm[7] * fy + Rm[8] * fz;
        const n = Ra[0] * bx + Ra[1] * by + Ra[2] * bz, e = Ra[3] * bx + Ra[4] * by + Ra[5] * bz, d = Ra[6] * bx + Ra[7] * by + Ra[8] * bz;
        return [e, n, -d];
    };
    return { o, dir };
}

function shoot(r, x, y, z, maxRange) {
    const [de, dn, du] = r.dir(x, y, z);
    return castRay(SCENE, r.o[0], r.o[1], r.o[2], de, dn, du, maxRange);
}

function gaussRand() {
    return Math.sqrt(-2 * Math.log(1 - Math.random())) * Math.cos(2 * Math.PI * Math.random());
}

let headerSeq = 0;
function header(frameId, ms = Date.now()) {
    const sec = Math.floor(ms / 1000), ns = Math.round((ms - sec * 1000) * 1e6);
    return ROS1 ? { seq: headerSeq++, stamp: { secs: sec, nsecs: ns }, frame_id: frameId }
        : { stamp: { sec, nanosec: ns }, frame_id: frameId };
}

const F32 = 7, F64 = 8, U8 = 2;
function cloud(frameId, fields, pointStep, count, data) {
    return {
        header: header(frameId), height: 1, width: count,
        fields: fields.map(([name, offset, datatype]) => ({ name, offset, datatype, count: 1 })),
        is_bigendian: false, point_step: pointStep, row_step: pointStep * count,
        data, is_dense: true
    };
}

// One Mid-360 scan (360° × -7…+52°, non-repetitive: random directions), shared
// by /livox/lidar (sensor frame) and /cloud_registered (world frame).
let lastScan = null;
function livoxScan() {
    const now = Date.now();
    if (lastScan && now - lastScan.t < 20) return lastScan;
    const r = rig(R_LIDAR);
    if (!r) return null;
    const sensor = [], world = [];
    for (let k = 0; k < LIDAR_POINTS; k++) {
        const az = Math.random() * 2 * Math.PI;
        const el = (-7 + Math.random() * 59) * DEG;
        const x = Math.cos(el) * Math.cos(az), y = Math.cos(el) * Math.sin(az), z = Math.sin(el);
        const d = shoot(r, x, y, z, 100);
        if (!d) { sensor.push(0, 0, 0); continue; }     // no return: Livox sends (0, 0, 0)
        sensor.push(x * d, y * d, z * d);
        const [de, dn, du] = r.dir(x, y, z);
        world.push(r.o[0] + de * d, r.o[1] + dn * d, r.o[2] + du * d);
    }
    lastScan = { t: now, sensor, world };
    return lastScan;
}

const GEN = {
    '/livox/lidar': () => {
        const s = livoxScan();
        if (!s) return null;
        const n = s.sensor.length / 3, step = 26;     // livox_ros_driver2: x y z intensity tag line timestamp
        const buf = Buffer.alloc(n * step);
        for (let k = 0; k < n; k++) {
            const o = k * step;
            buf.writeFloatLE(s.sensor[k * 3], o); buf.writeFloatLE(s.sensor[k * 3 + 1], o + 4); buf.writeFloatLE(s.sensor[k * 3 + 2], o + 8);
            buf.writeFloatLE(40 + Math.random() * 120, o + 12); buf[o + 16] = 0; buf[o + 17] = k % 4;
            buf.writeDoubleLE(Date.now() * 1e6, o + 18);
        }
        return cloud('livox_frame', [['x', 0, F32], ['y', 4, F32], ['z', 8, F32], ['intensity', 12, F32], ['tag', 16, U8], ['line', 17, U8], ['timestamp', 18, F64]],
            step, n, new Uint8Array(buf.buffer, buf.byteOffset, buf.length));
    },
    '/cloud_registered': () => {
        const s = livoxScan();
        if (!s) return null;
        const n = s.world.length / 3, buf = Buffer.alloc(n * 16);
        for (let k = 0; k < n; k++) {
            buf.writeFloatLE(s.world[k * 3], k * 16); buf.writeFloatLE(s.world[k * 3 + 1], k * 16 + 4);
            buf.writeFloatLE(s.world[k * 3 + 2], k * 16 + 8); buf.writeFloatLE(100, k * 16 + 12);
        }
        return cloud('map', [['x', 0, F32], ['y', 4, F32], ['z', 8, F32], ['intensity', 12, F32]], 16, n, new Uint8Array(buf.buffer, buf.byteOffset, buf.length));
    },
    '/scan': () => {
        const r = rig(R_DOWN);
        if (!r) return null;
        const amin = -135 * DEG, inc = 0.5 * DEG, n = 541, ranges = new Float32Array(n);
        for (let i = 0; i < n; i++) {
            const a = amin + i * inc;
            ranges[i] = shoot(r, Math.cos(a), Math.sin(a), 0, 100) || Infinity;
        }
        return {
            header: header('laser'), angle_min: amin, angle_max: amin + (n - 1) * inc, angle_increment: inc,
            time_increment: 0, scan_time: 0.05, range_min: 0.1, range_max: 100, ranges, intensities: new Float32Array(0)
        };
    },
    '/sonar/multibeam': () => {
        const r = rig(R_DOWN);
        if (!r) return null;
        const n = 256, buf = Buffer.alloc(n * 16);
        let k = 0;
        for (let i = 0; i < n; i++) {
            const a = (-65 + 130 * i / (n - 1)) * DEG;
            const x = Math.cos(a), y = Math.sin(a);
            const d = shoot(r, x, y, 0, 120);
            if (!d) continue;
            buf.writeFloatLE(x * d, k * 16); buf.writeFloatLE(y * d, k * 16 + 4); buf.writeFloatLE(0, k * 16 + 8);
            buf.writeFloatLE(Math.random() * 100, k * 16 + 12);
            k++;
        }
        return cloud('sonar', [['x', 0, F32], ['y', 4, F32], ['z', 8, F32], ['intensity', 12, F32]], 16, k,
            new Uint8Array(buf.buffer, buf.byteOffset, k * 16));
    },
    '/sonar/imaging': () => {
        const r = rig(R_DOWN);
        if (!r) return null;
        const n = SONAR.beams, half = SONAR.aperture / 2, buf = Buffer.alloc(n * 16);
        // Aeration grows with the hull's motion in the waves and with speed
        const motion = Math.hypot(pose.p, pose.q) + 0.05 * Math.hypot(pose.vn, pose.ve);
        const bubbles = SONAR.clean ? 0 : Math.min(0.25, 0.01 + 0.4 * motion);
        let k = 0;
        for (let i = 0; i < n; i++) {
            const a = (-half + SONAR.aperture * (i + 0.5) / n) * DEG;
            // The echo comes from somewhere in the vertical aperture …
            const el = SONAR.clean ? 0 : (Math.random() - 0.5) * SONAR.elevation * DEG;
            let d = shoot(r, Math.cos(a) * Math.cos(el), Math.sin(a) * Math.cos(el), Math.sin(el), SONAR.range);
            if (!SONAR.clean) {
                if (Math.random() < 0.03) d = 0;                                          // no return
                else if (Math.random() < bubbles) d = 0.5 + Math.random() * 4;             // aeration under the hull
                else if (d && Math.random() < 0.004) d = 2 + Math.random() * (d - 2);      // fish, particles
                if (d) d += (0.03 + 0.002 * d) * gaussRand();
            }
            if (!(d > 0.3)) continue;
            // … and is reported on the beam's axis
            buf.writeFloatLE(Math.cos(a) * d, k * 16); buf.writeFloatLE(Math.sin(a) * d, k * 16 + 4); buf.writeFloatLE(0, k * 16 + 8);
            buf.writeFloatLE(80 + Math.random() * 100, k * 16 + 12);
            k++;
        }
        return cloud('imaging_sonar', [['x', 0, F32], ['y', 4, F32], ['z', 8, F32], ['intensity', 12, F32]], 16, k,
            new Uint8Array(buf.buffer, buf.byteOffset, k * 16));
    },
    '/sonar/profiler': () => {
        const r = rig(R_PROFILER);
        if (!r) return null;
        const n = 400, buf = Buffer.alloc(2 * n * 16);
        let k = 0;
        for (let i = 0; i < 2 * n; i++) {
            const a = 2 * Math.PI * (i % n) / n, c = Math.cos(a), sn = Math.sin(a);
            const [x, y, z] = i < n ? [0, c, sn] : [c, sn, 0];
            const d = shoot(r, x, y, z, 40);
            if (!d) continue;
            buf.writeFloatLE(x * d, k * 16); buf.writeFloatLE(y * d, k * 16 + 4); buf.writeFloatLE(z * d, k * 16 + 8);
            buf.writeFloatLE(50 + Math.random() * 100, k * 16 + 12);
            k++;
        }
        return cloud('sonar_profiler', [['x', 0, F32], ['y', 4, F32], ['z', 8, F32], ['intensity', 12, F32]], 16, k,
            new Uint8Array(buf.buffer, buf.byteOffset, k * 16));
    },
    '/ping1d/range': () => {
        const r = rig(R_DOWN);
        if (!r) return null;
        const d = shoot(r, 1, 0, 0, 100);
        return {
            header: header('ping1d'), radiation_type: 0, field_of_view: 0.52, min_range: 0.5, max_range: 100,
            range: d ? d + (Math.random() - 0.5) * 0.1 : Infinity
        };
    },
    // The sweeps the head has finished (see SECTOR SCANNING SONAR)
    '/sonar/sector': () => sector.ready.shift() || null
};

// ============== SECTOR SCANNING SONAR ==============
// Rays across the beam: its axis, rings at half and full half-power width
// (--sector-beam in the sweep plane, --sector-fan across it) and, for a wide
// fan, a line along it every 1.5°; each weighted by the beam's power pattern
// (a Gaussian, half power at the edge of the -3 dB width).
const SECTOR_RAYS = (() => {
    const hb = SECTOR.beam / 2 * DEG, hf = SECTOR.fan / 2 * DEG, out = [[0, 0]];
    for (const f of [0.5, 1]) for (let k = 0; k < 8; k++) out.push([f * hb * Math.cos(k * Math.PI / 4), f * hf * Math.sin(k * Math.PI / 4)]);
    const nf = Math.floor(SECTOR.fan / 1.5);
    for (let k = 1; k < nf; k++) out.push([0, hf * k / nf], [0, -hf * k / nf]);
    const w = ([a, z]) => Math.exp(-Math.LN2 * ((hb ? (a / hb) ** 2 : 0) + (hf ? (z / hf) ** 2 : 0)));
    return out.map(r => [r[0], r[1], w(r)]);
})();
const SECTOR_DETECT = 0.25;        // the bottom: where this much of the echo's energy is back
const echoRays = SECTOR_RAYS.map(() => [0, 0]);

const sector = { angle: -SECTOR.half, dir: 1, next: null, batch: null, ready: [], pings: 0 };

function sectorEcho(r, a) {
    // The echo: each ray's return, in order of range, until DETECT of the energy
    let n = 0, total = 0;
    for (const [da, dz, w] of SECTOR_RAYS) {
        const az = a + da, cz = Math.cos(dz);
        const t = shoot(r, cz * Math.cos(az), cz * Math.sin(az), Math.sin(dz), SECTOR.range);
        total += w;
        if (t) { echoRays[n][0] = t; echoRays[n][1] = w; n++; }
    }
    const hits = echoRays.slice(0, n).sort((x, y) => x[0] - y[0]);
    let d = 0, acc = 0;
    for (const [t, w] of hits) { acc += w; if (acc >= SECTOR_DETECT * total) { d = t; break; } }
    if (!SONAR.clean) {
        // A transducer on a pole sees less aeration than one in the hull
        const motion = Math.hypot(pose.p, pose.q) + 0.05 * Math.hypot(pose.vn, pose.ve);
        const bubbles = Math.min(0.12, 0.005 + 0.2 * motion);
        if (Math.random() < 0.02) d = 0;                                          // no return
        else if (Math.random() < bubbles) d = 0.8 + Math.random() * 3;             // aeration
        else if (d && Math.random() < 0.004) d = 2 + Math.random() * (d - 2);      // fish, particles
        if (d) {
            const bin = SECTOR.range / 1200;                                       // 1 200 samples over the range
            d = Math.round((d + (0.03 + 0.002 * d) * gaussRand()) / bin) * bin;
        }
    }
    return d >= SECTOR.minRange ? d : Infinity;
}

// One ping at sensor time s (seconds on the autopilot's clock), then the head
// steps; at the end of the sector it turns back
function sectorPing(s) {
    const wall = wallTime(s);
    const r = rig(R_SECTOR, wall);
    if (!r) return;
    if (!sector.batch) sector.batch = { s0: s, wall0: wall, angle0: sector.angle, dir: sector.dir, ranges: [] };
    sector.batch.ranges.push(sectorEcho(r, sector.angle * DEG));
    sector.pings++;
    const half = SECTOR.half + 1e-9;
    let next = sector.angle + sector.dir * SECTOR.step;
    const turn = next > half || next < -half;
    if (turn) { sector.dir = -sector.dir; next = sector.angle + sector.dir * SECTOR.step; }
    sector.angle = next;
    if (turn || sector.batch.ranges.length >= SECTOR.perBatch) sectorFlush();
}

// The pings so far as one LaserScan: first ping's stamp, the ping period as
// time_increment, the head's step (signed: the sweep's direction) as angle_increment
function sectorFlush() {
    const b = sector.batch;
    sector.batch = null;
    if (!b || !b.ranges.length) return;
    const n = b.ranges.length, inc = b.dir * SECTOR.step * DEG;
    sector.ready.push({
        header: header('sector_sonar', b.wall0),
        angle_min: b.angle0 * DEG, angle_max: b.angle0 * DEG + (n - 1) * inc, angle_increment: inc,
        time_increment: SECTOR.untimed ? 0 : SECTOR.period, scan_time: SECTOR.period * SECTOR.sweepPings,
        range_min: SECTOR.minRange, range_max: SECTOR.range,
        ranges: Float32Array.from(b.ranges), intensities: new Float32Array(0)
    });
    // Like rosbridge, a subscriber gets what is published after it subscribed
    let listened = false;
    for (const c of clients) if (c.subs.has('/sonar/sector')) listened = true;
    if (!listened) sector.ready.length = 0;
    else if (sector.ready.length > 8) sector.ready.shift();
}

setInterval(() => {
    if (!anchor || !pose.have) { sector.next = null; return; }
    const s = sensorTime(Date.now());
    // Start, a SITL restart, or too far behind (a stalled loop): from now
    if (sector.next === null || s - sector.next > 2 || sector.next - s > 2) { sectorFlush(); sector.next = s; }
    while (sector.next <= s) { sectorPing(sector.next); sector.next += SECTOR.period; }
}, 5);

// name → { name, type, rate (Hz), gen }
const TOPICS = new Map([
    ['/livox/lidar', T('sensor_msgs', 'PointCloud2'), 10],
    ['/cloud_registered', T('sensor_msgs', 'PointCloud2'), 10],
    ['/scan', T('sensor_msgs', 'LaserScan'), 20],
    ['/sonar/multibeam', T('sensor_msgs', 'PointCloud2'), 10],
    ['/ping1d/range', T('sensor_msgs', 'Range'), 10],
    ['/sonar/profiler', T('sensor_msgs', 'PointCloud2'), 10],
    ['/sonar/imaging', T('sensor_msgs', 'PointCloud2'), SONAR.rate],
    ['/sonar/sector', T('sensor_msgs', 'LaserScan'), 100],
    ['/rosout', ROS1 ? 'rosgraph_msgs/Log' : 'rcl_interfaces/msg/Log', 0],
    ['/tf', T('tf2_msgs', 'TFMessage'), 0],
    ['/camera/image_raw', T('sensor_msgs', 'Image'), 0],
    ['/mavros/local_position/pose', T('geometry_msgs', 'PoseStamped'), 0]
].map(([name, type, rate]) => [name, { name, type, rate, gen: GEN[name] || null, last: 0, sent: 0 }]));

// ============== PUBLISHING ==============
const stats = { dropped: 0, bytes: 0, poses: 0 };

setInterval(() => {
    const now = Date.now();
    for (const t of TOPICS.values()) {
        if (!t.gen || now - t.last < 1000 / t.rate) continue;
        // Subscribers whose throttle_rate lets a message through now
        const due = [];
        for (const c of clients) {
            const s = c.subs.get(t.name);
            if (s && now - s.last >= s.throttle) due.push([c, s]);
        }
        if (!due.length) continue;
        t.last = now;
        const msg = t.gen();
        if (!msg) continue;
        const env = { op: 'publish', topic: t.name, msg };
        let json = null, cbor = null;
        for (const [c, s] of due) {
            s.last = now;
            if (s.compression === 'cbor') {
                cbor = cbor || cborEncode(env);
                c.sendBinary(cbor);
                stats.bytes += cbor.length;
            } else {
                json = json || jsonEncode(env);
                c.send(json);
                stats.bytes += json.length;
            }
            t.sent++;
        }
    }
}, 10);

// ============== LOG ==============
function log(s) { console.log(`[rosbridge-sim] ${s}`); }

let lastReport = Date.now();
setInterval(() => {
    const now = Date.now(), dt = (now - lastReport) / 1000;
    lastReport = now;
    const parts = [];
    for (const t of TOPICS.values()) if (t.sent) { parts.push(`${t.name} ${(t.sent / dt).toFixed(1)}/s`); t.sent = 0; }
    const p = pose.have ? `${pose.lat.toFixed(6)}, ${pose.lon.toFixed(6)}, ${pose.alt.toFixed(1)} m (${pose.src})` : 'no pose yet';
    const clock = simRate() !== 1 ? ` · clock ×${simRate().toFixed(1)}` : '';
    const pings = sector.pings ? ` · sector ${(sector.pings / dt).toFixed(1)} pings/s` : '';
    sector.pings = 0;
    log(`${clients.size} client(s) · ${p}${clock} · truth ${(stats.poses / dt).toFixed(1)} Hz${pings} · ${parts.join(' · ') || 'nothing published'} · ${(stats.bytes / dt / 1024).toFixed(0)} KB/s${stats.dropped ? ` · ${stats.dropped} dropped` : ''}`);
    stats.bytes = 0;
    stats.poses = 0;
}, 5000);

server.listen(PORT, () => {
    log(`listening on ws://0.0.0.0:${PORT} · scene ${SCENE_NAME} · ${ROS1 ? 'ROS 1' : 'ROS 2'} type names`);
    log(`mount: lidar ${LIDAR_MOUNT.join(',')} · down-looking ${DOWN_MOUNT.join(',')} · profiler ${PROFILER_MOUNT.join(',')} · sector ${SECTOR.mount.join(',')} · lever ${LEVER.join(',')}`);
    log(`sector sonar: ±${SECTOR.half}° in ${SECTOR.step}° steps, ${SECTOR.range} m, beam ${SECTOR.beam}° × ${SECTOR.fan}°, `
        + `${(1 / SECTOR.period).toFixed(1)} pings/s, a sweep every ${(SECTOR.period * SECTOR.sweepPings).toFixed(1)} s, ${SECTOR_RAYS.length} rays a ping`);
});
if (!POSE_ARG) connectMavlink();
