/**
 * msp-manager.js - MSP / MSP2 telemetry adapter (INAV, Betaflight)
 *
 * Follows the same contract as the CORV binary handler in main-mavlink.js: the flight
 * controller's native protocol is decoded here and re-emitted to the renderer as
 * synthetic MAVLink messages on the 'mavlink-message' channel. Nothing downstream
 * (HUD, 3D, maps, graphs) knows MSP exists.
 *
 * The essential difference from MAVLink: MSP is request/response, not a stream. If the
 * GCS does not ask, nothing arrives. A scheduler therefore cycles a fixed set of
 * requests at per-message rates, with exactly one request in flight at a time — MSP has
 * no sequence numbers, so overlapping requests cannot be matched to their replies.
 *
 * Framing:
 *   v1:  '$' 'M' <dir> <size:u8> <cmd:u8> <payload> <XOR of size..payload>
 *   v2:  '$' 'X' <dir> <flag:u8> <cmd:u16LE> <size:u16LE> <payload> <CRC8 DVB-S2>
 */

const { ipcMain } = require('electron');
const net = require('net');

let SerialPort = null;
function ensureSerialLoaded() {
    if (SerialPort) return;
    SerialPort = require('serialport').SerialPort;
}

// ── MSP command ids ──────────────────────────────────────────────────────────
const MSP_API_VERSION   = 1;
const MSP_FC_VARIANT    = 2;
const MSP_FC_VERSION    = 3;
const MSP_NAME          = 10;
const MSP_RAW_IMU       = 102;
const MSP_RC            = 105;
const MSP_RAW_GPS       = 106;
const MSP_COMP_GPS      = 107;
const MSP_ATTITUDE      = 108;
const MSP_ALTITUDE      = 109;
const MSP_ANALOG        = 110;
const MSP_BOXNAMES      = 116;
const MSP_BOXIDS        = 119;
const MSP_STATUS_EX     = 150;

const MSP2_INAV_STATUS   = 0x2000;
const MSP2_INAV_ANALOG   = 0x2002;
const MSP2_INAV_AIR_SPEED = 0x2009;

// ── Connection state ─────────────────────────────────────────────────────────
let mainWindow = null;
let port = null;          // SerialPort or net.Socket
let portKind = null;      // 'serial' | 'tcp'
let rxBuf = Buffer.alloc(0);
let schedulerTimer = null;
let pending = null;       // { cmd, timer, resolve }
let queue = [];           // pending command ids
let bytesRx = 0;
let statsTimer = null;
let lastStatsBytes = 0;

// Decoded vehicle state kept between polls, because each MSP reply carries only a
// slice of what one MAVLink message needs (VFR_HUD wants speed + altitude + heading,
// which arrive from three different commands).
const vs = {
    roll: 0, pitch: 0, yaw: 0,
    lat: 0, lon: 0, altGps: 0, groundspeed: 0, gpsCourse: 0, fixType: 0, numSat: 0, hdop: 65535,
    altBaro: 0, climb: 0, airspeed: 0,
    voltage: 0, current: 0, mahDrawn: 0, rssi: 0, batteryPercent: 0,
    armed: false, modeName: 'MANUAL', boxNames: [], boxIds: [], activeBoxes: 0n,
    accX: 0, accY: 0, accZ: 0, gyroX: 0, gyroY: 0, gyroZ: 0,
    rcChannels: [],
    fcVariant: '', fcVersion: '',
};

// ── Framing helpers ──────────────────────────────────────────────────────────

function crc8DvbS2(crc, byte) {
    crc ^= byte;
    for (let i = 0; i < 8; i++) {
        crc = (crc & 0x80) ? ((crc << 1) ^ 0xD5) & 0xFF : (crc << 1) & 0xFF;
    }
    return crc;
}

/** Encode an MSP request. Commands >= 255 must use v2. */
function encodeRequest(cmd, payload = Buffer.alloc(0)) {
    if (cmd > 254 || payload.length > 254) {
        const buf = Buffer.alloc(9 + payload.length);
        buf.write('$X<', 0, 'ascii');
        buf.writeUInt8(0, 3);                 // flag
        buf.writeUInt16LE(cmd, 4);
        buf.writeUInt16LE(payload.length, 6);
        payload.copy(buf, 8);
        let crc = 0;
        for (let i = 3; i < 8 + payload.length; i++) crc = crc8DvbS2(crc, buf[i]);
        buf.writeUInt8(crc, 8 + payload.length);
        return buf;
    }
    const buf = Buffer.alloc(6 + payload.length);
    buf.write('$M<', 0, 'ascii');
    buf.writeUInt8(payload.length, 3);
    buf.writeUInt8(cmd, 4);
    payload.copy(buf, 5);
    let checksum = payload.length ^ cmd;
    for (const b of payload) checksum ^= b;
    buf.writeUInt8(checksum & 0xFF, 5 + payload.length);
    return buf;
}

/**
 * Consume as many complete frames as rxBuf holds.
 * Returns nothing; each frame is dispatched to handleReply().
 */
function parseBuffer() {
    let i = 0;
    while (i < rxBuf.length) {
        // Resynchronise on '$'
        if (rxBuf[i] !== 0x24) { i++; continue; }
        if (i + 2 >= rxBuf.length) break;

        const version = rxBuf[i + 1]; // 'M' (v1) or 'X' (v2)
        const dir = rxBuf[i + 2];     // '>' ok, '!' error

        if (version === 0x4D) { // 'M'
            if (i + 5 > rxBuf.length) break;
            const size = rxBuf[i + 3];
            const cmd = rxBuf[i + 4];
            const total = 6 + size;
            if (i + total > rxBuf.length) break;
            const payload = rxBuf.subarray(i + 5, i + 5 + size);
            let checksum = size ^ cmd;
            for (const b of payload) checksum ^= b;
            if ((checksum & 0xFF) === rxBuf[i + 5 + size]) {
                if (dir !== 0x21) handleReply(cmd, Buffer.from(payload));
                else failPending(cmd);
                i += total;
                continue;
            }
            i++; // bad checksum — treat the '$' as noise and rescan
            continue;
        }

        if (version === 0x58) { // 'X'
            if (i + 8 > rxBuf.length) break;
            const cmd = rxBuf.readUInt16LE(i + 4);
            const size = rxBuf.readUInt16LE(i + 6);
            const total = 9 + size;
            if (i + total > rxBuf.length) break;
            let crc = 0;
            for (let k = i + 3; k < i + 8 + size; k++) crc = crc8DvbS2(crc, rxBuf[k]);
            if (crc === rxBuf[i + 8 + size]) {
                if (dir !== 0x21) handleReply(cmd, Buffer.from(rxBuf.subarray(i + 8, i + 8 + size)));
                else failPending(cmd);
                i += total;
                continue;
            }
            i++;
            continue;
        }

        i++; // unknown version byte
    }
    rxBuf = i > 0 ? rxBuf.subarray(i) : rxBuf;
    // Never let garbage accumulate without a valid frame
    if (rxBuf.length > 8192) rxBuf = rxBuf.subarray(rxBuf.length - 1024);
}

// ── Request scheduling ───────────────────────────────────────────────────────

// Poll rates. "slow" is for long-range radio links where a 25 Hz attitude poll
// would use the whole budget; it is selected at connect time.
const SCHEDULES = {
    normal: [
        { cmd: MSP_ATTITUDE, hz: 25 },
        { cmd: MSP_ALTITUDE, hz: 10 },
        { cmd: MSP_RAW_GPS, hz: 5 },
        { cmd: MSP2_INAV_STATUS, hz: 5 },
        { cmd: MSP_RAW_IMU, hz: 5 },
        { cmd: MSP_RC, hz: 5 },
        { cmd: MSP_COMP_GPS, hz: 2 },
        { cmd: MSP2_INAV_ANALOG, hz: 2 },
        { cmd: MSP2_INAV_AIR_SPEED, hz: 2 },
    ],
    slow: [
        { cmd: MSP_ATTITUDE, hz: 5 },
        { cmd: MSP_ALTITUDE, hz: 2 },
        { cmd: MSP_RAW_GPS, hz: 2 },
        { cmd: MSP2_INAV_STATUS, hz: 1 },
        { cmd: MSP2_INAV_ANALOG, hz: 0.5 },
        { cmd: MSP_COMP_GPS, hz: 0.5 },
    ],
};

let schedule = [];
const nextDue = new Map();
const misses = new Map();     // cmd → consecutive unanswered requests
let replyTimeoutMs = 500;
const TICK_MS = 20;

// A silent command must not hold the single request slot for long: with one request
// in flight, every timeout is dead air for the whole telemetry set. After this many
// consecutive misses the command is dropped from the schedule — a board with no pitot
// never answers MSP2_INAV_AIR_SPEED, and retrying it forever would cost ~15% of the
// poll budget for nothing.
const MAX_MISSES = 3;

function writeRaw(buf) {
    if (!port) return;
    try {
        port.write(buf);
    } catch (e) {
        console.error('[msp] write failed:', e.message);
    }
}

function sendRequest(cmd, payload) {
    pending = {
        cmd,
        timer: setTimeout(() => {
            // A dropped reply must not wedge the scheduler forever
            pending = null;
            const n = (misses.get(cmd) || 0) + 1;
            misses.set(cmd, n);
            if (n >= MAX_MISSES) {
                const before = schedule.length;
                schedule = schedule.filter(s => s.cmd !== cmd);
                if (schedule.length !== before) {
                    console.warn(`[msp] Command 0x${cmd.toString(16)} unanswered ${n}x — removed from the poll schedule`);
                }
            }
        }, replyTimeoutMs),
    };
    writeRaw(encodeRequest(cmd, payload));
}

function failPending(cmd) {
    if (pending && pending.cmd === cmd) {
        clearTimeout(pending.timer);
        pending = null;
    }
}

function tick() {
    if (!port || pending) return;

    // One-shot requests (identification, box names) go first
    if (queue.length) {
        sendRequest(queue.shift());
        return;
    }

    const now = Date.now();
    let best = null, bestLate = 0;
    for (const entry of schedule) {
        const due = nextDue.get(entry.cmd) ?? 0;
        const late = now - due;
        if (late >= 0 && late > bestLate) { best = entry; bestLate = late; }
    }
    if (!best) return;
    nextDue.set(best.cmd, now + 1000 / best.hz);
    sendRequest(best.cmd);
}

// ── Reply decoding ───────────────────────────────────────────────────────────

function handleReply(cmd, p) {
    failPending(cmd);
    misses.set(cmd, 0);
    try {
        decode(cmd, p);
    } catch (e) {
        console.warn(`[msp] decode error for cmd ${cmd}: ${e.message}`);
    }
}

function decode(cmd, p) {
    switch (cmd) {
        case MSP_FC_VARIANT:
            vs.fcVariant = p.toString('ascii', 0, Math.min(4, p.length));
            break;

        case MSP_FC_VERSION:
            if (p.length >= 3) vs.fcVersion = `${p[0]}.${p[1]}.${p[2]}`;
            break;

        case MSP_ATTITUDE:
            // roll/pitch in decidegrees, yaw in whole degrees
            if (p.length < 6) return;
            vs.roll = p.readInt16LE(0) / 10;
            vs.pitch = p.readInt16LE(2) / 10;
            vs.yaw = p.readInt16LE(4);
            emitAttitude();
            break;

        case MSP_ALTITUDE:
            if (p.length < 6) return;
            vs.altBaro = p.readInt32LE(0) / 100;   // cm → m
            vs.climb = p.readInt16LE(4) / 100;     // cm/s → m/s
            emitVfrHud();
            break;

        case MSP_RAW_GPS:
            if (p.length < 16) return;
            vs.fixType = p[0];                     // 0 none, 1 2D, 2 3D
            vs.numSat = p[1];
            vs.lat = p.readInt32LE(2);             // already 1e7
            vs.lon = p.readInt32LE(6);
            vs.altGps = p.readInt16LE(10);         // m
            vs.groundspeed = p.readUInt16LE(12) / 100; // cm/s → m/s
            vs.gpsCourse = p.readUInt16LE(14) / 10;    // decidegrees → degrees
            if (p.length >= 18) vs.hdop = p.readUInt16LE(16);
            emitGps();
            emitGlobalPosition();
            break;

        case MSP_COMP_GPS:
            // distance to home (m), direction to home (deg), heartbeat flag
            break;

        case MSP_RAW_IMU:
            if (p.length < 18) return;
            // INAV reports acceleration in 1/512 g units by default
            vs.accX = p.readInt16LE(0) / 512;
            vs.accY = p.readInt16LE(2) / 512;
            vs.accZ = p.readInt16LE(4) / 512;
            vs.gyroX = p.readInt16LE(6);
            vs.gyroY = p.readInt16LE(8);
            vs.gyroZ = p.readInt16LE(10);
            emitImu();
            break;

        case MSP_ANALOG:
            if (p.length < 7) return;
            vs.voltage = p[0] / 10;
            vs.mahDrawn = p.readUInt16LE(1);
            vs.rssi = p.readUInt16LE(3);
            vs.current = p.readInt16LE(5) / 100;
            emitSysStatus();
            break;

        case MSP2_INAV_ANALOG:
            // flags:u8, vbat:u16 (cV), amperage:i16 (cA), power:i32, mAhDrawn:u32,
            // mWhDrawn:u32, batteryRemainingCapacity:u32, batteryPercentage:u8, rssi:u16
            if (p.length < 23) return;
            vs.voltage = p.readUInt16LE(1) / 100;
            vs.current = p.readInt16LE(3) / 100;
            vs.mahDrawn = p.readUInt32LE(9);
            vs.batteryPercent = p[21];
            if (p.length >= 24) vs.rssi = p.readUInt16LE(22);
            emitSysStatus();
            break;

        case MSP2_INAV_AIR_SPEED:
            if (p.length < 4) return;
            vs.airspeed = p.readUInt32LE(0) / 100; // cm/s → m/s
            emitVfrHud();
            break;

        case MSP_STATUS_EX:
        case MSP2_INAV_STATUS: {
            // Both start with cycleTime:u16, i2cErrors:u16, sensors:u16 ...
            // The active-box bitmap position differs; INAV status carries it as a
            // 4x u32 array at the end.
            if (p.length < 11) return;
            let boxes = 0n;
            if (cmd === MSP2_INAV_STATUS && p.length >= 25) {
                // cycleTime u16, i2cErr u16, sensors u16, cpuLoad u16, profile u8,
                // armingFlags u32, boxModeFlags 4x u32
                // INAV armingFlags: bit 2 (0x04) is ARMED; the other bits are the
                // individual "cannot arm because…" reasons.
                const armingFlags = p.readUInt32LE(9);
                vs.armed = (armingFlags & 0x04) !== 0;
                for (let i = 0; i < 4; i++) {
                    const off = 13 + i * 4;
                    if (off + 4 <= p.length) {
                        boxes |= BigInt(p.readUInt32LE(off)) << BigInt(32 * i);
                    }
                }
            } else if (p.length >= 11) {
                boxes = BigInt(p.readUInt32LE(6));
                vs.armed = (boxes & 1n) !== 0n; // box 0 = ARM on Betaflight
            }
            vs.activeBoxes = boxes;
            vs.modeName = resolveModeName(boxes);
            emitHeartbeat();
            break;
        }

        case MSP_BOXNAMES:
            vs.boxNames = p.toString('ascii').split(';').filter(Boolean);
            break;

        case MSP_BOXIDS:
            vs.boxIds = Array.from(p);
            break;

        case MSP_RC: {
            const n = Math.floor(p.length / 2);
            vs.rcChannels = [];
            for (let i = 0; i < n; i++) vs.rcChannels.push(p.readUInt16LE(i * 2));
            emitRcChannels();
            break;
        }

        default:
            break;
    }
}

/**
 * INAV/Betaflight have no flight-mode enum: the mode is the set of active "boxes".
 * Names come from MSP_BOXNAMES, which is read once at connect. The first match in
 * this priority order wins, which is how the OSD presents it too.
 */
const MODE_PRIORITY = [
    ['FAILSAFE', 'FAILSAFE'],
    ['NAV RTH', 'RTH'],
    ['NAV WP', 'WAYPOINT'],
    ['NAV CRUISE', 'CRUISE'],
    ['NAV COURSE HOLD', 'COURSE HOLD'],
    ['NAV POSHOLD', 'POSHOLD'],
    ['NAV ALTHOLD', 'ALTHOLD'],
    ['ANGLE', 'ANGLE'],
    ['HORIZON', 'HORIZON'],
    ['MANUAL', 'MANUAL'],
    ['ACRO', 'ACRO'],
];

function isBoxActive(boxes, name) {
    const idx = vs.boxNames.indexOf(name);
    if (idx < 0) return false;
    return ((boxes >> BigInt(idx)) & 1n) === 1n;
}

function resolveModeName(boxes) {
    if (!vs.boxNames.length) return vs.armed ? 'ARMED' : 'DISARMED';
    for (const [box, label] of MODE_PRIORITY) {
        if (isBoxActive(boxes, box)) return label;
    }
    return 'ACRO';
}

// ── Emission as synthetic MAVLink ────────────────────────────────────────────

function send(msgId, data) {
    if (mainWindow && !mainWindow.isDestroyed()) {
        mainWindow.webContents.send('mavlink-message', { msgId, data, sysId: 1, compId: 1 });
    }
}

const DEG2RAD = Math.PI / 180;

function emitAttitude() {
    send(30, {
        roll: vs.roll * DEG2RAD,
        pitch: vs.pitch * DEG2RAD,
        yaw: vs.yaw * DEG2RAD,
        rollspeed: 0, pitchspeed: 0, yawspeed: 0,
    });
}

function emitVfrHud() {
    send(74, {
        airspeed: vs.airspeed || vs.groundspeed,
        groundspeed: vs.groundspeed,
        heading: Math.round(vs.yaw),
        throttle: 0,
        alt: vs.altBaro,
        climb: vs.climb,
    });
}

function emitGps() {
    // MSP fix: 0 = none, 1 = 2D, 2 = 3D → MAVLink GPS_FIX_TYPE
    const fix = vs.fixType >= 2 ? 3 : vs.fixType === 1 ? 2 : 1;
    send(24, {
        fixType: fix,
        lat: vs.lat,
        lon: vs.lon,
        alt: Math.round(vs.altGps * 1000),
        eph: vs.hdop,
        satellitesVisible: vs.numSat,
        cog: Math.round(vs.gpsCourse * 100),
        vel: Math.round(vs.groundspeed * 100),
    });
}

function emitGlobalPosition() {
    // Ground track decomposed into NE velocity — the HUD's flight-path marker and
    // the trajectory predictor both read these fields.
    const courseRad = vs.gpsCourse * DEG2RAD;
    send(33, {
        lat: vs.lat,
        lon: vs.lon,
        alt: Math.round((vs.altBaro || vs.altGps) * 1000),
        relativeAlt: Math.round(vs.altBaro * 1000),
        vx: Math.round(vs.groundspeed * Math.cos(courseRad) * 100),
        vy: Math.round(vs.groundspeed * Math.sin(courseRad) * 100),
        vz: Math.round(-vs.climb * 100),
        hdg: Math.round(vs.yaw * 100),
    });
}

function emitImu() {
    send(27, {
        xacc: Math.round(vs.accX * 1000),
        yacc: Math.round(vs.accY * 1000),
        zacc: Math.round(vs.accZ * 1000),
        xgyro: vs.gyroX, ygyro: vs.gyroY, zgyro: vs.gyroZ,
    });
}

function emitSysStatus() {
    send(1, {
        voltageBattery: Math.round(vs.voltage * 1000),
        currentBattery: Math.round(vs.current * 100),
        batteryRemaining: vs.batteryPercent || -1,
        dropRateComm: 0,
    });
    send(147, {
        voltages: [Math.round(vs.voltage * 1000)],
        currentBattery: Math.round(vs.current * 100),
        currentConsumed: vs.mahDrawn,
        batteryRemaining: vs.batteryPercent || -1,
    });
}

function emitRcChannels() {
    const c = vs.rcChannels;
    const chan = {};
    for (let i = 0; i < 18; i++) chan[`chan${i + 1}Raw`] = c[i] ?? 65535;
    send(65, { chancount: c.length, rssi: Math.min(255, Math.round(vs.rssi / 4)), ...chan });
}

// Synthetic INAV mode numbers. The renderer maps custom_mode → name through the
// mode table, so the numbering only has to be stable and shared with the profile.
const INAV_MODE_NUMBERS = {
    MANUAL: 0, ACRO: 1, ANGLE: 2, HORIZON: 3, ALTHOLD: 4, POSHOLD: 5,
    'COURSE HOLD': 6, CRUISE: 7, WAYPOINT: 8, RTH: 9, FAILSAFE: 10,
    ARMED: 1, DISARMED: 0,
};

function emitHeartbeat() {
    send(0, {
        type: 1,          // MAV_TYPE_FIXED_WING — refined below once FC name is known
        autopilot: 0,     // MAV_AUTOPILOT_GENERIC
        baseMode: vs.armed ? 209 : 81,
        customMode: INAV_MODE_NUMBERS[vs.modeName] ?? 0,
        systemStatus: vs.armed ? 4 : 3,
        mavlinkVersion: 3,
    });
}

// ── Link statistics ──────────────────────────────────────────────────────────

function startStats(label) {
    stopStats();
    bytesRx = 0;
    lastStatsBytes = 0;
    statsTimer = setInterval(() => {
        const delta = bytesRx - lastStatsBytes;
        lastStatsBytes = bytesRx;
        if (mainWindow && !mainWindow.isDestroyed()) {
            mainWindow.webContents.send('mavlink-link-stats', {
                type: label,
                kbps: Math.round((delta * 8) / 100) / 10,
                bytesRx,
            });
        }
    }, 1000);
}

function stopStats() {
    if (statsTimer) { clearInterval(statsTimer); statsTimer = null; }
}

function sendConnectionState(state) {
    if (mainWindow && !mainWindow.isDestroyed()) {
        mainWindow.webContents.send('mavlink-connection-state', state);
    }
}

// ── Connection lifecycle ─────────────────────────────────────────────────────

function onData(chunk) {
    bytesRx += chunk.length;
    rxBuf = rxBuf.length ? Buffer.concat([rxBuf, chunk]) : chunk;
    parseBuffer();
}

function startSession(profile) {
    rxBuf = Buffer.alloc(0);
    pending = null;
    nextDue.clear();
    misses.clear();
    // Copy: entries get dropped from the schedule on repeated timeouts, and the
    // template must survive for the next connection.
    schedule = (SCHEDULES[profile] || SCHEDULES.normal).map(s => ({ ...s }));
    // A radio link has real latency; USB and TCP answer in milliseconds.
    replyTimeoutMs = profile === 'slow' ? 2000 : 500;
    // Identify the FC and learn the box names before the periodic polling starts —
    // without MSP_BOXNAMES the flight mode cannot be resolved at all.
    queue = [MSP_FC_VARIANT, MSP_FC_VERSION, MSP_BOXNAMES, MSP_BOXIDS];
    schedulerTimer = setInterval(tick, TICK_MS);
}

function stopSession() {
    if (schedulerTimer) { clearInterval(schedulerTimer); schedulerTimer = null; }
    if (pending) { clearTimeout(pending.timer); pending = null; }
    queue = [];
    rxBuf = Buffer.alloc(0);
}

async function connectSerial(portPath, baudRate = 115200, profile = 'normal') {
    await disconnect();
    ensureSerialLoaded();
    return new Promise((resolve, reject) => {
        const sp = new SerialPort({ path: portPath, baudRate: Number(baudRate) || 115200 }, (err) => {
            if (err) return reject(new Error(`MSP serial open failed: ${err.message}`));
            port = sp;
            portKind = 'serial';
            sp.on('data', onData);
            sp.on('error', (e) => console.error('[msp] serial error:', e.message));
            sp.on('close', () => { if (port === sp) handleUnexpectedClose(); });
            startSession(profile);
            startStats('MSP');
            sendConnectionState('CONNECTED');
            console.log(`[msp] Connected to ${portPath} at ${baudRate} baud (${profile})`);
            resolve({ success: true });
        });
    });
}

async function connectTCP(host = '127.0.0.1', tcpPort = 5760, profile = 'normal') {
    await disconnect();
    return new Promise((resolve, reject) => {
        const sock = new net.Socket();
        sock.once('error', (e) => reject(new Error(`MSP TCP connect failed: ${e.message}`)));
        sock.connect(Number(tcpPort), host, () => {
            port = sock;
            portKind = 'tcp';
            sock.on('data', onData);
            sock.on('error', (e) => console.error('[msp] tcp error:', e.message));
            sock.on('close', () => { if (port === sock) handleUnexpectedClose(); });
            startSession(profile);
            startStats('MSP/TCP');
            sendConnectionState('CONNECTED');
            console.log(`[msp] Connected to ${host}:${tcpPort} (${profile})`);
            resolve({ success: true });
        });
    });
}

function handleUnexpectedClose() {
    console.warn('[msp] Link closed by peer');
    stopSession();
    stopStats();
    port = null;
    portKind = null;
    sendConnectionState('DISCONNECTED');
}

async function disconnect() {
    stopSession();
    stopStats();
    const p = port;
    port = null;
    portKind = null;
    if (!p) return { success: true };
    await new Promise((resolve) => {
        try {
            if (typeof p.close === 'function') p.close(() => resolve());
            else if (typeof p.destroy === 'function') { p.destroy(); resolve(); }
            else resolve();
        } catch (e) { resolve(); }
    });
    sendConnectionState('DISCONNECTED');
    return { success: true };
}

function initMSPHandlers(win) {
    mainWindow = win;

    ipcMain.handle('msp-connect-serial', (e, portPath, baudRate, profile) =>
        connectSerial(portPath, baudRate, profile));
    ipcMain.handle('msp-connect-tcp', (e, host, tcpPort, profile) =>
        connectTCP(host, tcpPort, profile));
    ipcMain.handle('msp-disconnect', () => disconnect());
    ipcMain.handle('msp-status', () => ({
        connected: !!port,
        kind: portKind,
        fcVariant: vs.fcVariant,
        fcVersion: vs.fcVersion,
        boxes: vs.boxNames.length,
    }));
}

function cleanup() {
    stopSession();
    stopStats();
    if (port) {
        try {
            if (typeof port.close === 'function') port.close();
            else if (typeof port.destroy === 'function') port.destroy();
        } catch (e) { /* shutting down anyway */ }
        port = null;
    }
}

module.exports = { initMSPHandlers, cleanup };
