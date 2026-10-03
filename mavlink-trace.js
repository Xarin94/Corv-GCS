/**
 * mavlink-trace.js - What the MAVLink link did, for the debug log (app-log.js)
 *
 * main-mavlink.js feeds it every received packet and every message the GCS
 * sends. It logs the events worth reading after the fact — components
 * appearing, heartbeats lost and back, arm / mode / system-status changes,
 * STATUSTEXT, command and mission acknowledgements, parameter and mission
 * transfers, GPS fix, EKF and sensor-health changes, fence breaches, compass
 * calibration — and while connected, every SNAPSHOT_MS, one link line (rates,
 * sequence-gap loss, top messages) and one vehicle line (position, battery,
 * GPS, radio, EKF). High-rate traffic is only counted.
 */

const appLog = require('./app-log');

const SNAPSHOT_MS = 10000;
const HB_TIMEOUT_MS = 3500;
const TOP_MSGS_EVERY = 3;              // snapshots between "top messages" lines

// Sent at a rate where one line each would bury everything else: counted only.
const TX_COUNT_ONLY = new Set(['RC_CHANNELS_OVERRIDE', 'TERRAIN_DATA', 'HEARTBEAT', 'SET_POSITION_TARGET_GLOBAL_INT']);
// Sent in bursts (transfers): first line and then at most one per TX_BURST_MS.
const TX_BURST = new Set(['MISSION_ITEM_INT', 'PARAM_REQUEST_READ', 'MISSION_REQUEST_INT']);
const TX_BURST_MS = 2000;
// Asked again and again (home polling, stream setup): one line per POLLED_LOG_MS
const POLLED_CMDS = new Set([511, 512]);   // SET_MESSAGE_INTERVAL, REQUEST_MESSAGE
const POLLED_LOG_MS = 30000;

let enums = null;
function E() {
    if (enums) return enums;
    const m = require('node-mavlink');
    const reg = {};
    for (const d of ['minimal', 'common', 'standard', 'ardupilotmega', 'development']) {
        const r = m[d] && m[d].REGISTRY;
        if (r) for (const [id, c] of Object.entries(r)) if (!reg[id]) reg[id] = c.MSG_NAME;
    }
    enums = { m, reg };
    return enums;
}
const nm = (en, v) => (en && en[v] !== undefined ? en[v] : v);
function msgName(id) { return E().reg[id] || `MSG_${id}`; }
function cmdName(id) {
    const { m } = E();
    const n = (m.ardupilotmega.MavCmd && m.ardupilotmega.MavCmd[id]) || m.common.MavCmd[id];
    return n ? `${n}(${id})` : `CMD_${id}`;
}
function resultName(r) { return nm(E().m.common.MavResult, r); }

/** Names of the set bits of a flag enum ("GPS, AHRS"). */
function bitNames(en, mask) {
    const out = [];
    for (let b = 0; b < 32; b++) {
        const v = 2 ** b;
        if (mask & v) out.push(en && en[v] !== undefined ? String(en[v]).replace(/^SENSOR_|^EKF_/, '') : `bit${b}`);
    }
    return out.join(', ');
}

const EKF_BAD_BITS = 1024 | 32768;   // UNINITIALIZED, GPS_GLITCHING
const fix = (v, d = 1) => (Number.isFinite(v) ? v.toFixed(d) : '?');

// ── State ────────────────────────────────────────────────────────────────────

let link = null;        // { desc, since, getBytes }
let snapTimer = null;
let snapCount = 0;
let lastSnapAt = 0, lastSnapBytes = 0;

let rxPackets = 0, rxPacketsSnap = 0;
const rxById = new Map();         // msgId → count since the last top-messages line
const unknownIds = new Set();
const seq = new Map();            // "sys/comp" → { last, lost, seen }
let txCount = 0, txCountSnap = 0;
const txByName = new Map();
const txBurstSkipped = new Map();
const txBurstLast = new Map();

const comps = new Map();          // "sys/comp" → { type, autopilot, baseMode, customMode, systemStatus, lastHb, lost }
let vehicle = null;               // "sys/comp" of the first autopilot heard
const v = {};                     // last values for the vehicle line and change detection

const paramPending = new Map();    // paramId → { t, kind } of a PARAM_SET / PARAM_REQUEST_READ
let paramBulk = null;              // after PARAM_REQUEST_LIST: { count, seen:Set, start, lastAt, done }
const missionXfer = { reqs: 0, items: 0, lastLog: 0 };

function reset() {
    rxPackets = rxPacketsSnap = txCount = txCountSnap = 0;
    rxById.clear(); unknownIds.clear(); seq.clear(); txByName.clear();
    txBurstSkipped.clear(); txBurstLast.clear(); comps.clear();
    paramPending.clear(); paramBulk = null;
    missionXfer.reqs = missionXfer.items = 0;
    vehicle = null;
    for (const k of Object.keys(v)) delete v[k];
    snapCount = 0;
}

// ── Link lifecycle ───────────────────────────────────────────────────────────

/**
 * @param {string} desc  e.g. "MAVLink serial COM3 @ 57600"
 * @param {() => number} getBytes  received byte counter of the link
 */
function onConnect(desc, getBytes) {
    reset();
    link = { desc, since: Date.now(), getBytes };
    lastSnapAt = Date.now();
    lastSnapBytes = getBytes ? getBytes() : 0;
    appLog.setHeader('Connection', `${desc} since ${new Date().toLocaleTimeString()}`);
    if (snapTimer) clearInterval(snapTimer);
    snapTimer = setInterval(snapshot, SNAPSHOT_MS);
    snapTimer.unref();
}

function onDisconnect(reason) {
    if (!link) return;
    const up = ((Date.now() - link.since) / 1000).toFixed(0);
    console.log(`[mav] link closed (${reason || 'disconnect'}) after ${up} s: ${rxPackets} packets in, ${txCount} out${vehicle ? `, vehicle ${vehicle}` : ', no vehicle heard'}`);
    appLog.setHeader('Connection', `none (last: ${link.desc}, closed ${new Date().toLocaleTimeString()}, ${reason || 'disconnect'})`);
    if (snapTimer) { clearInterval(snapTimer); snapTimer = null; }
    link = null;
}

// ── Receive ──────────────────────────────────────────────────────────────────

/** Every packet, decoded or not: counts and sequence-gap loss. */
function onPacket(header) {
    rxPackets++;
    const id = header.msgid;
    rxById.set(id, (rxById.get(id) || 0) + 1);
    const key = `${header.sysid}/${header.compid}`;
    let s = seq.get(key);
    if (!s) { s = { last: header.seq, lost: 0, seen: 1 }; seq.set(key, s); return; }
    const gap = (header.seq - s.last - 1) & 0xFF;
    if (gap > 0 && gap < 128) s.lost += gap;  // larger gaps are reordering or a reboot, not loss
    s.last = header.seq;
    s.seen++;
}

function onUndecoded(msgId) {
    if (unknownIds.has(msgId)) return;
    unknownIds.add(msgId);
    console.log(`[mav] message id ${msgId} (${msgName(msgId)}) not decoded by this GCS`);
}

function onMessage(msgId, d, sysId, compId) {
    const key = `${sysId}/${compId}`;
    const { m } = E();
    switch (msgId) {
        case 0: heartbeat(key, d); break;
        case 253: { // STATUSTEXT
            const sev = d.severity;
            const text = `[mav] STATUSTEXT ${key} ${nm(m.common.MavSeverity, sev)}: ${String(d.text || '').replace(/\0.*$/, '')}`;
            if (sev <= 3) console.error(text); else if (sev === 4) console.warn(text); else console.log(text);
            break;
        }
        case 77: { // COMMAND_ACK
            const ok = d.result === 0 || d.result === 5;
            if (ok && POLLED_CMDS.has(d.command) && appLog.throttle(`ack-${d.command}`, POLLED_LOG_MS) < 0) break;
            const text = `[mav] COMMAND_ACK ${key} ${cmdName(d.command)} → ${resultName(d.result)}${d.progress ? ` progress=${d.progress}` : ''}${d.resultParam2 ? ` param2=${d.resultParam2}` : ''}`;
            if (ok) console.log(text); else console.warn(text);
            break;
        }
        case 76: // COMMAND_LONG from the vehicle (accel-cal position requests…)
            console.log(`[mav] COMMAND_LONG from ${key}: ${cmdName(d.command)} p1=${d._param1} p2=${d._param2}`);
            break;
        case 47: { // MISSION_ACK
            const text = `[mav] MISSION_ACK ${key} ${nm(m.common.MavMissionResult, d.type)} (${nm(m.common.MavMissionType, d.missionType)}) after ${missionXfer.reqs} item requests, ${missionXfer.items} items received`;
            if (d.type === 0) console.log(text); else console.warn(text);
            missionXfer.reqs = missionXfer.items = 0;
            break;
        }
        case 44: // MISSION_COUNT
            console.log(`[mav] MISSION_COUNT ${key} ${d.count} items (${nm(m.common.MavMissionType, d.missionType)})`);
            missionXfer.items = 0;
            break;
        case 40: case 51: // MISSION_REQUEST(_INT)
            missionXfer.reqs++;
            missionProgress(`vehicle requests item ${d.seq}`);
            break;
        case 39: case 73: // MISSION_ITEM(_INT) received (download)
            missionXfer.items++;
            missionProgress(`received item ${d.seq}`);
            break;
        case 42: // MISSION_CURRENT
            if (v.missionSeq !== d.seq) {
                console.log(`[mav] mission current item ${d.seq}${d.total ? `/${d.total}` : ''}`);
                v.missionSeq = d.seq;
            }
            break;
        case 46: // MISSION_ITEM_REACHED
            console.log(`[mav] mission item ${d.seq} reached`);
            break;
        case 22: paramValue(d); break;
        case 242: { // HOME_POSITION
            const h = `${(d.latitude / 1e7).toFixed(7)},${(d.longitude / 1e7).toFixed(7)} ${(d.altitude / 1000).toFixed(1)} m`;
            if (v.home !== h) { console.log(`[mav] home ${h}`); v.home = h; }
            break;
        }
        case 24: { // GPS_RAW_INT
            if (v.gpsFix !== d.fixType) {
                const now = nm(m.common.GpsFixType, d.fixType);
                const text = v.gpsFix === undefined ? `[mav] GPS fix ${now} (${d.satellitesVisible} sats)`
                    : `[mav] GPS fix ${nm(m.common.GpsFixType, v.gpsFix)} → ${now} (${d.satellitesVisible} sats)`;
                if (v.gpsFix !== undefined && d.fixType < 3 && v.gpsFix >= 3) console.warn(text); else console.log(text);
                v.gpsFix = d.fixType;
            }
            v.sats = d.satellitesVisible; v.eph = d.eph;
            break;
        }
        case 1: sysStatus(d); break;
        case 193: { // EKF_STATUS_REPORT
            if (v.ekf !== d.flags) {
                const was = v.ekf || 0;
                const lostBits = was & ~d.flags, gained = d.flags & ~was;
                const en = m.ardupilotmega.EkfStatusFlags;
                const parts = [];
                if (gained) parts.push(`+ ${bitNames(en, gained)}`);
                if (lostBits) parts.push(`- ${bitNames(en, lostBits)}`);
                const text = `[mav] EKF flags 0x${(d.flags >>> 0).toString(16)} ${parts.join(' ')}`;
                // Losing a solution, or gaining UNINITIALIZED / GPS_GLITCHING, is bad news
                const bad = (lostBits & ~EKF_BAD_BITS) || (gained & EKF_BAD_BITS);
                if (v.ekf !== undefined && bad) console.warn(text); else console.log(text);
                v.ekf = d.flags;
            }
            v.ekfVar = `vel ${fix(d.velocityVariance, 2)} pos ${fix(d.posHorizVariance, 2)} hgt ${fix(d.posVertVariance, 2)} mag ${fix(d.compassVariance, 2)}`;
            break;
        }
        case 109: // RADIO_STATUS
            v.radio = `rssi ${d.rssi}/${d.remrssi} noise ${d.noise}/${d.remnoise} rxerr ${d.rxerrors} fixed ${d.fixed} txbuf ${d.txbuf}%`;
            break;
        case 33: // GLOBAL_POSITION_INT
            v.pos = `${(d.lat / 1e7).toFixed(6)},${(d.lon / 1e7).toFixed(6)} rel ${(d.relativeAlt / 1000).toFixed(1)} m amsl ${(d.alt / 1000).toFixed(1)} m hdg ${(d.hdg / 100).toFixed(0)}`;
            break;
        case 74: // VFR_HUD
            v.hud = `gs ${fix(d.groundspeed)} as ${fix(d.airspeed)} m/s climb ${fix(d.climb)} thr ${d.throttle}%`;
            break;
        case 241: { // VIBRATION
            const clip = (d.clipping0 || 0) + (d.clipping1 || 0) + (d.clipping2 || 0);
            if (v.clip !== undefined && clip > v.clip && appLog.throttle('vib-clip', 10000) >= 0) {
                console.warn(`[mav] accelerometer clipping +${clip - v.clip} (vibration ${fix(d.vibrationX)}/${fix(d.vibrationY)}/${fix(d.vibrationZ)})`);
            }
            v.clip = clip;
            v.vib = `${fix(d.vibrationX)}/${fix(d.vibrationY)}/${fix(d.vibrationZ)}`;
            break;
        }
        case 162: // FENCE_STATUS
            if (v.breach !== d.breachStatus) {
                const text = `[mav] fence ${d.breachStatus ? `BREACHED (${nm(m.common.FenceBreach, d.breachType)}, count ${d.breachCount})` : 'clear'}`;
                if (d.breachStatus) console.warn(text); else console.log(text);
                v.breach = d.breachStatus;
            }
            break;
        case 136: // TERRAIN_REPORT
            v.terrain = `pending ${d.pending} loaded ${d.loaded}`;
            break;
        case 191: // MAG_CAL_PROGRESS
            if (appLog.throttle(`magcal-${d.compassId}`, 3000) >= 0) {
                console.log(`[mav] compass ${d.compassId} calibration ${d.completionPct}% attempt ${d.attempt} (${nm(m.ardupilotmega.MagCalStatus || m.common.MagCalStatus, d.calStatus)})`);
            }
            break;
        case 192: { // MAG_CAL_REPORT
            const status = nm(m.ardupilotmega.MagCalStatus || m.common.MagCalStatus, d.calStatus);
            const text = `[mav] compass ${d.compassId} calibration ${status} fitness ${fix(d.fitness, 2)} offsets ${fix(d.ofsX, 0)}/${fix(d.ofsY, 0)}/${fix(d.ofsZ, 0)} autosaved ${d.autosaved}`;
            if (/SUCCESS/.test(status)) console.log(text); else console.warn(text);
            break;
        }
        default: break;
    }
}

function heartbeat(key, d) {
    const { m } = E();
    let c = comps.get(key);
    const now = Date.now();
    if (!c) {
        c = { lastHb: now, lost: false };
        comps.set(key, c);
        console.log(`[mav] new component ${key}: ${nm(m.minimal.MavType, d.type)} autopilot ${nm(m.minimal.MavAutopilot, d.autopilot)} mavlink v${d.mavlinkVersion}`);
        // The vehicle: the first component with a real autopilot
        if (!vehicle && d.autopilot !== 8 && d.type !== 6) {
            vehicle = key;
            appLog.setHeader('Vehicle', `${key} ${nm(m.minimal.MavType, d.type)} ${nm(m.minimal.MavAutopilot, d.autopilot)}`);
        }
    } else if (c.lost) {
        console.warn(`[mav] heartbeat from ${key} back after ${((now - c.lastHb) / 1000).toFixed(1)} s`);
        c.lost = false;
    }
    c.lastHb = now;
    if (key !== vehicle) return;

    const armed = !!(d.baseMode & 128);
    if (c.armed === undefined) {
        console.log(`[mav] vehicle ${key}: custom mode ${d.customMode}, ${armed ? 'ARMED' : 'disarmed'}, ${nm(m.minimal.MavState, d.systemStatus)}`);
        c.systemStatus = d.systemStatus;
    }
    if (c.armed !== undefined && c.armed !== armed) console.warn(`[mav] vehicle ${armed ? 'ARMED' : 'DISARMED'}`);
    c.armed = armed;
    if (c.customMode !== undefined && c.customMode !== d.customMode) console.log(`[mav] flight mode ${c.customMode} → ${d.customMode}`);
    c.customMode = d.customMode;
    if (c.systemStatus !== d.systemStatus) {
        const text = `[mav] system status ${nm(m.minimal.MavState, c.systemStatus)} → ${nm(m.minimal.MavState, d.systemStatus)}`;
        if (d.systemStatus >= 5) console.warn(text); else console.log(text); // CRITICAL / EMERGENCY / POWEROFF …
        c.systemStatus = d.systemStatus;
    }
    c.type = d.type;
}

/** Heartbeat watchdog, called by the snapshot timer and its own 1 s tick. */
function checkHeartbeats() {
    const now = Date.now();
    for (const [key, c] of comps) {
        if (!c.lost && now - c.lastHb > HB_TIMEOUT_MS) {
            c.lost = true;
            const text = `[mav] heartbeat lost from ${key} (${((now - c.lastHb) / 1000).toFixed(1)} s without one)`;
            if (key === vehicle) console.error(text); else console.warn(text);
        }
    }
}
setInterval(() => { if (link) { checkHeartbeats(); checkParamBulk(); } }, 1000).unref();

function sysStatus(d) {
    const { m } = E();
    const en = m.common.MavSysStatusSensor;
    // Present and enabled but not healthy: what the autopilot itself flags
    const bad = (d.onboardControlSensorsPresent & d.onboardControlSensorsEnabled & ~d.onboardControlSensorsHealth) >>> 0;
    if (v.badSensors !== bad) {
        if (bad) console.warn(`[mav] unhealthy sensors: ${bitNames(en, bad)}`);
        else if (v.badSensors !== undefined) console.log('[mav] all enabled sensors healthy');
        v.badSensors = bad;
    }
    v.bat = `${(d.voltageBattery / 1000).toFixed(2)} V ${d.currentBattery >= 0 ? (d.currentBattery / 100).toFixed(1) : '?'} A ${d.batteryRemaining}%`;
    v.load = d.load / 10;
    v.commDrop = d.dropRateComm / 100;
    if (d.batteryRemaining >= 0 && d.batteryRemaining <= 20 && appLog.throttle('bat-low', 60000) >= 0) {
        console.warn(`[mav] battery low: ${v.bat}`);
    }
}

function paramValue(d) {
    const id = String(d.paramId || '').replace(/ .*$/, '');
    const now = Date.now();
    const p = paramPending.get(id);
    if (p) {
        paramPending.delete(id);
        console.log(`[mav] PARAM_VALUE ${id} = ${d.paramValue} (answer to ${p.kind} after ${now - p.t} ms)`);
        return;
    }
    if (paramBulk && !paramBulk.done) {
        if (paramBulk.count === null) console.log(`[mav] parameters: first answer after ${now - paramBulk.start} ms, vehicle has ${d.paramCount}`);
        paramBulk.count = d.paramCount;
        paramBulk.lastAt = now;
        paramBulk.seen.add(d.paramIndex);
        const n = paramBulk.seen.size;
        if (n >= paramBulk.count) {
            paramBulk.done = true;
            console.log(`[mav] parameters: all ${n} received in ${((now - paramBulk.start) / 1000).toFixed(1)} s`);
        } else if (n % 200 === 0) {
            console.log(`[mav] parameters: ${n}/${paramBulk.count}`);
        }
        return;
    }
    // Unsolicited: a change made by another GCS, or a late answer
    const skipped = appLog.throttle('param-unsolicited', 2000);
    if (skipped >= 0) console.log(`[mav] PARAM_VALUE ${id} = ${d.paramValue} (#${d.paramIndex}/${d.paramCount})${skipped ? `  (+${skipped} more)` : ''}`);
}

/** A parameter download that stopped short (lost packets, link gone). */
function checkParamBulk() {
    if (!paramBulk || paramBulk.done || paramBulk.stalled) return;
    const idle = Date.now() - (paramBulk.lastAt || paramBulk.start);
    if (idle > 5000) {
        paramBulk.stalled = true;
        console.warn(`[mav] parameters: download stalled at ${paramBulk.seen.size}/${paramBulk.count ?? '?'} (${(idle / 1000).toFixed(0)} s without a PARAM_VALUE)`);
    }
}

function missionProgress(what) {
    const now = Date.now();
    if (now - missionXfer.lastLog < 1000) return;
    missionXfer.lastLog = now;
    console.log(`[mav] mission transfer: ${what} (${missionXfer.reqs} requests, ${missionXfer.items} items so far)`);
}

// ── Send ─────────────────────────────────────────────────────────────────────

/** A message the GCS is about to send. `name` is the MAVLink message name. */
function onSend(name, msg) {
    txCount++;
    txByName.set(name, (txByName.get(name) || 0) + 1);
    if (TX_COUNT_ONLY.has(name)) return;
    if (name === 'COMMAND_LONG' && POLLED_CMDS.has(msg.command)) {
        const skipped = appLog.throttle(`tx-${msg.command}-${msg.param1}`, POLLED_LOG_MS);
        if (skipped < 0) return;
        console.log(`[tx] ${name} ${describeTx(name, msg)}${skipped ? `  (+${skipped} more since the last line)` : ''}`);
        return;
    }
    if ((name === 'PARAM_SET' || name === 'PARAM_REQUEST_READ') && msg.paramId) {
        paramPending.set(String(msg.paramId), { t: Date.now(), kind: name });
    }
    if (name === 'PARAM_REQUEST_LIST') paramBulk = { count: null, seen: new Set(), start: Date.now(), lastAt: 0, done: false };
    if (TX_BURST.has(name)) {
        const now = Date.now();
        if (now - (txBurstLast.get(name) || 0) < TX_BURST_MS) {
            txBurstSkipped.set(name, (txBurstSkipped.get(name) || 0) + 1);
            return;
        }
        txBurstLast.set(name, now);
        const skipped = txBurstSkipped.get(name) || 0;
        txBurstSkipped.set(name, 0);
        console.log(`[tx] ${name} ${describeTx(name, msg)}${skipped ? `  (+${skipped} more since the last line)` : ''}`);
        return;
    }
    console.log(`[tx] ${name} ${describeTx(name, msg)}`);
}

function describeTx(name, msg) {
    const tgt = `→ ${msg.targetSystem || 1}/${msg.targetComponent || 1}`;
    switch (name) {
        case 'COMMAND_LONG':
            return `${cmdName(msg.command)} ${[1, 2, 3, 4, 5, 6, 7].map(i => `p${i}=${+Number(msg[`param${i}`] || 0).toFixed(4)}`).join(' ')} ${tgt}`;
        case 'COMMAND_INT':
            return `${cmdName(msg.command)} frame=${msg.frame || 0} p1=${msg.param1 || 0} p2=${msg.param2 || 0} x=${msg.x || 0} y=${msg.y || 0} z=${msg.z || 0} ${tgt}`;
        case 'SET_MODE':
            return `custom_mode=${msg.customMode} base_mode=${msg.baseMode} ${tgt}`;
        case 'PARAM_SET':
            return `${msg.paramId} = ${msg.paramValue} (type ${msg.paramType || 9}) ${tgt}`;
        case 'PARAM_REQUEST_READ':
            return `${msg.paramId || `#${msg.paramIndex}`} ${tgt}`;
        case 'MISSION_COUNT':
            return `${msg.count} items type ${msg.missionType || 0} ${tgt}`;
        case 'MISSION_ITEM_INT':
            return `#${msg.seq} ${cmdName(msg.command || 16)} frame ${msg.frame ?? 3} x=${msg.x} y=${msg.y} z=${msg.z} ${tgt}`;
        case 'MISSION_REQUEST_INT':
            return `#${msg.seq} type ${msg.missionType || 0} ${tgt}`;
        case 'MISSION_ACK':
            return `type ${msg.ackType || 0} mission ${msg.missionType || 0} ${tgt}`;
        case 'REQUEST_DATA_STREAM':
            return `stream ${msg.reqStreamId || 0} ${msg.reqMessageRate || 10} Hz start ${msg.startStop !== undefined ? msg.startStop : 1} ${tgt}`;
        default:
            return tgt;
    }
}

// ── Snapshot ─────────────────────────────────────────────────────────────────

function snapshot() {
    if (!link) return;
    checkHeartbeats();
    const now = Date.now();
    const dt = Math.max(0.001, (now - lastSnapAt) / 1000);
    const b = link.getBytes ? link.getBytes() : 0;
    const kbps = ((b - lastSnapBytes) * 8 / 1000 / dt).toFixed(1);
    const rxRate = ((rxPackets - rxPacketsSnap) / dt).toFixed(0);
    const txRate = ((txCount - txCountSnap) / dt).toFixed(1);
    lastSnapAt = now; lastSnapBytes = b; rxPacketsSnap = rxPackets; txCountSnap = txCount;

    let lost = 0, seen = 0;
    for (const s of seq.values()) { lost += s.lost; seen += s.seen; }
    const lossPct = seen + lost ? (100 * lost / (seen + lost)).toFixed(1) : '0.0';
    const c = vehicle ? comps.get(vehicle) : null;
    const hbAge = c ? ((now - c.lastHb) / 1000).toFixed(1) : '-';
    const line = `${link.desc} | rx ${kbps} kbps ${rxRate} msg/s | lost ${lossPct}% (${lost}/${seen + lost} since connect) | tx ${txRate} msg/s | hb ${hbAge} s ago | components ${[...comps.keys()].join(' ') || 'none'}`;
    console.log(`[link] ${line}`);

    if (c) {
        const { m } = E();
        const parts = [
            `${vehicle} ${nm(m.minimal.MavType, c.type)} mode ${c.customMode} ${c.armed ? 'ARMED' : 'disarmed'} ${nm(m.minimal.MavState, c.systemStatus)}`,
            v.pos ? `pos ${v.pos}` : 'no position',
            v.hud || '',
            v.bat ? `bat ${v.bat}` : '',
            v.gpsFix !== undefined ? `gps ${nm(m.common.GpsFixType, v.gpsFix)} ${v.sats} sats hdop ${(v.eph / 100).toFixed(1)}` : 'no gps',
            v.ekf !== undefined ? `ekf 0x${(v.ekf >>> 0).toString(16)} (${v.ekfVar})` : '',
            v.radio ? `radio ${v.radio}` : '',
            v.vib ? `vib ${v.vib} clip ${v.clip}` : '',
            v.load !== undefined ? `fc load ${v.load.toFixed(0)}% comm drop ${v.commDrop}%` : '',
            v.terrain ? `terrain ${v.terrain}` : ''
        ].filter(Boolean);
        console.log(`[vehicle] ${parts.join(' | ')}`);
        appLog.setHeader('Vehicle', `${parts.join(' | ')}  (at ${new Date().toLocaleTimeString()})`);
    }
    appLog.setHeader('Connection', `${line}  (since ${new Date(link.since).toLocaleTimeString()})`);

    if (++snapCount % TOP_MSGS_EVERY === 0) {
        const span = SNAPSHOT_MS * TOP_MSGS_EVERY / 1000;
        const top = [...rxById.entries()].sort((a, b2) => b2[1] - a[1]).slice(0, 12)
            .map(([id, n]) => `${msgName(id)} ${(n / span).toFixed(1)}`).join(', ');
        const tx = [...txByName.entries()].map(([n, k]) => `${n} ${k}`).join(', ');
        console.log(`[link] rx msg/s: ${top || 'none'}`);
        console.log(`[link] tx since connect: ${tx || 'none'}`);
        rxById.clear();
    }
}

module.exports = { onConnect, onDisconnect, onPacket, onUndecoded, onMessage, onSend };
