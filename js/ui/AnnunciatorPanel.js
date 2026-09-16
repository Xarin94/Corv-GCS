/**
 * AnnunciatorPanel.js - Vehicle health annunciators (CAS strip)
 *
 * Grades the same conditions Mission Planner flags on its HUD and shows them
 * as flashing glyphs in the column inboard of the airspeed tape:
 *
 *   WARNING (red, from the top)   — faults and interference that threaten
 *                                   the flight: failsafe, EKF, vibration,
 *                                   IMU/compass/baro/GPS health, GPS
 *                                   jamming/spoofing/glitch, RC loss, motor,
 *                                   battery, fence, link loss.
 *   CAUTION (amber, from the bottom, above the mini-map) — degraded but not
 *                                   yet dangerous: calibrations in progress,
 *                                   pre-arm failing, HDOP, 2D fix, terrain,
 *                                   logging, LiDAR, optical flow, proximity,
 *                                   weak link, low battery.
 *
 * Sources: SYS_STATUS sensor bits, HEARTBEAT system_status, EKF_STATUS_REPORT,
 * VIBRATION, GPS_RAW_INT, GNSS_INTEGRITY, MAG_CAL_*, RADIO_STATUS — all read
 * from STATE — plus STATUSTEXT pattern matches, which hold for a few seconds
 * because they are events, not states.
 *
 * In the demo flight (LIVE with no link) a random annunciator lights for a
 * few seconds at a time so the strip is visible in the preview.
 */

import { STATE, isDemoMode } from '../core/state.js';
import { onMessage } from '../mavlink/MAVLinkManager.js';

// ── Catalog ───────────────────────────────────────────────────────────────
// Order is display order within each list, so a fault always appears in the
// same slot relative to its neighbours instead of jumping around. `val`
// returns the quality readout printed under the label (live data); `demoVal`
// is what the preview shows instead, since there is no telemetry to read.
const WARN = 'warn', CAUT = 'caut';

// RADIO_STATUS RSSI when the radio reports it, else the SYS_STATUS-derived
// link quality — the two links that exist on a bare UDP/TCP SITL link.
function linkVal() {
    if (STATE.rssi !== null) return `RSSI ${STATE.rssi}`;
    return `LQ ${Math.round(STATE.linkQuality)}%`;
}
const gpsVal  = () => `${STATE.gpsNumSat} SAT\nHDOP ${STATE.gpsHdop.toFixed(1)}`;
const hdopVal = () => `HDOP ${STATE.gpsHdop.toFixed(1)}`;

const ALERTS = [
    { id: 'failsafe',  lvl: WARN, icon: 'failsafe', label: 'FAILSAFE', title: 'Autopilot in failsafe (system status CRITICAL / EMERGENCY)' },
    { id: 'linkLost',  lvl: WARN, icon: 'link',     label: 'LINK',     title: 'No heartbeat for 3 s', val: linkVal, demoVal: 'RSSI --' },
    { id: 'rc',        lvl: WARN, icon: 'rc',       label: 'RC',       title: 'RC receiver unhealthy / radio failsafe' },
    { id: 'ekf',       lvl: WARN, icon: 'ekf',      label: 'EKF',      title: 'EKF variance > 0.8' },
    { id: 'gpsGlitch', lvl: WARN, icon: 'glitch',   label: 'GPS GLITCH', title: 'EKF reports GPS glitching', val: gpsVal, demoVal: '7 SAT\nHDOP 2.8' },
    { id: 'gpsJam',    lvl: WARN, icon: 'jam',      label: 'GPS JAM',  title: 'GNSS receiver reports signal jamming', val: gpsVal, demoVal: '4 SAT\nHDOP 4.6' },
    { id: 'gpsSpoof',  lvl: WARN, icon: 'spoof',    label: 'SPOOF',    title: 'GNSS receiver reports signal spoofing', val: gpsVal, demoVal: '12 SAT\nHDOP 0.9' },
    { id: 'gps',       lvl: WARN, icon: 'gps',      label: 'GPS',      title: 'GPS unhealthy or no fix', val: gpsVal, demoVal: '3 SAT\nHDOP 9.9' },
    { id: 'vibe',      lvl: WARN, icon: 'vibe',     label: 'VIBE',     title: 'Vibration > 60 or accelerometer clipping' },
    { id: 'imu',       lvl: WARN, icon: 'imu',      label: 'IMU',      title: 'Gyro / accelerometer unhealthy' },
    { id: 'mag',       lvl: WARN, icon: 'mag',      label: 'COMPASS',  title: 'Compass unhealthy' },
    { id: 'baro',      lvl: WARN, icon: 'baro',     label: 'BARO',     title: 'Barometer unhealthy' },
    { id: 'ahrs',      lvl: WARN, icon: 'ahrs',     label: 'AHRS',     title: 'Attitude estimate unhealthy' },
    { id: 'aspd',      lvl: WARN, icon: 'aspd',     label: 'AIRSPD',   title: 'Airspeed sensor unhealthy' },
    { id: 'motor',     lvl: WARN, icon: 'motor',    label: 'MOTOR',    title: 'Motor output unhealthy' },
    { id: 'batt',      lvl: WARN, icon: 'batt',     label: 'BATT',     title: 'Battery unhealthy / critical / failsafe' },
    { id: 'fence',     lvl: WARN, icon: 'fence',    label: 'FENCE',    title: 'Geofence breach' },

    { id: 'ekfVar',    lvl: CAUT, icon: 'ekf',      label: 'EKF',      title: 'EKF variance > 0.5' },
    { id: 'vibeHi',    lvl: CAUT, icon: 'vibe',     label: 'VIBE',     title: 'Vibration > 30' },
    { id: 'gps2d',     lvl: CAUT, icon: 'gps',      label: '2D FIX',   title: 'GPS 2D fix only', val: gpsVal, demoVal: '5 SAT\nHDOP 3.1' },
    { id: 'hdop',      lvl: CAUT, icon: 'gps',      label: 'HDOP',     title: 'GPS HDOP > 2.0', val: hdopVal, demoVal: 'HDOP 2.6' },
    { id: 'prearm',    lvl: CAUT, icon: 'prearm',   label: 'PREARM',   title: 'Pre-arm checks failing' },
    { id: 'magCal',    lvl: CAUT, icon: 'cal',      label: 'MAG CAL',  title: 'Compass calibration in progress' },
    { id: 'imuCal',    lvl: CAUT, icon: 'cal',      label: 'IMU CAL',  title: 'Accelerometer / gyro calibration in progress' },
    { id: 'baroCal',   lvl: CAUT, icon: 'cal',      label: 'BARO CAL', title: 'Barometer calibration in progress' },
    { id: 'battLow',   lvl: CAUT, icon: 'batt',     label: 'BATT LOW', title: 'Battery low' },
    { id: 'linkWeak',  lvl: CAUT, icon: 'link',     label: 'LINK',     title: 'Telemetry link quality < 50% or RSSI low', val: linkVal, demoVal: 'RSSI 41' },
    { id: 'terrain',   lvl: CAUT, icon: 'terrain',  label: 'TERRAIN',  title: 'Terrain data unhealthy / missing' },
    { id: 'logging',   lvl: CAUT, icon: 'log',      label: 'LOG',      title: 'Onboard logging unhealthy' },
    { id: 'lidar',     lvl: CAUT, icon: 'lidar',    label: 'LIDAR',    title: 'Rangefinder unhealthy' },
    { id: 'flow',      lvl: CAUT, icon: 'flow',     label: 'OPT FLOW', title: 'Optical flow unhealthy' },
    { id: 'prox',      lvl: CAUT, icon: 'prox',     label: 'PROX',     title: 'Proximity sensor unhealthy' },
];

// MAV_SYS_STATUS_SENSOR bits
const S = {
    GYRO: 1, ACCEL: 2, MAG: 4, BARO: 8, DPRESS: 16, GPS: 32, FLOW: 64,
    LASER: 256, MOTOR: 32768, RC: 65536, GYRO2: 1 << 17, ACCEL2: 1 << 18,
    MAG2: 1 << 19, FENCE: 1 << 20, AHRS: 1 << 21, TERRAIN: 1 << 22,
    LOGGING: 1 << 24, BATTERY: 1 << 25, PROX: 1 << 26, PREARM: 1 << 28,
};
const EKF_GPS_GLITCHING = 1 << 15;

// Thresholds (Mission Planner's HUD rules)
const EKF_WARN = 0.8, EKF_CAUT = 0.5;
const VIBE_WARN = 60, VIBE_CAUT = 30;
const HDOP_CAUT = 2.0;
const BATT_LOW_PCT = 20;
const LINK_WEAK_PCT = 50, RSSI_WEAK = 60;    // RADIO_STATUS rssi is 0-254
const STALE_MS = 5000;                       // sensor data older than this is ignored
const HEARTBEAT_LOST_MS = 3000;
const CLIP_HOLD_MS = 10000;                  // a clipping event stays red this long
const TEXT_HOLD_MS = 10000;                  // a STATUSTEXT match holds this long

// STATUSTEXT → alert. First match wins; "cleared" texts are excluded up
// front so "Fence breach cleared" doesn't relight the fence warning.
const TEXT_RULES = [
    { re: /jam/i,                                        id: 'gpsJam' },
    { re: /spoof/i,                                      id: 'gpsSpoof' },
    { re: /gps glitch/i,                                 id: 'gpsGlitch' },
    { re: /radio failsafe|rc failsafe|throttle failsafe/i, id: 'rc' },
    { re: /battery.*(critical|failsafe)|crit batt/i,     id: 'batt' },
    { re: /battery.*low|low battery/i,                   id: 'battLow' },
    { re: /ekf.*failsafe|ekf variance/i,                 id: 'ekf' },
    { re: /fence breach|fence.*fail/i,                   id: 'fence' },
    { re: /failsafe/i,                                   id: 'failsafe' },
    { re: /prearm/i,                                     id: 'prearm' },
    { re: /calibrating (accel|gyro)|accel.*cal|gyro.*cal|place vehicle/i, id: 'imuCal' },
    { re: /calibrating baro|baro.*cal/i,                 id: 'baroCal' },
    { re: /compass.*cal|mag.*cal/i,                      id: 'magCal' },
    { re: /terrain.*(missing|unavailable|fail|disabled)/i, id: 'terrain' },
    { re: /log(ging)? fail|no logging|bad logging/i,     id: 'logging' },
];
const TEXT_CLEARED = /cleared|complete|success|ok\b|passed|recovered/i;

// ── State ─────────────────────────────────────────────────────────────────
let elWarn = null, elCaut = null;
const timed = new Map();          // alert id → expiry timestamp (STATUSTEXT holds)
let lastClipSum = -1, lastClipChange = 0;
let lastKey = '';                 // rendered signature, to skip no-op DOM work
const valEls = new Map();         // alert id → its .annun-val span, for live readouts

// Demo sequencer
const DEMO_ON_MS = 3200, DEMO_OFF_MS = 1000;
let demoId = null, demoUntil = 0, demoNextAt = 0;

export function initAnnunciatorPanel() {
    elWarn = document.getElementById('annun-warnings');
    elCaut = document.getElementById('annun-cautions');
    if (!elWarn || !elCaut) return;

    onMessage(253, (data) => {
        const text = data.text || '';
        if (!text || TEXT_CLEARED.test(text)) return;
        for (const rule of TEXT_RULES) {
            if (rule.re.test(text)) {
                timed.set(rule.id, Date.now() + TEXT_HOLD_MS);
                return;
            }
        }
    });
}

// ── Evaluation ────────────────────────────────────────────────────────────

function badSensor(bit) {
    return (STATE.sensorsPresent & bit) !== 0 &&
           (STATE.sensorsEnabled & bit) !== 0 &&
           (STATE.sensorsHealth & bit) === 0;
}

/**
 * Compute the set of active alert ids from STATE. Warnings suppress their
 * caution twin (ekf/ekfVar, vibe/vibeHi, gps/gps2d, batt/battLow,
 * linkLost/linkWeak) so a fault never shows twice.
 */
function evaluate(now) {
    const on = new Set();
    for (const [id, until] of timed) {
        if (until > now) on.add(id); else timed.delete(id);
    }

    if (isDemoMode()) {
        if (demoId) on.add(demoId);
        return on;
    }
    if (!STATE.connected) return on;

    const hbFresh = (now - STATE.lastHeartbeatTime) < HEARTBEAT_LOST_MS;
    if (STATE.lastHeartbeatTime && !hbFresh) on.add('linkLost');
    if (STATE.systemStatus === 5 || STATE.systemStatus === 6) on.add('failsafe');

    if (now - STATE.sysStatusTime < STALE_MS) {
        if (badSensor(S.GYRO) || badSensor(S.ACCEL) || badSensor(S.GYRO2) || badSensor(S.ACCEL2)) on.add('imu');
        if (badSensor(S.MAG) || badSensor(S.MAG2)) on.add('mag');
        if (badSensor(S.BARO))    on.add('baro');
        if (badSensor(S.DPRESS))  on.add('aspd');
        if (badSensor(S.GPS))     on.add('gps');
        if (badSensor(S.AHRS))    on.add('ahrs');
        if (badSensor(S.RC))      on.add('rc');
        if (badSensor(S.MOTOR))   on.add('motor');
        if (badSensor(S.BATTERY)) on.add('batt');
        if (badSensor(S.FENCE))   on.add('fence');
        if (badSensor(S.TERRAIN)) on.add('terrain');
        if (badSensor(S.LOGGING)) on.add('logging');
        if (badSensor(S.LASER))   on.add('lidar');
        if (badSensor(S.FLOW))    on.add('flow');
        if (badSensor(S.PROX))    on.add('prox');
        // Pre-arm only matters on the ground; once armed the bit is moot.
        if (!STATE.armed && badSensor(S.PREARM)) on.add('prearm');
    }

    if (now - STATE.ekfDataTime < STALE_MS) {
        if (STATE.ekfVariance > EKF_WARN)      on.add('ekf');
        else if (STATE.ekfVariance > EKF_CAUT) on.add('ekfVar');
        if (STATE.ekfFlags & EKF_GPS_GLITCHING) on.add('gpsGlitch');
    }

    // Vibration: level thresholds, plus clipping — any increase in the clip
    // counters is an event worth holding red for a while.
    const clipSum = STATE.vibClip0 + STATE.vibClip1 + STATE.vibClip2;
    if (lastClipSum >= 0 && clipSum > lastClipSum) lastClipChange = now;
    lastClipSum = clipSum;
    const vibMax = Math.max(STATE.vibX, STATE.vibY, STATE.vibZ);
    if (vibMax > VIBE_WARN || (now - lastClipChange) < CLIP_HOLD_MS) on.add('vibe');
    else if (vibMax > VIBE_CAUT) on.add('vibeHi');

    if (STATE.gpsDataTime && now - STATE.gpsDataTime < STALE_MS) {
        if (STATE.gpsFix < 2)       on.add('gps');
        else if (STATE.gpsFix === 2) on.add('gps2d');
        else if (STATE.gpsHdop > HDOP_CAUT) on.add('hdop');
    }
    if (now - STATE.gnssIntegrityTime < STALE_MS) {
        if (STATE.gnssJamming >= 2)  on.add('gpsJam');    // mitigated or detected
        if (STATE.gnssSpoofing >= 2) on.add('gpsSpoof');
    }

    if (STATE.magCalTime && now - STATE.magCalTime < STALE_MS) on.add('magCal');

    if (STATE.batteryVoltage > 0 && STATE.batteryRemaining >= 0 &&
        STATE.batteryRemaining <= BATT_LOW_PCT) on.add('battLow');

    if (hbFresh && (STATE.linkQuality < LINK_WEAK_PCT ||
        (STATE.rssi !== null && STATE.rssi < RSSI_WEAK))) on.add('linkWeak');

    // Warning beats its caution twin
    if (on.has('ekf'))      on.delete('ekfVar');
    if (on.has('vibe'))     on.delete('vibeHi');
    if (on.has('gps'))      { on.delete('gps2d'); on.delete('hdop'); }
    if (on.has('batt'))     on.delete('battLow');
    if (on.has('linkLost')) on.delete('linkWeak');
    return on;
}

// ── Demo ──────────────────────────────────────────────────────────────────

function stepDemo(now) {
    if (demoId && now >= demoUntil) {
        demoId = null;
        demoNextAt = now + DEMO_OFF_MS;
    }
    if (!demoId && now >= demoNextAt) {
        let pick;
        do { pick = ALERTS[Math.floor(Math.random() * ALERTS.length)].id; }
        while (pick === demoId);
        demoId = pick;
        demoUntil = now + DEMO_ON_MS;
    }
}

// ── Render ────────────────────────────────────────────────────────────────

function buildItem(def) {
    const item = document.createElement('div');
    item.className = `annun annun-${def.lvl}`;
    item.dataset.id = def.id;
    item.title = def.title;
    const svg = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
    const use = document.createElementNS('http://www.w3.org/2000/svg', 'use');
    use.setAttribute('href', `#an-${def.icon}`);
    svg.appendChild(use);
    const lbl = document.createElement('span');
    lbl.className = 'annun-lbl';
    lbl.textContent = def.label;
    item.append(svg, lbl);
    if (def.val) {
        const val = document.createElement('span');
        val.className = 'annun-val';
        item.appendChild(val);
        valEls.set(def.id, val);
    }
    return item;
}

function fillList(el, defs) {
    el.replaceChildren(...defs.map(buildItem));
}

function refreshValues(demo) {
    for (const [id, el] of valEls) {
        const def = ALERTS.find(d => d.id === id);
        const text = demo ? (def.demoVal || '') : def.val();
        if (el.textContent !== text) el.textContent = text;
    }
}

/**
 * Refresh the strip. Cheap enough for 10 Hz: the DOM is only rebuilt when
 * the active set changes.
 */
export function updateAnnunciatorPanel() {
    if (!elWarn || !elCaut) return;
    const now = Date.now();
    if (isDemoMode()) stepDemo(now);
    else if (demoId) { demoId = null; demoNextAt = 0; }

    const on = evaluate(now);
    const warns = [], cauts = [];
    for (const def of ALERTS) {
        if (!on.has(def.id)) continue;
        (def.lvl === WARN ? warns : cauts).push(def);
    }
    const key = warns.map(d => d.id).join(',') + '|' + cauts.map(d => d.id).join(',');
    if (key !== lastKey) {
        lastKey = key;
        valEls.clear();
        fillList(elWarn, warns);
        fillList(elCaut, cauts);
    }
    refreshValues(isDemoMode());
}
