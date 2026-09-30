/**
 * RelativeNav.js - Navigation without an absolute position
 *
 * What ArduPilot sends when the vehicle has never had a GPS (measured on an
 * ArduSub 4.7.1 SITL with GPS1_TYPE 0): no LOCAL_POSITION_NED at all,
 * GLOBAL_POSITION_INT with lat, lon, alt and relative_alt all 0, the EKF in
 * constant-position mode (EKF_STATUS_REPORT flag 128). Attitude and heading
 * are good; the EKF velocity is pure inertial drift (0.9 m/s reported while
 * the vehicle moved 2 m in 40 s). An EKF3 without a position source keeps a
 * local frame only once an origin exists — a GPS fix at some point, a DVL or
 * USBL set up with SET_GPS_GLOBAL_ORIGIN — and then LOCAL_POSITION_NED is the
 * position. ArduPlane dead-reckons on airspeed and the wind estimate after a
 * GPS loss, but with no GPS ever it has no origin either.
 *
 * So the GCS does what it can here:
 *   - Relative mode (SYS CONFIG): the view works in a local frame. The
 *     position is LOCAL_POSITION_NED when the vehicle sends it, otherwise it
 *     is dead-reckoned from a velocity. Height: LOCAL_POSITION_NED, else the
 *     relative altitude of a real fix, else the pressure sensor relative to
 *     its first reading (water depth on a sub, the barometer otherwise). The
 *     frame is laid around the world origin as synthetic lat/lon, so the rest
 *     of the GCS keeps working unchanged; the 3D view hides the terrain and
 *     draws a plane at zero instead.
 *   - Dead-reckoning track (SYS CONFIG): a position integrated from velocity
 *     alone — airspeed or ground speed along the heading, or the EKF / local
 *     velocity — drawn dashed next to the real track, so its drift shows.
 *
 * Integration runs on the autopilot's ATTITUDE timestamps (time_boot_ms), so
 * a log replayed faster than real time integrates the same way.
 */

import { STATE } from './state.js';
import { ORIGIN } from './constants.js';

const STORE_KEY = 'corv-relnav';
const FIXED_WING = new Set([1, 16, 19, 20, 21, 22, 23, 24, 25]);
const SUBMARINE = 12;
const FRESH_MS = 2000;
const MAX_STEP_S = 0.5;          // longer gaps (a stalled link) are not integrated
const TRACK_STEP_M = 0.5;
const MAX_TRACK = 20000;
const WATER_DENSITY = { fresh: 997, salt: 1025 };
const STANDARD_HPA = 1013.25;
const SURFACE_MAX_HPA = 1100;
const M_PER_DEG = 111320;

const settings = { relative: false, drTrack: false, velSource: 'auto', water: 'fresh' };
try { Object.assign(settings, JSON.parse(localStorage.getItem(STORE_KEY) || '{}')); } catch (e) { /* defaults */ }

// Latest inputs
let yaw = null;                  // rad
let bootMs = null;               // ATTITUDE time_boot_ms of the last step
let local = null;                // { x, y, z, vx, vy, vz, t }
let ekf = null;                  // { vn, ve, vd, t } — GLOBAL_POSITION_INT velocity
let fix = null;                  // { relAlt, t } while GLOBAL_POSITION_INT has a real position
let vfr = null;                  // { airspeed, groundspeed, heading, climb, t }
let pAir0 = null, pAir = null, pWater0 = null, pWater = null;

// Dead-reckoned position, metres north/east/down from its anchor
const dr = { n: 0, e: 0, d: 0, anchor: null, src: null };
const track = { ned: new Float32Array(MAX_TRACK * 3), count: 0, version: 0 };

export function getRelNavSettings() { return { ...settings }; }
export function isRelativeMode() { return settings.relative; }
export function isDeadReckoningTrack() { return settings.drTrack; }

/**
 * Change a setting (relative, drTrack, velSource, water); raises 'relNavChanged'.
 * Switching the relative mode (either way) or the track on starts a fresh estimate.
 */
export function setRelNavSetting(key, value) {
    if (!(key in settings) || settings[key] === value) return;
    settings[key] = value;
    try { localStorage.setItem(STORE_KEY, JSON.stringify(settings)); } catch (e) { /* not persisted */ }
    // The track is anchored in one frame or the other: a new frame, a new track
    if (key === 'relative' || (key === 'drTrack' && value)) resetRelNav();
    window.dispatchEvent(new CustomEvent('relNavChanged', { detail: { ...settings } }));
}

/** Start again from zero: position, pressure reference and track. */
export function resetRelNav() {
    dr.n = dr.e = dr.d = 0;
    dr.anchor = null;
    pAir0 = pAir = pWater0 = pWater = null;
    bootMs = null;
    track.count = 0;
    track.version++;
}

/** Feed every MAVLink message (called by the state mapper before its own mapping). */
export function relNavOnMessage(msgId, d) {
    const now = Date.now();
    switch (msgId) {
        case 30:   // ATTITUDE: heading, and the integration clock
            if (!Number.isFinite(d.yaw)) return;
            yaw = d.yaw;
            step(Number.isFinite(d.timeBootMs) ? d.timeBootMs : now);
            break;
        case 32:   // LOCAL_POSITION_NED
            if (!Number.isFinite(d.x)) return;
            local = { x: d.x, y: d.y, z: d.z, vx: d.vx, vy: d.vy, vz: d.vz, t: now };
            break;
        case 33:   // GLOBAL_POSITION_INT
            if (!Number.isFinite(d.vx)) return;
            ekf = { vn: d.vx / 100, ve: d.vy / 100, vd: d.vz / 100, t: now };
            if (d.lat !== 0 && d.lon !== 0 && Number.isFinite(d.relativeAlt)) fix = { relAlt: d.relativeAlt / 1000, t: now };
            break;
        case 74:   // VFR_HUD
            vfr = { airspeed: d.airspeed, groundspeed: d.groundspeed, heading: d.heading, climb: d.climb, t: now };
            break;
        case 29:   // SCALED_PRESSURE: the barometer
            if (Number.isFinite(d.pressAbs)) { pAir = d.pressAbs; if (pAir0 === null) pAir0 = pAir; }
            break;
        case 137:  // SCALED_PRESSURE2: a sub's external depth sensor
            if (Number.isFinite(d.pressAbs)) {
                pWater = d.pressAbs;
                // The first reading is the surface — unless the GCS connected
                // during a dive: above 1100 hPa the vehicle is already a metre
                // or more down, and standard sea-level pressure is the better zero
                if (pWater0 === null) pWater0 = pWater > SURFACE_MAX_HPA ? STANDARD_HPA : pWater;
            }
            break;
    }
}

const fresh = (o) => o && Date.now() - o.t < FRESH_MS;

/**
 * Velocity to dead-reckon on (m/s, NED), per the chosen source. 'auto': a
 * fixed wing's airspeed along the heading when it has one, else the local
 * frame's velocity, else the EKF's, else the ground speed along the heading.
 */
function pickVelocity() {
    const hdg = yaw !== null ? yaw : (fresh(vfr) && Number.isFinite(vfr.heading) ? vfr.heading * Math.PI / 180 : null);
    const vd = fresh(local) ? local.vz : fresh(ekf) ? ekf.vd : fresh(vfr) && Number.isFinite(vfr.climb) ? -vfr.climb : 0;
    const along = (speed, src) => (hdg !== null && Number.isFinite(speed))
        ? { vn: speed * Math.cos(hdg), ve: speed * Math.sin(hdg), vd, src } : null;
    switch (settings.velSource) {
        case 'air':    return fresh(vfr) ? along(vfr.airspeed, 'air') : null;
        case 'ground': return fresh(vfr) ? along(vfr.groundspeed, 'ground') : null;
        case 'ekf':
            if (fresh(local)) return { vn: local.vx, ve: local.vy, vd: local.vz, src: 'local' };
            return fresh(ekf) ? { vn: ekf.vn, ve: ekf.ve, vd: ekf.vd, src: 'ekf' } : null;
        default:
            if (FIXED_WING.has(STATE.vehicleType) && fresh(vfr) && vfr.airspeed > 3) return along(vfr.airspeed, 'air');
            if (fresh(local)) return { vn: local.vx, ve: local.vy, vd: local.vz, src: 'local' };
            if (fresh(ekf)) return { vn: ekf.vn, ve: ekf.ve, vd: ekf.vd, src: 'ekf' };
            return fresh(vfr) ? along(vfr.groundspeed, 'ground') : null;
    }
}

function step(t) {
    const prev = bootMs;
    bootMs = t;
    if (!settings.relative && !settings.drTrack) return;
    if (prev === null || t <= prev) return;           // first sample, or the autopilot rebooted
    const dt = (t - prev) / 1000;
    if (dt > MAX_STEP_S) return;
    const v = pickVelocity();
    if (!v) return;
    dr.src = v.src;
    if (!dr.anchor) {
        // Relative mode: the local frame's zero. Otherwise the vehicle's
        // position when the track starts.
        if (settings.relative) dr.anchor = { lat: ORIGIN.lat, lon: ORIGIN.lon, alt: 0 };
        else if (STATE.lat || STATE.lon) dr.anchor = { lat: STATE.lat, lon: STATE.lon, alt: STATE.rawAlt || 0 };
        else return;
    }
    dr.n += v.vn * dt;
    dr.e += v.ve * dt;
    dr.d += v.vd * dt;
    appendTrack();
}

function appendTrack() {
    const a = track.ned;
    let n = track.count;
    if (n > 0) {
        const o = (n - 1) * 3;
        if (Math.hypot(dr.n - a[o], dr.e - a[o + 1], dr.d - a[o + 2]) < TRACK_STEP_M) return;
    }
    if (n >= MAX_TRACK) {
        // Keep every other point
        for (let r = 0, w = 0; r < n; r += 2, w++) { a[w * 3] = a[r * 3]; a[w * 3 + 1] = a[r * 3 + 1]; a[w * 3 + 2] = a[r * 3 + 2]; }
        n = Math.ceil(n / 2);
    }
    a[n * 3] = dr.n; a[n * 3 + 1] = dr.e; a[n * 3 + 2] = dr.d;
    track.count = n + 1;
    track.version++;
}

/** Height (m, up) in the relative frame, and where it came from. */
function relativeHeight() {
    if (fresh(local)) return -local.z;
    if (fresh(fix)) return fix.relAlt;
    if (STATE.vehicleType === SUBMARINE && pWater !== null) {
        return -((pWater - pWater0) * 100) / (WATER_DENSITY[settings.water] * 9.80665);
    }
    if (pAir !== null && pAir > 0 && pAir0 > 0) return 44330 * (1 - Math.pow(pAir / pAir0, 1 / 5.255));
    return -dr.d;
}

/**
 * Relative mode: write the local position into STATE as synthetic lat/lon
 * around the world origin (1 m north = 1 m of world z), height above zero as
 * rawAlt, home at zero. Called once per frame.
 */
export function applyRelativeToState() {
    if (!settings.relative) return;
    const n = fresh(local) ? local.x : dr.n;
    const e = fresh(local) ? local.y : dr.e;
    STATE.lat = ORIGIN.lat + n / M_PER_DEG;
    STATE.lon = ORIGIN.lon + e / (M_PER_DEG * Math.cos(ORIGIN.lat * Math.PI / 180));
    STATE.rawAlt = relativeHeight();
    STATE.homeLat = ORIGIN.lat;
    STATE.homeLon = ORIGIN.lon;
    STATE.homeAlt = 0;
    // Without a fix the velocity shown is the one being integrated
    if (!fresh(local) && !fresh(fix)) {
        const v = pickVelocity();
        if (v) {
            STATE.vn = v.vn; STATE.ve = v.ve; STATE.vd = v.vd;
            STATE.gs = Math.hypot(v.vn, v.ve);
            STATE.vs = -v.vd;
        }
    }
}

/** Where the relative position comes from: 'local', 'dead-reckoning', or null outside the mode. */
export function relativeSource() {
    if (!settings.relative) return null;
    return fresh(local) ? 'local' : 'dead-reckoning';
}

/** Velocity source the dead reckoning is using ('air', 'ground', 'ekf', 'local'), null before the first step. */
export function deadReckoningSource() { return dr.src; }

/**
 * The dead-reckoned track in world coordinates.
 * @param {(lat:number, lon:number) => {x:number, z:number}} toWorld latLonToMeters
 * @param {number} offsetAlt the altitude offset every vehicle height gets in the scene
 * @param {Float32Array} out receives x, y, z per point (at least MAX_TRACK * 3)
 * @returns {number} points written
 */
export function deadReckoningWorld(toWorld, offsetAlt, out) {
    if (!dr.anchor || track.count < 2) return 0;
    const { lat, lon, alt } = dr.anchor;
    const kLon = M_PER_DEG * Math.cos(lat * Math.PI / 180);
    const a = track.ned;
    for (let i = 0; i < track.count; i++) {
        const w = toWorld(lat + a[i * 3] / M_PER_DEG, lon + a[i * 3 + 1] / kLon);
        out[i * 3] = w.x;
        out[i * 3 + 1] = alt - a[i * 3 + 2] + offsetAlt;
        out[i * 3 + 2] = w.z;
    }
    return track.count;
}

/** Changes whenever the track does (to rebuild its drawing only then). */
export function deadReckoningVersion() { return track.version; }

export const DR_TRACK_CAPACITY = MAX_TRACK;
