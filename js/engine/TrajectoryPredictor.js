/**
 * TrajectoryPredictor.js - Predicted trajectory computation
 * Computes future flight path based on current speed, turn rate, and vertical speed.
 * Inputs are low-pass filtered to avoid jitter / sudden visual jumps.
 * Vertical acceleration is derived from NED vd (global frame, not body frame).
 */

import { latLonToMeters } from '../core/utils.js';

const G = 9.81;
const MIN_SPEED = 2;           // m/s – below this, no prediction
const MIN_ROLL_FOR_TURN = 0.00873; // ~0.5° – below this, treat as straight
const MIN_TURN_RADIUS = 10;   // metres – clamp to avoid visual artifacts
const DEG_TO_M = 111320;      // metres per degree of latitude
const MAX_VERT_ACCEL = 5;     // m/s² – clamp vertical acceleration

// ── Low-pass filter state ──────────────────────────────────────────
// Alpha = 0 → no filtering (instant), 1 → frozen.
// 0.80 gives ~5-sample settling at 20 Hz update rate (~250 ms lag).
const LP_ALPHA = 0.88;

const lp = {
    gs: 0,
    roll: 0,
    vs: 0,
    vertAccel: 0,       // vertical acceleration (m/s²), NED-derived
    trackSin: 0,
    trackCos: 1,
    // For computing dvs/dt from NED vd
    prevVd: 0,
    prevVdTime: 0,
    initialised: false
};

/**
 * Feed raw state values through the low-pass filter and return smoothed values.
 * Vertical acceleration is computed from the NED vd component (global frame).
 */
function filtered(state) {
    const gs   = state.gs   || 0;
    const roll = state.roll || 0;
    const vs   = state.vs   || 0;
    const track = state.track || state.yaw || 0;

    // NED down velocity (positive = descending) → invert for climb-positive convention
    const vd = state.vd || 0;
    const now = performance.now();

    if (!lp.initialised) {
        lp.gs = gs;
        lp.roll = roll;
        lp.vs = vs;
        lp.vertAccel = 0;
        lp.trackSin = Math.sin(track);
        lp.trackCos = Math.cos(track);
        lp.prevVd = vd;
        lp.prevVdTime = now;
        lp.initialised = true;
    } else {
        lp.gs       = LP_ALPHA * lp.gs       + (1 - LP_ALPHA) * gs;
        lp.roll     = LP_ALPHA * lp.roll     + (1 - LP_ALPHA) * roll;
        lp.vs       = LP_ALPHA * lp.vs       + (1 - LP_ALPHA) * vs;
        lp.trackSin = LP_ALPHA * lp.trackSin + (1 - LP_ALPHA) * Math.sin(track);
        lp.trackCos = LP_ALPHA * lp.trackCos + (1 - LP_ALPHA) * Math.cos(track);

        // Compute vertical acceleration from NED vd derivative
        const dtMs = now - lp.prevVdTime;
        if (dtMs > 10) { // avoid div-by-zero, min 10ms between samples
            const dtSec = dtMs / 1000;
            // dvd/dt in NED (positive down), invert to climb-positive
            const rawAccel = -(vd - lp.prevVd) / dtSec;
            const clampedAccel = Math.max(-MAX_VERT_ACCEL, Math.min(MAX_VERT_ACCEL, rawAccel));
            lp.vertAccel = LP_ALPHA * lp.vertAccel + (1 - LP_ALPHA) * clampedAccel;
            lp.prevVd = vd;
            lp.prevVdTime = now;
        }
    }

    return {
        gs:        lp.gs,
        roll:      lp.roll,
        vs:        lp.vs,
        vertAccel: lp.vertAccel,
        track:     Math.atan2(lp.trackSin, lp.trackCos)
    };
}

/**
 * Compute turn angular velocity from roll and groundspeed.
 */
function computeOmega(roll, gs) {
    if (Math.abs(roll) < MIN_ROLL_FOR_TURN) return 0;
    const tanRoll = Math.tan(roll);
    const radius = (gs * gs) / (G * Math.abs(tanRoll));
    if (radius < MIN_TURN_RADIUS) {
        return (gs / MIN_TURN_RADIUS) * Math.sign(tanRoll);
    }
    return (G * tanRoll) / gs;
}

/**
 * Compute predicted flight path points (3D — for corridor mesh).
 * Vertical trajectory uses vs + vertAccel (from NED vd derivative).
 * @param {object} state - Vehicle STATE object
 * @param {number} [numPoints=40]
 * @param {number} [totalTimeSec=15]
 * @returns {Array<{x:number, y:number, z:number, lat:number, lon:number, alt:number, t:number}>}
 */
export function computePredictedPath(state, numPoints = 40, totalTimeSec = 15) {
    const f = filtered(state);
    if (f.gs < MIN_SPEED) return [];

    const dt = totalTimeSec / numPoints;
    const omega = computeOmega(f.roll, f.gs);
    const offsetAlt = state.offsetAlt || 0;

    let heading = f.track;
    let lat = state.lat || 0;
    let lon = state.lon || 0;
    let alt = (state.rawAlt || 0) + offsetAlt;
    let currentVs = f.vs;

    const points = [];
    for (let i = 0; i < numPoints; i++) {
        const t = (i + 1) * dt;
        heading += omega * dt;

        const cosHdg = Math.cos(heading);
        const sinHdg = Math.sin(heading);
        lat += (f.gs * cosHdg * dt) / DEG_TO_M;
        lon += (f.gs * sinHdg * dt) / (DEG_TO_M * Math.cos(lat * Math.PI / 180));

        // Vertical: vs evolves with vertical acceleration
        currentVs += f.vertAccel * dt;
        alt += currentVs * dt;

        const pos = latLonToMeters(lat, lon);
        points.push({ x: pos.x, y: alt, z: pos.z, lat, lon, alt, t });
    }

    return points;
}

/**
 * Compute predicted path in geographic coordinates only (for 2D ND).
 * Uses the same low-pass filtered values.
 * @param {object} state
 * @param {number} [numPoints=30]
 * @param {number} [totalTimeSec=15]
 * @returns {Array<{lat:number, lon:number, t:number}>}
 */
export function computePredictedPath2D(state, numPoints = 30, totalTimeSec = 15) {
    const f = filtered(state);
    if (f.gs < MIN_SPEED) return [];

    const dt = totalTimeSec / numPoints;
    const omega = computeOmega(f.roll, f.gs);

    let heading = f.track;
    let lat = state.lat || 0;
    let lon = state.lon || 0;

    const points = [];
    for (let i = 0; i < numPoints; i++) {
        const t = (i + 1) * dt;
        heading += omega * dt;
        lat += (f.gs * Math.cos(heading) * dt) / DEG_TO_M;
        lon += (f.gs * Math.sin(heading) * dt) / (DEG_TO_M * Math.cos(lat * Math.PI / 180));
        points.push({ lat, lon, t });
    }

    return points;
}
