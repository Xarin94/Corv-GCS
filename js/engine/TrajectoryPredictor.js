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

// ── Bank evolution model ───────────────────────────────────────────
// Instead of freezing the current bank for the whole horizon, the bank
// evolves along the path: measured roll rate carries it initially
// (decaying over RATE_TAU), then it relaxes toward the autopilot's
// commanded roll (NAV_CONTROLLER_OUTPUT) when guidance is fresh.
const RATE_TAU = 0.7;          // s – decay of the measured roll-rate contribution
const RELAX_TAU = 2.0;         // s – relaxation toward commanded nav roll
const MAX_BANK = 60 * Math.PI / 180; // rad – clamp predicted bank
const NAV_FRESH_MS = 3000;     // guidance considered stale after this
const VERT_ACCEL_TAU = 4;      // s – decay of extrapolated vertical acceleration

// ── Low-pass filter state ──────────────────────────────────────────
// Alpha = 0 → no filtering (instant), 1 → frozen.
// 0.80 gives ~5-sample settling at 20 Hz update rate (~250 ms lag).
const LP_ALPHA = 0.88;

const lp = {
    gs: 0,
    roll: 0,
    rollRate: 0,
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
    const rollRate = state.rollRate || 0;
    const vs   = state.vs   || 0;
    const track = state.track || state.yaw || 0;

    // NED down velocity (positive = descending) → invert for climb-positive convention
    const vd = state.vd || 0;
    const now = performance.now();

    if (!lp.initialised) {
        lp.gs = gs;
        lp.roll = roll;
        lp.rollRate = rollRate;
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
        // Lighter filter on the gyro rate: it must stay responsive to
        // capture roll-in/roll-out, jitter is absorbed by RATE_TAU decay.
        lp.rollRate = 0.7 * lp.rollRate + 0.3 * rollRate;
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
        rollRate:  lp.rollRate,
        vs:        lp.vs,
        vertAccel: lp.vertAccel,
        track:     Math.atan2(lp.trackSin, lp.trackCos)
    };
}

/**
 * Compute the predicted bank angle profile along the path.
 * bank(t) starts at the current filtered roll, is carried by the measured
 * roll rate (decaying over RATE_TAU) and relaxes toward the autopilot's
 * commanded roll when NAV_CONTROLLER_OUTPUT guidance is fresh. Without
 * fresh guidance the bank simply settles where the rate leaves it
 * (stick-held assumption), matching the old constant-bank behaviour
 * when the aircraft is not rolling.
 * @returns {Float64Array} bank per step, radians
 */
function computeBankProfile(f, state, numPoints, dt) {
    const navFresh = (state.navDataTime || 0) > 0 &&
        Date.now() - state.navDataTime < NAV_FRESH_MS;
    const targetBank = navFresh ? (state.navRoll || 0) * Math.PI / 180 : null;

    const banks = new Float64Array(numPoints);
    let bank = f.roll;
    for (let i = 0; i < numPoints; i++) {
        const t = i * dt;
        bank += f.rollRate * Math.exp(-t / RATE_TAU) * dt;
        if (targetBank !== null) {
            bank += (targetBank - bank) * Math.min(1, dt / RELAX_TAU);
        }
        bank = Math.max(-MAX_BANK, Math.min(MAX_BANK, bank));
        banks[i] = bank;
    }
    return banks;
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
 * Turn rate is re-derived per step from the evolving bank profile, so the
 * path captures roll-in/roll-out instead of freezing the current turn.
 * Vertical trajectory uses vs + decaying vertAccel (from NED vd derivative).
 * Each point carries its predicted bank so the corridor can twist like
 * wingtip trails.
 * @param {object} state - Vehicle STATE object
 * @param {number} [numPoints=40]
 * @param {number} [totalTimeSec=15]
 * @returns {Array<{x:number, y:number, z:number, lat:number, lon:number, alt:number, t:number, bank:number}>}
 */
export function computePredictedPath(state, numPoints = 40, totalTimeSec = 15) {
    const f = filtered(state);
    if (f.gs < MIN_SPEED) return [];

    const dt = totalTimeSec / numPoints;
    const banks = computeBankProfile(f, state, numPoints, dt);
    const offsetAlt = state.offsetAlt || 0;

    let heading = f.track;
    let lat = state.lat || 0;
    let lon = state.lon || 0;
    let alt = (state.rawAlt || 0) + offsetAlt;
    let currentVs = f.vs;

    const points = [];
    for (let i = 0; i < numPoints; i++) {
        const t = (i + 1) * dt;
        heading += computeOmega(banks[i], f.gs) * dt;

        const cosHdg = Math.cos(heading);
        const sinHdg = Math.sin(heading);
        lat += (f.gs * cosHdg * dt) / DEG_TO_M;
        lon += (f.gs * sinHdg * dt) / (DEG_TO_M * Math.cos(lat * Math.PI / 180));

        // Vertical: vs evolves with decaying vertical acceleration —
        // a constant extrapolation turns sensor noise into 20 s parabolas.
        currentVs += f.vertAccel * Math.exp(-t / VERT_ACCEL_TAU) * dt;
        alt += currentVs * dt;

        const pos = latLonToMeters(lat, lon);
        points.push({ x: pos.x, y: alt, z: pos.z, lat, lon, alt, t, bank: banks[i] });
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
    const banks = computeBankProfile(f, state, numPoints, dt);

    let heading = f.track;
    let lat = state.lat || 0;
    let lon = state.lon || 0;

    const points = [];
    for (let i = 0; i < numPoints; i++) {
        const t = (i + 1) * dt;
        heading += computeOmega(banks[i], f.gs) * dt;
        lat += (f.gs * Math.cos(heading) * dt) / DEG_TO_M;
        lon += (f.gs * Math.sin(heading) * dt) / (DEG_TO_M * Math.cos(lat * Math.PI / 180));
        points.push({ lat, lon, t });
    }

    return points;
}
