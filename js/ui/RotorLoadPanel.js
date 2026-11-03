/**
 * RotorLoadPanel.js - ROTOR LOAD schematic
 *
 * Top-down functional diagram of the propulsion: one round gauge per rotor,
 * placed where that motor sits on the airframe and driven by the live
 * SERVO_OUTPUT_RAW PWM of its output. Fixed wing and helicopter get a single
 * gauge for the main rotor / propeller.
 *
 * The gauges copy the geometry of a Boeing/Airbus N1 indicator: a bare 270°
 * scale circle starting lower-left, a fixed tick at zero, a moving tick and
 * filled arc that follow the PWM, and the raw value as a digital readout.
 *
 * A sailboat (Rover with a rotating mast) gets two round dials instead: the
 * wing and the rudder drawn as NACA foils at the commanded angle, and the
 * apparent wind.
 */

import { STATE, isDemoMode } from '../core/state.js';
import { requestParameter } from '../mavlink/CommandSender.js';

// ── Configuration (SYS CONFIG > ROTOR LOAD, persisted) ──────────────────

const CFG_KEY = 'rotor-load-cfg';

const DEFAULT_CFG = {
    enabled: true,
    pwmMin: 1000,     // motors stopped / scale floor
    pwmMax: 2000,     // scale ceiling
    thGreen: 1100,    // above this → green (spinning)
    thOrange: 1500,   // above this → orange (high load)
    thRed: 1700,      // above this → red (max load)
    frame: 'auto',    // 'auto' (from MAV_TYPE), 'sail', or a rotor count as a string
    singleChannel: 3, // output channel used for the single-rotor gauge (1-16)
    rudderMax: 35     // rudder angle at full steering: a property of the boat, not a parameter
};

let cfg = { ...DEFAULT_CFG };

function loadConfig() {
    try {
        const raw = localStorage.getItem(CFG_KEY);
        if (raw) cfg = { ...DEFAULT_CFG, ...JSON.parse(raw) };
    } catch (e) { /* corrupt entry → defaults */ }
    sanitizeConfig();
}

function saveConfig() {
    try { localStorage.setItem(CFG_KEY, JSON.stringify(cfg)); } catch (e) {}
}

/**
 * Thresholds that cross over each other would make the colours and the scale
 * meaningless, so they are forced back into ascending order.
 */
function sanitizeConfig() {
    const num = (v, def) => (Number.isFinite(v) ? v : def);
    cfg.pwmMin = Math.min(Math.max(num(cfg.pwmMin, 1000), 500), 2500);
    cfg.pwmMax = Math.min(Math.max(num(cfg.pwmMax, 2000), cfg.pwmMin + 100), 2500);
    cfg.thGreen = Math.min(Math.max(num(cfg.thGreen, 1100), cfg.pwmMin), cfg.pwmMax);
    cfg.thOrange = Math.min(Math.max(num(cfg.thOrange, 1500), cfg.thGreen), cfg.pwmMax);
    cfg.thRed = Math.min(Math.max(num(cfg.thRed, 1700), cfg.thOrange), cfg.pwmMax);
    cfg.singleChannel = Math.min(Math.max(Math.round(num(cfg.singleChannel, 3)), 1), 16);
    cfg.rudderMax = Math.min(Math.max(num(cfg.rudderMax, 35), 1), 180);
}

export function setRotorLoadConfig(patch) {
    cfg = { ...cfg, ...patch };
    sanitizeConfig();
    saveConfig();
    syncConfigInputs();
    applyVisibility();
}

// ── Airframe geometry ───────────────────────────────────────────────────

// Motor positions as ArduPilot defines them in AP_MotorsMatrix: angle in
// degrees clockwise from the nose, indexed by motor number. `outputs` is the
// 1-based servo output each motor drives (tricopters skip output 3).
const FRAMES = {
    3: { angles: [60, 300, 180],                                     outputs: [1, 2, 4] },
    4: { angles: [45, 225, 315, 135],                                outputs: [1, 2, 3, 4] },
    6: { angles: [30, 210, 270, 90, 330, 150],                       outputs: [1, 2, 3, 4, 5, 6] },
    8: { angles: [22.5, 202.5, 67.5, 157.5, 337.5, 292.5, 247.5, 112.5], outputs: [1, 2, 3, 4, 5, 6, 7, 8] }
};

// MAV_TYPE → number of rotors on the schematic. Plane and helicopter show the
// single main rotor only, as does anything we have no matrix for.
const ROTORS_BY_MAV_TYPE = {
    1: 1,   // FIXED_WING
    2: 4,   // QUADROTOR
    3: 1,   // COAXIAL
    4: 1,   // HELICOPTER
    13: 6,  // HEXAROTOR
    14: 8,  // OCTOROTOR
    15: 3,  // TRICOPTER
    16: 1,  // FLAPPING_WING (delta wings)
    19: 4, 20: 4, 21: 4, 22: 4, 23: 4, 24: 4, 25: 4,  // VTOL variants: lift quad
    27: 10, // DECAROTOR
    29: 12  // DODECAROTOR
};

// Vehicles with no rotor worth drawing
const HIDDEN_MAV_TYPES = [5, 6, 10, 11, 12, 17, 18];

/**
 * Build the {angles, outputs} table for a rotor count, generating an evenly
 * spaced ring for the counts with no explicit ArduPilot table (deca, dodeca).
 */
function frameFor(count) {
    if (FRAMES[count]) return FRAMES[count];
    const angles = [];
    const outputs = [];
    const step = 360 / count;
    for (let i = 0; i < count; i++) {
        // Alternate opposite arms, the way ArduPilot numbers its matrices
        const k = (i % 2 === 0) ? i / 2 : (count + i - 1) / 2;
        angles.push((step / 2 + k * step) % 360);
        outputs.push(i + 1);
    }
    return { angles, outputs };
}

/** Rotor count currently on screen: manual override, else MAV_TYPE, else 1. */
function activeRotorCount() {
    if (cfg.frame !== 'auto') {
        const n = parseInt(cfg.frame, 10);
        if (Number.isFinite(n) && n >= 1) return n;
    }
    return ROTORS_BY_MAV_TYPE[STATE.vehicleType] || 1;
}

function isHiddenVehicle() {
    return cfg.frame === 'auto' && HIDDEN_MAV_TYPES.includes(STATE.vehicleType);
}

// ── Sailboat: wing and rudder ───────────────────────────────────────────
//
// A Rover with SAIL_ENABLE or a MastRotation output draws two dials in place
// of the rotor gauges. ArduPilot spreads ±100 % of an angle output from
// SERVOn_TRIM to SERVOn_MIN / _MAX, so the commanded angle follows from
// SERVO_OUTPUT_RAW and those three parameters: times SAIL_ANGLE_MAX for the
// mast, times rudderMax for the rudder (full steering). The measured position
// of a DroneCAN servo (actuator.Status) never reaches MAVLink — ArduPilot only
// logs it, in CSRV — so the dials show the commanded angle.

const FUNC_MAST_ROTATION = 137;
const FUNC_GROUND_STEERING = 26;
const ROVER_MAV_TYPES = [10, 11];   // ground rover, surface boat

function param(name) {
    const p = STATE.parameters.get(name);
    return p && Number.isFinite(p.value) ? p.value : null;
}

/** Wing and rudder outputs and the mast scale, or null for a vehicle with no sail. */
function sailLayout() {
    let sail = 0;
    let rudder = 0;
    for (let ch = 1; ch <= 16; ch++) {
        const f = param(`SERVO${ch}_FUNCTION`);
        if (f === FUNC_MAST_ROTATION && !sail) sail = ch;
        if (f === FUNC_GROUND_STEERING && !rudder) rudder = ch;
    }
    if (cfg.frame !== 'sail' && !sail && !(param('SAIL_ENABLE') >= 1)) return null;
    // Until the parameters arrive: ArduPilot's defaults, and AutoSail's mast on 4
    return { sail: sail || 4, rudder: rudder || 1, sailMax: param('SAIL_ANGLE_MAX') ?? 90 };
}

/** The sail layout when the dials replace the rotor gauges, else null. */
function sailMode() {
    if (cfg.frame === 'sail') return sailLayout();
    if (cfg.frame !== 'auto' || !ROVER_MAV_TYPES.includes(STATE.vehicleType)) return null;
    return sailLayout();
}

// The parameters are read one by one when a Rover connects: a full download is
// not needed and may never happen on a slow link. Each round asks for what is
// still missing, a few rounds per connection at most.
const PROBE_GAP_MS = 120;
const PROBE_ROUND_MS = 4000;
const PROBE_ROUNDS = 3;
let probe = { hb: 0, sys: -1, queue: [], next: 0, rounds: 0 };

function wantedParams() {
    const names = ['SAIL_ENABLE', 'SAIL_ANGLE_MAX'];
    for (let ch = 1; ch <= 16; ch++) names.push(`SERVO${ch}_FUNCTION`);
    const lay = sailLayout();
    if (lay) {
        for (const ch of [lay.sail, lay.rudder]) {
            names.push(`SERVO${ch}_MIN`, `SERVO${ch}_TRIM`, `SERVO${ch}_MAX`);
        }
    }
    return names;
}

function probeSailParams(now) {
    if (!STATE.connected || STATE.mode !== 'LIVE' || !ROVER_MAV_TYPES.includes(STATE.vehicleType)) return;
    // A new link, or another vehicle on it, starts the rounds again
    if (STATE.heartbeatCount < probe.hb || STATE.systemId !== probe.sys) {
        probe = { hb: 0, sys: STATE.systemId, queue: [], next: 0, rounds: 0 };
    }
    probe.hb = STATE.heartbeatCount;
    if (now < probe.next) return;
    if (!probe.queue.length) {
        if (probe.rounds >= PROBE_ROUNDS) return;
        probe.queue = wantedParams().filter(n => param(n) === null);
        if (!probe.queue.length) return;
        probe.rounds++;
    }
    requestParameter(probe.queue.shift()).catch(() => {});
    probe.next = now + (probe.queue.length ? PROBE_GAP_MS : PROBE_ROUND_MS);
}

const wrap180 = d => ((d + 180) % 360 + 360) % 360 - 180;

/**
 * Angle commanded on a 1-based output, or null with no SERVO_OUTPUT_RAW: the
 * PWM as a fraction of the way from SERVOn_TRIM to _MAX (or _MIN), times the
 * angle at full output. ArduPilot's defaults stand in until they are read.
 */
function commandedDeg(output, maxDeg) {
    const pwm = STATE.servoOutputs[output - 1];
    if (!Number.isFinite(pwm) || pwm <= 0) return null;
    const lo = param(`SERVO${output}_MIN`) ?? 1100;
    const trim = param(`SERVO${output}_TRIM`) ?? 1500;
    const hi = param(`SERVO${output}_MAX`) ?? 1900;
    if (pwm >= trim) return hi > trim ? clamp((pwm - trim) / (hi - trim), 0, 1) * maxDeg : 0;
    return trim > lo ? -clamp((trim - pwm) / (trim - lo), 0, 1) * maxDeg : 0;
}

/**
 * Arc the trailing edge sweeps over the full travel, as [lo, hi] bearings
 * around the stern, or null for a full turn (nothing to mark).
 */
function travelDeg(maxDeg) {
    return maxDeg >= 179.5 ? null : [180 - maxDeg, 180 + maxDeg];
}

/**
 * Apparent wind from the bow (deg, + = from starboard), or null. AppWndDir is
 * the vane's own reading. WIND alone carries the true wind; with no wind speed
 * sensor ArduPilot makes it apparent + heading, so the heading comes off again.
 */
function apparentWindDeg(now) {
    if (STATE.appWindTime && now - STATE.appWindTime < 3000) return wrap180(STATE.appWindDir);
    if (STATE.windDataTime && now - STATE.windDataTime < 5000) {
        return wrap180(STATE.windDir - STATE.yaw * 180 / Math.PI);
    }
    return null;
}

// ── PWM sourcing ────────────────────────────────────────────────────────

/**
 * Live PWM for a 1-based output channel, or null when nothing has been
 * received. In demo mode there is no autopilot, so the values are synthesised
 * from the demo attitude to keep the schematic alive.
 */
function pwmFor(output, angleDeg) {
    if (isDemoMode()) return demoPwm(angleDeg);
    const v = STATE.servoOutputs[output - 1];
    return (Number.isFinite(v) && v > 0) ? v : null;
}

function demoPwm(angleDeg) {
    if (angleDeg == null) {
        // Single rotor: throttle tracks airspeed around a cruise setting
        return clamp(1350 + STATE.as * 12, cfg.pwmMin, cfg.pwmMax);
    }
    const a = angleDeg * Math.PI / 180;
    // Roll lifts the outboard arms, pitch the aft ones — same sign convention
    // as a real mixer, just scaled to something readable.
    const roll = STATE.roll * 260;
    const pitch = STATE.pitch * 260;
    const base = 1480 - STATE.vs * 20;
    return clamp(base - Math.sin(a) * roll + Math.cos(a) * pitch, 1050, 1950);
}

const clamp = (v, lo, hi) => Math.min(Math.max(v, lo), hi);

// ── Colours ─────────────────────────────────────────────────────────────

const COLOR_IDLE = '#2f9fff';   // blue: at or below idle, motor stopped
const COLOR_GREEN = '#44ff44';
const COLOR_ORANGE = '#ffaa00';
const COLOR_RED = '#ff3030';
const COLOR_NODATA = '#55656f';
const COLOR_SCALE = 'rgba(200, 220, 235, 0.3)';
const COLOR_TICK = 'rgba(200, 220, 235, 0.7)';
const COLOR_TEXT = 'rgba(200, 220, 235, 0.85)';
const COLOR_TRAVEL = 'rgba(47, 159, 255, 0.6)';
const COLOR_FOIL_FILL = 'rgba(68, 255, 68, 0.3)';

function zoneColor(pwm) {
    if (pwm == null) return COLOR_NODATA;
    if (pwm < cfg.thGreen) return COLOR_IDLE;
    if (pwm < cfg.thOrange) return COLOR_GREEN;
    if (pwm < cfg.thRed) return COLOR_ORANGE;
    return COLOR_RED;
}

// ── Rendering ───────────────────────────────────────────────────────────

let canvas = null;
let ctx = null;
let panelEl = null;
let wrapEl = null;     // caption + schematic, shown and hidden together
let captionEl = null;
let visible = null;    // last applied visibility (null: not applied yet)
let sailShown = null;  // dials (true) or rotor gauges (false) on screen
let dpr = 1;

const GAUGE_START = Math.PI * 0.75;   // lower-left
const GAUGE_SWEEP = Math.PI * 1.5;    // 270° clockwise
const LINE_W = 4;                     // arcs and ticks share one stroke weight

function resizeCanvas() {
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    if (w <= 0 || h <= 0) return false;
    dpr = window.devicePixelRatio || 1;
    const pw = Math.max(1, Math.round(w * dpr));
    const ph = Math.max(1, Math.round(h * dpr));
    if (canvas.width !== pw || canvas.height !== ph) {
        canvas.width = pw;
        canvas.height = ph;
    }
    return true;
}

/**
 * One EIS-style round gauge, in the spirit of a Boeing/Airbus N1 indicator:
 * a bare scale circle with an inward tick at zero, an inward moving tick at
 * the current value, and a filled arc between the two. No bezel, no needle,
 * no face, no digits — only the motor label sits in the middle.
 */
function drawGauge(cx, cy, r, pwm, label) {
    const color = zoneColor(pwm);
    const span = cfg.pwmMax - cfg.pwmMin;
    const frac = pwm == null ? 0 : clamp((pwm - cfg.pwmMin) / span, 0, 1);
    const valueAngle = GAUGE_START + frac * GAUGE_SWEEP;
    const scaleR = r * 0.88;

    // Scale circle. The coloured part rides exactly on it — same radius, same
    // width, rounded ends — so the two read as one continuous line with no
    // step where they meet.
    ctx.lineCap = 'round';
    ctx.lineWidth = LINE_W;

    ctx.beginPath();
    ctx.arc(cx, cy, scaleR, GAUGE_START, GAUGE_START + GAUGE_SWEEP);
    ctx.strokeStyle = 'rgba(200, 220, 235, 0.3)';
    ctx.stroke();

    if (pwm != null && frac > 0.001) {
        ctx.beginPath();
        ctx.arc(cx, cy, scaleR, GAUGE_START, valueAngle);
        ctx.strokeStyle = color;
        ctx.stroke();
    }

    // Fixed tick at zero
    tick(cx, cy, GAUGE_START, scaleR, r, 0.34, 'rgba(200, 220, 235, 0.7)');

    // Moving tick at the current value
    tick(cx, cy, valueAngle, scaleR, r, 0.44, pwm == null ? COLOR_NODATA : color);

    ctx.lineCap = 'butt';

    // Motor label in the middle — the arc carries the value
    if (label) {
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillStyle = color;
        ctx.font = `${clamp(Math.round(r * 0.42), 9, 30)}px 'Rajdhani', sans-serif`;
        ctx.fillText(label, cx, cy);
        ctx.textBaseline = 'alphabetic';
    }
}

/** Radial tick growing inward from the scale circle. */
function tick(cx, cy, angle, scaleR, r, len, color) {
    const dx = Math.cos(angle), dy = Math.sin(angle);
    const inner = scaleR - r * len;
    const outer = scaleR;
    ctx.beginPath();
    ctx.lineWidth = LINE_W;
    ctx.strokeStyle = color;
    ctx.moveTo(cx + dx * inner, cy + dy * inner);
    ctx.lineTo(cx + dx * outer, cy + dy * outer);
    ctx.stroke();
}

// Wing and rudder drawn as a symmetric NACA 00xx section: thick enough that
// the round leading edge and the sharp trailing edge read at dial size, and
// short enough that the trailing edge stays inside the 90° ticks
const FOIL_THICKNESS = 0.18;   // NACA 0018
const FOIL_CHORD = 1.05;       // of the dial radius
const FOIL_PIVOT = 0.3;        // mast / rudder stock, chord fraction from the leading edge
const FOIL_STEPS = 24;

/** Half thickness of a NACA 00xx section at x (0..1 of the chord), closed trailing edge. */
function nacaHalfThickness(x, t) {
    return 5 * t * (0.2969 * Math.sqrt(x) - 0.1260 * x - 0.3516 * x * x + 0.2843 * x ** 3 - 0.1036 * x ** 4);
}

/** The foil, leading edge towards bearing `le`, turning about (cx, cy). */
function drawFoil(cx, cy, le, chord, lw) {
    const a = (le - 90) * Math.PI / 180;
    const ux = Math.cos(a), uy = Math.sin(a);   // along the chord, towards the leading edge
    ctx.beginPath();
    // Upper surface leading → trailing edge, lower surface back; cosine
    // spacing puts the points where the nose curves
    for (let i = 0; i <= 2 * FOIL_STEPS; i++) {
        const k = i <= FOIL_STEPS ? i : 2 * FOIL_STEPS - i;
        const side = i <= FOIL_STEPS ? 1 : -1;
        const x = (1 - Math.cos(Math.PI * k / FOIL_STEPS)) / 2;
        const s = (FOIL_PIVOT - x) * chord;
        const h = side * nacaHalfThickness(x, FOIL_THICKNESS) * chord;
        const px = cx + ux * s - uy * h, py = cy + uy * s + ux * h;
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
    }
    ctx.closePath();
    ctx.fillStyle = COLOR_FOIL_FILL;
    ctx.fill();
    ctx.lineWidth = lw;
    ctx.lineJoin = 'round';
    ctx.strokeStyle = COLOR_GREEN;
    ctx.stroke();
    ctx.lineJoin = 'miter';
}

/**
 * One round dial, seen from above with the bow up and angles clockwise, the
 * same convention as the outputs and the wind vane: a scale ring with a tick
 * every 30°, the arc the trailing edge sweeps over the full travel (none for
 * a full turn), the foil turning about its stock, and the apparent wind as a
 * red stroke in from the rim on the side it comes from.
 * @param {number|null} le  bearing of the foil's leading edge, null with no output
 */
function drawDial(cx, cy, r, le, travel, windDeg) {
    const lw = Math.max(2, r * 0.07);
    const pt = (d, rr) => {
        const a = (d - 90) * Math.PI / 180;
        return [cx + Math.cos(a) * rr, cy + Math.sin(a) * rr];
    };
    const line = (p1, p2, width, color) => {
        ctx.beginPath();
        ctx.lineWidth = width;
        ctx.strokeStyle = color;
        ctx.moveTo(p1[0], p1[1]);
        ctx.lineTo(p2[0], p2[1]);
        ctx.stroke();
    };
    const dot = (p, rr, color) => {
        ctx.beginPath();
        ctx.arc(p[0], p[1], rr, 0, Math.PI * 2);
        ctx.fillStyle = color;
        ctx.fill();
    };
    ctx.lineCap = 'round';

    ctx.beginPath();
    ctx.lineWidth = lw;
    ctx.strokeStyle = COLOR_SCALE;
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.stroke();
    if (travel) {
        ctx.beginPath();
        ctx.lineWidth = lw;
        ctx.strokeStyle = COLOR_TRAVEL;
        ctx.arc(cx, cy, r, (travel[0] - 90) * Math.PI / 180, (travel[1] - 90) * Math.PI / 180);
        ctx.stroke();
    }
    for (let d = 0; d < 360; d += 30) {
        const major = d % 90 === 0;
        line(pt(d, r * (major ? 0.74 : 0.86)), pt(d, r), major ? lw : lw * 0.6, COLOR_TICK);
    }

    if (windDeg !== null) line(pt(windDeg, r * 1.12), pt(windDeg, r * 0.62), lw, COLOR_RED);

    if (le !== null) drawFoil(cx, cy, le, r * FOIL_CHORD, Math.max(1.5, lw * 0.5));
    dot([cx, cy], lw * 0.8, le === null ? COLOR_NODATA : COLOR_TEXT);
    ctx.lineCap = 'butt';
}

const signedDeg = d => (d > 0.05 ? '+' : d < -0.05 ? '\u2212' : '') + Math.abs(d).toFixed(0) + '\u00b0';

/** Wing (left) and rudder (right), each with its name and commanded angle underneath. */
function drawSail(w, h, lay) {
    const fs = clamp(Math.round(h * 0.11), 9, 18);
    const textH = fs * 2.3;
    const r = Math.max(8, Math.min(w / 4 - 6, (h - textH) / 2 - 6));
    const cy = (h - textH) / 2;
    const now = Date.now();
    const sailDeg = commandedDeg(lay.sail, lay.sailMax);
    const rudderDeg = commandedDeg(lay.rudder, cfg.rudderMax);
    // The wing's angle is where its leading edge points (into the wind). Steering
    // to starboard swings the rudder's trailing edge to starboard, so its
    // leading edge goes the other way.
    const dials = [
        { cx: w * 0.25, name: 'SAIL', deg: sailDeg, le: sailDeg,
          travel: travelDeg(lay.sailMax), wind: apparentWindDeg(now) },
        { cx: w * 0.75, name: 'RUDDER', deg: rudderDeg, le: rudderDeg === null ? null : -rudderDeg,
          travel: travelDeg(cfg.rudderMax), wind: null }
    ];
    ctx.textAlign = 'center';
    ctx.textBaseline = 'alphabetic';
    ctx.font = `${fs}px 'Rajdhani', sans-serif`;
    for (const d of dials) {
        drawDial(d.cx, cy, r, d.le, d.travel, d.wind);
        const y = cy + r + fs * 1.15;
        ctx.fillStyle = d.deg === null ? COLOR_NODATA : COLOR_TEXT;
        ctx.fillText(`${d.name} ${d.deg === null ? '---' : signedDeg(d.deg)}`, d.cx, y);
        if (d.wind !== null) {
            ctx.fillStyle = COLOR_RED;
            ctx.fillText(`WIND ${signedDeg(d.wind)}`, d.cx, y + fs * 1.1);
        }
    }
}

function draw(lay) {
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    // The panel is borderless over the 3D scene, so everything gets the same
    // drop shadow the rest of the HUD uses to stay readable over bright terrain
    ctx.shadowColor = 'rgba(0, 0, 0, 0.9)';
    ctx.shadowOffsetX = 1;
    ctx.shadowOffsetY = 1;
    ctx.shadowBlur = 2;

    if (lay) {
        drawSail(w, h, lay);
        return;
    }

    const cx = w / 2;
    const cy = h / 2;
    const count = activeRotorCount();

    if (count === 1) {
        const r = Math.min(w, h) * 0.4;
        drawGauge(cx, cy, r, pwmFor(cfg.singleChannel, null), `CH${cfg.singleChannel}`);
        return;
    }

    // Only the gauges are drawn — no airframe body, no arms. Their positions
    // already say which motor is which, so the outline would be pure clutter.
    // The ring is a true circle, not an ellipse: a quad has to look square.
    const { angles, outputs } = frameFor(count);
    const avail = Math.min(w, h) / 2 - 2;
    // Largest gauge whose neighbours on the ring still clear each other:
    // spacing 2·ring·sin(π/n) ≥ 2·gr with ring = avail − gr, minus a small gap
    // so neighbouring circles stay visually separate.
    const sep = Math.sin(Math.PI / count) * 0.9;
    const gr = clamp(avail * sep / (1 + sep), 8, 58);
    const ring = avail - gr;

    for (let i = 0; i < angles.length; i++) {
        const a = (angles[i] - 90) * Math.PI / 180;
        // Labelled by output channel, not by index — a tricopter's tail motor
        // is M4 on output 4, and the label has to match what the autopilot says
        drawGauge(cx + Math.cos(a) * ring, cy + Math.sin(a) * ring, gr,
                  pwmFor(outputs[i], angles[i]), `M${outputs[i]}`);
    }
}

// ── Panel lifecycle ─────────────────────────────────────────────────────

function applyVisibility(lay = sailMode()) {
    if (!panelEl) return;
    const sail = !!lay;
    if (sail !== sailShown) {
        // The dials sit side by side: the panel widens and the caption says what they are
        sailShown = sail;
        panelEl.classList.toggle('sail', sail);
        if (captionEl) captionEl.textContent = sail ? 'Sail \u00b7 Rudder' : 'Rotor Load';
    }
    const show = cfg.enabled && (sail || !isHiddenVehicle());
    if (show === visible) return;
    visible = show;
    // The caption goes with the schematic: hiding the canvas alone left
    // "ROTOR LOAD" sitting on the position / traffic panels below it, which
    // read as the gauges being covered by them.
    (wrapEl || panelEl).style.display = show ? '' : 'none';
}

/** Redraw the schematic — called from the render loop on the flight data tab. */
export function updateRotorLoadPanel() {
    if (!ctx || !panelEl) return;
    // A Rover is hidden until its parameters say it has a sail, so they are
    // asked for before the visibility check
    probeSailParams(Date.now());
    const lay = sailMode();
    applyVisibility(lay);
    if (!visible) return;
    if (!resizeCanvas()) return;
    draw(lay);
}

// ── SYS CONFIG wiring ───────────────────────────────────────────────────

const CFG_INPUTS = [
    ['rotor-cfg-pwmmin', 'pwmMin', 'number'],
    ['rotor-cfg-pwmmax', 'pwmMax', 'number'],
    ['rotor-cfg-green', 'thGreen', 'number'],
    ['rotor-cfg-orange', 'thOrange', 'number'],
    ['rotor-cfg-red', 'thRed', 'number'],
    ['rotor-cfg-channel', 'singleChannel', 'number'],
    ['rotor-cfg-ruddermax', 'rudderMax', 'number'],
    ['rotor-cfg-frame', 'frame', 'string'],
    ['rotor-cfg-enable', 'enabled', 'bool']
];

function syncConfigInputs() {
    for (const [id, key, kind] of CFG_INPUTS) {
        const el = document.getElementById(id);
        if (!el) continue;
        if (kind === 'bool') el.checked = !!cfg[key];
        else el.value = cfg[key];
    }
    // The sidebar's SHOW ROTOR LOAD drives the same setting (window.toggleRotorLoad)
    const sidebarChk = document.getElementById('chk-rotor-load');
    if (sidebarChk) sidebarChk.checked = !!cfg.enabled;
}

function bindConfigInputs() {
    for (const [id, key, kind] of CFG_INPUTS) {
        const el = document.getElementById(id);
        if (!el) continue;
        el.addEventListener('change', () => {
            let v;
            if (kind === 'bool') v = el.checked;
            else if (kind === 'number') v = parseFloat(el.value);
            else v = el.value;
            setRotorLoadConfig({ [key]: v });
        });
    }
}

export function initRotorLoadPanel() {
    loadConfig();
    panelEl = document.getElementById('rotor-load-panel');
    wrapEl = panelEl ? panelEl.closest('.bottom-bar-rotor') : null;
    captionEl = wrapEl ? wrapEl.querySelector('.panel-caption') : null;
    canvas = document.getElementById('rotor-load-canvas');
    ctx = canvas ? canvas.getContext('2d') : null;
    syncConfigInputs();
    bindConfigInputs();
    applyVisibility();
}
