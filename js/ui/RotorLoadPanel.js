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
 */

import { STATE, isDemoMode } from '../core/state.js';

// ── Configuration (SYS CONFIG > ROTOR LOAD, persisted) ──────────────────

const CFG_KEY = 'rotor-load-cfg';

const DEFAULT_CFG = {
    enabled: true,
    pwmMin: 1000,     // motors stopped / scale floor
    pwmMax: 2000,     // scale ceiling
    thGreen: 1100,    // above this → green (spinning)
    thOrange: 1500,   // above this → orange (high load)
    thRed: 1700,      // above this → red (max load)
    frame: 'auto',    // 'auto' (from MAV_TYPE) or a rotor count as a string
    singleChannel: 3  // output channel used for the single-rotor gauge (1-16)
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

function draw() {
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

function applyVisibility() {
    if (!panelEl) return;
    const show = cfg.enabled && !isHiddenVehicle();
    panelEl.style.display = show ? '' : 'none';
}

/** Redraw the schematic — called from the render loop on the flight data tab. */
export function updateRotorLoadPanel() {
    if (!ctx || !panelEl) return;
    applyVisibility();
    if (panelEl.style.display === 'none') return;
    if (!resizeCanvas()) return;
    draw();
}

// ── SYS CONFIG wiring ───────────────────────────────────────────────────

const CFG_INPUTS = [
    ['rotor-cfg-pwmmin', 'pwmMin', 'number'],
    ['rotor-cfg-pwmmax', 'pwmMax', 'number'],
    ['rotor-cfg-green', 'thGreen', 'number'],
    ['rotor-cfg-orange', 'thOrange', 'number'],
    ['rotor-cfg-red', 'thRed', 'number'],
    ['rotor-cfg-channel', 'singleChannel', 'number'],
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
    canvas = document.getElementById('rotor-load-canvas');
    ctx = canvas ? canvas.getContext('2d') : null;
    syncConfigInputs();
    bindConfigInputs();
    applyVisibility();
}
