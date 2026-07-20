/**
 * HUDRenderer.js - Canvas HUD Drawing
 * Handles the 2D canvas overlay for flight instruments
 * Graphics style ported from origin/main HUD class
 */

import { STATE, gHistoryBuffer } from '../core/state.js';

// HUD state
let canvas = null;
let ctx = null;
let hudDPR = 1;

// HUD message queue (Mission Planner style)
const hudMessages = [];
const MAX_HUD_MESSAGES = 5;
const HUD_MSG_DURATION = 5000; // ms
let prevArmedState = false;
let armedFlashTimer = 0;
let disarmedLabelEl = null;

// Optional dedicated G-load widget canvas (DOM widget)
let gCanvas = null;
let gCtx = null;

// Object pool for G-load graph points (avoids GC pressure from 300 objects × 60 FPS = 18k objects/sec)
const GLOAD_POOL_SIZE = 350;
const gLoadPointPool = new Array(GLOAD_POOL_SIZE);
for (let i = 0; i < GLOAD_POOL_SIZE; i++) {
    gLoadPointPool[i] = { x: 0, y: 0 };
}

// Style configuration (origin/main style)
const style = {
    lineWidth: 2,
    color: 'rgba(0, 255, 127, 1)',
    font: {
        style: 'normal',
        variant: 'normal',
        weight: 'bold',
        family: 'Arial',
        scale: 1,
    },
    hasShadow: true,
    shadow: {
        lineWidth: 2.5,
        color: 'rgba(0, 0, 0, 0.6)',
        offset: 1.8,
    },
    scale: 1,
    stepWidth: 8,
};

// Settings
const settings = {
    _pixelPerDeg: 12,
    _pixelPerRad: 12 * (180 / Math.PI),
    set pixelPerDeg(val) {
        this._pixelPerDeg = val;
        this._pixelPerRad = val * (180 / Math.PI);
    },
    get pixelPerDeg() { return this._pixelPerDeg; },
    get pixelPerRad() { return this._pixelPerRad; },
    uncagedMode: false,
    timezone: undefined,
    scale: 1,
};

// Virtual size (updated on resize)
let size = { width: 800, height: 600 };
let hudWrapperEl = null; // cached DOM reference

// Cached CSS variables (refreshed on resize, not per-frame)
const cachedCss = {
    accentRed: '#ff0000',
    textMain: '#ffffff',
    textDim: '#6e7f8d',
    fontData: 'monospace'
};

function refreshCachedCss() {
    try {
        const style = getComputedStyle(document.documentElement);
        cachedCss.accentRed = style.getPropertyValue('--accent-red').trim() || '#ff0000';
        cachedCss.textMain = style.getPropertyValue('--text-main').trim() || '#ffffff';
        cachedCss.textDim = style.getPropertyValue('--text-dim').trim() || '#6e7f8d';
        cachedCss.fontData = style.getPropertyValue('--font-data').trim() || 'monospace';
    } catch (_) {}
}

function getCssVar(name, fallback) {
    // Fast path for cached variables
    if (name === '--accent-red') return cachedCss.accentRed;
    if (name === '--text-main') return cachedCss.textMain;
    if (name === '--text-dim') return cachedCss.textDim;
    if (name === '--font-data') return cachedCss.fontData;
    try {
        const v = getComputedStyle(document.documentElement).getPropertyValue(name);
        const s = (v || '').trim();
        return s || fallback;
    } catch (_) {
        return fallback;
    }
}

/**
 * Set font with exact size
 */
function setFont(size, unit) {
    ctx.font = `${style.font.style} ${style.font.variant} ${style.font.weight} ${size}${unit} ${style.font.family}`;
}

/**
 * Set font with scale factor applied
 */
function setFontScale(fontSize, unit) {
    fontSize *= style.font.scale;
    setFont(fontSize, unit);
}

/**
 * Draw with shadow effect (draws twice: shadow first, then main)
 */
function drawWithShadow(drawCall) {
    if (style.hasShadow) {
        ctx.save();
        ctx.lineWidth = style.shadow.lineWidth;
        ctx.strokeStyle = style.shadow.color;
        ctx.fillStyle = style.shadow.color;
        ctx.translate(style.shadow.offset, style.shadow.offset);
        drawCall();
        ctx.restore();
    }
    drawCall();
}

/**
 * Initialize HUD renderer
 */
export function initHUD(canvasElement) {
    canvas = canvasElement;
    ctx = canvas ? canvas.getContext('2d') : null;
    hudDPR = window.devicePixelRatio || 1;
}

/**
 * Initialize optional G-load widget canvas
 */
export function initGLoadWidget(canvasElement) {
    gCanvas = canvasElement;
    gCtx = gCanvas ? gCanvas.getContext('2d') : null;
}

export function resizeGLoadWidget() {
    if (!gCanvas || !gCtx) return;
    hudDPR = window.devicePixelRatio || 1;
    const w = gCanvas.clientWidth || 0;
    const h = gCanvas.clientHeight || 0;
    if (w <= 0 || h <= 0) return;

    gCanvas.width = Math.max(1, Math.floor(w * hudDPR));
    gCanvas.height = Math.max(1, Math.floor(h * hudDPR));
}

export function drawGLoadWidget() {
    if (!gCanvas || !gCtx) return;

    const cssW = gCanvas.clientWidth || 0;
    const cssH = gCanvas.clientHeight || 0;
    if (cssW <= 0 || cssH <= 0) return;

    if (gCanvas.width === 0 || gCanvas.height === 0) {
        resizeGLoadWidget();
    }

    const dpr = hudDPR;
    const w = cssW;
    const h = cssH;

    const accentRed = getCssVar('--accent-red', '#ff0000');
    const textMain = getCssVar('--text-main', '#ffffff');
    const textDim = getCssVar('--text-dim', '#6e7f8d');

    gCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
    gCtx.clearRect(0, 0, w, h);

    const pad = Math.max(6, Math.min(14, Math.floor(Math.min(w, h) * 0.08)));
    const x = pad;
    const y = pad;
    const gw = Math.max(10, w - pad * 2);
    const gh = Math.max(10, h - pad * 2);

    const minG = -3;
    const maxG = 9;
    const px = gh / (maxG - minG);
    const y0 = y + (maxG * px);

    gCtx.save();
    gCtx.strokeStyle = textDim;
    gCtx.globalAlpha = 0.35;
    gCtx.lineWidth = 1;
    const lines = 5;
    for (let i = 1; i < lines; i++) {
        const yy = y + (gh * i) / lines;
        gCtx.beginPath();
        gCtx.moveTo(x, yy);
        gCtx.lineTo(x + gw, yy);
        gCtx.stroke();
    }
    gCtx.restore();

    gCtx.save();
    gCtx.strokeStyle = textMain;
    gCtx.globalAlpha = 0.35;
    gCtx.lineWidth = 1.5;
    gCtx.beginPath();
    gCtx.moveTo(x, y0);
    gCtx.lineTo(x + gw, y0);
    gCtx.stroke();
    gCtx.restore();

    const n = gHistoryBuffer.length || 0;
    if (n > 1) {
        const step = gw / (n - 1);
        // Reuse pooled objects instead of allocating new ones
        const pts = gLoadPointPool;
        for (let i = 0; i < n; i++) {
            const gx = x + i * step;
            let gy = y0 - (gHistoryBuffer.get(i) * px);
            gy = Math.max(y, Math.min(y + gh, gy));
            pts[i].x = gx;
            pts[i].y = gy;
        }

        gCtx.save();
        gCtx.fillStyle = accentRed;
        gCtx.globalAlpha = 0.12;
        gCtx.beginPath();
        gCtx.moveTo(pts[0].x, y0);
        gCtx.lineTo(pts[0].x, pts[0].y);
        for (let i = 1; i < n; i++) {
            gCtx.lineTo(pts[i].x, pts[i].y);
        }
        gCtx.lineTo(pts[n - 1].x, y0);
        gCtx.closePath();
        gCtx.fill();
        gCtx.restore();

        gCtx.save();
        gCtx.strokeStyle = accentRed;
        gCtx.lineWidth = 2;
        gCtx.lineJoin = 'round';
        gCtx.lineCap = 'round';
        gCtx.beginPath();
        gCtx.moveTo(pts[0].x, pts[0].y);
        for (let i = 1; i < n - 1; i++) {
            const midX = (pts[i].x + pts[i + 1].x) / 2;
            const midY = (pts[i].y + pts[i + 1].y) / 2;
            gCtx.quadraticCurveTo(pts[i].x, pts[i].y, midX, midY);
        }
        gCtx.lineTo(pts[n - 1].x, pts[n - 1].y);
        gCtx.stroke();
        gCtx.restore();

        const cur = gHistoryBuffer.get(n - 1);
        const curTxt = Number.isFinite(cur) ? `${cur.toFixed(2)}g` : '--';
        gCtx.save();
        gCtx.font = `${Math.max(10, Math.floor(Math.min(w, h) * 0.11))}px ${getCssVar('--font-data', 'monospace')}`;
        gCtx.fillStyle = textMain;
        gCtx.globalAlpha = 0.9;
        gCtx.textAlign = 'right';
        gCtx.textBaseline = 'top';
        gCtx.fillText(curTxt, x + gw, y - Math.max(0, pad - 4));
        gCtx.restore();

        gCtx.save();
        gCtx.font = `${Math.max(9, Math.floor(Math.min(w, h) * 0.09))}px ${getCssVar('--font-data', 'monospace')}`;
        gCtx.fillStyle = textDim;
        gCtx.globalAlpha = 0.85;
        gCtx.textAlign = 'left';
        gCtx.textBaseline = 'top';
        gCtx.fillText('+9g', x, y - Math.max(0, pad - 4));
        gCtx.textBaseline = 'bottom';
        gCtx.fillText('-3g', x, y + gh + Math.max(0, pad - 4));
        gCtx.restore();
    }
}

/**
 * Resize HUD canvas
 */
export function resizeHUD() {
    hudDPR = window.devicePixelRatio || 1;
    refreshCachedCss();

    let w, h;
    const wrapper = hudWrapperEl || (hudWrapperEl = document.getElementById('hud-wrapper'));
    if (wrapper) {
        w = wrapper.clientWidth;
        h = wrapper.clientHeight;
    } else {
        w = window.innerWidth;
        h = window.innerHeight;
    }

    size.width = w / style.scale;
    size.height = h / style.scale;

    canvas.width = w * hudDPR * settings.scale;
    canvas.height = h * hudDPR * settings.scale;
    canvas.style.width = w + 'px';
    canvas.style.height = h + 'px';

    return { width: w, height: h };
}

// Horizon-lock mode, set from main.js: the first-person camera pitch is
// held at zero so the horizon is always in frame regardless of attitude.
// The ladder no longer shifts with pitch (the camera already stays level),
// so the boresight/aircraft-reference symbol moves instead — it becomes the
// mobile element that shows true pitch against the fixed horizon.
let pitchLocked = false;
export function setHudPitchLocked(locked) { pitchLocked = !!locked; }

// Conformal angle→screen projection. The 3D scene behind the HUD is rendered
// with a perspective camera (CAMERA_FOV vertical), so a symbol at angular
// offset θ from the boresight lands on the world at focal·tan(θ), not the
// linear focal·θ. _pixelPerRad is the focal length (px per rad at boresight),
// tuned to the FOV; the linear form only holds near center and drifts off the
// real world at large offsets — e.g. the ground-track marker under a big
// wind-crab angle, or the boresight at high pitch in horizon-lock.
function projectAngle(rad) {
    return Math.tan(rad) * settings._pixelPerRad;
}

/**
 * Draw flight path marker - shows where aircraft is going.
 * Standard HUD symbology (MIL-STD-1787 / HGS): circle with wings and fin.
 */
function drawFlightPath(x, y) {
    ctx.translate(x, y);

    const r = 8;      // ring radius
    const wing = 12;  // wing stub length
    const fin = 8;    // vertical fin length

    ctx.beginPath();
    ctx.arc(0, 0, r, 0, Math.PI * 2);
    ctx.moveTo(r, 0);
    ctx.lineTo(r + wing, 0);
    ctx.moveTo(-r, 0);
    ctx.lineTo(-r - wing, 0);
    ctx.moveTo(0, -r);
    ctx.lineTo(0, -r - fin);
    ctx.stroke();

    ctx.translate(-x, -y);
}

/**
 * Draw inertial/ground-track marker - shows the GPS-only trajectory
 * (track vs heading, flight-path angle vs pitch), independent of the
 * body-frame AoA/SSA rotation used for the main FPM. HGS-style diamond,
 * open (no wings/fin) so it reads as distinct from the primary FPM.
 */
function drawGroundTrackMarker(x, y) {
    ctx.translate(x, y);

    const d = 7; // half-diagonal

    ctx.beginPath();
    ctx.moveTo(0, -d);
    ctx.lineTo(d, 0);
    ctx.lineTo(0, d);
    ctx.lineTo(-d, 0);
    ctx.closePath();
    ctx.stroke();

    ctx.translate(-x, -y);
}

/**
 * Draw HGS-style speed error worm on the FPM left wing.
 * Bar grows upward = faster than commanded airspeed, downward = slower.
 * Only drawn while the nav controller is providing guidance.
 */
function drawSpeedError(x, y) {
    const err = -STATE.aspdError; // aspd_error = target - current → + means fast
    const len = Math.max(-40, Math.min(40, err * 6));
    if (Math.abs(len) < 4) return;

    ctx.save();
    ctx.translate(x, y);
    ctx.lineWidth = style.lineWidth + 2;
    ctx.beginPath();
    ctx.moveTo(-20, 0);
    ctx.lineTo(-20, -len);
    ctx.stroke();
    ctx.restore();
}

/**
 * Draw boresight/waterline marker - shows where aircraft nose is pointing (fixed at center)
 */
function drawBoresight() {
    const wingLen = 25;
    const wingGap = 8;
    const centerDot = 3;
    const dropLine = 12;

    ctx.beginPath();

    // Center dot
    ctx.arc(0, 0, centerDot, 0, Math.PI * 2);

    // Left wing
    ctx.moveTo(-wingGap, 0);
    ctx.lineTo(-wingGap - wingLen, 0);
    ctx.lineTo(-wingGap - wingLen, dropLine);

    // Right wing
    ctx.moveTo(wingGap, 0);
    ctx.lineTo(wingGap + wingLen, 0);
    ctx.lineTo(wingGap + wingLen, dropLine);

    ctx.stroke();
}

/**
 * Draw horizon ladder
 */
function drawHorizonLadder(x, y) {
    ctx.translate(x, y);

    const length = 460;
    const space = 80;
    const q = 12;

    ctx.beginPath();

    // Right side
    ctx.moveTo(space / 2, 0);
    ctx.lineTo(length / 2 - q, 0);
    ctx.lineTo(length / 2, q);

    // Left side
    ctx.moveTo(-space / 2, 0);
    ctx.lineTo(-(length / 2 - q), 0);
    ctx.lineTo(-length / 2, q);

    ctx.stroke();

    ctx.translate(-x, -y);
}

/**
 * Draw pitch ladder at given degree
 */
function drawPitchLadder(x, y, value) {
    ctx.translate(x, y);

    const length = 200;
    const space = 80;
    const q = 12;

    // Below-horizon rungs are dashed (standard PFD/HUD convention)
    if (value < 0) ctx.setLineDash([10, 7]);

    ctx.beginPath();

    // Right ladder
    ctx.moveTo(space / 2, 0);
    ctx.lineTo(length / 2 - q, 0);
    ctx.lineTo(length / 2, value > 0 ? q : -q);

    // Left ladder
    ctx.moveTo(-space / 2, 0);
    ctx.lineTo(-(length / 2 - q), 0);
    ctx.lineTo(-length / 2, value > 0 ? q : -q);

    ctx.stroke();
    ctx.setLineDash([]);

    // Labels
    setFontScale(16, 'px');
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';

    const textBorder = 4;
    const textWidth = ctx.measureText('-90').width;

    ctx.fillText(value, length / 2 + textBorder + textWidth, value > 0 ? q / 2 : -q / 2);
    ctx.fillText(value, -(length / 2 + textBorder), value > 0 ? q / 2 : -q / 2);

    ctx.translate(-x, -y);
}

/**
 * Draw vertical scale (speed or altitude)
 */
function drawVerticalScale(x, y, value, exampleValue, stepRange, right) {
    ctx.save();
    ctx.translate(x, y);

    const mf = right ? -1 : 1;

    // Value indicator box
    const fontSize = 20 * style.font.scale;
    setFont(fontSize, 'px');

    const textSideBorder = 5;
    const textTopBorder = 4;
    const textWidth = ctx.measureText(exampleValue).width;

    const height = fontSize + 2 * textTopBorder;
    const length = textSideBorder * 2 + textWidth + height / 2;

    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';

    // Draw indicator box
    ctx.beginPath();
    ctx.moveTo(0, -height / 2);
    ctx.lineTo(mf * (textSideBorder * 2 + textWidth), -height / 2);
    ctx.lineTo(mf * length, 0);
    ctx.lineTo(mf * (textSideBorder * 2 + textWidth), height / 2);
    ctx.lineTo(0, height / 2);
    ctx.closePath();
    ctx.stroke();

    const text = Math.round(value);
    ctx.fillText(text, right ? -textSideBorder : textSideBorder + textWidth, 0);

    // Scale ticks
    const scaleFontSize = 16 * style.font.scale;
    setFont(scaleFontSize, 'px');
    const textBorder = 3;
    const border = 4;
    const stepLength = [16, 11, 7];

    if (!right) ctx.textAlign = 'left';

    ctx.translate(mf * (length + border), 0);

    // Clip region
    ctx.rect(
        0,
        -((stepRange * style.stepWidth) / 2),
        mf * (stepLength[0] + 2 * textBorder + ctx.measureText(exampleValue + '9').width),
        stepRange * style.stepWidth
    );
    ctx.clip();

    const stepMargin = 5;
    const stepZeroOffset = Math.ceil(stepRange / 2) + stepMargin;
    const stepValueOffset = Math.floor(value);
    const stepOffset = value - stepValueOffset;

    ctx.translate(0, (stepZeroOffset + stepOffset) * style.stepWidth);

    ctx.beginPath();
    for (let i = -stepZeroOffset + stepValueOffset; i < stepZeroOffset + stepValueOffset; i++) {
        ctx.moveTo(0, 0);
        switch (Math.abs(i) % 10) {
            case 0:
                ctx.lineTo(mf * stepLength[0], 0);
                ctx.fillText(i, mf * (stepLength[0] + textBorder), 0);
                break;
            case 5:
                ctx.lineTo(mf * stepLength[1], 0);
                break;
            default:
                ctx.lineTo(mf * stepLength[2], 0);
                break;
        }
        ctx.translate(0, -style.stepWidth);
    }
    ctx.stroke();

    ctx.restore();
}

/**
 * Draw heading indicator (fixed at top)
 */
function drawHeading(x, y, stepRange, bottom) {
    ctx.save();
    ctx.translate(x, y);

    const mf = bottom ? -1 : 1;
    const value = STATE.yaw * (180 / Math.PI);

    // Value indicator box
    const fontSize = 20 * style.font.scale;
    setFont(fontSize, 'px');

    const textSideBorder = 5;
    const textTopBorder = 4;
    const textWidth = ctx.measureText('360').width;

    const length = textSideBorder * 2 + textWidth;
    const height = textTopBorder * 1.5 + fontSize + length / 4;

    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';

    ctx.beginPath();
    ctx.moveTo(-length / 2, 0);
    ctx.lineTo(length / 2, 0);
    ctx.lineTo(length / 2, mf * (textTopBorder * 1.5 + fontSize));
    ctx.lineTo(0, mf * height);
    ctx.lineTo(-length / 2, mf * (textTopBorder * 1.5 + fontSize));
    ctx.closePath();
    ctx.stroke();

    let hdgValue = Math.round(value);
    if (hdgValue < 0) hdgValue += 360;
    hdgValue = hdgValue % 360;
    ctx.fillText(hdgValue, textWidth / 2, (mf * (2 * textTopBorder + fontSize)) / 2);

    // Scale
    const scaleFontSize = 16 * style.font.scale;
    setFont(scaleFontSize, 'px');
    const textBorder = 2;
    const border = 4;
    const stepLength = [16, 11, 7];

    ctx.textAlign = 'center';
    ctx.translate(0, mf * (height + border));

    // Clip
    ctx.rect(
        (-stepRange * style.stepWidth) / 2,
        0,
        style.stepWidth * stepRange,
        mf * (stepLength[0] + 2 * textBorder + scaleFontSize)
    );
    ctx.clip();

    // Ground-track marker: hollow diamond on the tape showing drift/crab
    {
        const trackDeg = STATE.track * (180 / Math.PI);
        let tDelta = trackDeg - value;
        while (tDelta > 180) tDelta -= 360;
        while (tDelta < -180) tDelta += 360;
        const tx = tDelta * style.stepWidth;
        const ty = mf * 8;
        ctx.save();
        ctx.beginPath();
        ctx.moveTo(tx, ty - 6);
        ctx.lineTo(tx + 5, ty);
        ctx.lineTo(tx, ty + 6);
        ctx.lineTo(tx - 5, ty);
        ctx.closePath();
        ctx.stroke();
        ctx.restore();
    }

    // Nav-bearing bug: filled caret at the autopilot commanded course
    // (accent-cyan, like a Garmin heading bug) while guidance is active
    const navFresh = STATE.navDataTime > 0 && Date.now() - STATE.navDataTime < 3000;
    if (navFresh && STATE.wpDist > 0) {
        let delta = STATE.navBearing - value;
        while (delta > 180) delta -= 360;
        while (delta < -180) delta += 360;
        const bx = delta * style.stepWidth;
        ctx.save();
        ctx.fillStyle = getCssVar('--accent-cyan', '#00d2ff');
        ctx.beginPath();
        ctx.moveTo(bx, mf * 2);
        ctx.lineTo(bx - 6, mf * 13);
        ctx.lineTo(bx + 6, mf * 13);
        ctx.closePath();
        ctx.fill();
        ctx.restore();
    }

    const stepMargin = 5;
    const stepZeroOffset = Math.ceil(stepRange / 2) + stepMargin;
    const stepValueOffset = Math.floor(value);
    const stepOffset = value - stepValueOffset;

    ctx.translate(-(stepZeroOffset + stepOffset) * style.stepWidth, 0);

    ctx.beginPath();
    for (let i = -stepZeroOffset + stepValueOffset; i < stepZeroOffset + stepValueOffset; i++) {
        const posI = Math.abs(i);

        ctx.moveTo(0, 0);
        switch (posI % 10) {
            case 0:
                ctx.lineTo(0, mf * stepLength[0]);
                break;
            case 5:
                ctx.lineTo(0, mf * stepLength[1]);
                break;
            default:
                ctx.lineTo(0, mf * stepLength[2]);
                break;
        }

        // Labels at cardinal and 10-degree marks
        if (posI % 90 === 0 || posI % 45 === 0 || posI % 10 === 0) {
            let labelText;
            const mod = ((i % 360) + 360) % 360;
            switch (mod) {
                case 0: labelText = 'N'; break;
                case 45: labelText = 'NE'; break;
                case 90: labelText = 'E'; break;
                case 135: labelText = 'SE'; break;
                case 180: labelText = 'S'; break;
                case 225: labelText = 'SW'; break;
                case 270: labelText = 'W'; break;
                case 315: labelText = 'NW'; break;
                default:
                    labelText = i >= 0 ? (i % 360) : (360 + (i % 360));
                    break;
            }
            ctx.fillText(labelText, 0, mf * (stepLength[0] + textBorder + scaleFontSize / 2));
        }

        ctx.translate(style.stepWidth, 0);
    }
    ctx.stroke();

    ctx.restore();
}

/**
 * Draw bank angle indicator (Garmin/Boeing PFD style):
 * fixed tick scale at ±10/20/30/45/60° around the boresight, a fixed
 * zero-reference triangle, a sky pointer that rotates with the horizon,
 * and a slip/skid trapezoid beneath it driven by lateral acceleration.
 */
function drawBankArc() {
    const DEG = Math.PI / 180;
    const radius = Math.min(180, size.height * 0.28);

    ctx.save();
    ctx.translate(size.width / 2, size.height / 2);

    // Fixed scale ticks
    ctx.beginPath();
    const ticks = [-60, -45, -30, -20, -10, 10, 20, 30, 45, 60];
    for (const t of ticks) {
        const len = (Math.abs(t) % 30 === 0) ? 14 : (Math.abs(t) === 45 ? 10 : 7);
        ctx.save();
        ctx.rotate(t * DEG);
        ctx.moveTo(0, -radius);
        ctx.lineTo(0, -radius - len);
        ctx.restore();
    }
    ctx.stroke();

    // Fixed zero-reference triangle (apex down, outside the arc)
    ctx.beginPath();
    ctx.moveTo(0, -radius - 2);
    ctx.lineTo(-7, -radius - 15);
    ctx.lineTo(7, -radius - 15);
    ctx.closePath();
    ctx.stroke();

    // Sky pointer: rotates with the horizon (same convention as pitch ladder)
    ctx.rotate(-STATE.roll);
    ctx.beginPath();
    ctx.moveTo(0, -radius + 3);
    ctx.lineTo(-7, -radius + 16);
    ctx.lineTo(7, -radius + 16);
    ctx.closePath();
    ctx.fill();

    // Slip/skid trapezoid: deflects laterally with body lateral acceleration
    // (centered under the pointer in coordinated flight)
    const skid = Math.max(-1, Math.min(1, -STATE.ay / 9.81 * 2)) * 16;
    ctx.translate(skid, 0);
    ctx.beginPath();
    ctx.moveTo(-8, -radius + 20);
    ctx.lineTo(8, -radius + 20);
    ctx.lineTo(10, -radius + 27);
    ctx.lineTo(-10, -radius + 27);
    ctx.closePath();
    ctx.fill();

    ctx.restore();
}

/**
 * Draw vertical speed indicator: compact non-linear tape right of the
 * pitch ladder with a pointer and digital readout (Garmin-style).
 * ±5 m/s occupies most of the scale, 5-10 m/s is compressed.
 */
function drawVSI() {
    const cx = size.width / 2;
    const cy = size.height / 2;
    const x = Math.min(cx + 320, size.width - 150);
    const H = Math.min(110, size.height * 0.16);
    const maxV = 10;

    const mapY = (v) => {
        const a = Math.min(Math.abs(v), maxV);
        const frac = a <= 5 ? (a / 5) * 0.72 : 0.72 + ((a - 5) / 5) * 0.28;
        return -Math.sign(v) * frac * H;
    };

    ctx.save();
    ctx.translate(x, cy);

    // Baseline and ticks (ticks to the left of the line)
    ctx.beginPath();
    ctx.moveTo(0, -H);
    ctx.lineTo(0, H);
    for (const v of [-10, -5, -2, -1, 1, 2, 5, 10]) {
        const len = (Math.abs(v) >= 5) ? 10 : 6;
        ctx.moveTo(0, mapY(v));
        ctx.lineTo(-len, mapY(v));
    }
    ctx.moveTo(0, 0);
    ctx.lineTo(-12, 0);
    ctx.stroke();

    // Tick labels
    setFontScale(12, 'px');
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    ctx.fillText('5', -14, mapY(5));
    ctx.fillText('5', -14, mapY(-5));

    // Pointer + digital value
    const vs = Number.isFinite(STATE.vs) ? STATE.vs : 0;
    const vy = mapY(vs);
    ctx.beginPath();
    ctx.moveTo(0, vy);
    ctx.lineTo(9, vy - 5);
    ctx.lineTo(9, vy + 5);
    ctx.closePath();
    ctx.fill();

    setFontScale(14, 'px');
    ctx.textAlign = 'left';
    ctx.fillText(vs.toFixed(1), 13, vy);

    setFontScale(10, 'px');
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    ctx.fillText('M/S', 0, H + 6);

    ctx.restore();
}

/**
 * Draw flight mode annunciator (persistent, top-center under the heading
 * tape — Airbus/Boeing FMA position) plus active-waypoint guidance line.
 */
function drawFMA() {
    const cx = size.width / 2;
    const y = 96;

    ctx.save();
    setFontScale(15, 'px');
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    ctx.fillText(STATE.flightMode || '---', cx, y);

    const navFresh = STATE.navDataTime > 0 && Date.now() - STATE.navDataTime < 3000;
    if (navFresh && STATE.wpDist > 0) {
        setFontScale(12, 'px');
        ctx.globalAlpha = 0.85;
        const xtk = STATE.xtrackError;
        ctx.fillText(`WP ${Math.round(STATE.wpDist)}m   XTK ${xtk >= 0 ? '+' : ''}${xtk.toFixed(0)}m`, cx, y + 20);
    }
    ctx.restore();
}

/**
 * Draw wind vector (Garmin-style): arrow showing where the wind blows
 * relative to the nose, with horizontal speed readout. Left of the FMA.
 */
function drawWind() {
    if (STATE.windDataTime === 0 || Date.now() - STATE.windDataTime > 5000) return;
    if (!Number.isFinite(STATE.windSpeed) || STATE.windSpeed < 0.5) return;

    const DEG = Math.PI / 180;
    const x = Math.max(20, size.width / 2 - 290);
    const y = 104;
    const headingDeg = STATE.yaw * (180 / Math.PI);
    // WIND.direction is where the wind comes FROM; arrow points where it goes
    const rot = (STATE.windDir + 180 - headingDeg) * DEG;

    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(rot);
    ctx.beginPath();
    ctx.moveTo(0, 10);
    ctx.lineTo(0, -10);
    ctx.moveTo(-4, -5);
    ctx.lineTo(0, -10);
    ctx.lineTo(4, -5);
    ctx.stroke();
    ctx.restore();

    setFontScale(12, 'px');
    ctx.textAlign = 'left';
    ctx.textBaseline = 'middle';
    ctx.fillText(`${STATE.windSpeed.toFixed(1)} M/S`, x + 12, y);
}

/**
 * Draw time display
 */
function drawTime(x, y) {
    ctx.translate(x, y);

    setFontScale(16, 'px');
    ctx.textAlign = 'left';
    ctx.textBaseline = 'top';

    const now = new Date();
    ctx.fillText(
        now.toLocaleTimeString(undefined, {
            timeZone: settings.timezone,
            hour12: false,
            hourCycle: 'h23',
            hour: '2-digit',
            minute: '2-digit',
            second: '2-digit',
        }),
        0,
        0
    );

    ctx.translate(-x, -y);
}

/**
 * Push a message to the HUD message queue
 */
export function pushHudMessage(text, level = 'info') {
    hudMessages.push({ text, level, time: performance.now() });
    if (hudMessages.length > MAX_HUD_MESSAGES) hudMessages.shift();
}

/**
 * Draw HUD status messages and ARM/DISARM state
 */
function drawHudMessages() {
    const now = performance.now();

    // Remove expired messages
    while (hudMessages.length > 0 && now - hudMessages[0].time > HUD_MSG_DURATION) {
        hudMessages.shift();
    }

    // Detect arm state change
    if (STATE.armed !== prevArmedState) {
        if (STATE.armed) {
            pushHudMessage('ARMED', 'warning');
        }
        prevArmedState = STATE.armed;
    }

    // Update disarmed label visibility (HTML element, not canvas)
    if (!disarmedLabelEl) disarmedLabelEl = document.getElementById('disarmed-label');
    if (disarmedLabelEl) {
        disarmedLabelEl.classList.toggle('hidden', STATE.armed);
    }

    // Draw message list (bottom-left, like Mission Planner)
    if (hudMessages.length > 0) {
        ctx.save();
        const fontSize = Math.max(8, Math.min(10, size.width * 0.009));
        setFont(fontSize, 'px');
        ctx.textAlign = 'left';
        ctx.textBaseline = 'bottom';

        const x = 12;
        let y = size.height - 60;

        for (let i = hudMessages.length - 1; i >= 0; i--) {
            const msg = hudMessages[i];
            const age = now - msg.time;
            const fadeAlpha = age > HUD_MSG_DURATION - 1000
                ? Math.max(0, (HUD_MSG_DURATION - age) / 1000)
                : 1;

            let color;
            switch (msg.level) {
                case 'error': color = `rgba(255, 60, 60, ${fadeAlpha})`; break;
                case 'warning': color = `rgba(255, 200, 50, ${fadeAlpha})`; break;
                default: color = `rgba(200, 255, 200, ${fadeAlpha})`; break;
            }

            ctx.fillStyle = 'rgba(0, 0, 0, ' + (0.4 * fadeAlpha) + ')';
            const textWidth = ctx.measureText(msg.text).width;
            ctx.fillRect(x - 4, y - fontSize - 2, textWidth + 8, fontSize + 4);

            ctx.fillStyle = color;
            ctx.fillText(msg.text, x, y);
            y -= fontSize + 6;
        }
        ctx.restore();
    }
}

/**
 * Draw the HUD overlay
 */
export function drawHUD() {
    if (!ctx) return;

    const dpr = hudDPR;
    const scale = dpr * style.scale * settings.scale;

    let w, h;
    const wrapper = hudWrapperEl || (hudWrapperEl = document.getElementById('hud-wrapper'));
    if (wrapper) {
        w = wrapper.clientWidth;
        h = wrapper.clientHeight;
    } else {
        w = window.innerWidth;
        h = window.innerHeight;
    }

    // Update virtual size
    size.width = w / style.scale;
    size.height = h / style.scale;

    // Check if resize needed
    if (
        Math.floor(h * dpr * settings.scale) !== canvas.height ||
        Math.floor(w * dpr * settings.scale) !== canvas.width
    ) {
        canvas.width = Math.floor(w * dpr * settings.scale);
        canvas.height = Math.floor(h * dpr * settings.scale);
        canvas.style.width = w + 'px';
        canvas.style.height = h + 'px';
    }

    ctx.setTransform(scale, 0, 0, scale, 0, 0);
    ctx.clearRect(0, 0, size.width, size.height);

    // Set default style — HUD color shifts red/orange when disarmed
    ctx.lineWidth = style.lineWidth;
    if (STATE.armed) {
        ctx.strokeStyle = style.color;
        ctx.fillStyle = style.color;
    } else {
        armedFlashTimer += 0.03;
        const t = (Math.sin(armedFlashTimer * 2.5) + 1) / 2; // 0..1
        const r = 255;
        const g = Math.round(40 + t * 100); // 40..140 (red → orange)
        const b = Math.round(t * 20);       // 0..20
        const disarmedColor = `rgba(${r}, ${g}, ${b}, 1)`;
        ctx.strokeStyle = disarmedColor;
        ctx.fillStyle = disarmedColor;
    }

    // === DYNAMIC UI (center, rotated with aircraft) ===
    ctx.translate(size.width / 2, size.height / 2);

    // Get state values (all in radians)
    const pitch = STATE.pitch;
    const roll = STATE.roll;
    const flightPitch = STATE.gamma;  // flight path angle
    const flightHeading = STATE.ssa;  // sideslip angle
    // Ladder shift for the current pitch reference: normally the camera
    // (and thus the world horizon) tracks true pitch, so the ladder shifts
    // to stay conformal. Locked, the camera never tilts, so the ladder
    // stays put and the boresight moves instead (see below).
    const pitchShift = pitchLocked ? 0 : pitch;

    // Flight path marker
    // AoA (alpha) = pitch - gamma, so FPM should be at -AoA relative to boresight
    // When alpha increases (nose up relative to flight path), FPM moves DOWN
    // FPM is air-relative in both modes (AoA/SSA, wind subtracted upstream).
    // Position relative to the horizon must stay (aoa - pitch); since the
    // horizon sits at pitchShift, the offset from center is aoa - pitch +
    // pitchShift — which reduces to aoa unlocked and aoa - pitch when locked.
    // Using flightPitch (ground gamma) here would drop wind from the vertical
    // and overlap the ground-track diamond in horizon-lock.
    let fpmX = projectAngle(flightHeading);
    let fpmY = projectAngle(STATE.aoa - pitch + pitchShift);
    // Cage the FPM at the edge of the field of view (HGS style): clamp to
    // the visible area and draw it dashed while caged.
    const cageX = size.width * 0.35;
    const cageY = size.height * 0.35;
    const fpmCaged = Math.abs(fpmX) > cageX || Math.abs(fpmY) > cageY;
    if (fpmCaged) {
        fpmX = Math.max(-cageX, Math.min(cageX, fpmX));
        fpmY = Math.max(-cageY, Math.min(cageY, fpmY));
    }
    drawWithShadow(() => {
        if (fpmCaged) ctx.setLineDash([4, 4]);
        drawFlightPath(fpmX, fpmY);
        if (fpmCaged) ctx.setLineDash([]);
    });

    // Ground-track marker (inertial/GPS-only trajectory): track vs heading
    // for the horizontal offset, flight-path angle vs pitch for the
    // vertical one — no body-frame AoA/SSA rotation, so it reads the same
    // in a bank regardless of roll coupling. Diamond, open, to read as
    // distinct from the AoA/SSA-based FPM above.
    let trackDiff = STATE.track - STATE.yaw;
    trackDiff = Math.atan2(Math.sin(trackDiff), Math.cos(trackDiff)); // wrap
    let gtmX = projectAngle(trackDiff);
    let gtmY = projectAngle(pitchShift - flightPitch);
    const gtmCaged = Math.abs(gtmX) > cageX || Math.abs(gtmY) > cageY;
    if (gtmCaged) {
        gtmX = Math.max(-cageX, Math.min(cageX, gtmX));
        gtmY = Math.max(-cageY, Math.min(cageY, gtmY));
    }
    drawWithShadow(() => {
        if (gtmCaged) ctx.setLineDash([4, 4]);
        drawGroundTrackMarker(gtmX, gtmY);
        if (gtmCaged) ctx.setLineDash([]);
    });

    // Speed error worm on FPM left wing (only with active nav guidance)
    if (STATE.navDataTime > 0 && Date.now() - STATE.navDataTime < 3000) {
        drawWithShadow(() => {
            drawSpeedError(fpmX, fpmY);
        });
    }

    // Pitch ladders (rotated and translated)
    // Note: roll is negated to match aircraft visual frame (bank right = horizon rotates left)
    // Locked, the ladder doesn't shift with pitch — the camera stays level so
    // the horizon is always in frame, and the boresight moves instead.
    drawWithShadow(() => {
        ctx.rotate(-roll);
        ctx.translate(0, pitchShift * settings._pixelPerRad);

        drawHorizonLadder(0, 0);

        const pitchDegStep = 10;
        for (let deg = pitchDegStep; deg <= 90; deg += pitchDegStep) {
            drawPitchLadder(0, -(deg * settings._pixelPerDeg), deg);
        }
        for (let deg = -pitchDegStep; deg >= -90; deg -= pitchDegStep) {
            drawPitchLadder(0, -(deg * settings._pixelPerDeg), deg);
        }
    });

    ctx.setTransform(scale, 0, 0, scale, 0, 0);

    // === FIXED UI ===

    // Boresight marker - shows aircraft nose direction. Fixed at center
    // normally; in horizon-lock mode the camera stays level so the marker
    // itself moves vertically to show true pitch against the fixed horizon.
    // Caged at the edge of the frame like the FPV markers, dashed while caged.
    let boresightYOffset = pitchLocked ? -projectAngle(pitch) : 0;
    const boresightCaged = pitchLocked && Math.abs(boresightYOffset) > cageY;
    if (boresightCaged) {
        boresightYOffset = Math.max(-cageY, Math.min(cageY, boresightYOffset));
    }
    ctx.translate(size.width / 2, size.height / 2 + boresightYOffset);
    drawWithShadow(() => {
        if (boresightCaged) ctx.setLineDash([4, 4]);
        drawBoresight();
        if (boresightCaged) ctx.setLineDash([]);
    });
    ctx.setTransform(scale, 0, 0, scale, 0, 0);

    const border = 16;

    // Heading (top)
    drawWithShadow(() => {
        drawHeading(size.width / 2, border, 61, false);
    });

    // Bank angle scale + sky pointer + slip/skid (replaces bottom roll arc)
    drawWithShadow(() => {
        drawBankArc();
    });

    // Vertical speed indicator (right of pitch ladder)
    drawWithShadow(() => {
        drawVSI();
    });

    // Flight mode annunciator + waypoint guidance (top-center)
    drawWithShadow(() => {
        drawFMA();
    });

    // Wind vector (left of FMA)
    drawWithShadow(() => {
        drawWind();
    });

    // HUD messages and ARM/DISARM state
    drawHudMessages();
}

// Export getters
export function getHudDPR() { return hudDPR; }
export function getCanvas() { return canvas; }
export function getContext() { return ctx; }
