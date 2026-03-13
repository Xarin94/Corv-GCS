/**
 * HUDRenderer.js - Canvas HUD Drawing
 * Handles the 2D canvas overlay for flight instruments
 * Graphics style ported from origin/main HUD class
 */

import { STATE } from '../core/state.js';

// HUD state
let canvas = null;
let ctx = null;
let hudDPR = 1;
let viewModeRef = 'FULLSCREEN';

// HUD message queue (Mission Planner style)
const hudMessages = [];
const MAX_HUD_MESSAGES = 5;
const HUD_MSG_DURATION = 5000; // ms
let prevArmedState = false;
let armedFlashTimer = 0;

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
    rollRadius: 'none',
    timezone: undefined,
    scale: 1,
};

// Virtual size (updated on resize)
let size = { width: 800, height: 600 };

function getCssVar(name, fallback) {
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

    const hist = STATE.gHistory || [];
    const n = hist.length || 0;
    if (n > 1) {
        const step = gw / (n - 1);
        // Reuse pooled objects instead of allocating new ones
        const pts = gLoadPointPool;
        for (let i = 0; i < n; i++) {
            const gx = x + i * step;
            let gy = y0 - (hist[i] * px);
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

        const cur = hist[n - 1];
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
 * Set view mode reference
 */
export function setViewMode(mode) {
    viewModeRef = mode;
}

/**
 * Resize HUD canvas
 */
export function resizeHUD() {
    hudDPR = window.devicePixelRatio || 1;

    let w, h;
    const wrapper = document.getElementById('hud-wrapper');
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

/**
 * Draw flight path marker (diamond shape) - shows where aircraft is going
 */
function drawFlightPath(x, y) {
    ctx.translate(x, y);

    const r = 12;

    // Diamond shape
    ctx.beginPath();
    ctx.moveTo(r, 0);
    ctx.lineTo(0, r);
    ctx.lineTo(-r, 0);
    ctx.lineTo(0, -r);
    ctx.closePath();

    // Wing lines
    const line = 9;
    ctx.moveTo(r, 0);
    ctx.lineTo(r + line, 0);
    ctx.moveTo(0, -r);
    ctx.lineTo(0, -r - line);
    ctx.moveTo(-r, 0);
    ctx.lineTo(-r - line, 0);

    ctx.stroke();

    ctx.translate(-x, -y);
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

    // Sub-degree dashed lines (-1, -2, -3 degrees)
    ctx.setLineDash([6, 4]);
    const subLength = 26;

    ctx.beginPath();
    for (let i = 0; i < 3; i++) {
        ctx.translate(0, settings._pixelPerDeg);
        ctx.moveTo(space / 2, 0);
        ctx.lineTo(space / 2 + subLength, 0);
        ctx.moveTo(-space / 2, 0);
        ctx.lineTo(-(space / 2 + subLength), 0);
    }
    ctx.stroke();

    ctx.setLineDash([]);
    ctx.translate(-x, -y - 3 * settings._pixelPerDeg);
}

/**
 * Draw pitch ladder at given degree
 */
function drawPitchLadder(x, y, value) {
    ctx.translate(x, y);

    const length = 200;
    const space = 80;
    const q = 12;

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
 * Draw roll indicator arc
 */
function drawRoll(x, y, stepRange, radius, bottom) {
    ctx.save();
    ctx.translate(x, y);

    const mf = bottom ? -1 : 1;
    const value = STATE.roll * (180 / Math.PI);

    // Value indicator
    const fontSize = 20 * style.font.scale;
    setFont(fontSize, 'px');

    const textSideBorder = 5;
    const textTopBorder = 4;
    const textWidth = ctx.measureText('180').width;

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

    ctx.fillText(Math.round(value), textWidth / 2, (mf * (2 * textTopBorder + fontSize)) / 2);

    // Arc scale
    const scaleFontSize = 16 * style.font.scale;
    setFont(scaleFontSize, 'px');
    const textBorder = 2;
    const border = 4;
    const stepLength = [16, 11, 7];

    ctx.textAlign = 'center';
    ctx.translate(0, mf * (height + border));

    if (settings.rollRadius === 'exact') {
        radius = (style.stepWidth * 180) / Math.PI;
    } else if (settings.rollRadius === 'center') {
        radius = size.height / 2 - (bottom ? size.height - y : y) - (height + border);
    }

    if (radius < 0) {
        ctx.restore();
        return;
    }

    ctx.translate(0, mf * radius);

    // Clip arc
    const angle = (stepRange * style.stepWidth) / radius;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.arc(0, 0, radius, (bottom ? 0.5 : 1.5) * Math.PI - angle / 2, (bottom ? 0.5 : 1.5) * Math.PI + angle / 2);
    ctx.closePath();
    ctx.clip();

    const stepMargin = 5;
    const stepZeroOffset = Math.ceil(stepRange / 2) + stepMargin;
    const stepValueOffset = Math.floor(value);
    const stepOffset = value - stepValueOffset;

    ctx.beginPath();
    for (let i = -stepZeroOffset + stepValueOffset; i < stepZeroOffset + stepValueOffset; i++) {
        ctx.rotate((mf * -(stepValueOffset - i + stepOffset) * style.stepWidth) / radius);
        ctx.translate(0, mf * -radius);

        ctx.moveTo(0, 0);
        switch (Math.abs(i) % 10) {
            case 0:
                ctx.lineTo(0, mf * stepLength[0]);
                let val = i % 360;
                if (val > 180 || val <= -180) {
                    val = val - Math.sign(i) * 360;
                }
                ctx.fillText(val, 0, mf * (stepLength[0] + textBorder + scaleFontSize / 2));
                break;
            case 5:
                ctx.lineTo(0, mf * stepLength[1]);
                break;
            default:
                ctx.lineTo(0, mf * stepLength[2]);
                break;
        }

        ctx.translate(0, mf * radius);
        ctx.rotate((mf * (stepValueOffset - i + stepOffset) * style.stepWidth) / radius);
    }
    ctx.stroke();

    ctx.restore();
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

    // Draw DISARMED flashing text (like Mission Planner)
    if (!STATE.armed) {
        armedFlashTimer += 0.05;
        const alpha = 0.4 + 0.6 * Math.abs(Math.sin(armedFlashTimer * 2));
        ctx.save();
        const fontSize = Math.max(20, Math.min(36, size.width * 0.035));
        setFont(fontSize, 'px');
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillStyle = `rgba(255, 80, 80, ${alpha})`;
        ctx.strokeStyle = 'rgba(0, 0, 0, 0.5)';
        ctx.lineWidth = 3;
        const y = size.height * 0.88;
        ctx.strokeText('DISARMED', size.width / 2, y);
        ctx.fillText('DISARMED', size.width / 2, y);
        ctx.restore();
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
    const wrapper = document.getElementById('hud-wrapper');
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

    // Set default style
    ctx.lineWidth = style.lineWidth;
    ctx.strokeStyle = style.color;
    ctx.fillStyle = style.color;

    // === DYNAMIC UI (center, rotated with aircraft) ===
    ctx.translate(size.width / 2, size.height / 2);

    // Get state values (all in radians)
    const pitch = STATE.pitch;
    const roll = STATE.roll;
    const flightPitch = STATE.gamma;  // flight path angle
    const flightHeading = STATE.ssa;  // sideslip angle

    // Flight path marker
    // AoA (alpha) = pitch - gamma, so FPM should be at -AoA relative to boresight
    // When alpha increases (nose up relative to flight path), FPM moves DOWN
    drawWithShadow(() => {
        drawFlightPath(
            flightHeading * settings._pixelPerRad,
            STATE.aoa * settings._pixelPerRad  // positive AoA = FPM below boresight
        );
    });

    // Pitch ladders (rotated and translated)
    // Note: roll is negated to match aircraft visual frame (bank right = horizon rotates left)
    drawWithShadow(() => {
        ctx.rotate(-roll);
        ctx.translate(0, pitch * settings._pixelPerRad);

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

    // Boresight marker (fixed at center - shows aircraft nose direction)
    ctx.translate(size.width / 2, size.height / 2);
    drawWithShadow(() => {
        drawBoresight();
    });
    ctx.setTransform(scale, 0, 0, scale, 0, 0);

    const border = 16;

    // Heading (top)
    drawWithShadow(() => {
        drawHeading(size.width / 2, border, 61, false);
    });

    // Roll (bottom)
    drawWithShadow(() => {
        drawRoll(size.width / 2, size.height - border, 51, 260, true);
    });

    // HUD messages and ARM/DISARM state
    drawHudMessages();
}

// Export getters
export function getHudDPR() { return hudDPR; }
export function getCanvas() { return canvas; }
export function getContext() { return ctx; }
