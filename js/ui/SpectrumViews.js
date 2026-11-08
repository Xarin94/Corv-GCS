/**
 * SpectrumViews.js - IMU vibration spectrum charts
 *
 * Two views of the spectra in core/SpectrumMonitor.js, each for the gyro or
 * the accelerometer:
 *   - live: the newest spectrum as a line, the 10 s peak hold as a thin line
 *     above it (GCS sidebar, FFT LIVE section)
 *   - waterfall: the last 60 s, frequency down the plot, time to the right,
 *     colour = level in dB (FFT bar on the left, ui/FftPanel.js)
 * Frequencies on a linear axis, 0 to 362.5 Hz.
 */

import {
    spectrumMonitor, FFT_BINS, FFT_DF, FFT_MAX_HZ, SPECTRUM_HISTORY_MS, binFreq, SENSORS
} from '../core/SpectrumMonitor.js';

const INK = { primary: '#f0efec', secondary: '#c3c2b7', muted: '#898781', grid: 'rgba(255,255,255,0.07)', axis: 'rgba(255,255,255,0.22)' };
const LINE_COLOR = '#3987e5';
const PEAK_COLOR = '#c98500';
const TICK_FONT = '10px "Roboto Mono", monospace';
const LABEL_FONT = '11px "Rajdhani", sans-serif';
const NO_DATA_HINT = 'HPF_FFT_ENABLE = 1 on the vehicle';
const FREQ_TICKS = [0, 50, 100, 150, 200, 250, 300, 350];

// Perceptual dark-to-bright map (inferno-like): low levels sink into the
// panel's black, strong lines stand out in yellow
const STOPS = [
    [0.00, [0, 0, 4]], [0.14, [40, 11, 84]], [0.29, [101, 21, 110]], [0.43, [159, 42, 99]],
    [0.57, [212, 72, 66]], [0.71, [245, 125, 21]], [0.86, [250, 193, 39]], [1.00, [252, 255, 164]]
];
const LUT = (() => {
    const lut = new Uint8ClampedArray(256 * 3);
    for (let i = 0; i < 256; i++) {
        const t = i / 255;
        let k = 1;
        while (k < STOPS.length - 1 && STOPS[k][0] < t) k++;
        const [t0, c0] = STOPS[k - 1], [t1, c1] = STOPS[k];
        const f = (t - t0) / (t1 - t0);
        for (let c = 0; c < 3; c++) lut[i * 3 + c] = c0[c] + f * (c1[c] - c0[c]);
    }
    return lut;
})();
export const COLORMAP_CSS = 'linear-gradient(90deg,' + STOPS.map(([t, c]) => `rgb(${c.join(',')}) ${t * 100}%`).join(',') + ')';

/** A canvas filling a box, sized by a ResizeObserver, with the pointer position. */
export class CanvasBox {
    constructor(box) {
        this.box = box;
        this.canvas = document.createElement('canvas');
        this.canvas.className = 'spec-canvas';
        box.appendChild(this.canvas);
        this.ctx = this.canvas.getContext('2d');
        this.w = 0; this.h = 0; this.hover = null;
        this.dirty = true;
        new ResizeObserver(entries => {
            const r = entries[0].contentRect;
            this.w = r.width; this.h = r.height;
            const dpr = window.devicePixelRatio || 1;
            this.canvas.width = Math.max(1, Math.round(r.width * dpr));
            this.canvas.height = Math.max(1, Math.round(r.height * dpr));
            this.dirty = true;
        }).observe(box);
        this.canvas.addEventListener('mousemove', e => { this.hover = { x: e.offsetX, y: e.offsetY }; this.dirty = true; });
        this.canvas.addEventListener('mouseleave', () => { this.hover = null; this.dirty = true; });
    }
    begin() {
        if (!(this.w > 0 && this.h > 0)) return false;
        const dpr = window.devicePixelRatio || 1;
        this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        this.ctx.clearRect(0, 0, this.w, this.h);
        return true;
    }
}

function freqAxis(ctx, X, y, pw) {
    ctx.font = TICK_FONT; ctx.fillStyle = INK.muted; ctx.textBaseline = 'alphabetic';
    const step = pw < 200 ? 100 : 50;
    for (const f of FREQ_TICKS) {
        if (f % step) continue;
        ctx.textAlign = f === 0 ? 'left' : 'center';
        ctx.fillText(f === 0 ? '0 Hz' : String(f), X(f), y);
    }
}

function noData(ctx, x, y, hint) {
    ctx.fillStyle = INK.muted; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.font = LABEL_FONT;
    ctx.fillText('NO DATA', x, y - 7);
    ctx.font = TICK_FONT;
    ctx.fillText(hint, x, y + 8);
}

/** Bin of the highest level, ignoring the first (the residual of the mean). */
export function peakOf(db) {
    let best = 1;
    for (let i = 2; i < db.length; i++) if (db[i] > db[best]) best = i;
    return { f: binFreq(best), db: db[best] };
}

const LIVE_PAD = { l: 34, r: 8, t: 8, b: 14 };

/**
 * Live spectrum. Returns the readout for the legend: the peak, or the
 * level under the pointer.
 */
export function drawLive(box, sensor, now) {
    if (!box.begin()) return null;
    const { ctx, w, h } = box;
    const s = spectrumMonitor.get(sensor);
    const latest = s.latest();
    const pw = w - LIVE_PAD.l - LIVE_PAD.r, ph = h - LIVE_PAD.t - LIVE_PAD.b;
    if (pw < 30 || ph < 30) return null;
    const X = f => LIVE_PAD.l + f / FFT_MAX_HZ * pw;
    if (!latest) {
        freqAxis(ctx, X, h - 2, pw);
        noData(ctx, LIVE_PAD.l + pw / 2, LIVE_PAD.t + ph / 2, NO_DATA_HINT);
        return null;
    }
    const peak = s.peakHold(now);
    let max = -Infinity, min = Infinity;
    for (let i = 1; i < FFT_BINS; i++) {
        max = Math.max(max, latest.db[i], peak ? peak[i] : -Infinity);
        min = Math.min(min, latest.db[i]);
    }
    let hi = Math.ceil((max + 3) / 10) * 10, lo = Math.floor((min - 3) / 10) * 10;
    lo = Math.max(lo, hi - 90);
    if (hi - lo < 30) lo = hi - 30;
    const Y = v => LIVE_PAD.t + ph - (Math.min(Math.max(v, lo), hi) - lo) / (hi - lo) * ph;

    // dB grid
    ctx.font = TICK_FONT; ctx.textBaseline = 'middle'; ctx.lineWidth = 1;
    const stepDb = (hi - lo) > 60 ? 20 : 10;
    for (let v = lo; v <= hi; v += stepDb) {
        const y = Math.round(Y(v)) + 0.5;
        ctx.strokeStyle = v === lo ? INK.axis : INK.grid;
        ctx.beginPath(); ctx.moveTo(LIVE_PAD.l, y); ctx.lineTo(LIVE_PAD.l + pw, y); ctx.stroke();
        ctx.fillStyle = INK.muted; ctx.textAlign = 'right';
        ctx.fillText(String(v), LIVE_PAD.l - 4, y);
    }
    for (const f of FREQ_TICKS) {
        if (!f) continue;
        const x = Math.round(X(f)) + 0.5;
        ctx.strokeStyle = INK.grid;
        ctx.beginPath(); ctx.moveTo(x, LIVE_PAD.t); ctx.lineTo(x, LIVE_PAD.t + ph); ctx.stroke();
    }
    freqAxis(ctx, X, h - 2, pw);

    ctx.save();
    ctx.beginPath(); ctx.rect(LIVE_PAD.l, LIVE_PAD.t - 1, pw, ph + 2); ctx.clip();
    ctx.lineJoin = 'round'; ctx.lineCap = 'round';
    const path = (db, color, width) => {
        ctx.strokeStyle = color; ctx.lineWidth = width;
        ctx.beginPath();
        for (let i = 0; i < FFT_BINS; i++) {
            const x = X(binFreq(i)), y = Y(db[i]);
            if (i) ctx.lineTo(x, y); else ctx.moveTo(x, y);
        }
        ctx.stroke();
    };
    if (peak) { ctx.globalAlpha = 0.85; path(peak, PEAK_COLOR, 1); ctx.globalAlpha = 1; }
    path(latest.db, LINE_COLOR, 2);
    ctx.restore();

    if (!spectrumMonitor.live(sensor, now)) {
        ctx.fillStyle = INK.muted; ctx.textAlign = 'right'; ctx.textBaseline = 'top'; ctx.font = LABEL_FONT;
        ctx.fillText('NO NEW DATA', LIVE_PAD.l + pw - 2, LIVE_PAD.t + 2);
    }

    // Pointer: the level of the bin under it
    if (box.hover && box.hover.x >= LIVE_PAD.l && box.hover.x <= LIVE_PAD.l + pw) {
        const i = Math.min(FFT_BINS - 1, Math.max(0, Math.round((box.hover.x - LIVE_PAD.l) / pw * FFT_MAX_HZ / FFT_DF) - 1));
        const x = Math.round(X(binFreq(i))) + 0.5;
        ctx.strokeStyle = INK.secondary; ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(x, LIVE_PAD.t); ctx.lineTo(x, LIVE_PAD.t + ph); ctx.stroke();
        return { label: 'AT', f: binFreq(i), db: latest.db[i] };
    }
    const p = peakOf(latest.db);
    return { label: 'PEAK', f: p.f, db: p.db };
}

const WF_PAD = { l: 40, r: 8, t: 6, b: 14 };
// colour scale of the waterfall, dB: below LO black, above HI the brightest
const WF_DB_LO = -100;
const WF_DB_HI = 0;
let wfImage = null;      // offscreen canvas: one column per pixel column, one row per bin

/**
 * Waterfall: frequency down the plot (0 Hz at the top), time to the right
 * (the last 60 s, now at the right edge), colour = level on a fixed scale
 * from −100 to 0 dB. Returns the colour range and the readout under the
 * pointer for the legend.
 */
export function drawWaterfall(box, sensor, now) {
    if (!box.begin()) return null;
    const { ctx, w, h } = box;
    const rows = spectrumMonitor.get(sensor).rows;
    const pw = w - WF_PAD.l - WF_PAD.r, ph = h - WF_PAD.t - WF_PAD.b;
    if (pw < 30 || ph < 30) return null;
    const Y = f => WF_PAD.t + f / FFT_MAX_HZ * ph;

    // Frequency labels down the left, time labels along the bottom
    ctx.font = TICK_FONT; ctx.fillStyle = INK.muted; ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
    const step = ph < 220 ? 100 : 50;
    for (const f of FREQ_TICKS) {
        if (f % step) continue;
        const y = f === 0 ? WF_PAD.t + 5 : Y(f);
        ctx.fillText(f === 0 ? '0 Hz' : String(f), WF_PAD.l - 4, y);
    }
    ctx.textBaseline = 'alphabetic';
    ctx.textAlign = 'left'; ctx.fillText('−60 s', WF_PAD.l, h - 2);
    ctx.textAlign = 'center'; ctx.fillText('−30 s', WF_PAD.l + pw / 2, h - 2);
    ctx.textAlign = 'right'; ctx.fillText('now', WF_PAD.l + pw, h - 2);
    ctx.strokeStyle = INK.axis; ctx.lineWidth = 1;
    ctx.strokeRect(WF_PAD.l - 0.5, WF_PAD.t - 0.5, pw + 1, ph + 1);
    if (!rows.length) {
        noData(ctx, WF_PAD.l + pw / 2, WF_PAD.t + ph / 2, NO_DATA_HINT);
        return null;
    }

    // Fixed colour scale, so the same colour is the same level over time
    const lo = WF_DB_LO, hi = WF_DB_HI;

    const W = Math.max(1, Math.round(pw));
    if (!wfImage || wfImage.width !== W) {
        wfImage = document.createElement('canvas');
        wfImage.width = W;
        wfImage.height = FFT_BINS;
    }
    const ictx = wfImage.getContext('2d');
    const img = ictx.createImageData(W, FFT_BINS);
    const px = img.data;
    // Each pixel column shows the spectrum that covers its time: the first
    // one received at or after it, within a second (a longer hole stays black)
    let r = 0;
    for (let x = 0; x < W; x++) {
        const t = now - (1 - (x + 0.5) / W) * SPECTRUM_HISTORY_MS;
        while (r < rows.length - 1 && rows[r].t < t) r++;
        const row = rows[r];
        if (row.t < t || row.t - t > 1000) continue;
        for (let i = 0; i < FFT_BINS; i++) {
            const k = Math.max(0, Math.min(255, Math.round((row.db[i] - lo) / (hi - lo) * 255))) * 3;
            const o = (i * W + x) * 4;
            px[o] = LUT[k]; px[o + 1] = LUT[k + 1]; px[o + 2] = LUT[k + 2]; px[o + 3] = 255;
        }
    }
    ictx.putImageData(img, 0, 0);
    ctx.imageSmoothingEnabled = false;
    // Bin i spans (i + 0.5) to (i + 1.5) * df: the image starts half a bin
    // below 0 Hz and its last half bin falls outside the plot
    ctx.save();
    ctx.beginPath(); ctx.rect(WF_PAD.l, WF_PAD.t, pw, ph); ctx.clip();
    ctx.drawImage(wfImage, WF_PAD.l, Y(0.5 * FFT_DF), pw, ph);
    ctx.restore();

    let readout = null;
    if (box.hover && box.hover.x >= WF_PAD.l && box.hover.x <= WF_PAD.l + pw && box.hover.y >= WF_PAD.t && box.hover.y <= WF_PAD.t + ph) {
        const i = Math.min(FFT_BINS - 1, Math.max(0, Math.round((box.hover.y - WF_PAD.t) / ph * FFT_MAX_HZ / FFT_DF) - 1));
        const age = (1 - (box.hover.x - WF_PAD.l) / pw) * SPECTRUM_HISTORY_MS;
        let row = null;
        for (let k = rows.length - 1; k >= 0; k--) if (rows[k].t >= now - age) row = rows[k]; else break;
        ctx.strokeStyle = 'rgba(255,255,255,0.5)'; ctx.lineWidth = 1;
        const y = Math.round(Y(binFreq(i))) + 0.5, x = Math.round(box.hover.x) + 0.5;
        ctx.beginPath(); ctx.moveTo(WF_PAD.l, y); ctx.lineTo(WF_PAD.l + pw, y); ctx.moveTo(x, WF_PAD.t); ctx.lineTo(x, WF_PAD.t + ph); ctx.stroke();
        if (row) readout = { f: binFreq(i), db: row.db[i], age: age / 1000 };
    }
    return { lo, hi, readout };
}

export function sensorLabel(sensor) { return SENSORS[sensor]?.label ?? sensor; }
export function sensorUnit(sensor) { return SENSORS[sensor]?.unit ?? ''; }
