/**
 * SpectrumViews.js - IMU vibration spectrum charts
 *
 * Two views of the spectra in core/SpectrumMonitor.js, each for the gyro or
 * the accelerometer, both in the FFT bar on the left (ui/FftPanel.js):
 *   - live: the newest spectrum as a line, the 10 s peak hold as a thin line
 *     above it
 *   - waterfall: the last 60 s, frequency down the plot, time to the right,
 *     colour = level in dB
 * Frequencies on a linear axis from 0 to the top bin of the source: 362.5 Hz
 * for the vehicle's spectrum, half the sample rate for one computed from a
 * log. A log's spectrum can have thousands of bins: each pixel shows the
 * highest of the bins it covers, so a narrow line never falls between pixels.
 */

import {
    spectrumMonitor, SPECTRUM_HISTORY_MS, binFreq, gridMaxHz, firstLineBin, SENSORS
} from '../core/SpectrumMonitor.js';

const INK = { primary: '#f0efec', secondary: '#c3c2b7', muted: '#898781', grid: 'rgba(255,255,255,0.07)', axis: 'rgba(255,255,255,0.22)' };
const LINE_COLOR = '#3987e5';
const PEAK_COLOR = '#c98500';
const TICK_FONT = '10px "Roboto Mono", monospace';
const LABEL_FONT = '11px "Rajdhani", sans-serif';

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

/**
 * Frequency ticks from 0 to maxHz, a 1-2-5 step at least minPx apart over
 * a span of px pixels.
 */
export function freqTicks(maxHz, px, minPx) {
    const raw = maxHz * minPx / Math.max(1, px);
    const mag = Math.pow(10, Math.floor(Math.log10(raw)));
    const step = [1, 2, 5, 10].map(m => m * mag).find(s => s >= raw) || 10 * mag;
    const ticks = [];
    for (let f = 0; f <= maxHz + 1e-9; f += step) ticks.push(+f.toPrecision(6));
    return ticks;
}
const fmtHz = f => (f === 0 ? '0 Hz' : f >= 1 || f === Math.round(f) ? String(+f.toFixed(1)) : String(+f.toFixed(2)));

function freqAxis(ctx, X, y, ticks) {
    ctx.font = TICK_FONT; ctx.fillStyle = INK.muted; ctx.textBaseline = 'alphabetic';
    for (const f of ticks) {
        ctx.textAlign = f === 0 ? 'left' : 'center';
        ctx.fillText(fmtHz(f), X(f), y);
    }
}

function noData(ctx, x, y, hint) {
    ctx.fillStyle = INK.muted; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.font = LABEL_FONT;
    ctx.fillText('NO DATA', x, y - 7);
    ctx.font = TICK_FONT;
    ctx.fillText(hint, x, y + 8);
}

/** Bin of the highest level, above the first bins (the residual of the mean). */
export function peakOf(db, grid) {
    let best = firstLineBin(grid);
    for (let i = best + 1; i < db.length; i++) if (db[i] > db[best]) best = i;
    return { f: binFreq(best, grid), db: db[best] };
}

/**
 * Bin ranges of `cells` equal cells over n bins (cells ≤ n): cell c covers
 * bins from[c] … from[c + 1] − 1. Cached per (n, cells).
 */
const cellCache = new Map();
function cellBounds(n, cells) {
    const key = n + ':' + cells;
    let b = cellCache.get(key);
    if (!b) {
        b = new Uint32Array(cells + 1);
        for (let c = 0; c <= cells; c++) b[c] = Math.floor(c * n / cells);
        if (cellCache.size > 16) cellCache.clear();
        cellCache.set(key, b);
    }
    return b;
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
    const grid = s.grid, n = grid.n, maxHz = gridMaxHz(grid);
    const latest = s.latest();
    const pw = w - LIVE_PAD.l - LIVE_PAD.r, ph = h - LIVE_PAD.t - LIVE_PAD.b;
    if (pw < 30 || ph < 30) return null;
    const X = f => LIVE_PAD.l + f / maxHz * pw;
    const ticks = freqTicks(maxHz, pw, 34);
    if (!latest) {
        freqAxis(ctx, X, h - 2, ticks);
        noData(ctx, LIVE_PAD.l + pw / 2, LIVE_PAD.t + ph / 2, spectrumMonitor.source(sensor).hint);
        return null;
    }
    const peak = s.peakHold(now);
    const first = Math.min(1, n - 1);
    let max = -Infinity, min = Infinity;
    for (let i = first; i < n; i++) {
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
    for (const f of ticks) {
        if (!f) continue;
        const x = Math.round(X(f)) + 0.5;
        ctx.strokeStyle = INK.grid;
        ctx.beginPath(); ctx.moveTo(x, LIVE_PAD.t); ctx.lineTo(x, LIVE_PAD.t + ph); ctx.stroke();
    }
    freqAxis(ctx, X, h - 2, ticks);

    ctx.save();
    ctx.beginPath(); ctx.rect(LIVE_PAD.l, LIVE_PAD.t - 1, pw, ph + 2); ctx.clip();
    ctx.lineJoin = 'round'; ctx.lineCap = 'round';
    // More bins than pixels: one point per pixel, the highest bin under it
    const cols = Math.max(2, Math.round(pw));
    const bounds = n > 2 * cols ? cellBounds(n, cols) : null;
    const path = (db, color, width) => {
        ctx.strokeStyle = color; ctx.lineWidth = width;
        ctx.beginPath();
        if (bounds) {
            for (let c = 0; c < cols; c++) {
                let v = -Infinity;
                for (let i = bounds[c]; i < bounds[c + 1]; i++) if (db[i] > v) v = db[i];
                const x = LIVE_PAD.l + (c + 0.5) / cols * pw, y = Y(v);
                if (c) ctx.lineTo(x, y); else ctx.moveTo(x, y);
            }
        } else {
            for (let i = 0; i < n; i++) {
                const x = X(binFreq(i, grid)), y = Y(db[i]);
                if (i) ctx.lineTo(x, y); else ctx.moveTo(x, y);
            }
        }
        ctx.stroke();
    };
    if (peak) { ctx.globalAlpha = 0.85; path(peak, PEAK_COLOR, 1); ctx.globalAlpha = 1; }
    path(latest.db, LINE_COLOR, bounds ? 1.5 : 2);
    ctx.restore();

    if (!spectrumMonitor.live(sensor, now)) {
        ctx.fillStyle = INK.muted; ctx.textAlign = 'right'; ctx.textBaseline = 'top'; ctx.font = LABEL_FONT;
        ctx.fillText('NO NEW DATA', LIVE_PAD.l + pw - 2, LIVE_PAD.t + 2);
    }

    // Pointer: the level of the bin under it
    if (box.hover && box.hover.x >= LIVE_PAD.l && box.hover.x <= LIVE_PAD.l + pw) {
        const i = Math.min(n - 1, Math.max(0, Math.round((box.hover.x - LIVE_PAD.l) / pw * maxHz / grid.df) - 1));
        const x = Math.round(X(binFreq(i, grid))) + 0.5;
        ctx.strokeStyle = INK.secondary; ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(x, LIVE_PAD.t); ctx.lineTo(x, LIVE_PAD.t + ph); ctx.stroke();
        return { label: 'AT', f: binFreq(i, grid), db: latest.db[i] };
    }
    const p = peakOf(latest.db, grid);
    return { label: 'PEAK', f: p.f, db: p.db };
}

const WF_PAD = { l: 40, r: 8, t: 6, b: 14 };
// colour scale of the waterfall, dB: below LO black, above HI the brightest
const WF_DB_LO = -100;
const WF_DB_HI = 0;
let wfImage = null;      // offscreen canvas: one column per pixel column, one row per bin (or pixel row)

/**
 * Waterfall: frequency down the plot (0 Hz at the top), time to the right
 * (the last 60 s, now at the right edge), colour = level on a fixed scale
 * from −100 to 0 dB. Returns the colour range and the readout under the
 * pointer for the legend.
 */
export function drawWaterfall(box, sensor, now) {
    if (!box.begin()) return null;
    const { ctx, w, h } = box;
    const s = spectrumMonitor.get(sensor);
    const rows = s.rows, grid = s.grid, n = grid.n, maxHz = gridMaxHz(grid);
    const pw = w - WF_PAD.l - WF_PAD.r, ph = h - WF_PAD.t - WF_PAD.b;
    if (pw < 30 || ph < 30) return null;
    const Y = f => WF_PAD.t + f / maxHz * ph;

    // Frequency labels down the left, time labels along the bottom
    ctx.font = TICK_FONT; ctx.fillStyle = INK.muted; ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
    for (const f of freqTicks(maxHz, ph, 26)) {
        const y = f === 0 ? WF_PAD.t + 5 : Y(f);
        if (f && y > WF_PAD.t + ph - 4) continue;
        ctx.fillText(fmtHz(f), WF_PAD.l - 4, y);
    }
    ctx.textBaseline = 'alphabetic';
    ctx.textAlign = 'left'; ctx.fillText('−60 s', WF_PAD.l, h - 2);
    ctx.textAlign = 'center'; ctx.fillText('−30 s', WF_PAD.l + pw / 2, h - 2);
    ctx.textAlign = 'right'; ctx.fillText('now', WF_PAD.l + pw, h - 2);
    ctx.strokeStyle = INK.axis; ctx.lineWidth = 1;
    ctx.strokeRect(WF_PAD.l - 0.5, WF_PAD.t - 0.5, pw + 1, ph + 1);
    if (!rows.length) {
        noData(ctx, WF_PAD.l + pw / 2, WF_PAD.t + ph / 2, spectrumMonitor.source(sensor).hint);
        return null;
    }

    // Fixed colour scale, so the same colour is the same level over time
    const lo = WF_DB_LO, hi = WF_DB_HI;

    const W = Math.max(1, Math.round(pw));
    // One image row per bin, or per pixel row when the bins outnumber them
    const H = Math.min(n, Math.max(1, Math.round(ph)));
    const bounds = H < n ? cellBounds(n, H) : null;
    if (!wfImage || wfImage.width !== W || wfImage.height !== H) {
        wfImage = document.createElement('canvas');
        wfImage.width = W;
        wfImage.height = H;
    }
    const ictx = wfImage.getContext('2d');
    const img = ictx.createImageData(W, H);
    const px = img.data;
    // Each pixel column shows the spectrum that covers its time: the first
    // one at or after it, within its hold (a longer hole stays black)
    const hold = s.holdMs();
    let r = 0;
    for (let x = 0; x < W; x++) {
        const t = now - (1 - (x + 0.5) / W) * SPECTRUM_HISTORY_MS;
        while (r < rows.length - 1 && rows[r].t < t) r++;
        const row = rows[r];
        if (row.t < t || row.t - t > hold || row.t > now) continue;
        const db = row.db;
        for (let y = 0; y < H; y++) {
            let v = db[y];
            if (bounds) { v = -Infinity; for (let i = bounds[y]; i < bounds[y + 1]; i++) if (db[i] > v) v = db[i]; }
            const k = Math.max(0, Math.min(255, Math.round((v - lo) / (hi - lo) * 255))) * 3;
            const o = (y * W + x) * 4;
            px[o] = LUT[k]; px[o + 1] = LUT[k + 1]; px[o + 2] = LUT[k + 2]; px[o + 3] = 255;
        }
    }
    ictx.putImageData(img, 0, 0);
    ctx.imageSmoothingEnabled = false;
    // Bin i spans (i + 0.5) to (i + 1.5) * df: the image starts half a bin
    // below 0 Hz and its last half bin falls outside the plot
    ctx.save();
    ctx.beginPath(); ctx.rect(WF_PAD.l, WF_PAD.t, pw, ph); ctx.clip();
    ctx.drawImage(wfImage, WF_PAD.l, Y(0.5 * grid.df), pw, ph);
    ctx.restore();

    let readout = null;
    if (box.hover && box.hover.x >= WF_PAD.l && box.hover.x <= WF_PAD.l + pw && box.hover.y >= WF_PAD.t && box.hover.y <= WF_PAD.t + ph) {
        const i = Math.min(n - 1, Math.max(0, Math.round((box.hover.y - WF_PAD.t) / ph * maxHz / grid.df) - 1));
        const age = (1 - (box.hover.x - WF_PAD.l) / pw) * SPECTRUM_HISTORY_MS;
        let row = null;
        for (let k = rows.length - 1; k >= 0; k--) if (rows[k].t >= now - age) { if (rows[k].t <= now) row = rows[k]; } else break;
        ctx.strokeStyle = 'rgba(255,255,255,0.5)'; ctx.lineWidth = 1;
        const y = Math.round(Y(binFreq(i, grid))) + 0.5, x = Math.round(box.hover.x) + 0.5;
        ctx.beginPath(); ctx.moveTo(WF_PAD.l, y); ctx.lineTo(WF_PAD.l + pw, y); ctx.moveTo(x, WF_PAD.t); ctx.lineTo(x, WF_PAD.t + ph); ctx.stroke();
        if (row && row.t - (now - age) <= hold) readout = { f: binFreq(i, grid), db: row.db[i], age: age / 1000 };
    }
    return { lo, hi, readout };
}

export function sensorLabel(sensor) { return SENSORS[sensor]?.label ?? sensor; }
export function sensorUnit(sensor) { return SENSORS[sensor]?.unit ?? ''; }
