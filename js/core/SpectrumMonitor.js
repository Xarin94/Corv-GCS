/**
 * SpectrumMonitor.js - IMU vibration spectra, from the vehicle or from a log
 *
 * Live: ArduPilot with the Hybrid-PF (HPF_FFT_ENABLE = 1) sends, four times a
 * second, the power spectral density of the IMU samples as DEBUG_FLOAT_ARRAY
 * (350): name "FFT_GYRO" or "FFT_ACC", array_id = 2 * sensor + part, 58 bins a
 * part, 116 bins from 3.125 to 362.5 Hz (bin i at (i + 1) * 800 / 256 Hz), in
 * dB of (rad/s)²/Hz for the gyro and (m/s²)²/Hz for the accelerometer, three
 * axes summed (libraries/AP_HybridPF/AP_HybridPF_Spectrum.h). The two parts
 * of a spectrum share time_usec; a spectrum is kept once both have arrived.
 * That resolution is the firmware's and stays as it is.
 *
 * LOG REPLAY: a .tlog replays the same messages. A .bin may hold the IMU
 * samples themselves: the main process computes the spectra from them
 * (log-spectrum.js) with the resolution of SYS CONFIG → FFT FROM LOGS and
 * sends them here ('log-spectrum'), same definition and units, bin i at
 * (i + 1) * df with the log's df.
 *
 * Time: live, the arrival time (performance.now()); in a replay, the replay
 * cursor in log ms, so the views stop when it is paused and follow a seek.
 * The last 60 s of each sensor feed the waterfall, the newest the live
 * chart, and a 10 s peak hold rides over it; the strongest lines of the
 * last 2 s head the FFT bar.
 */

import { onMessage } from '../mavlink/MAVLinkManager.js';

export const FFT_RATE_HZ = 800;
export const FFT_N = 256;
export const FFT_PART_BINS = 58;
export const FFT_BINS = 2 * FFT_PART_BINS;
export const FFT_DF = FFT_RATE_HZ / FFT_N;
export const SPECTRUM_HISTORY_MS = 60000;
/** The vehicle's spectrum: n bins, bin i at (i + 1) * df Hz. */
export const LIVE_GRID = Object.freeze({ df: FFT_DF, n: FFT_BINS });
const PEAK_HOLD_MS = 10000;
const STALE_MS = 2000;
const MIN_LINE_HZ = 5;       // lines are looked for above this (and above the first bins)

/** Centre frequency of bin i (0-based) on a grid, Hz. */
export const binFreq = (i, grid = LIVE_GRID) => (i + 1) * grid.df;
/** Highest frequency of a grid, Hz: the top of the charts. */
export const gridMaxHz = grid => grid.n * grid.df;
/** First bin a line can be in: above MIN_LINE_HZ and past the window's leakage of the mean. */
export const firstLineBin = grid => Math.min(grid.n - 1, Math.max(1, Math.ceil(MIN_LINE_HZ / grid.df) - 1));

export const SENSORS = {
    gyro: { id: 0, label: 'GYRO', unit: 'dB re 1 (rad/s)²/Hz' },
    accel: { id: 1, label: 'ACC', unit: 'dB re 1 (m/s²)²/Hz' },
};
const BY_ID = [SENSORS.gyro, SENSORS.accel];
const NAMES = ['FFT_GYRO', 'FFT_ACC'];

class SensorSpectra {
    constructor() { this.grid = LIVE_GRID; this.version = 0; this.reset(); }
    reset(grid = this.grid) {
        this.grid = grid;
        this.rows = [];          // { t (monitor clock, ms), db: Float32Array }
        this.pending = null;     // { usec, parts: [Float32Array|null, Float32Array|null] }
        this.version++;          // bumped at every change
        this._hold = null;
    }
    /** One part of a vehicle spectrum. */
    add(usec, part, data, now) {
        if (!this.pending || this.pending.usec !== usec) this.pending = { usec, parts: [null, null] };
        this.pending.parts[part] = Float32Array.from(data.slice(0, FFT_PART_BINS));
        const [a, b] = this.pending.parts;
        if (!a || !b) return;
        const db = new Float32Array(FFT_BINS);
        db.set(a, 0);
        db.set(b, FFT_PART_BINS);
        this.pending = null;
        this.push(now, db);
    }
    /** A whole spectrum on this sensor's grid, at time t. */
    push(t, db) {
        if (db.length !== this.grid.n) return;
        const prev = this.rows.length ? this.rows[this.rows.length - 1].t : null;
        if (prev !== null && t < prev) {
            // the replay clock went back: those spectra are of another time
            if (t < prev - 1000) this.rows = [];
            else t = prev;                                   // jitter of the interpolated cursor
        }
        this.rows.push({ t, db });
        const cut = t - SPECTRUM_HISTORY_MS;
        let i = 0;
        while (i < this.rows.length && this.rows[i].t < cut) i++;
        if (i) this.rows.splice(0, i);
        this.version++;
    }
    latest() { return this.rows.length ? this.rows[this.rows.length - 1] : null; }
    /**
     * How long a spectrum stands for: the waterfall paints it back this far,
     * and a sensor counts as live for twice that. 1 s for the vehicle's
     * 4 Hz; more for a log's batches, one every few seconds.
     */
    holdMs() {
        if (this._hold && this._hold.version === this.version) return this._hold.ms;
        const gaps = [];
        for (let i = 1; i < this.rows.length; i++) gaps.push(this.rows[i].t - this.rows[i - 1].t);
        gaps.sort((a, b) => a - b);
        const typical = gaps.length ? gaps[gaps.length >> 1] : 250;
        const ms = Math.min(10000, Math.max(1000, 1.5 * typical));
        this._hold = { version: this.version, ms };
        return ms;
    }
    /**
     * Strongest lines over the last windowMs (at least two spectra'
     * spacing): power averaged per bin, local maxima at least minProm dB
     * over the local floor (the median of the bins within ±25 Hz, so the IMU
     * filter's roll-off is not taken for a line), up to n of them, strongest
     * first. Lines below 5 Hz are left out (bin 0 of the vehicle's 3.1 Hz:
     * the residue of the mean removal; slow motion in a log). floor: median
     * of the bins under the strongest level, max: that bin. Null without
     * data in the window.
     */
    dominant(now, windowMs = 2000, n = 3, minProm = 6) {
        const g = this.grid, N = g.n;
        const floorBins = Math.min(64, Math.max(4, Math.round(25 / g.df)));
        const span = Math.max(windowMs, 2 * this.holdMs());
        const recent = this.rows.filter(r => r.t >= now - span && r.t <= now);
        if (!recent.length) return null;
        const db = new Float32Array(N);
        for (let i = 0; i < N; i++) {
            let p = 0;
            for (const r of recent) p += Math.pow(10, r.db[i] / 10);
            db[i] = 10 * Math.log10(p / recent.length);
        }
        const first = firstLineBin(g);
        const localFloor = i => {
            const v = [];
            for (let k = Math.max(first, i - floorBins); k <= Math.min(N - 1, i + floorBins); k++) {
                if (Math.abs(k - i) > 1) v.push(db[k]);
            }
            v.sort((a, b) => a - b);
            return v.length ? v[Math.floor(v.length / 2)] : db[i];
        };
        const peaks = [];
        let strongest = first;
        for (let i = first; i < N; i++) {
            if (db[i] > db[strongest]) strongest = i;
            const left = db[i - 1], right = i + 1 < N ? db[i + 1] : -Infinity;
            if (db[i] >= left && db[i] > right) {
                const prom = db[i] - localFloor(i);
                if (prom >= minProm) peaks.push({ f: binFreq(i, g), db: db[i], prom });
            }
        }
        peaks.sort((a, b) => b.db - a.db);
        return { floor: localFloor(strongest), max: { f: binFreq(strongest, g), db: db[strongest] }, peaks: peaks.slice(0, n) };
    }
    /** Highest level per bin over the last PEAK_HOLD_MS. */
    peakHold(now) {
        const out = new Float32Array(this.grid.n).fill(-Infinity);
        let any = false;
        for (let r = this.rows.length - 1; r >= 0 && this.rows[r].t >= now - PEAK_HOLD_MS; r--) {
            if (this.rows[r].t > now) continue;
            const db = this.rows[r].db;
            for (let i = 0; i < db.length; i++) if (db[i] > out[i]) out[i] = db[i];
            any = true;
        }
        return any ? out : null;
    }
}

class SpectrumMonitor {
    constructor() {
        this.sensors = { gyro: new SensorSpectra(), accel: new SensorSpectra() };
        this.subscribed = false;
        this.replay = null;      // { format, ms, perf, playing } while a log is loaded
        this.log = null;         // what the main process computes from a .bin (log-spectrum.js)
        this.gen = -1;
        this.version = 0;        // bumped when the source changes
    }
    subscribe() {
        if (this.subscribed) return;
        this.subscribed = true;
        onMessage(350, data => this.ingest(data, this.clock()));
        // A new link is a new vehicle: its spectra start empty
        window.addEventListener('mavlinkConnectionState', e => {
            if (e.detail?.state === 'CONNECTED') this.reset();
        });
        const lr = window.logReplay;
        if (!lr) return;
        lr.onState(s => {
            const on = s?.state === 'LOADED' || s?.state === 'PLAYING' || s?.state === 'PAUSED';
            if (on && !this.replay) {
                this.replay = { format: s.format, ms: 0, perf: performance.now(), playing: false };
                this.reset();
            } else if (!on && this.replay) {
                this.replay = null;
                this.log = null;
                this.reset();
            }
        });
        lr.onTick(t => {
            if (!this.replay || !t) return;
            this.replay.ms = Number(t.currentMs) || 0;
            this.replay.perf = performance.now();
            this.replay.playing = !!t.playing;
        });
        // seek: the old spectra are of another time
        lr.onResetState(() => { if (this.replay) this.resetRows(); });
        lr.onSpectrum?.(m => this.onLogSpectrum(m));
    }
    /** Now on the spectra's time base: arrival time live, the cursor in a replay. */
    clock() {
        const r = this.replay;
        if (!r) return performance.now();
        return r.playing ? r.ms + (performance.now() - r.perf) : r.ms;
    }
    onLogSpectrum(m) {
        if (!m) return;
        if (m.reset) {
            this.gen = m.gen;
            this.log = m.log || null;
            this.reset();
        }
        if (m.gen !== this.gen || !m.rows) return;
        for (const r of m.rows) {
            const s = this.sensors[r.sensor];
            if (s) s.push(r.t, r.db instanceof Float32Array ? r.db : Float32Array.from(r.db));
        }
    }
    /** Grid of a sensor for the current source. */
    gridFor(sensor) {
        const l = this.log?.sensors?.[sensor];
        return l?.n > 0 ? { df: l.df, n: l.n } : LIVE_GRID;
    }
    reset() {
        for (const [key, s] of Object.entries(this.sensors)) s.reset(this.gridFor(key));
        this.version++;
    }
    resetRows() { for (const s of Object.values(this.sensors)) s.reset(); }
    ingest(data, now) {
        if (this.log) return;      // a .bin's spectra come from its samples
        if (!data || !Array.isArray(data.data) && !ArrayBuffer.isView(data.data)) return;
        const id = Number(data.arrayId);
        const sensor = BY_ID[id >> 1];
        if (!sensor) return;
        const name = String(data.name || '').replace(/\0[\s\S]*$/, '').trim();
        if (name !== NAMES[sensor.id]) return;
        if (data.data.length < FFT_PART_BINS) return;
        this.sensors[sensor === SENSORS.gyro ? 'gyro' : 'accel'].add(Number(data.timeUsec), id & 1, Array.from(data.data), now);
    }
    get(sensor) { return this.sensors[sensor]; }
    live(sensor, now) {
        const s = this.sensors[sensor];
        const l = s.latest();
        return !!l && now - l.t < Math.max(STALE_MS, 2 * s.holdMs());
    }
    /**
     * Where the spectra of a sensor come from, for the views:
     *   { kind: 'vehicle' | 'tlog' | 'log', text, hint, streams?, stream? }
     * text: source and resolution in one line; hint: what to do when empty.
     */
    source(sensor) {
        const df = v => (v >= 10 ? v.toFixed(1) : v >= 1 ? v.toFixed(2) : v.toFixed(3)) + ' Hz';
        const dur = ms => (ms >= 10000 ? (ms / 1000).toFixed(0) + ' s' : ms >= 1000 ? (ms / 1000).toFixed(1) + ' s' : Math.round(ms) + ' ms');
        if (this.log) {
            const l = this.log.sensors?.[sensor];
            if (!l?.stream) {
                return { kind: 'log', text: 'LOG · NO IMU SAMPLES', hint: 'no IMU samples in the log', streams: [] };
            }
            const avg = l.avg > 1 ? ` × ${l.avg} avg` : '';
            const lim = l.limited ? ` (cut to the ${l.longest}-sample runs)` : '';
            return {
                kind: 'log',
                text: `LOG · ${l.N} pt${avg} · ${df(l.df)} · ${dur(l.windowMs)} window${lim}`,
                hint: `${dur(l.windowMs)} of samples needed for a spectrum`,
                streams: l.streams, stream: l.stream, label: l.label,
            };
        }
        if (this.replay) {
            return { kind: 'tlog', text: `LOG · VEHICLE SPECTRUM · ${FFT_N} pt at ${FFT_RATE_HZ} Hz · ${df(FFT_DF)}`, hint: 'no FFT_GYRO / FFT_ACC in this log' };
        }
        return { kind: 'vehicle', text: `VEHICLE · ${FFT_N} pt at ${FFT_RATE_HZ} Hz · ${df(FFT_DF)}`, hint: 'HPF_FFT_ENABLE = 1 on the vehicle' };
    }
    /** Use another IMU stream of the loaded .bin for a sensor. */
    selectStream(sensor, key) {
        window.logReplay?.selectSpectrumStream?.(sensor, key);
    }
}

export const spectrumMonitor = new SpectrumMonitor();
