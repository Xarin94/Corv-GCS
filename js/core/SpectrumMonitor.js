/**
 * SpectrumMonitor.js - IMU vibration spectra from the vehicle
 *
 * ArduPilot with the Hybrid-PF (HPF_FFT_ENABLE = 1) sends, four times a
 * second, the power spectral density of the IMU samples as DEBUG_FLOAT_ARRAY
 * (350): name "FFT_GYRO" or "FFT_ACC", array_id = 2 * sensor + part, 58 bins a
 * part, 116 bins from 3.125 to 362.5 Hz (bin i at (i + 1) * 800 / 256 Hz), in
 * dB of (rad/s)²/Hz for the gyro and (m/s²)²/Hz for the accelerometer, three
 * axes summed (libraries/AP_HybridPF/AP_HybridPF_Spectrum.h).
 *
 * The two parts of a spectrum share time_usec; a spectrum is kept once both
 * have arrived. The last 60 s of each sensor feed the waterfall, the newest
 * the live chart, and a 10 s peak hold rides over it; the strongest lines of
 * the last 2 s head the FFT bar.
 */

import { onMessage } from '../mavlink/MAVLinkManager.js';

export const FFT_RATE_HZ = 800;
export const FFT_N = 256;
export const FFT_PART_BINS = 58;
export const FFT_BINS = 2 * FFT_PART_BINS;
export const FFT_DF = FFT_RATE_HZ / FFT_N;
export const FFT_MAX_HZ = FFT_BINS * FFT_DF;
export const SPECTRUM_HISTORY_MS = 60000;
const PEAK_HOLD_MS = 10000;
const STALE_MS = 2000;

/** Centre frequency of bin i (0-based), Hz. */
export const binFreq = i => (i + 1) * FFT_DF;

export const SENSORS = {
    gyro: { id: 0, label: 'GYRO', unit: 'dB re 1 (rad/s)²/Hz' },
    accel: { id: 1, label: 'ACC', unit: 'dB re 1 (m/s²)²/Hz' },
};
const BY_ID = [SENSORS.gyro, SENSORS.accel];
const NAMES = ['FFT_GYRO', 'FFT_ACC'];

class SensorSpectra {
    constructor() { this.reset(); }
    reset() {
        this.rows = [];          // { t (performance.now), db: Float32Array }
        this.pending = null;     // { usec, parts: [Float32Array|null, Float32Array|null] }
        this.version = 0;        // bumped at every new spectrum
    }
    add(usec, part, data, now) {
        if (!this.pending || this.pending.usec !== usec) this.pending = { usec, parts: [null, null] };
        this.pending.parts[part] = Float32Array.from(data.slice(0, FFT_PART_BINS));
        const [a, b] = this.pending.parts;
        if (!a || !b) return;
        const db = new Float32Array(FFT_BINS);
        db.set(a, 0);
        db.set(b, FFT_PART_BINS);
        this.pending = null;
        this.rows.push({ t: now, db });
        const cut = now - SPECTRUM_HISTORY_MS;
        let i = 0;
        while (i < this.rows.length && this.rows[i].t < cut) i++;
        if (i) this.rows.splice(0, i);
        this.version++;
    }
    latest() { return this.rows.length ? this.rows[this.rows.length - 1] : null; }
    /**
     * Strongest lines over the last windowMs: power averaged per bin, local
     * maxima at least minProm dB over the local floor (the median of the bins
     * within ±FLOOR_BINS, so the IMU filter's roll-off is not taken for a
     * line), up to n of them, strongest first. Bin 0 (3.1 Hz, the residue of
     * the mean removal) is left out. floor: median of the bins under the
     * strongest level, max: that bin. Null without data in the window.
     */
    dominant(now, windowMs = 2000, n = 3, minProm = 6) {
        const FLOOR_BINS = 8;
        const recent = this.rows.filter(r => r.t >= now - windowMs);
        if (!recent.length) return null;
        const db = new Float32Array(FFT_BINS);
        for (let i = 0; i < FFT_BINS; i++) {
            let p = 0;
            for (const r of recent) p += Math.pow(10, r.db[i] / 10);
            db[i] = 10 * Math.log10(p / recent.length);
        }
        const localFloor = i => {
            const v = [];
            for (let k = Math.max(1, i - FLOOR_BINS); k <= Math.min(FFT_BINS - 1, i + FLOOR_BINS); k++) {
                if (Math.abs(k - i) > 1) v.push(db[k]);
            }
            v.sort((a, b) => a - b);
            return v[Math.floor(v.length / 2)];
        };
        const peaks = [];
        let strongest = 1;
        for (let i = 1; i < FFT_BINS; i++) {
            if (db[i] > db[strongest]) strongest = i;
            const left = db[i - 1], right = i + 1 < FFT_BINS ? db[i + 1] : -Infinity;
            if (db[i] >= left && db[i] > right) {
                const prom = db[i] - localFloor(i);
                if (prom >= minProm) peaks.push({ f: binFreq(i), db: db[i], prom });
            }
        }
        peaks.sort((a, b) => b.db - a.db);
        return { floor: localFloor(strongest), max: { f: binFreq(strongest), db: db[strongest] }, peaks: peaks.slice(0, n) };
    }
    /** Highest level per bin over the last PEAK_HOLD_MS. */
    peakHold(now) {
        const out = new Float32Array(FFT_BINS).fill(-Infinity);
        let any = false;
        for (let r = this.rows.length - 1; r >= 0 && this.rows[r].t >= now - PEAK_HOLD_MS; r--) {
            const db = this.rows[r].db;
            for (let i = 0; i < FFT_BINS; i++) if (db[i] > out[i]) out[i] = db[i];
            any = true;
        }
        return any ? out : null;
    }
}

class SpectrumMonitor {
    constructor() {
        this.sensors = { gyro: new SensorSpectra(), accel: new SensorSpectra() };
        this.subscribed = false;
    }
    subscribe() {
        if (this.subscribed) return;
        this.subscribed = true;
        onMessage(350, data => this.ingest(data, performance.now()));
        // A new link is a new vehicle: its spectra start empty
        window.addEventListener('mavlinkConnectionState', e => {
            if (e.detail?.state === 'CONNECTED') this.reset();
        });
    }
    ingest(data, now) {
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
        const l = this.sensors[sensor].latest();
        return !!l && now - l.t < STALE_MS;
    }
    reset() { for (const s of Object.values(this.sensors)) s.reset(); }
}

export const spectrumMonitor = new SpectrumMonitor();
