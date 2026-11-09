/**
 * log-spectrum.js - IMU vibration spectra computed from a DataFlash log
 *
 * Live, the vehicle sends its own spectrum (HPF_FFT_ENABLE, 256 points at
 * 800 Hz, js/core/SpectrumMonitor.js) and nothing here is involved. A .bin log
 * in LOG REPLAY may hold the IMU samples themselves; this module turns them
 * into spectra with the resolution chosen in SYS CONFIG → FFT FROM LOGS and
 * streams them to the renderer in step with the replay cursor, into the same
 * FFT bar on the left.
 *
 * Sources in the log, per sensor (gyro, accelerometer) and IMU instance:
 *   GYR / ACC    every sample at the sensor rate (LOG_BITMASK IMU_RAW,
 *                INS_RAW_LOG_OPT); GYR instances past the gyro count are the
 *                post-filter copies (INS_RAW_LOG_OPT "pre and post filter")
 *   ISBH / ISBD  batch sampler (INS_LOG_BAT_MASK): INS_LOG_BAT_CNT contiguous
 *                samples per batch, sensors and instances in turn, int16
 *                scaled by ISBH.mul
 *   IMU          the filtered sample at the loop rate
 *
 * The spectrum has the definition of the live one (docs/FFT_SPECTRUM_PROTOCOL.md):
 * mean removed, Hann window, one-sided PSD 2|X|² / (fs Σw²), three axes
 * summed, in dB of (rad/s)²/Hz or (m/s²)²/Hz, clipped at −200 dB. The log adds
 * Welch averaging: K windows of N samples overlapping by half, their PSD
 * averaged. Bin i is at (i + 1) fs / N, i = 0 … N/2 − 2.
 *
 * GYR / ACC samples are taken as evenly spaced at the rate of their run:
 * the SampleUS of FIFO sensors come in bursts (tens of µs apart, then a long
 * step), so resampling on them would add artefacts. A run ends at a hole in
 * the samples (logger overrun) and a window never straddles one. IMU records
 * carry the loop time, which is sound, but the logger drops some of them:
 * they are interpolated onto an even grid at their typical step, as the
 * firmware does for the live spectrum, holes up to 4 steps bridged.
 *
 * No Electron here: the scheduler sends through a callback, and the whole
 * module runs under plain Node for the tests.
 */

'use strict';

// ── Settings ─────────────────────────────────────────────────────────────

const FFT_SIZES = [256, 512, 1024, 2048, 4096, 8192];
const AVERAGES = [1, 2, 4, 8];
const STEPS_MS = [100, 250, 500, 1000];
const DEFAULT_CONFIG = { n: 1024, avg: 1, stepMs: 250 };

function sanitizeConfig(c) {
    const pick = (v, list, def) => (list.includes(Number(v)) ? Number(v) : def);
    return {
        n: pick(c?.n, FFT_SIZES, DEFAULT_CONFIG.n),
        avg: pick(c?.avg, AVERAGES, DEFAULT_CONFIG.avg),
        stepMs: pick(c?.stepMs, STEPS_MS, DEFAULT_CONFIG.stepMs),
    };
}

// ── Sample collection (fed by the .bin parser) ───────────────────────────

const ISB_ACCEL = 0, ISB_GYRO = 1;     // ISBH.type (AP_InertialSensor::IMU_SENSOR_TYPE)
const GAP_FACTOR = 3;                  // a step over 3 typical steps ends a run
const TYPICAL_SPAN = 8;                // typical step from spans of 8 steps (bursts average out)
const TYPICAL_QUANTILE = 0.2;          // … their 20th percentile, clear of the holes
const MIN_RUN_RATE = 0.75;             // a run slower than this × the typical rate hides holes
const RESAMPLE_GAP = 4;                // IMU at the loop rate: steps up to 4 typical ones are interpolated
const MIN_RUN = 64;                    // shorter runs are dropped

class Samples {
    constructor() {
        this.n = 0;
        this.t = new Float64Array(1024);   // µs since boot
        this.x = new Float32Array(1024);
        this.y = new Float32Array(1024);
        this.z = new Float32Array(1024);
    }
    push(t, x, y, z) {
        if (this.n === this.t.length) {
            const grow = (A, a) => { const b = new A(a.length * 2); b.set(a); return b; };
            this.t = grow(Float64Array, this.t);
            this.x = grow(Float32Array, this.x);
            this.y = grow(Float32Array, this.y);
            this.z = grow(Float32Array, this.z);
        }
        const i = this.n++;
        this.t[i] = t; this.x[i] = x; this.y[i] = y; this.z[i] = z;
    }
}

const FIELDS = {
    GYR: ['TimeUS', 'SampleUS', 'I', 'GyrX', 'GyrY', 'GyrZ'],
    ACC: ['TimeUS', 'SampleUS', 'I', 'AccX', 'AccY', 'AccZ'],
    IMU: ['TimeUS', 'I', 'GyrX', 'GyrY', 'GyrZ', 'AccX', 'AccY', 'AccZ'],
    ISBH: ['N', 'type', 'instance', 'mul', 'smp_cnt', 'SampleUS', 'smp_rate'],
    ISBD: ['N', 'seqno', 'x', 'y', 'z'],
};

/**
 * Byte offsets in the payload of the fields a message type needs, from its
 * FMT entry, resolved once per type (−1: not in this log's format).
 */
function offsets(entry) {
    if (!entry._imuOff) {
        const byName = {};
        for (const f of entry.fieldOffsets) byName[f.name] = f.offset;
        entry._imuOff = FIELDS[entry.name].map(n => (n in byName ? byName[n] : -1));
    }
    return entry._imuOff;
}

/** uint64 little-endian as a Number (exact below 2^53 µs), without a BigInt. */
const u64 = (b, o) => b.readUInt32LE(o) + b.readUInt32LE(o + 4) * 4294967296;

class ImuCollector {
    constructor() {
        this.cont = new Map();      // 'GYR:gyro:0' → { msg, sensor, inst, s: Samples }
        this.batches = new Map();   // 'ISB:1:0' → { sensor, inst, runs: [{ t0, fs, x, y, z }] }
        this.isb = null;            // batch being assembled: { key, n, count, fs, mul, t0, chunks: Map }
    }

    static wants(name) {
        return name === 'GYR' || name === 'ACC' || name === 'IMU' || name === 'ISBH' || name === 'ISBD';
    }

    /**
     * One record: entry is its FMT description, its payload starts at
     * buf[base] (after the 3-byte header). The parser has checked that the
     * whole record is in the buffer.
     */
    take(entry, buf, base) {
        const o = offsets(entry);
        try {
            switch (entry.name) {
                case 'GYR': case 'ACC': {
                    const [ts, su, ins, fx, fy, fz] = o;
                    if (fz < 0 || (su < 0 && ts < 0)) return;
                    const inst = ins >= 0 ? buf[base + ins] : 0;
                    const t = u64(buf, base + (su >= 0 ? su : ts));
                    const list = this._lists(entry)[0];
                    (list[inst] || this._cont(entry.name, entry.name === 'GYR' ? 'gyro' : 'accel', inst, list))
                        .s.push(t, buf.readFloatLE(base + fx), buf.readFloatLE(base + fy), buf.readFloatLE(base + fz));
                    return;
                }
                case 'IMU': {
                    const [ts, ins, gx, gy, gz, ax, ay, az] = o;
                    if (ts < 0 || gz < 0 || az < 0) return;
                    const inst = ins >= 0 ? buf[base + ins] : 0;
                    const t = u64(buf, base + ts);
                    const [lg, la] = this._lists(entry);
                    (lg[inst] || this._cont('IMU', 'gyro', inst, lg)).s.push(t, buf.readFloatLE(base + gx), buf.readFloatLE(base + gy), buf.readFloatLE(base + gz));
                    (la[inst] || this._cont('IMU', 'accel', inst, la)).s.push(t, buf.readFloatLE(base + ax), buf.readFloatLE(base + ay), buf.readFloatLE(base + az));
                    return;
                }
                case 'ISBH': {
                    const [n, type, ins, mul, cnt, su, rate] = o;
                    if (n < 0 || type < 0 || mul < 0 || cnt < 0 || su < 0 || rate < 0) return;
                    this._closeBatch();
                    const sensorType = buf[base + type];
                    const fs = buf.readFloatLE(base + rate);
                    const m = buf.readUInt16LE(base + mul);
                    if ((sensorType !== ISB_ACCEL && sensorType !== ISB_GYRO) || !(fs > 0) || !m) return;
                    const inst = ins >= 0 ? buf[base + ins] : 0;
                    this.isb = {
                        key: `ISB:${sensorType}:${inst}`,
                        sensor: sensorType === ISB_GYRO ? 'gyro' : 'accel', inst,
                        n: buf.readUInt16LE(base + n), count: buf.readUInt16LE(base + cnt),
                        fs, mul: m, t0: u64(buf, base + su), chunks: new Map(),
                    };
                    return;
                }
                case 'ISBD': {
                    const b = this.isb;
                    if (!b) return;
                    const [n, seq, fx, fy, fz] = o;
                    if (n < 0 || seq < 0 || fx < 0 || fy < 0 || fz < 0) return;
                    if (buf.readUInt16LE(base + n) !== b.n) return;
                    const read = off => { const a = new Float32Array(32); for (let k = 0; k < 32; k++) a[k] = buf.readInt16LE(base + off + 2 * k) / b.mul; return a; };
                    b.chunks.set(buf.readUInt16LE(base + seq), { x: read(fx), y: read(fy), z: read(fz) });
                    return;
                }
            }
        } catch (e) { /* malformed record: skip it */ }
    }

    /**
     * The per-instance stream arrays a message type writes to ([gyro] for
     * GYR, [accel] for ACC, [gyro, accel] for IMU), cached on its FMT entry
     * so a record costs no lookup.
     */
    _lists(entry) {
        if (entry._imuLists?.owner !== this) {
            const lists = entry.name === 'IMU' ? [[], []] : [[]];
            lists.owner = this;
            entry._imuLists = lists;
        }
        return entry._imuLists;
    }

    _cont(msg, sensor, inst, list) {
        const c = { msg, sensor, inst, s: new Samples() };
        list[inst] = c;
        this.cont.set(`${msg}:${sensor}:${inst}`, c);
        return c;
    }

    /** The batch in assembly goes in as its runs of consecutive chunks. */
    _closeBatch() {
        const b = this.isb;
        this.isb = null;
        if (!b || !b.chunks.size) return;
        let entry = this.batches.get(b.key);
        if (!entry) this.batches.set(b.key, entry = { sensor: b.sensor, inst: b.inst, runs: [] });
        const chunks = Math.floor(b.count / 32);
        let k = 0;
        while (k < chunks) {
            if (!b.chunks.has(k)) { k++; continue; }
            const start = k;
            while (k < chunks && b.chunks.has(k)) k++;
            const len = (k - start) * 32;
            if (len < MIN_RUN) continue;
            const run = { t0: b.t0 + start * 32 * 1e6 / b.fs, fs: b.fs, x: new Float32Array(len), y: new Float32Array(len), z: new Float32Array(len) };
            for (let c = start; c < k; c++) {
                const d = b.chunks.get(c), o = (c - start) * 32;
                run.x.set(d.x, o); run.y.set(d.y, o); run.z.set(d.z, o);
            }
            entry.runs.push(run);
        }
    }

    /**
     * The streams found, per sensor, best first. minUs is the time 0 of the
     * replay; params the log's first PARM values.
     */
    finish(minUs, params = {}) {
        this._closeBatch();
        const count = prefix => {
            const ids = [prefix + '_ID', prefix + '2_ID', prefix + '3_ID'].filter(k => k in params);
            return ids.length ? ids.filter(k => params[k] !== 0).length : 0;
        };
        const nGyro = count('INS_GYR'), nAcc = count('INS_ACC');
        const rawOpt = params.INS_RAW_LOG_OPT | 0, batOpt = params.INS_LOG_BAT_OPT | 0;
        const out = { gyro: [], accel: [], dropped: [] };

        for (const c of this.cont.values()) {
            const st = c.msg === 'IMU' ? resampledStream(c.s, minUs) : continuousStream(c.s, minUs);
            if (!st) {
                // e.g. an 8 kHz gyro the logger could only keep in bursts
                if (c.s.n >= MIN_RUN && !out.dropped.includes(`${c.msg} ${c.inst}`)) out.dropped.push(`${c.msg} ${c.inst}`);
                continue;
            }
            let imu = c.inst, filtered, kind;
            if (c.msg === 'IMU') { kind = 'loop'; filtered = true; }
            else {
                kind = 'raw';
                const nSensors = c.sensor === 'gyro' ? nGyro : nAcc;
                if (nSensors && c.inst >= nSensors) { imu = c.inst - nSensors; filtered = true; }
                else filtered = c.sensor === 'gyro' && !!(rawOpt & 4) && !(rawOpt & 8);
            }
            out[c.sensor].push(describe(st, `${c.msg}:${c.inst}`, c.sensor, kind, imu, filtered));
        }
        for (const [key, b] of this.batches) {
            const st = batchStream(b.runs, minUs);
            if (!st) continue;
            const nSensors = b.sensor === 'gyro' ? nGyro : nAcc;
            let imu = b.inst, filtered = !!(batOpt & 2) && !(batOpt & 4);
            if (nSensors && b.inst >= nSensors) { imu = b.inst - nSensors; filtered = true; }
            out[b.sensor].push(describe(st, key, b.sensor, 'batch', imu, filtered));
        }
        const rank = { raw: 0, batch: 1, loop: 2 };
        for (const list of [out.gyro, out.accel]) {
            list.sort((a, b) => rank[a.kind] - rank[b.kind] || a.filtered - b.filtered || a.imu - b.imu || b.fs - a.fs);
        }
        this.cont.clear();
        this.batches.clear();
        return out;
    }
}

function describe(st, key, sensor, kind, imu, filtered) {
    const what = kind === 'loop' ? 'loop rate' : (kind === 'batch' ? 'batch' : 'raw') + (filtered ? ' filtered' : '');
    const label = `IMU${imu + 1} · ${kind === 'raw' && filtered ? 'filtered' : what} · ${st.fs >= 100 ? Math.round(st.fs) : st.fs.toFixed(1)} Hz`;
    return { ...st, key, sensor, kind, imu, filtered, label };
}

function quantile(a, q) {
    const s = Array.from(a).sort((p, r) => p - r);
    return s.length ? s[Math.min(s.length - 1, Math.floor(q * s.length))] : 0;
}

/**
 * A stream sampled continuously, cut into runs of evenly spaced samples.
 *
 * The typical step comes from spans of TYPICAL_SPAN steps, not single ones:
 * FIFO sensors log a few samples tens of µs apart and then a long step, and
 * single steps would land on the short ones. Their 20th percentile, not the
 * median: when the logger cannot keep up (an 8 kHz gyro logged in bursts of
 * a few samples with ms-long holes) most spans hold a hole, and only the
 * lower ones show the sensor's own step.
 *
 * A run ends at a negative step or one over GAP_FACTOR typical steps; a run
 * shorter than MIN_RUN, or slower than MIN_RUN_RATE × the typical rate
 * (holes under the threshold), is dropped. Each run keeps its own rate.
 */
function continuousStream(s, minUs) {
    const n = s.n;
    if (n < MIN_RUN) return null;
    const sample = [];
    const stride = Math.max(1, Math.floor(n / 8000));
    for (let i = TYPICAL_SPAN; i < n; i += stride) sample.push((s.t[i] - s.t[i - TYPICAL_SPAN]) / TYPICAL_SPAN);
    const step = quantile(sample.filter(d => d > 0), TYPICAL_QUANTILE);
    if (!(step > 0)) return null;
    const gap = GAP_FACTOR * step;
    const runs = [];
    let start = 0;
    for (let i = 1; i <= n; i++) {
        const d = i < n ? s.t[i] - s.t[i - 1] : Infinity;
        if (d < 0 || d > gap) {
            const count = i - start;
            const span = s.t[i - 1] - s.t[start];
            if (count >= MIN_RUN && span > 0) {
                const fs = (count - 1) * 1e6 / span;
                if (fs >= MIN_RUN_RATE * 1e6 / step) runs.push({ start, count, fs, span });
            }
            start = i;
        }
    }
    if (!runs.length) return null;
    let num = 0, den = 0;
    for (const r of runs) { num += r.count - 1; den += r.span; delete r.span; }
    const t = new Float64Array(n);
    for (let i = 0; i < n; i++) t[i] = (s.t[i] - minUs) / 1000;
    return {
        fs: num * 1e6 / den,
        t, x: s.x.slice(0, n), y: s.y.slice(0, n), z: s.z.slice(0, n),
        runs, longest: Math.max(...runs.map(r => r.count)),
    };
}

/**
 * IMU records at the loop rate, interpolated onto an even grid at their
 * typical step (the median one: loop times are not bursty). A step over
 * RESAMPLE_GAP typical ones, or a negative one, ends a run.
 */
function resampledStream(s, minUs) {
    const n = s.n;
    if (n < MIN_RUN) return null;
    const sample = [];
    const stride = Math.max(1, Math.floor(n / 8000));
    for (let i = 1; i < n; i += stride) { const d = s.t[i] - s.t[i - 1]; if (d > 0) sample.push(d); }
    const step = quantile(sample, 0.5);
    if (!(step > 0)) return null;
    const segs = [];
    let start = 0;
    for (let i = 1; i <= n; i++) {
        const d = i < n ? s.t[i] - s.t[i - 1] : Infinity;
        if (d < 0 || d > RESAMPLE_GAP * step) {
            const count = Math.floor((s.t[i - 1] - s.t[start]) / step) + 1;
            if (count >= MIN_RUN) segs.push({ a: start, b: i - 1, count });
            start = i;
        }
    }
    if (!segs.length) return null;
    const total = segs.reduce((k, g) => k + g.count, 0);
    const t = new Float64Array(total), x = new Float32Array(total), y = new Float32Array(total), z = new Float32Array(total);
    const runs = [];
    let o = 0;
    for (const { a, b, count } of segs) {
        const t0 = s.t[a];
        let j = a;
        for (let k = 0; k < count; k++) {
            const tk = t0 + k * step;
            while (j < b && s.t[j + 1] <= tk) j++;
            const j1 = Math.min(j + 1, b), span = s.t[j1] - s.t[j];
            const f = span > 0 ? Math.min(1, Math.max(0, (tk - s.t[j]) / span)) : 0;
            t[o + k] = (tk - minUs) / 1000;
            x[o + k] = s.x[j] + f * (s.x[j1] - s.x[j]);
            y[o + k] = s.y[j] + f * (s.y[j1] - s.y[j]);
            z[o + k] = s.z[j] + f * (s.z[j1] - s.z[j]);
        }
        runs.push({ start: o, count, fs: 1e6 / step });
        o += count;
    }
    return { fs: 1e6 / step, t, x, y, z, runs, longest: Math.max(...runs.map(r => r.count)) };
}

/** Batches laid end to end: one run each, times from SampleUS and the rate. */
function batchStream(list, minUs) {
    if (!list.length) return null;
    list.sort((a, b) => a.t0 - b.t0);
    const n = list.reduce((k, r) => k + r.x.length, 0);
    const t = new Float64Array(n), x = new Float32Array(n), y = new Float32Array(n), z = new Float32Array(n);
    const runs = [];
    let o = 0, num = 0, den = 0;
    for (const r of list) {
        const len = r.x.length;
        for (let i = 0; i < len; i++) t[o + i] = (r.t0 + i * 1e6 / r.fs - minUs) / 1000;
        x.set(r.x, o); y.set(r.y, o); z.set(r.z, o);
        runs.push({ start: o, count: len, fs: r.fs });
        num += len * r.fs; den += len;
        o += len;
    }
    return { fs: num / den, t, x, y, z, runs, longest: Math.max(...runs.map(r => r.count)) };
}

// ── FFT and PSD ──────────────────────────────────────────────────────────

const plans = new Map();

/** Bit reversal and twiddles of an N-point radix-2 FFT, cached. */
function plan(N) {
    let p = plans.get(N);
    if (p) return p;
    const bits = Math.log2(N);
    const rev = new Uint32Array(N);
    for (let i = 0; i < N; i++) {
        let r = 0;
        for (let b = 0; b < bits; b++) r |= ((i >> b) & 1) << (bits - 1 - b);
        rev[i] = r;
    }
    const cos = new Float64Array(N / 2), sin = new Float64Array(N / 2);
    for (let k = 0; k < N / 2; k++) { cos[k] = Math.cos(2 * Math.PI * k / N); sin[k] = -Math.sin(2 * Math.PI * k / N); }
    // Hann, the firmware's: 0.5 (1 − cos(2πn / (N − 1)))
    const win = new Float64Array(N);
    let w2 = 0;
    for (let i = 0; i < N; i++) { win[i] = 0.5 * (1 - Math.cos(2 * Math.PI * i / (N - 1))); w2 += win[i] * win[i]; }
    p = { N, rev, cos, sin, win, w2, re: new Float64Array(N), im: new Float64Array(N) };
    plans.set(N, p);
    return p;
}

/** In-place FFT of p.re + i p.im (inputs already in bit-reversed order). */
function fft(p) {
    const { N, cos, sin, re, im } = p;
    for (let size = 2; size <= N; size <<= 1) {
        const half = size >> 1, step = N / size;
        for (let i = 0; i < N; i += size) {
            for (let j = 0, k = 0; j < half; j++, k += step) {
                const a = i + j, b = a + half;
                const tr = re[b] * cos[k] - im[b] * sin[k];
                const ti = re[b] * sin[k] + im[b] * cos[k];
                re[b] = re[a] - tr; im[b] = im[a] - ti;
                re[a] += tr; im[a] += ti;
            }
        }
    }
}

/**
 * Load two real signals (a as the real part, b as the imaginary one, b may
 * be null), mean removed and windowed, transform, and add |A_k|² + |B_k|² for
 * k = 1 … N/2 − 1 to acc[k − 1]. For real a and b the two spectra separate:
 * |A_k|² + |B_k|² = (|Z_k|² + |Z_{N−k}|²) / 2, which with b = 0 is |A_k|².
 */
function addPower(p, a, b, start, acc) {
    const { N, rev, win, re, im } = p;
    let ma = 0, mb = 0;
    for (let i = 0; i < N; i++) { ma += a[start + i]; if (b) mb += b[start + i]; }
    ma /= N; mb /= N;
    for (let i = 0; i < N; i++) {
        const r = rev[i];
        re[r] = (a[start + i] - ma) * win[i];
        im[r] = b ? (b[start + i] - mb) * win[i] : 0;
    }
    fft(p);
    for (let k = 1; k < N / 2; k++) {
        const j = N - k;
        acc[k - 1] += (re[k] * re[k] + im[k] * im[k] + re[j] * re[j] + im[j] * im[j]) / 2;
    }
}

/**
 * Welch PSD of the three axes summed, in dB: K windows of N samples, half
 * overlapping, the last ending at sample `end` (inclusive). N/2 − 1 values,
 * bin i at (i + 1) fs / N.
 */
function welchPsdDb(st, end, N, K, fs) {
    const p = plan(N);
    const L = N + (K - 1) * N / 2;
    const first = end - L + 1;
    const acc = new Float64Array(N / 2 - 1);
    for (let s = 0; s < K; s++) {
        const start = first + s * N / 2;
        addPower(p, st.x, st.y, start, acc);
        addPower(p, st.z, null, start, acc);
    }
    const scale = 2 / (fs * p.w2 * K);
    const db = new Float32Array(N / 2 - 1);
    for (let i = 0; i < db.length; i++) db[i] = 10 * Math.log10(Math.max(acc[i] * scale, 1e-20));
    return db;
}

/** FFT size and averages that fit the stream's longest run: averages go first. */
function fitResolution(st, cfg) {
    let N = cfg.n, K = cfg.avg;
    const span = (n, k) => n + (k - 1) * n / 2;
    while (K > 1 && span(N, K) > st.longest) K >>= 1;
    while (N > 64 && N > st.longest) N >>= 1;
    return { N, K, limited: N !== cfg.n || K !== cfg.avg };
}

/** Index of the last sample at or before tMs, −1 if none. */
function lastAtOrBefore(t, tMs) {
    let lo = 0, hi = t.length;
    while (lo < hi) { const m = (lo + hi) >> 1; if (t[m] <= tMs) lo = m + 1; else hi = m; }
    return lo - 1;
}

/** The run holding sample i. */
function runOf(st, i) {
    let lo = 0, hi = st.runs.length - 1;
    while (lo < hi) { const m = (lo + hi + 1) >> 1; if (st.runs[m].start <= i) lo = m; else hi = m - 1; }
    const r = st.runs[lo];
    return r && i >= r.start && i < r.start + r.count ? r : null;
}

/**
 * The spectrum of the newest full window at or before tMs: { end, t, db }
 * (end the index of its last sample, t that sample's time), or null when
 * the run there is shorter than the window.
 */
function spectrumAt(st, res, tMs) {
    let end = lastAtOrBefore(st.t, tMs);
    if (end < 0) return null;
    let run = runOf(st, end);
    if (!run) {
        // sample in a dropped short run: the previous run ends before it
        for (let r = st.runs.length - 1; r >= 0; r--) if (st.runs[r].start + st.runs[r].count - 1 < end) { run = st.runs[r]; break; }
        if (!run) return null;
        end = run.start + run.count - 1;
    }
    const L = res.N + (res.K - 1) * res.N / 2;
    if (end - run.start + 1 < L) return null;
    return { end, t: st.t[end], db: welchPsdDb(st, end, res.N, res.K, st.fs) };
}

// ── Scheduler: spectra in step with the replay cursor ────────────────────

const HISTORY_MS = 60000;      // the waterfall's span: filled again after a seek
const PUMP_MS = 50;
const BUDGET_MS = 25;          // compute per pump, so a heavy setting fills over a few pumps

class LogSpectrum {
    /**
     * send(channel, payload) reaches the renderer; nowMs() is the replay
     * cursor in log ms.
     */
    constructor(send) {
        this.send = send;
        this.cfg = { ...DEFAULT_CONFIG };
        this.streams = null;       // { gyro: [...], accel: [...] } of the loaded .bin, or null
        this.choice = { gyro: 'auto', accel: 'auto' };
        this.nowMs = () => 0;
        this.gen = 0;
        this.timer = null;
        this.active = {};          // sensor → { st, res, lastEnd }
        this.nextT = 0;
    }

    setConfig(c) {
        this.cfg = sanitizeConfig(c);
        if (this.streams) this.restart();
        return this.cfg;
    }

    selectStream(sensor, key) {
        if (sensor !== 'gyro' && sensor !== 'accel') return;
        this.choice[sensor] = key || 'auto';
        if (this.streams) this.restart();
    }

    /** A .bin log is loaded: streams from ImuCollector.finish(), or null for none. */
    load(streams, nowMs) {
        this.stop();
        this.streams = streams || { gyro: [], accel: [], dropped: [] };
        this.nowMs = nowMs;
        this.choice = { gyro: 'auto', accel: 'auto' };
        this.restart();
        this.timer = setInterval(() => this.pump(), PUMP_MS);
    }

    /** The log is gone (unloaded, or a .tlog that carries the vehicle's own spectrum). */
    unload() {
        this.stop();
        if (this.streams) {
            this.streams = null;
            this.send('log-spectrum', { gen: ++this.gen, reset: true, log: null });
        }
    }

    stop() {
        if (this.timer) clearInterval(this.timer);
        this.timer = null;
    }

    /** Start over from 60 s before the cursor: load, seek, a new setting. */
    restart() {
        this.gen++;
        this.active = {};
        const info = {};
        for (const sensor of ['gyro', 'accel']) {
            const list = this.streams[sensor];
            const st = list.find(s => s.key === this.choice[sensor]) || list[0] || null;
            if (!st) { info[sensor] = { streams: [], stream: null }; continue; }
            const res = fitResolution(st, this.cfg);
            this.active[sensor] = { st, res, lastEnd: -1 };
            info[sensor] = {
                streams: list.map(s => ({ key: s.key, label: s.label })),
                stream: st.key, choice: this.choice[sensor], label: st.label,
                fs: st.fs, N: res.N, avg: res.K, limited: res.limited, longest: st.longest,
                df: st.fs / res.N, n: res.N / 2 - 1, windowMs: (res.N + (res.K - 1) * res.N / 2) / st.fs * 1000,
            };
        }
        const now = this.nowMs();
        const step = this.cfg.stepMs;
        this.nextT = Math.max(0, Math.ceil((now - HISTORY_MS) / step) * step);
        this.lastNow = now;
        this.send('log-spectrum', { gen: this.gen, reset: true, log: { cfg: this.cfg, sensors: info, dropped: this.streams.dropped || [] } });
        this.pump();
    }

    pump() {
        if (!this.streams) return;
        const now = this.nowMs();
        // The cursor went back without a seek (play from the end rewinds): start over
        if (now + 1000 < this.lastNow) { this.lastNow = now; this.restart(); return; }
        this.lastNow = now;
        const step = this.cfg.stepMs;
        if (this.nextT < now - HISTORY_MS) this.nextT = Math.ceil((now - HISTORY_MS) / step) * step;
        const t0 = Date.now();
        const rows = [];
        while (this.nextT <= now && Date.now() - t0 < BUDGET_MS) {
            for (const [sensor, a] of Object.entries(this.active)) {
                const end = lastAtOrBefore(a.st.t, this.nextT);
                if (end < 0 || end === a.lastEnd) continue;
                const s = spectrumAt(a.st, a.res, this.nextT);
                if (!s || s.end === a.lastEnd) continue;
                a.lastEnd = s.end;
                rows.push({ sensor, t: s.t, db: s.db });
            }
            this.nextT += step;
        }
        if (rows.length) this.send('log-spectrum', { gen: this.gen, rows });
    }
}

module.exports = {
    ImuCollector, LogSpectrum, welchPsdDb, fitResolution, spectrumAt, sanitizeConfig,
    FFT_SIZES, AVERAGES, STEPS_MS, DEFAULT_CONFIG,
};
