#!/usr/bin/env node
/**
 * test-log-spectrum.js - offline checks for log-spectrum.js
 *
 * The vibration spectra the GCS computes from the IMU samples of a .bin log:
 *   - the PSD has the definition of the live one: same levels as a direct
 *     DFT of the firmware's formula, Parseval on white noise, a sine's power
 *     under its line, Welch averaging that flattens the floor
 *   - a DataFlash file written here with GYR, ACC, IMU, ISBH/ISBD and PARM is
 *     read back into streams: rates, post-filter instances, batch scaling,
 *     holes in the samples
 *   - the scheduler follows the replay cursor: the 60 s before it after a
 *     seek, one spectrum per batch, start over on rewind, no window across
 *     a hole
 *
 *   node scripts/test-log-spectrum.js
 */
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');

const root = path.join(__dirname, '..');
const { welchPsdDb, fitResolution, LogSpectrum, sanitizeConfig } = require(path.join(root, 'log-spectrum.js'));
const { indexBinFile } = require(path.join(root, 'log-replay-bin-parser.js'));

let failures = 0;
function check(name, cond, detail = '') {
    console.log(`${cond ? 'PASS' : 'FAIL'}  ${name}${detail ? '  ' + detail : ''}`);
    if (!cond) failures++;
}

// Deterministic noise
let seed = 12345;
const rand = () => ((seed = (seed * 1103515245 + 12345) & 0x7fffffff) / 0x7fffffff);
const gauss = () => Math.sqrt(-2 * Math.log(rand() + 1e-12)) * Math.cos(2 * Math.PI * rand());

function stream(n, fs, gen) {
    const x = new Float32Array(n), y = new Float32Array(n), z = new Float32Array(n), t = new Float64Array(n);
    for (let i = 0; i < n; i++) { const v = gen(i / fs); x[i] = v[0]; y[i] = v[1]; z[i] = v[2]; t[i] = i / fs * 1000; }
    return { fs, t, x, y, z, runs: [{ start: 0, count: n, fs }], longest: n };
}

// ── 1. PSD against a direct DFT of the firmware's formula ───────────────
{
    const N = 256, fs = 800;
    const st = stream(N, fs, () => [gauss(), gauss() * 0.5, gauss() * 2]);
    const db = welchPsdDb(st, N - 1, N, 1, fs);
    let w2 = 0;
    const w = [];
    for (let i = 0; i < N; i++) { w[i] = 0.5 * (1 - Math.cos(2 * Math.PI * i / (N - 1))); w2 += w[i] * w[i]; }
    let worst = 0;
    for (let k = 1; k < N / 2; k++) {
        let p = 0;
        for (const a of [st.x, st.y, st.z]) {
            let m = 0;
            for (let i = 0; i < N; i++) m += a[i];
            m /= N;
            let re = 0, im = 0;
            for (let i = 0; i < N; i++) {
                const v = (a[i] - m) * w[i];
                re += v * Math.cos(2 * Math.PI * k * i / N);
                im -= v * Math.sin(2 * Math.PI * k * i / N);
            }
            p += 2 * (re * re + im * im) / (fs * w2);
        }
        worst = Math.max(worst, Math.abs(10 * Math.log10(p) - db[k - 1]));
    }
    check('PSD = 10 log10(Σ 2|X|²/(fs Σw²)), bins 1 … N/2 − 1', db.length === N / 2 - 1 && worst < 1e-3, `max diff ${worst.toExponential(2)} dB`);
}

// ── 2. Parseval on white noise, and Welch flattening the floor ──────────
{
    const N = 1024, fs = 2000, sigma = 0.1, K = 8;
    const n = N + (K - 1) * N / 2;
    const st = stream(n, fs, () => [sigma * gauss(), sigma * gauss(), sigma * gauss()]);
    const expect = 10 * Math.log10(3 * 2 * sigma * sigma / fs);
    const mean = db => 10 * Math.log10(db.reduce((s, v) => s + Math.pow(10, v / 10), 0) / db.length);
    const spread = db => { const m = db.reduce((s, v) => s + v, 0) / db.length; return Math.sqrt(db.reduce((s, v) => s + (v - m) ** 2, 0) / db.length); };
    const one = welchPsdDb(st, N - 1, N, 1, fs);
    const avg = welchPsdDb(st, n - 1, N, K, fs);
    check('white noise: floor at 3 axes × 2σ²/fs', Math.abs(mean(avg) - expect) < 0.3, `${mean(avg).toFixed(2)} vs ${expect.toFixed(2)} dB`);
    check('Welch ×8 flattens the floor', spread(avg) < 0.5 * spread(one), `σ ${spread(one).toFixed(2)} → ${spread(avg).toFixed(2)} dB`);
}

// ── 3. A sine: its line where it should be, its power under it ──────────
{
    const N = 2048, fs = 1931.4, f0 = 187.3, A = 0.4;
    const st = stream(N, fs, t => [A * Math.sin(2 * Math.PI * f0 * t), 0, 0]);
    const db = welchPsdDb(st, N - 1, N, 1, fs);
    const df = fs / N;
    let best = 0;
    for (let i = 1; i < db.length; i++) if (db[i] > db[best]) best = i;
    const fPeak = (best + 1) * df;
    let p = 0;
    for (let i = best - 6; i <= best + 6; i++) p += Math.pow(10, db[i] / 10) * df;
    check('sine at 187.3 Hz: line in its bin', Math.abs(fPeak - f0) <= df / 2 + 1e-9, `${fPeak.toFixed(2)} Hz, df ${df.toFixed(3)} Hz`);
    check('sine power A²/2 under the line', Math.abs(p / (A * A / 2) - 1) < 0.03, `${p.toFixed(4)} vs ${(A * A / 2).toFixed(4)}`);
}

// ── 4. Resolution fitted to the runs ────────────────────────────────────
{
    const batch = { longest: 1024 };
    const a = fitResolution(batch, { n: 2048, avg: 1 });
    const b = fitResolution(batch, { n: 512, avg: 4 });
    const c = fitResolution(batch, { n: 1024, avg: 1 });
    check('batch of 1024: 2048 points come down to 1024', a.N === 1024 && a.K === 1 && a.limited);
    check('batch of 1024: 512 × 4 averages come down to 512 × 2 (1024 + 256 > 1024 → 768)', b.N === 512 && b.K === 2 && b.limited, JSON.stringify(b));
    check('batch of 1024: 1024 × 1 fits', c.N === 1024 && c.K === 1 && !c.limited);
    const s = sanitizeConfig({ n: 1000, avg: 3, stepMs: 'x' });
    check('settings off the lists fall back to the defaults', s.n === 1024 && s.avg === 1 && s.stepMs === 250);
}

// ── 5. A DataFlash file written here, read back ─────────────────────────
const SIZE = { Q: 8, q: 8, B: 1, b: 1, H: 2, h: 2, f: 4, I: 4, i: 4, Z: 64, N: 16, n: 4, a: 64 };
class Writer {
    constructor() { this.parts = []; this.fmts = {}; }
    fmt(type, name, format, labels) {
        const len = 3 + [...format].reduce((s, c) => s + SIZE[c], 0);
        const b = Buffer.alloc(89);
        b[0] = 0xA3; b[1] = 0x95; b[2] = 128; b[3] = type; b[4] = len;
        b.write(name, 5, 'ascii'); b.write(format, 9, 'ascii'); b.write(labels, 25, 'ascii');
        this.parts.push(b);
        this.fmts[name] = { type, len, format };
    }
    rec(name, values) {
        const f = this.fmts[name];
        const b = Buffer.alloc(f.len);
        b[0] = 0xA3; b[1] = 0x95; b[2] = f.type;
        let o = 3;
        [...f.format].forEach((c, i) => {
            const v = values[i];
            switch (c) {
                case 'Q': b.writeBigUInt64LE(BigInt(Math.round(v)), o); break;
                case 'B': b.writeUInt8(v, o); break;
                case 'H': b.writeUInt16LE(v, o); break;
                case 'f': b.writeFloatLE(v, o); break;
                case 'I': b.writeUInt32LE(v, o); break;
                case 'Z': case 'N': case 'n': b.write(String(v), o, 'ascii'); break;
                case 'a': for (let k = 0; k < 32; k++) b.writeInt16LE(v[k], o + 2 * k); break;
            }
            o += SIZE[c];
        });
        this.parts.push(b);
    }
    save(file) { fs.writeFileSync(file, Buffer.concat(this.parts)); }
}

const tmp = fs.mkdtempSync(path.join(os.tmpdir(), 'corv-logfft-'));
const binFile = path.join(tmp, 'imu.bin');
{
    const w = new Writer();
    w.fmt(64, 'PARM', 'QNf', 'TimeUS,Name,Value');
    w.fmt(65, 'ATT', 'Qfff', 'TimeUS,Roll,Pitch,Yaw');
    w.fmt(66, 'GYR', 'QBQfff', 'TimeUS,I,SampleUS,GyrX,GyrY,GyrZ');
    w.fmt(67, 'ACC', 'QBQfff', 'TimeUS,I,SampleUS,AccX,AccY,AccZ');
    w.fmt(68, 'IMU', 'QBffffffIIfBBHH', 'TimeUS,I,GyrX,GyrY,GyrZ,AccX,AccY,AccZ,EG,EA,T,GH,AH,GHz,AHz');
    w.fmt(69, 'ISBH', 'QHBBHHQf', 'TimeUS,N,type,instance,mul,smp_cnt,SampleUS,smp_rate');
    w.fmt(70, 'ISBD', 'QHHaaa', 'TimeUS,N,seqno,x,y,z');
    const T0 = 10e6;                                   // log starts 10 s after boot
    for (const [k, v] of [['INS_GYR_ID', 1], ['INS_GYR2_ID', 0], ['INS_ACC_ID', 1], ['INS_ACC2_ID', 0], ['INS_RAW_LOG_OPT', 9], ['INS_LOG_BAT_OPT', 5]]) w.rec('PARM', [T0, k, v]);
    // Raw gyro at 2000 Hz, 30 s, a 120 Hz line, with SampleUS in bursts of two
    // (as FIFO sensors log them) and a 0.2 s hole at 12 s; its post-filter
    // copy as instance 1 (one gyro, pre and post filter)
    const fsG = 2000, durS = 30;
    for (let i = 0; i < fsG * durS; i++) {
        const t = i / fsG;
        if (t >= 12 && t < 12.2) continue;
        const su = T0 + Math.floor(i / 2) * 1000 + (i % 2) * 20;    // pairs 20 µs apart, 980 µs between
        const g = 0.05 * Math.sin(2 * Math.PI * 120 * t) + 0.001 * gauss();
        w.rec('GYR', [su + 100, 0, su, g, 0.001 * gauss(), 0.001 * gauss()]);
        w.rec('GYR', [su + 100, 1, su, 0.2 * g, 0, 0]);
        if (i % 2 === 0) w.rec('ACC', [su + 100, 0, su, 0.5 * Math.sin(2 * Math.PI * 80 * t), 0, -9.8]);
        if (i % 40 === 0) {
            w.rec('ATT', [T0 + i * 500, 0, 0, 0]);
            // IMU1 at 50 Hz; IMU2 with a 7 Hz line and two records in every 7 dropped by the logger
            w.rec('IMU', [T0 + i * 500, 0, g, 0, 0, 0, 0, -9.8, 0, 0, 25, 1, 1, 2000, 1000]);
            const k = i / 40;
            if (k % 7 < 5) w.rec('IMU', [T0 + i * 500, 1, 0.1 * Math.sin(2 * Math.PI * 7 * t), 0, 0, 0, 0, -9.8, 0, 0, 25, 1, 1, 2000, 1000]);
        }
    }
    // Batch gyro: 1024 samples at 3276.8 Hz every 2 s, a 300 Hz line; one
    // chunk of the third batch lost
    const fsB = 3276.8, mul = 938;
    for (let bN = 0; bN < 10; bN++) {
        const su = T0 + 1e6 + bN * 2e6;
        w.rec('ISBH', [su + 400e3, bN, 1, 0, mul, 1024, su, fsB]);
        for (let c = 0; c < 32; c++) {
            if (bN === 2 && c === 20) continue;
            const v = k => Math.round(mul * 0.3 * Math.sin(2 * Math.PI * 300 * (c * 32 + k) / fsB));
            const x = Array.from({ length: 32 }, (_, k) => v(k));
            w.rec('ISBD', [su + 400e3 + c * 20e3, bN, c, x, new Array(32).fill(0), new Array(32).fill(0)]);
        }
    }
    w.save(binFile);
}

const parsed = indexBinFile(binFile);
const S = parsed.imuStreams;
const byKey = (list, key) => list.find(s => s.key === key);
{
    const raw = byKey(S.gyro, 'GYR:0'), post = byKey(S.gyro, 'GYR:1'), bat = byKey(S.gyro, 'ISB:1:0'), loop = byKey(S.gyro, 'IMU:0');
    check('gyro streams: raw, post-filter, batch, loop rate', raw && post && bat && loop, S.gyro.map(s => s.label).join(' | '));
    check('best gyro stream first: IMU1 raw', S.gyro[0] === raw);
    check('raw gyro rate from bursty SampleUS', raw && Math.abs(raw.fs - 2000) < 1, raw && raw.fs.toFixed(2));
    check('instance 1 with one gyro and pre+post filter = IMU1 filtered', post && post.imu === 0 && post.filtered && /IMU1 · filtered/.test(post.label), post && post.label);
    check('the 0.2 s hole splits the raw gyro in two runs', raw && raw.runs.length === 2, raw && JSON.stringify(raw.runs.map(r => r.count)));
    check('raw gyro times on the replay base (log starts at 10 s after boot)', raw && Math.abs(raw.t[0]) < 1, raw && raw.t[0].toFixed(3));
    check('batch rate and times', bat && Math.abs(bat.fs - 3276.8) < 0.01 && Math.abs(bat.t[0] - 1000) < 1e-6, bat && `${bat.fs} Hz, first at ${bat.t[0]} ms`);
    check('batch with a lost chunk: kept as its two runs (640 + 352)', bat && bat.runs.filter(r => r.count < 1024).map(r => r.count).join() === '640,352', bat && JSON.stringify(bat.runs.map(r => r.count)));
    const peak = bat && Math.max(...bat.x.subarray(0, 1024));
    check('batch samples scaled by ISBH.mul', Math.abs(peak - 0.3) < 0.002, peak && peak.toFixed(4));
    check('accel stream: ACC raw at 1000 Hz', byKey(S.accel, 'ACC:0') && Math.abs(byKey(S.accel, 'ACC:0').fs - 1000) < 1);
    check('IMU at loop rate, gyro and accel', loop && byKey(S.accel, 'IMU:0') && Math.abs(loop.fs - 50) < 0.1, loop && loop.label);
    const lossy = byKey(S.gyro, 'IMU:1');
    check('IMU records with 2 in 7 dropped: one run per side of the 12 s hole, resampled at 50 Hz', lossy && lossy.runs.length === 2 && Math.abs(lossy.fs - 50) < 0.01, lossy && `${lossy.runs.length} runs, ${lossy.fs.toFixed(2)} Hz`);
    if (lossy) {
        const N = 256, run = lossy.runs[1];
        const db = welchPsdDb(lossy, run.start + N - 1, N, 1, lossy.fs);
        let best = 0;
        for (let i = 1; i < db.length; i++) if (db[i] > db[best]) best = i;
        check('…and its 7 Hz line where it should be', Math.abs((best + 1) * 50 / N - 7) < 50 / N, `${((best + 1) * 50 / N).toFixed(2)} Hz`);
    }
    const replayed = new Set(parsed.index.map(e => e.msgId));
    check('IMU samples are not replayed as messages (only ATT and HEARTBEAT)', replayed.size === 2 && replayed.has(30) && replayed.has(0), [...replayed].join());
}

// ── 6. The scheduler ────────────────────────────────────────────────────
{
    const sent = [];
    let now = 0;
    const ls = new LogSpectrum((ch, p) => sent.push(p));
    ls.setConfig({ n: 1024, avg: 1, stepMs: 250 });
    ls.load(S, () => now);
    ls.stop();                                        // driven by hand below
    const reset = sent.find(p => p.reset);
    check('load: a reset with both sensors and their streams', reset && reset.log.sensors.gyro.stream === 'GYR:0' && reset.log.sensors.accel.stream === 'ACC:0');
    check('gyro resolution: 1024 points at 2000 Hz = 1.95 Hz, 511 bins', Math.abs(reset.log.sensors.gyro.df - 1.953) < 0.01 && reset.log.sensors.gyro.n === 511);

    sent.length = 0;
    now = 20000;
    ls.restart();
    for (let k = 0; k < 40; k++) ls.pump();
    const rows = sent.filter(p => p.rows).flatMap(p => p.rows).filter(r => r.sensor === 'gyro');
    const ts = rows.map(r => r.t);
    check('after a seek to 20 s: the spectra of 0 … 20 s', rows.length > 60 && ts[0] < 1000 && ts[ts.length - 1] <= 20000, `${rows.length} rows, ${ts[0].toFixed(0)} … ${ts[ts.length - 1].toFixed(0)} ms`);
    check('one spectrum every 250 ms', ts.slice(1).every((t, i) => t > ts[i]) && Math.abs((ts[ts.length - 1] - ts[0]) / (ts.length - 1) - 250) < 20);
    // after the hole the first window needs 1024 samples = 512 ms of new data
    check('no window across the 12 s hole', !rows.some(r => r.t > 12000 && r.t < 12200 + 511), rows.filter(r => r.t > 11500 && r.t < 13000).map(r => r.t.toFixed(0)).join(' '));
    const r0 = rows.find(r => r.t > 5000);
    let best = 0;
    for (let i = 1; i < r0.db.length; i++) if (r0.db[i] > r0.db[best]) best = i;
    check('the 120 Hz line in the log spectrum', Math.abs((best + 1) * 2000 / 1024 - 120) < 2, `${((best + 1) * 2000 / 1024).toFixed(1)} Hz`);

    // The batch stream: one spectrum per batch, at its last sample
    sent.length = 0;
    ls.selectStream('gyro', 'ISB:1:0');
    for (let k = 0; k < 40; k++) ls.pump();
    const brows = sent.filter(p => p.rows).flatMap(p => p.rows).filter(r => r.sensor === 'gyro');
    const sel = sent.find(p => p.reset).log.sensors.gyro;
    check('batch stream selected: 1024 points at 3277 Hz', sel.stream === 'ISB:1:0' && sel.N === 1024 && !sel.limited);
    check('one spectrum per whole batch (the one with a lost chunk has none)', brows.length === 9 && brows.every(r => Math.abs(((r.t - 1000) % 2000) - 1023 / 3.2768) < 1), brows.map(r => r.t.toFixed(0)).join(' '));

    // Rewind (play again from the end) without a seek: start over
    sent.length = 0;
    now = 3000;
    ls.pump();
    check('cursor back by more than 1 s: start over', sent.some(p => p.reset));
    ls.unload();
    check('unload: a reset with no log', sent[sent.length - 1].reset && sent[sent.length - 1].log === null);
}

fs.rmSync(tmp, { recursive: true, force: true });
console.log(failures ? `\n${failures} FAILED` : '\nall passed');
process.exit(failures ? 1 : 0);
