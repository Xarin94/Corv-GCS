#!/usr/bin/env node
/**
 * test-pose-smoother.js - offline checks for js/core/PoseSmoother.js
 *
 * Feeds telemetry-rate samples (25 Hz, jittered, in bursts, at 5 Hz, with
 * and without the vehicle clock and body rates) and renders at 60 fps, the
 * way the 3D view does, then checks that the drawn pose moves every frame at
 * the vehicle's speed, stays on the true track through a turn, snaps on a
 * seek and stops projecting when the link goes quiet.
 *
 *   node scripts/test-pose-smoother.js
 */
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const { pathToFileURL } = require('node:url');

const root = path.join(__dirname, '..');
const tmp = fs.mkdtempSync(path.join(os.tmpdir(), 'corv-pose-'));
fs.writeFileSync(path.join(tmp, 'package.json'), '{"type":"module"}');
fs.copyFileSync(path.join(root, 'js/core/PoseSmoother.js'), path.join(tmp, 'PoseSmoother.js'));

let failures = 0;
function check(name, cond, detail = '') {
    console.log(`${cond ? 'PASS' : 'FAIL'}  ${name}${detail ? '  ' + detail : ''}`);
    if (!cond) failures++;
}

const M = 111320, LAT0 = 45, LON0 = 10, MLON = M * Math.cos(LAT0 * Math.PI / 180);
const TAU = 0.075, FRAME = 1000 / 60, G = 9.81;
const V = 70;
const TURN = 20 * Math.PI / 180;   // rad/s: a coordinated turn at 70 m/s pulls 24 m/s²

/**
 * The vehicle at time t (ms): 70 m/s from heading 45°, climbing 2 m/s,
 * turning at `turn` rad/s in a coordinated bank (position on the arc,
 * NED velocity, body rates) — the telemetry an ArduPilot sends.
 */
function truth(t, turn = 0) {
    const s = t / 1000, h0 = Math.PI / 4, hdg = h0 + turn * s;
    let n, e;
    if (turn) { const r = V / turn; n = r * (Math.sin(hdg) - Math.sin(h0)); e = r * (Math.cos(h0) - Math.cos(hdg)); }
    else { n = Math.cos(h0) * V * s; e = Math.sin(h0) * V * s; }
    const roll = Math.atan(V * turn / G), pitch = 0.05;
    return {
        lat: LAT0 + n / M, lon: LON0 + e / MLON, rawAlt: 300 + 2 * s,
        vn: Math.cos(hdg) * V, ve: Math.sin(hdg) * V, vs: 2, gs: V, track: hdg,
        roll, pitch, yaw: ((hdg % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI),
        // Euler yaw rate ψ' with φ' = θ' = 0 is, in body axes, q = ψ' sinφ cosθ, r = ψ' cosφ cosθ
        rollRate: -Math.sin(pitch) * turn, pitchRate: turn * Math.sin(roll) * Math.cos(pitch), yawRate: turn * Math.cos(roll) * Math.cos(pitch)
    };
}

function metres(a, b) {
    return Math.hypot((a.lat - b.lat) * M, (a.lon - b.lon) * MLON);
}

/** take: when the vehicle sampled it; at: when it reached STATE (ms). */
function every(ms, seconds, at = take => take + 5) {
    const out = [];
    for (let take = 0; take <= seconds * 1000; take += ms) out.push({ take, at: at(take) });
    return out.sort((a, b) => a.at - b.at);
}

/** Render at 60 fps while the arrivals land in STATE; per-frame drawn speed, error and yaw rate. */
function fly(PoseSmoother, arrivals, { seconds = 4, tau = TAU, turn = 0, rates = true, boot = true } = {}) {
    const smoother = new PoseSmoother(), state = { ...truth(0, turn), posTime: 0, attTime: 0, posBootMs: null, attBootMs: null };
    const out = { speeds: [], errors: [], yawRates: [], yawErrors: [] };
    let next = 0, prev = null;
    for (let t = 0; t <= seconds * 1000; t += FRAME) {
        while (next < arrivals.length && arrivals[next].at <= t) {
            const a = arrivals[next++], sample = truth(a.take, turn);
            if (!rates) sample.rollRate = sample.pitchRate = sample.yawRate = 0;
            Object.assign(state, sample, { posTime: a.at, attTime: a.at,
                posBootMs: boot ? 10000 + a.take : null, attBootMs: boot ? 10000 + a.take : null });
        }
        const pose = { ...smoother.update(state, t, tau) };
        if (prev && t > 600) {
            const real = truth(t, turn);
            out.speeds.push(metres(pose, prev) / (FRAME / 1000));
            out.errors.push(metres(pose, real));
            let dy = pose.yaw - prev.yaw; dy = Math.atan2(Math.sin(dy), Math.cos(dy));
            out.yawRates.push(dy / (FRAME / 1000));
            out.yawErrors.push(Math.abs(Math.atan2(Math.sin(pose.yaw - real.yaw), Math.cos(pose.yaw - real.yaw))));
        }
        prev = pose;
    }
    return out;
}

const mean = a => a.reduce((x, y) => x + y, 0) / a.length;
const cv = a => { const m = mean(a); return Math.sqrt(mean(a.map(x => (x - m) ** 2))) / Math.abs(m); };
const still = a => a.filter(x => x < V * 0.25).length / a.length;
const max = a => Math.max(...a);
const deg = r => (r * 180 / Math.PI).toFixed(2) + '°';
// Seeded jitter, so a failure reproduces
let seed = 7;
const rand = () => (seed = (seed * 16807) % 2147483647) / 2147483647;

(async () => {
    const { PoseSmoother } = await import(pathToFileURL(path.join(tmp, 'PoseSmoother.js')));

    // ---------- 25 Hz, steady, straight ----------
    const steady = every(40, 5);
    const raw = fly(PoseSmoother, steady, { tau: 0 });
    check('raw telemetry at 25 Hz stands still most frames (the bug)', still(raw.speeds) > 0.5, (still(raw.speeds) * 100).toFixed(0) + ' % still');
    const s25 = fly(PoseSmoother, steady);
    check('25 Hz: moves every frame', still(s25.speeds) === 0);
    check('25 Hz: even speed', cv(s25.speeds) < 0.02, 'cv ' + cv(s25.speeds).toFixed(3));
    check('25 Hz: on the true track', max(s25.errors) < 0.5, 'max ' + max(s25.errors).toFixed(2) + ' m');

    // ---------- 25 Hz with ±15 ms arrival jitter ----------
    const jitter = every(40, 5, take => take + 20 + (rand() - 0.5) * 30);
    const jb = fly(PoseSmoother, jitter);
    const jn = fly(PoseSmoother, jitter, { boot: false });
    check('jitter, vehicle clock: even speed', cv(jb.speeds) < 0.02, 'cv ' + cv(jb.speeds).toFixed(3));
    check('jitter, vehicle clock: on the track', max(jb.errors) < 1.6, 'max ' + max(jb.errors).toFixed(2) + ' m (the link delay itself)');
    check('jitter, arrival times only: moves every frame, speed within 15 %', still(jn.speeds) === 0 && cv(jn.speeds) < 0.15, 'cv ' + cv(jn.speeds).toFixed(3));

    // ---------- cellular bursts: 4 samples land together every 160 ms ----------
    const bursts = every(40, 5, take => Math.ceil((take + 1) / 160) * 160 + 30);
    const rb = fly(PoseSmoother, bursts, { tau: 0 });
    for (const boot of [true, false]) {
        const sb = fly(PoseSmoother, bursts, { boot });
        check(`bursts${boot ? '' : ', arrival times only'}: moves every frame, speed within 10 %`, still(sb.speeds) === 0 && cv(sb.speeds) < 0.1,
            'cv ' + cv(sb.speeds).toFixed(3) + ', raw ' + (still(rb.speeds) * 100).toFixed(0) + ' % still');
    }

    // ---------- 20°/s coordinated turn: acceleration and body rates ----------
    const t25 = fly(PoseSmoother, steady, { turn: TURN });
    check('turn at 25 Hz: on the arc', max(t25.errors) < 0.5, 'max ' + max(t25.errors).toFixed(2) + ' m');
    check('turn at 25 Hz: yaw turns at the turn rate every frame', cv(t25.yawRates) < 0.05 && Math.abs(mean(t25.yawRates) - TURN) < 0.01,
        'cv ' + cv(t25.yawRates).toFixed(3) + ', ' + deg(mean(t25.yawRates)) + '/s');
    check('turn at 25 Hz: yaw on the true heading', max(t25.yawErrors) < 0.01, 'max ' + deg(max(t25.yawErrors)));
    // 5 Hz: 200 ms between fixes, where velocity alone cuts the corner
    const slow = every(200, 5);
    const t5 = fly(PoseSmoother, slow, { turn: TURN });
    const t5raw = fly(PoseSmoother, slow, { turn: TURN, tau: 0 });
    check('turn at 5 Hz: on the arc with the acceleration', max(t5.errors) < 0.5,
        'max ' + max(t5.errors).toFixed(2) + ' m, raw ' + max(t5raw.errors).toFixed(1) + ' m');
    check('turn at 5 Hz: moves every frame', still(t5.speeds) === 0 && cv(t5.speeds) < 0.05, 'cv ' + cv(t5.speeds).toFixed(3));

    // ---------- attitude from a source without rates (CORV-2 sends roll, pitch, yaw only) ----------
    const nr = fly(PoseSmoother, steady, { turn: TURN, rates: false });
    const nrRaw = fly(PoseSmoother, steady, { turn: TURN, rates: false, tau: 0 });
    check('no rates: yaw rate derived from the samples, smooth', cv(nr.yawRates) < 0.05 && cv(nrRaw.yawRates) > 1,
        'cv ' + cv(nr.yawRates).toFixed(3) + ' vs raw ' + cv(nrRaw.yawRates).toFixed(2));
    check('no rates: yaw on the true heading', max(nr.yawErrors) < 0.01, 'max ' + deg(max(nr.yawErrors)));

    // ---------- yaw through north in MAVLink's -π..π ----------
    {
        const smoother = new PoseSmoother(), state = { ...truth(0), yaw: -0.02, posTime: 0, attTime: 0, rollRate: 0, pitchRate: 0, yawRate: 0.5, roll: 0, pitch: 0 };
        let worst = 0, prev = smoother.update(state, 0, TAU).yaw;
        for (let t = FRAME; t < 1000; t += FRAME) {
            if (Math.floor(t / 40) !== Math.floor((t - FRAME) / 40)) {
                const y = -0.02 + 0.5 * t / 1000;
                state.yaw = y > Math.PI ? y - 2 * Math.PI : y; state.attTime = t;
            }
            const y = smoother.update(state, t, TAU).yaw;
            let d = y - prev; d = Math.atan2(Math.sin(d), Math.cos(d));
            worst = Math.max(worst, Math.abs(d)); prev = y;
        }
        check('yaw through north: no spin', worst < 0.05, 'largest step ' + worst.toFixed(3) + ' rad');
    }

    // ---------- seek / new vehicle: jump, no glide ----------
    {
        const smoother = new PoseSmoother(), state = { ...truth(0), posTime: 0, attTime: 0 };
        smoother.update(state, 0, TAU);
        Object.assign(state, { lat: LAT0 + 0.05, posTime: 40 });
        const pose = smoother.update(state, 41, TAU);
        check('seek of 5.5 km snaps', Math.abs(pose.lat - (LAT0 + 0.05)) * M < 1);
    }

    // ---------- link lost: projection stops ----------
    {
        const smoother = new PoseSmoother(), state = { ...truth(0, TURN), posTime: 0, attTime: 0 };
        let t = 0;
        for (; t <= 1000; t += 40) { Object.assign(state, truth(t, TURN), { posTime: t, attTime: t, posBootMs: t, attBootMs: t }); smoother.update(state, t, TAU); }
        t -= 40;
        const last = { ...state };
        const at = ms => ({ ...smoother.update(state, t + ms, TAU) });
        const p1 = at(250), p2 = at(500), p3 = at(750), p4 = at(6000);
        const ahead = metres(p2, last);
        check('link lost: projection goes on for 0.5 s', metres(p1, last) > V * 0.22 && ahead > V * 0.45, ahead.toFixed(1) + ' m ahead after 0.5 s');
        // Only what is left of the blend still moves it, by micrometres
        check('link lost: and no further', metres(p2, p3) < 0.01 && metres(p2, p4) < 0.01, (metres(p2, p4) * 1000).toFixed(3) + ' mm after');
        const offArc = metres(p2, truth(t + 500, TURN));
        check('link lost in a 20°/s turn: still on the arc after 0.5 s', offArc < 0.5, offArc.toFixed(2) + ' m off');
    }

    // ---------- STATE written every frame (demo flight): drawn as it is ----------
    {
        const smoother = new PoseSmoother(), state = { ...truth(0, TURN), posTime: 0, attTime: 0 };
        let worst = 0;
        for (let t = 0; t < 2000; t += FRAME) {
            Object.assign(state, truth(t, TURN));
            worst = Math.max(worst, metres(smoother.update(state, t, TAU), state));
        }
        check('per-frame STATE: drawn on it', worst < 0.05, worst.toFixed(3) + ' m');
    }

    // ---------- stamps left behind by a replay, then the demo writes STATE every frame ----------
    {
        const smoother = new PoseSmoother(), state = { ...truth(0), posTime: 0, attTime: 0 };
        for (let t = 0; t <= 1000; t += 40) { Object.assign(state, truth(t), { posTime: t, attTime: t, posBootMs: 5000 + t, attBootMs: 5000 + t }); smoother.update(state, t, TAU); }
        let worst = 0;
        for (let t = 1000; t < 6000; t += FRAME) {
            Object.assign(state, truth(t));   // posTime, posBootMs stay where the replay left them
            worst = Math.max(worst, metres(smoother.update(state, t, TAU), state));
        }
        check('stale vehicle stamps: per-frame STATE drawn on it', worst < 0.5, worst.toFixed(2) + ' m');
    }

    // ---------- smoothing off ----------
    {
        const smoother = new PoseSmoother(), state = { ...truth(1234), posTime: 0, attTime: 0 };
        const pose = smoother.update(state, 5000, 0);
        check('smoothing 0: STATE exactly', pose.lat === state.lat && pose.lon === state.lon && pose.alt === state.rawAlt && pose.yaw === state.yaw);
    }

    fs.rmSync(tmp, { recursive: true, force: true });
    console.log(failures ? `\n${failures} FAILED` : '\nall passed');
    process.exitCode = failures ? 1 : 0;
})().catch(error => { console.error(error); fs.rmSync(tmp, { recursive: true, force: true }); process.exitCode = 1; });
