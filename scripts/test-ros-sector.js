#!/usr/bin/env node
/**
 * test-ros-sector.js - a sector-scanning sonar on a moving boat, through the
 * GCS's own worker
 *
 * A mechanical sector sonar sends one beam per ping, 15 a second, so the
 * beams of one LaserScan were measured over a second: a boat at 2 m/s moves
 * 2 m in it. (A 60° sector here, a sweep every 2.2 s, so a few cross the
 * wreck.) This drives js/ros/RosWorker.js itself (in Node, with a stub
 * `self`) against scripts/rosbridge-sim.js on a straight track over the Garda
 * motorboat wreck — the same track fed to the worker as telemetry — and
 * compares the averaged surface with the scene:
 *
 *   timed    the emulator's LaserScans carry time_increment: every beam is
 *            placed with the pose of its own time
 *   untimed  time_increment 0 (a republisher that forgot it): every beam of a
 *            message with one pose, the wreck smeared along the track
 *   drift    the telemetry's altitude 0.4 m high (a boat's EKF drifting with
 *            the barometer): the bed comes out 0.4 m shallow with VERTICAL
 *            'vehicle', where it is with 'water' (the water surface at home)
 *   stalls   the GCS's main thread held 0.8 s every 5 s: the telemetry that
 *            came in meanwhile reaches the worker late, all at once — every
 *            message still finds its pose (the samples are timed by the
 *            autopilot's clock, not by their arrival)
 *
 *   node scripts/test-ros-sector.js
 */

const path = require('path');
const fs = require('fs');
const os = require('os');
const { spawn, spawnSync } = require('child_process');
const { pathToFileURL } = require('url');

if (typeof WebSocket === 'undefined') {
    const r = spawnSync(process.execPath, ['--experimental-websocket', __filename, ...process.argv.slice(2)], { stdio: 'inherit' });
    process.exit(r.status ?? 1);
}

const { SCENES, GARDA } = require('./ros-sim-scenes');

const ROOT = path.join(__dirname, '..');
const DEG = Math.PI / 180;
const WRECK = GARDA.objects.find(o => o.name.startsWith('Motorboat'));
// From 25 m before the wreck to 35 m past it, across its keel, at 2 m/s
const TRACK = { heading: 130, speed: 2, before: 25, seconds: 30 };
TRACK.e = WRECK.e - TRACK.before * Math.sin(TRACK.heading * DEG);
TRACK.n = WRECK.n - TRACK.before * Math.cos(TRACK.heading * DEG);
const CELL = 0.5;

// The track at time t (ms): scene metres, and lat/lon as the emulator converts them
function trackAt(t, t0) {
    const s = TRACK.speed * (t - t0) / 1000, a = TRACK.heading * DEG;
    const e = TRACK.e + s * Math.sin(a), n = TRACK.n + s * Math.cos(a);
    return { e, n, ...GARDA.toLatLon(e, n), vn: TRACK.speed * Math.cos(a), ve: TRACK.speed * Math.sin(a), yaw: a };
}

// ---------- child: one worker run, results as JSON on stdout ----------
async function child(port, t0, altBias = 0, vertical = 'vehicle', stalls = false) {
    // The worker's modules as ES modules in a scratch folder (this package is CommonJS)
    const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'corv-ros-worker-'));
    fs.writeFileSync(path.join(dir, 'package.json'), '{"type":"module"}');
    for (const rel of ['js/ros/RosWorker.js', 'js/ros/RosPoints.js', 'js/ros/SurfaceTiles.js', 'js/ros/SurfaceVolume.js', 'js/ros/SurfaceRaster.js', 'vendor/roslib/roslib.esm.min.js']) {
        fs.mkdirSync(path.join(dir, path.dirname(rel)), { recursive: true });
        fs.copyFileSync(path.join(ROOT, rel), path.join(dir, rel));
    }
    const tiles = new Map();
    let anchor = null, cell = CELL, status = null, noPose = 0;
    const misses = new Map();         // the worker's own account of each message with no pose
    globalThis.self = {
        postMessage(m) {
            if (m.op === 'status') {
                status = m.st;
                noPose += (m.st.rate && m.st.rate.noPose) || 0;
                if (m.st.poseMiss) misses.set(m.st.poseMiss.at, { ...m.st.poseMiss, sinceStart: m.st.poseMiss.at - t0 });
            }
            if (m.op !== 'tiles') return;
            if (m.reset) tiles.clear();
            anchor = m.anchor || anchor; cell = m.cell;
            for (const [tx, ty] of m.removed || []) tiles.delete(`${tx},${ty}`);
            for (const t of m.tiles) tiles.set(`${t.tx},${t.ty}`, t);
        }
    };
    await import(pathToFileURL(path.join(dir, 'js/ros/RosWorker.js')).href);
    const W = (data) => globalThis.self.onmessage({ data });
    W({ op: 'config', cfg: {
        transport: 'cbor', rateHz: 10, maxPoints: 1000, frame: 'auto', lagMs: 0, timeBase: 'arrival',
        minRange: 0.75, maxRange: 100, despike: 'on', mountRoll: 0, mountPitch: -90, mountYaw: 0,
        leverX: 0, leverY: 0, leverZ: 0, cell: CELL, memory: 20, maxTiles: 2048, layers: 'floor', vertical
    } });
    W({ op: 'connect', url: `ws://127.0.0.1:${port}` });
    W({ op: 'select', topic: '/sonar/sector', type: 'sensor_msgs/msg/LaserScan' });
    // Telemetry at 25 Hz: the track, level, the autopilot's clock = this one.
    // With stalls, every 5 s the samples are held 0.8 s, then handed over
    // together, each stamped with the time it was handed over
    const held = [];
    const feed = setInterval(() => {
        const now = Date.now(), k = trackAt(now, t0), b = now - t0;
        const home = { lat: GARDA.origin.lat, lon: GARDA.origin.lon, alt: 0 };
        held.push({ b, k });
        if (stalls && now - t0 > 1000 && (now - t0) % 5000 < 800) return;
        const att = [], pos = [];
        for (const h of held.splice(0)) {
            att.push({ t: now, b: h.b, roll: 0, pitch: 0, yaw: h.k.yaw, p: 0, q: 0, r: 0 });
            pos.push({ t: now, b: h.b, lat: h.k.lat, lon: h.k.lng, alt: altBias, vn: h.k.vn, ve: h.k.ve, vd: 0 });
        }
        W({ op: 'pose',
            p: { tA: now, tP: now, lat: k.lat, lon: k.lng, alt: altBias, roll: 0, pitch: 0, yaw: k.yaw, p: 0, q: 0, r: 0, vn: k.vn, ve: k.ve, vd: 0, rel: false, home },
            att, pos });
    }, 40);
    await new Promise(r => setTimeout(r, t0 + TRACK.seconds * 1000 - Date.now()));
    clearInterval(feed);
    await new Promise(r => setTimeout(r, 1100));         // the last tiles and status
    const st = status;
    W({ op: 'disconnect' });
    fs.rmSync(dir, { recursive: true, force: true });

    // Every filled cell against the scene under its centre. Level cells (the
    // scene within 10 cm over the cell and its neighbours) measure placement;
    // cells on the wreck measure whether it is where it is.
    const TILE = 30, errs = [], onWreck = [];
    let all = 0, off = 0;
    for (const t of tiles.values()) {
        for (let k = 0; k < TILE * TILE; k++) {
            if (!(t.w[k] > 0)) continue;
            const i = k % TILE, j = (k - i) / TILE;
            const x = (t.tx * TILE + i + 0.5) * cell, y = (t.ty * TILE + j + 0.5) * cell;
            const [e, n] = GARDA.toEN(anchor.lat + y / anchor.mPerLat, anchor.lon + x / anchor.mPerLon);
            const hs = [];
            for (let a = -1; a <= 1; a++) for (let c = -1; c <= 1; c++) hs.push(SCENES.garda.height(e + a * cell, n + c * cell));
            const level = Math.max(...hs) - Math.min(...hs) < 0.1;
            const err = t.h[k] - SCENES.garda.height(e, n);
            if (level) errs.push(err);
            all++;
            if (Math.abs(err) > 0.5) off++;
            if (SCENES.garda.height(e, n) - SCENES.garda.bed(e, n) > 1) onWreck.push(t.h[k] - SCENES.garda.bed(e, n));
        }
    }
    // (the surface is relative to its anchor, the first pose: with a biased
    // altitude, back to the water surface the scene's 0 is)
    const zero = anchor.alt;
    for (let k = 0; k < errs.length; k++) errs[k] += zero;
    const abs = errs.map(Math.abs).sort((a, b) => a - b), q = (f) => abs[Math.floor(f * (abs.length - 1))];
    const mean = errs.reduce((a, x) => a + x, 0) / errs.length;
    const rms = Math.sqrt(errs.reduce((a, x) => a + x * x, 0) / errs.length);
    process.stdout.write(JSON.stringify({
        cells: errs.length, rms, mean, p50: q(0.5), p95: q(0.95), worst: q(1), all, off,
        wreck: onWreck.length, wreckSeen: onWreck.filter(h => h > 1).length,
        rate: st && st.rate, timedSpan: st && st.timedSpan, state: st && st.state, noPose, clockRate: st && st.clockRate, misses: [...misses.values()]
    }) + '\n');
    process.exit(0);
}

// ---------- parent ----------
let failures = 0;
function check(name, cond, detail = '') {
    console.log(`${cond ? 'PASS' : 'FAIL'}  ${name}${detail ? '  ' + detail : ''}`);
    if (!cond) failures++;
}

async function main() {
    if (process.argv[2] === '--child') return child(parseInt(process.argv[3], 10), parseFloat(process.argv[4]), parseFloat(process.argv[5] || '0'), process.argv[6] || 'vehicle', process.argv[7] === 'stalls');
    const t0 = Date.now() + 1500;
    const sims = [], kids = [];
    process.on('exit', () => { for (const p of [...sims, ...kids]) p.kill(); });
    const sim = (port, extra) => new Promise((resolve, reject) => {
        const p = spawn(process.execPath, [path.join(__dirname, 'rosbridge-sim.js'), '--port', String(port), '--scene', 'garda',
            '--anchor', `${GARDA.origin.lat},${GARDA.origin.lon},0`, '--pose', `${TRACK.e},${TRACK.n},0,${TRACK.heading}`,
            '--speed', String(TRACK.speed), '--pose-t0', String(t0), '--sonar-clean', '--sector-aperture', '60', ...extra], { stdio: ['ignore', 'pipe', 'inherit'] });
        sims.push(p);
        p.stdout.on('data', (d) => { if (/listening/.test(String(d))) resolve(); });
        setTimeout(() => reject(new Error('emulator did not start')), 5000);
    });
    const run = (port, bias = 0, vertical = 'vehicle', stalls = false) => new Promise((resolve) => {
        const p = spawn(process.execPath, ['--experimental-websocket', __filename, '--child', String(port), String(t0), String(bias), vertical, stalls ? 'stalls' : ''], { stdio: ['ignore', 'pipe', 'inherit'] });
        kids.push(p);
        let out = '';
        p.stdout.on('data', (d) => { out += d; });
        p.on('exit', () => { try { resolve(JSON.parse(out.trim().split('\n').pop())); } catch (_) { resolve(null); } });
    });
    await sim(19195, []);
    await sim(19196, ['--sector-untimed']);
    console.log(`boat at ${TRACK.speed} m/s over the ${WRECK.name} (${WRECK.depth.toFixed(1)} m, ${WRECK.proud.toFixed(1)} m proud) for ${TRACK.seconds} s …`);
    const [timed, untimed, driftVehicle, driftWater, stalled] = await Promise.all([run(19195), run(19196), run(19195, 0.4, 'vehicle'), run(19195, 0.4, 'water'), run(19195, 0, 'vehicle', true)]);
    if (!timed || !untimed || !driftVehicle || !driftWater || !stalled) { check('worker runs', false, JSON.stringify({ timed, untimed, driftVehicle, driftWater, stalled })); process.exit(1); }
    const f = (r) => `${r.cells} level cells: |error| median ${r.p50.toFixed(3)} m, 95 % ${r.p95.toFixed(2)} m, RMS ${r.rms.toFixed(2)} m · `
        + `${r.off} of ${r.all} cells off by > 0.5 m · wreck ${r.wreckSeen}/${r.wreck} cells over 1 m`;
    check('timed: the worker reads time_increment', timed.timedSpan > 0.5 && timed.state === 'ACCUMULATING' && untimed.timedSpan === 0,
        `beams over ${timed.timedSpan.toFixed(2)} s a message · untimed ${untimed.timedSpan} s`);
    check('timed: every beam with its own pose → the bed where it is', timed.cells > 150 && timed.p50 < 0.04 && timed.p95 < 0.1, f(timed));
    check('timed: the wreck stands where it lies', timed.wreck >= 5 && timed.wreckSeen / timed.wreck > 0.8 && timed.off <= 0.05 * timed.all,
        `${timed.wreckSeen} of ${timed.wreck} cells over the wreck read over 1 m above the bed · ${timed.off} cells off`);
    // (the level bed hardly cares where along the track a beam lands; the wreck does)
    check('untimed: the same beams with one pose a message: the wreck smeared', untimed.rms > 2 * timed.rms
        && untimed.wreckSeen / untimed.wreck < timed.wreckSeen / timed.wreck, f(untimed));
    check('altitude 0.4 m high, VERTICAL vehicle: the bed 0.4 m shallow', Math.abs(driftVehicle.mean - 0.4) < 0.05, `mean error ${driftVehicle.mean.toFixed(3)} m`);
    // (before the first telemetry, a message has no pose in either run)
    check('main thread held 0.8 s every 5 s: every message still has its pose, the bed where it is', stalled.noPose <= timed.noPose
        && stalled.p95 < 1.5 * timed.p95 + 0.01 && stalled.wreckSeen / stalled.wreck > 0.8,
        `${stalled.noPose} messages without pose (${timed.noPose} without the holds) · ${f(stalled)} · clock ×${stalled.clockRate.toFixed(3)}`
        + (stalled.misses.length ? ` · ${JSON.stringify(stalled.misses)}` : ''));
    check('altitude 0.4 m high, VERTICAL water surface: the bed where it is', Math.abs(driftWater.mean - timed.mean) < 0.02 && Math.abs(driftWater.mean) < 0.05,
        `mean error ${driftWater.mean.toFixed(3)} m (no drift: ${timed.mean.toFixed(3)} m)`);
    console.log(failures ? `\n${failures} check(s) FAILED` : '\nall checks passed');
    process.exit(failures ? 1 : 0);
}

main();
