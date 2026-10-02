#!/usr/bin/env node
/**
 * test-ros-surface.js - offline checks for the ROS surface (js/ros/)
 *
 *  1. SurfaceTiles: 30 cm cells in 30 × 30 tiles, averaging, the moving
 *     average once a cell's memory is full, incremental changes, the tile
 *     budget, a small detail kept.
 *  2. RosPoints: PointCloud2 (livox_ros_driver2 layout, base64 and binary,
 *     float64 big endian), LaserScan (typed array, JSON nulls), Range,
 *     sampling, range limits, frame detection.
 *  3. Georeferencing: mounts, attitude, lever arm, world ENU / NED frames.
 *  4. End to end without SITL: scripts/rosbridge-sim.js with a fixed pose,
 *     the vendored roslib over a real WebSocket, CBOR and JSON, rosapi topic
 *     listing, throttle_rate; the averaged surface compared with the scene.
 *
 *   node scripts/test-ros-surface.js
 */

const path = require('path');
const { spawn } = require('child_process');
const fs = require('fs');
const { SCENES } = require('./ros-sim-scenes');

const ROOT = path.join(__dirname, '..');
// The renderer's ES modules, in a package Node reads as CommonJS: imported
// from their source (none of them imports another file)
const load = (rel) => import('data:text/javascript,' + encodeURIComponent(fs.readFileSync(path.join(ROOT, rel), 'utf8')));

let failures = 0;
function check(name, cond, detail = '') {
    console.log(`${cond ? 'PASS' : 'FAIL'}  ${name}${detail ? '  ' + detail : ''}`);
    if (!cond) failures++;
}
function near(a, b, tol) { return Math.abs(a - b) <= tol; }
const DEG = Math.PI / 180;

// Every filled cell of a SurfaceTiles: centre x, y, height, weight
function* cellsOf(st) {
    const c = st.cell;
    for (const t of st.tiles.values()) {
        for (let k = 0; k < TILE * TILE; k++) {
            if (!(t.w[k] > 0)) continue;
            const i = k % TILE, j = (k - i) / TILE;
            yield { key: t.key, k, x: (t.tx * TILE + i + 0.5) * c, y: (t.ty * TILE + j + 0.5) * c, h: t.h[k], w: t.w[k] };
        }
    }
}

// Two surfaces fed the same positions, cell by cell: RMS and worst difference, m
function diff(a, b) {
    let n = 0, sq = 0, worst = 0, missing = 0;
    for (const p of cellsOf(a)) {
        const t = b.tiles.get(p.key);
        if (!t || !(t.w[p.k] > 0)) { missing++; continue; }
        const e = p.h - t.h[p.k];
        sq += e * e; n++;
        worst = Math.max(worst, Math.abs(e));
    }
    return { cells: n, rms: n ? Math.sqrt(sq / n) : NaN, worst, same: missing === 0 && n === b.filled };
}

// Mean height of the scene over each filled cell vs the surface: RMS and
// worst error, m. How well the mesh stands for the scene; a cell holding
// one point holds that point, not the cell's mean, so this is looser than diff().
function compare(st, scene) {
    let n = 0, sq = 0, worst = 0;
    const c = st.cell;
    for (const p of cellsOf(st)) {
        let t = 0;
        for (let a = 0; a < 5; a++) for (let b = 0; b < 5; b++) t += scene.height(p.x + ((a + 0.5) / 5 - 0.5) * c, p.y + ((b + 0.5) / 5 - 0.5) * c);
        const err = p.h - t / 25;
        sq += err * err; n++;
        worst = Math.max(worst, Math.abs(err));
    }
    return { cells: n, rms: n ? Math.sqrt(sq / n) : NaN, worst };
}

let TILE = 30;

async function main() {
    const T = await load('js/ros/SurfaceTiles.js');
    const { SurfaceTiles } = T;
    TILE = T.TILE;
    const P = await load('js/ros/RosPoints.js');

    // ---------- 1. SurfaceTiles ----------
    {
        const st = new SurfaceTiles({ cell: 0.3, memory: 20 });
        const N = 60000, pts = new Float64Array(N * 3);
        for (let k = 0; k < N; k++) {
            const x = (Math.random() - 0.5) * 20, y = (Math.random() - 0.5) * 20;
            pts[k * 3] = x; pts[k * 3 + 1] = y; pts[k * 3 + 2] = 5 + 0.1 * x - 0.05 * y;
        }
        const used = st.add(pts, N);
        let worst = 0, cells = 0;
        for (const p of cellsOf(st)) { cells++; worst = Math.max(worst, Math.abs(p.h - (5 + 0.1 * p.x - 0.05 * p.y))); }
        check('plane: 30 cm cells average onto their centres', worst < 0.03 && used === N, `worst ${worst.toFixed(4)} m over ${cells} cells`);
        check('plane: tiles of 30 × 30 cells only where the data are', TILE === 30 && st.size === 16 && st.filled === cells, `${st.size} tiles for 20 × 20 m`);
    }
    {
        const st = new SurfaceTiles({ cell: 0.3 });
        st.add(new Float64Array([-0.1, -0.1, 1, 8.99, 0.1, 2, 9.0, 0.1, 3]), 3);
        const t00 = st.tiles.get(T.tileKey(-1, -1)), t0 = st.tiles.get(T.tileKey(0, 0)), t1 = st.tiles.get(T.tileKey(1, 0));
        check('tile / cell indexing, negative side too', t00 && t00.w[29 * 30 + 29] > 0 && t0 && t0.w[29] > 0 && t1 && t1.w[0] > 0 && st.size === 3);
        const first = st.takeChanges(), second = st.takeChanges();
        check('changes are incremental', first.tiles.length === 3 && second.tiles.length === 0
            && Number.isNaN(first.tiles[0].h[0]) !== Number.isNaN(first.tiles[0].h[29 * 30 + 29]));
        st.add(new Float64Array([0.1, 0.1, 5]), 1);
        const third = st.takeChanges();
        check('… only the tile that changed is sent again', third.tiles.length === 1 && third.tiles[0].tx === 0 && near(third.tiles[0].h[0], 5, 1e-9));
        check('one point = 1/3 of a sample', near(st.tiles.get(T.tileKey(0, 0)).w[0], 1 / 3, 1e-6));
        check('a new cell size clears', st.configure({ cell: 0.5 }) && st.size === 0 && st.cell === 0.5);
    }
    {
        // A cell follows a change once its memory is full: moving average
        // 4 points a message in one cell: each message is 2/3 of a sample
        const st = new SurfaceTiles({ cell: 1, memory: 10 });
        const p = (u) => new Float64Array([0.2, 0.2, u, 0.3, 0.3, u, 0.25, 0.25, u, 0.2, 0.3, u]);
        for (let k = 0; k < 50; k++) st.add(p(0), 4);
        const tile = st.tiles.get(T.tileKey(0, 0));
        const settled = tile.h[0];
        for (let k = 0; k < 30; k++) st.add(p(10), 4);
        const a = (2 / 3) / (10 + 2 / 3), expect = 10 * (1 - (1 - a) ** 30);
        check('moving average once MEMORY is full', settled === 0 && near(tile.h[0], expect, 1e-6), `h ${tile.h[0].toFixed(3)} after 30 messages at 10 m, expected ${expect.toFixed(3)}`);
    }
    {
        // Budget: 40 tiles in a row, room for 10: the ones nearest the vehicle stay
        const st = new SurfaceTiles({ cell: 0.3, maxTiles: 10 });
        for (let k = 0; k < 40; k++) st.add(new Float64Array([k * 9 + 4.5, 4.5, 0]), 1, 0, 0);
        const kept = [...st.tiles.values()].map(t => t.tx).sort((x, y) => x - y);
        const { removed } = st.takeChanges();
        // (the newest tile stays: it arrived within the budget)
        check('tile budget drops the farthest from the vehicle', st.size <= 10 && kept.slice(0, 9).join() === '0,1,2,3,4,5,6,7,8' && removed.length === 40 - st.size,
            `kept ${kept.join(',')} · ${removed.length} removed`);
    }
    {
        // A 1.2 m wide, 0.8 m rock survives the averaging at 30 cm; at 2 m it would be a 0.2 m bump
        const rock = (x, y) => 0.8 * Math.exp(-(x * x + y * y) / (2 * 0.6 * 0.6));
        const fine = new SurfaceTiles({ cell: 0.3 }), coarse = new SurfaceTiles({ cell: 2 });
        const N = 40000, pts = new Float64Array(N * 3);
        for (let k = 0; k < N; k++) { const x = (Math.random() - 0.5) * 8, y = (Math.random() - 0.5) * 8; pts[k * 3] = x; pts[k * 3 + 1] = y; pts[k * 3 + 2] = rock(x, y); }
        fine.add(pts, N); coarse.add(pts, N);
        const top = (st) => Math.max(...[...cellsOf(st)].map(p => p.h));
        check('a rock 1.2 m across keeps its height at 30 cm', top(fine) > 0.7 && top(coarse) < 0.45, `peak ${top(fine).toFixed(2)} m at 30 cm, ${top(coarse).toFixed(2)} m at 2 m`);
    }

    // ---------- 2. RosPoints decoding ----------
    check('kindOf ROS 1 / ROS 2 names', P.kindOf('sensor_msgs/PointCloud2') === 'cloud' && P.kindOf('sensor_msgs/msg/LaserScan') === 'scan'
        && P.kindOf('sensor_msgs/msg/Range') === 'range' && P.kindOf('sensor_msgs/msg/Image') === null);
    check('frameOf', P.frameOf('map') === 'world-enu' && P.frameOf('odom_ned') === 'world-ned' && P.frameOf('/odom') === 'world-enu'
        && P.frameOf('livox_frame') === 'sensor' && P.frameOf('base_link') === 'sensor'
        && P.frameOf('camera_init', 'world') === 'world-enu' && P.frameOf('map', 'sensor') === 'sensor');

    function livoxCloud(points, asBase64) {
        const step = 26, buf = Buffer.alloc(points.length * step);
        points.forEach(([x, y, z], k) => {
            buf.writeFloatLE(x, k * step); buf.writeFloatLE(y, k * step + 4); buf.writeFloatLE(z, k * step + 8);
            buf.writeFloatLE(99, k * step + 12); buf[k * step + 16] = 7; buf.writeDoubleLE(1e18, k * step + 18);
        });
        return {
            header: { frame_id: 'livox_frame' }, height: 1, width: points.length, point_step: step, row_step: step * points.length,
            is_bigendian: false, fields: [{ name: 'x', offset: 0, datatype: 7 }, { name: 'y', offset: 4, datatype: 7 }, { name: 'z', offset: 8, datatype: 7 },
                { name: 'intensity', offset: 12, datatype: 7 }, { name: 'tag', offset: 16, datatype: 2 }, { name: 'timestamp', offset: 18, datatype: 8 }],
            data: asBase64 ? buf.toString('base64') : new Uint8Array(buf.buffer, buf.byteOffset, buf.length)
        };
    }
    {
        const out = new Float32Array(3000);
        const pts = [[10, 1, -2], [0, 0, 0], [1, 0, 0], [NaN, 1, 1], [300, 0, 0], [-5, 5, 5]];
        const a = P.decodePoints('cloud', livoxCloud(pts, false), out, 100, 2, 100);
        const got = Array.from(out.subarray(0, a.count * 3));
        check('PointCloud2 binary: fields, no-return / NaN / range dropped', a.total === 6 && a.count === 2
            && near(got[0], 10, 1e-5) && near(got[2], -2, 1e-5) && near(got[3], -5, 1e-5), JSON.stringify({ a, got }));
        const b = P.decodePoints('cloud', livoxCloud(pts, true), out, 100, 2, 100);
        check('PointCloud2 base64 (rosbridge JSON) same points', b.count === 2 && near(out[0], 10, 1e-5));
        const many = Array.from({ length: 1000 }, (_, k) => [10 + k * 0.01, 0, 0]);
        const c = P.decodePoints('cloud', livoxCloud(many, false), out, 100, 0, Infinity);
        let increasing = true;
        for (let k = 1; k < c.count; k++) if (!(out[k * 3] > out[(k - 1) * 3])) increasing = false;
        check('PointCloud2 sampling: at most max, strided across the cloud', c.count === 100 && increasing && out[(c.count - 1) * 3] > 19.8,
            `${c.count} pts, last x ${out[(c.count - 1) * 3].toFixed(2)}`);
    }
    {
        const buf = Buffer.alloc(2 * 32);
        [[1.5, -2.5, 3.5], [4, 5, 6]].forEach(([x, y, z], k) => { buf.writeDoubleBE(x, k * 32); buf.writeDoubleBE(y, k * 32 + 8); buf.writeDoubleBE(z, k * 32 + 16); });
        const msg = { header: {}, height: 2, width: 1, point_step: 32, row_step: 32, is_bigendian: true,
            fields: [{ name: 'x', offset: 0, datatype: 8 }, { name: 'y', offset: 8, datatype: 8 }, { name: 'z', offset: 16, datatype: 8 }], data: new Uint8Array(buf) };
        const out = new Float32Array(30);
        const r = P.decodePoints('cloud', msg, out, 10);
        check('PointCloud2 float64 big endian, organized (height 2)', r.count === 2 && near(out[1], -2.5, 1e-6) && near(out[5], 6, 1e-6));
    }
    {
        const out = new Float32Array(3000);
        const scan = { angle_min: -Math.PI / 2, angle_increment: Math.PI / 2, range_min: 0.1, range_max: 30,
            ranges: new Float32Array([10, Infinity, 20, 50, NaN]) };
        const a = P.decodePoints('scan', scan, out, 100);
        check('LaserScan typed array: angles, Inf / NaN / beyond range_max dropped', a.total === 5 && a.count === 2
            && near(out[0], 0, 1e-5) && near(out[1], -10, 1e-5) && near(out[3], 0, 1e-5) && near(out[4], 20, 1e-5), JSON.stringify(Array.from(out.subarray(0, 6))));
        const b = P.decodePoints('scan', { ...scan, ranges: [10, null, 20, 50, null] }, out, 100);
        check('LaserScan JSON array with nulls', b.count === 2);
        const tagged = Buffer.alloc(8); tagged.writeFloatLE(12, 0); tagged.writeFloatLE(13, 4);
        const c = P.decodePoints('scan', { ...scan, angle_min: 0, ranges: { tag: 85, contents: new Uint8Array(tagged) } }, out, 100);
        check('LaserScan undecoded RFC 8746 tag 85', c.count === 2 && near(out[0], 12, 1e-5));
    }
    {
        const out = new Float32Array(30);
        const ok = P.decodePoints('range', { range: 12.5, min_range: 0.5, max_range: 50 }, out, 10);
        const inf = P.decodePoints('range', { range: Infinity, min_range: 0.5, max_range: 50 }, out, 10);
        const close = P.decodePoints('range', { range: 0.2, min_range: 0.5, max_range: 50 }, out, 10);
        const nul = P.decodePoints('range', { range: null, min_range: 0.5, max_range: 50 }, out, 10);
        check('Range: along x, limits respected', ok.count === 1 && inf.count === 0 && close.count === 0 && nul.count === 0);
    }

    // ---------- 3. Georeferencing ----------
    {
        const anchor = P.makeAnchor(45.6, 10.67, 100);
        const veh = P.toEnu(anchor, 45.6, 10.67, 150);
        const Ra = P.eulerToMatrix(0, 0, 0);
        const out = new Float64Array(9);
        const down = P.eulerToMatrix(0, -90 * DEG, 0);
        P.sensorToEnu(new Float32Array([50, 0, 0]), 1, down, [0, 0, 0], Ra, veh, out);
        check('down-looking Range 50 m from 50 m up → ground', near(out[0], 0, 1e-6) && near(out[1], 0, 1e-6) && near(out[2], 0, 1e-6), Array.from(out.subarray(0, 3)).join(','));
        P.sensorToEnu(new Float32Array([0, 0, 10]), 1, P.eulerToMatrix(Math.PI, 0, 0), [0, 0, 0], Ra, veh, out);
        check('inverted Livox: sensor +z is down', near(out[2], 40, 1e-6) && near(out[0], 0, 1e-6));
        P.sensorToEnu(new Float32Array([10, 0, 0]), 1, P.eulerToMatrix(0, 0, 0), [0, 0, 0], P.eulerToMatrix(0, 0, 90 * DEG), veh, out);
        check('heading east: forward is +E', near(out[0], 10, 1e-6) && near(out[1], 0, 1e-6));
        P.sensorToEnu(new Float32Array([0, 10, 0]), 1, P.eulerToMatrix(0, 0, 0), [0, 0, 0], P.eulerToMatrix(0, 0, 0), veh, out);
        check('sensor +y is left (west, heading north)', near(out[0], -10, 1e-6));
        P.sensorToEnu(new Float32Array([5, 0, 0]), 1, down, [1, 0.5, 0.3], P.eulerToMatrix(0, 0, 0), veh, out);
        check('lever arm in FRD (fwd 1, right 0.5, down 0.3)', near(out[0], 0.5, 1e-6) && near(out[1], 1, 1e-6) && near(out[2], 50 - 0.3 - 5, 1e-6));
        P.sensorToEnu(new Float32Array([50, 0, 0]), 1, down, [0, 0, 0], P.eulerToMatrix(10 * DEG, 0, 0), veh, out);
        check('roll right 10°: the beam lands west', out[0] < -8 && near(out[2], 50 - 50 * Math.cos(10 * DEG), 1e-6), Array.from(out.subarray(0, 3)).map(v => v.toFixed(2)).join(','));
        // Projection: 0.5 s ahead at 90°/s of yaw, level → 45°; at 10 m/s north → 5 m
        const pr = P.projectPose({ t: 0, roll: 0, pitch: 0, yaw: 0, p: 0, q: 0, r: 90 * DEG },
            { t: 0, lat: 45, lon: 10, alt: 100, vn: 10, ve: 0, vd: -2 }, 500);
        check('pose projected with the gyro rates and the velocity', near(pr.yaw, 45 * DEG, 1e-9) && near((pr.lat - 45) * 111320, 5, 1e-6) && near(pr.alt, 101, 1e-9),
            `yaw ${(pr.yaw / DEG).toFixed(3)}°, north ${((pr.lat - 45) * 111320).toFixed(3)} m`);
        // Banked 30°, constant body rates for 1 s: against the exact rotation R0 · exp([ω]× t)
        const w = [5 * DEG, -3 * DEG, 20 * DEG];
        const pb = P.projectPose({ t: 0, roll: 30 * DEG, pitch: 10 * DEG, yaw: 0, p: w[0], q: w[1], r: w[2] }, { t: 0, lat: 0, lon: 0, alt: 0, vn: 0, ve: 0, vd: 0 }, 1000);
        const R0 = P.eulerToMatrix(30 * DEG, 10 * DEG, 0), th = Math.hypot(...w), k = w.map(v => v / th);
        const K = [0, -k[2], k[1], k[2], 0, -k[0], -k[1], k[0], 0];
        const K2 = [0, 1, 2, 3, 4, 5, 6, 7, 8].map(i => { const r = Math.floor(i / 3), c = i % 3; return K[r * 3] * K[c] + K[r * 3 + 1] * K[3 + c] + K[r * 3 + 2] * K[6 + c]; });
        const E = K.map((v, i) => (i % 4 === 0 ? 1 : 0) + Math.sin(th) * v + (1 - Math.cos(th)) * K2[i]);
        const R1 = [0, 1, 2, 3, 4, 5, 6, 7, 8].map(i => { const r = Math.floor(i / 3), c = i % 3; return R0[r * 3] * E[c] + R0[r * 3 + 1] * E[3 + c] + R0[r * 3 + 2] * E[6 + c]; });
        const ex = { roll: Math.atan2(R1[7], R1[8]), pitch: -Math.asin(R1[6]), yaw: Math.atan2(R1[3], R1[0]) };
        const errDeg = Math.max(...['roll', 'pitch', 'yaw'].map(n => Math.abs(pb[n] - ex[n]))) / DEG;
        check('… Euler kinematics when banked = the exact rotation', errDeg < 0.2,
            `projected ${['roll', 'pitch', 'yaw'].map(n => (pb[n] / DEG).toFixed(2)).join('/')}° vs exact ${['roll', 'pitch', 'yaw'].map(n => (ex[n] / DEG).toFixed(2)).join('/')}°`);
        P.worldToEnu(new Float32Array([1, 2, 3]), 1, false, [10, 20, 30], out);
        const enuOk = out[0] === 11 && out[1] === 22 && out[2] === 33;
        P.worldToEnu(new Float32Array([1, 2, 3]), 1, true, [10, 20, 30], out);
        check('world ENU and NED frames', enuOk && out[0] === 12 && out[1] === 21 && out[2] === 27);
        const a2 = P.makeAnchor(47.2603, 11.3439, 0);
        const back = P.toEnu(a2, 47.2603 + 100 / 111320, 11.3439 + 100 / (111320 * Math.cos(47.2603 * DEG)), 0);
        check('anchor metres = latLonToMeters() metres', near(back[0], 100, 1e-6) && near(back[1], 100, 1e-6));
    }

    // ---------- 4. End to end: emulator + roslib over WebSocket ----------
    const { Ros, Topic } = await load('vendor/roslib/roslib.esm.min.js');
    const sims = [];
    function startSim(port, extra) {
        return new Promise((resolve, reject) => {
            const p = spawn(process.execPath, [path.join(__dirname, 'rosbridge-sim.js'), '--port', String(port), ...extra], { stdio: ['ignore', 'pipe', 'pipe'] });
            sims.push(p);
            const timer = setTimeout(() => reject(new Error('emulator did not start')), 5000);
            p.stdout.on('data', (d) => { if (/listening/.test(String(d))) { clearTimeout(timer); resolve(p); } });
            p.stderr.on('data', (d) => process.stderr.write(d));
        });
    }
    async function connect(port) {
        const ros = new Ros();
        await new Promise((resolve, reject) => {
            ros.on('connection', resolve);
            ros.on('error', () => reject(new Error('no connection')));
            ros.connect(`ws://127.0.0.1:${port}`);
        });
        return ros;
    }
    function collect(ros, name, type, compression, ms, throttle = 0) {
        return new Promise((resolve) => {
            const msgs = [];
            const t = new Topic({ ros, name, messageType: type, compression, throttle_rate: throttle, queue_length: 1 });
            t.subscribe((m) => msgs.push(m));
            setTimeout(() => { t.unsubscribe(); resolve(msgs); }, ms);
        });
    }
    // Static vehicle: 50 m over the terrain anchor, heading 30°; a boat on the lake surface
    const AIR = { e: 0, n: 0, u: 50, yaw: 30 };
    const anchor = P.makeAnchor(0, 0, 0);
    // Decode, georeference and average like RosWorker; `truth` (optional) gets
    // the same points at the scene's true height, so diff(grid, truth) is the
    // error of the chain alone
    function georef(msgs, kind, mount, pose, grid, { world = false, max = 400, rMin = 0.5, rMax = 120, lever = [0, 0, 0], truth = null, scene = null } = {}) {
        const pts = new Float32Array(5000 * 3), enu = new Float64Array(5000 * 3), tru = new Float64Array(5000 * 3);
        const Rm = P.eulerToMatrix(mount[0] * DEG, mount[1] * DEG, mount[2] * DEG);
        const Ra = P.eulerToMatrix(0, 0, pose.yaw * DEG);
        let sampled = 0, total = 0;
        for (const m of msgs) {
            const r = P.decodePoints(kind, m, pts, max, world ? 0 : rMin, world ? Infinity : rMax);
            sampled += r.count; total += r.total;
            if (world) P.worldToEnu(pts, r.count, false, [0, 0, 0], enu);
            else P.sensorToEnu(pts, r.count, Rm, lever, Ra, [pose.e, pose.n, pose.u], enu);
            grid.add(enu, r.count, pose.e, pose.n);
            if (truth) {
                for (let k = 0; k < r.count; k++) {
                    tru[k * 3] = enu[k * 3]; tru[k * 3 + 1] = enu[k * 3 + 1];
                    tru[k * 3 + 2] = scene.height(enu[k * 3], enu[k * 3 + 1]);
                }
                truth.add(tru, r.count, pose.e, pose.n);
            }
        }
        return { sampled, total };
    }
    try {
        await startSim(19090, ['--pose', `${AIR.e},${AIR.n},${AIR.u},${AIR.yaw}`, '--scene', 'terrain', '--lidar-points', '8000']);
        await startSim(19091, ['--pose', '0,0,0,0', '--scene', 'seabed', '--ros1']);
        await startSim(19092, ['--pose', '0,0,-5,0', '--scene', 'cave']);
        const ros = await connect(19090);
        const topics = await new Promise((res, rej) => ros.getTopics(res, rej));
        const usable = topics.topics.filter((_, i) => P.kindOf(topics.types[i]));
        check('rosapi lists the topics; 6 of 10 are drawable types', topics.topics.length === 10 && usable.length === 6, usable.join(' '));

        const t0 = Date.now();
        const livox = await collect(ros, '/livox/lidar', 'sensor_msgs/msg/PointCloud2', 'cbor', 2000, 200);
        const rate = livox.length / ((Date.now() - t0) / 1000);
        check('throttle_rate 200 ms: ≤ 5 messages/s from a 10 Hz topic', livox.length >= 6 && rate <= 6, `${livox.length} msgs, ${rate.toFixed(1)}/s`);
        check('CBOR PointCloud2 data arrives as bytes', livox[0] && livox[0].data instanceof Uint8Array && livox[0].width === 8000);
        let g = new SurfaceTiles({ cell: 0.3 }), gt = new SurfaceTiles({ cell: 0.3 });
        let st = georef(livox, 'cloud', [180, 0, 0], AIR, g, { rMin: 2.5, max: 5000, truth: gt, scene: SCENES.terrain });
        let d = diff(g, gt), c = compare(g, SCENES.terrain);
        check('Livox (CBOR, sensor frame, inverted) → points on the terrain', d.same && d.rms < 0.01 && d.cells > 5000,
            `${st.sampled} of ${st.total} pts used · ${d.cells} cells of 30 cm in ${g.size} tiles · chain error RMS ${d.rms.toFixed(4)} m`);
        check('  … and the mesh stands for the surface', c.rms < 0.1, `vs the cell-area mean: RMS ${c.rms.toFixed(3)} m, worst ${c.worst.toFixed(2)} m`);

        // (a CBOR message still in flight from the subscription just closed can
        // land first: roslib dispatches by topic name; keep the JSON ones)
        const livoxJson = (await collect(ros, '/livox/lidar', 'sensor_msgs/msg/PointCloud2', 'none', 1200, 200)).filter(m => typeof m.data === 'string');
        check('JSON PointCloud2 data arrives as base64', livoxJson.length >= 3, `${livoxJson.length} JSON messages`);
        g = new SurfaceTiles({ cell: 0.3 });
        georef(livoxJson, 'cloud', [180, 0, 0], AIR, g, { rMin: 2.5, max: 5000 });
        c = compare(g, SCENES.terrain);
        check('Livox (JSON) → same surface', c.rms < 0.1 && c.cells > 3000, `${c.cells} cells · RMS ${c.rms.toFixed(3)} m`);

        const world = await collect(ros, '/cloud_registered', 'sensor_msgs/msg/PointCloud2', 'cbor', 1500, 200);
        check('world-frame cloud says frame map', world[0] && P.frameOf(world[0].header.frame_id) === 'world-enu');
        g = new SurfaceTiles({ cell: 0.3 }); gt = new SurfaceTiles({ cell: 0.3 });
        georef(world, 'cloud', [0, 0, 0], AIR, g, { world: true, max: 5000, truth: gt, scene: SCENES.terrain });
        d = diff(g, gt); c = compare(g, SCENES.terrain);
        check('world-frame cloud (map, ENU) → points on the terrain', d.same && d.rms < 0.01 && d.cells > 3000,
            `${d.cells} cells · chain error RMS ${d.rms.toFixed(4)} m · vs cell-area mean ${c.rms.toFixed(2)} m`);

        const scans = await collect(ros, '/scan', 'sensor_msgs/msg/LaserScan', 'cbor', 1000, 0);
        check('CBOR LaserScan ranges decode as a typed array (tag 85)', scans[0] && scans[0].ranges instanceof Float32Array && scans[0].ranges.length === 541,
            scans[0] && Object.prototype.toString.call(scans[0].ranges));
        g = new SurfaceTiles({ cell: 0.3 }); gt = new SurfaceTiles({ cell: 0.3 });
        st = georef(scans, 'scan', [0, -90, 0], AIR, g, { truth: gt, scene: SCENES.terrain });
        d = diff(g, gt);
        check('push-broom LaserScan (x down) → a swath on the terrain', d.same && d.rms < 0.01 && d.cells >= 10,
            `${st.sampled} pts · ${d.cells} cells · chain error RMS ${d.rms.toFixed(4)} m`);

        const ranges = await collect(ros, '/ping1d/range', 'sensor_msgs/msg/Range', 'none', 800, 0);
        // (continued below with the cave)
        check('Range 50 m under the aircraft', ranges.length >= 5 && near(ranges[0].range, 50, 0.1), ranges[0] && ranges[0].range.toFixed(2));
        const none = await collect(ros, '/camera/image_raw', 'sensor_msgs/msg/Image', 'cbor', 300, 0);
        check('other types are listed, never published', none.length === 0);
        ros.close();

        // Sea bed, ROS 1 names, boat at the surface: echo sounder + multibeam
        const ros1 = await connect(19091);
        const t1 = await new Promise((res, rej) => ros1.getTopics(res, rej));
        check('ROS 1 type names', t1.types.includes('sensor_msgs/PointCloud2') && P.kindOf('sensor_msgs/Range') === 'range');
        const BOAT = { e: 0, n: 0, u: 0, yaw: 0 };
        const sounder = await collect(ros1, '/ping1d/range', 'sensor_msgs/Range', 'cbor', 800, 0);
        g = new SurfaceTiles({ cell: 0.3 });
        georef(sounder, 'range', [0, -90, 0], BOAT, g);
        const depth = -[...cellsOf(g)][0].h;
        check('echo sounder: depth under the boat', near(depth, -SCENES.seabed.height(0, 0), 0.1), `${depth.toFixed(2)} m vs ${(-SCENES.seabed.height(0, 0)).toFixed(2)} m`);
        const beams = await collect(ros1, '/sonar/multibeam', 'sensor_msgs/PointCloud2', 'cbor', 1000, 0);
        g = new SurfaceTiles({ cell: 0.3 }); gt = new SurfaceTiles({ cell: 0.3 });
        st = georef(beams, 'cloud', [0, -90, 0], BOAT, g, { max: 1000, truth: gt, scene: SCENES.seabed });
        d = diff(g, gt);
        check('multibeam swath → sea bed', d.same && d.rms < 0.01 && d.cells >= 100,
            `${st.sampled} of ${st.total} pts · ${d.cells} cells of 30 cm · chain error RMS ${d.rms.toFixed(4)} m`);
        ros1.close();

        // Cave, ROV still half way down the first shaft, from the emulator over
        // rosbridge: the dual profiler's returns on the rock, the wall all around.
        // (Two fans from one place have no area to mesh: the 3D surface needs
        // the sonar to move — scripts/test-ros-cave.js flies the whole cave.)
        const ros2 = await connect(19092);
        const ROV = [0, 0, -5];
        const prof = await collect(ros2, '/sonar/profiler', 'sensor_msgs/msg/PointCloud2', 'cbor', 1000, 0);
        const pts = new Float32Array(800 * 3), enu = new Float64Array(800 * 3);
        const I = P.eulerToMatrix(0, 0, 0);
        let n = 0, on = 0;
        const around = new Set();
        for (const m of prof) {
            const r = P.decodePoints('cloud', m, pts, 800, 0.3, 40);
            P.sensorToEnu(pts, r.count, I, [0, 0, 0], I, ROV, enu);
            for (let k = 0; k < r.count; k++) {
                const e = enu[k * 3], no = enu[k * 3 + 1], u = enu[k * 3 + 2];
                n++;
                if (Math.abs(SCENES.cave.sdf(e, no, u)) < 0.05) on++;
                if (Math.abs(u - ROV[2]) < 0.5) around.add(Math.round(Math.atan2(no, e) / (Math.PI / 4)) & 7);
            }
        }
        check('cave shaft: profiler returns on the rock, the wall all around', prof.length >= 5 && n > 1000 && on / n > 0.99 && around.size === 8,
            `${n} returns, ${(100 * on / n).toFixed(1)} % within 5 cm of the rock, wall seen in ${around.size} of 8 directions`);
        ros2.close();
    } catch (e) {
        check('end-to-end run', false, e.message);
    } finally {
        for (const p of sims) p.kill();
    }

    console.log(failures ? `\n${failures} check(s) FAILED` : '\nall checks passed');
    process.exit(failures ? 1 : 0);
}

main();
