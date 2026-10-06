#!/usr/bin/env node
/**
 * test-ros-surface.js - offline checks for the ROS surface (js/ros/)
 *
 *  1. SurfaceTiles: 30 cm cells in 30 × 30 tiles, averaging, the moving
 *     average once a cell's memory is full, incremental changes, the tile
 *     budget, a small detail kept.
 *  2. RosPoints: PointCloud2 (livox_ros_driver2 layout, base64 and binary,
 *     float64 big endian), LaserScan (typed array, JSON nulls, the time of
 *     each beam from time_increment), Range, sampling, range limits, frame
 *     detection.
 *  3. Georeferencing: mounts, attitude, lever arm, world ENU / NED frames.
 *  4. End to end without SITL: scripts/rosbridge-sim.js with a fixed pose,
 *     the vendored roslib over a real WebSocket, CBOR and JSON, rosapi topic
 *     listing, throttle_rate; the averaged surface compared with the scene;
 *     the sector sonar over the Garda objects (a moving boat and the real
 *     worker: scripts/test-ros-sector.js).
 *
 *   node scripts/test-ros-surface.js
 */

const path = require('path');
const { spawn, spawnSync } = require('child_process');
const fs = require('fs');

// roslib needs the WebSocket global: Node 22 has it, Node 20 behind a flag
if (typeof WebSocket === 'undefined') {
    const r = spawnSync(process.execPath, ['--experimental-websocket', __filename, ...process.argv.slice(2)], { stdio: 'inherit' });
    process.exit(r.status ?? 1);
}

const { SCENES, GARDA } = require('./ros-sim-scenes');

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
        // A sector sonar's batch: one beam per ping, 66 ms apart, the head turning back (negative step)
        const times = new Float32Array(1000);
        const sweep = { angle_min: 30 * DEG, angle_increment: -1.8 * DEG, time_increment: 0.066, range_min: 0.75, range_max: 40,
            ranges: new Float32Array([20, Infinity, 19, 18.5, 0.2, 18]) };
        const t = P.decodePoints('scan', sweep, out, 100, 0.5, 100, times);
        check('LaserScan time_increment: each beam its time, the span of the message', t.count === 4 && near(t.span, 5 * 0.066, 1e-9)
            && near(times[0], 0, 1e-9) && near(times[1], 0.132, 1e-6) && near(times[3], 0.33, 1e-6)
            && near(Math.atan2(out[1 * 3 + 1], out[1 * 3]) / DEG, 30 - 2 * 1.8, 1e-4),
            `span ${t.span.toFixed(3)} s · times ${Array.from(times.subarray(0, t.count), x => x.toFixed(3)).join(' ')}`);
        const u = P.decodePoints('scan', { ...sweep, time_increment: 0 }, out, 100, 0.5, 100, times);
        const v = P.decodePoints('scan', { ...sweep, time_increment: NaN }, out, 100, 0.5, 100, times);
        check('… time_increment 0 or missing: one instant', u.span === 0 && v.span === 0 && u.count === 4);
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
        // A SITL at speedup 20: 25 ms here are 0.5 s of the autopilot's rates and velocity
        const pk = P.projectPose({ t: 0, roll: 0, pitch: 0, yaw: 0, p: 0, q: 0, r: 90 * DEG },
            { t: 0, lat: 45, lon: 10, alt: 100, vn: 10, ve: 0, vd: 0 }, 25, 20);
        check('projection on the autopilot clock (×20)', near(pk.yaw, 45 * DEG, 1e-9) && near((pk.lat - 45) * 111320, 5, 1e-6));
    }

    // ---------- Spike filter (despike) ----------
    {
        // A 256-beam fan over a 40 m flat bed: 12 % aeration (0.5–4.5 m), a few fish
        const n = 256, fan = new Float32Array(n * 3), bad = new Set();
        for (let i = 0; i < n; i++) {
            const a = (-45 + 90 * (i + 0.5) / n) * DEG;
            let d = 40 / Math.cos(a) + 0.05 * (Math.random() - 0.5);
            if (Math.random() < 0.12) { d = 0.5 + 4 * Math.random(); bad.add(i); } else if (i % 97 === 50) { d = 15; bad.add(i); }
            fan[i * 3] = Math.cos(a) * d; fan[i * 3 + 1] = Math.sin(a) * d;
        }
        const orig = Float32Array.from(fan);
        const r = P.despike(fan, n);
        const keptR = new Set(Array.from({ length: r.count }, (_, k) => Math.hypot(fan[k * 3], fan[k * 3 + 1]).toFixed(4)));
        let badKept = 0, goodLost = 0;
        for (let i = 0; i < n; i++) {
            const k = keptR.has(Math.hypot(orig[i * 3], orig[i * 3 + 1]).toFixed(4));
            if (bad.has(i) && k) badKept++;
            if (!bad.has(i) && !k) goodLost++;
        }
        check('spike filter: aeration and fish dropped from a sonar fan', r.ordered && badKept === 0 && goodLost <= 0.03 * (n - bad.size),
            `${r.removed} dropped · ${badKept}/${bad.size} spikes kept · ${goodLost} bed returns lost`);
        // The same fan shuffled has no order: nothing is dropped
        const sh = Float32Array.from(orig);
        for (let i = n - 1; i > 0; i--) { const j = Math.floor(Math.random() * (i + 1)); for (let c = 0; c < 3; c++) [sh[i * 3 + c], sh[j * 3 + c]] = [sh[j * 3 + c], sh[i * 3 + c]]; }
        const u = P.despike(sh, n);
        check('spike filter: a cloud without order is left alone', !u.ordered && u.removed === 0);
        // The beams' times follow their points through the compaction
        const tf = Float32Array.from(orig), tt = Float32Array.from({ length: n }, (_, i) => i * 0.066);
        const kept = P.despike(tf, n, {}, tt);
        let timesOk = kept.count > 0;
        for (let k = 0; k < kept.count; k++) {
            const i = Math.round(tt[k] / 0.066);
            if (Math.abs(tf[k * 3] - orig[i * 3]) > 1e-6 || Math.abs(tf[k * 3 + 1] - orig[i * 3 + 1]) > 1e-6) timesOk = false;
        }
        check('spike filter: each kept point keeps its time', timesOk, `${kept.count} kept`);
        // A cave profile: a near wall then a far one, the edge between them kept
        const cv = new Float32Array(80 * 3);
        for (let i = 0; i < 80; i++) { const a = i / 80 * Math.PI, d = i < 40 ? 3 : 15; cv[i * 3] = Math.cos(a) * d; cv[i * 3 + 1] = Math.sin(a) * d; }
        const e = P.despike(cv, 80);
        check('spike filter: an edge between two walls is kept', e.ordered && e.removed === 0);
    }

    // ---------- Roughness (RMS about the local plane) ----------
    {
        const R = await load('js/ros/SurfaceRaster.js');
        const w = 60, h = 60, view = new Float32Array(w * h * 2), out = new Uint8Array(w * h);
        // 1 m cells: a bed sloping 5 % with 2 cm of noise, a 1.5 m boulder at (40, 20), a hole
        for (let j = 0; j < h; j++) for (let i = 0; i < w; i++) {
            view[(j * w + i) * 2] = -40 + 0.05 * i + 0.02 * j + 0.02 * (Math.random() - 0.5) * 2
                + 1.5 * Math.exp(-((i - 40) ** 2 + (j - 20) ** 2) / 4.5);
        }
        for (let j = 45; j < 55; j++) for (let i = 5; i < 15; i++) view[(j * w + i) * 2] = R.EMPTY;
        R.roughness(view, w, h, out, { cell: 1 });
        const at = (i, j) => out[j * w + i];
        check('roughness: a sloping bed is flat, a boulder is not', at(10, 10) <= 3 && at(50, 50) <= 3 && at(40, 20) >= 25 && at(10, 50) === 0,
            `slope ${at(10, 10)} / ${at(50, 50)} cm · boulder ${at(40, 20)} cm · hole ${at(10, 50)}`);

        // A sector sonar's sweeps: lines of cells 5 m apart (every 10th row of
        // 0.5 m cells) on a sloping bed. By default the holes between them stay
        // (4 cells at most); filled up to 8 m they join into one surface, on the slope
        const n = 96, data = new Float32Array(n * n).fill(R.EMPTY), wts = new Float32Array(n * n), drawn = new Float32Array(n * n * 2);
        for (let j = 0; j < n; j += 10) for (let i = 0; i < n; i++) { data[j * n + i] = -12 + 0.02 * i + 0.03 * j; wts[j * n + i] = 1; }
        const share = (opts) => {
            R.rasterize(data, wts, n, n, drawn, { cell: 0.5, ...opts });
            let k = 0, worst = 0;
            for (let j = 0; j <= 90; j++) for (let i = 8; i < 88; i++) {
                const v = drawn[(j * n + i) * 2];
                if (v > R.EMPTY) { k++; worst = Math.max(worst, Math.abs(v - (-12 + 0.02 * i + 0.03 * j))); }
            }
            return { share: k / (91 * 80), worst };
        };
        const def = share({}), wide = share({ fillLevels: R.fillLevelsFor(8, 0.5) });
        check('fill levels: 8 m at 0.5 m cells = 16 cells; 0 = the default 4; capped at 16 cells', R.fillLevelsFor(8, 0.5) === 4 && R.fillLevelsFor(0, 0.3) === 2
            && R.fillLevelsFor(2, 0.5) === 2 && R.fillLevelsFor(8, 0.3) === 4 && R.fillLevelsFor(1, 0.5) === 1);
        // (a coarse cell's mean stands at its centre: on a slope, a few cm off where its data are not centred)
        check('fill: sweeps 5 m apart stay lines by default, join when holes are filled up to 8 m', def.share < 0.5 && wide.share > 0.95 && wide.worst < 0.15,
            `drawn ${(100 * def.share).toFixed(0)} % by default, ${(100 * wide.share).toFixed(0)} % filled up to 8 m (worst ${wide.worst.toFixed(3)} m off the slope)`);
        // … and the coverage of an area is counted in the same blocks
        const st = new SurfaceTiles({ cell: 0.5 });
        const pts = [];
        for (let y = 0.25; y < 64; y += 5) for (let x = 0.25; x < 64; x += 0.5) pts.push(x, y, -12);
        st.add(new Float64Array(pts), pts.length / 3);
        const sq = [[0, 0], [64, 0], [64, 64], [0, 64]];
        const fine = T.coveredArea(st, sq) / 4096, blocks = T.coveredArea(st, sq, 16) / 4096;
        check('coverage in the fill blocks: the area the sweeps are drawn over', fine < 0.15 && blocks > 0.95,
            `${(100 * fine).toFixed(0)} % of the cells, ${(100 * blocks).toFixed(0)} % in 8 m blocks`);
    }

    // ---------- The complex Garda bed: no relief stepped through ----------
    {
        // castRay marches by 0.4 of the clearance: right only while slopes stay
        // ≤ 1.5. Against a 2 cm march over the ridge, scarp, pinnacle, boulders,
        // sand waves and the objects, along sector-sonar rays
        const { castRay } = require('./ros-sim-scenes');
        const g = SCENES['garda-complex'];
        const march = (oe, on, de, dn, du) => { for (let t = 0; t <= 40; t += 0.02) if (t * du <= g.height(oe + t * de, on + t * dn)) return t; return 0; };
        let worst = 0, missed = 0, n = 0;
        for (let k = 0; k < 1000; k++) {
            const oe = 215 + ((k * 0.618034) % 1) * 235, on = 195 + ((k * 0.754877) % 1) * 285;
            const az = k * 2.399963, off = (((k * 0.569840) % 1) * 2 - 1) * 60 * DEG;
            const de = Math.sin(off) * Math.cos(az), dn = Math.sin(off) * Math.sin(az), du = -Math.cos(off);
            const a = castRay(g, oe, on, 0, de, dn, du, 40), b = march(oe, on, de, dn, du);
            n++;
            if (!a !== !b) missed++;
            else worst = Math.max(worst, Math.abs(a - b));
        }
        check('garda-complex: rays never step through its relief (vs a 2 cm march)', missed === 0 && worst <= 0.021,
            `${n} rays · ${missed} hit / miss disagreements · worst ${(worst * 100).toFixed(1)} cm`);
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
    process.on('exit', () => { for (const p of sims) p.kill(); });
    try {
        await startSim(19090, ['--pose', `${AIR.e},${AIR.n},${AIR.u},${AIR.yaw}`, '--scene', 'terrain', '--lidar-points', '8000']);
        await startSim(19091, ['--pose', '0,0,0,0', '--scene', 'seabed', '--ros1']);
        await startSim(19092, ['--pose', '0,0,-5,0', '--scene', 'cave']);
        const ros = await connect(19090);
        const topics = await new Promise((res, rej) => ros.getTopics(res, rej));
        const usable = topics.topics.filter((_, i) => P.kindOf(topics.types[i]));
        check('rosapi lists the topics; 8 of 12 are drawable types', topics.topics.length === 12 && usable.length === 8, usable.join(' '));

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

        // Sector sonar, boat still over the container on the Garda bed, a
        // pencil beam with no disturbances: one beam per ping, the sweep in
        // parts, back and forth — every return exactly on the scene
        const SECT = { e: 335, n: 455, u: 0, yaw: 15 };
        const garda = ['--scene', 'garda', '--anchor', `${GARDA.origin.lat},${GARDA.origin.lon},0`, '--pose', `${SECT.e},${SECT.n},0,${SECT.yaw}`, '--sonar-clean'];
        await startSim(19093, [...garda, '--sector-beam', '0.001', '--sector-fan', '0.001']);
        await startSim(19094, garda);
        const rp = await connect(19093);
        const pencil = await collect(rp, '/sonar/sector', 'sensor_msgs/msg/LaserScan', 'cbor', 9500, 100);
        rp.close();
        const up = pencil.filter(m => m.angle_increment > 0).length, down = pencil.filter(m => m.angle_increment < 0).length;
        const pings = pencil.reduce((a, m) => a + m.ranges.length, 0);
        check('sector sonar: LaserScans of a quarter of a sweep, both ways, time_increment = ping period', up >= 2 && down >= 2
            && pencil.every(m => near(m.time_increment, 2 * 40 / 1480 + 0.012, 1e-6) && m.ranges.length >= 10 && m.ranges.length <= 17 && m.header.frame_id === 'sector_sonar'),
            `${pencil.length} messages (${up} sweeping one way, ${down} back), ${pings} pings in 9.5 s`);
        g = new SurfaceTiles({ cell: 0.3 }); gt = new SurfaceTiles({ cell: 0.3 });
        st = georef(pencil, 'scan', [0, -90, 0], SECT, g, { truth: gt, scene: SCENES.garda, rMin: 0.75, rMax: 40 });
        d = diff(g, gt);
        const cont = GARDA.objects.find(o => o.name.startsWith('Container'));
        const top = Math.max(...[...cellsOf(g)].map(p => p.h));
        check('… pencil beam: every return on the scene, the container on top of the bed', d.same && d.rms < 0.01 && near(top, -cont.depth + cont.proud, 0.1),
            `${st.sampled} returns · ${d.cells} cells · chain error RMS ${d.rms.toFixed(4)} m · highest ${top.toFixed(2)} m, container top ${(-cont.depth + cont.proud).toFixed(2)} m`);
        // A 2° beam: the bottom is where a quarter of the echo is back, put on
        // the beam's axis — the bed within centimetres where it is level, a
        // little shallow at the swath's edges, the container's top found, its
        // walls smeared by the footprint
        const rb = await connect(19094);
        const wide = await collect(rb, '/sonar/sector', 'sensor_msgs/msg/LaserScan', 'cbor', 9500, 100);
        rb.close();
        const spts = new Float32Array(2000 * 3), senu = new Float64Array(2000 * 3);
        const Rm = P.eulerToMatrix(0, -90 * DEG, 0), Ra = P.eulerToMatrix(0, 0, SECT.yaw * DEG);
        const errs = [], inner = [];
        let highest = -Infinity;
        for (const m of wide) {
            const r = P.decodePoints('scan', m, spts, 2000, 0.75, 40);
            P.sensorToEnu(spts, r.count, Rm, [0, 0, 0], Ra, [SECT.e, SECT.n, 0], senu);
            for (let k = 0; k < r.count; k++) {
                const dz = senu[k * 3 + 2] - SCENES.garda.height(senu[k * 3], senu[k * 3 + 1]);
                errs.push(Math.abs(dz));
                // within 30° of nadir: away from the oblique edges of the swath
                if (Math.hypot(spts[k * 3 + 1], spts[k * 3 + 2]) < 0.5 * Math.hypot(spts[k * 3], spts[k * 3 + 1], spts[k * 3 + 2])) inner.push(Math.abs(dz));
                highest = Math.max(highest, senu[k * 3 + 2]);
            }
        }
        const q = (a, f) => a.slice().sort((x, y) => x - y)[Math.floor(f * (a.length - 1))];
        check('… 2° beam: the bed within cm near nadir, the container\'s top found', errs.length > 100 && q(inner, 0.5) < 0.05 && q(errs, 0.8) < 0.25
            && near(highest, -cont.depth + cont.proud, 0.15),
            `${errs.length} returns · |error| median ${q(errs, 0.5).toFixed(3)} m (within 30° of nadir ${q(inner, 0.5).toFixed(3)} m), 80 % ${q(errs, 0.8).toFixed(2)} m, worst ${q(errs, 1).toFixed(2)} m · highest ${highest.toFixed(2)} m`);
    } catch (e) {
        check('end-to-end run', false, e.message);
    } finally {
        for (const p of sims) p.kill();
    }

    console.log(failures ? `\n${failures} check(s) FAILED` : '\nall checks passed');
    process.exit(failures ? 1 : 0);
}

main();
