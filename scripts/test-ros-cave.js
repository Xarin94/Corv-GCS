#!/usr/bin/env node
/**
 * test-ros-cave.js - a ROV down the stepped cave, offline: mounting and mesh
 *
 * Replays a descent of the 'cave' scene (scripts/ros-sim-scenes.js: shafts
 * 3 m across, 10 m deep, joined by 3 m passages, to 30 m) without SITL or the
 * GCS window: the ROV follows the centre line at 0.5 m/s, turning slowly and
 * rocking a few degrees; a dual 360° profiler (one fan across the vehicle, one
 * level) is ray-cast at 10 Hz the way scripts/rosbridge-sim.js does it. The
 * returns then go through the GCS's own code: js/ros/RosPoints.js (mount →
 * attitude → position) and SurfaceVolume.js (30 cm TSDF voxels, surface nets).
 *
 *  1. Mounting: every georeferenced point must lie on the rock (signed
 *     distance ≈ 0) with the mount the sensor really has, and must not with a
 *     wrong one — the check that the data agree with the mounting.
 *  2. Mesh: every triangle, corners and centre, within 35 cm of the rock, and
 *     enough of them to cover the cave's walls along the route.
 *
 *   node scripts/test-ros-cave.js
 */

const fs = require('fs');
const path = require('path');
const { SCENES, castRay, CAVE_PATH } = require('./ros-sim-scenes');

const ROOT = path.join(__dirname, '..');
const load = (rel) => import('data:text/javascript,' + encodeURIComponent(fs.readFileSync(path.join(ROOT, rel), 'utf8')));
const DEG = Math.PI / 180;
const cave = SCENES.cave;

let failures = 0;
function check(name, cond, detail = '') {
    console.log(`${cond ? 'PASS' : 'FAIL'}  ${name}${detail ? '  ' + detail : ''}`);
    if (!cond) failures++;
}
function quantiles(a) {
    const s = Float64Array.from(a).sort();
    const q = (f) => s[Math.min(s.length - 1, Math.floor(f * s.length))];
    return { n: s.length, p50: q(0.5), p99: q(0.99), max: s[s.length - 1] };
}
const cm = (v) => `${(v * 100).toFixed(1)} cm`;

// The route: over the first shaft, then down the centre line
function trajectory(speed = 0.5, hz = 10) {
    const route = [[0, 0, -1], ...CAVE_PATH.slice(1)];
    const out = [];
    let t = 0;
    for (let k = 0; k < route.length - 1; k++) {
        const a = route[k], b = route[k + 1];
        const len = Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
        for (let d = 0; d < len; d += speed / hz) {
            const f = d / len;
            out.push({
                pos: [a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2])],
                roll: 3 * DEG * Math.sin(t * 0.7), pitch: 2 * DEG * Math.sin(t * 0.5), yaw: 15 * DEG * t
            });
            t += 1 / hz;
        }
    }
    return out;
}

// Dual profiler, 2 × 400 beams: across the vehicle (sensor y–z) and level (x–y)
const BEAMS = [];
for (let i = 0; i < 800; i++) {
    const a = 2 * Math.PI * (i % 400) / 400, c = Math.cos(a), s = Math.sin(a);
    BEAMS.push(i < 400 ? [0, c, s] : [c, s, 0]);
}

async function main() {
    const P = await load('js/ros/RosPoints.js');
    const V = await load('js/ros/SurfaceVolume.js');

    // ---------- the scans, as the emulator makes them (true mount 0, 0, 0) ----------
    const TRUE_MOUNT = P.eulerToMatrix(0, 0, 0);
    const scans = [];
    for (const pose of trajectory()) {
        const Ra = P.eulerToMatrix(pose.roll, pose.pitch, pose.yaw);
        const pts = [];
        for (const [x, y, z] of BEAMS) {
            // sensor FLU → FRD → mount → body → NED → ENU: the ray direction
            const fx = x, fy = -y, fz = -z, m = TRUE_MOUNT;
            const bx = m[0] * fx + m[1] * fy + m[2] * fz, by = m[3] * fx + m[4] * fy + m[5] * fz, bz = m[6] * fx + m[7] * fy + m[8] * fz;
            const n = Ra[0] * bx + Ra[1] * by + Ra[2] * bz, e = Ra[3] * bx + Ra[4] * by + Ra[5] * bz, d = Ra[6] * bx + Ra[7] * by + Ra[8] * bz;
            const r = castRay(cave, pose.pos[0], pose.pos[1], pose.pos[2], e, n, -d, 40);
            if (r) pts.push(x * r, y * r, z * r);
        }
        scans.push({ pose, Ra, pts: Float32Array.from(pts) });
    }
    const total = scans.reduce((s, x) => s + x.pts.length / 3, 0);
    console.log(`${scans.length} scans, ${total} returns along ${CAVE_PATH.length - 1} legs of the cave`);

    // ---------- 1. mounting: the points on the rock ----------
    function georef(mountDeg) {
        const Rm = P.eulerToMatrix(mountDeg[0] * DEG, mountDeg[1] * DEG, mountDeg[2] * DEG);
        const dist = [], all = [];
        for (const s of scans) {
            const n = s.pts.length / 3, enu = new Float64Array(n * 3);
            P.sensorToEnu(s.pts, n, Rm, [0, 0, 0], s.Ra, s.pose.pos, enu);
            for (let k = 0; k < n; k++) dist.push(Math.abs(cave.sdf(enu[k * 3], enu[k * 3 + 1], enu[k * 3 + 2])));
            all.push({ enu, n, pos: s.pose.pos });
        }
        return { q: quantiles(dist), off: dist.filter(d => d > 0.3).length / dist.length, all };
    }
    const right = georef([0, 0, 0]);
    check('points on the rock with the sensor\'s real mount (0, 0, 0)', right.q.p99 < 0.02,
        `|distance to rock| median ${cm(right.q.p50)}, p99 ${cm(right.q.p99)}, max ${cm(right.q.max)}`);
    // A shaft is round: a fan turned about the vertical still lands on its wall,
    // so a wrong yaw or roll shows only where the cave is not symmetric (the
    // passages, the corners): look at the tail, not the median
    for (const [label, m] of [['echo-sounder preset (0, −90, 0)', [0, -90, 0]], ['yawed 90°', [0, 0, 90]], ['upside down', [180, 0, 0]]]) {
        const g = georef(m);
        check(`a wrong mount shows: ${label}`, g.q.p99 > 1 && g.off > 0.05,
            `${(100 * g.off).toFixed(0)} % of points > 30 cm off the rock, median ${cm(g.q.p50)}, p99 ${cm(g.q.p99)}`);
    }

    // ---------- 2. the mesh: TSDF volume, surface nets ----------
    const vol = new V.SurfaceVolume({ cell: 0.3, memory: 20 });
    for (const { enu, n, pos } of right.all) vol.integrate(pos, enu, n);
    const { meshes } = vol.takeAll();
    let tris = 0, bad = 0, area = 0;
    const dv = [];
    for (const m of meshes) {
        const p = m.pos;
        for (let k = 0; k < p.length; k += 9) {
            const a = [p[k], p[k + 1], p[k + 2]], b = [p[k + 3], p[k + 4], p[k + 5]], c = [p[k + 6], p[k + 7], p[k + 8]];
            const g = [0, 1, 2].map(q => (a[q] + b[q] + c[q]) / 3);
            const d = Math.max(...[a, b, c, g].map(q => Math.abs(cave.sdf(q[0], q[1], q[2]))));
            dv.push(d);
            if (d > 0.35) bad++;
            const u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]], w = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
            area += 0.5 * Math.hypot(u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]);
            tris++;
        }
    }
    const q = quantiles(dv);
    // The cave's own surface along the route: three shafts (≈ 28 m of wall), two passages, the bottoms
    const r0 = cave.radius, wall = 2 * Math.PI * r0 * 28.5 + 2 * (2 * Math.PI * r0 * 3) + 4 * Math.PI * r0 * r0;
    check('cave mesh (TSDF, surface nets): triangles on the rock', tris > 1000 && bad / tris < 0.02 && q.p50 < 0.1,
        `${tris} triangles in ${meshes.length} chunks · corners and centre within 35 cm of the rock: ${(100 * (tris - bad) / tris).toFixed(1)} % · median ${cm(q.p50)}, p99 ${cm(q.p99)}, worst ${cm(q.max)}`);
    check('… and it covers the cave', area > 0.6 * wall, `${area.toFixed(0)} m² of mesh for ~${wall.toFixed(0)} m² of cave wall along the route`);

    console.log(failures ? `\n${failures} check(s) FAILED` : '\nall checks passed');
    process.exit(failures ? 1 : 0);
}

main().catch(e => { console.error(e); process.exit(1); });
