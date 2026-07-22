#!/usr/bin/env node
/**
 * Procedural generator for the simplified vehicle models in models/.
 *
 * Every airframe supported by Mission Planner gets its own low-poly GLB so the
 * silhouette alone tells you what you are flying. Models are built in real
 * metres with the aircraft convention used by the renderer:
 *
 *     +X = nose,  +Y = up,  +Z = right wing
 *
 * (js/main.js yaws the loaded model by +90 deg to bring +X onto the world
 * heading axis, exactly as the old hand-made plane.glb expected.)
 *
 * Run:  node scripts/gen-models.js
 */

const fs = require('fs');
const path = require('path');

const OUT_DIR = path.join(__dirname, '..', 'models');

// ---------------------------------------------------------------- palette
// One shared palette keeps the fleet reading as a family; the accent is what
// you actually pick out at distance (nose, wingtips, rotor discs).
const C = {
    body:   [0.84, 0.86, 0.88, 1],  // light grey airframe
    dark:   [0.13, 0.15, 0.17, 1],  // control surfaces, booms, arms
    accent: [0.91, 0.35, 0.09, 1],  // orange — nose, tips, markers
    glass:  [0.20, 0.31, 0.38, 1],  // canopy / sensor pods
    metal:  [0.45, 0.48, 0.52, 1],  // motors, hardware
};

// ---------------------------------------------------------------- vec utils
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const cross = (a, b) => [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const norm = (v) => {
    const l = Math.hypot(v[0], v[1], v[2]) || 1;
    return [v[0] / l, v[1] / l, v[2] / l];
};

/** Build a point transform: scale, then rot X→Y→Z, then translate. */
function T({ pos = [0, 0, 0], rot = [0, 0, 0], scale = [1, 1, 1] } = {}) {
    const [rx, ry, rz] = rot;
    return (p) => {
        let x = p[0] * scale[0], y = p[1] * scale[1], z = p[2] * scale[2];
        let c = Math.cos(rx), s = Math.sin(rx);
        [y, z] = [y * c - z * s, y * s + z * c];
        c = Math.cos(ry); s = Math.sin(ry);
        [z, x] = [z * c - x * s, z * s + x * c];
        c = Math.cos(rz); s = Math.sin(rz);
        [x, y] = [x * c - y * s, x * s + y * c];
        return [x + pos[0], y + pos[1], z + pos[2]];
    };
}
const ID = T();
const rad = (d) => (d * Math.PI) / 180;

// ---------------------------------------------------------------- part/solid
class Part {
    constructor(color) {
        this.color = color;
        this.tris = [];   // { p: [a,b,c], n: [x,y,z] }
    }
}

/**
 * Append one convex solid to a part.
 *
 * Triangle winding is not trusted: normals are resolved by pushing each face
 * away from the solid's own centroid, which is exact for the convex primitives
 * used here and saves fighting vertex order in every generator below.
 */
function addSolid(part, faces) {
    let cx = 0, cy = 0, cz = 0, n = 0;
    for (const f of faces) for (const v of f) { cx += v[0]; cy += v[1]; cz += v[2]; n++; }
    const c = [cx / n, cy / n, cz / n];

    for (const f of faces) {
        for (let i = 1; i < f.length - 1; i++) {
            const a = f[0], b = f[i], d = f[i + 1];
            let nrm = norm(cross(sub(b, a), sub(d, a)));
            const mid = [(a[0] + b[0] + d[0]) / 3, (a[1] + b[1] + d[1]) / 3, (a[2] + b[2] + d[2]) / 3];
            if (dot(nrm, sub(mid, c)) < 0) {
                nrm = [-nrm[0], -nrm[1], -nrm[2]];
                part.tris.push({ p: [a, d, b], n: nrm });
            } else {
                part.tris.push({ p: [a, b, d], n: nrm });
            }
        }
    }
}

/** Box from 8 corners given as two quads (bottom, top) in matching order. */
function hexa(part, bottom, top, xf = ID) {
    const b = bottom.map(xf), t = top.map(xf);
    addSolid(part, [
        b, t,
        [b[0], b[1], t[1], t[0]],
        [b[1], b[2], t[2], t[1]],
        [b[2], b[3], t[3], t[2]],
        [b[3], b[0], t[0], t[3]],
    ]);
}

function box(part, [cx, cy, cz], [sx, sy, sz], xf = ID) {
    const hx = sx / 2, hy = sy / 2, hz = sz / 2;
    hexa(part,
        [[cx - hx, cy - hy, cz - hz], [cx + hx, cy - hy, cz - hz], [cx + hx, cy - hy, cz + hz], [cx - hx, cy - hy, cz + hz]],
        [[cx - hx, cy + hy, cz - hz], [cx + hx, cy + hy, cz - hz], [cx + hx, cy + hy, cz + hz], [cx - hx, cy + hy, cz + hz]],
        xf);
}

/**
 * Cylinder / cone / truncated cone along +X (nose direction), so fuselages and
 * booms need no extra rotation. r0 is the -X end, r1 the +X end.
 */
function tube(part, { x0, x1, r0, r1, seg = 14, y = 0, z = 0, ry = 1, rz = 1 }, xf = ID) {
    const ring = (x, r) => {
        const out = [];
        for (let i = 0; i < seg; i++) {
            const a = (i / seg) * Math.PI * 2;
            out.push(xf([x, y + Math.cos(a) * r * ry, z + Math.sin(a) * r * rz]));
        }
        return out;
    };
    const A = ring(x0, r0), B = ring(x1, r1);
    const faces = [];
    for (let i = 0; i < seg; i++) {
        const j = (i + 1) % seg;
        if (r0 > 1e-6 && r1 > 1e-6) faces.push([A[i], A[j], B[j], B[i]]);
        else if (r0 > 1e-6) faces.push([A[i], A[j], B[0]]);
        else faces.push([A[0], B[j], B[i]]);
    }
    if (r0 > 1e-6) faces.push(A.slice().reverse());
    if (r1 > 1e-6) faces.push(B);
    addSolid(part, faces);
}

/** Disc/short cylinder along +Y — rotor discs, wheels get their own axis. */
function disc(part, { r, h = 0.04, seg = 16 }, xf = ID) {
    const ring = (yy) => {
        const out = [];
        for (let i = 0; i < seg; i++) {
            const a = (i / seg) * Math.PI * 2;
            out.push(xf([Math.cos(a) * r, yy, Math.sin(a) * r]));
        }
        return out;
    };
    const A = ring(-h / 2), B = ring(h / 2);
    const faces = [A.slice().reverse(), B];
    for (let i = 0; i < seg; i++) {
        const j = (i + 1) % seg;
        faces.push([A[i], A[j], B[j], B[i]]);
    }
    addSolid(part, faces);
}

/** Ellipsoid — canopies, blimp envelopes, sensor pods. */
function ellipsoid(part, { rx, ry, rz, seg = 14, rings = 7 }, xf = ID) {
    const grid = [];
    for (let j = 0; j <= rings; j++) {
        const phi = (j / rings) * Math.PI;
        const row = [];
        for (let i = 0; i < seg; i++) {
            const th = (i / seg) * Math.PI * 2;
            row.push(xf([
                rx * Math.sin(phi) * Math.cos(th),
                ry * Math.cos(phi),
                rz * Math.sin(phi) * Math.sin(th),
            ]));
        }
        grid.push(row);
    }
    const faces = [];
    for (let j = 0; j < rings; j++) {
        for (let i = 0; i < seg; i++) {
            const k = (i + 1) % seg;
            faces.push([grid[j][i], grid[j][k], grid[j + 1][k], grid[j + 1][i]]);
        }
    }
    addSolid(part, faces);
}

/**
 * Wing panel: a tapered, swept, thin trapezoid running outboard along +Z
 * (mirror with scale [1,1,-1]). Sweep is the leading-edge offset at the tip,
 * dihedral the tip rise.
 */
function panel(part, {
    x = 0, y = 0, z0 = 0, span, rootChord, tipChord, thick = 0.1,
    sweep = 0, dihedral = 0,
}, xf = ID) {
    const z1 = z0 + span;
    const xr0 = x + rootChord / 2, xr1 = x - rootChord / 2;               // root LE/TE
    const xt0 = x - sweep + tipChord / 2, xt1 = x - sweep - tipChord / 2; // tip LE/TE
    const yt = y + dihedral;
    hexa(part,
        [[xr1, y - thick / 2, z0], [xr0, y - thick / 2, z0], [xt0, yt - thick / 2 * (tipChord / rootChord), z1], [xt1, yt - thick / 2 * (tipChord / rootChord), z1]],
        [[xr1, y + thick / 2, z0], [xr0, y + thick / 2, z0], [xt0, yt + thick / 2 * (tipChord / rootChord), z1], [xt1, yt + thick / 2 * (tipChord / rootChord), z1]],
        xf);
}

/**
 * A panel's span runs along +Z; rotating it about X by -90 deg stands it up as
 * a fin (+90 would hang it below the fuselage — used for rudders in water).
 */
const UP = T({ rot: [rad(-90), 0, 0] });
const DOWN = T({ rot: [rad(90), 0, 0] });

/** Square-section beam between two arbitrary points — legs, struts, masts. */
function strut(part, from, to, w = 0.08) {
    const dir = norm(sub(to, from));
    const up = Math.abs(dir[1]) > 0.9 ? [1, 0, 0] : [0, 1, 0];
    const u = norm(cross(dir, up)).map((v) => (v * w) / 2);
    const v = norm(cross(dir, u)).map((v2) => (v2 * w) / 2);
    const ring = (o) => [
        [o[0] - u[0] - v[0], o[1] - u[1] - v[1], o[2] - u[2] - v[2]],
        [o[0] + u[0] - v[0], o[1] + u[1] - v[1], o[2] + u[2] - v[2]],
        [o[0] + u[0] + v[0], o[1] + u[1] + v[1], o[2] + u[2] + v[2]],
        [o[0] - u[0] + v[0], o[1] - u[1] + v[1], o[2] - u[2] + v[2]],
    ];
    hexa(part, ring(from), ring(to));
}

/** Mirrored pair of wing panels. */
function wings(part, opts) {
    panel(part, opts);
    panel(part, opts, T({ scale: [1, 1, -1] }));
}

// ---------------------------------------------------------------- shared bits

/**
 * Spinner + two blades, pointing forward from x.
 *
 * Fixed-wing airframes pass blades:false — a stationary prop cross reads as
 * clutter in the 3D view (and as a wrong attitude cue when seen edge-on), so
 * they keep the spinner alone as a nose cap.
 */
function propeller(parts, { x, y = 0, z = 0, r = 0.55, spinner = true, blades = true }) {
    if (spinner) tube(parts.accent, { x0: x - 0.18, x1: x + 0.02, r0: 0.11, r1: 0.02, y, z, seg: 10 });
    if (!blades) return;
    for (const s of [1, -1]) {
        box(parts.dark, [0, 0, s * (r / 2 + 0.06)], [0.09, 0.02, r], T({ pos: [x - 0.2, y, z] }));
    }
}

/** Motor can + spinning-disc proxy + blade cross, for multirotors. */
function rotor(parts, { x, z, r = 0.62, y = 0.12, ccw = true }) {
    tube(parts.metal, { x0: -0.09, x1: 0.09, r0: 0.11, r1: 0.11, seg: 10 },
        T({ rot: [0, 0, rad(90)], pos: [x, y, z] }));
    // Thin translucent-looking disc reads as a turning prop at any distance.
    disc(parts.dark, { r, h: 0.02, seg: 18 }, T({ pos: [x, y + 0.16, z] }));
    disc(parts.accent, { r: r * 0.28, h: 0.03, seg: 10 }, T({ pos: [x, y + 0.18, z] }));
    void ccw;
}

/** Arm from the hub out to (x,z), with the rotor on top. */
function arm(parts, { x, z, y = 0, w = 0.11 }) {
    const len = Math.hypot(x, z);
    const ang = Math.atan2(z, x);
    box(parts.dark, [len / 2, 0, 0], [len, 0.09, w], T({ rot: [0, -ang, 0], pos: [0, y, 0] }));
}

// ---------------------------------------------------------------- airframes

const MODELS = {};

// --- Fixed wing (MAV_TYPE_FIXED_WING) — the classic, unmistakably an aeroplane
MODELS['plane'] = (P) => {
    // Fuselage: nose cone, constant mid section, tapered tail boom.
    tube(P.accent, { x0: 2.55, x1: 3.30, r0: 0.34, r1: 0.02, seg: 16 });
    tube(P.body,   { x0: 0.20, x1: 2.55, r0: 0.38, r1: 0.34, seg: 16 });
    tube(P.body,   { x0: -2.60, x1: 0.20, r0: 0.20, r1: 0.38, seg: 16 });
    tube(P.body,   { x0: -3.40, x1: -2.60, r0: 0.13, r1: 0.20, seg: 16 });

    ellipsoid(P.glass, { rx: 0.85, ry: 0.26, rz: 0.30, seg: 12, rings: 6 }, T({ pos: [1.35, 0.30, 0] }));

    // Main wing: 10 m span, mild sweep and dihedral.
    wings(P.body, { x: 0.55, y: -0.10, z0: 0.32, span: 4.68, rootChord: 1.55, tipChord: 0.85, thick: 0.20, sweep: 0.45, dihedral: 0.42 });
    // Wingtips in accent — attitude is readable from any angle.
    for (const s of [1, -1]) {
        box(P.accent, [0.10, 0.42, s * 4.85], [0.85, 0.12, 0.34], ID);
    }
    // Tailplane + fin, both as single clean panels — no split control surfaces.
    wings(P.body, { x: -3.05, y: 0.05, z0: 0.12, span: 1.60, rootChord: 0.85, tipChord: 0.50, thick: 0.12, sweep: 0.28 });
    panel(P.body, { x: -3.10, y: 0.14, z0: 0.06, span: 1.10, rootChord: 0.95, tipChord: 0.48, thick: 0.12, sweep: 0.62 }, UP);

    propeller(P, { x: 3.34, r: 1.05, blades: false });
};

// --- Flying wing / delta (MAV_TYPE_FLAPPING_WING slot in MP's list is rare;
//     this covers the pusher delta wings people actually fly)
MODELS['flying-wing'] = (P) => {
    // Strongly swept centre body blending into the outer panels.
    wings(P.body, { x: 0.10, y: 0, z0: 0.0, span: 1.30, rootChord: 2.60, tipChord: 1.70, thick: 0.34, sweep: 0.80 });
    wings(P.body, { x: -0.70, y: 0.02, z0: 1.30, span: 2.40, rootChord: 1.70, tipChord: 0.55, thick: 0.20, sweep: 1.05, dihedral: 0.18 });
    // Winglets.
    for (const s of [1, -1]) {
        panel(P.accent, { x: -1.72, y: 0.20, z0: 0.02, span: 0.62, rootChord: 0.70, tipChord: 0.34, thick: 0.09, sweep: 0.30 },
            T({ rot: [rad(-90), 0, 0], pos: [0, 0, s * 3.68] }));
    }
    ellipsoid(P.body, { rx: 1.20, ry: 0.26, rz: 0.42, seg: 12, rings: 6 }, T({ pos: [0.35, 0.22, 0] }));
    ellipsoid(P.glass, { rx: 0.45, ry: 0.17, rz: 0.26, seg: 10, rings: 5 }, T({ pos: [0.95, 0.34, 0] }));
    tube(P.accent, { x0: 1.35, x1: 1.72, r0: 0.22, r1: 0.03, seg: 12, y: 0.10 });
    // Pusher prop omitted: at this size the blade cross only added visual noise.
};

// --- Quadplane / VTOL (MAV_TYPE_VTOL_*)
MODELS['quadplane-vtol'] = (P) => {
    tube(P.accent, { x0: 2.00, x1: 2.65, r0: 0.30, r1: 0.02, seg: 14 });
    tube(P.body,   { x0: -2.40, x1: 2.00, r0: 0.22, r1: 0.30, seg: 14 });
    ellipsoid(P.glass, { rx: 0.70, ry: 0.22, rz: 0.26, seg: 12, rings: 6 }, T({ pos: [1.05, 0.26, 0] }));
    wings(P.body, { x: 0.30, y: -0.06, z0: 0.28, span: 3.90, rootChord: 1.30, tipChord: 0.70, thick: 0.18, sweep: 0.30, dihedral: 0.28 });
    wings(P.body, { x: -2.20, y: 0.02, z0: 0.10, span: 1.30, rootChord: 0.70, tipChord: 0.42, thick: 0.10, sweep: 0.22 });
    panel(P.body, { x: -2.25, y: 0.12, z0: 0.05, span: 0.90, rootChord: 0.80, tipChord: 0.40, thick: 0.10, sweep: 0.50 }, UP);
    propeller(P, { x: 2.68, r: 0.85, blades: false });

    // Lift booms carrying four vertical rotors — the VTOL giveaway.
    for (const s of [1, -1]) {
        box(P.dark, [0.10, 0.08, s * 1.70], [3.40, 0.13, 0.16]);
        rotor(P, { x: 1.65, z: s * 1.70, r: 0.60, y: 0.20 });
        rotor(P, { x: -1.45, z: s * 1.70, r: 0.60, y: 0.20 });
    }
};

// --- Multirotors --------------------------------------------------------
/** Shared multirotor builder: body pod, legs, N arms at the given angles. */
function multirotor(P, { angles, radius = 1.30, rotorR = 0.62, body = 0.90 }) {
    box(P.body, [0, 0, 0], [body, 0.26, body * 0.78]);
    // Forward arrow so heading is obvious from above.
    tube(P.accent, { x0: body / 2 - 0.05, x1: body / 2 + 0.42, r0: 0.19, r1: 0.03, seg: 10, y: 0.02 });
    ellipsoid(P.glass, { rx: 0.20, ry: 0.16, rz: 0.20, seg: 10, rings: 5 }, T({ pos: [body / 2 - 0.02, -0.14, 0] }));
    box(P.dark, [0, -0.18, 0], [body * 0.9, 0.12, body * 0.7]);

    for (const a of angles) {
        const x = Math.cos(rad(a)) * radius, z = Math.sin(rad(a)) * radius;
        arm(P, { x, z, y: 0.02 });
        rotor(P, { x, z, r: rotorR, y: 0.10 });
    }
}

MODELS['quadcopter'] = (P) => multirotor(P, { angles: [45, 135, 225, 315], radius: 1.20, rotorR: 0.62 });
MODELS['hexacopter'] = (P) => multirotor(P, { angles: [30, 90, 150, 210, 270, 330], radius: 1.35, rotorR: 0.58 });
MODELS['octocopter'] = (P) => multirotor(P, { angles: [22.5, 67.5, 112.5, 157.5, 202.5, 247.5, 292.5, 337.5], radius: 1.50, rotorR: 0.55 });

// --- Tricopter: two front arms + a single yawing tail rotor
MODELS['tricopter'] = (P) => {
    box(P.body, [0.05, 0, 0], [0.85, 0.24, 0.70]);
    tube(P.accent, { x0: 0.42, x1: 0.86, r0: 0.18, r1: 0.03, seg: 10 });
    for (const s of [1, -1]) {
        arm(P, { x: 0.85, z: s * 0.98, y: 0.02 });
        rotor(P, { x: 0.85, z: s * 0.98, r: 0.60, y: 0.10 });
    }
    box(P.dark, [-0.90, 0.02, 0], [1.60, 0.10, 0.13]);
    // Tilting tail motor, canted to show the yaw servo.
    tube(P.metal, { x0: -0.09, x1: 0.09, r0: 0.11, r1: 0.11, seg: 10 },
        T({ rot: [rad(18), 0, rad(90)], pos: [-1.65, 0.14, 0] }));
    disc(P.dark, { r: 0.58, h: 0.02, seg: 18 }, T({ rot: [rad(18), 0, 0], pos: [-1.65, 0.30, 0] }));
    disc(P.accent, { r: 0.16, h: 0.03, seg: 10 }, T({ rot: [rad(18), 0, 0], pos: [-1.65, 0.33, 0] }));
};

// --- Helicopter (MAV_TYPE_HELICOPTER)
MODELS['helicopter'] = (P) => {
    ellipsoid(P.body, { rx: 1.35, ry: 0.62, rz: 0.62, seg: 14, rings: 7 }, T({ pos: [0.55, 0.10, 0] }));
    ellipsoid(P.glass, { rx: 0.70, ry: 0.44, rz: 0.50, seg: 12, rings: 6 }, T({ pos: [1.35, 0.14, 0] }));
    tube(P.body, { x0: -3.30, x1: -0.30, r0: 0.13, r1: 0.36, seg: 12, y: 0.24 });

    // Mast, head and two-blade main rotor.
    tube(P.metal, { x0: -0.10, x1: 0.10, r0: 0.09, r1: 0.09, seg: 8 }, T({ rot: [0, 0, rad(90)], pos: [0.45, 0.85, 0] }));
    disc(P.metal, { r: 0.20, h: 0.12, seg: 10 }, T({ pos: [0.45, 0.96, 0] }));
    for (const s of [1, -1]) {
        box(P.dark, [0, 0, s * 2.05], [0.34, 0.045, 3.90], T({ pos: [0.45, 1.00, 0] }));
        box(P.accent, [0, 0, s * 3.85], [0.34, 0.05, 0.35], T({ pos: [0.45, 1.00, 0] }));
    }

    // Tail rotor on the port side, plus fin and stabiliser.
    disc(P.dark, { r: 0.62, h: 0.03, seg: 14 }, T({ rot: [0, 0, rad(90)], pos: [-3.30, 0.45, 0.16] }));
    disc(P.accent, { r: 0.14, h: 0.05, seg: 8 }, T({ rot: [0, 0, rad(90)], pos: [-3.30, 0.45, 0.20] }));
    panel(P.accent, { x: -3.35, y: 0.30, z0: 0.0, span: 0.70, rootChord: 0.75, tipChord: 0.42, thick: 0.09, sweep: 0.32 }, UP);
    wings(P.body, { x: -2.60, y: 0.26, z0: 0.08, span: 0.62, rootChord: 0.42, tipChord: 0.30, thick: 0.07 });
};

// --- Ground rover (MAV_TYPE_GROUND_ROVER)
MODELS['rover'] = (P) => {
    box(P.body, [0, 0.10, 0], [2.40, 0.36, 1.30]);
    box(P.body, [-0.25, 0.48, 0], [1.20, 0.42, 1.10]);
    box(P.glass, [0.30, 0.48, 0], [0.16, 0.34, 1.00]);
    tube(P.accent, { x0: 1.18, x1: 1.34, r0: 0.16, r1: 0.16, seg: 10, y: 0.14, z: 0.42 });
    tube(P.accent, { x0: 1.18, x1: 1.34, r0: 0.16, r1: 0.16, seg: 10, y: 0.14, z: -0.42 });
    // Mast + GPS puck.
    box(P.dark, [-0.70, 0.85, 0], [0.06, 0.55, 0.06]);
    disc(P.accent, { r: 0.16, h: 0.07, seg: 10 }, T({ pos: [-0.70, 1.15, 0] }));
    for (const sx of [1, -1]) {
        for (const sz of [1, -1]) {
            disc(P.dark, { r: 0.38, h: 0.26, seg: 14 }, T({ rot: [rad(90), 0, 0], pos: [sx * 0.80, -0.18, sz * 0.72] }));
            disc(P.metal, { r: 0.15, h: 0.28, seg: 8 }, T({ rot: [rad(90), 0, 0], pos: [sx * 0.80, -0.18, sz * 0.72] }));
        }
    }
};

// --- Surface boat (MAV_TYPE_SURFACE_BOAT)
MODELS['boat'] = (P) => {
    // Planing hull: wide transom aft, fine entry forward, flared sheerline.
    hexa(P.body,
        [[-1.80, -0.45, -0.55], [1.55, -0.22, -0.10], [1.55, -0.22, 0.10], [-1.80, -0.45, 0.55]],
        [[-1.80, 0.24, -0.82], [2.15, 0.34, -0.10], [2.15, 0.34, 0.10], [-1.80, 0.24, 0.82]]);
    box(P.body, [-0.30, 0.30, 0], [1.60, 0.16, 1.55]);   // deck
    box(P.body, [-0.45, 0.62, 0], [0.95, 0.50, 1.05]);   // cabin
    box(P.glass, [0.05, 0.66, 0], [0.10, 0.34, 0.90]);
    tube(P.accent, { x0: 1.55, x1: 2.15, r0: 0.14, r1: 0.03, seg: 10, y: 0.30 });
    box(P.dark, [-0.45, 1.20, 0], [0.06, 0.66, 0.06]);   // mast
    disc(P.accent, { r: 0.15, h: 0.06, seg: 10 }, T({ pos: [-0.45, 1.55, 0] }));
    // Rudder + prop tunnel aft.
    panel(P.dark, { x: -1.85, y: -0.45, z0: 0.0, span: 0.42, rootChord: 0.40, tipChord: 0.28, thick: 0.07, sweep: 0.10 }, DOWN);
    tube(P.metal, { x0: -1.70, x1: -1.35, r0: 0.13, r1: 0.13, seg: 10, y: -0.42 });
};

// --- Submarine (MAV_TYPE_SUBMARINE)
MODELS['submarine'] = (P) => {
    tube(P.body, { x0: -1.55, x1: 1.55, r0: 0.42, r1: 0.42, seg: 16 });
    tube(P.accent, { x0: 1.55, x1: 2.15, r0: 0.42, r1: 0.06, seg: 16 });
    tube(P.body, { x0: -2.05, x1: -1.55, r0: 0.16, r1: 0.42, seg: 16 });
    box(P.body, [0.15, 0.52, 0], [0.90, 0.55, 0.40]);           // sail
    box(P.glass, [0.55, 0.52, 0], [0.12, 0.30, 0.34]);
    // Dive planes fore and aft, plus cruciform tail.
    wings(P.dark, { x: 0.30, y: 0.05, z0: 0.38, span: 0.70, rootChord: 0.45, tipChord: 0.30, thick: 0.08 });
    wings(P.dark, { x: -1.60, y: 0.05, z0: 0.30, span: 0.75, rootChord: 0.50, tipChord: 0.30, thick: 0.08, sweep: 0.15 });
    panel(P.dark, { x: -1.60, y: 0.05, z0: 0.28, span: 0.70, rootChord: 0.50, tipChord: 0.30, thick: 0.08, sweep: 0.15 }, UP);
    panel(P.dark, { x: -1.60, y: 0.05, z0: 0.28, span: 0.70, rootChord: 0.50, tipChord: 0.30, thick: 0.08, sweep: 0.15 }, DOWN);
    // Ducted thruster.
    tube(P.metal, { x0: -2.30, x1: -2.05, r0: 0.26, r1: 0.26, seg: 12 });
    for (const s of [1, -1]) box(P.accent, [-2.18, 0, s * 0.11], [0.06, 0.02, 0.22]);
    // Side thrusters read as an ROV rather than a torpedo.
    for (const s of [1, -1]) {
        tube(P.metal, { x0: -0.55, x1: -0.20, r0: 0.13, r1: 0.13, seg: 8, y: -0.30, z: s * 0.52 });
    }
};

// --- Antenna tracker (MAV_TYPE_ANTENNA_TRACKER)
MODELS['antenna-tracker'] = (P) => {
    // Tripod: each leg splayed outward along its own bearing.
    for (const a of [90, 210, 330]) {
        strut(P.dark, [Math.cos(rad(a)) * 0.10, 0.08, Math.sin(rad(a)) * 0.10],
            [Math.cos(rad(a)) * 0.46, -1.10, Math.sin(rad(a)) * 0.46], 0.09);
    }
    disc(P.body, { r: 0.34, h: 0.16, seg: 12 }, T({ pos: [0, 0.10, 0] }));   // pan base
    box(P.body, [0, 0.42, 0], [0.30, 0.50, 0.44]);                          // yoke
    tube(P.metal, { x0: -0.30, x1: 0.30, r0: 0.05, r1: 0.05, seg: 8 }, T({ rot: [rad(90), 0, 0], pos: [0, 0.62, 0] }));
    // Tilted Yagi boom with elements — clearly a ground station, not a vehicle.
    const boom = T({ rot: [0, 0, rad(-22)], pos: [0, 0.62, 0] });
    tube(P.body, { x0: -0.25, x1: 1.45, r0: 0.05, r1: 0.05, seg: 8 }, boom);
    const els = [[-0.15, 0.62], [0.15, 0.56], [0.45, 0.50], [0.75, 0.46], [1.05, 0.42], [1.32, 0.38]];
    for (const [x, half] of els) box(P.accent, [x, 0, 0], [0.035, 0.035, half * 2], boom);
};

// --- Airship / blimp (MAV_TYPE_AIRSHIP)
MODELS['airship'] = (P) => {
    ellipsoid(P.body, { rx: 3.20, ry: 0.95, rz: 0.95, seg: 16, rings: 9 }, T({ pos: [0.10, 0.20, 0] }));
    ellipsoid(P.accent, { rx: 0.45, ry: 0.42, rz: 0.42, seg: 12, rings: 6 }, T({ pos: [3.05, 0.20, 0] }));
    box(P.body, [0.10, -0.85, 0], [1.30, 0.42, 0.60]);      // gondola
    box(P.glass, [0.65, -0.85, 0], [0.16, 0.28, 0.50]);
    // Cruciform tail fins.
    for (const r of [0, 90, 180, 270]) {
        panel(P.dark, { x: -2.35, y: 0, z0: 0.55, span: 0.85, rootChord: 1.00, tipChord: 0.50, thick: 0.09, sweep: 0.45 },
            T({ rot: [rad(r), 0, 0], pos: [0, 0.20, 0] }));
    }
    for (const s of [1, -1]) {
        tube(P.metal, { x0: -0.15, x1: 0.15, r0: 0.12, r1: 0.12, seg: 8 }, T({ pos: [0.10, -0.85, s * 0.45] }));
        disc(P.dark, { r: 0.40, h: 0.03, seg: 12 }, T({ rot: [0, 0, rad(90)], pos: [0.22, -0.85, s * 0.45] }));
    }
};

// ---------------------------------------------------------------- GLB writer
function buildGLB(parts) {
    const prims = parts.filter((p) => p.tris.length);

    const chunks = [];   // raw Float32Array pieces, in accessor order
    const accessors = [];
    const bufferViews = [];
    const materials = [];
    const primitives = [];
    let offset = 0;

    for (const part of prims) {
        const n = part.tris.length * 3;
        const pos = new Float32Array(n * 3);
        const nrm = new Float32Array(n * 3);
        const min = [Infinity, Infinity, Infinity];
        const max = [-Infinity, -Infinity, -Infinity];
        let k = 0;
        for (const t of part.tris) {
            for (const v of t.p) {
                pos[k] = v[0]; pos[k + 1] = v[1]; pos[k + 2] = v[2];
                nrm[k] = t.n[0]; nrm[k + 1] = t.n[1]; nrm[k + 2] = t.n[2];
                for (let i = 0; i < 3; i++) {
                    if (v[i] < min[i]) min[i] = v[i];
                    if (v[i] > max[i]) max[i] = v[i];
                }
                k += 3;
            }
        }

        const push = (arr, withBounds) => {
            const bv = bufferViews.length;
            bufferViews.push({ buffer: 0, byteOffset: offset, byteLength: arr.byteLength, target: 34962 });
            chunks.push(Buffer.from(arr.buffer, arr.byteOffset, arr.byteLength));
            offset += arr.byteLength;
            const acc = { bufferView: bv, componentType: 5126, count: n, type: 'VEC3' };
            if (withBounds) { acc.min = min; acc.max = max; }
            accessors.push(acc);
            return accessors.length - 1;
        };

        const aPos = push(pos, true);
        const aNrm = push(nrm, false);

        materials.push({
            pbrMetallicRoughness: {
                baseColorFactor: part.color,
                metallicFactor: part.color === C.metal ? 0.8 : 0.1,
                roughnessFactor: part.color === C.glass ? 0.25 : 0.65,
            },
        });
        primitives.push({ attributes: { POSITION: aPos, NORMAL: aNrm }, material: materials.length - 1 });
    }

    const bin = Buffer.concat(chunks);
    const json = {
        asset: { version: '2.0', generator: 'Corv-GCS scripts/gen-models.js' },
        scene: 0,
        scenes: [{ nodes: [0] }],
        nodes: [{ mesh: 0 }],
        meshes: [{ primitives }],
        materials,
        accessors,
        bufferViews,
        buffers: [{ byteLength: bin.length }],
    };

    let jsonBuf = Buffer.from(JSON.stringify(json), 'utf8');
    while (jsonBuf.length % 4) jsonBuf = Buffer.concat([jsonBuf, Buffer.from(' ')]);
    let binBuf = bin;
    while (binBuf.length % 4) binBuf = Buffer.concat([binBuf, Buffer.from([0])]);

    const header = Buffer.alloc(12);
    header.writeUInt32LE(0x46546c67, 0);
    header.writeUInt32LE(2, 4);
    header.writeUInt32LE(12 + 8 + jsonBuf.length + 8 + binBuf.length, 8);

    const jHead = Buffer.alloc(8);
    jHead.writeUInt32LE(jsonBuf.length, 0);
    jHead.writeUInt32LE(0x4e4f534a, 4);
    const bHead = Buffer.alloc(8);
    bHead.writeUInt32LE(binBuf.length, 0);
    bHead.writeUInt32LE(0x004e4942, 4);

    return Buffer.concat([header, jHead, jsonBuf, bHead, binBuf]);
}

// ---------------------------------------------------------------- main
function main() {
    fs.mkdirSync(OUT_DIR, { recursive: true });
    for (const [name, build] of Object.entries(MODELS)) {
        const parts = {};
        for (const key of Object.keys(C)) parts[key] = new Part(C[key]);
        build(parts);

        const glb = buildGLB(Object.values(parts));
        const file = path.join(OUT_DIR, `${name}.glb`);
        fs.writeFileSync(file, glb);

        const tris = Object.values(parts).reduce((s, p) => s + p.tris.length, 0);
        console.log(`${name.padEnd(18)} ${String(tris).padStart(5)} tris  ${(glb.length / 1024).toFixed(1)} KB`);
    }
}

main();
