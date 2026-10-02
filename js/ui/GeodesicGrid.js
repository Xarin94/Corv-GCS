/**
 * GeodesicGrid.js - The 80 sections ArduPilot's compass calibrator covers
 *
 * MAG_CAL_PROGRESS.completion_mask is 80 bits, one per section of
 * AP_GeodesicGrid (libraries/AP_Math/AP_GeodesicGrid.cpp): an icosahedron
 * whose 20 triangles are each split into four, the section number being
 * 4 * triangle + sub-triangle. To draw on a sphere which bits are set, the
 * GCS needs the same numbering, so the two tables below are the ones the
 * firmware uses and the sections are built from them rather than from an
 * icosahedron of our own (whose triangle order would not match).
 *
 * _inverses[i] maps a vector to its coordinates in the basis of triangle T_i's
 * vertices; inverting it gives the vertices back. T_{i+10} = -T_i.
 * _mid_inverses[i] does the same for the triangle of T_i's edge midpoints and
 * picks the sub-triangle: a negative x, y or z coordinate means sub-triangle
 * 3, 1 or 2; none means the middle one, 0.
 */

const INVERSES = [
    [[-0.309017,  0.500000,  0.190983], [ 0.000000,  0.000000, -0.618034], [-0.309017, -0.500000,  0.190983]],
    [[-0.190983,  0.309017, -0.500000], [-0.500000, -0.190983,  0.309017], [ 0.309017, -0.500000, -0.190983]],
    [[-0.618034,  0.000000,  0.000000], [ 0.190983, -0.309017, -0.500000], [ 0.190983, -0.309017,  0.500000]],
    [[-0.500000,  0.190983, -0.309017], [ 0.000000, -0.618034,  0.000000], [ 0.500000,  0.190983, -0.309017]],
    [[-0.190983, -0.309017, -0.500000], [-0.190983, -0.309017,  0.500000], [ 0.618034,  0.000000,  0.000000]],
    [[-0.309017, -0.500000, -0.190983], [ 0.190983,  0.309017, -0.500000], [ 0.500000, -0.190983,  0.309017]],
    [[ 0.309017, -0.500000,  0.190983], [ 0.000000,  0.000000, -0.618034], [ 0.309017,  0.500000,  0.190983]],
    [[ 0.190983, -0.309017, -0.500000], [ 0.500000,  0.190983,  0.309017], [-0.309017,  0.500000, -0.190983]],
    [[ 0.500000, -0.190983, -0.309017], [ 0.000000,  0.618034,  0.000000], [-0.500000, -0.190983, -0.309017]],
    [[ 0.309017,  0.500000, -0.190983], [-0.500000,  0.190983,  0.309017], [-0.190983, -0.309017, -0.500000]],
];

const MID_INVERSES = [
    [[ 0.000000,  1.000000, -0.618034], [ 0.000000, -1.000000, -0.618034], [-0.618034,  0.000000,  1.000000]],
    [[-1.000000,  0.618034,  0.000000], [ 0.000000, -1.000000,  0.618034], [ 0.618034,  0.000000, -1.000000]],
    [[-0.618034,  0.000000, -1.000000], [ 1.000000, -0.618034,  0.000000], [-0.618034,  0.000000,  1.000000]],
    [[-1.000000, -0.618034,  0.000000], [ 1.000000, -0.618034,  0.000000], [ 0.000000,  1.000000, -0.618034]],
    [[-1.000000, -0.618034,  0.000000], [ 0.618034,  0.000000,  1.000000], [ 0.618034,  0.000000, -1.000000]],
    [[-0.618034,  0.000000, -1.000000], [ 1.000000,  0.618034,  0.000000], [ 0.000000, -1.000000,  0.618034]],
    [[ 0.000000, -1.000000, -0.618034], [ 0.000000,  1.000000, -0.618034], [ 0.618034,  0.000000,  1.000000]],
    [[ 1.000000, -0.618034,  0.000000], [ 0.000000,  1.000000,  0.618034], [-0.618034,  0.000000, -1.000000]],
    [[ 1.000000,  0.618034,  0.000000], [-1.000000,  0.618034,  0.000000], [ 0.000000, -1.000000, -0.618034]],
    [[ 0.000000,  1.000000,  0.618034], [-1.000000, -0.618034,  0.000000], [ 0.618034,  0.000000, -1.000000]],
];

export const SECTION_COUNT = 80;

const mulMat = (m, v) => [
    m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
    m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
    m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
];

function invert3(m) {
    const [[a, b, c], [d, e, f], [g, h, i]] = m;
    const A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g;
    const det = a * A + b * B + c * C;
    return [
        [A / det, -(b * i - c * h) / det, (b * f - c * e) / det],
        [B / det, (a * i - c * g) / det, -(a * f - c * d) / det],
        [C / det, -(a * h - b * g) / det, (a * e - b * d) / det],
    ];
}

const normalize = v => { const l = Math.hypot(v[0], v[1], v[2]) || 1; return [v[0] / l, v[1] / l, v[2] / l]; };
const mid = (p, q) => [(p[0] + q[0]) / 2, (p[1] + q[1]) / 2, (p[2] + q[2]) / 2];
const triCentroid = (p, q, r) => [(p[0] + q[0] + r[0]) / 3, (p[1] + q[1] + r[1]) / 3, (p[2] + q[2] + r[2]) / 3];
const dot = (p, q) => p[0] * q[0] + p[1] * q[1] + p[2] * q[2];
const cross = (p, q) => [p[1] * q[2] - p[2] * q[1], p[2] * q[0] - p[0] * q[2], p[0] * q[1] - p[1] * q[0]];

/** AP_GeodesicGrid::_subtriangle_index for a vector known to cross T_triangle */
function subtriangleIndex(triangle, v) {
    let w = mulMat(MID_INVERSES[triangle % 10], v);
    if (triangle > 9) w = w.map(x => -x);
    if (w[0] < 0) return 3;
    if (w[1] < 0) return 1;
    if (w[2] < 0) return 2;
    return 0;
}

/**
 * The 80 sections as unit-vector triangles, indexed by section number.
 * Vertices are wound counter-clockwise seen from outside the sphere.
 * @returns {{a:number[], b:number[], c:number[], center:number[]}[]}
 */
function buildSections() {
    const sections = new Array(SECTION_COUNT);
    for (let t = 0; t < 20; t++) {
        const m = invert3(INVERSES[t % 10]);
        const sign = t > 9 ? -1 : 1;
        // Columns of the inverse are the triangle's vertices
        const [a, b, c] = [0, 1, 2].map(col => normalize([m[0][col] * sign, m[1][col] * sign, m[2][col] * sign]));
        const ab = mid(a, b), bc = mid(b, c), ca = mid(c, a);
        for (const tri of [[a, ab, ca], [ab, b, bc], [ca, bc, c], [ab, bc, ca]]) {
            const s = 4 * t + subtriangleIndex(t, triCentroid(...tri));
            let [p, q, r] = tri.map(normalize);
            if (dot(cross([q[0] - p[0], q[1] - p[1], q[2] - p[2]], [r[0] - p[0], r[1] - p[1], r[2] - p[2]]), p) < 0) [q, r] = [r, q];
            sections[s] = { a: p, b: q, c: r, center: normalize(triCentroid(p, q, r)) };
        }
    }
    return sections;
}

export const SECTIONS = buildSections();

/**
 * Section a direction falls in (the firmware's AP_GeodesicGrid::section, done
 * here by testing the 80 spherical triangles — 80 dot products is nothing at
 * the rate magnetometer samples arrive).
 * @param {number[]} v - any non-zero vector
 * @returns {number} 0..79, or -1 for the null vector
 */
export function sectionOf(v) {
    if (!v[0] && !v[1] && !v[2]) return -1;
    let best = -1, bestDot = -Infinity;
    for (let s = 0; s < SECTION_COUNT; s++) {
        const { a, b, c, center } = SECTIONS[s];
        if (dot(cross(a, b), v) >= 0 && dot(cross(b, c), v) >= 0 && dot(cross(c, a), v) >= 0) return s;
        const d = dot(center, v);
        if (d > bestDot) { bestDot = d; best = s; }
    }
    return best;   // on an edge, within rounding: the nearest section
}

/** True when bit `section` of a MAG_CAL_PROGRESS completion_mask is set */
export function maskHas(mask, section) {
    return !!mask && ((mask[section >> 3] || 0) & (1 << (section & 7))) !== 0;
}
