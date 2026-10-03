/**
 * MountPreview.js - Schematic 3D view of a sensor's mounting
 *
 * Drawn in the LIDAR and ROS mount settings: the vehicle as one thin triangle
 * with a small fin, fixed, seen from behind and above in a near-isometric
 * view; the sensor at its lever arm from the IMU, with its own axes (REP-103:
 * x red forward, y green left, z blue up); and the zone it collects from,
 * translucent, turned by the mount — a band, a cone, a fan, or the directions
 * the points actually came from.
 *
 * The orientation is the one the points are georeferenced with: the sensor
 * frame (FLU) is flipped to FRD and turned by eulerToMatrix(mount), exactly as
 * RosPoints.sensorToEnu() and lidar-core.js do, so what this shows is what the
 * map will get. The zone's radius is schematic, not to scale.
 *
 * Canvas 2D, orthographic: the view (x forward, y left, z up, metres) is
 * projected along a fixed direction from behind-right-above. Drawn on demand
 * (an input changed, the canvas resized), never per frame.
 */

import { eulerToMatrix } from '../ros/RosPoints.js';

const DEG = Math.PI / 180;
const ZONE_R = 0.8;                // m, schematic
const AXIS_L = 0.6;
const EXTENT = 1.2;                // m from the IMU to the canvas edge (the shorter side)
const COL = { x: '#ff5a4f', y: '#3ddc84', z: '#4d8dff', zone: '32, 210, 200' };

// Orthographic camera from behind-right-above, in the view frame (x fwd, y left, z up)
const CAM = (() => {
    const c = norm([-1.0, -0.85, 1.0]);            // towards the camera: ~37° above, near isometric
    const v = [-c[0], -c[1], -c[2]];               // view direction
    const right = norm(cross(v, [0, 0, 1]));
    const up = cross(right, v);
    return { v, right, up };
})();

function norm(a) { const l = Math.hypot(a[0], a[1], a[2]); return [a[0] / l, a[1] / l, a[2] / l]; }
function cross(a, b) { return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]; }
function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

/**
 * A sensor-frame direction (FLU) as the view frame sees it, through the mount:
 * FLU → FRD (x, −y, −z), R_mount, then body FRD → view (x, −y, −z).
 */
export function sensorToView(R, x, y, z) {
    const fx = x, fy = -y, fz = -z;
    const bx = R[0] * fx + R[1] * fy + R[2] * fz;
    const by = R[3] * fx + R[4] * fy + R[5] * fz;
    const bz = R[6] * fx + R[7] * fy + R[8] * fz;
    return [bx, -by, -bz];
}

/**
 * @param {HTMLCanvasElement} canvas
 * @param {object} o
 * @param {number[]} o.mount   roll, pitch, yaw, degrees (sensor relative to the body)
 * @param {number[]} o.lever   IMU → sensor, m, FRD
 * @param {object|null} o.zone { type: 'band', elMin, elMax } | { type: 'cone', half } |
 *                             { type: 'fan', min, max } | { type: 'dirs', az, el, bins } (degrees; bins: occupied indexes)
 * @param {string} [o.caption]
 */
export function drawMountPreview(canvas, { mount = [0, 0, 0], lever = [0, 0, 0], zone = null, caption = '' } = {}) {
    const cssW = canvas.clientWidth, cssH = canvas.clientHeight;
    if (!cssW || !cssH) return;                  // hidden (another sub-tab): drawn when it shows
    const dpr = window.devicePixelRatio || 1;
    if (canvas.width !== Math.round(cssW * dpr) || canvas.height !== Math.round(cssH * dpr)) {
        canvas.width = Math.round(cssW * dpr);
        canvas.height = Math.round(cssH * dpr);
    }
    const g = canvas.getContext('2d');
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, cssW, cssH);
    const ink = getComputedStyle(canvas).color || '#c8d0d8';

    const scale = Math.min(cssW, cssH) / (2 * EXTENT);
    const cx = cssW * 0.5, cy = cssH * 0.52;
    const P = (p) => [cx + dot(p, CAM.right) * scale, cy - dot(p, CAM.up) * scale];
    const depth = (p) => dot(p, CAM.v);
    const poly = (pts, fill, stroke, width = 1) => {
        g.beginPath();
        pts.forEach((p, i) => { const [x, y] = P(p); if (i) g.lineTo(x, y); else g.moveTo(x, y); });
        g.closePath();
        if (fill) { g.fillStyle = fill; g.fill(); }
        if (stroke) { g.strokeStyle = stroke; g.lineWidth = width; g.stroke(); }
    };
    const line = (a, b, stroke, width = 1, dash = null) => {
        const [x0, y0] = P(a), [x1, y1] = P(b);
        g.beginPath(); g.moveTo(x0, y0); g.lineTo(x1, y1);
        g.setLineDash(dash || []); g.strokeStyle = stroke; g.lineWidth = width; g.stroke(); g.setLineDash([]);
    };
    const label = (p, text, color, dx = 4, dy = -4) => {
        const [x, y] = P(p);
        g.fillStyle = color; g.font = '600 10px sans-serif'; g.fillText(text, x + dx, y + dy);
    };

    // Reference: level ground circle under the vehicle, faint
    const ring = [];
    for (let k = 0; k < 48; k++) { const a = k / 48 * 2 * Math.PI; ring.push([1.05 * Math.cos(a), 1.05 * Math.sin(a), 0]); }
    g.globalAlpha = 0.18; poly(ring, null, ink, 1); g.globalAlpha = 1;

    // The vehicle: one thin triangle, nose forward, with a small fin
    poly([[0.62, 0, 0], [-0.5, 0.3, 0], [-0.5, -0.3, 0]], 'rgba(200, 206, 214, 0.55)', ink, 1.2);
    poly([[-0.5, 0, 0], [-0.24, 0, 0], [-0.5, 0, 0.24]], 'rgba(200, 206, 214, 0.8)', ink, 1.2);
    label([0.62, 0, 0], 'FWD', ink, 6, 4);
    const [ix, iy] = P([0, 0, 0]);
    g.fillStyle = ink; g.beginPath(); g.arc(ix, iy, 2.2, 0, 2 * Math.PI); g.fill();

    // The sensor at its lever arm (FRD → view)
    const R = eulerToMatrix(mount[0] * DEG, mount[1] * DEG, mount[2] * DEG);
    const S = [lever[0] || 0, -(lever[1] || 0), -(lever[2] || 0)];
    const at = (d, r) => [S[0] + d[0] * r, S[1] + d[1] * r, S[2] + d[2] * r];

    // The zone, far faces first
    if (zone) {
        const faces = [];
        const quad = (a, b, c, d, alpha) => faces.push({ pts: [a, b, c, d], alpha, z: depth([(a[0] + c[0]) / 2, (a[1] + c[1]) / 2, (a[2] + c[2]) / 2]) });
        const tri = (a, b, c, alpha) => faces.push({ pts: [a, b, c], alpha, z: depth([(a[0] + b[0] + c[0]) / 3, (a[1] + b[1] + c[1]) / 3, (a[2] + b[2] + c[2]) / 3]) });
        const dirAt = (az, el) => sensorToView(R, Math.cos(el) * Math.cos(az), Math.cos(el) * Math.sin(az), Math.sin(el));
        const outline = [];
        if (zone.type === 'band') {
            const n = 48, e0 = zone.elMin * DEG, e1 = zone.elMax * DEG, lo = [], hi = [];
            for (let k = 0; k <= n; k++) {
                const a = k / n * 2 * Math.PI;
                lo.push(at(dirAt(a, e0), ZONE_R)); hi.push(at(dirAt(a, e1), ZONE_R));
            }
            for (let k = 0; k < n; k++) quad(lo[k], lo[k + 1], hi[k + 1], hi[k], 0.13);
            outline.push(lo, hi);
        } else if (zone.type === 'cone') {
            const n = 36, h = Math.max(1, zone.half) * DEG, rim = [];
            for (let k = 0; k <= n; k++) {
                const a = k / n * 2 * Math.PI;
                rim.push(at(sensorToView(R, Math.cos(h), Math.sin(h) * Math.cos(a), Math.sin(h) * Math.sin(a)), ZONE_R));
            }
            for (let k = 0; k < n; k++) tri(S, rim[k], rim[k + 1], 0.13);
            outline.push(rim);
            for (const k of [0, 9, 18, 27]) outline.push([S, rim[k]]);
        } else if (zone.type === 'fan') {
            const n = 40, a0 = zone.min * DEG, a1 = zone.max * DEG, arc = [];
            for (let k = 0; k <= n; k++) arc.push(at(dirAt(a0 + (a1 - a0) * k / n, 0), ZONE_R));
            for (let k = 0; k < n; k++) tri(S, arc[k], arc[k + 1], 0.16);
            outline.push(arc, [S, arc[0]], [S, arc[n]]);
        } else if (zone.type === 'dirs' && zone.bins && zone.bins.length) {
            const da = 2 * Math.PI / zone.az, de = Math.PI / zone.el;
            for (const b of zone.bins) {
                const i = b % zone.az, j = Math.floor(b / zone.az);
                const a0 = -Math.PI + i * da, e0 = -Math.PI / 2 + j * de;
                quad(at(dirAt(a0, e0), ZONE_R), at(dirAt(a0 + da, e0), ZONE_R), at(dirAt(a0 + da, e0 + de), ZONE_R), at(dirAt(a0, e0 + de), ZONE_R), 0.3);
            }
        }
        faces.sort((p, q) => q.z - p.z);
        for (const f of faces) poly(f.pts, `rgba(${COL.zone}, ${f.alpha})`, null);
        for (const o of outline) {
            g.beginPath();
            o.forEach((p, i) => { const [x, y] = P(p); if (i) g.lineTo(x, y); else g.moveTo(x, y); });
            g.strokeStyle = `rgba(${COL.zone}, 0.75)`; g.lineWidth = 1; g.stroke();
        }
    }

    // Lever arm, then the sensor's axes on top
    if (Math.hypot(S[0], S[1], S[2]) > 1e-3) line([0, 0, 0], S, ink, 1, [3, 3]);
    const axes = [['X', COL.x, sensorToView(R, 1, 0, 0)], ['Y', COL.y, sensorToView(R, 0, 1, 0)], ['Z', COL.z, sensorToView(R, 0, 0, 1)]];
    axes.sort((p, q) => depth(q[2]) - depth(p[2]));
    for (const [name, color, d] of axes) {
        const tip = at(d, AXIS_L);
        line(S, tip, color, 2.6);
        const [x, y] = P(tip);
        g.fillStyle = color; g.beginPath(); g.arc(x, y, 2.4, 0, 2 * Math.PI); g.fill();
        label(tip, name, color);
    }
    const [sx, sy] = P(S);
    g.fillStyle = '#ffffff'; g.strokeStyle = '#000000'; g.lineWidth = 1;
    g.beginPath(); g.arc(sx, sy, 3, 0, 2 * Math.PI); g.fill(); g.stroke();

    // Captions
    g.font = '10px sans-serif';
    g.fillStyle = ink; g.globalAlpha = 0.8;
    g.fillText('sensor X Y Z (REP-103) · zone not to scale', 6, 13);
    if (caption) g.fillText(caption, 6, cssH - 7);
    g.globalAlpha = 1;
}
