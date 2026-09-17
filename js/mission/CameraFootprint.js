/**
 * CameraFootprint.js - Where the camera looks, projected on the ground
 *
 * Pinhole camera at `alt` metres above flat ground, pointed by the gimbal:
 * pitch (0 = horizon, -90 = nadir) and azimuth (true heading of the optical
 * axis). The four corner rays of the field of view are intersected with the
 * ground plane; a corner that looks above the horizon is clamped to a maximum
 * range so an oblique shot still draws as a (long) trapezoid instead of
 * escaping to infinity. Flat ground at the vehicle's own terrain elevation is
 * the usual simplification for a 2D preview.
 */

import { cameraFov, localFrame } from './RouteModel.js';

const D2R = Math.PI / 180;

/**
 * @param {{lat,lng}} pos       camera position
 * @param {number} alt          height above the ground plane (m)
 * @param {number} azimuthDeg   true heading the camera looks along
 * @param {number} pitchDeg     0 horizon … -90 nadir
 * @param {object} cam          camera profile (sensorW/H, focal)
 * @param {number} maxRange     ground-distance cap for near-horizontal rays (m)
 * @returns {Array<{lat,lng}>|null} [nearLeft, nearRight, farRight, farLeft], or null when nothing hits the ground
 */
export function groundFootprint(pos, alt, azimuthDeg, pitchDeg, cam, maxRange = 8 * Math.max(alt, 10)) {
    if (!(alt > 0.5)) return null;
    const { fovH, fovV } = cameraFov(cam);
    const az = azimuthDeg * D2R, p = Math.max(-90, Math.min(89, pitchDeg)) * D2R;
    // ENU basis for the camera: forward on the ground, right, up
    const f = { x: Math.sin(az), y: Math.cos(az), z: 0 };
    const r = { x: Math.cos(az), y: -Math.sin(az), z: 0 };
    const u = { x: 0, y: 0, z: 1 };
    // Optical axis and the image "up" direction after pitching about `r`
    const d = { x: f.x * Math.cos(p), y: f.y * Math.cos(p), z: Math.sin(p) };
    const v = { x: -f.x * Math.sin(p), y: -f.y * Math.sin(p), z: Math.cos(p) };
    const th = Math.tan((fovH * D2R) / 2), tv = Math.tan((fovV * D2R) / 2);

    const corner = (sr, sv) => {
        const ray = {
            x: d.x + sr * th * r.x + sv * tv * v.x,
            y: d.y + sr * th * r.y + sv * tv * v.y,
            z: d.z + sr * th * r.z + sv * tv * v.z,
        };
        let t;
        if (ray.z < -1e-6) t = alt / -ray.z;
        else t = Infinity;
        const horiz = Math.hypot(ray.x, ray.y);
        const dist = Math.min(t * horiz, maxRange);
        if (!(dist >= 0) || horiz < 1e-9) return null;
        return { x: ray.x / horiz * dist, y: ray.y / horiz * dist };
    };
    const pts = [corner(-1, -1), corner(1, -1), corner(1, 1), corner(-1, 1)];
    if (pts.some(q => !q)) return null;
    const frame = localFrame(pos);
    return pts.map(frame.toLL);
}

/**
 * Gimbal aim at a point of interest from a vehicle position: azimuth and pitch.
 * @param {{lat,lng}} from  vehicle position
 * @param {number} altAgl   vehicle height above ground
 * @param {{lat,lng}} to    the POI
 * @param {number} targetH  POI height above the same ground (m)
 */
export function aimAt(from, altAgl, to, targetH = 0) {
    const frame = localFrame(from);
    const q = frame.toXY(to);
    const horiz = Math.hypot(q.x, q.y);
    const azimuth = (Math.atan2(q.x, q.y) / D2R + 360) % 360;
    const pitch = -Math.atan2(altAgl - targetH, horiz) / D2R;
    return { azimuth, pitch, distance: horiz };
}

/**
 * Photo positions along a lane at `spacing` metres, starting at the lane start.
 * @returns {Array<{lat,lng,heading}>}
 */
export function photosAlong(points, spacing, bearingFn, haversineFn) {
    const out = [];
    if (points.length < 2 || !(spacing > 0)) return out;
    let carry = 0;   // distance since the last photo, carried across vertices
    for (let i = 1; i < points.length; i++) {
        const a = points[i - 1], b = points[i];
        const len = haversineFn(a, b);
        const heading = bearingFn(a, b);
        if (i === 1) { out.push({ lat: a.lat, lng: a.lng, heading }); carry = 0; }
        let pos = spacing - carry;
        while (pos <= len) {
            const t = pos / len;
            out.push({ lat: a.lat + (b.lat - a.lat) * t, lng: a.lng + (b.lng - a.lng) * t, heading });
            pos += spacing;
        }
        carry = len - (pos - spacing);
    }
    return out;
}
