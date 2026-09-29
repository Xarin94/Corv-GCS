/**
 * Traffic3D.js - ADS-B traffic in the 3D view
 *
 * Each aircraft is a red circle sized in pixels as a function of its
 * distance — larger the closer it is, never smaller than a floor far away —
 * labelled with its callsign and its height relative to us, trailing the path
 * it has flown.
 *
 * Reports are sparse (OpenSky every 30 s, ADSB_VEHICLE about once a second),
 * so the trail is a cubic Hermite curve through them whose tangent at each
 * report is that report's own velocity: it bends where the aircraft turned
 * instead of cutting straight from fix to fix, and it fades with age, so the
 * bright end shows where the aircraft is going. Between reports the circle is
 * dead-reckoned along the last velocity, and the trail follows it.
 */

import { ThickLine } from './ThickLine.js';
import { SymbolLayer, SHAPE, makeLabel, fitLabel, disposeLabel } from './SymbolLayer.js';
import { latLonToMeters } from '../core/utils.js';
import { STATE } from '../core/state.js';

const TRAIL_SECONDS = 180;       // history drawn behind each aircraft
const MAX_SAMPLES = 90;          // reports kept per aircraft (1 Hz feed → 90 s)
const MAX_DEAD_RECKON_S = 30;    // never extrapolate further past the last report
const TRAIL_STEP_M = 60;         // curve resolution
const MAX_TRAIL_POINTS = 512;
const TRAIL_REFRESH_MS = 100;    // the circle moves every frame, the trail at 10 Hz
const LABEL_REFRESH_MS = 1000;
const LABEL_OFFSET_PX = 22;

let sceneRef = null;
let symbols = null;              // one circle per aircraft, same order as trackList
let color = 0xff2a2a;
let labelColor = 0xff2a2a;
const tracks = new Map();        // icao24 → track
let trackList = [];
let lastTrailRefresh = 0;
let viewportHeight = 0;
let cameraRef = null;

/**
 * @param {THREE.Scene} scene
 */
export function initTraffic3D(scene) {
    sceneRef = scene;
    symbols = new SymbolLayer({
        sizeNear: 38, sizeFar: 20, nearDist: 1000, farDist: 25000, ghostOpacity: 0.5, renderOrder: 14
    }).addTo(scene);
}

/**
 * Circle and trail colour, and label text colour (a new theme palette).
 * @param {number} hex
 * @param {number} [labelHex] defaults to the circle colour
 */
export function setTrafficColor(hex, labelHex = hex) {
    color = hex;
    labelColor = labelHex;
    for (const tr of trackList) {
        tr.line.setColor(hex);
        tr.labelText = null;         // redrawn on the next frame
    }
    rebuildSymbols();
}

/**
 * Take the latest traffic list (the nearest aircraft, from STATE.traffic).
 * New reports become trail samples; aircraft no longer listed are dropped.
 * @param {Array} list entries with icao24, lat, lon, alt, velocity (m/s),
 *   heading (deg true), vertRate (m/s), callsign, _ts / posTs (ms)
 */
export function updateTraffic3D(list) {
    if (!sceneRef) return;
    const offset = STATE.offsetAlt || 0;
    const listed = new Set();
    let changed = false;

    for (const ac of list || []) {
        if (ac.lat == null || ac.lon == null || !ac.icao24) continue;
        listed.add(ac.icao24);
        let tr = tracks.get(ac.icao24);
        if (!tr) {
            tr = {
                icao24: ac.icao24, samples: [], reportTs: 0,
                line: new ThickLine({ color, width: 2.5, vertexColors: true, renderOrder: 7 }).addTo(sceneRef),
                label: null, labelText: null, labelAt: 0,
                head: { x: 0, y: 0, z: 0 },
                flat: new Float32Array(MAX_TRAIL_POINTS * 3), rgba: new Float32Array(MAX_TRAIL_POINTS * 4)
            };
            tracks.set(ac.icao24, tr);
            changed = true;
        }
        tr.callsign = ac.callsign || '';
        tr.alt = ac.alt || 0;
        tr.vertRate = ac.vertRate || 0;

        const ts = ac.posTs || ac._ts || Date.now();
        if (ts !== tr.reportTs) {
            tr.reportTs = ts;
            addSample(tr, ac, ts / 1000, offset);
        }
    }

    for (const [icao, tr] of tracks) {
        if (listed.has(icao)) continue;
        tr.line.removeFrom(sceneRef);
        tr.line.dispose();
        disposeLabel(tr.label);
        tracks.delete(icao);
        changed = true;
    }
    if (changed) {
        trackList = [...tracks.values()];
        rebuildSymbols();
        lastTrailRefresh = 0;
    }
}

function addSample(tr, ac, t, offset) {
    const p = latLonToMeters(ac.lat, ac.lon);
    const y = (ac.alt || 0) + offset;
    const last = tr.samples[tr.samples.length - 1];
    // OpenSky repeats a state vector until the aircraft reports again: the
    // same position with a newer time would only restart the extrapolation
    if (last && Math.hypot(p.x - last.x, y - last.y, p.z - last.z) < 1) return;
    if (last && t <= last.t) return;

    // Velocity in world axes (x east, y up, z south), when the report has one
    let v = null;
    if (Number.isFinite(ac.velocity) && Number.isFinite(ac.heading)) {
        const h = ac.heading * Math.PI / 180;
        v = { x: ac.velocity * Math.sin(h), y: Number.isFinite(ac.vertRate) ? ac.vertRate : 0, z: -ac.velocity * Math.cos(h) };
    }
    tr.samples.push({ t, x: p.x, y, z: p.z, v });

    const cutoff = t - TRAIL_SECONDS - 30;
    while (tr.samples.length > 2 && (tr.samples.length > MAX_SAMPLES || tr.samples[1].t < cutoff)) tr.samples.shift();
}

function rebuildSymbols() {
    if (!symbols) return;
    symbols.setSymbols(trackList.map(tr => ({ x: tr.head.x, y: tr.head.y, z: tr.head.z, shape: SHAPE.CIRCLE, color, fill: true })));
}

/**
 * Advance every aircraft to now: dead-reckoned circle every frame, trail and
 * label when due.
 * @param {THREE.Camera} camera
 * @param {number} viewportHeightCss
 */
export function animateTraffic3D(camera, viewportHeightCss) {
    if (!symbols || trackList.length === 0) return;
    cameraRef = camera;
    viewportHeight = viewportHeightCss;
    const nowMs = Date.now();
    const now = nowMs / 1000;
    const trailDue = performance.now() - lastTrailRefresh >= TRAIL_REFRESH_MS;
    if (trailDue) lastTrailRefresh = performance.now();

    for (let i = 0; i < trackList.length; i++) {
        const tr = trackList[i];
        const last = tr.samples[tr.samples.length - 1];
        if (!last) continue;
        const dt = Math.max(0, Math.min(MAX_DEAD_RECKON_S, now - last.t));
        const v = last.v || { x: 0, y: 0, z: 0 };
        tr.head.x = last.x + v.x * dt;
        tr.head.y = last.y + v.y * dt;
        tr.head.z = last.z + v.z * dt;
        symbols.setPosition(i, tr.head.x, tr.head.y, tr.head.z);

        if (trailDue) buildTrail(tr, now, last.t + dt);
        updateLabel(tr, nowMs);
    }
}

/**
 * Trail through the reports, then along the dead-reckoned leg to the circle.
 * Alpha falls from 1 at the circle to 0 TRAIL_SECONDS back.
 */
function buildTrail(tr, now, headT) {
    const s = tr.samples;
    const flat = tr.flat, rgba = tr.rgba;
    let n = 0;
    const emit = (x, y, z, t) => {
        if (n >= MAX_TRAIL_POINTS) return;
        const age = Math.max(0, now - t);
        const a = Math.max(0, 1 - age / TRAIL_SECONDS);
        flat[n * 3] = x; flat[n * 3 + 1] = y; flat[n * 3 + 2] = z;
        rgba[n * 4] = 1; rgba[n * 4 + 1] = 1; rgba[n * 4 + 2] = 1; rgba[n * 4 + 3] = a * a;
        n++;
    };

    // Skip reports that have entirely faded
    let first = 0;
    while (first < s.length - 1 && now - s[first + 1].t > TRAIL_SECONDS) first++;
    // Keep the curve inside the point budget: on a dense feed step over reports
    const stride = Math.max(1, Math.ceil((s.length - first) / (MAX_TRAIL_POINTS / 4)));

    let prev = s[first];
    emit(prev.x, prev.y, prev.z, prev.t);
    for (let i = first + stride; i < s.length || prev !== s[s.length - 1]; i += stride) {
        const cur = s[Math.min(i, s.length - 1)];
        hermite(prev, cur, emit);
        prev = cur;
    }
    emit(tr.head.x, tr.head.y, tr.head.z, headT);

    tr.line.setPoints(flat, n);
    tr.line.setColors(rgba.subarray(0, n * 4));
}

/**
 * Cubic Hermite segment from a to b (a excluded, b included). Tangents are the
 * reported velocities scaled to the segment's duration; a missing or
 * implausible one (pointing backwards, or overshooting the gap by far) falls
 * back to the straight chord.
 */
function hermite(a, b, emit) {
    const dt = Math.max(1e-3, b.t - a.t);
    const cx = b.x - a.x, cy = b.y - a.y, cz = b.z - a.z;
    const chord = Math.hypot(cx, cy, cz);
    const steps = Math.max(1, Math.min(32, Math.ceil(chord / TRAIL_STEP_M)));
    const tangent = (v) => {
        if (!v) return { x: cx, y: cy, z: cz };
        let mx = v.x * dt, my = v.y * dt, mz = v.z * dt;
        if (mx * cx + my * cy + mz * cz <= 0) return { x: cx, y: cy, z: cz };
        const len = Math.hypot(mx, my, mz);
        if (len > 1.5 * chord && len > 0) {
            const k = 1.5 * chord / len;
            mx *= k; my *= k; mz *= k;
        }
        return { x: mx, y: my, z: mz };
    };
    const m0 = tangent(a.v), m1 = tangent(b.v);
    for (let k = 1; k <= steps; k++) {
        const u = k / steps, u2 = u * u, u3 = u2 * u;
        const h00 = 2 * u3 - 3 * u2 + 1, h10 = u3 - 2 * u2 + u;
        const h01 = -2 * u3 + 3 * u2, h11 = u3 - u2;
        emit(
            h00 * a.x + h10 * m0.x + h01 * b.x + h11 * m1.x,
            h00 * a.y + h10 * m0.y + h01 * b.y + h11 * m1.y,
            h00 * a.z + h10 * m0.z + h01 * b.z + h11 * m1.z,
            a.t + u * dt
        );
    }
}

/**
 * Callsign, and height relative to us with a trend arrow when climbing or
 * descending. Redrawn at most once a second, and only when the text changes.
 */
function updateLabel(tr, nowMs) {
    if (tr.label) tr.label.position.set(tr.head.x, tr.head.y, tr.head.z);
    if (tr.labelText !== null && nowMs - tr.labelAt < LABEL_REFRESH_MS) return;
    tr.labelAt = nowMs;

    const name = tr.callsign || tr.icao24.toUpperCase();
    const rel = Math.round((tr.alt - (STATE.rawAlt || 0)) / 10) * 10;
    const trend = tr.vertRate > 2 ? ' ↑' : tr.vertRate < -2 ? ' ↓' : '';
    const sub = `${rel >= 0 ? '+' : '−'}${Math.abs(rel)} m${trend}`;
    const text = name + '|' + sub;
    if (text === tr.labelText && tr.label) return;
    tr.labelText = text;

    disposeLabel(tr.label);
    tr.label = makeLabel(name, sub, labelColor);
    tr.label.position.set(tr.head.x, tr.head.y, tr.head.z);
    fitLabel(tr.label, cameraRef, viewportHeight, LABEL_OFFSET_PX);
    sceneRef.add(tr.label);
}

/** Re-fit the labels after a resize. */
export function resizeTraffic3D(camera, viewportHeightCss) {
    cameraRef = camera;
    viewportHeight = viewportHeightCss;
    for (const tr of trackList) if (tr.label) fitLabel(tr.label, camera, viewportHeightCss, LABEL_OFFSET_PX);
}
