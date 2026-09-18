/**
 * LidarCloud.js - Georeferenced point cloud in the 3D scene
 *
 * Draws the voxel map the main process accumulates from the Livox Mid-360.
 * Points arrive already georeferenced in ENU metres from a cloud origin
 * (lat/lon/alt MSL); this module places one THREE.Group at that origin in
 * scene coordinates and appends the points into fixed-size chunks of
 * THREE.Points, so a batch never reallocates a buffer that is already on the
 * GPU — the same growth strategy as the flight trail, just with a hard cap.
 *
 * Colour is computed in the vertex shader (height ramp or reflectivity), so
 * switching the mode or re-ranging the height scale never touches the
 * vertex data: the height range follows the 2nd–98th percentile of the
 * accumulated cloud through a small histogram, the way lidar viewers
 * (CloudCompare, Livox Viewer, DJI Terra) auto-scale their colour bar.
 *
 * A second, transient layer holds the LIVE points the worker sends while the
 * gate is closed (no GPS, EKF not converged…): vehicle-relative, written into
 * a ring buffer with a birth time, clipped and faded by the shader once older
 * than the TTL. Nothing per frame on the CPU: the operator sees the scan, the
 * map keeps only what was georeferenced.
 *
 * Scene frame reminder: x = east, y = up, z = south. The app's
 * latLonToMeters() scales longitude with cos(ORIGIN.lat) — the fixed app
 * origin, not the local latitude — so the group is scaled to that same
 * convention or the cloud would drift from the aircraft model away from the
 * origin.
 */

import { STATE } from '../core/state.js';
import { ORIGIN } from '../core/constants.js';
import { latLonToMeters } from '../core/utils.js';

const CHUNK = 262144;                  // points per BufferGeometry
const HIST_MIN = -1000, HIST_MAX = 3000, HIST_BINS = 4000;   // 1 m bins, relative to the origin altitude

let group = null;
let material = null;
let chunks = [];                       // [{ points, posAttr, intAttr, n }]
let total = 0;
let maxPoints = 3000000;
let origin = null;                     // { lat, lon, alt, mPerLat, mPerLon, epoch }
let visible = false;
let hist = new Int32Array(HIST_BINS);
let histDirty = false;
let lastRange = 0;

// Live layer
const LIVE_CAP = 262144;
let liveGroup = null;
let liveMaterial = null;
let livePos = null, liveInt = null, liveBirth = null;   // BufferAttributes
let liveHead = 0, liveCount = 0;
let liveFrame = 'ned';
let liveTtl = 3;
const liveEuler = new THREE.Euler(0, 0, 0, 'YXZ');

const VERT = `
attribute float intensity;
uniform float uSize;
uniform float uMode;
uniform float uHmin;
uniform float uHmax;
varying float vT;
void main() {
    vec4 mv = modelViewMatrix * vec4(position, 1.0);
    gl_Position = projectionMatrix * mv;
    gl_PointSize = uSize;
    float h = clamp((position.y - uHmin) / max(uHmax - uHmin, 0.5), 0.0, 1.0);
    vT = uMode < 0.5 ? h : intensity;
}`;

// Live variant: birth time per point, clipped past the TTL and faded over its
// last 40 %. Height ramp spans -60…+20 m around the vehicle.
const VERT_LIVE = `
attribute float intensity;
attribute float birth;
uniform float uSize;
uniform float uMode;
uniform float uNow;
uniform float uTtl;
varying float vT;
varying float vAlpha;
void main() {
    float age = uNow - birth;
    if (age > uTtl || age < 0.0) { gl_Position = vec4(2.0, 2.0, 2.0, 1.0); gl_PointSize = 0.0; vAlpha = 0.0; vT = 0.0; return; }
    vec4 mv = modelViewMatrix * vec4(position, 1.0);
    gl_Position = projectionMatrix * mv;
    gl_PointSize = uSize;
    vAlpha = clamp((uTtl - age) / (0.4 * uTtl), 0.0, 1.0);
    float h = clamp((position.y + 60.0) / 80.0, 0.0, 1.0);
    vT = uMode < 0.5 ? h : intensity;
}`;

// Turbo colour map, polynomial fit (Mikhailov, Google AI 2019).
const RAMP = `
uniform float uMode;
varying float vT;
vec3 turbo(float t) {
    const vec4 kR4 = vec4(0.13572138, 4.61539260, -42.66032258, 132.13108234);
    const vec4 kG4 = vec4(0.09140261, 2.19418839, 4.84296658, -14.18503333);
    const vec4 kB4 = vec4(0.10667330, 12.64194608, -60.58204836, 110.36276771);
    const vec2 kR2 = vec2(-152.94239396, 59.28637943);
    const vec2 kG2 = vec2(4.27729857, 2.82956604);
    const vec2 kB2 = vec2(-89.90310912, 27.34824973);
    vec4 v4 = vec4(1.0, t, t * t, t * t * t);
    vec2 v2 = v4.zw * v4.z;
    return vec3(dot(v4, kR4) + dot(v2, kR2), dot(v4, kG4) + dot(v2, kG2), dot(v4, kB4) + dot(v2, kB2));
}
vec3 ramp(float t) {
    return uMode < 0.5
        ? turbo(clamp(t, 0.0, 1.0))
        : mix(vec3(0.02, 0.18, 0.06), vec3(0.75, 1.0, 0.8), pow(clamp(t, 0.0, 1.0), 0.6));
}`;

const FRAG = RAMP + `
void main() {
    vec2 c = gl_PointCoord - 0.5;
    if (dot(c, c) > 0.25) discard;
    gl_FragColor = vec4(ramp(vT), 1.0);
}`;

const FRAG_LIVE = RAMP + `
varying float vAlpha;
void main() {
    vec2 c = gl_PointCoord - 0.5;
    if (dot(c, c) > 0.25 || vAlpha <= 0.0) discard;
    gl_FragColor = vec4(ramp(vT), vAlpha);
}`;

export function initLidarCloud(scene) {
    group = new THREE.Group();
    group.name = 'lidarCloud';
    group.visible = false;
    scene.add(group);

    material = new THREE.ShaderMaterial({
        uniforms: {
            uSize: { value: 2 * (window.devicePixelRatio || 1) },
            uMode: { value: 0 },
            uHmin: { value: -10 },
            uHmax: { value: 50 }
        },
        vertexShader: VERT,
        fragmentShader: FRAG,
        depthTest: true,
        depthWrite: true
    });

    liveGroup = new THREE.Group();
    liveGroup.name = 'lidarLive';
    liveGroup.visible = false;
    scene.add(liveGroup);
    liveMaterial = new THREE.ShaderMaterial({
        uniforms: {
            uSize: { value: 2 * (window.devicePixelRatio || 1) },
            uMode: { value: 0 },
            uNow: { value: 0 },
            uTtl: { value: liveTtl }
        },
        vertexShader: VERT_LIVE,
        fragmentShader: FRAG_LIVE,
        transparent: true,
        depthTest: true,
        depthWrite: false
    });
    const geo = new THREE.BufferGeometry();
    livePos = new THREE.BufferAttribute(new Float32Array(LIVE_CAP * 3), 3);
    liveInt = new THREE.BufferAttribute(new Uint8Array(LIVE_CAP), 1, true);
    liveBirth = new THREE.BufferAttribute(new Float32Array(LIVE_CAP), 1);
    for (const a of [livePos, liveInt, liveBirth]) a.setUsage(THREE.DynamicDrawUsage);
    geo.setAttribute('position', livePos);
    geo.setAttribute('intensity', liveInt);
    geo.setAttribute('birth', liveBirth);
    geo.setDrawRange(0, 0);
    const pts = new THREE.Points(geo, liveMaterial);
    pts.frustumCulled = false;
    liveGroup.add(pts);
}

/**
 * Live points: vehicle-relative (frame 'ned' = n,e,d levelled; 'body' = FRD),
 * appended into the ring with the current time as birth.
 */
export function appendLiveLidarPoints(batch) {
    if (!liveGroup || !batch || !batch.xyz) return;
    const xyz = batch.xyz, inten = batch.intensity;
    const count = Math.floor(xyz.length / 3);
    if (batch.frame) liveFrame = batch.frame;
    if (batch.ttl) liveTtl = batch.ttl;
    const now = performance.now() / 1000;
    const pos = livePos.array, ia = liveInt.array, ba = liveBirth.array;
    let k = 0;
    while (k < count) {
        const start = liveHead;
        const room = Math.min(LIVE_CAP - liveHead, count - k);
        for (let i = 0; i < room; i++) {
            const sidx = (k + i) * 3, d = (start + i) * 3;
            // (n,e,d) → x=e, y=-d, z=-n; the body frame (f,r,d) maps the same
            // way onto the vehicle's local axes (right=+x, down=-y, fwd=-z).
            pos[d] = xyz[sidx + 1]; pos[d + 1] = -xyz[sidx + 2]; pos[d + 2] = -xyz[sidx];
            ia[start + i] = inten ? inten[k + i] : 128;
            ba[start + i] = now;
        }
        markRange(livePos, start * 3, room * 3);
        markRange(liveInt, start, room);
        markRange(liveBirth, start, room);
        liveHead = (liveHead + room) % LIVE_CAP;
        liveCount = Math.min(LIVE_CAP, liveCount + room);
        k += room;
    }
    liveGroup.children[0].geometry.setDrawRange(0, liveCount);
}

// Partial upload; when two spans land in one frame, widen to cover both
// (WebGLAttributes resets updateRange.count to -1 after each upload).
function markRange(attr, offset, count) {
    if (attr.updateRange.count > 0) {
        const a = Math.min(attr.updateRange.offset, offset);
        const b = Math.max(attr.updateRange.offset + attr.updateRange.count, offset + count);
        attr.updateRange.offset = a;
        attr.updateRange.count = b - a;
    } else {
        attr.updateRange.offset = offset;
        attr.updateRange.count = count;
    }
    attr.needsUpdate = true;
}

export function clearLiveLidarPoints() {
    if (!liveGroup) return;
    liveHead = 0;
    liveCount = 0;
    liveGroup.children[0].geometry.setDrawRange(0, 0);
}

function newChunk() {
    const geo = new THREE.BufferGeometry();
    const posAttr = new THREE.BufferAttribute(new Float32Array(CHUNK * 3), 3);
    const intAttr = new THREE.BufferAttribute(new Uint8Array(CHUNK), 1, true);   // normalized → 0..1 in the shader
    posAttr.setUsage(THREE.DynamicDrawUsage);
    intAttr.setUsage(THREE.DynamicDrawUsage);
    geo.setAttribute('position', posAttr);
    geo.setAttribute('intensity', intAttr);
    geo.setDrawRange(0, 0);
    const points = new THREE.Points(geo, material);
    points.frustumCulled = false;      // bounds grow with every batch; culling 16 objects buys nothing
    points.renderOrder = newChunk._renderOrder || 0;
    group.add(points);
    const c = { points, posAttr, intAttr, n: 0 };
    chunks.push(c);
    return c;
}

/**
 * Anchor the cloud. Called by the main process when the first point of a
 * (new) map is accepted; clears anything drawn under a previous epoch.
 */
export function setLidarOrigin(o) {
    if (!group || !o) return;
    if (origin && origin.epoch !== o.epoch) clearLidarCloud();
    origin = o;
    const p = latLonToMeters(o.lat, o.lon);
    group.position.set(p.x, o.alt + (STATE.offsetAlt || 0), p.z);
    // ENU → scene under the app's flat-earth convention (see header).
    const kx = (111320 * Math.cos(ORIGIN.lat * Math.PI / 180)) / o.mPerLon;
    const kz = 111320 / o.mPerLat;
    group.scale.set(kx, 1, kz);
}

/**
 * Append a batch of ENU points (Float32Array x,y,z = e,n,u) with intensities.
 */
export function appendLidarPoints(batch) {
    if (!group || !origin || !batch || !batch.enu) return;
    if (batch.epoch !== undefined && batch.epoch !== origin.epoch) return;   // from before a clear
    const enu = batch.enu, inten = batch.intensity;
    const count = Math.floor(enu.length / 3);
    let k = 0;
    while (k < count && total < maxPoints) {
        let c = chunks[chunks.length - 1];
        if (!c || c.n >= CHUNK) c = newChunk();
        const start = c.n;
        const room = Math.min(CHUNK - c.n, count - k, maxPoints - total);
        const pos = c.posAttr.array, ia = c.intAttr.array;
        for (let i = 0; i < room; i++) {
            const s = (k + i) * 3, d = (start + i) * 3;
            const e = enu[s], n = enu[s + 1], u = enu[s + 2];
            pos[d] = e; pos[d + 1] = u; pos[d + 2] = -n;
            ia[start + i] = inten ? inten[k + i] : 128;
            const bin = Math.floor(u - HIST_MIN);
            if (bin >= 0 && bin < HIST_BINS) hist[bin]++;
        }
        c.n += room;
        total += room;
        k += room;
        c.points.geometry.setDrawRange(0, c.n);
        c.posAttr.updateRange.offset = start * 3;
        c.posAttr.updateRange.count = room * 3;
        c.posAttr.needsUpdate = true;
        c.intAttr.updateRange.offset = start;
        c.intAttr.updateRange.count = room;
        c.intAttr.needsUpdate = true;
    }
    histDirty = true;
}

export function clearLidarCloud() {
    if (!group) return;
    for (const c of chunks) {
        group.remove(c.points);
        c.points.geometry.dispose();
    }
    chunks = [];
    total = 0;
    hist.fill(0);
    histDirty = true;
}

export function setLidarCloudVisible(v) {
    visible = !!v;
    if (group) group.visible = visible;
    if (liveGroup) liveGroup.visible = visible;
    if (!visible) clearLiveLidarPoints();
}

export function setLidarColorMode(mode) {
    const m = mode === 'intensity' ? 1 : 0;
    if (material) material.uniforms.uMode.value = m;
    if (liveMaterial) liveMaterial.uniforms.uMode.value = m;
}

export function setLidarPointSize(px) {
    const v = Math.max(1, px || 2) * (window.devicePixelRatio || 1);
    if (material) material.uniforms.uSize.value = v;
    if (liveMaterial) liveMaterial.uniforms.uSize.value = v;
}

export function setLidarMaxPoints(n) {
    maxPoints = Math.max(1000, n | 0);
}

/**
 * Draw the cloud through the terrain mesh. SRTM is 30 m / ±5 m; the LiDAR
 * ground is the truth, and with the depth test on whole strips vanish
 * wherever the mesh sits a metre above the real surface.
 */
export function setLidarOverTerrain(over) {
    if (!material) return;
    material.depthTest = !over;
    material.depthWrite = !over;
    material.transparent = !!over;      // sorts after the opaque terrain
    material.needsUpdate = true;
    for (const c of chunks) c.points.renderOrder = over ? 10 : 0;
    newChunk._renderOrder = over ? 10 : 0;
    if (liveMaterial) {
        liveMaterial.depthTest = !over;
        liveMaterial.needsUpdate = true;
        liveGroup.children[0].renderOrder = over ? 11 : 1;
    }
}

export function getLidarPointCount() { return total; }

/**
 * Per-frame: follow the altitude offset and re-range the height colour bar.
 */
export function updateLidarCloud() {
    if (!visible) return;
    if (liveGroup && liveCount > 0) {
        // Vehicle-relative: sit on the aircraft; levelled (NED) points only
        // need the position, body-frame points also take the attitude, with
        // the same Euler order the vehicle model uses.
        const p = latLonToMeters(STATE.lat, STATE.lon);
        liveGroup.position.set(p.x, Math.max(STATE.rawAlt + (STATE.offsetAlt || 0), 1), p.z);
        if (liveFrame === 'body') liveEuler.set(STATE.pitch, -STATE.yaw, -STATE.roll, 'YXZ');
        else liveEuler.set(0, 0, 0, 'YXZ');
        liveGroup.setRotationFromEuler(liveEuler);
        liveMaterial.uniforms.uNow.value = performance.now() / 1000;
        liveMaterial.uniforms.uTtl.value = liveTtl;
    }
    if (!group || !origin) return;
    group.position.y = origin.alt + (STATE.offsetAlt || 0);
    const now = performance.now();
    if (histDirty && now - lastRange > 500) {
        lastRange = now;
        histDirty = false;
        const lo = percentile(0.02), hi = percentile(0.98);
        if (hi > lo) {
            material.uniforms.uHmin.value = lo;
            material.uniforms.uHmax.value = hi;
        }
    }
}

function percentile(q) {
    if (total === 0) return 0;
    const target = q * total;
    let acc = 0;
    for (let b = 0; b < HIST_BINS; b++) {
        acc += hist[b];
        if (acc >= target) return HIST_MIN + b;
    }
    return HIST_MAX;
}
