/**
 * RosVolume3D.js - A cave's 3D surface in the 3D view
 *
 * Draws the meshes SurfaceVolume.js builds in the worker (SURFACES 'cave'):
 * one geometry per chunk of 16³ voxels, replaced when the worker sends that
 * chunk again. Same look and switches as the height-field surface
 * (RosMesh3D.js, whose uniforms it shares): grey lines on the triangle edges
 * over a transparent body, the red → blue ramp by distance from the vehicle,
 * or roughness, VIEW on / off, through the terrain or depth-tested. A wall has
 * no height to fit a plane to, so here roughness is how much the faces around
 * each vertex disagree in direction (1 − |mean normal|, area-weighted): 0 on a
 * plane, full red from ~25° of spread (a rock, a ledge, a corner).
 *
 * The lines come from a barycentric attribute (1,0,0 / 0,1,0 / 0,0,1 on the
 * three corners of each triangle): the fragment shader draws ~1.5 px edges and
 * fades them where a triangle is only a few pixels tall, so far walls read as
 * a tinted sheet instead of a grey blur. The vertices are metres east, north,
 * up from the worker's anchor; one group maps them to the scene (flat-earth
 * scale, y up, z south) at the anchor's altitude plus the altitude offset, like
 * the vehicle model. Chunks are frustum-culled by three on their bounds.
 */

import { OVERLAY_LAYER } from '../engine/Layers.js';
import { STATE } from '../core/state.js';
import { ORIGIN } from '../core/constants.js';
import { latLonToMeters } from '../core/utils.js';
import { rosShared, rosView, ROS_RAMP_GLSL } from './RosMesh3D.js';

const RANGE_MS = 250;
const ROUGH_FULL_SPREAD = 0.1;     // 1 − |mean normal| drawn full red (~25°)

const VERTEX = `
attribute vec3 bary;
attribute float rough;
uniform vec3 uVehicle;
varying vec3 vBary;
varying float vDist;
varying float vRough;
void main() {
    vBary = bary;
    vRough = rough;
    vec4 world = modelMatrix * vec4(position, 1.0);
    vDist = distance(world.xyz, uVehicle);
    gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
}`;

const FRAGMENT = `
uniform vec3 uColor;
uniform float uFill;
uniform float uMode;
uniform float uNear;
uniform float uFar;
varying vec3 vBary;
varying float vDist;
varying float vRough;
${ROS_RAMP_GLSL}
void main() {
    vec3 fw = max(fwidth(vBary), vec3(1e-6));
    vec3 e = smoothstep(fw * 0.5, fw * 1.5, vBary);
    float edge = 1.0 - min(min(e.x, e.y), e.z);
    float keep = 1.0 - smoothstep(0.15, 0.35, max(fw.x, max(fw.y, fw.z)));   // triangle under ~5 px: no lines
    vec3 col = uColor;
    float fill = uFill;
    if (uMode > 1.5) {
        float t = clamp(vRough / ${ROUGH_FULL_SPREAD.toFixed(3)}, 0.0, 1.0);
        col = roughRamp(uColor, t);
        fill *= 1.0 + 3.5 * t;
    } else if (uMode > 0.5) {
        col = ramp(clamp((vDist - uNear) / max(uFar - uNear, 1.0), 0.0, 1.0));
        fill *= 2.5;
    }
    float a = edge * keep * 0.85 + fill;
    gl_FragColor = vec4(col, min(a, 1.0));
}`;

let group = null;
let material = null;
let anchor = null;
let cell = 0.3;
let builtOffset = null;
let lastRange = 0;
let range = { near: 0, far: 20 };
const chunks = new Map();          // key → { mesh, triangles }
let bary = new Float32Array(0);

export function initRosVolume(scene) {
    group = new THREE.Group();
    group.name = 'rosVolume';
    group.matrixAutoUpdate = false;
    group.visible = false;
    scene.add(group);
    material = new THREE.ShaderMaterial({
        uniforms: rosShared,
        vertexShader: VERTEX,
        fragmentShader: FRAGMENT,
        transparent: true,
        depthWrite: false,
        side: THREE.DoubleSide,
        forceSinglePass: true   // one draw per chunk: three splits transparent double-sided meshes into back, then front
    });
}

// 1,0,0 / 0,1,0 / 0,0,1 repeated, for `vertices` vertices (shared, grown on demand)
function baryFor(vertices) {
    if (bary.length < vertices * 3) {
        let n = Math.max(3, bary.length / 3);
        while (n < vertices) n *= 2;
        bary = new Float32Array(n * 3);
        for (let v = 0; v < n; v++) bary[v * 3 + (v % 3)] = 1;
    }
    return bary.subarray(0, vertices * 3);
}

/**
 * Per vertex of a triangle soup: 1 − |area-weighted mean of the normals of the
 * faces that share it| (vertices shared by position).
 */
export function normalSpread(pos) {
    const n = pos.length / 3, ids = new Int32Array(n), index = new Map();
    let u = 0;
    for (let v = 0; v < n; v++) {
        const key = `${pos[v * 3]},${pos[v * 3 + 1]},${pos[v * 3 + 2]}`;
        let id = index.get(key);
        if (id === undefined) { id = u++; index.set(key, id); }
        ids[v] = id;
    }
    const sum = new Float64Array(u * 3), area = new Float64Array(u);
    for (let t = 0; t < n; t += 3) {
        const ax = pos[t * 3], ay = pos[t * 3 + 1], az = pos[t * 3 + 2];
        const ux = pos[t * 3 + 3] - ax, uy = pos[t * 3 + 4] - ay, uz = pos[t * 3 + 5] - az;
        const vx = pos[t * 3 + 6] - ax, vy = pos[t * 3 + 7] - ay, vz = pos[t * 3 + 8] - az;
        const cx = uy * vz - uz * vy, cy = uz * vx - ux * vz, cz = ux * vy - uy * vx;
        const a = Math.hypot(cx, cy, cz);           // 2 × area; (cx, cy, cz) = normal × 2 × area
        if (!(a > 0)) continue;
        for (let k = 0; k < 3; k++) {
            const id = ids[t + k];
            sum[id * 3] += cx; sum[id * 3 + 1] += cy; sum[id * 3 + 2] += cz;
            area[id] += a;
        }
    }
    const out = new Float32Array(n);
    for (let v = 0; v < n; v++) {
        const id = ids[v];
        out[v] = area[id] > 0 ? 1 - Math.hypot(sum[id * 3], sum[id * 3 + 1], sum[id * 3 + 2]) / area[id] : 0;
    }
    return out;
}

/** Chunk meshes from the worker: { reset, anchor, cell, meshes: [{ key, pos }], removed: [key] }. */
export function applyRosVolume(m) {
    if (!group) return;
    if (m.reset || !m.anchor || !anchor || m.anchor.epoch !== anchor.epoch || m.cell !== cell) clearRosVolume();
    anchor = m.anchor;
    cell = m.cell;
    if (!anchor) return;
    builtOffset = null;                     // place the group on the next frame
    for (const key of m.removed || []) dropChunk(key);
    for (const { key, pos } of m.meshes || []) {
        dropChunk(key);
        if (!pos.length) continue;
        const geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
        geo.setAttribute('bary', new THREE.BufferAttribute(baryFor(pos.length / 3), 3));
        geo.setAttribute('rough', new THREE.BufferAttribute(normalSpread(pos), 1));
        geo.computeBoundingSphere();
        const mesh = new THREE.Mesh(geo, material);
        mesh.renderOrder = 3;
        mesh.layers.set(OVERLAY_LAYER);
        group.add(mesh);
        chunks.set(key, { mesh, triangles: pos.length / 9 });
    }
}

function dropChunk(key) {
    const c = chunks.get(key);
    if (!c) return;
    group.remove(c.mesh);
    c.mesh.geometry.dispose();
    chunks.delete(key);
}

export function clearRosVolume() {
    for (const key of [...chunks.keys()]) dropChunk(key);
    anchor = null;
}

/** Per frame: visibility and depth test from the shared switches, placement, colour range. */
export function updateRosVolume() {
    if (!group) return;
    group.visible = rosView.visible && rosView.shown && chunks.size > 0;
    if (!group.visible || !anchor) return;
    if (material.depthTest === rosView.overTerrain) {
        material.depthTest = !rosView.overTerrain;
        material.needsUpdate = true;
    }
    const offset = STATE.offsetAlt || 0;
    if (offset !== builtOffset) {
        builtOffset = offset;
        const base = latLonToMeters(anchor.lat, anchor.lon);
        const kx = (111320 * Math.cos(ORIGIN.lat * Math.PI / 180)) / anchor.mPerLon;
        const kz = 111320 / anchor.mPerLat;
        // (east, north, up) → (x = east, y = up, z = −north), then to the anchor
        group.matrix.set(kx, 0, 0, base.x, 0, 0, 1, anchor.alt + offset, 0, -kz, 0, base.z, 0, 0, 0, 1);
        group.matrixWorldNeedsUpdate = true;
    }
    const veh = latLonToMeters(STATE.lat, STATE.lon);
    rosShared.uVehicle.value.set(veh.x, (STATE.rawAlt || 0) + offset, veh.z);
    const now = performance.now();
    if (rosShared.uMode.value === 1 && now - lastRange > RANGE_MS) {
        lastRange = now;
        updateRange();
    }
}

// Nearest mesh vertex to the vehicle, among the chunks whose bounds come
// closest; the ramp runs from there to NEAR + max(20 m, 2 × NEAR)
function updateRange() {
    const v = rosShared.uVehicle.value, inv = new THREE.Matrix4().copy(group.matrix).invert();
    const p = v.clone().applyMatrix4(inv);            // vehicle in the chunks' frame
    let near = Infinity;
    for (const { mesh } of chunks.values()) {
        const s = mesh.geometry.boundingSphere;
        if (!s || s.center.distanceTo(p) - s.radius > near) continue;
        const a = mesh.geometry.attributes.position.array;
        for (let k = 0; k < a.length; k += 3) {
            const d = Math.hypot(a[k] - p.x, a[k + 1] - p.y, a[k + 2] - p.z);
            if (d < near) near = d;
        }
    }
    if (!Number.isFinite(near)) return;
    range = { near, far: near + Math.max(20, 2 * near) };
    rosShared.uNear.value = range.near;
    rosShared.uFar.value = range.far;
}

/** Distance range of the colour ramp, m. */
export function getRosVolumeColorRange() { return range; }

/** What is drawn now (status line, tests). */
export function getRosVolumeStats() {
    let triangles = 0;
    for (const c of chunks.values()) triangles += c.triangles;
    return { chunks: chunks.size, triangles };
}

/** The meshes drawn now, metres from the anchor (diagnostics, tests). */
export function getRosVolume() {
    return anchor ? { anchor, cell, meshes: [...chunks].map(([key, c]) => ({ key, pos: c.mesh.geometry.attributes.position.array })) } : null;
}
