/**
 * Water3D.js - Water surface and underwater cues in the 3D view
 *
 * Surface plane: a horizontal grid on a water surface — the lake or sea the
 * vehicle is on, found in the elevation data — or on zero in the relative
 * navigation mode, where it is the only reference in an empty world. It is
 * seen from above and from below, and lives in the overlay layer, so it never
 * hides what is under it. Over a lake seen from above the terrain shader draws
 * the same grid itself; the plane is needed for the view from under the
 * surface and over the open sea, where the terrain is the sea bed.
 *
 * Particles: while the camera is under that surface, specks suspended in the
 * water. They are fixed in the world and wrapped around the camera (each one
 * is the copy of a lattice point nearest to it), so travelling through them
 * shows the direction and speed of motion, whatever produced the position.
 */

import { OVERLAY_LAYER } from './Layers.js';

const PARTICLE_COUNT = 2500;
const PARTICLE_BOX_M = 60;       // particles fill a cube this wide around the camera

const PLANE_VERTEX = `
varying vec2 vXZ;
void main() {
    vec4 world = modelMatrix * vec4(position, 1.0);
    vXZ = world.xz;
    gl_Position = projectionMatrix * viewMatrix * world;
}
`;

// Triangles (three line families 60° apart) or squares, fading where lines
// crowd closer than a few pixels and towards the rim; a faint fill so the
// surface reads as a sheet from below.
const PLANE_FRAGMENT = `
uniform vec2 uCenter;
uniform float uRadius;
uniform float uCell;
uniform float uTriangles;
uniform vec3 uColor;
uniform float uOpacity;
varying vec2 vXZ;
void main() {
    float r = length(vXZ - uCenter);
    if (r > uRadius) discard;
    vec3 x;
    if (uTriangles > 0.5) {
        x = vec3(vXZ.y, 0.8660254 * vXZ.x - 0.5 * vXZ.y, 0.8660254 * vXZ.x + 0.5 * vXZ.y) / (uCell * 0.8660254);
    } else {
        x = vec3(vXZ.x, vXZ.y, vXZ.x) / uCell;
    }
    vec3 fw = max(fwidth(x), vec3(1e-5));
    vec3 distPx = abs(fract(x + 0.5) - 0.5) / fw;
    vec3 line = 1.0 - smoothstep(vec3(0.0), vec3(1.0), distPx);
    float keep = smoothstep(2.5, 6.0, 1.0 / max(fw.x, max(fw.y, fw.z)));
    float edge = 1.0 - smoothstep(0.8 * uRadius, uRadius, r);
    float a = (max(line.x, max(line.y, line.z)) * keep + 0.06) * edge * uOpacity;
    if (a < 0.004) discard;
    gl_FragColor = vec4(uColor, a);
}
`;

const PARTICLE_VERTEX = `
uniform vec3 uCam;
uniform float uBox;
uniform float uLevel;
uniform float uPixelRatio;
varying float vAlpha;
void main() {
    vec3 rel = mod(position * uBox - uCam + 0.5 * uBox, uBox) - 0.5 * uBox;
    vec3 world = uCam + rel;
    vec4 mv = viewMatrix * vec4(world, 1.0);
    vAlpha = (1.0 - smoothstep(0.25 * uBox, 0.5 * uBox, length(rel))) * step(world.y, uLevel);
    gl_PointSize = clamp(60.0 / max(-mv.z, 0.1), 1.0, 5.0) * uPixelRatio;
    gl_Position = projectionMatrix * mv;
}
`;

const PARTICLE_FRAGMENT = `
uniform vec3 uColor;
varying float vAlpha;
void main() {
    vec2 c = gl_PointCoord * 2.0 - 1.0;
    float r = dot(c, c);
    if (r > 1.0 || vAlpha < 0.01) discard;
    gl_FragColor = vec4(uColor, vAlpha * (1.0 - r) * 0.75);
}
`;

let plane = null;
let particles = null;

/** @param {THREE.Scene} scene */
export function initWater3D(scene) {
    const planeMaterial = new THREE.ShaderMaterial({
        uniforms: {
            uCenter: { value: new THREE.Vector2() },
            uRadius: { value: 500 },
            uCell: { value: 25 },
            uTriangles: { value: 1 },
            uColor: { value: new THREE.Color(0x2a7fff) },
            uOpacity: { value: 1 }
        },
        vertexShader: PLANE_VERTEX,
        fragmentShader: PLANE_FRAGMENT,
        transparent: true,
        depthWrite: false,
        side: THREE.DoubleSide,
        forceSinglePass: true,   // one draw: three splits transparent double-sided meshes into back, then front
        // Over a lake the plane lies on the terrain's own surface: pull it forward
        polygonOffset: true,
        polygonOffsetFactor: -1,
        polygonOffsetUnits: -4
    });
    const geometry = new THREE.PlaneGeometry(2, 2);
    geometry.rotateX(-Math.PI / 2);
    plane = new THREE.Mesh(geometry, planeMaterial);
    plane.frustumCulled = false;
    plane.renderOrder = 2;
    plane.layers.set(OVERLAY_LAYER);
    plane.visible = false;
    scene.add(plane);

    const pos = new Float32Array(PARTICLE_COUNT * 3);
    for (let i = 0; i < pos.length; i++) pos[i] = Math.random();
    const pg = new THREE.BufferGeometry();
    pg.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    particles = new THREE.Points(pg, new THREE.ShaderMaterial({
        uniforms: {
            uCam: { value: new THREE.Vector3() },
            uBox: { value: PARTICLE_BOX_M },
            uLevel: { value: 0 },
            uPixelRatio: { value: 1 },
            uColor: { value: new THREE.Color(0xa8d8ff) }
        },
        vertexShader: PARTICLE_VERTEX,
        fragmentShader: PARTICLE_FRAGMENT,
        transparent: true,
        depthWrite: false
    }));
    particles.frustumCulled = false;
    particles.renderOrder = 8;
    particles.layers.set(OVERLAY_LAYER);
    particles.visible = false;
    scene.add(particles);
}

/**
 * Per frame, before rendering.
 * @param {object} view
 * @param {THREE.Camera} view.camera
 * @param {number} view.pixelRatio
 * @param {{level:number, x:number, z:number, radius:number, cell:number,
 *   triangles:boolean, color:number, opacity:number}|null} view.plane surface
 *   grid to draw, null for none
 * @param {number|null} view.underwaterLevel surface over the camera when the
 *   camera is under water (particles shown), null otherwise
 */
export function updateWater3D({ camera, pixelRatio, plane: p, underwaterLevel }) {
    if (!plane) return;
    plane.visible = !!p;
    if (p) {
        const u = plane.material.uniforms;
        plane.position.set(p.x, p.level, p.z);
        plane.scale.set(p.radius, 1, p.radius);
        u.uCenter.value.set(p.x, p.z);
        u.uRadius.value = p.radius;
        u.uCell.value = p.cell;
        u.uTriangles.value = p.triangles ? 1 : 0;
        u.uColor.value.setHex(p.color);
        u.uOpacity.value = p.opacity;
    }
    const under = Number.isFinite(underwaterLevel);
    particles.visible = under;
    if (under) {
        const u = particles.material.uniforms;
        u.uCam.value.copy(camera.position);
        u.uLevel.value = underwaterLevel;
        u.uPixelRatio.value = pixelRatio;
    }
}
