/** GPU resources for the neutral PointCloudData model. */
import { OVERLAY_LAYER } from '../Layers.js';
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


export function createThreePointCloudRenderer({ three: THREE, scene, renderer, model }) {
    const group = new THREE.Group(), liveGroup = new THREE.Group();
    group.name = 'lidarCloud'; liveGroup.name = 'lidarLive';
    scene.add(group, liveGroup);
    const material = new THREE.ShaderMaterial({ uniforms: {
        uSize: { value: 2 }, uMode: { value: 0 }, uHmin: { value: -10 }, uHmax: { value: 50 }
    }, vertexShader: VERT, fragmentShader: FRAG, depthTest: true, depthWrite: true });
    const liveMaterial = new THREE.ShaderMaterial({ uniforms: {
        uSize: { value: 2 }, uMode: { value: 0 }, uNow: { value: 0 }, uTtl: { value: 3 }
    }, vertexShader: VERT_LIVE, fragmentShader: FRAG_LIVE,
        transparent: true, depthTest: true, depthWrite: false });
    const mapped = new Map();
    let generation = model.generation;
    function pointResource(block, live = false) {
        const geometry = new THREE.BufferGeometry();
        const position = new THREE.BufferAttribute(block.positions, 3).setUsage(THREE.DynamicDrawUsage);
        const intensity = new THREE.BufferAttribute(block.intensity, 1, true).setUsage(THREE.DynamicDrawUsage);
        geometry.setAttribute('position', position); geometry.setAttribute('intensity', intensity);
        let birth;
        if (live) {
            birth = new THREE.BufferAttribute(block.birth, 1).setUsage(THREE.DynamicDrawUsage);
            geometry.setAttribute('birth', birth);
        }
        geometry.setDrawRange(0, block.count);
        const points = new THREE.Points(geometry, live ? liveMaterial : material);
        points.frustumCulled = false; points.layers.set(OVERLAY_LAYER);
        points.renderOrder = model.overTerrain ? (live ? 11 : 10) : (live ? 1 : 0);
        (live ? liveGroup : group).add(points);
        return { points, geometry, position, intensity, birth };
    }
    const liveResource = pointResource(model.live, true);
    function clearHistory() {
        for (const r of mapped.values()) { group.remove(r.points); r.geometry.dispose(); }
        mapped.clear(); generation = model.generation;
    }
    function ensureGeneration() { if (generation !== model.generation) clearHistory(); }
    function mark(attribute, start, count) {
        attribute.addUpdateRange(start, count); attribute.needsUpdate = true;
    }
    function applyMapped(updates) {
        ensureGeneration();
        for (const { block, start, count } of updates) {
            let r = mapped.get(block.id);
            if (!r) { r = pointResource(block); mapped.set(block.id, r); }
            r.geometry.setDrawRange(0, block.count);
            mark(r.position, start * 3, count * 3); mark(r.intensity, start, count);
        }
    }
    function applyLive(updates) {
        for (const { start, count } of updates) {
            mark(liveResource.position, start * 3, count * 3);
            mark(liveResource.intensity, start, count); mark(liveResource.birth, start, count);
        }
        liveResource.geometry.setDrawRange(0, model.live.count);
    }
    function updateFrame(nowSeconds = performance.now() / 1000) {
        ensureGeneration();
        group.visible = liveGroup.visible = model.visible;
        group.position.fromArray(model.mapTransform.position); group.scale.fromArray(model.mapTransform.scale);
        liveGroup.position.fromArray(model.liveTransform.position); liveGroup.quaternion.fromArray(model.liveTransform.quaternion);
        const pointSize = model.pointSize * renderer.getPixelRatio(), mode = model.colorMode === 'intensity' ? 1 : 0;
        material.uniforms.uSize.value = liveMaterial.uniforms.uSize.value = pointSize;
        material.uniforms.uMode.value = liveMaterial.uniforms.uMode.value = mode;
        material.uniforms.uHmin.value = model.heightRange[0]; material.uniforms.uHmax.value = model.heightRange[1];
        liveMaterial.uniforms.uNow.value = nowSeconds; liveMaterial.uniforms.uTtl.value = model.live.ttl;
        const depthTest = !model.overTerrain;
        if (material.depthTest !== depthTest) {
            material.depthTest = material.depthWrite = depthTest; material.transparent = model.overTerrain; material.needsUpdate = true;
            liveMaterial.depthTest = depthTest; liveMaterial.needsUpdate = true;
            for (const r of mapped.values()) r.points.renderOrder = model.overTerrain ? 10 : 0;
            liveResource.points.renderOrder = model.overTerrain ? 11 : 1;
        }
        liveResource.geometry.setDrawRange(0, model.live.count);
    }
    function dispose() {
        clearHistory(); liveResource.geometry.dispose(); material.dispose(); liveMaterial.dispose();
        group.removeFromParent(); liveGroup.removeFromParent();
    }
    applyMapped(model.chunks.map(block => ({ block, start: 0, count: block.count })));
    updateFrame();
    return { applyMapped, applyLive, clearHistory, updateFrame, dispose };
}
