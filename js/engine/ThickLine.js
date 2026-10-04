/**
 * ThickLine.js - Screen-space thick lines for the 3D overlays
 *
 * WebGL draws GL_LINES one pixel wide whatever `linewidth` says, which is why
 * the trail, the mission route and the corridor edges used to be hairlines.
 * Here every segment is an instance of a small quad that the vertex shader
 * widens to a fixed width in pixels, with round caps so consecutive segments
 * join without notches. Same approach as three's LineSegments2 (an addon, not
 * part of the core build that is vendored).
 *
 * Points live in one Float32Array (x, y, z per point):
 *   'strip' — a polyline, segment i runs from point i to point i + 1
 *   'pairs' — independent segments, points 2i and 2i + 1
 * Optional RGBA per point (vertexColors) for fades along the line.
 *
 * `ghost` adds a second, fainter pass drawn only where the line is hidden
 * (depth test GREATER), so a path behind a ridge still reads as a path.
 * `dash` cuts the line into dashes measured in screen pixels, restarting at
 * every segment: the pattern stays the same from any distance.
 * Translucent passes use the stencil buffer to touch each pixel once: the round
 * caps of neighbouring segments overlap, and blending them twice would bead
 * the line at every joint.
 */

import { OVERLAY_LAYER } from './Layers.js';

// Stencil references: one per translucent pass, so passes never mask each other
let nextStencilRef = 1;

// Lines with edits waiting for upload. three uploads buffers while it walks
// the scene, before any onBeforeRender runs, so the dirty ranges are handed
// over from flushThickLines(), called once per frame ahead of render().
const pendingLines = new Set();

/** Hand every line's pending edits to three (call once per frame, before rendering). */
export function flushThickLines() {
    for (const line of pendingLines) line._flush();
    pendingLines.clear();
}
function takeStencilRef() {
    const ref = nextStencilRef;
    nextStencilRef = nextStencilRef >= 255 ? 1 : nextStencilRef + 1;
    return ref;
}

// Quad per segment: body between y = 0 (start) and y = 1 (end), plus a cap
// quad beyond each end (y = -1, y = 2) trimmed to a half disc in the fragment.
const QUAD_POSITIONS = [-1, 2, 0, 1, 2, 0, -1, 1, 0, 1, 1, 0, -1, 0, 0, 1, 0, 0, -1, -1, 0, 1, -1, 0];
const QUAD_UVS = [-1, 2, 1, 2, -1, 1, 1, 1, -1, -1, 1, -1, -1, -2, 1, -2];
const QUAD_INDEX = [0, 2, 1, 2, 3, 1, 2, 4, 3, 4, 5, 3, 4, 6, 5, 6, 7, 5];

const VERTEX_SHADER = `
uniform vec2 uResolution;
uniform float uWidth;
uniform float uDepthPull;
attribute vec3 instanceStart;
attribute vec3 instanceEnd;
#ifdef USE_LINE_COLORS
attribute vec4 instanceColorStart;
attribute vec4 instanceColorEnd;
varying vec4 vLineColor;
#endif
#ifdef USE_DASH
varying float vAlongW;
varying float vW;
#endif
varying vec2 vUv;

// Move the endpoint that lies behind the camera onto the near plane, so the
// projection of a segment reaching behind the eye does not flip.
void trimSegment(const in vec4 start, inout vec4 end) {
    float a = projectionMatrix[2][2];
    float b = projectionMatrix[3][2];
    float nearEstimate = -0.5 * b / a;
    float alpha = (nearEstimate - start.z) / (end.z - start.z);
    end.xyz = mix(start.xyz, end.xyz, alpha);
}

void main() {
#ifdef USE_LINE_COLORS
    vLineColor = (position.y < 0.5) ? instanceColorStart : instanceColorEnd;
#endif
    vUv = uv;
    float aspect = uResolution.x / uResolution.y;

    vec4 start = modelViewMatrix * vec4(instanceStart, 1.0);
    vec4 end = modelViewMatrix * vec4(instanceEnd, 1.0);
    if (projectionMatrix[2][3] == -1.0) {
        if (start.z < 0.0 && end.z >= 0.0) trimSegment(start, end);
        else if (end.z < 0.0 && start.z >= 0.0) trimSegment(end, start);
    }
    // Slide both ends a little toward the eye along their view rays: the
    // projected position is unchanged, but a line lying on the ground wins
    // the depth test against the ground it lies on.
    start.xyz *= 1.0 - uDepthPull;
    end.xyz *= 1.0 - uDepthPull;

    vec4 clipStart = projectionMatrix * start;
    vec4 clipEnd = projectionMatrix * end;
    vec2 ndcStart = clipStart.xy / clipStart.w;
    vec2 ndcEnd = clipEnd.xy / clipEnd.w;

    vec2 dir = ndcEnd - ndcStart;
    dir.x *= aspect;
    dir = length(dir) > 1e-9 ? normalize(dir) : vec2(1.0, 0.0);
    vec2 offset = vec2(dir.y, -dir.x);
    dir.x /= aspect;
    offset.x /= aspect;
    if (position.x < 0.0) offset *= -1.0;
    if (position.y < 0.0) offset += -dir;
    else if (position.y > 1.0) offset += dir;

    // uWidth px across: each side is half of it, and NDC spans 2 per viewport
    offset *= uWidth / uResolution.y;
    vec4 clip = (position.y < 0.5) ? clipStart : clipEnd;
#ifdef USE_DASH
    // Distance along the segment in device pixels, from its start. Varyings
    // are interpolated perspective-correct; carrying it times w, and w itself,
    // lets the fragment recover the linear screen-space value.
    float segPx = length((ndcEnd - ndcStart) * 0.5 * uResolution);
    float along = position.y < 0.0 ? -0.5 * uWidth : position.y > 1.0 ? segPx + 0.5 * uWidth : position.y * segPx;
    vAlongW = along * clip.w;
    vW = clip.w;
#endif
    offset *= clip.w;
    clip.xy += offset;
    gl_Position = clip;
}
`;

const FRAGMENT_SHADER = `
uniform vec3 uColor;
uniform float uOpacity;
#ifdef USE_LINE_COLORS
varying vec4 vLineColor;
#endif
#ifdef USE_DASH
uniform vec2 uDash;           // dash, gap in device pixels
varying float vAlongW;
varying float vW;
#endif
varying vec2 vUv;

void main() {
    // Round caps: outside the half disc beyond each end is not part of the line
    if (abs(vUv.y) > 1.0) {
        float a = vUv.x;
        float b = (vUv.y > 0.0) ? vUv.y - 1.0 : vUv.y + 1.0;
        if (a * a + b * b > 1.0) discard;
    }
#ifdef USE_DASH
    if (mod(vAlongW / vW, uDash.x + uDash.y) > uDash.x) discard;
#endif
    vec4 color = vec4(uColor, uOpacity);
#ifdef USE_LINE_COLORS
    color *= vLineColor;
#endif
    if (color.a <= 0.003) discard;
    gl_FragColor = color;
}
`;

const _size = new THREE.Vector2();

function makeMaterial({ color, width, opacity, vertexColors, dash, depthTest, depthFunc, translucent, depthPull }) {
    const material = new THREE.ShaderMaterial({
        uniforms: {
            uResolution: { value: new THREE.Vector2(1, 1) },
            uWidth: { value: width },
            uColor: { value: new THREE.Color(color) },
            uOpacity: { value: opacity },
            uDepthPull: { value: depthPull },
            uDash: { value: new THREE.Vector2(1, 0) }
        },
        vertexShader: VERTEX_SHADER,
        fragmentShader: FRAGMENT_SHADER,
        transparent: translucent,
        depthTest,
        depthWrite: !translucent,
        depthFunc
    });
    material.defines = {};
    if (vertexColors) material.defines.USE_LINE_COLORS = '';
    if (dash) material.defines.USE_DASH = '';
    material.userData.cssDash = dash || null;
    if (translucent) {
        material.stencilWrite = true;
        material.stencilRef = takeStencilRef();
        material.stencilFunc = THREE.NotEqualStencilFunc;
        material.stencilZPass = THREE.ReplaceStencilOp;
    }
    material.userData.cssWidth = width;
    return material;
}

// Keep the pixel width right across window resizes and pixel-ratio changes
function syncViewport(renderer, scene, camera, geometry, material) {
    renderer.getDrawingBufferSize(_size);
    material.uniforms.uResolution.value.copy(_size);
    material.uniforms.uWidth.value = material.userData.cssWidth * renderer.getPixelRatio();
    const dash = material.userData.cssDash;
    if (dash) material.uniforms.uDash.value.set(dash[0], dash[1]).multiplyScalar(renderer.getPixelRatio());
}

export class ThickLine {
    /**
     * @param {object} [opts]
     * @param {'strip'|'pairs'} [opts.mode='strip']
     * @param {number} [opts.color=0xffffff]
     * @param {number} [opts.width=3]        CSS pixels
     * @param {number} [opts.opacity=1]      below 1 the line is blended
     * @param {boolean} [opts.vertexColors]  RGBA per point, multiplied with color/opacity
     * @param {[number, number]} [opts.dash] dash and gap in CSS pixels (default: solid)
     * @param {boolean} [opts.depthTest=true]
     * @param {object|null} [opts.ghost]     { width, opacity } of the pass drawn where the line is hidden
     * @param {number} [opts.depthPull=0.002] fraction of the eye distance the line is pulled forward
     * @param {number} [opts.renderOrder=0]
     */
    constructor(opts = {}) {
        this.mode = opts.mode === 'pairs' ? 'pairs' : 'strip';
        this.vertexColors = !!opts.vertexColors;
        this.positions = null;
        this.colors = null;
        this.count = 0;       // points in use
        this.capacity = 0;    // points the arrays can hold
        this.visible = true;
        this._dirtyFrom = Infinity;   // point range edited since the last flush
        this._dirtyTo = 0;

        const opacity = opts.opacity ?? 1;
        const depthTest = opts.depthTest !== false;
        const depthPull = opts.depthPull ?? 0.002;
        const translucent = opacity < 1 || this.vertexColors;
        const dash = opts.dash || null;
        this.material = makeMaterial({
            color: opts.color ?? 0xffffff, width: opts.width ?? 3, opacity,
            vertexColors: this.vertexColors, dash, depthTest, depthFunc: THREE.LessEqualDepth,
            translucent, depthPull
        });
        this.ghostMaterial = opts.ghost ? makeMaterial({
            color: opts.color ?? 0xffffff, width: opts.ghost.width ?? 2, opacity: opts.ghost.opacity ?? 0.3,
            vertexColors: this.vertexColors, dash, depthTest: true, depthFunc: THREE.GreaterDepth,
            translucent: true, depthPull
        }) : null;

        this.geometry = this._createGeometry(16);
        this.mesh = this._createMesh(this.material, opts.renderOrder ?? 0);
        this.ghost = this.ghostMaterial ? this._createMesh(this.ghostMaterial, (opts.renderOrder ?? 0) - 1) : null;
    }

    _createMesh(material, renderOrder) {
        const mesh = new THREE.Mesh(this.geometry, material);
        mesh.frustumCulled = false;
        mesh.matrixAutoUpdate = false;
        mesh.renderOrder = renderOrder;
        mesh.onBeforeRender = syncViewport;
        mesh.layers.set(OVERLAY_LAYER);
        return mesh;
    }

    _createGeometry(capacity) {
        this.capacity = capacity;
        const positions = new Float32Array(capacity * 3);
        const colors = this.vertexColors ? new Float32Array(capacity * 4) : null;
        if (this.positions) positions.set(this.positions.subarray(0, Math.min(this.positions.length, positions.length)));
        if (colors && this.colors) colors.set(this.colors.subarray(0, Math.min(this.colors.length, colors.length)));
        this.positions = positions;
        this.colors = colors;

        const geometry = new THREE.InstancedBufferGeometry();
        geometry.setIndex(QUAD_INDEX);
        geometry.setAttribute('position', new THREE.Float32BufferAttribute(QUAD_POSITIONS, 3));
        geometry.setAttribute('uv', new THREE.Float32BufferAttribute(QUAD_UVS, 2));

        // Strip: instance i reads points i and i + 1 (stride 3, offsets 0 / 3).
        // Pairs: instance i reads points 2i and 2i + 1 (stride 6).
        const stride = this.mode === 'strip' ? 3 : 6;
        const posBuffer = new THREE.InstancedInterleavedBuffer(positions, stride, 1);
        posBuffer.setUsage(THREE.DynamicDrawUsage);
        geometry.setAttribute('instanceStart', new THREE.InterleavedBufferAttribute(posBuffer, 3, 0));
        geometry.setAttribute('instanceEnd', new THREE.InterleavedBufferAttribute(posBuffer, 3, 3));
        this.posBuffer = posBuffer;

        if (colors) {
            const cStride = this.mode === 'strip' ? 4 : 8;
            const colBuffer = new THREE.InstancedInterleavedBuffer(colors, cStride, 1);
            colBuffer.setUsage(THREE.DynamicDrawUsage);
            geometry.setAttribute('instanceColorStart', new THREE.InterleavedBufferAttribute(colBuffer, 4, 0));
            geometry.setAttribute('instanceColorEnd', new THREE.InterleavedBufferAttribute(colBuffer, 4, 4));
            this.colBuffer = colBuffer;
        }
        geometry.instanceCount = 0;
        return geometry;
    }

    /** Make room for `points` points, keeping the current ones. */
    reserve(points) {
        if (points <= this.capacity) return;
        let cap = Math.max(16, this.capacity);
        while (cap < points) cap *= 2;
        const old = this.geometry;
        this.geometry = this._createGeometry(cap);
        this.mesh.geometry = this.geometry;
        if (this.ghost) this.ghost.geometry = this.geometry;
        old.dispose();
        // A new buffer is uploaded whole the first time three sees it
        this._dirtyFrom = Infinity;
        this._dirtyTo = 0;
        this.geometry.instanceCount = this._segmentCount();
    }

    _segmentCount() {
        if (this.mode === 'strip') return Math.max(0, this.count - 1);
        return Math.floor(this.count / 2);
    }

    _markDirty(from, to) {
        if (from < this._dirtyFrom) this._dirtyFrom = from;
        if (to > this._dirtyTo) this._dirtyTo = to;
        this.geometry.instanceCount = this._segmentCount();
        pendingLines.add(this);
    }

    _flush() {
        if (this._dirtyFrom >= this._dirtyTo) return;
        const from = this._dirtyFrom, to = Math.min(this._dirtyTo, this.capacity);
        this.posBuffer.addUpdateRange(from * 3, (to - from) * 3);
        this.posBuffer.needsUpdate = true;
        if (this.colBuffer) {
            this.colBuffer.addUpdateRange(from * 4, (to - from) * 4);
            this.colBuffer.needsUpdate = true;
        }
        this._dirtyFrom = Infinity;
        this._dirtyTo = 0;
    }

    /**
     * Replace every point.
     * @param {ArrayLike<number>|Array<{x:number,y:number,z:number}>} points flat xyz, or objects
     * @param {number} [count] points to use (flat arrays only; default: all)
     */
    setPoints(points, count) {
        const isObjects = points.length > 0 && typeof points[0] === 'object';
        const n = isObjects ? points.length : (count ?? Math.floor(points.length / 3));
        this.reserve(n);
        const pos = this.positions;
        if (isObjects) {
            for (let i = 0; i < n; i++) {
                const p = points[i];
                pos[i * 3] = p.x; pos[i * 3 + 1] = p.y; pos[i * 3 + 2] = p.z;
            }
        } else if (ArrayBuffer.isView(points)) {
            pos.set(points.subarray(0, n * 3));
        } else {
            for (let i = 0; i < n * 3; i++) pos[i] = points[i];
        }
        this.count = n;
        this._markDirty(0, n);
    }

    /**
     * RGBA per point (0..1), multiplied with the line colour and opacity.
     * @param {ArrayLike<number>} rgba four values per point, for the points in use
     */
    setColors(rgba) {
        if (!this.colors) return;
        const n = Math.min(this.capacity * 4, rgba.length);
        for (let i = 0; i < n; i++) this.colors[i] = rgba[i];
        this._markDirty(0, Math.ceil(n / 4));
    }

    /** Append one point (strip mode); only the new values are uploaded. */
    push(x, y, z) {
        this.reserve(this.count + 1);
        const o = this.count * 3;
        this.positions[o] = x; this.positions[o + 1] = y; this.positions[o + 2] = z;
        this.count++;
        this._markDirty(this.count - 1, this.count);
    }

    /** Points in use are now the first `count` of the array (after an in-place edit). */
    setCount(count) {
        this.count = Math.max(0, Math.min(count, this.capacity));
        this._markDirty(0, this.count);
    }

    /** Line width in CSS pixels (the ghost pass keeps its own). */
    setWidth(width) {
        this.material.userData.cssWidth = width;
    }

    clear() {
        this.count = 0;
        this.geometry.instanceCount = 0;
    }

    setVisible(visible) {
        this.visible = !!visible;
        this.mesh.visible = this.visible;
        if (this.ghost) this.ghost.visible = this.visible;
    }

    setColor(color) {
        this.material.uniforms.uColor.value.set(color);
        if (this.ghostMaterial) this.ghostMaterial.uniforms.uColor.value.set(color);
    }

    addTo(scene) {
        scene.add(this.mesh);
        if (this.ghost) scene.add(this.ghost);
        return this;
    }

    removeFrom(scene) {
        scene.remove(this.mesh);
        if (this.ghost) scene.remove(this.ghost);
    }

    dispose() {
        pendingLines.delete(this);
        this.geometry.dispose();
        this.material.dispose();
        if (this.ghostMaterial) this.ghostMaterial.dispose();
    }
}
