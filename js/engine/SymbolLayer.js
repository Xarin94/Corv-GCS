/**
 * SymbolLayer.js - Map symbols and labels at a readable size on screen
 *
 * Waypoints and home used to be meshes of a few metres (a 1.2 m sphere per
 * waypoint), invisible past a few hundred metres. Here they are drawn like
 * symbols on a chart: a point sprite whose shape is computed in the fragment
 * shader, sized in pixels — larger when close, never smaller than a floor far
 * away — with a halo so the stroke reads over satellite imagery as well as
 * over the schematic terrain: dark by default, light over the light-theme
 * schematic view (setSymbolHalo). A second pass draws the symbols hidden
 * behind the terrain at reduced opacity.
 *
 * Labels are sprites with a canvas texture, kept at a fixed pixel height.
 */

import { OVERLAY_LAYER } from './Layers.js';

export const SHAPE = { SQUARE: 0, DIAMOND: 1, CIRCLE: 2, HOME: 3, DOT: 4, TRIANGLE: 5, TARGET: 6 };

// One halo colour for every symbol layer (the uniform object is shared)
const haloUniform = { value: new THREE.Vector4(0, 0, 0, 0.6) };

/**
 * Halo drawn around every symbol stroke: dark over imagery and black ground,
 * light over light ground.
 * @param {number} r @param {number} g @param {number} b 0..1
 * @param {number} a opacity
 */
export function setSymbolHalo(r, g, b, a) {
    haloUniform.value.set(r, g, b, a);
}

const VERTEX_SHADER = `
uniform float uPixelRatio;
uniform vec4 uSize;          // px near, px far, near distance, far distance
uniform float uDepthPull;
attribute vec3 aColor;
attribute float aShape;
attribute float aScale;
attribute float aFill;
varying vec3 vColor;
varying float vShape;
varying float vFill;
varying float vSizePx;
void main() {
    vec4 mv = modelViewMatrix * vec4(position, 1.0);
    float t = clamp((-mv.z - uSize.z) / (uSize.w - uSize.z), 0.0, 1.0);
    vSizePx = mix(uSize.x, uSize.y, t) * aScale;
    gl_PointSize = vSizePx * uPixelRatio;
    mv.xyz *= 1.0 - uDepthPull;
    gl_Position = projectionMatrix * mv;
    vColor = aColor;
    vShape = aShape;
    vFill = aFill;
}
`;

const FRAGMENT_SHADER = `
uniform float uOpacity;
uniform vec4 uHalo;
varying vec3 vColor;
varying float vShape;
varying float vFill;
varying float vSizePx;

float sdBox(vec2 p, vec2 b) {
    vec2 d = abs(p) - b;
    return length(max(d, 0.0)) + min(max(d.x, d.y), 0.0);
}

vec4 over(vec4 top, vec4 bottom) {
    float a = top.a + bottom.a * (1.0 - top.a);
    vec3 c = (top.rgb * top.a + bottom.rgb * bottom.a * (1.0 - top.a)) / max(a, 1e-5);
    return vec4(c, a);
}

void main() {
    // Sprite space: -1..1 across the point, y up
    vec2 p = gl_PointCoord * 2.0 - 1.0;
    p.y = -p.y;
    float px = 2.0 / vSizePx;              // one screen pixel in sprite units
    float stroke = 1.3 * px;               // half stroke width (2.6 px lines)
    float halo = 1.6 * px;                 // contrasting rim around the stroke
    int shape = int(vShape + 0.5);

    float d;          // signed distance to the outline (negative inside)
    float mark = 1e3; // extra strokes drawn inside the outline (the H of home)
    if (shape == 0) {
        d = sdBox(p, vec2(0.58));
    } else if (shape == 1) {
        d = (abs(p.x) + abs(p.y) - 0.78) * 0.7071;
    } else if (shape == 2) {
        d = length(p) - 0.62;
    } else if (shape == 3) {
        d = length(p) - 0.70;
        float bars = min(sdBox(p - vec2(-0.26, 0.0), vec2(0.0, 0.34)), sdBox(p - vec2(0.26, 0.0), vec2(0.0, 0.34)));
        mark = min(bars, sdBox(p, vec2(0.26, 0.0)));
    } else if (shape == 4) {
        d = length(p) - 0.45;
    } else if (shape == 6) {
        // Target: a ring with four ticks, the planner's POI glyph
        d = length(p) - 0.32;
        mark = min(min(sdBox(p - vec2(0.0, 0.58), vec2(0.0, 0.12)), sdBox(p + vec2(0.0, 0.58), vec2(0.0, 0.12))),
                   min(sdBox(p - vec2(0.58, 0.0), vec2(0.12, 0.0)), sdBox(p + vec2(0.58, 0.0), vec2(0.12, 0.0))));
    } else {
        // Triangle, apex up
        vec2 q = vec2(abs(p.x), p.y + 0.18);
        d = max(q.x * 0.866 + q.y * 0.5, -q.y) - 0.42;
    }

    float edge = abs(d);
    if (shape == 4) edge = d;              // the dot is solid
    edge = min(edge, abs(mark));
    float strokeA = 1.0 - smoothstep(stroke - px * 0.5, stroke + px * 0.5, edge);
    float haloA = 1.0 - smoothstep(stroke + halo - px * 0.5, stroke + halo + px * 0.5, edge);
    float inside = 1.0 - smoothstep(-px * 0.5, px * 0.5, d);

    vec4 col = vec4(uHalo.rgb, haloA * uHalo.a);
    col = over(vec4(vColor, inside * vFill * 0.35), col);
    col = over(vec4(vColor, strokeA), col);
    col.a *= uOpacity;
    if (col.a < 0.01) discard;
    gl_FragColor = col;
}
`;

function makeMaterial(sizes, opacity, depthFunc, depthPull) {
    return new THREE.ShaderMaterial({
        uniforms: {
            uPixelRatio: { value: 1 },
            uSize: { value: new THREE.Vector4(sizes.near, sizes.far, sizes.nearDist, sizes.farDist) },
            uOpacity: { value: opacity },
            uDepthPull: { value: depthPull },
            uHalo: haloUniform
        },
        vertexShader: VERTEX_SHADER,
        fragmentShader: FRAGMENT_SHADER,
        transparent: true,
        depthWrite: false,
        depthTest: true,
        depthFunc
    });
}

function syncPixelRatio(renderer, scene, camera, geometry, material) {
    material.uniforms.uPixelRatio.value = renderer.getPixelRatio();
}

export class SymbolLayer {
    /**
     * @param {object} [opts]
     * @param {number} [opts.sizeNear=30]     px at or under nearDist
     * @param {number} [opts.sizeFar=18]      px at or beyond farDist
     * @param {number} [opts.nearDist=300]    m
     * @param {number} [opts.farDist=6000]    m
     * @param {number} [opts.ghostOpacity=0.4] opacity of the symbols hidden by terrain (0 = none)
     * @param {number} [opts.renderOrder=10]
     */
    constructor(opts = {}) {
        const sizes = {
            near: opts.sizeNear ?? 30, far: opts.sizeFar ?? 18,
            nearDist: opts.nearDist ?? 300, farDist: opts.farDist ?? 6000
        };
        this.capacity = 0;
        this.count = 0;
        this.geometry = null;
        this.material = makeMaterial(sizes, 1, THREE.LessEqualDepth, 0.002);
        this.ghostMaterial = (opts.ghostOpacity ?? 0.4) > 0
            ? makeMaterial(sizes, opts.ghostOpacity ?? 0.4, THREE.GreaterDepth, 0.002) : null;
        this._allocate(16);

        const renderOrder = opts.renderOrder ?? 10;
        this.points = this._makePoints(this.material, renderOrder);
        this.ghost = this.ghostMaterial ? this._makePoints(this.ghostMaterial, renderOrder - 1) : null;
    }

    _makePoints(material, renderOrder) {
        const pts = new THREE.Points(this.geometry, material);
        pts.frustumCulled = false;
        pts.matrixAutoUpdate = false;
        pts.renderOrder = renderOrder;
        pts.onBeforeRender = syncPixelRatio;
        pts.layers.set(OVERLAY_LAYER);
        return pts;
    }

    _allocate(capacity) {
        if (this.geometry) this.geometry.dispose();
        this.capacity = capacity;
        const g = new THREE.BufferGeometry();
        g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(capacity * 3), 3));
        g.setAttribute('aColor', new THREE.BufferAttribute(new Float32Array(capacity * 3), 3));
        g.setAttribute('aShape', new THREE.BufferAttribute(new Float32Array(capacity), 1));
        g.setAttribute('aScale', new THREE.BufferAttribute(new Float32Array(capacity), 1));
        g.setAttribute('aFill', new THREE.BufferAttribute(new Float32Array(capacity), 1));
        g.setDrawRange(0, 0);
        this.geometry = g;
        if (this.points) this.points.geometry = g;
        if (this.ghost) this.ghost.geometry = g;
    }

    /**
     * Replace every symbol.
     * @param {Array<{x:number,y:number,z:number,shape:number,color:number,scale?:number,fill?:boolean}>} list
     */
    setSymbols(list) {
        if (list.length > this.capacity) {
            let cap = this.capacity;
            while (cap < list.length) cap *= 2;
            this._allocate(cap);
        }
        const a = this.geometry.attributes;
        const c = new THREE.Color();
        for (let i = 0; i < list.length; i++) {
            const s = list[i];
            a.position.array[i * 3] = s.x;
            a.position.array[i * 3 + 1] = s.y;
            a.position.array[i * 3 + 2] = s.z;
            c.set(s.color);
            a.aColor.array[i * 3] = c.r;
            a.aColor.array[i * 3 + 1] = c.g;
            a.aColor.array[i * 3 + 2] = c.b;
            a.aShape.array[i] = s.shape;
            a.aScale.array[i] = s.scale ?? 1;
            a.aFill.array[i] = s.fill ? 1 : 0;
        }
        for (const name in a) a[name].needsUpdate = true;
        this.count = list.length;
        this.geometry.setDrawRange(0, list.length);
    }

    /** Move one symbol (cheap enough to call every frame). */
    setPosition(index, x, y, z) {
        if (index < 0 || index >= this.count) return;
        const p = this.geometry.attributes.position;
        p.array[index * 3] = x;
        p.array[index * 3 + 1] = y;
        p.array[index * 3 + 2] = z;
        p.needsUpdate = true;
    }

    /** Change one symbol's scale / fill in place (e.g. the active waypoint). */
    setEmphasis(index, scale, fill) {
        if (index < 0 || index >= this.count) return;
        const a = this.geometry.attributes;
        a.aScale.array[index] = scale;
        a.aFill.array[index] = fill ? 1 : 0;
        a.aScale.needsUpdate = true;
        a.aFill.needsUpdate = true;
    }

    setVisible(visible) {
        this.points.visible = !!visible;
        if (this.ghost) this.ghost.visible = !!visible;
    }

    addTo(scene) {
        scene.add(this.points);
        if (this.ghost) scene.add(this.ghost);
        return this;
    }
}

// ---- Labels -------------------------------------------------------------------

const LABEL_FONT_PX = 15;       // CSS px of the main text
const LABEL_SUPERSAMPLE = 2;    // canvas pixels per CSS pixel
const LABEL_FONT = '"Rajdhani", "Roboto Mono", sans-serif';
let labelHalo = 'rgba(0, 0, 0, 0.85)';
let labelHaloPx = 3.5;

/**
 * Outline of label text drawn from now on (existing labels keep theirs).
 * @param {string} css colour
 * @param {number} [widthPx=3.5] stroke width in CSS pixels
 */
export function setLabelHalo(css, widthPx = 3.5) {
    labelHalo = css;
    labelHaloPx = widthPx;
}

/**
 * A text label kept at a fixed pixel height, placed beside its anchor.
 * @param {string} text   bold main text
 * @param {string} [sub]  smaller trailing text (e.g. altitude)
 * @param {number} color  hex
 * @returns {THREE.Sprite}
 */
export function makeLabel(text, sub, color) {
    const s = LABEL_SUPERSAMPLE;
    const main = `600 ${LABEL_FONT_PX * s}px ${LABEL_FONT}`;
    const small = `500 ${Math.round(LABEL_FONT_PX * 0.8 * s)}px ${LABEL_FONT}`;
    const measure = document.createElement('canvas').getContext('2d');
    measure.font = main;
    const wMain = measure.measureText(text).width;
    measure.font = small;
    const gap = sub ? 5 * s : 0;
    const wSub = sub ? measure.measureText(sub).width : 0;
    const pad = 4 * s;
    const w = Math.ceil(wMain + gap + wSub + pad * 2);
    const h = Math.ceil(LABEL_FONT_PX * 1.5 * s);

    const canvas = document.createElement('canvas');
    canvas.width = w;
    canvas.height = h;
    const ctx = canvas.getContext('2d');
    const css = '#' + new THREE.Color(color).getHexString();
    ctx.textBaseline = 'middle';
    ctx.lineJoin = 'round';
    ctx.lineWidth = labelHaloPx * s;
    ctx.strokeStyle = labelHalo;
    ctx.fillStyle = css;
    ctx.font = main;
    ctx.strokeText(text, pad, h / 2);
    ctx.fillText(text, pad, h / 2);
    if (sub) {
        ctx.font = small;
        ctx.globalAlpha = 0.85;
        ctx.strokeText(sub, pad + wMain + gap, h / 2 + 1 * s);
        ctx.fillText(sub, pad + wMain + gap, h / 2 + 1 * s);
    }

    const texture = new THREE.CanvasTexture(canvas);
    texture.minFilter = THREE.LinearFilter;
    texture.generateMipmaps = false;
    // No fog: a label is read at any distance (three's materials default to fog on)
    const material = new THREE.SpriteMaterial({
        map: texture, sizeAttenuation: false, depthTest: false, depthWrite: false, transparent: true, fog: false
    });
    const sprite = new THREE.Sprite(material);
    sprite.renderOrder = 20;
    sprite.frustumCulled = false;
    sprite.layers.set(OVERLAY_LAYER);
    sprite.userData.labelPx = { w: w / s, h: h / s };
    return sprite;
}

/**
 * Size a label sprite to its pixel dimensions for the current viewport and
 * place it `offsetPx` to the right of (and `liftPx` above) its anchor.
 * With sizeAttenuation off, a sprite of scale 1 is 2 / P[1][1] viewport
 * heights tall, whatever its distance.
 */
export function fitLabel(sprite, camera, viewportHeightCss, offsetPx = 18, liftPx = 0) {
    const px = sprite.userData.labelPx;
    if (!px || !camera || !(viewportHeightCss > 0)) return;
    const unit = 2 / (camera.projectionMatrix.elements[5] * viewportHeightCss);
    sprite.scale.set(px.w * unit, px.h * unit, 1);
    // center is the anchor in sprite UV space: shift the label right and up
    sprite.center.set(-offsetPx / px.w, 0.5 - liftPx / px.h);
}

export function disposeLabel(sprite) {
    if (!sprite) return;
    if (sprite.parent) sprite.parent.remove(sprite);
    sprite.material.map?.dispose();
    sprite.material.dispose();
}
