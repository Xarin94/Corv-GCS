/**
 * RosMesh3D.js - The averaged ROS surface in the 3D view (height field)
 *
 * Draws the cells RosWorker averages (30 cm by default, in tiles of 30 × 30
 * cells) as a triangle mesh with grey lines on the edges and a transparent
 * body, so terrain, imagery and the vehicle stay visible through it: terrain
 * under an aircraft, a sea bed under a boat. A cave's 3D surface is drawn by
 * RosVolume3D.js with the same uniforms and look. Colour can switch to a red →
 * blue ramp by distance from the vehicle: red is the nearest surface, blue is
 * NEAR + max(20 m, 2 × NEAR) away; or to ROUGHNESS: grey where the surface is
 * flat (level or sloping), redder the further it departs from its local plane
 * (SurfaceRaster.roughness: RMS about the plane fitted within 1.5 m; full red
 * at ROUGH_FULL), so rocks, a wreck, a scarp or a bad average stand out of a
 * flat bed.
 *
 * Adaptive resolution (SurfaceRaster.js). The cell size is the finest the
 * mesh gets: each cell is drawn at the finest of 30 cm, 60 cm, 1.2, 2.4, 4.8 m
 * with MIN SAMPLES there, and the lines at that level's spacing, so a single
 * return is never a vertex on its own. Holes up to 1.2 m are interpolated,
 * edges are not extended, and nothing is averaged or triangulated across a
 * height jump (a cliff, a wall): those stay open.
 *
 * Cost. At 30 cm a surveyed hectare is 110 000 cells, so nothing here is per
 * cell on the CPU, per frame:
 *   - Tiles are stored in blocks of 8 × 8 tiles (240 × 240 cells), one RG32F
 *     texture each (height, level), plus the first row and column of the next
 *     blocks so the blocks join.
 *   - The vertex shader builds each vertex from gl_VertexID and the texture:
 *     there is no vertex buffer, only an index buffer per block listing the
 *     triangles whose three corners have a height.
 *   - Level of detail: far blocks are drawn on every 2nd … 16th vertex, so a
 *     triangle never shrinks below ~6 px. The levels, the texture and the index
 *     are rebuilt when a block's data or detail changes, in view only, within a
 *     per-frame budget.
 *   - The lines are not geometry: the fragment shader draws the triangle edges
 *     from the cell coordinates (anti-aliased, ~1.5 px) and fades them out
 *     where cells get smaller than a few pixels; tile edges, every 30 cells,
 *     stay visible further away.
 *   - Blocks outside the view frustum are skipped.
 *
 * Over the terrain (default) the mesh is drawn through the terrain mesh: SRTM
 * is ±5 m and a lake's surface covers the sea bed. Otherwise it is depth-tested,
 * with a fainter pass where the terrain hides it.
 *
 * Heights are metres in the vehicle's altitude frame (STATE.rawAlt, MSL or the
 * relative frame) from the worker's anchor, so the mesh takes the same altitude
 * offset as the vehicle model; the anchor's metric frame is scaled to the
 * scene's flat-earth convention like the LiDAR cloud (LidarCloud.js).
 */

import { OVERLAY_LAYER } from '../engine/Layers.js';
import { getCamera, getRenderer } from '../engine/Scene3D.js';
import { STATE } from '../core/state.js';
import { ORIGIN } from '../core/constants.js';
import { latLonToMeters } from '../core/utils.js';
import { TILE, tileKey } from './SurfaceTiles.js';
import { EMPTY, LEVELS, rasterize, roughness, triangulate } from './SurfaceRaster.js';

const BLOCK_TILES = 8;
const BLOCK = BLOCK_TILES * TILE;          // cells per block side
const TEX = BLOCK + 1;                     // + the next blocks' first row / column
const STEPS = [1, 2, 4, 8, 16];            // vertex spacing per level of detail, cells
const MIN_TRI_PX = 6;
const REBUILD_QUADS_PER_FRAME = 70000;     // rebuild budget
const RANGE_MS = 250;
const GHOST_ALPHA = 0.35;
const ROUGH_FULL = 0.5;                    // m RMS about the local plane: full red
const MODES = { gray: 0, distance: 1, rough: 2 };

const VERTEX = `
uniform highp sampler2D uHeights;
uniform sampler2D uRough;
uniform float uCell;
uniform int uStep;
uniform int uW;
uniform vec3 uVehicle;
varying vec2 vCell;
varying float vDist;
varying float vLevel;
varying float vRough;
void main() {
    ivec2 g = ivec2(gl_VertexID % uW, gl_VertexID / uW) * uStep;
    vec2 hl = texelFetch(uHeights, g, 0).rg;     // height, resolution level
    vLevel = hl.g;
    vRough = texelFetch(uRough, g, 0).r * 2.55;  // m (stored in cm)
    vec4 local = vec4((float(g.x) + 0.5) * uCell, hl.r, -(float(g.y) + 0.5) * uCell, 1.0);
    vCell = vec2(g);
    vDist = distance((modelMatrix * local).xyz, uVehicle);
    gl_Position = projectionMatrix * modelViewMatrix * local;
}`;

// Distance colour ramp, red (near) → violet → blue (far); roughness: the grey
// of the lines (flat) → amber → red (irregular). Also RosVolume3D's
export const ROS_RAMP_GLSL = `
vec3 ramp(float t) {
    const vec3 R = vec3(1.0, 0.22, 0.16), M = vec3(0.72, 0.30, 0.88), B = vec3(0.18, 0.45, 1.0);
    return t < 0.5 ? mix(R, M, t * 2.0) : mix(M, B, t * 2.0 - 1.0);
}
vec3 roughRamp(vec3 base, float t) {
    const vec3 A = vec3(1.0, 0.68, 0.18), R = vec3(1.0, 0.16, 0.12);
    return t < 0.5 ? mix(base, A, t * 2.0) : mix(A, R, t * 2.0 - 1.0);
}`;

const FRAGMENT = `
uniform vec3 uColor;
uniform float uFill;
uniform float uAlpha;
uniform float uMode;
uniform float uNear;
uniform float uFar;
uniform float uRoughFull;
varying vec2 vCell;
varying float vDist;
varying float vLevel;
varying float vRough;
${ROS_RAMP_GLSL}
// Triangle edges of a lattice: along x, y and the x = y diagonal; faded out
// where its cells are smaller than a few pixels
float lattice(vec2 c) {
    vec3 g = vec3(c.x, c.y, c.x - c.y);
    vec3 fw = max(fwidth(g), vec3(1e-6));
    vec3 l = 1.0 - smoothstep(vec3(0.5), vec3(1.3), abs(fract(g + 0.5) - 0.5) / fw);
    return max(l.x, max(l.y, l.z)) * smoothstep(3.0, 6.0, 1.0 / max(fw.x, fw.y));
}
// The mesh of level k: vertices on the centres of 2^k × 2^k cell groups
float levelLines(float k) {
    float s = exp2(k);
    return lattice((vCell - 0.5 * (s - 1.0)) / s);
}
void main() {
    float k0 = floor(vLevel + 1e-3);
    float minor = mix(levelLines(k0), levelLines(k0 + 1.0), clamp(vLevel - k0, 0.0, 1.0));
    // Tile edges every 30 cells, readable from further away
    vec2 t = vCell / 30.0;
    vec2 ft = max(fwidth(t), vec2(1e-6));
    vec2 lt = 1.0 - smoothstep(vec2(0.6), vec2(1.6), abs(fract(t + 0.5) - 0.5) / ft);
    float major = max(lt.x, lt.y) * smoothstep(3.0, 6.0, 1.0 / max(ft.x, ft.y));
    vec3 col = uColor;
    float fill = uFill;
    if (uMode > 1.5) {
        float t = clamp(vRough / uRoughFull, 0.0, 1.0);
        col = roughRamp(uColor, t);
        fill *= 1.0 + 3.5 * t;
    } else if (uMode > 0.5) {
        col = ramp(clamp((vDist - uNear) / max(uFar - uNear, 1.0), 0.0, 1.0));
        fill *= 2.5;
    }
    float a = max(minor * 0.85, major * 0.6) + fill;
    gl_FragColor = vec4(col, min(a, 1.0) * uAlpha);
}`;

// ============== SHARED STATE ==============
// Also read by RosVolume3D.js: one look, one set of switches for both surfaces
const shared = {
    uCell: { value: 0.3 },
    uColor: { value: new THREE.Color(0xa6a6a6) },
    uFill: { value: 0.1 },
    uMode: { value: 0 },
    uVehicle: { value: new THREE.Vector3() },
    uNear: { value: 0 },
    uFar: { value: 20 },
    uRoughFull: { value: ROUGH_FULL }
};
const view = { visible: false, shown: true, overTerrain: true, minSamples: 1, range: { near: 0, far: 20 }, lastRange: 0 };
export { shared as rosShared, view as rosView };
const scratchIndex = new Uint16Array(BLOCK * BLOCK * 6);
const frustum = new THREE.Frustum();
const projView = new THREE.Matrix4();
const corner = new THREE.Vector3();
const tileBox = new THREE.Box3();
let root = null;
const layers = {};

// ============== A LAYER ==============
class SurfaceLayer {
    constructor(name) {
        this.name = name;
        this.axes = [0, 1, 2];                 // the plane is east, north; heights up
        this.permute = new THREE.Matrix4();
        this.group = new THREE.Group();
        this.group.name = `rosSurface-${name}`;
        root.add(this.group);
        this.anchor = null;
        this.cell = 0.3;
        this.tiles = new Map();                // tileKey → { tx, ty, h, w, minH, maxH }
        this.blocks = new Map();               // blockKey → block
        this.offset = 0;
    }

    apply(m) {
        if (m.reset || !m.anchor || !this.anchor || m.anchor.epoch !== this.anchor.epoch || m.cell !== this.cell) this.clear();
        this.anchor = m.anchor;
        this.cell = m.cell;
        shared.uCell.value = m.cell;
        if (!this.anchor) return;
        for (const [tx, ty] of m.removed || []) this.removeTile(tx, ty);
        for (const t of m.tiles || []) {
            let minH = Infinity, maxH = -Infinity;
            for (const v of t.h) if (!Number.isNaN(v)) { if (v < minH) minH = v; if (v > maxH) maxH = v; }
            const key = tileKey(t.tx, t.ty);
            this.tiles.set(key, { tx: t.tx, ty: t.ty, h: t.h, w: t.w, minH, maxH });
            const bx = Math.floor(t.tx / BLOCK_TILES), by = Math.floor(t.ty / BLOCK_TILES);
            const b = this.blocks.get(tileKey(bx, by)) || this.createBlock(bx, by);
            b.tiles.add(key);
            this.writeTile(t.tx, t.ty, t);
        }
    }

    removeTile(tx, ty) {
        const key = tileKey(tx, ty);
        if (!this.tiles.delete(key)) return;
        this.writeTile(tx, ty, null);
        const bk = tileKey(Math.floor(tx / BLOCK_TILES), Math.floor(ty / BLOCK_TILES));
        const b = this.blocks.get(bk);
        if (b) { b.tiles.delete(key); if (!b.tiles.size) this.disposeBlock(bk, b); }
    }

    // Copy a tile's heights and weights (null: empty) into its block and into
    // the aprons of the blocks west / south that end on its first column / row
    writeTile(tx, ty, t) {
        const gi0 = tx * TILE, gj0 = ty * TILE;
        const bx = Math.floor(gi0 / BLOCK), by = Math.floor(gj0 / BLOCK);
        for (const [ox, oy] of [[0, 0], [-1, 0], [0, -1], [-1, -1]]) {
            const b = this.blocks.get(tileKey(bx + ox, by + oy));
            if (!b) continue;
            const li0 = gi0 - (bx + ox) * BLOCK, lj0 = gj0 - (by + oy) * BLOCK;
            const i1 = Math.min(TILE, TEX - li0), j1 = Math.min(TILE, TEX - lj0);
            if (i1 <= 0 || j1 <= 0) continue;
            for (let j = 0; j < j1; j++) {
                const row = (lj0 + j) * TEX + li0;
                for (let i = 0; i < i1; i++) {
                    const v = t ? t.h[j * TILE + i] : NaN;
                    const ok = !Number.isNaN(v);
                    b.data[row + i] = ok ? v : EMPTY;
                    b.wts[row + i] = ok ? (t.w ? t.w[j * TILE + i] : 1) : 0;
                }
            }
            b.dirty = true;
        }
    }

    createBlock(bx, by) {
        const data = new Float32Array(TEX * TEX).fill(EMPTY);      // measured heights
        const wts = new Float32Array(TEX * TEX);                   // their samples
        const texData = new Float32Array(TEX * TEX * 2);           // drawn: height, level
        const tex = new THREE.DataTexture(texData, TEX, TEX, THREE.RGFormat, THREE.FloatType);
        tex.internalFormat = 'RG32F';
        const rough = new Uint8Array(TEX * TEX);                   // roughness, cm (ROUGH mode only)
        const roughTex = new THREE.DataTexture(rough, TEX, TEX, THREE.RedFormat, THREE.UnsignedByteType);
        roughTex.internalFormat = 'R8';
        roughTex.unpackAlignment = 1;                              // rows of 241 bytes
        const uniforms = { ...shared, uHeights: { value: tex }, uRough: { value: roughTex }, uStep: { value: 1 }, uW: { value: TEX }, uAlpha: { value: 1 } };
        const common = { vertexShader: VERTEX, fragmentShader: FRAGMENT, transparent: true, depthWrite: false, side: THREE.DoubleSide };
        const mat = new THREE.ShaderMaterial({ ...common, uniforms, depthTest: !view.overTerrain });
        const ghostMat = new THREE.ShaderMaterial({ ...common, uniforms: { ...uniforms, uAlpha: { value: GHOST_ALPHA } }, depthFunc: THREE.GreaterDepth });
        const b = {
            bx, by, data, wts, texData, tex, rough, roughTex, roughDone: false, mat, ghostMat, tiles: new Set(),
            geo: null, mesh: null, ghost: null, capacity: 0, triangles: 0,
            step: 0, dirty: true, box: new THREE.Box3(), levelCount: new Uint32Array(LEVELS + 1)
        };
        this.setGeometry(b, 1024);
        this.blocks.set(tileKey(bx, by), b);
        this.placeBlock(b);
        // The aprons: first column / row of the tiles east and north, and the corner
        for (const [ox, oy] of [[BLOCK_TILES, 0], [0, BLOCK_TILES], [BLOCK_TILES, BLOCK_TILES]]) {
            const n = ox && oy ? 1 : BLOCK_TILES;
            for (let k = 0; k < n; k++) {
                const tx = bx * BLOCK_TILES + (ox || k), ty = by * BLOCK_TILES + (oy || k);
                const t = this.tiles.get(tileKey(tx, ty));
                if (t) this.writeTile(tx, ty, t);
            }
        }
        return b;
    }

    // An index-only geometry (vertices come from gl_VertexID); replaced when it must grow
    setGeometry(b, capacity) {
        const geo = new THREE.BufferGeometry();
        const index = new THREE.BufferAttribute(new Uint16Array(capacity), 1);
        index.setUsage(THREE.DynamicDrawUsage);
        geo.setIndex(index);
        geo.setDrawRange(0, 0);
        if (b.geo) b.geo.dispose();
        b.geo = geo;
        b.capacity = capacity;
        if (!b.mesh) {
            b.mesh = new THREE.Mesh(geo, b.mat);
            b.ghost = new THREE.Mesh(geo, b.ghostMat);
            for (const [m, order] of [[b.mesh, 3], [b.ghost, 2]]) {
                m.frustumCulled = false;        // culled per block in update(); no vertex buffer to bound
                m.matrixAutoUpdate = false;     // placed by placeBlock()
                m.renderOrder = order;
                m.layers.set(OVERLAY_LAYER);
                m.visible = false;
                this.group.add(m);
            }
        } else {
            b.mesh.geometry = geo;
            b.ghost.geometry = geo;
        }
    }

    disposeBlock(key, b) {
        this.group.remove(b.mesh);
        this.group.remove(b.ghost);
        b.geo.dispose();
        b.mat.dispose();
        b.ghostMat.dispose();
        b.tex.dispose();
        b.roughTex.dispose();
        this.blocks.delete(key);
    }

    // Scene placement of block (bx, by): its cell (0, 0) corner in the surface's
    // plane → ENU from the anchor → scene, then the flat-earth scale and the
    // surface's rotation (a wall is the floor turned on its side)
    placeBlock(b) {
        const a = this.anchor;
        if (!a) return;
        const base = latLonToMeters(a.lat, a.lon);
        const kx = (111320 * Math.cos(ORIGIN.lat * Math.PI / 180)) / a.mPerLon;
        const kz = 111320 / a.mPerLat;
        const span = BLOCK * this.cell;
        const enu = [0, 0, 0];
        enu[this.axes[0]] = b.bx * span;
        enu[this.axes[1]] = b.by * span;
        b.matrix = b.matrix || new THREE.Matrix4();
        b.matrix.makeScale(kx, 1, kz).premultiply(new THREE.Matrix4().makeTranslation(
            base.x + enu[0] * kx, a.alt + this.offset + enu[2], base.z - enu[1] * kz)).multiply(this.permute);
        for (const m of [b.mesh, b.ghost]) { m.matrix.copy(b.matrix); m.matrixWorldNeedsUpdate = true; }
        let minH = Infinity, maxH = -Infinity;
        for (const key of b.tiles) {
            const t = this.tiles.get(key);
            if (t && t.minH < minH) minH = t.minH;
            if (t && t.maxH > maxH) maxH = t.maxH;
        }
        if (minH > maxH) { minH = 0; maxH = 0; }
        localBox(b, 0, 0, span + this.cell, span + this.cell, minH, maxH, b.box);
    }

    // The drawn heights (SurfaceRaster.rasterize: adaptive level per cell,
    // holes interpolated, nothing across a jump). Uploads.
    fillBlock(b) {
        b.levelCount = rasterize(b.data, b.wts, TEX, TEX, b.texData, { minSamples: view.minSamples, cell: this.cell });
        b.tex.needsUpdate = true;
        // Roughness only while it is shown; switching to it refills the blocks
        b.roughDone = shared.uMode.value === MODES.rough;
        if (b.roughDone) {
            roughness(b.texData, TEX, TEX, b.rough, { cell: this.cell });
            b.roughTex.needsUpdate = true;
        }
        b.dirty = false;
    }

    // Triangles of a block at vertex spacing s (SurfaceRaster.triangulate)
    buildIndex(b, s) {
        const W = BLOCK / s + 1;
        const n = triangulate(b.texData, TEX, TEX, s, scratchIndex, { cell: this.cell });
        const out = scratchIndex;
        if (n > b.capacity) {
            let cap = b.capacity;
            while (cap < n) cap *= 2;
            this.setGeometry(b, cap);
        }
        const index = b.geo.index;
        index.array.set(out.subarray(0, n));
        index.updateRange.offset = 0;
        index.updateRange.count = n;
        index.needsUpdate = true;
        b.geo.setDrawRange(0, n);
        b.triangles = n / 3;
        b.step = s;
        for (const m of [b.mat, b.ghostMat]) { m.uniforms.uStep.value = s; m.uniforms.uW.value = W; }
        return (W - 1) * (W - 1);
    }

    clear() {
        for (const [key, b] of this.blocks) this.disposeBlock(key, b);
        this.tiles.clear();
        this.anchor = null;
    }

    // Blocks in view, with the step their distance asks for; returns the work to do
    plan(camPos, focalPx, todo) {
        const offset = STATE.offsetAlt || 0;
        if (offset !== this.offset) {
            this.offset = offset;
            for (const b of this.blocks.values()) this.placeBlock(b);
        }
        for (const b of this.blocks.values()) {
            const inView = frustum.intersectsBox(b.box);
            if (inView) {
                const d = Math.max(1, b.box.distanceToPoint(camPos));
                let s = 1;
                for (const st of STEPS) { s = st; if (this.cell * st * focalPx / d >= MIN_TRI_PX) break; }
                if (b.dirty || s !== b.step) todo.push([d, this, b, s]);
            }
            const draw = inView && b.triangles > 0;
            b.mesh.visible = draw;
            b.ghost.visible = draw && !view.overTerrain;
        }
    }

    // Distance from v (scene) to the nearest tile's bounding box
    nearest(v) {
        if (!this.anchor) return Infinity;
        const span = TILE * this.cell;
        let near = Infinity;
        for (const t of this.tiles.values()) {
            if (!(t.maxH >= t.minH)) continue;
            const bx = Math.floor(t.tx / BLOCK_TILES), by = Math.floor(t.ty / BLOCK_TILES);
            const b = this.blocks.get(tileKey(bx, by));
            if (!b || !b.matrix) continue;
            const a0 = (t.tx - bx * BLOCK_TILES) * span, b0 = (t.ty - by * BLOCK_TILES) * span;
            localBox(b, a0, b0, a0 + span, b0 + span, t.minH, t.maxH, tileBox);
            near = Math.min(near, tileBox.distanceToPoint(v));
        }
        return near;
    }

    stats() {
        let triangles = 0, drawn = 0;
        const lv = new Array(LEVELS + 1).fill(0);
        for (const b of this.blocks.values()) {
            if (b.mesh.visible) { triangles += b.triangles; drawn++; }
            for (let k = 0; k <= LEVELS; k++) lv[k] += b.levelCount[k];
        }
        return { tiles: this.tiles.size, blocks: this.blocks.size, blocksDrawn: drawn, triangles, levels: lv };
    }
}

// Scene box of a block's local region: a from a0 to a1, b from b0 to b1 in the
// surface's plane (m from the block corner), heights h0…h1
function localBox(b, a0, b0, a1, b1, h0, h1, out) {
    out.makeEmpty();
    for (const x of [a0, a1]) for (const y of [h0, h1]) for (const z of [-b0, -b1]) {
        out.expandByPoint(corner.set(x, y, z).applyMatrix4(b.matrix));
    }
    return out;
}

// ============== API ==============
export function initRosMesh(scene) {
    root = new THREE.Group();
    root.name = 'rosSurface';
    root.visible = false;
    scene.add(root);
    layers.floor = new SurfaceLayer('floor');
}

/** Changed tiles from the worker: { layer, reset, anchor, cell, tiles: [{ tx, ty, h, w }], removed: [[tx, ty]] }. */
export function applyRosTiles(m) {
    const layer = layers[m.layer || 'floor'];
    if (layer) layer.apply(m);
}

export function clearRosMesh() {
    for (const l of Object.values(layers)) l.clear();
}

/** The surface is drawn (the ROS feature is on). */
export function setRosMeshVisible(v) {
    view.visible = !!v;
    if (root) root.visible = view.visible && view.shown;
}

/** The operator's show / hide; accumulation goes on while hidden. */
export function setRosMeshShown(v) {
    view.shown = !!v;
    if (root) root.visible = view.visible && view.shown;
}

/** Opacity of the triangles' body, 0…1 (0 = lines only). */
export function setRosMeshFillOpacity(a) {
    shared.uFill.value = Math.max(0, Math.min(1, a));
}

/** 'gray', 'distance' (red near → blue far from the vehicle) or 'rough' (grey flat → red irregular). */
export function setRosMeshColorMode(mode) {
    const v = MODES[mode] ?? MODES.gray;
    if (v === shared.uMode.value) return;
    shared.uMode.value = v;
    if (v === MODES.rough) {
        for (const l of Object.values(layers)) for (const b of l.blocks.values()) if (!b.roughDone) b.dirty = true;
    }
}

/** RMS about the local plane drawn full red, m. */
export function getRosRoughFull() { return shared.uRoughFull.value; }

/** Samples a cell needs to be drawn at its own size; fewer: a coarser level. */
export function setRosMeshMinSamples(n) {
    const v = Math.max(0.1, Number(n) || 1);
    if (v === view.minSamples) return;
    view.minSamples = v;
    for (const l of Object.values(layers)) for (const b of l.blocks.values()) b.dirty = true;
}

/** Draw through the terrain mesh (true) or depth-tested with a faint hidden pass. */
export function setRosMeshOverTerrain(over) {
    view.overTerrain = !!over;
    for (const l of Object.values(layers)) {
        for (const b of l.blocks.values()) { b.mat.depthTest = !view.overTerrain; b.mat.needsUpdate = true; }
    }
}

/** Distance range of the colour ramp, m (nearest surface → blue). */
export function getRosColorRange() { return view.range; }

/** Per frame: altitude offset, colour range, frustum, levels of detail, rebuilds. */
export function updateRosMesh() {
    if (!root || !root.visible) return;
    const camera = getCamera(), renderer = getRenderer();
    if (!camera || !renderer) return;
    const all = Object.values(layers);
    if (!all.some(l => l.blocks.size)) return;

    const veh = latLonToMeters(STATE.lat, STATE.lon);
    shared.uVehicle.value.set(veh.x, (STATE.rawAlt || 0) + (STATE.offsetAlt || 0), veh.z);
    const now = performance.now();
    if (shared.uMode.value === MODES.distance && now - view.lastRange > RANGE_MS) {
        view.lastRange = now;
        const near = Math.min(...all.map(l => l.nearest(shared.uVehicle.value)));
        if (Number.isFinite(near)) {
            view.range = { near, far: near + Math.max(20, 2 * near) };
            shared.uNear.value = view.range.near;
            shared.uFar.value = view.range.far;
        }
    }

    projView.multiplyMatrices(camera.projectionMatrix, camera.matrixWorldInverse);
    frustum.setFromProjectionMatrix(projView);
    const focalPx = renderer.getContext().drawingBufferHeight / (2 * Math.tan((camera.fov * Math.PI / 180) / 2));
    const todo = [];
    for (const l of all) l.plan(camera.position, focalPx, todo);
    // Nearest first, within the budget (a full-resolution block is 57 600 quads)
    todo.sort((p, q) => p[0] - q[0]);
    let budget = REBUILD_QUADS_PER_FRAME;
    for (const [, l, b, s] of todo) {
        if (budget <= 0) break;
        if (b.dirty) {
            l.placeBlock(b);
            l.fillBlock(b);
            budget -= BLOCK * BLOCK;
        }
        budget -= l.buildIndex(b, s);
        b.mesh.visible = b.triangles > 0;
        b.ghost.visible = b.mesh.visible && !view.overTerrain;
    }
}

/** Tiles drawn now, as the worker sent them (diagnostics, tests). */
export function getRosSurface(layer = 'floor') {
    const l = layers[layer];
    return l && l.anchor ? { anchor: l.anchor, cell: l.cell, tiles: l.tiles, axes: l.axes } : null;
}

/** What is drawn now (status line, tests): totals, and per layer. */
export function getRosMeshStats() {
    const per = {}, total = { tiles: 0, blocks: 0, blocksDrawn: 0, triangles: 0, levels: new Array(LEVELS + 1).fill(0) };
    for (const [name, l] of Object.entries(layers)) {
        const s = l.stats();
        per[name] = s;
        total.tiles += s.tiles; total.blocks += s.blocks; total.blocksDrawn += s.blocksDrawn; total.triangles += s.triangles;
        s.levels.forEach((v, k) => { total.levels[k] += v; });
    }
    return { ...total, layers: per };
}
