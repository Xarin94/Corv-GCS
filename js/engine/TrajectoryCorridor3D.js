/**
 * TrajectoryCorridor3D.js - Corridor outline for predicted trajectory (DJI RTH style)
 * Two green border lines + translucent green fill, slightly behind the aircraft.
 * The ribbon cross-section rolls with the predicted bank angle, so the two
 * border lines trace the predicted wingtip paths (twisting through turns).
 * Length scales with speed.
 *
 * The ribbon widens with prediction time, from about the wingspan at the
 * aircraft to several metres at the end: the further ahead, the less certain
 * the prediction — and a 1.2 m ribbon was invisible from the chase camera.
 * Borders are screen-space lines a few pixels wide, and a bar across the
 * ribbon marks every 5 s of predicted flight.
 */

import { ThickLine } from './ThickLine.js';
import { OVERLAY_LAYER } from './Layers.js';

const MAX_POINTS = 50;
const HALF_WIDTH_NEAR = 1.0;     // metres – half-width at the aircraft (~wingspan)
const HALF_WIDTH_FAR = 6.0;      // metres – half-width at the end of the prediction
const ALT_DROP = 1.2;            // metres – slight drop below aircraft altitude (near wing plane)
const START_OFFSET = -8;         // metres – push corridor start well behind the aircraft
const TICK_INTERVAL_S = 5;       // seconds between the bars across the ribbon
const BORDER_ALPHA = 0.95;
const FILL_ALPHA = 0.28;
const END_ALPHA = 0.35;          // fraction of the alpha left at the far end

// Speed-based prediction time: faster → longer corridor
const MIN_PRED_TIME = 5;   // seconds at MIN_SPEED
const MAX_PRED_TIME = 20;  // seconds at or above MAX_SPEED_REF
const MIN_SPEED_REF = 5;   // m/s
const MAX_SPEED_REF = 60;  // m/s

let leftLine = null;
let rightLine = null;
let ticks = null;
let fillMesh = null;
let fillGeo = null;
let fillPosAttr = null;
let fillColorAttr = null;
let sceneRef = null;
let borderColors = null;   // RGBA per point, shared by both borders
let tickColors = null;
let lineColor = 0x1aff33;
const fillRGB = [0.1, 0.9, 0.15];

/**
 * Corridor colour (the scene's palette follows the view and the UI theme).
 * @param {number} hex
 */
export function setCorridorColor(hex) {
    lineColor = hex;
    for (const line of [leftLine, rightLine, ticks]) if (line) line.setColor(hex);
    const c = new THREE.Color(hex);
    fillRGB[0] = c.r; fillRGB[1] = c.g; fillRGB[2] = c.b;
}

/**
 * Compute prediction time scaled by groundspeed.
 * @param {number} gs - groundspeed m/s
 * @returns {number} seconds
 */
export function getPredictionTime(gs) {
    if (gs <= MIN_SPEED_REF) return MIN_PRED_TIME;
    if (gs >= MAX_SPEED_REF) return MAX_PRED_TIME;
    const t = (gs - MIN_SPEED_REF) / (MAX_SPEED_REF - MIN_SPEED_REF);
    return MIN_PRED_TIME + t * (MAX_PRED_TIME - MIN_PRED_TIME);
}

/**
 * Initialise corridor (border lines + fill mesh) and add to scene (hidden).
 * @param {THREE.Scene} scene
 */
export function initCorridor(scene) {
    sceneRef = scene;

    // ── Border lines and 5 s bars ──
    leftLine = new ThickLine({ color: lineColor, width: 3, vertexColors: true, renderOrder: 3 }).addTo(scene);
    rightLine = new ThickLine({ color: lineColor, width: 3, vertexColors: true, renderOrder: 3 }).addTo(scene);
    ticks = new ThickLine({ mode: 'pairs', color: lineColor, width: 2.5, vertexColors: true, renderOrder: 3 }).addTo(scene);
    borderColors = new Float32Array(MAX_POINTS * 4);
    tickColors = new Float32Array(2 * Math.ceil(MAX_PRED_TIME / TICK_INTERVAL_S) * 4);

    // ── Fill mesh (translucent green strip between borders) ──
    fillGeo = new THREE.BufferGeometry();
    const fillPos = new Float32Array(MAX_POINTS * 2 * 3);  // 2 verts per point (left+right)
    const fillCol = new Float32Array(MAX_POINTS * 2 * 4);
    fillPosAttr = new THREE.BufferAttribute(fillPos, 3);
    fillColorAttr = new THREE.BufferAttribute(fillCol, 4);
    fillGeo.setAttribute('position', fillPosAttr);
    fillGeo.setAttribute('color', fillColorAttr);

    // Index buffer: quads between consecutive left/right pairs
    const maxTris = (MAX_POINTS - 1) * 2;
    const indices = new Uint16Array(maxTris * 3);
    let idx = 0;
    for (let i = 0; i < MAX_POINTS - 1; i++) {
        const l0 = i * 2, r0 = i * 2 + 1;
        const l1 = (i + 1) * 2, r1 = (i + 1) * 2 + 1;
        indices[idx++] = l0; indices[idx++] = l1; indices[idx++] = r1;
        indices[idx++] = l0; indices[idx++] = r1; indices[idx++] = r0;
    }
    fillGeo.setIndex(new THREE.BufferAttribute(indices, 1));
    fillGeo.setDrawRange(0, 0);

    const fillMat = new THREE.MeshBasicMaterial({
        vertexColors: true,
        transparent: true,
        side: THREE.DoubleSide,
        depthWrite: false
    });

    fillMesh = new THREE.Mesh(fillGeo, fillMat);
    fillMesh.frustumCulled = false;
    fillMesh.renderOrder = 2;
    fillMesh.visible = false;
    fillMesh.layers.set(OVERLAY_LAYER);
    scene.add(fillMesh);

    setCorridorVisible(false);
}

/**
 * Update corridor from predicted path points.
 * @param {Array<{x:number, y:number, z:number, t?:number, bank?:number}>} points
 */
export function updateCorridor(points) {
    if (!leftLine || !points || points.length < 2) {
        if (leftLine) {
            leftLine.clear();
            rightLine.clear();
            ticks.clear();
            fillGeo.setDrawRange(0, 0);
        }
        return;
    }

    const n = Math.min(points.length, MAX_POINTS);
    const left = [];
    const right = [];
    const fPos = fillPosAttr.array;
    const fCol = fillColorAttr.array;
    const tickPts = [];
    let tickN = 0;
    let nextTick = TICK_INTERVAL_S;

    // Compute backward offset direction from first segment
    let backDx = 0, backDz = 0;
    if (points.length >= 2) {
        const dx0 = points[1].x - points[0].x;
        const dz0 = points[1].z - points[0].z;
        const len0 = Math.sqrt(dx0 * dx0 + dz0 * dz0) || 1;
        backDx = -(dx0 / len0) * START_OFFSET; // negative offset = behind
        backDz = -(dz0 / len0) * START_OFFSET;
    }

    for (let i = 0; i < n; i++) {
        const p = points[i];

        // Direction on XZ plane
        let dx, dz;
        if (i < n - 1) {
            dx = points[i + 1].x - p.x;
            dz = points[i + 1].z - p.z;
        } else {
            dx = p.x - points[i - 1].x;
            dz = p.z - points[i - 1].z;
        }

        // Perpendicular on XZ plane. World frame is x=east, z=south, y=up,
        // so (-dz, dx) points to the STARBOARD side of the direction of travel.
        const len = Math.sqrt(dx * dx + dz * dz) || 1;
        const px = -dz / len;
        const pz = dx / len;

        const y = p.y - ALT_DROP;
        // Shift all points backward by START_OFFSET along first-segment direction
        const cx = p.x + backDx;
        const cz = p.z + backDz;

        // Roll the cross-section by the predicted bank angle: the ribbon
        // twists like wingtip trails (right bank → starboard edge drops).
        const along = i / (n - 1);
        const halfWidth = HALF_WIDTH_NEAR + (HALF_WIDTH_FAR - HALF_WIDTH_NEAR) * along;
        const bank = p.bank || 0;
        const cb = Math.cos(bank);
        const sb = Math.sin(bank);
        const wH = halfWidth * cb; // horizontal component
        const wV = halfWidth * sb; // vertical component

        const lx = cx + px * wH;             // starboard edge
        const lz = cz + pz * wH;
        const ly = y - wV;
        const rx = cx - px * wH;             // port edge
        const rz = cz - pz * wH;
        const ry = y + wV;

        // ── Border lines ──
        left.push({ x: lx, y: ly, z: lz });
        right.push({ x: rx, y: ry, z: rz });

        // Fade toward the tail + fade-in from start (masks origin behind aircraft)
        const fadeOut = 1 - (1 - END_ALPHA) * along;
        const fadeIn = Math.min(i / 4, 1);   // ramp up over first 4 points
        const fade = fadeOut * fadeIn;
        const ci = i * 4;
        borderColors[ci] = 1; borderColors[ci + 1] = 1; borderColors[ci + 2] = 1;
        borderColors[ci + 3] = BORDER_ALPHA * fade;

        // ── Fill mesh (starboard vertex, port vertex) ──
        const fli = (i * 2) * 3;
        fPos[fli] = lx; fPos[fli + 1] = ly; fPos[fli + 2] = lz;
        const fri = (i * 2 + 1) * 3;
        fPos[fri] = rx; fPos[fri + 1] = ry; fPos[fri + 2] = rz;

        const fillAlpha = FILL_ALPHA * fade;
        const fci = (i * 2) * 4;
        fCol[fci] = fillRGB[0]; fCol[fci + 1] = fillRGB[1]; fCol[fci + 2] = fillRGB[2]; fCol[fci + 3] = fillAlpha;
        const rci = (i * 2 + 1) * 4;
        fCol[rci] = fillRGB[0]; fCol[rci + 1] = fillRGB[1]; fCol[rci + 2] = fillRGB[2]; fCol[rci + 3] = fillAlpha;

        // ── A bar across the ribbon at every whole TICK_INTERVAL_S ──
        if (Number.isFinite(p.t) && p.t >= nextTick - 1e-6) {
            tickPts.push({ x: lx, y: ly, z: lz }, { x: rx, y: ry, z: rz });
            for (let k = 0; k < 2; k++) {
                const o = (tickN * 2 + k) * 4;
                tickColors[o] = 1; tickColors[o + 1] = 1; tickColors[o + 2] = 1;
                tickColors[o + 3] = BORDER_ALPHA * fade;
            }
            tickN++;
            nextTick += TICK_INTERVAL_S;
        }
    }

    leftLine.setPoints(left);
    rightLine.setPoints(right);
    leftLine.setColors(borderColors.subarray(0, n * 4));
    rightLine.setColors(borderColors.subarray(0, n * 4));
    ticks.setPoints(tickPts);
    ticks.setColors(tickColors.subarray(0, tickN * 2 * 4));
    fillGeo.setDrawRange(0, (n - 1) * 6);

    fillPosAttr.needsUpdate = true;
    fillColorAttr.needsUpdate = true;
    fillGeo.computeBoundingSphere();
}

/**
 * Show or hide the corridor.
 * @param {boolean} visible
 */
export function setCorridorVisible(visible) {
    if (leftLine) leftLine.setVisible(visible);
    if (rightLine) rightLine.setVisible(visible);
    if (ticks) ticks.setVisible(visible);
    if (fillMesh) fillMesh.visible = visible;
}

/**
 * Dispose corridor resources.
 */
export function disposeCorridor() {
    for (const line of [leftLine, rightLine, ticks]) {
        if (line && sceneRef) {
            line.removeFrom(sceneRef);
            line.dispose();
        }
    }
    if (fillMesh && sceneRef) {
        sceneRef.remove(fillMesh);
        fillGeo.dispose();
        fillMesh.material.dispose();
    }
    leftLine = rightLine = ticks = fillMesh = null;
    fillGeo = null;
}
