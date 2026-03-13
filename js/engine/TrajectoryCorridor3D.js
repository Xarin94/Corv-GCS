/**
 * TrajectoryCorridor3D.js - Corridor outline for predicted trajectory (DJI RTH style)
 * Two green border lines + translucent green fill, slightly behind the aircraft.
 * Length scales with speed.
 */

const MAX_POINTS = 50;
const CORRIDOR_HALF_WIDTH = 0.6; // metres – half-width (~1.2m total, drone wingspan +20%)
const ALT_DROP = 3;              // metres – slight drop below aircraft altitude
const START_OFFSET = -8;         // metres – push corridor start well behind the aircraft

// Speed-based prediction time: faster → longer corridor
const MIN_PRED_TIME = 5;   // seconds at MIN_SPEED
const MAX_PRED_TIME = 20;  // seconds at or above MAX_SPEED_REF
const MIN_SPEED_REF = 5;   // m/s
const MAX_SPEED_REF = 60;  // m/s

let leftLine = null;
let rightLine = null;
let fillMesh = null;
let leftGeo = null;
let rightGeo = null;
let fillGeo = null;
let leftPosAttr = null;
let rightPosAttr = null;
let leftColorAttr = null;
let rightColorAttr = null;
let fillPosAttr = null;
let fillColorAttr = null;
let sceneRef = null;

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

    // ── Border lines ──
    const makeLine = () => {
        const geo = new THREE.BufferGeometry();
        const positions = new Float32Array(MAX_POINTS * 3);
        const colors = new Float32Array(MAX_POINTS * 4);
        const pAttr = new THREE.BufferAttribute(positions, 3);
        const cAttr = new THREE.BufferAttribute(colors, 4);
        geo.setAttribute('position', pAttr);
        geo.setAttribute('color', cAttr);
        geo.setDrawRange(0, 0);

        const mat = new THREE.LineBasicMaterial({
            vertexColors: true,
            transparent: true,
            linewidth: 1,
            depthWrite: false
        });

        const line = new THREE.Line(geo, mat);
        line.frustumCulled = false;
        line.renderOrder = 3;
        line.visible = false;
        scene.add(line);
        return { line, geo, pAttr, cAttr };
    };

    const left = makeLine();
    leftLine = left.line; leftGeo = left.geo;
    leftPosAttr = left.pAttr; leftColorAttr = left.cAttr;

    const right = makeLine();
    rightLine = right.line; rightGeo = right.geo;
    rightPosAttr = right.pAttr; rightColorAttr = right.cAttr;

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
    scene.add(fillMesh);
}

/**
 * Update corridor from predicted path points.
 * @param {Array<{x:number, y:number, z:number}>} points
 */
export function updateCorridor(points) {
    if (!leftLine || !points || points.length < 2) {
        if (leftLine) {
            leftGeo.setDrawRange(0, 0);
            rightGeo.setDrawRange(0, 0);
            fillGeo.setDrawRange(0, 0);
        }
        return;
    }

    const n = Math.min(points.length, MAX_POINTS);
    const lPos = leftPosAttr.array;
    const rPos = rightPosAttr.array;
    const lCol = leftColorAttr.array;
    const rCol = rightColorAttr.array;
    const fPos = fillPosAttr.array;
    const fCol = fillColorAttr.array;

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

        // Perpendicular on XZ plane
        const len = Math.sqrt(dx * dx + dz * dz) || 1;
        const px = -dz / len;
        const pz = dx / len;

        const y = p.y - ALT_DROP;
        // Shift all points backward by START_OFFSET along first-segment direction
        const cx = p.x + backDx;
        const cz = p.z + backDz;

        const lx = cx + px * CORRIDOR_HALF_WIDTH;
        const lz = cz + pz * CORRIDOR_HALF_WIDTH;
        const rx = cx - px * CORRIDOR_HALF_WIDTH;
        const rz = cz - pz * CORRIDOR_HALF_WIDTH;

        // ── Border lines ──
        const vi = i * 3;
        lPos[vi] = lx; lPos[vi + 1] = y; lPos[vi + 2] = lz;
        rPos[vi] = rx; rPos[vi + 1] = y; rPos[vi + 2] = rz;

        // Fade-out toward tail + fade-in from start (masks origin behind aircraft)
        const fadeOut = 1 - (i / (n - 1));
        const fadeIn = Math.min(i / 4, 1);   // ramp up over first 4 points
        const fade = fadeOut * fadeIn;
        const borderAlpha = 0.8 * fade;
        const ci = i * 4;
        lCol[ci] = 0.1; lCol[ci + 1] = 1.0; lCol[ci + 2] = 0.2; lCol[ci + 3] = borderAlpha;
        rCol[ci] = 0.1; rCol[ci + 1] = 1.0; rCol[ci + 2] = 0.2; rCol[ci + 3] = borderAlpha;

        // ── Fill mesh (left vertex, right vertex) ──
        const fli = (i * 2) * 3;
        fPos[fli] = lx; fPos[fli + 1] = y; fPos[fli + 2] = lz;
        const fri = (i * 2 + 1) * 3;
        fPos[fri] = rx; fPos[fri + 1] = y; fPos[fri + 2] = rz;

        const fillAlpha = 0.18 * fadeOut * fadeIn;
        const fci = (i * 2) * 4;
        fCol[fci] = 0.1; fCol[fci + 1] = 0.9; fCol[fci + 2] = 0.15; fCol[fci + 3] = fillAlpha;
        const rci = (i * 2 + 1) * 4;
        fCol[rci] = 0.1; fCol[rci + 1] = 0.9; fCol[rci + 2] = 0.15; fCol[rci + 3] = fillAlpha;
    }

    leftGeo.setDrawRange(0, n);
    rightGeo.setDrawRange(0, n);
    fillGeo.setDrawRange(0, (n - 1) * 6);

    leftPosAttr.needsUpdate = true;
    rightPosAttr.needsUpdate = true;
    leftColorAttr.needsUpdate = true;
    rightColorAttr.needsUpdate = true;
    fillPosAttr.needsUpdate = true;
    fillColorAttr.needsUpdate = true;
    leftGeo.computeBoundingSphere();
    rightGeo.computeBoundingSphere();
    fillGeo.computeBoundingSphere();
}

/**
 * Show or hide the corridor.
 * @param {boolean} visible
 */
export function setCorridorVisible(visible) {
    if (leftLine) leftLine.visible = visible;
    if (rightLine) rightLine.visible = visible;
    if (fillMesh) fillMesh.visible = visible;
}

/**
 * Dispose corridor resources.
 */
export function disposeCorridor() {
    const cleanup = (obj, geo) => {
        if (obj && sceneRef) {
            sceneRef.remove(obj);
            geo.dispose();
            obj.material.dispose();
        }
    };
    cleanup(leftLine, leftGeo);
    cleanup(rightLine, rightGeo);
    cleanup(fillMesh, fillGeo);
    leftLine = rightLine = fillMesh = null;
    leftGeo = rightGeo = fillGeo = null;
}
