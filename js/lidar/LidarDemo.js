/**
 * LidarDemo.js - synthetic point cloud for the demo flight
 *
 * The demo flight (LIVE mode without a link) already injects coherent fake
 * telemetry: a patrol circuit over the terrain with attitude, position,
 * speeds and a rangefinder. This gives it a LiDAR to match: every frame a few
 * hundred rays leave the demo aircraft in a forward-down scan pattern and are
 * intersected with the real SRTM heightfield, so the cloud that builds up is
 * the actual relief under the circuit — fictitious points, real terrain.
 *
 * It is renderer-only (no sockets, no worker): rays are marched against
 * getTerrainElevationFromHGT() and tested against the demo obstacles (trees,
 * hangars, pylons from DemoObstacles.js), decimated through a voxel set and
 * appended to LidarCloud through the same origin/batch API the real link
 * uses, so the drawing, the colour ramps, the strip and CLEAR MAP behave
 * identically.
 *
 * Sensor model: a Mid-360 mounted with its spin axis along the fuselage.
 * The 360° sweep then fans around the flight axis — down, both sides, up —
 * and the -7°…+52° band leans that fan forward: a cross-track push-broom
 * that paints the ground and the obstacles under and beside the track. Rays
 * that go up hit nothing. Range is stretched to 150 m so the swath reaches
 * beyond the runway edges from 50 m AGL.
 */

import { STATE, isDemoMode } from '../core/state.js';
import { getTerrainElevationFromHGT } from '../terrain/TerrainManager.js';
import { setLidarOrigin, appendLidarPoints } from './LidarCloud.js';
import { DEMO_OBSTACLES } from '../engine/DemoObstacles.js';

const DEG = Math.PI / 180;
const SENSOR = {
    elMin: -7 * DEG,           // Mid-360 band, measured from the plane normal to the spin axis…
    elMax: 52 * DEG,           // …positive = leaning forward (spin axis = body X, +Z of the LiDAR forward)
    range: 150,                // m
    pointsPerSec: 20000,
    maxPerFrame: 700,
    voxel: 0.5,                // m
    stepCoarse: 10,            // m, ray march against the terrain
    budgetMs: 2.5              // per frame, whatever the ray count
};

let running = false;
let minRange = 2.5;            // from the LIDAR settings: closer returns are the airframe
let origin = null;             // { lat, lon, alt, mPerLat, mPerLon, epoch }
let epoch = 100;               // separate range from the worker's epochs
let voxels = new Set();
let carry = 0;
let lastT = 0;
let total = 0;

// Obstacles in the demo origin's ENU frame (rebuilt when the origin changes)
let obsBuiltFor = null;
let treeE, treeN, treeBase, treeTop, treeR, crownE, crownN, crownU, crownR;   // Float64Array
let boxMinE, boxMaxE, boxMinN, boxMaxN, boxMinU, boxMaxU, boxKind;
let nTrees = 0, nBoxes = 0;
// Candidates near the vehicle this frame
let candTrees = new Int32Array(0), candBoxes = new Int32Array(0), nCandT = 0, nCandB = 0;
let lastCandE = 1e9, lastCandN = 1e9;

const R = new Float64Array(9);
const batchXyz = new Float32Array(SENSOR.maxPerFrame * 3);
const batchInt = new Uint8Array(SENSOR.maxPerFrame);

function eulerToMatrix(roll, pitch, yaw, out) {
    const cr = Math.cos(roll), sr = Math.sin(roll), cp = Math.cos(pitch), sp = Math.sin(pitch), cy = Math.cos(yaw), sy = Math.sin(yaw);
    out[0] = cy * cp; out[1] = cy * sp * sr - sy * cr; out[2] = cy * sp * cr + sy * sr;
    out[3] = sy * cp; out[4] = sy * sp * sr + cy * cr; out[5] = sy * sp * cr - cy * sr;
    out[6] = -sp;     out[7] = cp * sr;                out[8] = cp * cr;
    return out;
}

function metersPerDegree(latDeg) {
    const f = latDeg * DEG;
    return {
        mPerLat: 111132.954 - 559.822 * Math.cos(2 * f) + 1.175 * Math.cos(4 * f),
        mPerLon: 111412.84 * Math.cos(f) - 93.5 * Math.cos(3 * f)
    };
}

export function startLidarDemo() {
    running = true;
    lastT = performance.now();
    carry = 0;
}

export function stopLidarDemo() {
    running = false;
}

export function isLidarDemoRunning() { return running; }

/** Settings the demo honours (the real client applies them in the worker). */
export function configureLidarDemo(cfg) {
    if (cfg && Number.isFinite(cfg.minRange)) minRange = Math.max(0, cfg.minRange);
}

// False echoes off the airframe, the reason MIN RANGE exists: with the spin
// axis along the fuselage the fan grazes the wing roots (sideways, in the
// plane of the disc) and the landing gear (down, slightly aft). Returns the
// range of such a hit for this ray, or -1.
function airframeHit(az, el) {
    const side = Math.abs(Math.abs(az) - Math.PI / 2);          // 0 = along a wing
    if (el < 3 * DEG && side < 0.5 && Math.random() < 0.35) return 0.8 + Math.random() * 1.6;   // wing root / strut
    if (el < 0 && Math.abs(az) < 0.35 && Math.random() < 0.25) return 0.5 + Math.random() * 0.5; // landing gear
    if (el < -3 * DEG && Math.abs(az) > 2.6 && Math.random() < 0.3) return 1.2 + Math.random() * 1.2; // fin / rudder
    return -1;
}

/** Forget the accumulated map; the next frame re-anchors (CLEAR MAP). */
export function resetLidarDemo() {
    origin = null;
    voxels = new Set();
    total = 0;
}

export function getLidarDemoPointCount() { return total; }

// Height of the terrain at an ENU offset from the origin, or null off-tile.
function terrainAt(e, n) {
    return getTerrainElevationFromHGT(origin.lat + n / origin.mPerLat, origin.lon + e / origin.mPerLon);
}

function prepareObstacles() {
    const T = DEMO_OBSTACLES.trees, B = DEMO_OBSTACLES.boxes;
    nTrees = T.length; nBoxes = B.length;
    treeE = new Float64Array(nTrees); treeN = new Float64Array(nTrees); treeBase = new Float64Array(nTrees);
    treeTop = new Float64Array(nTrees); treeR = new Float64Array(nTrees);
    crownE = new Float64Array(nTrees); crownN = new Float64Array(nTrees); crownU = new Float64Array(nTrees); crownR = new Float64Array(nTrees);
    T.forEach((t, i) => {
        treeE[i] = (t.lon - origin.lon) * origin.mPerLon;
        treeN[i] = (t.lat - origin.lat) * origin.mPerLat;
        treeBase[i] = t.base - origin.alt;
        treeTop[i] = treeBase[i] + t.trunkH + t.crownR * 0.8;
        treeR[i] = t.trunkR;
        crownE[i] = treeE[i]; crownN[i] = treeN[i]; crownU[i] = treeTop[i]; crownR[i] = t.crownR;
    });
    boxMinE = new Float64Array(nBoxes); boxMaxE = new Float64Array(nBoxes); boxMinN = new Float64Array(nBoxes);
    boxMaxN = new Float64Array(nBoxes); boxMinU = new Float64Array(nBoxes); boxMaxU = new Float64Array(nBoxes); boxKind = new Uint8Array(nBoxes);
    B.forEach((b, i) => {
        const e = (b.lon - origin.lon) * origin.mPerLon, n = (b.lat - origin.lat) * origin.mPerLat;
        boxMinE[i] = e - b.halfE; boxMaxE[i] = e + b.halfE;
        boxMinN[i] = n - b.halfN; boxMaxN[i] = n + b.halfN;
        boxMinU[i] = b.base - origin.alt; boxMaxU[i] = boxMinU[i] + b.h;
        boxKind[i] = b.kind === 'pylon' ? 1 : 0;
    });
    candTrees = new Int32Array(nTrees); candBoxes = new Int32Array(nBoxes);
    obsBuiltFor = origin;
    lastCandE = 1e9;
}

// Obstacles within reach of the vehicle; refreshed once it has moved 20 m.
function selectCandidates(vehE, vehN) {
    if (Math.abs(vehE - lastCandE) < 20 && Math.abs(vehN - lastCandN) < 20) return;
    lastCandE = vehE; lastCandN = vehN;
    const reach = SENSOR.range + 40;
    nCandT = 0; nCandB = 0;
    for (let i = 0; i < nTrees; i++) {
        if (Math.abs(treeE[i] - vehE) < reach && Math.abs(treeN[i] - vehN) < reach) candTrees[nCandT++] = i;
    }
    for (let i = 0; i < nBoxes; i++) {
        if (boxMinE[i] - vehE < reach && vehE - boxMaxE[i] < reach && boxMinN[i] - vehN < reach && vehN - boxMaxN[i] < reach) candBoxes[nCandB++] = i;
    }
}

// Nearest obstacle hit along the ray within tMax; returns t or -1, kind in hitKind.
let hitKind = 0;   // 1 trunk, 2 crown, 3 hangar, 4 pylon
function castObstacles(oe, on, ou, de, dn, du, tMax) {
    let best = tMax;
    hitKind = 0;
    for (let c = 0; c < nCandT; c++) {
        const i = candTrees[c];
        // Crown: sphere
        let ox = oe - crownE[i], oy = on - crownN[i], oz = ou - crownU[i];
        let b = ox * de + oy * dn + oz * du;
        let cc = ox * ox + oy * oy + oz * oz - crownR[i] * crownR[i];
        let disc = b * b - cc;
        if (disc > 0) {
            const t = -b - Math.sqrt(disc);
            if (t > 0.5 && t < best) { best = t; hitKind = 2; }
        }
        // Trunk: vertical cylinder between base and top
        const dh2 = de * de + dn * dn;
        if (dh2 > 1e-6) {
            ox = oe - treeE[i]; oy = on - treeN[i];
            b = (ox * de + oy * dn) / dh2;
            cc = (ox * ox + oy * oy - treeR[i] * treeR[i]) / dh2;
            disc = b * b - cc;
            if (disc > 0) {
                const t = -b - Math.sqrt(disc);
                if (t > 0.5 && t < best) {
                    const u = ou + du * t;
                    if (u >= treeBase[i] && u <= treeTop[i]) { best = t; hitKind = 1; }
                }
            }
        }
    }
    for (let c = 0; c < nCandB; c++) {
        const i = candBoxes[c];
        let tmin = 0.5, tmax = best;
        // slab test on E, N, U
        let t1, t2;
        if (Math.abs(de) < 1e-9) { if (oe < boxMinE[i] || oe > boxMaxE[i]) continue; }
        else { t1 = (boxMinE[i] - oe) / de; t2 = (boxMaxE[i] - oe) / de; if (t1 > t2) { const s = t1; t1 = t2; t2 = s; } if (t1 > tmin) tmin = t1; if (t2 < tmax) tmax = t2; if (tmin > tmax) continue; }
        if (Math.abs(dn) < 1e-9) { if (on < boxMinN[i] || on > boxMaxN[i]) continue; }
        else { t1 = (boxMinN[i] - on) / dn; t2 = (boxMaxN[i] - on) / dn; if (t1 > t2) { const s = t1; t1 = t2; t2 = s; } if (t1 > tmin) tmin = t1; if (t2 < tmax) tmax = t2; if (tmin > tmax) continue; }
        if (Math.abs(du) < 1e-9) { if (ou < boxMinU[i] || ou > boxMaxU[i]) continue; }
        else { t1 = (boxMinU[i] - ou) / du; t2 = (boxMaxU[i] - ou) / du; if (t1 > t2) { const s = t1; t1 = t2; t2 = s; } if (t1 > tmin) tmin = t1; if (t2 < tmax) tmax = t2; if (tmin > tmax) continue; }
        if (tmin < best) { best = tmin; hitKind = boxKind[i] ? 4 : 3; }
    }
    return hitKind ? best : -1;
}

/**
 * Per frame, from the render loop. Casts as many rays as the point rate and
 * the time budget allow: terrain by marching the heightfield, obstacles
 * analytically, nearest hit wins.
 */
export function updateLidarDemo() {
    if (!running || !isDemoMode() || STATE.terrainHeight === null) return;
    const t0 = performance.now();
    const dt = Math.min(0.1, (t0 - lastT) / 1000);
    lastT = t0;

    if (!origin) {
        const mpd = metersPerDegree(STATE.lat);
        origin = { lat: STATE.lat, lon: STATE.lon, alt: STATE.rawAlt, mPerLat: mpd.mPerLat, mPerLon: mpd.mPerLon, epoch: ++epoch };
        setLidarOrigin(origin);
    }

    carry += SENSOR.pointsPerSec * dt;
    let rays = Math.min(SENSOR.maxPerFrame, Math.floor(carry));
    carry -= rays;
    if (rays <= 0) return;

    if (DEMO_OBSTACLES.built && obsBuiltFor !== origin) prepareObstacles();

    eulerToMatrix(STATE.roll, STATE.pitch, STATE.yaw, R);
    const vehE = (STATE.lon - origin.lon) * origin.mPerLon;
    const vehN = (STATE.lat - origin.lat) * origin.mPerLat;
    const vehU = STATE.rawAlt - origin.alt;
    if (obsBuiltFor === origin) selectCandidates(vehE, vehN);
    const inv = 1 / SENSOR.voxel;
    let n = 0;

    for (let i = 0; i < rays; i++) {
        if ((i & 31) === 31 && performance.now() - t0 > SENSOR.budgetMs) break;
        // Spin axis = body X: az sweeps around the fuselage (0 = straight
        // down, π = straight up), el leans the fan forward.
        const az = Math.random() * 2 * Math.PI;
        const el = SENSOR.elMin + Math.random() * (SENSOR.elMax - SENSOR.elMin);
        const ce = Math.cos(el);
        const bx = Math.sin(el), by = ce * Math.sin(az), bz = ce * Math.cos(az);   // body FRD
        const dn = R[0] * bx + R[1] * by + R[2] * bz;
        const de = R[3] * bx + R[4] * by + R[5] * bz;
        const dd = R[6] * bx + R[7] * by + R[8] * bz;
        const du = -dd;

        let tHit = -1, kind = 0;
        const tAir = airframeHit(az, el);
        if (tAir > 0) {
            // The airframe itself, closer than anything else
            tHit = tAir; kind = 5;
        } else {
            if (du > 0.35) continue;                    // steeply up: sky, nothing to hit

            // Terrain: coarse march until the ray is under the heightfield, then bisect.
            let tTerrain = -1;
            if (dd > 0.02) {
                let tPrev = 0, hit = -1;
                for (let t = SENSOR.stepCoarse; t <= SENSOR.range; t += SENSOR.stepCoarse) {
                    const h = terrainAt(vehE + de * t, vehN + dn * t);
                    if (h === null) break;
                    if (vehU - dd * t <= h - origin.alt) { hit = t; break; }
                    tPrev = t;
                }
                if (hit > 0) {
                    let lo = tPrev, hi = hit;
                    for (let k = 0; k < 5; k++) {
                        const mid = (lo + hi) * 0.5;
                        const h = terrainAt(vehE + de * mid, vehN + dn * mid);
                        if (h === null) break;
                        if (vehU - dd * mid <= h - origin.alt) hi = mid; else lo = mid;
                    }
                    tTerrain = (lo + hi) * 0.5;
                }
            }

            // Obstacles closer than the terrain hit (or anywhere in range for up-going rays)
            tHit = tTerrain;
            const tObs = castObstacles(vehE, vehN, vehU, de, dn, du, tTerrain > 0 ? tTerrain : SENSOR.range);
            if (tObs > 0) { tHit = tObs; kind = hitKind; }
        }
        if (tHit < 0) continue;
        tHit += (Math.random() - 0.5) * 0.06;
        if (tHit < minRange) continue;                  // the MIN RANGE filter, as in the worker
        const E = vehE + de * tHit, N = vehN + dn * tHit, U = vehU + du * tHit;

        const key = ((Math.floor(E * inv) + 65536) * 131072 + (Math.floor(N * inv) + 65536)) * 131072 + (Math.floor(U * inv) + 65536);
        if (voxels.has(key)) continue;
        voxels.add(key);

        const o = n * 3;
        batchXyz[o] = E; batchXyz[o + 1] = N; batchXyz[o + 2] = U;
        // Reflectivity by material: vegetation dark, ground medium, hangar bright, steel brightest
        let refl = kind === 2 ? 35 + Math.random() * 40 : kind === 1 ? 50 + Math.random() * 30
                 : kind === 3 ? 120 + Math.random() * 50 : kind === 4 ? 190 + Math.random() * 50
                 : kind === 5 ? 150 + Math.random() * 60
                 : 60 + (1 - dd) * 100 + (Math.random() - 0.5) * 30;
        batchInt[n] = Math.max(0, Math.min(255, refl)) | 0;
        n++;
    }

    if (n > 0) {
        total += n;
        appendLidarPoints({ epoch: origin.epoch, enu: batchXyz.slice(0, n * 3), intensity: batchInt.slice(0, n) });
    }
}
