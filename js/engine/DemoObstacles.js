/**
 * DemoObstacles.js - trees, hangars and pylons for the demo LiDAR
 *
 * The demo patrol circuit flies 50 m over the airport; to give the demo LiDAR
 * something to reconstruct besides the terrain, a fixed field of obstacles
 * is planted around the circuit the first time the SRTM data is available:
 * forest patches (trunk + crown), a few hangars and a line of pylons.
 *
 * They are NOT drawn. They exist only as analytic shapes (vertical cylinder,
 * sphere, axis-aligned box) for LidarDemo to cast its rays against, so the
 * only place they show up is the point cloud once the LiDAR is enabled —
 * the way a real scan reveals what the satellite imagery does not.
 *
 * Positions are geographic (lat/lon + terrain-derived base altitude), laid
 * out with a seeded PRNG so every run of the demo looks the same.
 */

import { STATE, demoFlightState, isDemoMode } from '../core/state.js';
import { getTerrainElevationFromHGT } from '../terrain/TerrainManager.js';
import { DEMO_STRAIGHT, DEMO_TURN_RADIUS } from '../core/constants.js';

// Exposed to LidarDemo. All heights in metres MSL, sizes in metres.
export const DEMO_OBSTACLES = {
    built: false,
    trees: [],    // { lat, lon, base, trunkH, trunkR, crownR }  crown centre = base + trunkH + crownR * 0.8
    boxes: []     // { lat, lon, base, halfE, halfN, h, kind: 'hangar' | 'pylon' }
};

let seed = 0x9E3779B9;
function rand() {                                   // mulberry32
    seed = (seed + 0x6D2B79F5) | 0;
    let t = seed;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
}

/**
 * Per frame: build once the demo circuit and the terrain exist.
 */
export function updateDemoObstacles() {
    if (!DEMO_OBSTACLES.built && isDemoMode() && demoFlightState.initialized && STATE.terrainHeight !== null) build();
}

function build() {
    const lat0 = demoFlightState.centerLat, lon0 = demoFlightState.centerLon;
    const mLat = 111320, mLon = Math.max(1, 111320 * Math.cos(lat0 * Math.PI / 180));
    const at = (n, e) => ({ lat: lat0 + n / mLat, lon: lon0 + e / mLon });
    const ground = (lat, lon) => getTerrainElevationFromHGT(lat, lon);

    // The racetrack: legs at e = ±R, n ∈ [-H, H]; obstacles go under and
    // beside the legs, where the 50 m scan actually reaches.
    const R = DEMO_TURN_RADIUS, H = DEMO_STRAIGHT / 2;
    const trees = [], boxes = [];
    seed = 0x9E3779B9;

    // Forest patches along both legs
    for (let p = 0; p < 14; p++) {
        const leg = p % 2 === 0 ? R : -R;
        const cn = -H + (H * 2) * ((p >> 1) + 0.5) / 7 + (rand() - 0.5) * 120;
        const ce = leg + (rand() - 0.5) * 220;
        const count = 18 + Math.floor(rand() * 22);
        const radius = 35 + rand() * 40;
        for (let i = 0; i < count; i++) {
            const a = rand() * Math.PI * 2, r = Math.sqrt(rand()) * radius;
            const { lat, lon } = at(cn + Math.sin(a) * r, ce + Math.cos(a) * r);
            const base = ground(lat, lon);
            if (base === null) continue;
            trees.push({ lat, lon, base, trunkH: 3 + rand() * 6, trunkR: 0.25 + rand() * 0.25, crownR: 2.5 + rand() * 4.5 });
        }
    }
    // Hangars: scattered off the legs
    for (let i = 0; i < 8; i++) {
        const leg = i % 2 === 0 ? R : -R;
        const { lat, lon } = at(-H + (H * 2) * (i + 0.5) / 8, leg + (rand() < 0.5 ? -1 : 1) * (60 + rand() * 90));
        const base = ground(lat, lon);
        if (base === null) continue;
        boxes.push({ lat, lon, base, halfE: 8 + rand() * 18, halfN: 6 + rand() * 12, h: 6 + rand() * 12, kind: 'hangar' });
    }
    // A power line: pylons across the south end of the circuit
    for (let i = 0; i < 7; i++) {
        const { lat, lon } = at(-H - 120, -R - 200 + i * (2 * R + 400) / 6);
        const base = ground(lat, lon);
        if (base === null) continue;
        boxes.push({ lat, lon, base, halfE: 1.2, halfN: 1.2, h: 22 + rand() * 6, kind: 'pylon' });
    }

    DEMO_OBSTACLES.trees = trees;
    DEMO_OBSTACLES.boxes = boxes;
    DEMO_OBSTACLES.built = true;
    console.log(`[demo] lidar obstacles: ${trees.length} trees, ${boxes.length} boxes`);
}
