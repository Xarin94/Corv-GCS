/**
 * TerrainFeeder.js - MAVLink terrain data provider
 * Listens for TERRAIN_REQUEST from the vehicle and responds with TERRAIN_DATA
 * using locally cached SRTM1 HGT elevation data.
 *
 * Protocol: https://mavlink.io/en/services/terrain.html
 */

import { STATE } from '../core/state.js';
import { onMessage } from './MAVLinkManager.js';
import { getTerrainElevationFromHGT, getTerrainElevationAsync } from '../terrain/TerrainManager.js';

// ArduPilot AP_Terrain grid geometry
const GRID_BLOCK_SIZE = 4;        // 4x4 elevation points per block
const GRID_BLOCK_COLS = 8;        // 8 blocks across (X / longitude)
const GRID_BLOCK_ROWS = 7;        // 7 blocks down  (Y / latitude)
const GRID_BLOCK_TOTAL = GRID_BLOCK_COLS * GRID_BLOCK_ROWS; // 56 bits max in mask

// Rate limiting to avoid saturating serial links
const MAX_MESSAGES_PER_SECOND = 20;
let messagesSentThisSecond = 0;
let lastSecondReset = 0;

/**
 * Check if a bit is set using the two 32-bit mask halves.
 * The mask is split into maskLow (bits 0-31) and maskHigh (bits 32-55)
 * by main-mavlink.js to avoid BigInt→Number precision loss.
 */
function isBitSet(maskLow, maskHigh, bit) {
    if (bit < 32) {
        return ((maskLow >>> 0) & (1 << bit)) !== 0;
    }
    return ((maskHigh >>> 0) & (1 << (bit - 32))) !== 0;
}

/**
 * Initialize the terrain feeder - register MAVLink message handlers
 */
export function initTerrainFeeder() {
    // TERRAIN_REQUEST (msg 133) from vehicle
    onMessage(133, handleTerrainRequest);
    // TERRAIN_REPORT (msg 136) from vehicle (state mapping done in MAVLinkStateMapper)
    onMessage(136, handleTerrainReport);

    console.log('[terrain-feeder] Initialized - listening for TERRAIN_REQUEST');
}

/**
 * Handle TERRAIN_REQUEST from the vehicle.
 * For each requested block (set bit in mask), look up 16 elevation values
 * and send back a TERRAIN_DATA message.
 */
async function handleTerrainRequest(data) {
    if (!STATE.terrainFeedEnabled) return;

    const lat = data.lat;              // int32 degE7 - SW corner
    const lon = data.lon;              // int32 degE7 - SW corner
    const gridSpacing = data.gridSpacing; // uint16 meters (typically 100)
    const maskLow  = data.maskLow  || 0;  // bits 0-31 (from main-mavlink.js)
    const maskHigh = data.maskHigh || 0;  // bits 32-55

    if (!gridSpacing || gridSpacing <= 0) return;

    const latDeg = lat / 1e7;
    const lonDeg = lon / 1e7;
    const cosLat = Math.cos(latDeg * Math.PI / 180);

    // Avoid division by zero near poles
    if (cosLat < 0.01) return;

    let blocksSent = 0;

    for (let bit = 0; bit < GRID_BLOCK_TOTAL; bit++) {
        if (!isBitSet(maskLow, maskHigh, bit)) continue;

        // Rate limiting
        const now = Date.now();
        if (now - lastSecondReset >= 1000) {
            messagesSentThisSecond = 0;
            lastSecondReset = now;
        }
        if (messagesSentThisSecond >= MAX_MESSAGES_PER_SECOND) {
            await new Promise(r => setTimeout(r, 50));
            messagesSentThisSecond = 0;
            lastSecondReset = Date.now();
        }

        const blockX = bit % GRID_BLOCK_COLS;
        const blockY = Math.floor(bit / GRID_BLOCK_COLS);

        const elevations = new Array(16);
        let hasError = false;

        for (let j = 0; j < 16; j++) {
            const dx = j % GRID_BLOCK_SIZE;
            const dy = Math.floor(j / GRID_BLOCK_SIZE);

            // Total offset in meters from grid SW corner
            const eastMeters  = (blockX * GRID_BLOCK_SIZE + dx) * gridSpacing;
            const northMeters = (blockY * GRID_BLOCK_SIZE + dy) * gridSpacing;

            // Convert meter offsets to lat/lon
            const ptLat = latDeg + northMeters / 111111.0;
            const ptLon = lonDeg + eastMeters / (111111.0 * cosLat);

            // Try synchronous lookup first (fast path)
            let elev = getTerrainElevationFromHGT(ptLat, ptLon);

            // Fallback to async (may lazy-load from disk or auto-download)
            if (elev === null) {
                elev = await getTerrainElevationAsync(ptLat, ptLon);
            }

            if (elev === null) {
                elev = 0; // Sea level fallback - vehicle will re-request if needed
                hasError = true;
            }

            elevations[j] = Math.round(elev);
        }

        if (hasError) STATE.terrainFeedErrors++;

        // Send TERRAIN_DATA response
        try {
            await window.mavlink.sendMessage({
                type: 'TERRAIN_DATA',
                lat: lat,
                lon: lon,
                gridSpacing: gridSpacing,
                gridbit: bit,
                data: elevations
            });
            STATE.terrainFeedSent++;
            messagesSentThisSecond++;
            blocksSent++;
        } catch (e) {
            console.error('[terrain-feeder] Send error:', e);
        }
    }

    if (blocksSent > 0) {
        console.log(`[terrain-feeder] Sent ${blocksSent} blocks for (${latDeg.toFixed(4)}, ${lonDeg.toFixed(4)}) spacing=${gridSpacing}m`);
    }
}

/**
 * Handle TERRAIN_REPORT from vehicle (debug logging).
 * State mapping is already done by MAVLinkStateMapper.
 */
function handleTerrainReport(data) {
    if (data.pending > 0) {
        console.log(`[terrain-feeder] Vehicle terrain status: pending=${data.pending} loaded=${data.loaded}`);
    }
}
