/**
 * TerrainFeeder.js - MAVLink terrain data provider
 *
 * ArduPilot with TERRAIN_ENABLE asks the GCS for the terrain it lacks: a
 * TERRAIN_REQUEST names a grid (south-west corner, spacing) and a 56-bit mask
 * of the 4x4 blocks it still needs, and each block goes back as one
 * TERRAIN_DATA. The vehicle repeats the request every 2 s for whatever is
 * still missing, so:
 *   - a block is queued once however many requests name it, and sent again
 *     only if the vehicle still asks for it a while after it went out (the
 *     packet was lost);
 *   - one queue drains at a fixed pace, so a burst of requests cannot flood a
 *     telemetry radio;
 *   - a block with a point of unknown height is not sent at all. The vehicle
 *     keeps what it receives and stops asking, so a stand-in 0 m would be
 *     flown as the ground. Unanswered, it keeps asking.
 *
 * Heights come from the SRTM tiles TerrainManager loads (topo/ or downloaded).
 *
 * TERRAIN_REPORT says how many blocks the vehicle still lacks. When that does
 * not move for a while the feeder says why, once per connection: the vehicle
 * asks for nothing (ArduPilot does not ask the GCS while a grid waits to be
 * read from its SD card, and an unreadable card keeps it waiting for ever),
 * the GCS has no heights for what it asks, or the vehicle drops what is sent.
 *
 * Protocol: https://mavlink.io/en/services/terrain.html
 */

import { STATE } from '../core/state.js';
import { onMessage } from './MAVLinkManager.js';
import { getTerrainElevationChecked, getTerrainElevationAsync } from '../terrain/TerrainManager.js';
import { pushHudMessage } from '../hud/HUDRenderer.js';

// ArduPilot AP_Terrain: a grid is 7 x 8 blocks (north x east) of 4 x 4 points.
// Block `bit` starts (bit / 8) * 4 points north and (bit % 8) * 4 points east
// of the corner; point (x north, y east) of a block is data[x * 4 + y].
const BLOCK_POINTS = 4;
const BLOCKS_EAST = 8;
const BLOCK_COUNT = 56;

const SEND_RATE_HZ = 20;        // ~55-byte packets: about a fifth of a 57600-baud radio
const RESEND_AFTER_MS = 3000;   // requests repeat every 2 s; still asked after this = packet lost

// ArduPilot's Location::offset(): 1e-7 degrees per metre, longitude scaled at the mid latitude
const LOCATION_SCALING_FACTOR_INV = 89.83204953368922;

const STALL_MS = 20000;         // TERRAIN_REPORT unchanged this long with blocks pending = stalled

const queue = new Map();        // block key → block, in arrival order
const sentAt = new Map();       // block key → when it went out
const reportedMissing = new Set();
let pumping = false;

// What the vehicle reports and what it asks for, to explain a stall
const diag = { lastRequestAt: 0, lastSentAt: 0, lastMissingAt: 0, report: '', changedAt: 0, warned: false };

const blockKey = (lat, lon, spacing, bit) => `${lat},${lon},${spacing},${bit}`;
const sleep = ms => new Promise(r => setTimeout(r, ms));

/**
 * The mask arrives split into two 32-bit halves (main-mavlink.js), since a
 * Number cannot hold 56 bits exactly.
 */
function isBitSet(maskLow, maskHigh, bit) {
    if (bit < 32) return ((maskLow >>> 0) & (1 << bit)) !== 0;
    return ((maskHigh >>> 0) & (1 << (bit - 32))) !== 0;
}

/** Point `north` / `east` metres from a degE7 corner, as the vehicle computes it */
function offsetLatLon(latE7, lonE7, north, east) {
    const dlat = north * LOCATION_SCALING_FACTOR_INV;
    const midLat = (latE7 + dlat / 2) * 1e-7;
    const lonScale = Math.max(0.01, Math.cos(midLat * Math.PI / 180));
    const dlon = east * LOCATION_SCALING_FACTOR_INV / lonScale;
    return [(latE7 + dlat) * 1e-7, (lonE7 + dlon) * 1e-7];
}

/**
 * Initialize the terrain feeder - register MAVLink message handlers
 */
export function initTerrainFeeder() {
    // TERRAIN_REQUEST (msg 133) from vehicle
    onMessage(133, handleTerrainRequest);
    // TERRAIN_REPORT (msg 136) from vehicle (state mapping done in MAVLinkStateMapper)
    onMessage(136, handleTerrainReport);
    // A new link starts from what the vehicle asks then
    window.addEventListener('vehicleConnected', () => {
        queue.clear();
        sentAt.clear();
        reportedMissing.clear();
        Object.assign(diag, { lastRequestAt: 0, lastSentAt: 0, lastMissingAt: 0, report: '', changedAt: 0, warned: false });
    });

    console.log('[terrain-feeder] Initialized - listening for TERRAIN_REQUEST');
}

function handleTerrainRequest(data) {
    if (!STATE.terrainFeedEnabled) return;
    const { lat, lon, gridSpacing } = data;
    if (!Number.isFinite(lat) || !Number.isFinite(lon) || !(gridSpacing > 0)) return;
    if (Math.abs(lat) > 89e7) return;                          // no grid at the poles

    let maskLow = data.maskLow, maskHigh = data.maskHigh;
    if (!Number.isFinite(maskLow) && Number.isFinite(data.mask)) {
        maskLow = data.mask % 2 ** 32;
        maskHigh = Math.floor(data.mask / 2 ** 32);
    }

    const now = Date.now();
    if (!diag.lastRequestAt) {
        console.log(`[terrain-feeder] Vehicle asks for terrain: grid ${(lat * 1e-7).toFixed(4)}, ${(lon * 1e-7).toFixed(4)}, spacing ${gridSpacing} m`);
    }
    diag.lastRequestAt = now;
    if (sentAt.size > 4000) {
        for (const [k, t] of sentAt) if (now - t > 60000) sentAt.delete(k);
    }
    for (let bit = 0; bit < BLOCK_COUNT; bit++) {
        if (!isBitSet(maskLow || 0, maskHigh || 0, bit)) continue;
        const key = blockKey(lat, lon, gridSpacing, bit);
        if (queue.has(key)) continue;
        const sent = sentAt.get(key);
        if (sent && now - sent < RESEND_AFTER_MS) continue;  // answer still on its way
        queue.set(key, { key, lat, lon, spacing: gridSpacing, bit });
    }
    pump();
}

/** Drain the queue, one TERRAIN_DATA at a time */
async function pump() {
    if (pumping) return;
    pumping = true;
    try {
        while (queue.size) {
            if (!STATE.connected) { queue.clear(); break; }
            const block = queue.values().next().value;
            queue.delete(block.key);

            const heights = await blockHeights(block);
            if (!heights) { noteMissing(block); continue; }

            try {
                await window.mavlink.sendMessage({
                    type: 'TERRAIN_DATA',
                    lat: block.lat,
                    lon: block.lon,
                    gridSpacing: block.spacing,
                    gridbit: block.bit,
                    data: heights
                });
                sentAt.set(block.key, Date.now());
                diag.lastSentAt = Date.now();
                STATE.terrainFeedSent++;
            } catch (e) {
                console.error('[terrain-feeder] Send error:', e.message);
            }
            await sleep(1000 / SEND_RATE_HZ);
        }
    } finally {
        pumping = false;
    }
}

/** The 16 heights of a block (metres AMSL), or null if any of them is unknown */
async function blockHeights({ lat, lon, spacing, bit }) {
    const north0 = Math.floor(bit / BLOCKS_EAST) * BLOCK_POINTS;
    const east0 = (bit % BLOCKS_EAST) * BLOCK_POINTS;
    const heights = new Array(BLOCK_POINTS * BLOCK_POINTS);
    for (let x = 0; x < BLOCK_POINTS; x++) {
        for (let y = 0; y < BLOCK_POINTS; y++) {
            const [ptLat, ptLon] = offsetLatLon(lat, lon, (north0 + x) * spacing, (east0 + y) * spacing);
            let h = getTerrainElevationChecked(ptLat, ptLon);
            if (h === null) {
                // Tile not loaded yet: from disk, or downloaded (once per tile)
                await getTerrainElevationAsync(ptLat, ptLon);
                h = getTerrainElevationChecked(ptLat, ptLon);
            }
            if (h === null) return null;
            heights[x * BLOCK_POINTS + y] = Math.max(-32768, Math.min(32767, Math.round(h)));
        }
    }
    return heights;
}

function noteMissing({ lat, lon, spacing }) {
    STATE.terrainFeedErrors++;
    diag.lastMissingAt = Date.now();
    const grid = `${lat},${lon},${spacing}`;
    if (reportedMissing.has(grid)) return;
    reportedMissing.add(grid);
    console.warn(`[terrain-feeder] No elevation data near ${(lat * 1e-7).toFixed(4)}, ${(lon * 1e-7).toFixed(4)} — `
        + 'blocks left unanswered (SRTM tile missing or void)');
}

/**
 * TERRAIN_REPORT from the vehicle: logged when it changes, and explained once
 * if blocks stay pending. State mapping is done by MAVLinkStateMapper.
 */
function handleTerrainReport(data) {
    if (!Number.isFinite(data.pending)) return;
    const now = Date.now();
    const report = `${data.pending}/${data.loaded}`;
    if (report !== diag.report) {
        if (data.pending > 0 || diag.report) {
            console.log(`[terrain-feeder] Vehicle terrain status: pending=${data.pending} loaded=${data.loaded}`);
        }
        diag.report = report;
        diag.changedAt = now;
    }
    if (data.pending === 0 || diag.warned || now - diag.changedAt < STALL_MS) return;

    const recent = t => now - t < STALL_MS;
    let why, hud;
    if (!recent(diag.lastRequestAt)) {
        why = `the vehicle has ${data.pending} blocks pending but asks the GCS for none. ArduPilot does not ask while `
            + 'a grid waits to be read from its own storage, and it waits for ever if the storage cannot be read: '
            + 'the SD card is missing, full, write-protected or failing. Check the card and its APM/TERRAIN folder, '
            + 'then reboot the autopilot. (Firmware newer than 4.6 can also run without it: TERRAIN_OPTIONS bit 1, Disable Disk.)';
        hud = 'TERRAIN: vehicle SD card not readable';
    } else if (recent(diag.lastMissingAt) && !recent(diag.lastSentAt)) {
        why = 'the vehicle asks for terrain the GCS has no elevation data for (SRTM tile missing or void) — add the tile to topo/ or allow the download.';
        hud = 'TERRAIN: no elevation data here';
    } else if (recent(diag.lastSentAt)) {
        why = `blocks are being sent but the vehicle's count does not move (pending ${data.pending}, loaded ${data.loaded}).`;
        hud = 'TERRAIN: vehicle not taking data';
    } else {
        return;   // nothing to say yet
    }
    diag.warned = true;
    console.warn(`[terrain-feeder] Terrain stalled: ${why}`);
    pushHudMessage(hud, 'warning');
}
