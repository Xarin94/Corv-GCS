/**
 * ADSBManager.js - ADS-B Traffic Data via OpenSky Network
 * Fetches real-time aircraft positions, merges into STATE.traffic,
 * and provides CSV/JSON download functionality.
 */

import { STATE } from '../core/state.js';

let lastFetchTime = 0;
const FETCH_COOLDOWN = 10000; // 10s OpenSky rate limit

/**
 * Compute distance in meters between two lat/lon points (Haversine)
 */
function haversineM(lat1, lon1, lat2, lon2) {
    const R = 6371000;
    const dLat = (lat2 - lat1) * Math.PI / 180;
    const dLon = (lon2 - lon1) * Math.PI / 180;
    const a = Math.sin(dLat / 2) ** 2 +
        Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
        Math.sin(dLon / 2) ** 2;
    return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
}

/**
 * Fetch ADS-B data from OpenSky Network and merge into STATE.traffic.
 * Uses IPC bridge to main process (Node.js https) to bypass CORS.
 */
export async function fetchADSBData() {
    const now = Date.now();
    if (now - lastFetchTime < FETCH_COOLDOWN) {
        const wait = Math.ceil((FETCH_COOLDOWN - (now - lastFetchTime)) / 1000);
        return { error: `Rate limited, wait ${wait}s` };
    }

    const margin = 0.5; // degrees (~50km at mid-latitudes)
    const lamin = STATE.lat - margin;
    const lomin = STATE.lon - margin;
    const lamax = STATE.lat + margin;
    const lomax = STATE.lon + margin;

    // Use IPC bridge (main process) to avoid CORS issues in renderer
    const data = await window.adsb.fetch(lamin, lomin, lamax, lomax);
    lastFetchTime = Date.now();

    // Check for error returned from main process
    if (data.error) {
        throw new Error(data.error);
    }

    // OpenSky state vector columns:
    // 0=icao24, 1=callsign, 2=origin_country, 5=longitude, 6=latitude,
    // 7=baro_altitude, 8=on_ground, 9=velocity, 10=true_track,
    // 11=vertical_rate, 13=geo_altitude
    const fetched = (data.states || []).map(s => ({
        icao24: s[0],
        callsign: (s[1] || '').trim(),
        country: s[2],
        lon: s[5],
        lat: s[6],
        alt: s[7] !== null ? s[7] : (s[13] || 0),
        velocity: s[9],
        heading: s[10],
        vertRate: s[11],
        onGround: s[8],
        _ts: Date.now()
    })).filter(t => t.lat !== null && t.lon !== null);

    // Merge into STATE.traffic (replace by icao24, add new)
    for (const ac of fetched) {
        const idx = STATE.traffic.findIndex(t => t.icao24 === ac.icao24);
        if (idx >= 0) {
            STATE.traffic[idx] = ac;
        } else {
            STATE.traffic.push(ac);
        }
    }

    return { count: fetched.length, traffic: fetched };
}

/**
 * Get the N nearest traffic entries sorted by distance from drone
 */
export function getNearestTraffic(n = 4) {
    if (STATE.traffic.length === 0) return [];

    return STATE.traffic
        .map(t => ({
            ...t,
            dist: haversineM(STATE.lat, STATE.lon, t.lat, t.lon)
        }))
        .sort((a, b) => a.dist - b.dist)
        .slice(0, n);
}

/**
 * Download current STATE.traffic as CSV file
 */
export function downloadTrafficCSV() {
    const list = STATE.traffic;
    if (list.length === 0) return;
    const header = 'icao24,callsign,country,lat,lon,alt_m,velocity_ms,heading_deg,vert_rate_ms,on_ground\n';
    const rows = list.map(t =>
        `${t.icao24},${t.callsign || ''},${t.country || ''},${t.lat},${t.lon},${t.alt},${t.velocity},${t.heading},${t.vertRate},${t.onGround}`
    ).join('\n');
    triggerDownload(header + rows, `adsb_${new Date().toISOString().replace(/[:.]/g, '-')}.csv`, 'text/csv');
}

function triggerDownload(content, filename, mimeType) {
    const blob = new Blob([content], { type: mimeType });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
}
