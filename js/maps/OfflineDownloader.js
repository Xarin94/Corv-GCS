/**
 * OfflineDownloader.js - Bulk download satellite tiles + SRTM1 elevation
 * Used by the Sys Config "Offline Data Download" panel.
 */

import { latLonToTile } from '../core/utils.js';
import { estimateTileCount, downloadArea } from './TileCache.js';
import { addHGTFile } from '../terrain/TerrainManager.js';

const SRTM_BASE = 'https://elevation-tiles-prod.s3.amazonaws.com/skadi';
const MAX_CONCURRENT_SRTM = 4;

// Min satellite zoom to download. The 2D map opens at zoom 13 and the 3D
// terrain uses zoom 15-16; zoom < 11 is regional/world overview that loads
// fast from network anyway, so caching it offline is wasted bandwidth.
const SAT_MIN_ZOOM = 11;

// ============== SRTM1 Download ==============

/**
 * Build list of SRTM1 tile names for a lat/lon rectangle
 */
function buildSrtmTileList(latSouth, latNorth, lonWest, lonEast) {
    const tiles = [];
    const latMin = Math.floor(latSouth);
    const latMax = Math.floor(latNorth);
    const lonMin = Math.floor(lonWest);
    const lonMax = Math.floor(lonEast);

    for (let lat = latMin; lat <= latMax; lat++) {
        for (let lon = lonMin; lon <= lonMax; lon++) {
            const ns = lat >= 0 ? 'N' : 'S';
            const ew = lon >= 0 ? 'E' : 'W';
            const latStr = String(Math.abs(lat)).padStart(2, '0');
            const lonStr = String(Math.abs(lon)).padStart(3, '0');
            tiles.push(`${ns}${latStr}${ew}${lonStr}`);
        }
    }
    return tiles;
}

/**
 * Download a single SRTM1 .hgt.gz, decompress, save via IPC, and register in terrain
 * @returns {'downloaded'|'skipped'|'not_found'|'error'}
 */
async function downloadSrtmTile(name, signal) {
    const filename = `${name}.hgt`;

    // Skip if already downloaded on disk. Use the cheap exists() check — not
    // loadOne(), which would read and IPC-transfer the whole ~25 MB file.
    if (window.topography && window.topography.exists) {
        const present = await window.topography.exists(filename);
        if (present) return 'skipped';
    } else if (window.topography && window.topography.loadOne) {
        // Fallback for older preload bridge without exists()
        const existing = await window.topography.loadOne(filename);
        if (existing) return 'skipped';
    }

    const ns = name.substring(0, 1);
    const folder = `${ns}${name.substring(1, 3)}`;
    const url = `${SRTM_BASE}/${folder}/${name}.hgt.gz`;

    try {
        const res = await fetch(url, { signal });
        if (res.status === 404 || res.status === 403) return 'not_found';
        if (!res.ok) throw new Error(`HTTP ${res.status}`);

        const compressed = await res.arrayBuffer();

        // Decompress gzip in the browser using DecompressionStream
        const ds = new DecompressionStream('gzip');
        const writer = ds.writable.getWriter();
        writer.write(new Uint8Array(compressed));
        writer.close();

        const reader = ds.readable.getReader();
        const chunks = [];
        let totalLen = 0;
        while (true) {
            const { done, value } = await reader.read();
            if (done) break;
            chunks.push(value);
            totalLen += value.byteLength;
        }

        // Assemble into single ArrayBuffer
        const hgtBuf = new Uint8Array(totalLen);
        let offset = 0;
        for (const chunk of chunks) {
            hgtBuf.set(chunk, offset);
            offset += chunk.byteLength;
        }

        // Validate SRTM1 size
        const expected = 3601 * 3601 * 2;
        if (hgtBuf.byteLength !== expected) {
            console.warn(`[offline] ${name}.hgt unexpected size: ${hgtBuf.byteLength} (expected ${expected})`);
            return 'error';
        }

        // Save to disk via IPC
        const filename = `${name}.hgt`;
        if (window.topography && window.topography.save) {
            await window.topography.save(filename, hgtBuf.buffer);
        }

        // Register in terrain engine immediately
        const file = new File([hgtBuf.buffer], filename);
        addHGTFile(filename, file);

        return 'downloaded';
    } catch (e) {
        if (signal && signal.aborted) throw e;
        console.warn(`[offline] SRTM ${name} error:`, e.message);
        return 'error';
    }
}

/**
 * Download all SRTM1 tiles for a rectangle
 */
async function downloadAllSrtm(latSouth, latNorth, lonWest, lonEast, onProgress, signal) {
    const tiles = buildSrtmTileList(latSouth, latNorth, lonWest, lonEast);
    const total = tiles.length;
    let downloaded = 0, skipped = 0, notFound = 0, errors = 0;
    let idx = 0;

    const report = () => onProgress({
        phase: 'srtm', downloaded, skipped, notFound, errors,
        total, current: downloaded + skipped + notFound + errors
    });

    const worker = async () => {
        while (idx < tiles.length) {
            if (signal && signal.aborted) return;
            const tile = tiles[idx++];
            const result = await downloadSrtmTile(tile, signal);
            switch (result) {
                case 'downloaded': downloaded++; break;
                case 'skipped': skipped++; break;
                case 'not_found': notFound++; break;
                case 'error': errors++; break;
            }
            report();
        }
    };

    const workers = [];
    for (let i = 0; i < Math.min(MAX_CONCURRENT_SRTM, tiles.length); i++) {
        workers.push(worker());
    }
    await Promise.all(workers);

    return { downloaded, skipped, notFound, errors, total };
}

// ============== Estimation ==============

export function estimateOfflineDownload(latSouth, latNorth, lonWest, lonEast, satZoomMax, dlSatellite, dlSrtm) {
    let satTiles = 0;
    if (dlSatellite) {
        const bounds = { north: latNorth, south: latSouth, east: lonEast, west: lonWest };
        satTiles = estimateTileCount(bounds, SAT_MIN_ZOOM, satZoomMax);
    }

    let srtmTiles = 0;
    if (dlSrtm) {
        srtmTiles = buildSrtmTileList(latSouth, latNorth, lonWest, lonEast).length;
    }

    return { satTiles, srtmTiles };
}

// ============== Main Download ==============

// ============== UI Panel Init ==============

export function initOfflinePanel() {
    const northEl = document.getElementById('offline-lat-north');
    const southEl = document.getElementById('offline-lat-south');
    const westEl = document.getElementById('offline-lon-west');
    const eastEl = document.getElementById('offline-lon-east');
    const zoomEl = document.getElementById('offline-sat-zoom');
    const satChk = document.getElementById('offline-dl-satellite');
    const srtmChk = document.getElementById('offline-dl-srtm');
    const estimateEl = document.getElementById('offline-dl-estimate');
    const startBtn = document.getElementById('offline-dl-start');
    const cancelBtn = document.getElementById('offline-dl-cancel');
    const progressBar = document.getElementById('offline-dl-progress');
    const statusEl = document.getElementById('offline-dl-status');

    if (!startBtn) return;

    let abortController = null;

    // Hard caps: beyond these the download would take hours and can fill the
    // disk (IndexedDB tiles / ~25 MB per SRTM cell).
    const MAX_SAT_TILES = 300000;   // ~4-6 GB of imagery
    const MAX_SRTM_TILES = 200;     // ~5 GB of elevation data

    /**
     * Read and validate the user-entered rectangle + zoom.
     * Returns null (with a reason string) on invalid input — NaN would
     * otherwise slip through `n <= s` checks (NaN comparisons are false).
     */
    function readBounds() {
        const n = parseFloat(northEl.value);
        const s = parseFloat(southEl.value);
        const w = parseFloat(westEl.value);
        const e = parseFloat(eastEl.value);
        let z = parseInt(zoomEl.value, 10);
        if (![n, s, w, e].every(Number.isFinite)) {
            return { error: 'Invalid bounds (numbers required)' };
        }
        if (n <= s || e <= w) {
            return { error: 'Invalid bounds (North > South, East > West)' };
        }
        if (n > 85 || s < -85 || w < -180 || e > 180) {
            return { error: 'Bounds out of range (lat ±85, lon ±180)' };
        }
        if (!Number.isFinite(z)) z = 16;
        z = Math.max(SAT_MIN_ZOOM, Math.min(17, z));
        return { n, s, w, e, z };
    }

    function updateEstimate() {
        const b = readBounds();
        if (b.error) {
            estimateEl.textContent = b.error;
            return;
        }
        const dlSat = satChk.checked;
        const dlSrtm = srtmChk.checked;

        const est = estimateOfflineDownload(b.s, b.n, b.w, b.e, b.z, dlSat, dlSrtm);
        const parts = [];
        if (est.satTiles > 0) parts.push(`~${est.satTiles.toLocaleString()} sat tiles`);
        if (est.srtmTiles > 0) parts.push(`${est.srtmTiles} SRTM tiles (~${(est.srtmTiles * 25).toLocaleString()} MB)`);
        if (est.satTiles > MAX_SAT_TILES || est.srtmTiles > MAX_SRTM_TILES) {
            parts.push('— AREA TOO LARGE, reduce bounds or zoom');
        }
        estimateEl.textContent = parts.join(' + ') || 'Nothing selected';
    }

    [northEl, southEl, westEl, eastEl, zoomEl].forEach(el => el.addEventListener('input', updateEstimate));
    [satChk, srtmChk].forEach(el => el.addEventListener('change', updateEstimate));

    startBtn.addEventListener('click', async () => {
        const b = readBounds();
        if (b.error) { statusEl.textContent = b.error; return; }
        const { n, s, w, e, z } = b;

        // Refuse pathological areas before starting (disk/network protection)
        const est = estimateOfflineDownload(s, n, w, e, z, satChk.checked, srtmChk.checked);
        if (est.satTiles > MAX_SAT_TILES || est.srtmTiles > MAX_SRTM_TILES) {
            statusEl.textContent = 'Area too large — reduce bounds or zoom level';
            return;
        }

        abortController = new AbortController();
        startBtn.style.display = 'none';
        cancelBtn.style.display = '';
        progressBar.style.display = '';
        progressBar.value = 0;
        statusEl.textContent = 'Starting...';

        try {
            const results = await startOfflineDownload(s, n, w, e, z,
                satChk.checked, srtmChk.checked,
                (info) => {
                    progressBar.max = info.total;
                    progressBar.value = info.current;
                    if (info.phase === 'srtm') {
                        statusEl.textContent = `SRTM: ${info.downloaded} downloaded, ${info.notFound} no data, ${info.errors} errors / ${info.total}`;
                    } else {
                        statusEl.textContent = `Satellite: ${info.downloaded} / ${info.total}${info.failed ? ` (${info.failed} failed)` : ''}`;
                    }
                },
                abortController.signal
            );

            const parts = [];
            if (results.srtm) parts.push(`SRTM: ${results.srtm.downloaded} downloaded`);
            if (results.satellite) parts.push(`Sat: ${results.satellite.downloaded} tiles`);
            statusEl.textContent = 'Done! ' + parts.join(', ');
        } catch (err) {
            statusEl.textContent = abortController.signal.aborted ? 'Cancelled' : `Error: ${err.message}`;
        }

        startBtn.style.display = '';
        cancelBtn.style.display = 'none';
        abortController = null;
    });

    cancelBtn.addEventListener('click', () => {
        if (abortController) abortController.abort();
    });

    updateEstimate();
}

/**
 * Download satellite tiles + SRTM1 for a rectangle
 */
export async function startOfflineDownload(latSouth, latNorth, lonWest, lonEast, satZoomMax, dlSatellite, dlSrtm, onProgress, signal) {
    const results = { satellite: null, srtm: null };

    // Phase 1: SRTM
    if (dlSrtm) {
        results.srtm = await downloadAllSrtm(latSouth, latNorth, lonWest, lonEast, onProgress, signal);
        if (signal && signal.aborted) return results;
    }

    // Phase 2: Satellite tiles
    if (dlSatellite) {
        const bounds = { north: latNorth, south: latSouth, east: lonEast, west: lonWest };
        results.satellite = await downloadArea(
            bounds, SAT_MIN_ZOOM, satZoomMax, ['esri'],
            ({ downloaded, failed, total }) => {
                onProgress({ phase: 'satellite', downloaded, failed, total, current: downloaded + failed });
            },
            signal
        );
    }

    return results;
}
