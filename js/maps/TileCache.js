/**
 * TileCache.js - IndexedDB tile cache + bulk download engine
 * Persistent offline map tile storage, Mission Planner style.
 */

import { latLonToTile } from '../core/utils.js';

const DB_NAME = 'datad-tile-cache';
const DB_VERSION = 1;
const STORE_NAME = 'tiles';
const MAX_CONCURRENT = 6;

const PROVIDER_URLS = {
    osm: 'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
    esri: 'https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}'
};
const OSM_SUBDOMAINS = ['a', 'b', 'c'];

let dbPromise = null;

function openDB() {
    if (dbPromise) return dbPromise;
    dbPromise = new Promise((resolve, reject) => {
        const req = indexedDB.open(DB_NAME, DB_VERSION);
        req.onupgradeneeded = () => {
            const db = req.result;
            if (!db.objectStoreNames.contains(STORE_NAME)) {
                db.createObjectStore(STORE_NAME);
            }
        };
        req.onsuccess = () => resolve(req.result);
        req.onerror = () => reject(req.error);
    });
    return dbPromise;
}

function tileKey(provider, z, x, y) {
    return `${provider}/${z}/${x}/${y}`;
}

/**
 * Get a cached tile blob from IndexedDB
 * @returns {Promise<Blob|null>}
 */
export async function getTile(provider, z, x, y) {
    const db = await openDB();
    return new Promise((resolve) => {
        const tx = db.transaction(STORE_NAME, 'readonly');
        const store = tx.objectStore(STORE_NAME);
        const req = store.get(tileKey(provider, z, x, y));
        req.onsuccess = () => resolve(req.result || null);
        req.onerror = () => resolve(null);
    });
}

/**
 * Store a tile blob in IndexedDB
 */
export async function putTile(provider, z, x, y, blob) {
    const db = await openDB();
    return new Promise((resolve, reject) => {
        const tx = db.transaction(STORE_NAME, 'readwrite');
        const store = tx.objectStore(STORE_NAME);
        const req = store.put(blob, tileKey(provider, z, x, y));
        req.onsuccess = () => resolve();
        req.onerror = () => reject(req.error);
    });
}

/**
 * Build the fetch URL for a tile
 */
function buildTileUrl(provider, z, x, y, subdomainIdx) {
    if (provider === 'osm') {
        const s = OSM_SUBDOMAINS[subdomainIdx % OSM_SUBDOMAINS.length];
        return `https://${s}.tile.openstreetmap.org/${z}/${x}/${y}.png`;
    }
    // Esri uses {z}/{y}/{x}
    const s = subdomainIdx % 4;
    return `https://mt${s}.google.com/vt/lyrs=s&x=${x}&y=${y}&z=${z}`;
}

/**
 * Enumerate all tile coordinates for a bounds at a given zoom
 * @param {{north, south, east, west}} bounds
 * @param {number} z - zoom level
 * @returns {Array<{x,y,z}>}
 */
function tilesInBounds(bounds, z) {
    const nw = latLonToTile(bounds.north, bounds.west, z);
    const se = latLonToTile(bounds.south, bounds.east, z);
    const tiles = [];
    const minX = Math.min(nw.x, se.x);
    const maxX = Math.max(nw.x, se.x);
    const minY = Math.min(nw.y, se.y);
    const maxY = Math.max(nw.y, se.y);
    for (let x = minX; x <= maxX; x++) {
        for (let y = minY; y <= maxY; y++) {
            tiles.push({ x, y, z });
        }
    }
    return tiles;
}

/**
 * Estimate total tile count for a bounds across zoom range
 */
export function estimateTileCount(bounds, minZoom, maxZoom) {
    let total = 0;
    for (let z = minZoom; z <= maxZoom; z++) {
        const nw = latLonToTile(bounds.north, bounds.west, z);
        const se = latLonToTile(bounds.south, bounds.east, z);
        const minX = Math.min(nw.x, se.x);
        const maxX = Math.max(nw.x, se.x);
        const minY = Math.min(nw.y, se.y);
        const maxY = Math.max(nw.y, se.y);
        total += (maxX - minX + 1) * (maxY - minY + 1);
    }
    return total;
}

/**
 * Bulk download tiles for an area
 * @param {{north, south, east, west}} bounds
 * @param {number} minZoom
 * @param {number} maxZoom
 * @param {string[]} providers - ['osm'], ['esri'], or ['osm','esri']
 * @param {function} onProgress - ({downloaded, failed, total}) => void
 * @param {AbortSignal} [signal] - optional abort signal
 * @returns {Promise<{downloaded: number, failed: number, total: number}>}
 */
export async function downloadArea(bounds, minZoom, maxZoom, providers, onProgress, signal) {
    // Build full task list
    const tasks = [];
    for (const provider of providers) {
        for (let z = minZoom; z <= maxZoom; z++) {
            for (const tile of tilesInBounds(bounds, z)) {
                tasks.push({ provider, ...tile });
            }
        }
    }

    const total = tasks.length;
    let downloaded = 0;
    let failed = 0;
    let subdomainCounter = 0;

    const report = () => onProgress && onProgress({ downloaded, failed, total });

    // Process with concurrency limit
    let idx = 0;
    const next = async () => {
        while (idx < tasks.length) {
            if (signal && signal.aborted) return;
            const task = tasks[idx++];
            const sd = subdomainCounter++;

            // Skip if already cached
            const existing = await getTile(task.provider, task.z, task.x, task.y);
            if (existing) {
                downloaded++;
                report();
                continue;
            }

            const url = buildTileUrl(task.provider, task.z, task.x, task.y, sd);
            try {
                const res = await fetch(url, { signal });
                if (!res.ok) throw new Error(res.status);
                const blob = await res.blob();
                await putTile(task.provider, task.z, task.x, task.y, blob);
                downloaded++;
            } catch (e) {
                if (signal && signal.aborted) return;
                failed++;
            }
            report();
        }
    };

    const workers = [];
    for (let i = 0; i < Math.min(MAX_CONCURRENT, tasks.length); i++) {
        workers.push(next());
    }
    await Promise.all(workers);

    return { downloaded, failed, total };
}

/**
 * Get cache statistics
 * @returns {Promise<{count: number}>}
 */
export async function getCacheStats() {
    const db = await openDB();
    return new Promise((resolve) => {
        const tx = db.transaction(STORE_NAME, 'readonly');
        const store = tx.objectStore(STORE_NAME);
        const req = store.count();
        req.onsuccess = () => resolve({ count: req.result });
        req.onerror = () => resolve({ count: 0 });
    });
}

/**
 * Clear all cached tiles
 * @returns {Promise<void>}
 */
export async function clearCache() {
    const db = await openDB();
    return new Promise((resolve, reject) => {
        const tx = db.transaction(STORE_NAME, 'readwrite');
        const store = tx.objectStore(STORE_NAME);
        const req = store.clear();
        req.onsuccess = () => resolve();
        req.onerror = () => reject(req.error);
    });
}
