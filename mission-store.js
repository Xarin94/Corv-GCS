/**
 * mission-store.js - On-disk data root for saved missions, logs and the index
 *
 * Layout (all under one root so an installation is self-contained and copyable):
 *
 *   <root>/data/index.json      catalogue of every mission and log file
 *   <root>/data/missions/*.json saved missions
 *   <root>/data/logs/           .tlog / .crv flight logs (written by main-mavlink.js)
 *
 * The root is the installation directory when that is writable — which is what makes
 * the install portable — and falls back to the per-user data directory otherwise.
 * A default Windows install under "Program Files" is NOT writable without elevation,
 * so the fallback is the normal case there, not an error; getRoot() reports which one
 * is in use so the UI can show the real path.
 *
 * index.json is rebuilt from the directory contents on every read and write. It is a
 * cache for other tools to consume, never the source of truth: deleting it or dropping
 * a mission file in by hand both work.
 */

const { ipcMain, app, shell } = require('electron');
const fs = require('fs');
const path = require('path');

const INDEX_VERSION = 1;
const MISSION_EXT = '.json';

let cachedRoot = null;
let rootIsFallback = false;

function isWritable(dir) {
    try {
        fs.mkdirSync(dir, { recursive: true });
        const probe = path.join(dir, `.write-probe-${process.pid}`);
        fs.writeFileSync(probe, 'ok');
        fs.unlinkSync(probe);
        return true;
    } catch (e) {
        return false;
    }
}

/**
 * Resolve (once) the data root.
 * Packaged: the folder containing the executable. Development: the project folder.
 */
function getRoot() {
    if (cachedRoot) return cachedRoot;

    const installDir = app.isPackaged
        ? path.dirname(app.getPath('exe'))
        : app.getAppPath();
    const candidate = path.join(installDir, 'data');

    if (isWritable(candidate)) {
        cachedRoot = candidate;
        rootIsFallback = false;
    } else {
        cachedRoot = path.join(app.getPath('userData'), 'data');
        rootIsFallback = true;
        fs.mkdirSync(cachedRoot, { recursive: true });
        console.warn(`[store] Installation directory not writable, using ${cachedRoot}`);
    }
    console.log(`[store] Data root: ${cachedRoot}`);
    return cachedRoot;
}

function getMissionsDir() {
    const dir = path.join(getRoot(), 'missions');
    fs.mkdirSync(dir, { recursive: true });
    return dir;
}

function getLogsDir() {
    const dir = path.join(getRoot(), 'logs');
    fs.mkdirSync(dir, { recursive: true });
    return dir;
}

function getIndexPath() {
    return path.join(getRoot(), 'index.json');
}

/**
 * Filesystem-safe id derived from the mission name. Collisions are resolved by the
 * caller (save() suffixes -2, -3, …) rather than here, so a rename keeps a stable id.
 */
function slugify(name) {
    const base = String(name || '')
        .trim()
        .replace(/[^\w\-. ]+/g, '')
        .replace(/\s+/g, '-')
        .replace(/^[-.]+|[-.]+$/g, '')
        .slice(0, 60);
    return base || `mission-${Date.now()}`;
}

function readMissionFile(file) {
    const raw = fs.readFileSync(file, 'utf8');
    const data = JSON.parse(raw);
    if (!Array.isArray(data.items)) throw new Error('not a mission file (no items array)');
    return data;
}

/** Great-circle distance in metres, for the summary shown in the library list. */
function haversine(a, b) {
    const R = 6371000;
    const toRad = d => d * Math.PI / 180;
    const dLat = toRad(b.lat - a.lat);
    const dLon = toRad(b.lng - a.lng);
    const lat1 = toRad(a.lat), lat2 = toRad(b.lat);
    const h = Math.sin(dLat / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    return 2 * R * Math.asin(Math.sqrt(h));
}

function summarize(items) {
    const pts = items.filter(it => Number.isFinite(it.lat) && Number.isFinite(it.lng));
    let dist = 0;
    for (let i = 1; i < pts.length; i++) dist += haversine(pts[i - 1], pts[i]);
    const bounds = pts.length ? {
        minLat: Math.min(...pts.map(p => p.lat)), maxLat: Math.max(...pts.map(p => p.lat)),
        minLng: Math.min(...pts.map(p => p.lng)), maxLng: Math.max(...pts.map(p => p.lng)),
    } : null;
    return { waypoints: items.length, distanceM: Math.round(dist), bounds };
}

/** Scan missions/ and logs/ and rewrite index.json. Returns the index object. */
function rebuildIndex() {
    const missionsDir = getMissionsDir();
    const logsDir = getLogsDir();

    const missions = [];
    for (const name of fs.readdirSync(missionsDir)) {
        if (!name.toLowerCase().endsWith(MISSION_EXT)) continue;
        const file = path.join(missionsDir, name);
        try {
            const data = readMissionFile(file);
            const stat = fs.statSync(file);
            missions.push({
                id: path.basename(name, MISSION_EXT),
                file: path.join('missions', name).replace(/\\/g, '/'),
                name: data.name || path.basename(name, MISSION_EXT),
                notes: data.notes || '',
                created: data.created || stat.birthtime.toISOString(),
                modified: stat.mtime.toISOString(),
                ...summarize(data.items),
            });
        } catch (e) {
            // A corrupt or unrelated .json in the folder must not break the whole index
            console.warn(`[store] Skipping ${name}: ${e.message}`);
        }
    }
    missions.sort((a, b) => b.modified.localeCompare(a.modified));

    const logs = [];
    for (const name of fs.readdirSync(logsDir)) {
        const file = path.join(logsDir, name);
        try {
            const stat = fs.statSync(file);
            if (!stat.isFile()) continue;
            logs.push({
                file: path.join('logs', name).replace(/\\/g, '/'),
                name,
                type: path.extname(name).slice(1).toLowerCase(),
                size: stat.size,
                modified: stat.mtime.toISOString(),
            });
        } catch (e) { /* file vanished mid-scan */ }
    }
    logs.sort((a, b) => b.modified.localeCompare(a.modified));

    const index = {
        version: INDEX_VERSION,
        updated: new Date().toISOString(),
        root: getRoot(),
        portable: !rootIsFallback,
        missions,
        logs,
    };

    try {
        fs.writeFileSync(getIndexPath(), JSON.stringify(index, null, 2), 'utf8');
    } catch (e) {
        console.warn('[store] Could not write index.json:', e.message);
    }
    return index;
}

/**
 * Save a mission. With an `id` the existing file is overwritten (keeping its original
 * creation date); without one a new id is derived from the name, uniquified.
 */
function saveMission({ id, name, notes, items, vehicleType, meta }) {
    if (!Array.isArray(items) || items.length === 0) throw new Error('Mission is empty');
    const dir = getMissionsDir();

    let targetId = id;
    let created = new Date().toISOString();

    if (targetId) {
        const existing = path.join(dir, targetId + MISSION_EXT);
        if (fs.existsSync(existing)) {
            try { created = readMissionFile(existing).created || created; } catch (e) { /* keep new date */ }
        }
    } else {
        targetId = slugify(name);
        let n = 2;
        while (fs.existsSync(path.join(dir, targetId + MISSION_EXT))) {
            targetId = `${slugify(name)}-${n++}`;
        }
    }

    const payload = {
        format: 'corv-gcs-mission',
        version: 1,
        name: name || targetId,
        notes: notes || '',
        created,
        modified: new Date().toISOString(),
        vehicleType: vehicleType ?? null,
        meta: meta || {},
        items,
    };

    const file = path.join(dir, targetId + MISSION_EXT);
    // Write to a temp file and rename: a crash mid-write must not destroy the
    // mission that is being overwritten.
    const tmp = file + '.tmp';
    fs.writeFileSync(tmp, JSON.stringify(payload, null, 2), 'utf8');
    fs.renameSync(tmp, file);

    rebuildIndex();
    return { id: targetId, file, name: payload.name };
}

function loadMission(id) {
    const file = path.join(getMissionsDir(), path.basename(id) + MISSION_EXT);
    if (!fs.existsSync(file)) throw new Error(`Mission "${id}" not found`);
    const data = readMissionFile(file);
    return { id, ...data };
}

function deleteMission(id) {
    const file = path.join(getMissionsDir(), path.basename(id) + MISSION_EXT);
    if (fs.existsSync(file)) fs.unlinkSync(file);
    rebuildIndex();
    return true;
}

/** Rename keeps the file (and therefore the id) and only changes the display name. */
function renameMission(id, newName) {
    const data = loadMission(id);
    return saveMission({
        id,
        name: newName,
        notes: data.notes,
        items: data.items,
        vehicleType: data.vehicleType,
        meta: data.meta,
    });
}

function initMissionStoreHandlers() {
    // Build the layout and the index once at startup so the folders exist even
    // before the operator saves anything.
    try {
        rebuildIndex();
    } catch (e) {
        console.error('[store] Init failed:', e.message);
    }

    ipcMain.handle('store-get-root', () => ({
        root: getRoot(),
        missions: getMissionsDir(),
        logs: getLogsDir(),
        index: getIndexPath(),
        portable: !rootIsFallback,
    }));

    ipcMain.handle('store-reveal', (e, which) => {
        const target = which === 'logs' ? getLogsDir()
            : which === 'missions' ? getMissionsDir()
            : getRoot();
        shell.openPath(target);
        return target;
    });

    ipcMain.handle('missions-list', () => rebuildIndex());
    ipcMain.handle('missions-load', (e, id) => loadMission(id));
    ipcMain.handle('missions-save', (e, payload) => saveMission(payload));
    ipcMain.handle('missions-delete', (e, id) => deleteMission(id));
    ipcMain.handle('missions-rename', (e, id, name) => renameMission(id, name));
}

module.exports = {
    initMissionStoreHandlers,
    getRoot,
    getLogsDir,
    getMissionsDir,
    rebuildIndex,
};
