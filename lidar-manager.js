/**
 * lidar-manager.js - Livox Mid-360 point cloud: main-thread glue
 *
 * The protocol client, the georeferencing and the voxel map live in
 * lidar-core.js, hosted by lidar-worker.js on a worker thread (see the note
 * in lidar-core.js on why not the main thread). This module:
 *   - forks the worker on first use with the data root for the .ply files,
 *   - maps the 'lidar-*' ipcMain channels onto worker commands,
 *   - forwards the decoded MAVLink pose messages (ATTITUDE, GLOBAL_POSITION_INT,
 *     GPS_RAW_INT, EKF_STATUS_REPORT) to the worker while a link is active,
 *   - relays the worker's events ('lidar-points', 'lidar-origin', 'lidar-status')
 *     to the renderer.
 */

const { ipcMain } = require('electron');
const path = require('path');
const { Worker } = require('worker_threads');

let mainWindow = null;
let worker = null;
let active = false;            // a LiDAR link is up: pose messages are worth forwarding
let nextId = 1;
const pending = new Map();

const POSE_MSGS = new Set([24, 30, 33, 193]);

function ensureWorker() {
    if (worker) return worker;
    const { getRoot } = require('./mission-store');
    // Packaged builds keep the worker and its core outside app.asar (see
    // asarUnpack in package.json) so the thread loads them from a real path.
    const base = __dirname.includes('app.asar') ? __dirname.replace('app.asar', 'app.asar.unpacked') : __dirname;
    worker = new Worker(path.join(base, 'lidar-worker.js'), { workerData: { dataRoot: getRoot() } });
    worker.on('message', (m) => {
        if (m.id !== undefined) {
            const p = pending.get(m.id);
            if (!p) return;
            pending.delete(m.id);
            if (m.error) p.reject(new Error(m.error)); else p.resolve(m.result);
            return;
        }
        if (m.ch && mainWindow && !mainWindow.isDestroyed()) mainWindow.webContents.send(m.ch, m.d);
    });
    worker.on('error', (e) => {
        console.error('[lidar] worker error:', e.message);
        for (const p of pending.values()) p.reject(e);
        pending.clear();
    });
    worker.on('exit', (code) => {
        console.log(`[lidar] worker exited (${code})`);
        worker = null;
        active = false;
    });
    return worker;
}

function call(cmd, ...args) {
    const w = ensureWorker();
    return new Promise((resolve, reject) => {
        const id = nextId++;
        pending.set(id, { resolve, reject });
        w.postMessage({ id, cmd, args });
    });
}

// Decoded-message tap registered with main-mavlink.js.
function onMavlinkMessage(msgId, data) {
    if (!active || !worker || !POSE_MSGS.has(msgId)) return;
    worker.postMessage({ mav: true, msgId, data });
}

function initLidarHandlers(win) {
    mainWindow = win;

    ipcMain.handle('lidar-connect', async (_e, cfg) => {
        const res = await call('connect', cfg);
        active = !!(res && res.success);
        return res;
    });
    ipcMain.handle('lidar-disconnect', async () => {
        active = false;
        return worker ? call('disconnect') : { success: true };
    });
    ipcMain.handle('lidar-set-config', async (_e, cfg) => call('setConfig', cfg));
    ipcMain.handle('lidar-get-status', async () => call('getStatus'));
    ipcMain.handle('lidar-list-interfaces', async () => call('listInterfaces'));
    ipcMain.handle('lidar-clear', async () => call('clear'));
    ipcMain.handle('lidar-save-map', async () => call('saveMap'));
    ipcMain.handle('lidar-get-dir', async () => call('getDir'));
    ipcMain.handle('lidar-resync', async () => call('resync'));
}

function cleanup() {
    active = false;
    if (worker) {
        const w = worker;
        worker = null;
        w.postMessage({ id: nextId++, cmd: 'disconnect', args: [] });
        setTimeout(() => { try { w.terminate(); } catch (_) {} }, 200);
    }
}

module.exports = { initLidarHandlers, cleanup, onMavlinkMessage };
