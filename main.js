const { app, BrowserWindow, ipcMain, Menu, dialog, shell } = require('electron');
// Rolling debug log (last 5 min on disk): first, so every later line is captured
const appLog = require('./app-log');
appLog.init();
const fs = require('fs');
const path = require('path');
const { initMAVLinkHandlers, cleanup: cleanupMAVLink, registerDecodedMessageCallback } = require('./main-mavlink');
const { initSITLHandlers, cleanup: cleanupSITL } = require('./sitl-manager');
const { initRTKHandlers, cleanup: cleanupRTK } = require('./rtk-manager');
const { initFPVHandlers, cleanupFPV } = require('./fpv-manager');
const { initTelForwardHandlers, cleanup: cleanupTelFwd } = require('./telforward-manager');
const { initLogReplayHandlers, cleanup: cleanupLogReplay } = require('./log-replay-manager');
const { initMissionStoreHandlers } = require('./mission-store');
const { initMSPHandlers, cleanup: cleanupMSP } = require('./msp-manager');
const { initLidarHandlers, cleanup: cleanupLidar, onMavlinkMessage: lidarOnMavlink } = require('./lidar-manager');
const { initLteKeyHandlers } = require('./lte-link');

// Hide the application menu (will be set when app is ready)

// Force use of dedicated GPU (NVIDIA/AMD) instead of integrated graphics
app.commandLine.appendSwitch('force_high_performance_gpu');
app.commandLine.appendSwitch('ignore-gpu-blocklist');

// SharedArrayBuffer for the terrain worker pool: each HGT grid is decoded once
// into shared memory and read by the UI thread and every terrain worker
// (TerrainManager.js). Chromium otherwise exposes it only to cross-origin
// isolated pages, which would need COOP/COEP headers here and CORP/CORS on
// every imagery server. The page runs only the app's own code. Chromium reads
// a single enable-features switch: add any other feature to this list.
app.commandLine.appendSwitch('enable-features', 'SharedArrayBuffer');

// IPC handler to list 3D models in the models folder
ipcMain.handle('models-list', async () => {
  const modelsDir = path.join(__dirname, 'models');
  try {
    const entries = await fs.promises.readdir(modelsDir);
    const models = entries.filter(f => /\.(glb|gltf)$/i.test(f)).sort();
    console.log(`models-list: found ${models.length} models`);
    return models;
  } catch (err) {
    console.log('models-list: models folder not found');
    return [];
  }
});

// IPC handler to load a specific model file as ArrayBuffer
ipcMain.handle('models-load', async (event, filename) => {
  const modelsDir = path.join(__dirname, 'models');
  const filePath = path.join(modelsDir, filename);
  // Security: ensure we don't traverse outside models folder
  if (!filePath.startsWith(modelsDir)) {
    console.error('models-load: path traversal attempt blocked');
    return null;
  }
  try {
    const buf = await fs.promises.readFile(filePath);
    const ab = buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength);
    console.log(`models-load: loaded ${filename} (${ab.byteLength} bytes)`);
    return ab;
  } catch (err) {
    console.error(`models-load: failed to load ${filename}`, err.message);
    return null;
  }
});

// ── Terrain tiles (.hgt) ─────────────────────────────────────────────────────
// Packaged, __dirname is inside app.asar: a file, which has no topo/ folder (the
// build leaves the tiles out) and cannot be written to — tiles saved there were
// lost with ENOTDIR. So the tiles live outside the app:
//   - topography/ and topo/ next to the installation (the project folder in
//     development): tiles the operator supplies, read only;
//   - <data root>/terrain (mission-store.js): where downloaded tiles are saved,
//     writable even when the installation is under Program Files.
// Lookups ignore case: the renderer asks for N47E011.HGT, the offline downloader
// saves N47E011.hgt, and Linux file systems tell the two apart.

const HGT_NAME = /^[NS]\d{2}[EW]\d{3}\.hgt$/i;
const HGT_SIZES = new Set([1201 * 1201 * 2, 3601 * 3601 * 2]);   // SRTM3, SRTM1
const HGT_RESCAN_MS = 2000;

function hgtCacheDir() {
  return path.join(require('./mission-store').getRoot(), 'terrain');
}

/** Folders searched for tiles, first match wins: the operator's own, then the cache. */
function hgtDirs() {
  // An AppImage runs from a read-only mount in /tmp: its folder is where the .AppImage file is
  const base = !app.isPackaged ? app.getAppPath()
    : process.env.APPIMAGE ? path.dirname(process.env.APPIMAGE)
    : path.dirname(app.getPath('exe'));
  return [path.join(base, 'topography'), path.join(base, 'topo'), hgtCacheDir()];
}

let hgtIndex = null;     // UPPERCASE name → full path
let hgtIndexAt = 0;

async function scanHgt() {
  const index = new Map();
  const counts = [];
  for (const dir of hgtDirs()) {
    let entries;
    try { entries = await fs.promises.readdir(dir); } catch (e) { continue; }
    let n = 0;
    for (const e of entries) {
      if (!HGT_NAME.test(e)) continue;
      n++;
      const key = e.toUpperCase();
      if (!index.has(key)) index.set(key, path.join(dir, e));
    }
    counts.push(`${n} in ${dir}`);
  }
  hgtIndex = index;
  hgtIndexAt = Date.now();
  return counts;
}

/** Full path of a tile, or null. A miss rescans the folders (a tile copied in by hand). */
async function findHgt(filename) {
  const key = String(filename).toUpperCase();
  if (!hgtIndex) await scanHgt();
  let p = hgtIndex.get(key);
  if (!p && Date.now() - hgtIndexAt > HGT_RESCAN_MS) {
    await scanHgt();
    p = hgtIndex.get(key);
  }
  if (!p) return null;
  try {
    const st = await fs.promises.stat(p);
    if (st.isFile() && st.size > 0) return p;
  } catch (e) { /* deleted since the scan */ }
  hgtIndex.delete(key);
  return null;
}

// List the available tiles (names only, no data — avoids OOM)
ipcMain.handle('topography-load', async () => {
  const counts = await scanHgt();
  console.log(`[hgt] ${hgtIndex.size} tiles available (${counts.join(', ') || 'no terrain folder yet'}); downloads go to ${hgtCacheDir()}`);
  return [...hgtIndex.keys()];
});

// Load a single tile by name (on-demand, lazy)
ipcMain.handle('topography-load-one', async (event, filename) => {
  if (!HGT_NAME.test(filename || '')) return null;
  const full = await findHgt(filename);
  if (!full) return null;
  try {
    const buf = await fs.promises.readFile(full);
    console.log(`[hgt] loaded ${filename} from ${full} (${buf.byteLength} bytes)`);
    return buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength);
  } catch (err) {
    console.error(`[hgt] cannot read ${full}: ${err.message}`);
    return null;
  }
});

// Does a tile exist (cheap stat — no file read). Used by the offline downloader's
// skip check; reading the full ~25 MB file just to test existence is wasteful.
ipcMain.handle('topography-exists', async (event, filename) => {
  if (!HGT_NAME.test(filename || '')) return false;
  return !!(await findHgt(filename));
});

// Save a downloaded tile to the terrain cache. Through a temp file + rename: a
// truncated tile would pass the exists() check and never be downloaded again.
ipcMain.handle('topography-save', async (event, filename, arrayBuffer) => {
  if (!HGT_NAME.test(filename || '')) {
    console.error(`[hgt] refused to save "${filename}": not a tile name`);
    return false;
  }
  const size = arrayBuffer ? arrayBuffer.byteLength : 0;
  if (!HGT_SIZES.has(size)) {
    console.error(`[hgt] refused to save ${filename}: ${size} bytes is not an SRTM tile`);
    return false;
  }
  const dir = hgtCacheDir();
  const filePath = path.join(dir, filename.toUpperCase());
  const tmp = `${filePath}.tmp`;
  try {
    await fs.promises.mkdir(dir, { recursive: true });
    await fs.promises.writeFile(tmp, Buffer.from(arrayBuffer));
    await fs.promises.rename(tmp, filePath);
    if (!hgtIndex) await scanHgt();
    hgtIndex.set(filename.toUpperCase(), filePath);
    console.log(`[hgt] saved ${filename} to ${filePath} (${size} bytes)`);
    return true;
  } catch (err) {
    console.error(`[hgt] failed to save ${filename} to ${dir}: ${err.message}`);
    fs.promises.unlink(tmp).catch(() => {});
    return false;
  }
});

// Renderer error bridge: print uncaught errors and unhandled promise rejections
// from the renderer into this (main) process terminal output.
ipcMain.on('renderer-global-error', (event, payload) => {
  try {
    const header = payload && payload.type ? `[renderer:${payload.type}]` : '[renderer:error]';
    const where = payload && payload.filename ? ` ${payload.filename}${payload.lineno ? `:${payload.lineno}` : ''}${payload.colno ? `:${payload.colno}` : ''}` : '';

    if (payload && payload.type === 'unhandledrejection') {
      const reason = payload.reason || {};
      console.error(`${header}${where} ${reason.message || 'Unhandled promise rejection'}`);
      if (reason.stack) console.error(reason.stack);
      return;
    }

    const err = (payload && payload.error) || {};
    const msg = (payload && payload.message) || err.message || 'Uncaught error';
    console.error(`${header}${where} ${msg}`);
    if (err.stack) console.error(err.stack);
  } catch (e) {
    console.error('[renderer:error] failed to print payload', e);
  }
});

function createWindow() {
  const win = new BrowserWindow({
    width: 1200,
    height: 800,
    frame: false,
    icon: process.platform === 'win32'
      ? path.join(__dirname, 'assets', 'icons', 'icon.ico')
      : path.join(__dirname, 'assets', 'icons', 'icon-256x256.png'),
    webPreferences: {
      contextIsolation: true,
      preload: path.join(__dirname, 'preload.js')
    }
  });

  console.log('[window] created, loading html/index.html');
  win.loadFile(path.join(__dirname, 'html', 'index.html'));

  // Route window.open / target=_blank to the system browser
  win.webContents.setWindowOpenHandler(({ url }) => {
    if (url.startsWith('http://') || url.startsWith('https://')) {
      shell.openExternal(url);
    }
    return { action: 'deny' };
  });

  // Fix frameless window focus: force webContents focus when the OS window is activated
  win.on('focus', () => {
    if (!win.isDestroyed()) win.webContents.focus();
  });

  // Initialize MAVLink handlers for this window
  initMAVLinkHandlers(win);
  // Cellular (LTE relay) link: remembered module key, OS-keychain encrypted
  initLteKeyHandlers();

  // Initialize SITL launcher handlers
  initSITLHandlers(win);

  // Initialize RTK base station handlers
  initRTKHandlers(win);

  // Initialize FPV camera stream handlers
  initFPVHandlers(win);

  // Initialize Telemetry Forward handlers
  initTelForwardHandlers(win);

  // Initialize Log Replay handlers (must come AFTER initMAVLinkHandlers so the
  // replay engine can reuse handlePacket() and mainWindow from main-mavlink)
  initLogReplayHandlers(win);

  // Mission library / data root (creates data/, missions/, logs/ and index.json)
  initMissionStoreHandlers();

  // MSP (INAV / Betaflight) telemetry adapter
  initMSPHandlers(win);

  // Livox Mid-360 point cloud: direct UDP link to the LiDAR, georeferenced
  // with the pose taken straight from the decoded MAVLink stream.
  initLidarHandlers(win);
  registerDecodedMessageCallback(lidarOnMavlink);

  // Renderer console → terminal and debug log; window load/crash/hang events
  appLog.attachWindow(win);

  // IPC handlers for window controls
  ipcMain.on('window-minimize', () => win.minimize());
  ipcMain.on('window-maximize', () => {
    if (win.isMaximized()) win.unmaximize();
    else win.maximize();
  });
  ipcMain.on('window-close', () => {
    console.log('[window] close button');
    win.close();
  });
}

app.whenReady().then(() => {
  // Minimal hidden menu that preserves native keyboard shortcuts for text editing.
  // Setting null removes Cut/Copy/Paste/SelectAll accelerators in Electron frameless windows.
  const editMenu = Menu.buildFromTemplate([{
    label: 'Edit',
    submenu: [
      { role: 'undo' },
      { role: 'redo' },
      { type: 'separator' },
      { role: 'cut' },
      { role: 'copy' },
      { role: 'paste' },
      { role: 'selectAll' }
    ]
  }]);
  Menu.setApplicationMenu(editMenu);
  createWindow();
});

app.on('window-all-closed', () => {
  cleanupLogReplay();
  cleanupMAVLink();
  cleanupSITL();
  cleanupRTK();
  cleanupFPV();
  cleanupTelFwd();
  cleanupMSP();
  cleanupLidar();
  if (process.platform !== 'darwin') app.quit();
});

app.on('activate', () => {
  if (BrowserWindow.getAllWindows().length === 0) createWindow();
});

// TLOG recording is handled entirely in main-mavlink.js

// ── ADS-B fetch via OpenSky Network (bypass CORS from main process) ────
ipcMain.handle('adsb-fetch', async (event, lamin, lomin, lamax, lomax) => {
  const https = require('https');
  const url = `https://opensky-network.org/api/states/all?lamin=${lamin}&lomin=${lomin}&lamax=${lamax}&lomax=${lomax}`;
  try {
    const data = await new Promise((resolve, reject) => {
      const req = https.get(url, { timeout: 15000 }, (res) => {
        let body = '';
        res.on('data', (chunk) => { body += chunk; });
        res.on('end', () => {
          if (res.statusCode !== 200) {
            reject(new Error(`OpenSky HTTP ${res.statusCode}`));
            return;
          }
          try {
            resolve(JSON.parse(body));
          } catch (e) {
            reject(new Error('Invalid JSON from OpenSky'));
          }
        });
      });
      req.on('error', (e) => reject(e));
      req.on('timeout', () => { req.destroy(); reject(new Error('OpenSky timeout')); });
    });
    return data;
  } catch (err) {
    const skipped = appLog.throttle('adsb-fetch', 60000);
    if (skipped >= 0) console.warn(`[adsb] OpenSky fetch failed: ${err.message}${skipped ? ` (+${skipped} more failures)` : ''}`);
    return { states: null, error: err.message };
  }
});

ipcMain.on('devtools-open', (event) => {
  try {
    const wc = event && event.sender;
    if (!wc || wc.isDestroyed()) return;
    wc.openDevTools({ mode: 'detach' });
  } catch (e) {
    console.error('[main] failed to open devtools', e);
  }
});