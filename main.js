const { app, BrowserWindow, ipcMain, Menu, dialog } = require('electron');
const fs = require('fs');
const path = require('path');
const { initMAVLinkHandlers, cleanup: cleanupMAVLink } = require('./main-mavlink');
const { initSITLHandlers, cleanup: cleanupSITL } = require('./sitl-manager');
const { initRTKHandlers, cleanup: cleanupRTK } = require('./rtk-manager');
const { initFPVHandlers, cleanupFPV } = require('./fpv-manager');
const { initTelForwardHandlers, cleanup: cleanupTelFwd } = require('./telforward-manager');

// Hide the application menu (will be set when app is ready)

// Force use of dedicated GPU (NVIDIA/AMD) instead of integrated graphics
app.commandLine.appendSwitch('force_high_performance_gpu');
app.commandLine.appendSwitch('ignore-gpu-blocklist');

// IPC handler to list 3D models in the models folder
ipcMain.handle('models-list', async () => {
  const modelsDir = path.join(__dirname, 'models');
  try {
    const entries = await fs.promises.readdir(modelsDir);
    const models = entries.filter(f => /\.(glb|gltf)$/i.test(f));
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

// IPC handler to load .hgt files from the app folder (safely in main process)
ipcMain.handle('topography-load', async (event, folderName = 'topography') => {
  const candidates = [folderName, 'topo'];
  const base = __dirname; // folder where main.js sits

  for (const cand of candidates) {
    const dir = path.join(base, cand);
    try {
      const st = await fs.promises.stat(dir);
      if (!st.isDirectory()) continue;
    } catch (err) {
      continue;
    }

    const entries = await fs.promises.readdir(dir);
    const out = [];
    for (const ent of entries) {
      if (ent.toLowerCase().endsWith('.hgt')) {
        const full = path.join(dir, ent);
        try {
          const buf = await fs.promises.readFile(full);
          const ab = buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength);
          out.push({ name: ent.toUpperCase(), arrayBuffer: ab });
        } catch (err) {
          // ignore unreadable
        }
      }
    }

    console.log(`topography-load: found ${out.length} files in ${dir}`);
    return out;
  }

  console.log('topography-load: no folder found');
  return [];
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

  win.loadFile(path.join(__dirname, 'html', 'index.html'));

  // Fix frameless window focus: force webContents focus when the OS window is activated
  win.on('focus', () => {
    if (!win.isDestroyed()) win.webContents.focus();
  });

  // Initialize MAVLink handlers for this window
  initMAVLinkHandlers(win);

  // Initialize SITL launcher handlers
  initSITLHandlers(win);

  // Initialize RTK base station handlers
  initRTKHandlers(win);

  // Initialize FPV camera stream handlers
  initFPVHandlers(win);

  // Initialize Telemetry Forward handlers
  initTelForwardHandlers(win);

  // Forward renderer console.* messages to the terminal (PowerShell) so we can debug
  // without opening DevTools.
  win.webContents.on('console-message', (event) => {
    // Electron 39+: the event object includes WebContentsConsoleMessageEventParams.
    const level = event.level;
    const message = event.message;
    const line = event.lineNumber;
    const sourceId = event.sourceId;

    const prefix = `[renderer]${sourceId ? ` ${sourceId}` : ''}${line ? `:${line}` : ''}`;
    const text = `${prefix} ${message}`;

    // Chromium levels are numeric; map them to Node/Electron console methods.
    // (0=log, 1=warning, 2=error, 3=debug; other values can appear depending on Chromium)
    if (level === 2) console.error(text);
    else if (level === 1) console.warn(text);
    else if (level === 3) console.debug(text);
    else console.log(text);
  });

  // IPC handlers for window controls
  ipcMain.on('window-minimize', () => win.minimize());
  ipcMain.on('window-maximize', () => {
    if (win.isMaximized()) win.unmaximize();
    else win.maximize();
  });
  ipcMain.on('window-close', () => win.close());
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
  cleanupMAVLink();
  cleanupSITL();
  cleanupRTK();
  cleanupFPV();
  cleanupTelFwd();
  if (process.platform !== 'darwin') app.quit();
});

app.on('activate', () => {
  if (BrowserWindow.getAllWindows().length === 0) createWindow();
});

// ── CRV telemetry recording ────────────────────────────────────────────
let activeCRVStream = null;
let activeCRVPath = null;

function getCRVLogsDir() {
  const dir = path.join(app.getPath('userData'), 'logs');
  if (!fs.existsSync(dir)) {
    fs.mkdirSync(dir, { recursive: true });
  }
  return dir;
}

ipcMain.handle('crv-get-logs-dir', () => {
  return getCRVLogsDir();
});

ipcMain.handle('crv-start-recording', async () => {
  const logsDir = getCRVLogsDir();
  const now = new Date();
  const stamp = now.getFullYear().toString()
    + String(now.getMonth() + 1).padStart(2, '0')
    + String(now.getDate()).padStart(2, '0')
    + '_' + String(now.getHours()).padStart(2, '0')
    + String(now.getMinutes()).padStart(2, '0')
    + String(now.getSeconds()).padStart(2, '0');
  const filePath = path.join(logsDir, `flight_${stamp}.crv`);
  activeCRVStream = fs.createWriteStream(filePath);
  activeCRVPath = filePath;
  console.log(`[CRV] recording to ${filePath}`);
  return { success: true, filePath };
});

ipcMain.handle('crv-write-chunk', async (event, arrayBuffer) => {
  if (!activeCRVStream) return false;
  activeCRVStream.write(Buffer.from(arrayBuffer));
  return true;
});

ipcMain.handle('crv-stop-recording', async () => {
  if (activeCRVStream) {
    activeCRVStream.end();
    activeCRVStream = null;
  }
  const p = activeCRVPath;
  activeCRVPath = null;
  console.log(`[CRV] recording stopped${p ? ': ' + p : ''}`);
  return { filePath: p };
});

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