const { contextBridge, ipcRenderer } = require('electron');

function safeToString(value) {
  try {
    if (value instanceof Error) return value.stack || value.message || String(value);
    if (typeof value === 'string') return value;
    if (value === null) return 'null';
    if (value === undefined) return 'undefined';
    return JSON.stringify(value);
  } catch {
    try {
      return String(value);
    } catch {
      return '[unstringifiable]';
    }
  }
}

function serializeErrorLike(err) {
  if (!err) return { message: 'Unknown error' };
  if (err instanceof Error) {
    return {
      name: err.name,
      message: err.message,
      stack: err.stack
    };
  }

  // Some libraries throw plain objects/strings
  return {
    message: safeToString(err)
  };
}

// Window control APIs
contextBridge.exposeInMainWorld('windowControls', {
  minimize: () => ipcRenderer.send('window-minimize'),
  maximize: () => ipcRenderer.send('window-maximize'),
  close: () => ipcRenderer.send('window-close')
});

// Debug helpers (kept explicit due to contextIsolation)
contextBridge.exposeInMainWorld('devtools', {
  open: () => ipcRenderer.send('devtools-open')
});

// Preload bridge: call the main process to read .hgt files (avoid using fs in the preload script)
contextBridge.exposeInMainWorld('topography', {
  load: async (folderName = 'topography') => {
    try {
      // Ask main process to load files; returns array of { name, arrayBuffer }
      const files = await ipcRenderer.invoke('topography-load', folderName);
      console.debug('topography.load: main returned', files && files.length ? files.length : 0, 'files');
      return files || [];
    } catch (e) {
      console.debug('topography.load: ipc invoke failed', e);
      return [];
    }
  },
  save: async (filename, arrayBuffer) => {
    try {
      return await ipcRenderer.invoke('topography-save', filename, arrayBuffer);
    } catch (e) {
      console.debug('topography.save: ipc invoke failed', e);
      return false;
    }
  }
});

// Preload bridge: list and load 3D models from the models folder
contextBridge.exposeInMainWorld('models', {
  list: async () => {
    try {
      const models = await ipcRenderer.invoke('models-list');
      return models || [];
    } catch (e) {
      console.debug('models.list: ipc invoke failed', e);
      return [];
    }
  },
  load: async (filename) => {
    try {
      const ab = await ipcRenderer.invoke('models-load', filename);
      return ab;
    } catch (e) {
      console.debug('models.load: ipc invoke failed', e);
      return null;
    }
  }
});

// RTK Base Station API bridge
contextBridge.exposeInMainWorld('rtk', {
  listPorts: () => ipcRenderer.invoke('rtk-list-ports'),
  connect: (portPath, baudRate) => ipcRenderer.invoke('rtk-connect', portPath, baudRate),
  disconnect: () => ipcRenderer.invoke('rtk-disconnect'),
  getStats: () => ipcRenderer.invoke('rtk-get-stats'),
  getTypeNames: () => ipcRenderer.invoke('rtk-get-type-names'),
  onStatusUpdate: (callback) => ipcRenderer.on('rtk-status-update', (event, data) => callback(data)),
  onInjectRTCM: (callback) => ipcRenderer.on('rtk-inject-rtcm', (event, data) => callback(data))
});

// Telemetry Forward API bridge
contextBridge.exposeInMainWorld('telForward', {
  listPorts: () => ipcRenderer.invoke('telfwd-list-ports'),
  connect: (portPath, baudRate, protocol) => ipcRenderer.invoke('telfwd-connect', portPath, baudRate, protocol),
  disconnect: () => ipcRenderer.invoke('telfwd-disconnect'),
  getStats: () => ipcRenderer.invoke('telfwd-get-stats'),
  feedState: (snapshot) => ipcRenderer.invoke('telfwd-feed-state', snapshot),
  onStatusUpdate: (cb) => ipcRenderer.on('telfwd-status-update', (e, d) => cb(d))
});

// SITL launcher API bridge
contextBridge.exposeInMainWorld('sitl', {
  getOptions: () => ipcRenderer.invoke('sitl-get-options'),
  checkBinary: (vehicle, version) => ipcRenderer.invoke('sitl-check-binary', vehicle, version),
  download: (vehicle, version) => ipcRenderer.invoke('sitl-download', vehicle, version),
  launch: (vehicle, version, options) => ipcRenderer.invoke('sitl-launch', vehicle, version, options),
  stop: () => ipcRenderer.invoke('sitl-stop'),
  status: () => ipcRenderer.invoke('sitl-status'),
  onStatusUpdate: (callback) => ipcRenderer.on('sitl-status-update', (event, data) => callback(data))
});

// CORV Binary serial API bridge (parsing happens in main process)
contextBridge.exposeInMainWorld('corvSerial', {
  connect: (portPath, baudRate) => ipcRenderer.invoke('corv-connect-serial', portPath, baudRate),
  disconnect: () => ipcRenderer.invoke('mavlink-disconnect')
});

// MAVLink API bridge
contextBridge.exposeInMainWorld('mavlink', {
  connectSerial: (portPath, baudRate) => ipcRenderer.invoke('mavlink-connect-serial', portPath, baudRate),
  connectUDP: (host, port) => ipcRenderer.invoke('mavlink-connect-udp', host, port),
  connectTCP: (host, port) => ipcRenderer.invoke('mavlink-connect-tcp', host, port),
  disconnect: () => ipcRenderer.invoke('mavlink-disconnect'),
  sendCommand: (cmd) => ipcRenderer.invoke('mavlink-send-command', cmd),
  sendMessage: (msg) => ipcRenderer.invoke('mavlink-send-message', msg),
  setGcsMuted: (muted) => ipcRenderer.invoke('mavlink-set-gcs-muted', muted),
  onMessage: (callback) => ipcRenderer.on('mavlink-message', (event, msg) => callback(msg)),
  onConnectionState: (callback) => ipcRenderer.on('mavlink-connection-state', (event, state) => callback(state)),
  listPorts: () => ipcRenderer.invoke('serial-list-ports')
});

// FPV camera stream API bridge (SIYI HM30 / generic RTSP)
contextBridge.exposeInMainWorld('fpv', {
  start: (ip, port, path, options) => ipcRenderer.invoke('fpv-start', ip, port, path, options),
  stop: () => ipcRenderer.invoke('fpv-stop'),
  status: () => ipcRenderer.invoke('fpv-status'),
  onFrame: (callback) => ipcRenderer.on('fpv-frame', (event, base64) => callback(base64)),
  onError: (callback) => ipcRenderer.on('fpv-error', (event, msg) => callback(msg)),
  onStatus: (callback) => ipcRenderer.on('fpv-status', (event, status) => callback(status))
});

// ADS-B fetch API bridge (via main process to bypass CORS)
contextBridge.exposeInMainWorld('adsb', {
  fetch: (lamin, lomin, lamax, lomax) => ipcRenderer.invoke('adsb-fetch', lamin, lomin, lamax, lomax)
});

// CRV telemetry recording API bridge
contextBridge.exposeInMainWorld('crvLogger', {
  startRecording: () => ipcRenderer.invoke('crv-start-recording'),
  writeChunk: (arrayBuffer) => ipcRenderer.invoke('crv-write-chunk', arrayBuffer),
  stopRecording: () => ipcRenderer.invoke('crv-stop-recording'),
  getLogsDir: () => ipcRenderer.invoke('crv-get-logs-dir')
});

// Forward uncaught errors / unhandled promise rejections to the main process
// so they appear in the PowerShell terminal.
window.addEventListener('error', (event) => {
  const payload = {
    type: 'error',
    message: event.message,
    filename: event.filename,
    lineno: event.lineno,
    colno: event.colno,
    error: serializeErrorLike(event.error)
  };
  ipcRenderer.send('renderer-global-error', payload);
});

window.addEventListener('unhandledrejection', (event) => {
  const payload = {
    type: 'unhandledrejection',
    reason: serializeErrorLike(event.reason)
  };
  ipcRenderer.send('renderer-global-error', payload);
});