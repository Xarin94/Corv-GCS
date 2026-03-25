/**
 * fpv-manager.js - FPV Video Stream Manager (Main Process)
 *
 * Spawns VLC to transcode the SIYI HM30 RTSP stream to MJPEG HTTP on
 * localhost:8191. The main process connects to that stream, parses JPEG
 * frames, and forwards them to the renderer via IPC.
 *
 * Default SIYI HM30 settings:
 *   IP: 192.168.144.25, Port: 8554, Path: /main.264
 */

const { ipcMain } = require('electron');
const { spawn } = require('child_process');
const http = require('http');

let vlcProcess = null;
let httpRequest = null;
let mainWindow = null;
let isStreaming = false;

const VLC_PATH = 'C:/Program Files/VideoLAN/VLC/vlc.exe';
const LOCAL_PORT = 8191;

// ============== MJPEG FRAME PARSER ==============
const JPEG_SOI = Buffer.from([0xFF, 0xD8]);
const JPEG_EOI = Buffer.from([0xFF, 0xD9]);

class MJPEGParser {
  constructor(onFrame) {
    this.buffer = Buffer.alloc(0);
    this.onFrame = onFrame;
  }

  push(chunk) {
    this.buffer = Buffer.concat([this.buffer, chunk]);
    this._extract();
  }

  _extract() {
    while (true) {
      const soiIdx = this._indexOf(this.buffer, JPEG_SOI, 0);
      if (soiIdx === -1) { this.buffer = Buffer.alloc(0); return; }
      if (soiIdx > 0) this.buffer = this.buffer.subarray(soiIdx);

      const eoiIdx = this._indexOf(this.buffer, JPEG_EOI, 2);
      if (eoiIdx === -1) return;

      const frame = this.buffer.subarray(0, eoiIdx + 2);
      this.buffer = this.buffer.subarray(eoiIdx + 2);
      this.onFrame(frame);
    }
  }

  _indexOf(buf, search, from) {
    for (let i = from; i <= buf.length - search.length; i++) {
      if (buf[i] === search[0] && buf[i + 1] === search[1]) return i;
    }
    return -1;
  }

  reset() { this.buffer = Buffer.alloc(0); }
}

// ============== CONNECT TO VLC HTTP STREAM ==============
function connectToStream(retries = 20) {
  if (!vlcProcess || !isStreaming) return;

  const url = `http://127.0.0.1:${LOCAL_PORT}/`;

  const parser = new MJPEGParser((frame) => {
    if (mainWindow && !mainWindow.isDestroyed()) {
      mainWindow.webContents.send('fpv-frame', frame.toString('base64'));
    }
  });

  httpRequest = http.get(url, (res) => {
    console.log(`[FPV] Connected to VLC stream (${res.statusCode})`);

    if (mainWindow && !mainWindow.isDestroyed()) {
      mainWindow.webContents.send('fpv-status', { connected: true });
    }

    res.on('data', (chunk) => parser.push(chunk));

    res.on('end', () => {
      console.log('[FPV] VLC HTTP stream ended');
      httpRequest = null;
    });

    res.on('error', (err) => {
      console.error('[FPV] Stream read error:', err.message);
      httpRequest = null;
    });
  });

  httpRequest.on('error', (err) => {
    httpRequest = null;
    if (retries > 0 && vlcProcess && isStreaming) {
      console.log(`[FPV] VLC HTTP not ready, retrying... (${retries} left)`);
      setTimeout(() => connectToStream(retries - 1), 500);
    } else {
      console.error('[FPV] Cannot connect to VLC stream:', err.message);
      if (mainWindow && !mainWindow.isDestroyed()) {
        mainWindow.webContents.send('fpv-error', 'VLC stream unreachable: ' + err.message);
      }
    }
  });
}

// ============== START / STOP ==============
function startStream(ip, port, rtspPath, options = {}) {
  if (vlcProcess) stopStream();

  const fps = options.fps || 30;
  const rtspUrl = `rtsp://${ip}:${port}${rtspPath}`;
  console.log(`[FPV] Starting VLC: ${rtspUrl}`);

  const sout = `#transcode{vcodec=MJPG,fps=${fps}}:standard{access=http,mux=mpjpeg,dst=:${LOCAL_PORT}}`;

  vlcProcess = spawn(VLC_PATH, [
    '-I', 'dummy',
    '--no-video-title-show',
    '--no-sout-rtp-sap',
    '--no-sout-standard-sap',
    rtspUrl,
    '--sout', sout,
  ], { stdio: ['ignore', 'ignore', 'pipe'] });

  vlcProcess.stderr.on('data', (data) => {
    const msg = data.toString().trim();
    if (msg) console.log(`[FPV] VLC: ${msg}`);
  });

  vlcProcess.on('close', (code) => {
    console.log(`[FPV] VLC exited (${code})`);
    vlcProcess = null;
    isStreaming = false;
    if (httpRequest) { try { httpRequest.destroy(); } catch (_) {} httpRequest = null; }
    if (mainWindow && !mainWindow.isDestroyed()) {
      mainWindow.webContents.send('fpv-status', { connected: false, code });
    }
  });

  vlcProcess.on('error', (err) => {
    console.error('[FPV] VLC spawn error:', err.message);
    vlcProcess = null;
    isStreaming = false;
    if (mainWindow && !mainWindow.isDestroyed()) {
      mainWindow.webContents.send('fpv-error', 'VLC not found: ' + err.message);
      mainWindow.webContents.send('fpv-status', { connected: false });
    }
  });

  isStreaming = true;

  // Start polling VLC's HTTP server (retry up to 20 times every 500ms = 10s max)
  setTimeout(() => connectToStream(20), 1000);
}

function stopStream() {
  if (httpRequest) {
    try { httpRequest.destroy(); } catch (_) {}
    httpRequest = null;
  }
  if (vlcProcess) {
    console.log('[FPV] Stopping VLC');
    try { vlcProcess.kill('SIGTERM'); } catch (_) {}
    const proc = vlcProcess;
    setTimeout(() => { try { if (!proc.killed) proc.kill('SIGKILL'); } catch (_) {} }, 2000);
    vlcProcess = null;
  }
  isStreaming = false;
}

// ============== IPC HANDLERS ==============
function initFPVHandlers(win) {
  mainWindow = win;

  ipcMain.handle('fpv-start', async (event, ip, port, rtspPath, options) => {
    try {
      startStream(ip, port, rtspPath, options);
      return { success: true };
    } catch (err) {
      return { success: false, error: err.message };
    }
  });

  ipcMain.handle('fpv-stop', async () => {
    stopStream();
    return { success: true };
  });

  ipcMain.handle('fpv-status', async () => {
    return { streaming: isStreaming };
  });
}

function cleanupFPV() {
  stopStream();
  mainWindow = null;
}

module.exports = { initFPVHandlers, cleanupFPV };
