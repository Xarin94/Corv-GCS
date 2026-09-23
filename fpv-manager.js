/**
 * fpv-manager.js - FPV Video Stream Manager (Main Process)
 *
 * Preferred path: rtsp-client.js pulls the camera's H.264/H.265 stream and
 * the access units go straight to the renderer, which decodes them in
 * hardware with WebCodecs. No transcoding anywhere.
 *
 * Fallback: spawn VLC to transcode the RTSP stream to MJPEG HTTP on
 * localhost:8191, parse the JPEG frames and forward them via IPC. Used when
 * the renderer has no decoder for the camera's codec, the camera cannot be
 * reached by the native client, or decoding fails.
 *
 * Default SIYI HM30 settings:
 *   IP: 192.168.144.25, Port: 8554, Path: /main.264
 */

const { ipcMain } = require('electron');
const { spawn } = require('child_process');
const http = require('http');
const { RtspClient } = require('./rtsp-client');

let vlcProcess = null;
let httpRequest = null;
let mainWindow = null;
let isStreaming = false;
let rtspClient = null;
let lastStart = null;   // { rtspUrl, options } - for the fallback switch

// A native stream that produces no frame this long after PLAY is given up (e.g.
// UDP blocked by a firewall) in favour of VLC.
const FIRST_FRAME_TIMEOUT_MS = 6000;

function send(channel, payload) {
  if (mainWindow && !mainWindow.isDestroyed()) mainWindow.webContents.send(channel, payload);
}

const VLC_PATH = 'C:/Program Files/VideoLAN/VLC/vlc.exe';
const LOCAL_PORT = 8191;

// ============== MJPEG FRAME PARSER ==============
const JPEG_SOI = Buffer.from([0xFF, 0xD8]);
const JPEG_EOI = Buffer.from([0xFF, 0xD9]);

// Native Buffer.indexOf, and the EOI search resumes where the previous chunk
// left off: rescanning the whole partial frame byte by byte in JS on every
// network chunk was quadratic in the frame size.
class MJPEGParser {
  constructor(onFrame) {
    this.onFrame = onFrame;
    this.reset();
  }

  push(chunk) {
    this.buffer = this.buffer.length ? Buffer.concat([this.buffer, chunk]) : chunk;
    this._extract();
  }

  _extract() {
    while (true) {
      if (!this.inFrame) {
        const soiIdx = this.buffer.indexOf(JPEG_SOI);
        if (soiIdx === -1) {
          // Keep a trailing 0xFF: it may be the first half of an SOI split across chunks
          const last = this.buffer.length - 1;
          this.buffer = last >= 0 && this.buffer[last] === 0xFF ? this.buffer.subarray(last) : Buffer.alloc(0);
          return;
        }
        this.buffer = this.buffer.subarray(soiIdx);
        this.inFrame = true;
        this.scanFrom = 2;
      }

      const eoiIdx = this.buffer.indexOf(JPEG_EOI, this.scanFrom);
      if (eoiIdx === -1) {
        // Resume from the last byte next time: the EOI may straddle two chunks
        this.scanFrom = Math.max(2, this.buffer.length - 1);
        return;
      }

      const frame = this.buffer.subarray(0, eoiIdx + 2);
      this.buffer = this.buffer.subarray(eoiIdx + 2);
      this.inFrame = false;
      this.onFrame(frame);
    }
  }

  reset() {
    this.buffer = Buffer.alloc(0);
    this.inFrame = false;
    this.scanFrom = 2;
  }
}

// ============== CONNECT TO VLC HTTP STREAM ==============
function connectToStream(retries = 20) {
  if (!vlcProcess || !isStreaming) return;

  const url = `http://127.0.0.1:${LOCAL_PORT}/`;

  const parser = new MJPEGParser((frame) => {
    if (mainWindow && !mainWindow.isDestroyed()) {
      // Raw JPEG bytes (the renderer decodes them off-thread with createImageBitmap).
      // Base64 cost an encode here, a third more IPC traffic and a data-URL
      // decode in the renderer. The copy gives the frame its own exact-size
      // ArrayBuffer: IPC serializes a typed array's whole backing store.
      mainWindow.webContents.send('fpv-frame', new Uint8Array(frame));
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

// ============== NATIVE RTSP -> WEBCODECS ==============
/**
 * Start the native client. Resolves once PLAY succeeded; rejects if the camera
 * cannot be reached this way or its codec is not in `decoders` (the codecs the
 * renderer reported it can decode).
 */
async function startNative(rtspUrl, decoders) {
  const client = new RtspClient(rtspUrl, { supportedCodecs: decoders });
  rtspClient = client;
  let gotFrame = false;
  let watchdog = null;

  client.on('config', (cfg) => {
    console.log(`[FPV] Native stream: ${cfg.codec} over ${client.transport.toUpperCase()}`);
    send('fpv-video-config', cfg);
  });
  client.on('frame', (f) => {
    if (!gotFrame) {
      gotFrame = true;
      clearTimeout(watchdog);
      send('fpv-status', { connected: true, transport: 'webcodecs' });
    }
    // IPC serializes a typed array's whole backing store; small access units
    // can live in Node's shared 8 KB pool, so give those their own buffer.
    const d = f.data;
    const data = (d.byteOffset === 0 && d.byteLength === d.buffer.byteLength) ? d : new Uint8Array(d);
    send('fpv-video-chunk', { key: f.key, timestamp: f.timestamp, data });
  });
  client.on('close', (err) => {
    clearTimeout(watchdog);
    if (rtspClient !== client) return;   // replaced or stopped on purpose
    rtspClient = null;
    isStreaming = false;
    console.log(`[FPV] Native stream closed${err ? ': ' + err.message : ''}`);
    send('fpv-status', { connected: false });
  });

  try {
    await client.start();
  } catch (err) {
    if (rtspClient === client) rtspClient = null;
    client.stop();
    throw err;
  }
  watchdog = setTimeout(() => {
    if (gotFrame || rtspClient !== client) return;
    console.warn('[FPV] Native stream sent no frame - switching to VLC');
    switchToVlc();
  }, FIRST_FRAME_TIMEOUT_MS);
}

function stopNative() {
  if (!rtspClient) return;
  const client = rtspClient;
  rtspClient = null;   // before stop(): its 'close' must not report a disconnect
  client.stop();
}

function switchToVlc() {
  if (!lastStart) return;
  stopNative();
  startVlc(lastStart.rtspUrl, lastStart.options);
}

// ============== START / STOP ==============
async function startStream(ip, port, rtspPath, options = {}) {
  stopStream();

  const rtspUrl = `rtsp://${ip}:${port}${rtspPath}`;
  const startToken = lastStart = { rtspUrl, options };
  isStreaming = true;

  const decoders = Array.isArray(options.decoders) ? options.decoders : [];
  if (decoders.length) {
    try {
      console.log(`[FPV] Starting native RTSP: ${rtspUrl}`);
      await startNative(rtspUrl, decoders);
      return 'webcodecs';
    } catch (err) {
      // Stopped (or restarted) while connecting: nothing to fall back to
      if (lastStart !== startToken || !isStreaming) return 'stopped';
      console.warn(`[FPV] Native RTSP unavailable (${err.message}) - falling back to VLC`);
    }
  }
  startVlc(rtspUrl, options);
  return 'mjpeg';
}

function startVlc(rtspUrl, options = {}) {
  if (vlcProcess) return;
  const fps = options.fps || 30;
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
  stopNative();
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
      const transport = await startStream(ip, port, rtspPath, options);
      return { success: true, transport };
    } catch (err) {
      return { success: false, error: err.message };
    }
  });

  ipcMain.handle('fpv-fallback', async () => {
    if (!rtspClient) return { success: false };
    console.warn('[FPV] Renderer cannot decode the native stream - switching to VLC');
    switchToVlc();
    return { success: true };
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
