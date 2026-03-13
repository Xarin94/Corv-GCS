/**
 * fpv-manager.js - FPV Video Stream Manager (Main Process)
 *
 * Spawns ffmpeg to convert RTSP stream from SIYI HM30 camera
 * to MJPEG frames, sent via IPC to the renderer process.
 *
 * Default SIYI HM30 settings:
 *   IP: 192.168.144.25
 *   Port: 8554
 *   Path: /main.264
 *   RTSP URL: rtsp://192.168.144.25:8554/main.264
 */

const { ipcMain } = require('electron');
const { spawn } = require('child_process');

let ffmpegProcess = null;
let mainWindow = null;
let isStreaming = false;

// JPEG markers
const JPEG_SOI = Buffer.from([0xFF, 0xD8]);
const JPEG_EOI = Buffer.from([0xFF, 0xD9]);

/**
 * Parse MJPEG stream to extract individual JPEG frames.
 * Buffers incoming data and emits complete JPEG frames.
 */
class MJPEGParser {
  constructor(onFrame) {
    this.buffer = Buffer.alloc(0);
    this.onFrame = onFrame;
    this.frameCount = 0;
  }

  push(chunk) {
    this.buffer = Buffer.concat([this.buffer, chunk]);
    this._extractFrames();
  }

  _extractFrames() {
    while (true) {
      // Find SOI marker
      const soiIdx = this._indexOf(this.buffer, JPEG_SOI, 0);
      if (soiIdx === -1) {
        // No SOI found, discard everything
        this.buffer = Buffer.alloc(0);
        return;
      }

      // Discard data before SOI
      if (soiIdx > 0) {
        this.buffer = this.buffer.subarray(soiIdx);
      }

      // Find EOI marker (start searching after SOI)
      const eoiIdx = this._indexOf(this.buffer, JPEG_EOI, 2);
      if (eoiIdx === -1) {
        // Incomplete frame, wait for more data
        return;
      }

      // Extract complete JPEG frame (SOI to EOI inclusive)
      const frameEnd = eoiIdx + 2;
      const frame = this.buffer.subarray(0, frameEnd);
      this.buffer = this.buffer.subarray(frameEnd);

      this.frameCount++;
      this.onFrame(frame);
    }
  }

  _indexOf(buf, search, fromIndex) {
    for (let i = fromIndex; i <= buf.length - search.length; i++) {
      let found = true;
      for (let j = 0; j < search.length; j++) {
        if (buf[i + j] !== search[j]) {
          found = false;
          break;
        }
      }
      if (found) return i;
    }
    return -1;
  }

  reset() {
    this.buffer = Buffer.alloc(0);
    this.frameCount = 0;
  }
}

/**
 * Start the FPV stream by spawning ffmpeg.
 * @param {string} ip - Camera IP address
 * @param {number} port - RTSP port
 * @param {string} path - RTSP path (e.g., /main.264)
 * @param {object} options - Additional options
 * @param {number} options.fps - Target frame rate (default 30)
 * @param {number} options.quality - JPEG quality 1-31, lower=better (default 5)
 * @param {string} options.resolution - Output resolution (default: source)
 */
function startStream(ip, port, path, options = {}) {
  if (ffmpegProcess) {
    stopStream();
  }

  const fps = options.fps || 30;
  const quality = options.quality || 5;
  const resolution = options.resolution || null;

  const rtspUrl = `rtsp://${ip}:${port}${path}`;
  console.log(`[FPV] Starting stream: ${rtspUrl}`);

  const args = [
    '-rtsp_transport', 'tcp',
    '-i', rtspUrl,
    '-f', 'image2pipe',
    '-vcodec', 'mjpeg',
    '-q:v', String(quality),
    '-r', String(fps),
  ];

  if (resolution) {
    args.push('-s', resolution);
  }

  // Output to stdout
  args.push('-');

  ffmpegProcess = spawn('ffmpeg', args, {
    stdio: ['ignore', 'pipe', 'pipe']
  });

  const parser = new MJPEGParser((frame) => {
    if (mainWindow && !mainWindow.isDestroyed()) {
      // Send JPEG frame as base64 to avoid ArrayBuffer serialization overhead
      mainWindow.webContents.send('fpv-frame', frame.toString('base64'));
    }
  });

  ffmpegProcess.stdout.on('data', (chunk) => {
    parser.push(chunk);
  });

  ffmpegProcess.stderr.on('data', (data) => {
    const msg = data.toString().trim();
    // Only log important messages, not the verbose ffmpeg output
    if (msg.includes('Error') || msg.includes('error') || msg.includes('Connection refused')) {
      console.error(`[FPV] ffmpeg: ${msg}`);
      if (mainWindow && !mainWindow.isDestroyed()) {
        mainWindow.webContents.send('fpv-error', msg);
      }
    }
  });

  ffmpegProcess.on('close', (code) => {
    console.log(`[FPV] ffmpeg exited with code ${code}`);
    ffmpegProcess = null;
    isStreaming = false;
    if (mainWindow && !mainWindow.isDestroyed()) {
      mainWindow.webContents.send('fpv-status', { connected: false, code });
    }
  });

  ffmpegProcess.on('error', (err) => {
    console.error(`[FPV] ffmpeg spawn error:`, err.message);
    ffmpegProcess = null;
    isStreaming = false;
    if (mainWindow && !mainWindow.isDestroyed()) {
      mainWindow.webContents.send('fpv-error', `Failed to start ffmpeg: ${err.message}`);
      mainWindow.webContents.send('fpv-status', { connected: false, error: err.message });
    }
  });

  isStreaming = true;

  // Notify renderer that stream is starting
  if (mainWindow && !mainWindow.isDestroyed()) {
    mainWindow.webContents.send('fpv-status', { connected: true, url: rtspUrl });
  }

  return true;
}

/**
 * Stop the FPV stream.
 */
function stopStream() {
  if (ffmpegProcess) {
    console.log('[FPV] Stopping stream');
    try {
      ffmpegProcess.kill('SIGTERM');
      // Force kill after 2 seconds if still running
      const proc = ffmpegProcess;
      setTimeout(() => {
        try { if (proc && !proc.killed) proc.kill('SIGKILL'); } catch (_) {}
      }, 2000);
    } catch (_) {}
    ffmpegProcess = null;
  }
  isStreaming = false;
}

/**
 * Initialize FPV IPC handlers for a given BrowserWindow.
 */
function initFPVHandlers(win) {
  mainWindow = win;

  ipcMain.handle('fpv-start', async (event, ip, port, path, options) => {
    try {
      startStream(ip, port, path, options);
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

/**
 * Cleanup FPV resources.
 */
function cleanupFPV() {
  stopStream();
  mainWindow = null;
}

module.exports = { initFPVHandlers, cleanupFPV };
