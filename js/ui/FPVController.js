/**
 * FPVController.js - FPV Camera Stream Controller
 *
 * 3-state button:
 *   Click 1: start stream → CAMERA mode (camera only, full screen)
 *   Click 2: switch to    → AR mode (camera + 50% transparent 3D, terrain hidden)
 *   Click 3: stop         → OFF mode (full 3D, normal)
 */

import { setARMode } from '../engine/Scene3D.js';
import { setTerrainSatelliteEnabled } from '../terrain/TerrainManager.js';

// ============== STATES ==============
const MODE = { OFF: 0, CAMERA: 1, AR: 2 };
let fpvMode = MODE.OFF;
let fpvConnected = false;

// DOM references
let fpvCanvas = null;
let fpvCtx = null;
let fpvBtn = null;
let sceneContainer = null;

// Frame decoding: one JPEG in flight at a time; frames arriving meanwhile are dropped
let pendingFrame = false;

// ============== HARDWARE VIDEO DECODE (native RTSP path) ==============
// The main process pulls the camera's H.264/H.265 stream itself and sends the
// access units here; WebCodecs decodes them on the GPU's video engine. This is
// what replaces VLC transcoding every frame to MJPEG. The codecs are probed
// once and passed with fpv.start(), so the main process only picks the native
// path for a stream this renderer can decode.
const PROBE_CODECS = { H264: 'avc1.64001f', H265: 'hev1.1.6.L93.B0' };
let decoderSupport = null;     // Promise<string[]>
let videoDecoder = null;
let awaitingKey = true;
let decoderErrors = [];        // timestamps of recent decode errors

function probeDecoders() {
  if (!decoderSupport) {
    decoderSupport = (async () => {
      if (typeof VideoDecoder === 'undefined') return [];
      const out = [];
      for (const [name, codec] of Object.entries(PROBE_CODECS)) {
        try {
          const r = await VideoDecoder.isConfigSupported({ codec, hardwareAcceleration: 'prefer-hardware' });
          if (r.supported) out.push(name);
        } catch (_) {}
      }
      return out;
    })();
  }
  return decoderSupport;
}

function closeDecoder() {
  if (videoDecoder && videoDecoder.state !== 'closed') {
    try { videoDecoder.close(); } catch (_) {}
  }
  videoDecoder = null;
}

function onVideoConfig(cfg) {
  closeDecoder();
  const config = { codec: cfg.codec, hardwareAcceleration: 'prefer-hardware', optimizeForLatency: true };
  const decoder = new VideoDecoder({
    output: (frame) => {
      if (fpvMode !== MODE.OFF && fpvCtx) drawFrame(frame);
      frame.close();
    },
    error: (err) => {
      if (videoDecoder !== decoder) return;
      console.warn('[FPV] Decoder error:', err.message);
      closeDecoder();
      const now = performance.now();
      decoderErrors = decoderErrors.filter(t => now - t < 10000);
      decoderErrors.push(now);
      // Unsupported profile, or a decoder that keeps failing: let VLC handle it
      if (err.name === 'NotSupportedError' || decoderErrors.length >= 3) {
        if (window.fpv) window.fpv.fallback();
      } else {
        onVideoConfig(cfg);   // transient: start over from the next key frame
      }
    }
  });
  // An unsupported configuration is reported through the error callback
  decoder.configure(config);
  videoDecoder = decoder;
  awaitingKey = true;
}

function onVideoChunk(c) {
  const d = videoDecoder;
  if (!d || d.state !== 'configured' || fpvMode === MODE.OFF) return;
  if (awaitingKey) {
    if (!c.key) return;
    awaitingKey = false;
  }
  // The hardware decoder keeps up; if it ever falls behind, resync on the next
  // key frame rather than showing video that lags further and further.
  if (d.decodeQueueSize > 8) {
    awaitingKey = true;
    return;
  }
  d.decode(new EncodedVideoChunk({ type: c.key ? 'key' : 'delta', timestamp: c.timestamp, data: c.data }));
}

// ============== DEFAULTS ==============
const DEFAULTS = {
  ip: '192.168.144.25',
  port: 8554,
  path: '/main.264',
  fps: 30
};

// ============== SETTINGS ==============
function loadSettings() {
  try {
    const saved = localStorage.getItem('fpv-settings');
    if (saved) return { ...DEFAULTS, ...JSON.parse(saved) };
  } catch (_) {}
  return { ...DEFAULTS };
}

function saveSettings(s) {
  localStorage.setItem('fpv-settings', JSON.stringify(s));
}

export function saveFPVSettings() {
  saveSettings(getSysConfigValues());
}

function getSysConfigValues() {
  return {
    ip:   (document.getElementById('fpv-ip')?.value   || DEFAULTS.ip).trim(),
    port: parseInt(document.getElementById('fpv-port')?.value) || DEFAULTS.port,
    path: (document.getElementById('fpv-path')?.value || DEFAULTS.path).trim(),
    fps:  parseInt(document.getElementById('fpv-fps')?.value)  || DEFAULTS.fps,
  };
}

function populateSysConfigInputs(s) {
  const set = (id, v) => { const el = document.getElementById(id); if (el) el.value = v; };
  set('fpv-ip',   s.ip);
  set('fpv-port', s.port);
  set('fpv-path', s.path);
  set('fpv-fps',  s.fps);
}

// ============== INIT ==============
export function initFPV() {
  fpvCanvas      = document.getElementById('fpv-canvas');
  fpvBtn         = document.getElementById('btn-fpv');
  sceneContainer = document.getElementById('scene-container');

  if (!fpvCanvas || !fpvBtn) {
    console.warn('[FPV] Missing DOM elements, FPV disabled');
    return;
  }

  fpvCtx = fpvCanvas.getContext('2d');

  if (window.fpv) {
    if (window.fpv.onVideoConfig && typeof VideoDecoder !== 'undefined') {
      window.fpv.onVideoConfig(onVideoConfig);
      window.fpv.onVideoChunk(onVideoChunk);
    }

    // VLC fallback: frames arrive as raw JPEG bytes. createImageBitmap decodes them off the
    // main thread (a data-URL <img> did the base64 decode here), and a decode
    // error no longer leaves pendingFrame stuck, which froze the stream.
    window.fpv.onFrame((jpeg) => {
      if (fpvMode === MODE.OFF || pendingFrame) return;
      pendingFrame = true;
      createImageBitmap(new Blob([jpeg], { type: 'image/jpeg' }))
        .then((bitmap) => {
          if (fpvMode !== MODE.OFF && fpvCtx) drawFrame(bitmap);
          bitmap.close();
        })
        .catch(() => {})
        .finally(() => { pendingFrame = false; });
    });

    window.fpv.onError((msg) => {
      console.error('[FPV] Error:', msg);
      showFPVStatus('ERRORE: ' + msg, true);
    });

    window.fpv.onStatus((status) => {
      fpvConnected = status.connected;
      if (status.connected && fpvMode === MODE.OFF) {
        fpvMode = MODE.CAMERA;
      } else if (!status.connected) {
        fpvMode = MODE.OFF;
        closeDecoder();
      }
      applyViewState();
      updateButtonState();
      if (status.connected) showFPVStatus('Stream connected', false);
      else if (fpvMode === MODE.OFF) showFPVStatus('Stream disconnected', true);
    });
  }

  populateSysConfigInputs(loadSettings());
  updateButtonState();
}

/**
 * Draw a frame cropped to cover the canvas (centre crop, aspect preserved).
 * Accepts an ImageBitmap (MJPEG path) or a VideoFrame (WebCodecs path).
 */
function drawFrame(frame) {
  const w = fpvCanvas.width, h = fpvCanvas.height;
  const fw = frame.displayWidth || frame.width;
  const fh = frame.displayHeight || frame.height;
  const ia = fw / fh;
  const ca = w / h;
  let sx = 0, sy = 0, sw = fw, sh = fh;
  if (ia > ca) { sw = sh * ca; sx = (fw - sw) / 2; }
  else         { sh = sw / ca; sy = (fh - sh) / 2; }
  fpvCtx.drawImage(frame, sx, sy, sw, sh, 0, 0, w, h);
}

// ============== BUTTON ==============
export function onFPVButtonClick() {
  if (fpvMode === MODE.OFF) {
    // Click 1: start stream → CAMERA mode
    const settings = getSysConfigValues();
    saveSettings(settings);
    startFPVStream(settings);

  } else if (fpvMode === MODE.CAMERA) {
    // Click 2: switch to AR overlay
    fpvMode = MODE.AR;
    applyViewState();
    updateButtonState();
    showFPVStatus('AR overlay active', false);

  } else {
    // Click 3: stop → OFF (full 3D)
    stopFPVStream();
  }
}

// ============== VIEW STATE ==============
function applyViewState() {
  if (!fpvCanvas || !sceneContainer) return;

  if (fpvMode === MODE.OFF) {
    // Full 3D: camera hidden, scene fully visible and normal
    fpvCanvas.style.display            = 'none';
    sceneContainer.style.opacity       = '1';
    sceneContainer.style.mixBlendMode  = '';
    sceneContainer.style.zIndex        = '';
    sceneContainer.style.background    = '';
    sceneContainer.style.pointerEvents = '';
    setARMode(false);
    // Restore satellite textures if the user had them enabled
    if (window.satelliteEnabled !== false) setTerrainSatelliteEnabled(true);

  } else if (fpvMode === MODE.CAMERA) {
    // Camera only: keep scene-container at opacity:0 (not display:none)
    // so the WebGL context stays alive and the 3D scene keeps updating.
    fpvCanvas.style.display            = 'block';
    sceneContainer.style.opacity       = '0';
    sceneContainer.style.mixBlendMode  = '';
    sceneContainer.style.zIndex        = '';
    sceneContainer.style.background    = '';
    sceneContainer.style.pointerEvents = 'none';
    setARMode(false);

  } else {
    // AR: camera feed behind, 3D on top via mix-blend-mode:screen.
    // Black 3D background becomes transparent; coloured objects (drone, waypoints, terrain)
    // blend additively over the camera feed — no CSS opacity, no GPU sync stall.
    fpvCanvas.style.display             = 'block';
    sceneContainer.style.opacity        = '1';
    sceneContainer.style.mixBlendMode   = 'screen';
    sceneContainer.style.zIndex         = '2';
    sceneContainer.style.background     = '';
    sceneContainer.style.pointerEvents  = 'none';
    setARMode(true);
    setTerrainSatelliteEnabled(false);
  }
}

// ============== BUTTON STATE ==============
function updateButtonState() {
  if (!fpvBtn) return;
  fpvBtn.classList.toggle('active',    fpvMode !== MODE.OFF);
  fpvBtn.classList.toggle('ar-active', fpvMode === MODE.AR);

  if (fpvMode === MODE.OFF) {
    fpvBtn.title = 'FPV Camera — click to start stream';
  } else if (fpvMode === MODE.CAMERA) {
    fpvBtn.title = 'FPV Camera active — click for AR overlay';
  } else {
    fpvBtn.title = 'AR Mode active — click to stop';
  }
}

// ============== STREAM CONTROL ==============
async function startFPVStream(settings) {
  if (!window.fpv) { showFPVStatus('FPV not available', true); return; }
  showFPVStatus('Connecting to camera...');
  const decoders = await probeDecoders();
  decoderErrors = [];
  const result = await window.fpv.start(settings.ip, settings.port, settings.path, { fps: settings.fps, decoders });
  if (!result.success) {
    showFPVStatus('Connection failed: ' + (result.error || 'unknown error'), true);
  }
}

export async function stopFPVStream() {
  if (window.fpv) await window.fpv.stop();
  closeDecoder();
  fpvConnected = false;
  fpvMode = MODE.OFF;
  applyViewState();
  updateButtonState();
  showFPVStatus('Stream stopped', false);
}

// ============== STATUS ==============
function showFPVStatus(msg, isError = false) {
  const el = document.getElementById('fpv-status-msg');
  if (!el) return;
  el.textContent = msg;
  el.className = 'fpv-status-msg' + (isError ? ' error' : '');
  el.style.display = 'block';
  clearTimeout(el._hideTimeout);
  el._hideTimeout = setTimeout(() => { el.style.display = 'none'; }, 4000);
}

// ============== RESIZE ==============
export function resizeFPV() {
  if (!fpvCanvas) return;
  const wrapper = fpvCanvas.parentElement;
  if (!wrapper) return;
  const dpr = window.devicePixelRatio || 1;
  fpvCanvas.width  = wrapper.clientWidth  * dpr;
  fpvCanvas.height = wrapper.clientHeight * dpr;
  fpvCanvas.style.width  = wrapper.clientWidth  + 'px';
  fpvCanvas.style.height = wrapper.clientHeight + 'px';
}

// ============== GETTERS ==============
export function isFPVActive()    { return fpvMode !== MODE.OFF; }
export function isFPVConnected() { return fpvConnected; }
export function isFPVARMode()    { return fpvMode === MODE.AR; }
// Camera only: the 3D scene is transparent (opacity 0), so drawing it is wasted work
export function isFPVCameraMode() { return fpvMode === MODE.CAMERA; }
