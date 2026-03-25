/**
 * FPVController.js - FPV Camera Stream Controller
 *
 * Manages the SIYI HM30 (or generic RTSP) camera feed display.
 * - Settings are configured in SYS CONFIG > SIYI CAMERA STREAM panel
 * - Main screen button toggles stream on/off
 */

// ============== STATE ==============
let fpvActive = false;       // Currently showing FPV feed
let fpvConnected = false;    // Stream is connected

// DOM references (cached after init)
let fpvCanvas = null;
let fpvCtx = null;
let fpvBtn = null;
let sceneContainer = null;

// Offscreen image for decoding JPEG frames
const frameImg = new Image();
let pendingFrame = false;

// FPS tracking
let fpvFrameCount = 0;
let fpvLastFpsTime = 0;
let fpvFps = 0;

// ============== DEFAULT SIYI HM30 SETTINGS ==============
const DEFAULTS = {
  ip: '192.168.144.25',
  port: 8554,
  path: '/main.264',
  fps: 30,
  quality: 5
};

// ============== SETTINGS PERSISTENCE ==============
function loadSettings() {
  try {
    const saved = localStorage.getItem('fpv-settings');
    if (saved) {
      return { ...DEFAULTS, ...JSON.parse(saved) };
    }
  } catch (_) {}
  return { ...DEFAULTS };
}

function saveSettings(settings) {
  localStorage.setItem('fpv-settings', JSON.stringify(settings));
}

// Called by onchange handlers in the SYS CONFIG panel
export function saveFPVSettings() {
  const settings = getSysConfigValues();
  saveSettings(settings);
}

// ============== READ / WRITE SYSCFG INPUTS ==============
function getSysConfigValues() {
  return {
    ip: (document.getElementById('fpv-ip')?.value || DEFAULTS.ip).trim(),
    port: parseInt(document.getElementById('fpv-port')?.value) || DEFAULTS.port,
    path: (document.getElementById('fpv-path')?.value || DEFAULTS.path).trim(),
    fps: parseInt(document.getElementById('fpv-fps')?.value) || DEFAULTS.fps,
    quality: DEFAULTS.quality
  };
}

function populateSysConfigInputs(settings) {
  const ipInput = document.getElementById('fpv-ip');
  const portInput = document.getElementById('fpv-port');
  const pathInput = document.getElementById('fpv-path');
  const fpsInput = document.getElementById('fpv-fps');
  if (ipInput) ipInput.value = settings.ip;
  if (portInput) portInput.value = settings.port;
  if (pathInput) pathInput.value = settings.path;
  if (fpsInput) fpsInput.value = settings.fps;
}

// ============== INIT ==============
export function initFPV() {
  fpvCanvas = document.getElementById('fpv-canvas');
  fpvBtn = document.getElementById('btn-fpv');
  sceneContainer = document.getElementById('scene-container');

  if (!fpvCanvas || !fpvBtn) {
    console.warn('[FPV] Missing DOM elements, FPV disabled');
    return;
  }

  fpvCtx = fpvCanvas.getContext('2d');

  // Frame decode handler
  frameImg.onload = () => {
    if (!fpvActive || !fpvCtx) return;
    const w = fpvCanvas.width;
    const h = fpvCanvas.height;
    // Draw frame scaled to fill canvas (maintain aspect ratio, center-crop)
    const imgAspect = frameImg.naturalWidth / frameImg.naturalHeight;
    const canvasAspect = w / h;
    let sx = 0, sy = 0, sw = frameImg.naturalWidth, sh = frameImg.naturalHeight;
    if (imgAspect > canvasAspect) {
      // Image wider than canvas: crop sides
      sw = frameImg.naturalHeight * canvasAspect;
      sx = (frameImg.naturalWidth - sw) / 2;
    } else {
      // Image taller than canvas: crop top/bottom
      sh = frameImg.naturalWidth / canvasAspect;
      sy = (frameImg.naturalHeight - sh) / 2;
    }
    fpvCtx.drawImage(frameImg, sx, sy, sw, sh, 0, 0, w, h);
    pendingFrame = false;

    // FPS counter
    fpvFrameCount++;
    const now = performance.now();
    if (now - fpvLastFpsTime >= 1000) {
      fpvFps = fpvFrameCount;
      fpvFrameCount = 0;
      fpvLastFpsTime = now;
    }
  };

  // Listen for frames from main process
  if (window.fpv) {
    window.fpv.onFrame((base64) => {
      if (!fpvActive || pendingFrame) return;
      pendingFrame = true;
      frameImg.src = 'data:image/jpeg;base64,' + base64;
    });

    window.fpv.onError((msg) => {
      console.error('[FPV] Error:', msg);
      showFPVStatus('ERROR: ' + msg, true);
    });

    window.fpv.onStatus((status) => {
      fpvConnected = status.connected;
      updateButtonState();
      if (!status.connected && fpvActive) {
        showFPVStatus('Stream disconnesso', true);
        fpvActive = false;
        applyViewState();
      }
    });
  }

  // Populate SYS CONFIG inputs with saved settings
  const settings = loadSettings();
  populateSysConfigInputs(settings);

  // Update button initial state
  updateButtonState();
}

// ============== BUTTON CLICK HANDLER ==============
export function onFPVButtonClick() {
  if (!fpvConnected) {
    // Not connected: start stream with current settings
    const settings = getSysConfigValues();
    saveSettings(settings);
    startFPVStream(settings);
  } else {
    // Connected: stop stream
    stopFPVStream();
  }
}

// ============== TOGGLE FPV / 3D ==============
export function setFPVActive(active) {
  fpvActive = active;
  applyViewState();
}

function applyViewState() {
  if (!fpvCanvas || !sceneContainer) return;

  if (fpvActive) {
    fpvCanvas.style.display = 'block';
    sceneContainer.style.display = 'none';
  } else {
    fpvCanvas.style.display = 'none';
    sceneContainer.style.display = 'block';
  }

  updateButtonState();
}

// ============== BUTTON STATE ==============
function updateButtonState() {
  if (!fpvBtn) return;

  if (fpvConnected) {
    fpvBtn.classList.add('active');
    fpvBtn.title = 'FPV Camera (stream attivo - clicca per fermare)';
  } else {
    fpvBtn.classList.remove('active');
    fpvBtn.title = 'FPV Camera (clicca per avviare stream)';
  }
}

// ============== STREAM CONTROL ==============
async function startFPVStream(settings) {
  if (!window.fpv) {
    showFPVStatus('FPV non disponibile (ffmpeg bridge mancante)', true);
    return;
  }

  showFPVStatus('Connessione in corso...');

  const result = await window.fpv.start(
    settings.ip,
    settings.port,
    settings.path,
    { fps: settings.fps, quality: settings.quality }
  );

  if (result.success) {
    fpvConnected = true;
    fpvActive = true;
    applyViewState();
    showFPVStatus('Connesso', false);
  } else {
    showFPVStatus('Connessione fallita: ' + (result.error || 'errore sconosciuto'), true);
  }
}

export async function stopFPVStream() {
  if (window.fpv) {
    await window.fpv.stop();
  }
  fpvConnected = false;
  fpvActive = false;
  applyViewState();
  showFPVStatus('Stream fermato', false);
}

// ============== STATUS MESSAGES ==============
function showFPVStatus(msg, isError = false) {
  const statusEl = document.getElementById('fpv-status-msg');
  if (!statusEl) return;

  statusEl.textContent = msg;
  statusEl.className = 'fpv-status-msg' + (isError ? ' error' : '');
  statusEl.style.display = 'block';

  // Auto-hide after 4 seconds
  clearTimeout(statusEl._hideTimeout);
  statusEl._hideTimeout = setTimeout(() => {
    statusEl.style.display = 'none';
  }, 4000);
}

// ============== RESIZE ==============
export function resizeFPV() {
  if (!fpvCanvas) return;
  const wrapper = fpvCanvas.parentElement;
  if (!wrapper) return;

  const dpr = window.devicePixelRatio || 1;
  const w = wrapper.clientWidth;
  const h = wrapper.clientHeight;

  fpvCanvas.width = w * dpr;
  fpvCanvas.height = h * dpr;
  fpvCanvas.style.width = w + 'px';
  fpvCanvas.style.height = h + 'px';
}

// ============== GETTERS ==============
export function isFPVActive() { return fpvActive; }
export function isFPVConnected() { return fpvConnected; }
export function getFPVFps() { return fpvFps; }
