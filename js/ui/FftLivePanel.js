/**
 * FftLivePanel.js - FFT LIVE section of the GCS sidebar
 *
 * The newest IMU vibration spectrum of the vehicle with a 10 s peak hold,
 * gyro or accelerometer (data in core/SpectrumMonitor.js, chart in
 * ui/SpectrumViews.js). Drawn only while the sidebar and the section are
 * open; collapsed, it costs nothing.
 */

import { spectrumMonitor } from '../core/SpectrumMonitor.js';
import { CanvasBox, drawLive, sensorUnit } from './SpectrumViews.js';

const SENSOR_KEY = 'fft-live-sensor';
const DRAW_MS = 100;

let sectionEl = null, sidebarEl = null, box = null, readoutEl = null, unitEl = null;
let sensor = 'gyro';
let lastDraw = 0, lastVersion = -1;

function setSensor(s) {
    sensor = s === 'accel' ? 'accel' : 'gyro';
    try { localStorage.setItem(SENSOR_KEY, sensor); } catch (e) {}
    sectionEl?.querySelectorAll('[data-fft-sensor]').forEach(b => b.classList.toggle('active', b.dataset.fftSensor === sensor));
    if (unitEl) unitEl.textContent = sensorUnit(sensor);
    lastVersion = -1;
}

export function initFftLivePanel() {
    sectionEl = document.querySelector('.gcs-sidebar-section[data-section="fftlive"]');
    sidebarEl = document.getElementById('gcs-sidebar');
    if (!sectionEl) return;
    spectrumMonitor.subscribe();
    box = new CanvasBox(sectionEl.querySelector('.fft-live-plot'));
    readoutEl = sectionEl.querySelector('.fft-live-readout');
    unitEl = sectionEl.querySelector('.fft-live-unit');
    sectionEl.querySelectorAll('[data-fft-sensor]').forEach(b => b.addEventListener('click', () => setSensor(b.dataset.fftSensor)));
    let saved = 'gyro';
    try { saved = localStorage.getItem(SENSOR_KEY) || 'gyro'; } catch (e) {}
    setSensor(saved);
}

/** Called every animation frame. */
export function updateFftLivePanel() {
    if (!box) return;
    if (sidebarEl?.classList.contains('collapsed') || sectionEl.classList.contains('collapsed')) return;
    const now = performance.now();
    if (now - lastDraw < DRAW_MS) return;
    const version = spectrumMonitor.get(sensor).version;
    // redraw on new data, on resize or pointer moves, and once a second for the staleness note
    if (version === lastVersion && !box.dirty && now - lastDraw < 1000) return;
    lastDraw = now;
    lastVersion = version;
    box.dirty = false;
    const r = drawLive(box, sensor, now);
    const text = r ? `${r.label} ${r.f.toFixed(0)} Hz · ${r.db.toFixed(1)} dB` : '—';
    if (readoutEl.textContent !== text) readoutEl.textContent = text;
}
