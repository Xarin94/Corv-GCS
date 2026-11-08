/**
 * FftPanel.js - FFT bar (tab on the left edge, under EKF STATUS)
 *
 * The IMU vibration spectra of the vehicle (core/SpectrumMonitor.js) in a
 * black bar down the left side, in the place of the EKF STATUS bar: the two
 * tabs take turns, opening one closes the other.
 *
 *   top:    the strongest lines of the last 2 s, gyro and accelerometer:
 *           frequency, level and height over the noise floor
 *   below:  waterfall of the last 60 s, frequency down the plot (0 Hz at the
 *           top), time to the right, colour = level in dB; gyro or accel
 *
 * The vehicle sends the spectra with HPF_FFT_ENABLE = 1 (ArduPilot with the
 * Hybrid-PF, see docs/FFT_SPECTRUM_PROTOCOL.md).
 */

import { spectrumMonitor, SENSORS } from '../core/SpectrumMonitor.js';
import { CanvasBox, drawWaterfall, sensorUnit, COLORMAP_CSS } from './SpectrumViews.js';

const ENABLED_KEY = 'fft-panel-enabled';
const SENSOR_KEY = 'fft-panel-sensor';
const DRAW_MS = 100;         // 60 s across the plot: about 1 px of scroll a step
const TEXT_MS = 500;

let enabled = false, onFlightTab = true;
let panelEl = null, toggleEl = null, peaksEl = null, unitEl = null, rangeEl = null, readoutEl = null;
let box = null, sensor = 'gyro';
let lastDraw = 0, lastText = 0, lastPeaksHtml = '';

function setSensor(s) {
    sensor = s === 'accel' ? 'accel' : 'gyro';
    try { localStorage.setItem(SENSOR_KEY, sensor); } catch (e) {}
    panelEl?.querySelectorAll('[data-wf-sensor]').forEach(b => b.classList.toggle('active', b.dataset.wfSensor === sensor));
    if (unitEl) unitEl.textContent = sensorUnit(sensor) + ' · frequency down, time to the right';
    lastDraw = 0;
}

function applyVisibility() {
    panelEl?.classList.toggle('collapsed', !enabled);
    if (toggleEl) {
        toggleEl.classList.toggle('collapsed', !enabled);
        toggleEl.querySelector('.ekf-toggle-arrow').innerHTML = enabled ? '&#9664;' : '&#9654;';
        toggleEl.title = enabled ? 'Close FFT' : 'Open FFT';
    }
    // Same place as the EKF STATUS bar: either one open docks the flight view
    const docked = !!document.querySelector('#ekf-panel:not(.collapsed), #fft-panel:not(.collapsed)');
    if (document.body.classList.contains('ekf-docked') !== docked) {
        document.body.classList.toggle('ekf-docked', docked);
        requestAnimationFrame(() => window.dispatchEvent(new Event('resize')));
    }
}

const fmtDb = v => (v < 0 ? '−' : '') + Math.abs(v).toFixed(0);

function peaksHtml(now) {
    let html = '';
    for (const key of ['gyro', 'accel']) {
        const d = spectrumMonitor.get(key).dominant(now);
        html += `<span class="fft-peak-lbl">${SENSORS[key].label}</span>`;
        if (!d) {
            html += '<span class="fft-peak-none">NO DATA · HPF_FFT_ENABLE = 1 on the vehicle</span>';
            continue;
        }
        if (!d.peaks.length) {
            html += `<span><span class="fft-peak-main">${d.max.f.toFixed(1)} Hz ${fmtDb(d.max.db)} dB</span><br>`
                + `<span class="fft-peak-none">no line over the noise · floor ${fmtDb(d.floor)} dB</span></span>`;
            continue;
        }
        const [p, ...more] = d.peaks;
        html += '<span>'
            + `<span class="fft-peak-main"><b>${p.f.toFixed(1)} Hz</b> ${fmtDb(p.db)} dB · +${p.prom.toFixed(0)} dB over the floor</span><br>`
            + `<span class="fft-peak-more">${more.map(q => `${q.f.toFixed(1)} Hz ${fmtDb(q.db)} dB`).join(' · ')}`
            + `${more.length ? ' · ' : ''}floor ${fmtDb(d.floor)} dB</span>`
            + '</span>';
    }
    return html;
}

// ── Public ──────────────────────────────────────────────────────────────

export function initFftPanel() {
    panelEl = document.getElementById('fft-panel');
    if (!panelEl) return;
    toggleEl = document.getElementById('fft-toggle');
    toggleEl?.addEventListener('click', () => setFftPanelEnabled(!enabled));
    peaksEl = document.getElementById('fft-peaks');
    unitEl = document.getElementById('fft-unit');
    rangeEl = document.getElementById('fft-range');
    readoutEl = document.getElementById('fft-readout');
    const scale = panelEl.querySelector('.spec-scale i');
    if (scale) scale.style.background = COLORMAP_CSS;
    spectrumMonitor.subscribe();
    box = new CanvasBox(document.getElementById('fft-plot'));
    panelEl.querySelectorAll('[data-wf-sensor]').forEach(b => b.addEventListener('click', () => setSensor(b.dataset.wfSensor)));
    let saved = 'gyro';
    try { saved = localStorage.getItem(SENSOR_KEY) || 'gyro'; } catch (e) {}
    setSensor(saved);
    // one bar at a time on the left: another opening closes this one
    window.addEventListener('leftbar-open', e => { if (e.detail !== 'fft' && enabled) setFftPanelEnabled(false); });
    let open = false;
    try { open = localStorage.getItem(ENABLED_KEY) === '1'; } catch (e) {}
    if (open) setFftPanelEnabled(true); else applyVisibility();
}

export function setFftPanelEnabled(on) {
    enabled = !!on;
    if (enabled) window.dispatchEvent(new CustomEvent('leftbar-open', { detail: 'fft' }));
    try { localStorage.setItem(ENABLED_KEY, enabled ? '1' : '0'); } catch (e) {}
    applyVisibility();
    lastDraw = lastText = 0;
}

/** Called every animation frame; draws only while shown. */
export function updateFftPanel(visible) {
    if (!panelEl) return;
    if (visible !== onFlightTab) {
        onFlightTab = visible;
        panelEl.style.display = visible ? '' : 'none';
        if (toggleEl) toggleEl.style.display = visible ? '' : 'none';
    }
    if (!enabled || !visible) return;
    const now = performance.now();
    if (now - lastDraw >= DRAW_MS) {
        lastDraw = now;
        const r = drawWaterfall(box, sensor, now);
        const range = r ? `${r.lo} … ${r.hi} dB` : '—';
        if (rangeEl.textContent !== range) rangeEl.textContent = range;
        const ro = r?.readout ? `${r.readout.f.toFixed(1)} Hz · −${r.readout.age.toFixed(0)} s · ${r.readout.db.toFixed(1)} dB` : '';
        if (readoutEl.textContent !== ro) readoutEl.textContent = ro;
    }
    if (now - lastText >= TEXT_MS) {
        lastText = now;
        const html = peaksHtml(now);
        if (html !== lastPeaksHtml) { lastPeaksHtml = html; peaksEl.innerHTML = html; }
    }
}
