/**
 * FftPanel.js - FFT bar (tab on the left edge, under EKF STATUS)
 *
 * The IMU vibration spectra of the vehicle (core/SpectrumMonitor.js) in a
 * black bar down the left side, in the place of the EKF STATUS bar: the two
 * tabs take turns, opening one closes the other.
 *
 *   top:     the strongest lines of the last 2 s, gyro and accelerometer:
 *            frequency, level and height over the noise floor
 *   middle:  the newest spectrum with a 10 s peak hold; the peak, or the
 *            level under the pointer
 *   bottom:  waterfall of the last 60 s, frequency down the plot (0 Hz at
 *            the top), time to the right, colour = level in dB
 *
 * One GYRO | ACC switch drives the chart and the waterfall. The vehicle
 * sends the spectra with HPF_FFT_ENABLE = 1 (ArduPilot with the Hybrid-PF,
 * see docs/FFT_SPECTRUM_PROTOCOL.md). In LOG REPLAY of a .bin the GCS
 * computes them from the IMU samples of the log, at the resolution of
 * SYS CONFIG → FFT FROM LOGS; the line over the chart says which and picks
 * the IMU stream of the log.
 */

import { spectrumMonitor, SENSORS } from '../core/SpectrumMonitor.js';
import { CanvasBox, drawLive, drawWaterfall, sensorUnit, COLORMAP_CSS } from './SpectrumViews.js';

const ENABLED_KEY = 'fft-panel-enabled';
const SENSOR_KEY = 'fft-panel-sensor';
const DRAW_MS = 100;         // 60 s across the plot: about 1 px of scroll a step
const TEXT_MS = 500;

let enabled = false, onFlightTab = true;
let panelEl = null, toggleEl = null, peaksEl = null, unitEl = null, rangeEl = null, readoutEl = null;
let srcTextEl = null, streamEl = null, nowReadoutEl = null;
let box = null, nowBox = null, sensor = 'gyro';
let lastDraw = 0, lastNowDraw = 0, lastNowVersion = -1, lastText = 0, lastPeaksHtml = '', lastStreams = '';

function setSensor(s) {
    sensor = s === 'accel' ? 'accel' : 'gyro';
    try { localStorage.setItem(SENSOR_KEY, sensor); } catch (e) {}
    panelEl?.querySelectorAll('[data-wf-sensor]').forEach(b => b.classList.toggle('active', b.dataset.wfSensor === sensor));
    if (unitEl) unitEl.textContent = sensorUnit(sensor);
    lastDraw = lastText = 0;
    lastNowVersion = -1;
    lastStreams = '';
}

/** Source line: resolution, and for a .bin the IMU stream picker. */
function updateSource() {
    const src = spectrumMonitor.source(sensor);
    if (srcTextEl.textContent !== src.text) srcTextEl.textContent = src.text;
    const streams = src.kind === 'log' ? src.streams || [] : [];
    const sig = streams.map(o => o.key).join('|') + '#' + (src.stream || '');
    if (sig === lastStreams) return;
    lastStreams = sig;
    streamEl.replaceChildren(...streams.map(o => {
        const opt = document.createElement('option');
        opt.value = o.key;
        opt.textContent = o.label;
        return opt;
    }));
    if (src.stream) streamEl.value = src.stream;
    streamEl.hidden = !streams.length;
    streamEl.disabled = streams.length < 2;
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

const esc = s => String(s).replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

function peaksHtml(now) {
    let html = '';
    for (const key of ['gyro', 'accel']) {
        const d = spectrumMonitor.get(key).dominant(now);
        html += `<span class="fft-peak-lbl">${SENSORS[key].label}</span>`;
        if (!d) {
            html += `<span class="fft-peak-none">NO DATA · ${esc(spectrumMonitor.source(key).hint)}</span>`;
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
    nowReadoutEl = document.getElementById('fft-now-readout');
    srcTextEl = document.getElementById('fft-src-text');
    streamEl = document.getElementById('fft-stream');
    streamEl?.addEventListener('change', () => spectrumMonitor.selectStream(sensor, streamEl.value));
    const scale = panelEl.querySelector('.spec-scale i');
    if (scale) scale.style.background = COLORMAP_CSS;
    spectrumMonitor.subscribe();
    box = new CanvasBox(document.getElementById('fft-plot'));
    nowBox = new CanvasBox(document.getElementById('fft-now-plot'));
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
    lastNowVersion = -1;
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
    const perf = performance.now();
    const now = spectrumMonitor.clock();     // the replay cursor while a log plays
    if (perf - lastDraw >= DRAW_MS) {
        lastDraw = perf;
        const r = drawWaterfall(box, sensor, now);
        const range = r ? `${r.lo} … ${r.hi} dB` : '—';
        if (rangeEl.textContent !== range) rangeEl.textContent = range;
        const ro = r?.readout ? `${r.readout.f.toFixed(1)} Hz · −${r.readout.age.toFixed(0)} s · ${r.readout.db.toFixed(1)} dB` : '';
        if (readoutEl.textContent !== ro) readoutEl.textContent = ro;
    }
    // The newest spectrum: on new data, a resize or a pointer move, and once
    // a second for the staleness note
    const version = spectrumMonitor.get(sensor).version;
    if (perf - lastNowDraw >= DRAW_MS && (version !== lastNowVersion || nowBox.dirty || perf - lastNowDraw >= 1000)) {
        lastNowDraw = perf;
        lastNowVersion = version;
        nowBox.dirty = false;
        const r = drawLive(nowBox, sensor, now);
        const text = r ? `${r.label} ${r.f.toFixed(r.f < 10 ? 1 : 0)} Hz · ${r.db.toFixed(1)} dB` : '';
        if (nowReadoutEl && nowReadoutEl.textContent !== text) nowReadoutEl.textContent = text;
    }
    if (perf - lastText >= TEXT_MS) {
        lastText = perf;
        if (srcTextEl && streamEl) updateSource();
        const html = peaksHtml(now);
        if (html !== lastPeaksHtml) { lastPeaksHtml = html; peaksEl.innerHTML = html; }
    }
}
