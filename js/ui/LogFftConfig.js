/**
 * LogFftConfig.js - SYS CONFIG → FFT FROM LOGS
 *
 * Resolution of the vibration spectra the GCS computes from the IMU samples
 * of a .bin in LOG REPLAY (log-spectrum.js in the main process): FFT size,
 * Welch averaging and time step, kept in localStorage and sent to the main
 * process at start-up and at every change, which recomputes the spectra of
 * the loaded log at once. The vehicle's own spectrum (HPF_FFT_ENABLE) is the
 * firmware's and is not touched.
 *
 * The status line says what the loaded log holds: the stream feeding each
 * sensor with the resolution it gets, and the IMU samples left out.
 */

import { spectrumMonitor } from '../core/SpectrumMonitor.js';

const CFG_KEY = 'log-fft-cfg';
const DEFAULT_CFG = { n: 1024, avg: 1, stepMs: 250 };
const INPUTS = [['logfft-n', 'n'], ['logfft-avg', 'avg'], ['logfft-step', 'stepMs']];

let cfg = { ...DEFAULT_CFG };
let statusEl = null;

function load() {
    try {
        const raw = localStorage.getItem(CFG_KEY);
        if (raw) cfg = { ...DEFAULT_CFG, ...JSON.parse(raw) };
    } catch (e) { /* corrupt entry → defaults */ }
}

function save() {
    try { localStorage.setItem(CFG_KEY, JSON.stringify(cfg)); } catch (e) {}
}

async function send() {
    try {
        const applied = await window.logReplay?.setSpectrumConfig?.(cfg);
        // the main process keeps only values off its lists: show what it uses
        if (applied && (applied.n !== cfg.n || applied.avg !== cfg.avg || applied.stepMs !== cfg.stepMs)) {
            cfg = { ...applied };
            save();
            syncInputs();
        }
    } catch (e) { /* no main process (browser preview) */ }
}

function syncInputs() {
    for (const [id, key] of INPUTS) {
        const el = document.getElementById(id);
        if (el) el.value = String(cfg[key]);
    }
}

const fmtDf = v => (v >= 10 ? v.toFixed(1) : v >= 1 ? v.toFixed(2) : v.toFixed(3)) + ' Hz';
const fmtDur = ms => (ms >= 10000 ? (ms / 1000).toFixed(0) + ' s' : ms >= 1000 ? (ms / 1000).toFixed(1) + ' s' : Math.round(ms) + ' ms');
const esc = s => String(s).replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

function renderStatus() {
    if (!statusEl) return;
    const log = spectrumMonitor.log;
    if (!log) {
        statusEl.textContent = spectrumMonitor.replay ? 'a .tlog: the vehicle\'s own spectrum' : 'no .bin loaded';
        return;
    }
    const lines = [];
    for (const [key, name] of [['gyro', 'GYRO'], ['accel', 'ACC']]) {
        const l = log.sensors?.[key];
        if (!l?.stream) { lines.push(`${name}: no samples`); continue; }
        const cut = l.limited ? ` (cut to ${l.N} pt${l.avg > 1 ? ' × ' + l.avg : ''} by the ${l.longest}-sample runs)` : '';
        lines.push(`${name}: ${esc(l.label)} → ${fmtDf(l.df)}, ${fmtDur(l.windowMs)} per spectrum${cut}`);
    }
    if (log.dropped?.length) lines.push(`<span class="gz-err">left out, in fragments: ${esc(log.dropped.join(', '))}</span>`);
    statusEl.innerHTML = lines.join('<br>');
}

export function initLogFftConfig() {
    load();
    statusEl = document.getElementById('logfft-status');
    syncInputs();
    for (const [id, key] of INPUTS) {
        document.getElementById(id)?.addEventListener('change', e => {
            cfg = { ...cfg, [key]: Number(e.target.value) };
            save();
            send();
        });
    }
    spectrumMonitor.subscribe();
    // the status follows the log: loaded, another stream, new settings, unloaded
    window.logReplay?.onSpectrum?.(m => { if (m?.reset) setTimeout(renderStatus, 0); });
    window.logReplay?.onState?.(() => setTimeout(renderStatus, 0));
    send();
    renderStatus();
}
