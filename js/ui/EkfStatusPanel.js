/**
 * EkfStatusPanel.js - EKF STATUS charts (tab on the left edge)
 *
 * Three charts stacked in a black bar down the left side, the flight view
 * moved over to make room; the last 20 s, chosen for the estimator on board
 * (data in core/EkfMonitor.js). The EKF STATUS tab on the left edge opens and
 * closes it, as the GCS OPTIONS tab does the sidebar on the right:
 *
 *   ArduPilot EKF3  EKF INNOVATIONS  · GPS ACCURACY       · ATTITUDE EKF − DCM
 *   CORV RBPF       POSITION         · PARTICLE FILTER    · SHARED EKF tr(P)
 *
 * A sound estimate reads as: innovations under 0.5, the residual inside the
 * receiver's accuracy, the particle cloud no wider than the GPS error, ESS
 * recovering after each resample, the two attitude estimators within a degree
 * or two, the covariance flat. The header lists the solutions the filter
 * itself flags as good or bad.
 *
 * While the panel is open on an ArduPilot link, EKF_STATUS_REPORT, AHRS2 and
 * GPS_RAW_INT are asked for at 5 Hz (2 Hz by default) and put back on close.
 */

import { STATE } from '../core/state.js';
import { EkfMonitor, EKF_WINDOW_MS, AP_EKF_FLAGS as AF, CORV_FLAGS as CF, CORV_AIDING } from '../core/EkfMonitor.js';
import { onMessage } from '../mavlink/MAVLinkManager.js';
import { setMessageInterval, resetMessageInterval } from '../mavlink/CommandSender.js';

const ENABLED_KEY = 'ekf-status-enabled';
const DRAW_MS = 50;          // charts at 20 Hz: the 20 s window scrolls ~1.5 px a step
const TEXT_MS = 200;         // legends, chips and flags at 5 Hz
const GAP_MS = 2500;         // samples further apart are not joined (the debug packet is 1 Hz)
const ARDUPILOT = 3;         // MAV_AUTOPILOT_ARDUPILOTMEGA
const AP_RATES = [[193, 5], [178, 5], [24, 5]];   // EKF_STATUS_REPORT, AHRS2, GPS_RAW_INT

// Series colours: categorical slots of the reference palette, dark steps,
// checked against the panel glass (#0c1218) for colour-blind separation and
// contrast. Fixed order per chart: a series keeps its colour.
const SERIES_COLORS = ['#3987e5', '#d95926', '#199e70', '#c98500', '#d55181'];
// Reserved for state: thresholds and flags only, always with a label
const STATUS = { good: '#0ca30c', warning: '#fab219', critical: '#d03b3b' };
const INK = { primary: '#f0efec', secondary: '#c3c2b7', muted: '#898781', grid: 'rgba(255,255,255,0.07)', axis: 'rgba(255,255,255,0.22)' };

const FIX_NAMES = ['NO GPS', 'NO FIX', '2D', '3D', 'DGPS', 'RTK FLT', 'RTK FIX', 'STATIC', 'PPP'];
const CORV_FIX_NAMES = ['NO FIX', 'DR', '2D', '3D', 'GNSS+DR', 'RTK FIX', 'RTK FLT'];

const fmt = (v, digits) => (v === null || v === undefined || !Number.isFinite(v)) ? '—' : v.toFixed(digits);

const CHARTS = {
    ardupilot: {
        label: 'ARDUPILOT EKF3',
        charts: [
            {
                title: 'EKF INNOVATIONS', unit: 'test ratio · 1 = measurement rejected', min: 0, floor: 1, digits: 2,
                lines: [{ key: 'apVel', name: 'VEL' }, { key: 'apPosH', name: 'POS H' }, { key: 'apPosV', name: 'POS V' },
                        { key: 'apMag', name: 'MAG' }, { key: 'apTas', name: 'TAS', optional: true }],
                marks: [{ v: 0.5, status: 'warning', label: '0.5' }, { v: 0.8, status: 'critical', label: '0.8' }],
                chips: m => [['TERR', fmt(m.ap.terr, 2)]]
            },
            {
                title: 'GPS ACCURACY', unit: 'H, V in m · SPEED in m/s', min: 0, floor: 1, digits: 2,
                lines: [{ key: 'apHAcc', name: 'H' }, { key: 'apVAcc', name: 'V' }, { key: 'apVelAcc', name: 'SPEED', optional: true }],
                // Receivers that report no accuracy: the dilution of precision instead
                fallback: { unit: 'HDOP · the receiver reports no accuracy', floor: 2, lines: [{ key: 'apHdop', name: 'HDOP' }] },
                chips: m => [['FIX', FIX_NAMES[m.ap.fix] ?? '—'], ['SATS', m.ap.sats ?? '—'], ['HDOP', fmt(m.ap.hdop, 1)]]
            },
            {
                // AHRS2 is the estimator not in use: the DCM while the EKF flies, the EKF after a fall back to DCM
                title: 'ATTITUDE  EKF vs DCM', unit: '° · active estimator − backup (AHRS2)', symmetric: true, floor: 2, digits: 2,
                lines: [{ key: 'apDRoll', name: 'ROLL' }, { key: 'apDPitch', name: 'PITCH' }, { key: 'apDYaw', name: 'YAW' }],
                chips: m => {
                    const v = m.ap.vibe;
                    const worst = v ? Math.max(...v) : null;
                    return [['VIBE', v ? v.map(x => x.toFixed(0)).join('/') : '—', worst > 60 ? 'critical' : worst > 30 ? 'warning' : null],
                            ['CLIP', m.ap.clip ?? '—', m.ap.clip > 0 ? 'warning' : null]];
                }
            }
        ]
    },
    corv: {
        label: 'CORV RBPF',
        charts: [
            {
                title: 'POSITION', unit: 'm · raw GPS against the filter, cloud spread, receiver accuracy', min: 0, floor: 2, digits: 1,
                lines: [{ key: 'corvResH', name: 'GPS − PF' }, { key: 'corvSpread', name: 'PF SPREAD' },
                        { key: 'corvGpsH', name: 'GPS H ACC' }, { key: 'corvGpsV', name: 'GPS V ACC' }],
                chips: m => [['FIX', CORV_FIX_NAMES[m.corv.fix] ?? '—'], ['SATS', m.corv.sats ?? '—'], ['HDOP', fmt(m.corv.hdop, 1)]]
            },
            {
                title: 'PARTICLE FILTER', unit: '% · ticks = resamples', min: 0, max: 100, digits: 0,
                lines: [{ key: 'corvEss', name: 'ESS / N' }, { key: 'corvConf', name: 'CONFIDENCE' }],
                // pf_ess_threshold default: under it the cloud is resampled
                marks: [{ v: 50, label: 'RESAMPLE' }],
                events: 'corvResample',
                chips: m => [['N', m.corv.n ?? '—'], ['ESS', fmt(m.corv.ess, 0)], ['RESAMPLES', m.corv.resampleRate === null ? '—' : m.corv.resampleRate.toFixed(1) + '/s']]
            },
            {
                title: 'SHARED EKF  tr(P)', unit: 'trace of the bias covariance (log)', log: true, digits: 3,
                lines: [{ key: 'corvTrP', name: 'tr(P)' }],
                chips: m => {
                    const b = m.corv.gyroBias, q = m.corv.magQuality;
                    return [['GYRO BIAS', b ? Math.max(...b.map(Math.abs)).toFixed(3) + '°/s' : '—'],
                            ['MAG Q', fmt(q, 2), q !== null && q < 0.5 ? 'warning' : null],
                            ['AIDING', m.corv.flags === null ? '—' : CORV_AIDING[(m.corv.flags >> 9) & 3]]];
                }
            }
        ]
    }
};

const monitor = new EkfMonitor();
let enabled = false;
let panelEl = null, toggleEl = null, sourceEl = null, flagsEl = null, cardsEl = null;
let onFlightTab = true;      // the bar and its tab belong to the flight view
let cards = [];
let shownSource;             // source the cards were built for
let lastDraw = 0, lastText = 0, lastRates = 0;
let ratesRequested = false;

// ── Data in ─────────────────────────────────────────────────────────────

function subscribe() {
    for (const id of [193, 24, 30, 178, 241]) onMessage(id, data => monitor.ingestMavlink(id, data, performance.now()));
    const corv = window.corvSerial;
    if (corv) {
        corv.onNavStatus?.(n => monitor.ingestCorvNav(n, performance.now()));
        corv.onDebug?.(b => monitor.ingestCorvDebug(b, performance.now()));
        corv.onRawSensor?.(b => monitor.ingestCorvRaw(b, performance.now()));
    }
    // A new link is a new vehicle: its charts start empty
    window.addEventListener('mavlinkConnectionState', e => {
        if (e.detail?.state === 'CONNECTED') monitor.reset();
        if (e.detail?.state === 'DISCONNECTED') ratesRequested = false;
    });
}

/** 5 Hz EKF messages from an ArduPilot while the panel is open, the defaults back after. */
function manageRates(now) {
    if (now - lastRates < 1000) return;
    lastRates = now;
    const ardupilot = STATE.connected && STATE.autopilotType === ARDUPILOT && monitor.source(now) !== 'corv'
        && Date.now() - (STATE.lastHeartbeatTime || 0) < 3000;
    if (!ardupilot) { if (!STATE.connected) ratesRequested = false; return; }
    if (enabled && !ratesRequested) {
        ratesRequested = true;
        for (const [id, hz] of AP_RATES) setMessageInterval(id, hz).catch(() => {});
    } else if (!enabled && ratesRequested) {
        ratesRequested = false;
        for (const [id] of AP_RATES) resetMessageInterval(id).catch(() => {});
    }
}

// ── Panel ───────────────────────────────────────────────────────────────

function buildCards(source) {
    shownSource = source;
    cardsEl.textContent = '';
    cards = (CHARTS[source || 'ardupilot']).charts.map(spec => {
        const card = document.createElement('div');
        card.className = 'ekf-card';
        card.innerHTML = '<div class="ekf-card-head"><span class="ekf-card-title"></span><span class="ekf-chips"></span></div>'
            + '<div class="ekf-card-unit"></div><div class="ekf-plot"><canvas class="ekf-canvas"></canvas></div><div class="ekf-legend"></div>';
        card.querySelector('.ekf-card-title').textContent = spec.title;
        cardsEl.appendChild(card);
        const canvas = card.querySelector('canvas');
        const c = { spec, card, canvas, ctx: canvas.getContext('2d'), w: 0, h: 0, hover: null,
            unitEl: card.querySelector('.ekf-card-unit'), chipsEl: card.querySelector('.ekf-chips'),
            legendEl: card.querySelector('.ekf-legend'), legendKeys: '' };
        // Sizes from the observer on the box around the canvas: reading
        // clientWidth every frame forces layout
        new ResizeObserver(entries => {
            const r = entries[0].contentRect;
            c.w = r.width; c.h = r.height;
            const dpr = window.devicePixelRatio || 1;
            canvas.width = Math.max(1, Math.round(r.width * dpr));
            canvas.height = Math.max(1, Math.round(r.height * dpr));
        }).observe(card.querySelector('.ekf-plot'));
        canvas.addEventListener('mousemove', e => { c.hover = e.offsetX; });
        canvas.addEventListener('mouseleave', () => { c.hover = null; });
        return c;
    });
}

/** Lines to draw: the main set, or the fallback when none of it has data. */
function activeLines(spec) {
    const has = l => monitor.get(l.key).length > 0;
    let lines = spec.lines.filter(l => !l.optional || has(l));
    let unit = spec.unit, floor = spec.floor;
    if (spec.fallback && !lines.some(has) && spec.fallback.lines.some(has)) {
        ({ lines, unit, floor } = spec.fallback);
    }
    return { lines, unit, floor };
}

function niceCeil(v) {
    const p = Math.pow(10, Math.floor(Math.log10(v)));
    for (const m of [1, 2, 2.5, 5, 10]) if (m * p >= v - 1e-12) return m * p;
    return 10 * p;
}

const PAD = { l: 30, t: 6, b: 13 };
const END_LABEL_FONT = '10px "Rajdhani", sans-serif';

/** Right margin: room for the series names at the line ends (none for a single series). */
function rightPad(ctx, lines) {
    if (lines.length < 2) return 10;
    ctx.font = END_LABEL_FONT;
    const widest = Math.max(...lines.map(l => ctx.measureText(l.name).width));
    return Math.min(80, Math.ceil(widest) + 12);
}

function drawCard(c, now) {
    const { spec, ctx, w, h } = c;
    if (!(w > 0 && h > 0)) return;
    const dpr = window.devicePixelRatio || 1;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    const { lines, floor } = activeLines(spec);
    const padR = c.padR = rightPad(ctx, lines);
    const pw = w - PAD.l - padR, ph = h - PAD.t - PAD.b;
    if (pw < 20 || ph < 20) return;
    const t0 = now - EKF_WINDOW_MS;
    const X = t => PAD.l + (t - t0) / EKF_WINDOW_MS * pw;

    // Range over what is on screen
    let max = -Infinity, min = Infinity;
    for (const l of lines) {
        const s = monitor.get(l.key);
        for (let i = 0; i < s.length; i++) if (s.t[i] >= t0) {
            const v = spec.symmetric ? Math.abs(s.v[i]) : s.v[i];
            if (v > max) max = v; if (v < min) min = v;
        }
    }
    let lo, hi, scale = v => v;
    if (spec.log) {
        scale = v => Math.log10(Math.max(v, 1e-12));
        const a = Number.isFinite(min) ? Math.floor(scale(min)) : -3, b = Number.isFinite(max) ? Math.ceil(scale(max)) : 0;
        lo = Math.pow(10, a); hi = Math.pow(10, b > a ? b : a + 1);
    } else if (spec.symmetric) {
        hi = niceCeil(Math.max(floor, Number.isFinite(max) ? max * 1.15 : 0)); lo = -hi;
    } else {
        lo = spec.min; hi = spec.max ?? niceCeil(Math.max(floor, Number.isFinite(max) ? max * 1.15 : 0));
    }
    const Y = v => PAD.t + ph - (scale(Math.min(Math.max(v, lo), hi)) - scale(lo)) / (scale(hi) - scale(lo)) * ph;

    // Grid and y labels: recessive
    ctx.font = '10px "Roboto Mono", monospace';
    ctx.textBaseline = 'middle';
    const ticks = spec.log ? decades(lo, hi) : spec.symmetric ? [lo, 0, hi] : [lo, (lo + hi) / 2, hi];
    const marked = new Set((spec.marks || []).map(m => m.v));
    for (const v of ticks) {
        if (marked.has(v)) continue;   // the threshold line carries its own label
        const y = Math.round(Y(v)) + 0.5;
        ctx.strokeStyle = v === 0 || v === lo ? INK.axis : INK.grid;
        ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(PAD.l, y); ctx.lineTo(PAD.l + pw, y); ctx.stroke();
        ctx.fillStyle = INK.muted; ctx.textAlign = 'right';
        ctx.fillText(tickLabel(v, spec), PAD.l - 4, y);
    }
    // Time labels
    ctx.textBaseline = 'alphabetic'; ctx.fillStyle = INK.muted;
    ctx.textAlign = 'left'; ctx.fillText('−20 s', PAD.l, h - 2);
    ctx.textAlign = 'center'; ctx.fillText('−10 s', PAD.l + pw / 2, h - 2);
    ctx.textAlign = 'right'; ctx.fillText('now', PAD.l + pw, h - 2);

    // Thresholds: dashed, labelled; status colour only where the level is a state
    let lastLabelY = Infinity;
    for (const m of spec.marks || []) {
        if (m.v < lo || m.v > hi) continue;
        const y = Math.round(Y(m.v)) + 0.5;
        ctx.strokeStyle = m.status ? STATUS[m.status] : INK.secondary;
        ctx.globalAlpha = 0.8; ctx.setLineDash([4, 3]);
        ctx.beginPath(); ctx.moveTo(PAD.l, y); ctx.lineTo(PAD.l + pw, y); ctx.stroke();
        ctx.setLineDash([]); ctx.globalAlpha = 1;
        // Squeezed together on a wide scale: the lower label alone
        if (Math.abs(lastLabelY - y) < 10) continue;
        lastLabelY = y;
        ctx.textAlign = 'left'; ctx.textBaseline = 'bottom';
        // On a backing of the bar's black, so a line passing under it cannot hide it
        ctx.fillStyle = 'rgba(5,8,12,0.85)';
        ctx.fillRect(PAD.l + 1, y - 11, ctx.measureText(m.label).width + 4, 10);
        ctx.fillStyle = INK.secondary;
        ctx.fillText(m.label, PAD.l + 3, y - 1);
    }

    // Events (resamples): ticks on the baseline
    if (spec.events) {
        const s = monitor.get(spec.events);
        ctx.strokeStyle = INK.secondary; ctx.lineWidth = 1;
        for (let i = 0; i < s.length; i++) if (s.t[i] >= t0) {
            const x = Math.round(X(s.t[i])) + 0.5;
            ctx.beginPath(); ctx.moveTo(x, PAD.t + ph); ctx.lineTo(x, PAD.t + ph - 4 - Math.min(6, s.v[i])); ctx.stroke();
        }
    }

    // Series: 2 px lines, clipped to the plot
    ctx.save();
    ctx.beginPath(); ctx.rect(PAD.l, PAD.t - 1, pw, ph + 2); ctx.clip();
    ctx.lineWidth = 2; ctx.lineJoin = 'round'; ctx.lineCap = 'round';
    const ends = [];
    lines.forEach((l, i) => {
        const s = monitor.get(l.key), color = SERIES_COLORS[spec.lines.indexOf(l) >= 0 ? spec.lines.indexOf(l) : i];
        let started = false;
        ctx.strokeStyle = color;
        ctx.beginPath();
        for (let k = 0; k < s.length; k++) {
            if (s.t[k] < t0 && !(k + 1 < s.length && s.t[k + 1] >= t0)) continue;
            const x = X(s.t[k]), y = Y(s.v[k]);
            // A hole in the data (no fix, link lost) stays a hole
            if (started && s.t[k] - s.t[k - 1] <= GAP_MS) ctx.lineTo(x, y); else { ctx.moveTo(x, y); started = true; }
        }
        ctx.stroke();
        if (s.length && s.lastTime() >= t0) ends.push({ name: l.name, color, x: X(s.lastTime()), y: Y(s.last()) });
    });
    ctx.restore();

    // End points, and the names beside them (pushed apart so they never overlap)
    ctx.font = END_LABEL_FONT; ctx.textBaseline = 'middle'; ctx.textAlign = 'left';
    for (const e of ends) {
        ctx.fillStyle = 'rgba(12,18,24,0.9)';
        ctx.beginPath(); ctx.arc(e.x, e.y, 4, 0, Math.PI * 2); ctx.fill();
        ctx.fillStyle = e.color;
        ctx.beginPath(); ctx.arc(e.x, e.y, 2.5, 0, Math.PI * 2); ctx.fill();
    }
    if (lines.length > 1) {
        ends.sort((a, b) => a.y - b.y);
        for (let i = 1; i < ends.length; i++) ends[i].y = Math.max(ends[i].y, ends[i - 1].y + 10);
        const over = ends.length ? ends[ends.length - 1].y - (PAD.t + ph) : 0;
        if (over > 0) for (const e of ends) e.y -= over;
        ctx.fillStyle = INK.secondary;
        for (const e of ends) ctx.fillText(e.name, PAD.l + pw + 6, e.y);
    }

    // Hover: crosshair at the pointer's time
    if (c.hover !== null && c.hover >= PAD.l && c.hover <= PAD.l + pw) {
        ctx.strokeStyle = INK.secondary; ctx.lineWidth = 1;
        const x = Math.round(c.hover) + 0.5;
        ctx.beginPath(); ctx.moveTo(x, PAD.t); ctx.lineTo(x, PAD.t + ph); ctx.stroke();
    }

    if (!lines.some(l => monitor.get(l.key).length)) {
        ctx.fillStyle = INK.muted; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
        ctx.font = '11px "Rajdhani", sans-serif';
        ctx.fillText('NO DATA', PAD.l + pw / 2, PAD.t + ph / 2);
    }
}

function decades(lo, hi) {
    const out = [];
    for (let e = Math.round(Math.log10(lo)); e <= Math.round(Math.log10(hi)); e++) out.push(Math.pow(10, e));
    return out.length > 4 ? [out[0], out[Math.floor(out.length / 2)], out[out.length - 1]] : out;
}

function tickLabel(v, spec) {
    // One format down a log axis: powers of ten as 1e-3, 1e-2, 1e-1, 1e0
    if (spec.log) return v.toExponential(0).replace('e+', 'e');
    if (Math.abs(v) >= 10 || v === 0) return v.toFixed(0);
    return (+v.toFixed(2)).toString();
}

function updateText(now) {
    const source = monitor.source(now);
    sourceEl.textContent = source ? CHARTS[source].label + (monitor.live(now) ? '' : ' · NO DATA') : 'WAITING FOR ESTIMATOR DATA';
    flagsEl.innerHTML = flagChips(source);
    for (const c of cards) {
        const { lines, unit } = activeLines(c.spec);
        if (c.unitEl.textContent !== unit) c.unitEl.textContent = unit;
        c.chipsEl.innerHTML = c.spec.chips(monitor).map(([k, v, status]) =>
            `<span class="ekf-chip${status ? ' ' + status : ''}">${status ? '▲ ' : ''}${k} <b>${v}</b></span>`).join('');
        // Legend: always there for two or more series; values at the pointer, else the latest
        const pw = c.w - PAD.l - (c.padR ?? 10);
        const t = c.hover !== null && pw > 0 ? now - EKF_WINDOW_MS + (c.hover - PAD.l) / pw * EKF_WINDOW_MS : null;
        const keys = lines.map(l => l.key).join();
        if (keys !== c.legendKeys) {
            c.legendKeys = keys;
            c.legendEl.innerHTML = lines.map((l, i) => {
                const color = SERIES_COLORS[c.spec.lines.indexOf(l) >= 0 ? c.spec.lines.indexOf(l) : i];
                return `<span class="ekf-leg"><i style="background:${color}"></i>${l.name} <b></b></span>`;
            }).join('');
        }
        const values = c.legendEl.querySelectorAll('b');
        lines.forEach((l, i) => {
            const s = monitor.get(l.key);
            const v = t !== null ? s.at(t) : (s.length && s.lastTime() >= now - EKF_WINDOW_MS ? s.last() : null);
            const text = c.spec.log && v !== null ? v.toExponential(2) : fmt(v, c.spec.digits);
            if (values[i] && values[i].textContent !== text) values[i].textContent = text;
        });
    }
}

function chip(label, state) {
    const icon = state === 'good' ? '✓' : state === 'critical' ? '✕' : state === 'warning' ? '▲' : '•';
    return `<span class="ekf-flag ${state}">${icon} ${label}</span>`;
}

function flagChips(source) {
    if (source === 'ardupilot' && monitor.ap.flags !== null) {
        const f = monitor.ap.flags, ok = (bit, label) => chip(label, f & bit ? 'good' : 'critical');
        let out = ok(AF.ATTITUDE, 'ATT') + ok(AF.VELOCITY_HORIZ, 'VEL H') + ok(AF.VELOCITY_VERT, 'VEL V')
            + chip('POS H', f & (AF.POS_HORIZ_ABS | AF.POS_HORIZ_REL) ? 'good' : 'critical') + ok(AF.POS_VERT_ABS, 'POS V');
        if (f & AF.CONST_POS_MODE) out += chip('CONST POS', 'warning');
        if (f & AF.GPS_GLITCHING) out += chip('GPS GLITCH', 'critical');
        if (f & AF.UNINITIALIZED) out += chip('UNINITIALISED', 'critical');
        return out;
    }
    if (source === 'corv' && monitor.corv.flags !== null) {
        const f = monitor.corv.flags, ok = (bit, label) => chip(label, f & bit ? 'good' : 'critical');
        let out = ok(CF.INITIALIZED, 'INIT') + ok(CF.CONVERGED, 'CONV') + ok(CF.GPS_AVAILABLE, 'GPS')
            + ok(CF.IMU1_HEALTHY, 'IMU1') + ok(CF.IMU2_HEALTHY, 'IMU2') + ok(CF.BARO_HEALTHY, 'BARO') + ok(CF.MAG_HEALTHY, 'MAG');
        if (f & CF.ZUPT_ACTIVE) out += chip('ZUPT', 'info');
        if (f & CF.GPS_BYPASS) out += chip('GPS BYPASS', 'warning');
        if (f & CF.GPS_JAMMING) out += chip('JAMMING', 'critical');
        if (f & CF.GPS_SPOOFING) out += chip('SPOOFING', 'critical');
        return out;
    }
    return '';
}

function applyVisibility() {
    panelEl?.classList.toggle('collapsed', !enabled);
    if (toggleEl) {
        toggleEl.classList.toggle('collapsed', !enabled);
        toggleEl.querySelector('.ekf-toggle-arrow').innerHTML = enabled ? '&#9664;' : '&#9654;';
        toggleEl.title = enabled ? 'Close EKF STATUS' : 'Open EKF STATUS';
    }
    // The flight view gives the bar its width: 3D and HUD resize to what is left
    if (document.body.classList.contains('ekf-docked') !== enabled) {
        document.body.classList.toggle('ekf-docked', enabled);
        requestAnimationFrame(() => window.dispatchEvent(new Event('resize')));
    }
}

// ── Public ──────────────────────────────────────────────────────────────

export function initEkfStatusPanel() {
    panelEl = document.getElementById('ekf-panel');
    if (!panelEl) return;
    toggleEl = document.getElementById('ekf-toggle');
    toggleEl?.addEventListener('click', () => setEkfStatusEnabled(!enabled));
    sourceEl = document.getElementById('ekf-source');
    flagsEl = document.getElementById('ekf-flags');
    cardsEl = document.getElementById('ekf-cards');
    try { enabled = localStorage.getItem(ENABLED_KEY) === '1'; } catch (e) {}
    subscribe();
    buildCards(null);
    applyVisibility();
}

export function setEkfStatusEnabled(on) {
    enabled = !!on;
    try { localStorage.setItem(ENABLED_KEY, enabled ? '1' : '0'); } catch (e) {}
    applyVisibility();
    lastRates = 0;
}

/** Called every animation frame; draws only while shown. */
export function updateEkfStatusPanel(visible) {
    const now = performance.now();
    manageRates(now);
    if (panelEl && visible !== onFlightTab) {
        onFlightTab = visible;
        panelEl.style.display = visible ? '' : 'none';
        if (toggleEl) toggleEl.style.display = visible ? '' : 'none';
    }
    if (!enabled || !visible || !panelEl) return;
    const source = monitor.source(now);
    if (source !== shownSource && source) buildCards(source);
    if (now - lastDraw >= DRAW_MS) {
        lastDraw = now;
        for (const c of cards) drawCard(c, now);
    }
    if (now - lastText >= TEXT_MS) {
        lastText = now;
        updateText(now);
    }
}

/** For the tests and the debug console. */
export function getEkfMonitor() { return monitor; }
