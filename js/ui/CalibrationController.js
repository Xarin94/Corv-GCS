/**
 * CalibrationController.js - SETUP > ACCELEROMETER, COMPASS, GYRO / BARO
 *
 * The procedures Mission Planner runs, over the same MAVLink exchanges.
 *
 * Accelerometer, six positions — PREFLIGHT_CALIBRATION (param5 = 1). Once the
 * gyros are sampled the autopilot asks for each position with a COMMAND_LONG
 * ACCELCAL_VEHICLE_POS, repeated every second; the GCS answers with the same
 * command and position when the vehicle is in place. ArduPilot stops sending
 * its "Place vehicle ..." texts as soon as a GCS answers that way, so this page
 * follows the commands, not the texts. The end is the same command carrying
 * SUCCESS or FAILED as the position.
 *
 * Compass — DO_START_MAG_CAL, then a MAG_CAL_PROGRESS per compass (percentage
 * and the 80-section coverage mask) and, once done, a MAG_CAL_REPORT per
 * compass with the result. ACCEPT saves a result that was not auto-saved,
 * CANCEL stops. The raw samples (RAW_IMU, SCALED_IMU2/3) feed the 3D view.
 *
 * Level, simple accel, gyros, baro — one PREFLIGHT_CALIBRATION each, whose
 * COMMAND_ACK is the result.
 *
 * Every command waits for its COMMAND_ACK and says what came back: a refused
 * start (armed, unsupported) must not look like a calibration that silently
 * never happens.
 */

import { STATE } from '../core/state.js';
import { onMessage } from '../mavlink/MAVLinkManager.js';
import {
    calibrateAccel, calibrateLevel, calibrateAccelSimple, calibrateGyro, calibrateBaro,
    sendAccelCalPosition, ACCELCAL_POS, startMagCal, acceptMagCal, cancelMagCal, magCalFixedYaw,
    setMessageInterval, resetMessageInterval, setParameter, requestParameter, rebootAutopilot
} from '../mavlink/CommandSender.js';
import { setNavDot } from './TabController.js';
import { MagCal3D, MAG_COLORS } from './MagCal3D.js';

const $ = id => document.getElementById(id);

const RESULT_TEXT = {
    1: 'temporarily rejected — try again', 2: 'denied', 3: 'not supported by this firmware',
    4: 'failed', 5: 'still in progress', 6: 'cancelled'
};
const resultText = ack => RESULT_TEXT[ack.result] || `result ${ack.result}`;

// Autopilot texts worth repeating next to a calibration (the rest is chatter)
const CAL_TEXT = /cal|trim|baro|gyro|accel|compass|mag|disarm|level|sample|still|place|reboot|offset|fit/i;

// STATUSTEXT goes to whichever procedure is listening
const textListeners = new Set();

/** Text plus a state class (running / ok / error) on a status line */
function setResult(el, text, kind = '') {
    if (!el) return;
    const base = el.dataset.base || (el.dataset.base = el.classList[0] || 'cal-result');
    el.textContent = text;
    el.className = kind ? `${base} ${kind}` : base;
}

function linked(out) {
    if (STATE.connected) return true;
    setResult(out, 'Not connected to a vehicle.', 'error');
    return false;
}

function escapeHtml(s) {
    return String(s).replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
}

const paramValue = name => {
    const p = STATE.parameters.get(name);
    return p && Number.isFinite(p.value) ? p.value : null;
};

// ── Parameters shown on these pages ───────────────────────────────────────────

const ACCEL_IMUS = ['', '2', '3'].map(n => ({
    id: `INS_ACC${n}_ID`,
    ofs: ['X', 'Y', 'Z'].map(a => `INS_ACC${n}OFFS_${a}`),
    scal: ['X', 'Y', 'Z'].map(a => `INS_ACC${n}SCAL_${a}`)
}));
const ACCEL_PARAMS = [...ACCEL_IMUS.flatMap(i => [i.id, ...i.ofs, ...i.scal]), 'AHRS_TRIM_X', 'AHRS_TRIM_Y'];

// Device, orientation, mounting and offsets are stored per detected slot;
// "use for yaw" per priority (COMPASS_PRIOn_ID says which device that is).
const COMPASS_SLOTS = ['', '2', '3'].map((n, i) => ({
    dev: `COMPASS_DEV_ID${n}`,
    ext: ['COMPASS_EXTERNAL', 'COMPASS_EXTERN2', 'COMPASS_EXTERN3'][i],
    orient: `COMPASS_ORIENT${n}`,
    ofs: ['X', 'Y', 'Z'].map(a => `COMPASS_OFS${n}_${a}`)
}));
const COMPASS_PRIOS = [1, 2, 3].map((n, i) => ({ id: `COMPASS_PRIO${n}_ID`, use: ['COMPASS_USE', 'COMPASS_USE2', 'COMPASS_USE3'][i] }));
const COMPASS_PARAMS = [
    ...COMPASS_SLOTS.flatMap(s => [s.dev, s.ext, s.orient, ...s.ofs]),
    ...COMPASS_PRIOS.flatMap(p => [p.id, p.use]),
    'COMPASS_CAL_FIT'
];

/** Ask for the parameters a page shows; only the ones not loaded yet, unless forced */
function requestParams(names, force = false) {
    if (!STATE.connected) return;
    for (const n of names) {
        if (force || !STATE.parameters.has(n)) requestParameter(n).catch(() => {});
    }
}

// AP_HAL::Device bus types and AP_Compass_Backend::DevTypes
const BUS_TYPES = { 1: 'I2C', 2: 'SPI', 3: 'DroneCAN', 4: 'SITL', 5: 'MSP', 6: 'Serial', 7: 'WSPI' };
const COMPASS_TYPES = {
    0x01: 'HMC5883 (old)', 0x07: 'HMC5883', 0x02: 'LSM303D', 0x04: 'AK8963', 0x05: 'BMM150', 0x06: 'LSM9DS1',
    0x08: 'LIS3MDL', 0x09: 'AK09916', 0x0A: 'IST8310', 0x0B: 'ICM20948', 0x0C: 'MMC3416', 0x0D: 'QMC5883L',
    0x0E: 'MAG3110', 0x0F: 'SITL', 0x10: 'IST8308', 0x11: 'RM3100', 0x12: 'RM3100', 0x13: 'MMC5983',
    0x14: 'AK09918', 0x15: 'AK09915', 0x16: 'QMC5883P', 0x17: 'BMM350', 0x18: 'IIS2MDC', 0x1A: 'AF9838'
};

function describeDevice(devId) {
    if (!devId) return null;
    const busType = devId & 0x7, bus = (devId >> 3) & 0x1f, address = (devId >> 8) & 0xff, type = (devId >> 16) & 0xff;
    const busName = BUS_TYPES[busType] || `bus ${busType}`;
    const hex = n => '0x' + n.toString(16).toUpperCase().padStart(2, '0');
    let where = `${busName}${bus}`;
    if (busType === 1) where += ` ${hex(address)}`;
    else if (busType === 3) where += ` node ${address}`;
    else if (busType === 4) where = 'SITL';
    return { name: COMPASS_TYPES[type] || `type ${hex(type)}`, where };
}

// ArduPilot's Rotation enum, in order (rotations.h)
const ROTATIONS = [
    'None', 'Yaw45', 'Yaw90', 'Yaw135', 'Yaw180', 'Yaw225', 'Yaw270', 'Yaw315', 'Roll180', 'Roll180Yaw45',
    'Roll180Yaw90', 'Roll180Yaw135', 'Pitch180', 'Roll180Yaw225', 'Roll180Yaw270', 'Roll180Yaw315', 'Roll90',
    'Roll90Yaw45', 'Roll90Yaw90', 'Roll90Yaw135', 'Roll270', 'Roll270Yaw45', 'Roll270Yaw90', 'Roll270Yaw135',
    'Pitch90', 'Pitch270', 'Pitch180Yaw90', 'Pitch180Yaw270', 'Roll90Pitch90', 'Roll180Pitch90', 'Roll270Pitch90',
    'Roll90Pitch180', 'Roll270Pitch180', 'Roll90Pitch270', 'Roll180Pitch270', 'Roll270Pitch270',
    'Roll90Pitch180Yaw90', 'Roll90Yaw270', 'Roll90Pitch68Yaw293', 'Pitch315', 'Roll90Pitch315', 'Pitch7',
    'Roll45', 'Roll315'
];
const rotationName = v => v === null ? '—' : (v >= 100 ? `Custom ${v - 100 || ''}`.trim() : (ROTATIONS[v] || `#${v}`));

// ════════════════════════════════════════════════════════════════════════════
// ACCELEROMETER
// ════════════════════════════════════════════════════════════════════════════

// ACCELCAL_VEHICLE_POS 1..6. `view` is a side view (nose to the right) or a
// view from behind (red = left, green = right, as navigation lights); `lift`
// sits the drawing on the ground line.
const POSITIONS = [
    { pos: 1, title: 'LEVEL',       caption: 'flat, as it flies',            say: 'LEVEL',                    view: 'side',  rot: 0,   lift: 5 },
    { pos: 2, title: 'LEFT SIDE',   caption: 'left side down',               say: 'on its LEFT side',         view: 'rear',  rot: -90, lift: 25 },
    { pos: 3, title: 'RIGHT SIDE',  caption: 'right side down',              say: 'on its RIGHT side',        view: 'rear',  rot: 90,  lift: 25 },
    { pos: 4, title: 'NOSE DOWN',   caption: 'nose pointing at the ground',  say: 'NOSE DOWN',                view: 'side',  rot: 90,  lift: 32 },
    { pos: 5, title: 'NOSE UP',     caption: 'nose pointing at the sky',     say: 'NOSE UP',                  view: 'side',  rot: -90, lift: 25 },
    { pos: 6, title: 'BACK',        caption: 'upside down',                  say: 'on its BACK (upside down)', view: 'side', flip: true, lift: 16.5 },
];

function positionGlyph(p) {
    const ground = 70;
    const shape = p.view === 'side'
        ? '<rect class="body" x="-24" y="-4" width="46" height="8" rx="2"/>'
          + '<polygon class="nose" points="22,-4.5 31,0 22,4.5"/>'
          + '<line class="mast" x1="-6" y1="-4" x2="-6" y2="-11"/><circle class="mast-top" cx="-6" cy="-13" r="2.5"/>'
        : '<rect class="body" x="-21" y="-3.5" width="42" height="7" rx="2"/>'
          + '<line class="mast" x1="0" y1="-3.5" x2="0" y2="-11"/><circle class="mast-top" cx="0" cy="-13" r="2.5"/>'
          + '<circle class="nav-left" cx="-21" cy="0" r="3"/><circle class="nav-right" cx="21" cy="0" r="3"/>';
    const turn = p.flip ? 'scale(1,-1)' : `rotate(${p.rot})`;
    return `<svg class="accel-glyph" viewBox="0 0 100 80" aria-hidden="true">`
        + `<line class="ground" x1="8" y1="${ground}" x2="92" y2="${ground}"/>`
        + `<g transform="translate(50 ${ground - p.lift}) ${turn}">${shape}</g></svg>`;
}

const ACCEL_ACTIVE = new Set(['starting', 'waiting', 'position', 'sampling']);
let accel = freshAccel();
let accelEndedAt = 0;

function freshAccel() {
    return { phase: 'idle', pos: 0, done: new Set(), confirmedPos: 0, confirmedAt: 0, timer: null };
}

function initAccel() {
    const steps = $('accel-steps');
    if (!steps) return;
    steps.innerHTML = POSITIONS.map(p => `
        <div class="accel-step" data-pos="${p.pos}">
            ${positionGlyph(p)}
            <div class="accel-step-title"><span class="accel-step-num">${p.pos}</span>${p.title}</div>
            <div class="accel-step-caption">${p.caption}${p.view === 'rear' ? ' · seen from behind' : ''}</div>
        </div>`).join('');

    $('accel-cal-start')?.addEventListener('click', () => {
        if (accel.phase === 'position') continueAccel();
        else if (!ACCEL_ACTIVE.has(accel.phase)) startAccel();
    });
    $('accel-cal-abort')?.addEventListener('click', abortAccel);

    $('accel-level-start')?.addEventListener('click', () => runOneShot('accel-level-start', 'accel-level-status', calibrateLevel, {
        running: 'Levelling — keep the vehicle still…',
        done: 'Level saved (AHRS_TRIM_X / Y).',
        after: () => setTimeout(() => requestParams(['AHRS_TRIM_X', 'AHRS_TRIM_Y'], true), 500)
    }));
    $('accel-simple-start')?.addEventListener('click', () => runOneShot('accel-simple-start', 'accel-simple-status', calibrateAccelSimple, {
        running: 'Calibrating — level and still…',
        done: 'Accelerometers calibrated. Reboot the autopilot before arming.',
        after: () => setTimeout(() => requestParams(ACCEL_PARAMS, true), 500)
    }));
    $('accel-params-read')?.addEventListener('click', () => requestParams(ACCEL_PARAMS, true));

    // ACCELCAL_VEHICLE_POS arrives as a COMMAND_LONG from the vehicle
    onMessage(76, (data) => {
        if (data.command === 42429) onAccelPosition(Math.round(data._param1));
    });
    textListeners.add((text) => {
        if (!ACCEL_ACTIVE.has(accel.phase)) return;
        logAccel(text);
        // Fallback for the end: the text comes about a second before the command
        if (accel.pos > 0 && /calibration successful/i.test(text)) finishAccel(true);
        else if (accel.pos > 0 && /calibration (failed|cancelled)/i.test(text)) finishAccel(false, text);
    });
    renderAccel();
}

async function startAccel() {
    const out = $('accel-instruction');
    if (!linked(out)) return;
    accel = freshAccel();
    accel.phase = 'starting';
    $('accel-cal-log').textContent = '';
    setNavDot('accel-cal', true);
    renderAccel('Measuring the gyros first — keep the vehicle still…');
    let ack;
    try {
        ack = await calibrateAccel();
    } catch (e) {
        return failAccel(`No answer from the autopilot (${e.message}).`);
    }
    if (accel.phase !== 'starting') return;          // the first position came first
    if (ack.result !== 0) return failAccel(`The autopilot refused to start: ${resultText(ack)}.`);
    accel.phase = 'waiting';
    renderAccel('Waiting for the autopilot to ask for the first position…');
    accel.timer = setTimeout(() => {
        if (accel.phase === 'waiting') {
            failAccel('The autopilot accepted but never asked for a position (firmware without MAVLink accelerometer calibration?).');
        }
    }, 8000);
}

function onAccelPosition(pos) {
    if (pos === ACCELCAL_POS.SUCCESS || pos === ACCELCAL_POS.FAILED) {
        // Repeated every second after a calibration ends, until the next one
        // starts: only an end of the run this page has seen counts.
        if (ACCEL_ACTIVE.has(accel.phase) && accel.pos > 0) finishAccel(pos === ACCELCAL_POS.SUCCESS);
        return;
    }
    if (pos < 1 || pos > 6) return;
    // The repeat of a request our answer crossed on the way
    if (pos === accel.confirmedPos && Date.now() - accel.confirmedAt < 4000) return;
    if (!ACCEL_ACTIVE.has(accel.phase)) {
        if (Date.now() - accelEndedAt < 4000) return;      // a late request from the run that just ended
        // Started elsewhere (another GCS, or before this page was opened): follow it
        accel = freshAccel();
        accel.phase = 'waiting';
        setNavDot('accel-cal', true);
    }
    if (pos === accel.pos && accel.phase === 'position') return;

    clearTimeout(accel.timer);
    for (let p = 1; p < pos; p++) accel.done.add(p);
    accel.pos = pos;
    accel.phase = 'position';
    const p = POSITIONS[pos - 1];
    renderAccel(`Place the vehicle ${p.say} and hold it still, then press CONTINUE.`);
}

async function continueAccel() {
    const pos = accel.pos;
    accel.phase = 'sampling';
    accel.confirmedPos = pos;
    accel.confirmedAt = Date.now();
    renderAccel('Sampling — hold still…');
    let ack;
    try {
        ack = await sendAccelCalPosition(pos);
    } catch (e) {
        ack = { result: -1 };
    }
    if (accel.phase !== 'sampling' || accel.pos !== pos) return;
    if (ack.result !== 0) {
        accel.phase = 'position';
        accel.confirmedPos = 0;
        renderAccel(`The autopilot did not take position ${pos} (${ack.result === -1 ? 'no answer' : resultText(ack)}). Hold it still and press CONTINUE again.`);
    }
}

function finishAccel(ok, why = '') {
    if (!ACCEL_ACTIVE.has(accel.phase)) return;
    clearTimeout(accel.timer);
    accelEndedAt = Date.now();
    if (ok) {
        POSITIONS.forEach(p => accel.done.add(p.pos));
        accel.phase = 'done';
        renderAccel('Calibration successful — offsets saved. Reboot the autopilot before arming.');
        setTimeout(() => requestParams(ACCEL_PARAMS, true), 1000);
    } else {
        accel.phase = 'failed';
        renderAccel(`Calibration FAILED${why && !/failed/i.test(why) ? ` — ${why}` : ''}. Hold the vehicle still and square in every position, then start again.`);
    }
    setNavDot('accel-cal', false);
}

function failAccel(text) {
    clearTimeout(accel.timer);
    accelEndedAt = Date.now();
    accel.phase = 'failed';
    setNavDot('accel-cal', false);
    renderAccel(text);
}

async function abortAccel() {
    if (!ACCEL_ACTIVE.has(accel.phase)) return;
    const reboot = await confirm('MAVLink cannot cancel an accelerometer calibration: the autopilot keeps waiting for the positions until it is rebooted.\n\nReboot the autopilot now?');
    clearTimeout(accel.timer);
    accel = freshAccel();
    accelEndedAt = Date.now();
    setNavDot('accel-cal', false);
    if (reboot) {
        try { await rebootAutopilot(); } catch (e) { /* the link drops with the reboot */ }
        renderAccel('Aborted — autopilot rebooting.');
    } else {
        renderAccel('Aborted here — the autopilot is still waiting until it reboots.');
    }
}

function logAccel(text) {
    const log = $('accel-cal-log');
    if (!log) return;
    const lines = (log.textContent ? log.textContent.split('\n') : []).concat(text).slice(-5);
    log.textContent = lines.join('\n');
}

function renderAccel(instruction) {
    const active = ACCEL_ACTIVE.has(accel.phase);
    document.querySelectorAll('#accel-steps .accel-step').forEach(el => {
        const pos = Number(el.dataset.pos);
        el.classList.toggle('done', accel.done.has(pos) && !(active && pos === accel.pos));
        el.classList.toggle('current', active && pos === accel.pos && accel.phase === 'position');
        el.classList.toggle('sampling', active && pos === accel.pos && accel.phase === 'sampling');
    });
    const btn = $('accel-cal-start');
    if (btn) {
        btn.textContent = accel.phase === 'position' ? `CONTINUE (${POSITIONS[accel.pos - 1].title})`
            : accel.phase === 'sampling' ? 'SAMPLING…'
            : active ? 'STARTING…'
            : 'CALIBRATE ACCEL';
        btn.disabled = active && accel.phase !== 'position';
        btn.classList.toggle('is-armed', accel.phase === 'position');
    }
    const abort = $('accel-cal-abort');
    if (abort) abort.hidden = !active;
    if (instruction !== undefined) {
        setResult($('accel-instruction'), instruction,
            accel.phase === 'done' ? 'ok' : accel.phase === 'failed' ? 'error' : active ? 'running' : '');
    }
}

function renderAccelParams() {
    const table = $('accel-params');
    if (!table) return;
    const fmt = (v, d) => v === null ? '—' : v.toFixed(d);
    let rows = '';
    ACCEL_IMUS.forEach((imu, i) => {
        const id = paramValue(imu.id);
        if (i > 0 && !id) return;                          // no second / third IMU
        const ofs = imu.ofs.map(paramValue), scal = imu.scal.map(paramValue);
        const untouched = ofs.every(v => v === 0) && scal.every(v => v === 0 || v === 1);
        rows += `<tr><td>IMU ${i + 1}</td>
            <td>${ofs.map(v => fmt(v, 3)).join(' / ')}</td>
            <td>${scal.map(v => fmt(v, 3)).join(' / ')}</td>
            <td class="${untouched ? 'warn' : ''}">${ofs.includes(null) ? 'not read' : untouched ? 'not calibrated' : 'calibrated'}</td></tr>`;
    });
    const tx = paramValue('AHRS_TRIM_X'), ty = paramValue('AHRS_TRIM_Y');
    const deg = v => v === null ? '—' : `${(v * 180 / Math.PI).toFixed(2)}°`;
    rows += `<tr><td>LEVEL</td><td colspan="3">roll trim ${deg(tx)} · pitch trim ${deg(ty)}</td></tr>`;
    table.innerHTML = `<thead><tr><th></th><th>OFFSETS X / Y / Z (m/s²)</th><th>SCALE X / Y / Z</th><th></th></tr></thead><tbody>${rows}</tbody>`;
}

// ── One-command calibrations ─────────────────────────────────────────────────

async function runOneShot(btnId, outId, command, { running, done, after }) {
    const btn = $(btnId), out = $(outId);
    if (!linked(out)) return;
    btn.disabled = true;
    setResult(out, running, 'running');
    let said = '';
    const listen = (text) => { if (CAL_TEXT.test(text)) { said = text; setResult(out, `${running}\n${text}`, 'running'); } };
    textListeners.add(listen);
    try {
        const ack = await command();
        if (ack.result === 0) {
            setResult(out, said && !/^(calibrating|updating)/i.test(said) ? `${done}\n${said}` : done, 'ok');
            after?.();
        } else {
            setResult(out, `Refused: ${resultText(ack)}.${said ? `\n${said}` : ''}`, 'error');
        }
    } catch (e) {
        setResult(out, `No answer from the autopilot (${e.message}).`, 'error');
    } finally {
        textListeners.delete(listen);
        btn.disabled = false;
    }
}

function initSensors() {
    $('gyro-cal-start')?.addEventListener('click', () => runOneShot('gyro-cal-start', 'gyro-cal-status', calibrateGyro, {
        running: 'Measuring the gyros — keep the vehicle completely still…',
        done: 'Gyro offsets updated.'
    }));
    $('baro-cal-start')?.addEventListener('click', () => runOneShot('baro-cal-start', 'baro-cal-status', calibrateBaro, {
        running: 'Sampling ground pressure…',
        done: 'Barometer zeroed: the altitude now reads 0 here.'
    }));
}

// ════════════════════════════════════════════════════════════════════════════
// COMPASS
// ════════════════════════════════════════════════════════════════════════════

// MAG_CAL_STATUS, extended with ArduPilot's CompassCalibrator::Status
const MAG_STATUS = {
    0: 'NOT STARTED', 1: 'WAITING', 2: 'STEP 1', 3: 'STEP 2', 4: 'SUCCESS', 5: 'FAILED',
    6: 'BAD ORIENTATION', 7: 'BAD RADIUS', 8: 'BAD OFFSETS', 9: 'BAD SCALING', 10: 'RESIDUALS HIGH'
};
const MAG_FAIL_HINT = {
    5: 'the samples do not fit a sphere — turn the vehicle more slowly through every attitude, away from metal',
    6: 'the compass orientation does not match the autopilot — check COMPASS_ORIENT',
    7: 'the field strength is out of range — metal or magnets nearby, or the wrong compass setup',
    8: 'the offsets are too large (COMPASS_OFFS_MAX) — interference close to the compass',
    9: 'the scale factors are out of range — interference or a damaged sensor',
    10: 'the fit residuals are too high — try FITNESS Relaxed, or move away from interference'
};
const isMagRunning = s => s >= 1 && s <= 3;
const isMagFinal = s => s >= 4;

let mag = freshMag();
let view = null;
let statsTimer = null;
let finishTimer = null;

function freshMag() {
    return { running: false, starting: false, calMask: 0, progress: [], reports: [], said: '' };
}

function initCompass() {
    if (!$('subtab-compass-cal')) return;
    buildMagBars();
    buildLegend();

    $('mag-cal-start')?.addEventListener('click', startMag);
    $('mag-cal-accept')?.addEventListener('click', acceptMag);
    $('mag-cal-cancel')?.addEventListener('click', cancelMag);
    $('mag-cal-reboot')?.addEventListener('click', rebootAfterCal);
    $('mag-cal-fit')?.addEventListener('change', async (e) => {
        const out = $('mag-cal-instruction');
        if (!linked(out)) return;
        try {
            await setParameter('COMPASS_CAL_FIT', Number(e.target.value));
            setResult(out, `COMPASS_CAL_FIT set to ${e.target.value}.`);
        } catch (err) {
            setResult(out, `Could not set COMPASS_CAL_FIT: ${err.message}`, 'error');
        }
    });
    $('mag3d-focus')?.addEventListener('change', (e) => setFocus(Number(e.target.value)));
    $('mag3d-clear')?.addEventListener('click', () => { view?.clear(); refreshCoverage(); updateStats(); });
    $('mag3d-reset')?.addEventListener('click', () => view?.resetView());
    $('mag-fixed-start')?.addEventListener('click', () => {
        const yaw = Number($('mag-fixed-yaw')?.value);
        const out = $('mag-fixed-status');
        if (!Number.isFinite(yaw) || yaw < 0 || yaw >= 360) { setResult(out, 'Enter a heading from 0 to 359.9°.', 'error'); return; }
        runOneShot('mag-fixed-start', 'mag-fixed-status', () => magCalFixedYaw(yaw), {
            running: `Calibrating from a ${yaw}° heading…`,
            done: 'Compass offsets set from the heading. Reboot the autopilot to use them.',
            after: () => setTimeout(() => requestParams(COMPASS_PARAMS, true), 800)
        });
    });
    $('compass-list-read')?.addEventListener('click', () => requestParams(COMPASS_PARAMS, true));

    // The magnetometers: RAW_IMU carries the first compass by priority, SCALED_IMU2 / 3 the others
    const feed = (compass) => (d) => {
        if (!view || !(mag.running || compassPageShown())) return;
        view.addSample(compass, d.xmag, d.ymag, d.zmag);
    };
    onMessage(27, feed(0));
    onMessage(116, feed(1));
    onMessage(129, feed(2));
    onMessage(191, onMagProgress);
    onMessage(192, onMagReport);
    // "Disarm to allow compass calibration" and the like arrive before the refusal
    textListeners.add((text) => {
        if ((mag.starting || mag.running) && /fail|bad|disarm|healthy|enable/i.test(text)) mag.said = text;
    });
}

function compassPageShown() {
    return !!$('subtab-compass-cal')?.classList.contains('active')
        && !!document.getElementById('tab-setup')?.classList.contains('active');
}

/** Called whenever the COMPASS page is opened: the WebGL view is made on first sight */
function onCompassPageShown() {
    if (!view) {
        const canvas = $('mag-cal-canvas');
        if (!canvas || typeof THREE === 'undefined') return;
        view = new MagCal3D(canvas);
        view.setFocus(Number($('mag3d-focus')?.value) || 0);
        statsTimer = setInterval(updateStats, 250);
    }
    requestParams(COMPASS_PARAMS);
    view.requestRender();
}

function setFocus(i) {
    view?.setFocus(i);
    refreshCoverage();
    updateStats();
}

function refreshCoverage() {
    const focus = Number($('mag3d-focus')?.value) || 0;
    const p = mag.progress[focus];
    view?.setCoverage(mag.running && p && isMagRunning(p.status) ? p.mask : null);
}

// MAG_CAL_PROGRESS / REPORT ride on the EXTRA3 stream and the magnetometers on
// RAW_SENSORS, both of which can be slow or off: ask for them by message while
// a calibration runs, then hand them back.
async function raiseRates() {
    const want = [[191, 5], [192, 2], [27, 20], [116, 20], [129, 20]];
    await Promise.allSettled(want.map(([id, hz]) => setMessageInterval(id, hz)));
}

function restoreRates() {
    if (!STATE.connected) return;
    for (const id of [191, 192, 116, 129]) resetMessageInterval(id).catch(() => {});
    setMessageInterval(27, 10).catch(() => {});              // what requestAllDataStreams asks for
}

async function startMag() {
    const out = $('mag-cal-instruction');
    if (!linked(out)) return;
    const autosave = !!$('mag-cal-autoaccept')?.checked;
    clearTimeout(finishTimer);
    mag = freshMag();
    mag.starting = true;
    view?.clear();
    $('mag-cal-results').hidden = true;
    $('mag-cal-reboot').hidden = true;
    renderMagBars();
    setButtons({ start: false, accept: false, cancel: false });
    setInstruction('Starting…', 'running');

    await raiseRates();
    let ack;
    try {
        ack = await startMagCal({ autosave });
    } catch (e) {
        mag.starting = false;
        setButtons({ start: true, accept: false, cancel: false });
        restoreRates();
        return setInstruction(`No answer from the autopilot (${e.message}).`, 'error');
    }
    mag.starting = false;
    if (ack.result !== 0) {
        setButtons({ start: true, accept: false, cancel: false });
        restoreRates();
        const said = mag.said ? ` ${mag.said}.` : ' Disarm, and check that a compass is enabled.';
        return setInstruction(`The autopilot refused to start: ${resultText(ack)}.${said}`, 'error');
    }
    if (mag.running) return;                                  // progress came first and took over
    mag.running = true;
    setNavDot('compass-cal', true);
    setButtons({ start: false, accept: false, cancel: true });
    setInstruction('Turn the vehicle slowly through every attitude…', 'running');
}

function onMagProgress(d) {
    const id = d.compassId;
    if (!(id >= 0 && id < 3)) return;
    if (!mag.running) {
        if (!isMagRunning(d.calStatus)) return;
        // Started elsewhere, or before this page was opened: follow it
        mag = freshMag();
        mag.running = true;
        setNavDot('compass-cal', true);
        setButtons({ start: false, accept: false, cancel: true });
        $('mag-cal-results').hidden = true;
    }
    mag.calMask = d.calMask || mag.calMask;
    mag.progress[id] = { status: d.calStatus, pct: d.completionPct, attempt: d.attempt, mask: d.completionMask };
    renderMagBars();
    if (id === (Number($('mag3d-focus')?.value) || 0)) refreshCoverage();
    if (isMagRunning(d.calStatus)) clearTimeout(finishTimer);  // a retry is under way
    updateMagInstruction();
}

function onMagReport(d) {
    const id = d.compassId;
    if (!(id >= 0 && id < 3) || !isMagFinal(d.calStatus)) return;
    mag.reports[id] = d;
    mag.calMask = d.calMask || mag.calMask;
    const before = mag.progress[id];
    mag.progress[id] = { status: d.calStatus, pct: d.calStatus === 4 ? 100 : (before?.pct ?? 0), attempt: before?.attempt ?? 0, mask: before?.mask };
    renderMagBars();
    renderMagResults();
    if (!mag.running) return;
    // With retries on, a failure can be followed by a new attempt: decide once
    // nothing has been running for a moment.
    clearTimeout(finishTimer);
    finishTimer = setTimeout(() => {
        if (mag.running && calibratingIds().every(i => isMagFinal(mag.progress[i]?.status))) finishMag();
    }, 1500);
}

/** The compasses in this calibration: cal_mask, or every one heard from */
function calibratingIds() {
    const ids = [];
    for (let i = 0; i < 3; i++) {
        if (mag.calMask ? (mag.calMask & (1 << i)) : mag.progress[i]) ids.push(i);
    }
    return ids;
}

function finishMag() {
    mag.running = false;
    setNavDot('compass-cal', false);
    restoreRates();
    refreshCoverage();
    const ids = calibratingIds();
    const ok = ids.filter(i => mag.progress[i]?.status === 4);
    const bad = ids.filter(i => mag.progress[i]?.status !== 4);
    const unsaved = ok.filter(i => !mag.reports[i]?.autosaved);
    setButtons({ start: true, accept: unsaved.length > 0, cancel: false });

    if (bad.length) {
        const why = bad.map(i => `MAG ${i + 1}: ${MAG_FAIL_HINT[mag.progress[i]?.status] || MAG_STATUS[mag.progress[i]?.status] || 'failed'}`).join('. ');
        setInstruction(`${why}. Press START to try again${ok.length ? ', or ACCEPT to keep the compasses that passed' : ''}.`, 'error');
    } else if (unsaved.length) {
        setInstruction('Good result. Press ACCEPT to save it.', 'ok');
    } else {
        setInstruction('Calibration saved. Reboot the autopilot to use the new offsets.', 'ok');
        $('mag-cal-reboot').hidden = false;
    }
    setTimeout(() => requestParams(COMPASS_PARAMS, true), 1500);
}

async function acceptMag() {
    const ok = calibratingIds().filter(i => mag.progress[i]?.status === 4);
    const mask = ok.reduce((m, i) => m | (1 << i), 0);
    if (!mask) return;
    try {
        const ack = await acceptMagCal(mask);
        if (ack.result !== 0) return setInstruction(`ACCEPT refused: ${resultText(ack)}.`, 'error');
    } catch (e) {
        return setInstruction(`No answer from the autopilot (${e.message}).`, 'error');
    }
    setButtons({ start: true, accept: false, cancel: false });
    setInstruction('Calibration saved. Reboot the autopilot to use the new offsets.', 'ok');
    $('mag-cal-reboot').hidden = false;
    setTimeout(() => requestParams(COMPASS_PARAMS, true), 800);
}

async function cancelMag() {
    try {
        await cancelMagCal(0);
    } catch (e) { /* stop here regardless */ }
    clearTimeout(finishTimer);
    mag.running = false;
    setNavDot('compass-cal', false);
    restoreRates();
    refreshCoverage();
    setButtons({ start: true, accept: false, cancel: false });
    setInstruction('Cancelled. Nothing was saved.', '');
}

async function rebootAfterCal() {
    if (!await confirm('Reboot the autopilot now?')) return;
    try {
        await rebootAutopilot();
        setInstruction('Rebooting… reconnects by itself on most links.', '');
        $('mag-cal-reboot').hidden = true;
    } catch (e) {
        setInstruction(`Reboot failed: ${e.message}`, 'error');
    }
}

function setButtons({ start, accept, cancel }) {
    $('mag-cal-start').disabled = !start;
    $('mag-cal-accept').disabled = !accept;
    $('mag-cal-cancel').disabled = !cancel;
}

const setInstruction = (text, kind) => setResult($('mag-cal-instruction'), text, kind);

function updateMagInstruction() {
    if (!mag.running) return;
    const focus = Number($('mag3d-focus')?.value) || 0;
    const p = mag.progress[focus] || mag.progress.find(Boolean);
    if (!p) return;
    if (p.status === 1) return setInstruction('Waiting to start — keep the vehicle ready…', 'running');
    const st = view?.stats();
    const covered = st?.covered ?? null;
    const step = p.status === 3 ? 'Step 2 of 2: once more, every attitude. ' : '';
    const sphere = covered === null ? '' : `${covered}/80 directions covered. `;
    setInstruction(`${step}${sphere}Turn the vehicle until the white marker reaches the orange patch.`, 'running');
}

function buildMagBars() {
    const box = $('mag-cal-bars');
    if (!box) return;
    box.innerHTML = [0, 1, 2].map(i => `
        <div class="mag-bar" data-mag="${i}">
            <span class="mag-bar-name"><span class="mag-swatch" style="background:${MAG_COLORS[i]}"></span>MAG ${i + 1}</span>
            <div class="mag-bar-track"><div class="mag-bar-fill"></div></div>
            <span class="mag-bar-text">—</span>
        </div>`).join('');
}

function renderMagBars() {
    const ids = new Set(calibratingIds());
    document.querySelectorAll('#mag-cal-bars .mag-bar').forEach(row => {
        const i = Number(row.dataset.mag);
        const p = mag.progress[i];
        const fill = row.querySelector('.mag-bar-fill');
        const text = row.querySelector('.mag-bar-text');
        row.classList.toggle('idle', !p && !ids.has(i));
        row.classList.toggle('ok', p?.status === 4);
        row.classList.toggle('bad', !!p && p.status >= 5);
        fill.style.width = `${p ? Math.max(0, Math.min(100, p.pct)) : 0}%`;
        text.textContent = !p ? '—'
            : `${Math.round(p.pct)}% · ${MAG_STATUS[p.status] || p.status}${p.attempt > 1 ? ` · try ${p.attempt}` : ''}`;
    });
}

function renderMagResults() {
    const table = $('mag-cal-results');
    if (!table) return;
    const rows = mag.reports.map((r, i) => {
        if (!r) return '';
        const ok = r.calStatus === 4;
        const orient = Number.isFinite(r.newOrientation) && r.newOrientation !== r.oldOrientation
            ? `${rotationName(r.oldOrientation)} → ${rotationName(r.newOrientation)}`
            : rotationName(Number.isFinite(r.oldOrientation) ? r.oldOrientation : null);
        return `<tr class="${ok ? 'ok' : 'bad'}">
            <td><span class="mag-swatch" style="background:${MAG_COLORS[i]}"></span>MAG ${i + 1}</td>
            <td>${escapeHtml(MAG_STATUS[r.calStatus] || r.calStatus)}</td>
            <td>${Number.isFinite(r.fitness) ? r.fitness.toFixed(1) : '—'}</td>
            <td>${[r.ofsX, r.ofsY, r.ofsZ].map(v => Number.isFinite(v) ? Math.round(v) : '—').join(' / ')}</td>
            <td>${Number.isFinite(r.scaleFactor) && r.scaleFactor ? r.scaleFactor.toFixed(3) : '—'}</td>
            <td>${escapeHtml(orient)}</td>
            <td>${r.autosaved ? 'saved' : ok ? 'not saved' : '—'}</td></tr>`;
    }).join('');
    table.innerHTML = `<thead><tr><th></th><th>RESULT</th><th>FITNESS (mG)</th><th>OFFSETS X / Y / Z</th><th>SCALE</th><th>ORIENTATION</th><th></th></tr></thead><tbody>${rows}</tbody>`;
    table.hidden = !rows;
}

function buildLegend() {
    const el = $('mag3d-legend');
    if (!el) return;
    const sw = (c, t, round = true) => `<span class="mag3d-key"><span class="mag-swatch${round ? '' : ' square'}" style="background:${c}"></span>${t}</span>`;
    el.innerHTML = [0, 1, 2].map(i => sw(MAG_COLORS[i], `MAG ${i + 1}`)).join('')
        + sw('#1fbf62', 'covered', false) + sw('#ffa020', 'aim here', false) + sw('#ffffff', 'now');
}

function updateStats() {
    if (!view || !compassPageShown()) return;
    const st = view.stats();
    const focus = Number($('mag3d-focus')?.value) || 0;
    const el = $('mag3d-stats');
    if (!el) return;
    const parts = [`${st.points[focus]} pts`];
    if (st.field) parts.push(`|B| ${Math.round(st.field)} mG${st.fitted ? '' : ' (no fit yet)'}`);
    if (st.covered !== null) parts.push(`${st.covered}/80`);
    el.textContent = st.points[focus] ? parts.join(' · ') : 'No samples';
    updateMagInstruction();
}

function renderCompassList() {
    const table = $('compass-list');
    if (!table) return;
    const slots = COMPASS_SLOTS.map(s => ({
        dev: paramValue(s.dev), ext: paramValue(s.ext), orient: paramValue(s.orient), ofs: s.ofs.map(paramValue)
    }));
    const havePrio = COMPASS_PRIOS.some(p => STATE.parameters.has(p.id));
    // Rows in priority order, as the autopilot uses them (older firmware: slot order)
    const rows = COMPASS_PRIOS.map((p, i) => {
        const devId = havePrio ? paramValue(p.id) : slots[i].dev;
        const slot = havePrio ? slots.find(s => s.dev && s.dev === devId) : slots[i];
        return { prio: i + 1, devId, slot, use: paramValue(p.use), useParam: p.use };
    }).filter(r => r.devId);

    if (!rows.length) {
        table.innerHTML = `<tbody><tr><td class="dim">${STATE.connected ? 'No compass parameters yet — READ FROM VEHICLE.' : 'Not connected.'}</td></tr></tbody>`;
        return;
    }
    table.innerHTML = '<thead><tr><th>PRIORITY</th><th>DEVICE</th><th>BUS</th><th>USE</th><th>MOUNT</th><th>ORIENTATION</th><th>OFFSETS X / Y / Z</th></tr></thead><tbody>'
        + rows.map(r => {
            const d = describeDevice(r.devId);
            const missing = havePrio && !r.slot;
            const mount = !r.slot || r.slot.ext === null ? '—' : r.slot.ext ? 'external' : 'internal';
            const ofs = r.slot ? r.slot.ofs.map(v => v === null ? '—' : Math.round(v)).join(' / ') : '—';
            return `<tr>
                <td>${r.prio}</td>
                <td>${escapeHtml(d.name)}${missing ? ' <span class="warn">not detected</span>' : ''}</td>
                <td>${escapeHtml(d.where)}</td>
                <td><input type="checkbox" class="compass-use" data-param="${r.useParam}" ${r.use ? 'checked' : ''} ${r.use === null ? 'disabled' : ''} title="${r.useParam}: use this compass for heading"></td>
                <td>${mount}</td>
                <td>${escapeHtml(r.slot ? rotationName(r.slot.orient) : '—')}</td>
                <td>${ofs}</td></tr>`;
        }).join('') + '</tbody>';
    table.querySelectorAll('.compass-use').forEach(cb => cb.addEventListener('change', async () => {
        try {
            await setParameter(cb.dataset.param, cb.checked ? 1 : 0);
        } catch (e) {
            cb.checked = !cb.checked;
            alert(`Could not set ${cb.dataset.param}: ${e.message}`);
        }
    }));
}

function syncFitSelect() {
    const sel = $('mag-cal-fit');
    const v = paramValue('COMPASS_CAL_FIT');
    if (!sel || v === null || document.activeElement === sel) return;
    // A value off the list gets an entry of its own rather than being misread
    if (![...sel.options].some(o => Number(o.value) === v)) {
        let custom = sel.querySelector('option[data-custom]');
        if (!custom) {
            custom = document.createElement('option');
            custom.dataset.custom = '1';
            sel.appendChild(custom);
        }
        custom.value = String(v);
        custom.textContent = `Custom (${v})`;
    }
    sel.value = [...sel.options].find(o => Number(o.value) === v).value;
}

// ════════════════════════════════════════════════════════════════════════════

export function initCalibration() {
    onMessage(253, (d) => {
        const text = (d.text || '').replace(/\0/g, '').trim();
        if (text) textListeners.forEach(fn => fn(text, d.severity));
    });

    initAccel();
    initSensors();
    initCompass();

    // Parameter tables refresh once a burst of PARAM_VALUEs has settled (the
    // parameter cache is written by a handler registered after this one)
    const watched = new Set([...ACCEL_PARAMS, ...COMPASS_PARAMS]);
    let paramTimer = null;
    onMessage(22, (d) => {
        if (!watched.has((d.paramId || '').trim())) return;
        clearTimeout(paramTimer);
        paramTimer = setTimeout(() => { renderAccelParams(); renderCompassList(); syncFitSelect(); }, 200);
    });

    document.querySelector('.setup-nav-btn[data-section="accel-cal"]')?.addEventListener('click', () => {
        requestParams(ACCEL_PARAMS);
        renderAccelParams();
    });
    document.querySelector('.setup-nav-btn[data-section="compass-cal"]')?.addEventListener('click', () => {
        onCompassPageShown();
        renderCompassList();
        syncFitSelect();
    });
    // The pages may already be open when the SETUP tab is (re)entered
    document.querySelector('.gcs-tab[data-tab="setup"]')?.addEventListener('click', () => {
        if (compassPageShown()) onCompassPageShown();
    });

    renderAccelParams();
    renderCompassList();
}
