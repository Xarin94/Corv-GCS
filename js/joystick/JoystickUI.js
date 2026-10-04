/**
 * JoystickUI.js - UI controller for joystick configuration panel
 */

import { JoystickManager } from './JoystickManager.js';
import { STATE } from '../core/state.js';
import { LOOK_DIRECTIONS } from '../input/LookAroundInput.js';

let manager = null;
let renderedDevice = '';

function currentGamepad() {
    return manager?.gamepadIndex === null ? null : navigator.getGamepads()[manager.gamepadIndex];
}

function buildButtonOptions(selected) {
    let html = '<option value="-1">Unmapped</option>';
    const count = currentGamepad()?.buttons.length || 0;
    for (let i = 0; i < count; i++) {
        html += `<option value="${i}"${selected === i ? ' selected' : ''}>Button ${i}</option>`;
    }
    if (selected >= count && selected >= 0) html += `<option value="${selected}" selected>Button ${selected} (unavailable)</option>`;
    return html;
}

function buildSourceOptions(cfg) {
    const gp = currentGamepad();
    let html = `<option value="none:0"${cfg.sourceType === 'none' ? ' selected' : ''}>Unmapped</option>`;
    for (const type of ['axis', 'button']) {
        const count = (type === 'axis' ? gp?.axes.length : gp?.buttons.length) || 0;
        const label = type === 'axis' ? 'Axis' : 'Button';
        for (let i = 0; i < count; i++) {
            const selected = cfg.sourceType === type && cfg.sourceIndex === i;
            html += `<option value="${type}:${i}"${selected ? ' selected' : ''}>${label} ${i}</option>`;
        }
        if (cfg.sourceType === type && cfg.sourceIndex >= count) {
            html += `<option value="${type}:${cfg.sourceIndex}" selected>${label} ${cfg.sourceIndex} (unavailable)</option>`;
        }
    }
    return html;
}

function renderButtonMappings() {
    const servos = document.getElementById('joystick-servo-mapping');
    if (servos) {
        servos.innerHTML = manager.servoMap.map((cfg, i) => `
            <div class="joystick-servo-row">
                <span class="joystick-axis-label">CMD ${i + 1}</span>
                <label>BUTTON<select class="cfg-select" data-servo-button="${i}">${buildButtonOptions(cfg.button)}</select></label>
                <label>OUTPUT<select class="cfg-select" data-servo-output="${i}">${Array.from({ length: 16 }, (_, n) => `<option value="${n + 1}"${cfg.servo === n + 1 ? ' selected' : ''}>Servo ${n + 1}</option>`).join('')}</select></label>
                <label>POSITION %<input class="cfg-select" type="number" min="0" max="100" step="1" data-servo-percent="${i}" value="${cfg.percent}"></label>
            </div>`).join('');
        for (const [attr, field] of [['button', 'button'], ['output', 'servo'], ['percent', 'percent']]) {
            servos.querySelectorAll(`[data-servo-${attr}]`).forEach(el => {
                el.addEventListener('change', () => {
                    const index = Number(el.getAttribute(`data-servo-${attr}`));
                    const value = el.value === '' ? NaN : Number(el.value);
                    manager.setServoConfig(index, { [field]: value });
                    el.value = String(manager.servoMap[index][field]);
                });
            });
        }
    }
    const view = document.getElementById('joystick-view-mapping');
    if (view) {
        view.innerHTML = LOOK_DIRECTIONS.map(dir => `<label>${dir.toUpperCase()}<select class="cfg-select" data-view-button="${dir}">${buildButtonOptions(manager.viewMap[dir])}</select></label>`).join('');
        view.querySelectorAll('[data-view-button]').forEach(el => el.addEventListener('change', () => {
            manager.setViewButton(el.dataset.viewButton, Number(el.value));
        }));
    }
}

function renderMappings() {
    const gp = currentGamepad();
    renderedDevice = JSON.stringify([gp?.id, gp?.axes.length, gp?.buttons.length, manager.axisMap.length]);
    renderAxisRows();
    renderButtonMappings();
}

/**
 * Build channel option HTML for axis mapping selects
 */
function buildChannelOptions(selected) {
    let html = '<option value="0">--</option>';
    for (let i = 1; i <= 18; i++) {
        html += `<option value="${i}"${selected === i ? ' selected' : ''}>CH${i}</option>`;
    }
    return html;
}

/**
 * Render axis configuration rows for the detected gamepad
 */
function renderAxisRows() {
    const container = document.getElementById('joystick-axes-container');
    if (!container || !manager) return;

    const axisCount = manager.axisMap.length;
    if (axisCount === 0) {
        container.innerHTML = '<div class="joystick-placeholder">No axes detected</div>';
        return;
    }

    let html = '';
    for (let i = 0; i < axisCount; i++) {
        const cfg = manager.axisMap[i];
        const dzPct = Math.round(cfg.deadzone * 100);
        html += `
        <div class="joystick-axis-row" data-axis="${i}">
            <span class="joystick-axis-label">IN ${i + 1}</span>
            <select class="cfg-select joystick-source-select" data-axis="${i}" aria-label="Input ${i + 1} source">${buildSourceOptions(cfg)}</select>
            <select class="cfg-select joystick-ch-select" data-axis="${i}" aria-label="Input ${i + 1} RC channel">
                ${buildChannelOptions(cfg.channel)}
            </select>
            <label class="joystick-inv-label">
                <input type="checkbox" class="joystick-invert" data-axis="${i}" ${cfg.inverted ? 'checked' : ''}> INV
            </label>
            <div class="joystick-dz-group">
                <span class="joystick-dz-label">DZ</span>
                <input type="range" class="joystick-deadzone" data-axis="${i}" min="0" max="90" value="${dzPct}" aria-label="Input ${i + 1} deadzone">
                <span class="joystick-dz-val" data-dz-val="${i}">${dzPct}%</span>
            </div>
            <div class="joystick-bar-container">
                <div class="joystick-bar-center"></div>
                <div class="joystick-bar-fill" data-axis-bar="${i}"></div>
            </div>
            <span class="joystick-axis-value" data-axis-val="${i}">0.00</span>
        </div>`;
    }

    container.innerHTML = html;
    bindAxisEvents();
}

/**
 * Build channel output preview grid (CH1-CH18)
 */
function renderChannelPreview() {
    const container = document.getElementById('joystick-channel-preview');
    if (!container) return;

    let html = '';
    for (let i = 1; i <= 18; i++) {
        html += `<div class="joystick-ch-cell" data-ch-cell="${i}"><span class="ch-label">CH${i}</span><span data-ch-val="${i}">--</span></div>`;
    }
    container.innerHTML = html;
}

/**
 * Bind events on dynamically created axis config rows
 */
function bindAxisEvents() {
    document.querySelectorAll('.joystick-source-select').forEach(sel => {
        sel.addEventListener('change', () => {
            const [sourceType, sourceIndex] = sel.value.split(':');
            manager.setAxisConfig(Number(sel.dataset.axis), { sourceType, sourceIndex: Number(sourceIndex) });
        });
    });
    // Channel select
    document.querySelectorAll('.joystick-ch-select').forEach(sel => {
        sel.addEventListener('change', (e) => {
            const axis = parseInt(e.target.dataset.axis);
            manager.setAxisConfig(axis, { channel: parseInt(e.target.value) });
        });
    });

    // Invert checkbox
    document.querySelectorAll('.joystick-invert').forEach(cb => {
        cb.addEventListener('change', (e) => {
            const axis = parseInt(e.target.dataset.axis);
            manager.setAxisConfig(axis, { inverted: e.target.checked });
        });
    });

    // Deadzone slider
    document.querySelectorAll('.joystick-deadzone').forEach(slider => {
        slider.addEventListener('input', (e) => {
            const axis = parseInt(e.target.dataset.axis);
            const dzPct = parseInt(e.target.value);
            manager.setAxisConfig(axis, { deadzone: dzPct / 100 });
            const valEl = document.querySelector(`[data-dz-val="${axis}"]`);
            if (valEl) valEl.textContent = dzPct + '%';
        });
    });
}

/**
 * Update live preview bars and values
 */
function updateLivePreview() {
    if (!manager) return;
    // Avoid querying hidden setup controls every gamepad tick during flight.
    const panel = document.getElementById('subtab-joystick');
    if (document.hidden || !panel?.classList.contains('active') || panel.offsetParent === null) return;

    // Update axis bars
    for (let i = 0; i < manager.rawAxisValues.length; i++) {
        const bar = document.querySelector(`[data-axis-bar="${i}"]`);
        const valEl = document.querySelector(`[data-axis-val="${i}"]`);

        if (bar) {
            const raw = manager.rawAxisValues[i] ?? 0;
            // Bar: value -1..+1 mapped to 0%..100% position
            const pct = (raw + 1) / 2 * 100;
            if (raw >= 0) {
                bar.style.left = '50%';
                bar.style.width = (pct - 50) + '%';
            } else {
                bar.style.left = pct + '%';
                bar.style.width = (50 - pct) + '%';
            }
        }
        if (valEl) {
            valEl.textContent = manager.rawAxisValues[i]?.toFixed(2) ?? '--';
        }
    }

    // Update channel preview grid
    for (let i = 0; i < 18; i++) {
        const valEl = document.querySelector(`[data-ch-val="${i + 1}"]`);
        const cellEl = document.querySelector(`[data-ch-cell="${i + 1}"]`);
        if (!valEl) continue;

        const pwm = manager.channelValues[i];
        if (pwm === 0) {
            valEl.textContent = '--';
            if (cellEl) cellEl.classList.remove('active');
        } else {
            valEl.textContent = pwm;
            if (cellEl) cellEl.classList.add('active');
        }
    }

    // Update status
    updateStatus();
    const pressed = document.getElementById('joystick-pressed-buttons');
    if (pressed) pressed.textContent = manager.pressedButtons.join(', ') || '—';
}

/**
 * Update status text
 */
function updateStatus() {
    const servoStatus = document.getElementById('joystick-servo-status');
    if (servoStatus) servoStatus.textContent = !manager.servoButtonsEnabled ? 'Disabled'
        : manager.gamepadIndex === null ? 'Waiting for gamepad'
        : !STATE.connected ? 'Waiting for connection' : manager.servoStatus;
    const statusEl = document.getElementById('joystick-status');
    if (!statusEl) return;

    if (!manager.enabled) {
        statusEl.textContent = manager.gamepadIndex === null ? 'Disabled - No gamepad (press SCAN)' : 'RC disabled - input preview / FPV available';
        statusEl.className = 'cfg-val';
    } else if (manager.suspended || manager.gamepadIndex === null) {
        // Channels released; resumes by itself when the pad is readable again
        statusEl.textContent = manager.gamepadIndex === null
            ? 'SUSPENDED - gamepad unplugged (channels released)'
            : 'SUSPENDED - no gamepad data, focus the window (channels released)';
        statusEl.className = 'cfg-val joystick-status-warning';
    } else if (STATE.rcOverrideActive) {
        statusEl.textContent = `SENDING (${manager.sendRateHz}Hz)`;
        statusEl.className = 'cfg-val joystick-status-active';
    } else if (STATE.connected) {
        statusEl.textContent = `Starting (${manager.sendRateHz}Hz)`;
        statusEl.className = 'cfg-val joystick-status-warning';
    } else {
        statusEl.textContent = `Active (${manager.sendRateHz}Hz) - Waiting connection`;
        statusEl.className = 'cfg-val';
    }
}

/**
 * Refresh gamepad list in dropdown
 */
function refreshGamepadList() {
    const select = document.getElementById('joystick-gamepad-select');
    if (!select || !manager) return;

    const gamepads = manager.detectGamepads();
    const prevValue = select.value;

    select.innerHTML = '<option value="">-- Select gamepad --</option>';
    gamepads.forEach(gp => {
        const opt = document.createElement('option');
        opt.value = gp.index;
        opt.textContent = `[${gp.index}] ${gp.id} (${gp.axes}A/${gp.buttons}B)`;
        select.appendChild(opt);
    });

    // Selection: what the manager is actually using wins, then the previous
    // dropdown value, then the remembered pad id (auto-select after a replug
    // or an app restart). Setting .value fires no 'change', so the manager is
    // told explicitly — otherwise the list shows a pad that is not selected
    // and ENABLE silently does nothing.
    const wanted = manager.gamepadIndex !== null ? String(manager.gamepadIndex)
        : (prevValue && select.querySelector(`option[value="${prevValue}"]`)) ? prevValue
        : (gamepads.find(g => g.id === manager.gamepadId) || {}).index;
    if (wanted !== undefined && wanted !== '' && select.querySelector(`option[value="${wanted}"]`)) {
        select.value = String(wanted);
        if (manager.gamepadIndex !== parseInt(wanted) && manager.selectGamepad(parseInt(wanted))) {
            renderMappings();
        }
    } else {
        select.value = '';
    }
}

/**
 * Keep checkbox, dropdown and status in step with the manager. Called by the
 * manager on every state change (suspend/resume/lost pad/replug/disable), not
 * only from the polling loop, which is stopped exactly when it matters most.
 */
function syncFromManager() {
    if (!manager) return;
    const enableCb = document.getElementById('joystick-enable');
    if (enableCb && enableCb.checked !== manager.enabled) enableCb.checked = manager.enabled;
    const servoCb = document.getElementById('joystick-servo-enable');
    if (servoCb) servoCb.checked = manager.servoButtonsEnabled;
    const select = document.getElementById('joystick-gamepad-select');
    if (select) {
        const cur = manager.gamepadIndex === null ? '' : String(manager.gamepadIndex);
        if (cur === '') select.value = '';
        else if (select.value !== cur) refreshGamepadList();
    }
    const gp = currentGamepad();
    const signature = JSON.stringify([gp?.id, gp?.axes.length, gp?.buttons.length, manager.axisMap.length]);
    if (signature !== renderedDevice) renderMappings();
    updateStatus();
    updateLivePreview();
}

let initialized = false;

/**
 * Initialize joystick UI - entry point called from TabController
 */
export function initJoystick() {
    // Destroy previous manager to avoid leaked timers, listeners, and intervals
    if (manager) {
        manager.destroy();
        manager = null;
    }

    manager = new JoystickManager();

    // Live update callback
    manager.onUpdate = updateLivePreview;
    manager.onStateChange = syncFromManager;

    // Build channel preview grid
    renderChannelPreview();
    renderMappings();

    // Only bind static DOM events once (buttons, selects that don't get re-rendered)
    if (!initialized) {
        initialized = true;

        // SCAN button
        const scanBtn = document.getElementById('joystick-scan');
        if (scanBtn) {
            scanBtn.addEventListener('click', () => {
                refreshGamepadList();
            });
        }

        // Gamepad select
        const gpSelect = document.getElementById('joystick-gamepad-select');
        if (gpSelect) {
            gpSelect.addEventListener('change', (e) => {
                const idx = e.target.value;
                if (idx === '') {
                    if (manager) manager.clearGamepad();
                    renderMappings();
                    return;
                }
                if (manager && manager.selectGamepad(parseInt(idx))) {
                    renderMappings();
                }
            });
        }

        // Enable toggle
        const enableCb = document.getElementById('joystick-enable');
        if (enableCb) {
            enableCb.addEventListener('change', async (e) => {
                if (!manager) return;
                if (e.target.checked) {
                    // Pad picked in the dropdown but not (or no longer) selected
                    // in the manager — e.g. after an unplug/replug.
                    const gpSel = document.getElementById('joystick-gamepad-select');
                    if (manager.gamepadIndex === null && gpSel && gpSel.value !== '') {
                        manager.selectGamepad(parseInt(gpSel.value));
                    }
                    if (manager.gamepadIndex === null && !manager.reselectGamepad()) {
                        e.target.checked = false;
                        alert('No gamepad selected — press SCAN and pick one');
                        updateStatus();
                        return;
                    }
                    const confirmed = await confirm(
                        'RC Override will take control of the vehicle.\n' +
                        'The vehicle may move. Continue?'
                    );
                    if (!confirmed) {
                        e.target.checked = false;
                        return;
                    }
                    const ok = await manager.enable();
                    if (!ok && !manager.enabled) e.target.checked = false;
                } else {
                    manager.disable();
                }
                updateStatus();
            });
        }

        // Send rate
        document.getElementById('joystick-servo-enable')?.addEventListener('change', e => {
            manager.setServoButtonsEnabled(e.target.checked);
        });
        const rateSelect = document.getElementById('joystick-send-rate');
        if (rateSelect) {
            rateSelect.addEventListener('change', (e) => {
                if (manager) manager.setSendRate(parseInt(e.target.value));
            });
        }
    }

    // Restore saved rate in UI
    const rateSelect = document.getElementById('joystick-send-rate');
    if (rateSelect) rateSelect.value = String(manager.sendRateHz);

    // Auto-scan on init (gamepads may already be connected)
    setTimeout(() => refreshGamepadList(), 500);
}
