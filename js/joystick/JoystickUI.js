/**
 * JoystickUI.js - UI controller for joystick configuration panel
 */

import { JoystickManager } from './JoystickManager.js';
import { STATE } from '../core/state.js';

let manager = null;

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
    if (!container || !manager || manager.gamepadIndex === null) return;

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
            <span class="joystick-axis-label">AX ${i}</span>
            <select class="cfg-select joystick-ch-select" data-axis="${i}">
                ${buildChannelOptions(cfg.channel)}
            </select>
            <label class="joystick-inv-label">
                <input type="checkbox" class="joystick-invert" data-axis="${i}" ${cfg.inverted ? 'checked' : ''}> INV
            </label>
            <div class="joystick-dz-group">
                <span class="joystick-dz-label">DZ</span>
                <input type="range" class="joystick-deadzone" data-axis="${i}" min="0" max="50" value="${dzPct}">
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

    // Update axis bars
    for (let i = 0; i < manager.rawAxisValues.length; i++) {
        const bar = document.querySelector(`[data-axis-bar="${i}"]`);
        const valEl = document.querySelector(`[data-axis-val="${i}"]`);

        if (bar) {
            const raw = manager.rawAxisValues[i];
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
            valEl.textContent = manager.rawAxisValues[i].toFixed(2);
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
}

/**
 * Update status text
 */
function updateStatus() {
    const statusEl = document.getElementById('joystick-status');
    if (!statusEl) return;

    if (!manager.enabled) {
        statusEl.textContent = 'Disabled';
        statusEl.className = 'cfg-val';
    } else if (manager.gamepadIndex === null) {
        statusEl.textContent = 'No Gamepad';
        statusEl.className = 'cfg-val joystick-status-warning';
    } else if (STATE.rcOverrideActive) {
        statusEl.textContent = `SENDING (${manager.sendRateHz}Hz)`;
        statusEl.className = 'cfg-val joystick-status-active';
    } else if (STATE.connected) {
        statusEl.textContent = `Active (${manager.sendRateHz}Hz) - Not connected`;
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

    select.innerHTML = '<option value="">-- No gamepad detected --</option>';
    gamepads.forEach(gp => {
        const opt = document.createElement('option');
        opt.value = gp.index;
        opt.textContent = `[${gp.index}] ${gp.id} (${gp.axes}A/${gp.buttons}B)`;
        select.appendChild(opt);
    });

    // Restore selection
    if (prevValue && select.querySelector(`option[value="${prevValue}"]`)) {
        select.value = prevValue;
    }
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

    // Build channel preview grid
    renderChannelPreview();

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
                    if (manager) manager.disable();
                    document.getElementById('joystick-axes-container').innerHTML =
                        '<div class="joystick-placeholder">Connect a gamepad and click SCAN</div>';
                    return;
                }
                if (manager && manager.selectGamepad(parseInt(idx))) {
                    renderAxisRows();
                }
            });
        }

        // Enable toggle
        const enableCb = document.getElementById('joystick-enable');
        if (enableCb) {
            enableCb.addEventListener('change', async (e) => {
                if (!manager) return;
                if (e.target.checked) {
                    if (manager.gamepadIndex === null) {
                        e.target.checked = false;
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
                    manager.enable();
                } else {
                    manager.disable();
                }
                updateStatus();
            });
        }

        // Send rate
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
