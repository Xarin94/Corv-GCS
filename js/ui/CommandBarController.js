/**
 * CommandBarController.js - GCS Bottom Command Bar Controller
 * Handles ARM/DISARM, flight mode, takeoff/land/RTL, speed, and status display
 */

import { STATE } from '../core/state.js';
import { getGPSFixName, getAvailableFlightModes, getVehicleTypeName } from '../mavlink/MAVLinkStateMapper.js';
import {
    armVehicle, disarmVehicle, setFlightMode,
    takeoff, returnToLaunch, setMissionSpeed,
    setGuidedTarget, changeAltitude
} from '../mavlink/CommandSender.js';
import { isHeartbeatAlive } from '../mavlink/ConnectionManager.js';
import { onMessage } from '../mavlink/MAVLinkManager.js';

// Simple input dialog replacement for prompt() (not supported in sandboxed Electron)
function showInputDialog(message, defaultValue = '') {
    return new Promise((resolve) => {
        let closed = false;

        const overlay = document.createElement('div');
        const cs = getComputedStyle(document.documentElement);
        const bgDialog = cs.getPropertyValue('--bg-dialog').trim() || '#1a1f2e';
        const bgDialogInput = cs.getPropertyValue('--bg-dialog-input').trim() || '#0d1117';
        const textMain = cs.getPropertyValue('--text-main').trim() || '#fff';
        const textDim = cs.getPropertyValue('--text-dim').trim() || '#ccc';
        const bgSlider = cs.getPropertyValue('--bg-slider-track').trim() || '#333';
        const overlayBg = cs.getPropertyValue('--bg-overlay-light').trim() || 'rgba(0,0,0,0.6)';

        overlay.style.cssText = `position:fixed;inset:0;z-index:99999;background:${overlayBg};display:flex;align-items:center;justify-content:center;`;

        const box = document.createElement('div');
        box.style.cssText = `background:${bgDialog};border:1px solid #00d2ff;border-radius:6px;padding:16px 20px;min-width:280px;`;

        const label = document.createElement('div');
        label.style.cssText = `color:${textDim};font-size:12px;margin-bottom:10px;`;
        label.textContent = message;

        const input = document.createElement('input');
        input.type = 'text';
        input.inputMode = 'numeric';
        input.style.cssText = `width:100%;box-sizing:border-box;background:${bgDialogInput};color:${textMain};border:1px solid ${bgSlider};border-radius:3px;padding:6px 8px;font-size:13px;outline:none;text-align:center;user-select:text;-webkit-user-select:text;`;
        input.addEventListener('input', () => { input.value = input.value.replace(/[^0-9.\-]/g, ''); });

        const btnRow = document.createElement('div');
        btnRow.style.cssText = 'display:flex;gap:8px;margin-top:12px;justify-content:flex-end;';

        const cancelBtn = document.createElement('button');
        cancelBtn.textContent = 'CANCEL';
        cancelBtn.style.cssText = `background:${bgSlider};color:${textDim};border:none;padding:5px 14px;border-radius:3px;cursor:pointer;font-size:11px;`;

        const okBtn = document.createElement('button');
        okBtn.textContent = 'OK';
        okBtn.style.cssText = 'background:#00d2ff;color:#000;border:none;padding:5px 14px;border-radius:3px;cursor:pointer;font-weight:600;font-size:11px;';

        btnRow.append(cancelBtn, okBtn);
        box.append(label, input, btnRow);
        overlay.appendChild(box);
        document.body.appendChild(overlay);

        // Set value and focus after DOM insertion
        requestAnimationFrame(() => {
            input.value = defaultValue;
            input.focus();
            input.setSelectionRange(0, input.value.length);
        });

        const close = (val) => {
            if (closed) return;
            closed = true;
            overlay.remove();
            resolve(val);
        };
        okBtn.addEventListener('click', (e) => { e.stopPropagation(); close(input.value); });
        cancelBtn.addEventListener('click', (e) => { e.stopPropagation(); close(null); });
        input.addEventListener('keydown', (e) => {
            if (e.key === 'Enter') close(input.value);
            if (e.key === 'Escape') close(null);
        });
        overlay.addEventListener('mousedown', (e) => { if (e.target === overlay) close(null); });
    });
}

// DOM element cache
let els = {};
let prearmOk = false;
let lastVehicleType = -1; // track vehicle type changes for dropdown refresh
let flightTimeStart = 0; // timestamp when armed (ms)
let flightTimeAccum = 0; // accumulated time before current arm session
let lastDisplayedMode = ''; // for mode change banner
let modeBannerTimer = null;

// Flight mode category for color coding
const MODE_CATEGORIES = {
    // Manual modes (yellow)
    STABILIZE: 'manual', ACRO: 'manual', MANUAL: 'manual', TRAINING: 'manual',
    // Assisted modes (cyan)
    ALT_HOLD: 'assisted', POSHOLD: 'assisted', LOITER: 'assisted', FBWA: 'assisted', FBWB: 'assisted',
    CIRCLE: 'assisted', GUIDED: 'assisted', SPORT: 'assisted',
    FLOWHOLD: 'assisted', FOLLOW: 'assisted', ZIGZAG: 'assisted',
    // Auto modes (green)
    AUTO: 'auto', AUTOTUNE: 'auto', SMARTRTL: 'auto', BRAKE: 'auto',
    // Emergency/return modes (orange)
    RTL: 'emergency', LAND: 'emergency', QRTL: 'emergency', QLAND: 'emergency',
};

/**
 * Initialize command bar - bind events and cache elements
 */
export function initCommandBar() {
    els = {
        heartbeat: document.getElementById('cmd-heartbeat'),
        connStatus: document.getElementById('cmd-conn-status'),
        batVolt: document.getElementById('cmd-bat-volt'),
        batPct: document.getElementById('cmd-bat-pct'),
        batBar: document.getElementById('cmd-bat-bar'),
        gpsFix: document.getElementById('cmd-gps-fix'),
        gpsSat: document.getElementById('cmd-gps-sat'),
        gpsHdop: document.getElementById('cmd-gps-hdop'),
        flightMode: document.getElementById('cmd-flight-mode'),
        armBtn: document.getElementById('cmd-arm'),
        takeoffBtn: document.getElementById('cmd-takeoff'),
        rtlBtn: document.getElementById('cmd-rtl'),
        landBtn: document.getElementById('cmd-land'),
        speedInput: document.getElementById('cmd-speed-input'),
        rssi: document.getElementById('cmd-rssi'),
        remRssi: document.getElementById('cmd-remrssi'),
        autoBtn: document.getElementById('cmd-auto'),
        fbwaBtn: document.getElementById('cmd-fbwa'),
        posholdBtn: document.getElementById('cmd-poshold'),
        flightTime: document.getElementById('cmd-flight-time')
    };

    // ARM/DISARM button
    els.armBtn.addEventListener('click', async () => {
        if (STATE.connectionType === 'none' || STATE.connectionType === 'corv-binary') return;

        if (STATE.armed) {
            if (await confirm('DISARM the vehicle?')) {
                try { await disarmVehicle(); } catch (e) { alert('Disarm failed: ' + e.message); }
            }
        } else {
            if (await confirm('ARM the vehicle? Ensure area is clear.')) {
                try { await armVehicle(); } catch (e) { alert('Arm failed: ' + e.message); }
            }
        }
    });

    // Flight mode change
    els.flightMode.addEventListener('change', async (e) => {
        if (STATE.connectionType === 'none' || STATE.connectionType === 'corv-binary') return;
        try {
            await setFlightMode(e.target.value);
        } catch (err) {
            alert('Mode change failed: ' + err.message);
        }
    });

    // Takeoff (use custom input since prompt() is not supported in Electron sandboxed renderers)
    els.takeoffBtn.addEventListener('click', async () => {
        if (STATE.connectionType === 'none' || STATE.connectionType === 'corv-binary') return;
        const alt = await showInputDialog('Takeoff altitude AGL (m):', '10');
        if (alt === null) return;
        try {
            await takeoff(parseFloat(alt));
        } catch (err) {
            alert('Takeoff failed: ' + err.message);
        }
    });

    // RTL
    els.rtlBtn.addEventListener('click', async () => {
        if (STATE.connectionType === 'none' || STATE.connectionType === 'corv-binary') return;
        try { await returnToLaunch(); } catch (err) { alert('RTL failed: ' + err.message); }
    });

    // AUTO mission
    if (els.autoBtn) {
        els.autoBtn.addEventListener('click', async () => {
            if (STATE.connectionType === 'none' || STATE.connectionType === 'corv-binary') return;
            try { await setFlightMode('AUTO'); } catch (err) { alert('AUTO mode failed: ' + err.message); }
        });
    }

    // FBWA (plane only)
    if (els.fbwaBtn) {
        els.fbwaBtn.addEventListener('click', async () => {
            if (STATE.connectionType === 'none' || STATE.connectionType === 'corv-binary') return;
            try { await setFlightMode('FBWA'); } catch (err) { alert('FBWA mode failed: ' + err.message); }
        });
    }

    // POSHOLD (copter only)
    if (els.posholdBtn) {
        els.posholdBtn.addEventListener('click', async () => {
            if (STATE.connectionType === 'none' || STATE.connectionType === 'corv-binary') return;
            try { await setFlightMode('POSHOLD'); } catch (err) { alert('POSHOLD mode failed: ' + err.message); }
        });
    }

    // LAND
    els.landBtn.addEventListener('click', async () => {
        if (STATE.connectionType === 'none' || STATE.connectionType === 'corv-binary') return;
        try {
            await setFlightMode('LAND');
        } catch (err) {
            alert('Land failed: ' + err.message);
        }
    });

    // Speed input — send on Enter key
    const sendSpeed = async () => {
        const speed = parseFloat(els.speedInput.value);
        if (isNaN(speed) || speed <= 0) {
            console.warn('[speed] Invalid speed value:', els.speedInput.value);
            return;
        }
        if (STATE.connectionType === 'none' || STATE.connectionType === 'corv-binary') {
            console.warn('[speed] Not connected, connectionType:', STATE.connectionType);
            return;
        }
        try {
            console.log(`[speed] Sending DO_CHANGE_SPEED: ${speed} m/s`);
            await setMissionSpeed(speed);
            // Show brief "SET" feedback
            const lbl = els.speedInput.nextElementSibling;
            if (lbl) {
                const orig = lbl.textContent;
                lbl.textContent = 'SET';
                lbl.style.color = '#00ff88';
                setTimeout(() => { lbl.textContent = orig; lbl.style.color = ''; }, 1500);
            }
        } catch (err) {
            console.error('[speed] Speed change failed:', err);
        }
    };
    els.speedInput.addEventListener('keydown', (e) => {
        if (e.key === 'Enter') { e.preventDefault(); sendSpeed(); }
    });
    const speedSetBtn = document.getElementById('cmd-speed-set');
    if (speedSetBtn) speedSetBtn.addEventListener('click', () => sendSpeed());

    // Listen for connection state changes
    window.addEventListener('mavlinkConnectionState', (e) => {
        updateConnectionStatus(e.detail.state);
    });

    // Cache pre-arm and message elements
    els.prearm = document.getElementById('cmd-prearm');
    els.hudMsg = document.getElementById('cmd-hud-msg');
    els.msgLog = document.getElementById('cmd-msg-log');

    // Listen for STATUSTEXT messages (msg id 253)
    onMessage(253, (data) => {
        const text = data.text || '';

        // Update latest message display
        if (els.hudMsg) {
            els.hudMsg.textContent = text;
            els.hudMsg.title = text;
        }

        // Severity: 0-3 = error/critical, 4 = warning, 5-7 = info/notice
        const sev = data.severity ?? 6;
        if (els.hudMsg) {
            els.hudMsg.className = 'cmd-val cmd-msg-text' +
                (sev <= 3 ? ' severity-error' : sev <= 4 ? ' severity-warn' : '');
        }

        // Append to message log
        if (els.msgLog) {
            const sevLabel = sev <= 3 ? '[ERR] ' : sev <= 4 ? '[WRN] ' : '';
            els.msgLog.value += sevLabel + text + '\n';
            els.msgLog.scrollTop = els.msgLog.scrollHeight;
        }

        // Track pre-arm status from STATUSTEXT
        const lower = text.toLowerCase();
        if (lower.includes('prearm') && lower.includes('fail')) {
            prearmOk = false;
        } else if (lower.includes('prearm') && (lower.includes('good') || lower.includes('pass'))) {
            prearmOk = true;
        }
    });
}

/**
 * Populate flight mode dropdown based on vehicle type
 */
function populateFlightModes() {
    if (!els.flightMode) return;
    const modes = getAvailableFlightModes();
    const currentValue = els.flightMode.value;
    els.flightMode.innerHTML = '';
    modes.sort((a, b) => a.name.localeCompare(b.name));
    for (const mode of modes) {
        const opt = document.createElement('option');
        opt.value = mode.name;
        opt.textContent = mode.name;
        els.flightMode.appendChild(opt);
    }
    // Restore selection if still valid
    if (currentValue && els.flightMode.querySelector(`option[value="${currentValue}"]`)) {
        els.flightMode.value = currentValue;
    }
}

/**
 * Update command bar display - call from animation loop
 */
export function updateCommandBar() {
    if (!els.heartbeat) return;

    // Heartbeat
    const alive = isHeartbeatAlive();
    els.heartbeat.classList.toggle('alive', alive);

    // Connection status
    if (STATE.connectionType === 'none') {
        els.connStatus.textContent = 'DISCONNECTED';
        els.connStatus.className = 'cmd-status';
    } else if (STATE.connectionType === 'corv-binary') {
        els.connStatus.textContent = 'CORV BINARY';
        els.connStatus.className = 'cmd-status connected';
    } else if (alive) {
        els.connStatus.textContent = 'ACTIVE';
        els.connStatus.className = 'cmd-status active';
    } else if (STATE.connected) {
        els.connStatus.textContent = 'CONNECTED';
        els.connStatus.className = 'cmd-status connected';
    }

    // Battery
    const volt = STATE.batteryVoltage;
    els.batVolt.textContent = volt > 0 ? volt.toFixed(1) + 'V' : '0.0V';
    const pct = STATE.batteryRemaining;
    els.batPct.textContent = pct >= 0 ? pct + '%' : '--%';
    if (pct >= 0) {
        els.batBar.style.width = pct + '%';
        els.batBar.className = 'cmd-bat-bar-fill' + (pct < 20 ? ' critical' : pct < 40 ? ' low' : '');
    }

    // GPS with RTK color coding
    els.gpsFix.textContent = getGPSFixName(STATE.gpsFix);
    els.gpsSat.textContent = STATE.gpsNumSat + ' sat';
    if (els.gpsHdop) els.gpsHdop.textContent = STATE.gpsHdop < 99 ? 'HDOP ' + STATE.gpsHdop.toFixed(1) : '';
    // Color: RTK Fixed=green, RTK Float=yellow, 3D Fix=cyan, DGPS=cyan, <=2D=red
    if (STATE.gpsFix === 6) {
        els.gpsFix.style.color = '#00ff7f'; // RTK Fixed
    } else if (STATE.gpsFix === 5) {
        els.gpsFix.style.color = '#ffcc00'; // RTK Float
    } else if (STATE.gpsFix >= 3) {
        els.gpsFix.style.color = '#00d2ff'; // 3D Fix / DGPS
    } else {
        els.gpsFix.style.color = '#ff3333'; // No fix / 2D
    }

    // Refresh flight mode dropdown when vehicle type changes
    if (STATE.vehicleType !== lastVehicleType && STATE.vehicleType > 0) {
        lastVehicleType = STATE.vehicleType;
        populateFlightModes();
        // Show FBWA for planes, POSHOLD for copters
        const vTypeName = getVehicleTypeName(STATE.vehicleType);
        if (els.fbwaBtn) els.fbwaBtn.style.display = vTypeName === 'Plane' ? '' : 'none';
        if (els.posholdBtn) els.posholdBtn.style.display = vTypeName === 'Copter' ? '' : 'none';
    }

    // Flight mode + color coding
    if (els.flightMode.value !== STATE.flightMode && STATE.flightMode !== 'UNKNOWN') {
        const option = els.flightMode.querySelector(`option[value="${STATE.flightMode}"]`);
        if (option) {
            els.flightMode.value = STATE.flightMode;
        }
    }
    // Apply mode category color class
    const modeCat = MODE_CATEGORIES[STATE.flightMode] || 'assisted';
    els.flightMode.className = 'cmd-select cmd-mode-select mode-' + modeCat;

    // Mode change notification banner
    if (STATE.flightMode && STATE.flightMode !== 'UNKNOWN' && STATE.flightMode !== lastDisplayedMode) {
        if (lastDisplayedMode !== '') { // skip initial set
            showModeBanner(STATE.flightMode, modeCat);
        }
        lastDisplayedMode = STATE.flightMode;
    }

    // ARM state
    const wasArmed = els.armBtn.classList.contains('armed');
    if (STATE.armed) {
        els.armBtn.textContent = 'ARMED';
        els.armBtn.classList.add('armed');
        prearmOk = true;
        // Start flight timer on arm transition
        if (!wasArmed) flightTimeStart = Date.now();
    } else {
        els.armBtn.textContent = 'DISARMED';
        els.armBtn.classList.remove('armed');
        // Accumulate time on disarm transition
        if (wasArmed && flightTimeStart > 0) {
            flightTimeAccum += Date.now() - flightTimeStart;
            flightTimeStart = 0;
        }
    }

    // Flight time display
    if (els.flightTime) {
        let totalMs = flightTimeAccum;
        if (STATE.armed && flightTimeStart > 0) totalMs += Date.now() - flightTimeStart;
        const totalSec = Math.floor(totalMs / 1000);
        const mm = String(Math.floor(totalSec / 60)).padStart(2, '0');
        const ss = String(totalSec % 60).padStart(2, '0');
        els.flightTime.textContent = mm + ':' + ss;
        els.flightTime.style.color = STATE.armed ? 'var(--accent-cyan)' : '';
    }

    // Pre-arm indicator
    if (els.prearm) {
        if (STATE.armed || prearmOk) {
            els.prearm.className = 'cmd-prearm-status ok';
            els.prearm.title = 'Pre-arm: OK';
        } else if (alive) {
            els.prearm.className = 'cmd-prearm-status fail';
            els.prearm.title = 'Pre-arm: Check failed';
        } else {
            els.prearm.className = 'cmd-prearm-status';
            els.prearm.title = 'Pre-arm: Unknown';
        }
    }

    // RSSI — prefer RADIO_STATUS values; fall back to SYS_STATUS linkQuality
    if (STATE.rssi !== null) {
        els.rssi.textContent = STATE.rssi;
        if (STATE.remRssi !== null && els.remRssi) {
            els.remRssi.textContent = 'R:' + STATE.remRssi;
            els.remRssi.style.display = '';
        } else if (els.remRssi) {
            els.remRssi.style.display = 'none';
        }
    } else {
        const lq = STATE.linkQuality;
        els.rssi.textContent = lq > 0 ? Math.round(lq) + '%' : '--';
        if (els.remRssi) els.remRssi.style.display = 'none';
    }
}

function updateConnectionStatus(state) {
    if (!els.connStatus) return;
    els.connStatus.textContent = state;
    els.connStatus.className = 'cmd-status' +
        (state === 'CONNECTED' ? ' connected' : state === 'ACTIVE' ? ' active' : '');
}

/**
 * Show mode change notification banner
 */
function showModeBanner(modeName, category) {
    const banner = document.getElementById('mode-change-banner');
    const text = document.getElementById('mode-banner-text');
    if (!banner || !text) return;

    // Clear previous timer
    if (modeBannerTimer) clearTimeout(modeBannerTimer);

    text.textContent = modeName;
    banner.className = 'mode-banner mode-' + category;

    // Re-trigger animation by forcing reflow
    banner.style.animation = 'none';
    banner.offsetHeight; // force reflow
    banner.style.animation = '';

    // Hide after animation completes
    modeBannerTimer = setTimeout(() => {
        banner.classList.add('hidden');
    }, 2500);
}

