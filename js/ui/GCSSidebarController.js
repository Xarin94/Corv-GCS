/**
 * GCSSidebarController.js - GCS Sidebar Controller
 * Handles sidebar toggle, section collapse, and action button bindings
 * Pattern based on NDController.js
 */

import { STATE } from '../core/state.js';
import {
    setHomeCurrent, rebootAutopilot, setParameter
} from '../mavlink/CommandSender.js';
import { getVehicleTypeName } from '../mavlink/MAVLinkStateMapper.js';

let sidebarEl = null;
let toggleBtn = null;
let isCollapsed = true;

/**
 * Initialize GCS sidebar
 */
export function initGCSSidebar() {
    sidebarEl = document.getElementById('gcs-sidebar');
    toggleBtn = document.getElementById('gcs-toggle-sidebar');

    if (!sidebarEl || !toggleBtn) return;

    // Start collapsed
    sidebarEl.classList.add('collapsed');
    toggleBtn.classList.add('collapsed');
    toggleBtn.innerHTML = '&#9654;';

    // Toggle sidebar
    toggleBtn.addEventListener('click', () => {
        isCollapsed = !isCollapsed;
        sidebarEl.classList.toggle('collapsed', isCollapsed);
        toggleBtn.classList.toggle('collapsed', isCollapsed);
        toggleBtn.innerHTML = isCollapsed ? '&#9654;' : '&#9664;';
    });

    // Section collapse handlers
    document.querySelectorAll('.gcs-section-header').forEach(header => {
        header.addEventListener('click', () => {
            const section = header.closest('.gcs-sidebar-section');
            section.classList.toggle('collapsed');
        });
    });

    // ACTIONS section (ARM removed - now only in command bar; CAL buttons moved to Initial Setup tab)
    bindAction('gcs-set-home', async () => {
        if (await confirm('Set HOME to current position?')) await setHomeCurrent();
    });

    bindAction('gcs-reboot', async () => {
        if (await confirm('Reboot autopilot? Vehicle must be disarmed.')) await rebootAutopilot();
    });

    // RTL OPTIONS section
    // Read: store raw values from vehicle as baseline, display in user units
    bindAction('rtl-opts-read', () => {
        document.querySelectorAll('.rtl-param-input[data-param]').forEach(input => {
            const param = STATE.parameters.get(input.dataset.param);
            if (!param) return;
            const scale = parseFloat(input.dataset.scale) || 1;
            const displayVal = param.value / scale;
            input.value = scale === 1 ? displayVal : parseFloat(displayVal.toFixed(4));
            input.dataset.baseline = input.value; // track baseline for dirty check
        });
    });

    // Write: only send parameters the user actually changed
    bindAction('rtl-opts-write', async () => {
        const inputs = document.querySelectorAll('.rtl-param-input[data-param]');
        let count = 0;
        for (const input of inputs) {
            const val = parseFloat(input.value);
            if (isNaN(val)) continue;
            if (input.dataset.baseline !== undefined && String(val) === input.dataset.baseline) continue;
            const scale = parseFloat(input.dataset.scale) || 1;
            await setParameter(input.dataset.param, val * scale);
            input.dataset.baseline = String(val);
            count++;
        }
        if (count === 0) console.log('[RTL opts] No parameters changed');
    });

    // MISSION section
    bindAction('gcs-mission-read', () => {
        if (!window.mavlink) return;
        window.mavlink.sendMessage({
            type: 'MISSION_REQUEST_LIST',
            targetSystem: STATE.systemId,
            targetComponent: STATE.componentId,
            missionType: 0
        });
    });

}

/**
 * Helper to bind a click handler to a button by ID
 */
function bindAction(id, handler) {
    const el = document.getElementById(id);
    if (el) {
        el.addEventListener('click', async () => {
            try {
                await handler();
            } catch (e) {
                alert('Error: ' + e.message);
            }
        });
    }
}

/**
 * Update sidebar display - call from animation loop
 */
export function updateGCSSidebar() {
    // Update mission sequence display
    const seqEl = document.getElementById('gcs-mission-seq');
    if (seqEl) {
        seqEl.textContent = `${STATE.missionCurrentSeq}/${STATE.missionCount}`;
    }

    // Disable reboot when armed
    const rebootBtn = document.getElementById('gcs-reboot');
    if (rebootBtn) {
        rebootBtn.disabled = STATE.armed;
        rebootBtn.style.opacity = STATE.armed ? '0.3' : '';
        rebootBtn.style.pointerEvents = STATE.armed ? 'none' : '';
    }

    // Vehicle info section
    const vTypeEl = document.getElementById('gcs-vehicle-type');
    if (vTypeEl) vTypeEl.textContent = STATE.vehicleType > 0 ? getVehicleTypeName(STATE.vehicleType) : '--';
    const sysIdEl = document.getElementById('gcs-sys-id');
    if (sysIdEl) sysIdEl.textContent = STATE.systemId || '--';
    const fwEl = document.getElementById('gcs-fw-version');
    if (fwEl) fwEl.textContent = STATE.firmwareVersion || '--';
}
