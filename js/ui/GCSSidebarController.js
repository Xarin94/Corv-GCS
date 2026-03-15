/**
 * GCSSidebarController.js - GCS Sidebar Controller
 * Handles sidebar toggle, section collapse, and action button bindings
 * Pattern based on NDController.js
 */

import { STATE } from '../core/state.js';
import { RAD } from '../core/constants.js';
import { calculateDistance } from '../core/utils.js';
import {
    setHomeCurrent, rebootAutopilot, setParameter
} from '../mavlink/CommandSender.js';
import { getVehicleTypeName } from '../mavlink/MAVLinkStateMapper.js';
import { setTargetMarker, clearTargetMarker } from '../maps/MapEngine.js';
import { setTargetMarker3D, clearTargetMarker3D } from '../engine/Scene3D.js';

let sidebarEl = null;
let toggleBtn = null;
let isCollapsed = true;
let targetCoords = null;

export function getTargetCoords() { return targetCoords; }

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
    });

    // TARGET section
    bindAction('target-set', () => {
        const lat = parseFloat(document.getElementById('target-lat')?.value);
        const lon = parseFloat(document.getElementById('target-lon')?.value);
        if (isNaN(lat) || isNaN(lon) || lat < -90 || lat > 90 || lon < -180 || lon > 180) return;
        targetCoords = { lat, lon };
        setTargetMarker(lat, lon);
        setTargetMarker3D(lat, lon);
    });

    bindAction('target-clear', () => {
        targetCoords = null;
        clearTargetMarker();
        clearTargetMarker3D();
        const cmdTarget = document.getElementById('cmd-target');
        if (cmdTarget) cmdTarget.style.display = 'none';
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

    // Target indicator update (in command bar)
    const cmdTarget = document.getElementById('cmd-target');
    if (targetCoords && cmdTarget) {
        const dist = calculateDistance(STATE.lat, STATE.lon, targetCoords.lat, targetCoords.lon);
        const dLat = targetCoords.lat - STATE.lat;
        const dLon = targetCoords.lon - STATE.lon;
        const bearing = Math.atan2(dLon * Math.cos(STATE.lat * Math.PI / 180), dLat) * RAD;
        const yawDeg = STATE.yaw * RAD;
        const relative = bearing - yawDeg;
        const arrow = document.getElementById('target-arrow');
        if (arrow) arrow.style.transform = `rotate(${relative}deg)`;
        const distEl = document.getElementById('target-dist');
        if (distEl) distEl.textContent = dist >= 1000 ? (dist / 1000).toFixed(1) + ' km' : Math.round(dist) + ' m';
        cmdTarget.style.display = 'flex';
    }

    // Vehicle info section
    const vTypeEl = document.getElementById('gcs-vehicle-type');
    if (vTypeEl) vTypeEl.textContent = STATE.vehicleType > 0 ? getVehicleTypeName(STATE.vehicleType) : '--';
    const sysIdEl = document.getElementById('gcs-sys-id');
    if (sysIdEl) sysIdEl.textContent = STATE.systemId || '--';
    const fwEl = document.getElementById('gcs-fw-version');
    if (fwEl) fwEl.textContent = STATE.firmwareVersion || '--';
}
