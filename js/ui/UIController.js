/**
 * UIController.js - UI State and Updates
 * Handles UI elements, telemetry display, configuration panels
 */

import { STATE } from '../core/state.js';
import { RAD } from '../core/constants.js';
import { getNearestTraffic } from '../adsb/ADSBManager.js';

// Cached DOM references for high-frequency updates (avoids ~29 queries/frame)
let domCache = null;

// G-Load low-pass filter state
let filteredG = 1.0;
const G_FILTER_ALPHA = 0.8;

/**
 * Initialize DOM cache for performance
 */
function ensureDomCache() {
    if (domCache) return domCache;

    domCache = {
        // Main display elements
        dispAs: document.getElementById('disp-as'),
        dispGs: document.getElementById('disp-gs'),
        dispG: document.getElementById('disp-g'),
        dispAlt: document.getElementById('disp-alt'),
        dispVs: document.getElementById('disp-vs'),
        dispLat: document.getElementById('disp-lat'),
        dispLon: document.getElementById('disp-lon'),
        dispAb: document.getElementById('disp-ab'),
        dispAgl: document.getElementById('disp-agl'),
        dispLidar: document.getElementById('disp-lidar'),
        dispRalt: document.getElementById('disp-ralt'),

        // Telemetry panel elements
        telemetryPanel: document.getElementById('telemetry-panel'),
        tRoll: document.getElementById('t-roll'),
        tPitch: document.getElementById('t-pitch'),
        tYaw: document.getElementById('t-yaw'),
        tLat: document.getElementById('t-lat'),
        tLon: document.getElementById('t-lon'),
        tAlt: document.getElementById('t-alt'),
        tAs: document.getElementById('t-as'),
        tGs: document.getElementById('t-gs'),
        tVs: document.getElementById('t-vs'),
        tAx: document.getElementById('t-ax'),
        tAy: document.getElementById('t-ay'),
        tAz: document.getElementById('t-az'),

        // Status elements
        statusMsg: document.getElementById('status-msg'),
        fpsCounter: document.getElementById('fps-counter'),

        // Config/UI elements
        configPanel: document.getElementById('config-panel'),
        btnCfg: document.getElementById('btn-cfg'),
        btnTelem: document.getElementById('btn-telem'),

        // Traffic table elements
        trafficBar: document.getElementById('traffic-bar'),
        tfcRows: [0, 1, 2, 3].map(i => ({
            cs: document.getElementById(`tfc-cs-${i}`),
            alt: document.getElementById(`tfc-alt-${i}`),
            dist: document.getElementById(`tfc-dist-${i}`)
        }))
    };

    return domCache;
}

/**
 * Update all UI displays
 */
export function updateUI() {
    const dom = ensureDomCache();
    const alt = STATE.rawAlt + STATE.offsetAlt;

    // Main display updates using cached references
    if (dom.dispAs) dom.dispAs.textContent = STATE.as.toFixed(1);
    if (dom.dispGs) dom.dispGs.textContent = STATE.gs.toFixed(1);
    if (dom.dispAlt) dom.dispAlt.textContent = Math.round(alt);
    if (dom.dispRalt) dom.dispRalt.textContent = STATE.rangefinderDist != null ? STATE.rangefinderDist.toFixed(1) : '---';
    if (dom.dispVs) dom.dispVs.textContent = STATE.vs.toFixed(1);
    if (dom.dispLat) dom.dispLat.textContent = STATE.lat.toFixed(5);
    if (dom.dispLon) dom.dispLon.textContent = STATE.lon.toFixed(5);
    if (dom.dispAb) dom.dispAb.textContent = `${(STATE.aoa * RAD).toFixed(1)}° / ${(STATE.ssa * RAD).toFixed(1)}°`;

    // G-Load display with low-pass filter and color coding
    if (dom.dispG) {
        // Normal load factor: body-frame Z axis (vibration-immune, standard aviation G-meter)
        const rawG = STATE.az / 9.81;
        filteredG = G_FILTER_ALPHA * filteredG + (1 - G_FILTER_ALPHA) * rawG;
        dom.dispG.textContent = filteredG.toFixed(1);

        // Color coding: green (-1 to +4), yellow (-2 to +6), red (outside)
        let gColor;
        if (filteredG >= -1 && filteredG <= 4) {
            gColor = '#00ff7f'; // green
        } else if (filteredG >= -2 && filteredG <= 6) {
            gColor = '#ffcc00'; // yellow
        } else {
            gColor = '#ff3333'; // red
        }
        dom.dispG.style.color = gColor;
    }

    // LiDAR rangefinder display with color coding
    if (dom.dispLidar) {
        const dist = STATE.rangefinderDist;
        if (dist !== null && dist > 0) {
            dom.dispLidar.textContent = dist.toFixed(2);
            if (dist < 1) {
                dom.dispLidar.style.color = '#ff3333'; // red < 1m
            } else if (dist <= 10) {
                dom.dispLidar.style.color = '#ff8800'; // orange 1-10m
            } else {
                dom.dispLidar.style.color = ''; // white (default)
            }
        } else {
            dom.dispLidar.textContent = '---';
            dom.dispLidar.style.color = '';
        }
    }

    // ADS-B traffic table (4 nearest) - always visible
    if (dom.trafficBar) {
        const nearest = getNearestTraffic(4);
        for (let i = 0; i < 4; i++) {
            const row = dom.tfcRows[i];
            if (!row.cs) continue;
            if (i < nearest.length) {
                const t = nearest[i];
                row.cs.textContent = t.callsign || t.icao24;
                row.alt.textContent = Math.round(t.alt);
                row.dist.textContent = (t.dist / 1000).toFixed(1);
            } else {
                row.cs.textContent = '---';
                row.alt.textContent = '---';
                row.dist.textContent = '---';
            }
        }
    }

    // Telemetry Panel Update (only if visible)
    if (dom.telemetryPanel && dom.telemetryPanel.classList.contains('visible')) {
        updateTelemetryPanel(alt, dom);
    }
}

/**
 * Update telemetry panel displays
 */
function updateTelemetryPanel(alt, dom) {
    if (dom.tRoll) dom.tRoll.textContent = (STATE.roll * RAD).toFixed(2) + "°";
    if (dom.tPitch) dom.tPitch.textContent = (STATE.pitch * RAD).toFixed(2) + "°";
    if (dom.tYaw) dom.tYaw.textContent = ((STATE.yaw * RAD + 360) % 360).toFixed(0).padStart(3, '0') + "°";
    if (dom.tLat) dom.tLat.textContent = STATE.lat.toFixed(6);
    if (dom.tLon) dom.tLon.textContent = STATE.lon.toFixed(6);
    if (dom.tAlt) dom.tAlt.textContent = alt.toFixed(1) + " m";
    if (dom.tAs) dom.tAs.textContent = STATE.as.toFixed(1) + " m/s";
    if (dom.tGs) dom.tGs.textContent = STATE.gs.toFixed(1) + " m/s";
    if (dom.tVs) dom.tVs.textContent = STATE.vs.toFixed(1) + " m/s";
    if (dom.tAx) dom.tAx.textContent = STATE.ax.toFixed(2) + " g";
    if (dom.tAy) dom.tAy.textContent = STATE.ay.toFixed(2) + " g";
    if (dom.tAz) dom.tAz.textContent = STATE.az.toFixed(2) + " g";
}

/**
 * Toggle configuration panel visibility
 */
export function toggleConfig() {
    // Config panel moved to Setup tab (simulation section) - navigate there
    const setupTab = document.querySelector('.gcs-tab[data-tab="setup"]');
    if (setupTab) {
        setupTab.click();
        // Activate simulation section in vertical nav
        const simBtn = document.querySelector('.setup-nav-btn[data-section="simulation"]');
        if (simBtn) simBtn.click();
    }
}

let configAutoCloseInitialized = false;

/**
 * Close the System Config panel when clicking outside of it.
 */
export function initConfigAutoClose() {
    if (configAutoCloseInitialized) return;
    configAutoCloseInitialized = true;

    document.addEventListener('click', (e) => {
        const panel = document.getElementById('config-panel');
        const btn = document.getElementById('btn-cfg');
        if (!panel || !btn) return;
        if (!panel.classList.contains('visible')) return;

        const t = e && e.target;
        if (t && (panel.contains(t) || btn.contains(t))) return;

        panel.classList.remove('visible');
        btn.classList.remove('active');
    }, true);
}

/**
 * Toggle telemetry panel visibility
 */
export function toggleTelemetry() {
    const dom = ensureDomCache();
    if (dom.telemetryPanel) dom.telemetryPanel.classList.toggle('visible');
    if (dom.btnTelem) dom.btnTelem.classList.toggle('active');
}

/**
 * Update altitude offset
 * @param {string|number} val - New offset value
 */
export function updateOffset(val) {
    STATE.offsetAlt = parseFloat(val) || 0;
}

/**
 * Update AGL display
 * @param {number|null} height - Terrain height
 * @param {number} totalAlt - Total altitude
 */
export function updateAGLDisplay(height, totalAlt) {
    const dom = ensureDomCache();
    if (!dom.dispAgl) return;

    if (height !== null) {
        STATE.terrainHeight = height;
        const agl = totalAlt - height;
        dom.dispAgl.textContent = Math.round(agl);
    } else {
        dom.dispAgl.textContent = '---';
    }
}

/**
 * Update status message
 * @param {string} msg - Message to display
 * @param {string} color - CSS color value
 */
export function setStatusMessage(msg, color = 'var(--accent-cyan)') {
    const dom = ensureDomCache();
    if (dom.statusMsg) {
        dom.statusMsg.textContent = msg;
        dom.statusMsg.style.color = color;
    }
}

/**
 * Update FPS counter
 * @param {number} fps - Current FPS value
 */
export function updateFPSDisplay(fps) {
    const dom = ensureDomCache();
    if (dom.fpsCounter) {
        dom.fpsCounter.textContent = fps + ' FPS';
    }
}

/**
 * Initialize the header hamburger menu for secondary toggles.
 */
export function initMoreMenu() {
    const btn = document.getElementById('btn-more');
    const menu = document.getElementById('more-menu');
    if (!btn || !menu) return;

    const setExpanded = (expanded) => {
        try { btn.setAttribute('aria-expanded', expanded ? 'true' : 'false'); } catch (_) {}
    };

    const close = () => {
        if (!menu.classList.contains('visible')) return;
        menu.classList.remove('visible');
        setExpanded(false);
    };

    const toggle = (e) => {
        if (e) {
            e.preventDefault();
            e.stopPropagation();
        }
        const next = !menu.classList.contains('visible');
        if (next) menu.classList.add('visible');
        else menu.classList.remove('visible');
        setExpanded(next);
    };

    btn.addEventListener('click', toggle);

    // Close when clicking outside
    document.addEventListener('click', (e) => {
        if (!menu.classList.contains('visible')) return;
        const t = e && e.target;
        if (t && (menu.contains(t) || btn.contains(t))) return;
        close();
    });

    // Close on ESC
    document.addEventListener('keydown', (e) => {
        if (!menu.classList.contains('visible')) return;
        if (e.key === 'Escape') close();
    });

    // Close after selecting an item
    menu.querySelectorAll('button').forEach((b) => {
        b.addEventListener('click', () => close());
    });

    // Optional global hook (keeps consistent with other UI handlers)
    window.toggleMoreMenu = () => toggle();
}

/**
 * Open Chromium DevTools (main process handles the actual open).
 */
export function openDevTools() {
    try {
        if (window.devtools && typeof window.devtools.open === 'function') {
            window.devtools.open();
        } else {
            console.warn('devtools API not available (preload missing?)');
        }
    } catch (e) {
        console.warn('openDevTools failed', e);
    }
}

// Expose to global scope for HTML onclick handlers
window.toggleConfig = toggleConfig;
window.toggleTelemetry = toggleTelemetry;
window.updateOffset = updateOffset;
window.openDevTools = openDevTools;
