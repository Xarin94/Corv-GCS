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

// ── HUD Cell Customization ──────────────────────────────────────────────

const HUD_FIELD_DEFS = {
    as:          { label: 'Airspeed',    getter: () => STATE.as,                           fmt: v => v.toFixed(1),                              unit: 'M/S' },
    gs:          { label: 'Ground Spd',  getter: () => STATE.gs,                           fmt: v => v.toFixed(1),                              unit: 'M/S' },
    vs:          { label: 'Vert Speed',  getter: () => STATE.vs,                           fmt: v => v.toFixed(1),                              unit: 'M/S' },
    gload:       { label: 'G-Load',      getter: () => STATE.az / 9.81,                    fmt: v => v.toFixed(1),                              unit: 'G',     special: 'gload' },
    alt:         { label: 'Altitude',    getter: () => STATE.rawAlt + STATE.offsetAlt,      fmt: v => Math.round(v).toString(),                   unit: 'MSL' },
    agl:         { label: 'Terr Alt',    getter: () => null,                                fmt: v => Math.round(v).toString(),                   unit: 'AGL',   special: 'agl' },
    lidar:       { label: 'LiDAR',       getter: () => STATE.rangefinderDist,               fmt: v => v.toFixed(2),                              unit: 'm AGL', special: 'lidar' },
    roll:        { label: 'Roll',        getter: () => STATE.roll * RAD,                    fmt: v => v.toFixed(1),                              unit: 'DEG' },
    pitch:       { label: 'Pitch',       getter: () => STATE.pitch * RAD,                   fmt: v => v.toFixed(1),                              unit: 'DEG' },
    yaw:         { label: 'Heading',     getter: () => ((STATE.yaw * RAD) + 360) % 360,     fmt: v => v.toFixed(0).padStart(3, '0'),             unit: 'DEG' },
    battV:       { label: 'Battery V',   getter: () => STATE.batteryVoltage,                fmt: v => v.toFixed(1),                              unit: 'V' },
    battA:       { label: 'Battery A',   getter: () => STATE.batteryCurrent,                fmt: v => v.toFixed(1),                              unit: 'A' },
    battPct:     { label: 'Battery %',   getter: () => STATE.batteryRemaining,              fmt: v => Math.round(v).toString(),                   unit: '%' },
    gpsSat:      { label: 'GPS Sats',    getter: () => STATE.gpsNumSat,                     fmt: v => v.toString(),                              unit: '' },
    gpsHdop:     { label: 'GPS HDOP',    getter: () => STATE.gpsHdop,                       fmt: v => v.toFixed(1),                              unit: '' },
    linkQ:       { label: 'Link Qual',   getter: () => STATE.linkQuality,                   fmt: v => Math.round(v).toString(),                   unit: '%' },
    vibX:        { label: 'Vib X',       getter: () => STATE.vibX,                          fmt: v => v.toFixed(1),                              unit: '' },
    vibY:        { label: 'Vib Y',       getter: () => STATE.vibY,                          fmt: v => v.toFixed(1),                              unit: '' },
    vibZ:        { label: 'Vib Z',       getter: () => STATE.vibZ,                          fmt: v => v.toFixed(1),                              unit: '' },
    aoa:         { label: 'AoA',         getter: () => STATE.aoa * RAD,                     fmt: v => v.toFixed(1),                              unit: 'DEG' },
    ssa:         { label: 'SSA',         getter: () => STATE.ssa * RAD,                     fmt: v => v.toFixed(1),                              unit: 'DEG' },
    rtkBaseline: { label: 'RTK Base',    getter: () => STATE.rtkBaseline,                   fmt: v => v.toFixed(0),                              unit: 'mm' },
    rtkAccuracy: { label: 'RTK Acc',     getter: () => STATE.rtkAccuracy,                   fmt: v => v.toFixed(0),                              unit: 'mm' },
};

const DEFAULT_HUD_CELLS = [
    { field: 'as',    multiplier: 1, unitLabel: '' },
    { field: 'gs',    multiplier: 1, unitLabel: '' },
    { field: 'gload', multiplier: 1, unitLabel: '' },
    { field: 'alt',   multiplier: 1, unitLabel: '' },
    { field: 'agl',   multiplier: 1, unitLabel: '' },
    { field: 'lidar', multiplier: 1, unitLabel: '' },
];

const HUD_CELLS_STORAGE_KEY = 'datad-hud-cells';
let hudCellConfig = null;

/**
 * Initialize DOM cache for performance
 */
function ensureDomCache() {
    if (domCache) return domCache;

    const hudCellIds = ['disp-as', 'disp-gs', 'disp-g', 'disp-alt', 'disp-agl', 'disp-lidar'];
    const hudCells = hudCellIds.map(id => {
        const val = document.getElementById(id);
        const parent = val ? val.parentElement : null;
        return {
            val,
            lbl: parent ? parent.querySelector('.lbl') : null,
            unit: parent ? parent.querySelector('.unit') : null,
        };
    });

    domCache = {
        // Customizable HUD cells (left 0-2, right 3-5)
        hudCells,

        // Non-customizable display elements
        dispVs: document.getElementById('disp-vs'),
        dispLat: document.getElementById('disp-lat'),
        dispLon: document.getElementById('disp-lon'),
        dispAb: document.getElementById('disp-ab'),
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
    if (!hudCellConfig) loadHudCellConfig();

    // Non-customizable display updates
    if (dom.dispRalt) dom.dispRalt.textContent = STATE.rangefinderDist != null ? STATE.rangefinderDist.toFixed(1) : '---';
    if (dom.dispVs) dom.dispVs.textContent = STATE.vs.toFixed(1);
    if (dom.dispLat) dom.dispLat.textContent = STATE.lat.toFixed(5);
    if (dom.dispLon) dom.dispLon.textContent = STATE.lon.toFixed(5);
    if (dom.dispAb) dom.dispAb.textContent = `${(STATE.aoa * RAD).toFixed(1)}° / ${(STATE.ssa * RAD).toFixed(1)}°`;

    // Customizable HUD cells (config-driven loop)
    for (let i = 0; i < 6; i++) {
        const cell = dom.hudCells[i];
        if (!cell.val) continue;
        const cfg = hudCellConfig[i];
        const def = HUD_FIELD_DEFS[cfg.field];
        if (!def) continue;
        const mult = cfg.multiplier || 1;

        if (def.special === 'gload') {
            const rawG = HUD_FIELD_DEFS.gload.getter();
            filteredG = G_FILTER_ALPHA * filteredG + (1 - G_FILTER_ALPHA) * rawG;
            cell.val.textContent = (filteredG * mult).toFixed(1);
            let gColor;
            if (filteredG >= -1 && filteredG <= 4) gColor = '#00ff7f';
            else if (filteredG >= -2 && filteredG <= 6) gColor = '#ffcc00';
            else gColor = '#ff3333';
            cell.val.style.color = gColor;
        } else if (def.special === 'agl') {
            const alt = STATE.rawAlt + STATE.offsetAlt;
            if (STATE.terrainHeight !== null) {
                cell.val.textContent = Math.round((alt - STATE.terrainHeight) * mult);
            } else {
                cell.val.textContent = '---';
            }
            cell.val.style.color = '';
        } else if (def.special === 'lidar') {
            const dist = STATE.rangefinderDist;
            if (dist !== null && dist > 0) {
                cell.val.textContent = (dist * mult).toFixed(2);
                if (dist < 1) cell.val.style.color = '#ff3333';
                else if (dist <= 10) cell.val.style.color = '#ff8800';
                else cell.val.style.color = '';
            } else {
                cell.val.textContent = '---';
                cell.val.style.color = '';
            }
        } else {
            const raw = def.getter();
            if (raw === null || raw === undefined) {
                cell.val.textContent = '---';
            } else {
                cell.val.textContent = def.fmt(raw * mult);
            }
            cell.val.style.color = '';
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
 * Update terrain height in STATE (display handled by updateUI loop)
 */
export function updateAGLDisplay(height) {
    if (height !== null) {
        STATE.terrainHeight = height;
    }
}

// ── HUD Cell Config: persistence & UI ───────────────────────────────────

function loadHudCellConfig() {
    try {
        const raw = localStorage.getItem(HUD_CELLS_STORAGE_KEY);
        if (raw) {
            const parsed = JSON.parse(raw);
            if (Array.isArray(parsed) && parsed.length === 6) {
                // Validate every entry has a known field
                const valid = parsed.every(c => c && HUD_FIELD_DEFS[c.field]);
                if (valid) { hudCellConfig = parsed; return; }
            }
        }
    } catch (_) { /* ignore */ }
    hudCellConfig = structuredClone(DEFAULT_HUD_CELLS);
}

function saveHudCellConfig() {
    try { localStorage.setItem(HUD_CELLS_STORAGE_KEY, JSON.stringify(hudCellConfig)); } catch (_) { /* ignore */ }
}

function applyHudCellLabels() {
    const dom = ensureDomCache();
    for (let i = 0; i < 6; i++) {
        const cell = dom.hudCells[i];
        const cfg = hudCellConfig[i];
        const def = HUD_FIELD_DEFS[cfg.field];
        if (!def) continue;
        if (cell.lbl) cell.lbl.textContent = def.label;
        if (cell.unit) cell.unit.textContent = cfg.unitLabel || def.unit;
    }
}

function populateConfigUI() {
    const CELL_LABELS = ['LEFT 1', 'LEFT 2', 'LEFT 3', 'RIGHT 1', 'RIGHT 2', 'RIGHT 3'];
    for (let i = 0; i < 6; i++) {
        const sel = document.getElementById(`hud-cell-${i}-field`);
        if (!sel) continue;
        // Populate options
        sel.innerHTML = '';
        for (const [key, def] of Object.entries(HUD_FIELD_DEFS)) {
            const opt = document.createElement('option');
            opt.value = key;
            opt.textContent = def.label;
            sel.appendChild(opt);
        }
        sel.value = hudCellConfig[i].field;

        // Set multiplier and unit inputs
        const multInput = document.getElementById(`hud-cell-${i}-mult`);
        const unitInput = document.getElementById(`hud-cell-${i}-unit`);
        if (multInput) multInput.value = hudCellConfig[i].multiplier !== 1 ? hudCellConfig[i].multiplier : '';
        if (unitInput) unitInput.value = hudCellConfig[i].unitLabel || '';
    }
}

export function initHudCells() {
    loadHudCellConfig();
    applyHudCellLabels();
    populateConfigUI();
}

function setHudCellField(index, field) {
    if (!hudCellConfig || !HUD_FIELD_DEFS[field]) return;
    hudCellConfig[index].field = field;
    saveHudCellConfig();
    applyHudCellLabels();
}

function setHudCellMultiplier(index, mult) {
    if (!hudCellConfig) return;
    hudCellConfig[index].multiplier = isFinite(mult) ? mult : 1;
    saveHudCellConfig();
}

function setHudCellUnitLabel(index, label) {
    if (!hudCellConfig) return;
    hudCellConfig[index].unitLabel = label;
    saveHudCellConfig();
    applyHudCellLabels();
}

function resetHudCellConfig() {
    hudCellConfig = structuredClone(DEFAULT_HUD_CELLS);
    saveHudCellConfig();
    applyHudCellLabels();
    populateConfigUI();
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
window.setHudCellField = setHudCellField;
window.setHudCellMultiplier = setHudCellMultiplier;
window.setHudCellUnitLabel = setHudCellUnitLabel;
window.resetHudCellConfig = resetHudCellConfig;
