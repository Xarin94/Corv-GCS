/**
 * SplitView.js - Split View Mode with Plotly Charts
 * Handles the split screen layout with map and charts
 */

import { STATE, dataBuffer, BUFFER_SIZE, SAMPLE_INTERVAL } from '../core/state.js';
import { TRACE_CONFIG, RAD } from '../core/constants.js';
import { getPlaybackSpeed } from '../playback/LogPlayer.js';
import { calculateDistance } from '../core/utils.js';
import {
    initTraceManager,
    getActiveTraceConfigs,
    rebuildFormulaDataBuffer,
    sampleFormulaDataPoint,
    resetFormulaDataBuffer,
    buildLiveEntry,
    formulaDataBuffer,
    isTraceManagerInitialized
} from './TraceManager.js';

import { initND, resizeND, drawND, setNDVisibilityCheck } from './NDView.js';
import { initNDControls } from './NDController.js';
import { getHgtFileBounds } from '../terrain/TerrainManager.js';

// Split view state
let splitMap = null;
let splitMapMarker = null;
let splitPathLine = null;
let splitSatelliteLayer = null;
let splitSatelliteEnabled = true;
let splitHgtBoundsLayer = null;
let lastHgtBoundsKey = '';
let lastHgtBoundsRefreshAt = 0;
let livePathPoints = [];
let lastLiveLatLng = null;
let logLatLngCache = null;
let logLatLngCacheLen = 0;
let lastPlaybackRenderedIndex = -1;
const MAX_SPLIT_PATH_POINTS = 5000;
let plotlyInitialized = false;
let lastSampleTime = 0;
let viewMode = 'FULLSCREEN';
let pendingPlaybackPlotRebuild = false;
let plotWindowMinutes = 0; // 0 = ALL
let lastPlotlyRenderedCount = 0;
let lastPlotlyRenderedStartIndex = 0;
let lastPlotlyUpdatePerfMs = 0;
let lastPlotlyRelayoutPerfMs = 0;
let lastSamplePerfMs = performance.now();
let lastSampleSourceTimeMs = 0;

let plotlyResizeObserver = null;
let pendingPlotlyResizeRaf = 0;

function requestPlotlyResize() {
    if (!plotlyInitialized) return;
    if (isPlotCollapsed()) return;
    if (pendingPlotlyResizeRaf) return;
    pendingPlotlyResizeRaf = requestAnimationFrame(() => {
        pendingPlotlyResizeRaf = 0;
        try { Plotly.Plots.resize('plotly-chart'); } catch (e) {}
    });
}

// Split top panes selection (kept distinct: if user picks an already-used view, panes swap)
let splitPaneLeft = '3d';
let splitPaneRight = '2d';
let splitPaneControlsInitialized = false;
let ndInitialized = false;

/**
 * Check if 3D view is currently visible
 * @returns {boolean}
 */
export function is3DVisible() {
    if (viewMode === 'FULLSCREEN') return true;
    if (viewMode !== 'SPLIT') return false;
    return splitPaneLeft === '3d' || splitPaneRight === '3d';
}

/**
 * Check if 2D map is currently visible
 * @returns {boolean}
 */
export function is2DMapVisible() {
    if (viewMode !== 'SPLIT') return false;
    return splitPaneLeft === '2d' || splitPaneRight === '2d';
}

/**
 * Check if ND is currently visible
 * @returns {boolean}
 */
export function isNDVisible() {
    if (viewMode !== 'SPLIT') return false;
    return splitPaneLeft === 'nd' || splitPaneRight === 'nd';
}

function isPlotCollapsed() {
    try {
        return document.body && document.body.dataset && document.body.dataset.plotCollapsed === '1';
    } catch (e) {
        return false;
    }
}

function syncPlotCollapseButton() {
    const btn = document.getElementById('btn-plot-collapse');
    if (!btn) return;

    const collapsed = isPlotCollapsed();
    btn.textContent = collapsed ? 'EXPAND' : 'COLLAPSE';
    btn.setAttribute('title', collapsed ? 'Expand graph' : 'Collapse graph');
    btn.setAttribute('aria-label', collapsed ? 'Expand graph' : 'Collapse graph');
}

function setPlotCollapsed(collapsed) {
    const next = !!collapsed;
    try {
        if (next) document.body.dataset.plotCollapsed = '1';
        else delete document.body.dataset.plotCollapsed;
    } catch (e) {}

    syncPlotCollapseButton();

    // When expanding, Plotly needs a resize + a render to match the current window.
    if (!next && plotlyInitialized) {
        try { renderPlotlyFromBuffer(); } catch (e) {}
        requestPlotlyResize();
    }

    requestLayoutRefreshAfterPaneChange();
}

function normalizePaneValue(v) {
    const s = String(v || '').toLowerCase();
    if (s === '3d' || s === '2d' || s === 'nd') return s;
    return null;
}

function applySplitPaneLayout() {
    if (viewMode !== 'SPLIT') {
        try {
            delete document.body.dataset.splitLeft;
            delete document.body.dataset.splitRight;
        } catch (e) {}
        return;
    }

    document.body.dataset.splitLeft = splitPaneLeft;
    document.body.dataset.splitRight = splitPaneRight;
}

function syncSplitPaneSelects() {
    // Per-view containers each own a select; keep them aligned to their container's view.
    const hudSel = document.querySelector('#hud-wrapper .pane-view-select');
    const mapSel = document.querySelector('#split-map-container .pane-view-select');
    const ndSel = document.querySelector('#nd-container .pane-view-select');

    if (hudSel) hudSel.value = '3d';
    if (mapSel) mapSel.value = '2d';
    if (ndSel) ndSel.value = 'nd';
}

function getPaneFromContainer(containerEl) {
    if (!containerEl || viewMode !== 'SPLIT') return null;
    try {
        const cs = window.getComputedStyle(containerEl);
        if (!cs || cs.display === 'none') return null;
        const start = parseInt(cs.gridColumnStart, 10);
        if (start === 1) return 'left';
        if (start === 2) return 'right';
    } catch (e) {}

    // Fallback: compare position to viewport midpoint.
    try {
        const rect = containerEl.getBoundingClientRect();
        const mid = rect.left + rect.width / 2;
        return (mid < window.innerWidth / 2) ? 'left' : 'right';
    } catch (e) {
        return null;
    }
}

function requestLayoutRefreshAfterPaneChange() {
    // Leaflet map needs an explicit size invalidation after any layout changes.
    requestAnimationFrame(() => {
        try {
            if (splitMap) splitMap.invalidateSize();
        } catch (e) {}
        try {
            resizeND();
        } catch (e) {}
        // Re-use the app's existing resize handler (Three.js + HUD).
        try {
            if (typeof window.onresize === 'function') window.onresize();
        } catch (e) {}
    });
}

function setPaneView(pane, view) {
    const v = normalizePaneValue(view);
    if (!v) return;

    if (pane === 'left') {
        if (v === splitPaneRight) {
            const tmp = splitPaneLeft;
            splitPaneLeft = splitPaneRight;
            splitPaneRight = tmp;
        } else {
            splitPaneLeft = v;
        }
    } else if (pane === 'right') {
        if (v === splitPaneLeft) {
            const tmp = splitPaneRight;
            splitPaneRight = splitPaneLeft;
            splitPaneLeft = tmp;
        } else {
            splitPaneRight = v;
        }
    }

    applySplitPaneLayout();
    syncSplitPaneSelects();
    requestLayoutRefreshAfterPaneChange();
}

function initSplitPaneControls() {
    if (splitPaneControlsInitialized) return;

    const hudWrapper = document.getElementById('hud-wrapper');
    const splitMapContainer = document.getElementById('split-map-container');
    const ndContainer = document.getElementById('nd-container');

    const hudSel = hudWrapper ? hudWrapper.querySelector('.pane-view-select') : null;
    const mapSel = splitMapContainer ? splitMapContainer.querySelector('.pane-view-select') : null;
    const ndSel = ndContainer ? ndContainer.querySelector('.pane-view-select') : null;

    if (hudSel) {
        hudSel.value = '3d';
        addTrackedListener(hudSel, 'change', (e) => {
            const pane = getPaneFromContainer(hudWrapper);
            if (!pane) return;
            setPaneView(pane, e.target && e.target.value);
        });
    }

    if (mapSel) {
        mapSel.value = '2d';
        addTrackedListener(mapSel, 'change', (e) => {
            const pane = getPaneFromContainer(splitMapContainer);
            if (!pane) return;
            setPaneView(pane, e.target && e.target.value);
        });
    }

    if (ndSel) {
        ndSel.value = 'nd';
        addTrackedListener(ndSel, 'change', (e) => {
            const pane = getPaneFromContainer(ndContainer);
            if (!pane) return;
            setPaneView(pane, e.target && e.target.value);
        });
    }

    syncSplitPaneSelects();
    syncPlotCollapseButton();

    splitPaneControlsInitialized = true;
}

function initNDIfNeeded() {
    if (ndInitialized) return;
    const canvas = document.getElementById('nd-canvas');
    ndInitialized = !!initND(canvas);
    
    // Initialize ND control panel event handlers and set visibility check
    if (ndInitialized) {
        initNDControls();
        setNDVisibilityCheck(isNDVisible);
    }
}

// Coordinate box (bottom bar) DOM relocation for split view
let coordBoxOriginalParent = null;
let coordBoxOriginalNextSibling = null;

function moveCoordinateBoxForViewMode() {
    const coordBox = document.querySelector('.bottom-bar');
    if (!coordBox) return;

    if (viewMode === 'SPLIT') {
        const splitMapContainer = document.getElementById('split-map-container');
        if (!splitMapContainer) return;

        if (!coordBoxOriginalParent) {
            coordBoxOriginalParent = coordBox.parentElement;
            coordBoxOriginalNextSibling = coordBox.nextElementSibling;
        }

        if (coordBox.parentElement !== splitMapContainer) {
            splitMapContainer.appendChild(coordBox);
        }
        return;
    }

    // FULLSCREEN: restore to original location in #ui-layer
    if (coordBoxOriginalParent && coordBox.parentElement !== coordBoxOriginalParent) {
        if (coordBoxOriginalNextSibling && coordBoxOriginalNextSibling.parentElement === coordBoxOriginalParent) {
            coordBoxOriginalParent.insertBefore(coordBox, coordBoxOriginalNextSibling);
        } else {
            coordBoxOriginalParent.appendChild(coordBox);
        }
    }
}

// Event listener cleanup tracking
const eventListeners = [];

/**
 * Add event listener with cleanup tracking
 */
function addTrackedListener(target, event, handler) {
    target.addEventListener(event, handler);
    eventListeners.push({ target, event, handler });
}

/**
 * Remove all tracked event listeners
 */
function removeAllTrackedListeners() {
    for (const { target, event, handler } of eventListeners) {
        try {
            target.removeEventListener(event, handler);
        } catch (e) {
            // Ignore errors if target is no longer valid
        }
    }
    eventListeners.length = 0;

    if (plotlyResizeObserver) {
        try { plotlyResizeObserver.disconnect(); } catch (e) {}
        plotlyResizeObserver = null;
    }
    if (pendingPlotlyResizeRaf) {
        try { cancelAnimationFrame(pendingPlotlyResizeRaf); } catch (e) {}
        pendingPlotlyResizeRaf = 0;
    }
}

/**
 * Record a live/demo path point even when not in split view.
 * Stores points as [lat, lon] pairs so it doesn't depend on Leaflet types.
 */
export function recordLivePathPoint() {
    if (STATE.mode === 'PLAYBACK') return;
    const lat = STATE.lat;
    const lon = STATE.lon;
    if (!Number.isFinite(lat) || !Number.isFinite(lon)) return;

    if (!lastLiveLatLng) {
        lastLiveLatLng = { lat, lon };
        if (!livePathPoints.length) livePathPoints = [[lat, lon]];
        return;
    }

    const moved = calculateDistance(lastLiveLatLng.lat, lastLiveLatLng.lon, lat, lon);
    if (moved < 5) return;

    livePathPoints.push([lat, lon]);
    lastLiveLatLng = { lat, lon };

    if (livePathPoints.length > MAX_SPLIT_PATH_POINTS) {
        livePathPoints = livePathPoints.slice(livePathPoints.length - MAX_SPLIT_PATH_POINTS);
    }
}

function lowerBound(arr, x) {
    // First index i such that arr[i] >= x
    let lo = 0;
    let hi = arr.length;
    while (lo < hi) {
        const mid = (lo + hi) >> 1;
        if (arr[mid] < x) lo = mid + 1;
        else hi = mid;
    }
    return lo;
}

function getPlotWindowStartIndex() {
    const ts = formulaDataBuffer.timestamps;
    if (!ts || ts.length === 0) return 0;
    if (!plotWindowMinutes) return 0;

    const endMs = ts[ts.length - 1];
    const cutoff = endMs - (plotWindowMinutes * 60 * 1000);
    return lowerBound(ts, cutoff);
}

function getSmoothedEndTimeMs() {
    const lastTs = formulaDataBuffer.length ? formulaDataBuffer.getLastTimestamp() : 0;
    if (!lastTs) return 0;

    const nowPerf = performance.now();
    const dtPerf = Math.max(0, nowPerf - lastSamplePerfMs);
    const dtClamped = Math.min(dtPerf, SAMPLE_INTERVAL);
    const speed = (STATE.mode === 'PLAYBACK') ? (getPlaybackSpeed() || 1) : 1;

    // Use the source time from the last accepted sample as baseline.
    const base = lastSampleSourceTimeMs || lastTs;
    return base + (dtClamped * speed);
}

function renderPlotlyFromBuffer() {
    if (!plotlyInitialized || viewMode !== 'SPLIT') return;
    if (isPlotCollapsed()) return;

    const traceConfigs = getActiveTraceConfigs();
    const start = getPlotWindowStartIndex();
    const ts = formulaDataBuffer.timestamps.slice(start);
    const xDates = ts.map(t => new Date(t));

    const traces = traceConfigs.map((config) => ({
        x: xDates,
        y: (formulaDataBuffer.traces[config.index] || []).slice(start),
        name: config.label,
        type: 'scatter',
        mode: 'lines',
        yaxis: config.yaxis,
        line: { color: config.color, width: 2 }
    }));

    const layout = buildPlotlyLayoutFromConfigs(traceConfigs);
    Plotly.react('plotly-chart', traces, layout);

    // Ensure range matches the current WINDOW selection.
    if (plotWindowMinutes) {
        const endMs = getSmoothedEndTimeMs() || (formulaDataBuffer.length ? formulaDataBuffer.getLastTimestamp() : 0);
        if (endMs) {
            const windowMs = plotWindowMinutes * 60 * 1000;
            Plotly.relayout('plotly-chart', {
                'xaxis.range': [new Date(endMs - windowMs), new Date(endMs)]
            });
        }
    } else {
        Plotly.relayout('plotly-chart', { 'xaxis.autorange': true });
    }

    lastPlotlyRenderedCount = formulaDataBuffer.length;
    lastPlotlyRenderedStartIndex = getPlotWindowStartIndex();
}

function clampIndex(idx, len) {
    if (!Number.isFinite(idx)) return 0;
    if (len <= 0) return 0;
    return Math.max(0, Math.min(idx, len - 1));
}

function getLogEntryParts(entry) {
    const r = entry || {};
    const s = (r.state || r) || {};
    const i = (r.imu || r) || {};
    return { r, s, i };
}

function computeDerivedFromLogEntry(entry) {
    const { s, i } = getLogEntryParts(entry);
    const vn = (typeof s.vn === 'number') ? s.vn : 0;
    const ve = (typeof s.ve === 'number') ? s.ve : 0;
    const vd = (typeof s.vd === 'number') ? s.vd : 0;

    const as = Math.sqrt(vn ** 2 + ve ** 2 + vd ** 2);
    const gs = Math.sqrt(vn ** 2 + ve ** 2);
    const vs = -vd;
    const rawAlt = (typeof s.alt === 'number') ? s.alt : 0;
    const roll = (typeof s.roll === 'number') ? s.roll : 0;
    const pitch = (typeof s.pitch === 'number') ? s.pitch : 0;
    const ax = (typeof i.ax === 'number') ? i.ax : 0;
    const ay = (typeof i.ay === 'number') ? i.ay : 0;
    const az = (typeof i.az === 'number') ? i.az : 0;
    const gLoad = az / 9.81;  // Normal load factor (body Z axis)

    return {
        as,
        gs,
        vs,
        rawAlt,
        rollDeg: roll * RAD,
        pitchDeg: pitch * RAD,
        gLoad
    };
}

function rebuildPlotFromPlaybackPrefix() {
    if (viewMode !== 'SPLIT' || !plotlyInitialized) return;
    if (STATE.mode !== 'PLAYBACK') return;
    if (!Array.isArray(STATE.logData) || STATE.logData.length === 0) return;

    // Use TraceManager to rebuild formula data buffer
    rebuildFormulaDataBuffer();

    const lastTs = formulaDataBuffer.length ? formulaDataBuffer.getLastTimestamp() : 0;
    lastSampleTime = lastTs;
    lastSampleSourceTimeMs = lastSampleTime;
    lastSamplePerfMs = performance.now();

    pendingPlaybackPlotRebuild = false;
    // Re-render to avoid any implicit line continuity across a seek,
    // and apply the current WINDOW selection.
    renderPlotlyFromBuffer();
}

function buildLogLatLngCache() {
    if (!Array.isArray(STATE.logData) || STATE.logData.length === 0) {
        logLatLngCache = null;
        logLatLngCacheLen = 0;
        return;
    }
    if (logLatLngCache && logLatLngCacheLen === STATE.logData.length) return;

    logLatLngCache = STATE.logData.map((r) => {
        const s = (r && (r.state || r)) || {};
        const lat = (typeof s.lat === 'number') ? s.lat : STATE.lat;
        const lon = (typeof s.lon === 'number') ? s.lon : STATE.lon;
        return new L.LatLng(lat, lon);
    });
    logLatLngCacheLen = STATE.logData.length;
    lastPlaybackRenderedIndex = -1;
}

function downsample(points, maxPoints) {
    if (!points || points.length <= maxPoints) return points;
    const step = Math.ceil(points.length / maxPoints);
    const out = [];
    for (let i = 0; i < points.length; i += step) out.push(points[i]);
    const last = points[points.length - 1];
    if (out.length === 0 || out[out.length - 1] !== last) out.push(last);
    return out;
}

// Note: activeTraces is now managed by TraceManager

/**
 * Get current view mode
 * @returns {string}
 */
export function getViewMode() {
    return viewMode;
}

/**
 * Set view mode
 * @param {string} mode 
 */
export function setViewMode(mode) {
    viewMode = mode;
}

/**
 * Toggle between fullscreen and split view
 */
export function toggleViewMode() {
    viewMode = (viewMode === 'FULLSCREEN') ? 'SPLIT' : 'FULLSCREEN';
    document.body.classList.remove('mode-fullscreen', 'mode-split');
    document.body.classList.add(`mode-${viewMode.toLowerCase()}`);

    // Apply/clear split top pane layout attributes
    applySplitPaneLayout();

    const btn = document.getElementById('btn-mode');
    if (btn) {
        // Keep icon content; only update accessible label + tooltip.
        const nextAction = (viewMode === 'FULLSCREEN') ? 'Split view' : 'Fullscreen';
        btn.setAttribute('title', nextAction);
        btn.setAttribute('aria-label', nextAction);
    }

    resetDataBuffer();

    // Ensure the coordinate box is visible in split view (move it to the right map)
    moveCoordinateBoxForViewMode();

    if (viewMode === 'SPLIT') {
        initSplitPaneControls();
        initNDIfNeeded();
        initSplitView();
        syncSplitPaneSelects();
        requestLayoutRefreshAfterPaneChange();
    }

    // Return the new mode for resize handling
    return viewMode;
}

/**
 * Reset the data buffer for Plotly
 */
export function resetDataBuffer() {
    // Reset both old and new buffers using efficient clear
    dataBuffer.clear();
    lastSampleTime = 0;

    // Reset formula data buffer
    resetFormulaDataBuffer();

    if (plotlyInitialized) {
        const traceConfigs = getActiveTraceConfigs();
        const emptyTraces = traceConfigs.map((config) => ({
            x: [],
            y: [],
            name: config.label,
            type: 'scatter',
            mode: 'lines',
            yaxis: config.yaxis,
            line: { color: config.color, width: 2 }
        }));
        Plotly.react('plotly-chart', emptyTraces, buildPlotlyLayoutFromConfigs(traceConfigs));
    }
}

/**
 * Initialize split view components
 */
function initSplitView() {
    // Ensure split pane layout attributes exist before sizing.
    applySplitPaneLayout();
    initNDIfNeeded();

    // Initialize large map if not exists
    if (!splitMap) {
        splitMap = L.map('split-map', {
            zoomControl: true,
            attributionControl: false,
            keyboard: false
        }).setView([STATE.lat, STATE.lon], 13);
        
        splitSatelliteLayer = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
            maxZoom: 19,
            opacity: 0.9,
            attribution: 'Tiles &copy; Esri'
        });
        if (splitSatelliteEnabled) splitSatelliteLayer.addTo(splitMap);

        const planeIcon = L.divIcon({
            html: '<svg viewBox="0 0 32 32" style="filter:drop-shadow(0 0 2px #000);"><polygon points="16,2 20,28 16,24 12,28" fill="#00d2ff" stroke="#fff" stroke-width="1"/></svg>',
            className: 'plane-marker-icon',
            iconSize: [32, 32],
            iconAnchor: [16, 16]
        });
        splitMapMarker = L.marker([STATE.lat, STATE.lon], { icon: planeIcon }).addTo(splitMap);
    }

    if (!splitHgtBoundsLayer) {
        splitHgtBoundsLayer = L.layerGroup();
    }
    if (splitMap && !splitMap.hasLayer(splitHgtBoundsLayer)) {
        splitHgtBoundsLayer.addTo(splitMap);
    }

    // Initialize / re-add path line
    if (!splitPathLine) {
        splitPathLine = L.polyline([], {
            color: '#ff0000',
            weight: 2,
            opacity: 0.85
        }).addTo(splitMap);
    } else if (splitMap && !splitMap.hasLayer(splitPathLine)) {
        splitPathLine.addTo(splitMap);
    }

    // Seed path immediately when entering split view (LIVE/DEMO).
    if (STATE.mode !== 'PLAYBACK') {
        recordLivePathPoint();
        if (splitPathLine && livePathPoints.length) {
            splitPathLine.setLatLngs(livePathPoints);
        }
    }

    // Initialize Plotly
    if (!plotlyInitialized) {
        initPlotly();
    }

    // If we are entering split view while in playback, rebuild the plot from the log prefix
    // (prevents long jump segments after seek/skip).
    if (STATE.mode === 'PLAYBACK' && Array.isArray(STATE.logData) && STATE.logData.length > 0) {
        pendingPlaybackPlotRebuild = true;
        rebuildPlotFromPlaybackPrefix();
    }

    setTimeout(() => splitMap.invalidateSize(), 100);

    refreshHgtBoundsOverlay(true);
}

/**
 * Toggle split-map satellite tiles.
 * @param {boolean} enabled
 */
export function setSplitMapSatelliteEnabled(enabled) {
    splitSatelliteEnabled = !!enabled;
    if (!splitMap || !splitSatelliteLayer) return;

    if (splitSatelliteEnabled) {
        if (!splitMap.hasLayer(splitSatelliteLayer)) splitSatelliteLayer.addTo(splitMap);
    } else {
        if (splitMap.hasLayer(splitSatelliteLayer)) splitMap.removeLayer(splitSatelliteLayer);
    }
}

/**
 * Update split view map
 */
export function updateSplitMap() {
    if (viewMode !== 'SPLIT' || !splitMap) return;
    // Skip if 2D map is not visible
    if (!is2DMapVisible()) return;

    refreshHgtBoundsOverlay(false);

    const newLatLng = new L.LatLng(STATE.lat, STATE.lon);
    splitMapMarker.setLatLng(newLatLng);
    splitMap.panTo(newLatLng);

    // Update path line
    if (splitPathLine) {
        if (STATE.mode === 'PLAYBACK' && Array.isArray(STATE.logData) && STATE.logData.length > 0) {
            buildLogLatLngCache();
            const idx = Math.max(0, Math.min(STATE.logIndex, logLatLngCacheLen - 1));
            if (idx !== lastPlaybackRenderedIndex && logLatLngCache) {
                const prefix = logLatLngCache.slice(0, idx + 1);
                splitPathLine.setLatLngs(downsample(prefix, MAX_SPLIT_PATH_POINTS));
                lastPlaybackRenderedIndex = idx;
            }
        } else {
            // Keep collecting points even outside split view.
            recordLivePathPoint();
            splitPathLine.setLatLngs(livePathPoints);
        }
    }

    const iconElement = splitMapMarker.getElement();
    if (iconElement) {
        // IMPORTANT: Leaflet uses `transform: translate3d(...)` to place markers.
        // Rotating the marker element itself can break positioning (especially in split layouts).
        // Rotate only the inner SVG.
        const deg = STATE.yaw * RAD;
        const svg = iconElement.querySelector('svg');
        if (svg) {
            svg.style.transformOrigin = 'center center';
            svg.style.transform = `rotate(${deg}deg)`;
        }
    }
}

function refreshHgtBoundsOverlay(force) {
    if (!splitMap || !splitHgtBoundsLayer) return;
    const now = performance.now();
    if (!force && now - lastHgtBoundsRefreshAt < 2000) return;
    lastHgtBoundsRefreshAt = now;

    const bounds = getHgtFileBounds();
    if (!bounds.length) {
        if (lastHgtBoundsKey) {
            splitHgtBoundsLayer.clearLayers();
            lastHgtBoundsKey = '';
        }
        return;
    }

    const key = bounds.map(b => b.key).sort().join('|');
    if (!force && key === lastHgtBoundsKey) return;

    splitHgtBoundsLayer.clearLayers();
    const style = {
        color: '#00d2ff',
        weight: 1,
        opacity: 0.85,
        fill: false,
        dashArray: '4 4'
    };
    for (const b of bounds) {
        const rect = L.rectangle([[b.latBottom, b.lonLeft], [b.latTop, b.lonRight]], style);
        splitHgtBoundsLayer.addLayer(rect);
    }

    lastHgtBoundsKey = key;
}

/**
 * Build Plotly layout configuration from trace configs
 */
function buildPlotlyLayoutFromConfigs(traceConfigs) {
    const hasY1 = traceConfigs.some(c => c.yaxis === 'y');
    const hasY2 = traceConfigs.some(c => c.yaxis === 'y2');
    const hasY3 = traceConfigs.some(c => c.yaxis === 'y3');

    const marginL = 50;
    const marginR = (hasY2 ? 60 : 20) + (hasY3 ? 60 : 0);

    const layout = {
        paper_bgcolor: 'rgba(0,0,0,0)',
        plot_bgcolor: 'rgba(0,20,0,0.3)',
        margin: { l: marginL, r: marginR, t: 10, b: 30 },
        font: { family: 'Roboto Mono, monospace', color: '#00d2ff' },
        xaxis: { showgrid: true, gridcolor: '#333', zeroline: false, autorange: true },
        yaxis: { showgrid: true, gridcolor: '#333', zeroline: false, title: hasY1 ? 'Y1' : '' },
        showlegend: true,
        legend: { orientation: 'h', y: 1.05, x: 0.5, xanchor: 'center' }
    };

    if (hasY2) {
        layout.yaxis2 = {
            showgrid: false, zeroline: false, title: 'Y2',
            overlaying: 'y', side: 'right', titlefont: { color: '#FFFF00' },
            tickfont: { color: '#FFFF00' }
        };
    }

    if (hasY3) {
        layout.yaxis3 = {
            showgrid: false, zeroline: false, title: 'Y3',
            overlaying: 'y', side: 'right', titlefont: { color: '#FF00FF' },
            tickfont: { color: '#FF00FF' },
            anchor: 'free', position: hasY2 ? 0.95 : 1
        };
    }

    return layout;
}

/**
 * Build Plotly layout configuration (legacy, kept for compatibility)
 */
function buildPlotlyLayout(fields) {
    const hasY1 = fields.some(f => TRACE_CONFIG[f] && TRACE_CONFIG[f].yaxis === 'y');
    const hasY2 = fields.some(f => TRACE_CONFIG[f] && TRACE_CONFIG[f].yaxis === 'y2');
    const hasY3 = fields.some(f => TRACE_CONFIG[f] && TRACE_CONFIG[f].yaxis === 'y3');

    const marginL = 50;
    const marginR = (hasY2 ? 60 : 20) + (hasY3 ? 60 : 0);

    const layout = {
        paper_bgcolor: 'rgba(0,0,0,0)',
        plot_bgcolor: 'rgba(0,20,0,0.3)',
        margin: { l: marginL, r: marginR, t: 10, b: 30 },
        font: { family: 'Roboto Mono, monospace', color: '#00d2ff' },
        xaxis: { showgrid: true, gridcolor: '#333', zeroline: false, autorange: true },
        yaxis: { showgrid: true, gridcolor: '#333', zeroline: false, title: hasY1 ? 'Speed/G' : '' },
        showlegend: true,
        legend: { orientation: 'h', y: 1.05, x: 0.5, xanchor: 'center' }
    };

    if (hasY2) {
        layout.yaxis2 = {
            showgrid: false, zeroline: false, title: 'Altitude (m)',
            overlaying: 'y', side: 'right', titlefont: { color: '#FFFF00' },
            tickfont: { color: '#FFFF00' }
        };
    }

    if (hasY3) {
        layout.yaxis3 = {
            showgrid: false, zeroline: false, title: 'Angle (°)',
            overlaying: 'y', side: 'right', titlefont: { color: '#FF00FF' },
            tickfont: { color: '#FF00FF' },
            anchor: 'free', position: hasY2 ? 0.95 : 1
        };
    }

    return layout;
}

/**
 * Build Plotly traces (legacy, kept for compatibility)
 */
function buildPlotlyTraces(fields, withData = false) {
    return fields.map(field => ({
        x: withData ? dataBuffer.timestamps.map(t => new Date(t)) : [],
        y: withData ? dataBuffer[field] : [],
        name: TRACE_CONFIG[field] ? TRACE_CONFIG[field].name : field,
        type: 'scatter', mode: 'lines',
        yaxis: TRACE_CONFIG[field] ? TRACE_CONFIG[field].yaxis : 'y',
        line: { color: TRACE_CONFIG[field] ? TRACE_CONFIG[field].color : '#00FF00', width: 2 }
    }));
}

/**
 * Initialize Plotly chart
 */
function initPlotly() {
    // Initialize trace manager first
    initTraceManager();

    const traceConfigs = getActiveTraceConfigs();
    const traces = traceConfigs.map((config) => ({
        x: [],
        y: [],
        name: config.label,
        type: 'scatter',
        mode: 'lines',
        yaxis: config.yaxis,
        line: { color: config.color, width: 2 }
    }));

    const layout = buildPlotlyLayoutFromConfigs(traceConfigs);

    const config = {
        responsive: true,
        displayModeBar: false
    };

    Plotly.newPlot('plotly-chart', traces, layout, config);
    plotlyInitialized = true;

    // Graph collapse/expand button
    syncPlotCollapseButton();
    const collapseBtn = document.getElementById('btn-plot-collapse');
    if (collapseBtn) {
        addTrackedListener(collapseBtn, 'click', () => {
            setPlotCollapsed(!isPlotCollapsed());
        });
    }

    // Keep Plotly responsive when controls expand/collapse (e.g., adding traces)
    try {
        const container = document.getElementById('plotly-container');
        if (container && window.ResizeObserver) {
            plotlyResizeObserver = new ResizeObserver(() => {
                requestPlotlyResize();
            });
            plotlyResizeObserver.observe(container);
        }
    } catch (e) {}

    // Listen for trace configuration changes from TraceManager (tracked for cleanup)
    const traceChangeHandler = () => {
        renderPlotlyFromBuffer();
        requestPlotlyResize();
    };
    addTrackedListener(window, 'traceConfigChanged', traceChangeHandler);

    const windowSel = document.getElementById('plot-window');
    if (windowSel) {
        const applyWindow = () => {
            const v = (windowSel.value || 'all').toLowerCase();
            plotWindowMinutes = (v === 'all') ? 0 : (parseInt(v, 10) || 0);
            renderPlotlyFromBuffer();
            requestPlotlyResize();
        };
        addTrackedListener(windowSel, 'change', applyWindow);
        applyWindow();
    }
}

/**
 * Sample data point for Plotly
 */
export function sampleDataPoint() {
    // Use a consistent time base:
    // - LIVE: wall clock (Date.now)
    // - PLAYBACK: log timestamp (entry.t) so the chart reflects replay speed
    let sampleTimeMs = Date.now();
    let entry = null;

    if (STATE.mode === 'PLAYBACK' && Array.isArray(STATE.logData) && STATE.logData.length > 0) {
        entry = STATE.logData[STATE.logIndex];
        const t = entry && entry.t;
        if (typeof t === 'number' && Number.isFinite(t)) sampleTimeMs = t;
    }

    // If the user seeks backwards (or the log loops), allow immediate re-sampling.
    if (sampleTimeMs < lastSampleTime) {
        lastSampleTime = sampleTimeMs - SAMPLE_INTERVAL;
    }

    if (sampleTimeMs - lastSampleTime < SAMPLE_INTERVAL) return;
    lastSampleTime = sampleTimeMs;
    lastSampleSourceTimeMs = sampleTimeMs;
    lastSamplePerfMs = performance.now();

    // Sample for formula-based traces
    if (STATE.mode === 'PLAYBACK' && entry) {
        sampleFormulaDataPoint(entry, sampleTimeMs);
    } else {
        // For live mode, build entry from STATE
        const liveEntry = buildLiveEntry();
        sampleFormulaDataPoint(liveEntry, sampleTimeMs);
    }

    // Keep legacy buffer updated for compatibility using O(1) ring buffer push
    dataBuffer.pushTimestamp(sampleTimeMs);
    dataBuffer.pushAs(STATE.as);
    dataBuffer.pushGs(STATE.gs);
    dataBuffer.pushVs(STATE.vs);
    dataBuffer.pushRawAlt(STATE.rawAlt + STATE.offsetAlt);
    dataBuffer.pushRoll(STATE.roll * RAD);
    dataBuffer.pushPitch(STATE.pitch * RAD);
    dataBuffer.pushAz(STATE.az / 9.81);  // Normal load factor (body Z axis)
}

/**
 * Update Plotly chart
 */
export function updatePlotly() {
    if (!plotlyInitialized || viewMode !== 'SPLIT') return;
    if (isPlotCollapsed()) return;

    if (pendingPlaybackPlotRebuild) {
        rebuildPlotFromPlaybackPrefix();
        return;
    }

    const nowPerf = performance.now();

    // Smooth scrolling: move x-axis window at ~30fps even between samples.
    if (plotWindowMinutes) {
        if (nowPerf - lastPlotlyRelayoutPerfMs >= 33) {
            const endMs = getSmoothedEndTimeMs();
            if (endMs) {
                const windowMs = plotWindowMinutes * 60 * 1000;
                Plotly.relayout('plotly-chart', {
                    'xaxis.range': [new Date(endMs - windowMs), new Date(endMs)]
                });
            }
            lastPlotlyRelayoutPerfMs = nowPerf;
        }
    }

    // Update trace data only when needed (new samples or window start index moved).
    const start = getPlotWindowStartIndex();
    const curCount = formulaDataBuffer.length;
    const needsDataUpdate = (curCount !== lastPlotlyRenderedCount) || (start !== lastPlotlyRenderedStartIndex);
    if (!needsDataUpdate) return;

    // Throttle expensive data updates a bit to reduce jank.
    if (nowPerf - lastPlotlyUpdatePerfMs < 50) return;

    const traceConfigs = getActiveTraceConfigs();
    const ts = formulaDataBuffer.timestamps.slice(start);
    const xDates = ts.map(t => new Date(t));

    const update = {
        x: traceConfigs.map(() => xDates),
        y: traceConfigs.map(c => (formulaDataBuffer.traces[c.index] || []).slice(start))
    };

    const indices = traceConfigs.map((_, i) => i);
    Plotly.update('plotly-chart', update, {}, indices);
    lastPlotlyRenderedCount = curCount;
    lastPlotlyRenderedStartIndex = start;
    lastPlotlyUpdatePerfMs = nowPerf;
}

/**
 * Resize split view components
 */
export function resizeSplitView() {
    if (splitMap) splitMap.invalidateSize();
    if (plotlyInitialized && !isPlotCollapsed()) Plotly.Plots.resize('plotly-chart');
    try { resizeND(); } catch (e) {}
}

/**
 * Update ND rendering (called from main loop in split view)
 */
export function updateND() {
    if (!isNDVisible()) return;
    try {
        initNDIfNeeded();
        drawND();
    } catch (e) {}
}

/**
 * Check if Plotly is initialized
 * @returns {boolean}
 */
export function isPlotlyInitialized() {
    return plotlyInitialized;
}

/**
 * Cleanup split view resources (event listeners, etc.)
 * Called when switching away from split view to prevent memory leaks
 */
export function cleanupSplitView() {
    removeAllTrackedListeners();
}

// Expose to global scope
window.toggleViewMode = toggleViewMode;

// When scrubbing/seeking during playback, rebuild plot from the log prefix so
// the lines don't create a big segment across the skipped time.
window.addEventListener('logLoaded', () => {
    pendingPlaybackPlotRebuild = true;
    lastPlotlyRenderedCount = 0;
    lastPlotlyRenderedStartIndex = 0;
    // Rebuild formula buffer for new log
    if (isTraceManagerInitialized()) {
        rebuildFormulaDataBuffer();
    }
    if (viewMode === 'SPLIT') rebuildPlotFromPlaybackPrefix();
});
window.addEventListener('logSeek', () => {
    pendingPlaybackPlotRebuild = true;
    lastPlotlyRenderedCount = 0;
    lastPlotlyRenderedStartIndex = 0;
    // Rebuild formula buffer on seek
    if (isTraceManagerInitialized()) {
        rebuildFormulaDataBuffer();
    }
    if (viewMode === 'SPLIT') rebuildPlotFromPlaybackPrefix();
});
