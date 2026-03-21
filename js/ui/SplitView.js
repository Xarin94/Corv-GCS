/**
 * SplitView.js - Split View Mode with Plotly Charts
 * Handles the split screen layout with map and charts
 */

import { STATE, dataBuffer, BUFFER_SIZE, SAMPLE_INTERVAL } from '../core/state.js';
import { TRACE_CONFIG, RAD } from '../core/constants.js';
import { calculateDistance, downsample } from '../core/utils.js';
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

import { initND, resizeND, drawND, setNDVisibilityCheck, startNDLoop } from './NDView.js';
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
const MAX_SPLIT_PATH_POINTS = 5000;
let plotlyInitialized = false;
let lastSampleTime = 0;
let viewMode = 'FULLSCREEN';
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
    if (pendingPlotlyResizeRaf) return;
    pendingPlotlyResizeRaf = requestAnimationFrame(() => {
        pendingPlotlyResizeRaf = 0;
        try { Plotly.Plots.resize('plotly-chart'); } catch (e) {}
    });
}

let ndInitialized = false;

/**
 * Check if 3D view is currently visible (not in chart-only mode)
 */
export function is3DVisible() {
    return viewMode === 'FULLSCREEN';
}

/**
 * Check if 2D map is currently visible (always false in chart mode)
 */
export function is2DMapVisible() {
    return false;
}

/**
 * Check if ND is currently visible (always false in chart mode)
 */
export function isNDVisible() {
    return false;
}

function initNDIfNeeded() {
    if (ndInitialized) return;
    const canvas = document.getElementById('nd-canvas');
    ndInitialized = !!initND(canvas);
    if (ndInitialized) {
        initNDControls();
        setNDVisibilityCheck(isNDVisible);
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
    const speed = 1;

    // Use the source time from the last accepted sample as baseline.
    const base = lastSampleSourceTimeMs || lastTs;
    return base + (dtClamped * speed);
}

function renderPlotlyFromBuffer() {
    if (!plotlyInitialized || viewMode !== 'SPLIT') return;

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
    const gLoad = -az / 9.81;  // Normal load factor (negate NED body Z)

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

// downsample() imported from utils.js

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

    const btn = document.getElementById('btn-mode');
    if (btn) {
        const nextAction = (viewMode === 'FULLSCREEN') ? 'Charts' : 'Fullscreen';
        btn.setAttribute('title', nextAction);
        btn.setAttribute('aria-label', nextAction);
    }

    if (viewMode === 'SPLIT') {
        initPlotlyIfNeeded();
        requestAnimationFrame(() => requestPlotlyResize());
    }

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
function initPlotlyIfNeeded() {
    if (!plotlyInitialized) {
        initPlotly();
    }
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
        recordLivePathPoint();
        splitPathLine.setLatLngs(livePathPoints);
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

    // Keep Plotly responsive when container resizes
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
    const sampleTimeMs = Date.now();

    if (sampleTimeMs - lastSampleTime < SAMPLE_INTERVAL) return;
    lastSampleTime = sampleTimeMs;
    lastSampleSourceTimeMs = sampleTimeMs;
    lastSamplePerfMs = performance.now();

    // Sample for formula-based traces
    const liveEntry = buildLiveEntry();
    sampleFormulaDataPoint(liveEntry, sampleTimeMs);

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
    if (plotlyInitialized) {
        try { Plotly.Plots.resize('plotly-chart'); } catch (e) {}
    }
}

/**
 * Update ND rendering (called from main loop in split view)
 */
export function updateND() {
    if (!isNDVisible()) return;
    try {
        initNDIfNeeded();
        startNDLoop(); // ensure RAF loop is running when ND is visible
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

