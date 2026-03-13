/**
 * TraceManager.js - Manages custom trace configurations for Plotly charts
 * Handles trace UI, formula validation, and trace state management
 */

import { compileExpression, validateExpression } from '../core/ExpressionParser.js';
import { STATE } from '../core/state.js';
import { RingBuffer } from '../core/RingBuffer.js';

// Maximum number of traces
const MAX_TRACES = 4;

// Default colors for traces
const DEFAULT_COLORS = ['#00FF00', '#FFFF00', '#FF3300', '#FF00FF'];

// Default trace configurations
const DEFAULT_TRACES = [
    {
        expression: 'state.as',
        label: 'IAS (m/s)',
        color: '#00FF00',
        yaxis: 'y',
        enabled: true
    },
    {
        expression: 'state.alt',
        label: 'Altitude (m)',
        color: '#FFFF00',
        yaxis: 'y2',
        enabled: true
    }
];

// Curated preset list for the trace preset dropdowns.
// Keep this short and high-signal to avoid overwhelming the UI.
const TRACE_PRESETS = [
    { value: 'state.as', label: 'IAS (m/s)' },
    { value: 'state.gs', label: 'GS (m/s)' },
    { value: 'state.vs', label: 'VS (m/s)' },
    { value: 'state.alt', label: 'Altitude (m)' },
    { value: 'state.roll * RAD', label: 'Roll (deg)' },
    { value: 'state.pitch * RAD', label: 'Pitch (deg)' },
    { value: 'state.yaw * RAD', label: 'Yaw (deg)' },
    { value: '-imu.az / 9.81', label: 'G-Load' },
    { value: 'sqrt(state.vn^2 + state.ve^2 + state.vd^2)', label: 'Total Speed' }
];

// Current active trace configurations
let traceConfigs = [];
let compiledEvaluators = [];
let traceManagerInitialized = false;

// Buffer size (same as original)
const BUFFER_SIZE = 1200;

// Ring buffer based data storage for O(1) push operations
const timestampBuffer = new RingBuffer(BUFFER_SIZE, true);
const traceBuffers = [
    new RingBuffer(BUFFER_SIZE, true),
    new RingBuffer(BUFFER_SIZE, true),
    new RingBuffer(BUFFER_SIZE, true),
    new RingBuffer(BUFFER_SIZE, true)
];

// Proxy object that exposes array-like interface for compatibility with SplitView
// Uses getters to convert ring buffers to arrays on-demand
export const formulaDataBuffer = {
    get timestamps() {
        return timestampBuffer.toArray();
    },
    get traces() {
        return traceBuffers.map(rb => rb.toArray());
    },
    // Efficient length getter (O(1) - no array allocation)
    get length() {
        return timestampBuffer.length;
    },
    // Get last timestamp efficiently (O(1))
    getLastTimestamp() {
        return timestampBuffer.getLast();
    },
    // Direct access to ring buffers for efficient operations
    _timestamps: timestampBuffer,
    _traces: traceBuffers
};

/**
 * Initialize trace manager and UI
 */
export function initTraceManager() {
    if (traceManagerInitialized) return;

    // Initialize with default traces
    traceConfigs = DEFAULT_TRACES.map((t, i) => ({
        ...t,
        index: i
    }));

    // Fill remaining slots with empty configs
    for (let i = traceConfigs.length; i < MAX_TRACES; i++) {
        traceConfigs.push({
            expression: '',
            label: '',
            color: DEFAULT_COLORS[i],
            yaxis: 'y',
            enabled: false,
            index: i
        });
    }

    // Compile initial expressions
    recompileAllExpressions();

    // Populate preset dropdowns with curated options
    populatePresetDropdowns();

    // Setup UI event listeners
    setupTraceUIListeners();

    // Initial UI state
    syncUIFromConfigs();

    traceManagerInitialized = true;
}

function populatePresetDropdowns() {
    const selects = document.querySelectorAll('.trace-preset');
    if (!selects || selects.length === 0) return;

    selects.forEach((select) => {
        const prev = (select.value || '').trim();

        // Clear and rebuild options
        select.innerHTML = '';

        const customOpt = document.createElement('option');
        customOpt.value = '';
        customOpt.textContent = 'Custom...';
        select.appendChild(customOpt);

        for (const preset of TRACE_PRESETS) {
            const opt = document.createElement('option');
            opt.value = preset.value;
            opt.textContent = preset.label;
            select.appendChild(opt);
        }

        // Restore previous selection if it still exists
        if (prev && select.querySelector(`option[value="${cssEscape(prev)}"]`)) {
            select.value = prev;
        } else {
            select.value = '';
        }
    });
}

function cssEscape(value) {
    // Minimal CSS attribute value escape for querySelector.
    return String(value).replace(/\\/g, '\\\\').replace(/"/g, '\\"');
}

/**
 * Setup event listeners for trace configuration UI
 */
function setupTraceUIListeners() {
    // Preset dropdown changes
    document.querySelectorAll('.trace-preset').forEach((select) => {
        select.addEventListener('change', (e) => {
            const row = e.target.closest('.trace-row');
            const index = parseInt(row.dataset.traceIndex);
            const formula = e.target.value;
            const formulaInput = row.querySelector('.trace-formula');

            if (formula) {
                formulaInput.value = formula;
                // Get label from selected option
                const selectedOption = e.target.options[e.target.selectedIndex];
                const label = selectedOption.textContent;
                updateTraceConfig(index, { expression: formula, label });
            }
        });
    });

    // Formula input changes (debounced)
    const formulaDebounce = {};
    document.querySelectorAll('.trace-formula').forEach((input) => {
        const row = input.closest('.trace-row');
        const index = parseInt(row.dataset.traceIndex);

        input.addEventListener('input', (e) => {
            clearTimeout(formulaDebounce[index]);
            formulaDebounce[index] = setTimeout(() => {
                const expression = e.target.value.trim();
                if (expression) {
                    // Reset preset dropdown when custom formula is entered
                    const preset = row.querySelector('.trace-preset');
                    if (preset.value !== expression) {
                        preset.value = '';
                    }
                    updateTraceConfig(index, { expression, label: truncateLabel(expression) });
                }
            }, 500);
        });

        // Validate on blur
        input.addEventListener('blur', (e) => {
            clearTimeout(formulaDebounce[index]);
            const expression = e.target.value.trim();
            if (expression) {
                const result = validateExpression(expression);
                if (!result.valid) {
                    showTraceError(result.error, index);
                    e.target.classList.add('invalid');
                } else {
                    hideTraceError();
                    e.target.classList.remove('invalid');
                    updateTraceConfig(index, { expression, label: truncateLabel(expression) });
                }
            }
        });
    });

    // Color changes
    document.querySelectorAll('.trace-color').forEach((input) => {
        input.addEventListener('change', (e) => {
            const row = e.target.closest('.trace-row');
            const index = parseInt(row.dataset.traceIndex);
            const badge = row.querySelector('.trace-badge');
            badge.style.background = e.target.value;
            updateTraceConfig(index, { color: e.target.value });
        });
    });

    // Axis changes
    document.querySelectorAll('.trace-axis').forEach((select) => {
        select.addEventListener('change', (e) => {
            const row = e.target.closest('.trace-row');
            const index = parseInt(row.dataset.traceIndex);
            updateTraceConfig(index, { yaxis: e.target.value });
        });
    });

    // Remove trace buttons
    document.querySelectorAll('.trace-remove').forEach((btn) => {
        btn.addEventListener('click', (e) => {
            const row = e.target.closest('.trace-row');
            const index = parseInt(row.dataset.traceIndex);
            removeTrace(index);
        });
    });

    // Add trace button
    const addBtn = document.getElementById('add-trace-btn');
    if (addBtn) {
        addBtn.addEventListener('click', addTrace);
    }
}

/**
 * Sync UI state from trace configurations
 */
function syncUIFromConfigs() {
    traceConfigs.forEach((config, index) => {
        const row = document.querySelector(`.trace-row[data-trace-index="${index}"]`);
        if (!row) return;

        // Set visibility
        row.classList.toggle('hidden', !config.enabled);

        // Set formula
        const formulaInput = row.querySelector('.trace-formula');
        if (formulaInput && config.expression) {
            formulaInput.value = config.expression;
        }

        // Set color
        const colorInput = row.querySelector('.trace-color');
        if (colorInput) {
            colorInput.value = config.color;
        }
        const badge = row.querySelector('.trace-badge');
        if (badge) {
            badge.style.background = config.color;
        }

        // Set axis
        const axisSelect = row.querySelector('.trace-axis');
        if (axisSelect) {
            axisSelect.value = config.yaxis;
        }

        // Try to match preset
        const presetSelect = row.querySelector('.trace-preset');
        if (presetSelect && config.expression) {
            const matchingOption = presetSelect.querySelector(`option[value="${config.expression}"]`);
            if (matchingOption) {
                presetSelect.value = config.expression;
            } else {
                presetSelect.value = '';
            }
        }
    });

    updateAddButtonState();
    updateRemoveButtonStates();
}

/**
 * Truncate a formula to use as a label
 */
function truncateLabel(expr) {
    if (!expr) return '';
    if (expr.length <= 20) return expr;
    return expr.substring(0, 17) + '...';
}

/**
 * Update a trace configuration
 * @param {number} index - Trace index
 * @param {Object} updates - Configuration updates
 */
function updateTraceConfig(index, updates) {
    if (index < 0 || index >= traceConfigs.length) return;

    Object.assign(traceConfigs[index], updates);

    // Recompile if expression changed
    if (updates.expression !== undefined) {
        try {
            compiledEvaluators[index] = compileExpression(updates.expression);
            hideTraceError();

            // Mark formula input as valid
            const row = document.querySelector(`.trace-row[data-trace-index="${index}"]`);
            if (row) {
                const formulaInput = row.querySelector('.trace-formula');
                if (formulaInput) {
                    formulaInput.classList.remove('invalid');
                }
            }

            // Rebuild data buffer with new expression
            rebuildFormulaDataBuffer();
        } catch (e) {
            showTraceError(e.message, index);
            compiledEvaluators[index] = null;
        }
    }

    // Trigger re-render
    window.dispatchEvent(new CustomEvent('traceConfigChanged'));
}

/**
 * Add a new trace
 */
function addTrace() {
    const visibleCount = getVisibleTraceCount();
    if (visibleCount >= MAX_TRACES) return;

    // Find first disabled trace
    const disabledIndex = traceConfigs.findIndex(c => !c.enabled);
    if (disabledIndex === -1) return;

    // Enable it with default expression
    traceConfigs[disabledIndex].enabled = true;
    if (!traceConfigs[disabledIndex].expression) {
        // Set a default expression
        const presets = TRACE_PRESETS.map(p => p.value);
        const usedExpressions = traceConfigs.filter(c => c.enabled).map(c => c.expression);
        const availablePreset = presets.find(p => !usedExpressions.includes(p)) || presets[0];
        traceConfigs[disabledIndex].expression = availablePreset;
        traceConfigs[disabledIndex].label = truncateLabel(availablePreset);

        try {
            compiledEvaluators[disabledIndex] = compileExpression(availablePreset);
        } catch (e) {
            compiledEvaluators[disabledIndex] = null;
        }
    }

    // Show the row
    const row = document.querySelector(`.trace-row[data-trace-index="${disabledIndex}"]`);
    if (row) {
        row.classList.remove('hidden');
        const formulaInput = row.querySelector('.trace-formula');
        if (formulaInput) {
            formulaInput.value = traceConfigs[disabledIndex].expression;
        }
    }

    updateAddButtonState();
    updateRemoveButtonStates();

    rebuildFormulaDataBuffer();
    window.dispatchEvent(new CustomEvent('traceConfigChanged'));
}

/**
 * Remove a trace
 * @param {number} index - Trace index to remove
 */
function removeTrace(index) {
    const visibleCount = getVisibleTraceCount();
    if (visibleCount <= 1) return; // Keep at least one trace

    const row = document.querySelector(`.trace-row[data-trace-index="${index}"]`);
    if (!row) return;

    row.classList.add('hidden');

    if (traceConfigs[index]) {
        traceConfigs[index].enabled = false;
    }
    compiledEvaluators[index] = null;
    traceBuffers[index].clear();

    updateAddButtonState();
    updateRemoveButtonStates();

    window.dispatchEvent(new CustomEvent('traceConfigChanged'));
}

/**
 * Get count of visible traces
 */
function getVisibleTraceCount() {
    return traceConfigs.filter(c => c.enabled).length;
}

/**
 * Update add button state based on trace count
 */
function updateAddButtonState() {
    const addBtn = document.getElementById('add-trace-btn');
    if (!addBtn) return;

    const count = getVisibleTraceCount();
    addBtn.disabled = count >= MAX_TRACES;
    addBtn.textContent = count >= MAX_TRACES ? 'MAX TRACES' : '+ ADD TRACE';
}

/**
 * Update remove button states
 */
function updateRemoveButtonStates() {
    const visibleCount = getVisibleTraceCount();
    document.querySelectorAll('.trace-row:not(.hidden) .trace-remove').forEach(btn => {
        btn.disabled = visibleCount <= 1;
    });
}

/**
 * Recompile all expressions
 */
function recompileAllExpressions() {
    compiledEvaluators = traceConfigs.map((config) => {
        if (!config.expression || !config.enabled) return null;
        try {
            return compileExpression(config.expression);
        } catch (e) {
            console.warn(`Failed to compile trace ${config.index} expression:`, e.message);
            return null;
        }
    });
}

/**
 * Show trace error message
 * @param {string} message - Error message
 * @param {number} traceIndex - Index of trace with error
 */
function showTraceError(message, traceIndex) {
    const errorDiv = document.getElementById('trace-error');
    if (errorDiv) {
        errorDiv.textContent = `Trace ${traceIndex + 1}: ${message}`;
        errorDiv.classList.remove('hidden');
    }
}

/**
 * Hide trace error message
 */
function hideTraceError() {
    const errorDiv = document.getElementById('trace-error');
    if (errorDiv) {
        errorDiv.classList.add('hidden');
    }
}

/**
 * Evaluate all active traces for a log entry
 * @param {Object} entry - Log entry data
 * @returns {Array<number>} Array of values, one per trace slot
 */
export function evaluateTraces(entry) {
    return compiledEvaluators.map(evaluator => {
        if (!evaluator) return NaN;
        return evaluator(entry);
    });
}

/**
 * Rebuild formula data buffer from log data
 * Called when expressions change or log is loaded
 */
export function rebuildFormulaDataBuffer() {
    // Clear all ring buffers
    timestampBuffer.clear();
    traceBuffers.forEach(rb => rb.clear());

    if (STATE.mode !== 'PLAYBACK' || !Array.isArray(STATE.logData) || STATE.logData.length === 0) {
        return;
    }

    const len = STATE.logData.length;
    const idx = Math.max(0, Math.min(STATE.logIndex || 0, len - 1));
    const prefixLen = idx + 1;
    const step = Math.max(1, Math.ceil(prefixLen / BUFFER_SIZE));

    for (let i = 0; i < prefixLen; i += step) {
        const entry = STATE.logData[i];
        if (!entry || typeof entry.t !== 'number') continue;

        timestampBuffer.push(entry.t);
        const values = evaluateTraces(entry);
        values.forEach((v, traceIdx) => {
            traceBuffers[traceIdx].push(v);
        });
    }

    // Ensure current point is included
    if (idx >= 0 && idx < len) {
        const entry = STATE.logData[idx];
        const t = entry && entry.t;
        if (typeof t === 'number' && Number.isFinite(t)) {
            const lastT = timestampBuffer.getLast();
            if (lastT !== t) {
                timestampBuffer.push(t);
                const values = evaluateTraces(entry);
                values.forEach((v, traceIdx) => {
                    traceBuffers[traceIdx].push(v);
                });
            }
        }
    }
}

/**
 * Sample a single data point for live mode
 * @param {Object} entry - Current data entry
 * @param {number} timestamp - Current timestamp
 */
export function sampleFormulaDataPoint(entry, timestamp) {
    // O(1) push using ring buffers - no shift needed
    timestampBuffer.push(timestamp);
    const values = evaluateTraces(entry);
    values.forEach((v, idx) => {
        traceBuffers[idx].push(v);
    });
}

/**
 * Get active trace configurations for rendering
 * @returns {Array<Object>} Array of enabled trace configs
 */
export function getActiveTraceConfigs() {
    return traceConfigs
        .filter((c, i) => c.enabled && compiledEvaluators[i])
        .map((c) => ({
            ...c,
            data: formulaDataBuffer.traces[c.index] || []
        }));
}

/**
 * Reset formula data buffer
 */
export function resetFormulaDataBuffer() {
    timestampBuffer.clear();
    traceBuffers.forEach(rb => rb.clear());
}

/**
 * Build live entry from current STATE for expression evaluation
 * @returns {Object} Entry object compatible with expression evaluator
 */
export function buildLiveEntry() {
    return {
        t: Date.now(),
        state: {
            lat: STATE.lat,
            lon: STATE.lon,
            alt: STATE.rawAlt + STATE.offsetAlt,
            vn: 0,
            ve: 0,
            vd: -STATE.vs,
            roll: STATE.roll,
            pitch: STATE.pitch,
            yaw: STATE.yaw
        },
        imu: {
            ax: STATE.ax,
            ay: STATE.ay,
            az: STATE.az,
            gx: 0,
            gy: 0,
            gz: 0
        },
        // Flat fields for backwards compatibility
        lat: STATE.lat,
        lon: STATE.lon,
        alt: STATE.rawAlt + STATE.offsetAlt,
        roll: STATE.roll,
        pitch: STATE.pitch,
        yaw: STATE.yaw,
        ax: STATE.ax,
        ay: STATE.ay,
        az: STATE.az
    };
}

/**
 * Check if trace manager is initialized
 * @returns {boolean}
 */
export function isTraceManagerInitialized() {
    return traceManagerInitialized;
}

export default {
    initTraceManager,
    evaluateTraces,
    getActiveTraceConfigs,
    rebuildFormulaDataBuffer,
    sampleFormulaDataPoint,
    resetFormulaDataBuffer,
    buildLiveEntry,
    isTraceManagerInitialized,
    formulaDataBuffer
};
