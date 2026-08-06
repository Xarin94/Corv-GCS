/**
 * MissionHistory.js - Undo/redo for the mission editor
 *
 * Snapshot-based rather than command-based: a mission is a few hundred small plain
 * objects at most, so cloning the whole list on every edit costs less than the
 * bookkeeping an inverse-command log would need — and it cannot drift out of sync
 * with STATE.missionItems, which a dozen different call sites mutate directly.
 *
 * Usage: mutate STATE.missionItems as before, then call commitMission('label').
 * The commit compares against the last committed snapshot and does nothing if the
 * mission is unchanged, so it is safe to call from handlers that may be no-ops.
 */

import { STATE } from '../core/state.js';

const MAX_DEPTH = 100;

let baseline = [];        // snapshot of the last committed state
const undoStack = [];     // [{ items, label }] — states to go back to
const redoStack = [];
let suspended = false;    // true while applying a snapshot, to ignore re-entrant commits

function clone(items) {
    return items.map(it => ({ ...it }));
}

function sameMission(a, b) {
    if (a.length !== b.length) return false;
    for (let i = 0; i < a.length; i++) {
        const x = a[i], y = b[i];
        const keys = new Set([...Object.keys(x), ...Object.keys(y)]);
        for (const k of keys) {
            if (x[k] !== y[k]) return false;
        }
    }
    return true;
}

function notify() {
    window.dispatchEvent(new CustomEvent('missionHistoryChanged', {
        detail: {
            canUndo: undoStack.length > 0,
            canRedo: redoStack.length > 0,
            undoLabel: undoStack.length ? undoStack[undoStack.length - 1].label : null,
            redoLabel: redoStack.length ? redoStack[redoStack.length - 1].label : null,
        }
    }));
}

/** Replace the live mission in place — other modules hold a reference to the array. */
function applySnapshot(items) {
    suspended = true;
    STATE.missionItems.length = 0;
    for (const it of clone(items)) STATE.missionItems.push(it);
    STATE.missionItems.forEach((it, i) => { it.seq = i; });
    suspended = false;
}

/**
 * Record the current mission as a new history step.
 * @param {string} label - short description shown in the undo tooltip ('Add WP', 'Delete WP'…)
 * @returns {boolean} true if a step was actually recorded
 */
export function commitMission(label = 'Edit') {
    if (suspended) return false;
    if (sameMission(baseline, STATE.missionItems)) return false;

    undoStack.push({ items: baseline, label });
    if (undoStack.length > MAX_DEPTH) undoStack.shift();
    baseline = clone(STATE.missionItems);
    redoStack.length = 0;
    notify();
    return true;
}

/**
 * Forget the history and take the current mission as the new starting point.
 * Called when the mission is replaced wholesale (loaded from the library, downloaded
 * from the vehicle) — undoing across such a boundary is never what the operator means.
 */
export function resetMissionHistory() {
    baseline = clone(STATE.missionItems);
    undoStack.length = 0;
    redoStack.length = 0;
    notify();
}

/** @returns {string|null} the label of the undone step, or null if there was nothing to undo */
export function undoMission() {
    if (!undoStack.length) return null;
    const entry = undoStack.pop();
    redoStack.push({ items: baseline, label: entry.label });
    baseline = entry.items;
    applySnapshot(baseline);
    notify();
    return entry.label;
}

/** @returns {string|null} the label of the redone step, or null if there was nothing to redo */
export function redoMission() {
    if (!redoStack.length) return null;
    const entry = redoStack.pop();
    undoStack.push({ items: baseline, label: entry.label });
    baseline = entry.items;
    applySnapshot(baseline);
    notify();
    return entry.label;
}

export function canUndo() { return undoStack.length > 0; }
export function canRedo() { return redoStack.length > 0; }

/** True when the mission differs from the last save/load — drives the "unsaved" marker. */
export function historyDepth() { return undoStack.length; }
