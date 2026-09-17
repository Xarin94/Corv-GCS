/**
 * MissionHistory.js - Undo/redo for the flight-plan editor
 *
 * Snapshot-based rather than command-based: a route is a handful of segments
 * with a few hundred points at most, so cloning the whole document on every
 * edit costs less than the bookkeeping an inverse-command log would need — and
 * it cannot drift out of sync with the route object, which the editor mutates
 * directly from a dozen places (drag handles, inspector fields, context menu…).
 *
 * Usage: mutate the route as before, then call commitMission('label'). The
 * commit compares against the last committed snapshot and does nothing if the
 * document is unchanged, so it is safe to call from handlers that may be no-ops.
 */

import { getRoute, replaceRoute, cloneRoute } from './RouteModel.js';

const MAX_DEPTH = 100;

let baseline = snapshot();     // the last committed state
const undoStack = [];          // [{ doc, label }] — states to go back to
const redoStack = [];
let suspended = false;         // true while applying a snapshot, to ignore re-entrant commits

function snapshot() {
    return JSON.stringify(cloneRoute(getRoute()));
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

function apply(doc) {
    suspended = true;
    replaceRoute(JSON.parse(doc));
    suspended = false;
}

/**
 * Record the current route as a new history step.
 * @param {string} label - short description shown in the undo tooltip ('Add waypoint', 'Move point'…)
 * @returns {boolean} true if a step was actually recorded
 */
export function commitMission(label = 'Edit') {
    if (suspended) return false;
    const now = snapshot();
    if (now === baseline) return false;

    undoStack.push({ doc: baseline, label });
    if (undoStack.length > MAX_DEPTH) undoStack.shift();
    baseline = now;
    redoStack.length = 0;
    notify();
    return true;
}

/**
 * Forget the history and take the current route as the new starting point.
 * Called when the plan is replaced wholesale (loaded from the library, read from
 * the vehicle) — undoing across such a boundary is never what the operator means.
 */
export function resetMissionHistory() {
    baseline = snapshot();
    undoStack.length = 0;
    redoStack.length = 0;
    notify();
}

/** @returns {string|null} the label of the undone step, or null if there was nothing to undo */
export function undoMission() {
    if (!undoStack.length) return null;
    const entry = undoStack.pop();
    redoStack.push({ doc: baseline, label: entry.label });
    baseline = entry.doc;
    apply(baseline);
    notify();
    return entry.label;
}

/** @returns {string|null} the label of the redone step, or null if there was nothing to redo */
export function redoMission() {
    if (!redoStack.length) return null;
    const entry = redoStack.pop();
    undoStack.push({ doc: baseline, label: entry.label });
    baseline = entry.doc;
    apply(baseline);
    notify();
    return entry.label;
}

export function canUndo() { return undoStack.length > 0; }
export function canRedo() { return redoStack.length > 0; }

/** True when the route differs from the last save/load — drives the "unsaved" marker. */
export function historyDepth() { return undoStack.length; }
