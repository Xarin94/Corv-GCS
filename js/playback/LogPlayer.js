/**
 * LogPlayer.js - Log Playback Mode
 * Handles loading and playing back recorded flight logs
 */

import { STATE } from '../core/state.js';
import { ORIGIN, RAD } from '../core/constants.js';

let playbackSpeed = 1.0;
let playbackAnchorPerfMs = 0;
let playbackAnchorLogMs = 0;
let logTimes = [];

function getLogTimeMs(entry) {
    if (!entry) return 0;
    const t = entry.t;
    return (typeof t === 'number' && Number.isFinite(t)) ? t : 0;
}

function upperBound(arr, x) {
    // First index i such that arr[i] > x
    let lo = 0;
    let hi = arr.length;
    while (lo < hi) {
        const mid = (lo + hi) >> 1;
        if (arr[mid] <= x) lo = mid + 1;
        else hi = mid;
    }
    return lo;
}

function updatePlaybackAnchors(nowPerfMs) {
    playbackAnchorPerfMs = nowPerfMs;
    playbackAnchorLogMs = logTimes[STATE.logIndex] ?? getLogTimeMs(STATE.logData[STATE.logIndex]);
}

export function setPlaybackSpeed(speed) {
    const s = Number(speed);
    if (!Number.isFinite(s) || s <= 0) return;

    const now = performance.now();
    if (STATE.mode === 'PLAYBACK' && STATE.isPlaying && logTimes.length > 0) {
        // Preserve continuity: compute current virtual log time, then re-anchor.
        const currentVirtual = playbackAnchorLogMs + (now - playbackAnchorPerfMs) * playbackSpeed;
        playbackAnchorPerfMs = now;
        playbackAnchorLogMs = currentVirtual;
    }

    playbackSpeed = s;
}

export function getPlaybackSpeed() {
    return playbackSpeed;
}

/**
 * Parse log file content
 * @param {string} text - Log file text content
 */
export function parseLog(text) {
    STATE.logData = [];
    
    try {
        const j = JSON.parse(text);
        if (Array.isArray(j)) STATE.logData = j;
    } catch (e) {
        // Try line-by-line parsing
        text.split('\n').forEach(l => {
            if (l.trim()) {
                try {
                    STATE.logData.push(JSON.parse(l.trim().replace(/,$/, '')));
                } catch (err) {}
            }
        });
    }
    
    if (STATE.logData.length) {
        // Cache timestamps for real-time playback
        logTimes = STATE.logData.map(getLogTimeMs);

        // Switch to PLAYBACK mode
        STATE.mode = 'PLAYBACK';
        STATE.logIndex = 0;
        STATE.isPlaying = false;

        playbackSpeed = 1.0;
        updatePlaybackAnchors(performance.now());

        // UI Updates
        document.getElementById('storyline-panel').classList.add('visible');
        document.getElementById('scrubber').max = STATE.logData.length - 1;
        document.getElementById('scrubber').value = 0;
        const speedSel = document.getElementById('replay-speed');
        if (speedSel) speedSel.value = '1';
        document.getElementById('btn-load').classList.add('active');
        document.getElementById('btn-link').classList.remove('active');
        document.getElementById('btn-link').innerText = "CONNECT LINK";

        updateFromLog(0);

        // Notify listeners that a log has been loaded (useful for rebuilding paths).
        window.dispatchEvent(new CustomEvent('logLoaded', { detail: { length: STATE.logData.length } }));

        // Set Reload pos
        const s = STATE.logData[0].state || STATE.logData[0];
        STATE.lastReloadPos.lat = s.lat || ORIGIN.lat;
        STATE.lastReloadPos.lon = s.lon || ORIGIN.lon;
    }
}

/**
 * Update state from log entry at index
 * @param {number} idx - Log entry index
 */
export function updateFromLog(idx) {
    const r = STATE.logData[idx];
    if (!r) return;
    
    const s = r.state || r;
    const i = r.imu || r;
    
    STATE.roll = s.roll || 0;
    STATE.pitch = s.pitch || 0;
    STATE.yaw = s.yaw || 0;
    STATE.lat = s.lat || ORIGIN.lat;
    STATE.lon = s.lon || ORIGIN.lon;
    STATE.rawAlt = s.alt || 0;

    const vn = s.vn || 0;
    const ve = s.ve || 0;
    const vd = s.vd || 0;
    
    STATE.as = Math.sqrt(vn ** 2 + ve ** 2 + vd ** 2);
    STATE.gs = Math.sqrt(vn ** 2 + ve ** 2);
    STATE.vs = -vd;
    STATE.gamma = Math.atan2(-vd, STATE.gs);
    STATE.track = Math.atan2(ve, vn);
    STATE.aoa = STATE.pitch - STATE.gamma;
    
    let drift = STATE.track - STATE.yaw;
    while (drift > Math.PI) drift -= Math.PI * 2;
    while (drift < -Math.PI) drift += Math.PI * 2;
    STATE.ssa = drift;

    STATE.ax = i.ax || 0;
    STATE.ay = i.ay || 0;
    STATE.az = i.az || 0;

    // Update time label
    const t = (r.t - STATE.logData[0].t) / 1000;
    document.getElementById('time-lbl').innerText = t.toFixed(1) + "s";
    
    // Dispatch update event
    window.dispatchEvent(new CustomEvent('logUpdate'));
}

/**
 * Step playback forward
 */
export function stepForward() {
    if (STATE.mode !== 'PLAYBACK' || STATE.logData.length === 0) return;
    
    STATE.logIndex++;
    if (STATE.logIndex >= STATE.logData.length) {
        STATE.logIndex = 0;
    }
    
    document.getElementById('scrubber').value = STATE.logIndex;
    updateFromLog(STATE.logIndex);
}

/**
 * Advance playback in real time based on timestamps.
 * Call this from the render loop when STATE.isPlaying is true.
 * @param {number} nowPerfMs performance.now()
 */
export function tickPlayback(nowPerfMs) {
    if (STATE.mode !== 'PLAYBACK' || !STATE.isPlaying) return;
    if (!STATE.logData.length) return;
    if (!logTimes.length) logTimes = STATE.logData.map(getLogTimeMs);

    // If timestamps are missing or flat, fall back to old behavior.
    const lastT = logTimes[logTimes.length - 1];
    const firstT = logTimes[0];
    if (!Number.isFinite(firstT) || !Number.isFinite(lastT) || lastT <= firstT) {
        stepForward();
        return;
    }

    const targetLogMs = playbackAnchorLogMs + (nowPerfMs - playbackAnchorPerfMs) * playbackSpeed;

    // Loop behavior (matches prior behavior of wrapping to 0)
    if (targetLogMs >= lastT) {
        STATE.logIndex = 0;
        document.getElementById('scrubber').value = STATE.logIndex;
        updateFromLog(STATE.logIndex);
        updatePlaybackAnchors(nowPerfMs);
        return;
    }

    const ub = upperBound(logTimes, targetLogMs);
    const idx = Math.max(0, ub - 1);
    if (idx !== STATE.logIndex) {
        STATE.logIndex = idx;
        document.getElementById('scrubber').value = STATE.logIndex;
        updateFromLog(STATE.logIndex);
    }
}

/**
 * Toggle play/pause
 */
export function togglePlay() {
    STATE.isPlaying = !STATE.isPlaying;
    document.getElementById('btn-play').innerText = STATE.isPlaying ? "II" : "▶";

    if (STATE.isPlaying) {
        updatePlaybackAnchors(performance.now());
    }
}

/**
 * Seek to specific index
 * @param {number} idx - Target index
 */
export function seekTo(idx) {
    STATE.isPlaying = false;
    document.getElementById('btn-play').innerText = "▶";
    STATE.logIndex = idx;
    updateFromLog(STATE.logIndex);

    updatePlaybackAnchors(performance.now());

    // Notify listeners that the user scrubbed/seeked.
    window.dispatchEvent(new CustomEvent('logSeek', { detail: { index: idx } }));
}

/**
 * Initialize playback controls
 */
export function initPlaybackControls() {
    const btnPlay = document.getElementById('btn-play');
    const scrubber = document.getElementById('scrubber');
    const logInput = document.getElementById('log-input');
    const speedSel = document.getElementById('replay-speed');
    
    if (btnPlay) {
        btnPlay.onclick = togglePlay;
    }
    
    if (scrubber) {
        scrubber.oninput = (e) => {
            seekTo(parseInt(e.target.value));
        };
    }
    
    if (logInput) {
        logInput.onchange = (e) => {
            if (e.target.files.length) {
                const r = new FileReader();
                r.onload = ev => parseLog(ev.target.result);
                r.readAsText(e.target.files[0]);
            }
        };
    }

    if (speedSel) {
        speedSel.onchange = (e) => {
            setPlaybackSpeed(parseFloat(e.target.value));
        };
        // Ensure default matches UI
        setPlaybackSpeed(parseFloat(speedSel.value || '1'));
    }
}
