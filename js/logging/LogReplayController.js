/**
 * LogReplayController.js — Renderer controller for the Log Replay feature.
 *
 * Responsibilities:
 *   - Wire the sidebar LOG REPLAY section (OPEN FILE / UNLOAD / file info)
 *   - Wire the bottom-right timeline (play/pause, scrubber, current/total time, ×)
 *   - Gate visibility on telemetry connection state — replay is allowed only
 *     while disconnected. Connecting live auto-unloads any active replay.
 *   - Bridge IPC events from log-replay-manager into UI updates and invoke
 *     resetReplayState() + clearTrail() when the main signals a reset (seek).
 */

import { STATE, resetReplayState } from '../core/state.js';
import { latLonToMeters } from '../core/utils.js';
import { setTrailPoints, resetTrail, setTrailFrozen } from '../engine/Scene3D.js';
import { setMapTrail, unfreezeMapTrail } from '../maps/MapEngine.js';

const REPLAY_MODEL = 'foxx.glb';

let els = {};
let replayLoaded = false;
let replayPlaying = false;
let userIsScrubbing = false;
let seekDebounceTimer = null;
let liveConnected = false;  // derived from mavlinkConnectionState events
let previousModelName = null;  // model selected before replay took over

function formatMs(ms) {
    if (!Number.isFinite(ms) || ms < 0) ms = 0;
    const s = Math.floor(ms / 1000);
    const mm = Math.floor(s / 60).toString().padStart(2, '0');
    const ss = (s % 60).toString().padStart(2, '0');
    return `${mm}:${ss}`;
}

function cacheElements() {
    els.section = document.getElementById('gcs-section-logreplay');
    els.openBtn = document.getElementById('replay-open-file');
    els.unloadBtn = document.getElementById('replay-unload');
    els.hint = document.getElementById('replay-hint');
    els.fileInfo = document.getElementById('replay-file-info');
    els.fiName = document.getElementById('replay-fi-name');
    els.fiFormat = document.getElementById('replay-fi-format');
    els.fiDur = document.getElementById('replay-fi-dur');
    els.fiMsgs = document.getElementById('replay-fi-msgs');

    els.timeline = document.getElementById('log-replay-timeline');
    els.playBtn = document.getElementById('lrt-play');
    els.timeCurrent = document.getElementById('lrt-time-current');
    els.timeTotal = document.getElementById('lrt-time-total');
    els.scrubber = document.getElementById('lrt-scrubber');
    els.closeBtn = document.getElementById('lrt-close');
}

function setSectionVisible(visible) {
    if (!els.section) return;
    els.section.hidden = !visible;
}

function setTimelineVisible(visible) {
    if (!els.timeline) return;
    els.timeline.style.display = visible ? '' : 'none';
}

function setFileInfo(info) {
    if (!els.fileInfo) return;
    if (!info) {
        els.fileInfo.style.display = 'none';
        if (els.hint) els.hint.style.display = '';
        return;
    }
    els.fileInfo.style.display = '';
    if (els.hint) els.hint.style.display = 'none';
    if (els.fiName) {
        els.fiName.textContent = info.fileName || '--';
        els.fiName.title = info.fileName || '';
    }
    if (els.fiFormat) els.fiFormat.textContent = (info.format || '--').toUpperCase();
    if (els.fiDur) els.fiDur.textContent = formatMs(info.totalMs || 0);
    if (els.fiMsgs) els.fiMsgs.textContent = String(info.totalMessages || 0);
}

function setPlayIcon(playing) {
    if (!els.playBtn) return;
    // ► = U+25B6 (play), ❚❚ (two heavy vertical bars) for pause
    els.playBtn.innerHTML = playing ? '&#10074;&#10074;' : '&#9654;';
    els.playBtn.title = playing ? 'Pause' : 'Play';
}

function setOpenBtnEnabled(enabled) {
    if (!els.openBtn) return;
    els.openBtn.disabled = !enabled;
    els.openBtn.style.opacity = enabled ? '' : '0.4';
    els.openBtn.style.pointerEvents = enabled ? '' : 'none';
}

/**
 * Switch the 3D model to the one we use for log replay (foxx.glb) if it exists
 * in the model list. Remembers the previous selection for later restoration.
 */
function switchToReplayModel() {
    const select = document.getElementById('model-select');
    if (!select) return;
    const opt = Array.from(select.options).find(o => o.value.toLowerCase() === REPLAY_MODEL);
    if (!opt) return;  // model not present in the folder
    if (select.value === opt.value) return;  // already selected
    previousModelName = select.value;
    select.value = opt.value;
    select.dispatchEvent(new Event('change'));
}

function restorePreviousModel() {
    if (!previousModelName) return;
    const select = document.getElementById('model-select');
    if (!select) { previousModelName = null; return; }
    const exists = Array.from(select.options).some(o => o.value === previousModelName);
    if (exists && select.value !== previousModelName) {
        select.value = previousModelName;
        select.dispatchEvent(new Event('change'));
    }
    previousModelName = null;
}

async function handleOpenFile() {
    if (liveConnected) return;
    if (!window.logReplay) {
        alert('Log replay bridge not available');
        return;
    }
    try {
        const res = await window.logReplay.openDialog();
        if (!res || res.canceled) return;
        const loadRes = await window.logReplay.load(res.filePath);
        if (!loadRes || !loadRes.success) {
            alert('Failed to load log: ' + (loadRes && loadRes.error ? loadRes.error : 'unknown error'));
            return;
        }
        // Draw the entire recorded trajectory as a static red trail so the user
        // can see the whole flight path from the moment the log is loaded.
        drawFullTrack(loadRes.gpsTrack || []);
    } catch (e) {
        alert('Open log failed: ' + e.message);
    }
}

/**
 * Convert the loaded GPS track into both 3D world coordinates and map lat/lon,
 * push them to the trail renderers, and freeze live-trail updates.
 */
function drawFullTrack(gpsTrack) {
    if (!gpsTrack || !gpsTrack.length) {
        setTrailFrozen(false);
        unfreezeMapTrail();
        return;
    }
    // 3D: latLon → meters (x, z); altitude (m) → y
    const points3D = gpsTrack.map(p => {
        const m = latLonToMeters(p.lat, p.lon);
        return { x: m.x, y: p.alt || 0, z: m.z };
    });
    setTrailPoints(points3D);
    setTrailFrozen(true);

    // 2D mini-map polyline
    setMapTrail(gpsTrack);
}

async function handlePlayPause() {
    if (!replayLoaded || !window.logReplay) return;
    if (replayPlaying) await window.logReplay.pause();
    else await window.logReplay.play();
}

async function handleUnload() {
    if (!window.logReplay) return;
    await window.logReplay.unload();
}

function handleScrubberInput(e) {
    userIsScrubbing = true;
    const pct = Number(e.target.value);
    const total = Number(els.scrubber.dataset.totalMs || 0);
    const targetMs = Math.round(total * pct / 1000);
    if (els.timeCurrent) els.timeCurrent.textContent = formatMs(targetMs);

    if (seekDebounceTimer) clearTimeout(seekDebounceTimer);
    seekDebounceTimer = setTimeout(async () => {
        if (window.logReplay) {
            await window.logReplay.seek(targetMs);
        }
        userIsScrubbing = false;
    }, 80);
}

function onTick(tick) {
    if (!tick) return;
    const { currentMs, totalMs, playing } = tick;
    replayPlaying = !!playing;
    setPlayIcon(replayPlaying);

    if (els.scrubber && !userIsScrubbing) {
        els.scrubber.dataset.totalMs = String(totalMs || 0);
        const val = totalMs > 0 ? Math.round((currentMs / totalMs) * 1000) : 0;
        els.scrubber.value = String(Math.max(0, Math.min(1000, val)));
    }
    if (els.timeCurrent) els.timeCurrent.textContent = formatMs(currentMs);
    if (els.timeTotal) els.timeTotal.textContent = formatMs(totalMs);
}

function onState(state) {
    if (!state) return;
    const s = state.state;
    const wasLoaded = replayLoaded;
    if (s === 'LOADED' || s === 'PLAYING' || s === 'PAUSED') {
        replayLoaded = true;
        replayPlaying = (s === 'PLAYING');
        setPlayIcon(replayPlaying);
        setFileInfo({
            fileName: state.fileName,
            format: state.format,
            totalMs: state.totalMs,
            totalMessages: state.totalMessages
        });
        if (els.scrubber) els.scrubber.dataset.totalMs = String(state.totalMs || 0);
        setTimelineVisible(true);
        if (!wasLoaded) {
            // Disable the demo-mode aerobatics loop so it doesn't overwrite
            // the replayed attitude/position every frame. main.js runs demo
            // only while STATE.mode === 'LIVE' && !STATE.connected.
            STATE.mode = 'REPLAY';
            document.body.classList.add('log-replay-active');
            switchToReplayModel();
        }
    } else if (s === 'UNLOADED') {
        replayLoaded = false;
        replayPlaying = false;
        setPlayIcon(false);
        setFileInfo(null);
        setTimelineVisible(false);
        if (els.scrubber) { els.scrubber.value = '0'; els.scrubber.dataset.totalMs = '0'; }
        if (els.timeCurrent) els.timeCurrent.textContent = '00:00';
        if (els.timeTotal) els.timeTotal.textContent = '00:00';
        if (wasLoaded) {
            // Restore normal mode — demo loop and bottom-bar come back on
            STATE.mode = 'LIVE';
            document.body.classList.remove('log-replay-active');
            // Remove the static pre-drawn trail and let live updates resume
            setTrailFrozen(false);
            resetTrail();
            unfreezeMapTrail();
            restorePreviousModel();
        }
    }
}

function onResetState() {
    resetReplayState();
    // Do NOT clear the trail while a replay trajectory is pre-drawn — that
    // full-flight red line must stay visible across play/pause/seek. It is
    // only cleared explicitly on UNLOAD. During live operation we still clear.
    if (!replayLoaded) {
        try { if (window.clearTrail) window.clearTrail(); } catch {}
    }
}

function onConnectionStateChange(state) {
    // state ∈ DISCONNECTED | CONNECTING | CONNECTED | ACTIVE
    const nowLive = (state === 'CONNECTED' || state === 'ACTIVE');
    const wasLive = liveConnected;
    liveConnected = nowLive;

    if (nowLive) {
        // Telemetry connected → drop any active replay and hide UI
        if (replayLoaded && window.logReplay) {
            window.logReplay.unload();
        }
        setSectionVisible(false);
        setTimelineVisible(false);
        setOpenBtnEnabled(false);
    } else {
        // Disconnected or connecting — show section, but disable OPEN while CONNECTING
        setSectionVisible(true);
        setOpenBtnEnabled(state !== 'CONNECTING');
        // Timeline visibility follows replayLoaded (stays hidden if nothing loaded)
    }
}

export function initLogReplay() {
    cacheElements();

    if (!els.section || !els.timeline) {
        console.warn('[LogReplay] missing DOM elements; feature disabled');
        return;
    }

    if (els.openBtn) els.openBtn.addEventListener('click', handleOpenFile);
    if (els.unloadBtn) els.unloadBtn.addEventListener('click', handleUnload);
    if (els.playBtn) els.playBtn.addEventListener('click', handlePlayPause);
    if (els.closeBtn) els.closeBtn.addEventListener('click', handleUnload);
    if (els.scrubber) els.scrubber.addEventListener('input', handleScrubberInput);

    // IPC bridges
    if (window.logReplay) {
        window.logReplay.onTick(onTick);
        window.logReplay.onState(onState);
        window.logReplay.onResetState(onResetState);
    }

    // Listen for live connection state (dispatched by MAVLinkManager)
    window.addEventListener('mavlinkConnectionState', (ev) => {
        onConnectionStateChange(ev.detail && ev.detail.state);
    });

    // First paint: hide section if already live at startup (e.g. SITL autoconnect)
    if (STATE.connected) {
        liveConnected = true;
        setSectionVisible(false);
    } else {
        setSectionVisible(true);
    }
    setTimelineVisible(false);
}
