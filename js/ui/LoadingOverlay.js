/**
 * LoadingOverlay.js - Loading Screen Management
 *
 * The overlay stays up until the initial terrain is really on screen: HGT
 * files read (or downloaded), chunks built (queue + worker), and — with
 * satellite on — the first texture pass composed. Every stage is read from
 * TerrainManager.getInitialLoadStatus(), so nothing can slip through a
 * "queue is empty" window between two async stages.
 */

// Loading state
let initialLoadDone = false;
let autoLoadAttempted = false;   // loadTopographyAtStart() finished (ok or failed)
let loadingStartTime = Date.now();
let terrainPhaseComplete = false;
let hideScheduled = false;

const INITIAL_MIN_VISIBLE_MS = 600;
// Once the terrain phase is done we still wait for the satellite pass, but
// never beyond this: a slow/flaky network must not hold the app hostage.
const SATELLITE_PHASE_MAX_MS = 45000;
// Absolute ceiling for the whole overlay, whatever the terrain reports.
const OVERLAY_MAX_MS = 120000;
// A stage can look idle for a frame between two async hand-offs (e.g. HGT
// parsed → chunks not yet enqueued). Require the idle state to hold this long.
const SETTLE_MS = 400;
// HGT on disk but no chunk produced for this position (area not covered,
// download failed): stop waiting after this.
const NO_TERRAIN_SETTLE_MS = 3000;
let idleSince = 0;

function setMsg(overlay, text) {
    const t = overlay.querySelector('.loading-msg');
    if (t && text) t.textContent = text;
}

function setBar(overlay, pct) {
    const bar = overlay.querySelector('.loading-bar-fill');
    if (bar) bar.style.width = `${Math.max(0, Math.min(100, pct))}%`;
}

/**
 * Show loading overlay
 * @param {string} msg - Loading message
 */
export function showLoadingOverlay(msg) {
    const el = document.getElementById('loading-overlay');
    if (!el) return;

    el.classList.add('visible');
    setMsg(el, msg);
    setBar(el, 0);

    loadingStartTime = Date.now();
    initialLoadDone = false;
    terrainPhaseComplete = false;
    hideScheduled = false;
    idleSince = 0;
}

/**
 * Update the message only (no state reset) — for events that happen while
 * the overlay is already tracking the load, e.g. the connectivity probe.
 */
export function setLoadingMessage(msg) {
    const el = document.getElementById('loading-overlay');
    if (el && !initialLoadDone) setMsg(el, msg);
}

/**
 * Hide loading overlay
 */
export function hideLoadingOverlay() {
    const el = document.getElementById('loading-overlay');
    if (!el) return;

    el.classList.remove('visible');
    initialLoadDone = true;
}

/**
 * Schedule hiding overlay with delay
 * @param {number} extraDelay - Additional delay in ms
 */
export function scheduleHideLoadingOverlaySoon(extraDelay = 0) {
    if (hideScheduled || initialLoadDone) return;
    hideScheduled = true;

    const el = document.getElementById('loading-overlay');
    if (el) {
        setBar(el, 100);
        setMsg(el, 'SYSTEMS READY');
    }

    const elapsed = Date.now() - loadingStartTime;
    const rem = Math.max(0, INITIAL_MIN_VISIBLE_MS - elapsed);
    const totalDelay = rem + extraDelay + 20;

    if (totalDelay <= 20) {
        hideLoadingOverlay();
    } else {
        setTimeout(hideLoadingOverlay, totalDelay);
    }
}

/**
 * Check progress of initial loading. Called every frame from the render loop.
 * @param {ReturnType<import('../terrain/TerrainManager.js').getInitialLoadStatus>} st
 */
export function checkInitialLoadComplete(st) {
    if (initialLoadDone || hideScheduled) return;

    const overlay = document.getElementById('loading-overlay');
    if (!overlay) {
        initialLoadDone = true;
        return;
    }

    const elapsed = Date.now() - loadingStartTime;
    if (elapsed < INITIAL_MIN_VISIBLE_MS) return;
    if (elapsed > OVERLAY_MAX_MS) {
        console.warn('[loading] overlay timeout — forcing hide', st);
        scheduleHideLoadingOverlaySoon();
        return;
    }

    const satelliteEnabled = window.satelliteEnabled !== false;

    // ---- Phase 1: terrain (HGT → chunks) ----
    if (!terrainPhaseComplete) {
        const terrainIdle = st.hgtPending === 0 && st.chunksPending === 0;

        if (!autoLoadAttempted) {
            setMsg(overlay, st.hgtPending > 0
                ? `READING TERRAIN DATA... ${st.hgtLoaded} HGT`
                : 'LOCATING TERRAIN DATA...');
            setBar(overlay, 5);
            idleSince = 0;
            return;
        }

        // No chunk and nothing arriving: either there is no terrain at all, or
        // the HGT covering this position never came (offline, download failed).
        if (st.chunksActive === 0 && terrainIdle) {
            if (st.hgtLoaded === 0 && st.hgtAvailable === 0) {
                scheduleHideLoadingOverlaySoon();
                return;
            }
            if (!idleSince) idleSince = Date.now();
            if (Date.now() - idleSince >= NO_TERRAIN_SETTLE_MS) {
                console.warn('[loading] no terrain chunks for this position', st);
                scheduleHideLoadingOverlaySoon();
                return;
            }
            setMsg(overlay, 'WAITING FOR TERRAIN DATA...');
            setBar(overlay, 10);
            return;
        }

        if (terrainIdle && st.chunksActive > 0) {
            if (!idleSince) idleSince = Date.now();
            if (Date.now() - idleSince >= SETTLE_MS) {
                terrainPhaseComplete = true;
                idleSince = 0;
                // fall through to phase 2 below
            }
        } else {
            idleSince = 0;
        }

        if (!terrainPhaseComplete) {
            if (st.hgtPending > 0 && st.chunksActive === 0) {
                setMsg(overlay, `READING TERRAIN DATA... ${st.hgtLoaded} HGT`);
                setBar(overlay, 5 + 5 * Math.min(1, st.hgtLoaded / Math.max(1, st.hgtLoaded + st.hgtPending)));
            } else {
                const total = st.chunksActive + st.chunksPending;
                setMsg(overlay, `BUILDING TERRAIN... ${st.chunksActive} chunks (${st.chunksPending} in queue)`);
                setBar(overlay, 10 + 40 * (total > 0 ? st.chunksActive / total : 0));
            }
            // Terrain phase can be stuck on a HGT that will never arrive
            // (download failed, not on disk): bail out after the ceiling above.
            return;
        }
    }

    // ---- Phase 2: satellite textures ----
    if (!satelliteEnabled) {
        scheduleHideLoadingOverlaySoon(300);
        return;
    }

    const satIdle = st.firstTexturePassStarted &&
        st.texturesPending === 0 && st.tilesQueued === 0;

    if (satIdle) {
        if (!idleSince) idleSince = Date.now();
        if (Date.now() - idleSince >= SETTLE_MS) {
            scheduleHideLoadingOverlaySoon(300);
            return;
        }
    } else {
        idleSince = 0;
    }

    if (elapsed > SATELLITE_PHASE_MAX_MS) {
        console.warn('[loading] satellite phase timeout — showing what we have', st);
        scheduleHideLoadingOverlaySoon();
        return;
    }

    if (!st.firstTexturePassStarted) {
        setMsg(overlay, 'PREPARING SATELLITE...');
        setBar(overlay, 50);
    } else if (st.tilesTotal > 0) {
        const done = Math.min(st.tilesLoaded, st.tilesTotal);
        setMsg(overlay, `LOADING SATELLITE... ${done}/${st.tilesTotal} tiles (${st.texturedChunks} chunks)`);
        setBar(overlay, 50 + 45 * (done / st.tilesTotal));
    } else {
        setMsg(overlay, `LOADING SATELLITE... ${st.texturedChunks} chunks`);
        setBar(overlay, 50);
    }
}

/**
 * Mark auto-load as attempted
 */
export function setAutoLoadAttempted() {
    autoLoadAttempted = true;
}

/**
 * Check if initial load is done
 * @returns {boolean}
 */
export function isInitialLoadDone() {
    return initialLoadDone;
}

/**
 * Get loading start time
 * @returns {number}
 */
export function getLoadingStartTime() {
    return loadingStartTime;
}
