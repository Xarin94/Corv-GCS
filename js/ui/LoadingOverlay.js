/**
 * LoadingOverlay.js - Loading Screen Management
 * Handles the loading overlay and progress indication
 */

// Loading state
let initialLoadDone = false;
let autoLoadAttempted = false;
let loadingStartTime = Date.now(); // Initialize with current time
let terrainPhaseComplete = false; // Fase terreno completata

const INITIAL_MIN_VISIBLE_MS = 600;
const POST_COMPLETE_MS = 3000;

/**
 * Show loading overlay
 * @param {string} msg - Loading message
 */
export function showLoadingOverlay(msg) {
    const el = document.getElementById('loading-overlay');
    if (!el) return;
    
    el.classList.add('visible');
    
    const t = el.querySelector('.loading-msg');
    if (t && msg) t.textContent = msg;
    
    loadingStartTime = Date.now();
    initialLoadDone = false;
    terrainPhaseComplete = false; // Reset fase terreno
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
    const el = document.getElementById('loading-overlay');
    if (el) {
        const bar = el.querySelector('.loading-bar-fill');
        if (bar) bar.style.width = '100%';
        
        const txt = el.querySelector('.loading-msg');
        if (txt) txt.textContent = 'SYSTEMS READY';
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
 * Check progress of initial loading
 * @param {Object} activeChunks - Active terrain chunks
 * @param {Array} chunkCreationQueue - Chunk creation queue
 * @param {Array} tileLoadQueue - Tile load queue
 * @param {number} currentTileLoads - Current tile loads in progress
 * @param {number} totalTilesToLoad - Total tiles to load
 * @param {number} tilesLoadedCount - Tiles loaded so far
 */
export function checkInitialLoadComplete(activeChunks, chunkCreationQueue, tileLoadQueue, currentTileLoads, totalTilesToLoad = 0, tilesLoadedCount = 0) {
    if (initialLoadDone) return;
    
    const overlay = document.getElementById('loading-overlay');
    if (!overlay) {
        initialLoadDone = true;
        return;
    }

    const elapsed = Date.now() - loadingStartTime;
    if (elapsed < INITIAL_MIN_VISIBLE_MS) return;

    const totalChunks = Object.keys(activeChunks).length;
    const texturedChunks = Object.values(activeChunks).filter(m => m.userData && m.userData.textureLoaded).length;
    const chunkQueueEmpty = chunkCreationQueue.length === 0;
    const tilesInQueue = tileLoadQueue.length + currentTileLoads;
    const tileQueueEmpty = tilesInQueue === 0;

    // If no chunks yet and no queue, only hide if auto-load was explicitly attempted and failed
    if (totalChunks === 0 && chunkQueueEmpty && autoLoadAttempted) {
        scheduleHideLoadingOverlaySoon();
        return;
    }

    // Don't hide until we have chunks
    if (totalChunks === 0) {
        const txt = overlay.querySelector('.loading-msg');
        if (txt) txt.textContent = 'LOADING TERRAIN DATA...';
        return;
    }

    // Mark that we've started loading (have chunks)
    if (!autoLoadAttempted && totalChunks > 0) {
        autoLoadAttempted = true;
    }

    // Una volta che la coda chunk è vuota per la prima volta, segna fase terreno completa
    if (chunkQueueEmpty && !terrainPhaseComplete) {
        terrainPhaseComplete = true;
    }

    // Caricamento completo quando:
    // 1. Fase terreno completata
    // 2. Coda tile vuota E almeno una texture caricata (o satellite disabilitato)
    const satelliteEnabled = window.satelliteEnabled !== false;
    const texturesReady = !satelliteEnabled || (tileQueueEmpty && texturedChunks > 0);
    
    if (terrainPhaseComplete && texturesReady) {
        scheduleHideLoadingOverlaySoon(300);
        return;
    }

    // Update progress
    const txt = overlay.querySelector('.loading-msg');
    
    if (txt) {
        if (!terrainPhaseComplete) {
            // Fase 1: costruzione terreno
            txt.textContent = `BUILDING TERRAIN... ${totalChunks} chunks (${chunkCreationQueue.length} in queue)`;
        } else if (satelliteEnabled) {
            // Fase 2: caricamento satellite
            if (totalTilesToLoad > 0) {
                const pct = Math.round((tilesLoadedCount / totalTilesToLoad) * 100);
                txt.textContent = `LOADING SATELLITE... ${tilesLoadedCount}/${totalTilesToLoad} tiles (${texturedChunks} chunks)`;
            } else if (tilesInQueue > 0) {
                txt.textContent = `LOADING SATELLITE... (${tilesInQueue} tiles pending)`;
            } else if (texturedChunks === 0) {
                txt.textContent = `PREPARING SATELLITE...`;
            } else {
                txt.textContent = `LOADING SATELLITE... ${texturedChunks} chunks`;
            }
        } else {
            txt.textContent = `FINALIZING...`;
        }
    }

    const bar = overlay.querySelector('.loading-bar-fill');
    if (bar) {
        let pct = 0;
        if (!terrainPhaseComplete) {
            // Progresso fase terreno (0-50%)
            const chunkProgress = totalChunks / Math.max(1, totalChunks + chunkCreationQueue.length);
            pct = chunkProgress * 50;
        } else {
            // Progresso fase satellite (50-100%)
            pct = 50;
            if (satelliteEnabled && totalTilesToLoad > 0) {
                const tileProgress = tilesLoadedCount / totalTilesToLoad;
                pct = 50 + tileProgress * 50;
            } else if (texturedChunks > 0 || !satelliteEnabled) {
                pct = 100;
            }
        }
        bar.style.width = `${Math.min(100, pct)}%`;
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
