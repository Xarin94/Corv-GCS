/**
 * log-replay-manager.js
 *
 * Main-process engine for Log Replay. Indexes .tlog / .bin files and streams
 * their MAVLink messages into the renderer's existing telemetry pipeline via
 * IPC 'mavlink-message' — the same channel live connections use. The UI is
 * thus animated without any awareness that the source is a file.
 *
 * - .tlog: re-uses node-mavlink (via replay-scoped splitter/parser) and
 *   handlePacket() from main-mavlink.js for full decode fidelity.
 * - .bin:  uses a custom minimal parser (log-replay-bin-parser.js) that emits
 *   MAVLink-shaped data objects directly.
 *
 * Clock: a 20 Hz setInterval emits all packets whose tsMs <= wall-time target,
 * batching into ~50 ms slices. A separate 10 Hz UI tick pushes scrubber state.
 */

const { ipcMain, dialog } = require('electron');
const fs = require('fs');
const path = require('path');

const {
    handlePacket,
    emitFakeMavlinkMessage,
    getReplayParserBuilder,
    setReplayActive
} = require('./main-mavlink');
const { indexBinFile } = require('./log-replay-bin-parser');

const DEBUG = !!process.env.LOG_REPLAY_DEBUG;

// TLOG format constants
const TLOG_TS_BYTES = 8;        // uint64 LE microseconds since Unix epoch
const MAV_V1_STX = 0xFE;
const MAV_V2_STX = 0xFD;

// Sticky message whitelist — when the user seeks, re-emit the most recent prior
// message of these IDs so the UI shows correct state (mode, home pos, battery)
// instead of the last frame before the seek.
const STICKY_MSGIDS = new Set([0, 1, 24, 33, 42, 74, 147, 242, 253]);

// Gap compression: gaps between consecutive messages larger than this are
// clamped to 100 ms to avoid multi-minute pauses during replay.
const GAP_COMPRESS_THRESHOLD_MS = 60000;
const GAP_COMPRESS_TO_MS = 100;

// Play clock tick rate
const TICK_INTERVAL_MS = 50;
const UI_TICK_INTERVAL_MS = 100;

// Module state ────────────────────────────────────────────────────────────────

let mainWindow = null;
let replayParserFactory = null;

let replay = null;  // active replay state (or null)

function createEmptyReplay() {
    return {
        filePath: null,
        fileName: null,
        format: null,  // 'tlog' | 'bin'
        rawBuffer: null,
        index: [],
        totalMs: 0,
        cursor: 0,
        playing: false,
        baseWallClock: 0,
        baseLogMs: 0,
        tickTimer: null,
        uiTickTimer: null,
        // .tlog: replay-scoped splitter/parser (stateful, recreate on seek)
        splitter: null,
        parser: null
    };
}

// TLOG indexing ───────────────────────────────────────────────────────────────

/**
 * Walk a .tlog file and build a per-packet index. Each entry:
 *   { tsMs, msgId, offsetBytes, pktLen }
 * where offsetBytes points at the start of the raw MAVLink bytes (after the
 * 8-byte timestamp header), and pktLen is the length of those bytes.
 */
function indexTlogFile(filePath) {
    const buf = fs.readFileSync(filePath);
    const raw = [];
    const firstValid = [];  // for median-based baseline
    let gapWarnings = 0;

    let offset = 0;
    while (offset + TLOG_TS_BYTES + 3 <= buf.length) {
        const tsUs = buf.readBigUInt64LE(offset);
        const stxOff = offset + TLOG_TS_BYTES;
        const stx = buf[stxOff];

        let pktLen = 0;
        let msgId = 0;

        if (stx === MAV_V2_STX) {
            if (stxOff + 10 > buf.length) break;
            const payloadLen = buf[stxOff + 1];
            const incompatFlags = buf[stxOff + 2];
            const signed = (incompatFlags & 0x01) ? 13 : 0;
            pktLen = 12 + payloadLen + 2 + signed;
            msgId = buf[stxOff + 7] | (buf[stxOff + 8] << 8) | (buf[stxOff + 9] << 16);
        } else if (stx === MAV_V1_STX) {
            if (stxOff + 6 > buf.length) break;
            const payloadLen = buf[stxOff + 1];
            pktLen = 8 + payloadLen + 2;
            msgId = buf[stxOff + 5];
        } else {
            // Not a valid STX — resync by advancing one byte
            offset++;
            continue;
        }

        if (stxOff + pktLen > buf.length) break;

        const tsUsNum = Number(tsUs);
        // Filter out garbage timestamps (zero from pre-clock startup frames,
        // "clock not yet synced" values that default to ~2001, or absurd values
        // from corrupted blocks). Range ≈ 2017 .. 2100.
        const TS_MIN = 1_500_000_000_000_000;       // ~2017+
        const TS_MAX = 4_000_000_000_000_000_000;   // ~2096
        const tsValid = Number.isFinite(tsUsNum) && tsUsNum > TS_MIN && tsUsNum < TS_MAX;
        if (tsValid && firstValid.length < 20) firstValid.push(tsUsNum);

        raw.push({
            tsUs: tsValid ? tsUsNum : null,
            msgId,
            offsetBytes: stxOff,
            pktLen
        });

        offset = stxOff + pktLen;
    }

    // Median of the first 20 valid timestamps — robust against one-off outliers
    // (pre-clock-sync startup frames, garbage bytes parsing as valid STX).
    let minUs;
    if (firstValid.length) {
        const sorted = [...firstValid].sort((a, b) => a - b);
        minUs = sorted[Math.floor(sorted.length / 2)];
    } else {
        minUs = 0;
    }

    // Normalize timestamps to start at 0. Packets with invalid timestamps
    // inherit the previous packet's tsMs (kept in-order, just no gap advance).
    // Also clamp absurd forward jumps (> 1h) to a 100 ms micro-gap to keep
    // replay responsive when the source has garbage timestamps mid-stream.
    const MAX_FORWARD_JUMP_MS = 3600_000;  // 1h
    let prevTs = 0;
    const index = raw.map((r, i) => {
        let tsMs;
        if (r.tsUs === null) {
            tsMs = prevTs;
        } else {
            tsMs = Math.floor((r.tsUs - minUs) / 1000);
            if (!Number.isFinite(tsMs) || tsMs < 0) tsMs = prevTs;
            if (i > 0 && tsMs < prevTs) tsMs = prevTs + 1;
            if (i > 0 && tsMs > prevTs + MAX_FORWARD_JUMP_MS) tsMs = prevTs + 100;
        }
        prevTs = tsMs;
        return { tsMs, msgId: r.msgId, offsetBytes: r.offsetBytes, pktLen: r.pktLen };
    });

    // Compress gaps larger than threshold
    if (index.length > 1) {
        let shift = 0;
        for (let i = 1; i < index.length; i++) {
            const originalDt = index[i].tsMs - index[i - 1].tsMs;
            if (Number.isFinite(originalDt) && originalDt > GAP_COMPRESS_THRESHOLD_MS) {
                const delta = originalDt - GAP_COMPRESS_TO_MS;
                shift += delta;
                gapWarnings++;
            }
            index[i].tsMs -= shift;
        }
    }

    const totalMs = index.length ? index[index.length - 1].tsMs : 0;

    if (DEBUG) {
        console.log(`[replay] tlog indexed ${index.length} packets, duration=${totalMs}ms, gaps-compressed=${gapWarnings}`);
    }

    return {
        format: 'tlog',
        rawBuffer: buf,
        index,
        totalMs,
        totalMessages: index.length
    };
}

// Emission ────────────────────────────────────────────────────────────────────

function emitTlogPacketAt(i) {
    if (!replay || replay.format !== 'tlog') return;
    if (!replay.splitter || !replay.parser) return;
    const entry = replay.index[i];
    if (!entry) return;
    const slice = replay.rawBuffer.subarray(entry.offsetBytes, entry.offsetBytes + entry.pktLen);
    try {
        // Write into the splitter which pipes into the parser which triggers handlePacket()
        replay.splitter.write(Buffer.from(slice));
    } catch (e) {
        if (DEBUG) console.warn('[replay] tlog emit error:', e.message);
    }
}

function emitBinMessageAt(i) {
    if (!replay || replay.format !== 'bin') return;
    const entry = replay.index[i];
    if (!entry) return;
    emitFakeMavlinkMessage({
        msgId: entry.msgId,
        data: entry.data,
        sysId: entry.sysId || 1,
        compId: entry.compId || 1
    });
}

function emitAt(i) {
    if (!replay) return;
    if (replay.format === 'tlog') emitTlogPacketAt(i);
    else emitBinMessageAt(i);
}

// Splitter/parser lifecycle (tlog only) ──────────────────────────────────────

function buildTlogPipeline() {
    if (!replayParserFactory) return;
    const { splitter, parser } = replayParserFactory();
    splitter.pipe(parser);
    parser.on('data', (packet) => {
        try { handlePacket(packet); } catch (e) { /* ignore malformed */ }
    });
    replay.splitter = splitter;
    replay.parser = parser;
}

function teardownTlogPipeline() {
    if (replay && replay.splitter) {
        try { replay.splitter.destroy(); } catch {}
        replay.splitter = null;
    }
    if (replay && replay.parser) {
        try { replay.parser.destroy(); } catch {}
        replay.parser = null;
    }
}

// Sticky re-emit on seek ─────────────────────────────────────────────────────

function replayStickyMessagesUpTo(cursorIdx) {
    if (!replay) return;
    // Find the last occurrence of each sticky msgId before cursorIdx
    const lastIdxByMsg = new Map();
    for (let i = 0; i < cursorIdx; i++) {
        const id = replay.index[i].msgId;
        if (STICKY_MSGIDS.has(id)) lastIdxByMsg.set(id, i);
    }
    // Emit them in chronological order
    const entries = Array.from(lastIdxByMsg.values()).sort((a, b) => a - b);
    for (const i of entries) emitAt(i);
    if (DEBUG) console.log(`[replay] sticky replayed ${entries.length} messages up to cursor ${cursorIdx}`);
}

// Tick driver ────────────────────────────────────────────────────────────────

function currentLogMs() {
    if (!replay || !replay.playing) {
        return replay && replay.index[replay.cursor]
            ? replay.index[replay.cursor].tsMs
            : 0;
    }
    const elapsed = Date.now() - replay.baseWallClock;
    return replay.baseLogMs + elapsed;  // speed hard-coded at 1x
}

function tick() {
    if (!replay || !replay.playing) return;
    const targetLogMs = currentLogMs();

    let emitted = 0;
    while (replay.cursor < replay.index.length
        && replay.index[replay.cursor].tsMs <= targetLogMs) {
        emitAt(replay.cursor);
        replay.cursor++;
        emitted++;
        if (emitted > 200) break; // safety cap per tick
    }

    if (replay.cursor >= replay.index.length) {
        // Reached end of file — auto-pause on last frame
        internalPause();
        sendState('PAUSED');
    }
}

function startTimers() {
    stopTimers();
    replay.tickTimer = setInterval(tick, TICK_INTERVAL_MS);
    replay.uiTickTimer = setInterval(sendUiTick, UI_TICK_INTERVAL_MS);
}

function stopTimers() {
    if (!replay) return;
    if (replay.tickTimer) { clearInterval(replay.tickTimer); replay.tickTimer = null; }
    if (replay.uiTickTimer) { clearInterval(replay.uiTickTimer); replay.uiTickTimer = null; }
}

function internalPause() {
    if (!replay) return;
    replay.playing = false;
    stopTimers();
    sendUiTick();
}

function internalPlay() {
    if (!replay) return;
    if (replay.cursor >= replay.index.length) replay.cursor = 0; // rewind at EOF
    const cursorEntry = replay.index[replay.cursor];
    replay.baseLogMs = cursorEntry ? cursorEntry.tsMs : 0;
    replay.baseWallClock = Date.now();
    replay.playing = true;
    startTimers();
    sendUiTick();
}

function internalSeek(targetMs) {
    if (!replay) return;
    const wasPlaying = replay.playing;
    if (wasPlaying) internalPause();

    // Binary search for first entry with tsMs >= targetMs
    let lo = 0, hi = replay.index.length;
    while (lo < hi) {
        const mid = (lo + hi) >>> 1;
        if (replay.index[mid].tsMs < targetMs) lo = mid + 1;
        else hi = mid;
    }
    replay.cursor = Math.min(lo, Math.max(0, replay.index.length - 1));

    // Reset renderer visual state (trails, buffers) and rebuild the tlog parser
    // so stale stream state doesn't confuse the next batch.
    sendResetState();
    if (replay.format === 'tlog') {
        teardownTlogPipeline();
        buildTlogPipeline();
    }

    // Re-emit sticky messages so mode/home/battery are correct immediately
    replayStickyMessagesUpTo(replay.cursor);

    if (DEBUG) console.log(`[replay] seek to ${targetMs}ms → cursor ${replay.cursor}`);

    if (wasPlaying) internalPlay();
    else sendUiTick();
}

// IPC messaging ──────────────────────────────────────────────────────────────

function sendState(state) {
    if (!mainWindow || mainWindow.isDestroyed()) return;
    mainWindow.webContents.send('log-replay-state', {
        state,
        fileName: replay ? replay.fileName : null,
        format: replay ? replay.format : null,
        totalMs: replay ? replay.totalMs : 0,
        totalMessages: replay ? replay.index.length : 0
    });
}

function sendUiTick() {
    if (!mainWindow || mainWindow.isDestroyed()) return;
    if (!replay) return;
    mainWindow.webContents.send('log-replay-tick', {
        currentMs: currentLogMs(),
        totalMs: replay.totalMs,
        playing: replay.playing,
        cursor: replay.cursor
    });
}

function sendResetState() {
    if (!mainWindow || mainWindow.isDestroyed()) return;
    mainWindow.webContents.send('log-replay-reset-state', {});
}

// Load / unload ──────────────────────────────────────────────────────────────

function unload() {
    if (!replay) return;
    internalPause();
    teardownTlogPipeline();
    // Free the raw buffer reference
    replay.rawBuffer = null;
    replay.index = [];
    replay = null;
    setReplayActive(false);
    sendState('UNLOADED');
    if (mainWindow && !mainWindow.isDestroyed()) {
        mainWindow.webContents.send('log-replay-tick', {
            currentMs: 0, totalMs: 0, playing: false, cursor: 0
        });
    }
    if (DEBUG) console.log('[replay] unloaded');
}

function load(filePath) {
    // Drop any prior session
    if (replay) unload();

    const ext = path.extname(filePath).toLowerCase();
    const fileName = path.basename(filePath);
    const t0 = Date.now();
    let parsed;

    if (ext === '.tlog') {
        parsed = indexTlogFile(filePath);
    } else if (ext === '.bin') {
        parsed = indexBinFile(filePath);
        parsed.format = 'bin';
    } else {
        throw new Error(`Unsupported extension: ${ext}`);
    }

    replay = createEmptyReplay();
    replay.filePath = filePath;
    replay.fileName = fileName;
    replay.format = parsed.format;
    replay.rawBuffer = parsed.rawBuffer || null;
    replay.index = parsed.index;
    replay.totalMs = parsed.totalMs;
    replay.cursor = 0;

    setReplayActive(true);

    if (replay.format === 'tlog') buildTlogPipeline();

    const elapsed = Date.now() - t0;
    if (DEBUG) console.log(`[replay] loaded ${fileName} (${parsed.totalMessages} msgs, ${parsed.totalMs}ms) in ${elapsed}ms`);

    // Tell the renderer to clear the existing trail and reset telemetry state
    // so the replay starts with a fresh visualization, not glued onto previous data.
    sendResetState();

    sendState('LOADED');
    sendUiTick();

    return {
        success: true,
        fileName,
        format: replay.format,
        totalMs: replay.totalMs,
        totalMessages: parsed.totalMessages
    };
}

// IPC surface ─────────────────────────────────────────────────────────────────

function registerIpc() {
    ipcMain.handle('log-replay-open-dialog', async () => {
        const res = await dialog.showOpenDialog(mainWindow, {
            title: 'Open ArduPilot log',
            properties: ['openFile'],
            filters: [
                { name: 'ArduPilot logs', extensions: ['tlog', 'bin'] },
                { name: 'All files', extensions: ['*'] }
            ]
        });
        if (res.canceled || !res.filePaths || !res.filePaths.length) {
            return { canceled: true };
        }
        return { canceled: false, filePath: res.filePaths[0] };
    });

    ipcMain.handle('log-replay-load', async (_event, filePath) => {
        try {
            return load(filePath);
        } catch (e) {
            console.error('[replay] load failed:', e.message);
            return { success: false, error: e.message };
        }
    });

    ipcMain.handle('log-replay-play', async () => {
        if (!replay) return { success: false, error: 'no-log-loaded' };
        internalPlay();
        sendState('PLAYING');
        return { success: true };
    });

    ipcMain.handle('log-replay-pause', async () => {
        if (!replay) return { success: false, error: 'no-log-loaded' };
        internalPause();
        sendState('PAUSED');
        return { success: true };
    });

    ipcMain.handle('log-replay-seek', async (_event, targetMs) => {
        if (!replay) return { success: false, error: 'no-log-loaded' };
        internalSeek(Math.max(0, Math.min(replay.totalMs, Number(targetMs) || 0)));
        return { success: true, currentMs: currentLogMs() };
    });

    ipcMain.handle('log-replay-unload', async () => {
        unload();
        return { success: true };
    });
}

// Public API ──────────────────────────────────────────────────────────────────

function initLogReplayHandlers(win) {
    mainWindow = win;
    replayParserFactory = getReplayParserBuilder();
    registerIpc();
    if (DEBUG) console.log('[replay] handlers initialized');
}

function cleanup() {
    try { unload(); } catch {}
}

function isReplaying() {
    return !!replay;
}

module.exports = { initLogReplayHandlers, cleanup, isReplaying };
