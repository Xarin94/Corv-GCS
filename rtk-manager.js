/**
 * rtk-manager.js - RTK / GPS Inject Manager
 * Forwards RTCM3 correction data to the drone via MAVLink GPS_RTCM_DATA
 * messages (ID 233), Mission Planner style. Two correction sources:
 *  - Serial: a local GPS base station (e.g. u-blox F9P)
 *  - NTRIP:  an NTRIP caster over TCP/TLS (with sourcetable browsing,
 *            GGA upload for VRS networks, and auto-reconnect)
 * Only one source can be active at a time.
 */

const { ipcMain } = require('electron');
const net = require('net');
const tls = require('tls');
const { sendRawBuffer, getNextSequenceNumber } = require('./main-mavlink');

// Lazy-load serialport
let SerialPort = null;
function ensureSerialLoaded() {
    if (SerialPort) return;
    SerialPort = require('serialport').SerialPort;
}

// State
let rtkPort = null;
let mainWindow = null;

function freshStats() {
    return {
        connected: false,
        source: '',          // 'serial' | 'ntrip'
        portPath: '',
        baudRate: 0,
        bytesReceived: 0,
        rtcmMsgCount: 0,
        rtcmMsgPerSec: 0,
        rtcmLastTypes: [],   // last seen RTCM message types
        lastUpdateTime: 0
    };
}

let rtkStats = freshStats();

// RTCM3 message counter for rate calculation
let rtcmCountWindow = [];

/**
 * Parse RTCM3 frames from a raw byte stream.
 * RTCM3 frame: 0xD3 | 6-bit reserved (0) + 10-bit length | payload | 24-bit CRC
 */
class RTCM3Parser {
    constructor() {
        this.buffer = Buffer.alloc(0);
    }

    /**
     * Feed raw data and return array of { type, length, raw } for each complete RTCM3 message
     */
    parse(data) {
        this.buffer = Buffer.concat([this.buffer, data]);
        const messages = [];

        while (this.buffer.length >= 6) { // minimum: 3 header + 0 payload + 3 CRC
            // Find sync byte 0xD3
            const syncIdx = this.buffer.indexOf(0xD3);
            if (syncIdx < 0) {
                this.buffer = Buffer.alloc(0);
                break;
            }
            if (syncIdx > 0) {
                this.buffer = this.buffer.subarray(syncIdx);
            }
            if (this.buffer.length < 3) break;

            // Extract 10-bit length from bytes 1-2
            const len = ((this.buffer[1] & 0x03) << 8) | this.buffer[2];
            const frameLen = 3 + len + 3; // header + payload + CRC

            if (this.buffer.length < frameLen) break; // wait for more data

            const frame = this.buffer.subarray(0, frameLen);
            this.buffer = this.buffer.subarray(frameLen);

            // Extract message type (12 bits from first 2 bytes of payload)
            let msgType = 0;
            if (len >= 2) {
                msgType = (frame[3] << 4) | ((frame[4] >> 4) & 0x0F);
            }

            messages.push({
                type: msgType,
                length: len,
                raw: Buffer.from(frame) // copy
            });
        }

        // Prevent buffer from growing unbounded
        if (this.buffer.length > 8192) {
            this.buffer = this.buffer.subarray(this.buffer.length - 4096);
        }

        return messages;
    }
}

const rtcm3Parser = new RTCM3Parser();

// RTCM3 message type descriptions
const RTCM_TYPE_NAMES = {
    1001: 'GPS L1 Obs',
    1002: 'GPS L1 Ext Obs',
    1003: 'GPS L1/L2 Obs',
    1004: 'GPS L1/L2 Ext Obs',
    1005: 'Base Position',
    1006: 'Base Position + Height',
    1007: 'Antenna Descriptor',
    1008: 'Antenna Serial',
    1009: 'GLONASS L1 Obs',
    1010: 'GLONASS L1 Ext Obs',
    1011: 'GLONASS L1/L2 Obs',
    1012: 'GLONASS L1/L2 Ext Obs',
    1033: 'Receiver/Antenna Info',
    1074: 'GPS MSM4',
    1077: 'GPS MSM7',
    1084: 'GLONASS MSM4',
    1087: 'GLONASS MSM7',
    1094: 'Galileo MSM4',
    1097: 'Galileo MSM7',
    1124: 'BeiDou MSM4',
    1127: 'BeiDou MSM7',
    1230: 'GLONASS Code-Phase Bias',
    4072: 'u-blox Proprietary'
};

/**
 * Initialize RTK IPC handlers
 */
function initRTKHandlers(win) {
    mainWindow = win;

    // List serial ports
    ipcMain.handle('rtk-list-ports', async () => {
        try {
            ensureSerialLoaded();
            const ports = await SerialPort.list();
            return ports.map(p => ({
                path: p.path,
                manufacturer: p.manufacturer || '',
                friendlyName: p.friendlyName || '',
                vendorId: p.vendorId || '',
                productId: p.productId || '',
                serialNumber: p.serialNumber || ''
            }));
        } catch (e) {
            console.error('[rtk] Failed to list ports:', e.message);
            return [];
        }
    });

    // Connect to GPS base station
    ipcMain.handle('rtk-connect', async (event, portPath, baudRate) => {
        await disconnectRTK();
        stopNtrip(true); // single correction source at a time
        ensureSerialLoaded();

        try {
            rtkPort = new SerialPort({
                path: portPath,
                baudRate: baudRate || 115200,
                autoOpen: false
            });

            await new Promise((resolve, reject) => {
                rtkPort.open((err) => {
                    if (err) reject(err);
                    else resolve();
                });
            });

            rtkStats = {
                ...freshStats(),
                connected: true,
                source: 'serial',
                portPath,
                baudRate: baudRate || 115200,
                lastUpdateTime: Date.now()
            };
            rtcmCountWindow = [];

            rtkPort.on('data', (data) => {
                handleRTKData(data);
            });

            rtkPort.on('error', (err) => {
                console.error('[rtk] Serial error:', err.message);
                sendRTKStatus();
            });

            rtkPort.on('close', () => {
                console.log('[rtk] Port closed');
                rtkStats.connected = false;
                sendRTKStatus();
            });

            console.log(`[rtk] Connected to ${portPath} at ${baudRate || 115200} baud`);
            sendRTKStatus();
            return { success: true };
        } catch (e) {
            console.error('[rtk] Connect failed:', e.message);
            throw e;
        }
    });

    // Disconnect
    ipcMain.handle('rtk-disconnect', async () => {
        await disconnectRTK();
        return { success: true };
    });

    // Get current stats
    ipcMain.handle('rtk-get-stats', () => {
        return { ...rtkStats };
    });

    // Get RTCM type names
    ipcMain.handle('rtk-get-type-names', () => {
        return RTCM_TYPE_NAMES;
    });

    // ── NTRIP handlers ────────────────────────────────────────────────

    // Connect to an NTRIP caster mountpoint
    ipcMain.handle('ntrip-connect', async (event, cfg) => {
        if (!cfg || !cfg.host || !cfg.mountpoint) {
            throw new Error('NTRIP host and mountpoint are required');
        }
        await disconnectRTK(); // single correction source at a time
        stopNtrip(true);

        ntripConfig = {
            host: String(cfg.host).trim(),
            port: parseInt(cfg.port) || 2101,
            mountpoint: String(cfg.mountpoint).trim().replace(/^\//, ''),
            username: cfg.username || '',
            password: cfg.password || '',
            tls: !!cfg.tls,
            ggaMode: cfg.ggaMode || 'off',   // 'off' | 'vehicle' | 'manual'
            lat: Number(cfg.lat),
            lon: Number(cfg.lon),
            alt: Number(cfg.alt) || 0,
            ggaInterval: Math.max(1, parseInt(cfg.ggaInterval) || 10)
        };
        ntripUserDisconnect = false;
        ntripReconnectCount = 0;
        ntripGgaSentCount = 0;
        return ntripOpen();
    });

    // Disconnect from NTRIP caster
    ipcMain.handle('ntrip-disconnect', async () => {
        stopNtrip(true);
        return { success: true };
    });

    // Fetch the caster sourcetable (list of available mountpoints)
    ipcMain.handle('ntrip-get-sourcetable', async (event, cfg) => {
        return fetchSourcetable(cfg || {});
    });

    // Vehicle position feed from renderer (used for GGA upload in 'vehicle' mode)
    ipcMain.on('rtk-feed-position', (event, pos) => {
        if (pos && isFinite(pos.lat) && isFinite(pos.lon)) {
            lastVehiclePos = { lat: pos.lat, lon: pos.lon, alt: Number(pos.alt) || 0 };
        }
    });
}

/**
 * Handle raw data from GPS base station serial port
 */
function handleRTKData(data) {
    rtkStats.bytesReceived += data.length;

    // Parse RTCM3 frames
    const messages = rtcm3Parser.parse(data);

    for (const msg of messages) {
        rtkStats.rtcmMsgCount++;

        // Track message rate
        const now = Date.now();
        rtcmCountWindow.push(now);
        // Keep only last 5 seconds
        rtcmCountWindow = rtcmCountWindow.filter(t => now - t < 5000);
        rtkStats.rtcmMsgPerSec = Math.round(rtcmCountWindow.length / 5);

        // Track last message types (keep unique, max 10)
        const typeName = RTCM_TYPE_NAMES[msg.type] || `Type ${msg.type}`;
        const typeEntry = { id: msg.type, name: typeName, time: now };
        rtkStats.rtcmLastTypes = rtkStats.rtcmLastTypes.filter(t => t.id !== msg.type);
        rtkStats.rtcmLastTypes.unshift(typeEntry);
        if (rtkStats.rtcmLastTypes.length > 12) rtkStats.rtcmLastTypes.pop();

        // Forward RTCM3 data to drone via MAVLink GPS_RTCM_DATA
        forwardRTCMtoDrone(msg.raw);
    }

    // Send status update to renderer at max 4 Hz
    const now = Date.now();
    if (now - rtkStats.lastUpdateTime > 250) {
        rtkStats.lastUpdateTime = now;
        sendRTKStatus();
    }
}

/**
 * Forward RTCM3 message to drone via MAVLink GPS_RTCM_DATA (msg ID 233).
 * Builds raw MAVLink v2 packets directly and sends over the active connection.
 * Max payload per message: 180 bytes; fragments if larger.
 */
function forwardRTCMtoDrone(rawFrame) {
    const MAX_PAYLOAD = 180;
    const totalLen = rawFrame.length;

    if (totalLen <= MAX_PAYLOAD) {
        sendGpsRtcmDataPacket(0, rawFrame);
    } else {
        const fragments = Math.ceil(totalLen / MAX_PAYLOAD);
        const seqId = rtkStats.rtcmMsgCount & 0x1F;

        for (let i = 0; i < fragments; i++) {
            const offset = i * MAX_PAYLOAD;
            const chunk = rawFrame.subarray(offset, Math.min(offset + MAX_PAYLOAD, totalLen));
            const flags = 1 | ((i & 0x03) << 1) | ((seqId & 0x1F) << 3);
            sendGpsRtcmDataPacket(flags, chunk);
        }
    }
}

/**
 * Build and send a raw MAVLink v2 GPS_RTCM_DATA packet (msg ID 233)
 * Packet layout: flags(1) + len(1) + data(180) = 182 bytes payload
 */
function sendGpsRtcmDataPacket(flags, rtcmData) {
    // MAVLink v2 header: 0xFD, payload_len, incompat_flags, compat_flags, seq, sysid, compid, msgid(3 bytes)
    const PAYLOAD_LEN = 182; // GPS_RTCM_DATA fixed payload: 1+1+180
    const MSG_ID = 233;

    const packet = Buffer.alloc(12 + PAYLOAD_LEN + 2); // header(10) + payload + CRC(2)

    // Header
    packet[0] = 0xFD; // MAVLink v2 magic
    packet[1] = PAYLOAD_LEN;
    packet[2] = 0; // incompat flags
    packet[3] = 0; // compat flags
    packet[4] = getNextSequenceNumber();
    packet[5] = 255; // GCS system ID
    packet[6] = 190; // GCS component ID (MAV_COMP_ID_MISSIONPLANNER)
    packet[7] = MSG_ID & 0xFF;
    packet[8] = (MSG_ID >> 8) & 0xFF;
    packet[9] = (MSG_ID >> 16) & 0xFF;

    // Payload: flags(1) + len(1) + data(180, zero-padded)
    packet[10] = flags;
    packet[11] = rtcmData.length;
    rtcmData.copy(packet, 12, 0, Math.min(rtcmData.length, 180));

    // CRC (MAVLink uses X.25 CRC with CRC_EXTRA seed)
    const CRC_EXTRA_GPS_RTCM_DATA = 35; // CRC extra for GPS_RTCM_DATA
    let crc = 0xFFFF;
    // CRC over: payload_len, incompat, compat, seq, sysid, compid, msgid(3), payload
    for (let i = 1; i < 10 + PAYLOAD_LEN; i++) {
        crc = crcAccumulate(packet[i], crc);
    }
    crc = crcAccumulate(CRC_EXTRA_GPS_RTCM_DATA, crc);

    packet[10 + PAYLOAD_LEN] = crc & 0xFF;
    packet[10 + PAYLOAD_LEN + 1] = (crc >> 8) & 0xFF;

    sendRawBuffer(packet);
}

/**
 * MAVLink X.25 CRC accumulate
 */
function crcAccumulate(byte, crc) {
    let tmp = byte ^ (crc & 0xFF);
    tmp ^= (tmp << 4) & 0xFF;
    crc = ((crc >> 8) & 0xFF) ^ (tmp << 8) ^ (tmp << 3) ^ ((tmp >> 4) & 0xF);
    return crc & 0xFFFF;
}

// ─── NTRIP Client ─────────────────────────────────────────────────────
// Mission Planner-style NTRIP client: HTTP GET with Basic auth against a
// caster, accepts both "ICY 200 OK" (v1) and "HTTP/1.x 200" (v2, optionally
// chunked) responses, uploads GGA for VRS networks, auto-reconnects.

let ntripSocket = null;
let ntripConfig = null;          // config of the active/last session
let ntripUserDisconnect = false; // true → no auto-reconnect
let ntripReconnectTimer = null;
let ntripGgaTimer = null;
let ntripHeaderParsed = false;
let ntripHeaderBuf = Buffer.alloc(0);
let ntripChunked = false;
let ntripChunkBuf = Buffer.alloc(0);
let ntripChunkRemain = 0;        // bytes of chunk payload still expected
let ntripReconnectCount = 0;
let ntripGgaSentCount = 0;
let lastVehiclePos = null;       // { lat, lon, alt } fed by the renderer

/**
 * Open the NTRIP stream. Resolves once the caster accepts the request,
 * rejects on connection/auth/mountpoint errors.
 */
function ntripOpen() {
    return new Promise((resolve, reject) => {
        const cfg = ntripConfig;
        if (!cfg) return reject(new Error('No NTRIP configuration'));
        let settled = false;
        const ok = () => { if (!settled) { settled = true; resolve({ success: true }); } };
        const fail = (err) => { if (!settled) { settled = true; reject(err); } };

        ntripHeaderParsed = false;
        ntripHeaderBuf = Buffer.alloc(0);
        ntripChunked = false;
        ntripChunkBuf = Buffer.alloc(0);
        ntripChunkRemain = 0;

        const onConnect = () => {
            let req = `GET /${cfg.mountpoint} HTTP/1.1\r\n` +
                      `Host: ${cfg.host}:${cfg.port}\r\n` +
                      `User-Agent: NTRIP CORV-GCS\r\n` +
                      `Accept: */*\r\n`;
            if (cfg.username || cfg.password) {
                const auth = Buffer.from(`${cfg.username}:${cfg.password}`).toString('base64');
                req += `Authorization: Basic ${auth}\r\n`;
            }
            req += 'Connection: close\r\n\r\n';
            sock.write(req);
        };

        const sock = cfg.tls
            ? tls.connect({ host: cfg.host, port: cfg.port, rejectUnauthorized: false }, onConnect)
            : net.connect({ host: cfg.host, port: cfg.port }, onConnect);
        ntripSocket = sock;

        // Casters stream continuously — no data for 20 s means a dead link
        sock.setTimeout(20000);
        sock.on('timeout', () => {
            console.error('[ntrip] Socket timeout');
            sock.destroy();
        });

        sock.on('data', (data) => {
            if (!ntripHeaderParsed) {
                ntripHeaderBuf = Buffer.concat([ntripHeaderBuf, data]);
                const idx = ntripHeaderBuf.indexOf('\r\n\r\n');
                if (idx < 0) {
                    if (ntripHeaderBuf.length > 16384) sock.destroy();
                    return;
                }
                const header = ntripHeaderBuf.subarray(0, idx).toString('ascii');
                const body = ntripHeaderBuf.subarray(idx + 4);
                ntripHeaderBuf = Buffer.alloc(0);
                const statusLine = (header.split('\r\n')[0] || '').trim();

                if (/^SOURCETABLE/i.test(statusLine) || /Content-Type:\s*gnss\/sourcetable/i.test(header)) {
                    ntripUserDisconnect = true; // config error — do not auto-retry
                    fail(new Error(`Mountpoint "${cfg.mountpoint}" not found (caster returned sourcetable)`));
                    sock.destroy();
                    return;
                }
                if (!/^(ICY 200|HTTP\/\d\.\d 200)/i.test(statusLine)) {
                    ntripUserDisconnect = true; // auth/config error — do not auto-retry
                    fail(new Error(/401|Unauthorized/i.test(statusLine)
                        ? 'NTRIP authentication failed (check username/password)'
                        : `NTRIP caster error: ${statusLine}`));
                    sock.destroy();
                    return;
                }

                ntripChunked = /transfer-encoding:\s*chunked/i.test(header);
                ntripHeaderParsed = true;
                rtkStats = {
                    ...freshStats(),
                    connected: true,
                    source: 'ntrip',
                    lastUpdateTime: Date.now()
                };
                rtcmCountWindow = [];
                console.log(`[ntrip] Connected to ${cfg.host}:${cfg.port}/${cfg.mountpoint}${ntripChunked ? ' (chunked)' : ''}`);
                startGgaTimer();
                sendRTKStatus();
                ok();
                if (body.length) ntripFeedBody(body);
                return;
            }
            ntripFeedBody(data);
        });

        sock.on('error', (err) => {
            console.error('[ntrip] Socket error:', err.message);
            fail(err);
        });

        sock.on('close', () => {
            ntripHeaderParsed = false;
            stopGgaTimer();
            if (ntripSocket === sock) ntripSocket = null;
            if (rtkStats.source === 'ntrip' && rtkStats.connected) {
                rtkStats.connected = false;
                sendRTKStatus();
            }
            fail(new Error('NTRIP connection closed'));
            // Auto-reconnect unless the user disconnected or the config is invalid
            if (!ntripUserDisconnect && ntripConfig) {
                ntripReconnectCount++;
                console.log(`[ntrip] Connection lost — reconnecting in 5 s (attempt ${ntripReconnectCount})`);
                clearTimeout(ntripReconnectTimer);
                ntripReconnectTimer = setTimeout(() => {
                    if (!ntripUserDisconnect && ntripConfig) ntripOpen().catch(() => {});
                }, 5000);
            }
        });
    });
}

/**
 * Feed NTRIP body bytes into the shared RTCM pipeline (de-chunking if needed)
 */
function ntripFeedBody(data) {
    const payload = ntripChunked ? ntripDechunk(data) : data;
    if (payload && payload.length) handleRTKData(payload);
}

/**
 * Incremental HTTP chunked-transfer decoder (NTRIP v2 casters)
 */
function ntripDechunk(data) {
    ntripChunkBuf = Buffer.concat([ntripChunkBuf, data]);
    const out = [];
    while (ntripChunkBuf.length > 0) {
        if (ntripChunkRemain > 0) {
            const take = Math.min(ntripChunkRemain, ntripChunkBuf.length);
            out.push(ntripChunkBuf.subarray(0, take));
            ntripChunkBuf = ntripChunkBuf.subarray(take);
            ntripChunkRemain -= take;
            continue;
        }
        // Skip the CRLF that terminates the previous chunk, then read a size line
        let start = 0;
        while (start < ntripChunkBuf.length && (ntripChunkBuf[start] === 0x0D || ntripChunkBuf[start] === 0x0A)) start++;
        const lineEnd = ntripChunkBuf.indexOf('\r\n', start);
        if (lineEnd < 0) { ntripChunkBuf = ntripChunkBuf.subarray(start); break; }
        const sizeStr = ntripChunkBuf.subarray(start, lineEnd).toString('ascii').trim();
        ntripChunkBuf = ntripChunkBuf.subarray(lineEnd + 2);
        const size = parseInt(sizeStr, 16);
        if (!isFinite(size) || size <= 0) { ntripChunkBuf = Buffer.alloc(0); break; }
        ntripChunkRemain = size;
    }
    return out.length ? Buffer.concat(out) : null;
}

/**
 * Stop the NTRIP session. userRequested=true also clears the config
 * and suppresses auto-reconnect.
 */
function stopNtrip(userRequested) {
    if (userRequested) ntripUserDisconnect = true;
    if (ntripReconnectTimer) { clearTimeout(ntripReconnectTimer); ntripReconnectTimer = null; }
    stopGgaTimer();
    ntripHeaderParsed = false;
    if (ntripSocket) {
        try { ntripSocket.destroy(); } catch (e) { /* ignore */ }
        ntripSocket = null;
    }
    if (userRequested) ntripConfig = null;
    if (rtkStats.source === 'ntrip' && rtkStats.connected) {
        rtkStats.connected = false;
        sendRTKStatus();
    }
}

// ── GGA upload (VRS network support) ─────────────────────────────────

function startGgaTimer() {
    stopGgaTimer();
    const cfg = ntripConfig;
    if (!cfg || cfg.ggaMode === 'off') return;
    sendGga();
    ntripGgaTimer = setInterval(sendGga, cfg.ggaInterval * 1000);
}

function stopGgaTimer() {
    if (ntripGgaTimer) { clearInterval(ntripGgaTimer); ntripGgaTimer = null; }
}

function sendGga() {
    const cfg = ntripConfig;
    if (!cfg || !ntripSocket || !ntripHeaderParsed) return;
    const pos = cfg.ggaMode === 'manual'
        ? { lat: cfg.lat, lon: cfg.lon, alt: cfg.alt }
        : lastVehiclePos;
    if (!pos || !isFinite(pos.lat) || !isFinite(pos.lon)) return;
    if (pos.lat === 0 && pos.lon === 0) return;
    try {
        ntripSocket.write(buildGGA(pos.lat, pos.lon, pos.alt || 0));
        ntripGgaSentCount++;
    } catch (e) { /* ignore */ }
}

/**
 * Build an NMEA GGA sentence for the given position
 */
function buildGGA(lat, lon, alt) {
    const now = new Date();
    const t = String(now.getUTCHours()).padStart(2, '0')
            + String(now.getUTCMinutes()).padStart(2, '0')
            + String(now.getUTCSeconds()).padStart(2, '0') + '.00';
    const latAbs = Math.abs(lat);
    const latDeg = Math.floor(latAbs);
    const latStr = String(latDeg).padStart(2, '0')
                 + ((latAbs - latDeg) * 60).toFixed(7).padStart(10, '0');
    const lonAbs = Math.abs(lon);
    const lonDeg = Math.floor(lonAbs);
    const lonStr = String(lonDeg).padStart(3, '0')
                 + ((lonAbs - lonDeg) * 60).toFixed(7).padStart(10, '0');
    const body = `GPGGA,${t},${latStr},${lat >= 0 ? 'N' : 'S'},${lonStr},${lon >= 0 ? 'E' : 'W'},1,10,1.0,${alt.toFixed(1)},M,0.0,M,,`;
    let cs = 0;
    for (let i = 0; i < body.length; i++) cs ^= body.charCodeAt(i);
    return `$${body}*${cs.toString(16).toUpperCase().padStart(2, '0')}\r\n`;
}

// ── Sourcetable ──────────────────────────────────────────────────────

/**
 * Fetch and parse the caster sourcetable ("GET /").
 * Returns an array of { mountpoint, identifier, format, navSystem, country, lat, lon }.
 */
function fetchSourcetable(cfg) {
    return new Promise((resolve, reject) => {
        const host = String(cfg.host || '').trim();
        const port = parseInt(cfg.port) || 2101;
        if (!host) return reject(new Error('Caster host is required'));

        let buf = Buffer.alloc(0);
        let settled = false;
        let timer = null;

        const finish = () => {
            if (settled) return;
            settled = true;
            clearTimeout(timer);
            try { sock.destroy(); } catch (e) { /* ignore */ }
            const text = buf.toString('utf8');
            const statusLine = (text.split('\r\n')[0] || '').trim();
            if (!/200/.test(statusLine) && !/STR;/.test(text)) {
                return reject(new Error(`Sourcetable request failed: ${statusLine || 'no response'}`));
            }
            const entries = [];
            for (const line of text.split(/\r?\n/)) {
                if (!line.startsWith('STR;')) continue;
                const f = line.split(';');
                entries.push({
                    mountpoint: f[1] || '',
                    identifier: f[2] || '',
                    format: f[3] || '',
                    navSystem: f[6] || '',
                    country: f[8] || '',
                    lat: parseFloat(f[9]) || 0,
                    lon: parseFloat(f[10]) || 0
                });
            }
            resolve(entries);
        };
        const failOnce = (err) => {
            if (settled) return;
            settled = true;
            clearTimeout(timer);
            try { sock.destroy(); } catch (e) { /* ignore */ }
            reject(err);
        };

        const onConnect = () => {
            let req = `GET / HTTP/1.1\r\nHost: ${host}:${port}\r\nUser-Agent: NTRIP CORV-GCS\r\nAccept: */*\r\n`;
            if (cfg.username || cfg.password) {
                const auth = Buffer.from(`${cfg.username || ''}:${cfg.password || ''}`).toString('base64');
                req += `Authorization: Basic ${auth}\r\n`;
            }
            req += 'Connection: close\r\n\r\n';
            sock.write(req);
        };

        const sock = cfg.tls
            ? tls.connect({ host, port, rejectUnauthorized: false }, onConnect)
            : net.connect({ host, port }, onConnect);
        timer = setTimeout(finish, 10000);

        sock.on('data', (d) => {
            buf = Buffer.concat([buf, d]);
            if (buf.length > 2 * 1024 * 1024) return finish();
            if (buf.includes('ENDSOURCETABLE')) finish();
        });
        sock.on('error', failOnce);
        sock.on('close', finish);
    });
}

/**
 * Send RTK status to renderer
 */
function sendRTKStatus() {
    if (mainWindow && !mainWindow.isDestroyed()) {
        mainWindow.webContents.send('rtk-status-update', {
            connected: rtkStats.connected,
            source: rtkStats.source || '',
            portPath: rtkStats.portPath,
            baudRate: rtkStats.baudRate,
            bytesReceived: rtkStats.bytesReceived,
            rtcmMsgCount: rtkStats.rtcmMsgCount,
            rtcmMsgPerSec: rtkStats.rtcmMsgPerSec,
            rtcmLastTypes: rtkStats.rtcmLastTypes,
            ntrip: (rtkStats.source === 'ntrip' && ntripConfig) ? {
                host: ntripConfig.host,
                port: ntripConfig.port,
                mountpoint: ntripConfig.mountpoint,
                ggaMode: ntripConfig.ggaMode,
                reconnects: ntripReconnectCount,
                ggaSent: ntripGgaSentCount
            } : null
        });
    }
}

/**
 * Disconnect RTK serial port
 */
async function disconnectRTK() {
    if (rtkPort) {
        try {
            if (rtkPort.isOpen) {
                await new Promise((resolve) => rtkPort.close(resolve));
            }
        } catch (e) {
            console.error('[rtk] Disconnect error:', e.message);
        }
        rtkPort = null;
        rtkStats.connected = false;
        sendRTKStatus();
        console.log('[rtk] Disconnected');
    }
}

/**
 * Cleanup on app quit
 */
function cleanup() {
    stopNtrip(true);
    if (rtkPort && rtkPort.isOpen) {
        try { rtkPort.close(); } catch (e) { /* ignore */ }
    }
    rtkPort = null;
}

module.exports = { initRTKHandlers, cleanup };
