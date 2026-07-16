/**
 * telforward-manager.js - Telemetry Forward Manager (MAVLink Mirror)
 * Forwards live telemetry to one or more outputs simultaneously,
 * Mission Planner "MAVLink Mirror" style:
 *  - Serial port      (MAVLink passthrough or LTM)
 *  - UDP Client       (send to a host:port, e.g. a secondary GCS)
 *  - UDP Server       (listen on a port, forward to every peer that talks to us)
 * MAVLink outputs can optionally have "write access": packets received from
 * the mirror endpoint are injected into the vehicle link (bidirectional).
 */

const { ipcMain } = require('electron');
const dgram = require('dgram');

// Lazy-load serialport
let SerialPort = null;
function ensureSerialLoaded() {
    if (SerialPort) return;
    SerialPort = require('serialport').SerialPort;
}

// Lazy-require main-mavlink (for write-access injection)
let mavlinkMain = null;
function ensureMavlinkMain() {
    if (!mavlinkMain) mavlinkMain = require('./main-mavlink');
    return mavlinkMain;
}

// State
let mainWindow = null;
let outputs = new Map(); // id -> output object
let nextOutputId = 1;
let statusInterval = null;

// LTM timers (shared across all LTM outputs)
let ltmFastTimer = null;   // G+A frames at 5 Hz
let ltmSlowTimer = null;   // S frame at 2 Hz
let ltmOriginTimer = null; // O frame at 0.5 Hz

// Latest state snapshot from renderer (for LTM)
let latestState = null;

// UDP-server peers are forgotten after this much silence
const UDP_CLIENT_TTL_MS = 60000;

/**
 * Initialize Telemetry Forward IPC handlers
 */
function initTelForwardHandlers(win) {
    mainWindow = win;

    // Register the raw packet callback once — it fans out to all MAVLink outputs
    try {
        ensureMavlinkMain();
        if (mavlinkMain.registerRawPacketCallback) {
            mavlinkMain.registerRawPacketCallback(feedRawPacket);
        }
    } catch (e) {
        console.error('[telfwd] Could not register MAVLink callback:', e.message);
    }

    // List serial ports
    ipcMain.handle('telfwd-list-ports', async () => {
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
            console.error('[telfwd] Failed to list ports:', e.message);
            return [];
        }
    });

    // Add and start a forward output
    ipcMain.handle('telfwd-add-output', async (event, cfg) => {
        const output = await createOutput(cfg || {});
        outputs.set(output.id, output);
        ensureTimers();
        broadcastStatus();
        console.log(`[telfwd] Output #${output.id} added: ${output.label} (${output.protocol})`);
        return { success: true, id: output.id };
    });

    // Remove an output
    ipcMain.handle('telfwd-remove-output', async (event, id) => {
        const output = outputs.get(id);
        if (output) {
            closeOutput(output);
            outputs.delete(id);
            ensureTimers();
            broadcastStatus();
            console.log(`[telfwd] Output #${id} removed`);
        }
        return { success: true };
    });

    // Get current outputs
    ipcMain.handle('telfwd-get-outputs', () => {
        return serializeOutputs();
    });

    // Receive STATE snapshot from renderer (for LTM mode)
    ipcMain.handle('telfwd-feed-state', (event, snapshot) => {
        latestState = snapshot;
    });
}

// ─── Output lifecycle ─────────────────────────────────────────────────

/**
 * Create and start an output from a config object:
 * { type: 'serial'|'udp-client'|'udp-server', protocol: 'mavlink'|'ltm',
 *   writeAccess, portPath, baudRate, host, port, listenPort }
 */
async function createOutput(cfg) {
    const type = cfg.type;
    const protocol = cfg.protocol === 'ltm' ? 'ltm' : 'mavlink';
    const output = {
        id: nextOutputId++,
        type,
        protocol,
        writeAccess: protocol === 'mavlink' && !!cfg.writeAccess,
        connected: false,
        label: '',
        error: null,
        stats: { bytesSent: 0, msgCount: 0, msgPerSec: 0, bytesRx: 0, window: [] },
        // handles
        port: null,      // serial
        socket: null,    // udp
        clients: null    // udp-server peers: key 'addr:port' -> { address, port, lastSeen }
    };

    if (type === 'serial') {
        ensureSerialLoaded();
        const portPath = cfg.portPath;
        const baudRate = parseInt(cfg.baudRate) || 57600;
        if (!portPath) throw new Error('Serial port is required');
        // Refuse a port already used by another output
        for (const o of outputs.values()) {
            if (o.type === 'serial' && o.portPath === portPath) {
                throw new Error(`${portPath} is already in use by output #${o.id}`);
            }
        }
        const port = new SerialPort({ path: portPath, baudRate, autoOpen: false });
        await new Promise((resolve, reject) => {
            port.open((err) => err ? reject(err) : resolve());
        });
        port.on('error', (err) => {
            console.error(`[telfwd] #${output.id} serial error:`, err.message);
            output.error = err.message;
            broadcastStatus();
        });
        port.on('close', () => {
            output.connected = false;
            broadcastStatus();
        });
        if (output.writeAccess) {
            port.on('data', (data) => {
                output.stats.bytesRx += data.length;
                injectToVehicle(data);
            });
        }
        output.port = port;
        output.portPath = portPath;
        output.baudRate = baudRate;
        output.label = `SERIAL ${portPath} @ ${baudRate}`;
    } else if (type === 'udp-client') {
        const host = String(cfg.host || '').trim();
        const port = parseInt(cfg.port);
        if (!host || !port) throw new Error('UDP host and port are required');
        const socket = dgram.createSocket('udp4');
        socket.on('error', (err) => {
            console.error(`[telfwd] #${output.id} UDP error:`, err.message);
            output.error = err.message;
            broadcastStatus();
        });
        socket.on('message', (msg) => {
            output.stats.bytesRx += msg.length;
            if (output.writeAccess) injectToVehicle(msg);
        });
        output.socket = socket;
        output.host = host;
        output.udpPort = port;
        output.label = `UDP CLIENT → ${host}:${port}`;
    } else if (type === 'udp-server') {
        const listenPort = parseInt(cfg.listenPort);
        if (!listenPort) throw new Error('Listen port is required');
        const socket = dgram.createSocket('udp4');
        const clients = new Map();
        await new Promise((resolve, reject) => {
            socket.once('error', reject);
            socket.bind(listenPort, () => {
                socket.removeListener('error', reject);
                resolve();
            });
        });
        socket.on('error', (err) => {
            console.error(`[telfwd] #${output.id} UDP server error:`, err.message);
            output.error = err.message;
            broadcastStatus();
        });
        socket.on('message', (msg, rinfo) => {
            // Any peer that sends us data becomes a forward destination
            clients.set(`${rinfo.address}:${rinfo.port}`, {
                address: rinfo.address,
                port: rinfo.port,
                lastSeen: Date.now()
            });
            output.stats.bytesRx += msg.length;
            if (output.writeAccess) injectToVehicle(msg);
        });
        output.socket = socket;
        output.clients = clients;
        output.listenPort = listenPort;
        output.label = `UDP SERVER :${listenPort}`;
    } else {
        throw new Error(`Unknown output type: ${type}`);
    }

    output.connected = true;
    return output;
}

function closeOutput(output) {
    try {
        if (output.port && output.port.isOpen) output.port.close();
    } catch (e) { /* ignore */ }
    try {
        if (output.socket) output.socket.close();
    } catch (e) { /* ignore */ }
    output.port = null;
    output.socket = null;
    output.connected = false;
}

/**
 * Inject bytes received from a mirror endpoint into the vehicle link
 * (write access — a secondary GCS can send commands through us)
 */
function injectToVehicle(buf) {
    try {
        ensureMavlinkMain().sendRawBuffer(buf);
    } catch (e) { /* ignore */ }
}

// ─── Data fan-out ─────────────────────────────────────────────────────

/**
 * Called from main-mavlink.js for each received raw MAVLink packet.
 * Writes the raw buffer to every connected MAVLink output.
 */
function feedRawPacket(packet) {
    if (outputs.size === 0) return;
    if (!packet || !packet.buffer) return;
    let buf = null;
    for (const output of outputs.values()) {
        if (!output.connected || output.protocol !== 'mavlink') continue;
        if (!buf) buf = Buffer.isBuffer(packet.buffer) ? packet.buffer : Buffer.from(packet.buffer);
        writeOut(output, buf);
    }
}

/**
 * Write a buffer to an output (any type)
 */
function writeOut(output, buf) {
    try {
        if (output.type === 'serial') {
            if (output.port && output.port.isOpen) {
                output.port.write(buf);
                trackMessage(output, buf.length);
            }
        } else if (output.type === 'udp-client') {
            output.socket.send(buf, output.udpPort, output.host);
            trackMessage(output, buf.length);
        } else if (output.type === 'udp-server') {
            const now = Date.now();
            for (const [key, client] of output.clients) {
                if (now - client.lastSeen > UDP_CLIENT_TTL_MS) {
                    output.clients.delete(key);
                    continue;
                }
                output.socket.send(buf, client.port, client.address);
            }
            if (output.clients.size > 0) trackMessage(output, buf.length);
        }
    } catch (e) {
        // Silently ignore write errors to not spam the console
    }
}

// ─── LTM (Lightweight Telemetry) ─────────────────────────────────────

function hasLTMOutputs() {
    for (const o of outputs.values()) {
        if (o.protocol === 'ltm' && o.connected) return true;
    }
    return false;
}

function writeLTMToAll(frame) {
    if (!frame) return;
    for (const output of outputs.values()) {
        if (output.protocol !== 'ltm' || !output.connected) continue;
        writeOut(output, frame);
    }
}

function ensureTimers() {
    // LTM timers
    if (hasLTMOutputs()) {
        if (!ltmFastTimer) startLTMTimers();
    } else {
        stopLTMTimers();
    }
    // Status broadcast at 1 Hz while any output exists
    if (outputs.size > 0) {
        if (!statusInterval) {
            statusInterval = setInterval(() => {
                updateRates();
                broadcastStatus();
            }, 1000);
        }
    } else if (statusInterval) {
        clearInterval(statusInterval);
        statusInterval = null;
    }
}

function startLTMTimers() {
    // G-frame (GPS) + A-frame (Attitude) at 5 Hz
    ltmFastTimer = setInterval(() => {
        if (!latestState) return;
        writeLTMToAll(buildLTMGFrame(latestState));
        writeLTMToAll(buildLTMAFrame(latestState));
    }, 200);

    // S-frame (Status) at 2 Hz
    ltmSlowTimer = setInterval(() => {
        if (!latestState) return;
        writeLTMToAll(buildLTMSFrame(latestState));
    }, 500);

    // O-frame (Origin/Home) at 0.5 Hz
    ltmOriginTimer = setInterval(() => {
        if (!latestState) return;
        writeLTMToAll(buildLTMOFrame(latestState));
    }, 2000);
}

function stopLTMTimers() {
    if (ltmFastTimer) { clearInterval(ltmFastTimer); ltmFastTimer = null; }
    if (ltmSlowTimer) { clearInterval(ltmSlowTimer); ltmSlowTimer = null; }
    if (ltmOriginTimer) { clearInterval(ltmOriginTimer); ltmOriginTimer = null; }
}

/**
 * LTM G-Frame (GPS)
 * $T + G + lat(i32) + lon(i32) + groundspeed(u8, m/s) + alt(i32, cm) + sats_fix(u8) + CRC
 * Total: 3 + 14 + 1 = 18 bytes
 */
function buildLTMGFrame(s) {
    const buf = Buffer.alloc(18);
    buf[0] = 0x24; // $
    buf[1] = 0x54; // T
    buf[2] = 0x47; // G

    // Latitude in 1/10,000,000 degrees (1e-7)
    const lat = Math.round((s.lat || 0) * 1e7);
    buf.writeInt32LE(lat, 3);

    // Longitude in 1/10,000,000 degrees (1e-7)
    const lon = Math.round((s.lon || 0) * 1e7);
    buf.writeInt32LE(lon, 7);

    // Groundspeed in m/s
    buf[11] = Math.min(255, Math.max(0, Math.round(s.gs || 0)));

    // Altitude in cm (relative to home)
    const altCm = Math.round((s.relAlt || 0) * 100);
    buf.writeInt32LE(altCm, 12);

    // sats << 2 | fix (0=no GPS, 1=no fix, 2=2D, 3=3D)
    const sats = Math.min(63, s.gpsNumSat || 0);
    const fix = Math.min(3, s.gpsFix || 0);
    buf[16] = (sats << 2) | fix;

    // CRC: XOR of bytes 3..16
    buf[17] = ltmCRC(buf, 3, 17);
    return buf;
}

/**
 * LTM A-Frame (Attitude)
 * $T + A + pitch(i16) + roll(i16) + heading(i16) + CRC
 * Total: 3 + 6 + 1 = 10 bytes
 */
function buildLTMAFrame(s) {
    const buf = Buffer.alloc(10);
    buf[0] = 0x24; // $
    buf[1] = 0x54; // T
    buf[2] = 0x41; // A

    buf.writeInt16LE(Math.round(s.pitch || 0), 3);
    buf.writeInt16LE(Math.round(s.roll || 0), 5);

    // Heading 0-360
    let hdg = Math.round(s.yaw || 0);
    if (hdg < 0) hdg += 360;
    buf.writeInt16LE(hdg, 7);

    buf[9] = ltmCRC(buf, 3, 9);
    return buf;
}

/**
 * LTM S-Frame (Status)
 * $T + S + vbat(u16, mV) + capacity(u16, mAh) + rssi(u8) + airspeed(u8, m/s) + status(u8) + CRC
 * Total: 3 + 7 + 1 = 11 bytes
 */
function buildLTMSFrame(s) {
    const buf = Buffer.alloc(11);
    buf[0] = 0x24; // $
    buf[1] = 0x54; // T
    buf[2] = 0x53; // S

    // Battery voltage in mV
    const vbat = Math.round((s.batteryVoltage || 0) * 1000);
    buf.writeUInt16LE(Math.min(65535, vbat), 3);

    // Battery consumed capacity in mAh (not available from basic telemetry, send 0)
    buf.writeUInt16LE(0, 5);

    // RSSI (use linkQuality if available, else 0)
    buf[7] = Math.min(255, s.linkQuality || 0);

    // Airspeed in m/s
    buf[8] = Math.min(255, Math.max(0, Math.round(s.as || 0)));

    // Status byte: armed(bit0) | failsafe(bit1) | flightmode(bits 2-5)
    const armed = s.armed ? 1 : 0;
    const mode = ltmFlightMode(s.flightMode || '');
    buf[9] = armed | (mode << 2);

    buf[10] = ltmCRC(buf, 3, 10);
    return buf;
}

/**
 * LTM O-Frame (Origin / Home position)
 * $T + O + homeLat(i32) + homeLon(i32) + homeAlt(i32, cm MSL) + fix(u8) + sats(u8) + CRC
 * Total: 3 + 14 + 1 = 18 bytes
 */
function buildLTMOFrame(s) {
    const buf = Buffer.alloc(18);
    buf[0] = 0x24; // $
    buf[1] = 0x54; // T
    buf[2] = 0x4F; // O

    const homeLat = Math.round((s.homeLat || 0) * 1e7);
    buf.writeInt32LE(homeLat, 3);

    const homeLon = Math.round((s.homeLon || 0) * 1e7);
    buf.writeInt32LE(homeLon, 7);

    // Home altitude in cm MSL
    const homeAltCm = Math.round((s.homeAlt || 0) * 100);
    buf.writeInt32LE(homeAltCm, 11);

    // OSD on/off + fix
    buf[15] = Math.min(3, s.gpsFix || 0);
    buf[16] = Math.min(255, s.gpsNumSat || 0);

    buf[17] = ltmCRC(buf, 3, 17);
    return buf;
}

/**
 * LTM CRC: XOR of bytes from startIdx to endIdx-1
 */
function ltmCRC(buf, startIdx, endIdx) {
    let crc = 0;
    for (let i = startIdx; i < endIdx; i++) {
        crc ^= buf[i];
    }
    return crc;
}

/**
 * Map ArduPilot flight mode names to LTM flight mode codes
 * LTM modes: 0=Manual, 1=Rate, 2=Angle, 3=Horizon, 4=Acro,
 *            5=Stabilized1, 6=Stabilized2, 7=AltHold, 8=GPSHold,
 *            9=Waypoints, 10=HeadFree, 11=Circle, 12=RTH, 13=FollowMe,
 *            14=Land, 15=FlyByWireA, 16=FlyByWireB, 17=Cruise, 18=Unknown
 */
function ltmFlightMode(modeName) {
    const name = (modeName || '').toUpperCase();
    if (name.includes('MANUAL')) return 0;
    if (name.includes('ACRO')) return 4;
    if (name.includes('STABILIZE') || name.includes('STAB')) return 5;
    if (name.includes('ALT_HOLD') || name.includes('ALT HOLD')) return 7;
    if (name.includes('LOITER')) return 8;
    if (name.includes('AUTO')) return 9;
    if (name.includes('CIRCLE')) return 11;
    if (name.includes('RTL') || name.includes('RTH')) return 12;
    if (name.includes('LAND')) return 14;
    if (name.includes('FBWA') || name.includes('FLY BY WIRE A')) return 15;
    if (name.includes('FBWB') || name.includes('FLY BY WIRE B')) return 16;
    if (name.includes('CRUISE')) return 17;
    if (name.includes('GUIDED')) return 8;
    if (name.includes('POSHOLD')) return 8;
    return 18; // Unknown
}

// ─── Stats / status ──────────────────────────────────────────────────

function trackMessage(output, bytes) {
    output.stats.bytesSent += bytes;
    output.stats.msgCount++;
    output.stats.window.push(Date.now());
}

function updateRates() {
    const now = Date.now();
    for (const output of outputs.values()) {
        output.stats.window = output.stats.window.filter(t => now - t < 5000);
        output.stats.msgPerSec = Math.round(output.stats.window.length / 5);
    }
}

function serializeOutputs() {
    const list = [];
    for (const output of outputs.values()) {
        list.push({
            id: output.id,
            type: output.type,
            protocol: output.protocol,
            writeAccess: output.writeAccess,
            connected: output.connected,
            label: output.label,
            error: output.error,
            bytesSent: output.stats.bytesSent,
            msgCount: output.stats.msgCount,
            msgPerSec: output.stats.msgPerSec,
            bytesRx: output.stats.bytesRx,
            clientCount: output.clients ? output.clients.size : null
        });
    }
    return list;
}

function broadcastStatus() {
    if (mainWindow && !mainWindow.isDestroyed()) {
        mainWindow.webContents.send('telfwd-status-update', { outputs: serializeOutputs() });
    }
}

// ─── Cleanup ─────────────────────────────────────────────────────────

function cleanup() {
    stopLTMTimers();
    if (statusInterval) { clearInterval(statusInterval); statusInterval = null; }
    for (const output of outputs.values()) {
        closeOutput(output);
    }
    outputs.clear();
}

module.exports = { initTelForwardHandlers, cleanup };
