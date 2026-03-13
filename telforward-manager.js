/**
 * telforward-manager.js - Telemetry Forward Manager
 * Forwards live telemetry data to an external serial port
 * (e.g. antenna tracker, secondary GCS, data logger).
 * Supports MAVLink passthrough and LTM (Lightweight Telemetry) protocols.
 */

const { ipcMain } = require('electron');

// Lazy-load serialport
let SerialPort = null;
function ensureSerialLoaded() {
    if (SerialPort) return;
    SerialPort = require('serialport').SerialPort;
}

// State
let fwdPort = null;
let mainWindow = null;
let fwdStats = {
    connected: false,
    portPath: '',
    baudRate: 0,
    protocol: '',
    bytesSent: 0,
    msgCount: 0,
    msgPerSec: 0,
    lastUpdateTime: 0
};

// Message rate tracking
let msgCountWindow = [];

// LTM timers
let ltmFastTimer = null;   // G+A frames at 5 Hz
let ltmSlowTimer = null;   // S frame at 2 Hz
let ltmOriginTimer = null;  // O frame at 0.5 Hz

// Latest state snapshot from renderer (for LTM)
let latestState = null;

// Raw packet callback registration (for MAVLink passthrough)
let rawPacketCallbackRegistered = false;

/**
 * Initialize Telemetry Forward IPC handlers
 */
function initTelForwardHandlers(win) {
    mainWindow = win;

    // List serial ports
    ipcMain.handle('telfwd-list-ports', async () => {
        try {
            ensureSerialLoaded();
            const ports = await SerialPort.list();
            return ports.map(p => ({
                path: p.path,
                manufacturer: p.manufacturer || '',
                vendorId: p.vendorId || '',
                productId: p.productId || '',
                serialNumber: p.serialNumber || ''
            }));
        } catch (e) {
            console.error('[telfwd] Failed to list ports:', e.message);
            return [];
        }
    });

    // Connect and start forwarding
    ipcMain.handle('telfwd-connect', async (event, portPath, baudRate, protocol) => {
        await disconnectForward();
        ensureSerialLoaded();

        try {
            fwdPort = new SerialPort({
                path: portPath,
                baudRate: baudRate || 9600,
                autoOpen: false
            });

            await new Promise((resolve, reject) => {
                fwdPort.open((err) => {
                    if (err) reject(err);
                    else resolve();
                });
            });

            fwdStats = {
                connected: true,
                portPath,
                baudRate: baudRate || 9600,
                protocol: protocol || 'ltm',
                bytesSent: 0,
                msgCount: 0,
                msgPerSec: 0,
                lastUpdateTime: Date.now()
            };
            msgCountWindow = [];
            latestState = null;

            fwdPort.on('error', (err) => {
                console.error('[telfwd] Serial error:', err.message);
                sendStatus();
            });

            fwdPort.on('close', () => {
                console.log('[telfwd] Port closed');
                fwdStats.connected = false;
                stopTimers();
                sendStatus();
            });

            // Start protocol-specific forwarding
            if (protocol === 'mavlink') {
                startMAVLinkPassthrough();
            } else {
                startLTMForwarding();
            }

            console.log(`[telfwd] Connected to ${portPath} at ${baudRate || 9600} baud, protocol: ${protocol || 'ltm'}`);
            sendStatus();
            return { success: true };
        } catch (e) {
            console.error('[telfwd] Connect failed:', e.message);
            throw e;
        }
    });

    // Disconnect
    ipcMain.handle('telfwd-disconnect', async () => {
        await disconnectForward();
        return { success: true };
    });

    // Get current stats
    ipcMain.handle('telfwd-get-stats', () => {
        return { ...fwdStats };
    });

    // Receive STATE snapshot from renderer (for LTM mode)
    ipcMain.handle('telfwd-feed-state', (event, snapshot) => {
        latestState = snapshot;
    });
}

// ─── MAVLink Passthrough ──────────────────────────────────────────────

/**
 * Called from main-mavlink.js for each received raw MAVLink packet.
 * If connected in mavlink mode, write the raw buffer to the forward port.
 */
function feedRawPacket(packet) {
    if (!fwdStats.connected || fwdStats.protocol !== 'mavlink') return;
    if (!fwdPort || !fwdPort.isOpen) return;
    if (!packet || !packet.buffer) return;

    try {
        const buf = Buffer.isBuffer(packet.buffer) ? packet.buffer : Buffer.from(packet.buffer);
        fwdPort.write(buf);
        trackMessage(buf.length);
    } catch (e) {
        // Silently ignore write errors to not spam console
    }
}

function startMAVLinkPassthrough() {
    // Register raw packet callback in main-mavlink
    if (!rawPacketCallbackRegistered) {
        try {
            const mavlink = require('./main-mavlink');
            if (mavlink.registerRawPacketCallback) {
                mavlink.registerRawPacketCallback(feedRawPacket);
                rawPacketCallbackRegistered = true;
            }
        } catch (e) {
            console.error('[telfwd] Could not register MAVLink callback:', e.message);
        }
    }
}

// ─── LTM (Lightweight Telemetry) ─────────────────────────────────────

function startLTMForwarding() {
    // G-frame (GPS) + A-frame (Attitude) at 5 Hz
    ltmFastTimer = setInterval(() => {
        if (!latestState || !fwdPort || !fwdPort.isOpen) return;
        const gFrame = buildLTMGFrame(latestState);
        const aFrame = buildLTMAFrame(latestState);
        writeLTM(gFrame);
        writeLTM(aFrame);
    }, 200);

    // S-frame (Status) at 2 Hz
    ltmSlowTimer = setInterval(() => {
        if (!latestState || !fwdPort || !fwdPort.isOpen) return;
        const sFrame = buildLTMSFrame(latestState);
        writeLTM(sFrame);
    }, 500);

    // O-frame (Origin/Home) at 0.5 Hz
    ltmOriginTimer = setInterval(() => {
        if (!latestState || !fwdPort || !fwdPort.isOpen) return;
        const oFrame = buildLTMOFrame(latestState);
        writeLTM(oFrame);
    }, 2000);
}

function writeLTM(frame) {
    if (!frame || !fwdPort || !fwdPort.isOpen) return;
    try {
        fwdPort.write(frame);
        trackMessage(frame.length);
    } catch (e) {
        // Silently ignore
    }
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

// ─── Helpers ──────────────────────────────────────────────────────────

function trackMessage(bytes) {
    fwdStats.bytesSent += bytes;
    fwdStats.msgCount++;

    const now = Date.now();
    msgCountWindow.push(now);
    // Keep only last 5 seconds
    msgCountWindow = msgCountWindow.filter(t => now - t < 5000);
    fwdStats.msgPerSec = Math.round(msgCountWindow.length / 5);

    // Send status update at max 4 Hz
    if (now - fwdStats.lastUpdateTime > 250) {
        fwdStats.lastUpdateTime = now;
        sendStatus();
    }
}

function sendStatus() {
    if (mainWindow && !mainWindow.isDestroyed()) {
        mainWindow.webContents.send('telfwd-status-update', {
            connected: fwdStats.connected,
            portPath: fwdStats.portPath,
            baudRate: fwdStats.baudRate,
            protocol: fwdStats.protocol,
            bytesSent: fwdStats.bytesSent,
            msgCount: fwdStats.msgCount,
            msgPerSec: fwdStats.msgPerSec
        });
    }
}

function stopTimers() {
    if (ltmFastTimer) { clearInterval(ltmFastTimer); ltmFastTimer = null; }
    if (ltmSlowTimer) { clearInterval(ltmSlowTimer); ltmSlowTimer = null; }
    if (ltmOriginTimer) { clearInterval(ltmOriginTimer); ltmOriginTimer = null; }
}

async function disconnectForward() {
    stopTimers();
    latestState = null;

    if (fwdPort) {
        try {
            if (fwdPort.isOpen) {
                await new Promise((resolve) => fwdPort.close(resolve));
            }
        } catch (e) {
            console.error('[telfwd] Disconnect error:', e.message);
        }
        fwdPort = null;
        fwdStats.connected = false;
        sendStatus();
        console.log('[telfwd] Disconnected');
    }
}

function cleanup() {
    stopTimers();
    if (fwdPort && fwdPort.isOpen) {
        try { fwdPort.close(); } catch (e) { /* ignore */ }
    }
    fwdPort = null;
}

module.exports = { initTelForwardHandlers, cleanup };
