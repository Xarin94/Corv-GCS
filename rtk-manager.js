/**
 * rtk-manager.js - RTK Base Station GPS Manager
 * Connects to a GPS base station (e.g. u-blox F9P) via serial,
 * reads RTCM3 correction data, and forwards it to the drone
 * via MAVLink GPS_RTCM_DATA messages (ID 233).
 * Also parses UBX/NMEA for base station status display.
 */

const { ipcMain } = require('electron');
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
let rtkStats = {
    connected: false,
    portPath: '',
    baudRate: 0,
    bytesReceived: 0,
    rtcmMsgCount: 0,
    rtcmMsgPerSec: 0,
    rtcmLastTypes: [],   // last seen RTCM message types
    lastUpdateTime: 0
};

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
                connected: true,
                portPath,
                baudRate: baudRate || 115200,
                bytesReceived: 0,
                rtcmMsgCount: 0,
                rtcmMsgPerSec: 0,
                rtcmLastTypes: [],
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

/**
 * Send RTK status to renderer
 */
function sendRTKStatus() {
    if (mainWindow && !mainWindow.isDestroyed()) {
        mainWindow.webContents.send('rtk-status-update', {
            connected: rtkStats.connected,
            portPath: rtkStats.portPath,
            baudRate: rtkStats.baudRate,
            bytesReceived: rtkStats.bytesReceived,
            rtcmMsgCount: rtkStats.rtcmMsgCount,
            rtcmMsgPerSec: rtkStats.rtcmMsgPerSec,
            rtcmLastTypes: rtkStats.rtcmLastTypes
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
    if (rtkPort && rtkPort.isOpen) {
        try { rtkPort.close(); } catch (e) { /* ignore */ }
    }
    rtkPort = null;
}

module.exports = { initRTKHandlers, cleanup };
