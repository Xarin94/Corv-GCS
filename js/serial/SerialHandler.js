/**
 * SerialHandler.js - Serial Communication for Live Mode
 * Handles CORV binary telemetry via main-process serial (IPC)
 */

import { STATE } from '../core/state.js';
import { ORIGIN } from '../core/constants.js';
import { calculateCRC16 } from '../core/utils.js';

let buffer = new Uint8Array(4096);
let bufferLen = 0;

/**
 * Connect to serial port via main process IPC and start reading
 * @param {string} portPath - Serial port path (e.g. /dev/ttyUSB0)
 * @param {number} [baudRate=460800] - Baud rate
 */
export async function connectSerial(portPath, baudRate = 460800) {
    await window.corvSerial.connect(portPath, baudRate);

    // Reset buffer state
    buffer = new Uint8Array(4096);
    bufferLen = 0;

    // Switch to LIVE mode
    STATE.connected = true;
    STATE.mode = 'LIVE';
    STATE.connectionType = 'corv-binary';

    // Update UI
    document.getElementById('storyline-panel').classList.remove('visible');
    document.getElementById('btn-link').classList.add('active');
    document.getElementById('btn-link').innerText = "LINK ACTIVE";
    document.getElementById('btn-load').classList.remove('active');

    // Listen for incoming serial data from main process
    window.corvSerial.onData((data) => {
        if (STATE.mode !== 'LIVE') return;
        const chunk = new Uint8Array(data);
        if (bufferLen + chunk.length > buffer.length) bufferLen = 0;
        buffer.set(chunk, bufferLen);
        bufferLen += chunk.length;
        processBuffer();
    });
}

/**
 * Process the serial buffer, extracting complete CORV packets
 */
function processBuffer() {
    let processAgain = true;
    while (processAgain && bufferLen >= 5) {
        processAgain = false;

        // Find sync pattern
        let syncIndex = -1;
        for (let i = 0; i < bufferLen - 1; i++) {
            if (buffer[i] === 0xA5 && buffer[i + 1] === 0x5A) {
                syncIndex = i;
                break;
            }
        }

        if (syncIndex > 0) {
            buffer.copyWithin(0, syncIndex, bufferLen);
            bufferLen -= syncIndex;
            syncIndex = 0;
        }

        if (syncIndex === -1) {
            if (bufferLen > 0) {
                buffer[0] = buffer[bufferLen - 1];
                bufferLen = 1;
            }
            continue;
        }

        if (bufferLen < 5) continue;

        const packetType = buffer[2];
        const payloadLen = buffer[3];
        const totalLen = 5 + payloadLen + 2;

        if (bufferLen < totalLen) continue;

        const crcCalc = calculateCRC16(buffer, 2, 3 + payloadLen);
        const crcRecv = buffer[totalLen - 2] | (buffer[totalLen - 1] << 8);

        if (crcCalc === crcRecv) {
            const dv = new DataView(buffer.slice(5, 5 + payloadLen).buffer);
            if (packetType === 0x01) {
                parseNavigationPacket(dv);
            }
            buffer.copyWithin(0, totalLen, bufferLen);
            bufferLen -= totalLen;
            processAgain = true;
        } else {
            buffer.copyWithin(0, 1, bufferLen);
            bufferLen -= 1;
            processAgain = true;
        }
    }
}

/**
 * Parse navigation packet from serial data
 * @param {DataView} dv - Data view of packet payload
 */
function parseNavigationPacket(dv) {
    if (STATE.mode !== 'LIVE') return;

    STATE.roll = dv.getInt16(4, true) / 1000.0;
    STATE.pitch = dv.getInt16(6, true) / 1000.0;
    STATE.yaw = dv.getInt16(8, true) / 1000.0;

    const rawLat = dv.getFloat64(10, true);
    const rawLon = dv.getFloat64(18, true);
    STATE.rawAlt = dv.getFloat32(26, true);

    if (Math.abs(rawLat) > 0.1 || Math.abs(rawLon) > 0.1) {
        STATE.lat = rawLat;
        STATE.lon = rawLon;
    }

    // Air Data
    STATE.as = dv.getFloat32(54, true);
    STATE.gs = dv.getFloat32(58, true);
    STATE.aoa = dv.getFloat32(62, true);
    STATE.ssa = dv.getFloat32(66, true);

    const vd = dv.getFloat32(38, true);
    STATE.vs = -vd;

    // Acceleration data comes from MAVLink (SCALED_IMU), not from binary protocol

}
