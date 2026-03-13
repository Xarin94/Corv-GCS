/**
 * SerialHandler.js - Serial Communication for Live Mode
 * Handles WebSerial API for live telemetry data
 */

import { STATE } from '../core/state.js';
import { ORIGIN } from '../core/constants.js';
import { calculateCRC16 } from '../core/utils.js';

/**
 * Connect to serial port and start reading
 */
export async function connectSerial() {
    try {
        const port = await navigator.serial.requestPort();
        await port.open({ baudRate: 460800 });

        // Switch to LIVE mode
        STATE.connected = true;
        STATE.mode = 'LIVE';
        
        // Update UI
        document.getElementById('storyline-panel').classList.remove('visible');
        document.getElementById('btn-link').classList.add('active');
        document.getElementById('btn-link').innerText = "LINK ACTIVE";
        document.getElementById('btn-load').classList.remove('active');

        readSerialLoop(port);
    } catch (e) {
        alert("Serial Error: " + e.message);
    }
}

/**
 * Continuous serial reading loop
 * @param {SerialPort} port 
 */
async function readSerialLoop(port) {
    const reader = port.readable.getReader();
    let buffer = new Uint8Array(4096);
    let bufferLen = 0;

    try {
        while (true) {
            const { value, done } = await reader.read();
            if (done) break;
            if (STATE.mode !== 'LIVE') continue;

            if (bufferLen + value.length > buffer.length) bufferLen = 0;
            buffer.set(value, bufferLen);
            bufferLen += value.length;

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
    } catch (e) {
        console.error(e);
    } finally {
        reader.releaseLock();
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

    // Dispatch update event
    window.dispatchEvent(new CustomEvent('serialUpdate'));
}
