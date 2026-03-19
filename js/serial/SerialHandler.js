/**
 * SerialHandler.js - Serial Communication for Live Mode
 * Connects to CORV binary telemetry via main-process serial (IPC).
 * Parsing and MAVLink emulation happen in the main process.
 */

import { STATE } from '../core/state.js';

/**
 * Connect to serial port via main process IPC
 * @param {string} portPath - Serial port path (e.g. /dev/ttyUSB0)
 * @param {number} [baudRate=460800] - Baud rate
 */
export async function connectSerial(portPath, baudRate = 460800) {
    await window.corvSerial.connect(portPath, baudRate);

    // Switch to LIVE mode
    STATE.connected = true;
    STATE.mode = 'LIVE';
    STATE.connectionType = 'corv-binary';

    // Update UI
    document.getElementById('storyline-panel').classList.remove('visible');
    document.getElementById('btn-link').classList.add('active');
    document.getElementById('btn-link').innerText = "LINK ACTIVE";
    document.getElementById('btn-load').classList.remove('active');
}
