/**
 * ConnectionManager.js - Renderer-side connection lifecycle manager
 * Coordinates serial/UDP/TCP connections with the main process via IPC
 */

import { STATE } from '../core/state.js';
import { connectMAVLinkSerial, connectMAVLinkUDP, connectMAVLinkTCP, disconnectMAVLink, listSerialPorts, getConnectionState } from './MAVLinkManager.js';
import { connectSerial } from '../serial/SerialHandler.js';
import { requestAllDataStreams, requestHomePosition } from './CommandSender.js';
import { onMessage } from './MAVLinkManager.js';

// Connection config
let currentConnection = null;

/**
 * Connect using the specified connection type
 * @param {'corv-binary'|'mavlink-serial'|'mavlink-udp'} type
 * @param {object} options - Connection options
 */
export async function connect(type, options = {}) {
    // Disconnect existing connection first
    if (STATE.connected) {
        await disconnect();
    }

    switch (type) {
        case 'corv-binary':
            await connectSerial(options.port || '', options.baudRate || 460800);
            currentConnection = { type, ...options };
            break;

        case 'mavlink-serial':
            await connectMAVLinkSerial(
                options.port || '',
                options.baudRate || 57600
            );
            currentConnection = { type, ...options };
            // Request data streams after connection
            setTimeout(() => requestAllDataStreams(), 1000);
            // Request home position after streams are established
            setTimeout(() => requestHomePosition().catch(() => {}), 3000);
            break;

        case 'mavlink-udp':
            await connectMAVLinkUDP(
                options.host || '127.0.0.1',
                options.port || 14550
            );
            currentConnection = { type, ...options };
            setTimeout(() => requestAllDataStreams(), 1000);
            setTimeout(() => requestHomePosition().catch(() => {}), 3000);
            break;

        case 'mavlink-tcp':
            await connectMAVLinkTCP(
                options.host || '127.0.0.1',
                options.port || 5760
            );
            currentConnection = { type, ...options };
            setTimeout(() => requestAllDataStreams(), 1000);
            setTimeout(() => requestHomePosition().catch(() => {}), 3000);
            break;

        default:
            throw new Error(`Unknown connection type: ${type}`);
    }
}

/**
 * Disconnect current connection
 */
export async function disconnect() {
    if (!currentConnection) return;

    if (currentConnection.type === 'corv-binary') {
        await window.corvSerial.disconnect();
        STATE.connected = false;
        STATE.connectionType = 'none';
    } else {
        await disconnectMAVLink();
    }
    currentConnection = null;
}

/**
 * Get available serial ports for MAVLink connection
 */
export async function getAvailablePorts() {
    return await listSerialPorts();
}

/**
 * Get current connection info
 */
export function getConnectionInfo() {
    return {
        type: currentConnection?.type || 'none',
        state: getConnectionState(),
        connected: STATE.connected
    };
}

/**
 * Check if heartbeat is alive (received within last 3 seconds)
 */
export function isHeartbeatAlive() {
    if (!STATE.connected) return false;
    if (STATE.connectionType === 'corv-binary') return true;
    return (Date.now() - STATE.lastHeartbeatTime) < 3000;
}

// Request HOME_POSITION when vehicle arms (home is set at arm time)
// and poll periodically until received + on re-arm
let _prevArmed = false;
let _homePollingInterval = null;

function startHomePolling() {
    stopHomePolling();
    // Poll every 5s until home is received, then every 30s to catch updates
    const interval = STATE.homeLat !== null ? 30000 : 5000;
    _homePollingInterval = setInterval(() => {
        const connected = (Date.now() - STATE.lastHeartbeatTime) < 3000;
        if (!connected) { stopHomePolling(); return; }
        // Speed up polling if home not yet received
        if (STATE.homeLat === null && interval > 5000) {
            startHomePolling(); // restart with faster interval
            return;
        }
        requestHomePosition().catch(() => {});
    }, interval);
}

function stopHomePolling() {
    if (_homePollingInterval) { clearInterval(_homePollingInterval); _homePollingInterval = null; }
}

onMessage(0, () => { // heartbeat
    if (STATE.armed && !_prevArmed) {
        // Just armed — request home position after a short delay, then start polling
        setTimeout(() => {
            requestHomePosition().catch(() => {});
            startHomePolling();
        }, 1500);
    }
    _prevArmed = STATE.armed;

    // If connected but no home polling yet, start it
    if (!_homePollingInterval) {
        startHomePolling();
    }
});
