/**
 * MAVLinkManager.js - Central MAVLink message router for the renderer process
 * Receives parsed MAVLink messages from main process via IPC and dispatches events
 */

import { STATE } from '../core/state.js';
import { mapMessageToState } from './MAVLinkStateMapper.js';

// Message handlers registry
const messageHandlers = new Map();

// Connection state
let connectionState = 'DISCONNECTED'; // DISCONNECTED, CONNECTING, CONNECTED, ACTIVE

/**
 * Initialize MAVLink manager - set up IPC listeners
 */
export function initMAVLink() {
    if (!window.mavlink) {
        console.warn('MAVLink API not available (preload bridge missing)');
        return;
    }

    // Listen for MAVLink messages from main process
    window.mavlink.onMessage((msg) => {
        handleMessage(msg);
    });

    // Listen for connection state changes
    window.mavlink.onConnectionState((state) => {
        connectionState = state;
        STATE.connectionType = state === 'DISCONNECTED' ? 'none' : STATE.connectionType;
        window.dispatchEvent(new CustomEvent('mavlinkConnectionState', { detail: { state } }));
    });
}

/**
 * Handle incoming MAVLink message
 */
function handleMessage(msg) {
    const { msgId, data, sysId, compId } = msg;

    // Update heartbeat tracking
    if (msgId === 0) {
        STATE.heartbeatCount++;
        STATE.lastHeartbeatTime = Date.now();
        STATE.systemId = sysId;
        STATE.componentId = compId;
    }

    // Map message fields to STATE
    mapMessageToState(msgId, data);

    // Call registered handlers (copy array to allow safe removal during iteration)
    const handlers = messageHandlers.get(msgId);
    if (handlers) {
        const snapshot = [...handlers];
        for (const handler of snapshot) {
            try {
                handler(data, sysId, compId);
            } catch (e) {
                console.error(`MAVLink handler error for msg ${msgId}:`, e);
            }
        }
    }

    // Dispatch global update event (mirrors serialUpdate pattern)
    window.dispatchEvent(new CustomEvent('serialUpdate'));
}

/**
 * Register a handler for a specific message ID
 */
export function onMessage(msgId, handler) {
    if (!messageHandlers.has(msgId)) {
        messageHandlers.set(msgId, []);
    }
    messageHandlers.get(msgId).push(handler);
}

/**
 * Remove a handler for a specific message ID
 */
export function offMessage(msgId, handler) {
    const handlers = messageHandlers.get(msgId);
    if (handlers) {
        const idx = handlers.indexOf(handler);
        if (idx !== -1) handlers.splice(idx, 1);
    }
}

/**
 * Connect via MAVLink serial
 */
export async function connectMAVLinkSerial(portPath, baudRate = 57600) {
    if (!window.mavlink) return;
    STATE.connectionType = 'mavlink-serial';
    connectionState = 'CONNECTING';
    try {
        await window.mavlink.connectSerial(portPath, baudRate);
        STATE.connected = true;
        STATE.mode = 'LIVE';
        connectionState = 'CONNECTED';
    } catch (e) {
        connectionState = 'DISCONNECTED';
        STATE.connectionType = 'none';
        throw e;
    }
}

/**
 * Connect via MAVLink UDP (for SITL)
 */
export async function connectMAVLinkUDP(host = '127.0.0.1', port = 14550) {
    if (!window.mavlink) return;
    STATE.connectionType = 'mavlink-udp';
    connectionState = 'CONNECTING';
    try {
        await window.mavlink.connectUDP(host, port);
        STATE.connected = true;
        STATE.mode = 'LIVE';
        connectionState = 'CONNECTED';
    } catch (e) {
        connectionState = 'DISCONNECTED';
        STATE.connectionType = 'none';
        throw e;
    }
}

export async function connectMAVLinkTCP(host = '127.0.0.1', port = 5760) {
    if (!window.mavlink) return;
    STATE.connectionType = 'mavlink-tcp';
    connectionState = 'CONNECTING';
    try {
        await window.mavlink.connectTCP(host, port);
        STATE.connected = true;
        STATE.mode = 'LIVE';
        connectionState = 'CONNECTED';
    } catch (e) {
        connectionState = 'DISCONNECTED';
        STATE.connectionType = 'none';
        throw e;
    }
}

/**
 * Disconnect MAVLink
 */
export async function disconnectMAVLink() {
    if (!window.mavlink) return;
    try {
        await window.mavlink.disconnect();
    } finally {
        connectionState = 'DISCONNECTED';
        STATE.connected = false;
        STATE.connectionType = 'none';
    }
}

/**
 * Get current connection state
 */
export function getConnectionState() {
    return connectionState;
}

/**
 * List available serial ports
 */
export async function listSerialPorts() {
    if (!window.mavlink) return [];
    return await window.mavlink.listPorts();
}
