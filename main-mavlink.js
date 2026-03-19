/**
 * main-mavlink.js - MAVLink handler for the Electron main process
 * Manages serial/UDP connections, MAVLink parsing, heartbeat, and IPC forwarding
 */

const { ipcMain } = require('electron');
const dgram = require('dgram');
const { PassThrough } = require('stream');

// Lazy-load native modules to avoid ABI mismatch at startup
let SerialPort = null;
let MavLinkPacketSplitter, MavLinkPacketParser, MavLinkProtocolV2, send, minimal, common;
let messageRegistry = null; // Maps msgId -> message class for deserialization

function ensureMAVLinkLoaded() {
    if (MavLinkPacketSplitter) return;
    const mavlink = require('node-mavlink');
    MavLinkPacketSplitter = mavlink.MavLinkPacketSplitter;
    MavLinkPacketParser = mavlink.MavLinkPacketParser;
    MavLinkProtocolV2 = mavlink.MavLinkProtocolV2;
    send = mavlink.send;
    minimal = mavlink.minimal;
    common = mavlink.common;
    protocol = new MavLinkProtocolV2();

    // Build unified message registry from all dialects
    messageRegistry = new Map();
    if (minimal.REGISTRY) {
        for (const [id, clazz] of Object.entries(minimal.REGISTRY)) {
            messageRegistry.set(Number(id), clazz);
        }
    }
    if (common.REGISTRY) {
        for (const [id, clazz] of Object.entries(common.REGISTRY)) {
            messageRegistry.set(Number(id), clazz);
        }
    }
    // Also include ardupilotmega if available
    const ardupilotmega = mavlink.ardupilotmega;
    if (ardupilotmega && ardupilotmega.REGISTRY) {
        for (const [id, clazz] of Object.entries(ardupilotmega.REGISTRY)) {
            messageRegistry.set(Number(id), clazz);
        }
    }
}

function ensureSerialLoaded() {
    if (SerialPort) return;
    try {
        SerialPort = require('serialport').SerialPort;
    } catch (e) {
        console.error('[mavlink] Failed to load serialport:', e.message);
        console.error('[mavlink] Run: npx electron-rebuild to fix native module compatibility');
        throw new Error('serialport native module not compatible with this Electron version. Run: npx electron-rebuild');
    }
}

// Connection state
let activeConnection = null;  // { type, port/socket, reader, splitter, parser }
let heartbeatInterval = null;
let mainWindow = null;
let sequenceNumber = 0;

// MAVLink protocol instance for sending (initialized lazily)
let protocol = null;

/**
 * Initialize MAVLink IPC handlers
 * @param {BrowserWindow} win - The main browser window
 */
function initMAVLinkHandlers(win) {
    mainWindow = win;

    // List available serial ports
    ipcMain.handle('serial-list-ports', async () => {
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
            console.error('[mavlink] Failed to list ports:', e.message);
            return [];
        }
    });

    // Connect via serial
    ipcMain.handle('mavlink-connect-serial', async (event, portPath, baudRate) => {
        await disconnectCurrent();
        ensureSerialLoaded();
        ensureMAVLinkLoaded();
        try {
            const port = new SerialPort({
                path: portPath,
                baudRate: baudRate || 57600,
                autoOpen: false
            });

            await new Promise((resolve, reject) => {
                port.open((err) => {
                    if (err) reject(err);
                    else resolve();
                });
            });

            // Set up MAVLink parsing pipeline
            const splitter = new MavLinkPacketSplitter();
            const parser = new MavLinkPacketParser();

            port.pipe(splitter).pipe(parser);

            parser.on('data', (packet) => {
                handlePacket(packet);
            });

            port.on('error', (err) => {
                console.error('[mavlink] Serial error:', err.message);
                sendConnectionState('DISCONNECTED');
            });

            port.on('close', () => {
                console.log('[mavlink] Serial port closed');
                sendConnectionState('DISCONNECTED');
            });

            activeConnection = { type: 'serial', port, splitter, parser };
            startHeartbeat();
            sendConnectionState('CONNECTED');
            console.log(`[mavlink] Connected to ${portPath} at ${baudRate} baud`);
            return { success: true };
        } catch (e) {
            console.error('[mavlink] Serial connect failed:', e.message);
            throw e;
        }
    });

    // Connect via UDP
    ipcMain.handle('mavlink-connect-udp', async (event, host, port) => {
        await disconnectCurrent();
        ensureMAVLinkLoaded();
        try {
            const socket = dgram.createSocket('udp4');
            const passthrough = new PassThrough();
            const splitter = new MavLinkPacketSplitter();
            const parser = new MavLinkPacketParser();

            passthrough.pipe(splitter).pipe(parser);

            let remoteAddress = host || '127.0.0.1';
            let remotePort = port || 14550;
            let hasRemote = false;

            // Bind to listen for incoming packets
            await new Promise((resolve, reject) => {
                socket.bind(remotePort, () => {
                    console.log(`[mavlink] UDP listening on port ${remotePort}`);
                    resolve();
                });
                socket.on('error', reject);
            });

            socket.on('message', (msg, rinfo) => {
                // Track remote address for sending back
                if (!hasRemote) {
                    remoteAddress = rinfo.address;
                    remotePort = rinfo.port;
                    hasRemote = true;
                }
                passthrough.write(msg);
            });

            parser.on('data', (packet) => {
                handlePacket(packet);
            });

            socket.on('error', (err) => {
                console.error('[mavlink] UDP error:', err.message);
            });

            activeConnection = {
                type: 'udp',
                socket,
                passthrough,
                splitter,
                parser,
                getRemote: () => ({ address: remoteAddress, port: remotePort }),
                hasRemote: () => hasRemote
            };
            startHeartbeat();
            sendConnectionState('CONNECTED');
            console.log(`[mavlink] UDP connected to ${host}:${port}`);
            return { success: true };
        } catch (e) {
            console.error('[mavlink] UDP connect failed:', e.message);
            throw e;
        }
    });

    // Connect via TCP (used for SITL on WSL which exposes TCP 5760)
    ipcMain.handle('mavlink-connect-tcp', async (event, host, port) => {
        await disconnectCurrent();
        ensureMAVLinkLoaded();
        const net = require('net');
        try {
            const tcpHost = host || '127.0.0.1';
            const tcpPort = port || 5760;

            let socket = new net.Socket();
            const passthrough = new PassThrough();
            const splitter = new MavLinkPacketSplitter();
            const parser = new MavLinkPacketParser();

            passthrough.pipe(splitter).pipe(parser);

            // Retry TCP connection up to 6 times (SITL may need time to bind)
            for (let attempt = 1; attempt <= 6; attempt++) {
                try {
                    await new Promise((resolve, reject) => {
                        socket.connect(tcpPort, tcpHost, () => {
                            console.log(`[mavlink] TCP connected to ${tcpHost}:${tcpPort}`);
                            resolve();
                        });
                        socket.once('error', reject);
                    });
                    break;
                } catch (err) {
                    console.log(`[mavlink] TCP attempt ${attempt}/6 failed: ${err.message}`);
                    if (attempt === 6) throw err;
                    socket.destroy();
                    await new Promise(r => setTimeout(r, 2000));
                    socket = new net.Socket();
                }
            }

            socket.on('data', (data) => {
                passthrough.write(data);
            });

            parser.on('data', (packet) => {
                handlePacket(packet);
            });

            socket.on('error', (err) => {
                console.error('[mavlink] TCP error:', err.message);
            });

            socket.on('close', () => {
                console.log('[mavlink] TCP connection closed');
                sendConnectionState('DISCONNECTED');
            });

            activeConnection = {
                type: 'tcp',
                socket,
                passthrough,
                splitter,
                parser,
                getRemote: () => ({ address: tcpHost, port: tcpPort }),
                hasRemote: () => true
            };
            sendConnectionState('CONNECTED');
            // Delay heartbeat start for TCP - give SITL time to finish initialization
            setTimeout(() => startHeartbeat(), 2000);
            return { success: true };
        } catch (e) {
            console.error('[mavlink] TCP connect failed:', e.message);
            throw e;
        }
    });

    // Connect CORV binary via serial (raw data forwarded to renderer)
    ipcMain.handle('corv-connect-serial', async (event, portPath, baudRate) => {
        await disconnectCurrent();
        ensureSerialLoaded();
        try {
            const port = new SerialPort({
                path: portPath,
                baudRate: baudRate || 460800,
                autoOpen: false
            });

            await new Promise((resolve, reject) => {
                port.open((err) => {
                    if (err) reject(err);
                    else resolve();
                });
            });

            port.on('data', (data) => {
                if (mainWindow && !mainWindow.isDestroyed()) {
                    // Convert Buffer to Uint8Array for clean IPC serialization through contextBridge
                    const bytes = new Uint8Array(data.buffer, data.byteOffset, data.byteLength);
                    mainWindow.webContents.send('corv-serial-data', bytes);
                }
            });

            port.on('error', (err) => {
                console.error('[corv] Serial error:', err.message);
                sendConnectionState('DISCONNECTED');
            });

            port.on('close', () => {
                console.log('[corv] Serial port closed');
                sendConnectionState('DISCONNECTED');
            });

            activeConnection = { type: 'serial', port };
            sendConnectionState('CONNECTED');
            console.log(`[corv] Connected to ${portPath} at ${baudRate} baud`);
            return { success: true };
        } catch (e) {
            console.error('[corv] Serial connect failed:', e.message);
            throw e;
        }
    });

    // Disconnect
    ipcMain.handle('mavlink-disconnect', async () => {
        await disconnectCurrent();
        return { success: true };
    });

    // Send a MAVLink command (COMMAND_LONG)
    ipcMain.handle('mavlink-send-command', async (event, cmd) => {
        if (!activeConnection) throw new Error('Not connected');
        try {
            await sendMAVLinkCommand(cmd);
            return { success: true };
        } catch (e) {
            console.error('[mavlink] Send command failed:', e.message);
            throw e;
        }
    });

    // Send a MAVLink message
    ipcMain.handle('mavlink-send-message', async (event, msg) => {
        if (!activeConnection) throw new Error('Not connected');
        try {
            await sendMAVLinkMessage(msg);
            return { success: true };
        } catch (e) {
            console.error('[mavlink] Send message failed:', e.message);
            throw e;
        }
    });

    // Toggle GCS output mute (suppress all outgoing messages)
    ipcMain.handle('mavlink-set-gcs-muted', async (event, muted) => {
        gcsOutputMuted = !!muted;
        console.log('[mavlink] GCS output', gcsOutputMuted ? 'MUTED' : 'UNMUTED');
        return { muted: gcsOutputMuted };
    });
}

/**
 * Handle a parsed MAVLink packet
 */
function handlePacket(packet) {
    if (!mainWindow || mainWindow.isDestroyed()) return;
    if (!packet.header || !packet.payload) return;

    // Forward raw packet to telemetry forwarder (if registered)
    if (rawPacketCallback) rawPacketCallback(packet);

    const msgId = packet.header.msgid;

    try {
        // Look up message class from registry
        const MessageClass = messageRegistry ? messageRegistry.get(msgId) : null;
        let data = {};

        if (MessageClass && packet.protocol) {
            // Deserialize payload into typed message instance
            const instance = packet.protocol.data(packet.payload, MessageClass);
            // Convert to plain object, resolving enums to numbers and BigInts
            data = instanceToPlainObject(instance, MessageClass);
        }

        mainWindow.webContents.send('mavlink-message', {
            msgId,
            data,
            sysId: packet.header.sysid,
            compId: packet.header.compid
        });
    } catch (e) {
        console.error(`[mavlink] Packet parse error for msg ${msgId}:`, e.message);
        // For unknown/malformed messages, send with empty data
        mainWindow.webContents.send('mavlink-message', {
            msgId,
            data: {},
            sysId: packet.header.sysid,
            compId: packet.header.compid
        });
    }
}

/**
 * Convert a deserialized MAVLink message instance to a plain object,
 * resolving enum strings to their numeric values and BigInt to Number.
 */
function instanceToPlainObject(instance, MessageClass) {
    const result = {};
    const fields = MessageClass.FIELDS || [];

    for (const field of fields) {
        let value = instance[field.name];
        if (typeof value === 'bigint') {
            value = Number(value);
        } else if (typeof value === 'string' && !field.type.startsWith('char')) {
            // Enum field serialized as string name - try to resolve to number
            // node-mavlink stores enum values as the string key name
            // We need to find the numeric value from the enum lookup
            value = resolveEnumValue(value);
        }
        result[field.name] = value;
    }
    return result;
}

/**
 * Resolve a string enum value to its numeric equivalent.
 * Searches known MAVLink enum objects for the string key.
 */
const _enumCache = new Map();
function resolveEnumValue(str) {
    if (_enumCache.has(str)) return _enumCache.get(str);

    // Search through all dialect enum objects
    const dialects = [minimal, common];
    try {
        const ardu = require('node-mavlink').ardupilotmega;
        if (ardu) dialects.push(ardu);
    } catch (e) { /* ignore */ }

    for (const dialect of dialects) {
        for (const key of Object.keys(dialect)) {
            const obj = dialect[key];
            if (obj && typeof obj === 'object' && !Array.isArray(obj) && obj[str] !== undefined) {
                const numVal = obj[str];
                if (typeof numVal === 'number') {
                    _enumCache.set(str, numVal);
                    return numVal;
                }
            }
        }
    }
    // If we can't resolve, return the string as-is
    _enumCache.set(str, str);
    return str;
}

/**
 * Send a COMMAND_LONG message
 */
async function sendMAVLinkCommand(cmd) {
    const msg = new common.CommandLong();
    msg.targetSystem = cmd.targetSystem || 1;
    msg.targetComponent = cmd.targetComponent || 1;
    msg.command = cmd.command;
    msg.confirmation = cmd.confirmation || 0;
    msg._param1 = cmd.param1 || 0;
    msg._param2 = cmd.param2 || 0;
    msg._param3 = cmd.param3 || 0;
    msg._param4 = cmd.param4 || 0;
    msg._param5 = cmd.param5 || 0;
    msg._param6 = cmd.param6 || 0;
    msg._param7 = cmd.param7 || 0;
    await sendToConnection(msg);
}

/**
 * Send a generic MAVLink message
 */
async function sendMAVLinkMessage(msg) {
    let mavMsg;

    switch (msg.type) {
        case 'SET_MODE': {
            mavMsg = new common.SetMode();
            mavMsg.targetSystem = msg.targetSystem || 1;
            mavMsg.baseMode = msg.baseMode || 209;
            mavMsg.customMode = msg.customMode || 0;
            break;
        }
        case 'PARAM_SET': {
            mavMsg = new common.ParamSet();
            mavMsg.targetSystem = msg.targetSystem || 1;
            mavMsg.targetComponent = msg.targetComponent || 1;
            mavMsg.paramId = msg.paramId;
            mavMsg.paramValue = msg.paramValue;
            mavMsg.paramType = msg.paramType || 9;
            console.log(`[mavlink] PARAM_SET: ${msg.paramId} = ${msg.paramValue} (type=${mavMsg.paramType}, target=${mavMsg.targetSystem}/${mavMsg.targetComponent})`);
            break;
        }
        case 'PARAM_REQUEST_LIST': {
            mavMsg = new common.ParamRequestList();
            mavMsg.targetSystem = msg.targetSystem || 1;
            mavMsg.targetComponent = msg.targetComponent || 1;
            break;
        }
        case 'PARAM_REQUEST_READ': {
            mavMsg = new common.ParamRequestRead();
            mavMsg.targetSystem = msg.targetSystem || 1;
            mavMsg.targetComponent = msg.targetComponent || 1;
            mavMsg.paramId = msg.paramId || '';
            mavMsg.paramIndex = msg.paramIndex !== undefined ? msg.paramIndex : -1;
            break;
        }
        case 'MISSION_REQUEST_LIST': {
            mavMsg = new common.MissionRequestList();
            mavMsg.targetSystem = msg.targetSystem || 1;
            mavMsg.targetComponent = msg.targetComponent || 1;
            mavMsg.missionType = msg.missionType || 0;
            break;
        }
        case 'MISSION_COUNT': {
            mavMsg = new common.MissionCount();
            mavMsg.targetSystem = msg.targetSystem || 1;
            mavMsg.targetComponent = msg.targetComponent || 1;
            mavMsg.count = msg.count || 0;
            mavMsg.missionType = msg.missionType || 0;
            break;
        }
        case 'MISSION_ITEM_INT': {
            mavMsg = new common.MissionItemInt();
            mavMsg.targetSystem = msg.targetSystem || 1;
            mavMsg.targetComponent = msg.targetComponent || 1;
            mavMsg.seq = msg.seq || 0;
            mavMsg.frame = msg.frame || 3; // MAV_FRAME_GLOBAL_RELATIVE_ALT
            mavMsg.command = msg.command || 16; // MAV_CMD_NAV_WAYPOINT
            mavMsg.current = msg.current || 0;
            mavMsg.autocontinue = msg.autocontinue !== undefined ? msg.autocontinue : 1;
            mavMsg.param1 = msg.param1 || 0;
            mavMsg.param2 = msg.param2 || 0;
            mavMsg.param3 = msg.param3 || 0;
            mavMsg.param4 = msg.param4 || 0;
            mavMsg.x = msg.x || 0;
            mavMsg.y = msg.y || 0;
            mavMsg.z = msg.z || 0;
            mavMsg.missionType = msg.missionType || 0;
            break;
        }
        case 'MISSION_ACK': {
            mavMsg = new common.MissionAck();
            mavMsg.targetSystem = msg.targetSystem || 1;
            mavMsg.targetComponent = msg.targetComponent || 1;
            mavMsg.type = msg.ackType || 0;
            mavMsg.missionType = msg.missionType || 0;
            break;
        }
        case 'REQUEST_DATA_STREAM': {
            mavMsg = new common.RequestDataStream();
            mavMsg.targetSystem = msg.targetSystem || 1;
            mavMsg.targetComponent = msg.targetComponent || 1;
            mavMsg.reqStreamId = msg.reqStreamId || 0;
            mavMsg.reqMessageRate = msg.reqMessageRate || 10;
            mavMsg.startStop = msg.startStop !== undefined ? msg.startStop : 1;
            console.log(`[mavlink] REQUEST_DATA_STREAM: stream=${mavMsg.reqStreamId} rate=${mavMsg.reqMessageRate}Hz start=${mavMsg.startStop}`);
            break;
        }
        case 'RC_CHANNELS_OVERRIDE': {
            mavMsg = new common.RcChannelsOverride();
            mavMsg.targetSystem = msg.targetSystem || 1;
            mavMsg.targetComponent = msg.targetComponent || 1;
            const ch = msg.channels || [];
            mavMsg.chan1Raw  = ch[0]  || 0;
            mavMsg.chan2Raw  = ch[1]  || 0;
            mavMsg.chan3Raw  = ch[2]  || 0;
            mavMsg.chan4Raw  = ch[3]  || 0;
            mavMsg.chan5Raw  = ch[4]  || 0;
            mavMsg.chan6Raw  = ch[5]  || 0;
            mavMsg.chan7Raw  = ch[6]  || 0;
            mavMsg.chan8Raw  = ch[7]  || 0;
            mavMsg.chan9Raw  = ch[8]  || 0;
            mavMsg.chan10Raw = ch[9]  || 0;
            mavMsg.chan11Raw = ch[10] || 0;
            mavMsg.chan12Raw = ch[11] || 0;
            mavMsg.chan13Raw = ch[12] || 0;
            mavMsg.chan14Raw = ch[13] || 0;
            mavMsg.chan15Raw = ch[14] || 0;
            mavMsg.chan16Raw = ch[15] || 0;
            mavMsg.chan17Raw = ch[16] || 0;
            mavMsg.chan18Raw = ch[17] || 0;
            break;
        }
        case 'SET_POSITION_TARGET_GLOBAL_INT': {
            mavMsg = new common.SetPositionTargetGlobalInt();
            mavMsg.targetSystem = msg.targetSystem || 1;
            mavMsg.targetComponent = msg.targetComponent || 1;
            mavMsg.timeBootMs = 0;
            mavMsg.coordinateFrame = msg.coordinateFrame || 6;
            mavMsg.typeMask = msg.typeMask || 0b0000111111111000;
            mavMsg.latInt = msg.latInt || 0;
            mavMsg.lonInt = msg.lonInt || 0;
            mavMsg.alt = msg.alt || 0;
            mavMsg.vx = 0; mavMsg.vy = 0; mavMsg.vz = 0;
            mavMsg.afx = 0; mavMsg.afy = 0; mavMsg.afz = 0;
            mavMsg.yaw = 0; mavMsg.yawRate = 0;
            break;
        }
        case 'COMMAND_LONG': {
            await sendMAVLinkCommand(msg);
            return;
        }
        default:
            throw new Error(`Unknown message type: ${msg.type}`);
    }

    await sendToConnection(mavMsg);
}

/**
 * Send a MAVLink message to the active connection
 */
async function sendToConnection(msg) {
    if (gcsOutputMuted) return;
    if (!activeConnection) throw new Error('No active connection');

    if (activeConnection.type === 'serial') {
        await send(activeConnection.port, msg, protocol);
    } else if (activeConnection.type === 'udp') {
        const { socket, getRemote, hasRemote } = activeConnection;
        if (!hasRemote()) {
            console.warn('[mavlink] No remote endpoint yet for UDP');
            return;
        }
        const remote = getRemote();
        const buffer = protocol.serialize(msg, sequenceNumber++);
        sequenceNumber = sequenceNumber & 0xFF;
        socket.send(buffer, remote.port, remote.address);
    } else if (activeConnection.type === 'tcp') {
        const { socket } = activeConnection;
        const buffer = protocol.serialize(msg, sequenceNumber++);
        sequenceNumber = sequenceNumber & 0xFF;
        socket.write(buffer);
    }
}

/**
 * Start heartbeat timer (1 Hz)
 */
function startHeartbeat() {
    stopHeartbeat();
    heartbeatInterval = setInterval(() => {
        if (!activeConnection) return;
        try {
            const hb = new minimal.Heartbeat();
            hb.type = 6;          // MAV_TYPE_GCS
            hb.autopilot = 8;     // MAV_AUTOPILOT_INVALID
            hb.baseMode = 0;
            hb.customMode = 0;
            hb.systemStatus = 4;  // MAV_STATE_ACTIVE
            hb.mavlinkVersion = 3;
            sendToConnection(hb).catch(() => {});
        } catch (e) {
            // Ignore heartbeat send errors
        }
    }, 1000);
}

/**
 * Stop heartbeat timer
 */
function stopHeartbeat() {
    if (heartbeatInterval) {
        clearInterval(heartbeatInterval);
        heartbeatInterval = null;
    }
}

/**
 * Disconnect current connection
 */
async function disconnectCurrent() {
    stopHeartbeat();
    if (!activeConnection) return;

    try {
        if (activeConnection.type === 'serial') {
            const port = activeConnection.port;
            if (port.isOpen) {
                await new Promise((resolve) => port.close(resolve));
            }
        } else if (activeConnection.type === 'udp') {
            activeConnection.socket.close();
            activeConnection.passthrough.destroy();
        } else if (activeConnection.type === 'tcp') {
            activeConnection.socket.destroy();
            activeConnection.passthrough.destroy();
        }
    } catch (e) {
        console.error('[mavlink] Disconnect error:', e.message);
    }

    activeConnection = null;
    sendConnectionState('DISCONNECTED');
    console.log('[mavlink] Disconnected');
}

/**
 * Send connection state to renderer
 */
function sendConnectionState(state) {
    if (mainWindow && !mainWindow.isDestroyed()) {
        mainWindow.webContents.send('mavlink-connection-state', state);
    }
}

/**
 * Cleanup on app quit
 */
function cleanup() {
    stopHeartbeat();
    if (activeConnection) {
        try {
            if (activeConnection.type === 'serial' && activeConnection.port.isOpen) {
                activeConnection.port.close();
            } else if (activeConnection.type === 'udp') {
                activeConnection.socket.close();
            } else if (activeConnection.type === 'tcp') {
                activeConnection.socket.destroy();
            }
        } catch (e) {
            // Ignore
        }
        activeConnection = null;
    }
}

/**
 * Send raw bytes to the active MAVLink connection (used by RTK for RTCM injection)
 */
function sendRawBuffer(buffer) {
    if (gcsOutputMuted) return;
    if (!activeConnection) return;
    if (activeConnection.type === 'serial') {
        if (activeConnection.port && activeConnection.port.isOpen) {
            activeConnection.port.write(buffer);
        }
    } else if (activeConnection.type === 'udp') {
        const { socket, getRemote, hasRemote } = activeConnection;
        if (hasRemote()) {
            const remote = getRemote();
            socket.send(buffer, remote.port, remote.address);
        }
    } else if (activeConnection.type === 'tcp') {
        if (activeConnection.socket) {
            activeConnection.socket.write(buffer);
        }
    }
}

/**
 * Get the current sequence number and increment it
 */
function getNextSequenceNumber() {
    const seq = sequenceNumber;
    sequenceNumber = (sequenceNumber + 1) & 0xFF;
    return seq;
}

// Raw packet callback for telemetry forwarding (MAVLink passthrough)
let rawPacketCallback = null;
function registerRawPacketCallback(cb) { rawPacketCallback = typeof cb === 'function' ? cb : null; }

// GCS output mute flag — when true, suppress ALL outgoing messages (heartbeat, RTK, RC override, commands)
let gcsOutputMuted = false;
function isGcsOutputMuted() { return gcsOutputMuted; }

module.exports = { initMAVLinkHandlers, cleanup, sendRawBuffer, getNextSequenceNumber, registerRawPacketCallback, isGcsOutputMuted };
