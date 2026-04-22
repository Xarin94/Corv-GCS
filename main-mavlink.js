/**
 * main-mavlink.js - MAVLink handler for the Electron main process
 * Manages serial/UDP connections, MAVLink parsing, heartbeat, and IPC forwarding
 */

const { ipcMain, app } = require('electron');
const dgram = require('dgram');
const fs = require('fs');
const path = require('path');
const { PassThrough, Transform } = require('stream');

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
    protocol = new MavLinkProtocolV2(255, 190);  // sysid 255, compid 190 (MAV_COMP_ID_MISSIONPLANNER)

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

// ── TLOG recording state ──────────────────────────────────────────────────────
let tlogStream = null;
let tlogPath = null;
let tlogAutoStarted = false;  // track auto-start so we auto-stop on disconnect

function getTlogLogsDir() {
    const dir = path.join(app.getPath('userData'), 'logs');
    if (!fs.existsSync(dir)) fs.mkdirSync(dir, { recursive: true });
    return dir;
}

function startTlogRecording() {
    if (tlogStream) return { success: true, filePath: tlogPath };
    const logsDir = getTlogLogsDir();
    const now = new Date();
    const stamp = now.getFullYear()
        + '-' + String(now.getMonth() + 1).padStart(2, '0')
        + '-' + String(now.getDate()).padStart(2, '0')
        + ' ' + String(now.getHours()).padStart(2, '0')
        + '-' + String(now.getMinutes()).padStart(2, '0')
        + '-' + String(now.getSeconds()).padStart(2, '0');
    const filePath = path.join(logsDir, `${stamp}.tlog`);
    tlogStream = fs.createWriteStream(filePath);
    tlogPath = filePath;
    console.log(`[tlog] recording to ${filePath}`);
    return { success: true, filePath };
}

function stopTlogRecording() {
    if (!tlogStream) return { filePath: null };
    tlogStream.end();
    tlogStream = null;
    const p = tlogPath;
    tlogPath = null;
    tlogAutoStarted = false;
    console.log(`[tlog] recording stopped${p ? ': ' + p : ''}`);
    return { filePath: p };
}

/**
 * Write a raw MAVLink packet to the TLOG file with an 8-byte timestamp header.
 * TLOG format: [uint64 LE microseconds since Unix epoch] [raw MAVLink packet bytes]
 */
function writeTlogPacket(rawPacketBuffer) {
    if (!tlogStream) return;
    // MavLinkPacketSplitter emits { buffer, timestamp } objects in objectMode —
    // extract the raw Buffer; fall back to the value itself if already a Buffer.
    const buf = Buffer.isBuffer(rawPacketBuffer) ? rawPacketBuffer : rawPacketBuffer && rawPacketBuffer.buffer;
    if (!buf) return;
    const nowUs = BigInt(Date.now()) * 1000n;
    const tsBuf = Buffer.alloc(8);
    tsBuf.writeBigUInt64LE(nowUs, 0);
    tlogStream.write(tsBuf);
    tlogStream.write(buf);
}

/**
 * Create a Transform stream that taps raw MAVLink packets for TLOG recording.
 * Passes data through unchanged (sits between splitter and parser).
 */
function createTlogTap() {
    return new Transform({
        objectMode: true,
        transform(chunk, encoding, callback) {
            writeTlogPacket(chunk);
            callback(null, chunk);
        }
    });
}

/**
 * Initialize MAVLink IPC handlers
 * @param {BrowserWindow} win - The main browser window
 */
function initMAVLinkHandlers(win) {
    mainWindow = win;

    // TLOG recording IPC handlers
    ipcMain.handle('tlog-start-recording', async () => startTlogRecording());
    ipcMain.handle('tlog-stop-recording', async () => stopTlogRecording());
    ipcMain.handle('tlog-get-logs-dir', () => getTlogLogsDir());

    // List available serial ports
    ipcMain.handle('serial-list-ports', async () => {
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

            // Set up MAVLink parsing pipeline with TLOG tap
            const splitter = new MavLinkPacketSplitter();
            const tlogTap = createTlogTap();
            const parser = new MavLinkPacketParser();

            port.pipe(splitter).pipe(tlogTap).pipe(parser);

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
            const tlogTap = createTlogTap();
            const parser = new MavLinkPacketParser();

            passthrough.pipe(splitter).pipe(tlogTap).pipe(parser);

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
            const tlogTap = createTlogTap();
            const parser = new MavLinkPacketParser();

            passthrough.pipe(splitter).pipe(tlogTap).pipe(parser);

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

    // Connect CORV binary via serial — parse packets and emit as mavlink-message
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

            // CORV binary packet parser state
            const corvBuf = Buffer.alloc(4096);
            let corvLen = 0;

            port.on('data', (chunk) => {
                if (!mainWindow || mainWindow.isDestroyed()) return;

                // Append chunk to parser buffer
                if (corvLen + chunk.length > corvBuf.length) corvLen = 0;
                chunk.copy(corvBuf, corvLen);
                corvLen += chunk.length;

                // Process complete packets
                let again = true;
                while (again && corvLen >= 5) {
                    again = false;

                    // Find sync 0xA5 0x5A
                    let si = -1;
                    for (let i = 0; i < corvLen - 1; i++) {
                        if (corvBuf[i] === 0xA5 && corvBuf[i + 1] === 0x5A) { si = i; break; }
                    }
                    if (si > 0) { corvBuf.copyWithin(0, si, corvLen); corvLen -= si; si = 0; }
                    if (si === -1) { if (corvLen > 0) { corvBuf[0] = corvBuf[corvLen - 1]; corvLen = 1; } continue; }
                    if (corvLen < 5) continue;

                    const pType = corvBuf[2];
                    const pLen = corvBuf[3];
                    const total = 5 + pLen + 2;
                    if (corvLen < total) continue;

                    // CRC-16-CCITT check (over bytes 2..2+3+pLen-1)
                    const crcCalc = corvCRC16(corvBuf, 2, 3 + pLen);
                    const crcRecv = corvBuf[total - 2] | (corvBuf[total - 1] << 8);

                    if (crcCalc === crcRecv) {
                        const payload = Buffer.from(corvBuf.subarray(5, 5 + pLen));
                        if (pType === 0x01) corvEmitNavigation(payload);
                        else if (pType === 0x02) corvEmitDebug(payload);
                        else if (pType === 0x03) corvEmitRawSensor(payload);
                        else if (pType === 0x11) corvEmitConfigResponse(payload);
                        corvBuf.copyWithin(0, total, corvLen);
                        corvLen -= total;
                        again = true;
                    } else {
                        corvBuf.copyWithin(0, 1, corvLen);
                        corvLen -= 1;
                        again = true;
                    }
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

    // Send a raw CORV config packet (0x10) — renderer builds the full packet
    ipcMain.handle('corv-send-config', async (event, packetBytes) => {
        if (!activeConnection || activeConnection.type !== 'serial') {
            throw new Error('No active serial connection');
        }
        const buf = Buffer.from(packetBytes);
        activeConnection.port.write(buf);
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

            // TERRAIN_REQUEST (133): preserve 64-bit mask as two 32-bit halves
            // Number() loses precision above 2^53, but terrain mask uses up to 56 bits
            if (msgId === 133 && typeof instance.mask === 'bigint') {
                data.maskLow  = Number(instance.mask & BigInt(0xFFFFFFFF));
                data.maskHigh = Number((instance.mask >> BigInt(32)) & BigInt(0xFFFFFFFF));
            }
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
        case 'TERRAIN_DATA': {
            mavMsg = new common.TerrainData();
            mavMsg.lat = msg.lat;
            mavMsg.lon = msg.lon;
            mavMsg.gridSpacing = msg.gridSpacing;
            mavMsg.gridbit = msg.gridbit;
            mavMsg.data = msg.data; // int16[16]
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
        // Serialize for TLOG recording, then send
        const outBuf = protocol.serialize(msg, sequenceNumber);
        writeTlogPacket(outBuf);
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
        writeTlogPacket(buffer);
        socket.send(buffer, remote.port, remote.address);
    } else if (activeConnection.type === 'tcp') {
        const { socket } = activeConnection;
        const buffer = protocol.serialize(msg, sequenceNumber++);
        sequenceNumber = sequenceNumber & 0xFF;
        writeTlogPacket(buffer);
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
    // Auto-start/stop TLOG recording on connection state changes
    if (state === 'CONNECTED' && !tlogStream) {
        startTlogRecording();
        tlogAutoStarted = true;
    } else if (state === 'DISCONNECTED' && tlogAutoStarted) {
        stopTlogRecording();
    }

    if (mainWindow && !mainWindow.isDestroyed()) {
        mainWindow.webContents.send('mavlink-connection-state', state);
    }
}

/**
 * Cleanup on app quit
 */
function cleanup() {
    stopTlogRecording();
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

// ── CORV Binary Protocol helpers ──────────────────────────────────────────────

/**
 * CRC-16-CCITT (poly 0x1021, init 0xFFFF)
 */
function corvCRC16(buf, offset, length) {
    let crc = 0xFFFF;
    for (let i = offset; i < offset + length; i++) {
        crc ^= buf[i] << 8;
        for (let b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) & 0xFFFF : (crc << 1) & 0xFFFF;
        }
    }
    return crc;
}

/**
 * Send a synthetic mavlink-message to the renderer
 */
function corvSendMsg(msgId, data) {
    if (mainWindow && !mainWindow.isDestroyed()) {
        mainWindow.webContents.send('mavlink-message', { msgId, data, sysId: 1, compId: 1 });
    }
}

/**
 * Parse CORV Navigation packet (0x01, 104-byte payload) and emit as MAVLink messages
 */
function corvEmitNavigation(p) {
    // Attitude (offsets 4-9: int16 scaled /1000 → radians)
    const roll  = p.readInt16LE(4) / 1000.0;
    const pitch = p.readInt16LE(6) / 1000.0;
    const yaw   = p.readInt16LE(8) / 1000.0;

    // ATTITUDE (30) — radians
    corvSendMsg(30, { roll, pitch, yaw });

    // Position (offsets 10-29)
    const lat = p.readDoubleLE(10);
    const lon = p.readDoubleLE(18);
    const alt_m = p.readFloatLE(26);

    // Velocity NED (offsets 30-41) — m/s
    const vn = p.readFloatLE(30);
    const ve = p.readFloatLE(34);
    const vd = p.readFloatLE(38);

    // GLOBAL_POSITION_INT (33) — expects int7 lat/lon, mm alt, cm/s velocity
    corvSendMsg(33, {
        lat: Math.round(lat * 1e7),
        lon: Math.round(lon * 1e7),
        alt: Math.round(alt_m * 1000),
        vx: Math.round(vn * 100),
        vy: Math.round(ve * 100),
        vz: Math.round(vd * 100)
    });

    // Air data (offsets 54-69)
    const airspeed    = p.readFloatLE(54);
    const groundspeed = p.readFloatLE(58);

    // VFR_HUD (74)
    corvSendMsg(74, {
        airspeed,
        groundspeed,
        climb: -vd
    });

    // IMU (offsets 70-87: accel int16 /100 → m/s², gyro float rad/s)
    const ax = p.readInt16LE(70) / 100.0;
    const ay = p.readInt16LE(72) / 100.0;
    const az = p.readInt16LE(74) / 100.0;

    // SCALED_IMU (26) — expects mG (milliG)
    corvSendMsg(26, {
        xacc: Math.round(ax / 9.81 * 1000),
        yacc: Math.round(ay / 9.81 * 1000),
        zacc: Math.round(az / 9.81 * 1000)
    });

    // GPS quality (offsets 96-101)
    const fixType   = p.readUInt8(96);
    const numSat    = p.readUInt8(97);
    const hdop      = p.readFloatLE(98);

    // GPS_RAW_INT (24) — eph in cm
    corvSendMsg(24, {
        fixType,
        satellitesVisible: numSat,
        eph: Math.round(hdop * 100)
    });

    // Status flags (offset 102-103)
    const flags = p.readUInt16LE(102);

    // Synthetic HEARTBEAT (0) — keep UI alive, armed based on INITIALIZED flag
    corvSendMsg(0, {
        baseMode: (flags & 0x0002) ? 209 : 81,  // INITIALIZED → armed-like
        customMode: 0,
        autopilot: 0,
        type: 1  // fixed wing
    });
}

/**
 * Parse CORV Debug packet (0x02, 92-byte payload — protocol v8) and forward to renderer
 * Layout:
 *   0-3   ts (uint32)
 *   4-15  gyro bias XYZ (3 x float)
 *   16-27 accel bias XYZ (3 x float)
 *   28-31 baro bias (float)
 *   32-35 mag quality (float)
 *   36-47 hard iron XYZ (3 x float)
 *   48-55 loop/filt/sens/maxl (4 x uint16, µs)
 *   56-59 ghacc (float, m)
 *   60-63 gvacc (float, m)
 *   64    baroc (uint8)
 *   65    gpsq  (uint8)
 *   66    imuf  (uint8)
 *   67    reserved
 *   68-69 year (uint16)
 *   70-75 mo,day,hr,mn,sc,tv (6 x uint8)
 *   76-79 pf ESS (float)
 *   80-83 pf spread (float)
 *   84-85 pf resample count (uint16)
 *   86-87 pf N particles (uint16)
 *   88-91 indicated airspeed (float, m/s)
 */
function corvEmitDebug(p) {
    if (p.length < 92) return;
    const loopTime   = p.readUInt16LE(48);
    const filterTime = p.readUInt16LE(50);
    const sensorTime = p.readUInt16LE(52);
    const maxLoop    = p.readUInt16LE(54);
    console.log(`[corv] Debug v8: loop=${loopTime}us filter=${filterTime}us sens=${sensorTime}us max=${maxLoop}us`);

    if (mainWindow && !mainWindow.isDestroyed()) {
        mainWindow.webContents.send('corv-debug', Array.from(p));
    }
}

/**
 * Parse CORV Raw Sensor packet (0x03, 74-byte payload — protocol v4) and forward to renderer
 */
function corvEmitRawSensor(p) {
    if (p.length < 74) return;
    if (mainWindow && !mainWindow.isDestroyed()) {
        mainWindow.webContents.send('corv-raw-sensor', Array.from(p));
    }
}

/**
 * Forward CORV Config Response packet (0x11) payload to renderer
 */
function corvEmitConfigResponse(p) {
    if (mainWindow && !mainWindow.isDestroyed()) {
        mainWindow.webContents.send('corv-config-response', Array.from(p));
    }
}

module.exports = { initMAVLinkHandlers, cleanup, sendRawBuffer, getNextSequenceNumber, registerRawPacketCallback, isGcsOutputMuted };
