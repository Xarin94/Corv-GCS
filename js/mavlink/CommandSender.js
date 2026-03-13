/**
 * CommandSender.js - MAVLink command encoding and sending
 * Sends commands to the flight controller via IPC bridge
 */

import { STATE } from '../core/state.js';
import { getFlightModeNumber } from './MAVLinkStateMapper.js';
import { onMessage, offMessage } from './MAVLinkManager.js';

/**
 * Send a raw MAVLink command via IPC
 */
async function sendCommand(cmd) {
    if (!window.mavlink) throw new Error('MAVLink not available');
    return await window.mavlink.sendCommand(cmd);
}

/**
 * Send a MAVLink message via IPC
 */
async function sendMessage(msg) {
    if (!window.mavlink) throw new Error('MAVLink not available');
    return await window.mavlink.sendMessage(msg);
}

/**
 * ARM the vehicle
 */
export async function armVehicle() {
    return sendCommand({
        type: 'COMMAND_LONG',
        targetSystem: STATE.systemId,
        targetComponent: STATE.componentId,
        command: 400, // MAV_CMD_COMPONENT_ARM_DISARM
        param1: 1,    // 1 = arm
        param2: 0
    });
}

/**
 * DISARM the vehicle
 */
export async function disarmVehicle(force = false) {
    return sendCommand({
        type: 'COMMAND_LONG',
        targetSystem: STATE.systemId,
        targetComponent: STATE.componentId,
        command: 400, // MAV_CMD_COMPONENT_ARM_DISARM
        param1: 0,    // 0 = disarm
        param2: force ? 21196 : 0 // 21196 = force disarm
    });
}

/**
 * Set flight mode by name
 */
export async function setFlightMode(modeName) {
    const modeNum = getFlightModeNumber(modeName, STATE.vehicleType);
    if (modeNum === -1) throw new Error(`Unknown mode: ${modeName}`);
    return sendMessage({
        type: 'SET_MODE',
        targetSystem: STATE.systemId,
        baseMode: 209, // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | SAFETY_ARMED | ...
        customMode: modeNum
    });
}

/**
 * Set flight mode by number
 */
export async function setFlightModeNum(modeNum) {
    return sendMessage({
        type: 'SET_MODE',
        targetSystem: STATE.systemId,
        baseMode: 209,
        customMode: modeNum
    });
}

/**
 * Takeoff to specified altitude (AGL)
 * Switches to GUIDED mode first, then arms if needed, then sends takeoff command
 */
export async function takeoff(altitude = 10) {
    // ArduCopter requires GUIDED mode for takeoff command
    await setFlightMode('GUIDED');
    // Small delay to let mode change take effect
    await new Promise(r => setTimeout(r, 500));
    // Arm if not already armed
    if (!STATE.armed) {
        await armVehicle();
        await new Promise(r => setTimeout(r, 1000));
    }
    return sendCommand({
        type: 'COMMAND_LONG',
        targetSystem: STATE.systemId,
        targetComponent: STATE.componentId,
        command: 22, // MAV_CMD_NAV_TAKEOFF
        param1: 0,   // min pitch
        param2: 0,
        param3: 0,
        param4: 0,   // yaw angle
        param5: 0,   // lat (0 = current)
        param6: 0,   // lon (0 = current)
        param7: altitude
    });
}

/**
 * Land at current position
 */
export async function land() {
    return sendCommand({
        type: 'COMMAND_LONG',
        targetSystem: STATE.systemId,
        targetComponent: STATE.componentId,
        command: 21, // MAV_CMD_NAV_LAND
        param1: 0, param2: 0, param3: 0, param4: 0,
        param5: 0, param6: 0, param7: 0
    });
}

/**
 * Return to Launch
 */
export async function returnToLaunch() {
    return setFlightMode('RTL');
}

/**
 * Change mission speed
 */
export async function setMissionSpeed(speed, speedType = 1) {
    return sendCommand({
        type: 'COMMAND_LONG',
        targetSystem: STATE.systemId,
        targetComponent: STATE.componentId,
        command: 178, // MAV_CMD_DO_CHANGE_SPEED
        param1: speedType, // 0=airspeed, 1=groundspeed
        param2: speed,     // speed in m/s
        param3: -1,        // throttle (-1 = no change)
        param4: 0, param5: 0, param6: 0, param7: 0
    });
}

/**
 * Set home position to current location
 */
export async function setHomeCurrent() {
    return sendCommand({
        type: 'COMMAND_LONG',
        targetSystem: STATE.systemId,
        targetComponent: STATE.componentId,
        command: 179, // MAV_CMD_DO_SET_HOME
        param1: 1,    // 1 = use current position
        param2: 0, param3: 0, param4: 0,
        param5: 0, param6: 0, param7: 0
    });
}

/**
 * Request HOME_POSITION message (ID 242) from the autopilot
 */
export async function requestHomePosition() {
    return sendCommand({
        type: 'COMMAND_LONG',
        targetSystem: STATE.systemId,
        targetComponent: STATE.componentId,
        command: 512, // MAV_CMD_REQUEST_MESSAGE
        param1: 242,  // HOME_POSITION message ID
        param2: 0, param3: 0, param4: 0,
        param5: 0, param6: 0, param7: 0
    });
}

/**
 * Reboot autopilot
 */
export async function rebootAutopilot() {
    return sendCommand({
        type: 'COMMAND_LONG',
        targetSystem: STATE.systemId,
        targetComponent: STATE.componentId,
        command: 246, // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
        param1: 1,    // 1 = reboot autopilot
        param2: 0, param3: 0, param4: 0,
        param5: 0, param6: 0, param7: 0
    });
}

/**
 * Start accelerometer calibration
 */
export async function calibrateAccel() {
    return sendCommand({
        type: 'COMMAND_LONG',
        targetSystem: STATE.systemId,
        targetComponent: STATE.componentId,
        command: 241, // MAV_CMD_PREFLIGHT_CALIBRATION
        param1: 0,    // gyro
        param2: 0,    // mag
        param3: 0,    // ground pressure
        param4: 0,    // radio
        param5: 1,    // accel
        param6: 0,    // compass/motor interference
        param7: 0
    });
}

/**
 * Start compass calibration
 */
export async function calibrateCompass() {
    return sendCommand({
        type: 'COMMAND_LONG',
        targetSystem: STATE.systemId,
        targetComponent: STATE.componentId,
        command: 241, // MAV_CMD_PREFLIGHT_CALIBRATION
        param1: 0, param2: 1, param3: 0, param4: 0,
        param5: 0, param6: 0, param7: 0
    });
}

/**
 * Start gyroscope calibration
 */
export async function calibrateGyro() {
    return sendCommand({
        type: 'COMMAND_LONG',
        targetSystem: STATE.systemId,
        targetComponent: STATE.componentId,
        command: 241, // MAV_CMD_PREFLIGHT_CALIBRATION
        param1: 1, param2: 0, param3: 0, param4: 0,
        param5: 0, param6: 0, param7: 0
    });
}

/**
 * Request data stream at specified rate
 */
export async function requestDataStream(streamId, rate = 10, start = 1) {
    return sendMessage({
        type: 'REQUEST_DATA_STREAM',
        targetSystem: STATE.systemId,
        targetComponent: STATE.componentId,
        reqStreamId: streamId,
        reqMessageRate: rate,
        startStop: start
    });
}

/**
 * Request all common data streams at default rates
 */
export async function requestAllDataStreams() {
    // MAV_DATA_STREAM values
    const streams = [
        { id: 1, rate: 4 },   // RAW_SENSORS
        { id: 2, rate: 2 },   // EXTENDED_STATUS
        { id: 3, rate: 4 },   // RC_CHANNELS
        { id: 6, rate: 10 },  // POSITION
        { id: 10, rate: 25 }, // EXTRA1 (attitude) - high rate for smooth HUD
        { id: 11, rate: 10 }, // EXTRA2 (VFR_HUD)
        { id: 12, rate: 2 },  // EXTRA3
    ];
    for (const s of streams) {
        await requestDataStream(s.id, s.rate);
    }
}

/**
 * Set a single parameter with PARAM_VALUE acknowledgment
 * Retries up to 3 times if no ACK received within timeout
 */
export async function setParameter(paramId, value, paramType = 9) {
    const MAX_RETRIES = 3;
    const ACK_TIMEOUT = 2000; // ms

    for (let attempt = 1; attempt <= MAX_RETRIES; attempt++) {
        // Send PARAM_SET
        await sendMessage({
            type: 'PARAM_SET',
            targetSystem: STATE.systemId,
            targetComponent: STATE.componentId,
            paramId: paramId,
            paramValue: value,
            paramType: paramType
        });

        // Wait for PARAM_VALUE acknowledgment (msg 22)
        const ack = await new Promise((resolve) => {
            const timeout = setTimeout(() => {
                offMessage(22, handler);
                resolve(null);
            }, ACK_TIMEOUT);

            function handler(data) {
                if ((data.paramId || '').trim() === paramId.trim()) {
                    clearTimeout(timeout);
                    offMessage(22, handler);
                    resolve(data);
                }
            }
            onMessage(22, handler);
        });

        if (ack) {
            // Update local parameter cache
            STATE.parameters.set(paramId, {
                value: ack.paramValue,
                type: ack.paramType,
                index: ack.paramIndex
            });
            return ack;
        }

        console.warn(`[ParamSet] No ACK for ${paramId} (attempt ${attempt}/${MAX_RETRIES})`);
    }

    throw new Error(`Parameter ${paramId} not acknowledged after ${MAX_RETRIES} attempts`);
}

/**
 * Request all parameters
 */
export async function requestAllParameters() {
    STATE.parametersReceived = 0;
    STATE.parameters.clear();
    return sendMessage({
        type: 'PARAM_REQUEST_LIST',
        targetSystem: STATE.systemId,
        targetComponent: STATE.componentId
    });
}

/**
 * Request a single parameter by name
 */
export async function requestParameter(paramId) {
    return sendMessage({
        type: 'PARAM_REQUEST_READ',
        targetSystem: STATE.systemId,
        targetComponent: STATE.componentId,
        paramId: paramId,
        paramIndex: -1
    });
}

/**
 * Send guided target position (SET_POSITION_TARGET_GLOBAL_INT)
 * @param {number} lat - Latitude in degrees
 * @param {number} lng - Longitude in degrees
 * @param {number} alt - Altitude in meters (relative)
 */
export async function setGuidedTarget(lat, lng, alt) {
    return sendMessage({
        type: 'SET_POSITION_TARGET_GLOBAL_INT',
        targetSystem: STATE.systemId,
        targetComponent: STATE.componentId,
        coordinateFrame: 6, // MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        typeMask: 0b0000111111111000, // position only
        latInt: Math.round(lat * 1e7),
        lonInt: Math.round(lng * 1e7),
        alt: alt
    });
}

/**
 * Change altitude in GUIDED mode
 * @param {number} alt - New altitude in meters (relative)
 */
export async function changeAltitude(alt) {
    return sendCommand({
        type: 'COMMAND_LONG',
        targetSystem: STATE.systemId,
        targetComponent: STATE.componentId,
        command: 186, // MAV_CMD_DO_CHANGE_ALTITUDE  (alt via guided)
        param1: alt,
        param2: 3,   // MAV_FRAME_GLOBAL_RELATIVE_ALT
        param3: 0, param4: 0, param5: 0, param6: 0, param7: 0
    });
}

/**
 * Send RC_CHANNELS_OVERRIDE message
 * @param {number[]} channels - Array of 18 PWM values (1000-2000), 65535 = no change, 0 = release
 */
export async function sendRCChannelsOverride(channels) {
    return sendMessage({
        type: 'RC_CHANNELS_OVERRIDE',
        targetSystem: STATE.systemId,
        targetComponent: STATE.componentId,
        channels: channels
    });
}

/**
 * Send DO_SET_SERVO command
 * @param {number} servoNum - Servo number (1-based)
 * @param {number} pwm - PWM value (1000-2000)
 */
export async function sendServoTest(servoNum, pwm) {
    return sendCommand({
        type: 'COMMAND_LONG',
        targetSystem: STATE.systemId,
        targetComponent: STATE.componentId,
        command: 183, // MAV_CMD_DO_SET_SERVO
        param1: servoNum,
        param2: pwm,
        param3: 0, param4: 0, param5: 0, param6: 0, param7: 0
    });
}

/**
 * Send DO_SET_RELAY command
 * @param {number} relayNum - Relay number (0-based)
 * @param {number} state - 0=off, 1=on
 */
export async function sendRelayToggle(relayNum, state) {
    return sendCommand({
        type: 'COMMAND_LONG',
        targetSystem: STATE.systemId,
        targetComponent: STATE.componentId,
        command: 181, // MAV_CMD_DO_SET_RELAY
        param1: relayNum,
        param2: state,
        param3: 0, param4: 0, param5: 0, param6: 0, param7: 0
    });
}

/**
 * Upload mission items to autopilot using MAVLink mission protocol.
 * 1. Send MISSION_COUNT
 * 2. Autopilot sends MISSION_REQUEST_INT for each item
 * 3. Respond with MISSION_ITEM_INT
 * 4. Autopilot sends MISSION_ACK when done
 */
export async function uploadMission(items) {
    if (!items || items.length === 0) throw new Error('No mission items to upload');

    return new Promise((resolve, reject) => {
        const timeout = setTimeout(() => {
            cleanup();
            reject(new Error('Mission upload timeout'));
        }, 15000);

        // Handler for MISSION_REQUEST_INT (51) — autopilot asks for item N
        const onRequest = (data) => {
            const seq = data.seq;
            if (seq >= items.length) return;
            const item = items[seq];
            sendMessage({
                type: 'MISSION_ITEM_INT',
                targetSystem: STATE.systemId,
                targetComponent: STATE.componentId,
                seq: seq,
                frame: item.frame || 3,
                command: item.command || 16,
                current: seq === 0 ? 1 : 0,
                autocontinue: 1,
                param1: item.param1 || 0,
                param2: item.param2 || 0,
                param3: item.param3 || 0,
                param4: item.param4 || 0,
                x: Math.round((item.lat || 0) * 1e7),
                y: Math.round((item.lng || 0) * 1e7),
                z: item.alt || 0,
                missionType: 0
            }).catch(() => {});
        };

        // Handler for MISSION_REQUEST (40) — older protocol version
        const onRequestOld = onRequest;

        // Handler for MISSION_ACK (47)
        const onAck = (data) => {
            cleanup();
            if (data.type === 0) {
                resolve({ success: true, count: items.length });
            } else {
                reject(new Error(`Mission ACK error: type=${data.type}`));
            }
        };

        const cleanup = () => {
            clearTimeout(timeout);
            offMessage(51, onRequest);
            offMessage(40, onRequestOld);
            offMessage(47, onAck);
        };

        // Register handlers
        onMessage(51, onRequest);      // MISSION_REQUEST_INT
        onMessage(40, onRequestOld);   // MISSION_REQUEST
        onMessage(47, onAck);          // MISSION_ACK

        // Send MISSION_COUNT to start upload
        sendMessage({
            type: 'MISSION_COUNT',
            targetSystem: STATE.systemId,
            targetComponent: STATE.componentId,
            count: items.length,
            missionType: 0
        }).catch(e => { cleanup(); reject(e); });
    });
}
