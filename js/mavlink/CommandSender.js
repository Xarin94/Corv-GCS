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
 * @param {boolean} force - bypass arming checks (param2 = 21196)
 */
export async function armVehicle(force = false) {
    return sendCommand({
        type: 'COMMAND_LONG',
        targetSystem: STATE.systemId,
        targetComponent: STATE.componentId,
        command: 400, // MAV_CMD_COMPONENT_ARM_DISARM
        param1: 1,    // 1 = arm
        param2: force ? 21196 : 0 // 21196 = force arm (skip pre-arm checks)
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

// MAV_RESULT names, for reporting a rejected command back to the user
const MAV_RESULT_NAMES = {
    0: 'ACCEPTED', 1: 'TEMPORARILY_REJECTED', 2: 'DENIED',
    3: 'UNSUPPORTED', 4: 'FAILED', 5: 'IN_PROGRESS', 6: 'CANCELLED'
};

/**
 * Wait for the COMMAND_ACK of a given command.
 * @param {number} command - MAV_CMD id to match
 * @param {number} timeoutMs
 * @returns {Promise<{result:number, resultName:string}>}
 */
function waitForCommandAck(command, timeoutMs = 3000) {
    return new Promise((resolve, reject) => {
        let done = false;
        const finish = (fn, arg) => {
            if (done) return;
            done = true;
            clearTimeout(timer);
            offMessage(77, onAck);
            fn(arg);
        };
        const onAck = (data) => {
            if (data.command !== command) return;
            // IN_PROGRESS is a progress update, not the final answer — keep waiting
            if (data.result === 5) return;
            finish(resolve, { result: data.result, resultName: MAV_RESULT_NAMES[data.result] || `RESULT_${data.result}` });
        };
        const timer = setTimeout(() => finish(reject, new Error(`No COMMAND_ACK for command ${command} (timeout)`)), timeoutMs);
        onMessage(77, onAck); // COMMAND_ACK
    });
}

/**
 * Set the home position.
 *
 * Sent as COMMAND_INT (not COMMAND_LONG): DO_SET_HOME carries a position, and in a
 * COMMAND_LONG the lat/lon travel as float32 params, which loses several metres of
 * precision. COMMAND_INT passes them as int32 degE7 with an explicit frame.
 *
 * The ACK is awaited and the autopilot is asked for a fresh HOME_POSITION on success,
 * otherwise the GCS would keep showing the previous home and the command would look
 * like it was ignored.
 *
 * @param {object} [loc] - omit to use the vehicle's current position
 * @param {number} loc.lat - degrees
 * @param {number} loc.lon - degrees
 * @param {number} [loc.alt] - metres AMSL (0 = let the autopilot use terrain/current alt)
 * @returns {Promise<{result:number, resultName:string}>}
 */
export async function setHome(loc = null) {
    const useCurrent = !loc;
    if (!useCurrent && (!Number.isFinite(loc.lat) || !Number.isFinite(loc.lon))) {
        throw new Error('Invalid home coordinates');
    }

    await sendMessage({
        type: 'COMMAND_INT',
        targetSystem: STATE.systemId,
        targetComponent: STATE.componentId,
        frame: 0,     // MAV_FRAME_GLOBAL (alt = AMSL)
        command: 179, // MAV_CMD_DO_SET_HOME
        param1: useCurrent ? 1 : 0, // 1 = use current position
        param2: 0, param3: 0, param4: 0,
        x: useCurrent ? 0 : Math.round(loc.lat * 1e7),
        y: useCurrent ? 0 : Math.round(loc.lon * 1e7),
        z: useCurrent ? 0 : (Number.isFinite(loc.alt) ? loc.alt : 0)
    });

    const ack = await waitForCommandAck(179);
    if (ack.result === 0) {
        // Drop the stale home so the marker/map can't keep showing the old one,
        // then pull the new one from the autopilot.
        STATE.homeLat = null;
        STATE.homeLon = null;
        STATE.homeAlt = null;
        setTimeout(() => requestHomePosition().catch(() => {}), 300);
    }
    return ack;
}

/**
 * Set home position to current location
 * @returns {Promise<{result:number, resultName:string}>}
 */
export async function setHomeCurrent() {
    return setHome(null);
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
 * Request a single message at a fixed rate via MAV_CMD_SET_MESSAGE_INTERVAL (511).
 * This is the modern, reliable mechanism: ArduPilot honors it regardless of the
 * SR*_* stream parameters, whereas the deprecated REQUEST_DATA_STREAM is ignored
 * for some stream groups (notably EXTRA1/EXTRA2 → ATTITUDE/VFR_HUD) when the
 * matching SR* param is 0 — which is why those messages can be displayed by
 * another GCS yet never reach (or get recorded by) this one.
 * @param {number} msgId  MAVLink message ID
 * @param {number} rateHz Desired rate in Hz (<=0 disables the message)
 */
export async function setMessageInterval(msgId, rateHz) {
    const intervalUs = rateHz > 0 ? Math.round(1e6 / rateHz) : -1;
    return sendCommand({
        type: 'COMMAND_LONG',
        targetSystem: STATE.systemId,
        targetComponent: STATE.componentId,
        command: 511, // MAV_CMD_SET_MESSAGE_INTERVAL
        param1: msgId,
        param2: intervalUs,
        param7: 0     // 0 = respond on the requesting link
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
        { id: 6, rate: 25 },  // POSITION (GLOBAL_POSITION_INT) — matched to
                              //   EXTRA1 (attitude) so the EKF NED velocity
                              //   used for AoA/SSA refreshes at the same rate
                              //   as roll/pitch/yaw. Lower rates make the HUD
                              //   flight-path marker visibly step-jitter.
        { id: 10, rate: 25 }, // EXTRA1 (attitude) - high rate for smooth HUD
        { id: 11, rate: 10 }, // EXTRA2 (VFR_HUD)
        { id: 12, rate: 2 },  // EXTRA3
    ];
    // Per-stream try/catch so one failed request never skips the rest.
    for (const s of streams) {
        try { await requestDataStream(s.id, s.rate); } catch (e) { /* keep going */ }
    }

    // Reliable backstop: explicitly pin the per-message rates that the EXTRA
    // streams are supposed to provide. Many ArduPilot setups ignore the
    // REQUEST_DATA_STREAM for EXTRA1/EXTRA2, leaving ATTITUDE/VFR_HUD missing
    // from both the HUD horizon and the recorded tlog.
    const messages = [
        { id: 30,  rate: 25 }, // ATTITUDE   (EXTRA1) — artificial horizon
        { id: 74,  rate: 10 }, // VFR_HUD    (EXTRA2) — airspeed/altitude/climb
        { id: 31,  rate: 10 }, // ATTITUDE_QUATERNION (fallback attitude source)
        { id: 241, rate: 2 },  // VIBRATION  (EXTRA3)
    ];
    for (const m of messages) {
        try { await setMessageInterval(m.id, m.rate); } catch (e) { /* keep going */ }
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
                // ?? not || — frame=0 (MAV_FRAME_GLOBAL / absolute MSL) must not be coerced to 3
                frame: item.frame ?? 3,
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
