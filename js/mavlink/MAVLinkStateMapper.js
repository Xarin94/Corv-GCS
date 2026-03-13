/**
 * MAVLinkStateMapper.js - Maps MAVLink messages to STATE object fields
 * Translates MAVLink message data into the application's STATE format
 */

import { STATE } from '../core/state.js';

// ArduPilot Copter flight mode mapping (custom_mode -> name)
const ARDUPILOT_COPTER_MODES = {
    0: 'STABILIZE', 1: 'ACRO', 2: 'ALT_HOLD', 3: 'AUTO',
    4: 'GUIDED', 5: 'LOITER', 6: 'RTL', 7: 'CIRCLE',
    9: 'LAND', 11: 'DRIFT', 13: 'SPORT', 14: 'FLIP',
    15: 'AUTOTUNE', 16: 'POSHOLD', 17: 'BRAKE', 18: 'THROW',
    19: 'AVOID_ADSB', 20: 'GUIDED_NOGPS', 21: 'SMART_RTL',
    22: 'FLOWHOLD', 23: 'FOLLOW', 24: 'ZIGZAG', 25: 'SYSTEMID',
    26: 'AUTOROTATE', 27: 'AUTO_RTL'
};

// ArduPilot Plane flight mode mapping
const ARDUPILOT_PLANE_MODES = {
    0: 'MANUAL', 1: 'CIRCLE', 2: 'STABILIZE', 3: 'TRAINING',
    4: 'ACRO', 5: 'FBWA', 6: 'FBWB', 7: 'CRUISE',
    8: 'AUTOTUNE', 10: 'AUTO', 11: 'RTL', 12: 'LOITER',
    13: 'TAKEOFF', 14: 'AVOID_ADSB', 15: 'GUIDED',
    17: 'QSTABILIZE', 18: 'QHOVER', 19: 'QLOITER',
    20: 'QLAND', 21: 'QRTL', 22: 'QAUTOTUNE', 23: 'QACRO',
    24: 'THERMAL', 25: 'LOITER_ALT_QLAND'
};

// ArduPilot Rover flight mode mapping
const ARDUPILOT_ROVER_MODES = {
    0: 'MANUAL', 1: 'ACRO', 3: 'STEERING', 4: 'HOLD',
    5: 'LOITER', 6: 'FOLLOW', 7: 'SIMPLE',
    10: 'AUTO', 11: 'RTL', 12: 'SMART_RTL',
    15: 'GUIDED'
};

// ArduPilot Sub flight mode mapping
const ARDUPILOT_SUB_MODES = {
    0: 'STABILIZE', 1: 'ACRO', 2: 'ALT_HOLD',
    3: 'AUTO', 4: 'GUIDED', 7: 'CIRCLE',
    9: 'SURFACE', 16: 'POSHOLD', 19: 'MANUAL'
};

// MAV_TYPE to vehicle category
// 1=plane, 2=quad, 3=coaxial, 4=heli, 10=rover, 11=boat, 12=sub,
// 13=hex, 14=octo, 15=tricopter, 20=quad(VTOL), 21=tiltrotor
const ROVER_TYPES = [10, 11]; // ground rover, boat
const SUB_TYPES = [12]; // submarine
const PLANE_TYPES = [1, 20, 21]; // fixed wing, VTOL variants

// GPS fix type names
const GPS_FIX_TYPES = {
    0: 'No GPS', 1: 'No Fix', 2: '2D Fix', 3: '3D Fix',
    4: 'DGPS', 5: 'RTK Float', 6: 'RTK Fixed'
};

/**
 * Get flight mode name from heartbeat data
 */
export function getFlightModeName(customMode, vehicleType) {
    const modes = getModesForType(vehicleType);
    return modes[customMode] || `MODE_${customMode}`;
}

/**
 * Get flight mode number from name (for sending SET_MODE)
 */
export function getFlightModeNumber(modeName, vehicleType) {
    const modes = getModesForType(vehicleType);
    for (const [num, name] of Object.entries(modes)) {
        if (name === modeName) return parseInt(num);
    }
    return -1;
}

/**
 * Get mode map for a given vehicle type
 */
function getModesForType(vehicleType) {
    if (PLANE_TYPES.includes(vehicleType)) return ARDUPILOT_PLANE_MODES;
    if (ROVER_TYPES.includes(vehicleType)) return ARDUPILOT_ROVER_MODES;
    if (SUB_TYPES.includes(vehicleType)) return ARDUPILOT_SUB_MODES;
    return ARDUPILOT_COPTER_MODES;
}

/**
 * Get GPS fix type name
 */
export function getGPSFixName(fixType) {
    return GPS_FIX_TYPES[fixType] || `Fix ${fixType}`;
}

/**
 * Get available flight modes for current vehicle type
 */
export function getAvailableFlightModes() {
    const modes = getModesForType(STATE.vehicleType);
    return Object.entries(modes).map(([num, name]) => ({ num: parseInt(num), name }));
}

/**
 * Get vehicle type category name
 */
export function getVehicleTypeName(vehicleType) {
    if (PLANE_TYPES.includes(vehicleType)) return 'Plane';
    if (ROVER_TYPES.includes(vehicleType)) return 'Rover';
    if (SUB_TYPES.includes(vehicleType)) return 'Sub';
    return 'Copter';
}

/**
 * Map a MAVLink message to STATE fields
 * @param {number} msgId - MAVLink message ID
 * @param {object} data - Parsed message fields
 */
export function mapMessageToState(msgId, data) {
    switch (msgId) {
        case 0: mapHeartbeat(data); break;
        case 1: mapSysStatus(data); break;
        case 24: mapGpsRawInt(data); break;
        case 26: mapScaledImu(data); break;
        case 30: mapAttitude(data); break;
        case 33: mapGlobalPositionInt(data); break;
        case 42: mapMissionCurrent(data); break;
        case 35: mapRcChannelsRaw(data); break;
        case 36: mapServoOutputRaw(data); break;
        case 65: mapRcChannels(data); break;
        case 74: mapVfrHud(data); break;
        case 77: mapCommandAck(data); break;
        case 127: mapGpsRtk(data); break;
        case 132: mapDistanceSensor(data); break;
        case 241: mapVibration(data); break;
        case 242: mapHomePosition(data); break;
        case 253: mapStatusText(data); break;
    }
}

function mapHeartbeat(data) {
    STATE.baseMode = data.baseMode;
    STATE.customMode = data.customMode;
    STATE.autopilotType = data.autopilot;
    STATE.vehicleType = data.type;
    STATE.armed = (data.baseMode & 128) !== 0; // MAV_MODE_FLAG_SAFETY_ARMED = 128
    STATE.flightModeNum = data.customMode;
    STATE.flightMode = getFlightModeName(data.customMode, data.type);
}

function mapSysStatus(data) {
    STATE.batteryVoltage = data.voltageBattery / 1000; // mV -> V
    STATE.batteryCurrent = data.currentBattery / 100;  // cA -> A
    STATE.batteryRemaining = data.batteryRemaining;     // 0-100%
    STATE.linkQuality = 100 - data.dropRateComm / 100; // percent
}

function mapGpsRawInt(data) {
    STATE.gpsFix = data.fixType;
    STATE.gpsNumSat = data.satellitesVisible;
    STATE.gpsHdop = data.eph / 100; // cm -> m (HDOP * 100)
}

function mapScaledImu(data) {
    // ScaledImu: mG -> m/s^2 (divide by 1000 * 9.81, but already in mG so /1000*9.81)
    STATE.ax = data.xacc / 1000 * 9.81;
    STATE.ay = data.yacc / 1000 * 9.81;
    STATE.az = data.zacc / 1000 * 9.81;
}

function mapAttitude(data) {
    STATE.roll = data.roll;   // already radians
    STATE.pitch = data.pitch;
    STATE.yaw = data.yaw;
}

function mapGlobalPositionInt(data) {
    const lat = data.lat / 1e7;
    const lon = data.lon / 1e7;
    if (Math.abs(lat) > 0.1 || Math.abs(lon) > 0.1) {
        STATE.lat = lat;
        STATE.lon = lon;
    }
    STATE.rawAlt = data.alt / 1000; // mm -> m
    STATE.vs = -data.vz / 100;      // cm/s -> m/s, NED to Up

    // Store NED velocity for AoA/SSA computation
    STATE.vn = (data.vx || 0) / 100; // cm/s -> m/s
    STATE.ve = (data.vy || 0) / 100;
    STATE.vd = (data.vz || 0) / 100;

    // Compute flight path angle (gamma) and track
    const vHoriz = Math.sqrt(STATE.vn * STATE.vn + STATE.ve * STATE.ve);
    STATE.gamma = Math.atan2(-STATE.vd, vHoriz); // positive = climbing
    STATE.track = Math.atan2(STATE.ve, STATE.vn); // NED track angle

    // Compute AoA and SSA by rotating NED velocity into body frame
    computeAeroAngles();
}

/**
 * Rotate NED velocity to body frame and compute AoA (alpha) and SSA (beta)
 * Body frame: X = forward, Y = right, Z = down
 */
function computeAeroAngles() {
    const cr = Math.cos(STATE.roll),  sr = Math.sin(STATE.roll);
    const cp = Math.cos(STATE.pitch), sp = Math.sin(STATE.pitch);
    const cy = Math.cos(STATE.yaw),   sy = Math.sin(STATE.yaw);

    // NED to body rotation (ZYX Euler: yaw -> pitch -> roll)
    const vn = STATE.vn, ve = STATE.ve, vd = STATE.vd;

    // Body-frame velocity components
    const Vbx = cp * cy * vn + cp * sy * ve - sp * vd;
    const Vby = (sr * sp * cy - cr * sy) * vn + (sr * sp * sy + cr * cy) * ve + sr * cp * vd;
    const Vbz = (cr * sp * cy + sr * sy) * vn + (cr * sp * sy - sr * cy) * ve + cr * cp * vd;

    // Only compute when there's meaningful forward speed
    if (Math.abs(Vbx) > 0.5) {
        STATE.aoa = Math.atan2(Vbz, Vbx);  // alpha: positive = nose above flight path
        STATE.ssa = Math.atan2(Vby, Vbx);   // beta: positive = wind from right
    } else {
        STATE.aoa = 0;
        STATE.ssa = 0;
    }
}

function mapMissionCurrent(data) {
    STATE.missionCurrentSeq = data.seq;
    if (data.total !== undefined) STATE.missionCount = data.total;
}

function mapRcChannelsRaw(data) {
    // RC_CHANNELS_RAW (msg 35) - 8 channels only, older protocol
    STATE.rcChannels[0] = data.chan1Raw || 0;
    STATE.rcChannels[1] = data.chan2Raw || 0;
    STATE.rcChannels[2] = data.chan3Raw || 0;
    STATE.rcChannels[3] = data.chan4Raw || 0;
    STATE.rcChannels[4] = data.chan5Raw || 0;
    STATE.rcChannels[5] = data.chan6Raw || 0;
    STATE.rcChannels[6] = data.chan7Raw || 0;
    STATE.rcChannels[7] = data.chan8Raw || 0;
}

function mapRcChannels(data) {
    // RC_CHANNELS (msg 65) - 18 channels, preferred
    STATE.rcChannels[0] = data.chan1Raw || 0;
    STATE.rcChannels[1] = data.chan2Raw || 0;
    STATE.rcChannels[2] = data.chan3Raw || 0;
    STATE.rcChannels[3] = data.chan4Raw || 0;
    STATE.rcChannels[4] = data.chan5Raw || 0;
    STATE.rcChannels[5] = data.chan6Raw || 0;
    STATE.rcChannels[6] = data.chan7Raw || 0;
    STATE.rcChannels[7] = data.chan8Raw || 0;
    STATE.rcChannels[8] = data.chan9Raw || 0;
    STATE.rcChannels[9] = data.chan10Raw || 0;
    STATE.rcChannels[10] = data.chan11Raw || 0;
    STATE.rcChannels[11] = data.chan12Raw || 0;
    STATE.rcChannels[12] = data.chan13Raw || 0;
    STATE.rcChannels[13] = data.chan14Raw || 0;
    STATE.rcChannels[14] = data.chan15Raw || 0;
    STATE.rcChannels[15] = data.chan16Raw || 0;
    STATE.rcChannels[16] = data.chan17Raw || 0;
    STATE.rcChannels[17] = data.chan18Raw || 0;
}

function mapVfrHud(data) {
    STATE.as = data.airspeed;
    STATE.gs = data.groundspeed;
    // VfrHud climb overrides GlobalPositionInt vs if available
    if (data.climb !== undefined) STATE.vs = data.climb;
}

function mapVibration(data) {
    STATE.vibX = data.vibrationX || 0;
    STATE.vibY = data.vibrationY || 0;
    STATE.vibZ = data.vibrationZ || 0;
    STATE.vibClip0 = data.clipping0 || 0;
    STATE.vibClip1 = data.clipping1 || 0;
    STATE.vibClip2 = data.clipping2 || 0;
    // Keep last 120 samples (~2 min at 1Hz)
    STATE.vibHistory.push({ x: STATE.vibX, y: STATE.vibY, z: STATE.vibZ, t: Date.now() });
    if (STATE.vibHistory.length > 120) STATE.vibHistory.shift();
}

function mapServoOutputRaw(data) {
    STATE.servoOutputs[0] = data.servo1Raw || 0;
    STATE.servoOutputs[1] = data.servo2Raw || 0;
    STATE.servoOutputs[2] = data.servo3Raw || 0;
    STATE.servoOutputs[3] = data.servo4Raw || 0;
    STATE.servoOutputs[4] = data.servo5Raw || 0;
    STATE.servoOutputs[5] = data.servo6Raw || 0;
    STATE.servoOutputs[6] = data.servo7Raw || 0;
    STATE.servoOutputs[7] = data.servo8Raw || 0;
    STATE.servoOutputs[8] = data.servo9Raw || 0;
    STATE.servoOutputs[9] = data.servo10Raw || 0;
    STATE.servoOutputs[10] = data.servo11Raw || 0;
    STATE.servoOutputs[11] = data.servo12Raw || 0;
    STATE.servoOutputs[12] = data.servo13Raw || 0;
    STATE.servoOutputs[13] = data.servo14Raw || 0;
    STATE.servoOutputs[14] = data.servo15Raw || 0;
    STATE.servoOutputs[15] = data.servo16Raw || 0;
}

function mapGpsRtk(data) {
    // GPS_RTK (127): RTK status from drone
    STATE.rtkIar = data.iar_num_hypotheses || 0;
    // Baseline in NED (mm) → distance
    const n = data.baseline_a_mm || 0;
    const e = data.baseline_b_mm || 0;
    const d = data.baseline_c_mm || 0;
    STATE.rtkBaseline = Math.sqrt(n * n + e * e + d * d);
    STATE.rtkAccuracy = data.accuracy || 0;
}

function mapDistanceSensor(data) {
    // DISTANCE_SENSOR (132): currentDistance in cm, orientation 25 = downward-facing
    if (data.orientation === undefined || data.orientation === 25) {
        STATE.rangefinderDist = data.currentDistance / 100; // cm -> m
    }
}

const MAV_RESULT_NAMES = {
    0: 'ACCEPTED', 1: 'TEMPORARILY_REJECTED', 2: 'DENIED',
    3: 'UNSUPPORTED', 4: 'FAILED', 5: 'IN_PROGRESS',
    6: 'CANCELLED'
};

const MAV_CMD_NAMES = {
    22: 'TAKEOFF', 21: 'LAND', 176: 'SET_MODE', 400: 'ARM/DISARM',
    178: 'CHANGE_SPEED', 179: 'SET_HOME', 512: 'REQUEST_MESSAGE',
    246: 'REBOOT', 241: 'CALIBRATION', 20: 'RTL'
};

function mapCommandAck(data) {
    const cmdName = MAV_CMD_NAMES[data.command] || `CMD_${data.command}`;
    const resultName = MAV_RESULT_NAMES[data.result] || `RESULT_${data.result}`;
    const level = data.result === 0 ? 'info' : 'warning';
    STATE.lastCmdAck = { command: data.command, result: data.result, cmdName, resultName };
    // Dispatch event so HUD can display it
    window.dispatchEvent(new CustomEvent('commandAck', { detail: { cmdName, resultName, level } }));
}

function mapHomePosition(data) {
    const lat = data.latitude / 1e7;
    const lon = data.longitude / 1e7;
    if (Math.abs(lat) > 0.01 || Math.abs(lon) > 0.01) {
        STATE.homeLat = lat;
        STATE.homeLon = lon;
        STATE.homeAlt = data.altitude / 1000; // mm -> m
    }
}

function mapStatusText(data) {
    STATE.statusText = data.text || '';
    STATE.statusSeverity = data.severity || 0;
}
