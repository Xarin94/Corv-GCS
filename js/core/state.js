/**
 * state.js - Application State Management
 * Central state object and state-related utilities
 */

import { ORIGIN } from './constants.js';
import { RingBuffer } from './RingBuffer.js';

/**
 * Global application state
 */
export const STATE = {
    mode: 'LIVE',
    roll: 0,
    pitch: 0,
    yaw: 0,
    aoa: 0,
    ssa: 0,
    gamma: 0,
    track: 0,
    lat: ORIGIN.lat,
    lon: ORIGIN.lon,
    rawAlt: 0,
    offsetAlt: 0,
    as: 0,
    gs: 0,
    vs: 0,
    ax: 0,
    ay: 0,
    az: 0,
    // Body angular rates (rad/s) from ATTITUDE
    rollRate: 0,
    pitchRate: 0,
    yawRate: 0,
    // NED velocity (m/s) for AoA/SSA computation
    vn: 0, ve: 0, vd: 0,
    terrainHeight: null,
    gHistory: new Array(300).fill(1.0),
    lastUpdatePos: { x: 0, z: 0 },
    lastReloadPos: { lat: null, lon: null },
    runwaysLoaded: false,
    connected: false,

    // MAVLink / GCS state
    connectionType: 'none',  // 'none', 'corv-binary', 'mavlink-serial', 'mavlink-udp', 'mavlink-tcp'
    armed: false,
    flightMode: 'UNKNOWN',
    flightModeNum: 0,
    baseMode: 0,
    customMode: 0,
    batteryVoltage: 0,
    batteryCurrent: 0,
    batteryRemaining: -1,
    gpsFix: 0,
    gpsNumSat: 0,
    gpsHdop: 99.9,
    linkQuality: 0,
    rssi: null,       // local RX RSSI from RADIO_STATUS (0-254, 255=invalid)
    remRssi: null,    // remote RSSI from RADIO_STATUS (0-254, 255=invalid)
    heartbeatCount: 0,
    lastHeartbeatTime: 0,
    systemId: 1,
    componentId: 1,
    autopilotType: 0,
    vehicleType: 0,
    firmwareVersion: '',
    rcChannels: new Array(18).fill(0),
    missionCount: 0,
    missionCurrentSeq: 0,
    parameters: new Map(),
    parameterCount: 0,
    parametersReceived: 0,
    missionItems: [],
    geofenceItems: [],
    rallyPoints: [],
    statusText: '',
    statusSeverity: 0,

    // Rangefinder / LiDAR
    rangefinderDist: null, // meters, null = no data

    // Autopilot guidance (NAV_CONTROLLER_OUTPUT, msg 62)
    navRoll: 0,         // deg, commanded roll from nav controller
    navPitch: 0,        // deg, commanded pitch from nav controller
    navBearing: 0,      // deg, commanded course from nav controller
    targetBearing: 0,   // deg, bearing to active waypoint
    wpDist: 0,          // m, distance to active waypoint
    altError: 0,        // m, altitude error (target - current)
    aspdError: 0,       // m/s, airspeed error (target - current)
    xtrackError: 0,     // m, crosstrack error
    navDataTime: 0,     // ms timestamp of last msg 62 (0 = never)

    // Wind estimate (WIND, msg 168 - ArduPilot)
    windDir: 0,         // deg, direction the wind is coming FROM
    windSpeed: 0,       // m/s horizontal
    windSpeedZ: 0,      // m/s vertical
    windDataTime: 0,    // ms timestamp of last msg 168 (0 = never)

    // RTK base station
    rtkBaseConnected: false,
    rtkBaseMsgPerSec: 0,
    // GPS_RTK message (ID 127) from drone
    rtkIar: 0,
    rtkBaseline: 0,    // mm
    rtkAccuracy: 0,    // mm

    // Vibration data
    vibX: 0, vibY: 0, vibZ: 0,
    vibClip0: 0, vibClip1: 0, vibClip2: 0,
    vibHistory: [], // ring buffer of {x,y,z,t}

    // RC calibration state
    rcCalMin: new Array(16).fill(2000),
    rcCalMax: new Array(16).fill(1000),
    rcCalTrim: new Array(16).fill(1500),
    rcCalibrating: false,

    // Home position
    homeLat: null,
    homeLon: null,
    homeAlt: null,

    // Servo outputs
    servoOutputs: new Array(16).fill(0),

    // Joystick state
    joystickEnabled: false,
    joystickConnected: false,
    rcOverrideActive: false,

    // Terrain feed to vehicle (MAVLink terrain protocol)
    terrainFeedEnabled: true,
    terrainPending: 0,        // from TERRAIN_REPORT: blocks still pending on vehicle
    terrainLoaded: 0,         // from TERRAIN_REPORT: blocks loaded by vehicle
    terrainFeedSent: 0,       // count of TERRAIN_DATA messages sent by GCS
    terrainFeedErrors: 0,     // count of elevation lookups that returned null

    // ADS-B traffic (from OpenSky API or MAVLink ADSB_VEHICLE)
    // Array of { icao24, callsign, lat, lon, alt, velocity, heading, vertRate, onGround, dist }
    traffic: []
};

/**
 * Demo mode attitude smoothing state
 */
export const demoAttitude = {
    pitch: { current: 0, target: 0, velocity: 0 },
    roll:  { current: 0, target: 0, velocity: 0 },
    yaw:   { current: Math.PI, target: Math.PI, velocity: 0 }
};

export let demoTargetChangeTime = 0;
export function setDemoTargetChangeTime(time) {
    demoTargetChangeTime = time;
}

/**
 * Survey pattern state machine for drone demo
 */
export const demoSurveyState = {
    legIndex: 0,
    distOnLeg: 0,
    turning: false,
    turnProgress: 0,
    direction: 1,       // +1 = turn right, -1 = turn left
    legHeading: 0       // current leg heading in radians (north)
};

// Ring buffer for G history
const _gHistoryBuffer = new RingBuffer(300, true);
// Initialize with 1.0 values
for (let i = 0; i < 300; i++) _gHistoryBuffer.push(1.0);

/**
 * Push G history for G-load graph
 */
export function pushGHistory() {
    // Normal load factor: negate body-frame Z (NED: -1G at rest → +1G load)
    const g = -STATE.az / 9.81;
    _gHistoryBuffer.push(g);
}

/** Expose ring buffer for direct read access (avoids toArray() allocation) */
export const gHistoryBuffer = _gHistoryBuffer;

/**
 * Reset all live-telemetry state. Called by the Log Replay controller before
 * loading a new log and when seeking backward, so the UI doesn't keep stale
 * trails/values from a previous position. Does NOT touch connection state,
 * user preferences, or RC calibration.
 */
export function resetReplayState() {
    STATE.roll = 0; STATE.pitch = 0; STATE.yaw = 0;
    STATE.aoa = 0; STATE.ssa = 0; STATE.gamma = 0; STATE.track = 0;
    STATE.lat = ORIGIN.lat; STATE.lon = ORIGIN.lon;
    STATE.rawAlt = 0; STATE.offsetAlt = 0;
    STATE.as = 0; STATE.gs = 0; STATE.vs = 0;
    STATE.ax = 0; STATE.ay = 0; STATE.az = 0;
    STATE.rollRate = 0; STATE.pitchRate = 0; STATE.yawRate = 0;
    STATE.vn = 0; STATE.ve = 0; STATE.vd = 0;
    STATE.terrainHeight = null;
    STATE.gHistory.fill(1.0);
    _gHistoryBuffer.clear();
    for (let i = 0; i < 300; i++) _gHistoryBuffer.push(1.0);
    STATE.lastUpdatePos = { x: 0, z: 0 };
    STATE.lastReloadPos = { lat: null, lon: null };

    STATE.armed = false;
    STATE.flightMode = 'UNKNOWN';
    STATE.flightModeNum = 0;
    STATE.baseMode = 0;
    STATE.customMode = 0;
    STATE.batteryVoltage = 0;
    STATE.batteryCurrent = 0;
    STATE.batteryRemaining = -1;
    STATE.gpsFix = 0;
    STATE.gpsNumSat = 0;
    STATE.gpsHdop = 99.9;
    STATE.linkQuality = 0;
    STATE.rssi = null;
    STATE.remRssi = null;
    STATE.heartbeatCount = 0;
    STATE.lastHeartbeatTime = 0;

    STATE.rcChannels.fill(0);
    STATE.servoOutputs.fill(0);
    STATE.missionCurrentSeq = 0;
    STATE.missionItems = [];
    STATE.geofenceItems = [];
    STATE.rallyPoints = [];
    STATE.statusText = '';
    STATE.statusSeverity = 0;

    STATE.rangefinderDist = null;

    STATE.navRoll = 0; STATE.navPitch = 0;
    STATE.navBearing = 0; STATE.targetBearing = 0; STATE.wpDist = 0;
    STATE.altError = 0; STATE.aspdError = 0; STATE.xtrackError = 0;
    STATE.navDataTime = 0;
    STATE.windDir = 0; STATE.windSpeed = 0; STATE.windSpeedZ = 0;
    STATE.windDataTime = 0;

    STATE.rtkIar = 0;
    STATE.rtkBaseline = 0;
    STATE.rtkAccuracy = 0;

    STATE.vibX = 0; STATE.vibY = 0; STATE.vibZ = 0;
    STATE.vibClip0 = 0; STATE.vibClip1 = 0; STATE.vibClip2 = 0;
    STATE.vibHistory = [];

    STATE.homeLat = null;
    STATE.homeLon = null;
    STATE.homeAlt = null;

    STATE.terrainPending = 0;
    STATE.terrainLoaded = 0;
    STATE.terrainFeedSent = 0;
    STATE.terrainFeedErrors = 0;

    STATE.traffic = [];

}
