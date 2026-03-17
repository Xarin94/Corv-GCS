/**
 * state.js - Application State Management
 * Central state object and state-related utilities
 */

import { ORIGIN, BUFFER_SIZE as CONST_BUFFER_SIZE, SAMPLE_INTERVAL as CONST_SAMPLE_INTERVAL } from './constants.js';
import { RingBuffer } from './RingBuffer.js';

// Re-export constants for convenience
export const BUFFER_SIZE = CONST_BUFFER_SIZE;
export const SAMPLE_INTERVAL = CONST_SAMPLE_INTERVAL;

/**
 * Global application state
 */
export const STATE = {
    mode: 'LIVE', // 'LIVE' or 'PLAYBACK'
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
    // NED velocity (m/s) for AoA/SSA computation
    vn: 0, ve: 0, vd: 0,
    terrainHeight: null,
    gHistory: new Array(300).fill(1.0),
    logData: [],
    logIndex: 0,
    isPlaying: false,
    lastUpdatePos: { x: 0, z: 0 },
    lastReloadPos: { lat: null, lon: null },
    runwaysLoaded: false,
    connected: false,

    // MAVLink / GCS state
    connectionType: 'none',  // 'none', 'corv-binary', 'mavlink-serial', 'mavlink-udp'
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

/**
 * View mode state
 */
export let viewMode = 'FULLSCREEN';
export function setViewMode(mode) {
    viewMode = mode;
}

/**
 * Plotly state
 */
export let plotlyInitialized = false;
export function setPlotlyInitialized(value) {
    plotlyInitialized = value;
}

export let activeTraces = ['as', 'rawAlt', 'az'];
export function setActiveTraces(traces) {
    activeTraces = traces;
}

// Ring buffer based legacy data buffer for O(1) push operations
const _timestampsBuffer = new RingBuffer(BUFFER_SIZE, true);
const _asBuffer = new RingBuffer(BUFFER_SIZE, true);
const _gsBuffer = new RingBuffer(BUFFER_SIZE, true);
const _vsBuffer = new RingBuffer(BUFFER_SIZE, true);
const _rawAltBuffer = new RingBuffer(BUFFER_SIZE, true);
const _rollBuffer = new RingBuffer(BUFFER_SIZE, true);
const _pitchBuffer = new RingBuffer(BUFFER_SIZE, true);
const _azBuffer = new RingBuffer(BUFFER_SIZE, true);

// Proxy object that exposes array-like interface for backwards compatibility
export const dataBuffer = {
    get timestamps() { return _timestampsBuffer.toArray(); },
    set timestamps(v) { _timestampsBuffer.clear(); v.forEach(x => _timestampsBuffer.push(x)); },
    get as() { return _asBuffer.toArray(); },
    set as(v) { _asBuffer.clear(); v.forEach(x => _asBuffer.push(x)); },
    get gs() { return _gsBuffer.toArray(); },
    set gs(v) { _gsBuffer.clear(); v.forEach(x => _gsBuffer.push(x)); },
    get vs() { return _vsBuffer.toArray(); },
    set vs(v) { _vsBuffer.clear(); v.forEach(x => _vsBuffer.push(x)); },
    get rawAlt() { return _rawAltBuffer.toArray(); },
    set rawAlt(v) { _rawAltBuffer.clear(); v.forEach(x => _rawAltBuffer.push(x)); },
    get roll() { return _rollBuffer.toArray(); },
    set roll(v) { _rollBuffer.clear(); v.forEach(x => _rollBuffer.push(x)); },
    get pitch() { return _pitchBuffer.toArray(); },
    set pitch(v) { _pitchBuffer.clear(); v.forEach(x => _pitchBuffer.push(x)); },
    get az() { return _azBuffer.toArray(); },
    set az(v) { _azBuffer.clear(); v.forEach(x => _azBuffer.push(x)); },
    // Direct push methods for efficient O(1) insertion
    pushTimestamp(v) { _timestampsBuffer.push(v); },
    pushAs(v) { _asBuffer.push(v); },
    pushGs(v) { _gsBuffer.push(v); },
    pushVs(v) { _vsBuffer.push(v); },
    pushRawAlt(v) { _rawAltBuffer.push(v); },
    pushRoll(v) { _rollBuffer.push(v); },
    pushPitch(v) { _pitchBuffer.push(v); },
    pushAz(v) { _azBuffer.push(v); },
    // Clear all buffers
    clear() {
        _timestampsBuffer.clear();
        _asBuffer.clear();
        _gsBuffer.clear();
        _vsBuffer.clear();
        _rawAltBuffer.clear();
        _rollBuffer.clear();
        _pitchBuffer.clear();
        _azBuffer.clear();
    }
};

export let lastSampleTime = 0;
export function setLastSampleTime(time) {
    lastSampleTime = time;
}

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
    // Update STATE.gHistory reference for compatibility
    STATE.gHistory = _gHistoryBuffer.toArray();
}

/**
 * Reset data buffer
 */
export function resetDataBuffer() {
    dataBuffer.clear();
    lastSampleTime = 0;
}
