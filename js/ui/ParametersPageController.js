/**
 * ParametersPageController.js - Dedicated full-screen parameters page
 * Handles reading, searching, editing, saving and loading ArduPilot parameters
 */

import { STATE } from '../core/state.js';
import { requestAllParameters, setParameter } from '../mavlink/CommandSender.js';
import { onMessage } from '../mavlink/MAVLinkManager.js';

// ArduPilot parameter descriptions (common subset)
const PARAM_DESCRIPTIONS = {
    // Attitude Control
    ATC_RAT_RLL_P: 'Roll rate controller P gain',
    ATC_RAT_RLL_I: 'Roll rate controller I gain',
    ATC_RAT_RLL_D: 'Roll rate controller D gain',
    ATC_RAT_RLL_FF: 'Roll rate controller feed forward',
    ATC_RAT_RLL_FLTD: 'Roll rate controller D term filter frequency (Hz)',
    ATC_RAT_RLL_FLTT: 'Roll rate controller target filter frequency (Hz)',
    ATC_RAT_PIT_P: 'Pitch rate controller P gain',
    ATC_RAT_PIT_I: 'Pitch rate controller I gain',
    ATC_RAT_PIT_D: 'Pitch rate controller D gain',
    ATC_RAT_PIT_FF: 'Pitch rate controller feed forward',
    ATC_RAT_YAW_P: 'Yaw rate controller P gain',
    ATC_RAT_YAW_I: 'Yaw rate controller I gain',
    ATC_RAT_YAW_D: 'Yaw rate controller D gain',
    ATC_RAT_YAW_FF: 'Yaw rate controller feed forward',
    ATC_ANG_RLL_P: 'Roll angle controller P gain',
    ATC_ANG_PIT_P: 'Pitch angle controller P gain',
    ATC_ANG_YAW_P: 'Yaw angle controller P gain',
    ATC_ACCEL_R_MAX: 'Maximum roll acceleration (centideg/s/s)',
    ATC_ACCEL_P_MAX: 'Maximum pitch acceleration (centideg/s/s)',
    ATC_ACCEL_Y_MAX: 'Maximum yaw acceleration (centideg/s/s)',
    ATC_INPUT_TC: 'Attitude control input time constant (seconds)',
    // ArduCopter
    ANGLE_MAX: 'Maximum lean angle in all flight modes (centidegrees)',
    LOIT_SPEED: 'Maximum horizontal speed in Loiter mode (cm/s)',
    LOIT_ACC_MAX: 'Maximum acceleration in Loiter mode (cm/s/s)',
    LOIT_BRK_ACCEL: 'Braking acceleration in Loiter mode (cm/s/s)',
    WPNAV_SPEED: 'Waypoint horizontal speed (cm/s)',
    WPNAV_SPEED_UP: 'Waypoint climb speed (cm/s)',
    WPNAV_SPEED_DN: 'Waypoint descent speed (cm/s)',
    WPNAV_ACCEL: 'Waypoint horizontal acceleration (cm/s/s)',
    WPNAV_RADIUS: 'Waypoint acceptance radius (cm)',
    RTL_ALT: 'RTL altitude (cm above home)',
    RTL_ALT_FINAL: 'RTL final altitude above home (cm)',
    RTL_SPEED: 'RTL return speed (cm/s). 0=WPNAV_SPEED',
    LAND_SPEED: 'Final stage landing speed (cm/s)',
    LAND_SPEED_HIGH: 'Landing speed until close to ground (cm/s)',
    PILOT_SPEED_UP: 'Pilot max climb rate (cm/s)',
    PILOT_SPEED_DN: 'Pilot max descent rate (cm/s)',
    PILOT_ACCEL_Z: 'Pilot vertical acceleration (cm/s/s)',
    // Position Control
    PSC_POSXY_P: 'Position XY controller P gain',
    PSC_VELXY_P: 'Velocity XY controller P gain',
    PSC_VELXY_I: 'Velocity XY controller I gain',
    PSC_VELXY_D: 'Velocity XY controller D gain',
    PSC_POSZ_P: 'Position Z controller P gain',
    PSC_VELZ_P: 'Velocity Z controller P gain',
    PSC_ACCZ_P: 'Accel Z controller P gain',
    PSC_ACCZ_I: 'Accel Z controller I gain',
    PSC_ACCZ_D: 'Accel Z controller D gain',
    // Battery
    BATT_MONITOR: 'Battery monitoring type (0=disabled, 3=analog V, 4=analog V+I)',
    BATT_CAPACITY: 'Battery capacity (mAh)',
    BATT_LOW_VOLT: 'Low battery voltage (V)',
    BATT_CRT_VOLT: 'Critical battery voltage (V)',
    BATT_ARM_VOLT: 'Minimum voltage to allow arming (V)',
    BATT_FS_LOW_ACT: 'Low battery failsafe action',
    BATT_FS_CRT_ACT: 'Critical battery failsafe action',
    // Arming
    ARMING_CHECK: 'Arming checks bitmask (0=disabled, 1=all)',
    ARMING_RUDDER: 'Rudder arming (0=disabled, 1=arm/disarm, 2=arm only)',
    // Compass
    COMPASS_USE: 'Enable first compass for yaw (0=disabled, 1=enabled)',
    COMPASS_ORIENT: 'Compass orientation',
    COMPASS_OFS_X: 'Compass X offset',
    COMPASS_OFS_Y: 'Compass Y offset',
    COMPASS_OFS_Z: 'Compass Z offset',
    // EKF
    EK2_ENABLE: 'Enable EKF2 (0=disabled, 1=enabled)',
    EK3_ENABLE: 'Enable EKF3 (0=disabled, 1=enabled)',
    AHRS_EKF_TYPE: 'EKF type (2=EKF2, 3=EKF3, 11=ExternalAHRS)',
    // Fence
    FENCE_ENABLE: 'Enable geofence (0=disabled, 1=enabled)',
    FENCE_TYPE: 'Fence type bitmask (1=max alt, 2=circle, 4=polygon)',
    FENCE_ALT_MAX: 'Maximum altitude fence (m)',
    FENCE_RADIUS: 'Circular fence radius (m)',
    FENCE_ACTION: 'Fence breach action (0=report, 1=RTL, 2=land)',
    // Flight modes
    FLTMODE1: 'Flight mode for PWM channel 5 position 1',
    FLTMODE2: 'Flight mode for PWM channel 5 position 2',
    FLTMODE3: 'Flight mode for PWM channel 5 position 3',
    FLTMODE4: 'Flight mode for PWM channel 5 position 4',
    FLTMODE5: 'Flight mode for PWM channel 5 position 5',
    FLTMODE6: 'Flight mode for PWM channel 5 position 6',
    // GPS
    GPS_TYPE: 'GPS receiver type (0=none, 1=auto, 2=uBlox)',
    GPS_NAVFILTER: 'Navigation filter mode (0=portable, 8=airborne 4G)',
    // INS (IMU)
    INS_GYRO_FILTER: 'Gyro low-pass filter frequency (Hz)',
    INS_ACCEL_FILTER: 'Accelerometer low-pass filter frequency (Hz)',
    // Logging
    LOG_BITMASK: 'Log bitmask (0=disabled)',
    LOG_BACKEND_TYPE: 'Log backend type (1=file, 2=MAVLink, 3=both)',
    // Motor
    MOT_SPIN_ARM: 'Motor spin when armed (0.0 to 0.3)',
    MOT_SPIN_MIN: 'Motor minimum spin when flying (0.0 to 0.3)',
    MOT_BAT_VOLT_MAX: 'Battery max voltage for motor scaling (V)',
    MOT_BAT_VOLT_MIN: 'Battery min voltage for motor scaling (V)',
    MOT_THST_EXPO: 'Motor thrust curve expo (0=linear, 1=curved)',
    MOT_THST_HOVER: 'Throttle hover value (0.0 to 1.0)',
    // RC
    RC_SPEED: 'ESC update speed (Hz)',
    RCMAP_ROLL: 'Roll input channel',
    RCMAP_PITCH: 'Pitch input channel',
    RCMAP_THROTTLE: 'Throttle input channel',
    RCMAP_YAW: 'Yaw input channel',
    // Serial
    SERIAL0_BAUD: 'Serial 0 baud rate',
    SERIAL0_PROTOCOL: 'Serial 0 protocol (1=MAVLink1, 2=MAVLink2)',
    SERIAL1_BAUD: 'Serial 1 baud rate',
    SERIAL1_PROTOCOL: 'Serial 1 protocol',
    // Safety
    BRD_SAFETY_DEFLT: 'Default safety switch state (0=off, 1=on)',
    FS_THR_ENABLE: 'Throttle failsafe enable (0=disabled, 1=enabled)',
    FS_THR_VALUE: 'Throttle failsafe value (PWM)',
    FS_GCS_ENABLE: 'GCS failsafe enable',
    // Servo
    SERVO1_FUNCTION: 'Servo 1 output function',
    SERVO2_FUNCTION: 'Servo 2 output function',
    SERVO3_FUNCTION: 'Servo 3 output function',
    SERVO4_FUNCTION: 'Servo 4 output function',
    // System
    SYSID_THISMAV: 'MAVLink system ID of this vehicle',
    FRAME_CLASS: 'Frame class (1=quad, 2=hexa, 3=octa, 5=Y6, 7=tri)',
    FRAME_TYPE: 'Frame type (0=plus, 1=X, 2=V, 3=H)',
    // Terrain
    TERRAIN_ENABLE: 'Enable terrain following (0=disabled, 1=enabled)',
    TERRAIN_FOLLOW: 'Terrain follow mode bitmask',
};

let isOpen = false;
let searchFilter = '';
let initialized = false;

/**
 * Initialize parameters page
 */
export function initParamsPage() {
    if (initialized) return;
    initialized = true;

    const closeBtn = document.getElementById('params-page-close');
    if (closeBtn) closeBtn.addEventListener('click', toggleParamsPage);

    bindAction('params-read-all', async () => {
        const progressEl = document.getElementById('params-progress');
        if (progressEl) progressEl.style.display = 'flex';
        await requestAllParameters();
    });

    const searchInput = document.getElementById('params-search');
    let searchDebounceTimer = null;
    if (searchInput) {
        searchInput.addEventListener('input', (e) => {
            searchFilter = e.target.value.toUpperCase();
            clearTimeout(searchDebounceTimer);
            searchDebounceTimer = setTimeout(() => renderParamsTable(), 250);
        });
    }

    bindAction('params-save', saveParameterFile);
    bindAction('params-load', loadParameterFile);

    // Listen for PARAM_VALUE messages (throttle table render to avoid lag)
    let paramRenderPending = false;
    onMessage(22, (data) => {
        STATE.parameters.set(data.paramId, {
            value: data.paramValue,
            type: data.paramType,
            index: data.paramIndex
        });
        STATE.parameterCount = data.paramCount;
        STATE.parametersReceived = STATE.parameters.size;
        updateProgress();
        if (isOpen && !paramRenderPending) {
            paramRenderPending = true;
            setTimeout(() => {
                paramRenderPending = false;
                // Skip render if user is typing in search or editing a param value
                const active = document.activeElement;
                if (active && (active.id === 'params-search' || active.classList.contains('param-val-input'))) {
                    return;
                }
                renderParamsTable();
            }, 500);
        }
    });

    // ESC to close
    document.addEventListener('keydown', (e) => {
        if (e.key === 'Escape' && isOpen) toggleParamsPage();
    });
}

/**
 * Toggle parameters page visibility
 */
export function toggleParamsPage() {
    const page = document.getElementById('params-page');
    if (!page) return;

    isOpen = !isOpen;
    page.classList.toggle('open', isOpen);

    if (isOpen) {
        renderParamsTable();
    }
}

function bindAction(id, handler) {
    const el = document.getElementById(id);
    if (el) {
        el.addEventListener('click', async () => {
            try { await handler(); } catch (e) { alert('Error: ' + e.message); }
        });
    }
}

function updateProgress() {
    const fillEl = document.getElementById('params-fill');
    const countEl = document.getElementById('params-count');
    const progressEl = document.getElementById('params-progress');
    if (!fillEl || !countEl) return;

    if (STATE.parameterCount > 0) {
        if (progressEl) progressEl.style.display = 'flex';
        const pct = (STATE.parametersReceived / STATE.parameterCount * 100).toFixed(0);
        fillEl.style.width = pct + '%';
        countEl.textContent = `${STATE.parametersReceived}/${STATE.parameterCount}`;
    }
}

/**
 * Get parameter description
 */
function getParamDescription(name) {
    // Direct match
    if (PARAM_DESCRIPTIONS[name]) return PARAM_DESCRIPTIONS[name];

    // Try prefix match (e.g., ATC_RAT_RLL_P matches ATC_RAT_RLL_P)
    // For indexed params like SERVO5_FUNCTION, try SERVOn_FUNCTION pattern
    const base = name.replace(/\d+/, 'n');
    for (const [key, desc] of Object.entries(PARAM_DESCRIPTIONS)) {
        const keyBase = key.replace(/\d+/, 'n');
        if (keyBase === base) return desc;
    }

    // Group-based fallback descriptions
    const prefix = name.split('_')[0];
    const GROUP_HINTS = {
        ATC: 'Attitude controller parameter',
        PSC: 'Position/velocity controller parameter',
        WPNAV: 'Waypoint navigation parameter',
        LOIT: 'Loiter mode parameter',
        RTL: 'Return-to-launch parameter',
        BATT: 'Battery monitor parameter',
        COMPASS: 'Compass/magnetometer parameter',
        EK2: 'Extended Kalman Filter 2 parameter',
        EK3: 'Extended Kalman Filter 3 parameter',
        INS: 'Inertial sensor parameter',
        MOT: 'Motor output parameter',
        RC: 'RC input parameter',
        SERVO: 'Servo output parameter',
        SERIAL: 'Serial port parameter',
        GPS: 'GPS receiver parameter',
        FENCE: 'Geofence parameter',
        LOG: 'Logging parameter',
        FS: 'Failsafe parameter',
        PILOT: 'Pilot input parameter',
        LAND: 'Landing parameter',
        ARMING: 'Arming check parameter',
        BRD: 'Board configuration parameter',
        AHRS: 'Attitude/heading reference parameter',
        SCHED: 'Task scheduler parameter',
        TERRAIN: 'Terrain following parameter',
        FLOW: 'Optical flow parameter',
        RNGFND: 'Rangefinder parameter',
        NTF: 'Notification parameter',
        RALLY: 'Rally point parameter',
        MIS: 'Mission parameter',
        FLTMODE: 'Flight mode selection',
        FRAME: 'Vehicle frame parameter',
        SYSID: 'System identification parameter',
        SR: 'Telemetry stream rate parameter',
    };
    return GROUP_HINTS[prefix] || '';
}

/**
 * Format a parameter value based on its MAVLink type
 * Types 1-6 are integer types (UINT8, INT8, UINT16, INT16, UINT32, INT32)
 * Types 9-10 are float types (REAL32, REAL64)
 */
export function formatParamValue(value, type) {
    if (typeof value !== 'number') return value;
    // Integer types: show as integer
    if (type >= 1 && type <= 6) {
        return Math.round(value).toString();
    }
    // ArduPilot sends all params as REAL32 (type 9), even integer ones.
    // If value is effectively an integer, display without decimals.
    if (Math.abs(value - Math.round(value)) < 1e-4) {
        return Math.round(value).toString();
    }
    // Float types: show meaningful precision, strip trailing zeros
    const str = value.toFixed(6);
    return str.replace(/\.?0+$/, '') || '0';
}

/**
 * Render the parameter table
 */
function renderParamsTable() {
    const tbody = document.getElementById('params-table-body');
    if (!tbody) return;

    const params = Array.from(STATE.parameters.entries())
        .filter(([name]) => !searchFilter || name.includes(searchFilter))
        .sort((a, b) => a[0].localeCompare(b[0]));

    tbody.innerHTML = params.map(([name, param]) => {
        const desc = getParamDescription(name);
        const val = formatParamValue(param.value, param.type);
        return `<tr>
            <td class="param-name">${name}</td>
            <td><input class="param-val-input" type="text" value="${val}"
                       data-param-name="${name}" data-param-type="${param.type}"></td>
            <td class="param-desc">${desc}</td>
        </tr>`;
    }).join('');

    // Bind edit handlers
    tbody.querySelectorAll('input[data-param-name]').forEach(input => {
        input.addEventListener('change', async (e) => {
            const paramName = e.target.dataset.paramName;
            const paramType = parseInt(e.target.dataset.paramType) || 9;
            // Parse as integer for integer types (1-6), float otherwise
            const raw = e.target.value.trim();
            const newValue = (paramType >= 1 && paramType <= 6)
                ? parseInt(raw, 10)
                : parseFloat(raw);
            if (isNaN(newValue)) return;

            try {
                await setParameter(paramName, newValue, paramType);
                e.target.style.borderColor = '#44ff44';
                setTimeout(() => { e.target.style.borderColor = ''; }, 2000);
            } catch (err) {
                e.target.style.borderColor = '#ff4444';
                alert('Parameter set failed: ' + err.message);
            }
        });
    });
}

function saveParameterFile() {
    if (STATE.parameters.size === 0) {
        alert('No parameters loaded. Read parameters first.');
        return;
    }
    const lines = Array.from(STATE.parameters.entries())
        .sort((a, b) => a[0].localeCompare(b[0]))
        .map(([name, param]) => `${name},${formatParamValue(param.value, param.type)}`);
    const blob = new Blob([lines.join('\n')], { type: 'text/plain' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'parameters.param';
    a.click();
    URL.revokeObjectURL(url);
}

function loadParameterFile() {
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = '.param,.txt';
    input.addEventListener('change', async (e) => {
        const file = e.target.files[0];
        if (!file) return;
        const text = await file.text();
        const lines = text.split('\n').filter(l => l.trim() && !l.startsWith('#'));
        let count = 0;
        for (const line of lines) {
            const parts = line.split(',');
            if (parts.length >= 2) {
                const name = parts[0].trim();
                const value = parseFloat(parts[1].trim());
                if (!isNaN(value)) {
                    try {
                        await setParameter(name, value);
                        count++;
                    } catch (err) {
                        console.error(`Failed to set ${name}:`, err);
                    }
                }
            }
        }
        alert(`Loaded ${count} parameters from ${file.name}`);
        renderParamsTable();
    });
    input.click();
}
