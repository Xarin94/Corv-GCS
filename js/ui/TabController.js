/**
 * TabController.js - GCS Tab Navigation Controller
 * Handles switching between Flight Data, Flight Plan, Setup and Sys Config, and owns
 * the Setup / Sys Config pages. The Flight Plan page is FlightPlanController.js.
 */

import { STATE } from '../core/state.js';
import { t } from '../core/i18n.js';
import { connect, disconnect, getAvailablePorts } from '../mavlink/ConnectionManager.js';
import { setParameter, requestAllParameters, requestParameter, requestDataStream, calibrateAccel, calibrateCompass, calibrateGyro, sendServoTest, sendRelayToggle } from '../mavlink/CommandSender.js';
import { onMessage } from '../mavlink/MAVLinkManager.js';
import { getVehicleTypeName } from '../mavlink/MAVLinkStateMapper.js';
import {
    formatParamValue, refreshParametersPanel, setParamsTableRenderer,
    beginFullRead, isFullReadActive
} from './ParametersPageController.js';
import { initJoystick } from '../joystick/JoystickUI.js';
import { getTerrainElevationAsync, resetAutoDownloadFailures } from '../terrain/TerrainManager.js';
import { onFlightPlanShown, onFlightPlanHidden } from './FlightPlanController.js';

let currentTab = 'flight-data';
/**
 * Initialize tab controller
 */
export function initTabs() {
    // Add has-tabs class to body for CSS adjustments
    document.body.classList.add('has-tabs');

    // Tab click handlers
    document.querySelectorAll('.gcs-tab').forEach(tab => {
        tab.addEventListener('click', () => {
            switchTab(tab.dataset.tab);
        });
    });

    // Keyboard shortcuts: Ctrl+1..4 for tab switching
    const TAB_SHORTCUTS = { '1': 'flight-data', '2': 'flight-plan', '3': 'setup', '4': 'sys-config' };
    document.addEventListener('keydown', (e) => {
        const tag = document.activeElement?.tagName;
        if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') return;
        if (e.ctrlKey && !e.shiftKey && !e.altKey && TAB_SHORTCUTS[e.key]) {
            e.preventDefault();
            switchTab(TAB_SHORTCUTS[e.key]);
        }
    });

    // Sub-tab switching (generic for all tab containers)
    initSubTabs();

    // Setup vertical nav switching
    initSetupVerticalNav();

    // Initial Setup tab handlers
    initSetupTab();

    // Config/Tuning tab handlers
    initConfigTuningTab();

    // Simulation tab handlers
    initSimulationTab();

    // RTK/GPS tab handlers
    initRTKTab();

    // Telemetry Forward tab handlers
    initTelForwardTab();

    // CORV Setup tab handlers
    initCorvSetupTab();
}

/**
 * Initialize sub-tab switching for all containers with .sub-tab-bar
 */
function initSubTabs() {
    document.querySelectorAll('.sub-tab-bar').forEach(bar => {
        const container = bar.parentElement;
        const buttons = bar.querySelectorAll('.sub-tab');
        const panels = container.querySelectorAll('.sub-tab-content');

        buttons.forEach(btn => {
            btn.addEventListener('click', () => {
                const target = btn.dataset.subtab;

                // Deactivate all in this container
                buttons.forEach(b => b.classList.remove('active'));
                panels.forEach(p => p.classList.remove('active'));

                // Activate clicked
                btn.classList.add('active');
                const panel = container.querySelector(`#subtab-${target}`);
                if (panel) panel.classList.add('active');
            });
        });
    });
}

/**
 * Initialize setup vertical nav switching (collapsible macro-category groups)
 */
function initSetupVerticalNav() {
    const nav = document.querySelector('.setup-vertical-nav');
    if (!nav) return;

    const buttons = nav.querySelectorAll('.setup-nav-btn');
    const contentArea = document.querySelector('.setup-content-area');
    if (!contentArea) return;

    const panels = contentArea.querySelectorAll('.sub-tab-content');

    // Collapsible group headers (state persisted across sessions)
    const COLLAPSE_KEY = 'setup-nav-collapsed';
    let collapsedState = {};
    try { collapsedState = JSON.parse(localStorage.getItem(COLLAPSE_KEY)) || {}; } catch (e) { /* ignore */ }

    nav.querySelectorAll('.setup-nav-group').forEach(group => {
        const name = group.dataset.group;
        if (collapsedState[name]) group.classList.add('collapsed');
        const header = group.querySelector('.setup-nav-group-header');
        if (!header) return;
        header.addEventListener('click', () => {
            group.classList.toggle('collapsed');
            collapsedState[name] = group.classList.contains('collapsed');
            try { localStorage.setItem(COLLAPSE_KEY, JSON.stringify(collapsedState)); } catch (e) { /* ignore */ }
        });
    });

    buttons.forEach(btn => {
        btn.addEventListener('click', () => {
            const section = btn.dataset.section;

            // Deactivate all
            buttons.forEach(b => b.classList.remove('active'));
            panels.forEach(p => p.classList.remove('active'));

            // Activate clicked
            btn.classList.add('active');
            const panel = contentArea.querySelector(`#subtab-${section}`);
            if (panel) panel.classList.add('active');

            // Screens that show live parameter values only refresh on entry:
            // nothing pushes into them while they are hidden.
            if (section === 'parameters') refreshParametersPanel();
            if (section === 'pid-tuning') populatePidInputs();

            // Make sure the group containing the active section is expanded
            // (matters when sections are activated programmatically)
            const group = btn.closest('.setup-nav-group');
            if (group && group.classList.contains('collapsed')) {
                group.classList.remove('collapsed');
                collapsedState[group.dataset.group] = false;
                try { localStorage.setItem(COLLAPSE_KEY, JSON.stringify(collapsedState)); } catch (e) { /* ignore */ }
            }
        });
    });
}

/**
 * Set the status dot on a setup-nav section button (and its group header,
 * so activity is visible while the group is collapsed)
 */
function setNavDot(section, on) {
    const btn = document.querySelector(`.setup-nav-btn[data-section="${section}"]`);
    if (!btn) return;
    const dot = btn.querySelector('.nav-status-dot');
    if (dot) dot.classList.toggle('on', !!on);

    const group = btn.closest('.setup-nav-group');
    if (group) {
        const anyOn = [...group.querySelectorAll('.nav-status-dot')].some(d => d.classList.contains('on'));
        const groupDot = group.querySelector('.nav-group-dot');
        if (groupDot) groupDot.classList.toggle('on', anyOn);
    }
}

/**
 * Switch to a tab
 */
export function switchTab(tabName) {
    if (currentTab === tabName) return;
    const previousTab = currentTab;

    // Deactivate all tabs and content
    document.querySelectorAll('.gcs-tab').forEach(t => t.classList.remove('active'));
    document.querySelectorAll('.tab-content').forEach(c => c.classList.remove('active'));

    // Activate selected tab
    const tabBtn = document.querySelector(`.gcs-tab[data-tab="${tabName}"]`);
    const tabContent = document.getElementById(`tab-${tabName}`);

    if (tabBtn) tabBtn.classList.add('active');
    if (tabContent) tabContent.classList.add('active');

    currentTab = tabName;

    // Hide GCS sidebar on tabs that don't use it (e.g. Flight Plan has its own panel)
    const hideSidebar = (tabName === 'flight-plan' || tabName === 'setup');
    const gcsSidebar = document.getElementById('gcs-sidebar');
    const gcsSidebarToggle = document.getElementById('gcs-toggle-sidebar');
    if (gcsSidebar) gcsSidebar.style.display = hideSidebar ? 'none' : '';
    if (gcsSidebarToggle) gcsSidebarToggle.style.display = hideSidebar ? 'none' : '';
    // Expand content to fill sidebar gap
    const tabEl = document.getElementById(`tab-${tabName}`);
    if (tabEl && (tabName === 'flight-plan' || tabName === 'setup' || tabName === 'sys-config')) {
        tabEl.style.right = hideSidebar ? '0' : '';
    }

    // The route editor lives in FlightPlanController; it initialises itself on first show
    if (tabName === 'flight-plan') onFlightPlanShown();
    else if (previousTab === 'flight-plan') onFlightPlanHidden();

    // Trigger resize for 3D view when switching back
    if (tabName === 'flight-data') {
        window.dispatchEvent(new Event('resize'));
    }

    // Render params table when switching to setup tab
    if (tabName === 'setup') {
        renderCfgParamsTable();
    }
}

/**
 * Get current active tab
 */
export function getCurrentTab() {
    return currentTab;
}

function portLabel(p) {
    if (p.friendlyName) return p.friendlyName;
    if (p.manufacturer) return `${p.path} (${p.manufacturer})`;
    return p.path;
}

/**
 * Initialize Initial Setup tab
 */
function initSetupTab() {
    const scanBtn = document.getElementById('setup-scan-ports');
    const connectBtn = document.getElementById('setup-connect');
    const disconnectBtn = document.getElementById('setup-disconnect');
    const connTypeSel = document.getElementById('setup-conn-type');

    // Show only the fields relevant to the selected connection type
    const updateConnFields = () => {
        const type = connTypeSel?.value;
        // serial/baud for serial & legacy binary, udp host/port for UDP, tcp host/port for TCP
        const groups = {
            'serial-field': type === 'mavlink-serial' || type === 'corv-binary' || type === 'msp-serial',
            'udp-field': type === 'mavlink-udp',
            'tcp-field': type === 'mavlink-tcp' || type === 'msp-tcp',
            'msp-field': type === 'msp-serial' || type === 'msp-tcp'
        };
        for (const [cls, visible] of Object.entries(groups)) {
            document.querySelectorAll('.conn-field.' + cls).forEach(el => {
                el.style.display = visible ? '' : 'none';
            });
        }
    };
    if (connTypeSel) {
        connTypeSel.addEventListener('change', updateConnFields);
        updateConnFields();
    }

    if (scanBtn) {
        scanBtn.addEventListener('click', async () => {
            const ports = await getAvailablePorts();
            const select = document.getElementById('setup-serial-port');
            if (select) {
                select.innerHTML = ports.length === 0
                    ? '<option value="">No ports found</option>'
                    : ports.map(p => `<option value="${p.path}">${portLabel(p)}</option>`).join('');
            }
        });
    }

    if (connectBtn) {
        connectBtn.addEventListener('click', async () => {
            const type = document.getElementById('setup-conn-type')?.value;
            try {
                if (type === 'mavlink-serial') {
                    const port = document.getElementById('setup-serial-port')?.value;
                    const baud = parseInt(document.getElementById('setup-baud')?.value) || 57600;
                    if (!port) { alert('Select a serial port first'); return; }
                    await connect('mavlink-serial', { port, baudRate: baud });
                } else if (type === 'mavlink-udp') {
                    const host = document.getElementById('setup-udp-host')?.value || '127.0.0.1';
                    const port = parseInt(document.getElementById('setup-udp-port')?.value) || 14550;
                    await connect('mavlink-udp', { host, port });
                } else if (type === 'mavlink-tcp') {
                    const host = document.getElementById('setup-tcp-host')?.value || '127.0.0.1';
                    const port = parseInt(document.getElementById('setup-tcp-port')?.value) || 5760;
                    await connect('mavlink-tcp', { host, port });
                } else if (type === 'corv-binary') {
                    const port = document.getElementById('setup-serial-port')?.value;
                    const baud = parseInt(document.getElementById('setup-baud')?.value) || 460800;
                    if (!port) { alert('Select a serial port first'); return; }
                    await connect('corv-binary', { port, baudRate: baud });
                } else if (type === 'msp-serial') {
                    const port = document.getElementById('setup-serial-port')?.value;
                    const baud = parseInt(document.getElementById('setup-baud')?.value) || 115200;
                    const profile = document.getElementById('setup-msp-profile')?.value || 'normal';
                    if (!port) { alert('Select a serial port first'); return; }
                    await connect('msp-serial', { port, baudRate: baud, profile });
                } else if (type === 'msp-tcp') {
                    const host = document.getElementById('setup-tcp-host')?.value || '127.0.0.1';
                    const port = parseInt(document.getElementById('setup-tcp-port')?.value) || 5760;
                    const profile = document.getElementById('setup-msp-profile')?.value || 'normal';
                    await connect('msp-tcp', { host, port, profile });
                }
            } catch (e) {
                alert('Connection failed: ' + e.message);
            }
        });
    }

    if (disconnectBtn) {
        disconnectBtn.addEventListener('click', async () => {
            await disconnect();
        });
    }

    // Update firmware info from heartbeat
    onMessage(0, (data) => {
        const autopilotEl = document.getElementById('setup-autopilot');
        const vehicleEl = document.getElementById('setup-vehicle');
        const sysidEl = document.getElementById('setup-sysid');
        if (autopilotEl) autopilotEl.textContent = `Type ${data.autopilot}`;
        if (vehicleEl) vehicleEl.textContent = `Type ${data.type}`;
        if (sysidEl) sysidEl.textContent = STATE.systemId;
    });

    // Initialize joystick/gamepad support
    initJoystick();

    // Calibration buttons (moved from sidebar to Initial Setup > Connection sub-tab)
    bindBtn('setup-cal-accel', async () => {
        if (await confirm('Start accelerometer calibration?')) await calibrateAccel();
    });
    bindBtn('setup-cal-compass', async () => {
        if (await confirm('Start compass calibration?')) await calibrateCompass();
    });
    bindBtn('setup-cal-gyro', async () => {
        if (await confirm('Start gyroscope calibration?')) await calibrateGyro();
    });

    // Flight Modes sub-tab
    initFlightModes();

    // Failsafe sub-tab
    initFailsafe();

    // Radio Calibration sub-tab
    initRadioCalibration();

    // Calibration wizard progress tracking
    initCalibrationWizard();
}

/**
 * Helper to bind a click handler to a button by ID
 */
function bindBtn(id, handler) {
    const el = document.getElementById(id);
    if (el) el.addEventListener('click', async () => {
        try { await handler(); } catch (e) { alert('Error: ' + e.message); }
    });
}

// Flight mode parameter names
const FLTMODE_PARAMS = ['FLTMODE1', 'FLTMODE2', 'FLTMODE3', 'FLTMODE4', 'FLTMODE5', 'FLTMODE6'];

/**
 * Initialize Flight Modes sub-tab
 */
function initFlightModes() {
    bindBtn('fltmode-read', async () => {
        // Request FLTMODE_CH and FLTMODE1-6
        await requestParameter('FLTMODE_CH');
        for (const p of FLTMODE_PARAMS) await requestParameter(p);
        // Wait for params to arrive, then populate
        setTimeout(populateFlightModes, 1500);
    });

    bindBtn('fltmode-write', async () => {
        let count = 0;
        // Write FLTMODE_CH
        const chSel = document.getElementById('fltmode-channel');
        if (chSel) {
            await setParameter('FLTMODE_CH', parseInt(chSel.value));
            count++;
        }
        // Write FLTMODE1-6
        for (let i = 1; i <= 6; i++) {
            const sel = document.getElementById(`fltmode-${i}`);
            if (sel) {
                await setParameter(`FLTMODE${i}`, parseInt(sel.value));
                count++;
            }
        }
        alert(`Written ${count} flight mode parameters`);
    });

    // Auto-populate when params arrive
    onMessage(22, () => populateFlightModes());
}

function populateFlightModes() {
    // Populate channel selector
    const chParam = STATE.parameters.get('FLTMODE_CH');
    if (chParam) {
        const chSel = document.getElementById('fltmode-channel');
        if (chSel) chSel.value = String(Math.round(chParam.value));
    }
    // Populate mode selectors
    for (let i = 1; i <= 6; i++) {
        const param = STATE.parameters.get(`FLTMODE${i}`);
        if (param) {
            const sel = document.getElementById(`fltmode-${i}`);
            if (sel) sel.value = String(Math.round(param.value));
        }
    }
}

// Failsafe parameter mappings: element ID -> parameter name
const FS_MAP = {
    'fs-batt-low-volt': 'BATT_LOW_VOLT',
    'fs-batt-crt-volt': 'BATT_CRT_VOLT',
    'fs-batt-low-act': 'BATT_FS_LOW_ACT',
    'fs-batt-crt-act': 'BATT_FS_CRT_ACT',
    'fs-thr-enable': 'FS_THR_ENABLE',
    'fs-thr-value': 'FS_THR_VALUE',
    'fs-gcs-enable': 'FS_GCS_ENABLE',
};

/**
 * Initialize Failsafe sub-tab
 */
function initFailsafe() {
    bindBtn('fs-read', async () => {
        for (const paramName of Object.values(FS_MAP)) {
            await requestParameter(paramName);
        }
        setTimeout(populateFailsafe, 1500);
    });

    bindBtn('fs-write', async () => {
        let count = 0;
        for (const [elId, paramName] of Object.entries(FS_MAP)) {
            const el = document.getElementById(elId);
            if (!el) continue;
            const val = parseFloat(el.value);
            if (isNaN(val)) continue;
            await setParameter(paramName, val);
            count++;
        }
        alert(`Written ${count} failsafe parameters`);
    });

    // Auto-populate when params arrive
    onMessage(22, () => populateFailsafe());
}

function populateFailsafe() {
    for (const [elId, paramName] of Object.entries(FS_MAP)) {
        const param = STATE.parameters.get(paramName);
        if (!param) continue;
        const el = document.getElementById(elId);
        if (!el) continue;
        if (el.tagName === 'SELECT') {
            el.value = String(Math.round(param.value));
        } else {
            el.value = param.value;
        }
    }
}

// Parameter descriptions (common ArduPilot params)
const PARAM_DESCRIPTIONS = {
    ATC_RAT_RLL_P: 'Roll rate controller P gain', ATC_RAT_RLL_I: 'Roll rate controller I gain',
    ATC_RAT_RLL_D: 'Roll rate controller D gain', ATC_RAT_RLL_FF: 'Roll rate controller feed forward',
    ATC_RAT_PIT_P: 'Pitch rate controller P gain', ATC_RAT_PIT_I: 'Pitch rate controller I gain',
    ATC_RAT_PIT_D: 'Pitch rate controller D gain', ATC_RAT_PIT_FF: 'Pitch rate controller feed forward',
    ATC_RAT_YAW_P: 'Yaw rate controller P gain', ATC_RAT_YAW_I: 'Yaw rate controller I gain',
    ATC_RAT_YAW_D: 'Yaw rate controller D gain', ATC_RAT_YAW_FF: 'Yaw rate controller feed forward',
    ATC_ANG_RLL_P: 'Roll angle controller P gain', ATC_ANG_PIT_P: 'Pitch angle controller P gain',
    ATC_ANG_YAW_P: 'Yaw angle controller P gain',
    ANGLE_MAX: 'Max lean angle (centideg)', LOIT_SPEED: 'Loiter max horizontal speed (cm/s)',
    WPNAV_SPEED: 'Waypoint horizontal speed (cm/s)', WPNAV_SPEED_UP: 'Waypoint climb speed (cm/s)',
    WPNAV_SPEED_DN: 'Waypoint descent speed (cm/s)', WPNAV_ACCEL: 'Waypoint horizontal accel (cm/s/s)',
    WPNAV_RADIUS: 'Waypoint acceptance radius (cm)',
    RTL_ALT: 'RTL altitude (cm above home)', RTL_ALT_FINAL: 'RTL final altitude (cm)',
    LAND_SPEED: 'Final landing speed (cm/s)', LAND_SPEED_HIGH: 'Landing speed until close (cm/s)',
    PSC_POSXY_P: 'Position XY P gain', PSC_VELXY_P: 'Velocity XY P gain',
    PSC_POSZ_P: 'Position Z P gain', PSC_VELZ_P: 'Velocity Z P gain',
    BATT_MONITOR: 'Battery monitoring type', BATT_CAPACITY: 'Battery capacity (mAh)',
    BATT_LOW_VOLT: 'Low battery voltage (V)', BATT_CRT_VOLT: 'Critical battery voltage (V)',
    ARMING_CHECK: 'Arming checks bitmask', COMPASS_USE: 'Enable first compass',
    EK3_ENABLE: 'Enable EKF3', AHRS_EKF_TYPE: 'EKF type (2=EKF2, 3=EKF3)',
    FENCE_ENABLE: 'Enable geofence', FENCE_TYPE: 'Fence type bitmask',
    FENCE_ALT_MAX: 'Max altitude fence (m)', FENCE_RADIUS: 'Circular fence radius (m)',
    GPS_TYPE: 'GPS receiver type', INS_GYRO_FILTER: 'Gyro LPF frequency (Hz)',
    INS_ACCEL_FILTER: 'Accel LPF frequency (Hz)',
    MOT_SPIN_ARM: 'Motor spin when armed', MOT_SPIN_MIN: 'Motor min spin flying',
    MOT_THST_HOVER: 'Throttle hover value', RC_SPEED: 'ESC update speed (Hz)',
    SERIAL0_BAUD: 'Serial 0 baud rate', SERIAL0_PROTOCOL: 'Serial 0 protocol',
    SYSID_THISMAV: 'MAVLink system ID', FRAME_CLASS: 'Frame class',
    FRAME_TYPE: 'Frame type', FS_THR_ENABLE: 'Throttle failsafe enable',
    FS_GCS_ENABLE: 'GCS failsafe enable', LOG_BITMASK: 'Log bitmask',
};

const GROUP_HINTS = {
    ATC: 'Attitude controller', PSC: 'Position/velocity controller', WPNAV: 'Waypoint navigation',
    LOIT: 'Loiter mode', RTL: 'Return-to-launch', BATT: 'Battery monitor',
    COMPASS: 'Compass/magnetometer', EK2: 'EKF2', EK3: 'EKF3', INS: 'Inertial sensor',
    MOT: 'Motor output', RC: 'RC input', SERVO: 'Servo output', SERIAL: 'Serial port',
    GPS: 'GPS receiver', FENCE: 'Geofence', LOG: 'Logging', FS: 'Failsafe',
    PILOT: 'Pilot input', LAND: 'Landing', ARMING: 'Arming check', BRD: 'Board config',
    AHRS: 'Attitude/heading ref', TERRAIN: 'Terrain following', FLTMODE: 'Flight mode',
    FRAME: 'Vehicle frame', SYSID: 'System ID', SR: 'Telemetry stream rate',
    MIS: 'Mission', RALLY: 'Rally point', NTF: 'Notification', RNGFND: 'Rangefinder',
};

function getParamDescription(name) {
    if (PARAM_DESCRIPTIONS[name]) return PARAM_DESCRIPTIONS[name];
    const base = name.replace(/\d+/, 'n');
    for (const [key, desc] of Object.entries(PARAM_DESCRIPTIONS)) {
        if (key.replace(/\d+/, 'n') === base) return desc;
    }
    const prefix = name.split('_')[0];
    return GROUP_HINTS[prefix] || '';
}

let cfgSearchFilter = '';
let cfgChangedParams = new Map(); // Track changed values

function renderCfgParamsTable() {
    const tbody = document.getElementById('cfg-params-table-body');
    if (!tbody) return;

    const params = Array.from(STATE.parameters.entries())
        .filter(([name]) => !cfgSearchFilter || name.includes(cfgSearchFilter))
        .sort((a, b) => a[0].localeCompare(b[0]));

    const visible = params.slice(0, 200);

    tbody.innerHTML = visible.map(([name, param]) => {
        const desc = getParamDescription(name);
        const val = formatParamValue(param.value, param.type);
        return `<tr>
            <td class="param-name">${name}</td>
            <td><input class="param-val-input" type="text" value="${val}"
                       data-cfg-param="${name}" data-param-type="${param.type}"></td>
            <td class="param-desc">${desc}</td>
        </tr>`;
    }).join('');

    if (params.length > 200) {
        tbody.innerHTML += `<tr><td colspan="3" class="param-desc" style="text-align:center;padding:12px;">
            ... ${params.length - 200} more parameters (refine search)
        </td></tr>`;
    }

    // Track changes
    tbody.querySelectorAll('input[data-cfg-param]').forEach(input => {
        input.addEventListener('change', (e) => {
            const paramName = e.target.dataset.cfgParam;
            const paramType = parseInt(e.target.dataset.paramType) || 9;
            const raw = e.target.value.trim();
            const newValue = (paramType >= 1 && paramType <= 6)
                ? parseInt(raw, 10)
                : parseFloat(raw);
            if (isNaN(newValue)) return;
            const orig = STATE.parameters.get(paramName);
            if (orig && orig.value !== newValue) {
                cfgChangedParams.set(paramName, { value: newValue, type: paramType });
                e.target.style.borderColor = '#ffaa00';
            } else {
                cfgChangedParams.delete(paramName);
                e.target.style.borderColor = '';
            }
        });
    });
}

function updateCfgProgress() {
    const fillEl = document.getElementById('cfg-params-fill');
    const countEl = document.getElementById('cfg-params-count');
    const progressEl = document.getElementById('cfg-params-progress');
    if (!fillEl || !countEl) return;
    // A single catalog read still carries the vehicle's full paramCount, so
    // without this it would paint the bar as "1/1300" — a stalled download.
    if (!isFullReadActive()) return;

    if (STATE.parameterCount > 0) {
        if (progressEl) progressEl.style.display = 'flex';
        const pct = (STATE.parametersReceived / STATE.parameterCount * 100).toFixed(0);
        fillEl.style.width = pct + '%';
        countEl.textContent = `${STATE.parametersReceived}/${STATE.parameterCount}`;
    }
}

/**
 * Fill every input on the PID screen from the parameters already downloaded.
 *
 * Values used to arrive only through the PARAM_VALUE listener, so the screen
 * stayed at 0 unless a download happened to be running while it was open —
 * even when the parameter was sitting in STATE all along.
 */
function populatePidInputs() {
    applyPidVehicleVisibility();
    document.querySelectorAll('.pid-input[data-param]').forEach(input => {
        const param = STATE.parameters.get(input.dataset.param);
        input.value = param ? Number(param.value).toFixed(4) : '';
        input.placeholder = param ? '' : '--';
    });
}

/**
 * Show only the panels that apply to the connected vehicle class.
 *
 * MAV_TYPE 0 means no HEARTBEAT has identified the airframe yet. Note that
 * getVehicleTypeName() falls back to 'Copter' rather than reporting unknown,
 * so the vehicleType has to be tested directly — otherwise a disconnected GCS
 * would hide every Plane panel behind a guess.
 */
let lastPidVehicleType = null;

function applyPidVehicleVisibility() {
    const known = STATE.vehicleType > 0;
    const vehicle = known ? getVehicleTypeName(STATE.vehicleType) : null;
    lastPidVehicleType = STATE.vehicleType;
    document.querySelectorAll('#subtab-pid-tuning .setup-panel[data-vehicle]').forEach(panel => {
        const applies = panel.dataset.vehicle.split(/\s+/);
        panel.classList.toggle('hidden-vehicle', known && !applies.includes(vehicle));
    });
}

function setPidStatus(text, cls = '') {
    const el = document.getElementById('pid-status');
    if (!el) return;
    el.textContent = text;
    el.className = 'pid-status' + (cls ? ' ' + cls : '');
}

/**
 * Initialize Config/Tuning tab
 */
function initConfigTuningTab() {
    // PID read: one PARAM_REQUEST_READ per visible field. Targeted reads, not a
    // full PARAM_REQUEST_LIST — this screen is ~30 values, the full list is
    // over a thousand.
    const readAllBtn = document.getElementById('pid-read-all');
    if (readAllBtn) {
        readAllBtn.addEventListener('click', async () => {
            applyPidVehicleVisibility();
            const inputs = Array.from(document.querySelectorAll(
                '#subtab-pid-tuning .setup-panel:not(.hidden-vehicle) .pid-input[data-param]'));
            const names = [...new Set(inputs.map(i => i.dataset.param))];
            readAllBtn.disabled = true;
            let done = 0;
            try {
                for (const name of names) {
                    setPidStatus(`Reading ${++done}/${names.length}...`);
                    try { await requestParameter(name); } catch (e) { /* keep going */ }
                    await new Promise(r => setTimeout(r, 60));
                }
                // Answers arrive asynchronously, so settle before painting
                await new Promise(r => setTimeout(r, 600));
                populatePidInputs();
                const missing = names.filter(n => !STATE.parameters.has(n));
                if (missing.length) {
                    setPidStatus(`${names.length - missing.length}/${names.length} read — ${missing.length} unsupported on this firmware`, '');
                } else {
                    setPidStatus(`${names.length} parameters read`, 'ok');
                }
            } finally {
                readAllBtn.disabled = false;
            }
        });
    }

    // PID write all button
    const writeAllBtn = document.getElementById('pid-write-all');
    if (writeAllBtn) {
        writeAllBtn.addEventListener('click', async () => {
            const inputs = document.querySelectorAll(
                '#subtab-pid-tuning .setup-panel:not(.hidden-vehicle) .pid-input[data-param]');
            let count = 0;
            for (const input of inputs) {
                const paramName = input.dataset.param;
                const value = parseFloat(input.value);
                if (!isNaN(value)) {
                    try {
                        await setParameter(paramName, value);
                        input.style.borderColor = '#44ff44';
                        count++;
                    } catch (e) {
                        input.style.borderColor = '#ff4444';
                    }
                }
            }
            setPidStatus(`${count} parameters written`, 'ok');
            setTimeout(() => {
                inputs.forEach(i => i.style.borderColor = '');
            }, 2000);
        });
    }

    // The catalog's single reads land in STATE.parameters; the table beside it
    // is ours, so it redraws through this.
    setParamsTableRenderer(renderCfgParamsTable);

    // Config params - READ ALL
    const cfgReadBtn = document.getElementById('cfg-params-read');
    if (cfgReadBtn) {
        cfgReadBtn.addEventListener('click', async () => {
            const progressEl = document.getElementById('cfg-params-progress');
            if (progressEl) progressEl.style.display = 'flex';
            beginFullRead();
            await requestAllParameters();
        });
    }

    // Config params - WRITE CHANGED
    const cfgWriteBtn = document.getElementById('cfg-params-write');
    if (cfgWriteBtn) {
        cfgWriteBtn.addEventListener('click', async () => {
            if (cfgChangedParams.size === 0) {
                alert('No parameters changed. Edit values first.');
                return;
            }
            let count = 0;
            for (const [name, p] of cfgChangedParams) {
                try {
                    await setParameter(name, p.value, p.type);
                    count++;
                } catch (e) {
                    console.error(`Failed to write ${name}:`, e);
                }
            }
            alert(`Written ${count}/${cfgChangedParams.size} parameters`);
            cfgChangedParams.clear();
            renderCfgParamsTable();
        });
    }

    // Config params - search (debounced)
    const cfgSearchInput = document.getElementById('cfg-params-search');
    let cfgSearchDebounce = null;
    if (cfgSearchInput) {
        cfgSearchInput.addEventListener('input', (e) => {
            cfgSearchFilter = e.target.value.toUpperCase();
            clearTimeout(cfgSearchDebounce);
            cfgSearchDebounce = setTimeout(() => renderCfgParamsTable(), 250);
        });
    }

    // Extended Tuning sliders - live value display
    document.querySelectorAll('.tuning-slider').forEach(slider => {
        const valEl = document.getElementById(slider.id + '-val');
        if (valEl) {
            slider.addEventListener('input', () => {
                valEl.textContent = slider.value;
            });
        }
    });

    // Extended Tuning READ
    bindBtn('ext-tuning-read', async () => {
        const sliders = document.querySelectorAll('.tuning-slider[data-params]');
        const paramNames = new Set();
        sliders.forEach(s => s.dataset.params.split(',').forEach(p => paramNames.add(p)));
        for (const name of paramNames) {
            await requestParameter(name);
        }
        setTimeout(populateExtTuning, 1500);
    });

    // Extended Tuning WRITE
    bindBtn('ext-tuning-write', async () => {
        let count = 0;
        const sliders = document.querySelectorAll('.tuning-slider[data-params]');
        for (const slider of sliders) {
            const val = parseFloat(slider.value);
            if (isNaN(val)) continue;
            const params = slider.dataset.params.split(',');
            for (const paramName of params) {
                await setParameter(paramName.trim(), val);
                count++;
            }
        }
        alert(`Written ${count} tuning parameters`);
    });

    // Vibration display
    initVibrationDisplay();

    // Servo/Relay
    initServoRelay();

    // Update PID inputs and config params table when parameters are received (throttled)
    let cfgParamRenderPending = false;
    onMessage(22, (data) => {
        // Update PID inputs if they match
        const input = document.querySelector(`.pid-input[data-param="${data.paramId}"]`);
        if (input) {
            input.value = data.paramValue.toFixed(4);
            input.placeholder = '';
        }

        // Update extended tuning sliders if they match
        updateExtTuningSlider(data.paramId, data.paramValue);

        // The airframe can be identified after this screen is already open
        if (STATE.vehicleType !== lastPidVehicleType) applyPidVehicleVisibility();

        // Update config params table (throttled to avoid lag during bulk param read)
        updateCfgProgress();
        if (currentTab === 'setup' && !cfgParamRenderPending) {
            cfgParamRenderPending = true;
            setTimeout(() => {
                cfgParamRenderPending = false;
                renderCfgParamsTable();
            }, 500);
        }
    });
}

function populateExtTuning() {
    document.querySelectorAll('.tuning-slider[data-params]').forEach(slider => {
        const firstParam = slider.dataset.params.split(',')[0].trim();
        const param = STATE.parameters.get(firstParam);
        if (param) {
            slider.value = param.value;
            const valEl = document.getElementById(slider.id + '-val');
            if (valEl) valEl.textContent = Number(param.value).toFixed(3);
        }
    });
}

function updateExtTuningSlider(paramId, value) {
    document.querySelectorAll('.tuning-slider[data-params]').forEach(slider => {
        const params = slider.dataset.params.split(',').map(p => p.trim());
        if (params.includes(paramId)) {
            slider.value = value;
            const valEl = document.getElementById(slider.id + '-val');
            if (valEl) valEl.textContent = Number(value).toFixed(3);
        }
    });
}

/**
 * Initialize Simulation tab
 */
function initSimulationTab() {
    const statusEl = document.getElementById('sitl-status');

    // SITL status updates from main process
    if (window.sitl && window.sitl.onStatusUpdate) {
        window.sitl.onStatusUpdate((data) => {
            if (statusEl) statusEl.textContent = data.message || data.state;
        });
    }

    // Download button
    const downloadBtn = document.getElementById('sitl-download-btn');
    if (downloadBtn) {
        downloadBtn.addEventListener('click', async () => {
            const vehicle = document.getElementById('sitl-vehicle')?.value || 'copter';
            const version = document.getElementById('sitl-version')?.value || 'stable';
            downloadBtn.disabled = true;
            downloadBtn.textContent = 'DOWNLOADING...';
            try {
                await window.sitl.download(vehicle, version);
                downloadBtn.textContent = 'DOWNLOADED';
                setTimeout(() => { downloadBtn.textContent = 'DOWNLOAD'; downloadBtn.disabled = false; }, 2000);
            } catch (e) {
                if (statusEl) statusEl.textContent = 'Download failed: ' + e.message;
                downloadBtn.textContent = 'DOWNLOAD';
                downloadBtn.disabled = false;
            }
        });
    }

    // Launch & Connect button
    const launchBtn = document.getElementById('sitl-launch-btn');
    if (launchBtn) {
        launchBtn.addEventListener('click', async () => {
            const vehicle = document.getElementById('sitl-vehicle')?.value || 'copter';
            const version = document.getElementById('sitl-version')?.value || 'stable';
            const homeLat = parseFloat(document.getElementById('sitl-home-lat')?.value) || 47.2603;
            const homeLon = parseFloat(document.getElementById('sitl-home-lon')?.value) || 11.3439;
            const speedup = parseInt(document.getElementById('sitl-speedup')?.value) || 1;

            // Get terrain elevation at home position — allow retry if a prior attempt failed
            resetAutoDownloadFailures();
            const terrainElev = await getTerrainElevationAsync(homeLat, homeLon);
            const homeAlt = (terrainElev !== null && terrainElev > 0) ? terrainElev : 0;
            console.log(`[sitl] Home: ${homeLat}, ${homeLon}, terrain=${terrainElev}, homeAlt=${homeAlt}`);

            launchBtn.disabled = true;
            launchBtn.textContent = 'STARTING...';

            try {
                // Check if binary exists, download if not
                const exists = await window.sitl.checkBinary(vehicle, version);
                if (!exists) {
                    if (statusEl) statusEl.textContent = 'Binary not found, downloading...';
                    await window.sitl.download(vehicle, version);
                }

                // Launch SITL
                const result = await window.sitl.launch(vehicle, version, {
                    homeLat, homeLon, homeAlt, speedup
                });

                // Auto-connect using the connection type returned by SITL
                if (result && result.success) {
                    const connType = result.connectionType || 'mavlink-udp';
                    const connHost = result.host || '127.0.0.1';
                    const connPort = result.port || 14550;
                    setTimeout(async () => {
                        try {
                            await connect(connType, { host: connHost, port: connPort });
                            if (statusEl) statusEl.textContent = `${vehicle} SITL running — connected (${connType})`;
                        } catch (e) {
                            if (statusEl) statusEl.textContent = `SITL running but connection failed: ${e.message}`;
                        }
                    }, 1000);
                }

                launchBtn.textContent = 'LAUNCH & CONNECT';
                launchBtn.disabled = false;
            } catch (e) {
                if (statusEl) statusEl.textContent = 'Launch failed: ' + e.message;
                launchBtn.textContent = 'LAUNCH & CONNECT';
                launchBtn.disabled = false;
            }
        });
    }

    // Stop button
    const stopBtn = document.getElementById('sitl-stop-btn');
    if (stopBtn) {
        stopBtn.addEventListener('click', async () => {
            try {
                await disconnect();
                await window.sitl.stop();
                if (statusEl) statusEl.textContent = 'SITL stopped';
            } catch (e) {
                if (statusEl) statusEl.textContent = 'Stop error: ' + e.message;
            }
        });
    }

    // Manual UDP connect button
    const connectBtn = document.getElementById('sitl-connect');
    if (connectBtn) {
        connectBtn.addEventListener('click', async () => {
            const host = document.getElementById('sitl-host')?.value || '127.0.0.1';
            const port = parseInt(document.getElementById('sitl-port')?.value) || 14550;
            try {
                await connect('mavlink-udp', { host, port });
                if (statusEl) statusEl.textContent = 'Connected to SITL';
            } catch (e) {
                alert('SITL connection failed: ' + e.message);
            }
        });
    }
}

// ============================================================
// RTK / GPS TAB
// ============================================================

function initRTKTab() {
    if (!window.rtk) return;

    const NTRIP_STORAGE_KEY = 'ntrip-settings';
    const statusEl = document.getElementById('rtk-conn-status');
    const sourceSelect = document.getElementById('rtk-source');
    const ntripPanel = document.getElementById('rtk-ntrip-panel');
    const serialPanel = document.getElementById('rtk-serial-panel');
    let ntripConnected = false;
    let ggaFeedInterval = null;

    // Source selector: show only the relevant source panel
    function updateSourcePanels() {
        const src = sourceSelect ? sourceSelect.value : 'ntrip';
        if (ntripPanel) ntripPanel.style.display = src === 'ntrip' ? '' : 'none';
        if (serialPanel) serialPanel.style.display = src === 'serial' ? '' : 'none';
    }
    if (sourceSelect) {
        sourceSelect.addEventListener('change', updateSourcePanels);
        updateSourcePanels();
    }

    // Scan ports
    const scanBtn = document.getElementById('rtk-scan-ports');
    if (scanBtn) {
        const doScan = async () => {
            const portSelect = document.getElementById('rtk-serial-port');
            if (!portSelect) return;
            const ports = await window.rtk.listPorts();
            portSelect.innerHTML = '<option value="">Select port...</option>';
            ports.forEach(p => {
                portSelect.innerHTML += `<option value="${p.path}">${portLabel(p)}</option>`;
            });
        };
        scanBtn.addEventListener('click', doScan);
        // Auto-scan on first tab visit
        doScan();
    }

    // Connect (serial base station)
    const connectBtn = document.getElementById('rtk-connect-btn');
    if (connectBtn) {
        connectBtn.addEventListener('click', async () => {
            const portPath = document.getElementById('rtk-serial-port')?.value;
            const baudRate = parseInt(document.getElementById('rtk-baud')?.value) || 115200;
            if (!portPath) { alert('Select a serial port first'); return; }
            try {
                await window.rtk.connect(portPath, baudRate);
                if (statusEl) statusEl.textContent = `Connected to ${portPath}`;
            } catch (e) {
                alert('RTK connect failed: ' + e.message);
            }
        });
    }

    // Disconnect (serial base station)
    const disconnectBtn = document.getElementById('rtk-disconnect-btn');
    if (disconnectBtn) {
        disconnectBtn.addEventListener('click', async () => {
            await window.rtk.disconnect();
            if (statusEl) statusEl.textContent = 'Disconnected';
        });
    }

    // ── NTRIP ────────────────────────────────────────────────────────

    // Restore saved NTRIP settings
    try {
        const saved = JSON.parse(localStorage.getItem(NTRIP_STORAGE_KEY));
        if (saved) {
            const set = (id, val) => { const el = document.getElementById(id); if (el && val != null) el.value = val; };
            set('ntrip-host', saved.host);
            set('ntrip-port', saved.port);
            set('ntrip-mount', saved.mountpoint);
            set('ntrip-user', saved.username);
            set('ntrip-gga-mode', saved.ggaMode);
            set('ntrip-gga-interval', saved.ggaInterval);
            set('ntrip-gga-lat', saved.lat);
            set('ntrip-gga-lon', saved.lon);
            set('ntrip-gga-alt', saved.alt);
            const tlsEl = document.getElementById('ntrip-tls');
            if (tlsEl) tlsEl.checked = !!saved.tls;
            if (saved.source && sourceSelect) { sourceSelect.value = saved.source; updateSourcePanels(); }
        }
    } catch (e) { /* ignore */ }

    // GGA manual position fields visibility
    const ggaModeSelect = document.getElementById('ntrip-gga-mode');
    function updateGgaFields() {
        const manual = ggaModeSelect && ggaModeSelect.value === 'manual';
        document.querySelectorAll('.ntrip-gga-manual').forEach(el => {
            el.style.display = manual ? '' : 'none';
        });
    }
    if (ggaModeSelect) {
        ggaModeSelect.addEventListener('change', updateGgaFields);
        updateGgaFields();
    }

    function readNtripConfig() {
        return {
            host: document.getElementById('ntrip-host')?.value.trim() || '',
            port: parseInt(document.getElementById('ntrip-port')?.value) || 2101,
            mountpoint: document.getElementById('ntrip-mount')?.value.trim() || '',
            username: document.getElementById('ntrip-user')?.value || '',
            password: document.getElementById('ntrip-pass')?.value || '',
            tls: document.getElementById('ntrip-tls')?.checked || false,
            ggaMode: document.getElementById('ntrip-gga-mode')?.value || 'off',
            lat: parseFloat(document.getElementById('ntrip-gga-lat')?.value),
            lon: parseFloat(document.getElementById('ntrip-gga-lon')?.value),
            alt: parseFloat(document.getElementById('ntrip-gga-alt')?.value) || 0,
            ggaInterval: parseInt(document.getElementById('ntrip-gga-interval')?.value) || 10
        };
    }

    function saveNtripConfig(cfg) {
        try {
            localStorage.setItem(NTRIP_STORAGE_KEY, JSON.stringify({
                host: cfg.host, port: cfg.port, mountpoint: cfg.mountpoint,
                username: cfg.username, tls: cfg.tls,
                ggaMode: cfg.ggaMode, ggaInterval: cfg.ggaInterval,
                lat: cfg.lat, lon: cfg.lon, alt: cfg.alt,
                source: sourceSelect ? sourceSelect.value : 'ntrip'
            }));
        } catch (e) { /* ignore */ }
    }

    // Fetch sourcetable and populate the mountpoint list
    const srcTableBtn = document.getElementById('ntrip-sourcetable-btn');
    if (srcTableBtn) {
        srcTableBtn.addEventListener('click', async () => {
            const cfg = readNtripConfig();
            if (!cfg.host) { alert('Enter the caster host first'); return; }
            srcTableBtn.disabled = true;
            srcTableBtn.textContent = '...';
            try {
                const entries = await window.rtk.ntripGetSourcetable(cfg);
                const listRow = document.getElementById('ntrip-mount-list-row');
                const listSelect = document.getElementById('ntrip-mount-list');
                if (listSelect) {
                    listSelect.innerHTML = '';
                    const placeholder = document.createElement('option');
                    placeholder.value = '';
                    placeholder.textContent = `-- ${entries.length} mountpoints --`;
                    listSelect.appendChild(placeholder);
                    for (const e of entries) {
                        const opt = document.createElement('option');
                        opt.value = e.mountpoint;
                        const details = [e.format, e.navSystem, e.country].filter(Boolean).join(' · ');
                        opt.textContent = e.mountpoint + (details ? ` (${details})` : '');
                        listSelect.appendChild(opt);
                    }
                }
                if (listRow) listRow.style.display = entries.length ? '' : 'none';
                if (!entries.length) alert('Caster returned an empty sourcetable');
            } catch (e) {
                alert('Sourcetable fetch failed: ' + e.message);
            } finally {
                srcTableBtn.disabled = false;
                srcTableBtn.textContent = 'LIST';
            }
        });
    }

    // Picking from the sourcetable fills the mountpoint field
    const mountListSelect = document.getElementById('ntrip-mount-list');
    if (mountListSelect) {
        mountListSelect.addEventListener('change', () => {
            if (mountListSelect.value) {
                const mountInput = document.getElementById('ntrip-mount');
                if (mountInput) mountInput.value = mountListSelect.value;
            }
        });
    }

    // NTRIP connect
    const ntripConnectBtn = document.getElementById('ntrip-connect-btn');
    if (ntripConnectBtn) {
        ntripConnectBtn.addEventListener('click', async () => {
            const cfg = readNtripConfig();
            if (!cfg.host || !cfg.mountpoint) { alert('Caster host and mountpoint are required'); return; }
            ntripConnectBtn.disabled = true;
            if (statusEl) statusEl.textContent = 'Connecting to caster...';
            try {
                await window.rtk.ntripConnect(cfg);
                saveNtripConfig(cfg);
            } catch (e) {
                if (statusEl) statusEl.textContent = 'Error: ' + e.message;
                alert('NTRIP connect failed: ' + e.message);
            } finally {
                ntripConnectBtn.disabled = false;
            }
        });
    }

    // NTRIP disconnect
    const ntripDisconnectBtn = document.getElementById('ntrip-disconnect-btn');
    if (ntripDisconnectBtn) {
        ntripDisconnectBtn.addEventListener('click', async () => {
            await window.rtk.ntripDisconnect();
            if (statusEl) statusEl.textContent = 'Disconnected';
        });
    }

    // Feed the vehicle position to the main process for GGA upload (1 Hz,
    // only while an NTRIP session is up)
    function startGgaFeed() {
        if (ggaFeedInterval) return;
        ggaFeedInterval = setInterval(() => {
            if (!ntripConnected) return;
            if (isFinite(STATE.lat) && isFinite(STATE.lon) && (STATE.lat !== 0 || STATE.lon !== 0)) {
                window.rtk.feedPosition({ lat: STATE.lat, lon: STATE.lon, alt: STATE.rawAlt || 0 });
            }
        }, 1000);
    }
    function stopGgaFeed() {
        if (ggaFeedInterval) { clearInterval(ggaFeedInterval); ggaFeedInterval = null; }
    }

    // RTCM injection is handled directly in the main process (rtk-manager.js)
    // via raw MAVLink GPS_RTCM_DATA packets sent over the active connection.

    // Status updates from main process
    if (window.rtk.onStatusUpdate) {
        window.rtk.onStatusUpdate((data) => {
            STATE.rtkBaseConnected = data.connected;
            STATE.rtkBaseMsgPerSec = data.rtcmMsgPerSec || 0;

            ntripConnected = data.connected && data.source === 'ntrip';
            if (ntripConnected) startGgaFeed(); else stopGgaFeed();
            setNavDot('rtk-gps', data.connected);

            // Connection status
            if (statusEl) {
                if (!data.connected) {
                    statusEl.textContent = 'Not connected';
                    statusEl.style.color = '';
                } else if (data.source === 'ntrip' && data.ntrip) {
                    statusEl.textContent = `NTRIP — ${data.ntrip.host}:${data.ntrip.port}/${data.ntrip.mountpoint}`;
                    statusEl.style.color = 'var(--accent-cyan)';
                } else {
                    statusEl.textContent = `Serial — ${data.portPath}`;
                    statusEl.style.color = 'var(--accent-cyan)';
                }
            }

            // Stream info
            const streamEl = document.getElementById('rtk-stream-status');
            if (streamEl) {
                streamEl.textContent = data.connected
                    ? (data.rtcmMsgPerSec > 0 ? 'Streaming RTCM3' : 'Connected, waiting for data...')
                    : 'No data';
                streamEl.style.color = data.rtcmMsgPerSec > 0 ? '#00ff7f' : '';
            }

            const srcEl = document.getElementById('rtk-active-source');
            if (srcEl) {
                srcEl.textContent = data.connected
                    ? (data.source === 'ntrip' ? 'NTRIP Caster' : 'Serial Base')
                    : '---';
            }

            const rateEl = document.getElementById('rtk-msg-rate');
            if (rateEl) rateEl.textContent = `${data.rtcmMsgPerSec || 0} msg/s`;

            const totalEl = document.getElementById('rtk-msg-total');
            if (totalEl) totalEl.textContent = String(data.rtcmMsgCount || 0);

            const bytesEl = document.getElementById('rtk-bytes-rx');
            if (bytesEl) {
                const bytes = data.bytesReceived || 0;
                bytesEl.textContent = bytes > 1048576
                    ? (bytes / 1048576).toFixed(1) + ' MB'
                    : bytes > 1024
                        ? (bytes / 1024).toFixed(1) + ' KB'
                        : bytes + ' B';
            }

            const ggaEl = document.getElementById('rtk-gga-sent');
            if (ggaEl) ggaEl.textContent = data.ntrip ? String(data.ntrip.ggaSent || 0) : '---';

            const reconnEl = document.getElementById('rtk-reconnects');
            if (reconnEl) reconnEl.textContent = data.ntrip ? String(data.ntrip.reconnects || 0) : '---';

            // Message types list
            const typesEl = document.getElementById('rtk-msg-types');
            if (typesEl && data.rtcmLastTypes && data.rtcmLastTypes.length > 0) {
                typesEl.innerHTML = data.rtcmLastTypes.map(t =>
                    `<div style="display:flex; justify-content:space-between; padding:1px 4px;">` +
                    `<span style="color:#00d2ff;">${t.id}</span>` +
                    `<span style="flex:1; margin-left:8px;">${t.name}</span>` +
                    `</div>`
                ).join('');
            }
        });
    }

    // Update drone RTK display from STATE (run at 4 Hz)
    setInterval(() => {
        const fixEl = document.getElementById('rtk-drone-fix');
        const satEl = document.getElementById('rtk-drone-sat');
        const hdopEl = document.getElementById('rtk-drone-hdop');
        const baselineEl = document.getElementById('rtk-drone-baseline');
        const accuracyEl = document.getElementById('rtk-drone-accuracy');
        const iarEl = document.getElementById('rtk-drone-iar');

        if (fixEl) {
            const fixName = GPS_FIX_NAMES[STATE.gpsFix] || `Fix ${STATE.gpsFix}`;
            fixEl.textContent = fixName;
            if (STATE.gpsFix === 6) { fixEl.style.color = '#00ff7f'; } // RTK Fixed = green
            else if (STATE.gpsFix === 5) { fixEl.style.color = '#ffcc00'; } // RTK Float = yellow
            else if (STATE.gpsFix >= 3) { fixEl.style.color = '#00d2ff'; } // 3D Fix = cyan
            else { fixEl.style.color = '#ff3333'; } // No fix = red
        }
        if (satEl) satEl.textContent = STATE.gpsNumSat || '---';
        if (hdopEl) hdopEl.textContent = STATE.gpsHdop < 99 ? STATE.gpsHdop.toFixed(1) : '---';
        if (baselineEl) {
            const bl = STATE.rtkBaseline;
            baselineEl.textContent = bl > 0 ? (bl / 1000).toFixed(3) + ' m' : '---';
        }
        if (accuracyEl) {
            const acc = STATE.rtkAccuracy;
            accuracyEl.textContent = acc > 0 ? (acc / 10).toFixed(1) + ' mm' : '---';
        }
        if (iarEl) iarEl.textContent = STATE.rtkIar > 0 ? String(STATE.rtkIar) : '---';
    }, 250);
}

// GPS fix names for RTK tab display
const GPS_FIX_NAMES = {
    0: 'No GPS', 1: 'No Fix', 2: '2D Fix', 3: '3D Fix',
    4: 'DGPS', 5: 'RTK Float', 6: 'RTK Fixed'
};

// ============================================================
// TELEMETRY FORWARD TAB
// ============================================================

function initTelForwardTab() {
    if (!window.telForward) return;

    const STORAGE_KEY = 'telfwd-settings';
    let feedInterval = null;
    let displayInterval = null;

    // DOM elements
    const typeSelect = document.getElementById('telfwd-type');
    const portSelect = document.getElementById('telfwd-serial-port');
    const baudSelect = document.getElementById('telfwd-baud');
    const protoSelect = document.getElementById('telfwd-protocol');
    const udpHostInput = document.getElementById('telfwd-udp-host');
    const udpPortInput = document.getElementById('telfwd-udp-port');
    const listenPortInput = document.getElementById('telfwd-listen-port');
    const writeAccessCheck = document.getElementById('telfwd-write-access');
    const writeAccessRow = document.getElementById('telfwd-write-access-row');
    const addBtn = document.getElementById('telfwd-add-btn');
    const addStatus = document.getElementById('telfwd-add-status');
    const outputsList = document.getElementById('telfwd-outputs-list');
    const dispLat = document.getElementById('telfwd-lat');
    const dispLon = document.getElementById('telfwd-lon');
    const dispAlt = document.getElementById('telfwd-alt');
    const dispHdg = document.getElementById('telfwd-heading');
    const dispGs = document.getElementById('telfwd-gs');

    // Restore saved form settings
    try {
        const saved = JSON.parse(localStorage.getItem(STORAGE_KEY));
        if (saved) {
            if (saved.type && typeSelect) typeSelect.value = saved.type;
            if (saved.baudRate && baudSelect) baudSelect.value = String(saved.baudRate);
            if (saved.protocol && protoSelect) protoSelect.value = saved.protocol;
            if (saved.host && udpHostInput) udpHostInput.value = saved.host;
            if (saved.port && udpPortInput) udpPortInput.value = String(saved.port);
            if (saved.listenPort && listenPortInput) listenPortInput.value = String(saved.listenPort);
        }
    } catch (e) { /* ignore */ }

    // Show only the fields relevant to the selected output type / protocol
    function updateFormFields() {
        const type = typeSelect ? typeSelect.value : 'udp-client';
        document.querySelectorAll('#subtab-tel-forward .telfwd-field').forEach(el => {
            el.style.display = el.classList.contains(`telfwd-field-${type}`) ? '' : 'none';
        });
        // Write access only makes sense for MAVLink passthrough
        const isMavlink = !protoSelect || protoSelect.value === 'mavlink';
        if (writeAccessRow) writeAccessRow.style.display = isMavlink ? '' : 'none';
        if (type === 'serial' && portSelect && portSelect.options.length <= 1) scanPorts();
    }
    if (typeSelect) typeSelect.addEventListener('change', updateFormFields);
    if (protoSelect) protoSelect.addEventListener('change', updateFormFields);
    updateFormFields();

    // Scan serial ports
    async function scanPorts() {
        if (!portSelect) return;
        const ports = await window.telForward.listPorts();
        const prev = portSelect.value;
        portSelect.innerHTML = '<option value="">Select port...</option>';
        for (const p of ports) {
            const opt = document.createElement('option');
            opt.value = p.path;
            opt.textContent = portLabel(p);
            portSelect.appendChild(opt);
        }
        if (prev) portSelect.value = prev;
    }

    const scanBtn = document.getElementById('telfwd-scan-ports');
    if (scanBtn) scanBtn.addEventListener('click', scanPorts);

    // Add output
    if (addBtn) {
        addBtn.addEventListener('click', async () => {
            const type = typeSelect ? typeSelect.value : 'udp-client';
            const cfg = {
                type,
                protocol: protoSelect ? protoSelect.value : 'mavlink',
                writeAccess: writeAccessCheck ? writeAccessCheck.checked : false,
                portPath: portSelect ? portSelect.value : '',
                baudRate: baudSelect ? parseInt(baudSelect.value) : 57600,
                host: udpHostInput ? udpHostInput.value.trim() : '',
                port: udpPortInput ? parseInt(udpPortInput.value) : 0,
                listenPort: listenPortInput ? parseInt(listenPortInput.value) : 0
            };

            if (type === 'serial' && !cfg.portPath) {
                if (addStatus) addStatus.textContent = 'Select a serial port first';
                return;
            }
            if (type === 'udp-client' && (!cfg.host || !cfg.port)) {
                if (addStatus) addStatus.textContent = 'Enter UDP host and port';
                return;
            }
            if (type === 'udp-server' && !cfg.listenPort) {
                if (addStatus) addStatus.textContent = 'Enter a listen port';
                return;
            }

            try {
                addBtn.disabled = true;
                await window.telForward.addOutput(cfg);
                if (addStatus) { addStatus.textContent = 'Output started'; addStatus.style.color = 'var(--accent-cyan)'; }
                localStorage.setItem(STORAGE_KEY, JSON.stringify({
                    type, protocol: cfg.protocol, baudRate: cfg.baudRate,
                    host: cfg.host, port: cfg.port, listenPort: cfg.listenPort
                }));
            } catch (e) {
                if (addStatus) { addStatus.textContent = 'Error: ' + e.message; addStatus.style.color = '#ff3333'; }
            } finally {
                addBtn.disabled = false;
            }
        });
    }

    // Remove buttons (event delegation on the outputs list)
    if (outputsList) {
        outputsList.addEventListener('click', async (ev) => {
            const btn = ev.target.closest('.telfwd-remove-btn');
            if (!btn) return;
            const id = parseInt(btn.dataset.id);
            if (isFinite(id)) {
                try { await window.telForward.removeOutput(id); } catch (e) { /* ignore */ }
            }
        });
    }

    // Render the active outputs list (built with createElement \u2014 labels may
    // contain user-entered host strings)
    function renderOutputs(list) {
        if (!outputsList) return;
        outputsList.innerHTML = '';
        if (!list || list.length === 0) {
            const empty = document.createElement('span');
            empty.className = 'telfwd-empty';
            empty.style.cssText = 'opacity:0.5; font-size:11px;';
            empty.textContent = 'No outputs configured';
            outputsList.appendChild(empty);
            return;
        }
        for (const o of list) {
            const row = document.createElement('div');
            row.className = 'telfwd-output-row' + (o.connected ? '' : ' disconnected');

            const head = document.createElement('div');
            head.className = 'telfwd-output-head';

            const title = document.createElement('span');
            title.className = 'telfwd-output-title';
            title.textContent = o.label;
            head.appendChild(title);

            if (o.writeAccess) {
                const badge = document.createElement('span');
                badge.className = 'telfwd-badge';
                badge.title = 'Write access: packets from this endpoint are injected into the vehicle link';
                badge.textContent = 'RW';
                head.appendChild(badge);
            }

            const removeBtn = document.createElement('button');
            removeBtn.className = 'gcs-btn telfwd-remove-btn';
            removeBtn.dataset.id = String(o.id);
            removeBtn.textContent = 'REMOVE';
            head.appendChild(removeBtn);

            const stats = document.createElement('div');
            stats.className = 'telfwd-output-stats';
            const parts = [
                o.protocol === 'mavlink' ? 'MAVLink' : 'LTM',
                `${o.msgPerSec || 0} msg/s`,
                formatBytes(o.bytesSent || 0) + ' TX'
            ];
            if (o.bytesRx > 0) parts.push(formatBytes(o.bytesRx) + ' RX');
            if (o.clientCount != null) parts.push(`${o.clientCount} peer${o.clientCount === 1 ? '' : 's'}`);
            if (!o.connected) parts.push('DISCONNECTED');
            if (o.error) parts.push('ERR: ' + o.error);
            stats.textContent = parts.join(' \u00B7 ');

            row.appendChild(head);
            row.appendChild(stats);
            outputsList.appendChild(row);
        }
    }

    // Feed STATE to main process for LTM encoding
    function startStateFeed() {
        if (feedInterval) return;
        feedInterval = setInterval(() => {
            window.telForward.feedState({
                lat: STATE.lat,
                lon: STATE.lon,
                relAlt: STATE.rawAlt || 0,
                roll: STATE.roll,
                pitch: STATE.pitch,
                yaw: STATE.yaw,
                gs: STATE.gs,
                as: STATE.as,
                vs: STATE.vs,
                batteryVoltage: STATE.batteryVoltage,
                batteryCurrent: STATE.batteryCurrent,
                batteryRemaining: STATE.batteryRemaining,
                linkQuality: STATE.linkQuality,
                gpsFix: STATE.gpsFix,
                gpsNumSat: STATE.gpsNumSat,
                armed: STATE.armed,
                flightMode: STATE.flightMode,
                homeLat: STATE.homeLat,
                homeLon: STATE.homeLon,
                homeAlt: STATE.homeAlt
            });
        }, 200); // 5 Hz
    }

    function stopStateFeed() {
        if (feedInterval) { clearInterval(feedInterval); feedInterval = null; }
    }

    // Update CURRENT DATA display
    function startDisplayUpdate() {
        if (displayInterval) return;
        displayInterval = setInterval(() => {
            if (dispLat) dispLat.textContent = STATE.lat ? STATE.lat.toFixed(7) : '---';
            if (dispLon) dispLon.textContent = STATE.lon ? STATE.lon.toFixed(7) : '---';
            if (dispAlt) dispAlt.textContent = STATE.rawAlt != null ? STATE.rawAlt.toFixed(1) + ' m' : '---';
            if (dispHdg) {
                let hdg = STATE.yaw || 0;
                if (hdg < 0) hdg += 360;
                dispHdg.textContent = hdg.toFixed(0) + '\u00B0';
            }
            if (dispGs) dispGs.textContent = STATE.gs != null ? STATE.gs.toFixed(1) + ' m/s' : '---';
        }, 250);
    }

    function stopDisplayUpdate() {
        if (displayInterval) { clearInterval(displayInterval); displayInterval = null; }
    }

    // Apply an outputs snapshot to the UI and timers
    function applyOutputs(list) {
        renderOutputs(list);
        setNavDot('tel-forward', list.some(o => o.connected));
        if (list.some(o => o.protocol === 'ltm' && o.connected)) startStateFeed();
        else stopStateFeed();
        if (list.length > 0) startDisplayUpdate();
        else stopDisplayUpdate();
    }

    // Status updates from main process: { outputs: [...] }
    window.telForward.onStatusUpdate((data) => {
        applyOutputs((data && data.outputs) || []);
    });

    // Restore the list if outputs already exist (e.g. after a renderer reload)
    window.telForward.getOutputs()
        .then(list => { if (list && list.length) applyOutputs(list); })
        .catch(() => {});

    function formatBytes(bytes) {
        if (bytes < 1024) return bytes + ' B';
        if (bytes < 1048576) return (bytes / 1024).toFixed(1) + ' KB';
        return (bytes / 1048576).toFixed(1) + ' MB';
    }
}

// ============================================================
// RADIO CALIBRATION
// ============================================================
const RC_CH_COUNT = 16;

function initRadioCalibration() {
    const container = document.getElementById('rc-cal-bars');
    if (!container) return;

    // Re-request RC stream to ensure data flows on this page
    if (STATE.connected) {
        requestDataStream(3, 4).catch(() => {}); // Stream 3 = RC_CHANNELS at 4Hz
    }

    // Build channel bars
    let html = '';
    for (let i = 0; i < RC_CH_COUNT; i++) {
        html += `<div class="rc-cal-channel">
            <span class="rc-cal-label">CH${i + 1}</span>
            <div class="rc-cal-bar-outer">
                <div class="rc-cal-bar-center"></div>
                <div class="rc-cal-bar-fill" id="rc-bar-${i}"></div>
            </div>
            <span class="rc-cal-val" id="rc-val-${i}">0</span>
            <span class="rc-cal-minmax" id="rc-mm-${i}">--/--</span>
        </div>`;
    }
    container.innerHTML = html;

    // Start/stop calibration
    bindBtn('rc-cal-start', () => {
        STATE.rcCalibrating = !STATE.rcCalibrating;
        const btn = document.getElementById('rc-cal-start');
        const statusEl = document.getElementById('rc-cal-status');
        if (STATE.rcCalibrating) {
            // Reset min/max
            STATE.rcCalMin = new Array(16).fill(2000);
            STATE.rcCalMax = new Array(16).fill(1000);
            if (btn) { btn.textContent = 'STOP CALIBRATION'; btn.style.borderColor = 'var(--accent-orange)'; }
            if (statusEl) statusEl.textContent = 'Calibrating... move all sticks to extremes';
        } else {
            // Capture trim at center
            for (let i = 0; i < 16; i++) STATE.rcCalTrim[i] = STATE.rcChannels[i];
            if (btn) { btn.textContent = 'START CALIBRATION'; btn.style.borderColor = ''; }
            if (statusEl) statusEl.textContent = 'Calibration stopped. Trim captured.';
        }
    });

    // Save calibration (writes RC1_MIN, RC1_MAX, RC1_TRIM, etc.)
    bindBtn('rc-cal-save', async () => {
        if (STATE.rcCalibrating) { alert('Stop calibration first'); return; }
        let count = 0;
        const errors = [];
        for (let i = 0; i < 16; i++) {
            const ch = i + 1;
            if (STATE.rcCalMin[i] < STATE.rcCalMax[i]) {
                try {
                    console.log(`[RC Cal] Saving CH${ch}: MIN=${STATE.rcCalMin[i]} MAX=${STATE.rcCalMax[i]} TRIM=${STATE.rcCalTrim[i]}`);
                    await setParameter(`RC${ch}_MIN`, STATE.rcCalMin[i]);
                    await setParameter(`RC${ch}_MAX`, STATE.rcCalMax[i]);
                    await setParameter(`RC${ch}_TRIM`, STATE.rcCalTrim[i]);
                    count += 3;
                } catch (e) {
                    console.error(`[RC Cal] Failed to save CH${ch}:`, e.message);
                    errors.push(`CH${ch}: ${e.message}`);
                }
            } else {
                console.log(`[RC Cal] Skipping CH${ch}: min=${STATE.rcCalMin[i]} max=${STATE.rcCalMax[i]} (no valid range)`);
            }
        }
        if (errors.length > 0) {
            alert(`Saved ${count} params, ${errors.length} errors:\n${errors.join('\n')}`);
        } else if (count === 0) {
            alert('No channels calibrated. Run calibration first (START → move sticks → STOP).');
        } else {
            alert(`Saved ${count} RC calibration parameters`);
        }
    });

    // Reset
    bindBtn('rc-cal-reset', () => {
        STATE.rcCalMin = new Array(16).fill(2000);
        STATE.rcCalMax = new Array(16).fill(1000);
        STATE.rcCalTrim = new Array(16).fill(1500);
        STATE.rcCalibrating = false;
        const btn = document.getElementById('rc-cal-start');
        if (btn) { btn.textContent = 'START CALIBRATION'; btn.style.borderColor = ''; }
    });

    // Update bars on RC_CHANNELS (msg 65) and RC_CHANNELS_RAW (msg 35)
    onMessage(65, () => updateRcCalBars());
    onMessage(35, () => updateRcCalBars());
}

function updateRcCalBars() {
    for (let i = 0; i < RC_CH_COUNT; i++) {
        const v = STATE.rcChannels[i];
        if (v === 0 || v === 65535) continue; // No data or unused channel

        // Update min/max during calibration
        if (STATE.rcCalibrating) {
            if (v < STATE.rcCalMin[i]) STATE.rcCalMin[i] = v;
            if (v > STATE.rcCalMax[i]) STATE.rcCalMax[i] = v;
        }

        // Update bar position (1000-2000 range)
        const pct = Math.max(0, Math.min(100, (v - 1000) / 10));
        const bar = document.getElementById(`rc-bar-${i}`);
        if (bar) {
            bar.style.left = Math.min(pct, 50) + '%';
            bar.style.width = Math.abs(pct - 50) + '%';
        }

        const valEl = document.getElementById(`rc-val-${i}`);
        if (valEl) valEl.textContent = v;

        const mmEl = document.getElementById(`rc-mm-${i}`);
        if (mmEl) mmEl.textContent = `${STATE.rcCalMin[i]}/${STATE.rcCalMax[i]}`;
    }
}

// ============================================================
// CALIBRATION WIZARD
// ============================================================
let activeCalibration = null;

function initCalibrationWizard() {
    // Listen to STATUSTEXT for calibration progress
    onMessage(253, (data) => {
        if (!activeCalibration) return;
        const text = (data.text || '').toLowerCase();
        const progressEl = document.getElementById('cal-progress');
        const fillEl = document.getElementById('cal-progress-fill');
        const msgEl = document.getElementById('cal-progress-msg');
        const statusEl = document.getElementById(`cal-${activeCalibration}-status`);

        if (text.includes('calibrat')) {
            if (progressEl) progressEl.style.display = 'block';

            // Parse progress hints from STATUSTEXT
            if (text.includes('place vehicle')) {
                if (msgEl) msgEl.textContent = data.text;
                if (fillEl) fillEl.style.width = '20%';
            } else if (text.includes('side')) {
                if (msgEl) msgEl.textContent = data.text;
                if (fillEl) fillEl.style.width = '50%';
            } else if (text.includes('success') || text.includes('done') || text.includes('complete')) {
                if (fillEl) fillEl.style.width = '100%';
                if (msgEl) msgEl.textContent = 'Calibration complete!';
                if (statusEl) { statusEl.textContent = 'Done'; statusEl.className = 'cal-wizard-status done'; }
                setTimeout(() => { if (progressEl) progressEl.style.display = 'none'; }, 3000);
                activeCalibration = null;
            } else if (text.includes('fail')) {
                if (msgEl) msgEl.textContent = 'Calibration failed: ' + data.text;
                if (fillEl) fillEl.style.width = '0%';
                if (statusEl) { statusEl.textContent = 'Failed'; statusEl.className = 'cal-wizard-status'; }
                activeCalibration = null;
            } else {
                if (msgEl) msgEl.textContent = data.text;
            }
        }
    });

    // Override calibration button handlers to track wizard state
    ['accel', 'compass', 'gyro'].forEach(type => {
        const btn = document.getElementById(`setup-cal-${type}`);
        if (!btn) return;
        // Remove old handlers by replacing the element
        const newBtn = btn.cloneNode(true);
        btn.parentNode.replaceChild(newBtn, btn);

        newBtn.addEventListener('click', async () => {
            if (activeCalibration) { alert('Another calibration is in progress'); return; }
            if (!await confirm(`Start ${type} calibration?`)) return;

            activeCalibration = type;
            const statusEl = document.getElementById(`cal-${type}-status`);
            if (statusEl) { statusEl.textContent = 'Running...'; statusEl.className = 'cal-wizard-status running'; }

            const progressEl = document.getElementById('cal-progress');
            const fillEl = document.getElementById('cal-progress-fill');
            const msgEl = document.getElementById('cal-progress-msg');
            if (progressEl) progressEl.style.display = 'block';
            if (fillEl) fillEl.style.width = '10%';
            if (msgEl) msgEl.textContent = `Starting ${type} calibration...`;

            try {
                if (type === 'accel') await calibrateAccel();
                else if (type === 'compass') await calibrateCompass();
                else if (type === 'gyro') await calibrateGyro();
            } catch (e) {
                alert('Calibration command failed: ' + e.message);
                activeCalibration = null;
                if (statusEl) { statusEl.textContent = 'Error'; statusEl.className = 'cal-wizard-status'; }
            }
        });
    });
}

// ============================================================
// VIBRATION DISPLAY
// ============================================================
let vibAnimFrame = null;

function initVibrationDisplay() {
    const canvas = document.getElementById('vib-chart');
    if (!canvas) return;

    // Update vibration values on msg 241
    onMessage(241, () => {
        const xEl = document.getElementById('vib-x-val');
        const yEl = document.getElementById('vib-y-val');
        const zEl = document.getElementById('vib-z-val');
        if (xEl) xEl.textContent = STATE.vibX.toFixed(1);
        if (yEl) yEl.textContent = STATE.vibY.toFixed(1);
        if (zEl) zEl.textContent = STATE.vibZ.toFixed(1);

        document.getElementById('vib-clip0').textContent = STATE.vibClip0;
        document.getElementById('vib-clip1').textContent = STATE.vibClip1;
        document.getElementById('vib-clip2').textContent = STATE.vibClip2;
    });

    // Animate vibration chart when visible
    function drawVibChart() {
        vibAnimFrame = requestAnimationFrame(drawVibChart);
        if (currentTab !== 'setup') return;

        const ctx = canvas.getContext('2d');
        const w = canvas.width;
        const h = canvas.height;
        ctx.clearRect(0, 0, w, h);

        const hist = STATE.vibHistory;
        if (hist.length < 2) return;

        // Draw threshold lines
        const maxVal = 80;
        const yAt = (v) => h - (v / maxVal) * h;

        ctx.strokeStyle = 'rgba(255,255,0,0.2)';
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.moveTo(0, yAt(30)); ctx.lineTo(w, yAt(30));
        ctx.stroke();
        ctx.strokeStyle = 'rgba(255,0,0,0.2)';
        ctx.beginPath();
        ctx.moveTo(0, yAt(60)); ctx.lineTo(w, yAt(60));
        ctx.stroke();
        ctx.setLineDash([]);

        // Draw X, Y, Z lines
        const colors = ['#ff4444', '#44ff44', '#4488ff'];
        const keys = ['x', 'y', 'z'];
        const step = w / (hist.length - 1);

        keys.forEach((key, ci) => {
            ctx.strokeStyle = colors[ci];
            ctx.lineWidth = 1.5;
            ctx.beginPath();
            hist.forEach((pt, i) => {
                const x = i * step;
                const y = yAt(Math.abs(pt[key]));
                if (i === 0) ctx.moveTo(x, y);
                else ctx.lineTo(x, y);
            });
            ctx.stroke();
        });
    }
    drawVibChart();
}

// ============================================================
// SERVO / RELAY CONTROL
// ============================================================
function initServoRelay() {
    const container = document.getElementById('servo-bars');
    if (!container) return;

    // Build 16 servo output bars
    let html = '';
    for (let i = 0; i < 16; i++) {
        html += `<div class="servo-bar-cell">
            <span class="servo-bar-label">S${i + 1}</span>
            <div class="servo-bar-outer"><div class="servo-bar-fill" id="servo-fill-${i}"></div></div>
            <span class="servo-bar-val" id="servo-val-${i}">0</span>
        </div>`;
    }
    container.innerHTML = html;

    // Update servo bars on SERVO_OUTPUT_RAW (msg 36)
    onMessage(36, () => {
        for (let i = 0; i < 16; i++) {
            const v = STATE.servoOutputs[i];
            const fill = document.getElementById(`servo-fill-${i}`);
            const val = document.getElementById(`servo-val-${i}`);
            if (fill) fill.style.height = Math.max(0, Math.min(100, (v - 1000) / 10)) + '%';
            if (val) val.textContent = v || '0';
        }
    });

    // Relay toggle buttons
    document.querySelectorAll('.relay-btn').forEach(btn => {
        btn.addEventListener('click', async () => {
            const relay = parseInt(btn.dataset.relay);
            const isOn = btn.dataset.state === 'on';
            try {
                await sendRelayToggle(relay, isOn ? 0 : 1);
                btn.dataset.state = isOn ? 'off' : 'on';
                btn.textContent = isOn ? 'OFF' : 'ON';
                btn.classList.toggle('on', !isOn);
            } catch (e) {
                alert('Relay toggle failed: ' + e.message);
            }
        });
    });

    // Servo test slider
    const pwmSlider = document.getElementById('servo-test-pwm');
    const pwmVal = document.getElementById('servo-test-val');
    if (pwmSlider && pwmVal) {
        pwmSlider.addEventListener('input', () => { pwmVal.textContent = pwmSlider.value; });
    }

    bindBtn('servo-test-send', async () => {
        const ch = parseInt(document.getElementById('servo-test-ch')?.value) || 1;
        const pwm = parseInt(document.getElementById('servo-test-pwm')?.value) || 1500;
        await sendServoTest(ch, pwm);
    });
}

/* ═══════════════════════════════════════════════════════════════
   CORV SETUP TAB — Config protocol (0x10/0x11) for CORV INS
═══════════════════════════════════════════════════════════════ */
function initCorvSetupTab() {
    if (!window.corvSerial) return;

    const CFG_STRUCT_SIZE = 109;  // v9: 106 + 3 SSA noise bytes
    const CMD_SET_CONFIG  = 0x01;
    const CMD_GET_CONFIG  = 0x02;
    const CMD_SAVE_CONFIG = 0x03;
    let configSeq = 0;

    // --- DOM refs ---
    const statusEl = document.getElementById('corv-cfg-status');
    const btnRead     = document.getElementById('corv-cfg-btn-read');
    const btnSendSave = document.getElementById('corv-cfg-btn-send-save');
    if (!statusEl || !btnRead) return;

    // --- Toggle helpers ---
    function isToggleOn(id) {
        const el = document.getElementById(id);
        return el ? el.classList.contains('on') : false;
    }
    function setToggle(id, val) {
        const el = document.getElementById(id);
        if (!el) return;
        if (val) el.classList.add('on');
        else el.classList.remove('on');
    }

    // Bind toggle click on all corv-cfg-toggle elements
    document.querySelectorAll('.corv-cfg-toggle').forEach(el => {
        el.addEventListener('click', () => el.classList.toggle('on'));
    });

    // --- Status display ---
    function setCfgStatus(msg, color) {
        statusEl.textContent = msg;
        statusEl.style.color = color || 'var(--text-dim)';
    }

    // --- Scientific notation formatter ---
    function fmtSci(v) {
        if (v === 0) return '0';
        const e = Math.floor(Math.log10(Math.abs(v)));
        if (e >= -2 && e <= 4) return parseFloat(v.toPrecision(6)).toString();
        return v.toExponential(3);
    }

    // --- CRC-16-CCITT (poly 0x1021, init 0xFFFF) ---
    function crc16ccitt(data, offset, length) {
        let crc = 0xFFFF;
        for (let i = offset; i < offset + length; i++) {
            crc ^= data[i] << 8;
            for (let bit = 0; bit < 8; bit++) {
                crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) & 0xFFFF : (crc << 1) & 0xFFFF;
            }
        }
        return crc;
    }

    // --- Send config packet (0x10) via IPC ---
    async function sendConfigPacket(cmdId, cmdData) {
        if (STATE.connectionType !== 'corv-binary') {
            setCfgStatus('Not connected via CORV Binary', 'var(--accent-red)');
            return false;
        }
        const payloadLen = 1 + (cmdData ? cmdData.length : 0);
        const totalLen = 5 + payloadLen + 2;
        const buf = new Uint8Array(totalLen);
        buf[0] = 0xA5;
        buf[1] = 0x5A;
        buf[2] = 0x10;
        buf[3] = payloadLen;
        buf[4] = configSeq++ & 0xFF;
        buf[5] = cmdId;
        if (cmdData) {
            for (let i = 0; i < cmdData.length; i++) buf[6 + i] = cmdData[i];
        }
        const crc = crc16ccitt(buf, 2, 3 + payloadLen);
        buf[5 + payloadLen] = crc & 0xFF;
        buf[5 + payloadLen + 1] = (crc >> 8) & 0xFF;

        try {
            await window.corvSerial.sendConfig(Array.from(buf));
            return true;
        } catch (e) {
            setCfgStatus('Send failed: ' + e.message, 'var(--accent-red)');
            return false;
        }
    }

    // --- Build 109-byte SystemConfig struct from UI (protocol v9) ---
    function buildConfigStruct() {
        const buf = new ArrayBuffer(CFG_STRUCT_SIZE);
        const dv = new DataView(buf);
        let o = 0;

        // GPS (5 bytes)
        dv.setUint8(o, parseInt(document.getElementById('corv-cfg-gps-type').value)); o += 1;
        dv.setUint32(o, parseInt(document.getElementById('corv-cfg-gps-baud').value), true); o += 4;

        // Telemetry (10 bytes)
        dv.setUint32(o, parseInt(document.getElementById('corv-cfg-serial1-baud').value), true); o += 4;
        dv.setUint8(o, parseInt(document.getElementById('corv-cfg-output-proto').value)); o += 1;
        dv.setUint8(o, isToggleOn('corv-cfg-telem-usb') ? 1 : 0); o += 1;
        dv.setUint8(o, isToggleOn('corv-cfg-telem-serial1') ? 1 : 0); o += 1;
        dv.setUint8(o, parseInt(document.getElementById('corv-cfg-nav-rate').value)); o += 1;
        dv.setUint8(o, parseInt(document.getElementById('corv-cfg-debug-rate').value)); o += 1;
        dv.setUint8(o, parseInt(document.getElementById('corv-cfg-raw-rate').value)); o += 1;

        // Feature flags (8 bytes)
        dv.setUint8(o, isToggleOn('corv-cfg-mag') ? 1 : 0); o += 1;
        dv.setUint8(o, 0); o += 1; // reserved (was gps_heading_init)
        dv.setUint8(o, isToggleOn('corv-cfg-earth-rot') ? 1 : 0); o += 1;
        dv.setUint8(o, isToggleOn('corv-cfg-zupt') ? 1 : 0); o += 1;
        dv.setUint8(o, isToggleOn('corv-cfg-accel-lev') ? 1 : 0); o += 1;
        dv.setUint8(o, isToggleOn('corv-cfg-wind') ? 1 : 0); o += 1;
        dv.setUint8(o, isToggleOn('corv-cfg-airspeed') ? 1 : 0); o += 1;
        dv.setUint8(o, isToggleOn('corv-cfg-gps-sim') ? 1 : 0); o += 1;

        // Hardware (3 bytes)
        dv.setUint8(o, 0); o += 1; // baro_sensor_type — auto-detected
        dv.setUint8(o, parseInt(document.getElementById('corv-cfg-airspeed-bus').value)); o += 1;
        dv.setUint8(o, parseInt(document.getElementById('corv-cfg-airspeed-mount').value)); o += 1;

        // Particle Filter: uint16 + 6 floats (26 bytes)
        dv.setUint16(o, parseInt(document.getElementById('corv-cfg-pf-n').value), true); o += 2;
        const pfIds = ['corv-cfg-pf-ess','corv-cfg-pf-rough-att','corv-cfg-pf-rough-pos',
                       'corv-cfg-pf-rough-vel','corv-cfg-pf-gps-h','corv-cfg-pf-gps-v'];
        for (const id of pfIds) { dv.setFloat32(o, parseFloat(document.getElementById(id).value), true); o += 4; }

        // Shared EKF bias noise (5 floats)
        const biasNoiseIds = ['corv-cfg-bias-gyro','corv-cfg-bias-accel','corv-cfg-bias-hiron',
                              'corv-cfg-bias-baro','corv-cfg-bias-wind'];
        for (const id of biasNoiseIds) { dv.setFloat32(o, parseFloat(document.getElementById(id).value), true); o += 4; }

        // Shared EKF initial covariance (5 floats)
        const biasInitIds = ['corv-cfg-init-gbias','corv-cfg-init-abias','corv-cfg-init-hiron',
                             'corv-cfg-init-bbias','corv-cfg-init-wind'];
        for (const id of biasInitIds) { dv.setFloat32(o, parseFloat(document.getElementById(id).value), true); o += 4; }

        // Per-particle EKF process noise (2 floats)
        dv.setFloat32(o, parseFloat(document.getElementById('corv-cfg-ekf-vel-q').value), true); o += 4;
        dv.setFloat32(o, parseFloat(document.getElementById('corv-cfg-ekf-pos-q').value), true); o += 4;

        // board_type + mag_bus (2 bytes)
        dv.setUint8(o, parseInt(document.getElementById('corv-cfg-board-type').value)); o += 1;
        dv.setUint8(o, parseInt(document.getElementById('corv-cfg-mag-bus').value)); o += 1;

        // Airspeed ratio (1 float)
        dv.setFloat32(o, parseFloat(document.getElementById('corv-cfg-airspeed-ratio').value) || 1.0, true); o += 4;

        // Airspeed angular gates (3 bytes, uint8 deg; config v9 layout)
        dv.setUint8(o, parseInt(document.getElementById('corv-cfg-sideslip-noise').value) || 3); o += 1;
        dv.setUint8(o, parseInt(document.getElementById('corv-cfg-aoa-valid').value) || 8); o += 1;
        dv.setUint8(o, parseInt(document.getElementById('corv-cfg-air-angle-legacy').value) || 6); o += 1;

        return new Uint8Array(buf);
    }

    // --- Parse 109-byte SystemConfig struct into UI (protocol v9) ---
    function parseConfigStruct(data) {
        if (data.length < CFG_STRUCT_SIZE) return;
        const dv = new DataView(data.buffer, data.byteOffset, data.length);
        let o = 0;

        // GPS
        document.getElementById('corv-cfg-gps-type').value = dv.getUint8(o); o += 1;
        document.getElementById('corv-cfg-gps-baud').value = dv.getUint32(o, true); o += 4;

        // Telemetry
        document.getElementById('corv-cfg-serial1-baud').value = dv.getUint32(o, true); o += 4;
        document.getElementById('corv-cfg-output-proto').value = dv.getUint8(o); o += 1;
        setToggle('corv-cfg-telem-usb', dv.getUint8(o)); o += 1;
        setToggle('corv-cfg-telem-serial1', dv.getUint8(o)); o += 1;
        document.getElementById('corv-cfg-nav-rate').value = dv.getUint8(o); o += 1;
        document.getElementById('corv-cfg-debug-rate').value = dv.getUint8(o); o += 1;
        document.getElementById('corv-cfg-raw-rate').value = dv.getUint8(o); o += 1;

        // Feature flags (8 bytes)
        setToggle('corv-cfg-mag', dv.getUint8(o)); o += 1;
        o += 1; // reserved (was gps_heading_init)
        setToggle('corv-cfg-earth-rot', dv.getUint8(o)); o += 1;
        setToggle('corv-cfg-zupt', dv.getUint8(o)); o += 1;
        setToggle('corv-cfg-accel-lev', dv.getUint8(o)); o += 1;
        setToggle('corv-cfg-wind', dv.getUint8(o)); o += 1;
        setToggle('corv-cfg-airspeed', dv.getUint8(o)); o += 1;
        setToggle('corv-cfg-gps-sim', dv.getUint8(o)); o += 1;

        // Hardware (3 bytes)
        o += 1; // baro_sensor_type — auto-detected, skip
        document.getElementById('corv-cfg-airspeed-bus').value = dv.getUint8(o); o += 1;
        document.getElementById('corv-cfg-airspeed-mount').value = dv.getUint8(o); o += 1;

        // Particle Filter
        document.getElementById('corv-cfg-pf-n').value = dv.getUint16(o, true); o += 2;
        const pfIds = ['corv-cfg-pf-ess','corv-cfg-pf-rough-att','corv-cfg-pf-rough-pos',
                       'corv-cfg-pf-rough-vel','corv-cfg-pf-gps-h','corv-cfg-pf-gps-v'];
        for (const id of pfIds) { document.getElementById(id).value = fmtSci(dv.getFloat32(o, true)); o += 4; }

        // Shared EKF bias noise
        const biasNoiseIds = ['corv-cfg-bias-gyro','corv-cfg-bias-accel','corv-cfg-bias-hiron',
                              'corv-cfg-bias-baro','corv-cfg-bias-wind'];
        for (const id of biasNoiseIds) { document.getElementById(id).value = fmtSci(dv.getFloat32(o, true)); o += 4; }

        // Shared EKF initial covariance
        const biasInitIds = ['corv-cfg-init-gbias','corv-cfg-init-abias','corv-cfg-init-hiron',
                             'corv-cfg-init-bbias','corv-cfg-init-wind'];
        for (const id of biasInitIds) { document.getElementById(id).value = fmtSci(dv.getFloat32(o, true)); o += 4; }

        // Per-particle EKF process noise
        document.getElementById('corv-cfg-ekf-vel-q').value = fmtSci(dv.getFloat32(o, true)); o += 4;
        document.getElementById('corv-cfg-ekf-pos-q').value = fmtSci(dv.getFloat32(o, true)); o += 4;

        // board_type + mag_bus
        document.getElementById('corv-cfg-board-type').value = dv.getUint8(o); o += 1;
        document.getElementById('corv-cfg-mag-bus').value = dv.getUint8(o); o += 1;

        // Airspeed ratio
        document.getElementById('corv-cfg-airspeed-ratio').value = dv.getFloat32(o, true).toFixed(3); o += 4;

        // Airspeed angular gates (3 bytes, uint8 deg; config v9 layout)
        document.getElementById('corv-cfg-sideslip-noise').value   = dv.getUint8(o); o += 1;
        document.getElementById('corv-cfg-aoa-valid').value        = dv.getUint8(o); o += 1;
        document.getElementById('corv-cfg-air-angle-legacy').value = dv.getUint8(o); o += 1;
    }

    // --- Handle 0x11 Config Response from device ---
    const RESP_NAMES = ['OK', 'ERROR', 'CRC_FAIL', 'INVALID'];
    const CMD_NAMES = { 0x01: 'SET', 0x02: 'GET', 0x03: 'SAVE', 0x04: 'RESET', 0x05: 'REBOOT' };
    let pendingSaveAfterSet = false;

    function handleConfigResponse(payloadArr) {
        const payload = new Uint8Array(payloadArr);
        if (payload.length < 2) return;
        const respCode = payload[0];
        const cmdId = payload[1];
        const respName = RESP_NAMES[respCode] || 'UNKNOWN';
        const cmdName = CMD_NAMES[cmdId] || '0x' + cmdId.toString(16);

        if (respCode === 0 && cmdId === CMD_GET_CONFIG && payload.length >= 2 + CFG_STRUCT_SIZE) {
            parseConfigStruct(payload.slice(2, 2 + CFG_STRUCT_SIZE));
            setCfgStatus('Config loaded from device', 'var(--accent-green)');
        } else if (respCode === 0 && cmdId === CMD_SET_CONFIG && pendingSaveAfterSet) {
            pendingSaveAfterSet = false;
            setCfgStatus('Config written, saving to EEPROM...', 'var(--accent-cyan)');
            sendConfigPacket(CMD_SAVE_CONFIG);
        } else if (respCode === 0) {
            setCfgStatus(`${cmdName}: ${respName}`, 'var(--accent-green)');
        } else {
            setCfgStatus(`${cmdName}: ${respName}`, 'var(--accent-red)');
            pendingSaveAfterSet = false;
        }
    }

    // Register response listener
    window.corvSerial.onConfigResponse(handleConfigResponse);

    // --- Action buttons ---
    btnRead.addEventListener('click', async () => {
        setCfgStatus('Reading config...', 'var(--accent-cyan)');
        await sendConfigPacket(CMD_GET_CONFIG);
    });

    btnSendSave.addEventListener('click', async () => {
        setCfgStatus('Writing config...', 'var(--accent-cyan)');
        pendingSaveAfterSet = true;
        const ok = await sendConfigPacket(CMD_SET_CONFIG, buildConfigStruct());
        if (!ok) pendingSaveAfterSet = false;
    });
}
