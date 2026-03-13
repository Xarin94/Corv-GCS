/**
 * MissionCommands.js - Mission Command Catalog
 * Single source of truth for all MAV_CMD definitions used in mission planning.
 * Based on ArduPilot MAVLink command set.
 */

export const COMMAND_CATEGORIES = {
    nav:       { label: 'Navigation', order: 0 },
    condition: { label: 'Condition',  order: 1 },
    do:        { label: 'DO Commands', order: 2 },
    camera:    { label: 'Camera / Gimbal', order: 3 }
};

/**
 * Master command catalog.
 * Each entry: { name, shortName, category, color, hasLocation, params }
 * params keys are 'param1'..'param4'; each has { label, unit, min, max, step, default }
 */
export const MISSION_COMMANDS = {
    // ── Navigation ──────────────────────────────────────────────
    16: {
        name: 'WAYPOINT', shortName: 'WP', category: 'nav', color: '#44ff44', hasLocation: true,
        params: {
            param1: { label: 'Delay',        unit: 's',   min: 0, max: 999, step: 1, default: 0 },
            param2: { label: 'Accept Radius', unit: 'm',   min: 0, max: 999, step: 1, default: 0 },
            param3: { label: 'Pass Radius',   unit: 'm',   min: 0, max: 999, step: 1, default: 0 },
            param4: { label: 'Yaw',           unit: 'deg', min: 0, max: 360, step: 1, default: 0 }
        }
    },
    17: {
        name: 'LOITER UNLIM', shortName: 'LOITER', category: 'nav', color: '#4488ff', hasLocation: true,
        params: {
            param3: { label: 'Radius', unit: 'm', min: -999, max: 999, step: 1, default: 0 }
        }
    },
    18: {
        name: 'LOITER TURNS', shortName: 'LOI_T', category: 'nav', color: '#4488ff', hasLocation: true,
        params: {
            param1: { label: 'Turns',  unit: '',  min: 1, max: 999, step: 1, default: 1 },
            param3: { label: 'Radius', unit: 'm', min: -999, max: 999, step: 1, default: 50 }
        }
    },
    19: {
        name: 'LOITER TIME', shortName: 'LOI_S', category: 'nav', color: '#4488ff', hasLocation: true,
        params: {
            param1: { label: 'Time',   unit: 's', min: 0, max: 9999, step: 1, default: 10 },
            param3: { label: 'Radius', unit: 'm', min: -999, max: 999, step: 1, default: 50 }
        }
    },
    20: {
        name: 'RTL', shortName: 'RTL', category: 'nav', color: '#ff8800', hasLocation: true,
        params: {}
    },
    21: {
        name: 'LAND', shortName: 'LAND', category: 'nav', color: '#ff3333', hasLocation: true,
        params: {
            param1: { label: 'Abort Alt', unit: 'm',   min: 0, max: 999, step: 1, default: 0 },
            param4: { label: 'Yaw',       unit: 'deg', min: 0, max: 360, step: 1, default: 0 }
        }
    },
    22: {
        name: 'TAKEOFF', shortName: 'TKOFF', category: 'nav', color: '#ffcc00', hasLocation: true,
        params: {
            param1: { label: 'Pitch', unit: 'deg', min: 0, max: 90, step: 1, default: 0 }
        }
    },
    30: {
        name: 'CONTINUE & CHANGE ALT', shortName: 'CHGALT', category: 'nav', color: '#44ff44', hasLocation: true,
        params: {
            param1: { label: 'Action', unit: '', min: 0, max: 2, step: 1, default: 0 }
        }
    },
    31: {
        name: 'LOITER TO ALT', shortName: 'LOI_ALT', category: 'nav', color: '#4488ff', hasLocation: true,
        params: {
            param1: { label: 'Heading Req', unit: '',  min: 0, max: 1, step: 1, default: 0 },
            param2: { label: 'Radius',      unit: 'm', min: -999, max: 999, step: 1, default: 50 },
            param4: { label: 'Xtrack Loc',  unit: '',  min: 0, max: 1, step: 1, default: 0 }
        }
    },
    82: {
        name: 'SPLINE WP', shortName: 'SPLINE', category: 'nav', color: '#cc44ff', hasLocation: true,
        params: {
            param1: { label: 'Delay', unit: 's', min: 0, max: 999, step: 1, default: 0 }
        }
    },
    84: {
        name: 'VTOL TAKEOFF', shortName: 'VTOL_TK', category: 'nav', color: '#ffcc00', hasLocation: true,
        params: {
            param2: { label: 'Transition Hdg', unit: '', min: 0, max: 3, step: 1, default: 0 }
        }
    },
    85: {
        name: 'VTOL LAND', shortName: 'VTOL_LND', category: 'nav', color: '#ff3333', hasLocation: true,
        params: {
            param1: { label: 'Land Options', unit: '', min: 0, max: 1, step: 1, default: 0 },
            param4: { label: 'Approach Alt', unit: 'm', min: 0, max: 999, step: 1, default: 0 }
        }
    },
    92: {
        name: 'GUIDED ENABLE', shortName: 'GUIDED', category: 'nav', color: '#44ff44', hasLocation: false,
        params: {
            param1: { label: 'Enable', unit: '', min: 0, max: 1, step: 1, default: 1 }
        }
    },
    94: {
        name: 'PAYLOAD PLACE', shortName: 'PAYLOAD', category: 'nav', color: '#ff8800', hasLocation: true,
        params: {
            param1: { label: 'Max Descent', unit: 'm', min: 0, max: 999, step: 1, default: 0 }
        }
    },

    // ── Conditions ──────────────────────────────────────────────
    93: {
        name: 'DELAY', shortName: 'DELAY', category: 'condition', color: '#aaaaaa', hasLocation: false,
        params: {
            param1: { label: 'Delay', unit: 's', min: 0, max: 9999, step: 1, default: 5 }
        }
    },
    112: {
        name: 'CONDITION DELAY', shortName: 'C_DELAY', category: 'condition', color: '#aaaaaa', hasLocation: false,
        params: {
            param1: { label: 'Delay', unit: 's', min: 0, max: 9999, step: 1, default: 5 }
        }
    },
    113: {
        name: 'CONDITION CHANGE ALT', shortName: 'C_ALT', category: 'condition', color: '#aaaaaa', hasLocation: false,
        params: {
            param1: { label: 'Rate',   unit: 'm/s', min: -10, max: 10, step: 0.1, default: 1 },
            param2: { label: 'Target', unit: 'm',   min: 0, max: 9999, step: 1, default: 100 }
        }
    },
    114: {
        name: 'CONDITION DISTANCE', shortName: 'C_DIST', category: 'condition', color: '#aaaaaa', hasLocation: false,
        params: {
            param1: { label: 'Distance', unit: 'm', min: 0, max: 99999, step: 1, default: 50 }
        }
    },
    115: {
        name: 'CONDITION YAW', shortName: 'C_YAW', category: 'condition', color: '#aaaaaa', hasLocation: false,
        params: {
            param1: { label: 'Angle',     unit: 'deg',   min: 0, max: 360, step: 1, default: 0 },
            param2: { label: 'Speed',     unit: 'deg/s', min: 0, max: 360, step: 1, default: 0 },
            param3: { label: 'Direction', unit: '',      min: -1, max: 1, step: 1, default: 0 },
            param4: { label: 'Relative',  unit: '',      min: 0, max: 1, step: 1, default: 0 }
        }
    },

    // ── DO Commands ─────────────────────────────────────────────
    176: {
        name: 'DO SET MODE', shortName: 'MODE', category: 'do', color: '#66ccff', hasLocation: false,
        params: {
            param1: { label: 'Mode #', unit: '', min: 0, max: 30, step: 1, default: 0 }
        }
    },
    177: {
        name: 'DO JUMP', shortName: 'JUMP', category: 'do', color: '#66ccff', hasLocation: false,
        params: {
            param1: { label: 'WP #',   unit: '', min: 0, max: 999, step: 1, default: 0 },
            param2: { label: 'Repeat', unit: '', min: -1, max: 999, step: 1, default: 1 }
        }
    },
    178: {
        name: 'DO CHANGE SPEED', shortName: 'SPEED', category: 'do', color: '#66ccff', hasLocation: false,
        params: {
            param1: { label: 'Type',     unit: '',    min: 0, max: 3, step: 1, default: 1 },
            param2: { label: 'Speed',    unit: 'm/s', min: 0, max: 100, step: 0.5, default: 5 },
            param3: { label: 'Throttle', unit: '%',   min: -1, max: 100, step: 1, default: -1 }
        }
    },
    179: {
        name: 'DO SET HOME', shortName: 'HOME', category: 'do', color: '#66ccff', hasLocation: true,
        params: {
            param1: { label: 'Use Current', unit: '', min: 0, max: 1, step: 1, default: 0 }
        }
    },
    181: {
        name: 'DO SET RELAY', shortName: 'RELAY', category: 'do', color: '#66ccff', hasLocation: false,
        params: {
            param1: { label: 'Relay #', unit: '', min: 0, max: 15, step: 1, default: 0 },
            param2: { label: 'On/Off',  unit: '', min: 0, max: 1, step: 1, default: 1 }
        }
    },
    182: {
        name: 'DO REPEAT RELAY', shortName: 'RPT_RLY', category: 'do', color: '#66ccff', hasLocation: false,
        params: {
            param1: { label: 'Relay #', unit: '',  min: 0, max: 15, step: 1, default: 0 },
            param2: { label: 'Count',   unit: '',  min: 1, max: 999, step: 1, default: 1 },
            param3: { label: 'Period',  unit: 's', min: 0, max: 999, step: 0.5, default: 1 }
        }
    },
    183: {
        name: 'DO SET SERVO', shortName: 'SERVO', category: 'do', color: '#66ccff', hasLocation: false,
        params: {
            param1: { label: 'Servo #', unit: '',   min: 1, max: 16, step: 1, default: 1 },
            param2: { label: 'PWM',     unit: 'us', min: 500, max: 2500, step: 1, default: 1500 }
        }
    },
    184: {
        name: 'DO REPEAT SERVO', shortName: 'RPT_SRV', category: 'do', color: '#66ccff', hasLocation: false,
        params: {
            param1: { label: 'Servo #', unit: '',   min: 1, max: 16, step: 1, default: 1 },
            param2: { label: 'PWM',     unit: 'us', min: 500, max: 2500, step: 1, default: 1500 },
            param3: { label: 'Count',   unit: '',   min: 1, max: 999, step: 1, default: 1 },
            param4: { label: 'Period',  unit: 's',  min: 0, max: 999, step: 0.5, default: 1 }
        }
    },
    189: {
        name: 'DO LAND START', shortName: 'LND_ST', category: 'do', color: '#ff3333', hasLocation: true,
        params: {}
    },
    193: {
        name: 'DO PAUSE/CONTINUE', shortName: 'PAUSE', category: 'do', color: '#66ccff', hasLocation: false,
        params: {
            param1: { label: 'Pause(0)/Continue(1)', unit: '', min: 0, max: 1, step: 1, default: 0 }
        }
    },
    194: {
        name: 'DO SET REVERSE', shortName: 'REVERSE', category: 'do', color: '#66ccff', hasLocation: false,
        params: {
            param1: { label: 'Reverse', unit: '', min: 0, max: 1, step: 1, default: 0 }
        }
    },
    195: {
        name: 'DO SET ROI LOCATION', shortName: 'ROI_LOC', category: 'do', color: '#ff66aa', hasLocation: true,
        params: {}
    },
    197: {
        name: 'DO SET ROI NONE', shortName: 'ROI_OFF', category: 'do', color: '#ff66aa', hasLocation: false,
        params: {}
    },
    201: {
        name: 'DO SET ROI', shortName: 'ROI', category: 'do', color: '#ff66aa', hasLocation: true,
        params: {}
    },
    205: {
        name: 'DO MOUNT CONTROL', shortName: 'MOUNT', category: 'do', color: '#ff66aa', hasLocation: false,
        params: {
            param1: { label: 'Pitch', unit: 'deg', min: -90, max: 90, step: 1, default: 0 },
            param2: { label: 'Roll',  unit: 'deg', min: -45, max: 45, step: 1, default: 0 },
            param3: { label: 'Yaw',   unit: 'deg', min: -180, max: 180, step: 1, default: 0 }
        }
    },
    206: {
        name: 'DO CAM TRIGG DIST', shortName: 'TRIGG_D', category: 'do', color: '#ffaa44', hasLocation: false,
        params: {
            param1: { label: 'Distance', unit: 'm', min: 0, max: 9999, step: 1, default: 25 },
            param2: { label: 'Shutter',  unit: '',  min: 0, max: 1, step: 1, default: 0 },
            param3: { label: 'Trigger',  unit: '',  min: 0, max: 1, step: 1, default: 1 }
        }
    },
    207: {
        name: 'DO FENCE ENABLE', shortName: 'FENCE', category: 'do', color: '#66ccff', hasLocation: false,
        params: {
            param1: { label: 'Enable', unit: '', min: 0, max: 2, step: 1, default: 1 }
        }
    },
    208: {
        name: 'DO PARACHUTE', shortName: 'CHUTE', category: 'do', color: '#ff3333', hasLocation: false,
        params: {
            param1: { label: 'Action', unit: '', min: 0, max: 2, step: 1, default: 2 }
        }
    },
    211: {
        name: 'DO GRIPPER', shortName: 'GRIP', category: 'do', color: '#66ccff', hasLocation: false,
        params: {
            param1: { label: 'Gripper #', unit: '', min: 1, max: 4, step: 1, default: 1 },
            param2: { label: 'Action',    unit: '', min: 0, max: 1, step: 1, default: 0 }
        }
    },
    212: {
        name: 'DO AUTOTUNE ENABLE', shortName: 'ATUNE', category: 'do', color: '#66ccff', hasLocation: false,
        params: {
            param1: { label: 'Enable', unit: '', min: 0, max: 1, step: 1, default: 1 }
        }
    },

    // ── Camera / Gimbal ─────────────────────────────────────────
    203: {
        name: 'DO DIGICAM CONTROL', shortName: 'SHUTTER', category: 'camera', color: '#ffaa44', hasLocation: false,
        params: {
            param1: { label: 'Session',    unit: '', min: 0, max: 1, step: 1, default: 0 },
            param2: { label: 'Zoom Abs',   unit: '', min: 0, max: 100, step: 1, default: 0 },
            param4: { label: 'Focus Lock', unit: '', min: 0, max: 1, step: 1, default: 0 }
        }
    },
    530: {
        name: 'SET CAMERA MODE', shortName: 'CAM_M', category: 'camera', color: '#ffaa44', hasLocation: false,
        params: {
            param2: { label: 'Mode', unit: '', min: 0, max: 7, step: 1, default: 0 }
        }
    },
    2000: {
        name: 'IMAGE START CAPTURE', shortName: 'IMG_ON', category: 'camera', color: '#ffaa44', hasLocation: false,
        params: {
            param2: { label: 'Interval', unit: 's', min: 0, max: 999, step: 0.5, default: 0 },
            param3: { label: 'Count',    unit: '',  min: 0, max: 9999, step: 1, default: 1 }
        }
    },
    2001: {
        name: 'IMAGE STOP CAPTURE', shortName: 'IMG_OFF', category: 'camera', color: '#ffaa44', hasLocation: false,
        params: {}
    },
    2500: {
        name: 'VIDEO START CAPTURE', shortName: 'VID_ON', category: 'camera', color: '#ffaa44', hasLocation: false,
        params: {
            param2: { label: 'Status Freq', unit: 'Hz', min: 0, max: 10, step: 0.1, default: 0 }
        }
    },
    2501: {
        name: 'VIDEO STOP CAPTURE', shortName: 'VID_OFF', category: 'camera', color: '#ffaa44', hasLocation: false,
        params: {}
    }
};

// ── Helper functions ────────────────────────────────────────────

export function getCmdDef(id) {
    return MISSION_COMMANDS[id] || null;
}

export function getCmdName(id) {
    return MISSION_COMMANDS[id]?.name || `CMD ${id}`;
}

export function getCmdShortName(id) {
    return MISSION_COMMANDS[id]?.shortName || `C${id}`;
}

export function getCmdColor(id) {
    return MISSION_COMMANDS[id]?.color || '#44ff44';
}

export function getCmdParams(id) {
    return MISSION_COMMANDS[id]?.params || null;
}

export function isNavCmd(id) {
    const cmd = MISSION_COMMANDS[id];
    return cmd ? cmd.hasLocation : false;
}

/**
 * Return commands grouped by category, sorted by category order.
 * Each entry: { id, name, shortName, category }
 */
export function getGroupedCommands() {
    const groups = {};
    for (const [catKey, catInfo] of Object.entries(COMMAND_CATEGORIES)) {
        groups[catKey] = { label: catInfo.label, order: catInfo.order, commands: [] };
    }
    for (const [id, cmd] of Object.entries(MISSION_COMMANDS)) {
        const cat = cmd.category;
        if (groups[cat]) {
            groups[cat].commands.push({ id: parseInt(id), name: cmd.name, shortName: cmd.shortName, category: cat });
        }
    }
    return Object.values(groups).sort((a, b) => a.order - b.order);
}

/**
 * Get default param values for a command.
 */
export function getCmdDefaults(id) {
    const params = MISSION_COMMANDS[id]?.params;
    if (!params) return { param1: 0, param2: 0, param3: 0, param4: 0 };
    return {
        param1: params.param1?.default || 0,
        param2: params.param2?.default || 0,
        param3: params.param3?.default || 0,
        param4: params.param4?.default || 0
    };
}
