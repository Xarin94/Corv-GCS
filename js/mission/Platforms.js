/**
 * Platforms.js - Which flight stack the GCS is talking to
 *
 * The route editor was written against ArduPilot: every segment and every
 * action compiles to a MAV_CMD and the MAVLink mission protocol carries the
 * list. INAV is a different machine — a flat list of at most a few dozen
 * waypoints with seven possible actions, no DO_ commands, no take-off item, no
 * splines, travelling over MSP rather than MAVLink. Betaflight is a third case
 * again: no navigation stack at all, so no mission of any kind.
 *
 * Rather than scatter `if (inav)` through the compiler and the UI, the
 * differences live here as one table: what a platform supports, which link it
 * wants, and — for everything it cannot do — why. The UI reads that "why"
 * straight into the tooltip of every control it greys out, so the operator is
 * never left guessing whether a button is broken or simply meaningless on this
 * airframe.
 *
 * The active platform is a GCS-wide setting (SYS CONFIG → FLIGHT STACK), kept
 * in localStorage and announced as a `platformChanged` window event. It is not
 * a property of the route: a mission file opens the same way whatever stack is
 * selected, and only what can be *flown* depends on this.
 *
 * Keys used by `unsupportedReason`:
 *   segment    <type>                                    — a tool in the palette
 *   action     <type>                                    — an entry of the ACTIONS menu
 *   routeParam <key> | <key>.<option>                    — a route settings field
 *   segField   <segType>.<key> | <segType>.<key>.<option> — an inspector field
 */

export const PLATFORM_ORDER = ['ardupilot', 'inav', 'betaflight'];

export const PLATFORMS = {
    ardupilot: {
        id: 'ardupilot',
        label: 'ArduPilot',
        short: 'AP',
        blurb: 'Copter · Plane · Rover — MAVLink',
        transport: 'mavlink',
        missions: true,
        // Copter/Plane store 700 items on most boards — conservative.
        itemLimit: 700,
        // What the CONNECTION page is pre-set to when this stack is picked.
        connection: { type: 'mavlink-serial', baud: 57600 },
        hint: 'Full MAVLink mission: every segment type, every action, terrain-following waypoints.',
        unsupported: {},
    },

    inav: {
        id: 'inav',
        label: 'INAV',
        short: 'INAV',
        blurb: 'Waypoint missions over MSP',
        transport: 'msp',
        missions: true,
        // NAV_MAX_WAYPOINTS: 60 on most targets. The board reports its own
        // figure in MSP_WP_GETINFO and that one wins once the link is up.
        itemLimit: 60,
        connection: { type: 'msp-serial', baud: 115200 },
        hint: 'INAV waypoint mission over MSP: navigation, hold, POI, heading, speed, land and RTH. '
            + 'Upload with the aircraft disarmed, then fly it in WP mode.',
        unsupported: {
            segment: {
                circle: 'An INAV waypoint has no loiter radius, direction or turn count — a circle cannot be expressed',
            },
            action: {
                trigger_dist: 'INAV missions carry no camera commands — set the trigger up on the camera itself',
                shot:         'INAV missions carry no camera commands — set the trigger up on the camera itself',
                gimbal:       'INAV missions carry no gimbal commands',
                servo:        'INAV missions carry no servo commands (programming framework only)',
                relay:        'INAV missions carry no relay commands (programming framework only)',
                raw:          'A raw MAVLink command has no INAV waypoint equivalent',
            },
            routeParam: {
                takeoff:    'INAV has no take-off waypoint — arm, take off in a flight mode, then switch to WP',
                takeoffAlt: 'INAV has no take-off waypoint — arm, take off in a flight mode, then switch to WP',
                turnType:   'INAV flies straight legs — there is no spline waypoint',
            },
            segField: {
                'waypoint.turn':         'INAV flies straight legs — there is no spline waypoint',
                'waypoint.acceptRadius': 'INAV uses the nav_wp_radius parameter for every waypoint',
                'landing.vtol':          'INAV has no VTOL landing waypoint',
                'landing.abortAlt':      'INAV has no landing abort altitude in the mission',
                'area.trigger':          'INAV missions carry no camera trigger — the lanes upload as plain waypoints',
                'corridor.trigger':      'INAV missions carry no camera trigger — the passes upload as plain waypoints',
                'poi.mode.clear':        'INAV cannot cancel a POI from the mission — it holds until the mission ends',
            },
        },
    },

    betaflight: {
        id: 'betaflight',
        label: 'Betaflight',
        short: 'BF',
        blurb: 'Acro / racing — no navigation',
        transport: 'msp',
        missions: false,
        itemLimit: 0,
        connection: { type: 'msp-serial', baud: 115200 },
        hint: 'Betaflight has no navigation stack: telemetry and configuration only, no waypoint missions.',
        noMissionReason: 'Betaflight has no navigation stack — it cannot fly a waypoint mission',
        unsupported: {},
    },
};

// ── The active platform ───────────────────────────────────────────────────────

const LS_KEY = 'corv.platform';
const DEFAULT_PLATFORM = 'ardupilot';

let active = DEFAULT_PLATFORM;
try {
    const saved = localStorage.getItem(LS_KEY);
    if (saved && PLATFORMS[saved]) active = saved;
} catch (e) { /* private mode, first run — the default stands */ }

/** The id of the stack the GCS is set up for. */
export function activePlatformId() {
    return active;
}

/** The descriptor of the active stack (or of `id`, when given). */
export function getPlatform(id = active) {
    return PLATFORMS[id] || PLATFORMS[DEFAULT_PLATFORM];
}

/**
 * Switch stack. Everything that depends on it listens for `platformChanged`
 * rather than being called from here, so the setting has exactly one owner.
 * @returns {boolean} true when the value actually changed
 */
export function setActivePlatform(id) {
    if (!PLATFORMS[id] || id === active) return false;
    active = id;
    try { localStorage.setItem(LS_KEY, id); } catch (e) { /* not worth failing over */ }
    window.dispatchEvent(new CustomEvent('platformChanged', { detail: { platform: id } }));
    return true;
}

/** Can this stack fly a mission at all? (Betaflight cannot.) */
export function missionsSupported(id = active) {
    return !!getPlatform(id).missions;
}

/** Does the mission travel over MSP instead of the MAVLink mission protocol? */
export function usesMspMissions(id = active) {
    return getPlatform(id).transport === 'msp' && missionsSupported(id);
}

// ── Capability lookups ────────────────────────────────────────────────────────

/**
 * Why `key` cannot be used on this platform, or null when it can.
 * @param {'segment'|'action'|'routeParam'|'segField'} kind
 * @param {string} key   see the header for the key shapes
 * @param {string} [id]  platform id, default the active one
 */
export function unsupportedReason(kind, key, id = active) {
    const p = getPlatform(id);
    if (!p.missions) return p.noMissionReason || `${p.label} cannot fly missions`;
    return p.unsupported?.[kind]?.[key] || null;
}

export function segmentAllowed(type, id = active) {
    return !unsupportedReason('segment', type, id);
}

export function actionAllowed(type, id = active) {
    return !unsupportedReason('action', type, id);
}
