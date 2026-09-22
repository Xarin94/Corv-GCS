/**
 * InavMission.js - Compiled MAVLink items → INAV waypoint mission (and back)
 *
 * INAV stores a mission as a flat list of fixed 21-byte records, numbered from
 * 1, the last one flagged 0xA5:
 *
 *     u8 index · u8 action · i32 lat(1e7) · i32 lon(1e7) · i32 alt(cm)
 *     i16 p1 · i16 p2 · i16 p3 · u8 flag
 *
 * Only seven actions exist, and none of them is a DO_ command: what the route
 * cannot express as navigation, hold, POI, heading, land, RTH or jump simply
 * does not travel. The translation is therefore deliberately lossy, and every
 * drop is reported as an issue on the route card rather than silently
 * swallowed — the operator has to see that the camera trigger they placed will
 * not fly.
 *
 * Working from the *compiled* items rather than from the route means the whole
 * geometry pipeline (survey lanes, corridor passes, terrain resolution, AGL
 * altitudes) is reused as-is; this module only re-encodes the result.
 *
 * Altitudes: an INAV waypoint is relative to the arming point unless bit 0 of
 * p3 says AMSL (INAV 5+). Relative is the default here because every firmware
 * understands it and the compiler already knows the take-off elevation.
 */

// NAV_MAX_WAYPOINTS on most targets. The board reports its own limit in
// MSP_WP_GETINFO, and that one wins whenever the link is up.
export const INAV_MAX_WAYPOINTS = 60;

export const NAV_WP_ACTION = {
    WAYPOINT:  1,
    HOLD_TIME: 3,
    RTH:       4,
    SET_POI:   5,
    JUMP:      6,
    SET_HEAD:  7,
    LAND:      8,
};

export const WP_FLAG_LAST = 0xA5;
export const P3_ALT_AMSL = 0x01;

const ACTION_NAME = {
    1: 'WAYPOINT', 3: 'HOLD', 4: 'RTH', 5: 'POI', 6: 'JUMP', 7: 'HEADING', 8: 'LAND',
};

export function inavActionName(a) {
    return ACTION_NAME[a] || `ACTION ${a}`;
}

/** Navigation actions: the mission has to end on one of these. */
const NAVIGATIONAL = new Set([NAV_WP_ACTION.WAYPOINT, NAV_WP_ACTION.HOLD_TIME, NAV_WP_ACTION.RTH, NAV_WP_ACTION.LAND]);

/**
 * Translate compiled mission items into an INAV waypoint list.
 *
 * @param {Array}  items  compiler output (altMsl / altRel already resolved)
 * @param {object} opts   { amsl:boolean, limit:number }
 * @returns {{ wps:Array, issues:Array }}
 */
export function toInavMission(items, opts = {}) {
    const amsl = !!opts.amsl;
    const limit = +opts.limit > 0 ? +opts.limit : INAV_MAX_WAYPOINTS;
    const wps = [];
    const issues = [];
    const add = (level, text, segId = null) => issues.push({ level, text, segId });

    const altCm = (it) => {
        const m = amsl ? it.altMsl : it.altRel;
        return Math.round((Number.isFinite(m) ? m : (+it.alt || 0)) * 100);
    };
    const p3Alt = () => (amsl ? P3_ALT_AMSL : 0);
    const push = (action, it, p1 = 0, p2 = 0, p3 = 0) => {
        wps.push({
            action,
            lat: Math.round((it?.lat || 0) * 1e7),
            lon: Math.round((it?.lng || 0) * 1e7),
            alt: it ? altCm(it) : 0,
            p1: clampI16(p1), p2: clampI16(p2), p3: clampI16(p3),
            flag: 0,
            segId: it?.segId ?? null,
        });
    };

    let speedCms = 0;       // DO_CHANGE_SPEED travels as p1 of the waypoints that follow it
    let holdS = 0;          // a delay with no waypoint before it waits at the next one

    // The compiler emits an action AFTER the point it belongs to, because that is
    // when ArduPilot runs a DO_ item. A wait therefore means "hold at the
    // waypoint just reached", which in INAV is that record turning into
    // HOLD_TIME — not the one that follows.
    const holdAt = (seconds) => {
        const prev = wps[wps.length - 1];
        if (prev && prev.action === NAV_WP_ACTION.WAYPOINT) {
            prev.action = NAV_WP_ACTION.HOLD_TIME;
            prev.p2 = prev.p1;                  // the speed moves to p2 on a hold
            prev.p1 = clampI16(Math.round(seconds));
            return;
        }
        if (prev && prev.action === NAV_WP_ACTION.HOLD_TIME) {
            prev.p1 = clampI16(prev.p1 + Math.round(seconds));
            return;
        }
        holdS = Math.max(holdS, seconds);       // nothing to hold at yet — wait at the next one
    };
    let splineSeen = false;
    let takeoffSeen = false;
    const loiterSegs = new Set();
    const droppedCmds = new Map();

    for (const it of items || []) {
        if (it.isHome) continue;                       // the INAV list has no home entry
        switch (it.command) {
            case 16:                                    // WAYPOINT
            case 82: {                                  // SPLINE_WAYPOINT — flown straight
                if (it.command === 82) splineSeen = true;
                const hold = Math.round(Math.max(holdS, +it.param1 || 0));
                holdS = 0;
                if (hold > 0) push(NAV_WP_ACTION.HOLD_TIME, it, hold, speedCms, p3Alt());
                else push(NAV_WP_ACTION.WAYPOINT, it, speedCms, 0, p3Alt());
                break;
            }
            case 22: case 84:                           // TAKEOFF / VTOL_TAKEOFF
                takeoffSeen = true;
                break;
            case 17: case 18: case 19: case 31:         // LOITER family
                loiterSegs.add(it.segId);
                break;
            case 20:
                // RTH with p1 = 0: what happens over home stays the board
                // nav_rth_allow_landing decision, as for any other RTH.
                push(NAV_WP_ACTION.RTH, null, 0, 0, 0);
                break;
            case 21: case 85:                           // LAND / VTOL_LAND
                push(NAV_WP_ACTION.LAND, it, 0, 0, p3Alt());
                break;
            case 201: case 195:                         // DO_SET_ROI (location)
                push(NAV_WP_ACTION.SET_POI, it, 0, 0, 0);
                break;
            case 197:                                   // DO_SET_ROI_NONE — no equivalent
                break;
            case 115: {                                 // CONDITION_YAW
                const hdg = Math.round(((+it.param1 || 0) % 360 + 360) % 360);
                push(NAV_WP_ACTION.SET_HEAD, null, hdg, 0, 0);
                break;
            }
            case 178:                                   // DO_CHANGE_SPEED
                speedCms = Math.max(0, Math.round((+it.param2 || 0) * 100));
                break;
            case 93: case 112:                          // DELAY / CONDITION_DELAY
                holdAt(Math.max(0, +it.param1 || 0));
                break;
            case 177:                                   // DO_JUMP
                push(NAV_WP_ACTION.JUMP, null, Math.round(+it.param1 || 0), Math.round(+it.param2 || 0), 0);
                break;
            default:
                droppedCmds.set(it.command, (droppedCmds.get(it.command) || 0) + 1);
        }
    }

    // Numbering and the end-of-list flag
    wps.forEach((w, i) => { w.index = i + 1; w.flag = 0; });
    if (wps.length) wps[wps.length - 1].flag = WP_FLAG_LAST;

    // ── What the translation had to give up ───────────────────────────────────
    if (takeoffSeen) add('warn', 'INAV has no take-off waypoint — automatic take-off is not uploaded');
    if (splineSeen) add('warn', 'INAV flies straight legs — spline waypoints upload as ordinary waypoints');
    for (const segId of loiterSegs) add('warn', 'A circle cannot be uploaded to INAV — the segment is skipped', segId);
    for (const [cmd, n] of droppedCmds) add('warn', `${n} × command ${cmd} has no INAV equivalent — not uploaded`);

    if (!wps.length) {
        add('error', 'Nothing in this route can be uploaded to INAV');
    } else {
        if (wps.length > limit) add('error', `${wps.length} waypoints — INAV stores at most ${limit}`);
        if (!NAVIGATIONAL.has(wps[wps.length - 1].action)) {
            add('warn', `The mission ends on ${inavActionName(wps[wps.length - 1].action)} — INAV expects a navigation waypoint last`);
        }
    }

    return { wps, issues };
}

/**
 * The reverse: a list read back from the board, in the MAVLink-ish shape
 * MissionTransfer.itemsToRoute() consumes. A synthetic home item leads, because
 * that function reads item 0 as the home position.
 */
export function inavToItems(wps) {
    const items = [{ command: 16, isHome: true, frame: 0, lat: 0, lng: 0, alt: 0, param1: 0, param2: 0, param3: 0, param4: 0 }];
    const blank = { param1: 0, param2: 0, param3: 0, param4: 0 };
    let lastSpeed = 0;

    const speedItem = (cms) => {
        const v = Math.round((cms / 100) * 10) / 10;
        if (v <= 0 || v === lastSpeed) return;
        lastSpeed = v;
        items.push({ command: 178, frame: 3, lat: 0, lng: 0, alt: 0, ...blank, param1: 1, param2: v, param3: -1 });
    };

    for (const w of wps || []) {
        const frame = (w.p3 & P3_ALT_AMSL) ? 0 : 3;     // AMSL flag → absolute, else relative to home
        const loc = { frame, lat: (w.lat || 0) / 1e7, lng: (w.lon || 0) / 1e7, alt: (w.alt || 0) / 100 };
        switch (w.action) {
            case NAV_WP_ACTION.WAYPOINT:
                speedItem(w.p1);
                items.push({ command: 16, ...loc, ...blank });
                break;
            case NAV_WP_ACTION.HOLD_TIME:
                speedItem(w.p2);
                items.push({ command: 16, ...loc, ...blank, param1: w.p1 || 0 });
                break;
            case NAV_WP_ACTION.RTH:
                items.push({ command: 20, frame: 3, lat: 0, lng: 0, alt: 0, ...blank });
                break;
            case NAV_WP_ACTION.LAND:
                items.push({ command: 21, ...loc, ...blank });
                break;
            case NAV_WP_ACTION.SET_POI:
                items.push({ command: 201, ...loc, ...blank });
                break;
            case NAV_WP_ACTION.SET_HEAD:
                items.push({ command: 115, frame: 3, lat: 0, lng: 0, alt: 0, ...blank, param1: w.p1 || 0 });
                break;
            case NAV_WP_ACTION.JUMP:
                items.push({ command: 177, frame: 3, lat: 0, lng: 0, alt: 0, ...blank, param1: w.p1 || 0, param2: w.p2 || 0 });
                break;
            default:
                break;
        }
    }
    return items;
}

function clampI16(v) {
    const n = Math.round(+v || 0);
    return Math.max(-32768, Math.min(32767, n));
}
