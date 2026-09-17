/**
 * MissionTransfer.js - Mission download from the vehicle, and the reverse
 * mapping from a flat MAVLink item list back into route segments.
 *
 * The compiler is lossy in one direction only: a lawn-mower grid read back from
 * the autopilot comes back as plain waypoints, never as the area it was
 * generated from — the shape cannot be recovered from the flat list. That is
 * the trade-off that lets an old mission file, or a plan written by Mission
 * Planner, open here as an editable route.
 */

import { STATE } from '../core/state.js';
import { onMessage, offMessage } from '../mavlink/MAVLinkManager.js';
import { createRoute, createSegment, createAction, defaultRouteParams } from './RouteModel.js';

// ── Items → route ─────────────────────────────────────────────────────────────

/**
 * @param {Array} items  MAVLink-shaped items ({command, lat, lng, alt, frame, param1..4, isHome?})
 * @param {object} opts  { name, fromVehicle } — fromVehicle keeps the raw frames instead of the
 *                       app's "alt is AGL" convention used by saved files.
 */
export function itemsToRoute(items, opts = {}) {
    const route = createRoute();
    route.name = opts.name || null;
    const P = route.params = defaultRouteParams();
    if (!items?.length) return route;

    // float32 on the wire → 671.7000122; one decimetre is all a plan needs
    const r1 = v => Number.isFinite(v) ? Math.round(v * 10) / 10 : v;
    let list = items.map(it => ({ ...it, alt: r1(it.alt), param1: r1(it.param1), param2: r1(it.param2), param3: r1(it.param3), param4: r1(it.param4) }));
    let homeZ = null;
    // seq 0 is the home position on ArduPilot; saved files flag it explicitly
    if (list[0]?.isHome || (opts.fromVehicle && list.length && list[0].command === 16)) {
        const h = list[0];
        if (Number.isFinite(h.lat) && Number.isFinite(h.lng) && (h.lat || h.lng)) P.home = { lat: h.lat, lng: h.lng };
        homeZ = Number.isFinite(h.alt) ? h.alt : null;
        list = list.slice(1);
    }

    if (opts.fromVehicle) {
        // Frame of the flying waypoints decides the altitude mode
        const frames = list.filter(it => it.command === 16 || it.command === 82).map(it => it.frame);
        const mode = frames.filter(f => f === 3).length >= frames.length / 2 ? 'rel'
            : frames.some(f => f === 10) ? 'agl' : 'amsl';
        P.altMode = mode;
    } else {
        P.altMode = 'agl';   // saved files carry AGL in `alt`
    }

    P.takeoff = false;
    P.endAction = 'none';
    let defaultSpeedSet = false;
    let last = null;                 // last segment that can carry actions
    const segs = route.segments;

    const attach = (action) => {
        if (last) last.actions.push(action);
        else {
            // Actions before the first navigation point: keep them on a zero-length
            // waypoint so nothing read from the vehicle is silently dropped.
            const anchor = createSegment('waypoint', [P.home || { lat: items[0].lat, lng: items[0].lng }]);
            segs.push(anchor); last = anchor; anchor.actions.push(action);
        }
    };

    for (let i = 0; i < list.length; i++) {
        const it = list[i];
        const isLast = i === list.length - 1;
        const alt = Number.isFinite(it.alt) ? it.alt : null;
        switch (it.command) {
            case 16: case 82: {
                const seg = createSegment('waypoint', [it], { alt, turn: it.command === 82 ? 'spline' : 'route', delay: it.param1 || 0, acceptRadius: it.param2 || 0 });
                segs.push(seg); last = seg;
                break;
            }
            case 17: case 18: case 19: case 31: {
                const r = Math.abs(it.param3 || it.param2 || 50);
                const dir = (it.param3 || it.param2 || 0) < 0 ? 'ccw' : 'cw';
                const seg = createSegment('circle', [it], { alt, radius: r, turns: it.command === 18 ? Math.max(1, it.param1 || 1) : 1, direction: dir });
                segs.push(seg); last = seg;
                break;
            }
            case 22: case 84: {
                P.takeoff = true;
                // Take-off height is always above the take-off point; an absolute frame carries home MSL in it
                const rel = (it.frame === 0 && homeZ !== null) ? alt - homeZ : alt;
                if (rel > 0) P.takeoffAlt = r1(rel);
                break;
            }
            case 20:
                if (isLast) P.endAction = 'rtl';
                else attach(createAction('raw', { command: 20 }));
                break;
            case 21: case 85: {
                if (isLast && last && Math.abs(last.points[0].lat - it.lat) < 1e-6 && Math.abs(last.points[0].lng - it.lng) < 1e-6 && last.type !== 'landing') {
                    P.endAction = 'land';
                } else {
                    const seg = createSegment('landing', [it], { vtol: it.command === 85, abortAlt: it.param1 || 0 });
                    segs.push(seg); last = seg;
                }
                break;
            }
            case 201: case 195: {
                const seg = createSegment('poi', [it], { mode: 'set', alt: alt || 0 });
                segs.push(seg);
                break;
            }
            case 197: {
                const seg = createSegment('poi', [it.lat || it.lng ? it : (last?.points[0] || P.home || { lat: 0, lng: 0 })], { mode: 'clear' });
                segs.push(seg);
                break;
            }
            case 178:
                if (!defaultSpeedSet && !last) { P.defaultSpeed = it.param2 || P.defaultSpeed; defaultSpeedSet = true; }
                else if (last && !last.params.speed) last.params.speed = it.param2 || 0;
                else attach(createAction('speed', { speed: it.param2 || 0 }));
                break;
            case 206: attach(createAction('trigger_dist', { distance: it.param1 || 0 })); break;
            case 203: attach(createAction('shot')); break;
            case 205: attach(createAction('gimbal', { pitch: it.param1 || 0, yaw: it.param3 || 0 })); break;
            case 115: attach(createAction('yaw', { heading: it.param1 || 0 })); break;
            case 93: case 112: attach(createAction('wait', { seconds: it.param1 || 0 })); break;
            case 183: attach(createAction('servo', { channel: it.param1 || 0, pwm: it.param2 || 0 })); break;
            case 181: attach(createAction('relay', { relay: it.param1 || 0, state: it.param2 ? 1 : 0 })); break;
            default:
                attach(createAction('raw', { command: it.command, param1: it.param1 || 0, param2: it.param2 || 0, param3: it.param3 || 0, param4: it.param4 || 0 }));
        }
    }

    // A speed change that only ever sits on the first segment is really the route speed
    if (!defaultSpeedSet && segs[0]?.params.speed > 0 && segs.every((s, i) => i === 0 || !s.params.speed)) {
        P.defaultSpeed = segs[0].params.speed;
        segs[0].params.speed = 0;
    }
    return route;
}

// ── Vehicle → GCS ─────────────────────────────────────────────────────────────

async function sendMessage(msg) {
    if (!window.mavlink) throw new Error('MAVLink not available');
    return window.mavlink.sendMessage(msg);
}

/**
 * Read the mission stored on the autopilot.
 * @param {(done:number, total:number)=>void} onProgress
 * @returns {Promise<Array>} items in the compiler's shape (lat/lng in degrees)
 */
export function downloadMission(onProgress = () => {}) {
    return new Promise((resolve, reject) => {
        const items = [];
        let total = -1;
        let timer = null;
        const target = { targetSystem: STATE.systemId, targetComponent: STATE.componentId, missionType: 0 };

        const cleanup = () => {
            clearTimeout(timer);
            offMessage(44, onCount);
            offMessage(73, onItem);
            offMessage(39, onItem);
        };
        const fail = (msg) => { cleanup(); reject(new Error(msg)); };
        const arm = () => { clearTimeout(timer); timer = setTimeout(() => fail('Vehicle did not answer'), 4000); };

        const request = (seq) => {
            arm();
            sendMessage({ type: 'MISSION_REQUEST_INT', ...target, seq }).catch(e => fail(e.message));
        };

        const onCount = (data) => {
            if (total >= 0) return;
            total = data.count || 0;
            onProgress(0, total);
            if (total === 0) { finish(); return; }
            request(0);
        };

        const onItem = (data) => {
            const seq = data.seq;
            if (seq !== items.length) return;   // duplicate or out of order — ignore, the timer re-asks
            const isInt = data.x !== undefined && Math.abs(data.x) > 1000;
            items.push({
                seq,
                command: data.command,
                frame: data.frame,
                lat: isInt ? data.x / 1e7 : data.x,
                lng: isInt ? data.y / 1e7 : data.y,
                alt: data.z,
                param1: data.param1, param2: data.param2, param3: data.param3, param4: data.param4,
            });
            onProgress(items.length, total);
            if (items.length >= total) finish();
            else request(items.length);
        };

        const finish = () => {
            cleanup();
            sendMessage({ type: 'MISSION_ACK', ...target, ackType: 0 }).catch(() => {});
            resolve(items);
        };

        onMessage(44, onCount);   // MISSION_COUNT
        onMessage(73, onItem);    // MISSION_ITEM_INT
        onMessage(39, onItem);    // MISSION_ITEM (legacy autopilots)
        arm();
        sendMessage({ type: 'MISSION_REQUEST_LIST', ...target }).catch(e => fail(e.message));
    });
}
