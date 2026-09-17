/**
 * RouteCompiler.js - Route → MAVLink mission items
 *
 * This is the "calculate route" step: it expands every segment into navigation
 * points (lawn-mower lanes for an area scan, offset passes for a corridor, a
 * loiter for a circle…), interleaves the DO_/CONDITION_ items the actions ask
 * for, subdivides legs so the straight line between two waypoints never strays
 * more than the AGL tolerance from the terrain-following altitude, and finally
 * resolves every point's AGL / AMSL / relative altitude against the elevation
 * model. It also produces the statistics on the route card and the list of
 * warnings and errors that gate the upload.
 *
 * Pure function of (route, context) — no DOM, no STATE. The controller feeds it
 * a terrain lookup and the take-off point.
 *
 * Output items keep the legacy shape consumed by the 3D scene, the mini-map and
 * the library (`alt` = height above ground) and add `altMsl` / `altRel` plus a
 * `segId` back-reference so the map can highlight the selected segment.
 */

import {
    SEGMENT_TYPES, haversine, bearing, localFrame, centroid, polygonArea, segmentAlt, surveyGeometry,
} from './RouteModel.js';
import { groundFootprint, aimAt, photosAlong } from './CameraFootprint.js';

const D2R = Math.PI / 180;
const ARDUPILOT_ITEM_LIMIT = 700;   // conservative: Copter/Plane store 700 on most boards
const CLIMB_RATE = 2.5;             // m/s, for the duration estimate only
const DESCENT_RATE = 1.5;

// ── Public entry point ────────────────────────────────────────────────────────

/**
 * @param {object} route   the document (see RouteModel)
 * @param {object} ctx     { terrain(lat,lng)→m|null, home:{lat,lng}|null, homeAlt, vehicleType }
 */
export function compileRoute(route, ctx) {
    const P = route.params;
    const terrain = ctx.terrain || (() => null);
    const issues = [];
    const segStats = {};
    const addIssue = (level, text, segId = null) => issues.push({ level, text, segId });

    // ── Take-off point ────────────────────────────────────────────────────────
    const firstPt = route.segments.find(s => s.points.length)?.points[0] || null;
    const home = P.home || ctx.home || firstPt;
    if (!home || !route.segments.length) {
        if (!route.segments.length) addIssue('info', 'Pick a tool on the map and start drawing');
        const elev = home ? (terrain(home.lat, home.lng) ?? ctx.homeAlt ?? 0) : 0;
        return { items: [], stats: emptyStats(), issues, navPath: [], segStats, home: home ? { ...home, elev } : null };
    }
    const homeTerrain = terrain(home.lat, home.lng);
    const homeElev = homeTerrain !== null ? homeTerrain : (ctx.homeAlt || 0);
    if (homeTerrain === null) addIssue('warn', 'No elevation data at the take-off point — vehicle home altitude assumed');

    const isPlane = ctx.vehicleType === 1;

    // Intermediate representation: { lat,lng, alt(mode units)|null, command, p1..p4, segId, derived, loc, kind }
    const out = [];
    const push = (it) => { out.push(it); return it; };
    const doItem = (command, p, segId, extra = {}) => push({
        command, lat: 0, lng: 0, alt: 0, loc: false, segId, derived: false,
        param1: p[0] || 0, param2: p[1] || 0, param3: p[2] || 0, param4: p[3] || 0, ...extra,
    });

    push({ command: 16, lat: home.lat, lng: home.lng, alt: 0, loc: true, isHome: true, segId: null, derived: false, param1: 0, param2: 0, param3: 0, param4: 0 });

    if (P.takeoff) {
        push({
            command: 22, lat: home.lat, lng: home.lng, alt: +P.takeoffAlt || 30, loc: true, segId: null, derived: false,
            param1: isPlane ? 15 : 0, param2: 0, param3: 0, param4: 0, kind: 'takeoff',
        });
    }

    let currentSpeed = 0;
    const setSpeed = (v, segId) => {
        if (v > 0 && Math.abs(v - currentSpeed) > 0.01) {
            doItem(178, [1, v, -1, 0], segId);
            currentSpeed = v;
        }
    };
    if (+P.defaultSpeed > 0) setSpeed(+P.defaultSpeed, null);

    let lastNav = home;      // last located nav point, for lane-ordering and duration
    let lastNavItem = null;  // …and the item it became, for shot positions
    let roiActive = false;
    let roiTarget = null;    // {lat, lng, alt} the camera is locked on
    const gimbal = { pitch: Number.isFinite(+P.gimbalPitch) ? +P.gimbalPitch : -90, yaw: +P.gimbalYaw || 0 };
    const shots = [];        // single-photo actions: the item the photo is taken at

    // ── Segments ──────────────────────────────────────────────────────────────
    for (const seg of route.segments) {
        const def = SEGMENT_TYPES[seg.type];
        if (!def) continue;
        const sp = seg.params;
        const alt = segmentAlt(seg, P);
        const stats = segStats[seg.id] = { lengthM: 0, durationS: 0, waypoints: 0, photos: 0, lanes: 0, gsd: null, area: 0 };
        const speed = +sp.speed > 0 ? +sp.speed : (+P.defaultSpeed || 0);

        if (seg.type === 'poi') {
            if (sp.mode === 'clear') {
                doItem(197, [0, 0, 0, 0], seg.id);
                roiActive = false;
                roiTarget = null;
            } else {
                if (!seg.points[0]) continue;
                push({
                    command: 201, lat: seg.points[0].lat, lng: seg.points[0].lng, alt: +sp.alt || 0, loc: true, roi: true,
                    segId: seg.id, derived: false, param1: 0, param2: 0, param3: 0, param4: 0,
                });
                roiActive = true;
                roiTarget = { lat: seg.points[0].lat, lng: seg.points[0].lng, alt: +sp.alt || 0, segId: seg.id };
            }
            continue;
        }

        // Expand the segment into its navigation points
        let navPts = [];
        const cmdWp = ((sp.turn === 'route' ? P.turnType : sp.turn) === 'spline') ? 82 : 16;
        switch (seg.type) {
            case 'waypoint':
                if (seg.points[0]) navPts.push({ ...seg.points[0], alt, command: cmdWp, param1: +sp.delay || 0, param2: +sp.acceptRadius || 0 });
                break;
            case 'circle': {
                if (!seg.points[0]) break;
                const r = Math.max(5, +sp.radius || 50);
                const turns = Math.max(1, Math.round(+sp.turns || 1));
                navPts.push({ ...seg.points[0], alt, command: 18, param1: turns, param3: sp.direction === 'ccw' ? -r : r, loiter: { r, turns } });
                break;
            }
            case 'perimeter': {
                if (seg.points.length < 3) { addIssue('error', 'Perimeter needs at least 3 vertices', seg.id); break; }
                let pts = seg.points.slice();
                if (sp.direction === 'ccw') pts = [pts[0], ...pts.slice(1).reverse()];
                if (sp.closed) pts.push(pts[0]);
                navPts = pts.map(p => ({ ...p, alt, command: cmdWp }));
                break;
            }
            case 'area': {
                if (seg.points.length < 3) { addIssue('error', 'Area scan needs at least 3 vertices', seg.id); break; }
                const g = surveyGeometry(seg, alt, P.camera);
                stats.gsd = g.gsd;
                stats.area = polygonArea(seg.points);
                const lanes = lawnmower(seg.points, +sp.angle || 0, g.sideDistance, +sp.overshoot || 0, lastNav);
                let pts = lanes.points;
                stats.lanes = lanes.count;
                if (sp.doubleGrid) {
                    const cross = lawnmower(seg.points, (+sp.angle || 0) + 90, g.sideDistance, +sp.overshoot || 0, pts[pts.length - 1] || lastNav);
                    pts = pts.concat(cross.points.map(q => ({ ...q, lane: 'x' + q.lane })));
                    stats.lanes += cross.count;
                }
                if (!pts.length) { addIssue('error', 'Area too small for the lane spacing — reduce side distance or overlap', seg.id); break; }
                navPts = pts.map(p => ({ ...p, alt, command: 16 }));
                if (sp.trigger) navPts.trigger = g.triggerDistance;
                break;
            }
            case 'corridor': {
                if (seg.points.length < 2) { addIssue('error', 'Corridor needs at least 2 points', seg.id); break; }
                const g = surveyGeometry(seg, alt, P.camera);
                stats.gsd = g.gsd;
                const passes = corridorPasses(seg.points, Math.max(1, +sp.width || 60), g.sideDistance, lastNav);
                stats.lanes = passes.count;
                navPts = passes.points.map(p => ({ ...p, alt, command: 16 }));
                if (sp.trigger) navPts.trigger = g.triggerDistance;
                break;
            }
            case 'landing':
                if (seg.points[0]) navPts.push({ ...seg.points[0], alt: 0, command: sp.vtol ? 85 : 21, param1: +sp.abortAlt || 0, landing: true });
                break;
        }
        if (!navPts.length) continue;

        setSpeed(speed, seg.id);

        const actionsAt = (idx) => {
            if (idx > 0 && P.actionExec !== 'every') return;
            for (const a of seg.actions || []) {
                emitAction(a, seg.id, doItem);
                // A DO_ command runs once the previous waypoint is reached, so that is where the camera is
                if (a.type === 'gimbal') { gimbal.pitch = +a.pitch || 0; gimbal.yaw = +a.yaw || 0; }
                if (a.type === 'shot' && lastNavItem) shots.push({ item: lastNavItem, segId: seg.id, gimbal: { ...gimbal } });
            }
        };
        // Actions are emitted AFTER the point they belong to: ArduPilot runs a DO_
        // command once the preceding NAV item is reached, which is what "at the
        // segment start" means to the operator. The trigger-by-
        // distance therefore starts at the first lane point, not on the transit leg.
        navPts.forEach((np, i) => {
            const it = push({
                command: np.command, lat: np.lat, lng: np.lng, alt: np.alt, loc: true, segId: seg.id, derived: false,
                param1: np.param1 || 0, param2: np.param2 || 0, param3: np.param3 || 0, param4: np.param4 || 0,
                loiter: np.loiter, landing: np.landing, segIdx: i, lane: np.lane,
                gimbal: { ...gimbal }, roiTarget: roiActive ? roiTarget : null, trigger: navPts.trigger || 0,
            });
            stats.waypoints++;
            stats.lengthM += haversine(lastNav, np);
            if (np.loiter) stats.lengthM += 2 * Math.PI * np.loiter.r * np.loiter.turns;
            lastNav = np;
            lastNavItem = it;
            if (i === 0 && navPts.trigger) doItem(206, [navPts.trigger, 0, 1, 0], seg.id);
            actionsAt(i);
        });

        if (navPts.trigger) doItem(206, [0, 0, 0, 0], seg.id);
        if (speed > 0) stats.durationS = stats.lengthM / speed;
        for (const a of seg.actions || []) if (a.type === 'wait') stats.durationS += +a.seconds || 0;
    }

    // ── End of route ──────────────────────────────────────────────────────────
    const lastSeg = route.segments[route.segments.length - 1];
    const endsWithLanding = lastSeg?.type === 'landing';
    if (roiActive) doItem(197, [0, 0, 0, 0], null);
    if (!endsWithLanding && P.endAction === 'rtl') {
        doItem(20, [0, 0, 0, 0], null, { kind: 'rtl' });
    } else if (!endsWithLanding && P.endAction === 'land' && lastNav !== home) {
        push({ command: 21, lat: lastNav.lat, lng: lastNav.lng, alt: 0, loc: true, segId: null, derived: false, param1: 0, param2: 0, param3: 0, param4: 0, landing: true });
    }

    // ── Terrain-following subdivision (AGL mode only) ─────────────────────────
    let items = out;
    if (P.altMode === 'agl') items = subdivideForTerrain(out, +P.aglTolerance || 10, terrain);

    // ── Resolve altitudes ─────────────────────────────────────────────────────
    let missingTerrain = 0;
    for (const it of items) {
        if (!it.loc) continue;
        const t = terrain(it.lat, it.lng);
        it.terrain = t;
        if (it.isHome) { it.altMsl = homeElev; it.altRel = 0; it.alt = 0; continue; }
        if (it.roi) { it.altMsl = homeElev + it.alt; it.altRel = it.alt; continue; }
        if (it.landing) { it.altMsl = t !== null ? t : homeElev; it.altRel = it.altMsl - homeElev; it.alt = 0; continue; }
        if (it.kind === 'takeoff') { it.altMsl = homeElev + it.alt; it.altRel = it.alt; continue; }   // always above the take-off point
        if (t === null && !it.derived) missingTerrain++;
        const a = +it.alt || 0;
        switch (P.altMode) {
            case 'amsl':
                it.altMsl = a;
                it.altRel = a - homeElev;
                it.alt = t !== null ? a - t : a - homeElev;
                break;
            case 'rel':
                it.altMsl = homeElev + a;
                it.altRel = a;
                it.alt = t !== null ? it.altMsl - t : a;
                break;
            default: // agl
                it.altMsl = (t !== null ? t : homeElev) + a;
                it.altRel = it.altMsl - homeElev;
                it.alt = a;
        }
    }
    if (missingTerrain) addIssue('warn', `No elevation data under ${missingTerrain} point${missingTerrain > 1 ? 's' : ''} — take-off elevation assumed`);

    // ── Validation ────────────────────────────────────────────────────────────
    const navLocated = items.filter(it => it.loc && !it.roi);
    const flying = navLocated.filter(it => !it.isHome && !it.landing);
    const minClear = +P.minClearance || 0;
    const maxAgl = +P.maxAgl || 0;
    const below = new Set(), low = new Set(), high = new Set();
    for (const it of flying) {
        if (it.terrain === null) continue;
        const agl = it.altMsl - it.terrain;
        if (agl <= 0) below.add(it.segId);
        else if (agl < minClear) low.add(it.segId);
        if (maxAgl > 0 && agl > maxAgl) high.add(it.segId);
    }
    // Legs: the straight line may clip a ridge between two safe points (AMSL / REL
    // modes have no subdivision, and AGL keeps only within tolerance).
    for (let i = 1; i < navLocated.length; i++) {
        const a = navLocated[i - 1], b = navLocated[i];
        if (a.landing || b.landing || b.isHome || a.isHome) continue;   // climb-out and descent are vertical
        const hit = legClearance(a, b, terrain);
        if (hit === null) continue;
        if (hit <= 0) below.add(b.segId);
        else if (hit < minClear) low.add(b.segId);
    }
    for (const id of below) addIssue('error', 'Path goes below the terrain', id);
    for (const id of low) if (!below.has(id)) addIssue('warn', `Clearance under ${minClear} m`, id);
    for (const id of high) addIssue('warn', `Above the ${maxAgl} m AGL ceiling`, id);

    if (items.length > ARDUPILOT_ITEM_LIMIT) addIssue('error', `${items.length} items — ArduPilot stores at most ${ARDUPILOT_ITEM_LIMIT}`);
    if (!P.takeoff && !isPlane && route.segments.length) addIssue('warn', 'No automatic take-off — the vehicle must already be airborne when AUTO starts');
    if (route.segments.length && flying.length) {
        const far = haversine(home, flying[0]);
        if (far > 5000) addIssue('warn', `First waypoint is ${(far / 1000).toFixed(1)} km from the take-off point`);
    }
    for (const seg of route.segments) {
        const spd = +seg.params.speed || +P.defaultSpeed || 0;
        if (spd > 40) addIssue('warn', `${spd} m/s is fast for a multirotor`, seg.id);
    }

    // ── Camera preview: photo footprints and POI view cones ───────────────────
    const camera = buildCameraPreview(route, items, navLocated, shots);
    for (const [segId, n] of camera.photosBySeg) if (segStats[segId]) segStats[segId].photos = n;

    // ── Sequence numbers, stats, profile path ─────────────────────────────────
    items.forEach((it, i) => { it.seq = i; it.frame = it.loc ? 0 : 3; });

    const navPath = [];
    let dist = 0, prev = null;
    for (const it of navLocated) {
        if (prev) dist += haversine(prev, it);
        navPath.push({ lat: it.lat, lng: it.lng, dist, altMsl: it.altMsl, agl: it.terrain !== null ? it.altMsl - it.terrain : null, terrain: it.terrain, segId: it.segId, seq: it.seq, derived: !!it.derived, isHome: !!it.isHome, landing: !!it.landing, loiter: it.loiter });
        prev = it;
    }
    const stats = buildStats(route, items, navLocated, segStats, homeElev, home);

    return { items, stats, issues, navPath, segStats, home: { ...home, elev: homeElev }, camera };
}

// ── Camera preview ────────────────────────────────────────────────────────────

const MAX_PHOTOS = 4000;

/**
 * Where every photo lands and what the camera sees while locked on a POI.
 *  photos: [{ lat, lng, poly, segId }]        one per trigger / shot, poly = ground footprint
 *  cones:  [{ from, poly, target, segId }]     from a waypoint towards the POI
 */
function buildCameraPreview(route, items, navLocated, shots) {
    const cam = route.params.camera;
    const photos = [], cones = [];
    const photosBySeg = new Map();
    const aglOf = it => it.terrain !== null && it.terrain !== undefined ? it.altMsl - it.terrain : it.altRel;

    // Trigger-by-distance along survey lanes: consecutive base points sharing a lane id
    let lane = null;
    const flushLane = () => {
        if (!lane || lane.pts.length < 2) { lane = null; return; }
        const spacing = lane.trigger;
        const g = lane.gimbal;
        for (const ph of photosAlong(lane.pts, spacing, bearing, haversine)) {
            if (photos.length >= MAX_PHOTOS) break;
            const poly = groundFootprint(ph, lane.agl, ph.heading + (g.yaw || 0), g.pitch ?? -90, cam);
            if (poly) photos.push({ lat: ph.lat, lng: ph.lng, poly, segId: lane.segId });
            photosBySeg.set(lane.segId, (photosBySeg.get(lane.segId) || 0) + 1);
        }
        lane = null;
    };
    for (const it of navLocated) {
        if (it.derived || !it.lane || !(it.trigger > 0)) { flushLane(); continue; }
        if (lane && (lane.segId !== it.segId || lane.id !== it.lane)) flushLane();
        if (!lane) lane = { id: it.lane, segId: it.segId, trigger: it.trigger, gimbal: it.gimbal, agl: Math.max(1, aglOf(it)), pts: [] };
        lane.pts.push(it);
    }
    flushLane();

    // Single shots: taken at the waypoint the vehicle has just reached, looking along the next leg
    for (const sh of shots) {
        const it = sh.item;
        const i = navLocated.indexOf(it);
        const next = navLocated[i + 1] || navLocated[i - 1];
        const heading = next ? bearing(it, next) : 0;
        const poly = groundFootprint(it, Math.max(1, aglOf(it)), heading + (sh.gimbal.yaw || 0), sh.gimbal.pitch ?? -90, cam);
        if (poly) photos.push({ lat: it.lat, lng: it.lng, poly, segId: sh.segId, shot: true });
        photosBySeg.set(sh.segId, (photosBySeg.get(sh.segId) || 0) + 1);
    }

    // POI: from every base point flown while the ROI is active, the wedge the camera sees
    for (const it of navLocated) {
        if (!it.roiTarget || it.derived || it.isHome || it.landing || it.lane) continue;
        const agl = Math.max(1, aglOf(it));
        const t = it.roiTarget;
        const targetH = t.alt - (it.altRel - agl);   // ROI height is relative to home; bring it to the local ground
        const spots = it.loiter
            ? Array.from({ length: 6 }, (_, k) => { const f = localFrame(it); const a = k * Math.PI / 3; return f.toLL({ x: it.loiter.r * Math.sin(a), y: it.loiter.r * Math.cos(a) }); })
            : [it];
        for (const pos of spots) {
            const aim = aimAt(pos, agl, t, targetH);
            // Show the frame around the POI, not everything to the horizon behind it
            const poly = groundFootprint(pos, agl, aim.azimuth, aim.pitch, cam, Math.min(aim.distance * 1.4, aim.distance + 4 * agl));
            if (poly) cones.push({ from: { lat: pos.lat, lng: pos.lng }, poly, target: { lat: t.lat, lng: t.lng }, segId: it.segId, poiId: t.segId, onBasePoint: !it.loiter });
        }
    }
    return { photos, cones, photosBySeg, truncated: photos.length >= MAX_PHOTOS };
}

// ── Actions ───────────────────────────────────────────────────────────────────

function emitAction(a, segId, doItem) {
    switch (a.type) {
        case 'trigger_dist': doItem(206, [+a.distance || 0, 0, 1, 0], segId); break;
        case 'shot':         doItem(203, [0, 0, 0, 0], segId, { param5: 1 }); break;
        case 'gimbal':       doItem(205, [+a.pitch || 0, 0, +a.yaw || 0, 0], segId, { alt: 2 }); break;   // z = MAV_MOUNT_MODE_MAVLINK_TARGETING
        case 'yaw':          doItem(115, [+a.heading || 0, 0, 1, 0], segId); break;
        case 'wait':         doItem(93, [+a.seconds || 0, -1, -1, 0], segId); break;
        case 'speed':        doItem(178, [1, +a.speed || 0, -1, 0], segId); break;
        case 'servo':        doItem(183, [+a.channel || 0, +a.pwm || 0, 0, 0], segId); break;
        case 'relay':        doItem(181, [+a.relay || 0, +a.state ? 1 : 0, 0, 0], segId); break;
        case 'raw':          doItem(+a.command || 0, [+a.param1 || 0, +a.param2 || 0, +a.param3 || 0, +a.param4 || 0], segId); break;
    }
}

// ── Terrain following ─────────────────────────────────────────────────────────

/**
 * Insert derived waypoints wherever the straight line between two AGL points
 * would deviate more than `tol` metres from the terrain-following altitude.
 * Douglas-Peucker on the deviation profile: split at the worst sample, recurse.
 */
function subdivideForTerrain(items, tol, terrain) {
    const result = [];
    let prev = null;
    for (const it of items) {
        if (it.loc && !it.roi && !it.isHome && !it.landing && prev && !prev.landing) {
            const inserted = [];
            splitLeg(prev, it, terrain, tol, inserted, 0);
            for (const d of inserted) result.push(d);
        }
        result.push(it);
        if (it.loc && !it.roi) prev = it;
    }
    return result;
}

function splitLeg(a, b, terrain, tol, acc, depth) {
    if (depth > 7) return;
    const len = haversine(a, b);
    if (len < 40) return;
    const n = Math.min(300, Math.max(4, Math.ceil(len / 20)));
    const ta = terrain(a.lat, a.lng), tb = terrain(b.lat, b.lng);
    if (ta === null || tb === null) return;
    const mslA = ta + (+a.alt || 0), mslB = tb + (+b.alt || 0);
    let worst = 0, worstT = -1;
    for (let s = 1; s < n; s++) {
        const t = s / n;
        const lat = a.lat + (b.lat - a.lat) * t, lng = a.lng + (b.lng - a.lng) * t;
        const g = terrain(lat, lng);
        if (g === null) continue;
        const wanted = g + (a.alt + (b.alt - a.alt) * t);
        const straight = mslA + (mslB - mslA) * t;
        const dev = Math.abs(wanted - straight);
        if (dev > worst) { worst = dev; worstT = t; }
    }
    if (worst <= tol || worstT < 0) return;
    const mid = {
        command: b.command === 82 ? 82 : 16, loc: true, derived: true, segId: b.segId,
        lat: a.lat + (b.lat - a.lat) * worstT, lng: a.lng + (b.lng - a.lng) * worstT,
        alt: a.alt + (b.alt - a.alt) * worstT, param1: 0, param2: 0, param3: 0, param4: 0,
    };
    splitLeg(a, mid, terrain, tol, acc, depth + 1);
    acc.push(mid);
    splitLeg(mid, b, terrain, tol, acc, depth + 1);
}

/** Minimum clearance along a straight leg (m), or null when there is no terrain data. */
function legClearance(a, b, terrain) {
    const len = haversine(a, b);
    const n = Math.min(200, Math.max(2, Math.ceil(len / 25)));
    let min = null;
    for (let s = 1; s < n; s++) {
        const t = s / n;
        const g = terrain(a.lat + (b.lat - a.lat) * t, a.lng + (b.lng - a.lng) * t);
        if (g === null) continue;
        const alt = a.altMsl + (b.altMsl - a.altMsl) * t;
        const c = alt - g;
        if (min === null || c < min) min = c;
    }
    return min;
}

// ── Survey patterns ───────────────────────────────────────────────────────────

function rotator(headingDeg) {
    // Lane heading measured clockwise from north. Rotate so the lane direction
    // becomes the local +x axis (east).
    const α = -Math.atan2(Math.cos(headingDeg * D2R), Math.sin(headingDeg * D2R));
    const c = Math.cos(α), s = Math.sin(α);
    return {
        fwd: p => ({ x: p.x * c - p.y * s, y: p.x * s + p.y * c }),
        inv: p => ({ x: p.x * c + p.y * s, y: -p.x * s + p.y * c }),
    };
}

/**
 * Boustrophedon lanes across a polygon. Lanes run along `headingDeg`; the set is
 * centred inside the polygon so the outermost lanes sit half a spacing from the
 * boundary. Returns the lane end-points in flight order, entering at whichever
 * corner is nearest to `startNear`.
 */
function lawnmower(polyLL, headingDeg, spacing, overshoot, startNear) {
    const f = localFrame(centroid(polyLL));
    const rot = rotator(headingDeg);
    const Q = polyLL.map(p => rot.fwd(f.toXY(p)));
    const ys = Q.map(q => q.y);
    const minY = Math.min(...ys), maxY = Math.max(...ys);
    const extent = maxY - minY;
    const n = Math.max(1, Math.ceil(extent / spacing));
    const y0 = minY + (extent - (n - 1) * spacing) / 2;

    const lanes = [];   // [{y, spans:[[x0,x1],…]}]
    for (let k = 0; k < n; k++) {
        const y = y0 + k * spacing;
        const xs = [];
        for (let i = 0, j = Q.length - 1; i < Q.length; j = i++) {
            const a = Q[j], b = Q[i];
            if ((a.y <= y && b.y > y) || (b.y <= y && a.y > y)) {
                xs.push(a.x + (y - a.y) * (b.x - a.x) / (b.y - a.y));
            }
        }
        xs.sort((p, q) => p - q);
        const spans = [];
        for (let c = 0; c + 1 < xs.length; c += 2) {
            if (xs[c + 1] - xs[c] < 0.5) continue;
            spans.push([xs[c] - overshoot, xs[c + 1] + overshoot]);
        }
        if (spans.length) lanes.push({ y, spans });
    }
    if (!lanes.length) return { points: [], count: 0 };

    const build = (reverseLanes, startRight) => {
        const order = reverseLanes ? lanes.slice().reverse() : lanes;
        const pts = [];
        order.forEach((lane, idx) => {
            const rightward = (idx % 2 === 0) !== startRight;   // alternate per lane
            const spans = rightward ? lane.spans : lane.spans.slice().reverse();
            for (const [x0, x1] of spans) {
                const laneId = `${lane.y.toFixed(2)}:${x0.toFixed(1)}`;
                if (rightward) { pts.push({ x: x0, y: lane.y, lane: laneId }); pts.push({ x: x1, y: lane.y, lane: laneId }); }
                else           { pts.push({ x: x1, y: lane.y, lane: laneId }); pts.push({ x: x0, y: lane.y, lane: laneId }); }
            }
        });
        return pts;
    };
    const near = startNear ? rot.fwd(f.toXY(startNear)) : null;
    let best = null, bestD = Infinity;
    for (const rl of [false, true]) for (const sr of [false, true]) {
        const pts = build(rl, sr);
        const d = near ? Math.hypot(pts[0].x - near.x, pts[0].y - near.y) : 0;
        if (d < bestD) { bestD = d; best = pts; }
        if (!near) break;
    }
    return { points: best.map(q => ({ ...f.toLL(rot.inv(q)), lane: q.lane })), count: lanes.length };
}

/** Offset a planar polyline by `d` metres (positive = left of travel), mitred corners. */
function offsetPolyline(P, d) {
    const n = P.length;
    const out = [];
    const norm = (i, j) => {
        const dx = P[j].x - P[i].x, dy = P[j].y - P[i].y;
        const l = Math.hypot(dx, dy) || 1;
        return { x: -dy / l, y: dx / l };
    };
    for (let i = 0; i < n; i++) {
        let nx, ny;
        if (i === 0) ({ x: nx, y: ny } = norm(0, 1));
        else if (i === n - 1) ({ x: nx, y: ny } = norm(n - 2, n - 1));
        else {
            const a = norm(i - 1, i), b = norm(i, i + 1);
            nx = a.x + b.x; ny = a.y + b.y;
            const l = Math.hypot(nx, ny);
            if (l < 1e-6) { nx = a.x; ny = a.y; }
            else {
                nx /= l; ny /= l;
                const cosHalf = Math.max(0.33, nx * a.x + ny * a.y);   // mitre limit ≈ 3
                nx /= cosHalf; ny /= cosHalf;
            }
        }
        out.push({ x: P[i].x + nx * d, y: P[i].y + ny * d });
    }
    return out;
}

/** Closed outline of a corridor band (for drawing), `width` metres wide around the centre line. */
export function corridorOutline(lineLL, width) {
    if (lineLL.length < 2) return [];
    const f = localFrame(centroid(lineLL));
    const P = lineLL.map(f.toXY);
    const left = offsetPolyline(P, width / 2);
    const right = offsetPolyline(P, -width / 2).reverse();
    return left.concat(right).map(f.toLL);
}

/** Parallel passes along a polyline, covering `width` centred on it. */
function corridorPasses(lineLL, width, spacing, startNear) {
    const f = localFrame(centroid(lineLL));
    const P = lineLL.map(f.toXY);
    const n = Math.max(1, Math.round(width / spacing));
    const passes = [];
    for (let k = 0; k < n; k++) {
        const d = (k - (n - 1) / 2) * spacing;
        passes.push(offsetPolyline(P, d));
    }
    const build = (reverse, startBack) => {
        const order = reverse ? passes.slice().reverse() : passes;
        const pts = [];
        order.forEach((pass, idx) => {
            const back = (idx % 2 === 1) !== startBack;
            const seq = back ? pass.slice().reverse() : pass;
            for (const q of seq) pts.push({ ...q, lane: `p${passes.indexOf(pass)}` });
        });
        return pts;
    };
    const near = startNear ? f.toXY(startNear) : null;
    let best = null, bestD = Infinity;
    for (const rv of [false, true]) for (const sb of [false, true]) {
        const pts = build(rv, sb);
        const d = near ? Math.hypot(pts[0].x - near.x, pts[0].y - near.y) : 0;
        if (d < bestD) { bestD = d; best = pts; }
        if (!near) break;
    }
    return { points: best.map(q => ({ ...f.toLL(q), lane: q.lane })), count: n };
}

// ── Statistics ────────────────────────────────────────────────────────────────

function emptyStats() {
    return { lengthM: 0, durationS: 0, items: 0, waypoints: 0, minAgl: null, maxAgl: null, minAmsl: null, maxAmsl: null, photos: 0, areaM2: 0 };
}

function buildStats(route, items, navLocated, segStats, homeElev, home) {
    const s = emptyStats();
    s.items = items.length;
    s.waypoints = navLocated.filter(it => !it.isHome).length;
    for (const it of navLocated) {
        if (it.isHome || it.landing) continue;
        const agl = it.terrain !== null ? it.altMsl - it.terrain : null;
        if (agl !== null) { s.minAgl = s.minAgl === null ? agl : Math.min(s.minAgl, agl); s.maxAgl = s.maxAgl === null ? agl : Math.max(s.maxAgl, agl); }
        s.minAmsl = s.minAmsl === null ? it.altMsl : Math.min(s.minAmsl, it.altMsl);
        s.maxAmsl = s.maxAmsl === null ? it.altMsl : Math.max(s.maxAmsl, it.altMsl);
    }
    for (const st of Object.values(segStats)) { s.photos += st.photos; s.areaM2 += st.area || 0; }

    // Length & duration follow the item sequence so speed changes are honoured
    let speed = 0, prev = null, len = 0, dur = 0;
    const P = route.params;
    if (P.takeoff) dur += (+P.takeoffAlt || 30) / CLIMB_RATE;
    for (const it of items) {
        if (it.command === 178) { speed = it.param2; continue; }
        if (it.command === 93) { dur += it.param1; continue; }
        if (!it.loc || it.roi) continue;
        if (prev) {
            const d = haversine(prev, it);
            len += d;
            dur += speed > 0 ? d / speed : d / 10;
        }
        if (it.loiter) {
            const c = 2 * Math.PI * it.loiter.r * it.loiter.turns;
            len += c;
            dur += speed > 0 ? c / speed : c / 10;
        }
        if (it.landing) dur += Math.max(0, (prev?.altMsl ?? it.altMsl) - it.altMsl) / DESCENT_RATE;
        prev = it;
    }
    const last = items[items.length - 1];
    if (last?.kind === 'rtl' && prev && home) {
        const d = haversine(prev, home);
        len += d;
        dur += (speed > 0 ? d / speed : d / 10) + Math.max(0, prev.altMsl - homeElev) / DESCENT_RATE;
    }
    s.lengthM = len;
    s.durationS = dur;
    return s;
}

