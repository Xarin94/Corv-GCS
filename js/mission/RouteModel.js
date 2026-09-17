/**
 * RouteModel.js - The flight-plan document
 *
 * A route is what the operator edits: an ordered list of high-level SEGMENTS
 * (waypoint, circle, perimeter, area scan, corridor, POI, landing) plus the
 * route-wide parameters (altitude mode, default speed, take-off, end action…).
 * It is deliberately NOT a list of MAVLink commands — RouteCompiler turns the
 * route into STATE.missionItems a moment after the operator stops editing.
 * Everything downstream (upload, 3D trajectory, mini-map, library summary)
 * keeps consuming the compiled items.
 *
 * The route object is a singleton mutated in place: the history module and the
 * library swap its contents with replaceRoute() so every reference stays valid.
 */

// ── Segment catalogue ─────────────────────────────────────────────────────────
// `fields` drive the inspector; `points` says how many base points the tool
// collects on the map (1 = single click, 'poly' = closed polygon, 'line' = open
// polyline). Colours follow the command palette in MissionCommands.js.

export const SEGMENT_TYPES = {
    waypoint: {
        label: 'Waypoint', short: 'WP', points: 1, color: '#44ff44', hotkey: 'W',
        hint: 'Click on the map to add waypoints — Esc to stop',
        defaults: { alt: null, speed: 0, turn: 'route', delay: 0, acceptRadius: 0 },
        fields: [
            { key: 'alt',          label: 'Altitude',      unit: 'm',   type: 'number', min: -500, max: 10000, step: 1, nullable: true, placeholder: 'route' },
            { key: 'speed',        label: 'Speed',         unit: 'm/s', type: 'number', min: 0, max: 100, step: 0.5, zeroLabel: 'route' },
            { key: 'turn',         label: 'Turn type',     type: 'select', options: [['route', 'Route default'], ['straight', 'Straight'], ['spline', 'Spline']] },
            { key: 'delay',        label: 'Hold',          unit: 's',   type: 'number', min: 0, max: 3600, step: 1 },
            { key: 'acceptRadius', label: 'Accept radius', unit: 'm',   type: 'number', min: 0, max: 500, step: 1, zeroLabel: 'auto' },
        ],
    },
    circle: {
        label: 'Circle', short: 'CIRC', points: 1, color: '#4488ff', hotkey: 'C',
        hint: 'Click to place the circle centre',
        defaults: { alt: null, radius: 50, turns: 1, direction: 'cw', speed: 0 },
        fields: [
            { key: 'alt',       label: 'Altitude',  unit: 'm',   type: 'number', min: -500, max: 10000, step: 1, nullable: true, placeholder: 'route' },
            { key: 'radius',    label: 'Radius',    unit: 'm',   type: 'number', min: 5, max: 5000, step: 5 },
            { key: 'turns',     label: 'Loops',     type: 'number', min: 1, max: 100, step: 1 },
            { key: 'direction', label: 'Direction', type: 'select', options: [['cw', 'Clockwise'], ['ccw', 'Counter-clockwise']] },
            { key: 'speed',     label: 'Speed',     unit: 'm/s', type: 'number', min: 0, max: 100, step: 0.5, zeroLabel: 'route' },
        ],
    },
    perimeter: {
        label: 'Perimeter', short: 'PERIM', points: 'poly', color: '#ffaa00', hotkey: 'P',
        hint: 'Click the vertices — Enter or click the first vertex to close, Esc to cancel',
        defaults: { alt: null, speed: 0, direction: 'cw', closed: true },
        fields: [
            { key: 'alt',       label: 'Altitude',  unit: 'm',   type: 'number', min: -500, max: 10000, step: 1, nullable: true, placeholder: 'route' },
            { key: 'speed',     label: 'Speed',     unit: 'm/s', type: 'number', min: 0, max: 100, step: 0.5, zeroLabel: 'route' },
            { key: 'direction', label: 'Direction', type: 'select', options: [['cw', 'As drawn'], ['ccw', 'Reversed']] },
            { key: 'closed',    label: 'Return to start', type: 'check' },
        ],
    },
    area: {
        label: 'Area scan', short: 'AREA', points: 'poly', color: '#ff6600', hotkey: 'A',
        hint: 'Click the vertices of the area — Enter or click the first vertex to close, Esc to cancel',
        defaults: {
            alt: null, speed: 0, angle: 0, spacingMode: 'camera', sideDistance: 30,
            sideOverlap: 70, forwardOverlap: 80,
            overshoot: 0, doubleGrid: false, trigger: true,
        },
        fields: [
            { key: 'alt',            label: 'Altitude',       unit: 'm',   type: 'number', min: -500, max: 10000, step: 1, nullable: true, placeholder: 'route' },
            { key: 'speed',          label: 'Speed',          unit: 'm/s', type: 'number', min: 0, max: 100, step: 0.5, zeroLabel: 'route' },
            { key: 'angle',          label: 'Direction',      unit: '°',   type: 'number', min: -180, max: 180, step: 1 },
            { key: 'spacingMode',    label: 'Lane spacing',   type: 'select', options: [['camera', 'From camera'], ['manual', 'Manual']] },
            { key: 'sideDistance',   label: 'Side distance',  unit: 'm',   type: 'number', min: 1, max: 5000, step: 1, when: { spacingMode: 'manual' } },
            { key: 'sideOverlap',    label: 'Side overlap',   unit: '%',   type: 'number', min: 0, max: 95, step: 5, when: { spacingMode: 'camera' } },
            { key: 'forwardOverlap', label: 'Forward overlap', unit: '%',  type: 'number', min: 0, max: 95, step: 5 },
            { key: 'overshoot',      label: 'Overshoot',      unit: 'm',   type: 'number', min: 0, max: 500, step: 5 },
            { key: 'doubleGrid',     label: 'Double grid',    type: 'check' },
            { key: 'trigger',        label: 'Camera trigger by distance', type: 'check' },
        ],
    },
    corridor: {
        label: 'Corridor', short: 'CORR', points: 'line', color: '#cc44ff', hotkey: 'K',
        hint: 'Click along the corridor — Enter to finish, Esc to cancel',
        defaults: {
            alt: null, speed: 0, width: 60, spacingMode: 'camera', sideDistance: 30,
            sideOverlap: 70, forwardOverlap: 80, trigger: true,
        },
        fields: [
            { key: 'alt',            label: 'Altitude',       unit: 'm',   type: 'number', min: -500, max: 10000, step: 1, nullable: true, placeholder: 'route' },
            { key: 'speed',          label: 'Speed',          unit: 'm/s', type: 'number', min: 0, max: 100, step: 0.5, zeroLabel: 'route' },
            { key: 'width',          label: 'Corridor width', unit: 'm',   type: 'number', min: 1, max: 5000, step: 5 },
            { key: 'spacingMode',    label: 'Lane spacing',   type: 'select', options: [['camera', 'From camera'], ['manual', 'Manual']] },
            { key: 'sideDistance',   label: 'Side distance',  unit: 'm',   type: 'number', min: 1, max: 5000, step: 1, when: { spacingMode: 'manual' } },
            { key: 'sideOverlap',    label: 'Side overlap',   unit: '%',   type: 'number', min: 0, max: 95, step: 5, when: { spacingMode: 'camera' } },
            { key: 'forwardOverlap', label: 'Forward overlap', unit: '%',  type: 'number', min: 0, max: 95, step: 5 },
            { key: 'trigger',        label: 'Camera trigger by distance', type: 'check' },
        ],
    },
    poi: {
        label: 'Point of interest', short: 'POI', points: 1, color: '#ff66aa', hotkey: 'I',
        hint: 'Click to place the point the camera should look at',
        defaults: { alt: 0, mode: 'set' },
        fields: [
            { key: 'mode', label: 'Camera', type: 'select', options: [['set', 'Look at this point'], ['clear', 'Stop looking (ROI off)']] },
            { key: 'alt',  label: 'Target height', unit: 'm', type: 'number', min: -500, max: 10000, step: 1, when: { mode: 'set' } },
        ],
    },
    landing: {
        label: 'Landing', short: 'LAND', points: 1, color: '#ff3333', hotkey: 'L',
        hint: 'Click to place the landing point',
        defaults: { vtol: false, abortAlt: 0 },
        fields: [
            { key: 'vtol',     label: 'VTOL landing', type: 'check' },
            { key: 'abortAlt', label: 'Abort altitude', unit: 'm', type: 'number', min: 0, max: 1000, step: 5, zeroLabel: 'off' },
        ],
    },
};

export const SEGMENT_ORDER = ['waypoint', 'circle', 'perimeter', 'area', 'corridor', 'poi', 'landing'];

// ── Action catalogue ──────────────────────────────────────────────────────────
// Actions hang off a segment and compile to DO_/CONDITION_ items placed before
// the segment's first navigation point (or, with actionExec 'every', before each).

export const ACTION_TYPES = {
    trigger_dist: {
        label: 'Camera trigger by distance', short: 'TRIG', cmd: 206,
        defaults: { distance: 20 },
        fields: [{ key: 'distance', label: 'Every', unit: 'm', type: 'number', min: 0, max: 5000, step: 1, zeroLabel: 'stop' }],
        summary: a => a.distance > 0 ? `every ${a.distance} m` : 'stop',
    },
    shot: {
        label: 'Take a photo', short: 'SHOT', cmd: 203,
        defaults: {},
        fields: [],
        summary: () => 'single shot',
    },
    gimbal: {
        label: 'Camera attitude', short: 'GIMB', cmd: 205,
        defaults: { pitch: -90, yaw: 0 },
        fields: [
            { key: 'pitch', label: 'Pitch', unit: '°', type: 'number', min: -90, max: 30, step: 5 },
            { key: 'yaw',   label: 'Yaw',   unit: '°', type: 'number', min: -180, max: 180, step: 5 },
        ],
        summary: a => `pitch ${a.pitch}° yaw ${a.yaw}°`,
    },
    yaw: {
        label: 'Set heading', short: 'YAW', cmd: 115,
        defaults: { heading: 0 },
        fields: [{ key: 'heading', label: 'Heading', unit: '°', type: 'number', min: 0, max: 359, step: 5 }],
        summary: a => `${a.heading}°`,
    },
    wait: {
        label: 'Wait', short: 'WAIT', cmd: 93,
        defaults: { seconds: 5 },
        fields: [{ key: 'seconds', label: 'Duration', unit: 's', type: 'number', min: 0, max: 3600, step: 1 }],
        summary: a => `${a.seconds} s`,
    },
    speed: {
        label: 'Change speed', short: 'SPD', cmd: 178,
        defaults: { speed: 10 },
        fields: [{ key: 'speed', label: 'Speed', unit: 'm/s', type: 'number', min: 0.5, max: 100, step: 0.5 }],
        summary: a => `${a.speed} m/s`,
    },
    servo: {
        label: 'Set servo', short: 'SRV', cmd: 183,
        defaults: { channel: 9, pwm: 1500 },
        fields: [
            { key: 'channel', label: 'Channel', type: 'number', min: 1, max: 16, step: 1 },
            { key: 'pwm',     label: 'PWM', unit: 'µs', type: 'number', min: 800, max: 2200, step: 10 },
        ],
        summary: a => `ch${a.channel} → ${a.pwm}`,
    },
    relay: {
        label: 'Set relay', short: 'RLY', cmd: 181,
        defaults: { relay: 0, state: 1 },
        fields: [
            { key: 'relay', label: 'Relay', type: 'number', min: 0, max: 5, step: 1 },
            { key: 'state', label: 'State', type: 'select', options: [['1', 'On'], ['0', 'Off']] },
        ],
        summary: a => `relay ${a.relay} ${+a.state ? 'on' : 'off'}`,
    },
    raw: {
        // Anything read back from a vehicle or an old file that has no editor of its own
        label: 'MAVLink command', short: 'CMD', cmd: null,
        defaults: { command: 0, param1: 0, param2: 0, param3: 0, param4: 0 },
        fields: [
            { key: 'command', label: 'Command', type: 'number', min: 0, max: 65535, step: 1 },
            { key: 'param1', label: 'Param 1', type: 'number', step: 0.01 },
            { key: 'param2', label: 'Param 2', type: 'number', step: 0.01 },
            { key: 'param3', label: 'Param 3', type: 'number', step: 0.01 },
            { key: 'param4', label: 'Param 4', type: 'number', step: 0.01 },
        ],
        summary: a => `cmd ${a.command}`,
    },
};

export const ACTION_ORDER = ['trigger_dist', 'shot', 'gimbal', 'yaw', 'wait', 'speed', 'servo', 'relay'];

const D2R = Math.PI / 180;

// ── Camera profile ────────────────────────────────────────────────────────────
// One camera per route (the payload profile). Sensor size + focal length give
// the field of view; pixel count gives the GSD. All of the footprint drawing,
// survey lane spacing and trigger distance derive from this.

export const CAMERA_PRESETS = {
    generic:   { name: 'Generic 4:3 (73°)',      sensorW: 6.17, sensorH: 4.63, focal: 4.2,  imgW: 4000, imgH: 3000 },
    mavic3:    { name: 'DJI Mavic 3 (4/3")',     sensorW: 17.3, sensorH: 13.0, focal: 12.3, imgW: 5280, imgH: 3956 },
    h20wide:   { name: 'DJI Zenmuse H20 wide',   sensorW: 6.17, sensorH: 4.55, focal: 4.5,  imgW: 4056, imgH: 3040 },
    p1_35:     { name: 'DJI Zenmuse P1 · 35 mm', sensorW: 35.9, sensorH: 24.0, focal: 35,   imgW: 8192, imgH: 5460 },
    a6000_20:  { name: 'Sony α6000 · 20 mm',     sensorW: 23.5, sensorH: 15.6, focal: 20,   imgW: 6000, imgH: 4000 },
    rx1r2:     { name: 'Sony RX1R II · 35 mm',   sensorW: 35.9, sensorH: 24.0, focal: 35,   imgW: 7952, imgH: 5304 },
    rededge:   { name: 'MicaSense RedEdge',      sensorW: 4.8,  sensorH: 3.6,  focal: 5.5,  imgW: 1280, imgH: 960 },
    custom:    { name: 'Custom' },
};

export function defaultCamera() {
    return { preset: 'generic', ...CAMERA_PRESETS.generic };
}

export const CAMERA_FIELDS = [
    { key: 'preset',  label: 'Camera',        type: 'select', options: Object.entries(CAMERA_PRESETS).map(([k, v]) => [k, v.name]) },
    { key: 'sensorW', label: 'Sensor width',  unit: 'mm', type: 'number', min: 1, max: 100, step: 0.1 },
    { key: 'sensorH', label: 'Sensor height', unit: 'mm', type: 'number', min: 1, max: 100, step: 0.1 },
    { key: 'focal',   label: 'Focal length',  unit: 'mm', type: 'number', min: 1, max: 600, step: 0.1 },
    { key: 'imgW',    label: 'Image width',   unit: 'px', type: 'number', min: 100, max: 20000, step: 1 },
    { key: 'imgH',    label: 'Image height',  unit: 'px', type: 'number', min: 100, max: 20000, step: 1 },
];

/** Horizontal / vertical field of view in degrees. */
export function cameraFov(cam) {
    const f = Math.max(0.1, +cam?.focal || 4.2);
    const w = Math.max(0.1, +cam?.sensorW || 6.17), h = Math.max(0.1, +cam?.sensorH || 4.63);
    return {
        fovH: 2 * Math.atan(w / (2 * f)) / D2R,
        fovV: 2 * Math.atan(h / (2 * f)) / D2R,
    };
}

// ── Route-level parameters ────────────────────────────────────────────────────

export const ROUTE_PARAM_FIELDS = [
    { key: 'altMode',      label: 'Altitude mode', type: 'select', options: [['agl', 'AGL — follow terrain'], ['amsl', 'AMSL — constant'], ['rel', 'Relative to take-off']] },
    { key: 'defaultAlt',   label: 'Default altitude', unit: 'm',   type: 'number', min: -500, max: 10000, step: 5 },
    { key: 'aglTolerance', label: 'AGL tolerance',    unit: 'm',   type: 'number', min: 1, max: 200, step: 1, when: { altMode: 'agl' } },
    { key: 'defaultSpeed', label: 'Default speed',    unit: 'm/s', type: 'number', min: 0, max: 100, step: 0.5, zeroLabel: 'vehicle' },
    { key: 'turnType',     label: 'Turn type',        type: 'select', options: [['straight', 'Straight'], ['spline', 'Spline']] },
    { key: 'takeoff',      label: 'Automatic take-off', type: 'check' },
    { key: 'takeoffAlt',   label: 'Take-off altitude', unit: 'm', type: 'number', min: 1, max: 1000, step: 5, when: { takeoff: true } },
    { key: 'endAction',    label: 'After last segment', type: 'select', options: [['rtl', 'Return to launch'], ['land', 'Land in place'], ['none', 'Nothing']] },
    { key: 'maxAgl',       label: 'Max altitude AGL', unit: 'm', type: 'number', min: 0, max: 10000, step: 10, zeroLabel: 'no limit' },
    { key: 'minClearance', label: 'Min clearance',    unit: 'm', type: 'number', min: 0, max: 1000, step: 5 },
    { key: 'actionExec',   label: 'Segment actions run', type: 'select', options: [['start', 'At segment start'], ['every', 'At every point']] },
];

/** Gimbal defaults live next to the camera profile in the CAMERA popover. */
export const GIMBAL_FIELDS = [
    { key: 'gimbalPitch', label: 'Default camera pitch', unit: '°', type: 'number', min: -90, max: 30, step: 5, title: '-90 = nadir (straight down), 0 = horizon' },
    { key: 'gimbalYaw',   label: 'Default camera yaw',   unit: '°', type: 'number', min: -180, max: 180, step: 5, title: 'Relative to the flight direction' },
];

export function defaultRouteParams() {
    return {
        altMode: 'agl',
        defaultAlt: 100,
        aglTolerance: 10,
        defaultSpeed: 10,
        turnType: 'straight',
        takeoff: true,
        takeoffAlt: 30,
        endAction: 'rtl',
        maxAgl: 120,
        minClearance: 20,
        actionExec: 'start',
        home: null,          // {lat, lng} — take-off point; null = vehicle home / position
        camera: defaultCamera(),
        gimbalPitch: -90,    // ° — nadir unless a segment carries a gimbal action
        gimbalYaw: 0,        // ° relative to the flight direction
    };
}

// ── The document ──────────────────────────────────────────────────────────────

let nextId = 1;
function newId() {
    return `s${Date.now().toString(36)}${(nextId++).toString(36)}`;
}

export function createRoute() {
    return { name: null, params: defaultRouteParams(), segments: [] };
}

const route = createRoute();

export function getRoute() {
    return route;
}

/** Replace the document contents in place — every holder of the reference keeps working. */
export function replaceRoute(next) {
    route.name = next?.name ?? null;
    route.params = { ...defaultRouteParams(), ...(next?.params || {}) };
    route.params.camera = { ...defaultCamera(), ...(next?.params?.camera || {}) };
    route.segments.length = 0;
    for (const s of next?.segments || []) route.segments.push(normalizeSegment(s));
}

export function clearRoute() {
    route.segments.length = 0;
    route.name = null;
}

export function cloneRoute(r = route) {
    return JSON.parse(JSON.stringify(r));
}

/** Build a segment of `type` at the given points with catalogue defaults. */
export function createSegment(type, points, overrides = {}) {
    const def = SEGMENT_TYPES[type];
    if (!def) throw new Error(`Unknown segment type ${type}`);
    return {
        id: newId(),
        type,
        points: points.map(p => ({ lat: p.lat, lng: p.lng })),
        params: { ...def.defaults, ...overrides },
        actions: [],
    };
}

function normalizeSegment(s) {
    const def = SEGMENT_TYPES[s.type] || SEGMENT_TYPES.waypoint;
    return {
        id: s.id || newId(),
        type: def === SEGMENT_TYPES[s.type] ? s.type : 'waypoint',
        points: (s.points || []).map(p => ({ lat: +p.lat, lng: +p.lng })),
        params: { ...def.defaults, ...(s.params || {}) },
        actions: (s.actions || []).map(a => ({ ...(ACTION_TYPES[a.type]?.defaults || {}), ...a })),
    };
}

export function createAction(type, overrides = {}) {
    const def = ACTION_TYPES[type];
    if (!def) throw new Error(`Unknown action type ${type}`);
    return { type, ...def.defaults, ...overrides };
}

export function getSegment(id) {
    return route.segments.find(s => s.id === id) || null;
}

export function segmentIndex(id) {
    return route.segments.findIndex(s => s.id === id);
}

/** Effective altitude of a segment: its own, or the route default. */
export function segmentAlt(seg, params = route.params) {
    const a = seg.params?.alt;
    return (a === null || a === undefined || a === '') ? params.defaultAlt : +a;
}

/** Whether a field is visible given the current values (`when` clauses). */
export function fieldVisible(field, values) {
    if (!field.when) return true;
    return Object.entries(field.when).every(([k, v]) => values[k] === v);
}

// ── Geometry helpers (shared by compiler and UI) ─────────────────────────────

const R_EARTH = 6371000;

export function haversine(a, b) {
    const dLat = (b.lat - a.lat) * D2R;
    const dLon = (b.lng - a.lng) * D2R;
    const h = Math.sin(dLat / 2) ** 2 +
        Math.cos(a.lat * D2R) * Math.cos(b.lat * D2R) * Math.sin(dLon / 2) ** 2;
    return 2 * R_EARTH * Math.asin(Math.sqrt(h));
}

/** Initial bearing a→b in degrees [0, 360). */
export function bearing(a, b) {
    const φ1 = a.lat * D2R, φ2 = b.lat * D2R, Δλ = (b.lng - a.lng) * D2R;
    const y = Math.sin(Δλ) * Math.cos(φ2);
    const x = Math.cos(φ1) * Math.sin(φ2) - Math.sin(φ1) * Math.cos(φ2) * Math.cos(Δλ);
    return (Math.atan2(y, x) / D2R + 360) % 360;
}

/**
 * Local planar frame around `origin`: metres east (x) and north (y). Accurate to
 * well under a metre for the few-kilometre extents a survey polygon has.
 */
export function localFrame(origin) {
    const kLat = 111320;
    const kLng = 111320 * Math.cos(origin.lat * D2R);
    return {
        toXY: p => ({ x: (p.lng - origin.lng) * kLng, y: (p.lat - origin.lat) * kLat }),
        toLL: q => ({ lat: origin.lat + q.y / kLat, lng: origin.lng + q.x / kLng }),
    };
}

export function centroid(points) {
    if (!points.length) return null;
    let lat = 0, lng = 0;
    for (const p of points) { lat += p.lat; lng += p.lng; }
    return { lat: lat / points.length, lng: lng / points.length };
}

/** Planar polygon area in m² (shoelace in the local frame). */
export function polygonArea(points) {
    if (points.length < 3) return 0;
    const f = localFrame(centroid(points));
    const xy = points.map(f.toXY);
    let a = 0;
    for (let i = 0, j = xy.length - 1; i < xy.length; j = i++) {
        a += xy[j].x * xy[i].y - xy[i].x * xy[j].y;
    }
    return Math.abs(a) / 2;
}

export function pathLength(points) {
    let d = 0;
    for (let i = 1; i < points.length; i++) d += haversine(points[i - 1], points[i]);
    return d;
}

/** Nadir footprint on the ground at `alt` metres, from the camera profile. */
export function cameraFootprint(alt, cam) {
    const { fovH, fovV } = cameraFov(cam);
    return {
        width: 2 * alt * Math.tan((fovH * D2R) / 2),
        height: 2 * alt * Math.tan((fovV * D2R) / 2),
    };
}

/**
 * Lane spacing and trigger distance for a survey segment, from either the
 * manual side distance or the route camera. Also the GSD — the number a
 * surveyor actually plans against.
 */
export function surveyGeometry(seg, alt, cam = route.params.camera) {
    const p = seg.params;
    const fp = cameraFootprint(Math.max(1, alt), cam);
    const sideDistance = p.spacingMode === 'manual'
        ? Math.max(1, +p.sideDistance || 30)
        : Math.max(1, fp.width * (1 - (p.sideOverlap || 0) / 100));
    const triggerDistance = Math.max(0.5, fp.height * (1 - (p.forwardOverlap || 0) / 100));
    const gsd = cam?.imgW > 0 ? (fp.width / cam.imgW) * 100 : null;   // cm/px
    return { sideDistance, triggerDistance, gsd, footprint: fp };
}
