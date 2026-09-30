/**
 * Mission3D.js - The mission route in the 3D view
 *
 * Drawn the way the flight plan page draws it, so the two views read alike:
 *   - between segments and through plain waypoints, a solid line in the route
 *     colour, with a symbol, a drop line and a label on every waypoint;
 *   - a pattern (area scan, corridor, perimeter): its passes as a thinner
 *     dashed line in the segment's colour, marked only where the vehicle
 *     enters and leaves it. A survey compiles to hundreds of waypoints, and a
 *     symbol on each one buried the lanes they make up;
 *   - a circle: the loiter ring in the circle colour, with arrows for the
 *     direction of turn. The route meets the ring at the point nearest to
 *     where the vehicle comes from and leaves it from there, as ArduPilot
 *     flies it — never through the centre;
 *   - a landing: across at height, then straight down (a fixed-wing NAV_LAND
 *     keeps its glide);
 *   - return to launch: a dashed line back over home, then down;
 *   - a POI: a target at the height the camera aims at, on a pole from the
 *     ground.
 */

import { ThickLine } from './ThickLine.js';
import { SymbolLayer, SHAPE, makeLabel, fitLabel, disposeLabel } from './SymbolLayer.js';

const PATTERN_TYPES = new Set(['area', 'corridor', 'perimeter']);
const SEGMENT_TAGS = { area: 'AREA', corridor: 'CORR', perimeter: 'PERIM', circle: 'CIRC' };
const KIND_SHAPES = { takeoff: SHAPE.TRIANGLE, land: SHAPE.DIAMOND, loiter: SHAPE.CIRCLE, wp: SHAPE.SQUARE };
const KIND_TAGS = { takeoff: 'T/O', land: 'LAND', loiter: 'LOIT' };

const MAX_WP_LABELS = 60;      // plain waypoints; a mission read back from a vehicle can have hundreds
const LABEL_OFFSET_PX = 17;
const RING_SEGMENTS = 72;
const RING_ARROWS = 4;
const DASH = [9, 6];           // px: dash, gap

let sceneRef = null;
let cameraRef = null;
let rendererRef = null;
let palette = null;

let routeLine = null;          // transit legs and plain waypoints
let patternLine = null;        // pattern passes, dashed, coloured per segment
let ringLine = null;           // loiter rings and their arrows
let returnLine = null;         // return to launch, dashed
let dropLines = null;          // symbol → ground, coloured per symbol
let activeLeg = null;          // the leg being flown
let symbols = null;
let labels = [];

let mission = null;            // last input, rebuilt when the palette changes
let legs = new Map();          // seq → flat xyz pairs of the path flown towards that item
let symbolBySeq = new Map();   // seq → index in `symbols`
let activeSeq = null;
let emphasized = -1;

/**
 * @param {THREE.Scene} scene
 * @param {THREE.Camera} camera
 * @param {THREE.WebGLRenderer} renderer
 * @param {object} initialPalette route, activeLeg, label and one colour per segment type (see Scene3D)
 */
export function initMission3D(scene, camera, renderer, initialPalette) {
    sceneRef = scene;
    cameraRef = camera;
    rendererRef = renderer;
    palette = initialPalette;
    routeLine = new ThickLine({ mode: 'pairs', color: palette.route, width: 3, ghost: { width: 2, opacity: 0.35 }, renderOrder: 4 }).addTo(scene);
    patternLine = new ThickLine({ mode: 'pairs', vertexColors: true, width: 2, dash: DASH, ghost: { width: 1.5, opacity: 0.3 }, renderOrder: 4 }).addTo(scene);
    ringLine = new ThickLine({ mode: 'pairs', color: palette.circle, width: 2.5, ghost: { width: 1.5, opacity: 0.35 }, renderOrder: 4 }).addTo(scene);
    returnLine = new ThickLine({ mode: 'pairs', color: palette.route, width: 2, dash: DASH, ghost: { width: 1.5, opacity: 0.3 }, renderOrder: 4 }).addTo(scene);
    dropLines = new ThickLine({ mode: 'pairs', vertexColors: true, width: 1.5, opacity: 0.5, renderOrder: 3 }).addTo(scene);
    activeLeg = new ThickLine({ mode: 'pairs', color: palette.activeLeg, width: 4.5, renderOrder: 6 }).addTo(scene);
    symbols = new SymbolLayer({ sizeNear: 30, sizeFar: 18, nearDist: 300, farDist: 6000, ghostOpacity: 0.45, renderOrder: 10 }).addTo(scene);
}

/** A new theme palette: line colours now, symbols and labels by rebuilding. */
export function setMission3DPalette(next) {
    palette = next;
    if (!routeLine) return;
    routeLine.setColor(palette.route);
    ringLine.setColor(palette.circle);
    returnLine.setColor(palette.route);
    activeLeg.setColor(palette.activeLeg);
    if (mission) updateMission3D(mission);
}

const segmentColor = (type) => palette[type] ?? palette.route;

function rgba(hex, out) {
    out.push(((hex >> 16) & 255) / 255, ((hex >> 8) & 255) / 255, (hex & 255) / 255, 1);
}

function pushPair(out, a, b) {
    out.push(a.x, a.y, a.z, b.x, b.y, b.z);
}

/**
 * Draw a mission.
 * @param {{points: Array, pois?: Array, rtlSeq?: number|null}} next
 *   points — located navigation items in mission order, world space:
 *     { x, y, z, ground, seq, kind: 'home'|'takeoff'|'land'|'loiter'|'wp', alt,
 *       derived, segId, segType, loiter: {r, turns, ccw}|null, glide, hold }
 *     `ground` is the terrain height under the point, `derived` marks the
 *     terrain-following points the compiler inserted (path only, no symbol),
 *     `glide` a landing flown down a slope rather than vertically.
 *   pois — { x, y, z, ground, seq }, y being the height the camera aims at.
 *   rtlSeq — sequence number of a closing RETURN_TO_LAUNCH, null for none.
 */
export function updateMission3D(next) {
    if (!sceneRef) return;
    clearMission3D();
    const pts = next?.points || [];
    if (!pts.length) return;
    mission = next;

    // Pattern span: from the first to the last base point of the segment.
    // Terrain-following points before the first belong to the transit leg.
    const span = new Map();
    pts.forEach((p, i) => {
        if (!PATTERN_TYPES.has(p.segType) || p.derived) return;
        const s = span.get(p.segId);
        if (s) s.last = i;
        else span.set(p.segId, { first: i, last: i });
    });

    const route = [], pattern = [], patternRgba = [], ring = [], back = [];
    const syms = [];
    const marks = [];      // labels to make: { at, main, sub, color, plain }
    let from = null;       // where the vehicle sets off for the next item
    let home = null;

    for (let i = 0; i < pts.length; i++) {
        const p = pts[i];
        if (p.kind === 'home') {
            home = home || p;
            from = p;
            continue;
        }
        const s = PATTERN_TYPES.has(p.segType) ? span.get(p.segId) : null;
        const inPattern = !!s && i > s.first && i <= s.last;
        const leg = [];
        let ringPairs = null;
        let exit = p;

        if (p.loiter && p.loiter.r > 0) {
            ringPairs = loiterRing(p, from);
            exit = ringPairs.entry;
            if (from) pushPair(leg, from, exit);
            for (const v of ringPairs.pairs) ring.push(v);
        } else if (p.kind === 'land' && !p.glide && from) {
            const over = { x: p.x, y: Math.max(from.y, p.y), z: p.z };
            if (Math.hypot(over.x - from.x, over.z - from.z) > 0.5) pushPair(leg, from, over);
            pushPair(leg, over, p);
        } else if (from) {
            pushPair(leg, from, p);
        }

        if (inPattern) {
            for (const v of leg) pattern.push(v);
            for (let k = 0; k < leg.length / 3; k++) rgba(segmentColor(p.segType), patternRgba);
        } else {
            for (const v of leg) route.push(v);
        }
        legs.set(p.seq, ringPairs ? leg.concat(ringPairs.pairs) : leg);
        from = exit;

        // Symbol and label. Inside a pattern only its ends are marked.
        if (p.derived) continue;
        const first = !!s && i === s.first, last = !!s && i === s.last;
        if (s && !first && !last) continue;
        const seq = Number.isFinite(p.seq) ? String(p.seq) : '';
        const alt = Number.isFinite(p.alt) ? `${Math.round(p.alt)} m` : '';
        let shape, color, main, sub = '', plain = false;
        if (p.loiter && p.loiter.r > 0) {
            shape = SHAPE.CIRCLE;
            color = palette.circle;
            main = `${seq} ${SEGMENT_TAGS.circle}`.trim();
            sub = alt + (p.loiter.turns > 1 ? ` · ${p.loiter.turns}×` : '');
        } else if (s) {
            shape = SHAPE.SQUARE;
            color = segmentColor(p.segType);
            main = first ? `${seq} ${SEGMENT_TAGS[p.segType]}`.trim() : seq;
            sub = first ? alt : '';
        } else {
            shape = KIND_SHAPES[p.kind] ?? SHAPE.SQUARE;
            color = palette.route;
            const tag = KIND_TAGS[p.kind];
            main = seq ? seq + (tag ? ' ' + tag : '') : (tag || 'WP');
            if (p.kind !== 'land') sub = alt + (p.hold > 0 ? ` · ${Math.round(p.hold)} s` : '');
            plain = p.kind === 'wp';
        }
        symbolBySeq.set(p.seq, syms.length);
        syms.push({ x: p.x, y: p.y, z: p.z, shape, color });
        marks.push({ at: p, main: main || 'WP', sub, color, plain, ground: p.ground });
    }

    // Return to launch: back over home at the height of the last leg, then down
    if (Number.isFinite(next.rtlSeq) && home && from && from !== home) {
        const over = { x: home.x, y: Math.max(from.y, home.y), z: home.z };
        const leg = [];
        if (Math.hypot(over.x - from.x, over.z - from.z) > 0.5) pushPair(leg, from, over);
        pushPair(leg, over, home);
        for (const v of leg) back.push(v);
        legs.set(next.rtlSeq, leg);
        const mid = { x: (from.x + over.x) / 2, y: over.y, z: (from.z + over.z) / 2 };
        marks.push({ at: mid, main: 'RTL', sub: '', color: palette.route, plain: false, ground: null, noDrop: true });
    }

    // POIs: not flown, so not in the route
    for (const poi of next.pois || []) {
        const ground = Number.isFinite(poi.ground) ? poi.ground : poi.y;
        const top = { x: poi.x, y: Math.max(poi.y, ground), z: poi.z };
        syms.push({ x: top.x, y: top.y, z: top.z, shape: SHAPE.TARGET, color: palette.poi });
        const h = top.y - ground;
        marks.push({ at: top, main: `${Number.isFinite(poi.seq) ? poi.seq + ' ' : ''}POI`, sub: h >= 1 ? `${Math.round(h)} m` : '', color: palette.poi, plain: false, ground });
    }

    // Drop lines from every marked point to a dot on the ground, its shadow
    const drops = [], dropRgba = [];
    for (const m of marks) {
        if (m.noDrop || !Number.isFinite(m.ground) || m.at.y - m.ground < 1) continue;
        drops.push(m.at.x, m.at.y, m.at.z, m.at.x, m.ground, m.at.z);
        rgba(m.color, dropRgba);
        rgba(m.color, dropRgba);
        syms.push({ x: m.at.x, y: m.ground, z: m.at.z, shape: SHAPE.DOT, color: m.color, scale: 0.3 });
    }

    routeLine.setPoints(route, route.length / 3);
    patternLine.setPoints(pattern, pattern.length / 3);
    patternLine.setColors(patternRgba);
    ringLine.setPoints(ring, ring.length / 3);
    returnLine.setPoints(back, back.length / 3);
    dropLines.setPoints(drops, drops.length / 3);
    dropLines.setColors(dropRgba);
    symbols.setSymbols(syms);

    // Labels: every marked point, but a long list of plain waypoints gets
    // every n-th one (and the last) so the screen stays legible
    const plainCount = marks.filter(m => m.plain).length;
    const every = Math.max(1, Math.ceil(plainCount / MAX_WP_LABELS));
    const viewportH = rendererRef ? rendererRef.getSize(new THREE.Vector2()).y : 0;
    let plainIdx = 0;
    for (const m of marks) {
        if (m.plain) {
            const idx = plainIdx++;
            if (idx % every !== 0 && idx !== plainCount - 1) continue;
        }
        const label = makeLabel(m.main, m.sub, palette.label ?? m.color);
        label.position.set(m.at.x, m.at.y, m.at.z);
        fitLabel(label, cameraRef, viewportH, LABEL_OFFSET_PX);
        sceneRef.add(label);
        labels.push(label);
    }

    setMission3DActiveSeq(activeSeq, true);
}

/**
 * The ring of a loiter, starting and ending at its entry point: the point of
 * the ring nearest to `from` (due north of the centre when `from` is on the
 * centre or unknown). Arrows along it show the direction of turn.
 * @returns {{entry: {x,y,z}, pairs: number[]}}
 */
function loiterRing(p, from) {
    const r = p.loiter.r;
    // World axes: x east, z south. Angles are bearings from the centre.
    let a0 = 0;
    if (from) {
        const east = from.x - p.x, north = p.z - from.z;
        if (Math.hypot(east, north) > 0.5) a0 = Math.atan2(east, north);
    }
    const at = (a) => ({ x: p.x + r * Math.sin(a), y: p.y, z: p.z - r * Math.cos(a) });
    const sign = p.loiter.ccw ? -1 : 1;
    const entry = at(a0);
    const pairs = [];
    let prev = entry;
    for (let k = 1; k <= RING_SEGMENTS; k++) {
        const q = at(a0 + sign * k * 2 * Math.PI / RING_SEGMENTS);
        pushPair(pairs, prev, q);
        prev = q;
    }
    // Chevrons pointing along the direction of travel, at the same bearings
    // as the flight plan page's arrows, sized with the ring so they read with it
    const len = Math.max(4, Math.min(120, r * 0.28));
    for (let k = 0; k < RING_ARROWS; k++) {
        const a = (k + 0.5) * 2 * Math.PI / RING_ARROWS;
        const tip = at(a);
        const te = sign * Math.cos(a), tn = -sign * Math.sin(a);   // travel direction (east, north)
        const re = Math.sin(a), rn = Math.cos(a);                  // outward
        for (const side of [1, -1]) {
            const e = -te * len + re * side * len * 0.6;
            const n = -tn * len + rn * side * len * 0.6;
            pushPair(pairs, { x: tip.x + e, y: p.y, z: tip.z - n }, tip);
        }
    }
    return { entry, pairs };
}

/**
 * Highlight the item being flown (MISSION_CURRENT) and the path leading to it.
 * @param {number|null} seq mission sequence number, null for none
 * @param {boolean} [force] re-apply even if unchanged (after a rebuild)
 */
export function setMission3DActiveSeq(seq, force = false) {
    if (!symbols) return;
    if (seq === activeSeq && !force) return;
    activeSeq = seq;
    if (emphasized >= 0) symbols.setEmphasis(emphasized, 1, false);
    emphasized = symbolBySeq.get(seq) ?? -1;
    if (emphasized >= 0) symbols.setEmphasis(emphasized, 1.35, true);
    const leg = legs.get(seq);
    if (leg && leg.length) activeLeg.setPoints(leg, leg.length / 3);
    else activeLeg.clear();
}

/** Remove the whole mission (lines, symbols, labels). */
export function clearMission3D() {
    mission = null;
    legs = new Map();
    symbolBySeq = new Map();
    emphasized = -1;
    if (!routeLine) return;
    for (const line of [routeLine, patternLine, ringLine, returnLine, dropLines, activeLeg]) line.clear();
    symbols.setSymbols([]);
    for (const label of labels) disposeLabel(label);
    labels = [];
}

/** Re-fit the labels after a resize. */
export function resizeMission3D(camera, viewportHeightCss) {
    cameraRef = camera;
    for (const label of labels) fitLabel(label, camera, viewportHeightCss, LABEL_OFFSET_PX);
}
