/**
 * FlightPlanController.js - The Flight Plan page
 *
 * Route editor: pick a tool from the palette on
 * the map, draw the figure (one click for a waypoint, vertices for an area…),
 * then tune it in the inspector that opens under the segment's card. The route
 * is recalculated a moment after every edit — the calculated waypoints, the
 * statistics on the route card and the warnings that gate UPLOAD all come from
 * RouteCompiler; this file only draws and edits.
 *
 * Layers on the map:
 *   segments  – what the operator drew: polygons, corridor bands, circles, POIs
 *   path      – the calculated flight line with direction arrows
 *   points    – calculated waypoints (small) and segment base points (large)
 *   handles   – drag handles of the selected segment (vertices, mid-points, centre, radius)
 *   draft     – the figure being drawn
 */

import { STATE } from '../core/state.js';
import { uploadMission } from '../mavlink/CommandSender.js';
import { getTerrainElevationFromHGT, getTerrainElevationAsync, resetAutoDownloadFailures } from '../terrain/TerrainManager.js';
import { cachedTileLayer } from '../maps/CachedTileLayer.js';
import {
    SEGMENT_TYPES, SEGMENT_ORDER, ACTION_TYPES, ACTION_ORDER, ROUTE_PARAM_FIELDS, CAMERA_FIELDS, GIMBAL_FIELDS, CAMERA_PRESETS,
    getRoute, replaceRoute, clearRoute, createRoute, createSegment, createAction, getSegment,
    segmentAlt, fieldVisible, haversine, bearing, localFrame, centroid, surveyGeometry, cameraFov, cameraFootprint,
} from '../mission/RouteModel.js';
import { compileRoute, corridorOutline } from '../mission/RouteCompiler.js';
import { commitMission, undoMission, redoMission, resetMissionHistory, canUndo, canRedo } from '../mission/MissionHistory.js';
import { initMissionLibrary, openMissionLibrary, saveCurrentMission, getCurrentMissionName, detachCurrentMission } from '../mission/MissionLibrary.js';
import { downloadMission, itemsToRoute } from '../mission/MissionTransfer.js';

// ── Module state ──────────────────────────────────────────────────────────────

let map = null;
let satLayer = null;
let satVisible = true;
let showCalcPoints = true;
let showCamera = true;
let initialized = false;
let visible = false;

let tool = 'select';
let draft = null;                 // { type, points, line, fill, markers, rubber }
let selectedId = null;
let hoverId = null;

let compiled = { items: [], stats: null, issues: [], navPath: [], segStats: {}, home: null, camera: { photos: [], cones: [] } };
let compileTimer = null;
let compileToken = 0;

const layers = {};
const figures = new Map();        // segId → { fig, axis, points[], label, glyph } for live updates while dragging
let cameraRenderer = null;
let vehicleMarker = null;
let homeMarker = null;
let hoverMarker = null;
let centeredOnce = false;

// Elevation profile view state
const profile = { zoom: 1, scroll: 0, hoverDist: null, dragging: false, dragX: 0, scrollStart: 0, bound: false };

const isPlane = () => STATE.vehicleType === 1;

// ── Icons ─────────────────────────────────────────────────────────────────────

const ICON = {
    select:    '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linejoin="round"><path d="M5 3l14 8-6 2-3 6z"/></svg>',
    waypoint:  '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linejoin="round"><path d="M12 21s7-6.5 7-12a7 7 0 1 0-14 0c0 5.5 7 12 7 12z"/><circle cx="12" cy="9" r="2.5"/></svg>',
    circle:    '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="8"/><circle cx="12" cy="12" r="1.5" fill="currentColor"/><path d="M12 12h8" stroke-dasharray="2 2"/></svg>',
    perimeter: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linejoin="round"><path d="M5 6l9-2 6 7-4 9-11-3z"/><circle cx="5" cy="6" r="1.5" fill="currentColor"/><circle cx="14" cy="4" r="1.5" fill="currentColor"/><circle cx="20" cy="11" r="1.5" fill="currentColor"/><circle cx="16" cy="20" r="1.5" fill="currentColor"/><circle cx="5" cy="17" r="1.5" fill="currentColor"/></svg>',
    area:      '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linejoin="round"><path d="M4 5l16-1v15L4 20z"/><path d="M7 8h10M7 12h10M7 16h10" stroke-width="1.3"/></svg>',
    corridor:  '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M3 18c4-2 5-9 9-9s5 6 9 4"/><path d="M3 13c4-2 5-9 9-9s5 6 9 4" stroke-width="1.2" opacity=".6"/><path d="M3 23c4-2 5-9 9-9s5 6 9 4" stroke-width="1.2" opacity=".6"/></svg>',
    poi:       '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="3"/><path d="M12 2v4M12 18v4M2 12h4M18 12h4"/></svg>',
    landing:   '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"><path d="M12 3v11"/><path d="M7 10l5 5 5-5"/><path d="M4 20h16"/></svg>',
    home:      '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linejoin="round"><path d="M3 11l9-8 9 8v10h-6v-6H9v6H3z"/></svg>',
    trash:     '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"><path d="M4 7h16M10 11v6M14 11v6M6 7l1 13h10l1-13M9 7V4h6v3"/></svg>',
    warn:      '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linejoin="round"><path d="M12 3L2 21h20z"/><path d="M12 10v5M12 18v.5"/></svg>',
};

// ── Init ──────────────────────────────────────────────────────────────────────

export function initFlightPlan() {
    if (initialized) return;
    initialized = true;
    initMap();
    initTools();
    initRouteCard();
    initSegmentList();
    initProfile();
    initKeyboard();
    initMissionLibrary(() => {
        selectedId = null;
        resetMissionHistory();
        syncNameField();
        scheduleCompile(0);
        setTimeout(fitRoute, 300);
    });
    scheduleCompile(0);
}

/** Called by the tab controller whenever the page becomes visible. */
export function onFlightPlanShown() {
    visible = true;
    if (!initialized) initFlightPlan();
    setTimeout(() => {
        map?.invalidateSize();
        renderProfile();
        redrawArrows();
    }, 120);
}

export function onFlightPlanHidden() {
    visible = false;
}

// ── Map ───────────────────────────────────────────────────────────────────────

function initMap() {
    const container = document.getElementById('mission-map-full');
    if (!container || typeof L === 'undefined') return;

    map = L.map(container, {
        center: [STATE.lat || 46.0, STATE.lon || 11.0],
        zoom: 13,
        zoomControl: false,
        keyboard: false,
        doubleClickZoom: false,
    });
    L.control.zoom({ position: 'topright' }).addTo(map);

    cachedTileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        maxZoom: 19, attribution: '&copy; OpenStreetMap', provider: 'osm'
    }).addTo(map);
    satLayer = cachedTileLayer('https://mt{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}', {
        subdomains: ['0', '1', '2', '3'], maxZoom: 20, provider: 'esri'
    });
    satLayer.addTo(map);

    for (const name of ['camera', 'segments', 'path', 'arrows', 'points', 'cameraHover', 'handles', 'draft']) {
        layers[name] = L.layerGroup().addTo(map);
    }
    // Photo dots sit above the route line (overlay pane, 400) but under the
    // base-point markers (marker pane, 600) so those stay clickable.
    // Hundreds of them go on one canvas, not hundreds of SVG nodes.
    map.createPane('fpCamera').style.zIndex = 590;
    cameraRenderer = L.canvas({ padding: 0.5, pane: 'fpCamera' });
    // The hovered footprint must not sit between the mouse and the dot's canvas:
    // an <svg> root swallows pointer events, the canvas would get mouseout and
    // the frame would vanish the instant it appears. Own pane, no pointer events.
    const hoverPane = map.createPane('fpCameraHover');
    hoverPane.style.zIndex = 630;
    hoverPane.style.pointerEvents = 'none';
    map.createPane('fpHandles').style.zIndex = 650;
    map.createPane('fpLabels').style.zIndex = 640;

    // Debug hook for the devtools console (the module scope is not reachable otherwise)
    window.__flightPlan = { map, layers, route: getRoute, compiled: () => compiled };

    map.on('click', onMapClick);
    map.on('dblclick', onMapDblClick);
    map.on('contextmenu', onMapContextMenu);
    map.on('mousemove', onMapMouseMove);
    map.on('zoomend', () => { redrawArrows(); renderCameraLayer(); });
    map.on('movestart', hideContextMenu);

    document.getElementById('fp-sat-toggle')?.addEventListener('click', (e) => {
        satVisible = !satVisible;
        if (satVisible) satLayer.addTo(map); else map.removeLayer(satLayer);
        e.currentTarget.classList.toggle('active', satVisible);
    });
    document.getElementById('fp-calc-toggle')?.addEventListener('click', (e) => {
        showCalcPoints = !showCalcPoints;
        e.currentTarget.classList.toggle('active', showCalcPoints);
        renderMap();
    });
    document.getElementById('fp-cam-toggle')?.addEventListener('click', (e) => {
        showCamera = !showCamera;
        e.currentTarget.classList.toggle('active', showCamera);
        renderCameraLayer();
    });
    document.getElementById('fp-fit')?.addEventListener('click', fitRoute);

    setInterval(() => {
        if (!visible) return;
        updateVehicleMarker();
        updateHomeMarker();
    }, 500);
}

function fitRoute() {
    if (!map) return;
    const pts = [];
    for (const s of getRoute().segments) for (const p of s.points) pts.push([p.lat, p.lng]);
    if (compiled.home) pts.push([compiled.home.lat, compiled.home.lng]);
    if (pts.length < 1) return;
    if (pts.length === 1) { map.setView(pts[0], 15); return; }
    map.fitBounds(L.latLngBounds(pts), { padding: [40, 40], maxZoom: 17 });
}

// Vehicle & home ---------------------------------------------------------------

const VEHICLE_SVG = '<svg viewBox="0 0 32 32" width="32" height="32" style="filter:drop-shadow(0 0 3px rgba(0,0,0,0.7));"><polygon points="16,2 22,28 16,22 10,28" fill="#00d2ff" stroke="#fff" stroke-width="1.5"/></svg>';

function updateVehicleMarker() {
    if (!map) return;
    const lat = STATE.lat, lon = STATE.lon;
    if (!lat && !lon) return;
    const yawDeg = (STATE.yaw || 0) * (180 / Math.PI);
    if (!vehicleMarker) {
        const icon = L.divIcon({
            html: `<div style="transform:rotate(${yawDeg}deg);width:32px;height:32px;">${VEHICLE_SVG}</div>`,
            iconSize: [32, 32], iconAnchor: [16, 16], className: 'vehicle-map-marker'
        });
        vehicleMarker = L.marker([lat, lon], { icon, interactive: false, zIndexOffset: 1000 }).addTo(map);
    } else {
        vehicleMarker.setLatLng([lat, lon]);
        const inner = vehicleMarker.getElement()?.querySelector('div');
        if (inner) inner.style.transform = `rotate(${yawDeg}deg)`;
    }
    if (!centeredOnce && !getRoute().segments.length && STATE.homeLat === null) {
        map.setView([lat, lon], 15);
        centeredOnce = true;
    }
}

function updateHomeMarker() {
    if (!map) return;
    const P = getRoute().params;
    const custom = !!P.home;
    // Same precedence as the compiler: explicit point, vehicle home, the point the plan was compiled with
    const hasVehicleHome = STATE.homeLat !== null && STATE.homeLat !== undefined;
    const src = custom ? P.home
        : hasVehicleHome ? { lat: STATE.homeLat, lng: STATE.homeLon }
        : compiled.home ? { lat: compiled.home.lat, lng: compiled.home.lng }
        : (STATE.lat || STATE.lon) ? { lat: STATE.lat, lng: STATE.lon } : null;
    if (!src) { if (homeMarker) { homeMarker.remove(); homeMarker = null; } return; }
    const { lat, lng } = src;

    if (!homeMarker) {
        const icon = L.divIcon({ html: `<div class="fp-home-icon${custom ? ' is-custom' : ''}">H</div>`, iconSize: [26, 26], iconAnchor: [13, 13], className: 'vehicle-map-marker' });
        homeMarker = L.marker([lat, lng], { icon, draggable: true, zIndexOffset: 900, title: 'Take-off point — drag to move' }).addTo(map);
        homeMarker.on('dragend', () => {
            const ll = homeMarker.getLatLng();
            getRoute().params.home = { lat: ll.lat, lng: ll.lng };
            commitMission('Move take-off point');
            renderHomeDesc();
            scheduleCompile();
        });
        homeMarker.on('click', () => { selectSegment(null); openRouteSettings(); });
        if (!centeredOnce && !custom) {
            map.setView([lat, lng], 15);
            centeredOnce = true;
        }
    } else {
        homeMarker.setLatLng([lat, lng]);
        homeMarker.getElement()?.querySelector('.fp-home-icon')?.classList.toggle('is-custom', custom);
    }
    // A real vehicle home arriving or moving is a source change for the compiler
    if (!custom && hasVehicleHome && compiled.home && (Math.abs(compiled.home.lat - lat) > 1e-7 || Math.abs(compiled.home.lng - lng) > 1e-7)) {
        scheduleCompile();
    }
}

// ── Tools ─────────────────────────────────────────────────────────────────────

function initTools() {
    const bar = document.getElementById('fp-tools');
    if (!bar) return;
    const tools = [['select', 'Select / pan', 'V'], ...SEGMENT_ORDER.map(t => [t, SEGMENT_TYPES[t].label, SEGMENT_TYPES[t].hotkey])];
    bar.innerHTML = tools.map(([id, label, key]) =>
        `<button class="fp-tool${id === 'select' ? ' active' : ''}" data-tool="${id}" title="${label} (${key})">${ICON[id]}</button>`
    ).join('');
    bar.querySelectorAll('.fp-tool').forEach(btn => btn.addEventListener('click', () => setTool(btn.dataset.tool)));
    L.DomEvent.disableClickPropagation(bar);
    L.DomEvent.disableScrollPropagation(bar);
    const layersEl = document.getElementById('fp-layers');
    if (layersEl) L.DomEvent.disableClickPropagation(layersEl);
}

function setTool(name) {
    if (draft) cancelDraft();
    tool = name;
    document.querySelectorAll('#fp-tools .fp-tool').forEach(b => b.classList.toggle('active', b.dataset.tool === name));
    const mapEl = document.getElementById('mission-map-full');
    if (mapEl) mapEl.classList.toggle('is-drawing', name !== 'select');
    const hint = document.getElementById('fp-map-hint');
    if (hint) {
        const def = SEGMENT_TYPES[name];
        hint.style.display = def ? 'block' : 'none';
        if (def) hint.textContent = def.hint;
    }
}

function onMapClick(e) {
    hideContextMenu();
    if (tool === 'select') { selectSegment(null); return; }
    const def = SEGMENT_TYPES[tool];
    if (!def) return;
    const p = { lat: e.latlng.lat, lng: e.latlng.lng };

    if (def.points === 1) {
        const seg = createSegment(tool, [p]);
        addSegment(seg, `Add ${def.label.toLowerCase()}`);
        if (tool !== 'waypoint') setTool('select');     // waypoints chain; the others are one-shot
        return;
    }
    // Polygon / polyline: accumulate vertices
    if (!draft) draft = { type: tool, points: [], markers: [], line: null, fill: null, rubber: null };
    if (def.points === 'poly' && draft.points.length >= 3) {
        const first = map.latLngToContainerPoint(draft.points[0]);
        if (first.distanceTo(map.latLngToContainerPoint(e.latlng)) < 12) { finishDraft(); return; }
    }
    draft.points.push(p);
    renderDraft();
}

function onMapDblClick(e) {
    if (draft) { L.DomEvent.stop(e); finishDraft(); }
}

function onMapMouseMove(e) {
    if (draft && draft.points.length) {
        const last = draft.points[draft.points.length - 1];
        const pts = [[last.lat, last.lng], [e.latlng.lat, e.latlng.lng]];
        if (!draft.rubber) draft.rubber = L.polyline(pts, { color: '#fff', weight: 1, dashArray: '4 4', opacity: 0.6, interactive: false }).addTo(layers.draft);
        else draft.rubber.setLatLngs(pts);
    }
}

function renderDraft() {
    if (!draft) return;
    const def = SEGMENT_TYPES[draft.type];
    const lls = draft.points.map(p => [p.lat, p.lng]);
    if (draft.line) draft.line.remove();
    if (draft.fill) draft.fill.remove();
    draft.line = L.polyline(lls, { color: def.color, weight: 2, dashArray: '6 4', interactive: false }).addTo(layers.draft);
    if (def.points === 'poly' && lls.length >= 3) {
        draft.fill = L.polygon(lls, { color: def.color, weight: 0, fillOpacity: 0.12, interactive: false }).addTo(layers.draft);
    }
    while (draft.markers.length < draft.points.length) {
        const i = draft.markers.length;
        const m = L.circleMarker(lls[i], { radius: i === 0 ? 6 : 4, color: '#fff', weight: 1.5, fillColor: def.color, fillOpacity: 1, pane: 'fpHandles' }).addTo(layers.draft);
        draft.markers.push(m);
    }
}

function finishDraft() {
    if (!draft) return;
    const def = SEGMENT_TYPES[draft.type];
    const need = def.points === 'poly' ? 3 : 2;
    const d = draft;
    if (d.points.length < need) { cancelDraft(); return; }
    layers.draft.clearLayers();
    draft = null;
    const seg = createSegment(d.type, d.points);
    addSegment(seg, `Add ${def.label.toLowerCase()}`);
    setTool('select');
}

function cancelDraft() {
    layers.draft.clearLayers();
    draft = null;
}

// ── Editing primitives ────────────────────────────────────────────────────────

function addSegment(seg, label) {
    // A landing must be the last thing; anything added after it goes before it
    const segs = getRoute().segments;
    const lastLanding = segs.length && segs[segs.length - 1].type === 'landing' && seg.type !== 'landing';
    if (lastLanding) segs.splice(segs.length - 1, 0, seg); else segs.push(seg);
    commitMission(label);
    selectSegment(seg.id, { scroll: true });
    scheduleCompile();
}

function deleteSegment(id) {
    const segs = getRoute().segments;
    const i = segs.findIndex(s => s.id === id);
    if (i < 0) return;
    segs.splice(i, 1);
    if (selectedId === id) selectedId = null;
    commitMission('Delete segment');
    scheduleCompile();
}

function selectSegment(id, opts = {}) {
    selectedId = id;
    renderSegmentList();
    renderMap();
    renderProfile();
    if (id && opts.scroll) {
        document.querySelector(`.fp-seg[data-id="${id}"]`)?.scrollIntoView({ block: 'nearest', behavior: 'smooth' });
    }
    if (id && opts.pan) {
        const seg = getSegment(id);
        const c = seg && centroid(seg.points);
        if (c) map.panTo([c.lat, c.lng]);
    }
}

function segmentLabel(seg, idx) {
    return `${idx + 1} · ${SEGMENT_TYPES[seg.type]?.label || seg.type}`;
}

// ── Compile ───────────────────────────────────────────────────────────────────

export function scheduleCompile(delay = 250) {
    clearTimeout(compileTimer);
    setStatus('busy');
    compileTimer = setTimeout(runCompile, delay);
}

async function runCompile() {
    const token = ++compileToken;
    await ensureRouteTerrain();
    if (token !== compileToken) return;

    const P = getRoute().params;
    // Take-off point: explicit, else the vehicle's home, else the vehicle position —
    // taken once, so a moving vehicle without a home does not recompile the plan every tick
    const home = P.home
        || (STATE.homeLat !== null && STATE.homeLat !== undefined ? { lat: STATE.homeLat, lng: STATE.homeLon } : null)
        || (compiled.home ? { lat: compiled.home.lat, lng: compiled.home.lng } : null)
        || (STATE.lat || STATE.lon ? { lat: STATE.lat, lng: STATE.lon } : null);

    compiled = compileRoute(getRoute(), {
        terrain: getTerrainElevationFromHGT,
        home,
        homeAlt: STATE.homeAlt || 0,
        vehicleType: STATE.vehicleType,
    });

    // Publish the calculated mission for the 3D scene, mini-map and library
    STATE.missionItems.length = 0;
    for (const it of compiled.items) STATE.missionItems.push(it);
    window.dispatchEvent(new CustomEvent('missionUpdated'));

    renderRouteCard();
    // Re-rendering the list would steal focus from a field the operator is still editing
    if (document.activeElement?.closest?.('.fp-seg-body')) refreshSummaries();
    else renderSegmentList();
    renderMap();
    renderProfile();
}

function refreshSummaries() {
    for (const seg of getRoute().segments) renderCardSummary(seg);
}

/** Make sure the 1° elevation tiles under every base point and leg are parsed. */
async function ensureRouteTerrain() {
    const seen = new Set();
    const add = (lat, lng) => {
        if (!Number.isFinite(lat) || !Number.isFinite(lng)) return;
        seen.add(`${Math.floor(lat)}_${Math.floor(lng)}`);
    };
    const P = getRoute().params;
    const pts = [];
    if (P.home) pts.push(P.home);
    else if (STATE.homeLat !== null && STATE.homeLat !== undefined) pts.push({ lat: STATE.homeLat, lng: STATE.homeLon });
    for (const s of getRoute().segments) for (const p of s.points) pts.push(p);
    for (let i = 0; i < pts.length; i++) {
        add(pts[i].lat, pts[i].lng);
        if (i > 0) {
            const a = pts[i - 1], b = pts[i];
            const steps = Math.max(2, Math.ceil(Math.max(Math.abs(b.lat - a.lat), Math.abs(b.lng - a.lng)) * 2));
            for (let s = 1; s < steps; s++) add(a.lat + (b.lat - a.lat) * s / steps, a.lng + (b.lng - a.lng) * s / steps);
        }
    }
    if (!seen.size) return;
    resetAutoDownloadFailures();
    await Promise.all([...seen].map(k => {
        const [lat, lon] = k.split('_').map(Number);
        return getTerrainElevationAsync(lat + 0.5, lon + 0.5);
    }));
}

// ── Route card ────────────────────────────────────────────────────────────────

function initRouteCard() {
    const nameEl = document.getElementById('fp-route-name');
    nameEl?.addEventListener('change', () => {
        getRoute().name = nameEl.value.trim() || null;
        commitMission('Rename route');
    });
    syncNameField();

    document.getElementById('fp-upload')?.addEventListener('click', doUpload);
    document.getElementById('fp-download')?.addEventListener('click', doDownload);
    document.getElementById('fp-save')?.addEventListener('click', async () => {
        const btn = document.getElementById('fp-save');
        try {
            const saved = await saveCurrentMission();
            if (!saved) return;
            btn.textContent = 'SAVED';
            setTimeout(() => { btn.textContent = 'SAVE'; }, 1500);
            syncNameField();
        } catch (err) {
            alert('Save failed: ' + err.message);
        }
    });
    document.getElementById('fp-library')?.addEventListener('click', () => openMissionLibrary());

    document.getElementById('fp-camera-btn')?.addEventListener('click', toggleCameraPopover);
    document.getElementById('fp-camera-close')?.addEventListener('click', closeCameraPopover);
    document.getElementById('fp-route-settings')?.addEventListener('click', toggleRouteSettings);
    document.getElementById('fp-route-params-close')?.addEventListener('click', closeRouteSettings);
    document.getElementById('fp-chips')?.addEventListener('click', toggleRouteSettings);
    document.getElementById('fp-home-vehicle')?.addEventListener('click', () => {
        if (!STATE.lat && !STATE.lon) { alert('No vehicle position available'); return; }
        getRoute().params.home = { lat: STATE.lat, lng: STATE.lon };
        commitMission('Set take-off point');
        renderHomeDesc(); updateHomeMarker(); scheduleCompile();
    });
    document.getElementById('fp-home-reset')?.addEventListener('click', () => {
        getRoute().params.home = null;
        commitMission('Reset take-off point');
        renderHomeDesc(); updateHomeMarker(); scheduleCompile();
    });

    // Menu
    const menuBtn = document.getElementById('fp-route-menu');
    const menu = document.getElementById('fp-route-menu-list');
    menuBtn?.addEventListener('click', (e) => {
        e.stopPropagation();
        closeRouteSettings();
        closeCameraPopover();
        menu.style.display = menu.style.display === 'none' ? 'block' : 'none';
    });
    document.addEventListener('click', () => { if (menu) menu.style.display = 'none'; });
    menu?.addEventListener('click', (e) => {
        const act = e.target.closest('button')?.dataset.act;
        if (act) onMenuAction(act);
    });
    document.getElementById('fp-import-file')?.addEventListener('change', onImportFile);

    document.getElementById('fp-issues')?.addEventListener('click', (e) => {
        const id = e.target.closest('[data-seg]')?.dataset.seg;
        if (id) selectSegment(id, { scroll: true, pan: true });
    });

    // Undo / redo
    const undoBtn = document.getElementById('fp-undo');
    const redoBtn = document.getElementById('fp-redo');
    undoBtn?.addEventListener('click', () => applyHistory(undoMission()));
    redoBtn?.addEventListener('click', () => applyHistory(redoMission()));
    window.addEventListener('missionHistoryChanged', (e) => {
        const { canUndo: cu, canRedo: cr, undoLabel, redoLabel } = e.detail;
        if (undoBtn) { undoBtn.disabled = !cu; undoBtn.title = cu ? `Undo: ${undoLabel} (Ctrl+Z)` : 'Nothing to undo'; }
        if (redoBtn) { redoBtn.disabled = !cr; redoBtn.title = cr ? `Redo: ${redoLabel} (Ctrl+Y)` : 'Nothing to redo'; }
    });
    if (undoBtn) undoBtn.disabled = !canUndo();
    if (redoBtn) redoBtn.disabled = !canRedo();
}

function applyHistory(label) {
    if (!label) return;
    if (selectedId && !getSegment(selectedId)) selectedId = null;
    syncNameField();
    renderHomeDesc();
    updateHomeMarker();
    scheduleCompile(0);
}

function syncNameField() {
    const el = document.getElementById('fp-route-name');
    if (!el) return;
    el.value = getRoute().name || getCurrentMissionName() || '';
}

function setStatus(state, title) {
    const el = document.getElementById('fp-status');
    if (!el) return;
    el.dataset.state = state;
    el.title = title || { ok: 'Route calculated', warn: 'Route calculated with warnings', error: 'Route has errors', busy: 'Calculating…', empty: 'Empty route' }[state] || '';
}

function fmtDist(m) { return m >= 1000 ? `${(m / 1000).toFixed(m >= 10000 ? 0 : 1)} km` : `${Math.round(m)} m`; }
function fmtTime(s) {
    if (!s) return '--';
    const m = Math.round(s / 60);
    return m >= 60 ? `${Math.floor(m / 60)}h ${String(m % 60).padStart(2, '0')}m` : `${m} min`;
}
function fmtArea(m2) { return m2 >= 1e6 ? `${(m2 / 1e6).toFixed(2)} km²` : `${(m2 / 1e4).toFixed(1)} ha`; }

function renderRouteCard() {
    const st = compiled.stats;
    const segs = getRoute().segments;
    const errors = compiled.issues.filter(i => i.level === 'error');
    const warns = compiled.issues.filter(i => i.level === 'warn');
    setStatus(!segs.length ? 'empty' : errors.length ? 'error' : warns.length ? 'warn' : 'ok');

    const set = (id, v) => { const el = document.getElementById(id); if (el) el.textContent = v; };
    set('fp-stat-len', segs.length && st ? fmtDist(st.lengthM) : '--');
    set('fp-stat-time', segs.length && st ? fmtTime(st.durationS) : '--');
    set('fp-stat-wp', segs.length && st ? `${st.waypoints}` : '--');
    set('fp-stat-alt', segs.length && st && st.maxAgl !== null ? `${Math.round(st.maxAgl)} m` : '--');
    set('fp-seg-count', segs.length ? `(${segs.length})` : '');

    const P = getRoute().params;
    const chips = [
        `${{ agl: 'AGL', amsl: 'AMSL', rel: 'REL' }[P.altMode]} ${P.defaultAlt} m`,
        P.defaultSpeed > 0 ? `${P.defaultSpeed} m/s` : 'vehicle speed',
        P.takeoff ? `T/O ${Math.round(P.takeoffAlt)} m` : 'no take-off',
        { rtl: 'RTL', land: 'LAND', none: 'end: nothing' }[P.endAction],
        P.turnType === 'spline' ? 'spline' : null,
        P.camera?.name && P.camera.preset !== 'generic' ? P.camera.name : null,
        st?.photos ? `${st.photos} photos` : null,
    ].filter(Boolean);
    const chipsEl = document.getElementById('fp-chips');
    if (chipsEl) chipsEl.innerHTML = chips.map(c => `<span class="fp-chip">${c}</span>`).join('');

    const issuesEl = document.getElementById('fp-issues');
    if (issuesEl) {
        // One line per distinct problem; the same warning on forty waypoints reads "… (40 segments)"
        const groups = new Map();
        for (const i of compiled.issues) {
            if (i.level === 'info' && segs.length) continue;
            const key = `${i.level}|${i.text}`;
            const g = groups.get(key) || { level: i.level, text: i.text, segIds: [] };
            if (i.segId) g.segIds.push(i.segId);
            groups.set(key, g);
        }
        const list = [...groups.values()];
        issuesEl.innerHTML = list.slice(0, 8).map(g => {
            const first = g.segIds[0] ? getSegment(g.segIds[0]) : null;
            const idx = first ? segs.indexOf(first) : -1;
            const where = g.segIds.length > 1 ? `<b>${g.segIds.length} seg</b> ` : first ? `<b>${idx + 1}</b> ` : '';
            return `<div class="fp-issue is-${g.level}" ${first ? `data-seg="${g.segIds[0]}"` : ''}>${where}${escapeHtml(g.text)}</div>`;
        }).join('') + (list.length > 8 ? `<div class="fp-issue is-info">… ${list.length - 8} more</div>` : '');
    }

    const upload = document.getElementById('fp-upload');
    if (upload) upload.classList.toggle('is-blocked', errors.length > 0);
    renderHomeDesc();
}

function renderHomeDesc() {
    const el = document.getElementById('fp-home-desc');
    if (!el) return;
    const P = getRoute().params;
    if (P.home) el.textContent = `${P.home.lat.toFixed(5)}, ${P.home.lng.toFixed(5)} (custom)`;
    else if (STATE.homeLat !== null && STATE.homeLat !== undefined) el.textContent = `vehicle home ${STATE.homeLat.toFixed(5)}, ${STATE.homeLon.toFixed(5)}`;
    else if (STATE.lat || STATE.lon) el.textContent = 'vehicle position (no home yet)';
    else el.textContent = 'first segment (no vehicle)';
}

// Route settings popover ----------------------------------------------------------

function toggleRouteSettings(e) {
    e?.stopPropagation();
    const pop = document.getElementById('fp-route-params');
    if (!pop) return;
    if (pop.style.display !== 'none') { closeRouteSettings(); return; }
    openRouteSettings();
}

function openRouteSettings() {
    const pop = document.getElementById('fp-route-params');
    const body = document.getElementById('fp-route-params-body');
    if (!pop || !body) return;
    const P = getRoute().params;
    body.innerHTML = renderFields(ROUTE_PARAM_FIELDS, P, 'rp');
    bindFields(body, ROUTE_PARAM_FIELDS, P, (key) => {
        commitMission(`Route ${key}`);
        // Dependent fields (AGL tolerance, take-off altitude) appear or vanish
        if (ROUTE_PARAM_FIELDS.some(f => f.when && f.when[key] !== undefined)) openRouteSettings();
        renderRouteCard();
        scheduleCompile();
    });
    renderHomeDesc();
    pop.style.display = 'block';
    document.getElementById('fp-route-menu-list').style.display = 'none';
}

function closeRouteSettings() {
    const pop = document.getElementById('fp-route-params');
    if (pop) pop.style.display = 'none';
}

// Camera & gimbal popover ----------------------------------------------------------

function toggleCameraPopover(e) {
    e?.stopPropagation();
    const pop = document.getElementById('fp-camera');
    if (!pop) return;
    if (pop.style.display !== 'none') { closeCameraPopover(); return; }
    closeRouteSettings();
    openCameraPopover();
}

function openCameraPopover() {
    const pop = document.getElementById('fp-camera');
    const body = document.getElementById('fp-camera-body');
    if (!pop || !body) return;
    const P = getRoute().params;
    const cam = P.camera;
    body.innerHTML = `<div class="fp-fields">${renderFields(CAMERA_FIELDS, cam, 'cam')}</div>
        <div class="fp-popover-lbl" style="margin-top:8px">GIMBAL</div>
        <div class="fp-fields">${renderFields(GIMBAL_FIELDS, P, 'gim')}</div>`;
    const [camFields, gimFields] = body.querySelectorAll('.fp-fields');
    bindFields(camFields, CAMERA_FIELDS, cam, (key) => {
        if (key === 'preset') {
            const preset = CAMERA_PRESETS[cam.preset];
            if (preset && cam.preset !== 'custom') Object.assign(cam, preset);
            else cam.name = 'Custom';
            openCameraPopover();          // sensor fields follow the preset
        } else if (cam.preset !== 'custom') {
            cam.preset = 'custom';        // hand-edited values are a custom camera
            cam.name = 'Custom';
            const sel = camFields.querySelector('select[data-key=preset]');
            if (sel) sel.value = 'custom';
        }
        commitMission('Camera profile');
        renderCameraDerived();
        renderRouteCard();
        if (selectedId) renderSegmentList();
        scheduleCompile();
    });
    bindFields(gimFields, GIMBAL_FIELDS, P, () => {
        commitMission('Gimbal defaults');
        renderCameraDerived();
        scheduleCompile();
    });
    renderCameraDerived();
    pop.style.display = 'block';
    document.getElementById('fp-route-menu-list').style.display = 'none';
}

function renderCameraDerived() {
    const el = document.getElementById('fp-camera-derived');
    if (!el) return;
    const P = getRoute().params;
    const { fovH, fovV } = cameraFov(P.camera);
    const fp = cameraFootprint(Math.max(1, +P.defaultAlt || 100), P.camera);
    const gsd = P.camera.imgW > 0 ? (fp.width / P.camera.imgW) * 100 : null;
    el.innerHTML = `<span>FOV <b>${fovH.toFixed(1)}° × ${fovV.toFixed(1)}°</b></span>
        <span>at ${P.defaultAlt} m: footprint <b>${fp.width.toFixed(0)} × ${fp.height.toFixed(0)} m</b></span>
        ${gsd ? `<span>GSD <b>${gsd.toFixed(2)} cm/px</b></span>` : ''}
        <span>pitch <b>${P.gimbalPitch}°</b>${P.gimbalYaw ? ` yaw <b>${P.gimbalYaw}°</b>` : ''}</span>`;
}

function closeCameraPopover() {
    const pop = document.getElementById('fp-camera');
    if (pop) pop.style.display = 'none';
}

// Menu ----------------------------------------------------------------------------

async function onMenuAction(act) {
    const route = getRoute();
    switch (act) {
        case 'new':
            if (route.segments.length && !await confirm('Start a new route? Unsaved changes will be lost.')) return;
            replaceRoute(createRoute());
            detachCurrentMission();
            selectedId = null;
            resetMissionHistory();
            syncNameField();
            scheduleCompile(0);
            break;
        case 'clear':
            if (!route.segments.length) return;
            if (!await confirm('Remove every segment from the route?')) return;
            clearRoute();
            selectedId = null;
            commitMission('Clear route');
            syncNameField();
            scheduleCompile(0);
            break;
        case 'invert': {
            const segs = route.segments;
            const landing = segs.length && segs[segs.length - 1].type === 'landing' ? segs.pop() : null;
            segs.reverse();
            for (const s of segs) if (s.type === 'perimeter' || s.type === 'corridor') s.points.reverse();
            if (landing) segs.push(landing);
            commitMission('Invert route');
            scheduleCompile();
            break;
        }
        case 'flatten': {
            if (!compiled.items.length) return;
            if (!await confirm('Replace every segment with the calculated waypoints? Areas and corridors will no longer be editable as shapes.')) return;
            const flat = itemsToRoute(compiled.items.map(it => ({ ...it, alt: it.alt })), { name: route.name });
            flat.params = { ...route.params, takeoff: flat.params.takeoff, endAction: flat.params.endAction, altMode: 'agl' };
            replaceRoute(flat);
            selectedId = null;
            commitMission('Convert to waypoints');
            scheduleCompile(0);
            break;
        }
        case 'fit': fitRoute(); break;
        case 'import': document.getElementById('fp-import-file')?.click(); break;
        case 'export': exportWaypoints(); break;
    }
}

/** Mission Planner / QGC "QGC WPL 110" text — the lingua franca of ArduPilot plans. */
function exportWaypoints() {
    if (!compiled.items.length) { alert('Nothing to export — the route is empty.'); return; }
    const lines = ['QGC WPL 110'];
    for (const it of uploadItems()) {
        lines.push([it.seq, it.seq === 0 ? 1 : 0, it.frame, it.command, it.param1, it.param2, it.param3, it.param4,
            it.lat.toFixed(8), it.lng.toFixed(8), (+it.alt).toFixed(2), 1].join('\t'));
    }
    const blob = new Blob([lines.join('\n') + '\n'], { type: 'text/plain' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = `${(getRoute().name || 'route').replace(/[^\w.-]+/g, '_')}.waypoints`;
    document.body.appendChild(a);
    a.click();
    setTimeout(() => { URL.revokeObjectURL(a.href); a.remove(); }, 1000);
}

async function onImportFile(e) {
    const file = e.target.files?.[0];
    e.target.value = '';
    if (!file) return;
    try {
        const text = await file.text();
        let route;
        if (text.trimStart().startsWith('{')) {
            const data = JSON.parse(text);
            route = data.route?.segments ? data.route : itemsToRoute(data.items || [], { name: data.name });
        } else {
            route = itemsToRoute(parseWpl(text), { fromVehicle: true, name: file.name.replace(/\.[^.]+$/, '') });
        }
        if (getRoute().segments.length && !await confirm('Replace the current route with the imported one?')) return;
        route.name = route.name || file.name.replace(/\.[^.]+$/, '');
        replaceRoute(route);
        detachCurrentMission();
        selectedId = null;
        resetMissionHistory();
        syncNameField();
        scheduleCompile(0);
        setTimeout(fitRoute, 400);
    } catch (err) {
        alert('Import failed: ' + err.message);
    }
}

function parseWpl(text) {
    const lines = text.split(/\r?\n/);
    if (!/^QGC WPL/i.test(lines[0] || '')) throw new Error('Not a QGC WPL file');
    const items = [];
    for (const line of lines.slice(1)) {
        const f = line.trim().split(/\s+/);
        if (f.length < 12) continue;
        items.push({
            seq: +f[0], frame: +f[2], command: +f[3],
            param1: +f[4], param2: +f[5], param3: +f[6], param4: +f[7],
            lat: +f[8], lng: +f[9], alt: +f[10],
        });
    }
    return items;
}

// ── Upload / download ─────────────────────────────────────────────────────────

/** Items in the exact frame/altitude ArduPilot expects. */
function uploadItems() {
    return compiled.items.map(it => {
        if (it.roi) return { ...it, frame: 3, alt: it.altRel };
        if (it.loc) return { ...it, frame: 0, alt: Math.round(it.altMsl * 10) / 10 };   // absolute MSL
        return { ...it, frame: 3 };
    });
}

async function doUpload() {
    const btn = document.getElementById('fp-upload');
    if (STATE.connectionType === 'none') return alert('Not connected');
    if (!getRoute().segments.length) return alert('The route is empty');
    const errors = compiled.issues.filter(i => i.level === 'error');
    if (errors.length && !await confirm(`The route has ${errors.length} error${errors.length > 1 ? 's' : ''}:\n${errors.map(e => '• ' + e.text).join('\n')}\n\nUpload anyway?`)) return;
    btn.disabled = true;
    btn.textContent = 'UPLOADING…';
    try {
        await ensureRouteTerrain();
        const result = await uploadMission(uploadItems());
        btn.textContent = `DONE (${result.count})`;
    } catch (e) {
        alert('Upload failed: ' + e.message);
        btn.textContent = 'UPLOAD';
    } finally {
        setTimeout(() => { btn.textContent = 'UPLOAD'; btn.disabled = false; }, 2000);
    }
}

async function doDownload() {
    const btn = document.getElementById('fp-download');
    if (STATE.connectionType === 'none') return alert('Not connected');
    if (getRoute().segments.length && !await confirm('Replace the current route with the mission stored on the vehicle?')) return;
    btn.disabled = true;
    btn.textContent = 'READING…';
    try {
        const items = await downloadMission((done, total) => { btn.textContent = `READING ${done}/${total}`; });
        const route = itemsToRoute(items, { fromVehicle: true, name: 'From vehicle' });
        replaceRoute(route);
        detachCurrentMission();
        selectedId = null;
        resetMissionHistory();
        syncNameField();
        scheduleCompile(0);
        setTimeout(fitRoute, 400);
        btn.textContent = `READ ${items.length}`;
    } catch (e) {
        alert('Read failed: ' + e.message);
        btn.textContent = 'READ';
    } finally {
        setTimeout(() => { btn.textContent = 'READ'; btn.disabled = false; }, 2000);
    }
}

// ── Segment list & inspector ──────────────────────────────────────────────────

function initSegmentList() {
    const list = document.getElementById('fp-seg-list');
    if (!list) return;

    list.addEventListener('click', (e) => {
        const card = e.target.closest('.fp-seg');
        if (!card) return;
        const id = card.dataset.id;
        const act = e.target.closest('[data-act]')?.dataset.act;
        if (act === 'delete') { deleteSegment(id); return; }
        if (act === 'add-action') return;
        if (act === 'del-action') {
            const seg = getSegment(id);
            const i = +e.target.closest('[data-action-idx]').dataset.actionIdx;
            seg.actions.splice(i, 1);
            commitMission('Remove action');
            renderSegmentList();
            scheduleCompile();
            return;
        }
        if (e.target.closest('.fp-seg-body')) return;      // clicks inside the inspector
        if (e.target.closest('.fp-seg-head')) {
            selectSegment(selectedId === id ? null : id, { pan: e.detail === 2 });
        }
    });

    list.addEventListener('change', (e) => {
        const sel = e.target.closest('select[data-act="add-action"]');
        if (!sel) return;
        const seg = getSegment(sel.closest('.fp-seg').dataset.id);
        if (!seg || !sel.value) return;
        seg.actions.push(createAction(sel.value));
        sel.value = '';
        commitMission('Add action');
        renderSegmentList();
        scheduleCompile();
    });

    // Drag-to-reorder cards
    let dragId = null;
    list.addEventListener('dragstart', (e) => {
        const card = e.target.closest('.fp-seg');
        if (!card) return;
        dragId = card.dataset.id;
        card.classList.add('dragging');
        e.dataTransfer.effectAllowed = 'move';
    });
    list.addEventListener('dragend', () => {
        dragId = null;
        list.querySelectorAll('.fp-seg').forEach(c => c.classList.remove('dragging', 'drag-over'));
    });
    list.addEventListener('dragover', (e) => {
        const card = e.target.closest('.fp-seg');
        if (!card || !dragId) return;
        e.preventDefault();
        list.querySelectorAll('.fp-seg').forEach(c => c.classList.remove('drag-over'));
        card.classList.add('drag-over');
    });
    list.addEventListener('drop', (e) => {
        const card = e.target.closest('.fp-seg');
        if (!card || !dragId) return;
        e.preventDefault();
        const segs = getRoute().segments;
        const from = segs.findIndex(s => s.id === dragId);
        const to = segs.findIndex(s => s.id === card.dataset.id);
        if (from < 0 || to < 0 || from === to) return;
        const [moved] = segs.splice(from, 1);
        segs.splice(to, 0, moved);
        commitMission('Reorder segments');
        scheduleCompile();
    });
}

function segmentSummary(seg) {
    const P = getRoute().params;
    const st = compiled.segStats[seg.id];
    const parts = [];
    if (seg.type !== 'poi' && seg.type !== 'landing') parts.push(`${segmentAlt(seg, P)} m`);
    if (seg.params.speed > 0) parts.push(`${seg.params.speed} m/s`);
    switch (seg.type) {
        case 'circle': parts.push(`r ${seg.params.radius} m`, `${seg.params.turns}×`); break;
        case 'perimeter': parts.push(`${seg.points.length} vtx`); if (st) parts.push(fmtDist(st.lengthM)); break;
        case 'area':
            if (st) { parts.push(fmtArea(st.area || 0), `${st.lanes} lanes`); if (st.gsd) parts.push(`${st.gsd.toFixed(1)} cm/px`); if (st.photos) parts.push(`${st.photos} img`); }
            break;
        case 'corridor':
            parts.push(`${seg.params.width} m wide`);
            if (st) { parts.push(`${st.lanes} pass${st.lanes > 1 ? 'es' : ''}`); if (st.photos) parts.push(`${st.photos} img`); }
            break;
        case 'poi': parts.push(seg.params.mode === 'clear' ? 'ROI off' : `look at, ${seg.params.alt} m`); break;
        case 'landing': parts.push(seg.params.vtol ? 'VTOL' : 'land'); break;
    }
    if (seg.actions.length) parts.push(`${seg.actions.length} action${seg.actions.length > 1 ? 's' : ''}`);
    return parts.join(' · ');
}

function renderSegmentList() {
    const list = document.getElementById('fp-seg-list');
    if (!list) return;
    const segs = getRoute().segments;
    if (!segs.length) {
        list.innerHTML = `<div class="fp-empty">
            <div class="fp-empty-title">No segments yet</div>
            <div>Pick a tool on the map: a <b>waypoint</b>, a <b>circle</b>, a <b>perimeter</b> to fly around, an <b>area</b> to scan, a <b>corridor</b> to map, a <b>POI</b> for the camera, a <b>landing</b>.</div>
            <div class="fp-empty-hint">The route is calculated as you draw. Open <b>⚙ Route settings</b> for altitude mode, speed, take-off and what happens at the end.</div>
        </div>`;
        return;
    }
    const issuesBySeg = {};
    for (const i of compiled.issues) if (i.segId) (issuesBySeg[i.segId] ||= []).push(i);

    list.innerHTML = segs.map((seg, idx) => {
        const def = SEGMENT_TYPES[seg.type];
        const open = seg.id === selectedId;
        const iss = issuesBySeg[seg.id] || [];
        const worst = iss.some(i => i.level === 'error') ? 'error' : iss.length ? 'warn' : '';
        return `<div class="fp-seg${open ? ' is-open' : ''}${worst ? ' has-' + worst : ''}" data-id="${seg.id}" style="--seg-color:${def.color}">
            <div class="fp-seg-head" draggable="true">
                <span class="fp-seg-idx">${idx + 1}</span>
                <span class="fp-seg-icon">${ICON[seg.type]}</span>
                <span class="fp-seg-text">
                    <span class="fp-seg-name">${def.label}</span>
                    <span class="fp-seg-sum">${escapeHtml(segmentSummary(seg))}</span>
                </span>
                ${worst ? `<span class="fp-seg-badge is-${worst}" title="${escapeHtml(iss.map(i => i.text).join('\n'))}">${ICON.warn}</span>` : ''}
                <button class="fp-icon-btn fp-seg-del" data-act="delete" title="Delete segment">${ICON.trash}</button>
            </div>
            ${open ? renderInspector(seg, iss) : ''}
        </div>`;
    }).join('');

    if (selectedId) {
        const seg = getSegment(selectedId);
        const body = list.querySelector(`.fp-seg[data-id="${selectedId}"] .fp-seg-body`);
        if (seg && body) bindInspector(body, seg);
    }
}

function renderInspector(seg, issues) {
    const def = SEGMENT_TYPES[seg.type];
    const P = getRoute().params;
    const values = { ...seg.params };
    // Nullable altitude: show the route default as placeholder
    const fields = def.fields.map(f => f.key === 'alt' && f.nullable ? { ...f, placeholder: `${P.defaultAlt}`, title: 'Empty = route default altitude' } : f);
    let extra = '';
    if (seg.type === 'area' || seg.type === 'corridor') {
        const g = surveyGeometry(seg, segmentAlt(seg, P), P.camera);
        extra = `<div class="fp-derived">
            <span>lane spacing <b>${g.sideDistance.toFixed(1)} m</b></span>
            <span>trigger every <b>${g.triggerDistance.toFixed(1)} m</b></span>
            ${g.gsd ? `<span>GSD <b>${g.gsd.toFixed(2)} cm/px</b></span>` : ''}
            <span>footprint <b>${g.footprint.width.toFixed(0)} × ${g.footprint.height.toFixed(0)} m</b></span>
        </div>`;
    }
    const canAct = seg.type !== 'poi';
    const actions = canAct ? `<div class="fp-actions">
        <div class="fp-actions-head">
            <span>ACTIONS</span>
            <select class="cfg-select cfg-select-sm fp-add-action" data-act="add-action" title="Add an action executed at the start of this segment">
                <option value="">+ add…</option>
                ${ACTION_ORDER.map(k => `<option value="${k}">${ACTION_TYPES[k].label}</option>`).join('')}
            </select>
        </div>
        ${seg.actions.map((a, i) => {
            const ad = ACTION_TYPES[a.type] || ACTION_TYPES.raw;
            return `<div class="fp-action" data-action-idx="${i}">
                <div class="fp-action-head"><span class="fp-action-name">${ad.label}</span><span class="fp-action-sum">${escapeHtml(ad.summary(a))}</span>
                    <button class="fp-icon-btn" data-act="del-action" title="Remove action">&times;</button></div>
                ${ad.fields.length ? `<div class="fp-fields is-compact">${renderFields(ad.fields, a, `a${i}`)}</div>` : ''}
            </div>`;
        }).join('')}
    </div>` : '';
    return `<div class="fp-seg-body">
        ${issues.length ? `<div class="fp-seg-issues">${issues.map(i => `<div class="fp-issue is-${i.level}">${escapeHtml(i.text)}</div>`).join('')}</div>` : ''}
        <div class="fp-fields">${renderFields(fields, values, 'p')}</div>
        ${extra}
        ${actions}
        <div class="fp-seg-coords">${seg.points.length === 1
            ? `${seg.points[0].lat.toFixed(6)}, ${seg.points[0].lng.toFixed(6)}`
            : `${seg.points.length} points · drag the handles on the map, right-click a vertex to insert or remove`}</div>
    </div>`;
}

function bindInspector(body, seg) {
    const def = SEGMENT_TYPES[seg.type];
    bindFields(body.querySelector('.fp-fields:not(.is-compact)'), def.fields, seg.params, (key) => {
        commitMission(`Edit ${def.label.toLowerCase()}`);
        // Fields with `when` clauses may have appeared/disappeared
        if (def.fields.some(f => f.when && f.when[key] !== undefined)) renderSegmentList();
        else renderCardSummary(seg);
        scheduleCompile();
    });
    body.querySelectorAll('.fp-action').forEach(el => {
        const i = +el.dataset.actionIdx;
        const a = seg.actions[i];
        const ad = ACTION_TYPES[a.type] || ACTION_TYPES.raw;
        bindFields(el.querySelector('.fp-fields'), ad.fields, a, () => {
            commitMission('Edit action');
            el.querySelector('.fp-action-sum').textContent = ad.summary(a);
            renderCardSummary(seg);
            scheduleCompile();
        });
    });
}

function renderCardSummary(seg) {
    const el = document.querySelector(`.fp-seg[data-id="${seg.id}"] .fp-seg-sum`);
    if (el) el.textContent = segmentSummary(seg);
    const P = getRoute().params;
    const derived = document.querySelector(`.fp-seg[data-id="${seg.id}"] .fp-derived`);
    if (derived && (seg.type === 'area' || seg.type === 'corridor')) {
        const g = surveyGeometry(seg, segmentAlt(seg, P), P.camera);
        derived.innerHTML = `<span>lane spacing <b>${g.sideDistance.toFixed(1)} m</b></span><span>trigger every <b>${g.triggerDistance.toFixed(1)} m</b></span>${g.gsd ? `<span>GSD <b>${g.gsd.toFixed(2)} cm/px</b></span>` : ''}<span>footprint <b>${g.footprint.width.toFixed(0)} × ${g.footprint.height.toFixed(0)} m</b></span>`;
    }
}

// Generic schema-driven fields ----------------------------------------------------

function renderFields(fields, values, prefix) {
    return fields.filter(f => fieldVisible(f, values)).map(f => {
        const id = `fp-${prefix}-${f.key}`;
        const v = values[f.key];
        let control;
        if (f.type === 'select') {
            control = `<select class="cfg-select cfg-select-sm" id="${id}" data-key="${f.key}">
                ${f.options.map(([val, lbl]) => `<option value="${val}"${String(v) === String(val) ? ' selected' : ''}>${lbl}</option>`).join('')}
            </select>`;
        } else if (f.type === 'check') {
            control = `<label class="fp-switch"><input type="checkbox" id="${id}" data-key="${f.key}"${v ? ' checked' : ''}><span></span></label>`;
        } else {
            const empty = v === null || v === undefined || v === '' || (f.zeroLabel && +v === 0);
            const ph = f.placeholder || f.zeroLabel || '';
            control = `<span class="fp-num"><input type="number" class="gcs-input" id="${id}" data-key="${f.key}" value="${empty ? '' : v}" placeholder="${ph}"${f.title ? ` title="${f.title}"` : ''}
                ${f.min !== undefined ? `min="${f.min}"` : ''} ${f.max !== undefined ? `max="${f.max}"` : ''} step="${f.step ?? 1}">${f.unit ? `<span class="fp-unit">${f.unit}</span>` : ''}</span>`;
        }
        return `<div class="fp-field${f.type === 'check' ? ' is-check' : ''}"><label for="${id}">${f.label}</label>${control}</div>`;
    }).join('');
}

function bindFields(root, fields, target, onChange) {
    if (!root) return;
    root.querySelectorAll('[data-key]').forEach(el => {
        const f = fields.find(x => x.key === el.dataset.key);
        if (!f) return;
        const handler = () => {
            let v;
            if (f.type === 'check') v = el.checked;
            else if (f.type === 'select') v = el.value;
            else {
                if (el.value === '') v = f.nullable ? null : 0;
                else {
                    v = parseFloat(el.value);
                    if (!Number.isFinite(v)) return;
                    if (f.min !== undefined) v = Math.max(f.min, v);
                    if (f.max !== undefined) v = Math.min(f.max, v);
                }
            }
            if (target[f.key] === v) return;
            target[f.key] = v;
            if (onChange) onChange(f.key, v);
        };
        el.addEventListener('change', handler);
        if (f.type === 'number') {
            el.addEventListener('keydown', (e) => { if (e.key === 'Enter') { e.preventDefault(); el.blur(); } });
        }
    });
}

// ── Map rendering ─────────────────────────────────────────────────────────────

function renderMap() {
    if (!map) return;
    for (const name of ['segments', 'path', 'arrows', 'points', 'handles']) layers[name].clearLayers();
    const segs = getRoute().segments;
    const sel = selectedId;
    renderCameraLayer();

    // Segment figures
    figures.clear();
    segs.forEach((seg, idx) => {
        const def = SEGMENT_TYPES[seg.type];
        const isSel = seg.id === sel;
        const lls = seg.points.map(p => [p.lat, p.lng]);
        const base = { color: def.color, weight: isSel ? 2.5 : 1.5, opacity: isSel ? 1 : 0.75 };
        const onSel = (e) => { L.DomEvent.stop(e); if (tool === 'select') selectSegment(seg.id, { scroll: true }); };
        const rec = { fig: null, axis: null, points: [], label: null };
        figures.set(seg.id, rec);
        switch (seg.type) {
            case 'area':
                rec.fig = L.polygon(lls, { ...base, dashArray: '6 4', fillColor: def.color, fillOpacity: isSel ? 0.16 : 0.08 });
                break;
            case 'perimeter':
                rec.fig = L.polygon(lls, { ...base, fillOpacity: 0.02, fillColor: def.color });
                break;
            case 'corridor': {
                const outline = corridorOutline(seg.points, +seg.params.width || 60);
                rec.fig = L.polygon(outline.map(p => [p.lat, p.lng]), { ...base, dashArray: '6 4', fillColor: def.color, fillOpacity: isSel ? 0.14 : 0.07 });
                rec.axis = L.polyline(lls, { color: def.color, weight: 1, opacity: 0.5, dashArray: '2 4', interactive: false }).addTo(layers.segments);
                break;
            }
            case 'circle':
                rec.fig = L.circle(lls[0], { radius: +seg.params.radius || 50, ...base, fillOpacity: isSel ? 0.08 : 0.03, fillColor: def.color });
                break;
        }
        if (rec.fig) {
            rec.fig.on('click', onSel);
            rec.fig.on('contextmenu', (e) => showContextMenu(e, { segId: seg.id }));
            rec.fig.addTo(layers.segments);
        }

        // Base points: the operator's own points, larger than calculated ones
        seg.points.forEach((p, pi) => {
            const single = def.points === 1;
            const r = single ? 7 : 4;
            const isPoi = seg.type === 'poi', isLand = seg.type === 'landing';
            const m = L.circleMarker([p.lat, p.lng], {
                radius: isSel ? r + 1 : r, color: isSel ? '#fff' : 'rgba(255,255,255,0.85)', weight: isSel ? 2 : 1.5,
                fillColor: def.color, fillOpacity: single ? 1 : 0.9, pane: isSel ? 'fpHandles' : 'markerPane',
                dashArray: isPoi ? '2 2' : null,
            });
            m.on('click', onSel);
            m.on('contextmenu', (e) => showContextMenu(e, { segId: seg.id, pointIdx: pi }));
            attachConeHover(m, p);
            if (isSel) makeDraggable(m, (ll) => { p.lat = ll.lat; p.lng = ll.lng; updateFigure(seg); }, () => commitAndCompile('Move point'));
            m.addTo(isSel ? layers.handles : layers.points);
            rec.points.push(m);
            if (isLand) {
                rec.glyph = L.marker([p.lat, p.lng], { icon: L.divIcon({ html: `<div class="fp-glyph" style="color:${def.color}">${ICON.landing}</div>`, className: 'vehicle-map-marker', iconSize: [18, 18], iconAnchor: [9, 9] }), interactive: false, pane: 'fpLabels' }).addTo(layers.points);
            }
        });

        // Index label near the first point. A plan read back from the vehicle is
        // one waypoint per segment, so past a few dozen only every 5th gets a label.
        const lp = labelPoint(seg);
        const sparse = segs.length > 40 && seg.type === 'waypoint' && !isSel && (idx + 1) % 5 !== 0;
        if (lp && !sparse) {
            const chip = seg.type === 'waypoint' ? '' : `<small>${def.short}</small>`;
            rec.label = L.marker([lp.lat, lp.lng], {
                icon: L.divIcon({ html: `<span class="fp-seg-label${isSel ? ' is-sel' : ''}" style="border-color:${def.color}">${idx + 1}${chip}</span>`, className: 'vehicle-map-marker', iconSize: [0, 0], iconAnchor: [-9, 9] }),
                interactive: false, pane: 'fpLabels',
            }).addTo(layers.points);
        }

        if (isSel) renderHandles(seg);
    });

    // Calculated flight path: home → …, ROI excluded
    const nav = compiled.navPath;
    const lls = nav.map(p => [p.lat, p.lng]);
    if (lls.length >= 2) {
        L.polyline(lls, { color: 'rgba(0,0,0,0.55)', weight: 5, opacity: 0.6, interactive: false }).addTo(layers.path);
        L.polyline(lls, { color: '#44ff44', weight: 2, opacity: 0.95, interactive: false }).addTo(layers.path);
        // Highlight the selected segment's part of the path
        if (sel) {
            const part = [];
            nav.forEach((p, i) => { if (p.segId === sel) { if (i > 0 && !part.length) part.push(lls[i - 1]); part.push(lls[i]); } });
            if (part.length >= 2) L.polyline(part, { color: '#fff', weight: 3.5, opacity: 0.9, interactive: false }).addTo(layers.path);
        }
    }
    // Calculated waypoints
    if (showCalcPoints) {
        for (const p of nav) {
            if (p.isHome) continue;
            const seg = p.segId ? getSegment(p.segId) : null;
            const def = seg ? SEGMENT_TYPES[seg.type] : null;
            const single = def?.points === 1;
            if (single && !p.derived) continue;          // already drawn as the base point
            const c = conesByKey.get(posKey(p));
            const m = L.circleMarker([p.lat, p.lng], {
                radius: p.derived ? 2 : 3, color: p.derived ? 'rgba(255,255,255,0.7)' : '#fff', weight: 1,
                fillColor: p.derived ? '#44ff44' : (def?.color || '#44ff44'), fillOpacity: 1, interactive: !!c, bubblingMouseEvents: false,
            });
            if (c) { m.on('mouseover', () => showCone(c)); m.on('mouseout', hideFootprint); }
            m.addTo(layers.points);
        }
    }
    redrawArrows();
    updateHomeMarker();
}

/**
 * Camera preview: a small red dot on the route at every photo, a pink dot where
 * the camera is aimed at a POI. The ground footprint / view wedge is drawn only
 * while the mouse is over its dot, so a dense survey stays readable.
 */
const conesByKey = new Map();     // "lat,lng" → cone, so base-point markers can show a wedge on hover
const posKey = p => `${p.lat.toFixed(6)},${p.lng.toFixed(6)}`;

function renderCameraLayer() {
    if (!map || !layers.camera) return;
    layers.camera.clearLayers();
    layers.cameraHover.clearLayers();
    conesByKey.clear();
    if (!showCamera) return;
    const { photos = [], cones = [] } = compiled.camera || {};
    for (const c of cones) conesByKey.set(posKey(c.from), c);

    // Full size when zoomed in; shrink as neighbouring photos get closer than a
    // dot apart on screen, so a lane never turns into a solid red bar
    let radius = 4;
    if (photos.length > 1) {
        const a = map.latLngToLayerPoint([photos[0].lat, photos[0].lng]);
        const b = map.latLngToLayerPoint([photos[1].lat, photos[1].lng]);
        const px = a.distanceTo(b);
        radius = Math.max(1.5, Math.min(4, px / 3));
    }
    for (const ph of photos) {
        const dot = L.circleMarker([ph.lat, ph.lng], {
            renderer: cameraRenderer, radius: ph.shot ? radius + 1.5 : radius, color: '#ff3030', weight: 1,
            fillColor: '#ff3030', fillOpacity: 1, opacity: 0.9, bubblingMouseEvents: false,
        });
        dot.on('mouseover', () => showFootprint(ph));
        dot.on('mouseout', hideFootprint);
        dot.addTo(layers.camera);
    }
    // Loiter view points have no base marker of their own: give them a dot
    for (const c of cones) {
        if (c.onBasePoint) continue;
        const dot = L.circleMarker([c.from.lat, c.from.lng], {
            renderer: cameraRenderer, radius: 4.5, color: '#ff66aa', weight: 1, fillColor: '#ff66aa', fillOpacity: 1, bubblingMouseEvents: false,
        });
        dot.on('mouseover', () => showCone(c));
        dot.on('mouseout', hideFootprint);
        dot.addTo(layers.camera);
    }
}

function showFootprint(ph) {
    layers.cameraHover.clearLayers();
    L.polygon(ph.poly.map(p => [p.lat, p.lng]), {
        pane: 'fpCameraHover', interactive: false, color: '#ff4040', weight: 1.5, fillColor: '#ff3030', fillOpacity: 0.18,
    }).addTo(layers.cameraHover);
    L.circleMarker([ph.lat, ph.lng], { pane: 'fpCameraHover', interactive: false, radius: 4, color: '#fff', weight: 1.5, fillColor: '#ff3030', fillOpacity: 1 }).addTo(layers.cameraHover);
}

function showCone(c) {
    layers.cameraHover.clearLayers();
    const far = [c.poly[3], c.poly[2]];
    L.polygon([[c.from.lat, c.from.lng], ...far.map(p => [p.lat, p.lng])], {
        pane: 'fpCameraHover', interactive: false, color: '#ff80b0', weight: 1, dashArray: '3 3', fillColor: '#ff66aa', fillOpacity: 0.12,
    }).addTo(layers.cameraHover);
    L.polygon(c.poly.map(p => [p.lat, p.lng]), {
        pane: 'fpCameraHover', interactive: false, color: '#ff4040', weight: 1.5, fillColor: '#ff3030', fillOpacity: 0.18,
    }).addTo(layers.cameraHover);
    L.polyline([[c.from.lat, c.from.lng], [c.target.lat, c.target.lng]], {
        pane: 'fpCameraHover', interactive: false, color: 'rgba(255, 102, 170, 0.7)', weight: 1, dashArray: '1 4',
    }).addTo(layers.cameraHover);
}

function hideFootprint() {
    layers.cameraHover.clearLayers();
}

/** Hover handlers for a base-point marker that is also a POI viewpoint. */
function attachConeHover(marker, p) {
    const c = conesByKey.get(posKey(p));
    if (!c) return;
    marker.on('mouseover', () => showCone(c));
    marker.on('mouseout', hideFootprint);
}

/** Direction arrows along the calculated path, spaced in screen pixels so zoom never crowds them. */
function redrawArrows() {
    if (!map) return;
    layers.arrows.clearLayers();
    const nav = compiled.navPath;
    if (nav.length < 2) return;
    const MIN_PX = 70;
    let carry = 0;
    for (let i = 1; i < nav.length; i++) {
        const a = map.latLngToLayerPoint([nav[i - 1].lat, nav[i - 1].lng]);
        const b = map.latLngToLayerPoint([nav[i].lat, nav[i].lng]);
        const len = a.distanceTo(b);
        if (len < 4) continue;
        let pos = MIN_PX / 2 - carry;
        const brg = bearing(nav[i - 1], nav[i]);
        const sel = selectedId && nav[i].segId === selectedId;
        while (pos < len) {
            const t = pos / len;
            const ll = map.layerPointToLatLng(L.point(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t));
            L.marker(ll, {
                icon: L.divIcon({ html: `<div class="fp-arrow${sel ? ' is-sel' : ''}" style="transform:rotate(${brg}deg)"></div>`, className: 'vehicle-map-marker', iconSize: [12, 12], iconAnchor: [6, 6] }),
                interactive: false, pane: 'fpLabels',
            }).addTo(layers.arrows);
            pos += MIN_PX;
        }
        carry = len - (pos - MIN_PX);
    }
}

/** Vertex mid-points (insert), centre (move all) and radius (circle) handles of the selected segment. */
function renderHandles(seg) {
    const def = SEGMENT_TYPES[seg.type];
    const pts = seg.points;

    if (def.points === 'poly' || def.points === 'line') {
        const n = pts.length;
        const edges = def.points === 'poly' ? n : n - 1;
        for (let i = 0; i < edges; i++) {
            const a = pts[i], b = pts[(i + 1) % n];
            const mid = { lat: (a.lat + b.lat) / 2, lng: (a.lng + b.lng) / 2 };
            const m = L.circleMarker([mid.lat, mid.lng], { radius: 4, color: '#fff', weight: 1, fillColor: def.color, fillOpacity: 0.45, pane: 'fpHandles', className: 'fp-mid-handle' });
            m.on('mousedown', (e) => {
                // Insert a vertex and immediately drag it
                L.DomEvent.stop(e);
                const p = { lat: mid.lat, lng: mid.lng };
                pts.splice(i + 1, 0, p);
                renderMap();
                const inserted = findHandleFor(seg.id, i + 1);
                if (inserted) startDrag(inserted, e.originalEvent, (ll) => { p.lat = ll.lat; p.lng = ll.lng; updateFigure(seg); }, () => commitAndCompile('Insert vertex'));
            });
            m.addTo(layers.handles);
        }
        // Centre handle moves the whole figure
        const c = centroid(pts);
        const cm = L.marker([c.lat, c.lng], {
            icon: L.divIcon({ html: '<div class="fp-move-handle"></div>', className: 'vehicle-map-marker', iconSize: [18, 18], iconAnchor: [9, 9] }),
            draggable: true, pane: 'fpHandles', title: 'Drag to move the whole segment',
        });
        let start = null, orig = null;
        cm.on('dragstart', () => { start = cm.getLatLng(); orig = pts.map(p => ({ ...p })); });
        cm.on('drag', () => {
            const now = cm.getLatLng();
            const dLat = now.lat - start.lat, dLng = now.lng - start.lng;
            pts.forEach((p, i) => { p.lat = orig[i].lat + dLat; p.lng = orig[i].lng + dLng; });
            updateFigure(seg);
        });
        cm.on('dragend', () => commitAndCompile('Move segment'));
        cm.addTo(layers.handles);
    }

    if (seg.type === 'circle') {
        const c = pts[0];
        const r = +seg.params.radius || 50;
        const f = localFrame(c);
        const rim = f.toLL({ x: r, y: 0 });
        const rm = L.circleMarker([rim.lat, rim.lng], { radius: 5, color: '#fff', weight: 1.5, fillColor: def.color, fillOpacity: 1, pane: 'fpHandles' });
        makeDraggable(rm, (ll) => {
            seg.params.radius = Math.max(5, Math.round(haversine(c, ll)));
            updateFigure(seg);
        }, () => { commitAndCompile('Change radius'); renderSegmentList(); });
        rm.addTo(layers.handles);
    }
}

function findHandleFor(segId, pointIdx) {
    const seg = getSegment(segId);
    if (!seg) return null;
    const p = seg.points[pointIdx];
    let found = null;
    layers.handles.eachLayer(l => {
        if (found || !(l instanceof L.CircleMarker)) return;
        const ll = l.getLatLng();
        if (Math.abs(ll.lat - p.lat) < 1e-9 && Math.abs(ll.lng - p.lng) < 1e-9 && l.options.radius >= 5) found = l;
    });
    return found;
}

function labelPoint(seg) {
    return seg.points[0];
}

/** Move a segment's drawn figure to follow its points — used while dragging, before the recompile. */
function updateFigure(seg) {
    const rec = figures.get(seg.id);
    if (!rec) return;
    const lls = seg.points.map(p => [p.lat, p.lng]);
    if (rec.fig) {
        if (seg.type === 'circle') { rec.fig.setLatLng(lls[0]); rec.fig.setRadius(+seg.params.radius || 50); }
        else if (seg.type === 'corridor') rec.fig.setLatLngs(corridorOutline(seg.points, +seg.params.width || 60).map(p => [p.lat, p.lng]));
        else rec.fig.setLatLngs(lls);
    }
    if (rec.axis) rec.axis.setLatLngs(lls);
    rec.points.forEach((m, i) => { if (lls[i]) m.setLatLng(lls[i]); });
    if (rec.glyph) rec.glyph.setLatLng(lls[0]);
    const lp = labelPoint(seg);
    if (rec.label && lp) rec.label.setLatLng([lp.lat, lp.lng]);
}

function commitAndCompile(label) {
    commitMission(label);
    renderCardSummary(getSegment(selectedId) || {});
    scheduleCompile();
}

/** Mouse-drag for circle markers (Leaflet only makes L.Marker draggable). */
function makeDraggable(marker, onMove, onEnd) {
    marker.on('mousedown', (e) => {
        L.DomEvent.stop(e);
        startDrag(marker, e.originalEvent, onMove, onEnd);
    });
}

function startDrag(marker, originalEvent, onMove, onEnd) {
    map.dragging.disable();
    const mapEl = map.getContainer();
    mapEl.classList.add('is-dragging-handle');
    const move = (ev) => {
        const ll = map.mouseEventToLatLng(ev);
        marker.setLatLng(ll);
        onMove(ll);
    };
    const up = () => {
        window.removeEventListener('mousemove', move);
        window.removeEventListener('mouseup', up);
        map.dragging.enable();
        mapEl.classList.remove('is-dragging-handle');
        onEnd(marker.getLatLng());
    };
    window.addEventListener('mousemove', move);
    window.addEventListener('mouseup', up);
}

// ── Context menu ──────────────────────────────────────────────────────────────

function onMapContextMenu(e) {
    L.DomEvent.stop(e);
    if (draft) { showContextMenu(e, { drafting: true }); return; }
    showContextMenu(e, {});
}

function showContextMenu(e, ctx) {
    const el = document.getElementById('fp-ctx');
    if (!el) return;
    if (e.originalEvent) L.DomEvent.stop(e);
    const ll = e.latlng;
    const items = [];
    if (ctx.drafting) {
        items.push(['finish', 'Finish drawing'], ['cancel', 'Cancel drawing']);
    } else if (ctx.segId) {
        const seg = getSegment(ctx.segId);
        const def = SEGMENT_TYPES[seg.type];
        items.push(['select', `Select ${def.label.toLowerCase()}`]);
        if (ctx.pointIdx !== undefined && (def.points === 'poly' || def.points === 'line')) {
            const min = def.points === 'poly' ? 3 : 2;
            items.push(['insert', 'Insert vertex after']);
            if (seg.points.length > min) items.push(['delvtx', 'Delete vertex']);
        }
        items.push(['delete', 'Delete segment']);
    } else {
        items.push(['wp', 'Add waypoint here'], ['poi', 'Add POI here'], ['home', 'Take-off point here']);
        if (selectedId) items.push(['deselect', 'Deselect']);
    }
    el.innerHTML = items.map(([a, l]) => `<button data-act="${a}"${a === 'delete' || a === 'delvtx' ? ' class="is-danger"' : ''}>${l}</button>`).join('');
    const wrap = document.querySelector('.fp-map-wrap').getBoundingClientRect();
    const oe = e.originalEvent;
    el.style.left = `${oe.clientX - wrap.left}px`;
    el.style.top = `${oe.clientY - wrap.top}px`;
    el.style.display = 'block';
    el.onclick = (ev) => {
        const act = ev.target.closest('button')?.dataset.act;
        hideContextMenu();
        if (!act) return;
        const seg = ctx.segId ? getSegment(ctx.segId) : null;
        switch (act) {
            case 'finish': finishDraft(); break;
            case 'cancel': cancelDraft(); setTool('select'); break;
            case 'select': selectSegment(ctx.segId, { scroll: true }); break;
            case 'delete': deleteSegment(ctx.segId); break;
            case 'insert': {
                const i = ctx.pointIdx, n = seg.points.length;
                const a = seg.points[i], b = seg.points[(i + 1) % n];
                seg.points.splice(i + 1, 0, { lat: (a.lat + b.lat) / 2, lng: (a.lng + b.lng) / 2 });
                commitMission('Insert vertex'); selectSegment(seg.id); scheduleCompile();
                break;
            }
            case 'delvtx':
                seg.points.splice(ctx.pointIdx, 1);
                commitMission('Delete vertex'); selectSegment(seg.id); scheduleCompile();
                break;
            case 'wp': addSegment(createSegment('waypoint', [{ lat: ll.lat, lng: ll.lng }]), 'Add waypoint'); break;
            case 'poi': addSegment(createSegment('poi', [{ lat: ll.lat, lng: ll.lng }]), 'Add POI'); break;
            case 'home':
                getRoute().params.home = { lat: ll.lat, lng: ll.lng };
                commitMission('Set take-off point'); renderHomeDesc(); updateHomeMarker(); scheduleCompile();
                break;
            case 'deselect': selectSegment(null); break;
        }
    };
}

function hideContextMenu() {
    const el = document.getElementById('fp-ctx');
    if (el) el.style.display = 'none';
}

// ── Keyboard ──────────────────────────────────────────────────────────────────

function initKeyboard() {
    document.addEventListener('keydown', (e) => {
        if (!visible || document.querySelector('.params-page.open')) return;
        const tag = (document.activeElement?.tagName || '').toLowerCase();
        const typing = tag === 'input' || tag === 'select' || tag === 'textarea';
        if (e.key === 'Escape') {
            if (draft) { cancelDraft(); setTool('select'); }
            else if (tool !== 'select') setTool('select');
            else if (selectedId) selectSegment(null);
            hideContextMenu();
            closeRouteSettings();
            closeCameraPopover();
            return;
        }
        if (typing) return;
        if (e.key === 'Enter' && draft) { e.preventDefault(); finishDraft(); return; }
        if ((e.key === 'Delete' || e.key === 'Backspace') && selectedId) { e.preventDefault(); deleteSegment(selectedId); return; }
        if (e.ctrlKey || e.metaKey) {
            const k = e.key.toLowerCase();
            if (k === 'z' && !e.shiftKey) { e.preventDefault(); applyHistory(undoMission()); }
            else if (k === 'y' || (k === 'z' && e.shiftKey)) { e.preventDefault(); applyHistory(redoMission()); }
            return;
        }
        if (e.altKey) return;
        const k = e.key.toUpperCase();
        if (k === 'V') setTool('select');
        else for (const tname of SEGMENT_ORDER) if (SEGMENT_TYPES[tname].hotkey === k) { setTool(tname); break; }
    });
    document.addEventListener('click', (e) => {
        if (!e.target.closest('#fp-ctx')) hideContextMenu();
    });
}

// ── Elevation profile ─────────────────────────────────────────────────────────

function initProfile() {
    const canvas = document.getElementById('mission-3d-canvas');
    if (!canvas || profile.bound) return;
    profile.bound = true;
    const PAD_L = 52, PAD_R = 16;

    canvas.addEventListener('wheel', (e) => {
        e.preventDefault();
        const plotW = canvas.width - PAD_L - PAD_R;
        if (e.ctrlKey || e.metaKey) {
            const factor = e.deltaY < 0 ? 1.2 : 1 / 1.2;
            const old = profile.zoom;
            profile.zoom = Math.max(1, Math.min(40, profile.zoom * factor));
            const mx = e.clientX - canvas.getBoundingClientRect().left - PAD_L;
            const ratio = (mx + profile.scroll) / (plotW * old);
            profile.scroll = ratio * plotW * profile.zoom - mx;
        } else {
            profile.scroll += e.deltaY + e.deltaX;
        }
        profile.scroll = Math.max(0, Math.min(plotW * profile.zoom - plotW, profile.scroll));
        renderProfile();
    }, { passive: false });
    canvas.addEventListener('mousedown', (e) => { profile.dragging = true; profile.dragX = e.clientX; profile.scrollStart = profile.scroll; profile.moved = false; });
    window.addEventListener('mousemove', (e) => {
        if (profile.dragging) {
            const plotW = canvas.width - PAD_L - PAD_R;
            const dx = profile.dragX - e.clientX;
            if (Math.abs(dx) > 2) profile.moved = true;
            profile.scroll = Math.max(0, Math.min(plotW * profile.zoom - plotW, profile.scrollStart + dx));
            renderProfile();
        }
    });
    window.addEventListener('mouseup', () => { profile.dragging = false; });
    canvas.addEventListener('mousemove', (e) => {
        const rect = canvas.getBoundingClientRect();
        profile.hoverX = e.clientX - rect.left;
        profile.hoverY = e.clientY - rect.top;
        renderProfile();
    });
    canvas.addEventListener('mouseleave', () => {
        profile.hoverX = null;
        if (hoverMarker) { hoverMarker.remove(); hoverMarker = null; }
        const tip = document.getElementById('fp-profile-tip');
        if (tip) tip.style.display = 'none';
        renderProfile();
    });
    canvas.addEventListener('click', () => {
        if (profile.moved) return;
        if (profile.hoverSeg) selectSegment(profile.hoverSeg, { scroll: true });
    });
    window.addEventListener('resize', () => { if (visible) renderProfile(); });
}

function niceStep(range, count) {
    const raw = range / Math.max(1, count);
    const mag = Math.pow(10, Math.floor(Math.log10(raw || 1)));
    const norm = raw / mag;
    return (norm < 1.5 ? 1 : norm < 3.5 ? 2 : norm < 7.5 ? 5 : 10) * mag;
}

function renderProfile() {
    const canvas = document.getElementById('mission-3d-canvas');
    if (!canvas || !visible) return;
    const panel = canvas.parentElement;
    canvas.width = panel.clientWidth || 600;
    canvas.height = panel.clientHeight || 170;
    const ctx = canvas.getContext('2d');
    const w = canvas.width, h = canvas.height;
    const pad = { top: 34, bottom: 24, left: 52, right: 16 };

    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = 'rgba(5, 12, 20, 0.95)';
    ctx.fillRect(0, 0, w, h);

    const nav = compiled.navPath;
    const P = getRoute().params;
    const mono = '"Roboto Mono", monospace';
    if (nav.length < 2) {
        ctx.fillStyle = '#6e7f8d';
        ctx.font = `11px ${mono}`;
        ctx.textAlign = 'center';
        ctx.fillText('Elevation profile — draw at least one segment', w / 2, h / 2);
        return;
    }

    const totalDist = nav[nav.length - 1].dist || 1;
    // Dense terrain samples along the calculated path
    const terr = [];
    const SAMPLES = Math.min(1200, Math.max(200, Math.round(totalDist / 15)));
    for (let i = 1; i < nav.length; i++) {
        const a = nav[i - 1], b = nav[i];
        const segLen = b.dist - a.dist;
        const n = Math.max(1, Math.round(SAMPLES * segLen / totalDist));
        for (let s = 0; s < n; s++) {
            const t = s / n;
            const lat = a.lat + (b.lat - a.lat) * t, lng = a.lng + (b.lng - a.lng) * t;
            const g = getTerrainElevationFromHGT(lat, lng);
            // Climb-out and final descent are vertical manoeuvres, not low flying
            terr.push({ dist: a.dist + segLen * t, elev: g, route: a.altMsl + (b.altMsl - a.altMsl) * t, vertical: a.isHome || b.landing });
        }
    }
    const last = nav[nav.length - 1];
    terr.push({ dist: totalDist, elev: last.terrain, route: last.altMsl });

    const elevs = terr.map(p => p.elev).filter(v => v !== null);
    const routeAlts = nav.map(p => p.altMsl);
    const lo = Math.min(...elevs, ...routeAlts) - 15;
    const hi = Math.max(...elevs, ...routeAlts) + 25;
    const range = (hi - lo) || 100;
    const plotW = w - pad.left - pad.right, plotH = h - pad.top - pad.bottom;
    const totalPlotW = plotW * profile.zoom;
    const X = d => pad.left + (d / totalDist) * totalPlotW - profile.scroll;
    const Y = a => pad.top + plotH - ((a - lo) / range) * plotH;
    const distAt = x => ((x - pad.left + profile.scroll) / totalPlotW) * totalDist;

    ctx.save();
    ctx.beginPath(); ctx.rect(pad.left, 0, plotW, h); ctx.clip();

    // Grid
    const altStep = niceStep(range, 4);
    ctx.strokeStyle = 'rgba(255,255,255,0.06)'; ctx.lineWidth = 1;
    for (let a = Math.ceil(lo / altStep) * altStep; a <= hi; a += altStep) {
        const y = Math.round(Y(a)) + 0.5;
        ctx.beginPath(); ctx.moveTo(pad.left, y); ctx.lineTo(pad.left + plotW, y); ctx.stroke();
    }
    const distStep = niceStep(totalDist / profile.zoom, Math.max(4, Math.floor(plotW / 90)));
    ctx.strokeStyle = 'rgba(255,255,255,0.04)';
    for (let d = 0; d <= totalDist; d += distStep) {
        const x = Math.round(X(d)) + 0.5;
        if (x < pad.left || x > pad.left + plotW) continue;
        ctx.beginPath(); ctx.moveTo(x, pad.top); ctx.lineTo(x, pad.top + plotH); ctx.stroke();
    }

    // Terrain
    const tPts = terr.filter(p => p.elev !== null);
    if (tPts.length) {
        ctx.beginPath();
        ctx.moveTo(X(tPts[0].dist), Y(lo));
        for (const p of tPts) ctx.lineTo(X(p.dist), Y(p.elev));
        ctx.lineTo(X(tPts[tPts.length - 1].dist), Y(lo));
        ctx.closePath();
        const g = ctx.createLinearGradient(0, Y(hi), 0, Y(lo));
        g.addColorStop(0, 'rgba(96, 128, 72, 0.55)');
        g.addColorStop(1, 'rgba(40, 56, 32, 0.25)');
        ctx.fillStyle = g; ctx.fill();
        ctx.beginPath();
        tPts.forEach((p, i) => i ? ctx.lineTo(X(p.dist), Y(p.elev)) : ctx.moveTo(X(p.dist), Y(p.elev)));
        ctx.strokeStyle = 'rgba(140, 190, 110, 0.7)'; ctx.lineWidth = 1.2; ctx.stroke();

        // Minimum-clearance floor and the stretches that dip under it
        const minC = +P.minClearance || 0;
        if (minC > 0) {
            ctx.beginPath();
            tPts.forEach((p, i) => i ? ctx.lineTo(X(p.dist), Y(p.elev + minC)) : ctx.moveTo(X(p.dist), Y(p.elev + minC)));
            ctx.strokeStyle = 'rgba(255, 170, 0, 0.35)'; ctx.setLineDash([3, 4]); ctx.lineWidth = 1; ctx.stroke(); ctx.setLineDash([]);
        }
        ctx.fillStyle = 'rgba(255, 60, 60, 0.28)';
        let run = null;
        const flush = () => { if (run) { ctx.fillRect(X(run.a), pad.top, Math.max(1, X(run.b) - X(run.a)), plotH); run = null; } };
        for (const p of tPts) {
            const bad = !p.vertical && p.route - p.elev < minC && p.dist > 0 && p.dist < totalDist;
            if (bad) { if (!run) run = { a: p.dist, b: p.dist }; else run.b = p.dist; }
            else flush();
        }
        flush();
    }

    // Waypoint verticals, base points only
    ctx.strokeStyle = 'rgba(255,255,255,0.10)'; ctx.lineWidth = 1;
    for (const p of nav) {
        if (p.derived || p.isHome) continue;
        const x = Math.round(X(p.dist)) + 0.5;
        ctx.beginPath(); ctx.moveTo(x, pad.top); ctx.lineTo(x, pad.top + plotH); ctx.stroke();
    }

    // Route line, selected segment brighter
    ctx.beginPath();
    nav.forEach((p, i) => i ? ctx.lineTo(X(p.dist), Y(p.altMsl)) : ctx.moveTo(X(p.dist), Y(p.altMsl)));
    ctx.strokeStyle = '#44ff44'; ctx.lineWidth = 2; ctx.stroke();
    if (selectedId) {
        ctx.beginPath();
        let started = false;
        nav.forEach((p, i) => {
            if (p.segId === selectedId) {
                if (!started && i > 0) { ctx.moveTo(X(nav[i - 1].dist), Y(nav[i - 1].altMsl)); started = true; }
                if (!started) { ctx.moveTo(X(p.dist), Y(p.altMsl)); started = true; } else ctx.lineTo(X(p.dist), Y(p.altMsl));
            } else if (started) { started = false; }
        });
        ctx.strokeStyle = '#ffffff'; ctx.lineWidth = 3; ctx.stroke();
    }

    // Points
    const segs = getRoute().segments;
    for (const p of nav) {
        const x = X(p.dist), y = Y(p.altMsl);
        if (x < pad.left - 10 || x > pad.left + plotW + 10) continue;
        const seg = p.segId ? getSegment(p.segId) : null;
        const color = p.isHome ? '#ff8800' : (seg ? SEGMENT_TYPES[seg.type].color : '#44ff44');
        ctx.beginPath(); ctx.arc(x, y, p.derived ? 2 : 4, 0, Math.PI * 2);
        ctx.fillStyle = color; ctx.fill();
        if (!p.derived) { ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.2; ctx.stroke(); }
    }
    // Segment labels once, at the first point of each
    ctx.font = `bold 9px ${mono}`; ctx.textAlign = 'center';
    const seen = new Set();
    let lastLabelX = -Infinity;
    for (const p of nav) {
        if (!p.segId || p.derived || seen.has(p.segId)) continue;
        seen.add(p.segId);
        const idx = segs.findIndex(s => s.id === p.segId);
        const x = X(p.dist);
        if (x < pad.left || x > pad.left + plotW) continue;
        if (x - lastLabelX < 16 && p.segId !== selectedId) continue;   // no overlapping numbers
        lastLabelX = x;
        ctx.fillStyle = p.segId === selectedId ? '#fff' : '#cfd8dd';
        ctx.fillText(`${idx + 1}`, x, Y(p.altMsl) - 8);
    }
    if (nav[0].isHome) { ctx.fillStyle = '#ff8800'; ctx.fillText('H', X(0), Y(nav[0].altMsl) - 8); }

    // Hover cursor
    profile.hoverSeg = null;
    if (profile.hoverX !== null && profile.hoverX !== undefined && profile.hoverX >= pad.left && profile.hoverX <= pad.left + plotW) {
        const d = Math.max(0, Math.min(totalDist, distAt(profile.hoverX)));
        let i = 1; while (i < nav.length - 1 && nav[i].dist < d) i++;
        const a = nav[i - 1], b = nav[i];
        const t = b.dist > a.dist ? (d - a.dist) / (b.dist - a.dist) : 0;
        const lat = a.lat + (b.lat - a.lat) * t, lng = a.lng + (b.lng - a.lng) * t;
        const alt = a.altMsl + (b.altMsl - a.altMsl) * t;
        const g = getTerrainElevationFromHGT(lat, lng);
        profile.hoverSeg = b.segId || a.segId;
        const x = Math.round(X(d)) + 0.5;
        ctx.strokeStyle = 'rgba(0, 210, 255, 0.8)'; ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(x, pad.top); ctx.lineTo(x, pad.top + plotH); ctx.stroke();
        ctx.beginPath(); ctx.arc(x, Y(alt), 4, 0, Math.PI * 2); ctx.fillStyle = '#00d2ff'; ctx.fill();

        const tip = document.getElementById('fp-profile-tip');
        if (tip) {
            const seg = profile.hoverSeg ? getSegment(profile.hoverSeg) : null;
            const idx = seg ? segs.indexOf(seg) + 1 : null;
            tip.innerHTML = `<b>${fmtDist(d)}</b>${seg ? ` · ${idx} ${SEGMENT_TYPES[seg.type].label}` : ''}<br>
                AMSL <b>${Math.round(alt)} m</b>${g !== null ? ` · AGL <b>${Math.round(alt - g)} m</b> · terrain ${Math.round(g)} m` : ''}`;
            tip.style.display = 'block';
            tip.style.left = `${Math.min(w - 190, profile.hoverX + 12)}px`;
            tip.style.top = `${Math.max(4, Math.min(h - 46, profile.hoverY - 40))}px`;
        }
        if (map) {
            if (!hoverMarker) hoverMarker = L.circleMarker([lat, lng], { radius: 6, color: '#00d2ff', weight: 2, fillColor: '#00d2ff', fillOpacity: 0.4, interactive: false, pane: 'fpHandles' }).addTo(map);
            else hoverMarker.setLatLng([lat, lng]);
        }
    }
    ctx.restore();

    // Axes
    ctx.fillStyle = '#6e7f8d'; ctx.font = `9px ${mono}`; ctx.textAlign = 'right';
    for (let a = Math.ceil(lo / altStep) * altStep; a <= hi; a += altStep) ctx.fillText(`${Math.round(a)}`, pad.left - 6, Y(a) + 3);
    ctx.textAlign = 'center';
    for (let d = 0; d <= totalDist; d += distStep) {
        const x = X(d);
        if (x < pad.left || x > pad.left + plotW) continue;
        ctx.fillText(fmtDist(d), x, h - 8);
    }
    // Header
    ctx.textAlign = 'left'; ctx.fillStyle = '#00d2ff'; ctx.font = `bold 10px "Rajdhani", sans-serif`;
    ctx.fillText('ELEVATION PROFILE', pad.left, 15);
    const st = compiled.stats;
    if (st) {
        ctx.fillStyle = '#889999'; ctx.font = `9px ${mono}`;
        const parts = [`${fmtDist(st.lengthM)}`, `${fmtTime(st.durationS)}`, `${st.waypoints} WP`];
        if (st.minAgl !== null) parts.push(`AGL ${Math.round(st.minAgl)}–${Math.round(st.maxAgl)} m`);
        if (st.maxAmsl !== null) parts.push(`AMSL max ${Math.round(st.maxAmsl)} m`);
        parts.push({ agl: 'AGL mode', amsl: 'AMSL mode', rel: 'REL mode' }[P.altMode]);
        ctx.fillText(parts.join('   ·   '), pad.left + 130, 15);
        if (profile.zoom > 1.05) { ctx.textAlign = 'right'; ctx.fillText(`${profile.zoom.toFixed(1)}×`, w - pad.right, 15); }
    }
}

// ── Utils ─────────────────────────────────────────────────────────────────────

function escapeHtml(s) {
    return String(s ?? '').replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
}
