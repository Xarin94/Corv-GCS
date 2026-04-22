/**
 * TabController.js - GCS Tab Navigation Controller
 * Handles switching between Flight Data, Flight Plan, Initial Setup, Config/Tuning, Simulation
 */

import { STATE } from '../core/state.js';
import { connect, disconnect, getAvailablePorts } from '../mavlink/ConnectionManager.js';
import { setParameter, requestAllParameters, requestParameter, requestDataStream, calibrateAccel, calibrateCompass, calibrateGyro, sendServoTest, sendRelayToggle, uploadMission } from '../mavlink/CommandSender.js';
import { onMessage } from '../mavlink/MAVLinkManager.js';
import { formatParamValue } from './ParametersPageController.js';
import { initJoystick } from '../joystick/JoystickUI.js';
import { getTerrainElevationFromHGT, getTerrainElevationAsync, resetAutoDownloadFailures } from '../terrain/TerrainManager.js';
import { getCmdShortName, getCmdColor, getCmdParams, getCmdDefaults, isNavCmd, getGroupedCommands } from '../mission/MissionCommands.js';
import { cachedTileLayer } from '../maps/CachedTileLayer.js';

let currentTab = 'flight-data';
let missionMap = null; // Leaflet map for flight plan tab
let missionSatelliteLayer = null;
let missionSatelliteVisible = true;

/**
 * Ensure HGT elevation data is parsed for all 1° tiles covered by the mission path.
 * Covers waypoint locations AND all 1° tiles along segments between waypoints.
 * After this resolves, synchronous getTerrainElevationFromHGT() will return data
 * for any point along the mission path (provided the HGT file was loaded).
 */
async function ensureMissionTerrainLoaded() {
    const items = STATE.missionItems;
    if (!items || items.length === 0) return;
    const navItems = items.filter(it => isNavCmd(it.command));
    // Collect unique 1° tile keys along the entire mission path
    const seen = new Set();
    const addTile = (lat, lng) => {
        if (!Number.isFinite(lat) || !Number.isFinite(lng)) return;
        const key = `${Math.floor(lat)}_${Math.floor(lng)}`;
        if (!seen.has(key)) seen.add(key);
    };
    for (let i = 0; i < navItems.length; i++) {
        addTile(navItems[i].lat, navItems[i].lng);
        // Sample along segment to catch any 1° tile boundaries crossed
        if (i < navItems.length - 1) {
            const latStep = navItems[i + 1].lat - navItems[i].lat;
            const lngStep = navItems[i + 1].lng - navItems[i].lng;
            // Sample every ~0.5° to ensure we don't miss tiles
            const steps = Math.max(2, Math.ceil(Math.max(Math.abs(latStep), Math.abs(lngStep)) * 2));
            for (let s = 1; s <= steps; s++) {
                const t = s / steps;
                addTile(navItems[i].lat + latStep * t, navItems[i].lng + lngStep * t);
            }
        }
    }
    // Also add non-nav items
    for (const it of items) addTile(it.lat, it.lng);

    // Allow one retry for tiles that previously failed (e.g. transient AWS error)
    resetAutoDownloadFailures();

    const promises = [];
    for (const key of seen) {
        const [lat, lon] = key.split('_').map(Number);
        promises.push(getTerrainElevationAsync(lat + 0.5, lon + 0.5));
    }
    await Promise.all(promises);
}

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
 * Initialize setup vertical nav switching
 */
function initSetupVerticalNav() {
    const nav = document.querySelector('.setup-vertical-nav');
    if (!nav) return;

    const buttons = nav.querySelectorAll('.setup-nav-btn');
    const contentArea = document.querySelector('.setup-content-area');
    if (!contentArea) return;

    const panels = contentArea.querySelectorAll('.sub-tab-content');

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
        });
    });
}

/**
 * Switch to a tab
 */
export function switchTab(tabName) {
    if (currentTab === tabName) return;

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

    // Initialize tab-specific content on first switch
    if (tabName === 'flight-plan') {
        if (!missionMap) {
            initMissionMap();
        }
        // Always invalidate map size after layout changes (sidebar hide, tab switch)
        setTimeout(() => {
            if (missionMap) missionMap.invalidateSize();
            render3DMission();
        }, 120);
    }

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

/**
 * Initialize the mission planning map (Flight Plan tab)
 */
function initMissionMap() {
    const container = document.getElementById('mission-map-full');
    if (!container || typeof L === 'undefined') return;

    missionMap = L.map(container, {
        center: [STATE.lat || 46.0, STATE.lon || 11.0],
        zoom: 13,
        zoomControl: true,
        keyboard: false
    });

    cachedTileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        maxZoom: 19,
        attribution: '&copy; OpenStreetMap',
        provider: 'osm'
    }).addTo(missionMap);

    // Satellite layer — added to map by default, togglable via toolbar
    missionSatelliteLayer = cachedTileLayer('https://mt{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}', {
        subdomains: ['0','1','2','3'],
        maxZoom: 20,
        provider: 'esri'
    });
    missionSatelliteLayer.addTo(missionMap);

    // Click to add waypoint or survey polygon vertex (left-click)
    missionMap.on('click', (e) => {
        if (surveyDrawMode) {
            addSurveyVertex(e.latlng.lat, e.latlng.lng);
        } else if (missionEditMode) {
            addWaypointAtLocation(e.latlng.lat, e.latlng.lng);
        }
    });

    // Double-click to close survey polygon
    missionMap.on('dblclick', (e) => {
        if (surveyDrawMode && surveyPolygonPoints.length >= 3) {
            L.DomEvent.stop(e);
            // dblclick fires after two click events — remove the last vertex added by the second click
            if (surveyPolygonPoints.length > 3) {
                surveyPolygonPoints.pop();
                if (surveyPolygonMarkers.length) surveyPolygonMarkers.pop().remove();
            }
            finishSurveyPolygon();
        }
    });

    // Right-click also adds waypoint (regardless of edit mode)
    missionMap.on('contextmenu', (e) => {
        if (!surveyDrawMode) {
            addWaypointAtLocation(e.latlng.lat, e.latlng.lng);
        }
    });

    // Map toolbar: grouped buttons (topleft, below zoom)
    const mapToolbar = L.control({ position: 'topleft' });
    mapToolbar.onAdd = function () {
        const bar = L.DomUtil.create('div', 'leaflet-bar mission-toolbar');
        bar.innerHTML = `
            <a href="#" class="mission-tb-btn" id="mission-edit-toggle" title="Add waypoints (click on map)">
                <svg viewBox="0 0 24 24" width="14" height="14" fill="none"><path d="M12 2C8.13 2 5 5.13 5 9c0 5.25 7 13 7 13s7-7.75 7-13c0-3.87-3.13-7-7-7z" stroke="currentColor" stroke-width="2" stroke-linejoin="round"/><circle cx="12" cy="9" r="2" stroke="currentColor" stroke-width="2"/><path d="M16 6l4 0M18 4v4" stroke="currentColor" stroke-width="2" stroke-linecap="round"/></svg>
            </a>
            <a href="#" class="mission-tb-btn" id="mission-survey-toggle" title="Draw survey area (click vertices, double-click to close)">
                <svg viewBox="0 0 24 24" width="14" height="14" fill="none"><path d="M4 4h16v16H4z" stroke="currentColor" stroke-width="2" stroke-dasharray="3 2" stroke-linejoin="round"/><path d="M8 12h8M12 8v8" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" opacity="0.6"/></svg>
            </a>
            <a href="#" class="mission-tb-btn active" id="mission-sat-toggle" title="Toggle satellite imagery">
                <svg viewBox="0 0 24 24" width="14" height="14" fill="none"><circle cx="12" cy="12" r="9" stroke="currentColor" stroke-width="2"/><path d="M3 12h18M12 3c-2.5 3-2.5 9 0 9s2.5 6 0 9" stroke="currentColor" stroke-width="1.5"/></svg>
            </a>
        `;
        L.DomEvent.disableClickPropagation(bar);

        // ADD WP toggle
        bar.querySelector('#mission-edit-toggle').addEventListener('click', (e) => {
            e.preventDefault();
            missionEditMode = !missionEditMode;
            if (missionEditMode && surveyDrawMode) { surveyDrawMode = false; bar.querySelector('#mission-survey-toggle').classList.remove('active'); clearSurveyDraw(); }
            e.currentTarget.classList.toggle('active', missionEditMode);
        });

        // SURVEY AREA toggle
        bar.querySelector('#mission-survey-toggle').addEventListener('click', (e) => {
            e.preventDefault();
            surveyDrawMode = !surveyDrawMode;
            if (surveyDrawMode && missionEditMode) { missionEditMode = false; bar.querySelector('#mission-edit-toggle').classList.remove('active'); }
            if (surveyDrawMode) {
                clearSurveyDraw();
                missionMap.doubleClickZoom.disable();
            } else {
                clearSurveyDraw();
                missionMap.doubleClickZoom.enable();
            }
            e.currentTarget.classList.toggle('active', surveyDrawMode);
        });

        // SATELLITE TOGGLE
        bar.querySelector('#mission-sat-toggle').addEventListener('click', (e) => {
            e.preventDefault();
            missionSatelliteVisible = !missionSatelliteVisible;
            if (missionSatelliteVisible) {
                missionSatelliteLayer.addTo(missionMap);
            } else {
                missionMap.removeLayer(missionSatelliteLayer);
            }
            e.currentTarget.classList.toggle('active', missionSatelliteVisible);
        });

        return bar;
    };
    mapToolbar.addTo(missionMap);

    // Initialize mission control buttons
    initMissionControls();

    // Initialize survey grid generator
    initSurveyGrid();

    // Invalidate size after a short delay (container may not be fully rendered)
    setTimeout(() => missionMap.invalidateSize(), 200);
}

let missionMarkers = [];
let missionPolyline = null;
let missionEditMode = false;
let surveyDrawMode = false;
let surveyPolygonPoints = [];
let surveyPolygonLayer = null;
let surveyPolygonMarkers = [];
let surveyPreviewLayer = null;
let altProfileVisible = false;
let selectedWpIdx = -1; // Currently selected waypoint index for editing

// 3D strip pan/zoom state
let strip3DScrollX = 0;   // horizontal scroll offset in pixels
let strip3DZoom = 1;       // zoom level (1 = fit all)
let strip3DDragging = false;
let strip3DDragStartX = 0;
let strip3DScrollStart = 0;
let strip3DListenersAttached = false;

// Vehicle position marker on mission map
let vehicleMarker = null;

const VEHICLE_ICON_SVG = `<svg viewBox="0 0 32 32" width="32" height="32" style="filter:drop-shadow(0 0 3px rgba(0,0,0,0.7));">
  <polygon points="16,2 22,28 16,22 10,28" fill="#00d2ff" stroke="#fff" stroke-width="1.5"/>
</svg>`;

function updateVehicleMarker() {
    if (!missionMap) return;
    const lat = STATE.lat;
    const lon = STATE.lon;
    if (!lat && !lon) return;

    const yawDeg = (STATE.yaw || 0) * (180 / Math.PI);

    if (!vehicleMarker) {
        const icon = L.divIcon({
            html: `<div style="transform:rotate(${yawDeg}deg);width:32px;height:32px;">${VEHICLE_ICON_SVG}</div>`,
            iconSize: [32, 32],
            iconAnchor: [16, 16],
            className: 'vehicle-map-marker'
        });
        vehicleMarker = L.marker([lat, lon], { icon, interactive: false, zIndexOffset: 1000 }).addTo(missionMap);
    } else {
        vehicleMarker.setLatLng([lat, lon]);
        const el = vehicleMarker.getElement();
        if (el) {
            const inner = el.querySelector('div');
            if (inner) inner.style.transform = `rotate(${yawDeg}deg)`;
        }
    }
}

// Home position marker on mission map
let homeMarker = null;

const HOME_ICON_SVG = `<svg viewBox="0 0 24 24" width="24" height="24" style="filter:drop-shadow(0 0 2px rgba(0,0,0,0.7));">
  <path d="M12 3L2 12h3v8h5v-5h4v5h5v-8h3L12 3z" fill="#ff8800" stroke="#fff" stroke-width="1"/>
</svg>`;

/**
 * Check if the straight-line path between two waypoints intersects the terrain.
 * Samples N points along the segment and compares interpolated flight altitude (MSL) with terrain.
 */
function checkTerrainIntersection(prevItem, currItem) {
    const SAMPLES = 20;
    const prevElev = getTerrainElevationFromHGT(prevItem.lat, prevItem.lng);
    const currElev = getTerrainElevationFromHGT(currItem.lat, currItem.lng);
    if (prevElev === null || currElev === null) return null; // no data
    const prevMSL = prevElev + prevItem.alt;
    const currMSL = currElev + currItem.alt;
    for (let s = 1; s < SAMPLES; s++) {
        const t = s / SAMPLES;
        const lat = prevItem.lat + (currItem.lat - prevItem.lat) * t;
        const lng = prevItem.lng + (currItem.lng - prevItem.lng) * t;
        const flightAlt = prevMSL + (currMSL - prevMSL) * t;
        const groundElev = getTerrainElevationFromHGT(lat, lng);
        if (groundElev !== null && flightAlt <= groundElev) return true; // collision
    }
    return false;
}

let _missionMapCenteredOnHome = false;

function updateHomeMarker() {
    if (!missionMap) return;
    if (STATE.homeLat === null || STATE.homeLon === null) return;

    if (!homeMarker) {
        const icon = L.divIcon({
            html: HOME_ICON_SVG,
            iconSize: [24, 24],
            iconAnchor: [12, 12],
            className: 'vehicle-map-marker'
        });
        homeMarker = L.marker([STATE.homeLat, STATE.homeLon], { icon, interactive: true, zIndexOffset: 900 }).addTo(missionMap);
        homeMarker.bindPopup(`<b>HOME</b><br>Alt: ${STATE.homeAlt ? STATE.homeAlt.toFixed(0) + 'm' : '?'}`);

        // Center mission map on home position the first time it's received
        if (!_missionMapCenteredOnHome) {
            missionMap.setView([STATE.homeLat, STATE.homeLon], 15);
            _missionMapCenteredOnHome = true;
        }
    } else {
        homeMarker.setLatLng([STATE.homeLat, STATE.homeLon]);
        homeMarker.setPopupContent(`<b>HOME</b><br>Alt: ${STATE.homeAlt ? STATE.homeAlt.toFixed(0) + 'm' : '?'}`);
    }
}

// Periodically update vehicle and home markers on mission map (only when visible)
setInterval(() => {
    if (currentTab !== 'flight-plan') return;
    updateVehicleMarker();
    updateHomeMarker();
}, 500);

// CMD_NAMES and CMD_COLORS are now derived from MissionCommands.js catalog

/**
 * Add a waypoint at the given location
 */
function addWaypointAtLocation(lat, lng) {
    const seq = STATE.missionItems.length;
    const altInput = document.getElementById('mission-default-alt');
    const cmdSelect = document.getElementById('mission-wp-cmd');
    const alt = altInput ? parseFloat(altInput.value) || 100 : 100;
    const command = cmdSelect ? parseInt(cmdSelect.value) || 16 : 16;

    const defaults = getCmdDefaults(command);
    const item = {
        seq,
        command,
        lat,
        lng,
        alt,
        frame: 0, // MAV_FRAME_GLOBAL (absolute MSL — converted at upload)
        param1: defaults.param1,
        param2: defaults.param2,
        param3: defaults.param3,
        param4: defaults.param4
    };

    STATE.missionItems.push(item);
    updateMissionDisplay();
}

/**
 * Populate the mission command dropdown with grouped optgroups from catalog
 */
function populateCmdDropdown() {
    const select = document.getElementById('mission-wp-cmd');
    if (!select) return;
    select.innerHTML = '';
    const groups = getGroupedCommands();
    for (const group of groups) {
        const optgroup = document.createElement('optgroup');
        optgroup.label = group.label;
        for (const cmd of group.commands) {
            const opt = document.createElement('option');
            opt.value = cmd.id;
            opt.textContent = cmd.name;
            optgroup.appendChild(opt);
        }
        select.appendChild(optgroup);
    }
}

/**
 * Render per-waypoint parameter input fields for the selected waypoint
 */
function renderParamFields(wpIdx) {
    const container = document.getElementById('mission-wp-params');
    if (!container) return;
    if (wpIdx < 0 || wpIdx >= STATE.missionItems.length) {
        container.style.display = 'none';
        container.innerHTML = '';
        return;
    }
    const item = STATE.missionItems[wpIdx];
    const paramDefs = getCmdParams(item.command);
    if (!paramDefs || Object.keys(paramDefs).length === 0) {
        container.style.display = 'none';
        container.innerHTML = '';
        return;
    }
    container.style.display = 'block';
    container.innerHTML = Object.entries(paramDefs).map(([key, def]) => {
        const val = item[key] !== undefined ? item[key] : def.default;
        return `<div class="mission-param-row">
            <label>${def.label}</label>
            <input type="number" class="gcs-input input-narrow mission-param-input"
                   data-param="${key}" value="${val}"
                   min="${def.min}" max="${def.max}" step="${def.step}">
            <span class="mission-param-unit">${def.unit}</span>
        </div>`;
    }).join('');
    // Attach change handlers
    container.querySelectorAll('.mission-param-input').forEach(inp => {
        inp.addEventListener('change', () => {
            if (wpIdx >= 0 && wpIdx < STATE.missionItems.length) {
                STATE.missionItems[wpIdx][inp.dataset.param] = parseFloat(inp.value) || 0;
            }
        });
    });
}

/**
 * Get a brief param summary string for a mission item (for the waypoint list)
 */
function getParamSummary(item) {
    const defs = getCmdParams(item.command);
    if (!defs) return '';
    const parts = [];
    for (const [key, def] of Object.entries(defs)) {
        const val = item[key];
        if (val !== undefined && val !== 0) {
            parts.push(`${def.label}:${val}${def.unit}`);
        }
    }
    return parts.length > 0 ? parts.join(' ') : '';
}

/**
 * Initialize mission controls (clear, set home, command/alt editing)
 */
function initMissionControls() {
    // Populate command dropdown from catalog
    populateCmdDropdown();

    const clearBtn = document.getElementById('mission-clear');
    if (clearBtn) {
        clearBtn.addEventListener('click', async () => {
            if (STATE.missionItems.length === 0) return;
            if (!await confirm('Clear all waypoints?')) return;
            STATE.missionItems.length = 0;
            selectedWpIdx = -1;
            updateMissionDisplay();
        });
    }

    const setHomeBtn = document.getElementById('mission-set-home');
    if (setHomeBtn) {
        setHomeBtn.addEventListener('click', () => {
            if (STATE.lat && STATE.lon) {
                const home = {
                    seq: 0, command: 16, lat: STATE.lat, lng: STATE.lon,
                    alt: 0, frame: 0
                };
                if (STATE.missionItems.length > 0 && STATE.missionItems[0].seq === 0) {
                    STATE.missionItems[0] = home;
                } else {
                    STATE.missionItems.unshift(home);
                }
                updateMissionDisplay();
            } else {
                alert('No vehicle position available');
            }
        });
    }

    // Upload mission to autopilot
    const uploadBtn = document.getElementById('mission-upload');
    if (uploadBtn) {
        uploadBtn.addEventListener('click', async () => {
            if (STATE.connectionType === 'none') return alert('Not connected');
            if (STATE.missionItems.length === 0) return alert('No waypoints to upload');
            uploadBtn.disabled = true;
            uploadBtn.textContent = 'UPLOADING...';
            try {
                // ArduPilot requires seq 0 = HOME. Auto-prepend if missing.
                let itemsToUpload = STATE.missionItems.map(it => ({ ...it }));
                const hasHome = itemsToUpload.length > 0 && itemsToUpload[0].frame === 0;
                if (!hasHome) {
                    const homeLat = STATE.homeLat || STATE.lat || 0;
                    const homeLng = STATE.homeLon || STATE.lon || 0;
                    const homeItem = { seq: 0, command: 16, lat: homeLat, lng: homeLng, alt: 0, frame: 0 };
                    itemsToUpload = [homeItem, ...itemsToUpload.map((it, i) => ({ ...it, seq: i + 1 }))];
                }

                // Ensure terrain data is available for all waypoints before computing
                await ensureMissionTerrainLoaded();

                // Absolute MSL: alt_MSL = terrain_elevation(wp) + desired_AGL
                // Frame 0 (MAV_FRAME_GLOBAL) = absolute MSL — ArduPilot flies exactly at this altitude
                const homeWp = itemsToUpload[0];
                const homeElev = getTerrainElevationFromHGT(homeWp.lat, homeWp.lng);

                // Set home item altitude to actual terrain MSL (ArduPilot uses this as reference)
                if (homeElev !== null) homeWp.alt = Math.round(homeElev);

                for (let i = 1; i < itemsToUpload.length; i++) {
                    const wp = itemsToUpload[i];
                    const wpElev = getTerrainElevationFromHGT(wp.lat, wp.lng);
                    // Fallback: if terrain not available for this WP, use home elevation
                    const baseElev = wpElev !== null ? wpElev : (homeElev !== null ? homeElev : (STATE.homeAlt || 0));
                    wp.alt = Math.round(baseElev + (wp.alt || 0));
                    wp.frame = 0; // MAV_FRAME_GLOBAL — absolute MSL
                }

                const result = await uploadMission(itemsToUpload);
                uploadBtn.textContent = `DONE (${result.count})`;
                setTimeout(() => { uploadBtn.textContent = 'UPLOAD'; uploadBtn.disabled = false; }, 2000);
            } catch (e) {
                alert('Upload failed: ' + e.message);
                uploadBtn.textContent = 'UPLOAD';
                uploadBtn.disabled = false;
            }
        });
    }

    // Download mission from autopilot
    const downloadBtn = document.getElementById('mission-download');
    if (downloadBtn) {
        downloadBtn.addEventListener('click', async () => {
            if (STATE.connectionType === 'none') return alert('Not connected');
            if (!window.mavlink) return;
            downloadBtn.disabled = true;
            downloadBtn.textContent = 'READING...';
            try {
                await window.mavlink.sendMessage({
                    type: 'MISSION_REQUEST_LIST',
                    targetSystem: STATE.systemId,
                    targetComponent: STATE.componentId,
                    missionType: 0
                });
                downloadBtn.textContent = 'DOWNLOAD';
                downloadBtn.disabled = false;
            } catch (e) {
                alert('Download failed: ' + e.message);
                downloadBtn.textContent = 'DOWNLOAD';
                downloadBtn.disabled = false;
            }
        });
    }

    // Change selected waypoint's command type when dropdown changes
    const cmdSelect = document.getElementById('mission-wp-cmd');
    if (cmdSelect) {
        cmdSelect.addEventListener('change', () => {
            if (selectedWpIdx >= 0 && selectedWpIdx < STATE.missionItems.length) {
                const newCmd = parseInt(cmdSelect.value) || 16;
                STATE.missionItems[selectedWpIdx].command = newCmd;
                const defaults = getCmdDefaults(newCmd);
                STATE.missionItems[selectedWpIdx].param1 = defaults.param1;
                STATE.missionItems[selectedWpIdx].param2 = defaults.param2;
                STATE.missionItems[selectedWpIdx].param3 = defaults.param3;
                STATE.missionItems[selectedWpIdx].param4 = defaults.param4;
                renderParamFields(selectedWpIdx);
                updateMissionDisplay();
            }
        });
    }

    // Change selected waypoint's altitude when alt input changes
    const altInput = document.getElementById('mission-default-alt');
    if (altInput) {
        altInput.addEventListener('change', () => {
            if (selectedWpIdx >= 0 && selectedWpIdx < STATE.missionItems.length) {
                STATE.missionItems[selectedWpIdx].alt = parseFloat(altInput.value) || 100;
                updateMissionDisplay();
            }
        });
    }

    // Deselect button
    const deselectBtn = document.getElementById('mission-wp-deselect');
    if (deselectBtn) {
        deselectBtn.addEventListener('click', () => {
            selectedWpIdx = -1;
            renderParamFields(-1);
            updateMissionDisplay();
        });
    }

    // ESC key to deselect
    document.addEventListener('keydown', (e) => {
        if (e.key === 'Escape' && selectedWpIdx >= 0) {
            selectedWpIdx = -1;
            renderParamFields(-1);
            updateMissionDisplay();
        }
    });
}

/**
 * Calculate total mission distance in meters
 */
function calcMissionDistance() {
    let dist = 0;
    const navItems = STATE.missionItems.filter(it => isNavCmd(it.command));
    for (let i = 1; i < navItems.length; i++) {
        const a = navItems[i - 1];
        const b = navItems[i];
        if (missionMap) {
            dist += missionMap.distance([a.lat, a.lng], [b.lat, b.lng]);
        }
    }
    return dist;
}

/**
 * Update mission display on map and list
 */
async function updateMissionDisplay() {
    // Ensure HGT elevation data is parsed for distant waypoint tiles
    await ensureMissionTerrainLoaded();

    // Clear existing markers and polyline
    missionMarkers.forEach(m => m.remove());
    missionMarkers = [];
    if (missionPolyline) { missionPolyline.remove(); missionPolyline = null; }

    if (missionMap) {
        // Draw polyline connecting nav waypoints (solid green)
        const navCoords = STATE.missionItems
            .filter(it => isNavCmd(it.command))
            .map(it => [it.lat, it.lng]);
        if (navCoords.length >= 2) {
            missionPolyline = L.polyline(navCoords, {
                color: '#44ff44', weight: 2.5, opacity: 0.85
            }).addTo(missionMap);
        }

        // Add colored circle markers by command type
        STATE.missionItems.forEach((item, i) => {
            const cmdName = getCmdShortName(item.command);
            const color = getCmdColor(item.command);
            const marker = L.circleMarker([item.lat, item.lng], {
                radius: 8,
                fillColor: color,
                color: '#ffffff',
                weight: 2,
                opacity: 0.9,
                fillOpacity: 0.85,
                title: `${i}: ${cmdName} ${item.alt}m`
            }).addTo(missionMap);

            // Add sequence number label
            const label = L.divIcon({
                className: 'mission-wp-label',
                html: `<span>${i}</span>`,
                iconSize: [20, 20],
                iconAnchor: [10, 10]
            });
            const labelMarker = L.marker([item.lat, item.lng], {
                icon: label,
                interactive: false
            }).addTo(missionMap);
            missionMarkers.push(labelMarker);

            const popupTerrainElev = getTerrainElevationFromHGT(item.lat, item.lng);
            const popupAsl = popupTerrainElev !== null ? `${Math.round(popupTerrainElev + item.alt)}m ASL` : '---';
            marker.bindPopup(`<b>${i}: ${cmdName}</b><br>AGL: ${item.alt}m | ASL: ${popupAsl}<br>${item.lat.toFixed(6)}, ${item.lng.toFixed(6)}`);
            missionMarkers.push(marker);

            // Make circle markers draggable via mouse events
            marker.on('mousedown', function () {
                missionMap.dragging.disable();
                const onMove = (e) => {
                    marker.setLatLng(e.latlng);
                    labelMarker.setLatLng(e.latlng);
                };
                missionMap.on('mousemove', onMove);
                missionMap.once('mouseup', () => {
                    missionMap.off('mousemove', onMove);
                    missionMap.dragging.enable();
                    const pos = marker.getLatLng();
                    STATE.missionItems[i].lat = pos.lat;
                    STATE.missionItems[i].lng = pos.lng;
                    updateMissionDisplay();
                });
            });
        });
    }

    // Update waypoint list panel
    const listEl = document.getElementById('mission-wp-list');
    if (listEl) {
        listEl.innerHTML = STATE.missionItems.map((item, i) => {
            const cmdName = getCmdShortName(item.command);
            const dotColor = getCmdColor(item.command);
            const isSelected = i === selectedWpIdx;
            const isActive = i === STATE.missionCurrentSeq;
            const terrainElev = getTerrainElevationFromHGT(item.lat, item.lng);
            const agl = item.alt;
            const aslStr = terrainElev !== null ? `${Math.round(terrainElev + agl)}` : '---';
            // Compute altitude relative to home (what ArduPilot actually flies)
            const homeIt = STATE.missionItems[0];
            const homeElev = homeIt ? getTerrainElevationFromHGT(homeIt.lat, homeIt.lng) : null;
            const relHomeStr = (terrainElev !== null && homeElev !== null && i > 0)
                ? `${Math.round(terrainElev - homeElev + agl)}`
                : '---';
            // Terrain intersection check from previous waypoint
            let terrainDotHtml = '';
            if (i > 0) {
                const collision = checkTerrainIntersection(STATE.missionItems[i - 1], item);
                if (collision !== null) {
                    terrainDotHtml = `<span class="wp-terrain-dot ${collision ? 'danger' : 'safe'}"></span>`;
                }
            }
            const paramStr = getParamSummary(item);
            const paramLine = paramStr ? `<div class="wp-row-params"><span class="mission-wp-param-summary">${paramStr}</span></div>` : '';
            return `<div class="gcs-mission-item${isActive ? ' active' : ''}${isSelected ? ' selected' : ''}" data-wp-idx="${i}" draggable="true">
                <div class="wp-row-main">
                    <span class="wp-drag-handle" title="Drag to reorder"><svg viewBox="0 0 24 24" width="12" height="12" fill="none"><path d="M8 6h2M14 6h2M8 10h2M14 10h2M8 14h2M14 14h2M8 18h2M14 18h2" stroke="currentColor" stroke-width="2" stroke-linecap="round"/></svg></span>
                    <span class="gcs-mission-seq">${i}</span>
                    <span class="wp-color-dot" style="background:${dotColor}"></span>
                    <span class="wp-cmd-name">${cmdName}</span>
                    <span class="wp-alt-value" title="AGL / MSL / RelHome">${terrainDotHtml}${agl}m <span class="wp-asl-value">${aslStr}m</span> <span class="wp-relh-value" style="color:#ffaa00">[${relHomeStr}]</span></span>
                    <button class="mission-wp-del" data-wp-del="${i}" title="Remove">&times;</button>
                </div>
                ${paramLine}
            </div>`;
        }).join('');

        // Click to select waypoint
        listEl.querySelectorAll('.gcs-mission-item').forEach(row => {
            row.addEventListener('click', (e) => {
                if (e.target.classList.contains('mission-wp-del')) return;
                const idx = parseInt(row.dataset.wpIdx);
                selectedWpIdx = (selectedWpIdx === idx) ? -1 : idx;
                // Update selection visuals
                listEl.querySelectorAll('.gcs-mission-item').forEach(r => r.classList.remove('selected'));
                if (selectedWpIdx >= 0) {
                    row.classList.add('selected');
                    // Update command dropdown to match selected WP
                    const cmdSelect = document.getElementById('mission-wp-cmd');
                    if (cmdSelect) cmdSelect.value = STATE.missionItems[idx].command;
                    // Update alt input to match selected WP
                    const altInput = document.getElementById('mission-default-alt');
                    if (altInput) altInput.value = STATE.missionItems[idx].alt;
                    // Show param fields for selected WP
                    renderParamFields(idx);
                    // Pan map to selected WP
                    if (missionMap) {
                        const item = STATE.missionItems[idx];
                        missionMap.panTo([item.lat, item.lng]);
                    }
                } else {
                    renderParamFields(-1);
                }
            });
        });

        // Delete button handlers
        listEl.querySelectorAll('.mission-wp-del').forEach(btn => {
            btn.addEventListener('click', (e) => {
                e.stopPropagation();
                const idx = parseInt(btn.dataset.wpDel);
                STATE.missionItems.splice(idx, 1);
                STATE.missionItems.forEach((it, j) => it.seq = j);
                if (selectedWpIdx === idx) selectedWpIdx = -1;
                else if (selectedWpIdx > idx) selectedWpIdx--;
                updateMissionDisplay();
            });
        });

        // Drag-to-reorder handlers
        let dragSrcIdx = null;
        listEl.querySelectorAll('.gcs-mission-item').forEach(row => {
            row.addEventListener('dragstart', (e) => {
                dragSrcIdx = parseInt(row.dataset.wpIdx);
                row.classList.add('dragging');
                e.dataTransfer.effectAllowed = 'move';
                e.dataTransfer.setData('text/plain', dragSrcIdx);
            });
            row.addEventListener('dragend', () => {
                row.classList.remove('dragging');
                listEl.querySelectorAll('.gcs-mission-item').forEach(r => r.classList.remove('drag-over'));
            });
            row.addEventListener('dragover', (e) => {
                e.preventDefault();
                e.dataTransfer.dropEffect = 'move';
                listEl.querySelectorAll('.gcs-mission-item').forEach(r => r.classList.remove('drag-over'));
                row.classList.add('drag-over');
            });
            row.addEventListener('dragleave', () => {
                row.classList.remove('drag-over');
            });
            row.addEventListener('drop', (e) => {
                e.preventDefault();
                const dropIdx = parseInt(row.dataset.wpIdx);
                if (dragSrcIdx !== null && dragSrcIdx !== dropIdx) {
                    // Reorder mission items
                    const [moved] = STATE.missionItems.splice(dragSrcIdx, 1);
                    STATE.missionItems.splice(dropIdx, 0, moved);
                    STATE.missionItems.forEach((it, j) => it.seq = j);
                    // Update selection to follow moved item
                    if (selectedWpIdx === dragSrcIdx) selectedWpIdx = dropIdx;
                    else if (selectedWpIdx > dragSrcIdx && selectedWpIdx <= dropIdx) selectedWpIdx--;
                    else if (selectedWpIdx < dragSrcIdx && selectedWpIdx >= dropIdx) selectedWpIdx++;
                    updateMissionDisplay();
                }
                dragSrcIdx = null;
            });
        });
    }

    // Update edit hint bar
    const editHint = document.getElementById('mission-wp-edit-hint');
    const editIdx = document.getElementById('mission-wp-edit-idx');
    if (editHint) {
        if (selectedWpIdx >= 0 && selectedWpIdx < STATE.missionItems.length) {
            editHint.style.display = 'flex';
            if (editIdx) editIdx.textContent = selectedWpIdx;
        } else {
            editHint.style.display = 'none';
        }
    }

    // Update info
    const countEl = document.getElementById('mission-wp-count');
    if (countEl) countEl.textContent = `WP: ${STATE.missionItems.length}`;

    const distEl = document.getElementById('mission-total-dist');
    if (distEl) {
        const dist = calcMissionDistance();
        distEl.textContent = dist >= 1000
            ? `Dist: ${(dist / 1000).toFixed(1)} km`
            : `Dist: ${Math.round(dist)} m`;
    }

    // Notify 3D scene to update trajectory
    window.dispatchEvent(new CustomEvent('missionUpdated'));

    // Update altitude profile if visible
    if (altProfileVisible) renderAltitudeProfile();
    render3DMission();
}

/**
 * Toggle altitude profile panel
 */
function toggleAltitudeProfile() {
    const container = document.getElementById('mission-map-full');
    if (!container) return;

    altProfileVisible = !altProfileVisible;
    let panel = document.getElementById('alt-profile-panel');

    if (!altProfileVisible) {
        if (panel) panel.style.display = 'none';
        return;
    }

    if (!panel) {
        panel = document.createElement('div');
        panel.id = 'alt-profile-panel';
        panel.innerHTML = '<canvas id="alt-profile-canvas" width="800" height="150"></canvas>';
        container.appendChild(panel);
    }
    panel.style.display = 'block';
    renderAltitudeProfile();
}

/**
 * Render side-view elevation profile of mission path with terrain.
 * X-axis = cumulative distance along the path, Y-axis = altitude MSL.
 * Supports horizontal scroll (drag / mouse wheel) and zoom (Ctrl+wheel).
 */
function render3DMission() {
    const canvas = document.getElementById('mission-3d-canvas');
    if (!canvas) return;
    const ctx = canvas.getContext('2d');

    const panel = canvas.parentElement;
    canvas.width = panel.clientWidth || 600;
    canvas.height = panel.clientHeight || 170;

    const w = canvas.width;
    const h = canvas.height;
    const pad = { top: 24, bottom: 28, left: 50, right: 20 };

    // Attach pan/zoom listeners once
    if (!strip3DListenersAttached) {
        strip3DListenersAttached = true;
        canvas.addEventListener('wheel', (e) => {
            e.preventDefault();
            if (e.ctrlKey || e.metaKey) {
                // Zoom
                const zoomFactor = e.deltaY < 0 ? 1.15 : 1 / 1.15;
                const oldZoom = strip3DZoom;
                strip3DZoom = Math.max(1, Math.min(20, strip3DZoom * zoomFactor));
                // Keep zoom centered on mouse position
                const rect = canvas.getBoundingClientRect();
                const mx = e.clientX - rect.left - pad.left;
                const plotW = w - pad.left - pad.right;
                const ratio = (mx + strip3DScrollX) / (plotW * oldZoom);
                strip3DScrollX = ratio * (plotW * strip3DZoom) - mx;
            } else {
                // Horizontal scroll
                strip3DScrollX += e.deltaY + e.deltaX;
            }
            // Clamp scroll
            const plotW = w - pad.left - pad.right;
            const maxScroll = Math.max(0, plotW * strip3DZoom - plotW);
            strip3DScrollX = Math.max(0, Math.min(maxScroll, strip3DScrollX));
            render3DMission();
        }, { passive: false });

        canvas.addEventListener('mousedown', (e) => {
            strip3DDragging = true;
            strip3DDragStartX = e.clientX;
            strip3DScrollStart = strip3DScrollX;
            canvas.style.cursor = 'grabbing';
        });
        window.addEventListener('mousemove', (e) => {
            if (!strip3DDragging) return;
            const dx = strip3DDragStartX - e.clientX;
            const plotW = w - pad.left - pad.right;
            const maxScroll = Math.max(0, plotW * strip3DZoom - plotW);
            strip3DScrollX = Math.max(0, Math.min(maxScroll, strip3DScrollStart + dx));
            render3DMission();
        });
        window.addEventListener('mouseup', () => {
            if (strip3DDragging) {
                strip3DDragging = false;
                canvas.style.cursor = 'grab';
            }
        });
        canvas.style.cursor = 'grab';
    }

    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = 'rgba(5, 12, 20, 0.95)';
    ctx.fillRect(0, 0, w, h);

    const navItems = STATE.missionItems.filter(it => isNavCmd(it.command));

    if (navItems.length < 2) {
        ctx.fillStyle = '#889999';
        ctx.font = '11px monospace';
        ctx.textAlign = 'center';
        ctx.fillText('Add at least 2 nav waypoints for elevation view', w / 2, h / 2);
        return;
    }

    // Haversine distance between two points (meters)
    const haversine = (lat1, lon1, lat2, lon2) => {
        const R = 6371000;
        const dLat = (lat2 - lat1) * Math.PI / 180;
        const dLon = (lon2 - lon1) * Math.PI / 180;
        const a = Math.sin(dLat / 2) ** 2 +
                  Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
                  Math.sin(dLon / 2) ** 2;
        return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    };

    // Calculate cumulative distances along mission path
    const cumDist = [0];
    for (let i = 1; i < navItems.length; i++) {
        cumDist.push(cumDist[i - 1] + haversine(
            navItems[i - 1].lat, navItems[i - 1].lng,
            navItems[i].lat, navItems[i].lng
        ));
    }
    const totalDist = cumDist[cumDist.length - 1] || 1;

    // Get terrain elevation at each waypoint + sample between waypoints
    const terrainAlts = navItems.map(it => {
        const elev = getTerrainElevationFromHGT(it.lat, it.lng);
        return elev !== null ? elev : 0;
    });
    const mslAlts = navItems.map((it, i) => terrainAlts[i] + (it.alt || 0));

    // Build detailed terrain profile by interpolating between waypoints
    const terrainProfile = []; // {dist, elev}
    const TERRAIN_SAMPLES = 80;
    for (let i = 0; i < navItems.length - 1; i++) {
        const segLen = cumDist[i + 1] - cumDist[i];
        const samples = Math.max(2, Math.ceil(TERRAIN_SAMPLES * (segLen / totalDist)));
        for (let s = 0; s < samples; s++) {
            const t = s / samples;
            const lat = navItems[i].lat + t * (navItems[i + 1].lat - navItems[i].lat);
            const lng = navItems[i].lng + t * (navItems[i + 1].lng - navItems[i].lng);
            const d = cumDist[i] + t * segLen;
            const elev = getTerrainElevationFromHGT(lat, lng);
            terrainProfile.push({ dist: d, elev: elev !== null ? elev : 0 });
        }
    }
    // Add last point
    terrainProfile.push({ dist: totalDist, elev: terrainAlts[terrainAlts.length - 1] });

    // Calculate altitude range
    const allElevs = terrainProfile.map(p => p.elev);
    const minTerrain = Math.min(...allElevs, ...terrainAlts);
    const maxMsl = Math.max(...mslAlts);
    const altFloor = minTerrain - 20;
    const altCeil = maxMsl + 30;
    const altRange = (altCeil - altFloor) || 100;

    const plotW = w - pad.left - pad.right;
    const plotH = h - pad.top - pad.bottom;
    const totalPlotW = plotW * strip3DZoom;

    // Project: distance → X (with zoom + scroll), altitude → Y
    const projX = (dist) => pad.left + (dist / totalDist) * totalPlotW - strip3DScrollX;
    const projY = (altMsl) => pad.top + plotH - ((altMsl - altFloor) / altRange) * plotH;

    // Clip to plot area
    ctx.save();
    ctx.beginPath();
    ctx.rect(pad.left, 0, plotW, h);
    ctx.clip();

    // Draw altitude grid lines
    const altStep = niceStep(altRange, 5);
    const altGridStart = Math.ceil(altFloor / altStep) * altStep;
    ctx.strokeStyle = 'rgba(255,255,255,0.06)';
    ctx.lineWidth = 0.5;
    ctx.fillStyle = '#556';
    ctx.font = '8px monospace';
    ctx.textAlign = 'right';
    for (let a = altGridStart; a <= altCeil; a += altStep) {
        const y = projY(a);
        if (y < pad.top || y > pad.top + plotH) continue;
        ctx.beginPath();
        ctx.moveTo(pad.left, y);
        ctx.lineTo(pad.left + plotW, y);
        ctx.stroke();
    }

    // Draw distance grid lines
    const distStep = niceStep(totalDist, Math.max(4, Math.floor(totalPlotW / 80)));
    ctx.strokeStyle = 'rgba(255,255,255,0.04)';
    for (let d = 0; d <= totalDist; d += distStep) {
        const x = projX(d);
        if (x < pad.left || x > pad.left + plotW) continue;
        ctx.beginPath();
        ctx.moveTo(x, pad.top);
        ctx.lineTo(x, pad.top + plotH);
        ctx.stroke();
    }

    // Draw terrain fill
    ctx.beginPath();
    ctx.moveTo(projX(terrainProfile[0].dist), projY(altFloor));
    terrainProfile.forEach(p => ctx.lineTo(projX(p.dist), projY(p.elev)));
    ctx.lineTo(projX(terrainProfile[terrainProfile.length - 1].dist), projY(altFloor));
    ctx.closePath();
    const terrGrad = ctx.createLinearGradient(0, projY(altCeil), 0, projY(altFloor));
    terrGrad.addColorStop(0, 'rgba(80, 120, 60, 0.5)');
    terrGrad.addColorStop(1, 'rgba(40, 60, 30, 0.2)');
    ctx.fillStyle = terrGrad;
    ctx.fill();

    // Draw terrain line
    ctx.beginPath();
    terrainProfile.forEach((p, i) => {
        const x = projX(p.dist);
        const y = projY(p.elev);
        i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    });
    ctx.strokeStyle = 'rgba(100, 160, 80, 0.6)';
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // Draw vertical drop lines from each WP to terrain
    navItems.forEach((item, i) => {
        const x = projX(cumDist[i]);
        const yTop = projY(mslAlts[i]);
        const yBot = projY(terrainAlts[i]);
        ctx.strokeStyle = 'rgba(68, 255, 68, 0.12)';
        ctx.lineWidth = 1;
        ctx.setLineDash([3, 3]);
        ctx.beginPath(); ctx.moveTo(x, yTop); ctx.lineTo(x, yBot); ctx.stroke();
        ctx.setLineDash([]);
    });

    // Draw flight path line
    ctx.beginPath();
    navItems.forEach((item, i) => {
        const x = projX(cumDist[i]);
        const y = projY(mslAlts[i]);
        i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    });
    ctx.strokeStyle = '#44ff44';
    ctx.lineWidth = 2;
    ctx.stroke();

    // Draw waypoint dots and labels
    navItems.forEach((item, i) => {
        const x = projX(cumDist[i]);
        const y = projY(mslAlts[i]);
        // Skip if outside visible area
        if (x < pad.left - 20 || x > pad.left + plotW + 20) return;

        const color = getCmdColor(item.command);
        ctx.beginPath();
        ctx.arc(x, y, 5, 0, Math.PI * 2);
        ctx.fillStyle = color;
        ctx.fill();
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 1.5;
        ctx.stroke();

        // WP index above
        ctx.fillStyle = '#fff';
        ctx.font = '9px monospace';
        ctx.textAlign = 'center';
        ctx.fillText(`${i}`, x, y - 9);
        // AGL altitude below
        ctx.fillStyle = '#889999';
        ctx.font = '8px monospace';
        ctx.fillText(`${item.alt}m`, x, y + 14);
    });

    ctx.restore(); // end clip

    // Draw Y-axis labels (altitude) — outside clip
    ctx.fillStyle = '#556';
    ctx.font = '8px monospace';
    ctx.textAlign = 'right';
    for (let a = altGridStart; a <= altCeil; a += altStep) {
        const y = projY(a);
        if (y < pad.top || y > pad.top + plotH) continue;
        ctx.fillText(`${Math.round(a)}m`, pad.left - 4, y + 3);
    }

    // Draw X-axis labels (distance) — outside clip
    ctx.fillStyle = '#556';
    ctx.font = '8px monospace';
    ctx.textAlign = 'center';
    for (let d = 0; d <= totalDist; d += distStep) {
        const x = projX(d);
        if (x < pad.left || x > pad.left + plotW) continue;
        const label = d >= 1000 ? `${(d / 1000).toFixed(1)}km` : `${Math.round(d)}m`;
        ctx.fillText(label, x, h - pad.bottom + 14);
    }

    // Title
    ctx.fillStyle = '#00d2ff';
    ctx.font = '10px monospace';
    ctx.textAlign = 'left';
    ctx.fillText('ELEVATION PROFILE', pad.left, 14);

    // Zoom indicator
    if (strip3DZoom > 1.05) {
        ctx.fillStyle = '#556';
        ctx.textAlign = 'right';
        ctx.fillText(`${strip3DZoom.toFixed(1)}x`, w - pad.right, 14);
    }
}

/** Calculate a "nice" step for grid lines given a range and desired ~count */
function niceStep(range, count) {
    const raw = range / count;
    const mag = Math.pow(10, Math.floor(Math.log10(raw)));
    const norm = raw / mag;
    let step;
    if (norm < 1.5) step = 1;
    else if (norm < 3.5) step = 2;
    else if (norm < 7.5) step = 5;
    else step = 10;
    return step * mag;
}

/**
 * Render altitude profile on canvas
 */
function renderAltitudeProfile() {
    const canvas = document.getElementById('alt-profile-canvas');
    if (!canvas) return;
    const ctx = canvas.getContext('2d');

    // Resize canvas to container
    const panel = canvas.parentElement;
    canvas.width = panel.clientWidth || 800;
    canvas.height = 150;

    const w = canvas.width;
    const h = canvas.height;
    const pad = { top: 20, bottom: 25, left: 45, right: 15 };

    ctx.clearRect(0, 0, w, h);

    // Background
    ctx.fillStyle = 'rgba(5, 12, 20, 0.92)';
    ctx.fillRect(0, 0, w, h);

    const navItems = STATE.missionItems.filter(it => isNavCmd(it.command));
    if (navItems.length < 2) {
        ctx.fillStyle = '#889999';
        ctx.font = '11px monospace';
        ctx.textAlign = 'center';
        ctx.fillText('Add at least 2 nav waypoints to see altitude profile', w / 2, h / 2);
        return;
    }

    // Calculate cumulative distance
    const distances = [0];
    for (let i = 1; i < navItems.length; i++) {
        const a = navItems[i - 1], b = navItems[i];
        const d = missionMap ? missionMap.distance([a.lat, a.lng], [b.lat, b.lng]) : 0;
        distances.push(distances[i - 1] + d);
    }
    const totalDist = distances[distances.length - 1] || 1;
    const maxAlt = Math.max(...navItems.map(it => it.alt), 10);
    const minAlt = Math.min(...navItems.map(it => it.alt), 0);
    const altRange = (maxAlt - minAlt) || 10;

    const plotW = w - pad.left - pad.right;
    const plotH = h - pad.top - pad.bottom;

    const xScale = (d) => pad.left + (d / totalDist) * plotW;
    const yScale = (a) => pad.top + plotH - ((a - minAlt) / altRange) * plotH;

    // Grid lines
    ctx.strokeStyle = 'rgba(255,255,255,0.08)';
    ctx.lineWidth = 0.5;
    for (let i = 0; i <= 4; i++) {
        const y = pad.top + (plotH / 4) * i;
        ctx.beginPath(); ctx.moveTo(pad.left, y); ctx.lineTo(w - pad.right, y); ctx.stroke();
    }

    // Altitude fill
    ctx.beginPath();
    ctx.moveTo(xScale(distances[0]), yScale(0));
    navItems.forEach((item, i) => ctx.lineTo(xScale(distances[i]), yScale(item.alt)));
    ctx.lineTo(xScale(distances[distances.length - 1]), yScale(0));
    ctx.closePath();
    ctx.fillStyle = 'rgba(68, 255, 68, 0.12)';
    ctx.fill();

    // Altitude line
    ctx.beginPath();
    navItems.forEach((item, i) => {
        const x = xScale(distances[i]), y = yScale(item.alt);
        i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    });
    ctx.strokeStyle = '#44ff44';
    ctx.lineWidth = 2;
    ctx.stroke();

    // Waypoint dots
    navItems.forEach((item, i) => {
        const x = xScale(distances[i]), y = yScale(item.alt);
        const color = getCmdColor(item.command);
        ctx.beginPath();
        ctx.arc(x, y, 5, 0, Math.PI * 2);
        ctx.fillStyle = color;
        ctx.fill();
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 1.5;
        ctx.stroke();
    });

    // Labels
    ctx.fillStyle = '#889999';
    ctx.font = '9px monospace';
    ctx.textAlign = 'center';

    // X-axis: distance labels
    navItems.forEach((item, i) => {
        const x = xScale(distances[i]);
        const d = distances[i];
        const label = d >= 1000 ? `${(d / 1000).toFixed(1)}km` : `${Math.round(d)}m`;
        ctx.fillText(label, x, h - 5);
    });

    // Y-axis: altitude labels
    ctx.textAlign = 'right';
    for (let i = 0; i <= 4; i++) {
        const alt = minAlt + (altRange / 4) * (4 - i);
        const y = pad.top + (plotH / 4) * i;
        ctx.fillText(`${Math.round(alt)}m`, pad.left - 5, y + 3);
    }

    // Title
    ctx.fillStyle = '#00d2ff';
    ctx.font = '10px monospace';
    ctx.textAlign = 'left';
    ctx.fillText('ALTITUDE PROFILE', pad.left, 13);
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
                } else if (type === 'corv-binary') {
                    const port = document.getElementById('setup-serial-port')?.value;
                    const baud = parseInt(document.getElementById('setup-baud')?.value) || 460800;
                    if (!port) { alert('Select a serial port first'); return; }
                    await connect('corv-binary', { port, baudRate: baud });
                }
            } catch (e) {
                alert('Connection failed: ' + e.message);
            }
        });
    }

    if (disconnectBtn) {
        disconnectBtn.addEventListener('click', async () => {
            await disconnect();
            _missionMapCenteredOnHome = false;
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

    if (STATE.parameterCount > 0) {
        if (progressEl) progressEl.style.display = 'flex';
        const pct = (STATE.parametersReceived / STATE.parameterCount * 100).toFixed(0);
        fillEl.style.width = pct + '%';
        countEl.textContent = `${STATE.parametersReceived}/${STATE.parameterCount}`;
    }
}

/**
 * Initialize Config/Tuning tab
 */
function initConfigTuningTab() {
    // PID write all button
    const writeAllBtn = document.getElementById('pid-write-all');
    if (writeAllBtn) {
        writeAllBtn.addEventListener('click', async () => {
            const inputs = document.querySelectorAll('.pid-input[data-param]');
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
            alert(`Written ${count} PID parameters`);
            setTimeout(() => {
                inputs.forEach(i => i.style.borderColor = '');
            }, 2000);
        });
    }

    // Config params - READ ALL
    const cfgReadBtn = document.getElementById('cfg-params-read');
    if (cfgReadBtn) {
        cfgReadBtn.addEventListener('click', async () => {
            const progressEl = document.getElementById('cfg-params-progress');
            if (progressEl) progressEl.style.display = 'flex';
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
        }

        // Update extended tuning sliders if they match
        updateExtTuningSlider(data.paramId, data.paramValue);

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

    const statusEl = document.getElementById('rtk-conn-status');

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

    // Connect
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

    // Disconnect
    const disconnectBtn = document.getElementById('rtk-disconnect-btn');
    if (disconnectBtn) {
        disconnectBtn.addEventListener('click', async () => {
            await window.rtk.disconnect();
            if (statusEl) statusEl.textContent = 'Disconnected';
        });
    }

    // RTCM injection is handled directly in the main process (rtk-manager.js)
    // via raw MAVLink GPS_RTCM_DATA packets sent over the active connection.

    // Status updates from main process
    if (window.rtk.onStatusUpdate) {
        window.rtk.onStatusUpdate((data) => {
            STATE.rtkBaseConnected = data.connected;
            STATE.rtkBaseMsgPerSec = data.rtcmMsgPerSec || 0;

            // Connection status
            if (statusEl) {
                statusEl.textContent = data.connected ? `Connected — ${data.portPath}` : 'Not connected';
            }

            // Stream info
            const streamEl = document.getElementById('rtk-stream-status');
            if (streamEl) {
                streamEl.textContent = data.connected
                    ? (data.rtcmMsgPerSec > 0 ? 'Streaming RTCM3' : 'Connected, waiting for data...')
                    : 'No data';
                streamEl.style.color = data.rtcmMsgPerSec > 0 ? '#00ff7f' : '';
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
    let isConnected = false;

    // DOM elements
    const portSelect = document.getElementById('telfwd-serial-port');
    const baudSelect = document.getElementById('telfwd-baud');
    const protoSelect = document.getElementById('telfwd-protocol');
    const connectBtn = document.getElementById('telfwd-connect-btn');
    const disconnectBtn = document.getElementById('telfwd-disconnect-btn');
    const connStatus = document.getElementById('telfwd-conn-status');
    const streamStatus = document.getElementById('telfwd-stream-status');
    const msgRate = document.getElementById('telfwd-msg-rate');
    const msgTotal = document.getElementById('telfwd-msg-total');
    const bytesTx = document.getElementById('telfwd-bytes-tx');
    const activeProto = document.getElementById('telfwd-active-protocol');
    const dispLat = document.getElementById('telfwd-lat');
    const dispLon = document.getElementById('telfwd-lon');
    const dispAlt = document.getElementById('telfwd-alt');
    const dispHdg = document.getElementById('telfwd-heading');
    const dispGs = document.getElementById('telfwd-gs');

    // Restore saved settings
    try {
        const saved = JSON.parse(localStorage.getItem(STORAGE_KEY));
        if (saved) {
            if (saved.baudRate && baudSelect) baudSelect.value = String(saved.baudRate);
            if (saved.protocol && protoSelect) protoSelect.value = saved.protocol;
        }
    } catch (e) { /* ignore */ }

    // Scan ports
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
        // Restore previous selection or saved port
        try {
            const saved = JSON.parse(localStorage.getItem(STORAGE_KEY));
            if (prev) portSelect.value = prev;
            else if (saved && saved.port) portSelect.value = saved.port;
        } catch (e) { /* ignore */ }
    }

    const scanBtn = document.getElementById('telfwd-scan-ports');
    if (scanBtn) {
        scanBtn.addEventListener('click', scanPorts);
        // Auto-scan on first load
        scanPorts();
    }

    // Connect
    if (connectBtn) {
        connectBtn.addEventListener('click', async () => {
            const port = portSelect ? portSelect.value : '';
            const baud = baudSelect ? parseInt(baudSelect.value) : 9600;
            const proto = protoSelect ? protoSelect.value : 'ltm';

            if (!port) {
                if (connStatus) connStatus.textContent = 'Select a port first';
                return;
            }

            try {
                connectBtn.disabled = true;
                if (connStatus) connStatus.textContent = 'Connecting...';
                await window.telForward.connect(port, baud, proto);

                // Save settings
                localStorage.setItem(STORAGE_KEY, JSON.stringify({ port, baudRate: baud, protocol: proto }));

                isConnected = true;
                setUIConnected(true);

                // Start feeding state for LTM mode
                if (proto === 'ltm') {
                    startStateFeed();
                }
                // Start display update
                startDisplayUpdate();
            } catch (e) {
                if (connStatus) connStatus.textContent = 'Error: ' + e.message;
                connectBtn.disabled = false;
            }
        });
    }

    // Disconnect
    if (disconnectBtn) {
        disconnectBtn.addEventListener('click', async () => {
            try {
                await window.telForward.disconnect();
            } catch (e) { /* ignore */ }
            isConnected = false;
            setUIConnected(false);
            stopStateFeed();
            stopDisplayUpdate();
        });
    }

    // UI state toggle
    function setUIConnected(connected) {
        if (connectBtn) connectBtn.disabled = connected;
        if (disconnectBtn) disconnectBtn.disabled = !connected;
        if (portSelect) portSelect.disabled = connected;
        if (baudSelect) baudSelect.disabled = connected;
        if (protoSelect) protoSelect.disabled = connected;
        if (connStatus) {
            connStatus.textContent = connected ? 'Connected' : 'Not connected';
            connStatus.style.color = connected ? 'var(--clr-cyan)' : 'var(--clr-txt-dim)';
        }
    }

    // Feed STATE to main process for LTM encoding
    function startStateFeed() {
        stopStateFeed();
        feedInterval = setInterval(() => {
            if (!isConnected) return;
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
        stopDisplayUpdate();
        displayInterval = setInterval(() => {
            if (!isConnected) return;
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

    // Listen for status updates from main process
    window.telForward.onStatusUpdate((data) => {
        if (streamStatus) streamStatus.textContent = data.connected ? 'Streaming' : 'Idle';
        if (msgRate) msgRate.textContent = data.msgPerSec + ' msg/s';
        if (msgTotal) msgTotal.textContent = data.msgCount.toLocaleString();
        if (bytesTx) bytesTx.textContent = formatBytes(data.bytesSent);
        if (activeProto) activeProto.textContent = data.protocol === 'mavlink' ? 'MAVLink' : 'LTM';

        // Detect disconnect from main process side
        if (!data.connected && isConnected) {
            isConnected = false;
            setUIConnected(false);
            stopStateFeed();
            stopDisplayUpdate();
        }
    });

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

// ============================================================
// SURVEY / GRID AUTO-PLANNING (polygon-based)
// ============================================================

/**
 * Add a vertex to the survey polygon being drawn on the map
 */
function addSurveyVertex(lat, lng) {
    if (!missionMap) return;
    surveyPolygonPoints.push([lat, lng]);

    // Add small marker for vertex
    const marker = L.circleMarker([lat, lng], {
        radius: 5, fillColor: '#ff6600', color: '#fff', weight: 2, fillOpacity: 0.9
    }).addTo(missionMap);
    surveyPolygonMarkers.push(marker);

    // Update polygon overlay
    if (surveyPolygonLayer) surveyPolygonLayer.remove();
    if (surveyPolygonPoints.length >= 2) {
        surveyPolygonLayer = L.polygon(surveyPolygonPoints, {
            color: '#ff6600', weight: 2, fillColor: '#ff6600', fillOpacity: 0.12, dashArray: '6 4'
        }).addTo(missionMap);
    }

    // Auto-close if user clicks near the first point (> 3 vertices)
    if (surveyPolygonPoints.length >= 4) {
        const first = surveyPolygonPoints[0];
        const distPx = missionMap.latLngToContainerPoint([lat, lng])
            .distanceTo(missionMap.latLngToContainerPoint(first));
        if (distPx < 15) {
            surveyPolygonPoints.pop(); // remove duplicate close point
            surveyPolygonMarkers.pop().remove();
            finishSurveyPolygon();
        }
    }
}

/**
 * Finish drawing the survey polygon
 */
function finishSurveyPolygon() {
    if (surveyPolygonPoints.length < 3 || !missionMap) return;
    surveyDrawMode = false;
    missionMap.doubleClickZoom.enable();
    const toggleBtn = document.getElementById('mission-survey-toggle');
    if (toggleBtn) toggleBtn.classList.remove('active');

    // Remove vertex markers (keep only the solid polygon)
    surveyPolygonMarkers.forEach(m => m.remove());
    surveyPolygonMarkers = [];

    // Solidify the polygon
    if (surveyPolygonLayer) surveyPolygonLayer.remove();
    surveyPolygonLayer = L.polygon(surveyPolygonPoints, {
        color: '#ff6600', weight: 2, fillColor: '#ff6600', fillOpacity: 0.15
    }).addTo(missionMap);

    const infoEl = document.getElementById('survey-info');
    if (infoEl) infoEl.textContent = `Polygon: ${surveyPolygonPoints.length} vertices`;
}

/**
 * Clear survey drawing state
 */
function clearSurveyDraw() {
    surveyPolygonPoints = [];
    surveyPolygonMarkers.forEach(m => m.remove());
    surveyPolygonMarkers = [];
    if (surveyPolygonLayer) { surveyPolygonLayer.remove(); surveyPolygonLayer = null; }
    if (surveyPreviewLayer) { surveyPreviewLayer.remove(); surveyPreviewLayer = null; }
    const infoEl = document.getElementById('survey-info');
    if (infoEl) infoEl.textContent = '';
}

/**
 * Point-in-polygon test (ray casting)
 */
function pointInPolygon(lat, lng, polygon) {
    let inside = false;
    for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
        const yi = polygon[i][0], xi = polygon[i][1];
        const yj = polygon[j][0], xj = polygon[j][1];
        if (((yi > lat) !== (yj > lat)) && (lng < (xj - xi) * (lat - yi) / (yj - yi) + xi)) {
            inside = !inside;
        }
    }
    return inside;
}

/**
 * Generate a lawn-mower survey grid inside the drawn polygon.
 * Clips grid lines to polygon edges for proper coverage.
 */
function generateSurveyFromPolygon(spacing, angle, alt) {
    const poly = surveyPolygonPoints;
    if (poly.length < 3) return;

    // Bounding box
    const lats = poly.map(p => p[0]);
    const lngs = poly.map(p => p[1]);
    const minLat = Math.min(...lats), maxLat = Math.max(...lats);
    const minLng = Math.min(...lngs), maxLng = Math.max(...lngs);
    const centerLat = (minLat + maxLat) / 2;

    const latSpacing = spacing / 111320;
    const lngSpacing = spacing / (111320 * Math.cos(centerLat * Math.PI / 180));

    // Expand bounding box slightly for clipping margin
    const margin = Math.max(latSpacing, lngSpacing) * 0.5;

    // Append after existing waypoints (don't clear current mission)
    let seq = STATE.missionItems.length;

    if (Math.abs(angle) < 45 || Math.abs(angle) > 135) {
        // Sweep along longitude lines (N-S)
        const steps = Math.ceil((maxLng - minLng) / lngSpacing);
        for (let i = 0; i <= steps; i++) {
            const lng = minLng + i * lngSpacing;
            // Find polygon intersections along this longitude line
            const crossings = [];
            for (let j = 0, k = poly.length - 1; j < poly.length; k = j++) {
                const x1 = poly[k][1], y1 = poly[k][0];
                const x2 = poly[j][1], y2 = poly[j][0];
                if ((x1 <= lng && x2 > lng) || (x2 <= lng && x1 > lng)) {
                    const lat = y1 + (lng - x1) * (y2 - y1) / (x2 - x1);
                    crossings.push(lat);
                }
            }
            crossings.sort((a, b) => a - b);
            // Take pairs of crossings as line segments inside polygon
            for (let c = 0; c + 1 < crossings.length; c += 2) {
                const startLat = i % 2 === 0 ? crossings[c] : crossings[c + 1];
                const endLat   = i % 2 === 0 ? crossings[c + 1] : crossings[c];
                STATE.missionItems.push({ seq: seq++, command: 16, lat: startLat, lng, alt, frame: 3, param1: 0, param2: 0, param3: 0, param4: 0 });
                STATE.missionItems.push({ seq: seq++, command: 16, lat: endLat,   lng, alt, frame: 3, param1: 0, param2: 0, param3: 0, param4: 0 });
            }
        }
    } else {
        // Sweep along latitude lines (E-W)
        const steps = Math.ceil((maxLat - minLat) / latSpacing);
        for (let i = 0; i <= steps; i++) {
            const lat = minLat + i * latSpacing;
            const crossings = [];
            for (let j = 0, k = poly.length - 1; j < poly.length; k = j++) {
                const y1 = poly[k][0], x1 = poly[k][1];
                const y2 = poly[j][0], x2 = poly[j][1];
                if ((y1 <= lat && y2 > lat) || (y2 <= lat && y1 > lat)) {
                    const lng = x1 + (lat - y1) * (x2 - x1) / (y2 - y1);
                    crossings.push(lng);
                }
            }
            crossings.sort((a, b) => a - b);
            for (let c = 0; c + 1 < crossings.length; c += 2) {
                const startLng = i % 2 === 0 ? crossings[c] : crossings[c + 1];
                const endLng   = i % 2 === 0 ? crossings[c + 1] : crossings[c];
                STATE.missionItems.push({ seq: seq++, command: 16, lat, lng: startLng, alt, frame: 3, param1: 0, param2: 0, param3: 0, param4: 0 });
                STATE.missionItems.push({ seq: seq++, command: 16, lat, lng: endLng,   alt, frame: 3, param1: 0, param2: 0, param3: 0, param4: 0 });
            }
        }
    }

    return seq;
}

/**
 * Compute the footprint width from altitude and FOV:
 *   footprint = 2 * alt * tan(fov/2)
 * Relationship: spacing = footprint * (1 - overlap/100)
 *
 * The parameter left empty (or last cleared) is auto-calculated from the others.
 * Priority: altitude, fov, overlap are "primary"; spacing is derived by default.
 * If the user clears any one field, that becomes the auto-calculated one.
 */
function surveyAutoCalc(changedId) {
    const altEl = document.getElementById('survey-alt');
    const fovEl = document.getElementById('survey-fov');
    const overlapEl = document.getElementById('survey-overlap');
    const spacingEl = document.getElementById('survey-spacing');

    const alt = parseFloat(altEl?.value);
    const fov = parseFloat(fovEl?.value);
    const overlap = parseFloat(overlapEl?.value);
    const spacing = parseFloat(spacingEl?.value);

    const hasAlt = !isNaN(alt) && altEl?.value !== '';
    const hasFov = !isNaN(fov) && fovEl?.value !== '';
    const hasOverlap = !isNaN(overlap) && overlapEl?.value !== '';
    const hasSpacing = !isNaN(spacing) && spacingEl?.value !== '';

    const filled = [hasAlt, hasFov, hasOverlap, hasSpacing].filter(Boolean).length;
    if (filled < 3) return; // need at least 3 to solve the 4th

    if (hasAlt && hasFov && hasOverlap && !hasSpacing) {
        // Solve spacing
        const footprint = 2 * alt * Math.tan((fov * Math.PI / 180) / 2);
        const s = footprint * (1 - overlap / 100);
        spacingEl.value = Math.round(s * 10) / 10;
        spacingEl.placeholder = '';
    } else if (hasAlt && hasFov && hasSpacing && !hasOverlap) {
        // Solve overlap
        const footprint = 2 * alt * Math.tan((fov * Math.PI / 180) / 2);
        if (footprint > 0) {
            const o = (1 - spacing / footprint) * 100;
            overlapEl.value = Math.round(o);
        }
    } else if (hasAlt && hasOverlap && hasSpacing && !hasFov) {
        // Solve FOV: footprint = spacing / (1 - overlap/100), fov = 2*atan(footprint/(2*alt))
        const footprint = spacing / (1 - overlap / 100);
        if (alt > 0) {
            const f = 2 * Math.atan(footprint / (2 * alt)) * 180 / Math.PI;
            fovEl.value = Math.round(f);
        }
    } else if (hasFov && hasOverlap && hasSpacing && !hasAlt) {
        // Solve altitude: footprint = spacing / (1 - overlap/100), alt = footprint / (2*tan(fov/2))
        const footprint = spacing / (1 - overlap / 100);
        const a = footprint / (2 * Math.tan((fov * Math.PI / 180) / 2));
        altEl.value = Math.round(a);
    } else if (filled === 4 && changedId) {
        // All 4 filled and user changed one — recalculate spacing (the derived param)
        // unless user just changed spacing, in which case recalculate overlap
        if (changedId === 'survey-spacing') {
            const footprint = 2 * alt * Math.tan((fov * Math.PI / 180) / 2);
            if (footprint > 0) {
                const o = (1 - spacing / footprint) * 100;
                overlapEl.value = Math.round(o);
            }
        } else {
            const footprint = 2 * alt * Math.tan((fov * Math.PI / 180) / 2);
            const s = footprint * (1 - overlap / 100);
            spacingEl.value = Math.round(s * 10) / 10;
        }
    }
}

export function initSurveyGrid() {
    // Wire up auto-calculation on all survey param inputs
    const paramIds = ['survey-alt', 'survey-fov', 'survey-overlap', 'survey-spacing'];
    for (const id of paramIds) {
        const el = document.getElementById(id);
        if (el) {
            el.addEventListener('input', () => surveyAutoCalc(id));
        }
    }
    // Initial auto-calc (spacing from defaults)
    surveyAutoCalc(null);

    bindBtn('survey-generate', () => {
        if (surveyPolygonPoints.length < 3) {
            alert('Draw a survey area first using the survey tool on the map toolbar');
            return;
        }
        // Ensure auto-calc ran
        surveyAutoCalc(null);
        const spacing = parseFloat(document.getElementById('survey-spacing')?.value) || 30;
        const angle = parseFloat(document.getElementById('survey-angle')?.value) || 0;
        const alt = parseFloat(document.getElementById('survey-alt')?.value) || 50;
        console.log('[Survey] Generating grid:', { vertices: surveyPolygonPoints.length, spacing, angle, alt, poly: surveyPolygonPoints });
        const countBefore = STATE.missionItems.length;
        generateSurveyFromPolygon(spacing, angle, alt);
        const added = STATE.missionItems.length - countBefore;
        console.log('[Survey] Added', added, 'survey waypoints (total:', STATE.missionItems.length, ')');
        const infoEl = document.getElementById('survey-info');
        if (added === 0) {
            if (infoEl) infoEl.textContent = 'No waypoints generated — try smaller spacing';
        } else {
            if (infoEl) infoEl.textContent = `Added ${added} survey WPs (total: ${STATE.missionItems.length})`;
        }
        updateMissionDisplay();
    });

    bindBtn('survey-clear', () => {
        clearSurveyDraw();
        // Also clear generated waypoints
        STATE.missionItems.length = 0;
        updateMissionDisplay();
    });
}

/* ═══════════════════════════════════════════════════════════════
   CORV SETUP TAB — Config protocol (0x10/0x11) for CORV INS
═══════════════════════════════════════════════════════════════ */
function initCorvSetupTab() {
    if (!window.corvSerial) return;

    const CFG_STRUCT_SIZE = 106;
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

    // --- Build 106-byte SystemConfig struct from UI (protocol v7) ---
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

        return new Uint8Array(buf);
    }

    // --- Parse 106-byte SystemConfig struct into UI (protocol v7) ---
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
