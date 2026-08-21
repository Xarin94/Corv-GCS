/**
 * main.js - Application Entry Point
 * Initializes all modules and runs the main animation loop
 */

// Core imports
import {
    ORIGIN, CAMERA_FOV, VISIBILITY_RADIUS, RELOAD_DISTANCE, RAD,
    DEMO_CRUISE_SPEED, DEMO_SPEED_VARIANCE, DEMO_CRUISE_AGL, DEMO_MIN_CLEARANCE,
    DEMO_MAX_VS, DEMO_MAX_BANK, DEMO_ROLL_RATE, DEMO_HDG_GAIN,
    DEMO_STRAIGHT, DEMO_TURN_RADIUS, DEMO_CAPTURE_R
} from './core/constants.js';
import { STATE, demoFlightState, pushGHistory } from './core/state.js';
import { latLonToMeters, calculateDistance, lerpColor, getHeightColor } from './core/utils.js';
import { fetchADSBData, downloadTrafficCSV, getNearestTraffic } from './adsb/ADSBManager.js';

// Engine imports
import { 
    init3D, updateTrail, updateCamera, render, resize,
    resetTrail, setTrailPoints,
    updateMissionTrajectory, clearMissionTrajectory,
    getScene, getCamera, getRenderer, getSunLight, getAmbientLight,
    getCurrentSunDirection, isSunlightEnabled, setSunlightEnabled,
    getTimeOverride, setTimeOverride, getShadowChunkSize,
    getLastShadowChunk, setLastShadowChunk,
    updateHomeMarker3D,
    updateTrafficMarkers3D,
    updateTargetMarker3D
} from './engine/Scene3D.js';

// Trajectory corridor imports
import { computePredictedPath } from './engine/TrajectoryPredictor.js';
import { initCorridor, updateCorridor, setCorridorVisible, getPredictionTime } from './engine/TrajectoryCorridor3D.js';

// Terrain imports
import {
    initTerrain, getTerrainElevationCached, updateTerrainChunks,
    updateTerrainHillshading, setHillshadeNeedsUpdate,
    addHGTFile, getHGTFileCount, getActiveChunks, setAvailableHgtFiles,
    getChunkCreationQueue, getTileLoadQueue, getCurrentTileLoads,
    getTotalTilesToLoad, getTilesLoaded,
    getTerrainElevationFromHGT, getRunwayObjects,
    refreshNearbyChunkTextures, resetTextureRefreshPosition,
    setMapBrightness,
    getMemoryStats,
    updateWireframeProximity
} from './terrain/TerrainManager.js';

// HUD imports
import { initHUD, drawHUD, resizeHUD, pushHudMessage, setHudPitchLocked, updateArmStateUI } from './hud/HUDRenderer.js';

// Map imports
import { initMap, updateMap, invalidateSize as invalidateMapSize, updateMissionOverlay, updateTrafficOverlay, resetMapTrail } from './maps/MapEngine.js';

// Serial imports
import { connectSerial } from './serial/SerialHandler.js';

// TLOG Logger import (replaces CRVLogger)
import { TlogLogger } from './logging/TlogLogger.js';

// UI imports
import { initRotorLoadPanel, updateRotorLoadPanel } from './ui/RotorLoadPanel.js';
import { updateUI, toggleConfig, toggleTelemetry, updateOffset, updateAGLDisplay, setStatusMessage, updateFPSDisplay, initMoreMenu, initConfigAutoClose, initHudCells } from './ui/UIController.js';

// Split view imports

// MAVLink imports
import { initMAVLink, onMessage } from './mavlink/MAVLinkManager.js';
import { setParameter, requestParameter } from './mavlink/CommandSender.js';
import { initTerrainFeeder } from './mavlink/TerrainFeeder.js';
import { computeAeroAngles } from './mavlink/MAVLinkStateMapper.js';

// GCS imports
import { initCommandBar, updateCommandBar } from './ui/CommandBarController.js';
import { initGCSSidebar, updateGCSSidebar, getTargetCoords } from './ui/GCSSidebarController.js';
import { initLogReplay } from './logging/LogReplayController.js';
import { initTabs, getCurrentTab } from './ui/TabController.js';
import { initParamsPage, toggleParamsPage } from './ui/ParametersPageController.js';

import { setTerrainSatelliteEnabled } from './terrain/TerrainManager.js';
import { initOfflinePanel } from './maps/OfflineDownloader.js';

// FPV imports
import { initFPV, onFPVButtonClick, saveFPVSettings, resizeFPV, isFPVARMode, stopFPVStream } from './ui/FPVController.js';

// Loading overlay imports
import {
    showLoadingOverlay, hideLoadingOverlay, scheduleHideLoadingOverlaySoon,
    checkInitialLoadComplete, setAutoLoadAttempted
} from './ui/LoadingOverlay.js';


// ============== CAMERA / MODEL (1P / 3P) ==============
let cameraMode = 'FIRST'; // 'FIRST' | 'THIRD'
// Horizon-lock: holds the first-person camera pitch at zero so the horizon
// stays in frame regardless of the drone's actual attitude. The HUD's
// boresight/aircraft-reference symbol becomes the mobile element in this
// mode (see setHudPitchLocked in HUDRenderer.js).
let horizonLocked = false;
let vehicle = null;
let vehicleLoadStarted = false;
let vehicleLoadFailed = false;
let currentModelName = '';
let modelScale = 1.0;
let loadedModel = null; // Reference to the currently loaded 3D model

// Every model in models/ is generated at true real-world scale by
// scripts/gen-models.js, so the scale slider is a pure preference and no model
// needs a normalization pass. Fixed wing is what we show until told otherwise.
const DEFAULT_MODEL_NAME = 'plane.glb';

// MAV_TYPE (HEARTBEAT.type) → model file. The airframe you are actually flying
// should be the one on screen, so the model follows the heartbeat instead of
// whatever was picked last session. A manual pick from the dropdown sticks
// until the vehicle type itself changes.
const MODEL_BY_MAV_TYPE = {
    1: 'plane.glb',             // FIXED_WING
    2: 'quadcopter.glb',        // QUADROTOR
    3: 'helicopter.glb',        // COAXIAL
    4: 'helicopter.glb',        // HELICOPTER
    5: 'antenna-tracker.glb',   // ANTENNA_TRACKER
    7: 'airship.glb',           // AIRSHIP
    8: 'airship.glb',           // FREE_BALLOON
    10: 'rover.glb',            // GROUND_ROVER
    11: 'boat.glb',             // SURFACE_BOAT
    12: 'submarine.glb',        // SUBMARINE
    13: 'hexacopter.glb',       // HEXAROTOR
    14: 'octocopter.glb',       // OCTOROTOR
    15: 'tricopter.glb',        // TRICOPTER
    16: 'flying-wing.glb',      // FLAPPING_WING — the delta wings people fly
    19: 'quadplane-vtol.glb',   // VTOL_TAILSITTER_DUOROTOR
    20: 'quadplane-vtol.glb',   // VTOL_TAILSITTER_QUADROTOR
    21: 'quadplane-vtol.glb',   // VTOL_TILTROTOR
    22: 'quadplane-vtol.glb',   // VTOL_FIXEDROTOR
    23: 'quadplane-vtol.glb',   // VTOL_TAILSITTER
    24: 'quadplane-vtol.glb',   // VTOL_TILTWING
    25: 'quadplane-vtol.glb',   // VTOL_RESERVED5
    27: 'octocopter.glb',       // DECAROTOR
    29: 'octocopter.glb',       // DODECAROTOR
};
let lastModelVehicleType = 0;

const orbit = {
    yaw: 0,
    pitch: 0.35,
    distance: 350,
    height: 40,
    minPitch: -1.2,
    maxPitch: 1.2,
    minDistance: 50,
    maxDistance: 2500,
    rotateSpeed: 0.005,
    zoomSpeed: 0.12
};

let isOrbitDragging = false;
let orbitLastX = 0;
let orbitLastY = 0;

// Smoothed attitude: time-based interpolation between MAVLink samples
const smoothAtt = { roll: 0, pitch: 0, yaw: 0 };
let ATT_SMOOTH = 0.15; // slider control: 0 = raw, higher = more smoothing

// Previous and current attitude samples for interpolation
const attPrev = { roll: 0, pitch: 0, yaw: 0, time: 0 };
const attCurr = { roll: 0, pitch: 0, yaw: 0, time: 0 };
let lastStateRoll = NaN, lastStatePitch = NaN, lastStateYaw = NaN;

function lerpAngle(a, b, t) {
    let diff = b - a;
    if (diff > Math.PI) diff -= 2 * Math.PI;
    if (diff < -Math.PI) diff += 2 * Math.PI;
    return a + diff * t;
}

function updateSmoothedAttitude() {
    const now = performance.now();

    // Detect new attitude sample from MAVLink (STATE changed since last frame)
    if (STATE.roll !== lastStateRoll || STATE.pitch !== lastStatePitch || STATE.yaw !== lastStateYaw) {
        attPrev.roll = attCurr.roll;
        attPrev.pitch = attCurr.pitch;
        attPrev.yaw = attCurr.yaw;
        attPrev.time = attCurr.time;

        attCurr.roll = STATE.roll;
        attCurr.pitch = STATE.pitch;
        attCurr.yaw = STATE.yaw;
        attCurr.time = now;

        lastStateRoll = STATE.roll;
        lastStatePitch = STATE.pitch;
        lastStateYaw = STATE.yaw;
    }

    // Interpolate between previous and current sample
    const dt = attCurr.time - attPrev.time;
    if (dt > 0 && ATT_SMOOTH > 0) {
        // How far we are from the current sample arrival, normalized to sample interval
        const elapsed = now - attCurr.time;
        // t = 0 at prev sample time, t = 1 at current sample time
        // We extrapolate slightly beyond 1.0 to stay responsive
        const t = Math.min((elapsed + dt) / dt, 1.5);
        const tClamped = Math.max(0, Math.min(t, 1.0));

        // Blend: at ATT_SMOOTH=0 use raw, at ATT_SMOOTH=0.5 fully interpolated
        const blend = Math.min(ATT_SMOOTH * 2, 1.0);
        const interpRoll = attPrev.roll + (attCurr.roll - attPrev.roll) * tClamped;
        const interpPitch = attPrev.pitch + (attCurr.pitch - attPrev.pitch) * tClamped;
        const interpYaw = lerpAngle(attPrev.yaw, attCurr.yaw, tClamped);

        smoothAtt.roll = interpRoll * blend + STATE.roll * (1 - blend);
        smoothAtt.pitch = interpPitch * blend + STATE.pitch * (1 - blend);
        smoothAtt.yaw = lerpAngle(STATE.yaw, interpYaw, blend);
    } else {
        smoothAtt.roll = STATE.roll;
        smoothAtt.pitch = STATE.pitch;
        smoothAtt.yaw = STATE.yaw;
    }
}

function initVehicle() {
    const scene = getScene();
    if (!scene || vehicle) return;

    // Root wrapper we can always position/rotate from STATE.
    vehicle = new THREE.Group();
    vehicle.visible = false;
    scene.add(vehicle);

    // Try to load model list and pick first available
    loadModelList();
}

/**
 * Load the list of available models and populate the dropdown
 */
async function loadModelList() {
    const select = document.getElementById('model-select');
    if (!select) return;

    try {
        const models = window.models ? await window.models.list() : [];
        select.innerHTML = '';
        
        if (models.length === 0) {
            select.innerHTML = '<option value="">No models found</option>';
            // Fall back to placeholder
            createPlaceholderModel();
            return;
        }

        models.forEach((m, i) => {
            const opt = document.createElement('option');
            opt.value = m;
            opt.textContent = m.replace(/\.(glb|gltf)$/i, '');
            select.appendChild(opt);
        });

        // Load the first model by default (or saved preference)
        const savedModel = localStorage.getItem('selectedModel');
        const savedScale = localStorage.getItem('modelScale');
        
        if (savedScale) {
            modelScale = parseFloat(savedScale) || 1.0;
            const slider = document.getElementById('scale-slider');
            const display = document.getElementById('scale-display');
            if (slider) slider.value = modelScale;
            if (display) display.textContent = modelScale.toFixed(1);
        }

        const initial = (savedModel && models.includes(savedModel)) ? savedModel
            : (models.includes(DEFAULT_MODEL_NAME) ? DEFAULT_MODEL_NAME : models[0]);
        select.value = initial;
        loadModel(initial);
    } catch (e) {
        console.error('Failed to load model list:', e);
        select.innerHTML = '<option value="">Error loading models</option>';
        createPlaceholderModel();
    }
}

/**
 * Load a specific 3D model from the models folder
 */
async function loadModel(filename) {
    if (!filename || !vehicle) return;
    
    // Remove previous model if any
    if (loadedModel) {
        vehicle.remove(loadedModel);
        loadedModel = null;
    }
    
    currentModelName = filename;
    localStorage.setItem('selectedModel', filename);

    try {
        if (!window.models || !THREE.GLTFLoader) {
            console.warn('Models API or GLTFLoader not available');
            createPlaceholderModel();
            return;
        }

        const arrayBuffer = await window.models.load(filename);
        if (!arrayBuffer) {
            console.error('Failed to load model:', filename);
            createPlaceholderModel();
            return;
        }

        const loader = new THREE.GLTFLoader();
        loader.parse(arrayBuffer, '', (gltf) => {
            if (!vehicle) return;
            
            const model = gltf.scene || gltf.scenes?.[0];
            if (!model) {
                console.error('No scene in GLTF');
                createPlaceholderModel();
                return;
            }

            model.traverse((obj) => {
                if (obj && obj.isMesh) {
                    obj.castShadow = true;
                    obj.receiveShadow = true;
                }
            });

            // Adjustments
            model.position.set(0, 0, 0);
            model.rotation.set(0, Math.PI / 2, 0);

            model.scale.set(modelScale, modelScale, modelScale);

            // Recenter pivot
            try {
                model.updateMatrixWorld(true);
                const box = new THREE.Box3().setFromObject(model);
                const center = new THREE.Vector3();
                box.getCenter(center);
                model.position.sub(center);
            } catch (e) {}

            // Remove old model and add new
            if (loadedModel) {
                vehicle.remove(loadedModel);
            }
            loadedModel = model;
            vehicle.add(model);
            
            console.log('Model loaded:', filename);
        }, (error) => {
            console.error('Failed to parse GLTF:', error);
            createPlaceholderModel();
        });
    } catch (e) {
        console.error('Error loading model:', e);
        createPlaceholderModel();
    }
}

/**
 * Follow the vehicle type announced by HEARTBEAT: swap the 3D model whenever
 * the type changes to one we have an airframe for. Driven from the render loop
 * — it is a couple of integer compares per frame and needs no extra plumbing
 * through the MAVLink layer.
 */
function syncModelToVehicleType() {
    const type = STATE.vehicleType;
    if (!type || type === lastModelVehicleType) return;
    lastModelVehicleType = type;

    const wanted = MODEL_BY_MAV_TYPE[type];
    if (!wanted || wanted === currentModelName) return;

    const select = document.getElementById('model-select');
    if (!select) return;
    const opt = Array.from(select.options).find(o => o.value.toLowerCase() === wanted);
    if (!opt) return;  // model file missing from models/

    select.value = opt.value;
    loadModel(opt.value);
}

/**
 * Update the scale of the current model
 */
function updateModelScale(scale) {
    modelScale = scale;
    localStorage.setItem('modelScale', scale.toString());
    if (loadedModel) loadedModel.scale.set(scale, scale, scale);
}

/**
 * Create a placeholder model when no GLB is available
 */
function createPlaceholderModel() {
    if (!vehicle || loadedModel) return;

    const bodyMat = new THREE.MeshStandardMaterial({ color: 0xdddddd, roughness: 0.6, metalness: 0.1 });
    const darkMat = new THREE.MeshStandardMaterial({ color: 0x222222, roughness: 0.8, metalness: 0.0 });

    const fuselage = new THREE.Mesh(new THREE.CylinderGeometry(1.2, 1.6, 10, 12), bodyMat);
    fuselage.rotation.z = Math.PI / 2;
    fuselage.castShadow = true;
    fuselage.receiveShadow = true;

    const nose = new THREE.Mesh(new THREE.ConeGeometry(1.2, 3, 12), bodyMat);
    nose.position.x = 6.3;
    nose.rotation.z = -Math.PI / 2;
    nose.castShadow = true;
    nose.receiveShadow = true;

    const wing = new THREE.Mesh(new THREE.BoxGeometry(10, 0.25, 2.2), darkMat);
    wing.position.x = -0.5;
    wing.castShadow = true;
    wing.receiveShadow = true;

    const tail = new THREE.Mesh(new THREE.BoxGeometry(3, 0.2, 1.2), darkMat);
    tail.position.x = -5.0;
    tail.position.y = 0.2;
    tail.castShadow = true;
    tail.receiveShadow = true;

    const fin = new THREE.Mesh(new THREE.BoxGeometry(0.2, 1.6, 1.0), darkMat);
    fin.position.x = -5.3;
    fin.position.y = 0.9;
    fin.castShadow = true;
    fin.receiveShadow = true;

    vehicle.add(fuselage, nose, wing, tail, fin);
}

function setCameraMode(mode) {
    cameraMode = mode === 'THIRD' ? 'THIRD' : 'FIRST';

    const btn = document.getElementById('btn-cam');
    if (btn) {
        if (cameraMode === 'THIRD') {
            btn.classList.add('active');
            btn.textContent = '3P';
        } else {
            btn.classList.remove('active');
            btn.textContent = '1P';
        }
    }

    if (vehicle) {
        vehicle.visible = (cameraMode === 'THIRD');
    }

    // Engage/disengage the shadow pass immediately on camera mode change
    const sl = getSunLight();
    if (sl) sl.castShadow = (cameraMode === 'THIRD') && isSunlightEnabled();

    // Hide HUD overlay in 3rd person to avoid clutter.
    const hudCanvas = document.getElementById('hud-canvas');
    if (hudCanvas) {
        hudCanvas.style.display = (cameraMode === 'THIRD') ? 'none' : 'block';
    }

    // When entering 3rd person, start behind the vehicle.
    if (cameraMode === 'THIRD') {
        orbit.yaw = (-STATE.yaw) + Math.PI;
        orbit.pitch = 0.25;
    }
}

function toggleCameraMode() {
    setCameraMode(cameraMode === 'FIRST' ? 'THIRD' : 'FIRST');
}

function toggleHorizonLock() {
    horizonLocked = !horizonLocked;
    setHudPitchLocked(horizonLocked); // boresight becomes mobile, ladder stays put
    const btn = document.getElementById('btn-tilt');
    if (btn) btn.classList.toggle('active', horizonLocked);
}

function initThirdPersonControls() {
    const renderer = getRenderer();
    if (!renderer) return;

    const el = renderer.domElement;
    el.style.touchAction = 'none';

    el.addEventListener('pointerdown', (e) => {
        if (cameraMode !== 'THIRD') return;
        if (e.button !== 0) return;
        isOrbitDragging = true;
        orbitLastX = e.clientX;
        orbitLastY = e.clientY;
        try { el.setPointerCapture(e.pointerId); } catch (_) {}
    });

    el.addEventListener('pointermove', (e) => {
        if (cameraMode !== 'THIRD') return;
        if (!isOrbitDragging) return;

        const dx = e.clientX - orbitLastX;
        const dy = e.clientY - orbitLastY;
        orbitLastX = e.clientX;
        orbitLastY = e.clientY;

        orbit.yaw -= dx * orbit.rotateSpeed;
        orbit.pitch -= dy * orbit.rotateSpeed;
        orbit.pitch = Math.max(orbit.minPitch, Math.min(orbit.maxPitch, orbit.pitch));
    });

    el.addEventListener('pointerup', (e) => {
        if (e.button !== 0) return;
        isOrbitDragging = false;
        try { el.releasePointerCapture(e.pointerId); } catch (_) {}
    });

    el.addEventListener('pointercancel', () => {
        isOrbitDragging = false;
    });

    el.addEventListener('wheel', (e) => {
        if (cameraMode !== 'THIRD') return;
        e.preventDefault();
        const delta = Math.sign(e.deltaY);
        const factor = 1 + delta * orbit.zoomSpeed;
        orbit.distance *= factor;
        orbit.distance = Math.max(orbit.minDistance, Math.min(orbit.maxDistance, orbit.distance));
    }, { passive: false });
}

// Expose for HTML onclick
window.toggleCameraMode = toggleCameraMode;
window.toggleHorizonLock = toggleHorizonLock;

// ============== SUN POSITION CALCULATOR ==============
function calculateSunPosition(date, lat, lon) {
    const JD = Math.floor(365.25 * (date.getFullYear() + 4716)) + 
               Math.floor(30.6001 * ((date.getMonth() + 1 < 3 ? date.getMonth() + 13 : date.getMonth() + 1))) + 
               date.getDate() - 1524.5;
    
    const hours = date.getUTCHours() + date.getUTCMinutes() / 60 + date.getUTCSeconds() / 3600;
    const JDfrac = JD + hours / 24;
    const T = (JDfrac - 2451545.0) / 36525.0;
    
    let L0 = 280.46646 + T * (36000.76983 + T * 0.0003032);
    L0 = L0 % 360;
    
    let M = 357.52911 + T * (35999.05029 - T * 0.0001537);
    M = M % 360;
    const Mrad = M * Math.PI / 180;
    
    const e = 0.016708634 - T * (0.000042037 + T * 0.0000001267);
    const C = (1.914602 - T * (0.004817 + T * 0.000014)) * Math.sin(Mrad) +
              (0.019993 - T * 0.000101) * Math.sin(2 * Mrad) +
              0.000289 * Math.sin(3 * Mrad);
    
    const sunLon = L0 + C;
    const omega = 125.04 - 1934.136 * T;
    const lambda = sunLon - 0.00569 - 0.00478 * Math.sin(omega * Math.PI / 180);
    const lambdaRad = lambda * Math.PI / 180;
    
    const epsilon0 = 23.439291 - T * (0.0130042 + T * (0.00000016 - T * 0.000000504));
    const epsilon = epsilon0 + 0.00256 * Math.cos(omega * Math.PI / 180);
    const epsilonRad = epsilon * Math.PI / 180;
    
    const sinDec = Math.sin(epsilonRad) * Math.sin(lambdaRad);
    const declination = Math.asin(sinDec);
    
    const y = Math.tan(epsilonRad / 2) ** 2;
    const L0rad = L0 * Math.PI / 180;
    const EoT = 4 * (180 / Math.PI) * (y * Math.sin(2 * L0rad) - 
                2 * e * Math.sin(Mrad) + 
                4 * e * y * Math.sin(Mrad) * Math.cos(2 * L0rad) -
                0.5 * y * y * Math.sin(4 * L0rad) - 
                1.25 * e * e * Math.sin(2 * Mrad));
    
    const timeOffset = EoT + 4 * lon;
    const trueSolarTime = hours * 60 + timeOffset;
    let hourAngle = trueSolarTime / 4 - 180;
    const haRad = hourAngle * Math.PI / 180;
    
    const latRad = lat * Math.PI / 180;
    const cosZenith = Math.sin(latRad) * Math.sin(declination) + 
                      Math.cos(latRad) * Math.cos(declination) * Math.cos(haRad);
    const zenith = Math.acos(Math.max(-1, Math.min(1, cosZenith)));
    const altitude = Math.PI / 2 - zenith;
    
    const sinAz = -Math.cos(declination) * Math.sin(haRad) / Math.sin(zenith);
    const cosAz = (Math.sin(declination) - Math.sin(latRad) * cosZenith) / 
                  (Math.cos(latRad) * Math.sin(zenith));
    
    let azimuth = Math.atan2(sinAz, cosAz);
    if (azimuth < 0) azimuth += 2 * Math.PI;
    
    return { altitude, azimuth, declination: declination * 180 / Math.PI };
}

// ============== SUN POSITION UPDATE ==============
function updateSunPosition() {
    const sunLight = getSunLight();
    const ambientLight = getAmbientLight();
    const camera = getCamera();
    const scene = getScene();
    const currentSunDirection = getCurrentSunDirection();
    
    if (!sunLight || !isSunlightEnabled()) return;

    // Shadows only in 3rd person: the vehicle is the sole shadow caster and is
    // invisible in 1st person — skip the shadow pass + per-fragment PCF there.
    sunLight.castShadow = (cameraMode === 'THIRD');
    ambientLight.intensity = 0.6;
    
    const SHADOW_CHUNK_SIZE = getShadowChunkSize();
    const currentChunkX = Math.floor(camera.position.x / SHADOW_CHUNK_SIZE);
    const currentChunkZ = Math.floor(camera.position.z / SHADOW_CHUNK_SIZE);
    const lastChunk = getLastShadowChunk();
    const chunkChanged = (currentChunkX !== lastChunk.x || currentChunkZ !== lastChunk.z);
    
    if (chunkChanged) {
        setLastShadowChunk(currentChunkX, currentChunkZ);
    }
    
    let now;
    const timeOverride = getTimeOverride();
    if (timeOverride !== null) {
        now = new Date();
        now.setHours(Math.floor(timeOverride / 60), timeOverride % 60, 0, 0);
    } else {
        now = new Date();
    }
    
    const sunPos = calculateSunPosition(now, STATE.lat, STATE.lon);
    const sunDist = 30000;
    
    const x = sunDist * Math.cos(sunPos.altitude) * Math.sin(sunPos.azimuth);
    const y = sunDist * Math.sin(sunPos.altitude);
    const z = -sunDist * Math.cos(sunPos.altitude) * Math.cos(sunPos.azimuth);
    
    currentSunDirection.set(x, y, z).normalize();
    
    const chunkCenterX = (currentChunkX + 0.5) * SHADOW_CHUNK_SIZE;
    const chunkCenterZ = (currentChunkZ + 0.5) * SHADOW_CHUNK_SIZE;
    
    sunLight.position.set(chunkCenterX + x, Math.max(1000, y), chunkCenterZ + z);
    sunLight.target.position.set(chunkCenterX, 0, chunkCenterZ);
    
    if (chunkChanged) {
        sunLight.shadow.camera.updateProjectionMatrix();
    }
    
    const altitudeDeg = sunPos.altitude * 180 / Math.PI;
    
    if (altitudeDeg < -6) {
        sunLight.intensity = 0.0;
        sunLight.color.setHex(0x223344);
        scene.background.setHex(0x0a1020);
        scene.fog.color.setHex(0x0a1020);
    } else if (altitudeDeg < 0) {
        const t = (altitudeDeg + 6) / 6;
        sunLight.intensity = t * 1.0;
        sunLight.color.setHex(0xff8844);
        const skyColor = lerpColor(0x0a1020, 0x553322, t);
        scene.background.setHex(skyColor);
        scene.fog.color.setHex(skyColor);
    } else if (altitudeDeg < 15) {
        const t = altitudeDeg / 15;
        sunLight.intensity = 1.0 + t * 0.5;
        const sunColor = lerpColor(0xff6622, 0xffeedd, t);
        sunLight.color.setHex(sunColor);
        const skyColor = lerpColor(0x553322, 0x87ceeb, t);
        scene.background.setHex(skyColor);
        scene.fog.color.setHex(skyColor);
    } else {
        sunLight.intensity = 1.5;
        sunLight.color.setHex(0xffffff);
        scene.background.setHex(0x87ceeb);
        scene.fog.color.setHex(0x87ceeb);
    }
    
    updateTerrainHillshading();
}

// ============== FRUSTUM CULLING ==============
const frustum = new THREE.Frustum();
const projScreenMatrix = new THREE.Matrix4();

function updateChunkVisibility() {
    const camera = getCamera();
    if (!camera) return;

    camera.updateMatrixWorld();
    projScreenMatrix.multiplyMatrices(camera.projectionMatrix, camera.matrixWorldInverse);
    frustum.setFromProjectionMatrix(projScreenMatrix);

    const activeChunks = getActiveChunks();
    for (const mesh of Object.values(activeChunks)) {
        // Recompute bounds if they are missing or known to be stale. The terrain
        // geometry is built from a 1×1 PlaneGeometry whose vertices are replaced by
        // real world coordinates, so the original tiny bounding sphere must be
        // refreshed before frustum culling decisions are made.
        if (!mesh.geometry.boundingSphere || !mesh.userData || !mesh.userData.boundsValid) {
            mesh.geometry.computeBoundingSphere();
            if (mesh.userData) mesh.userData.boundsValid = true;
        }
        // Frustum.intersectsObject() uses mesh.matrixWorld; make sure it is current
        // before testing, otherwise freshly created chunks can be culled with a
        // stale transform and disappear.
        mesh.updateMatrixWorld();
        mesh.visible = frustum.intersectsObject(mesh);
    }
}

// Terrain elevation under home, resolved once per home position. getTerrainElevationCached()
// keeps a single-entry cache shared with the vehicle query, so we memoize here instead of
// re-querying it every frame. Stays null (and keeps retrying) until the HGT tile is loaded.
let homeTerrainElev = null;
let homeTerrainKey = null;

function getHomeTerrainElevation() {
    if (STATE.homeLat === null || STATE.homeLon === null) return null;
    const key = `${STATE.homeLat},${STATE.homeLon}`;
    if (key !== homeTerrainKey || homeTerrainElev === null) {
        homeTerrainKey = key;
        homeTerrainElev = getTerrainElevationCached(STATE.homeLat, STATE.homeLon);
    }
    return homeTerrainElev;
}

// ============== 3D WORLD UPDATE ==============
function update3DWorld() {
    const camera = getCamera();
    if (!camera) return;
    
    const planePos = latLonToMeters(STATE.lat, STATE.lon);
    let totalAlt = STATE.rawAlt + STATE.offsetAlt;

    // Periodic terrain refresh: HGT files can be lazy-loaded or the worker can
    // be slow to answer. Re-calling updateTerrainChunks() keeps chunk creation
    // moving even when the aircraft is not moving.
    if (getHGTFileCount() > 0) {
        const now = performance.now();
        if (!update3DWorld._lastTerrainRefresh || now - update3DWorld._lastTerrainRefresh > 1000) {
            update3DWorld._lastTerrainRefresh = now;
            updateTerrainChunks();
        }
    }

    // Update smoothed attitude for jitter-free rendering
    updateSmoothedAttitude();

    const terrHeight = getTerrainElevationCached(STATE.lat, STATE.lon);
    if (terrHeight !== null) {
        STATE.terrainHeight = terrHeight;
        // Ensure vehicle renders above terrain (visual clamp only, doesn't modify STATE.rawAlt)
        if (window._groundClampEnabled !== false && totalAlt < terrHeight + 0.5) {
            totalAlt = terrHeight + 0.5;
        }
    }
    
    updateAGLDisplay(terrHeight);

    // Ensure vehicle exists and follows the same state as the (old) 1st-person camera.
    initVehicle();
    if (vehicle) {
        vehicle.position.set(planePos.x, Math.max(totalAlt, 1), planePos.z);
        vehicle.rotation.order = 'YXZ';
        vehicle.rotation.x = smoothAtt.pitch;
        vehicle.rotation.z = -smoothAtt.roll;
        vehicle.rotation.y = -smoothAtt.yaw;
        vehicle.visible = (cameraMode === 'THIRD');
    }

    if (cameraMode === 'THIRD' && vehicle) {
        const target = vehicle.position;
        const r = orbit.distance;

        const cosPitch = Math.cos(orbit.pitch);
        const sinPitch = Math.sin(orbit.pitch);
        const sinYaw = Math.sin(orbit.yaw);
        const cosYaw = Math.cos(orbit.yaw);

        const offX = r * cosPitch * sinYaw;
        const offY = r * sinPitch + orbit.height;
        const offZ = r * cosPitch * cosYaw;

        camera.position.set(target.x + offX, target.y + offY, target.z + offZ);
        camera.lookAt(target.x, target.y + orbit.height * 0.2, target.z);
    } else {
        // First-person camera (existing behavior)
        camera.position.set(planePos.x, Math.max(totalAlt, 1), planePos.z);
        camera.rotation.order = 'YXZ';
        camera.rotation.x = horizonLocked ? 0 : smoothAtt.pitch;
        camera.rotation.z = -smoothAtt.roll;
        camera.rotation.y = -smoothAtt.yaw;
    }

    // Update predicted trajectory corridor (throttled to every 3rd frame)
    if (trajectoryEnabled) {
        if (!update3DWorld._trajFc) update3DWorld._trajFc = 0;
        if (++update3DWorld._trajFc % 3 === 0) {
            const predTime = getPredictionTime(STATE.gs || 0);
            const path = computePredictedPath(STATE, 40, predTime);
            updateCorridor(path);
        }
    }

    updateTrail(planePos.x, totalAlt, planePos.z);

    if (STATE.connected && STATE.lastReloadPos.lat) {
        const distFromLastReload = calculateDistance(
            STATE.lastReloadPos.lat,
            STATE.lastReloadPos.lon,
            STATE.lat,
            STATE.lon
        );

        if (distFromLastReload > RELOAD_DISTANCE) {
            reloadMapAndRunways();
        }
    }

    const dist = Math.sqrt(
        (planePos.x - STATE.lastUpdatePos.x) ** 2 + 
        (planePos.z - STATE.lastUpdatePos.z) ** 2
    );
    
    if (getHGTFileCount() > 0 && (Object.keys(getActiveChunks()).length === 0 || dist > 2000)) {
        STATE.lastUpdatePos = { x: planePos.x, z: planePos.z };
        updateTerrainChunks();
    }

    updateChunkVisibility();
    updateWireframeProximity();
    render();
}

// ============== RELOAD MAP AND RUNWAYS ==============
async function reloadMapAndRunways() {
    lastMapReloadAt = performance.now();
    document.getElementById('reload-indicator').classList.add('visible');
    STATE.lastReloadPos.lat = STATE.lat;
    STATE.lastReloadPos.lon = STATE.lon;
    STATE.lastUpdatePos = { x: 9999999, z: 9999999 };
    await updateTerrainChunks();
    if (getHGTFileCount() > 0) await fetchRunwaysAuto();
    setTimeout(() => {
        document.getElementById('reload-indicator').classList.remove('visible');
    }, 500);
}

// ============== RUNWAY FETCHING ==============
async function fetchRunwaysAuto() {
    if (getHGTFileCount() === 0) return;
    const query = `[out:json];way["aeroway"="runway"](around:25000,${STATE.lat},${STATE.lon});out geom;`;
    try {
        const res = await fetch("https://overpass-api.de/api/interpreter", { method: 'POST', body: query });
        const data = await res.json();
        if (data.elements) {
            drawRunways(data.elements);
            document.getElementById('btn-scan').innerText = `${data.elements.length} RUNWAYS`;
        }
    } catch (e) {}
}

function drawRunways(elements) {
    const scene = getScene();
    const runwayObjects = getRunwayObjects();
    
    runwayObjects.forEach(o => { 
        scene.remove(o); 
        o.geometry.dispose(); 
    });
    runwayObjects.length = 0;
    
    const mat = new THREE.MeshBasicMaterial({ color: 0x111111, side: THREE.DoubleSide });
    const matL = new THREE.MeshBasicMaterial({ color: 0xffffff, side: THREE.DoubleSide });
    
    elements.forEach(el => {
        if (el.geometry && el.geometry.length >= 2) {
            const pts = el.geometry.map(pt => {
                const pos = latLonToMeters(pt.lat, pt.lon);
                const h = getTerrainElevationFromHGT(pt.lat, pt.lon) || 0;
                return { x: pos.x, z: pos.z, h: h };
            });
            createStrip(pts, 45, 2.0, mat, runwayObjects);
            createStrip(pts, 2, 2.05, matL, runwayObjects);
        }
    });
}

function createStrip(pts, w, hOff, mat, runwayObjects) {
    const scene = getScene();
    const verts = [];
    
    for (let i = 0; i < pts.length - 1; i++) {
        const p1 = pts[i];
        const p2 = pts[i + 1];
        const dx = p2.x - p1.x;
        const dz = p2.z - p1.z;
        const len = Math.sqrt(dx * dx + dz * dz);
        const nx = -dz / len;
        const nz = dx / len;
        const ox = nx * (w / 2);
        const oz = nz * (w / 2);
        
        verts.push(
            p1.x - ox, p1.h + hOff, p1.z - oz,
            p1.x + ox, p1.h + hOff, p1.z + oz,
            p2.x - ox, p2.h + hOff, p2.z - oz,
            p1.x + ox, p1.h + hOff, p1.z + oz,
            p2.x + ox, p2.h + hOff, p2.z + oz,
            p2.x - ox, p2.h + hOff, p2.z - oz
        );
    }
    
    if (verts.length) {
        const g = new THREE.BufferGeometry();
        g.setAttribute('position', new THREE.Float32BufferAttribute(verts, 3));
        g.computeVertexNormals();
        const m = new THREE.Mesh(g, mat);
        scene.add(m);
        runwayObjects.push(m);
    }
}

// ============== MINIMAP HOVER SIZE (1/5 screen area) ==============
const MINIMAP_ASPECT = 260 / 180; // ~1.44
function updateMinimapHoverSize() {
    const area = window.innerWidth * window.innerHeight;
    const h = Math.round(Math.sqrt(area / (5 * MINIMAP_ASPECT)));
    const w = Math.round(h * MINIMAP_ASPECT);
    document.documentElement.style.setProperty('--minimap-hover-w', `${w}px`);
    document.documentElement.style.setProperty('--minimap-hover-h', `${h}px`);
}

// ============== MINIMAP / 3D SWAP ==============
function isMinimapSwapped() {
    return document.body.classList.contains('minimap-swapped');
}

function toggleMinimapSwap() {
    document.body.classList.toggle('minimap-swapped');
    // Let DOM settle, then resize both renderers
    setTimeout(() => {
        handleResize();
        invalidateMapSize();
    }, 50);
}

function initMinimapSwap() {
    const miniMapContainer = document.getElementById('mini-map-container');
    const sceneContainer = document.getElementById('scene-container');
    if (miniMapContainer) {
        miniMapContainer.addEventListener('click', () => {
            if (!isMinimapSwapped()) toggleMinimapSwap();
        });
    }
    if (sceneContainer) {
        sceneContainer.addEventListener('click', () => {
            if (isMinimapSwapped()) toggleMinimapSwap();
        });
        // Keep 3D renderer in sync when CSS transitions change scene-container size (hover)
        const ro = new ResizeObserver(() => {
            if (isMinimapSwapped() && sceneContainer.clientWidth > 0) {
                resize(sceneContainer.clientWidth, sceneContainer.clientHeight);
            }
        });
        ro.observe(sceneContainer);
    }
    // Keep Leaflet map in sync when mini-map-container resizes (hover or swap)
    if (miniMapContainer) {
        const ro2 = new ResizeObserver(() => invalidateMapSize());
        ro2.observe(miniMapContainer);
    }
}

// ============== RESIZE HANDLER ==============
function handleResize() {
    updateMinimapHoverSize();

    const { width, height } = resizeHUD();

    if (isMinimapSwapped()) {
        // 3D is in the small corner panel — use scene-container's actual size
        const sc = document.getElementById('scene-container');
        if (sc) {
            resize(sc.clientWidth, sc.clientHeight);
        }
    } else {
        resize(width, height);
    }

    invalidateMapSize();
    resizeFPV();
}

// ============== FPS COUNTER ==============
let fpsFrameCount = 0;
let fpsLastTime = performance.now();
let lastFrameTime = performance.now();
let lastRenderTime = 0; // Throttle heavy 3D render to TARGET_RENDER_FPS
const TARGET_RENDER_FPS = 60; // caps render work on >60Hz monitors
const RENDER_INTERVAL = 1000 / TARGET_RENDER_FPS; // ~16.6ms

// Slow-path update throttles (these don't need to run at monitor refresh rate)
let lastSunPositionUpdate = 0;     // solar almanac + hillshade check: 1 Hz
let lastUiBarUpdate = 0;           // command bar / sidebar DOM writes: 10 Hz
let lastTrafficMarkersUpdate = 0;  // ADS-B 3D markers (data refreshes every ~10 s): 2 Hz
let lastTelemetryUiUpdate = 0;     // HUD cells / telemetry panel text: 20 Hz


// Map reload throttling
let lastMapReloadAt = 0;

function updateFPS() {
    fpsFrameCount++;
    const now = performance.now();
    if (now - fpsLastTime >= 1000) {
        updateFPSDisplay(fpsFrameCount);
        fpsFrameCount = 0;
        fpsLastTime = now;
    }
}

/**
 * Build the demo patrol circuit: a closed racetrack (two straight legs joined
 * by 180° turns) around the current aircraft position. Waypoints are geographic
 * fixes, so the guidance re-captures them every lap → a stable, periodic flight.
 * The aircraft starts on the first waypoint heading north up the right leg.
 */
function initDemoCircuit(st, metersPerLat) {
    st.centerLat = STATE.lat;
    st.centerLon = STATE.lon;
    const mLon = Math.max(1, metersPerLat * Math.cos(st.centerLat * Math.PI / 180));
    const R = DEMO_TURN_RADIUS;
    const H = DEMO_STRAIGHT / 2;
    const arcSteps = 6;
    const pts = [];
    // n = metres north, e = metres east relative to the circuit centre
    const push = (n, e) => pts.push({
        lat: st.centerLat + n / metersPerLat,
        lon: st.centerLon + e / mLon
    });
    // Right straight, north-bound (east side of the track)
    push(-H, R); push(0, R); push(H, R);
    // Top 180° turn: θ 0→π gives (R·cosθ, H + R·sinθ)
    for (let i = 1; i < arcSteps; i++) {
        const th = Math.PI * i / arcSteps;
        push(H + R * Math.sin(th), R * Math.cos(th));
    }
    // Left straight, south-bound (west side of the track)
    push(H, -R); push(0, -R); push(-H, -R);
    // Bottom 180° turn: θ π→2π gives (R·cosθ, -H + R·sinθ)
    for (let i = 1; i < arcSteps; i++) {
        const th = Math.PI + Math.PI * i / arcSteps;
        push(-H + R * Math.sin(th), R * Math.cos(th));
    }
    st.waypoints = pts;
    // Start parked on the first fix, wings level, pointing up the right leg
    STATE.lat = pts[0].lat;
    STATE.lon = pts[0].lon;
    STATE.yaw = 0; // north
    st.wpIndex = 1;
    st.prevWpDist = null;
    st.bank = 0;
    st.speed = DEMO_CRUISE_SPEED;
    st.speedTarget = DEMO_CRUISE_SPEED;
    const terrain = STATE.terrainHeight !== null ? STATE.terrainHeight : 600;
    st.baseAlt = terrain + DEMO_CRUISE_AGL;
    st.baseAltLocked = STATE.terrainHeight !== null;
    STATE.rawAlt = st.baseAlt;
    st.windChangeTime = 0; // pick an initial gust on the first frame
    st.initialized = true;
}

// ============== ANIMATION LOOP ==============
function animate() {
    requestAnimationFrame(animate);
    
    const now = performance.now();
    const deltaTime = (now - lastFrameTime) / 1000;
    lastFrameTime = now;
    
    updateFPS();
    syncModelToVehicleType();

    // The sun moves ~0.25°/min: recomputing the solar almanac (new Date() +
    // trig) every frame is wasted work — 1 Hz is more than enough.
    if (now - lastSunPositionUpdate >= 1000) {
        lastSunPositionUpdate = now;
        updateSunPosition();
    }

    // Demo mode - realistic fixed-wing patrol circuit with coordinated turns
    if (STATE.mode === 'LIVE' && !STATE.connected) {
        const metersPerLat = 111320;
        const st = demoFlightState;
        const dt = Math.min(0.1, Math.max(1e-3, deltaTime)); // clamp against frame hitches

        if (!st.initialized) initDemoCircuit(st, metersPerLat);

        // Lock cruise altitude to the real centre-terrain height once it loads
        if (STATE.terrainHeight !== null && !st.baseAltLocked) {
            st.baseAlt = STATE.terrainHeight + DEMO_CRUISE_AGL;
            STATE.rawAlt = st.baseAlt;
            st.baseAltLocked = true;
        }

        const metersPerLon = Math.max(1, metersPerLat * Math.cos(st.centerLat * Math.PI / 180));

        // ── Waypoint guidance ─────────────────────────────────────────
        const wp = st.waypoints[st.wpIndex];
        const dN = (wp.lat - STATE.lat) * metersPerLat;
        const dE = (wp.lon - STATE.lon) * metersPerLon;
        const distWp = Math.hypot(dN, dE);
        // Advance on capture, or once we've flown past the fix (distance growing)
        if (distWp < DEMO_CAPTURE_R ||
            (st.prevWpDist !== null && distWp > st.prevWpDist && distWp < DEMO_CAPTURE_R * 2.5)) {
            st.wpIndex = (st.wpIndex + 1) % st.waypoints.length;
            st.prevWpDist = null;
        } else {
            st.prevWpDist = distWp;
        }
        const desiredHeading = Math.atan2(dE, dN); // 0 = north, +CW toward east

        // ── Coordinated turn: bank commands the yaw rate ──────────────
        let hdgErr = desiredHeading - STATE.yaw;
        hdgErr = Math.atan2(Math.sin(hdgErr), Math.cos(hdgErr)); // wrap to [-π, π]
        const bankCmd = Math.max(-DEMO_MAX_BANK,
            Math.min(DEMO_MAX_BANK, DEMO_HDG_GAIN * hdgErr));
        // Roll toward the commanded bank: first-order (τ≈0.7s), rate-limited
        let rollRate = (bankCmd - st.bank) / 0.7;
        rollRate = Math.max(-DEMO_ROLL_RATE, Math.min(DEMO_ROLL_RATE, rollRate));
        st.bank += rollRate * dt;
        STATE.roll = st.bank;
        STATE.rollRate = rollRate;

        // ── Airspeed: gentle drift, a little slower in steep banks ────
        const bankFrac = Math.abs(st.bank) / DEMO_MAX_BANK;
        st.speedTarget = DEMO_CRUISE_SPEED - 3 * bankFrac +
            Math.sin(now * 0.0003) * DEMO_SPEED_VARIANCE * 0.5;
        st.speed += (st.speedTarget - st.speed) * Math.min(1, dt / 3); // τ≈3s
        const V = Math.max(12, st.speed);

        // Coordinated-turn yaw rate: ω = g·tan(φ) / V
        const omega = 9.81 * Math.tan(st.bank) / V;
        const twoPi = Math.PI * 2;
        STATE.yaw = ((STATE.yaw + omega * dt) % twoPi + twoPi) % twoPi;
        STATE.yawRate = omega;

        // ── Wind: slowly-varying air-mass velocity (NED) ─────────────
        if (now > st.windChangeTime) {
            st.windChangeTime = now + 20000 + Math.random() * 20000; // new gust 20-40 s
            const wSpeed = 3 + Math.random() * 3;   // 3-6 m/s (well below airspeed)
            const wFrom = Math.random() * twoPi;     // direction wind comes FROM
            st.windTargetN = -Math.cos(wFrom) * wSpeed; // air-velocity vector (blows toward)
            st.windTargetE = -Math.sin(wFrom) * wSpeed;
        }
        st.windN += (st.windTargetN - st.windN) * Math.min(1, dt / 8); // τ≈8s
        st.windE += (st.windTargetE - st.windE) * Math.min(1, dt / 8);
        STATE.windSpeed = Math.hypot(st.windN, st.windE);
        STATE.windDir = (Math.atan2(-st.windE, -st.windN) * 180 / Math.PI + 360) % 360; // FROM
        STATE.windDataTime = now;

        // ── Altitude: fixed cruise MSL, gentle rate-limited terrain follow
        const terrainElev = STATE.terrainHeight !== null
            ? STATE.terrainHeight : (st.baseAlt - DEMO_CRUISE_AGL);
        let targetAlt = st.baseAlt;
        const clearanceFloor = terrainElev + DEMO_MIN_CLEARANCE;
        if (targetAlt < clearanceFloor) targetAlt = clearanceFloor; // climb over rising ground
        if (!Number.isFinite(STATE.rawAlt) || STATE.rawAlt <= 0) STATE.rawAlt = targetAlt;
        let vsCmd = (targetAlt - STATE.rawAlt) * 0.15; // low gain — no bump-hugging
        vsCmd = Math.max(-DEMO_MAX_VS, Math.min(DEMO_MAX_VS, vsCmd));
        STATE.vs += (vsCmd - STATE.vs) * Math.min(1, dt / 1.5); // smooth VS (τ≈1.5s)
        STATE.rawAlt += STATE.vs * dt;

        // ── Pitch: air-mass climb angle + load-factor pull in the turn ─
        const climbAngle = Math.asin(Math.max(-0.5, Math.min(0.5, STATE.vs / V)));
        const loadComp = (1 / Math.cos(st.bank) - 1) * 0.06; // nose-up to hold alt in bank
        const pitchCmd = climbAngle + loadComp;
        const prevPitch = STATE.pitch;
        STATE.pitch += (pitchCmd - STATE.pitch) * Math.min(1, dt / 0.6);
        STATE.pitchRate = (STATE.pitch - prevPitch) / dt;

        // ── Integrate GROUND position = airspeed-along-nose + wind ────
        const cosPitch = Math.cos(STATE.pitch);
        const airHoriz = V * dt * cosPitch;                 // horizontal air distance
        const northMeters = Math.cos(STATE.yaw) * airHoriz + st.windN * dt;
        const eastMeters = Math.sin(STATE.yaw) * airHoriz + st.windE * dt;
        STATE.lat += northMeters / metersPerLat;
        STATE.lon += eastMeters / metersPerLon;

        // Ground velocity (NED) and the inertial velocity vector for the HUD
        STATE.vn = northMeters / dt;
        STATE.ve = eastMeters / dt;
        STATE.vd = -STATE.vs;
        const groundHoriz = Math.hypot(STATE.vn, STATE.ve);
        STATE.as = V;
        STATE.gs = groundHoriz;
        const inertialTrack = Math.atan2(STATE.ve, STATE.vn);
        const inertialGamma = Math.atan2(STATE.vs, Math.max(1, groundHoriz));
        STATE.track = ((inertialTrack % twoPi) + twoPi) % twoPi;
        STATE.gamma = inertialGamma;
        // AoA/SSA (air-relative, wind subtracted) for the primary FPM — same
        // physics as real telemetry, so both paths agree. STATE.track/gamma
        // above stay ground-referenced for the ground-track marker.
        computeAeroAngles();

        // Simulate LiDAR rangefinder (downward-facing)
        if (STATE.terrainHeight !== null) {
            const agl = STATE.rawAlt - STATE.terrainHeight;
            const noise = (Math.random() - 0.5) * 0.04 + agl * 0.001 * (Math.random() - 0.5);
            STATE.rangefinderDist = Math.max(0.01, agl + noise);
        } else {
            STATE.rangefinderDist = null;
        }

        // G-load: coordinated-turn load factor n = 1/cos(φ) (az negative at rest)
        const loadFactor = Math.min(3, 1 / Math.cos(st.bank));
        STATE.ax = 0;
        STATE.ay = 0;
        STATE.az = -9.81 * loadFactor;
        pushGHistory();
    }

    // Telemetry text (40+ DOM writes) is unreadable above ~20 Hz — no need to
    // run it at monitor refresh rate (60-144 Hz).
    if (now - lastTelemetryUiUpdate >= 50) {
        lastTelemetryUiUpdate = now;
        updateUI();
    }
    // Throttle heavy 3D rendering to 30fps max.
    // The animation loop runs at monitor refresh rate (60-144Hz) but heavy GPU work
    // (3D render, terrain, frustum culling) is capped to free the main thread
    // for MAVLink parsing, RC radio input, and UI responsiveness.
    const onFlightDataTab = getCurrentTab() === 'flight-data';
    const renderDue = (now - lastRenderTime) >= RENDER_INTERVAL;

    if (onFlightDataTab && renderDue) {
        lastRenderTime = now;
        // The DISARMED banner is an HTML overlay shown in both camera modes, so
        // it is refreshed outside the first-person-only HUD draw.
        updateArmStateUI();
        updateRotorLoadPanel();
        // Draw HUD in first-person 3D or when FPV is active
        if (cameraMode !== 'THIRD') {
            drawHUD();
        }
        update3DWorld();
        updateHomeMarker3D(getHomeTerrainElevation());

        // Check if nearby chunks need high-res textures (skip in AR mode — terrain is hidden)
        if (!isFPVARMode()) {
            refreshNearbyChunkTextures();
        }
    }

    // Update GCS command bar, sidebar and target marker at 10 Hz — DOM writes
    // don't need to run at monitor refresh rate.
    if (now - lastUiBarUpdate >= 100) {
        lastUiBarUpdate = now;
        updateCommandBar();
        updateGCSSidebar();
        const tc = getTargetCoords();
        if (tc) {
            const tElev = getTerrainElevationCached(tc.lat, tc.lon);
            updateTargetMarker3D(tElev);
        }
    }

    // ADS-B data refreshes every ~10 s — 2 Hz is plenty for the 3D markers
    if (now - lastTrafficMarkersUpdate >= 500) {
        lastTrafficMarkersUpdate = now;
        updateTrafficMarkers3D(getNearestTraffic(4));
    }

    updateMap();

    checkInitialLoadComplete(
        getActiveChunks(), 
        getChunkCreationQueue(), 
        getTileLoadQueue(), 
        getCurrentTileLoads(),
        getTotalTilesToLoad(),
        getTilesLoaded()
    );
}

// ============== MISSION TRAJECTORY 3D ==============
const NAV_CMDS_3D = [16, 17, 18, 19, 21, 22, 82];

function buildMissionTrajectory3D() {
    const items = STATE.missionItems.filter(it => NAV_CMDS_3D.includes(it.command));
    if (items.length === 0) {
        clearMissionTrajectory();
        return;
    }

    // In the 3D world, terrain mesh Y = MSL elevation (from HGT data).
    // Waypoints with frame 3 (GLOBAL_RELATIVE_ALT) have alt relative to home.
    // To position them correctly above terrain, use terrain elevation at each WP
    // as base, then add the relative altitude. This works both with and without
    // a live vehicle connection.
    const offset = STATE.offsetAlt || 0;

    const points = items.map(item => {
        const pos = latLonToMeters(item.lat, item.lng);
        // WP alt is always AGL: terrain elevation + specified altitude
        const terrainElev = getTerrainElevationCached(item.lat, item.lng);
        const baseAlt = terrainElev !== null ? terrainElev : (STATE.homeAlt || STATE.rawAlt || 0);
        const worldY = baseAlt + (item.alt || 0) + offset;
        return { x: pos.x, y: worldY, z: pos.z };
    });

    updateMissionTrajectory(points);
}

// ============== SATELLITE/SUNLIGHT TOGGLES ==============
function setSatelliteEnabled(enabled) {
    window.satelliteEnabled = !!enabled;
    const btn = document.getElementById('btn-sat');
    if (btn) {
        if (window.satelliteEnabled) btn.classList.add('active');
        else btn.classList.remove('active');
    }
    
    // Apply across terrain + maps
    try { setTerrainSatelliteEnabled(window.satelliteEnabled); } catch (e) {}

    setHillshadeNeedsUpdate();
    updateTerrainHillshading(true);
}

function toggleSatellite() {
    setSatelliteEnabled(!window.satelliteEnabled);
}

function toggleSunlight() {
    const enabled = !isSunlightEnabled();
    setSunlightEnabled(enabled);
    window.sunlightEnabled = enabled;
    
    const btn = document.getElementById('btn-sun');
    const sunLight = getSunLight();
    const ambientLight = getAmbientLight();
    const camera = getCamera();
    const scene = getScene();
    const currentSunDirection = getCurrentSunDirection();
    
    if (enabled) {
        btn.classList.add('active');
        ambientLight.intensity = 0.6;
        sunLight.intensity = 1.5;
        sunLight.castShadow = (cameraMode === 'THIRD');
        updateSunPosition();
        setHillshadeNeedsUpdate();
        updateTerrainHillshading(true);
    } else {
        btn.classList.remove('active');
        sunLight.position.set(camera.position.x, camera.position.y + 30000, camera.position.z);
        sunLight.target.position.copy(camera.position);
        sunLight.intensity = 0.5;
        sunLight.color.setHex(0xffffff);
        sunLight.castShadow = false;
        ambientLight.intensity = 0.6;
        scene.background.setHex(0x87ceeb);
        scene.fog.color.setHex(0x87ceeb);
        currentSunDirection.set(0, 1, 0);
        setHillshadeNeedsUpdate();
        updateTerrainHillshading(true);
    }

    updateMapBrightnessVisibility();
}

// ============== THEME TOGGLE ==============
function toggleTheme() {
    const html = document.documentElement;
    const isLight = html.getAttribute('data-theme') === 'light';
    html.setAttribute('data-theme', isLight ? 'dark' : 'light');
    document.getElementById('btn-theme').classList.toggle('active', !isLight);
}

// ============== TRAJECTORY CORRIDOR TOGGLE ==============
let trajectoryEnabled = false;

function toggleTrajectory() {
    trajectoryEnabled = !trajectoryEnabled;
    document.getElementById('btn-traj')?.classList.toggle('active', trajectoryEnabled);
    setCorridorVisible(trajectoryEnabled);
}

// ============== VIEW TOGGLE KEYBOARD SHORTCUTS ==============
// Single-key quick toggles for the 3D view. Active only on the Flight Data
// tab and ignored while typing in a field or with a modifier held.
//   T = toggle horizon-lock view             P = predicted trajectory corridor
//   M = satellite map overlay               L = realistic sunlight
function initViewShortcuts() {
    document.addEventListener('keydown', (e) => {
        if (e.ctrlKey || e.altKey || e.metaKey) return;
        const tag = document.activeElement?.tagName;
        if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') return;
        const flightTab = document.getElementById('tab-flight-data');
        if (!flightTab || !flightTab.classList.contains('active')) return;

        switch (e.key.toLowerCase()) {
            case 't': toggleHorizonLock(); break;
            case 'p': toggleTrajectory(); break;
            case 'm': toggleSatellite(); break;
            case 'l': toggleSunlight(); break;
            default: return;
        }
        e.preventDefault();
    });
}

// ============== HGT FILE INPUT ==============
function setupHGTInput() {
    document.getElementById('hgt-input').onchange = (e) => {
        const files = e.target.files;
        let c = 0;
        for (let i = 0; i < files.length; i++) {
            if (files[i].name.toLowerCase().endsWith('.hgt')) {
                addHGTFile(files[i].name, files[i]);
                c++;
            }
        }
        setStatusMessage(`${c} HGT LOADED`, 'var(--accent-cyan)');
        updateTerrainChunks();
    };
}

// ============== TIME SLIDER ==============
function setupTimeSlider() {
    document.getElementById('time-slider').oninput = (e) => {
        const minutes = parseInt(e.target.value);
        setTimeOverride(minutes);
        // Apply immediately — the animation loop only refreshes the sun at 1 Hz
        updateSunPosition();
        const hours = Math.floor(minutes / 60);
        const mins = minutes % 60;
        document.getElementById('time-display').textContent = 
            `${hours.toString().padStart(2, '0')}:${mins.toString().padStart(2, '0')}`;
    };
}

// ============== MAP BRIGHTNESS (STATIC) ==============
function setupMapBrightnessSlider() {
    const slider = document.getElementById('map-brightness-slider');
    const display = document.getElementById('map-brightness-display');
    if (!slider || !display) return;

    const apply = () => {
        const value = parseFloat(slider.value);
        display.textContent = value.toFixed(2);
        setMapBrightness(value);
    };

    slider.addEventListener('input', apply);
    apply();
}

function updateMapBrightnessVisibility() {
    const row = document.getElementById('map-brightness-row');
    if (!row) return;
    row.style.display = isSunlightEnabled() ? 'none' : '';
}

// ============== ATTITUDE SMOOTHING SLIDER ==============
function setupAttSmoothSlider() {
    const slider = document.getElementById('att-smooth-slider');
    const display = document.getElementById('att-smooth-display');
    if (!slider || !display) return;
    slider.addEventListener('input', () => {
        ATT_SMOOTH = parseFloat(slider.value);
        display.textContent = ATT_SMOOTH.toFixed(2);
    });
}

// ============== STREAM RATES ==============
const SR_PARAMS = [
    'SR0_RAW_SENS', 'SR0_EXT_STAT', 'SR0_RC_CHAN', 'SR0_RAW_CTRL',
    'SR0_POSITION', 'SR0_EXTRA1', 'SR0_EXTRA2', 'SR0_EXTRA3', 'SR0_PARAMS'
];

function setupStreamRates() {
    // Read button: request each SR param from vehicle
    const readBtn = document.getElementById('sr-read');
    if (readBtn) {
        readBtn.addEventListener('click', () => {
            SR_PARAMS.forEach(p => requestParameter(p).catch(() => {}));
        });
    }

    // Write button: send changed values
    const writeBtn = document.getElementById('sr-write');
    if (writeBtn) {
        writeBtn.addEventListener('click', async () => {
            const inputs = document.querySelectorAll('#tab-sys-config input[data-param]');
            for (const inp of inputs) {
                const val = parseInt(inp.value, 10);
                if (!isNaN(val)) {
                    try {
                        await setParameter(inp.dataset.param, val);
                        inp.style.borderColor = '#44ff44';
                        setTimeout(() => { inp.style.borderColor = ''; }, 2000);
                    } catch (e) {
                        inp.style.borderColor = '#ff4444';
                    }
                }
            }
        });
    }

    // Update inputs when param values are received
    onMessage(22, (data) => {
        const inp = document.querySelector(`#tab-sys-config input[data-param="${data.paramId}"]`);
        if (inp) {
            inp.value = Math.round(data.paramValue);
        }
    });
}

// ============== MODEL SELECTOR ==============
function setupModelSelector() {
    const select = document.getElementById('model-select');
    const slider = document.getElementById('scale-slider');
    const display = document.getElementById('scale-display');
    
    if (select) {
        select.addEventListener('change', (e) => {
            if (e.target.value) {
                loadModel(e.target.value);
            }
        });
    }
    
    if (slider && display) {
        slider.addEventListener('input', (e) => {
            const scale = parseFloat(e.target.value);
            display.textContent = scale.toFixed(1);
            updateModelScale(scale);
        });
    }
}

// ============== AUTO LOAD TOPOGRAPHY ==============
async function loadTopographyAtStart() {
    try {
        if (!window.topography || !window.topography.load) {
            console.debug('topography API not available in preload');
            setAutoLoadAttempted();
            setStatusMessage('AUTO HGT LOAD: not available', '#ff4444');
            hideLoadingOverlay();
            return;
        }

        console.debug('Invoking topography.load...');
        // Now returns just filenames (strings), not file contents
        const names = await window.topography.load('topography');

        if (!names || names.length === 0) {
            setAutoLoadAttempted();
            setStatusMessage('AUTO HGT LOAD: no files found', '#ffcc00');
            scheduleHideLoadingOverlaySoon();
            return;
        }

        // Register available files for lazy on-demand loading
        setAvailableHgtFiles(names);
        setStatusMessage(`${names.length} HGT AVAILABLE (lazy)`, 'var(--accent-cyan)');

        // Trigger terrain update — will lazy-load nearby tiles
        updateTerrainChunks();
        setAutoLoadAttempted();
    } catch (e) {
        console.warn('Topography load failed', e);
        setAutoLoadAttempted();
        setStatusMessage('AUTO HGT LOAD: error', '#ff4444');
        scheduleHideLoadingOverlaySoon();
    }
}

// ============== CONNECTIVITY CHECK ==============
async function checkConnectivity() {
    const TEST_URL = 'https://mt0.google.com/vt/lyrs=s&x=0&y=0&z=0';
    try {
        const controller = new AbortController();
        const timeout = setTimeout(() => controller.abort(), 5000);
        const res = await fetch(TEST_URL, { signal: controller.signal });
        clearTimeout(timeout);
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        // Connection OK - satellite stays enabled
        showLoadingOverlay('Loading maps...');
    } catch (e) {
        // No connection - disable satellite, go wireframe-only
        console.warn('Connectivity check failed, switching to wireframe mode:', e.message);
        window.satelliteEnabled = false;
        try { setSatelliteEnabled(false); } catch (_) {}
        setStatusMessage('NO CONNECTION — WIREFRAME MODE', '#ff8800');
        pushHudMessage('[WARNING] No internet connection — satellite maps unavailable, using wireframe 3D', 'warning');
        showLoadingOverlay('Loading terrain (offline)...');
    }
}

// ============== INITIALIZATION ==============
function init() {
    // Initialize 3D scene
    const container = document.getElementById('scene-container');
    const { scene, camera, renderer } = init3D(container);

    // Camera mode + orbit controls
    initThirdPersonControls();
    setCameraMode('FIRST');
    
    // Initialize terrain manager
    initTerrain(scene, renderer, getCurrentSunDirection());
    window.sunlightEnabled = isSunlightEnabled();

    // Realistic sunlight OFF by default at startup: flat lighting is lighter
    // (no shadow pass, no per-sun hillshade refresh) and more readable.
    // Going through toggleSunlight() applies the full disable path (button
    // state, overhead light, hillshade, brightness slider visibility).
    if (isSunlightEnabled()) toggleSunlight();

    // Initialize HUD
    initHUD(document.getElementById('hud-canvas'));
    
    // Setup event listeners
    window.onresize = handleResize;
    setupHGTInput();
    initViewShortcuts();
    setupTimeSlider();
    setupMapBrightnessSlider();
    setupAttSmoothSlider();
    setupStreamRates();
    setupModelSelector();

    // Header hamburger (secondary toggles)
    initMoreMenu();
    initConfigAutoClose();

    // Initialize MAVLink and GCS controls
    initMAVLink();
    initTerrainFeeder();
    initCommandBar();
    initGCSSidebar();
    initLogReplay();
    initHudCells();
    initRotorLoadPanel();
    initTabs();
    initOfflinePanel();
    initMap('mini-map');
    initMinimapSwap();
    updateMinimapHoverSize();
    initParamsPage();
    initFPV();
    window.toggleParamsPage = toggleParamsPage;

    // Listen for mission updates and rebuild 3D trajectory + 2D mini-map overlay
    window.addEventListener('missionUpdated', () => {
        buildMissionTrajectory3D();
        updateMissionOverlay();
    });

    // Forward STATUSTEXT messages to HUD
    const SEVERITY_LEVELS = ['EMERGENCY', 'ALERT', 'CRITICAL', 'ERROR', 'WARNING', 'NOTICE', 'INFO', 'DEBUG'];
    let _mavMsgOverlay = null;
    const MAV_MSG_DURATION = 10000;
    const MAV_MSG_MAX = 5;
    function pushMavOverlayError(text) {
        if (!_mavMsgOverlay) _mavMsgOverlay = document.getElementById('mav-msg-overlay');
        if (!_mavMsgOverlay) return;
        const item = document.createElement('div');
        item.className = 'mav-msg-item';
        item.textContent = text;
        _mavMsgOverlay.insertBefore(item, _mavMsgOverlay.firstChild);
        while (_mavMsgOverlay.children.length > MAV_MSG_MAX) {
            _mavMsgOverlay.removeChild(_mavMsgOverlay.lastChild);
        }
        setTimeout(() => {
            item.classList.add('fading');
            setTimeout(() => { if (item.parentNode) item.parentNode.removeChild(item); }, 700);
        }, MAV_MSG_DURATION);
    }
    onMessage(253, (data) => {
        const sev = data.severity ?? 6;
        const prefix = SEVERITY_LEVELS[sev] || 'INFO';
        const text = `[${prefix}] ${data.text || ''}`;
        const level = sev <= 3 ? 'error' : sev <= 4 ? 'warning' : 'info';
        pushHudMessage(text, level);
        if (sev <= 3) pushMavOverlayError(text);
    });

    // Command ACK toast overlay
    let ackHideTimer = 0;
    let ackFadeTimer = 0;
    function showAckToast(cmdName, resultName) {
        const toast = document.getElementById('cmd-ack-toast');
        if (!toast) return;
        toast.textContent = `${cmdName}: ${resultName}`;
        toast.className = 'cmd-ack-toast';
        const accepted = resultName === 'ACCEPTED' || resultName === 'IN_PROGRESS';
        const denied = resultName === 'DENIED' || resultName === 'FAILED' || resultName === 'TEMPORARILY_REJECTED';
        toast.classList.add(accepted ? 'ack-accepted' : denied ? 'ack-denied' : 'ack-other');
        clearTimeout(ackHideTimer);
        clearTimeout(ackFadeTimer);
        ackFadeTimer = setTimeout(() => toast.classList.add('fade-out'), 2500);
        ackHideTimer = setTimeout(() => toast.classList.add('hidden'), 3000);
    }

    // Show COMMAND_ACK results on HUD + toast
    window.addEventListener('commandAck', (e) => {
        const { cmdName, resultName, level } = e.detail;
        pushHudMessage(`${cmdName}: ${resultName}`, level);
        showAckToast(cmdName, resultName);
    });

    // In split view the window size doesn't change, but the hud wrapper does.
    // Observe layout size changes and keep Three.js renderer in sync.
    try {
        const wrapper = document.getElementById('hud-wrapper');
        if (wrapper && window.ResizeObserver) {
            const ro = new ResizeObserver(() => {
                // Avoid doing heavy work in the observer callback directly.
                requestAnimationFrame(handleResize);
            });
            ro.observe(wrapper);
        }
    } catch (e) {}
    
    // Initial resize
    handleResize();

    updateMapBrightnessVisibility();

    // Setup satellite toggle state
    window.satelliteEnabled = true;
    try {
        setSatelliteEnabled(window.satelliteEnabled);
    } catch (e) {}

    // Listen for runtime connection loss (many consecutive tile errors)
    window.addEventListener('connectionLost', () => {
        setSatelliteEnabled(false);
        setStatusMessage('CONNECTION LOST — WIREFRAME MODE', '#ff8800');
        pushHudMessage('[WARNING] Connection lost — satellite maps disabled, using wireframe 3D', 'warning');
    });

    // Initialize trajectory corridor (hidden by default)
    initCorridor(getScene());

    // Show loading and start auto-load
    showLoadingOverlay('Checking connection...');

    // Check connectivity before loading maps
    checkConnectivity().then(() => {
        loadTopographyAtStart();
    });
    
    // Start ADS-B auto-polling (OpenSky + MAVLink ADSB_VEHICLE)
    startADSBPolling();

    // Start animation loop
    animate();
}

// ============== EXPOSE GLOBAL FUNCTIONS ==============
window.toggleConfig = toggleConfig;
window.toggleTelemetry = toggleTelemetry;
window.updateOffset = updateOffset;
window.connectSerial = connectSerial;
window.fetchRunways = fetchRunwaysAuto;
window.fetchRunwaysAuto = fetchRunwaysAuto;
window.toggleSatellite = toggleSatellite;
window.toggleSunlight = toggleSunlight;
window.toggleTheme = toggleTheme;
window.toggleTrajectory = toggleTrajectory;
window.onFPVButtonClick = onFPVButtonClick;
window.saveFPVSettings = saveFPVSettings;

// GCS output mute toggle — suppresses all outgoing MAVLink (heartbeat, RTK, RC override, commands)
let _gcsMuted = false;
window.toggleGcsMute = async function(forceState) {
    const next = typeof forceState === 'boolean' ? forceState : !_gcsMuted;
    if (next === _gcsMuted) return;
    _gcsMuted = next;
    try {
        await window.mavlink.setGcsMuted(_gcsMuted);
    } catch (e) {
        console.error('[GCS Mute] Failed:', e.message);
        _gcsMuted = !_gcsMuted; // revert
        const chk = document.getElementById('chk-mute-gcs');
        if (chk) chk.checked = _gcsMuted;
        return;
    }
    localStorage.setItem('gcs-muted', _gcsMuted ? '1' : '0');
    pushHudMessage(_gcsMuted ? 'GCS output MUTED — no messages will be sent' : 'GCS output UNMUTED', _gcsMuted ? 'warning' : 'info');
};

// Clear flight trail
window.clearTrail = function() {
    resetTrail();
    resetMapTrail();
    pushHudMessage('Trail cleared');
};

// ADS-B auto-polling (OpenSky every 30s, MAVLink comes via message handler)
const ADSB_POLL_INTERVAL = 30000;
let adsbPollTimer = null;

async function adsbPoll() {
    try {
        const result = await fetchADSBData();
        if (result.error) return;
    } catch (e) {
        // Silently retry next interval
    }
}

function startADSBPolling() {
    if (adsbPollTimer) return;
    setTimeout(() => {
        adsbPoll();
        adsbPollTimer = setInterval(adsbPoll, ADSB_POLL_INTERVAL);
    }, 3000);
}

// ADS-B enable/disable toggle
let _adsbEnabled = true;
window.toggleADSB = function(enabled) {
    _adsbEnabled = enabled;
    const bar = document.getElementById('traffic-bar');
    if (bar) bar.style.display = enabled ? '' : 'none';
    if (enabled) {
        startADSBPolling();
    } else {
        if (adsbPollTimer) { clearInterval(adsbPollTimer); adsbPollTimer = null; }
        STATE.traffic = [];
        updateTrafficOverlay(); // remove dots from 2D map immediately
    }
    pushHudMessage(enabled ? 'ADS-B traffic enabled' : 'ADS-B traffic disabled', 'info');
};

// Sync ADS-B checkbox in sidebar with sys config + persist
window.toggleADSB = (function(orig) {
    return function(enabled) {
        orig(enabled);
        const sidebarChk = document.getElementById('chk-adsb-enable');
        const syscfgChk = document.getElementById('syscfg-adsb-enable');
        if (sidebarChk) sidebarChk.checked = enabled;
        if (syscfgChk) syscfgChk.checked = enabled;
        localStorage.setItem('adsb-enabled', enabled ? '1' : '0');
    };
})(window.toggleADSB);

// Battery voltage-based percentage calculation
window._batVMin = 9.6; window._batVMax = 12.6;
window.updateBatteryVoltageRange = function() {
    let vmin = parseFloat(document.getElementById('syscfg-bat-vmin')?.value);
    let vmax = parseFloat(document.getElementById('syscfg-bat-vmax')?.value);
    if (!Number.isFinite(vmin) || vmin <= 0) vmin = 9.6;
    if (!Number.isFinite(vmax) || vmax <= 0) vmax = 12.6;
    // vmax <= vmin would divide by zero / invert the % estimate downstream
    if (vmax <= vmin) { vmin = 9.6; vmax = 12.6; }
    window._batVMin = vmin;
    window._batVMax = vmax;
    const cells = Math.round(vmax / 4.2);
    const cellsEl = document.getElementById('syscfg-bat-cells');
    if (cellsEl) cellsEl.textContent = cells + 'S';
    localStorage.setItem('bat-vmin', vmin);
    localStorage.setItem('bat-vmax', vmax);
};

// Restore all GCS settings on load
(function restoreGcsSettings() {
    // Battery voltage range
    const vmin = parseFloat(localStorage.getItem('bat-vmin'));
    const vmax = parseFloat(localStorage.getItem('bat-vmax'));
    if (!isNaN(vmin)) { window._batVMin = vmin; const el = document.getElementById('syscfg-bat-vmin'); if (el) el.value = vmin; }
    if (!isNaN(vmax)) { window._batVMax = vmax; const el = document.getElementById('syscfg-bat-vmax'); if (el) el.value = vmax; }
    window.updateBatteryVoltageRange();

    // ADS-B toggle
    const adsbSaved = localStorage.getItem('adsb-enabled');
    if (adsbSaved === '0') {
        _adsbEnabled = false;
        const bar = document.getElementById('traffic-bar');
        if (bar) bar.style.display = 'none';
        const sidebarChk = document.getElementById('chk-adsb-enable');
        const syscfgChk = document.getElementById('syscfg-adsb-enable');
        if (sidebarChk) sidebarChk.checked = false;
        if (syscfgChk) syscfgChk.checked = false;
        if (adsbPollTimer) { clearInterval(adsbPollTimer); adsbPollTimer = null; }
    }

    // GCS Mute
    const muteSaved = localStorage.getItem('gcs-muted');
    if (muteSaved === '1') {
        const muteChk = document.getElementById('chk-mute-gcs');
        if (muteChk) { muteChk.checked = true; window.toggleGcsMute(true); }
    }

    // Ground clamp
    const clampSaved = localStorage.getItem('ground-clamp-enabled');
    if (clampSaved === '0') {
        window._groundClampEnabled = false;
        const el = document.getElementById('syscfg-ground-clamp-enable');
        if (el) el.checked = false;
    }
})();

window.toggleGroundClamp = function(enabled) {
    window._groundClampEnabled = enabled;
    localStorage.setItem('ground-clamp-enabled', enabled ? '1' : '0');
};

// Download traffic CSV from menu
window.downloadTraffic = function() {
    if (STATE.traffic.length === 0) {
        pushHudMessage('No traffic data to download', 'warning');
        return;
    }
    downloadTrafficCSV();
    pushHudMessage(`Traffic CSV downloaded (${STATE.traffic.length} entries)`);
};

// TLOG recording
const _tlogLogger = new TlogLogger();
window.toggleRecording = () => _tlogLogger.toggleRecording();

// Initialize when DOM is ready
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
} else {
    init();
}
