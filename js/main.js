/**
 * main.js - Application Entry Point
 * Initializes all modules and runs the main animation loop
 */

// Core imports
import {
    ORIGIN, CAMERA_FOV, VISIBILITY_RADIUS, RELOAD_DISTANCE, RAD,
    DEMO_TARGET_INTERVAL, DEMO_SMOOTHING, DEMO_BASE_SPEED, DEMO_SPEED_VARIANCE,
    DEMO_ALT_AGL, DEMO_PITCH_RANGE, DEMO_ROLL_RANGE, DEMO_LEG_LENGTH, DEMO_LEG_SPACING,
    TRACE_CONFIG
} from './core/constants.js';
import { STATE, demoAttitude, demoSurveyState, pushGHistory, dataBuffer, activeTraces } from './core/state.js';
import { ndConfig } from './ui/NDView.js';
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
    addHGTFile, getHGTFileCount, getActiveChunks,
    getChunkCreationQueue, getTileLoadQueue, getCurrentTileLoads,
    getTotalTilesToLoad, getTilesLoaded,
    getTerrainElevationFromHGT, getRunwayObjects,
    refreshNearbyChunkTextures, resetTextureRefreshPosition,
    setMapBrightness,
    getMemoryStats,
    updateWireframeProximity
} from './terrain/TerrainManager.js';

// HUD imports
import { initHUD, drawHUD, resizeHUD, setViewMode as setHUDViewMode, pushHudMessage } from './hud/HUDRenderer.js';

// Map imports
import { initMap, updateMap, invalidateSize as invalidateMapSize, updateMissionOverlay } from './maps/MapEngine.js';

// Serial imports
import { connectSerial } from './serial/SerialHandler.js';

// Playback imports
import { initPlaybackControls, tickPlayback, updateFromLog } from './playback/LogPlayer.js';

// UI imports
import { updateUI, toggleConfig, toggleTelemetry, updateOffset, updateAGLDisplay, setStatusMessage, updateFPSDisplay, initMoreMenu, initConfigAutoClose, initHudCells } from './ui/UIController.js';

// Split view imports
import {
    toggleViewMode, getViewMode, sampleDataPoint, updateSplitMap,
    updatePlotly, resizeSplitView, isPlotlyInitialized, setSplitMapSatelliteEnabled,
    updateND,
    recordLivePathPoint,
    is3DVisible, is2DMapVisible, isNDVisible
} from './ui/SplitView.js';

// MAVLink imports
import { initMAVLink, onMessage } from './mavlink/MAVLinkManager.js';
import { setParameter, requestParameter } from './mavlink/CommandSender.js';

// GCS imports
import { initCommandBar, updateCommandBar } from './ui/CommandBarController.js';
import { initGCSSidebar, updateGCSSidebar, getTargetCoords } from './ui/GCSSidebarController.js';
import { initTabs, getCurrentTab } from './ui/TabController.js';
import { initParamsPage, toggleParamsPage } from './ui/ParametersPageController.js';

import { setTerrainSatelliteEnabled } from './terrain/TerrainManager.js';

// FPV imports
import { initFPV, onFPVButtonClick, resizeFPV, isFPVActive, openFPVSettings, stopFPVStream } from './ui/FPVController.js';

// Loading overlay imports
import {
    showLoadingOverlay, hideLoadingOverlay, scheduleHideLoadingOverlaySoon,
    checkInitialLoadComplete, setAutoLoadAttempted
} from './ui/LoadingOverlay.js';

// CRV Logger import
import { CRVLogger } from './logging/CRVLogger.js';

// ============== CAMERA / MODEL (1P / 3P) ==============
let cameraMode = 'FIRST'; // 'FIRST' | 'THIRD'
let vehicle = null;
let vehicleLoadStarted = false;
let vehicleLoadFailed = false;
let currentModelName = '';
let modelScale = 1.0;
let loadedModel = null; // Reference to the currently loaded 3D model

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

        if (savedModel && models.includes(savedModel)) {
            select.value = savedModel;
            loadModel(savedModel);
        } else {
            loadModel(models[0]);
        }
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
 * Update the scale of the current model
 */
function updateModelScale(scale) {
    modelScale = scale;
    localStorage.setItem('modelScale', scale.toString());
    
    if (loadedModel) {
        loadedModel.scale.set(scale, scale, scale);
    }
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
    
    sunLight.castShadow = true;
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
        if (!mesh.geometry.boundingSphere) {
            mesh.geometry.computeBoundingSphere();
        }
        mesh.visible = frustum.intersectsObject(mesh);
    }
}

// ============== 3D WORLD UPDATE ==============
function update3DWorld() {
    const camera = getCamera();
    if (!camera) return;
    
    // Check if 3D view is visible - skip expensive rendering if not
    const visible3D = is3DVisible();
    
    const planePos = latLonToMeters(STATE.lat, STATE.lon);
    let totalAlt = STATE.rawAlt + STATE.offsetAlt;

    // Update smoothed attitude for jitter-free rendering
    updateSmoothedAttitude();

    const terrHeight = getTerrainElevationCached(STATE.lat, STATE.lon);
    if (terrHeight !== null) {
        STATE.terrainHeight = terrHeight;
        // Ensure vehicle renders above terrain (visual clamp only, doesn't modify STATE.rawAlt)
        if (totalAlt < terrHeight + 0.5) {
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
        camera.rotation.x = smoothAtt.pitch;
        camera.rotation.z = -smoothAtt.roll;
        camera.rotation.y = -smoothAtt.yaw;
    }

    // Update predicted trajectory corridor (throttled to every 3rd frame)
    if (trajectoryEnabled && visible3D) {
        if (!update3DWorld._trajFc) update3DWorld._trajFc = 0;
        if (++update3DWorld._trajFc % 3 === 0) {
            const predTime = getPredictionTime(STATE.gs || 0);
            const path = computePredictedPath(STATE, 40, predTime);
            updateCorridor(path);
        }
    }

    if (STATE.mode !== 'PLAYBACK') {
        updateTrail(planePos.x, totalAlt, planePos.z);
    }

    if ((STATE.mode === 'PLAYBACK' || STATE.connected) && STATE.lastReloadPos.lat) {
        const distFromLastReload = calculateDistance(
            STATE.lastReloadPos.lat,
            STATE.lastReloadPos.lon,
            STATE.lat,
            STATE.lon
        );

        const reloadDistance = (STATE.mode === 'PLAYBACK')
            ? (RELOAD_DISTANCE * PLAYBACK_RELOAD_DISTANCE_MULTIPLIER)
            : RELOAD_DISTANCE;

        const nowMs = performance.now();
        const cooldownOk = (STATE.mode !== 'PLAYBACK') || ((nowMs - lastMapReloadAt) >= PLAYBACK_RELOAD_COOLDOWN_MS);

        if (distFromLastReload > reloadDistance && cooldownOk) {
            reloadMapAndRunways();
        }
    }

    const dist = Math.sqrt(
        (planePos.x - STATE.lastUpdatePos.x) ** 2 + 
        (planePos.z - STATE.lastUpdatePos.z) ** 2
    );
    
    // Only update terrain and render if 3D is visible
    if (visible3D) {
        if (getHGTFileCount() > 0 && (Object.keys(getActiveChunks()).length === 0 || dist > 2000)) {
            STATE.lastUpdatePos = { x: planePos.x, z: planePos.z };
            updateTerrainChunks();
        }

        updateChunkVisibility();
        updateWireframeProximity();
        render();
    }
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

// ============== RESIZE HANDLER ==============
function handleResize() {
    const viewMode = getViewMode();
    setHUDViewMode(viewMode);

    const { width, height } = resizeHUD();
    resize(width, height);

    invalidateMapSize();
    resizeSplitView();
    resizeFPV();
}

// ============== FPS COUNTER ==============
let fpsFrameCount = 0;
let fpsLastTime = performance.now();
let lastFrameTime = performance.now();
let lastRenderTime = 0; // Throttle heavy 3D render to TARGET_RENDER_FPS
const TARGET_RENDER_FPS = 60;
const RENDER_INTERVAL = 1000 / TARGET_RENDER_FPS; // ~16.6ms
let demoTargetChangeTime = 0;


// Demo speed smoothing (m/s)
let demoSpeed = DEMO_BASE_SPEED;
let demoSpeedTarget = DEMO_BASE_SPEED;
let demoSpeedVel = 0;

// Storyline panel placement (move into split-map panel during playback)
let storylineOriginalParent = null;
let storylineOriginalNextSibling = null;

function updateStorylinePanelPlacement() {
    const panel = document.getElementById('storyline-panel');
    if (!panel) return;

    if (!storylineOriginalParent) {
        storylineOriginalParent = panel.parentElement;
        storylineOriginalNextSibling = panel.nextSibling;
    }

    const wantSplitMap = (getViewMode() === 'SPLIT') && (STATE.mode === 'PLAYBACK');
    const targetParent = wantSplitMap
        ? document.getElementById('split-map-container')
        : storylineOriginalParent;

    if (!targetParent) return;

    if (panel.parentElement !== targetParent) {
        if (!wantSplitMap && storylineOriginalParent) {
            // Restore original ordering in the DOM when going back.
            if (storylineOriginalNextSibling && storylineOriginalNextSibling.parentElement === storylineOriginalParent) {
                storylineOriginalParent.insertBefore(panel, storylineOriginalNextSibling);
            } else {
                storylineOriginalParent.appendChild(panel);
            }
        } else {
            targetParent.appendChild(panel);
        }
    }
}

// Playback trail cache (world positions per log entry)
let playbackWorldCache = null;
let playbackWorldCacheLen = 0;
let lastPlaybackTrailIndex = -1;
const MAX_PLAYBACK_TRAIL_POINTS = 20000;

function buildPlaybackWorldCache() {
    if (!Array.isArray(STATE.logData) || STATE.logData.length === 0) {
        playbackWorldCache = null;
        playbackWorldCacheLen = 0;
        lastPlaybackTrailIndex = -1;
        return;
    }
    if (playbackWorldCache && playbackWorldCacheLen === STATE.logData.length) return;

    playbackWorldCache = STATE.logData.map((r) => {
        const s = (r && (r.state || r)) || {};
        const lat = (typeof s.lat === 'number') ? s.lat : STATE.lat;
        const lon = (typeof s.lon === 'number') ? s.lon : STATE.lon;
        const alt = (typeof s.alt === 'number') ? s.alt : 0;
        const pos = latLonToMeters(lat, lon);
        const y = alt + STATE.offsetAlt;
        return { x: pos.x, y, z: pos.z };
    });
    playbackWorldCacheLen = STATE.logData.length;
    lastPlaybackTrailIndex = -1;
}

function downsampleWorld(points, maxPoints) {
    if (!points || points.length <= maxPoints) return points;
    const step = Math.ceil(points.length / maxPoints);
    const out = [];
    for (let i = 0; i < points.length; i += step) out.push(points[i]);
    const last = points[points.length - 1];
    if (out.length === 0 || out[out.length - 1] !== last) out.push(last);
    return out;
}

function updatePlaybackTrailIfNeeded() {
    if (STATE.mode !== 'PLAYBACK') {
        // Reset caches when exiting playback
        lastPlaybackTrailIndex = -1;
        return;
    }
    if (!Array.isArray(STATE.logData) || STATE.logData.length === 0) return;

    buildPlaybackWorldCache();
    const idx = Math.max(0, Math.min(STATE.logIndex, playbackWorldCacheLen - 1));
    if (idx === lastPlaybackTrailIndex || !playbackWorldCache) return;

    const prefix = playbackWorldCache.slice(0, idx + 1);
    setTrailPoints(downsampleWorld(prefix, MAX_PLAYBACK_TRAIL_POINTS));
    lastPlaybackTrailIndex = idx;
}

// Map reload throttling (helps in PLAYBACK where position can advance faster)
let lastMapReloadAt = 0;
const PLAYBACK_RELOAD_DISTANCE_MULTIPLIER = 3;
const PLAYBACK_RELOAD_COOLDOWN_MS = 1500;

function updateFPS() {
    fpsFrameCount++;
    const now = performance.now();
    if (now - fpsLastTime >= 1000) {
        updateFPSDisplay(fpsFrameCount);
        fpsFrameCount = 0;
        fpsLastTime = now;
    }
}

// ============== ANIMATION LOOP ==============
function animate() {
    requestAnimationFrame(animate);
    
    const now = performance.now();
    const deltaTime = (now - lastFrameTime) / 1000;
    lastFrameTime = now;
    
    updateFPS();
    updateSunPosition();

    // Playback handling
    if (STATE.mode === 'PLAYBACK' && STATE.isPlaying && STATE.logData.length > 0) {
        tickPlayback(now);
    }

    // Demo mode - fixed-wing survey drone
    if (STATE.mode === 'LIVE' && !STATE.connected) {
        const metersPerLat = 111320;
        const headingSelected = ndConfig && ndConfig.selectedHdg !== null;
        const selectedHdgRad = headingSelected ? ((ndConfig.selectedHdg % 360) / RAD) : null;
        const sv = demoSurveyState;

        // HDG SEL override: steer toward selected heading
        if (headingSelected && selectedHdgRad !== null) {
            demoAttitude.yaw.target = selectedHdgRad;
            demoAttitude.roll.target = 0;
            demoAttitude.pitch.target = 0;
            sv.turning = false;
        } else {
            // Survey pattern state machine
            const dist = Math.max(10, demoSpeed) * deltaTime;

            if (sv.turning) {
                // Smooth 180-degree turn between survey legs
                sv.turnProgress += deltaTime / 10; // ~10s per turn
                if (sv.turnProgress >= 1.0) {
                    // Turn complete
                    sv.turning = false;
                    sv.turnProgress = 0;
                    sv.distOnLeg = 0;
                    sv.legIndex++;
                    sv.direction *= -1; // alternate turn direction
                    sv.legHeading = (sv.legHeading + Math.PI) % (Math.PI * 2);
                    demoAttitude.yaw.target = sv.legHeading;
                    demoAttitude.roll.target = 0;
                    demoAttitude.pitch.target = (Math.random() - 0.5) * DEMO_PITCH_RANGE * 0.5;
                } else {
                    // During turn: interpolate heading, apply bank
                    const turnAngle = Math.PI * sv.turnProgress; // 0 -> PI
                    demoAttitude.yaw.target = sv.legHeading + turnAngle * sv.direction;
                    demoAttitude.roll.target = sv.direction * DEMO_ROLL_RANGE * 0.5 *
                        Math.sin(sv.turnProgress * Math.PI); // smooth bank envelope
                    demoAttitude.pitch.target = DEMO_PITCH_RANGE * 0.3; // slight nose-up in turn
                }
            } else {
                // Straight survey leg
                sv.distOnLeg += dist;
                demoAttitude.roll.target = (Math.random() - 0.5) * 0.02; // near-level wings
                demoAttitude.pitch.target = (Math.random() - 0.5) * DEMO_PITCH_RANGE * 0.3;
                demoAttitude.yaw.target = sv.legHeading;

                if (sv.distOnLeg >= DEMO_LEG_LENGTH) {
                    // Start turn
                    sv.turning = true;
                    sv.turnProgress = 0;
                }
            }

            // Vary speed slightly during legs
            if (now - demoTargetChangeTime > DEMO_TARGET_INTERVAL) {
                demoTargetChangeTime = now;
                demoSpeedTarget = DEMO_BASE_SPEED + (Math.random() - 0.5) * DEMO_SPEED_VARIANCE * 2;
            }
        }

        const smoothFactor = DEMO_SMOOTHING;
        const dampFactor = 0.92;

        demoAttitude.pitch.velocity = demoAttitude.pitch.velocity * dampFactor +
            (demoAttitude.pitch.target - demoAttitude.pitch.current) * smoothFactor;
        demoAttitude.pitch.current += demoAttitude.pitch.velocity;

        demoAttitude.roll.velocity = demoAttitude.roll.velocity * dampFactor +
            (demoAttitude.roll.target - demoAttitude.roll.current) * smoothFactor;
        demoAttitude.roll.current += demoAttitude.roll.velocity;

        demoAttitude.yaw.velocity = demoAttitude.yaw.velocity * dampFactor +
            (demoAttitude.yaw.target - demoAttitude.yaw.current) * smoothFactor;
        demoAttitude.yaw.current += demoAttitude.yaw.velocity;

        // Smooth speed changes
        demoSpeedVel = demoSpeedVel * dampFactor + (demoSpeedTarget - demoSpeed) * smoothFactor;
        demoSpeed += demoSpeedVel;
        const speed = Math.max(10, demoSpeed);

        STATE.pitch = demoAttitude.pitch.current;
        STATE.roll = demoAttitude.roll.current;

        const twoPi = Math.PI * 2;
        STATE.yaw = ((demoAttitude.yaw.current % twoPi) + twoPi) % twoPi;

        // Move according to yaw + pitch
        const moveDist = speed * deltaTime;
        const cosPitch = Math.cos(STATE.pitch);
        const sinPitch = Math.sin(STATE.pitch);
        const northMeters = Math.cos(STATE.yaw) * (moveDist * cosPitch);
        const eastMeters = Math.sin(STATE.yaw) * (moveDist * cosPitch);

        const metersPerLon = Math.max(1, metersPerLat * Math.cos(STATE.lat * Math.PI / 180));
        STATE.lat += northMeters / metersPerLat;
        STATE.lon += eastMeters / metersPerLon;

        // Terrain-following altitude: maintain AGL over terrain
        const terrainElev = STATE.terrainHeight !== null ? STATE.terrainHeight : 600;
        const targetAlt = terrainElev + DEMO_ALT_AGL;
        if (!Number.isFinite(STATE.rawAlt) || STATE.rawAlt <= 0) STATE.rawAlt = targetAlt;
        // Smooth altitude tracking toward target AGL
        STATE.rawAlt += (targetAlt - STATE.rawAlt) * 0.02;
        const upMeters = (targetAlt - STATE.rawAlt) * deltaTime;
        STATE.vs = deltaTime > 0 ? (upMeters / deltaTime) : 0;

        // Airspeed / groundspeed
        STATE.as = speed;
        STATE.gs = Math.abs(speed * cosPitch);

        // Simulate LiDAR rangefinder (downward-facing)
        if (STATE.terrainHeight !== null) {
            const agl = STATE.rawAlt - STATE.terrainHeight;
            const noise = (Math.random() - 0.5) * 0.04 + agl * 0.001 * (Math.random() - 0.5);
            STATE.rangefinderDist = Math.max(0.01, agl + noise);
        } else {
            STATE.rangefinderDist = null;
        }

        // G-load: ~1g with small turn-load variation
        STATE.ax = 0;
        STATE.ay = 0;
        STATE.az = 9.81 * (1 + Math.min(0.15, Math.abs(STATE.roll) * 0.5));
        pushGHistory();
    }

    updateUI();
    sampleDataPoint();
    recordLivePathPoint();

    // Throttle heavy 3D rendering to 30fps max.
    // The animation loop runs at monitor refresh rate (60-144Hz) but heavy GPU work
    // (3D render, terrain, frustum culling) is capped to free the main thread
    // for MAVLink parsing, RC radio input, and UI responsiveness.
    const onFlightDataTab = getCurrentTab() === 'flight-data';
    const renderDue = (now - lastRenderTime) >= RENDER_INTERVAL;

    if (onFlightDataTab && renderDue) {
        lastRenderTime = now;
        // Draw HUD when 3D view is visible (first-person) or FPV is active
        if (cameraMode !== 'THIRD' && (is3DVisible() || isFPVActive())) {
            drawHUD();
        }
        updatePlaybackTrailIfNeeded();
        update3DWorld();
        updateHomeMarker3D();

        // Check if nearby chunks need high-res textures
        if (is3DVisible()) {
            refreshNearbyChunkTextures();
        }

        if (getViewMode() === 'SPLIT') {
            updateSplitMap();
            updatePlotly();
            updateND();
        }
    }

    // Update GCS command bar, sidebar, and mini-map
    updateCommandBar();
    updateGCSSidebar();
    const tc = getTargetCoords();
    if (tc) {
        const tElev = getTerrainElevationCached(tc.lat, tc.lon);
        updateTargetMarker3D(tElev);
    }
    updateTrafficMarkers3D(getNearestTraffic(4));
    if (getViewMode() !== 'SPLIT') updateMap();

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
    try { setSplitMapSatelliteEnabled(window.satelliteEnabled); } catch (e) {}

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
        sunLight.castShadow = true;
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
    ndConfig.showPredictedPath = trajectoryEnabled;
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
        const files = await window.topography.load('topography');

        if (!files || files.length === 0) {
            setAutoLoadAttempted();
            setStatusMessage('AUTO HGT LOAD: no files found', '#ffcc00');
            scheduleHideLoadingOverlaySoon();
            return;
        }

        let c = 0;
        for (const f of files) {
            try {
                let data = f.arrayBuffer || f.arraybuffer || f.buffer || null;
                if (data && data.buffer) data = data.buffer;
                if (!data) continue;

                const file = new File([data], f.name, { type: 'application/octet-stream' });
                addHGTFile(f.name, file);
                c++;
            } catch (err) {
                console.warn('Error creating File from topography payload', f && f.name, err);
            }
        }

        if (c > 0) {
            setStatusMessage(`${c} HGT LOADED (auto)`, 'var(--accent-cyan)');
            // Start terrain loading - setAutoLoadAttempted will be called 
            // by checkInitialLoadComplete once chunks start appearing
            updateTerrainChunks();
        } else {
            setAutoLoadAttempted();
            setStatusMessage('AUTO HGT LOAD: no valid files', '#ffcc00');
            scheduleHideLoadingOverlaySoon();
        }
    } catch (e) {
        console.warn('Topography load failed', e);
        setAutoLoadAttempted();
        setStatusMessage('AUTO HGT LOAD: error', '#ff4444');
        scheduleHideLoadingOverlaySoon();
    }
}

// ============== CONNECTIVITY CHECK ==============
async function checkConnectivity() {
    const TEST_URL = 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/0/0/0';
    try {
        const controller = new AbortController();
        const timeout = setTimeout(() => controller.abort(), 5000);
        const res = await fetch(TEST_URL, { mode: 'cors', signal: controller.signal });
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
    
    // Initialize HUD
    initHUD(document.getElementById('hud-canvas'));
    
    // Initialize playback controls
    initPlaybackControls();

    // Keep HUD numeric readouts in sync during playback scrubbing/stepping.
    window.addEventListener('logUpdate', () => {
        updateUI();
    });

    // Ensure path/trail snaps correctly on log load/seek.
    window.addEventListener('logLoaded', () => {
        playbackWorldCache = null;
        playbackWorldCacheLen = 0;
        lastPlaybackTrailIndex = -1;
        resetTrail();
        updatePlaybackTrailIfNeeded();
        updateStorylinePanelPlacement();
        // Force high-res texture reload for new position
        resetTextureRefreshPosition();
    });
    window.addEventListener('logSeek', () => {
        lastPlaybackTrailIndex = -1;
        updatePlaybackTrailIfNeeded();
        updateStorylinePanelPlacement();
    });
    
    // Setup event listeners
    window.onresize = handleResize;
    setupHGTInput();
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
    initCommandBar();
    initGCSSidebar();
    initHudCells();
    initTabs();
    initMap('mini-map');
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
    onMessage(253, (data) => {
        const sev = data.severity ?? 6;
        const prefix = SEVERITY_LEVELS[sev] || 'INFO';
        const text = `[${prefix}] ${data.text || ''}`;
        const level = sev <= 3 ? 'error' : sev <= 4 ? 'warning' : 'info';
        pushHudMessage(text, level);
    });

    // Show COMMAND_ACK results on HUD
    window.addEventListener('commandAck', (e) => {
        const { cmdName, resultName, level } = e.detail;
        pushHudMessage(`${cmdName}: ${resultName}`, level);
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

    // Initial placement for playback controls
    updateStorylinePanelPlacement();
    
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
window.toggleViewMode = () => {
    const mode = toggleViewMode();
    handleResize();
    updateStorylinePanelPlacement();
    return mode;
};
window.onFPVButtonClick = onFPVButtonClick;

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
    pushHudMessage(_gcsMuted ? 'GCS output MUTED — no messages will be sent' : 'GCS output UNMUTED', _gcsMuted ? 'warning' : 'info');
};

// Clear flight trail
window.clearTrail = function() {
    resetTrail();
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

// Download traffic CSV from menu
window.downloadTraffic = function() {
    if (STATE.traffic.length === 0) {
        pushHudMessage('No traffic data to download', 'warning');
        return;
    }
    downloadTrafficCSV();
    pushHudMessage(`Traffic CSV downloaded (${STATE.traffic.length} entries)`);
};

// CRV recording
const _crvLogger = new CRVLogger();
window.toggleRecording = () => _crvLogger.toggleRecording();

// Initialize when DOM is ready
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
} else {
    init();
}
