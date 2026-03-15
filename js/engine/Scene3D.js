/**
 * Scene3D.js - Three.js Scene Management
 * Handles 3D scene initialization, camera, renderer, lighting
 */

import { CAMERA_FOV, VISIBILITY_RADIUS } from '../core/constants.js';
import { STATE, demoAttitude } from '../core/state.js';
import { latLonToMeters } from '../core/utils.js';

// Module-level references
let scene, camera, renderer;
let sunLight, ambientLight;
let trailLine, trailIdx = 0;

// Mission trajectory 3D visualization
let missionLine = null;
let missionWpMarkers = [];
let missionAltLines = [];

// Trail limit to prevent memory leak on long sessions
const MAX_TRAIL_POINTS = 50000;

// Shadow chunk tracking
const SHADOW_CHUNK_SIZE = 5000;
let lastShadowChunkX = null;
let lastShadowChunkZ = null;

// Sun direction for hillshading
let currentSunDirection = null;
let sunlightEnabled = true;
let timeOverride = null;

/**
 * Initialize the 3D scene
 * @param {HTMLElement} container - Container element for renderer
 */
export function init3D(container) {
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x87ceeb);
    scene.fog = new THREE.FogExp2(0x87ceeb, 0.00005);

    camera = new THREE.PerspectiveCamera(
        CAMERA_FOV, 
        window.innerWidth / window.innerHeight, 
        1, 
        300000
    );
    
    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    // Prevent canvas from stealing focus from input fields
    renderer.domElement.tabIndex = -1;
    renderer.domElement.style.outline = 'none';
    container.appendChild(renderer.domElement);

    // Initialize lighting
    initLighting();

    // Grid helper
    const grid = new THREE.GridHelper(50000, 500, 0x333333, 0x111111);
    scene.add(grid);

    // Trail line
    initTrailLine();

    // Mission trajectory line
    initMissionLine();

    // Initialize sun direction vector
    currentSunDirection = new THREE.Vector3(0, 1, 0);

    return { scene, camera, renderer };
}

/**
 * Initialize scene lighting
 */
function initLighting() {
    // Sun directional light with shadows
    sunLight = new THREE.DirectionalLight(0xffffff, 1.5);
    sunLight.position.set(20000, 30000, 10000);
    sunLight.castShadow = true;
    sunLight.shadow.mapSize.width = 4096;
    sunLight.shadow.mapSize.height = 4096;
    sunLight.shadow.camera.near = 1;
    sunLight.shadow.camera.far = 50000;
    // Shadow camera area matches SHADOW_CHUNK_SIZE for better resolution
    // 5000x5000 area with 2048x2048 map = ~2.4m per pixel
    const shadowHalfSize = SHADOW_CHUNK_SIZE * 0.6;
    sunLight.shadow.camera.left = -shadowHalfSize;
    sunLight.shadow.camera.right = shadowHalfSize;
    sunLight.shadow.camera.top = shadowHalfSize;
    sunLight.shadow.camera.bottom = -shadowHalfSize;
    sunLight.shadow.bias = -0.0005;
    sunLight.shadow.normalBias = 0.02;
    sunLight.shadow.radius = 2;
    scene.add(sunLight);
    scene.add(sunLight.target);

    // Ambient light
    ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambientLight);
}

/**
 * Initialize trail line for flight path visualization
 */
function initTrailLine() {
    const trailGeo = new THREE.BufferGeometry();
    trailGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(60000), 3));
    trailLine = new THREE.Line(
        trailGeo, 
        new THREE.LineBasicMaterial({ color: 0xff0000, opacity: 0.8, transparent: true })
    );
    trailLine.frustumCulled = false;
    scene.add(trailLine);
}

function ensureTrailCapacity(pointCount) {
    if (!trailLine) return;
    const posAttr = trailLine.geometry.attributes.position;
    const currentPoints = Math.floor(posAttr.array.length / 3);
    if (pointCount <= currentPoints) return;

    // Grow to next power-ish to avoid frequent reallocations.
    let newPoints = currentPoints;
    while (newPoints < pointCount) newPoints *= 2;
    const newArray = new Float32Array(newPoints * 3);
    newArray.set(posAttr.array);
    trailLine.geometry.setAttribute('position', new THREE.BufferAttribute(newArray, 3));
}

/**
 * Update trail with new position
 * @param {number} x - X position
 * @param {number} y - Y position (altitude)
 * @param {number} z - Z position
 */
export function updateTrail(x, y, z) {
    if (!trailLine) return;

    // Check if we've hit the limit - downsample the trail by 2x to free space
    if (trailIdx >= MAX_TRAIL_POINTS) {
        downsampleTrail();
    }

    ensureTrailCapacity(trailIdx + 1);

    const posAttr = trailLine.geometry.attributes.position;
    const pos = posAttr.array;

    pos[trailIdx * 3] = x;
    pos[trailIdx * 3 + 1] = y;
    pos[trailIdx * 3 + 2] = z;
    trailIdx++;
    trailLine.geometry.setDrawRange(0, trailIdx);
    posAttr.updateRange.offset = Math.max(0, (trailIdx - 1) * 3);
    posAttr.updateRange.count = 3;
    posAttr.needsUpdate = true;
}

/**
 * Downsample trail by keeping every other point
 * Called when MAX_TRAIL_POINTS is reached
 */
function downsampleTrail() {
    if (!trailLine || trailIdx < 2) return;

    const posAttr = trailLine.geometry.attributes.position;
    const pos = posAttr.array;

    // Keep every other point (downsample 2:1)
    let writeIdx = 0;
    for (let readIdx = 0; readIdx < trailIdx; readIdx += 2) {
        const ro = readIdx * 3;
        const wo = writeIdx * 3;
        pos[wo] = pos[ro];
        pos[wo + 1] = pos[ro + 1];
        pos[wo + 2] = pos[ro + 2];
        writeIdx++;
    }

    trailIdx = writeIdx;
    trailLine.geometry.setDrawRange(0, trailIdx);
    posAttr.needsUpdate = true;
}

/**
 * Clear the trail line.
 */
export function resetTrail() {
    if (!trailLine) return;
    trailIdx = 0;
    trailLine.geometry.setDrawRange(0, 0);
    const posAttr = trailLine.geometry.attributes.position;
    posAttr.updateRange.offset = 0;
    posAttr.updateRange.count = 0;
    posAttr.needsUpdate = true;
}

/**
 * Set the trail line from a list of points.
 * @param {Array<{x:number,y:number,z:number}>} points
 */
export function setTrailPoints(points) {
    if (!trailLine) return;
    if (!Array.isArray(points) || points.length === 0) {
        resetTrail();
        return;
    }

    ensureTrailCapacity(points.length);
    const posAttr = trailLine.geometry.attributes.position;
    const pos = posAttr.array;

    for (let i = 0; i < points.length; i++) {
        const p = points[i];
        const o = i * 3;
        pos[o] = p.x;
        pos[o + 1] = p.y;
        pos[o + 2] = p.z;
    }

    trailIdx = points.length;
    trailLine.geometry.setDrawRange(0, trailIdx);
    posAttr.updateRange.offset = 0;
    posAttr.updateRange.count = trailIdx * 3;
    posAttr.needsUpdate = true;
}

/**
 * Initialize mission trajectory line
 */
function initMissionLine() {
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(300), 3));
    missionLine = new THREE.Line(geo, new THREE.LineBasicMaterial({
        color: 0x44ff44,
        opacity: 0.85,
        transparent: true
    }));
    missionLine.frustumCulled = false;
    missionLine.renderOrder = 1;
    scene.add(missionLine);
}

/**
 * Update mission trajectory 3D visualization
 * @param {Array<{x:number, y:number, z:number}>} points - World-space waypoint positions
 */
export function updateMissionTrajectory(points) {
    if (!missionLine || !scene) return;

    // Clear old markers and altitude lines
    clearMissionMarkers();

    if (!points || points.length === 0) {
        missionLine.geometry.setDrawRange(0, 0);
        return;
    }

    // Hide the connecting line
    missionLine.geometry.setDrawRange(0, 0);

    // Create waypoint spheres and altitude lines
    const wpMat = new THREE.MeshBasicMaterial({ color: 0x44ff44, transparent: true, opacity: 0.9 });
    const wpGeo = new THREE.SphereGeometry(1.2, 8, 6);
    const altLineMat = new THREE.LineBasicMaterial({ color: 0x44ff44, opacity: 0.3, transparent: true });

    for (let i = 0; i < points.length; i++) {
        const p = points[i];

        // Waypoint sphere
        const sphere = new THREE.Mesh(wpGeo, wpMat);
        sphere.position.set(p.x, p.y, p.z);
        sphere.frustumCulled = false;
        scene.add(sphere);
        missionWpMarkers.push(sphere);

        // Vertical altitude line (from ground y=0 to waypoint)
        const altGeo = new THREE.BufferGeometry();
        altGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array([
            p.x, 0, p.z,
            p.x, p.y, p.z
        ]), 3));
        const altLine = new THREE.Line(altGeo, altLineMat);
        altLine.frustumCulled = false;
        scene.add(altLine);
        missionAltLines.push(altLine);
    }
}

/**
 * Clear mission trajectory markers and altitude lines
 */
function clearMissionMarkers() {
    for (const m of missionWpMarkers) {
        scene.remove(m);
        if (m.geometry) m.geometry.dispose();
    }
    missionWpMarkers.length = 0;

    for (const l of missionAltLines) {
        scene.remove(l);
        if (l.geometry) l.geometry.dispose();
    }
    missionAltLines.length = 0;
}

/**
 * Clear entire mission trajectory (line + markers)
 */
export function clearMissionTrajectory() {
    clearMissionMarkers();
    if (missionLine) {
        missionLine.geometry.setDrawRange(0, 0);
    }
}

/**
 * Update camera position and rotation
 */
export function updateCamera() {
    if (!camera) return;
    
    const planePos = latLonToMeters(STATE.lat, STATE.lon);
    let totalAlt = STATE.rawAlt + STATE.offsetAlt;

    camera.position.set(planePos.x, Math.max(totalAlt, 1), planePos.z);
    camera.rotation.order = 'YXZ';
    camera.rotation.x = STATE.pitch;
    camera.rotation.z = -STATE.roll;
    camera.rotation.y = -STATE.yaw;
}

/**
 * Render the scene
 */
export function render() {
    if (renderer && scene && camera) {
        renderer.render(scene, camera);
    }
}

/**
 * Resize renderer
 * @param {number} width 
 * @param {number} height 
 */
export function resize(width, height) {
    if (camera) {
        camera.aspect = width / height;
        camera.clearViewOffset();
        camera.updateProjectionMatrix();
    }
    if (renderer) {
        renderer.setSize(width, height);
    }
}

// Home position 3D marker
let homeMarker3D = null;
let homeMarkerLastLat = null;
let homeMarkerLastLon = null;

// Target marker 3D
let targetMarker3D = null;

// ADS-B traffic 3D markers
let trafficMarkers3D = [];

/**
 * Update or create a 3D home position marker
 */
export function updateHomeMarker3D() {
    if (!scene) return;
    if (STATE.homeLat === null || STATE.homeLon === null) return;

    // Skip if position hasn't changed
    if (homeMarkerLastLat === STATE.homeLat && homeMarkerLastLon === STATE.homeLon) return;
    homeMarkerLastLat = STATE.homeLat;
    homeMarkerLastLon = STATE.homeLon;

    // Remove old marker
    if (homeMarker3D) {
        scene.remove(homeMarker3D);
        homeMarker3D.traverse(c => { if (c.geometry) c.geometry.dispose(); if (c.material) c.material.dispose(); });
    }

    const pos = latLonToMeters(STATE.homeLat, STATE.homeLon);
    const alt = (STATE.homeAlt || 0) + (STATE.offsetAlt || 0);

    const group = new THREE.Group();

    // Vertical pole
    const poleGeo = new THREE.CylinderGeometry(1, 1, 80, 8);
    const poleMat = new THREE.MeshBasicMaterial({ color: 0xff8800 });
    const pole = new THREE.Mesh(poleGeo, poleMat);
    pole.position.y = 40;
    group.add(pole);

    // "H" marker sphere on top
    const sphereGeo = new THREE.SphereGeometry(8, 16, 16);
    const sphereMat = new THREE.MeshBasicMaterial({ color: 0xff8800 });
    const sphere = new THREE.Mesh(sphereGeo, sphereMat);
    sphere.position.y = 85;
    group.add(sphere);

    // Ring around sphere
    const ringGeo = new THREE.TorusGeometry(12, 1.5, 8, 32);
    const ringMat = new THREE.MeshBasicMaterial({ color: 0xffffff });
    const ring = new THREE.Mesh(ringGeo, ringMat);
    ring.position.y = 85;
    ring.rotation.x = Math.PI / 2;
    group.add(ring);

    group.position.set(pos.x, alt, pos.z);
    scene.add(group);
    homeMarker3D = group;
}

/**
 * Set or update 3D target marker at given coordinates
 * Marker is positioned at terrain level (Y updated via updateTargetMarker3D)
 */
export function setTargetMarker3D(lat, lon) {
    if (!scene) return;
    clearTargetMarker3D();

    const pos = latLonToMeters(lat, lon);
    const group = new THREE.Group();

    // Vertical pole (red)
    const poleGeo = new THREE.CylinderGeometry(1.2, 1.2, 100, 8);
    const poleMat = new THREE.MeshBasicMaterial({ color: 0xff0000 });
    const pole = new THREE.Mesh(poleGeo, poleMat);
    pole.position.y = 50;
    group.add(pole);

    // Top sphere
    const sphereGeo = new THREE.SphereGeometry(10, 16, 16);
    const sphereMat = new THREE.MeshBasicMaterial({ color: 0xff0000 });
    const sphere = new THREE.Mesh(sphereGeo, sphereMat);
    sphere.position.y = 105;
    group.add(sphere);

    // Ring around sphere
    const ringGeo = new THREE.TorusGeometry(15, 2, 8, 32);
    const ringMat = new THREE.MeshBasicMaterial({ color: 0xff4444 });
    const ring = new THREE.Mesh(ringGeo, ringMat);
    ring.position.y = 105;
    ring.rotation.x = Math.PI / 2;
    group.add(ring);

    group.position.set(pos.x, 0, pos.z);
    scene.add(group);
    targetMarker3D = group;
}

/**
 * Update 3D target marker Y position to match terrain elevation
 * @param {number|null} terrainElevation - ground elevation in meters
 */
export function updateTargetMarker3D(terrainElevation) {
    if (!targetMarker3D) return;
    const y = (terrainElevation || 0) + (STATE.offsetAlt || 0);
    targetMarker3D.position.y = y;
}

/**
 * Remove 3D target marker
 */
export function clearTargetMarker3D() {
    if (!targetMarker3D || !scene) return;
    scene.remove(targetMarker3D);
    targetMarker3D.traverse(c => { if (c.geometry) c.geometry.dispose(); if (c.material) c.material.dispose(); });
    targetMarker3D = null;
}

/**
 * Update 3D traffic markers (wireframe pyramids) for nearest ADS-B traffic
 * @param {Array} nearest - Array from getNearestTraffic() with lat, lon, alt, dist
 */
export function updateTrafficMarkers3D(nearest) {
    if (!scene) return;

    // Remove old markers
    for (const m of trafficMarkers3D) {
        scene.remove(m);
        m.traverse(c => { if (c.geometry) c.geometry.dispose(); if (c.material) c.material.dispose(); });
    }
    trafficMarkers3D = [];

    if (!nearest || nearest.length === 0) return;

    for (const ac of nearest) {
        if (ac.lat == null || ac.lon == null) continue;
        const pos = latLonToMeters(ac.lat, ac.lon);
        const alt = (ac.alt || 0) + (STATE.offsetAlt || 0);

        // Inverted wireframe pyramid (cone pointing down)
        const coneGeo = new THREE.ConeGeometry(30, 50, 4);
        const coneMat = new THREE.MeshBasicMaterial({ color: 0xff2222, wireframe: true });
        const cone = new THREE.Mesh(coneGeo, coneMat);
        cone.rotation.x = Math.PI; // point down
        cone.position.set(pos.x, alt, pos.z);

        scene.add(cone);
        trafficMarkers3D.push(cone);
    }
}

// Getters
export function getScene() { return scene; }
export function getCamera() { return camera; }
export function getRenderer() { return renderer; }
export function getSunLight() { return sunLight; }
export function getAmbientLight() { return ambientLight; }
export function getCurrentSunDirection() { return currentSunDirection; }
export function isSunlightEnabled() { return sunlightEnabled; }
export function getTimeOverride() { return timeOverride; }
export function getShadowChunkSize() { return SHADOW_CHUNK_SIZE; }
export function getLastShadowChunk() { return { x: lastShadowChunkX, z: lastShadowChunkZ }; }

// Setters
export function setSunlightEnabled(enabled) { sunlightEnabled = enabled; }
export function setTimeOverride(time) { timeOverride = time; }
export function setLastShadowChunk(x, z) { 
    lastShadowChunkX = x; 
    lastShadowChunkZ = z; 
}
