/**
 * Scene3D.js - Three.js Scene Management
 * Handles 3D scene initialization, camera, renderer, lighting, and the
 * overlays drawn over the terrain: flight trail, mission route, home.
 */

import { CAMERA_FOV } from '../core/constants.js';
import { STATE } from '../core/state.js';
import { latLonToMeters } from '../core/utils.js';
import { setLodViewParams, setTerrainSchematicLight } from '../terrain/TerrainManager.js';
import { ThickLine, flushThickLines } from './ThickLine.js';
import { SymbolLayer, SHAPE, makeLabel, fitLabel, disposeLabel, setSymbolHalo, setLabelHalo } from './SymbolLayer.js';
import { WORLD_LAYER, OVERLAY_LAYER, toOverlayLayer } from './Layers.js';
import { setCorridorColor } from './TrajectoryCorridor3D.js';
import { initTraffic3D, updateTraffic3D, animateTraffic3D, resizeTraffic3D, setTrafficColor } from './Traffic3D.js';
import { initMission3D, updateMission3D, setMission3DActiveSeq, clearMission3D, setMission3DPalette, resizeMission3D } from './Mission3D.js';
import { initWater3D, updateWater3D } from './Water3D.js';

// Module-level references
let scene, camera, renderer;
let sunLight, ambientLight;
// When true, updateTrail() is a no-op — the trail is driven externally via
// setTrailPoints() (used during log replay to show the whole pre-recorded path).
let trailFrozen = false;

// Trail limit to prevent memory leak on long sessions
const MAX_TRAIL_POINTS = 50000;
// A trail point is only added once the vehicle has moved this far from the last
// one. Appending every rendered frame filled the buffer with duplicates while
// hovering or parked, and hit MAX_TRAIL_POINTS after ~25 min of flight.
const TRAIL_MIN_STEP_M = 2;
// A jump farther than this is not flight — a new connection after the demo,
// another vehicle, a log loaded: the trail starts again instead of drawing a
// line across the country.
const TRAIL_MAX_JUMP_M = 2000;

// Overlay colours: the same meaning as on the 2D mini-map (red trail and
// traffic, green route and waypoints, orange home), bright to hold up on
// imagery and on black; labels take their symbol's colour. Over the
// light-theme schematic view the ground is pale green: the greens deepen (the
// light UI theme's own accent green), the others darken a little, symbol
// halos turn white, and every label is dark blue with a thin black edge, like
// the readouts over the scene (body.scene-light). Circles, patterns and POIs
// take their segment colour from the flight plan page (RouteModel), a shade
// darker over the light ground.
const PALETTE_DARK = {
    trail: 0xff3b30, route: 0x44ff44, activeLeg: 0xd8ffd0, home: 0xff8800,
    ownship: 0xffffff, traffic: 0xff2a2a,
    circle: 0x4488ff, area: 0xff6600, corridor: 0xcc44ff, perimeter: 0xffaa00, poi: 0xff66aa,
    halo: [0, 0, 0, 0.6], label: null, labelHalo: 'rgba(0, 0, 0, 0.85)', labelHaloPx: 3.5
};
const PALETTE_LIGHT = {
    trail: 0xe3261c, route: 0x0a8f2a, activeLeg: 0x05561b, home: 0xd96a00,
    ownship: 0x10324a, traffic: 0xe3261c,
    circle: 0x1c5fd8, area: 0xd65400, corridor: 0x9a2ccc, perimeter: 0xb87800, poi: 0xd02c78,
    halo: [1, 1, 1, 0.8], label: 0x0a37a6, labelHalo: 'rgba(0, 0, 0, 0.6)', labelHaloPx: 1.6
};
let palette = PALETTE_DARK;

const HOME_POLE_M = 100;       // home symbol height above the ground
const HOME_RING_M = 25;        // radius of the ring around home on the ground

// Sun direction for hillshading
let currentSunDirection = null;
let sunlightEnabled = true;

// Backdrop. With the satellite imagery on the scene has a sky; off, it is the
// schematic view: black sky and black fog, so the terrain isolines fade into
// the dark with distance — gently enough that ridges 30 km away (the
// schematic radius) still read. With the light UI theme the schematic sky is
// a pale blue gradient (drawn by the outline pass) and the fog its horizon
// colour. In AR the backdrop is black and fog-free, since the canvas is
// screen-blended over the camera feed.
const SKY_FOG_DENSITY = 0.00005;
const SCHEMATIC_FOG_DENSITY = 0.00003;
const LIGHT_SKY_HORIZON = 0xe2eef7;
const LIGHT_SKY_ZENITH = 0x86bbe8;
let skyColor = 0x87ceeb;
let schematicView = false;
let arMode = false;
let lightTheme = false;

// Pooled materials and geometries (avoid per-call allocation)
const pooledGeo = {};
const pooledMat = {};
let timeOverride = null;

// Overlays (created in init3D; the mission route lives in Mission3D.js)
let trail = null;
let groundGrid = null;      // reference grid at 0 m under the terrain
let homePole = null;
let homeRing = null;
let homeSymbol = null;
let homeLabel = null;
let ownship = null;         // ring around the aircraft in the chase view

/**
 * Initialize the 3D scene
 * @param {HTMLElement} container - Container element for renderer
 */
export function init3D(container) {
    scene = new THREE.Scene();
    scene.background = new THREE.Color(skyColor);
    scene.fog = new THREE.FogExp2(skyColor, SKY_FOG_DENSITY);

    camera = new THREE.PerspectiveCamera(
        CAMERA_FOV,
        window.innerWidth / window.innerHeight,
        1,
        300000
    );
    // Both layers in one pass, except in the schematic view (renderSchematic)
    camera.layers.enable(OVERLAY_LAYER);

    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false, powerPreference: 'high-performance' });
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.setSize(window.innerWidth, window.innerHeight);
    // No shadow map: the vehicle was the only caster, and over a 6 km shadow
    // frustum its shadow covered one or two texels — a full extra pass plus PCF
    // sampling on every terrain fragment for something nobody could see.
    // Prevent canvas from stealing focus from input fields
    renderer.domElement.tabIndex = -1;
    renderer.domElement.style.outline = 'none';
    container.appendChild(renderer.domElement);

    // Initialize lighting
    initLighting();

    // Grid helper
    groundGrid = new THREE.GridHelper(50000, 500, 0x333333, 0x111111);
    scene.add(groundGrid);

    initOverlays();

    // Initialize sun direction vector
    currentSunDirection = new THREE.Vector3(0, 1, 0);

    // Initialize pooled geometries and materials
    pooledGeo.targetPole = new THREE.CylinderGeometry(1.2, 1.2, 100, 8);
    pooledGeo.targetSphere = new THREE.SphereGeometry(10, 16, 16);
    pooledGeo.targetRing = new THREE.TorusGeometry(15, 2, 8, 32);

    pooledMat.targetRed = new THREE.MeshBasicMaterial({ color: 0xff0000 });
    pooledMat.targetRing = new THREE.MeshBasicMaterial({ color: 0xff4444 });

    return { scene, camera, renderer };
}

/**
 * Initialize scene lighting
 */
function initLighting() {
    // Sun directional light (no shadows — see init3D)
    sunLight = new THREE.DirectionalLight(0xffffff, 1.5);
    sunLight.position.set(20000, 30000, 10000);
    sunLight.layers.enableAll();   // lights are layered too: keep them in every pass
    scene.add(sunLight);
    scene.add(sunLight.target);

    // Ambient light
    ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    ambientLight.layers.enableAll();
    scene.add(ambientLight);
}

/**
 * Lines and symbols drawn over the terrain. Widths and symbol sizes are in
 * screen pixels, so the trail, the route and home stay readable from any
 * distance; the parts hidden behind terrain are drawn again, faint.
 */
function initOverlays() {
    trail = new ThickLine({ color: palette.trail, width: 3.5, ghost: { width: 2, opacity: 0.35 }, renderOrder: 5 }).addTo(scene);
    initWater3D(scene);

    initMission3D(scene, camera, renderer, palette);

    homePole = new ThickLine({ mode: 'pairs', color: palette.home, width: 3, ghost: { width: 2, opacity: 0.4 }, renderOrder: 5 }).addTo(scene);
    homeRing = new ThickLine({ color: palette.home, width: 3, ghost: { width: 2, opacity: 0.4 }, renderOrder: 5 }).addTo(scene);
    homeSymbol = new SymbolLayer({ sizeNear: 46, sizeFar: 30, nearDist: 300, farDist: 8000, ghostOpacity: 0.5, renderOrder: 12 }).addTo(scene);
    homePole.setVisible(false);
    homeRing.setVisible(false);
    homeSymbol.setVisible(false);

    ownship = new SymbolLayer({ sizeNear: 34, sizeFar: 26, nearDist: 200, farDist: 3000, ghostOpacity: 0.5, renderOrder: 13 }).addTo(scene);
    ownship.setSymbols([{ x: 0, y: 0, z: 0, shape: SHAPE.CIRCLE, color: palette.ownship }]);
    ownship.setVisible(false);

    initTraffic3D(scene);
}

/**
 * Ring around the aircraft for the chase view, where the true-scale model
 * shrinks to a few pixels at the default orbit distance.
 * @param {THREE.Vector3|null} position world position, null to hide
 */
export function updateOwnshipMarker(position) {
    if (!ownship) return;
    ownship.setVisible(!!position);
    if (position) ownship.setPosition(0, position.x, position.y, position.z);
}

// ============== BACKDROP ==============

function applyBackdrop() {
    if (!scene) return;
    if (!scene.background || !scene.background.isColor) scene.background = new THREE.Color();
    if (arMode) {
        scene.background.setHex(0x000000);
        scene.fog = null;
        return;
    }
    const color = schematicView ? (lightTheme ? LIGHT_SKY_HORIZON : 0x000000) : skyColor;
    const density = schematicView ? SCHEMATIC_FOG_DENSITY : SKY_FOG_DENSITY;
    scene.background.setHex(color);
    if (!scene.fog) scene.fog = new THREE.FogExp2(color, density);
    scene.fog.color.setHex(color);
    scene.fog.density = density;
}

/** Light schematic look: light UI theme, satellite imagery off, not in AR. */
function isLightSchematic() {
    return lightTheme && schematicView && !arMode;
}

/**
 * Backdrop, terrain palette and overlay palette for the current view, theme
 * and AR state.
 */
function applyTheme() {
    applyBackdrop();
    setTerrainSchematicLight(lightTheme && !arMode);
    applyPalette();
}

function applyPalette() {
    const next = isLightSchematic() ? PALETTE_LIGHT : PALETTE_DARK;
    if (next === palette || !trail) return;
    palette = next;
    // The readouts and the HUD drawn over the scene follow (main.js)
    window.dispatchEvent(new CustomEvent('sceneLightChange', { detail: { light: palette === PALETTE_LIGHT } }));
    setSymbolHalo(...palette.halo);
    setLabelHalo(palette.labelHalo, palette.labelHaloPx);
    trail.setColor(palette.trail);
    homePole.setColor(palette.home);
    homeRing.setColor(palette.home);
    ownship.setSymbols([{ x: 0, y: 0, z: 0, shape: SHAPE.CIRCLE, color: palette.ownship }]);
    setCorridorColor(palette.route);
    setTrafficColor(palette.traffic, palette.label ?? palette.traffic);
    setMission3DPalette(palette);
    // The home label carries its colour: build it again
    if (homeLabel) {
        disposeLabel(homeLabel);
        homeLabel = null;
        homeMarkerLastKey = null;
    }
}

/**
 * UI theme. The light one gives the schematic view a pale blue sky and green
 * ground; the satellite view and AR are the same in both.
 * @param {boolean} light
 */
export function setLightTheme(light) {
    lightTheme = !!light;
    applyTheme();
}

/**
 * Sky colour for the time of day (sunlight on) or the default blue. Kept, but
 * not shown, while the schematic view or AR mode own the backdrop.
 * @param {number} hex
 */
export function setSkyColor(hex) {
    skyColor = hex;
    applyBackdrop();
}

/**
 * Schematic view: black sky and fog, for the isoline terrain drawn while the
 * satellite imagery is off.
 * @param {boolean} enabled
 */
export function setSchematicView(enabled) {
    schematicView = !!enabled;
    applyTheme();
}

export function isSchematicView() { return schematicView; }

// ============== TRAIL ==============

/**
 * Update trail with new position
 * @param {number} x - X position
 * @param {number} y - Y position (altitude)
 * @param {number} z - Z position
 */
export function updateTrail(x, y, z) {
    if (!trail) return;
    if (trailFrozen) return;  // full trail is pre-drawn during log replay

    const n = trail.count;
    if (n > 0) {
        const last = trail.positions;
        const o = (n - 1) * 3;
        const dx = x - last[o], dy = y - last[o + 1], dz = z - last[o + 2];
        const d2 = dx * dx + dy * dy + dz * dz;
        if (d2 < TRAIL_MIN_STEP_M * TRAIL_MIN_STEP_M) return;
        if (d2 > TRAIL_MAX_JUMP_M * TRAIL_MAX_JUMP_M) trail.clear();
    }

    // Check if we've hit the limit - downsample the trail by 2x to free space
    if (n >= MAX_TRAIL_POINTS) {
        downsampleTrail();
    }

    trail.push(x, y, z);
}

/**
 * Downsample trail by keeping every other point
 * Called when MAX_TRAIL_POINTS is reached
 */
function downsampleTrail() {
    const n = trail.count;
    if (n < 2) return;

    const pos = trail.positions;

    // Keep every other point (downsample 2:1)
    let writeIdx = 0;
    for (let readIdx = 0; readIdx < n; readIdx += 2) {
        const ro = readIdx * 3;
        const wo = writeIdx * 3;
        pos[wo] = pos[ro];
        pos[wo + 1] = pos[ro + 1];
        pos[wo + 2] = pos[ro + 2];
        writeIdx++;
    }

    trail.setCount(writeIdx);
}

/**
 * Freeze (or unfreeze) live trail updates. While frozen, updateTrail() is a
 * no-op and the caller is expected to drive the trail via setTrailPoints().
 */
export function setTrailFrozen(frozen) {
    trailFrozen = !!frozen;
}

/**
 * Clear the trail line.
 */
export function resetTrail() {
    if (trail) trail.clear();
}

/**
 * Set the trail line from a list of points.
 * @param {Array<{x:number,y:number,z:number}>} points
 */
export function setTrailPoints(points) {
    if (!trail) return;
    if (!Array.isArray(points) || points.length === 0) {
        resetTrail();
        return;
    }
    trail.setPoints(points);
}

/** The reference grid at 0 m, shown where no terrain covers it; the relative mode draws its own. */
export function setGroundGridVisible(visible) {
    if (groundGrid) groundGrid.visible = !!visible;
}

/**
 * Water surface grid and underwater particles for this frame (see Water3D.js).
 * @param {object} view plane and underwaterLevel, as updateWater3D() takes them
 */
export function updateWaterView(view) {
    if (!camera || !renderer) return;
    updateWater3D({ camera, pixelRatio: renderer.getPixelRatio(), ...view });
}

// ============== MISSION ==============

/**
 * Draw the mission route (see Mission3D.js for how each kind of item looks).
 * @param {{points: Array, pois?: Array, rtlSeq?: number|null}} mission
 */
export function updateMissionTrajectory(mission) {
    updateMission3D(mission);
}

/**
 * Highlight the item being flown (MISSION_CURRENT) and the path leading to it.
 * @param {number|null} seq mission sequence number, null for none
 */
export function setMissionActiveSeq(seq) {
    setMission3DActiveSeq(seq);
}

/** Clear the entire mission route (lines, symbols, labels). */
export function clearMissionTrajectory() {
    clearMission3D();
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
        animateTraffic3D(camera, renderer.getSize(_viewport).y);
        flushThickLines();
        // AR forces the schematic terrain too (FPVController), and its
        // outlines are what shows over the camera feed
        if (schematicView || arMode) renderSchematic();
        else renderer.render(scene, camera);
    }
}

// ============== SCHEMATIC OUTLINES ==============
// Every visible edge of the terrain gets a line: where a ridge hides the
// ground behind it (other mountains, or the valley beyond) and, brighter and
// thicker, where it stands against the sky — the horizon. Terrain has no
// such edges in its geometry to draw; they are found in the depth buffer.
//
//   1. world layer (terrain, runways, vehicle) → render target, colour + depth
//   2. full-screen pass → canvas: the colour plus the lines, and the depth
//      written back with gl_FragDepth
//   3. overlay layer (trail, route, symbols...) → canvas, depth-tested against
//      that depth, and multisampled, unlike the render target
//
// Overlays stay out of pass 1 so they are not outlined themselves: every
// LiDAR point would otherwise become an edge. A water surface (marked by the
// terrain with alpha 0) writes no depth in pass 2, so what is under the water
// — route, trail, the vehicle, drawn again in the overlay layer — shows in
// full instead of as a faint ghost.

const OUTLINE_VERTEX = `
varying vec2 vUv;
void main() {
    vUv = uv;
    gl_Position = vec4(position.xy, 0.0, 1.0);
}
`;

const OUTLINE_FRAGMENT = `
uniform sampler2D tColor;
uniform sampler2D tDepth;
uniform vec2 uTexel;          // one pixel in uv
uniform float uNear;
uniform float uFar;
uniform float uFogDensity;
uniform float uProfilePx;     // line widths in device pixels
uniform float uSkylinePx;
uniform float uBrightness;
uniform vec3 uProfileColor;   // light lines on dark ground, dark on light
uniform vec3 uSkylineColor;
uniform vec3 uSkyHorizon;     // sky gradient (both black in the dark theme)
uniform vec3 uSkyZenith;
uniform mat4 uInvProjection;
uniform mat4 uCameraWorld;
varying vec2 vUv;

// Sky by elevation of the view ray through this pixel, so the gradient stays
// on the real horizon when the aircraft pitches and banks.
vec3 skyColor() {
    vec4 view = uInvProjection * vec4(vUv * 2.0 - 1.0, 1.0, 1.0);
    vec3 dir = normalize((uCameraWorld * vec4(view.xyz / view.w, 0.0)).xyz);
    return mix(uSkyHorizon, uSkyZenith, smoothstep(0.0, 0.55, dir.y));
}

// 1 / view depth. It varies linearly across a plane in screen space, so the
// pixel on one side predicts the one on the other: a neighbour much farther
// than that prediction is across an edge, while a slope seen at a grazing
// angle — a big but steady change in depth — is not.
float invDepth(float d) {
    return 1.0 / uNear + d * (1.0 / uFar - 1.0 / uNear);
}

void main() {
    vec4 color = texture2D(tColor, vUv);
    float dc = texture2D(tDepth, vUv).r;
    // The terrain marks a water surface with alpha 0: its depth is not
    // written back, so the overlays under the water are drawn in full
    float waterSurface = color.a < 0.5 ? 1.0 : 0.0;
    color.a = 1.0;
    if (dc >= 1.0) {
        // Sky — unless it is a crack between two terrain chunks of different
        // detail, one pixel wide: ground on both sides of it horizontally or
        // vertically. Those are filled from a neighbour, not outlined.
        vec2 dx = vec2(uTexel.x, 0.0), dy = vec2(0.0, uTexel.y);
        float r = texture2D(tDepth, vUv + dx).r, l = texture2D(tDepth, vUv - dx).r;
        float u = texture2D(tDepth, vUv + dy).r, d = texture2D(tDepth, vUv - dy).r;
        if (max(r, l) < 1.0) {
            gl_FragDepth = max(r, l);
            gl_FragColor = vec4(texture2D(tColor, vUv + dx).rgb, 1.0);
        } else if (max(u, d) < 1.0) {
            gl_FragDepth = max(u, d);
            gl_FragColor = vec4(texture2D(tColor, vUv + dy).rgb, 1.0);
        } else {
            gl_FragDepth = dc;
            gl_FragColor = vec4(skyColor(), 1.0);
        }
        return;
    }
    gl_FragDepth = waterSurface > 0.5 ? 1.0 : dc;
    float ic = invDepth(dc);
    float profile = 0.0;
    float skyline = 0.0;

    // Lines are drawn on the near side of an edge only, so a ridge gets one
    // line of the requested width however far the ground behind it is.
    for (int k = 0; k < 4; k++) {
        vec2 dir = k == 0 ? vec2(1.0, 0.0) : k == 1 ? vec2(-1.0, 0.0) : k == 2 ? vec2(0.0, 1.0) : vec2(0.0, -1.0);
        for (int s = 1; s <= 4; s++) {
            float fs = float(s);
            if (fs > uSkylinePx) break;
            vec2 o = dir * uTexel * fs;
            float dn = texture2D(tDepth, vUv + o).r;
            if (dn >= 1.0) {
                // Real sky goes on beyond; a crack is over within two pixels
                if (texture2D(tDepth, vUv + dir * uTexel * (fs + 2.0)).r >= 1.0) skyline = 1.0;
                break;
            }
            if (fs > uProfilePx) continue;
            float dm = texture2D(tDepth, vUv - o).r;
            float predicted = dm >= 1.0 ? ic : 2.0 * ic - invDepth(dm);
            float gap = (predicted - invDepth(dn)) / ic;
            // A 3 m bump seen from 45 m above flat ground opens a ~7 % gap; a
            // ridge in front of the next valley, 20 % and more
            profile = max(profile, smoothstep(0.06, 0.14, gap));
        }
    }

    // Far ridges fade a little with the fog, never out: the depth cue stays
    // but every profile keeps its line. The skyline is drawn at full strength.
    float z = 1.0 / ic;
    float fog = 1.0 - exp(-uFogDensity * uFogDensity * z * z);
    float profileA = profile * mix(1.0, 0.6, fog);
    color.rgb = mix(color.rgb, uProfileColor * uBrightness, profileA);
    color.rgb = mix(color.rgb, uSkylineColor * uBrightness, skyline);
    gl_FragColor = color;
}
`;

let outline = null;
let outlineBrightness = 1;
const _bufferSize = new THREE.Vector2();
const _viewport = new THREE.Vector2();

function getOutline() {
    renderer.getDrawingBufferSize(_bufferSize);
    const w = Math.max(1, _bufferSize.x), h = Math.max(1, _bufferSize.y);
    if (!outline) {
        // 32-bit float depth: r128 maps every other depth type to 16 bits on WebGL2
        const depthTexture = new THREE.DepthTexture(w, h);
        depthTexture.type = THREE.FloatType;
        const target = new THREE.WebGLRenderTarget(w, h, {
            minFilter: THREE.NearestFilter,
            magFilter: THREE.NearestFilter,
            format: THREE.RGBAFormat,
            depthBuffer: true,
            stencilBuffer: false,
            depthTexture
        });
        target.texture.generateMipmaps = false;
        const material = new THREE.ShaderMaterial({
            uniforms: {
                tColor: { value: target.texture },
                tDepth: { value: depthTexture },
                uTexel: { value: new THREE.Vector2() },
                uNear: { value: 1 },
                uFar: { value: 1000 },
                uFogDensity: { value: 0 },
                uProfilePx: { value: 2 },
                uSkylinePx: { value: 3 },
                uBrightness: { value: 1 },
                uProfileColor: { value: new THREE.Color() },
                uSkylineColor: { value: new THREE.Color() },
                uSkyHorizon: { value: new THREE.Color() },
                uSkyZenith: { value: new THREE.Color() },
                uInvProjection: { value: new THREE.Matrix4() },
                uCameraWorld: { value: new THREE.Matrix4() }
            },
            vertexShader: OUTLINE_VERTEX,
            fragmentShader: OUTLINE_FRAGMENT,
            // Depth is only written with the test enabled: ALWAYS passes
            depthTest: true,
            depthFunc: THREE.AlwaysDepth,
            depthWrite: true
        });
        const quad = new THREE.Mesh(new THREE.PlaneGeometry(2, 2), material);
        quad.frustumCulled = false;
        const quadScene = new THREE.Scene();
        quadScene.add(quad);
        outline = { target, material, quadScene, quadCamera: new THREE.OrthographicCamera(-1, 1, 1, -1, 0, 1) };
    }
    if (outline.target.width !== w || outline.target.height !== h) outline.target.setSize(w, h);

    const u = outline.material.uniforms;
    const dpr = renderer.getPixelRatio();
    u.uTexel.value.set(1 / w, 1 / h);
    u.uNear.value = camera.near;
    u.uFar.value = camera.far;
    u.uFogDensity.value = scene.fog ? scene.fog.density : 0;
    u.uProfilePx.value = Math.max(1, Math.round(1.3 * dpr));
    u.uSkylinePx.value = Math.max(2, Math.round(2.4 * dpr));
    u.uBrightness.value = outlineBrightness;
    u.uInvProjection.value.copy(camera.projectionMatrixInverse);
    u.uCameraWorld.value.copy(camera.matrixWorld);
    if (isLightSchematic()) {
        u.uProfileColor.value.setRGB(0.12, 0.24, 0.17);
        u.uSkylineColor.value.setRGB(0.05, 0.19, 0.29);
        u.uSkyHorizon.value.setHex(LIGHT_SKY_HORIZON);
        u.uSkyZenith.value.setHex(LIGHT_SKY_ZENITH);
    } else {
        u.uProfileColor.value.setRGB(0.86, 0.90, 0.93);
        u.uSkylineColor.value.setRGB(0.97, 0.98, 1.00);
        u.uSkyHorizon.value.setHex(0x000000);
        u.uSkyZenith.value.setHex(0x000000);
    }
    return outline;
}

function renderSchematic() {
    const o = getOutline();

    camera.layers.set(WORLD_LAYER);
    renderer.setRenderTarget(o.target);
    renderer.render(scene, camera);

    renderer.setRenderTarget(null);
    renderer.render(o.quadScene, o.quadCamera);

    // A colour background makes three clear the canvas whatever autoClear
    // says: detach it, or the overlays would wipe the terrain drawn above
    camera.layers.set(OVERLAY_LAYER);
    const autoClear = renderer.autoClear;
    const background = scene.background;
    renderer.autoClear = false;
    scene.background = null;
    renderer.render(scene, camera);
    scene.background = background;
    renderer.autoClear = autoClear;

    camera.layers.enable(WORLD_LAYER);
}

/**
 * Brightness of the outlines, from the MAP BRIGHTNESS slider like the
 * schematic terrain itself (0.85, the default, draws them as designed).
 * @param {number} value 0.3 .. 1.6
 */
export function setOutlineBrightness(value) {
    const v = Number(value);
    if (Number.isFinite(v)) outlineBrightness = Math.max(0.3, Math.min(1.6, v)) / 0.85;
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
        // Terrain LOD is a screen-space error budget, so it has to follow the
        // viewport: the same chunk needs more triangles in a taller window.
        setLodViewParams(camera ? camera.fov : 60, renderer.domElement.height);
    }
    // Labels are sized in pixels of the viewport
    resizeMission3D(camera, height);
    if (homeLabel) fitLabel(homeLabel, camera, height, 26);
    resizeTraffic3D(camera, height);
}

// Home position 3D marker
let homeMarkerLastKey = null;

// Target marker 3D
let targetMarker3D = null;

/**
 * Update or remove the 3D home marker: a ring on the ground, a pole and the
 * home symbol at its top.
 *
 * The terrain mesh Y axis is MSL elevation from the HGT data, while HOME_POSITION
 * reports the vehicle's own AMSL altitude — the two can disagree by tens of metres
 * (geoid model, baro/GPS bias), which sinks the marker below the terrain surface.
 * The caller anchors it to the terrain elevation at home whenever that is
 * available, the same way mission waypoints are placed.
 *
 * @param {{lat:number, lon:number, ground:number}|null} home - position and
 *   MSL ground elevation, or null to hide the marker
 */
export function updateHomeMarker3D(home) {
    if (!scene || !homeSymbol) return;

    if (!home || !Number.isFinite(home.lat) || !Number.isFinite(home.lon)) {
        if (homeMarkerLastKey !== null) {
            homePole.setVisible(false);
            homeRing.setVisible(false);
            homeSymbol.setVisible(false);
            if (homeLabel) homeLabel.visible = false;
            homeMarkerLastKey = null;
        }
        return;
    }

    const base = (Number.isFinite(home.ground) ? home.ground : 0) + (STATE.offsetAlt || 0);

    // Rebuild only when position or resolved ground altitude actually changed
    const key = `${home.lat.toFixed(7)},${home.lon.toFixed(7)},${base.toFixed(1)}`;
    if (key === homeMarkerLastKey) return;
    homeMarkerLastKey = key;

    const pos = latLonToMeters(home.lat, home.lon);
    const top = base + HOME_POLE_M;

    homePole.setPoints([{ x: pos.x, y: base, z: pos.z }, { x: pos.x, y: top, z: pos.z }]);
    const ring = [];
    const SEGMENTS = 64;
    for (let i = 0; i <= SEGMENTS; i++) {
        const a = (i / SEGMENTS) * Math.PI * 2;
        ring.push({ x: pos.x + Math.cos(a) * HOME_RING_M, y: base + 0.5, z: pos.z + Math.sin(a) * HOME_RING_M });
    }
    homeRing.setPoints(ring);
    homeSymbol.setSymbols([
        { x: pos.x, y: top, z: pos.z, shape: SHAPE.HOME, color: palette.home },
        { x: pos.x, y: base, z: pos.z, shape: SHAPE.DOT, color: palette.home, scale: 0.3 }
    ]);

    if (!homeLabel) {
        homeLabel = makeLabel('HOME', '', palette.label ?? palette.home);
        scene.add(homeLabel);
    }
    homeLabel.position.set(pos.x, top, pos.z);
    fitLabel(homeLabel, camera, renderer ? renderer.getSize(new THREE.Vector2()).y : 0, 26);

    homePole.setVisible(true);
    homeRing.setVisible(true);
    homeSymbol.setVisible(true);
    homeLabel.visible = true;
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

    // Vertical pole (pooled geometry/material)
    const pole = new THREE.Mesh(pooledGeo.targetPole, pooledMat.targetRed);
    pole.position.y = 50;
    group.add(pole);

    // Top sphere
    const sphere = new THREE.Mesh(pooledGeo.targetSphere, pooledMat.targetRed);
    sphere.position.y = 105;
    group.add(sphere);

    // Ring around sphere
    const ring = new THREE.Mesh(pooledGeo.targetRing, pooledMat.targetRing);
    ring.position.y = 105;
    ring.rotation.x = Math.PI / 2;
    group.add(ring);

    group.position.set(pos.x, 0, pos.z);
    scene.add(toOverlayLayer(group));
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
    // geometry/materials are pooled — just remove from scene
    targetMarker3D = null;
}

/**
 * ADS-B traffic: red circles with callsign and relative height, trailing a
 * curve through their reported positions (see Traffic3D.js).
 * @param {Array} nearest - Array from getNearestTraffic() with lat, lon, alt, dist
 */
export function updateTrafficMarkers3D(nearest) {
    if (!scene) return;
    updateTraffic3D(nearest || []);
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

// AR overlay mode: black background (becomes transparent via CSS mix-blend-mode:screen)
export function setARMode(enabled) {
    arMode = !!enabled;
    applyTheme();
    renderer.setClearColor(0x000000, 1);
}

// Setters
export function setSunlightEnabled(enabled) { sunlightEnabled = enabled; }
export function setTimeOverride(time) { timeOverride = time; }
