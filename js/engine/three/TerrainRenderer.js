/** Three/WebGL terrain resources. Domain records enter; GPU handles stay here. */
import { TERRAIN_GRID } from '../../render/TerrainData.js';
import {
    initSatelliteDetail, disposeSatelliteDetail, updateSatelliteDetail, getSatelliteDetailStats,
    satelliteDetailUniforms, SATELLITE_DETAIL_GLSL
} from '../../terrain/SatelliteDetail.js';

export function createThreeTerrainRenderer({ three: THREE, scene, renderer, world, loadTileImage, onTextureDisposed = () => {} }) {
const sceneRef = scene;
const chunkResources = new Map();
const mapBrightness = world.terrainStyle.brightness;
const SCHEMATIC_RADIUS = world.terrainStyle.schematicRadius;
// ============== GPU TERRAIN ==============
// Chunk geometry lives on the GPU as elevation only. A chunk's samples (Int16,
// straight from the HGT grid) are one layer of a texture array per grid size,
// and the vertex shader rebuilds each vertex position and its normal from them
// with texelFetch. Every chunk built on the same grid shares one triangle list
// and one UV set.
//
// Chunks without a satellite map (everything beyond the satellite radius: at
// the default 10 km, ~390 of ~420 resident) are drawn as instances of one
// InstancedMesh per grid — a handful of draw calls — after a per-chunk frustum
// test in updateTerrainInstances(). Chunks with a map keep a mesh of their own
// (the map differs per chunk), on the same shared grid geometry. This replaced ~420
// meshes, each with its own position and normal buffers, which cost three.js a
// culling test, a matrix update and a draw call per chunk on every frame.

// Hillshade, and the height palette of chunks without a satellite map, are
// computed in the same vertex shader. The formula is the one the CPU used to
// bake into vertex colours:
//   sunlight on:  0.45 + 1.05 * max(0, N·sun)      sunlight off: map brightness
// times the height palette when the chunk has no map. It scales the diffuse
// colour, and MeshLambertMaterial then applies the scene lights.
//
// With the satellite imagery off the palette gives way to the schematic style
// (uSchematic): near-black ground with a faint hillshade — green with the
// light UI theme — isolines every 10 m and index isolines every 50 / 250 m,
// the look of the mission simulations in Top Gun: Maverick. It is written
// over the final colour, so scene lights and sunlight play no part in it.
const terrainShadingUniforms = {
    uSunDir: { value: new THREE.Vector3(0, 1, 0) }, // replaced by the scene's sun vector in initTerrain()
    uSunlightOn: { value: 1 },
    uBrightness: { value: mapBrightness },
    uSchematic: { value: 0 },
    // The schematic world ends on a circle this far from the camera; its rim
    // is the horizon line the schematic view outlines (Scene3D). The centre is
    // set every frame in updateTerrainInstances(): three only uploads its own
    // cameraPosition for a few material types, and Lambert is not one of them.
    uSchematicRadius: { value: SCHEMATIC_RADIUS },
    uSchematicCenter: { value: new THREE.Vector2() },
    // Light UI theme: green ground and dark lines instead of dark ground and light lines
    uSchematicLight: { value: 0 },
    // Triangle grid (setTerrainLowAltGrid): strength 0..1 from the aircraft's
    // height above the ground, and the aircraft's world x/z it is centred on
    uTriGrid: { value: 0 },
    uTriCenter: { value: new THREE.Vector2() },
    // A sub in water the elevation data does not show (a quarry, a small
    // lake): x, z, surface height, radius — ground at that height within the
    // radius is its water (setTerrainSubWater). Radius 0: off.
    uSubWater: { value: new THREE.Vector4(0, 0, 0, 0) }
};

// Triangle grid of the schematic view, flying low: TRI_GRID_CELL_M triangles
// within TRI_GRID_RADIUS_M of the aircraft, in full up to TRI_GRID_FULL_AGL
// above the ground and gone at TRI_GRID_MAX_AGL. Low down the isolines are few
// and far apart, and a flat valley floor has none: triangles of a fixed size
// are what shows how high the aircraft is, growing on screen as it descends.
// Everywhere else the shader skips it.
const TRI_GRID_CELL_M = TERRAIN_GRID.cellM;
const TRI_GRID_RADIUS_M = TERRAIN_GRID.radiusM;
const TRI_GRID_FULL_AGL = 150;
const TRI_GRID_MAX_AGL = 200;

const TERRAIN_VERTEX_PARS = `
uniform vec3 uSunDir;
uniform float uSunlightOn;
uniform float uBrightness;
uniform float uSchematic;
uniform vec4 uSubWater;
varying float vTerrainWater;
uniform highp isampler2DArray uHeights;
uniform int uGridMax;                 // vertices per side - 1
#ifdef USE_INSTANCING
attribute vec4 aChunk;                // x0, z0, dx, dz: world position of grid vertex (0,0), spacing
attribute float aLayer;
#else
uniform vec4 uChunk;
uniform float uLayer;
#endif
varying float vTerrainLight;
varying vec3 vTerrainTint;
varying float vTerrainH;
varying vec2 vTerrainXZ;
varying float vSchemShade;
// Same palette as getHeightColor() in core/utils.js
vec3 terrainHeightColor(float h) {
    const vec3 G = vec3(0.0431372549, 0.4, 0.137254902);
    const vec3 Y = vec3(0.902, 0.7647058824, 0.3529411765);
    const vec3 O = vec3(0.902, 0.494, 0.133);
    const vec3 R = vec3(0.906, 0.298, 0.235);
    const vec3 P = vec3(0.608, 0.349, 0.713);
    const vec3 B = vec3(0.204, 0.596, 0.858);
    if (h <= -100.0) return vec3(0.0);
    if (h <= 700.0) return G;
    if (h <= 1400.0) return mix(G, Y, (h - 700.0) / 700.0);
    if (h <= 2100.0) return mix(Y, O, (h - 1400.0) / 700.0);
    if (h <= 2800.0) return mix(O, R, (h - 2100.0) / 700.0);
    if (h <= 3500.0) return mix(R, P, (h - 2800.0) / 700.0);
    if (h <= 4000.0) return mix(P, B, (h - 3500.0) / 500.0);
    return B;
}
float terrainHeight(ivec2 g, int layer) {
    return float(texelFetch(uHeights, ivec3(g, layer), 0).r);
}
`;

// Replaces <beginnormal_vertex>. position.xy is the grid vertex (column, row);
// rows run south, columns east. The normal is the central difference the
// terrain worker used to compute (one-sided on the chunk edge): n = S × E.
const TERRAIN_BEGINNORMAL = `
#ifdef USE_INSTANCING
vec4 tChunk = aChunk;
int tLayer = int(aLayer + 0.5);
#else
vec4 tChunk = uChunk;
int tLayer = int(uLayer + 0.5);
#endif
ivec2 tG = ivec2(position.xy + 0.5);
float tH = terrainHeight(tG, tLayer);
ivec2 tE = ivec2(min(tG.x + 1, uGridMax), tG.y);
ivec2 tW = ivec2(max(tG.x - 1, 0), tG.y);
ivec2 tS = ivec2(tG.x, min(tG.y + 1, uGridMax));
ivec2 tN = ivec2(tG.x, max(tG.y - 1, 0));
float tHE = terrainHeight(tE, tLayer), tHW = terrainHeight(tW, tLayer);
float tHS = terrainHeight(tS, tLayer), tHN = terrainHeight(tN, tLayer);
float tEx = float(tE.x - tW.x) * tChunk.z;
float tEy = tHE - tHW;
float tSz = float(tS.y - tN.y) * tChunk.w;
float tSy = tHS - tHN;
vec3 objectNormal = normalize(vec3(-tSz * tEy, tSz * tEx, -tSy * tEx));
// Height the schematic isolines follow: lightly smoothed, so the metre-level
// noise of SRTM on flat ground does not break a level into dozens of rings.
// On an even slope the neighbours cancel out and it equals tH.
float tHiso = 0.5 * tH + 0.125 * (tHE + tHW + tHS + tHN);
// Water surface (schematic view only): SRTM flattens every lake to one height
// and the sea to 0, while real ground is never flat to the metre across a
// dozen samples — the same test as getWaterSurfaceAt(). Below 0 the data is
// bathymetry, the sea bed, which is ground.
float tWater = 0.0;
if (uSchematic > 0.5 && tH >= 0.0 && tHE == tH && tHW == tH && tHS == tH && tHN == tH) {
    int tX0 = max(tG.x - 1, 0), tX1 = min(tG.x + 1, uGridMax);
    int tY0 = max(tG.y - 1, 0), tY1 = min(tG.y + 1, uGridMax);
    bool tFlat =
        terrainHeight(ivec2(tX1, tY0), tLayer) == tH && terrainHeight(ivec2(tX0, tY0), tLayer) == tH &&
        terrainHeight(ivec2(tX1, tY1), tLayer) == tH && terrainHeight(ivec2(tX0, tY1), tLayer) == tH &&
        terrainHeight(ivec2(min(tG.x + 2, uGridMax), tG.y), tLayer) == tH &&
        terrainHeight(ivec2(max(tG.x - 2, 0), tG.y), tLayer) == tH &&
        terrainHeight(ivec2(tG.x, min(tG.y + 2, uGridMax)), tLayer) == tH &&
        terrainHeight(ivec2(tG.x, max(tG.y - 2, 0)), tLayer) == tH;
    tWater = tFlat ? 1.0 : 0.0;
}
`;

// Replaces <begin_vertex>
const TERRAIN_BEGIN_VERTEX = `
vec3 transformed = vec3(tChunk.x + float(tG.x) * tChunk.z, tH, tChunk.y + float(tG.y) * tChunk.w);
vTerrainLight = uSunlightOn > 0.5
    ? 0.45 + 1.05 * max(0.0, dot(objectNormal, uSunDir))
    : uBrightness;
// The colour of a chunk without a satellite map
vTerrainTint = terrainHeightColor(tH);
vTerrainH = tHiso;
vTerrainXZ = transformed.xz;
if (uSchematic > 0.5 && uSubWater.w > 0.0 && tH <= uSubWater.z + 1.5 && length(transformed.xz - uSubWater.xy) < uSubWater.w) tWater = 1.0;
vTerrainWater = tWater;
// Cartographic hillshade for the schematic style: light from the north-west,
// 45° up (x east, z south), whatever the sun is doing.
vSchemShade = max(0.0, dot(objectNormal, vec3(-0.5, 0.70710678, -0.5)));
`;

const TERRAIN_FRAGMENT_PARS = `
varying float vTerrainLight;
varying vec3 vTerrainTint;
varying float vTerrainH;
varying vec2 vTerrainXZ;
varying float vSchemShade;
varying float vTerrainWater;
uniform float uSchematic;
uniform float uSchematicRadius;
uniform vec2 uSchematicCenter;
uniform float uSchematicLight;
uniform float uBrightness;
uniform float uTriGrid;
uniform vec2 uTriCenter;
uniform vec4 uSubWater;

// Isolines of height h every 'interval' metres, 'widthPx' wide on screen.
// fwidth() gives the spacing of neighbouring lines in pixels: where they would
// crowd closer than 'minSpacingPx' (far away, steep slopes seen edge-on) the
// level fades out, so the next, coarser level takes over instead of the
// lines merging into a grey smear.
// Levels sit half a metre off the round figure: SRTM heights are whole
// metres, so a flat area at exactly 580 m would lie on the 580 line
// everywhere and be painted solid.
float schematicIsoline(float h, float interval, float widthPx, float minSpacingPx) {
    float x = (h - 0.5) / interval;
    float fw = max(fwidth(x), 1e-5);
    float distPx = abs(fract(x + 0.5) - 0.5) / fw;
    float line = 1.0 - smoothstep(widthPx * 0.5 - 0.5, widthPx * 0.5 + 0.5, distPx);
    return line * smoothstep(minSpacingPx, minSpacingPx * 2.5, 1.0 / fw);
}

// Kilometre grid on the ground: on a flat valley floor there are no isolines,
// and the grid is what still shows scale and ground speed.
float schematicGrid(vec2 p, float cell, float widthPx) {
    vec2 x = p / cell;
    vec2 fw = max(fwidth(x), vec2(1e-5));
    vec2 distPx = abs(fract(x + 0.5) - 0.5) / fw;
    vec2 line = 1.0 - smoothstep(vec2(widthPx * 0.5 - 0.5), vec2(widthPx * 0.5 + 0.5), distPx);
    vec2 keep = smoothstep(vec2(6.0), vec2(16.0), 1.0 / fw);
    return max(line.x * keep.x, line.y * keep.y);
}

// Distance of p along the normals of the triangle grid's three line families
// (60° apart, meeting at the same vertices), in line spacings. Linear in p.
vec3 triGridCoords(vec2 p) {
    return vec3(p.y, 0.8660254 * p.x - 0.5 * p.y, 0.8660254 * p.x + 0.5 * p.y)
        / (${TRI_GRID_CELL_M.toFixed(1)} * 0.8660254);
}

// Equilateral triangles laid flat in world space and draped over the relief,
// out to the grid radius around the aircraft with a soft edge. dpx / dpy are
// the screen derivatives of p, taken by the caller: this runs in a branch that
// differs from pixel to pixel, where derivatives are undefined, and the
// coordinates being linear in p, theirs follow. The three families fade
// together, from the most crowded, so the grid thins out as triangles and not
// as streaks where the ground is seen at a grazing angle.
float schematicTriGrid(vec2 p, vec2 dpx, vec2 dpy, float widthPx) {
    float r = length(p - uTriCenter);
    if (r >= ${TRI_GRID_RADIUS_M.toFixed(1)}) return 0.0;
    vec3 x = triGridCoords(p);
    vec3 fw = max(abs(triGridCoords(dpx)) + abs(triGridCoords(dpy)), vec3(1e-5));
    vec3 distPx = abs(fract(x + 0.5) - 0.5) / fw;
    vec3 line = 1.0 - smoothstep(vec3(widthPx * 0.5 - 0.5), vec3(widthPx * 0.5 + 0.5), distPx);
    float keep = smoothstep(2.5, 6.0, 1.0 / max(fw.x, max(fw.y, fw.z)));
    float edge = 1.0 - smoothstep(${(TRI_GRID_RADIUS_M * 0.8).toFixed(1)}, ${TRI_GRID_RADIUS_M.toFixed(1)}, r);
    return max(line.x, max(line.y, line.z)) * keep * edge;
}

// Three levels, as on a topographic chart: faint 10 m lines, stronger 50 m
// index lines, and 250 m lines that stay when the others have faded.
// Dark theme: grey ground shaded from black, light lines, amber 250 m lines.
// Light theme: the same hillshade in greens, dark green lines, burnt orange.
// Flying low, a faint triangle grid lies under the isolines around the aircraft.
// Water (a lake, or the sea where the data flattens it to 0) is dark blue with
// the triangle grid in blue, marking the surface; the sea bed (bathymetry,
// below 0) is ground tinted blue, its isolines the depth contours.
vec3 schematicTerrainColor() {
    float grid = schematicGrid(vTerrainXZ, 1000.0, 1.0);
    vec2 dpx = dFdx(vTerrainXZ), dpy = dFdy(vTerrainXZ);
    float tri = uTriGrid > 0.001 ? schematicTriGrid(vTerrainXZ, dpx, dpy, 1.0) * uTriGrid : 0.0;
    float minor = schematicIsoline(vTerrainH, 10.0, 1.0, 3.5);
    float index = schematicIsoline(vTerrainH, 50.0, 1.4, 3.5);
    float major = schematicIsoline(vTerrainH, 250.0, 2.0, 3.0);
    float water = smoothstep(0.5, 0.95, vTerrainWater);
    float bed = (1.0 - water) * smoothstep(0.0, -2.0, vTerrainH);
    // A sub's own water where the data shows none: its surface grid is the
    // plane's (Water3D), level — the ground under it is only roughly flat
    if (uSubWater.w > 0.0) tri *= 1.0 - water;
    vec3 col;
    if (uSchematicLight > 0.5) {
        col = mix(vec3(0.31, 0.48, 0.28), vec3(0.80, 0.89, 0.67), vSchemShade);
        col = mix(col, col * vec3(0.72, 0.86, 1.15), bed);
        col = mix(col, vec3(0.62, 0.76, 0.90), water);
        col = mix(col, vec3(0.27, 0.43, 0.39), grid * 0.35);
        col = mix(col, mix(vec3(0.25, 0.41, 0.30), vec3(0.10, 0.33, 0.72), water), tri * mix(0.18, 0.4, water));
        col = mix(col, vec3(0.22, 0.37, 0.20), minor * 0.4);
        col = mix(col, vec3(0.12, 0.25, 0.12), index * 0.65);
        col = mix(col, vec3(0.62, 0.30, 0.05), major * 0.9);
    } else {
        col = vec3(0.022, 0.028, 0.034) + vec3(0.115, 0.125, 0.135) * vSchemShade;
        col = mix(col, col * vec3(0.70, 0.90, 1.35), bed);
        col = mix(col, vec3(0.015, 0.045, 0.085), water);
        col = mix(col, vec3(0.12, 0.24, 0.28), grid * 0.7);
        col = mix(col, mix(vec3(0.20, 0.32, 0.35), vec3(0.16, 0.42, 0.85), water), tri * mix(0.3, 0.55, water));
        col = mix(col, vec3(0.38, 0.43, 0.47), minor * 0.75);
        col = mix(col, vec3(0.74, 0.79, 0.82), index * 0.85);
        col = mix(col, vec3(1.00, 0.60, 0.16), major);
    }
    // The MAP BRIGHTNESS slider (default 0.85) scales the whole drawing
    return col * (uBrightness / 0.85);
}
${SATELLITE_DETAIL_GLSL}`;

// After <color_fragment>: the chunk's satellite map (the height tint without
// one), the full-resolution imagery around the aircraft over it, then the light
const TERRAIN_COLOR_FRAGMENT = `
#ifdef USE_MAP
vec3 terrainBase = diffuseColor.rgb;
#else
vec3 terrainBase = diffuseColor.rgb * vTerrainTint;
#endif
diffuseColor.rgb = satelliteDetail(terrainBase, vTerrainXZ) * vTerrainLight;
`;

// Replaces <fog_fragment>: the schematic colour replaces the lit one, then the
// (black) scene fog fades it with distance. The colour is computed before the
// radius test so its fwidth() calls run on every fragment of the quad.
const TERRAIN_FOG_FRAGMENT = `
#ifndef USE_MAP
if (uSchematic > 0.5) {
    gl_FragColor.rgb = schematicTerrainColor();
    // Alpha 0 marks a water surface for the outline pass (Scene3D), which
    // then lets whatever is under the water show through it
    gl_FragColor.a = vTerrainWater > 0.5 ? 0.0 : 1.0;
    if (length(vTerrainXZ - uSchematicCenter) > uSchematicRadius) discard;
}
#endif
#include <fog_fragment>
`;

function applyTerrainShading(shader) {
    // Shading uniforms are shared by every chunk; the chunk uniforms belong to
    // this material (a chunk's own mesh, or an instanced batch's grid).
    Object.assign(shader.uniforms, terrainShadingUniforms, satelliteDetailUniforms, this.userData.terrain);
    shader.vertexShader = shader.vertexShader
        .replace('#include <common>', '#include <common>\n' + TERRAIN_VERTEX_PARS)
        .replace('#include <beginnormal_vertex>', TERRAIN_BEGINNORMAL)
        .replace('#include <begin_vertex>', TERRAIN_BEGIN_VERTEX);
    shader.fragmentShader = shader.fragmentShader
        .replace('#include <common>', '#include <common>\n' + TERRAIN_FRAGMENT_PARS)
        .replace('#include <color_fragment>', '#include <color_fragment>\n' + TERRAIN_COLOR_FRAGMENT)
        .replace('#include <fog_fragment>', TERRAIN_FOG_FRAGMENT);
}

function createTerrainMaterial(params) {
    const material = new THREE.MeshLambertMaterial(Object.assign({
        side: THREE.FrontSide  // heightfield seen from above: backface culling halves rasterization
    }, params));
    material.userData.terrain = {
        uHeights: { value: null },
        uGridMax: { value: 0 },
        uChunk: { value: new THREE.Vector4() },
        uLayer: { value: 0 }
    };
    // A single shared function (it reads its uniforms from `this`): three.js keys
    // its program cache on the callback's source, so every terrain material
    // compiles to the same few programs.
    material.onBeforeCompile = applyTerrainShading;
    return material;
}

// Material slot of chunks drawn through an instanced batch (no map). It is
// never rendered itself; a chunk only gets a material of its own while it
// carries a map. Never disposed.
const untexturedTerrainMaterial = createTerrainMaterial();

/** Point a non-instanced terrain material at a chunk's elevation layer. */
function syncChunkUniforms(material, ud) {
    const t = material.userData.terrain;
    t.uHeights.value = ud.heightStore.texture;
    t.uGridMax.value = ud.geoW - 1;
    t.uChunk.value.set(ud.chunkVec[0], ud.chunkVec[1], ud.chunkVec[2], ud.chunkVec[3]);
    t.uLayer.value = ud.heightLayer;
}

/**
 * Put a satellite map on a chunk, or remove it (texture = null). The chunk's
 * previous map is disposed. With a map the chunk is drawn as a mesh of its own;
 * without one it goes back to its grid's instanced batch.
 */
function setChunkMap(mesh, texture) {
    world.terrain.setAppearance(mesh.userData.renderRecord.id, texture?.userData.renderDescriptor || null);
    const own = mesh.material !== untexturedTerrainMaterial ? mesh.material : null;
    if (own && own.map) {
        try { own.map.dispose(); onTextureDisposed(); } catch (e) {}
        own.map = null;
    }
    if (texture) {
        const material = own || createTerrainMaterial();
        material.map = texture;
        material.needsUpdate = true;
        if (!own) {
            syncChunkUniforms(material, mesh.userData);
            mesh.material = material;
            if (sceneRef) sceneRef.add(mesh);
        }
    } else if (own) {
        own.dispose();
        mesh.material = untexturedTerrainMaterial;
        if (mesh.parent) mesh.parent.remove(mesh);
    }
}

// ---- Shared grid geometry -------------------------------------------------
const gridGeometries = new Map(); // geoW -> BufferGeometry

/**
 * Grid of geoW x geoW vertices: position = (column, row, 0), same UVs and same
 * triangle order as the THREE.PlaneGeometry the chunks used to be built from,
 * so the winding FrontSide culling depends on is unchanged. Never disposed.
 */
function getGridGeometry(geoW) {
    let geometry = gridGeometries.get(geoW);
    if (geometry) return geometry;

    const seg = geoW - 1;
    const IndexArray = geoW * geoW > 65535 ? Uint32Array : Uint16Array;
    const index = new IndexArray(seg * seg * 6);
    let k = 0;
    for (let r = 0; r < seg; r++) {
        for (let c = 0; c < seg; c++) {
            const a = r * geoW + c;
            const b = (r + 1) * geoW + c;
            const cc = (r + 1) * geoW + c + 1;
            const d = r * geoW + c + 1;
            index[k++] = a; index[k++] = b; index[k++] = d;
            index[k++] = b; index[k++] = cc; index[k++] = d;
        }
    }

    const position = new Float32Array(geoW * geoW * 3);
    const uv = new Float32Array(geoW * geoW * 2);
    for (let r = 0; r < geoW; r++) {
        for (let c = 0; c < geoW; c++) {
            const i = r * geoW + c;
            position[i * 3] = c;
            position[i * 3 + 1] = r;
            uv[i * 2] = c / seg;
            uv[i * 2 + 1] = 1 - r / seg;
        }
    }

    geometry = new THREE.BufferGeometry();
    geometry.setIndex(new THREE.BufferAttribute(index, 1));
    geometry.setAttribute('position', new THREE.BufferAttribute(position, 3));
    geometry.setAttribute('uv', new THREE.BufferAttribute(uv, 2));
    // Grid space, not world space: meshes on it are culled per chunk instead
    geometry.boundingSphere = new THREE.Sphere(new THREE.Vector3(), Infinity);
    gridGeometries.set(geoW, geometry);
    return geometry;
}

// ---- Elevation texture arrays ----------------------------------------------
// One R16I 2D-array texture per grid size, one layer per chunk. The samples of
// every layer live in one CPU array (2 bytes per sample, ~2 MB in total), the
// texture's own data: a new chunk uploads its layer alone (addLayerUpdate),
// and growing the array, or a WebGL context restore, uploads all of it.
const heightStores = new Map(); // geoW -> store
const HEIGHT_STORE_INITIAL_LAYERS = 32;

function getHeightStore(geoW) {
    let store = heightStores.get(geoW);
    if (store) return store;
    const capacity = HEIGHT_STORE_INITIAL_LAYERS;
    const data = new Int16Array(geoW * geoW * capacity);
    const texture = new THREE.DataArrayTexture(data, geoW, geoW, capacity);
    texture.format = THREE.RedIntegerFormat;
    texture.type = THREE.ShortType;
    texture.internalFormat = 'R16I';
    texture.minFilter = THREE.NearestFilter;
    texture.magFilter = THREE.NearestFilter;
    texture.unpackAlignment = 2;   // rows of 2-byte samples with odd widths
    texture.needsUpdate = true;
    // While a whole upload is pending (new storage) no layer may be listed
    // alone: three would upload only the listed layers into it
    store = { geoW, capacity, data, texture, used: new Set(), free: [], next: 0, fullUpload: true };
    texture.onUpdate = () => { store.fullUpload = false; };
    heightStores.set(geoW, store);
    return store;
}

/** The whole array goes up at the next render. */
function uploadWholeHeightStore(store) {
    store.fullUpload = true;
    store.texture.clearLayerUpdates();
    store.texture.needsUpdate = true;
}

/** three re-creates its textures after a WebGL context restore: from all the layers. */
function restoreHeightStores() {
    for (const store of heightStores.values()) uploadWholeHeightStore(store);
}

// The GPU caps the layers of an array texture (2048 on desktop). A wide terrain
// radius at high latitude could ask for more chunks of one grid size than that:
// those beyond the cap are skipped rather than the whole texture failing.
const gl = renderer.getContext();
const maxHeightLayers = gl.getParameter?.(gl.MAX_ARRAY_TEXTURE_LAYERS) || 2048;
let heightLayersFullWarned = false;

/** Twice the layers, up to the GPU's cap. The texture object stays (the materials point at it); its WebGL storage is replaced. */
function growHeightStore(store) {
    store.capacity = Math.min(store.capacity * 2, maxHeightLayers);
    const data = new Int16Array(store.geoW * store.geoW * store.capacity);
    data.set(store.data);
    store.data = data;
    store.texture.image = { data, width: store.geoW, height: store.geoW, depth: store.capacity };
    store.texture.dispose();   // frees the old storage; the next render allocates the new size
    uploadWholeHeightStore(store);
}

/** @returns {number} the layer, or -1 when the array is at the GPU's cap and full */
function allocHeightLayer(store, heights) {
    const layer = store.free.length ? store.free.pop() : store.next++;
    if (layer >= store.capacity) {
        if (store.capacity >= maxHeightLayers) {
            store.next--;
            if (!heightLayersFullWarned) {
                heightLayersFullWarned = true;
                console.warn(`[terrain] ${maxHeightLayers} elevation layers for ${store.geoW}² chunks (GPU limit): further chunks are not drawn — reduce the 3D terrain radius`);
            }
            return -1;
        }
        growHeightStore(store);
    }
    store.used.add(layer);
    store.data.set(heights, layer * store.geoW * store.geoW);
    if (!store.fullUpload) store.texture.addLayerUpdate(layer);
    store.texture.needsUpdate = true;
    return layer;
}

function freeHeightLayer(store, layer) {
    if (store && store.used.delete(layer)) store.free.push(layer);
}

/**
 * Store a chunk's elevation samples and derive what the shader and the
 * culling need: grid origin and spacing in world metres, bounding sphere.
 * Replaces the chunk's previous layer (LOD rebuild).
 */
function setChunkHeights(mesh, record) {
    const ud = mesh.userData;
    const geoW = record.width;
    if (ud.heightStore) freeHeightLayer(ud.heightStore, ud.heightLayer);
    const store = getHeightStore(geoW);
    ud.heightStore = store;
    ud.heightLayer = allocHeightLayer(store, record.heights);
    ud.geoW = geoW;
    ud.lodStep = record.step;
    ud.chunkVec = record.grid;
    ud.sphere = record.sphere;
    ud.renderRecord = record;
    mesh.geometry = getGridGeometry(geoW);
    ud.batch = getInstanceBatch(geoW);
    if (mesh.material !== untexturedTerrainMaterial) syncChunkUniforms(mesh.material, ud);
}

// ---- Instanced batches (chunks without a map) ------------------------------
const instanceBatches = new Map(); // geoW -> batch
const INSTANCE_CAPACITY = 4096;    // > the chunk budget at the widest terrain radius (70 km: 2200)
let chunksVisible = true;

function getInstanceBatch(geoW) {
    let batch = instanceBatches.get(geoW);
    if (batch) return batch;

    const grid = getGridGeometry(geoW);
    const chunkAttr = new THREE.InstancedBufferAttribute(new Float32Array(INSTANCE_CAPACITY * 4), 4);
    const layerAttr = new THREE.InstancedBufferAttribute(new Float32Array(INSTANCE_CAPACITY), 1);
    chunkAttr.setUsage(THREE.DynamicDrawUsage);
    layerAttr.setUsage(THREE.DynamicDrawUsage);
    // Its own geometry object (the instance attributes are per batch) on the
    // grid's shared index / position / uv buffers
    const geometry = new THREE.BufferGeometry();
    geometry.setIndex(grid.index);
    geometry.setAttribute('position', grid.attributes.position);
    geometry.setAttribute('uv', grid.attributes.uv);
    geometry.setAttribute('aChunk', chunkAttr);
    geometry.setAttribute('aLayer', layerAttr);
    geometry.boundingSphere = grid.boundingSphere;

    const material = createTerrainMaterial();
    material.userData.terrain.uHeights.value = getHeightStore(geoW).texture;
    material.userData.terrain.uGridMax.value = geoW - 1;

    const mesh = new THREE.InstancedMesh(geometry, material, INSTANCE_CAPACITY);
    // Vertices come out of the shader in world space: identity instance matrices
    const m = mesh.instanceMatrix.array;
    for (let i = 0; i < INSTANCE_CAPACITY; i++) {
        m[i * 16] = m[i * 16 + 5] = m[i * 16 + 10] = m[i * 16 + 15] = 1;
    }
    mesh.frustumCulled = false;   // culled per chunk in updateTerrainInstances()
    mesh.matrixAutoUpdate = false;
    mesh.count = 0;
    mesh.visible = false;
    if (sceneRef) sceneRef.add(mesh);

    batch = { mesh, chunkAttr, layerAttr, count: 0 };
    instanceBatches.set(geoW, batch);
    return batch;
}

// Resident chunks as an array for the per-frame pass: iterating activeChunks
// with for...in (an object whose keys come and go) cost several times more.
const chunkList = [];

function listChunk(mesh) {
    mesh.userData.listIndex = chunkList.length;
    chunkList.push(mesh);
}

function unlistChunk(mesh) {
    const i = mesh.userData.listIndex;
    if (i === undefined || chunkList[i] !== mesh) return;
    const last = chunkList.pop();
    if (last !== mesh) {
        chunkList[i] = last;
        last.userData.listIndex = i;
    }
    mesh.userData.listIndex = undefined;
}

/**
 * Per-frame terrain culling, called right before rendering: chunks with a map
 * get their mesh shown or hidden, chunks without one are packed into their
 * grid's instanced batch.
 * @param {THREE.Camera} camera
 */
function updateTerrainInstances(cameraData, { camera, aircraft, pixelAngle, satelliteEnabled }) {
    if (!cameraData.revision) return;
    syncStyle();
    const _planes = cameraData.planes;
    for (const batch of instanceBatches.values()) batch.count = 0;

    // Schematic view: chunks wholly outside its radius would only be discarded
    // pixel by pixel in the fragment shader
    const clipRadius = terrainShadingUniforms.uSchematic.value > 0.5 ? terrainShadingUniforms.uSchematicRadius.value : Infinity;
    const cx = cameraData.position[0], cz = cameraData.position[2];
    terrainShadingUniforms.uSchematicCenter.value.set(cx, cz);

    for (let n = 0; n < chunkList.length; n++) {
        const mesh = chunkList[n];
        const ud = mesh.userData;
        if (ud.heightLayer < 0) { mesh.visible = false; continue; }   // no elevation layer (GPU cap)
        const s = ud.sphere;
        // Sphere against the six frustum planes (same test as Frustum.intersectsSphere)
        let visible = chunksVisible && Math.hypot(s.x - cx, s.z - cz) - s.r <= clipRadius;
        for (let p = 0; visible && p < 24; p += 4) {
            if (_planes[p] * s.x + _planes[p + 1] * s.y + _planes[p + 2] * s.z + _planes[p + 3] < -s.r) visible = false;
        }

        if (mesh.material !== untexturedTerrainMaterial) {   // own mesh (has a map)
            mesh.visible = visible;
            continue;
        }
        if (!visible) continue;
        const batch = ud.batch;
        if (batch.count >= INSTANCE_CAPACITY) continue;
        const i = batch.count++;
        const c = batch.chunkAttr.array;
        c[i * 4] = ud.chunkVec[0];
        c[i * 4 + 1] = ud.chunkVec[1];
        c[i * 4 + 2] = ud.chunkVec[2];
        c[i * 4 + 3] = ud.chunkVec[3];
        batch.layerAttr.array[i] = ud.heightLayer;
    }

    for (const batch of instanceBatches.values()) {
        batch.mesh.count = batch.count;
        batch.mesh.visible = batch.count > 0;
        if (batch.count === 0) continue;
        batch.chunkAttr.clearUpdateRanges();
        batch.chunkAttr.addUpdateRange(0, batch.count * 4);
        batch.chunkAttr.needsUpdate = true;
        batch.layerAttr.clearUpdateRanges();
        batch.layerAttr.addUpdateRange(0, batch.count);
        batch.layerAttr.needsUpdate = true;
    }

    // Full-resolution imagery around the aircraft, over the chunk textures
    updateSatelliteDetail({
        camera,
        lat: aircraft.lat,
        lon: aircraft.lon,
        groundY: aircraft.terrainHeight,
        pixelAngle,
        enabled: chunksVisible && satelliteEnabled && terrainShadingUniforms.uSchematic.value < 0.5
    });
}


function syncStyle() {
    const s = world.terrainStyle, u = terrainShadingUniforms;
    chunksVisible = s.visible;
    u.uSunDir.value.set(...s.sunDirection);
    u.uSunlightOn.value = s.sunlight ? 1 : 0;
    u.uBrightness.value = s.brightness;
    u.uSchematic.value = s.schematic ? 1 : 0;
    u.uSchematicLight.value = s.light ? 1 : 0;
    u.uSchematicRadius.value = s.schematicRadius;
    u.uTriGrid.value = s.gridStrength;
    u.uTriCenter.value.set(s.gridCenter.x, s.gridCenter.y);
    u.uSubWater.value.set(...s.subWater);
}
function createChunk(record, metadata) {
    const existing = chunkResources.get(record.id);
    if (existing) { Object.assign(existing.handle.data, metadata); updateHeights(existing.handle, record); return existing.handle; }
    const mesh = new THREE.Mesh(getGridGeometry(record.width), untexturedTerrainMaterial);
    mesh.frustumCulled = false;
    mesh.matrixAutoUpdate = false;
    mesh.userData = { ...metadata, textureLoaded: false };
    setChunkHeights(mesh, record);
    listChunk(mesh);
    const handle = { id: record.id, uuid: record.id, data: { ...metadata, textureLoaded: false, lodStep: record.step } };
    chunkResources.set(record.id, { mesh, handle });
    return handle;
}
function updateHeights(handle, record) {
    const mesh = chunkResources.get(handle.id)?.mesh;
    if (!mesh) return;
    setChunkHeights(mesh, record);
    handle.data.lodStep = record.step;
}
function setMap(handle, texture) {
    const mesh = chunkResources.get(handle.id)?.mesh;
    if (!mesh) {
        if (texture) { texture.dispose(); onTextureDisposed(); }
        return;
    }
    setChunkMap(mesh, texture);
}
function removeChunk(handle) {
    const mesh = chunkResources.get(handle.id)?.mesh;
    if (!mesh) return;
    setChunkMap(mesh, null);
    freeHeightLayer(mesh.userData.heightStore, mesh.userData.heightLayer);
    mesh.userData.heightStore = null;
    if (mesh.parent) mesh.parent.remove(mesh);
    unlistChunk(mesh);
    chunkResources.delete(handle.id);
    handle.data.disposed = true;
}
function makeCompressedTexture(msg) {
    const texture = new THREE.CompressedTexture(msg.mips, msg.padWidth, msg.padHeight, THREE.RGB_S3TC_DXT1_Format);
    texture.wrapS = texture.wrapT = THREE.ClampToEdgeWrapping;
    texture.minFilter = THREE.LinearMipmapLinearFilter;
    texture.magFilter = THREE.LinearFilter;
    texture.anisotropy = renderer.capabilities.getMaxAnisotropy();
    texture.repeat.set(msg.width / msg.padWidth, msg.height / msg.padHeight);
    texture.userData.renderDescriptor = { kind: 'bc1', width: msg.width, height: msg.height,
        paddedWidth: msg.padWidth, paddedHeight: msg.padHeight,
        uvScale: [msg.width / msg.padWidth, msg.height / msg.padHeight], mips: msg.mips };
    texture.needsUpdate = true;
    return texture;
}
function makeCanvasTexture(canvas) {
    const texture = new THREE.CanvasTexture(canvas);
    texture.wrapS = texture.wrapT = THREE.ClampToEdgeWrapping;
    texture.minFilter = THREE.LinearMipmapLinearFilter;
    texture.magFilter = THREE.LinearFilter;
    texture.anisotropy = renderer.capabilities.getMaxAnisotropy();
    // Keep the source alive for context restore; don't read pixels on the UI thread.
    texture.userData.renderDescriptor = { kind: 'canvas-reference', width: canvas.width, height: canvas.height };
    renderer.initTexture(texture);
    return texture;
}
function getStats() {
    return {
        heightLayers: [...heightStores.values()].reduce((n, st) => n + st.used.size, 0),
        heightMB: +([...heightStores.values()].reduce((n, st) => n + st.data.byteLength, 0) / 1048576).toFixed(1),
        satelliteDetail: getSatelliteDetailStats()
    };
}
function dispose() {
    renderer.domElement.removeEventListener('webglcontextrestored', restoreHeightStores);
    disposeSatelliteDetail();
    for (const { handle } of [...chunkResources.values()]) removeChunk(handle);
    for (const batch of instanceBatches.values()) {
        batch.mesh.removeFromParent(); batch.mesh.geometry.dispose(); batch.mesh.material.dispose();
    }
    for (const geometry of gridGeometries.values()) geometry.dispose();
    for (const store of heightStores.values()) store.texture.dispose();
    untexturedTerrainMaterial.dispose();
    instanceBatches.clear(); gridGeometries.clear(); heightStores.clear();
}
renderer.domElement.addEventListener('webglcontextrestored', restoreHeightStores);
initSatelliteDetail(renderer, loadTileImage);
return { createChunk, updateHeights, setMap, removeChunk, hasChunk: handle => chunkResources.has(handle.id),
    updateFrame: updateTerrainInstances, makeCompressedTexture, makeCanvasTexture, getStats, dispose,
    capabilities: { bc1: !!renderer.getContext().getExtension('WEBGL_compressed_texture_s3tc'),
        maxTextureSize: renderer.capabilities.maxTextureSize } };
}
