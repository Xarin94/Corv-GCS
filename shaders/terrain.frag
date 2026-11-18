#version 440
// Terrain colour (port of the fragment part of TerrainRenderer.js).
// Schematic style: near-black ground with a north-west hillshade, isolines
// every 10 / 50 / 250 m that fade level by level where they would crowd
// closer than a few pixels, a 1 km grid, water in dark blue, the triangle grid
// around the aircraft flying low; the world ends on the schematic radius.
// Otherwise the chunk's satellite image (the height palette beyond the
// satellite radius), lit by the sun. Exponential-squared fog to the sky
// colour, as three's FogExp2.

layout(location = 0) in float vLight;
layout(location = 1) in float vH;
layout(location = 2) in vec2 vXZ;
layout(location = 3) in float vShade;
layout(location = 4) in float vDist;
layout(location = 5) in float vWater;
layout(location = 6) in vec3 vTint;
layout(location = 7) in vec2 vUV;
layout(location = 8) flat in float vSat;

layout(location = 0) out vec4 fragColor;

layout(std140, binding = 0) uniform Buf {
    mat4 viewProj;
    vec4 camPos;
    vec4 sunDir;
    vec4 params;
    vec4 extra;
    vec4 zone;
    vec4 zone2;
    vec4 detailMap[4];   // per level: window centre in world x, z; texture px per metre in x, z
    vec4 detailWin[4];   // per level: the centre's px in the window; the window's origin in the texture
    vec4 detailOn;       // 1 for a level in use
};
layout(binding = 2) uniform sampler2DArray satellite;
layout(binding = 3) uniform sampler2D zoneCov;     // R no-fly, G conditional from the surface, B zones above the ground
layout(binding = 4) uniform sampler2D zoneFloor;   // R lowest floor AGL, G lowest floor MSL (60000: none)
// Satellite imagery at zoom 17..20 around the aircraft (SatelliteDetail), premultiplied, repeat wrapping
layout(binding = 5) uniform sampler2D detail0;
layout(binding = 6) uniform sampler2D detail1;
layout(binding = 7) uniform sampler2D detail2;
layout(binding = 8) uniform sampler2D detail3;

const float TRI_CELL = 25.0;     // TERRAIN_GRID.cellM
const float TRI_RADIUS = 500.0;   // TERRAIN_GRID.radiusM

float isoline(float h, float interval, float widthPx, float minSpacingPx)
{
    // Levels half a metre off the round figure: SRTM heights are whole metres
    float x = (h - 0.5) / interval;
    float fw = max(fwidth(x), 1e-5);
    float distPx = abs(fract(x + 0.5) - 0.5) / fw;
    float line = 1.0 - smoothstep(widthPx * 0.5 - 0.5, widthPx * 0.5 + 0.5, distPx);
    return line * smoothstep(minSpacingPx, minSpacingPx * 2.5, 1.0 / fw);
}

float kmGrid(vec2 p, float cell, float widthPx)
{
    vec2 x = p / cell;
    vec2 fw = max(fwidth(x), vec2(1e-5));
    vec2 distPx = abs(fract(x + 0.5) - 0.5) / fw;
    vec2 line = 1.0 - smoothstep(vec2(widthPx * 0.5 - 0.5), vec2(widthPx * 0.5 + 0.5), distPx);
    vec2 keep = smoothstep(vec2(6.0), vec2(16.0), 1.0 / fw);
    return max(line.x * keep.x, line.y * keep.y);
}

vec3 triCoords(vec2 p)
{
    return vec3(p.y, 0.8660254 * p.x - 0.5 * p.y, 0.8660254 * p.x + 0.5 * p.y) / (TRI_CELL * 0.8660254);
}

float triGrid(vec2 p, vec2 dpx, vec2 dpy, float widthPx)
{
    float r = length(p - extra.yz);
    if (r >= TRI_RADIUS) return 0.0;
    vec3 x = triCoords(p);
    vec3 fw = max(abs(triCoords(dpx)) + abs(triCoords(dpy)), vec3(1e-5));
    vec3 distPx = abs(fract(x + 0.5) - 0.5) / fw;
    vec3 line = 1.0 - smoothstep(vec3(widthPx * 0.5 - 0.5), vec3(widthPx * 0.5 + 0.5), distPx);
    float keep = smoothstep(2.5, 6.0, 1.0 / max(fw.x, max(fw.y, fw.z)));
    float edge = 1.0 - smoothstep(TRI_RADIUS * 0.8, TRI_RADIUS, r);
    return max(line.x, max(line.y, line.z)) * keep * edge;
}

vec3 schematicColor()
{
    float gridLine = kmGrid(vXZ, 1000.0, 1.0);
    vec2 dpx = dFdx(vXZ), dpy = dFdy(vXZ);
    float tri = extra.x > 0.001 ? triGrid(vXZ, dpx, dpy, 1.0) * extra.x : 0.0;
    float minor = isoline(vH, 10.0, 1.0, 3.5);
    float index = isoline(vH, 50.0, 1.4, 3.5);
    float major = isoline(vH, 250.0, 2.0, 3.0);
    float water = smoothstep(0.5, 0.95, vWater);
    float bed = (1.0 - water) * smoothstep(0.0, -2.0, vH);
    vec3 col;
    if (extra.w > 0.5) {
        col = mix(vec3(0.31, 0.48, 0.28), vec3(0.80, 0.89, 0.67), vShade);
        col = mix(col, col * vec3(0.72, 0.86, 1.15), bed);
        col = mix(col, vec3(0.62, 0.76, 0.90), water);
        col = mix(col, vec3(0.27, 0.43, 0.39), gridLine * 0.35);
        col = mix(col, mix(vec3(0.25, 0.41, 0.30), vec3(0.10, 0.33, 0.72), water), tri * mix(0.18, 0.4, water));
        col = mix(col, vec3(0.22, 0.37, 0.20), minor * 0.4);
        col = mix(col, vec3(0.12, 0.25, 0.12), index * 0.65);
        col = mix(col, vec3(0.62, 0.30, 0.05), major * 0.9);
    } else {
        col = vec3(0.022, 0.028, 0.034) + vec3(0.115, 0.125, 0.135) * vShade;
        col = mix(col, col * vec3(0.70, 0.90, 1.35), bed);
        col = mix(col, vec3(0.015, 0.045, 0.085), water);
        col = mix(col, vec3(0.12, 0.24, 0.28), gridLine * 0.7);
        col = mix(col, mix(vec3(0.20, 0.32, 0.35), vec3(0.16, 0.42, 0.85), water), tri * mix(0.3, 0.55, water));
        col = mix(col, vec3(0.38, 0.43, 0.47), minor * 0.75);
        col = mix(col, vec3(0.74, 0.79, 0.82), index * 0.85);
        col = mix(col, vec3(1.00, 0.60, 0.16), major);
    }
    return col * (params.y / 0.85);
}

// Satellite detail (SatelliteDetail.js): the levels are composited finest
// first, each under the ones before it, and a fragment stops as soon as it is
// covered. Reads in a branch that differs between neighbouring fragments cannot
// take their mip level from implicit derivatives: textureGrad() gets the world
// x/z derivatives, taken once before any branch, scaled to each level.
const float DETAIL_SIZE = 2048.0;
const float DETAIL_FADE = 128.0;   // half a tile

vec4 detailUnder(vec4 acc, vec4 t, vec2 p)
{
    vec2 edge = min(p, DETAIL_SIZE - p);
    float w = smoothstep(0.0, DETAIL_FADE, min(edge.x, edge.y));
    return acc + t * w * (1.0 - acc.a);
}

vec3 satelliteDetail(vec3 base, vec2 xz)
{
    vec2 dx = dFdx(xz), dy = dFdy(xz);
    vec4 acc = vec4(0.0);
    if (detailOn.w > 0.5) {
        vec2 p = (xz - detailMap[3].xy) * detailMap[3].zw + detailWin[3].xy;
        vec2 k = detailMap[3].zw / DETAIL_SIZE;
        if (min(min(p.x, p.y), DETAIL_SIZE - max(p.x, p.y)) > 0.0)
            acc = detailUnder(acc, textureGrad(detail3, (p + detailWin[3].zw) / DETAIL_SIZE, dx * k, dy * k), p);
    }
    if (detailOn.z > 0.5 && acc.a < 0.999) {
        vec2 p = (xz - detailMap[2].xy) * detailMap[2].zw + detailWin[2].xy;
        vec2 k = detailMap[2].zw / DETAIL_SIZE;
        if (min(min(p.x, p.y), DETAIL_SIZE - max(p.x, p.y)) > 0.0)
            acc = detailUnder(acc, textureGrad(detail2, (p + detailWin[2].zw) / DETAIL_SIZE, dx * k, dy * k), p);
    }
    if (detailOn.y > 0.5 && acc.a < 0.999) {
        vec2 p = (xz - detailMap[1].xy) * detailMap[1].zw + detailWin[1].xy;
        vec2 k = detailMap[1].zw / DETAIL_SIZE;
        if (min(min(p.x, p.y), DETAIL_SIZE - max(p.x, p.y)) > 0.0)
            acc = detailUnder(acc, textureGrad(detail1, (p + detailWin[1].zw) / DETAIL_SIZE, dx * k, dy * k), p);
    }
    if (detailOn.x > 0.5 && acc.a < 0.999) {
        vec2 p = (xz - detailMap[0].xy) * detailMap[0].zw + detailWin[0].xy;
        vec2 k = detailMap[0].zw / DETAIL_SIZE;
        if (min(min(p.x, p.y), DETAIL_SIZE - max(p.x, p.y)) > 0.0)
            acc = detailUnder(acc, textureGrad(detail0, (p + detailWin[0].zw) / DETAIL_SIZE, dx * k, dy * k), p);
    }
    return base * (1.0 - acc.a) + acc.rgb;
}

// UAS geographical zones painted on the terrain (GeoZoneTerrain.js, colours of
// GeoZoneStyle.js): no-fly red with black bands, conditions violet with bands,
// a ceiling coloured by the height that may be flown
const vec3 GZ_PROHIBITED = vec3(0.9098, 0.1373, 0.1137);
const vec3 GZ_CONDITIONAL = vec3(0.6902, 0.3765, 1.0);
const float GZ_CEILING_MAX = 150.0;
const float GZ_BAND_M = 40.0;

vec3 geoZoneCeilingColor(float m)
{
    const vec3 c0 = vec3(1.0, 0.2510, 0.0941);
    const vec3 c1 = vec3(1.0, 0.3294, 0.1020);
    const vec3 c2 = vec3(1.0, 0.5490, 0.1020);
    const vec3 c3 = vec3(1.0, 0.7686, 0.1020);
    const vec3 c4 = vec3(0.8863, 0.9098, 0.1176);
    const vec3 c5 = vec3(0.5490, 0.8706, 0.2353);
    if (m <= 15.0) return mix(c0, c1, clamp(m / 15.0, 0.0, 1.0));
    if (m <= 30.0) return mix(c1, c2, clamp((m - 15.0) / 15.0, 0.0, 1.0));
    if (m <= 60.0) return mix(c2, c3, clamp((m - 30.0) / 30.0, 0.0, 1.0));
    if (m <= 90.0) return mix(c3, c4, clamp((m - 60.0) / 30.0, 0.0, 1.0));
    if (m <= 120.0) return mix(c4, c5, clamp((m - 90.0) / 30.0, 0.0, 1.0));
    return c5;
}

vec4 geoZoneTint(vec2 xz, float h)
{
    if (zone2.x < 0.5) return vec4(0.0);
    // Textures and derivatives before any branch that differs between fragments
    vec2 uv = (xz - zone.xy) * zone.zw;
    vec2 uvc = clamp(uv, 0.0, 1.0);
    vec3 c = texture(zoneCov, uvc).rgb;
    vec2 f = texture(zoneFloor, uvc).rg;
    float s = (xz.x + xz.y) * 0.70710678 / GZ_BAND_M;
    float sw = fwidth(s);
    vec3 cw = max(fwidth(c), vec3(0.02));
    if (uv.x < 0.0 || uv.y < 0.0 || uv.x > 1.0 || uv.y > 1.0) return vec4(0.0);
    vec3 cov = smoothstep(0.5 - cw, 0.5 + cw, c);
    float ceilM = min(f.r, f.g - h);
    float noFly = max(cov.r, cov.b * step(ceilM, 0.5));
    float tri = abs(fract(s) - 0.5);
    float w = max(sw, 1e-4);
    float dark = mix(smoothstep(0.25 - w, 0.25 + w, tri), 0.5, smoothstep(0.12, 0.3, sw));
    vec4 col = vec4(geoZoneCeilingColor(ceilM), 0.42 * cov.b * (1.0 - smoothstep(GZ_CEILING_MAX - 30.0, GZ_CEILING_MAX, ceilM)));
    col = mix(col, vec4(mix(GZ_CONDITIONAL, vec3(0.02), dark * 0.8), 0.5), cov.g);
    return mix(col, vec4(mix(GZ_PROHIBITED, vec3(0.02), dark), 0.62), noFly);
}

void main()
{
    // Computed before the radius test so the derivatives run on every fragment of the quad
    vec3 schem = schematicColor();
    vec3 col;
    vec3 fogColor;
    float alpha = 1.0;
    if (params.w > 0.5) {
        // Alpha 0 marks a water surface for the outline pass (OutlineRenderer),
        // which then lets whatever is under the water show through it
        alpha = vWater > 0.5 ? 0.0 : 1.0;
        col = schem;
        fogColor = extra.w > 0.5 ? vec3(0.78, 0.86, 0.93) : vec3(0.0);
        if (length(vXZ - camPos.xz) > params.z) discard;
    } else {
        // Sampled on every fragment, before choosing, so the mip derivatives are defined
        vec3 sat = texture(satellite, vec3(vUV, max(vSat, 0.0))).rgb;
        // The full-resolution imagery around the aircraft over the chunk's, then the light
        col = satelliteDetail(vSat >= 0.0 ? sat : vTint, vXZ) * vLight;
        fogColor = vec3(0.62, 0.74, 0.86);
    }
    vec4 gz = geoZoneTint(vXZ, vH);
    col = mix(col, gz.rgb, gz.a);
    float d = camPos.w * vDist;
    float fog = 1.0 - exp(-d * d);
    fragColor = vec4(mix(col, fogColor, fog), alpha);
}
