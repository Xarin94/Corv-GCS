#version 440
// Terrain chunk (port of the vertex part of TerrainRenderer.js).
// Heights live in a texture array, one layer per chunk (R16 unorm with a
// +32768 offset: QRhi has no signed 16-bit format). The vertex shader
// rebuilds the position and the normal (central differences, one-sided on the
// chunk edge) from them; every chunk of a grid size shares one vertex buffer
// of (column, row) pairs, and the decimation is the index buffer.
// A chunk with satellite imagery carries its layer in the image array; the
// image covers the chunk edge to edge, so (column, row) / (grid - 1) is its UV.

layout(location = 0) in vec2 grid;        // column, row
layout(location = 1) in vec4 chunk;       // per instance: x0, z0, dx, dz
layout(location = 2) in vec2 layerMax;    // per instance: texture array layer, grid max index
layout(location = 3) in float satLayer;   // per instance: satellite image layer, -1 none

layout(location = 0) out float vLight;
layout(location = 1) out float vH;
layout(location = 2) out vec2 vXZ;
layout(location = 3) out float vShade;
layout(location = 4) out float vDist;
layout(location = 5) out float vWater;
layout(location = 6) out vec3 vTint;
layout(location = 7) out vec2 vUV;
layout(location = 8) flat out float vSat;

layout(std140, binding = 0) uniform Buf {
    mat4 viewProj;
    vec4 camPos;      // xyz camera, w fog density
    vec4 sunDir;      // xyz towards the sun, w sunlight on
    vec4 params;      // x unused, y brightness, z schematic radius, w schematic (1) / height tint (0)
    vec4 extra;       // x triangle grid strength, yz triangle grid centre, w light theme
    vec4 zone;        // UAS zones window: world x, z of its north-west corner; 1 / width, 1 / height
    vec4 zone2;       // x zones on
    vec4 detailMap[4];   // satellite detail (fragment stage)
    vec4 detailWin[4];
    vec4 detailOn;
};
layout(binding = 1) uniform sampler2DArray heights;

float hAt(ivec2 g, int l)
{
    return floor(texelFetch(heights, ivec3(g, l), 0).r * 65535.0 + 0.5) - 32768.0;
}

void main()
{
    int gmax = int(layerMax.y + 0.5);
    int l = int(layerMax.x + 0.5);
    ivec2 g = ivec2(grid + 0.5);
    float h = hAt(g, l);
    ivec2 e = ivec2(min(g.x + 1, gmax), g.y);
    ivec2 w = ivec2(max(g.x - 1, 0), g.y);
    ivec2 s = ivec2(g.x, min(g.y + 1, gmax));
    ivec2 n = ivec2(g.x, max(g.y - 1, 0));
    float hE = hAt(e, l), hW = hAt(w, l), hS = hAt(s, l), hN = hAt(n, l);
    float ex = float(e.x - w.x) * chunk.z;
    float ey = hE - hW;
    float sz = float(s.y - n.y) * chunk.w;
    float sy = hS - hN;
    vec3 normal = normalize(vec3(-sz * ey, sz * ex, -sy * ex));

    // Isolines follow a lightly smoothed height: metre-level SRTM noise on flat
    // ground would otherwise break a level into dozens of rings
    vH = 0.5 * h + 0.125 * (hE + hW + hS + hN);

    // Water: SRTM flattens lakes to one height and the sea to 0
    float water = 0.0;
    if (params.w > 0.5 && h >= 0.0 && hE == h && hW == h && hS == h && hN == h) {
        int x0 = max(g.x - 1, 0), x1 = min(g.x + 1, gmax);
        int y0 = max(g.y - 1, 0), y1 = min(g.y + 1, gmax);
        bool isFlat = hAt(ivec2(x1, y0), l) == h && hAt(ivec2(x0, y0), l) == h
                 && hAt(ivec2(x1, y1), l) == h && hAt(ivec2(x0, y1), l) == h
                 && hAt(ivec2(min(g.x + 2, gmax), g.y), l) == h && hAt(ivec2(max(g.x - 2, 0), g.y), l) == h
                 && hAt(ivec2(g.x, min(g.y + 2, gmax)), l) == h && hAt(ivec2(g.x, max(g.y - 2, 0)), l) == h;
        water = isFlat ? 1.0 : 0.0;
    }
    vWater = water;

    vec3 world = vec3(chunk.x + float(g.x) * chunk.z, h, chunk.y + float(g.y) * chunk.w);
    vXZ = world.xz;
    vLight = sunDir.w > 0.5 ? 0.45 + 1.05 * max(0.0, dot(normal, sunDir.xyz)) : params.y;
    // Cartographic hillshade of the schematic style: light from the north-west, 45 degrees up
    vShade = max(0.0, dot(normal, vec3(-0.5, 0.70710678, -0.5)));
    vDist = length(world - camPos.xyz);

    // Height palette of getHeightColor()
    const vec3 G = vec3(0.0431372549, 0.4, 0.137254902);
    const vec3 Y = vec3(0.902, 0.7647058824, 0.3529411765);
    const vec3 O = vec3(0.902, 0.494, 0.133);
    const vec3 R = vec3(0.906, 0.298, 0.235);
    const vec3 P = vec3(0.608, 0.349, 0.713);
    const vec3 B = vec3(0.204, 0.596, 0.858);
    vec3 tint = B;
    if (h <= -100.0) tint = vec3(0.0);
    else if (h <= 700.0) tint = G;
    else if (h <= 1400.0) tint = mix(G, Y, (h - 700.0) / 700.0);
    else if (h <= 2100.0) tint = mix(Y, O, (h - 1400.0) / 700.0);
    else if (h <= 2800.0) tint = mix(O, R, (h - 2100.0) / 700.0);
    else if (h <= 3500.0) tint = mix(R, P, (h - 2800.0) / 700.0);
    else if (h <= 4000.0) tint = mix(P, B, (h - 3500.0) / 500.0);
    vTint = tint;
    vUV = vec2(g) / float(gmax);
    vSat = satLayer;

    gl_Position = viewProj * vec4(world, 1.0);
}
