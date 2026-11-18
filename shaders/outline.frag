#version 440
// Schematic outlines (renderSchematic of js/engine/Scene3D.js). Every visible
// edge of the terrain gets a line: where a ridge hides the ground behind it
// and, brighter and thicker, where it stands against the sky. The terrain has
// no such edges in its geometry: they are found in its depth, drawn in the
// pass before into tDepth. This pass writes the colour with the lines and the
// depth back, so the overlays drawn next are depth-tested against the ground.
// Texels are fetched by pixel: the output and the textures have one size, and
// gl_FragCoord counts rows the way each backend stores them.

layout(location = 0) in vec2 vNdc;
layout(location = 0) out vec4 fragColor;

layout(std140, binding = 0) uniform Buf {
    mat4 invViewProj;
    vec4 camPos;         // xyz camera, w fog density
    vec4 planes;         // x near, y far
    vec4 widths;         // x profile px, y skyline px, z brightness
    vec4 profileColor;   // light lines on dark ground, dark on light
    vec4 skylineColor;
    vec4 skyHorizon;     // sky gradient (both black in the dark theme)
    vec4 skyZenith;
};
layout(binding = 1) uniform sampler2D tColor;
layout(binding = 2) uniform sampler2D tDepth;

ivec2 gSize;

float depthAt(ivec2 p) { return texelFetch(tDepth, clamp(p, ivec2(0), gSize - 1), 0).r; }
vec4 colorAt(ivec2 p) { return texelFetch(tColor, clamp(p, ivec2(0), gSize - 1), 0); }

// Sky by elevation of the view ray through this pixel, so the gradient stays
// on the real horizon when the aircraft pitches and banks
vec3 skyColor()
{
    vec4 far = invViewProj * vec4(vNdc, 1.0, 1.0);
    vec3 dir = normalize(far.xyz / far.w - camPos.xyz);
    return mix(skyHorizon.rgb, skyZenith.rgb, smoothstep(0.0, 0.55, dir.y));
}

// 1 / view depth. It varies linearly across a plane in screen space, so the
// pixel on one side predicts the one on the other: a neighbour much farther
// than that prediction is across an edge, while a slope seen at a grazing
// angle (a big but steady change in depth) is not.
float invDepth(float d)
{
    return 1.0 / planes.x + d * (1.0 / planes.y - 1.0 / planes.x);
}

void main()
{
    gSize = textureSize(tDepth, 0);
    ivec2 p = ivec2(gl_FragCoord.xy);
    vec4 color = colorAt(p);
    float dc = depthAt(p);
    // The terrain marks a water surface with alpha 0: its depth is not written
    // back, so the overlays under the water are drawn in full
    float waterSurface = color.a < 0.5 ? 1.0 : 0.0;
    color.a = 1.0;
    if (dc >= 1.0) {
        // Sky, unless it is a crack between two terrain chunks of different
        // detail, one pixel wide: ground on both sides of it horizontally or
        // vertically. Those are filled from a neighbour, not outlined.
        float r = depthAt(p + ivec2(1, 0)), l = depthAt(p - ivec2(1, 0));
        float u = depthAt(p + ivec2(0, 1)), d = depthAt(p - ivec2(0, 1));
        if (max(r, l) < 1.0) {
            gl_FragDepth = max(r, l);
            fragColor = vec4(colorAt(p + ivec2(1, 0)).rgb, 1.0);
        } else if (max(u, d) < 1.0) {
            gl_FragDepth = max(u, d);
            fragColor = vec4(colorAt(p + ivec2(0, 1)).rgb, 1.0);
        } else {
            gl_FragDepth = dc;
            fragColor = vec4(skyColor(), 1.0);
        }
        return;
    }
    gl_FragDepth = waterSurface > 0.5 ? 1.0 : dc;
    float ic = invDepth(dc);
    float profile = 0.0;
    float skyline = 0.0;

    // Lines are drawn on the near side of an edge only, so a ridge gets one
    // line of the requested width however far the ground behind it is
    for (int k = 0; k < 4; k++) {
        ivec2 dir = k == 0 ? ivec2(1, 0) : k == 1 ? ivec2(-1, 0) : k == 2 ? ivec2(0, 1) : ivec2(0, -1);
        for (int s = 1; s <= 4; s++) {
            float fs = float(s);
            if (fs > widths.y) break;
            float dn = depthAt(p + dir * s);
            if (dn >= 1.0) {
                // Real sky goes on beyond; a crack is over within two pixels
                if (depthAt(p + dir * (s + 2)) >= 1.0) skyline = 1.0;
                break;
            }
            if (fs > widths.x) continue;
            float dm = depthAt(p - dir * s);
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
    float fog = 1.0 - exp(-camPos.w * camPos.w * z * z);
    float profileA = profile * mix(1.0, 0.6, fog);
    color.rgb = mix(color.rgb, profileColor.rgb * widths.z, profileA);
    color.rgb = mix(color.rgb, skylineColor.rgb * widths.z, skyline);
    fragColor = color;
}
