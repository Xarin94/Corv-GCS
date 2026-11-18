#version 440
// The averaged ROS surface (FRAGMENT of js/ros/RosMesh3D.js). Grey mode: the
// triangle edges of the lattice drawn from the cell coordinates (anti-aliased,
// ~1.5 px, faded out where cells get smaller than a few pixels; tile edges
// every 30 cells readable from further away) over a faint body. The colour
// modes: a solid surface shaded from the north-west, red (near) -> blue (far)
// by distance from the vehicle, or blue (as rough as the surface is on
// average) -> red (well above) by roughness. Premultiplied alpha.
layout(location = 0) in vec2 vCell;
layout(location = 1) in float vDist;
layout(location = 2) in float vLevel;
layout(location = 3) in float vRough;
layout(location = 4) in vec3 vNormal;
layout(location = 5) in vec3 vWorld;
layout(location = 0) out vec4 fragColor;

layout(std140, binding = 0) uniform Buf {
    mat4 viewProj;
    mat4 model;
    vec4 vehicle;
    vec4 color;
    vec4 params;
    vec4 rough;
    vec4 flags;
    vec4 cam;
};

// Distance colour ramp, red (near) -> violet -> blue (far); roughness the same ramp the other way
vec3 ramp(float t)
{
    const vec3 R = vec3(1.0, 0.22, 0.16), M = vec3(0.72, 0.30, 0.88), B = vec3(0.18, 0.45, 1.0);
    return t < 0.5 ? mix(R, M, t * 2.0) : mix(M, B, t * 2.0 - 1.0);
}
vec3 roughRamp(float r, float mean, float full)
{
    return ramp(1.0 - clamp((r - mean) / max(full - mean, 1e-3), 0.0, 1.0));
}
// Lit from the north-west, 45 deg up (x east, y up, z south); the side facing the camera
vec3 shade(vec3 col, vec3 n)
{
    n = normalize(n);
    if (dot(n, cam.xyz - vWorld) < 0.0) n = -n;
    const vec3 L = vec3(-0.5, 0.707, 0.5);
    return col * (0.42 + 0.68 * max(dot(n, normalize(L)), 0.0));
}

// Triangle edges of a lattice: along x, y and the x = y diagonal; faded out
// where its cells are smaller than a few pixels
float lattice(vec2 c)
{
    vec3 g = vec3(c.x, c.y, c.x - c.y);
    vec3 fw = max(fwidth(g), vec3(1e-6));
    vec3 l = 1.0 - smoothstep(vec3(0.5), vec3(1.3), abs(fract(g + 0.5) - 0.5) / fw);
    return max(l.x, max(l.y, l.z)) * smoothstep(3.0, 6.0, 1.0 / max(fw.x, fw.y));
}
// The mesh of level k: vertices on the centres of 2^k x 2^k cell groups
float levelLines(float k)
{
    float s = exp2(k);
    return lattice((vCell - 0.5 * (s - 1.0)) / s);
}

void main()
{
    float alpha = params.w;
    if (params.x > 0.5) {
        vec3 c = params.x > 1.5 ? roughRamp(vRough, rough.x, rough.y)
                                : ramp(clamp((vDist - params.y) / max(params.z - params.y, 1.0), 0.0, 1.0));
        fragColor = vec4(shade(c, vNormal) * alpha, alpha);
        return;
    }
    float k0 = floor(vLevel + 1e-3);
    float minor = mix(levelLines(k0), levelLines(k0 + 1.0), clamp(vLevel - k0, 0.0, 1.0));
    // Tile edges every 30 cells, readable from further away
    vec2 t = vCell / 30.0;
    vec2 ft = max(fwidth(t), vec2(1e-6));
    vec2 lt = 1.0 - smoothstep(vec2(0.6), vec2(1.6), abs(fract(t + 0.5) - 0.5) / ft);
    float major = max(lt.x, lt.y) * smoothstep(3.0, 6.0, 1.0 / max(ft.x, ft.y));
    float a = min(max(minor * 0.85, major * 0.6) + color.a, 1.0) * alpha;
    fragColor = vec4(color.rgb * a, a);
}
