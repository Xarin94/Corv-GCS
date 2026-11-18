#version 440
// A cave's 3D surface (FRAGMENT of js/ros/RosVolume3D.js): ~1.5 px edges from
// the barycentric coordinates over a faint body, faded where a triangle is
// only a few pixels tall; in the colour modes a solid surface, each face
// shaded flat. Premultiplied alpha.
layout(location = 0) in vec3 vBary;
layout(location = 1) in float vDist;
layout(location = 2) in float vRough;
layout(location = 3) in vec3 vWorld;
layout(location = 0) out vec4 fragColor;

layout(std140, binding = 0) uniform Buf {
    mat4 viewProj;
    mat4 model;
    vec4 vehicle;
    vec4 color;
    vec4 params;
    vec4 spread;
    vec4 cam;
};

vec3 ramp(float t)
{
    const vec3 R = vec3(1.0, 0.22, 0.16), M = vec3(0.72, 0.30, 0.88), B = vec3(0.18, 0.45, 1.0);
    return t < 0.5 ? mix(R, M, t * 2.0) : mix(M, B, t * 2.0 - 1.0);
}
vec3 roughRamp(float r, float mean, float full)
{
    return ramp(1.0 - clamp((r - mean) / max(full - mean, 1e-3), 0.0, 1.0));
}
vec3 shade(vec3 col, vec3 n)
{
    n = normalize(n);
    if (dot(n, cam.xyz - vWorld) < 0.0) n = -n;
    const vec3 L = vec3(-0.5, 0.707, 0.5);
    return col * (0.42 + 0.68 * max(dot(n, normalize(L)), 0.0));
}

void main()
{
    if (params.x > 0.5) {
        vec3 c = params.x > 1.5 ? roughRamp(vRough, spread.x, spread.y)
                                : ramp(clamp((vDist - params.y) / max(params.z - params.y, 1.0), 0.0, 1.0));
        fragColor = vec4(shade(c, cross(dFdx(vWorld), dFdy(vWorld))), 1.0);
        return;
    }
    vec3 fw = max(fwidth(vBary), vec3(1e-6));
    vec3 e = smoothstep(fw * 0.5, fw * 1.5, vBary);
    float edge = 1.0 - min(min(e.x, e.y), e.z);
    float keep = 1.0 - smoothstep(0.15, 0.35, max(fw.x, max(fw.y, fw.z)));   // triangle under ~5 px: no lines
    float a = min(edge * keep * 0.85 + color.a, 1.0);
    fragColor = vec4(color.rgb * a, a);
}
