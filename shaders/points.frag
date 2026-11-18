#version 440
// Round LiDAR points: the turbo colour map by height (polynomial fit,
// Mikhailov, Google AI 2019) or a green ramp by reflectivity (RAMP of
// js/engine/three/PointCloudRenderer.js). Premultiplied alpha.
layout(location = 0) in vec2 vCorner;
layout(location = 1) in float vT;
layout(location = 2) in float vAlpha;
layout(location = 0) out vec4 fragColor;

layout(std140, binding = 0) uniform Buf {
    mat4 mvp;
    vec4 params;
    vec4 viewport;
};

vec3 turbo(float t)
{
    const vec4 kR4 = vec4(0.13572138, 4.61539260, -42.66032258, 132.13108234);
    const vec4 kG4 = vec4(0.09140261, 2.19418839, 4.84296658, -14.18503333);
    const vec4 kB4 = vec4(0.10667330, 12.64194608, -60.58204836, 110.36276771);
    const vec2 kR2 = vec2(-152.94239396, 59.28637943);
    const vec2 kG2 = vec2(4.27729857, 2.82956604);
    const vec2 kB2 = vec2(-89.90310912, 27.34824973);
    vec4 v4 = vec4(1.0, t, t * t, t * t * t);
    vec2 v2 = v4.zw * v4.z;
    return vec3(dot(v4, kR4) + dot(v2, kR2), dot(v4, kG4) + dot(v2, kG2), dot(v4, kB4) + dot(v2, kB2));
}

vec3 ramp(float t)
{
    return params.y < 0.5 ? turbo(clamp(t, 0.0, 1.0))
                          : mix(vec3(0.02, 0.18, 0.06), vec3(0.75, 1.0, 0.8), pow(clamp(t, 0.0, 1.0), 0.6));
}

void main()
{
    if (dot(vCorner, vCorner) > 1.0 || vAlpha <= 0.0) discard;
    fragColor = vec4(ramp(vT) * vAlpha, vAlpha);
}
