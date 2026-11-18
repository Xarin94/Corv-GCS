#version 440
// Triangles (three line families 60 degrees apart) or squares, fading where
// the lines crowd closer than a few pixels and towards the rim, with a faint
// fill so the surface reads as a sheet from below.
layout(location = 0) in vec2 vXZ;
layout(location = 0) out vec4 fragColor;

layout(std140, binding = 0) uniform Buf {
    mat4 viewProj;
    vec4 plane;
    vec4 color;
    vec4 style;
};

void main()
{
    float r = length(vXZ - plane.xy);
    if (r > plane.z) discard;
    vec3 x;
    if (style.x > 0.5)
        x = vec3(vXZ.y, 0.8660254 * vXZ.x - 0.5 * vXZ.y, 0.8660254 * vXZ.x + 0.5 * vXZ.y) / (plane.w * 0.8660254);
    else
        x = vec3(vXZ.x, vXZ.y, vXZ.x) / plane.w;
    vec3 fw = max(fwidth(x), vec3(1e-5));
    vec3 distPx = abs(fract(x + 0.5) - 0.5) / fw;
    vec3 line = 1.0 - smoothstep(vec3(0.0), vec3(1.0), distPx);
    float keep = smoothstep(2.5, 6.0, 1.0 / max(fw.x, max(fw.y, fw.z)));
    float edge = 1.0 - smoothstep(0.8 * plane.z, plane.z, r);
    float a = (max(line.x, max(line.y, line.z)) * keep + 0.06) * edge * color.a;
    if (a < 0.004) discard;
    fragColor = vec4(color.rgb * a, a);
}
