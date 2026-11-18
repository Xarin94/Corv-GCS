#version 440
// Water surface grid (PLANE_VERTEX / PLANE_FRAGMENT of js/engine/Water3D.js):
// a square on the surface around the vehicle, seen from above and from below.
layout(location = 0) in vec3 pos;
layout(location = 0) out vec2 vXZ;

layout(std140, binding = 0) uniform Buf {
    mat4 viewProj;
    vec4 plane;    // x, y centre (world x, z), z radius, w cell
    vec4 color;    // rgb, a opacity
    vec4 style;    // x triangles (1) / squares (0)
};

void main()
{
    vXZ = pos.xz;
    gl_Position = viewProj * vec4(pos, 1.0);
}
