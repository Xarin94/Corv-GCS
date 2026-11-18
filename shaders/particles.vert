#version 440
// Specks suspended in the water while the camera is under the surface
// (PARTICLE_VERTEX of js/engine/Water3D.js). They are fixed in the world and
// wrapped around the camera: each one is the copy of a lattice point nearest
// to it, so travelling through them shows the direction and speed of motion,
// whatever produced the position. Quads instead of points: the point size is
// not available on every backend (Direct3D).
layout(location = 0) in vec2 corner;      // -1..1
layout(location = 1) in vec3 lattice;     // per instance, 0..1
layout(location = 0) out vec2 vCoord;
layout(location = 1) out float vAlpha;

layout(std140, binding = 0) uniform Buf {
    mat4 viewProj;
    vec4 cam;      // xyz camera, w box size m
    vec4 params;   // x surface level, yz viewport px
    vec4 color;
};

void main()
{
    float box = cam.w;
    vec3 rel = mod(lattice * box - cam.xyz + 0.5 * box, vec3(box)) - 0.5 * box;
    vec3 world = cam.xyz + rel;
    vec4 c = viewProj * vec4(world, 1.0);
    vAlpha = (1.0 - smoothstep(0.25 * box, 0.5 * box, length(rel))) * step(world.y, params.x);
    vCoord = corner;
    if (c.w < 0.3 || vAlpha < 0.01) {
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
        return;
    }
    float sizePx = clamp(60.0 / max(c.w, 0.1), 1.0, 5.0) + 1.0;
    gl_Position = c + vec4(corner * 0.5 * sizePx / (0.5 * params.yz) * c.w, 0.0, 0.0);
}
