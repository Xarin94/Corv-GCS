#version 440
// The averaged ROS surface, one block of 241 x 241 cells (VERTEX of
// js/ros/RosMesh3D.js): each vertex built from its index and the block's
// texture of drawn heights - no vertex buffer, only an index list of the
// triangles whose three corners have a height, at the block's level of detail.
layout(location = 0) out vec2 vCell;
layout(location = 1) out float vDist;
layout(location = 2) out float vLevel;
layout(location = 3) out float vRough;
layout(location = 4) out vec3 vNormal;
layout(location = 5) out vec3 vWorld;

layout(std140, binding = 0) uniform Buf {
    mat4 viewProj;
    mat4 model;
    vec4 vehicle;   // xyz world, w cell m
    vec4 color;     // rgb, a fill opacity
    vec4 params;    // x mode (0 grey, 1 distance, 2 roughness), y near, z far, w alpha
    vec4 rough;     // x mean m, y full m, z step (cells), w vertices per row
    vec4 flags;     // x 1: the depth-reset pass, y relief (shading: heights x2)
    vec4 cam;       // xyz camera world
};
layout(binding = 1) uniform sampler2D heights;   // RGBA32F: height (< -1e8 none), level, roughness m

const int TEX = 241;

// A neighbour's height, this vertex's where there is none
float near(ivec2 p, float h)
{
    float v = texelFetch(heights, clamp(p, ivec2(0), ivec2(TEX - 1)), 0).r;
    return v < -1e8 ? h : v;
}

void main()
{
    int step = int(rough.z + 0.5), w = int(rough.w + 0.5);
    int id = gl_VertexIndex;
    ivec2 g = ivec2(id % w, id / w) * step;
    vec4 t = texelFetch(heights, g, 0);   // height, resolution level, roughness
    vLevel = t.g;
    vRough = t.b;
    float cell = vehicle.w;
    vec4 local = vec4((float(g.x) + 0.5) * cell, t.r, -(float(g.y) + 0.5) * cell, 1.0);
    vCell = vec2(g);
    vec4 world = model * local;
    vWorld = world.xyz;
    vDist = distance(world.xyz, vehicle.xyz);
    // Normal of y = h(x, z), x east, z = -north, from the neighbours at this level of detail
    float ds = 2.0 * float(step) * cell / flags.y;
    float dx = near(g + ivec2(step, 0), t.r) - near(g - ivec2(step, 0), t.r);
    float dn = near(g + ivec2(0, step), t.r) - near(g - ivec2(0, step), t.r);
    vNormal = normalize(mat3(model) * vec3(-dx / ds, 1.0, dn / ds));
    gl_Position = viewProj * world;
    if (flags.x > 0.5) gl_Position.z = gl_Position.w * 0.999999;   // the far plane: clears the depth under the surface
}
