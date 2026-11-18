#version 440
// A cave's 3D surface (VERTEX of js/ros/RosVolume3D.js): the chunk meshes of
// the TSDF, metres east, north, up from the anchor; a barycentric attribute
// for the lines and the spread of the face normals for the roughness colours.
layout(location = 0) in vec3 position;
layout(location = 1) in vec3 bary;
layout(location = 2) in float roughIn;
layout(location = 0) out vec3 vBary;
layout(location = 1) out float vDist;
layout(location = 2) out float vRough;
layout(location = 3) out vec3 vWorld;

layout(std140, binding = 0) uniform Buf {
    mat4 viewProj;
    mat4 model;
    vec4 vehicle;   // xyz world
    vec4 color;     // rgb, a fill opacity
    vec4 params;    // x mode, y near, z far
    vec4 spread;    // x mean, y full
    vec4 cam;       // xyz camera world
};

void main()
{
    vBary = bary;
    vRough = roughIn;
    vec4 world = model * vec4(position, 1.0);
    vWorld = world.xyz;
    vDist = distance(world.xyz, vehicle.xyz);
    gl_Position = viewProj * world;
}
