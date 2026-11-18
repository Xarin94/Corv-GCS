#version 440
// Vehicle model: one triangle list of position, normal and material colour,
// lit by the sun and an ambient term (Lambert; MeshStandardMaterial in Electron).
layout(location = 0) in vec3 pos;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec3 color;
layout(location = 0) out vec3 vColor;

layout(std140, binding = 0) uniform Buf {
    mat4 mvp;
    mat4 model;
    vec4 light;    // xyz towards the sun
    vec4 params;   // x ambient, y diffuse
};

void main()
{
    vec3 n = normalize(mat3(model) * normal);
    float l = params.x + params.y * max(dot(n, light.xyz), 0.0) + 0.15 * max(n.y, 0.0);
    vColor = color * l;
    gl_Position = mvp * vec4(pos, 1.0);
}
