#version 440
// Translucent triangles with a colour per vertex: the fill of the predicted
// trajectory corridor (TrajectoryCorridor3D.js).
layout(location = 0) in vec3 pos;
layout(location = 1) in vec4 color;
layout(location = 0) out vec4 vColor;

layout(std140, binding = 0) uniform Buf {
    mat4 viewProj;
};

void main()
{
    vColor = color;
    gl_Position = viewProj * vec4(pos, 1.0);
}
