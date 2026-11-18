#version 440
// HUD instrument (HudInstruments.cpp): a texture painted white with a black
// shadow, tinted on the GPU, so the HUD colour (green, blue, the disarmed
// pulse) changes a uniform instead of repainting the instrument.

layout(location = 0) in vec4 qt_VertexPosition;
layout(location = 1) in vec2 qt_VertexTexCoord;
layout(location = 0) out vec2 vTex;

layout(std140, binding = 0) uniform buf {
    mat4 qt_Matrix;
    vec4 tint;
    float qt_Opacity;
};

void main()
{
    vTex = qt_VertexTexCoord;
    gl_Position = qt_Matrix * qt_VertexPosition;
}
