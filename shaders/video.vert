#version 440
// Full-screen triangle of the FPV camera image (VideoRenderer): the frame covers
// the view, cropped about its centre with its aspect kept (drawFrame of FPVController.js).
layout(location = 0) in vec2 pos;
layout(location = 0) out vec2 vUV;

layout(std140, binding = 0) uniform Buf {
    mat4 clipCorr;
    vec4 crop;      // xy: the part of the frame shown, 1 = whole
    vec4 mode;      // x interleaved chroma, y full range, z BT.709
};

void main()
{
    vec2 uv = vec2(pos.x * 0.5 + 0.5, 0.5 - pos.y * 0.5);
    vUV = (uv - 0.5) * crop.xy + 0.5;
    gl_Position = clipCorr * vec4(pos, 0.0, 1.0);
}
