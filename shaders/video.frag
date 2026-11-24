#version 440
// YUV of the decoder to RGB. In AR the blend is "screen" (VideoRenderer): the
// black of the 3D view lets the camera image through.
layout(location = 0) in vec2 vUV;
layout(location = 0) out vec4 fragColor;

layout(std140, binding = 0) uniform Buf {
    mat4 clipCorr;
    vec4 crop;
    vec4 mode;
};
layout(binding = 1) uniform sampler2D tY;
layout(binding = 2) uniform sampler2D tU;   // U, or U V interleaved
layout(binding = 3) uniform sampler2D tV;

void main()
{
    float y = texture(tY, vUV).r;
    vec4 c1 = texture(tU, vUV);
    float c2 = texture(tV, vUV).r;
    vec2 uv = (mode.x > 0.5 ? c1.rg : vec2(c1.r, c2)) - 128.0 / 255.0;
    vec3 rgb;
    if (mode.y > 0.5) {
        rgb = mode.z > 0.5 ? vec3(y + 1.5748 * uv.y, y - 0.187324 * uv.x - 0.468124 * uv.y, y + 1.8556 * uv.x)
                           : vec3(y + 1.402 * uv.y, y - 0.344136 * uv.x - 0.714136 * uv.y, y + 1.772 * uv.x);
    } else {
        float l = 1.164383 * (y - 16.0 / 255.0);
        rgb = mode.z > 0.5 ? vec3(l + 1.792741 * uv.y, l - 0.213249 * uv.x - 0.532909 * uv.y, l + 2.112402 * uv.x)
                           : vec3(l + 1.596027 * uv.y, l - 0.391762 * uv.x - 0.812968 * uv.y, l + 2.017232 * uv.x);
    }
    fragColor = vec4(clamp(rgb, 0.0, 1.0), 1.0);
}
