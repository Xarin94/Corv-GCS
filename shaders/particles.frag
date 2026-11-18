#version 440
layout(location = 0) in vec2 vCoord;
layout(location = 1) in float vAlpha;
layout(location = 0) out vec4 fragColor;

layout(std140, binding = 0) uniform Buf {
    mat4 viewProj;
    vec4 cam;
    vec4 params;
    vec4 color;
};

void main()
{
    float r = dot(vCoord, vCoord);
    if (r > 1.0 || vAlpha < 0.01) discard;
    float a = vAlpha * (1.0 - r) * 0.75;
    fragColor = vec4(color.rgb * a, a);
}
