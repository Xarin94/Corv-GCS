#version 440
layout(location = 0) in vec2 vUV;
layout(location = 0) out vec4 fragColor;

layout(std140, binding = 0) uniform Buf {
    mat4 viewProj;
    vec4 viewport;
};
layout(binding = 1) uniform sampler2D atlas;

void main()
{
    vec4 t = texture(atlas, vUV);   // premultiplied
    if (t.a < 0.004) discard;
    fragColor = t * viewport.z;
}
