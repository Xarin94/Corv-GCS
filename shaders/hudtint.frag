#version 440
// Premultiplied texture: white symbol over a black shadow. Multiplying the
// colour channels by the tint gives exactly the symbol drawn in that colour
// over the same shadow.

layout(location = 0) in vec2 vTex;
layout(location = 0) out vec4 fragColor;

layout(std140, binding = 0) uniform buf {
    mat4 qt_Matrix;
    vec4 tint;
    float qt_Opacity;
};
layout(binding = 1) uniform sampler2D tex;

void main()
{
    vec4 c = texture(tex, vTex);
    fragColor = vec4(c.rgb * tint.rgb, c.a) * qt_Opacity;
}
