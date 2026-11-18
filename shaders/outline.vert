#version 440
// Full-screen triangle of the schematic outline pass (OutlineRenderer).
layout(location = 0) in vec2 pos;
layout(location = 0) out vec2 vNdc;

void main()
{
    vNdc = pos;
    gl_Position = vec4(pos, 0.0, 1.0);
}
