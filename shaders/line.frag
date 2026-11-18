#version 440
layout(location = 0) in vec4 vColor;
layout(location = 1) in float vEdge;
layout(location = 2) flat in float vHalf;
layout(location = 3) noperspective in float vAlong;
layout(location = 4) flat in float vDash;   // dash length px, the gap 0.7 of it (ThickLine dash [10, 7])
layout(location = 0) out vec4 fragColor;

void main()
{
    if (vDash > 0.0 && mod(max(vAlong, 0.0), vDash * 1.7) > vDash) discard;
    float a = vColor.a * clamp(vHalf + 0.5 - abs(vEdge), 0.0, 1.0);
    if (a < 0.003) discard;
    fragColor = vec4(vColor.rgb * a, a);   // premultiplied
}
