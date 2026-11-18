#version 440
// Screen-space thick line segments (port of js/engine/ThickLine.js).
// One instance per segment, four vertices (a triangle strip) per instance:
// both ends are projected, offset by their pixel offsets (symbols drawn as
// rings around a world point), and the quad is widened in pixels across the
// segment, so a line keeps its width at any distance. An end behind the
// camera is moved along the segment onto the near plane first.
// The ghost pass draws every segment again without the depth test, thinner
// and faint: the parts behind the terrain stay readable (ThickLine ghost).

layout(location = 0) in vec2 corner;      // x: 0 at a, 1 at b; y: -1 / +1 across
layout(location = 1) in vec4 pa;          // per instance: end a, world
layout(location = 2) in vec4 pb;          // end b
layout(location = 3) in vec4 offs;        // pixel offsets of a (xy) and b (zw)
layout(location = 4) in vec4 color;
layout(location = 5) in vec4 style;       // x width px, y ghost opacity (0: no ghost), z ghost width px, w dash px (0: solid)

layout(location = 0) out vec4 vColor;
layout(location = 1) out float vEdge;
layout(location = 2) flat out float vHalf;
layout(location = 3) noperspective out float vAlong;   // px from end a, for the dashes
layout(location = 4) flat out float vDash;

layout(std140, binding = 0) uniform Buf {
    mat4 viewProj;
    vec4 viewport;   // x, y size px; z 1 = ghost pass; w opacity
};

void main()
{
    bool ghost = viewport.z > 0.5;
    float width = ghost ? style.z : style.x;
    float alpha = color.a * viewport.w * (ghost ? style.y : 1.0);
    vec4 ca = viewProj * vec4(pa.xyz, 1.0);
    vec4 cb = viewProj * vec4(pb.xyz, 1.0);
    const float near = 1.0;
    if ((ca.w < near && cb.w < near) || alpha <= 0.0 || width <= 0.0) {
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);   // dropped: outside the clip volume
        vColor = vec4(0.0);
        vEdge = 0.0;
        vHalf = 0.0;
        vAlong = 0.0;
        vDash = 0.0;
        return;
    }
    if (ca.w < near) ca = mix(ca, cb, (near - ca.w) / (cb.w - ca.w));
    else if (cb.w < near) cb = mix(cb, ca, (near - cb.w) / (ca.w - cb.w));

    vec2 halfVp = 0.5 * viewport.xy;
    vec2 sa = ca.xy / ca.w * halfVp + offs.xy;
    vec2 sb = cb.xy / cb.w * halfVp + offs.zw;
    vec2 d = sb - sa;
    float len = length(d);
    vec2 dir = len > 1e-3 ? d / len : vec2(1.0, 0.0);
    vec2 n = vec2(-dir.y, dir.x);
    float hw = 0.5 * width;
    float geo = hw + 1.0;                      // one pixel more for the antialiased edge
    bool atB = corner.x > 0.5;
    vec4 c = atB ? cb : ca;
    vec2 s = (atB ? sb : sa) + dir * (atB ? 0.5 * hw : -0.5 * hw) + n * geo * corner.y;
    gl_Position = vec4(s / halfVp * c.w, c.z, c.w);
    vColor = vec4(color.rgb, alpha);
    vEdge = corner.y * geo;
    vHalf = hw;
    vAlong = atB ? len + 0.5 * hw : -0.5 * hw;
    vDash = style.w;
}
