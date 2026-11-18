#version 440
// Text labels over the 3D view (makeLabel / fitLabel of js/engine/SymbolLayer.js):
// each label is an image in an atlas, drawn as a quad of fixed size in pixels
// next to its world point, so it stays legible at any distance.
layout(location = 0) in vec2 seg;         // the segment corners: x 0..1, y -1..1
layout(location = 1) in vec4 anchor;      // per instance: world point
layout(location = 2) in vec4 rect;        // x, y offset of the lower-left corner in px (y up), width, height px
layout(location = 3) in vec4 uv;          // u0, v0 (top), u1, v1 (bottom) in the atlas
layout(location = 0) out vec2 vUV;

layout(std140, binding = 0) uniform Buf {
    mat4 viewProj;
    vec4 viewport;    // x, y size px; z opacity; w +1 when NDC y points up, -1 when down
};

void main()
{
    vec2 corner = vec2(seg.x, seg.y * 0.5 + 0.5);
    vec4 c = viewProj * vec4(anchor.xyz, 1.0);
    vUV = vec2(mix(uv.x, uv.z, corner.x), mix(uv.w, uv.y, corner.y));
    if (c.w < 1.0) {
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
        return;
    }
    // On whole pixels, so the text stays sharp
    vec2 at = floor((c.xy / c.w * 0.5 + 0.5) * viewport.xy + 0.5);
    vec2 px = vec2(rect.x + corner.x * rect.z, (rect.y + corner.y * rect.w) * viewport.w);
    gl_Position = vec4((at + px) / viewport.xy * 2.0 - 1.0, 0.0, 1.0);
}
