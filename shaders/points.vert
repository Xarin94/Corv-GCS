#version 440
// LiDAR map points (VERT of js/engine/three/PointCloudRenderer.js): one quad
// per point, a few pixels wide (the point size is not available on every
// backend), coloured by height over the map's 2-98 % range or by reflectivity.
layout(location = 0) in vec2 corner;   // -1..1
layout(location = 1) in vec4 point;    // per instance: xyz from the cloud's origin (x east, y up, z south), w intensity 0..1
layout(location = 0) out vec2 vCorner;
layout(location = 1) out float vT;
layout(location = 2) out float vAlpha;

layout(std140, binding = 0) uniform Buf {
    mat4 mvp;
    vec4 params;     // x size px, y mode (0 height, 1 intensity), z hmin, w hmax
    vec4 viewport;   // xy px, z now s, w ttl s
};

void main()
{
    vec4 c = mvp * vec4(point.xyz, 1.0);
    vCorner = corner;
    vAlpha = 1.0;
    float h = clamp((point.y - params.z) / max(params.w - params.z, 0.5), 0.0, 1.0);
    vT = params.y < 0.5 ? h : point.w;
    if (c.w <= 0.0) {
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
        return;
    }
    gl_Position = c + vec4(corner * params.x / viewport.xy * c.w, 0.0, 0.0);
}
