#version 440
// LiDAR live points (VERT_LIVE of js/engine/three/PointCloudRenderer.js): the
// scan that could not be georeferenced, around the vehicle, with the time each
// point arrived: clipped past the TTL and faded over its last 40 %. Height
// ramp over -60...+20 m around the vehicle.
layout(location = 0) in vec2 corner;
layout(location = 1) in vec4 point;    // per instance: xyz from the vehicle, w intensity 0..1
layout(location = 2) in float birth;   // per instance: s
layout(location = 0) out vec2 vCorner;
layout(location = 1) out float vT;
layout(location = 2) out float vAlpha;

layout(std140, binding = 0) uniform Buf {
    mat4 mvp;
    vec4 params;     // x size px, y mode (0 height, 1 intensity)
    vec4 viewport;   // xy px, z now s, w ttl s
};

void main()
{
    float age = viewport.z - birth;
    vCorner = corner;
    vT = 0.0;
    vAlpha = 0.0;
    vec4 c = mvp * vec4(point.xyz, 1.0);
    if (age > viewport.w || age < 0.0 || c.w <= 0.0) {
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
        return;
    }
    vAlpha = clamp((viewport.w - age) / (0.4 * viewport.w), 0.0, 1.0);
    float h = clamp((point.y + 60.0) / 80.0, 0.0, 1.0);
    vT = params.y < 0.5 ? h : point.w;
    gl_Position = c + vec4(corner * params.x / viewport.xy * c.w, 0.0, 0.0);
}
