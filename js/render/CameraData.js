/** Column-major matrices, OpenGL clip depth [-1, 1], world metres E/up/south. */
export class CameraData {
    constructor() {
        this.revision = 0;
        this.position = [0, 0, 0];
        this.quaternion = [0, 0, 0, 1]; // x,y,z,w; forward is -Z
        this.projection = new Float64Array(16);
        this.view = new Float64Array(16);
        this.world = new Float64Array(16);
        this.planes = new Float64Array(24);
        this.viewProjection = new Float64Array(16);
        this.viewport = [1, 1, 1]; // CSS width, CSS height, render DPR
    }
    set({ position, quaternion, projection, view, world, near, far, fov, viewport }) {
        this.position[0] = position[0]; this.position[1] = position[1]; this.position[2] = position[2];
        for (let i = 0; i < 4; i++) this.quaternion[i] = quaternion[i];
        this.projection.set(projection); this.view.set(view); this.world.set(world);
        this.near = near; this.far = far; this.fov = fov;
        if (viewport) for (let i = 0; i < 3; i++) this.viewport[i] = viewport[i];
        const m = this.viewProjection;
        for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) {
            let v = 0;
            for (let k = 0; k < 4; k++) v += this.projection[k * 4 + r] * this.view[c * 4 + k];
            m[c * 4 + r] = v;
        }
        // row 4 ± rows 1,2,3, normalised: backend-independent sphere culling.
        for (let axis = 0; axis < 3; axis++) for (let side = 0; side < 2; side++) {
            const sign = side ? -1 : 1, offset = (axis * 2 + side) * 4;
            const x = m[3] + sign * m[axis], y = m[7] + sign * m[4 + axis], z = m[11] + sign * m[8 + axis];
            const length = Math.hypot(x, y, z);
            this.planes[offset] = x / length; this.planes[offset + 1] = y / length;
            this.planes[offset + 2] = z / length;
            this.planes[offset + 3] = (m[15] + sign * m[12 + axis]) / length;
        }
        this.revision++;
    }
    intersectsSphere(s) {
        for (let p = 0; p < 24; p += 4) {
            const f = this.planes;
            if (f[p] * s.x + f[p + 1] * s.y + f[p + 2] * s.z + f[p + 3] < -s.r) return false;
        }
        return true;
    }
}
