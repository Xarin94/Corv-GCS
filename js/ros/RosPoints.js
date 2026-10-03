/**
 * RosPoints.js - Points out of ROS sensor messages, sampled and georeferenced
 *
 * Three message types cover the sensors this is for:
 *   sensor_msgs/PointCloud2  3D LiDAR (livox_ros_driver2, velodyne, ouster),
 *                            a multibeam sonar, a SLAM map (FAST-LIO, MAVROS)
 *   sensor_msgs/LaserScan    2D scanner, push-broom LiDAR, a multibeam swath
 *   sensor_msgs/Range        single beam: echo sounder (Ping1D), altimeter
 * ROS 2 type names (sensor_msgs/msg/PointCloud2) are the same types.
 *
 * Not every point is used: at most `max` per message, picked with a uniform
 * stride from a random start, so a 20 000-point scan costs what `max` points
 * cost and successive scans sample different points. Range limits apply in
 * the sensor frame; NaN, ±Inf and the (0, 0, 0) "no return" points are
 * dropped.
 *
 * rosbridge sends uint8[] (PointCloud2.data) as base64 in JSON and as a CBOR
 * byte string; float32[] (LaserScan.ranges) as a JSON array (non-finite values
 * become null) or a CBOR typed array (RFC 8746 tag 85). Both are read here.
 *
 * Frames (REP-103/105). Points in a sensor frame (x forward, y left, z up;
 * a Range or LaserScan measures along x) are georeferenced through the vehicle
 * pose with the chain lidar-core.js uses for the Livox:
 *   p_frd  = (x, -y, -z)                  REP-103 FLU → body FRD
 *   p_body = R_mount · p_frd + lever      mount attitude vs the autopilot IMU
 *   p_ned  = R_att · p_body               ATTITUDE
 *   ENU    = vehicle ENU + (e, n, -d)
 * Points in a world-fixed frame (map, odom: ENU from the EKF origin, as MAVROS
 * publishes it; *_ned frames are NED) are placed at that origin.
 */

export const SUPPORTED_TYPES = {
    'sensor_msgs/PointCloud2': 'cloud',
    'sensor_msgs/LaserScan': 'scan',
    'sensor_msgs/Range': 'range'
};

/** 'cloud' | 'scan' | 'range' for a ROS 1 or ROS 2 type name, null if unsupported. */
export function kindOf(type) {
    return SUPPORTED_TYPES[String(type || '').replace('/msg/', '/')] || null;
}

// REP-105 world-fixed frames
const WORLD_FRAME = /(^|\/)(map|odom|world|earth|local_origin)(_ned|_enu)?$/i;

/** Frame handling for a header.frame_id: 'world-enu', 'world-ned' or 'sensor'. */
export function frameOf(frameId, forced = 'auto') {
    const id = String(frameId || '');
    const ned = /_ned$/i.test(id);
    if (forced === 'world') return ned ? 'world-ned' : 'world-enu';
    if (forced === 'sensor') return 'sensor';
    return WORLD_FRAME.test(id) ? (ned ? 'world-ned' : 'world-enu') : 'sensor';
}

// ============== DECODING ==============
function base64ToBytes(s) {
    if (typeof Uint8Array.fromBase64 === 'function') return Uint8Array.fromBase64(s);
    const bin = atob(s);
    const out = new Uint8Array(bin.length);
    for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
    return out;
}

function bytesOf(data) {
    if (data instanceof Uint8Array) return data;
    if (typeof data === 'string') return base64ToBytes(data);
    if (Array.isArray(data)) return Uint8Array.from(data);
    if (data && ArrayBuffer.isView(data)) return new Uint8Array(data.buffer, data.byteOffset, data.byteLength);
    return null;
}

// float32[] as it may arrive: typed array, JSON array (null = non-finite),
// or an RFC 8746 tag a decoder left undecoded.
function floatsOf(v) {
    if (!v) return null;
    if (v instanceof Float32Array || v instanceof Float64Array || Array.isArray(v)) return v;
    if (v.contents instanceof Uint8Array && (v.tag === 85 || v.tag === 81)) {
        const b = v.contents, little = v.tag === 85;
        const dv = new DataView(b.buffer, b.byteOffset, b.byteLength);
        const out = new Float32Array(b.byteLength >> 2);
        for (let i = 0; i < out.length; i++) out[i] = dv.getFloat32(i * 4, little);
        return out;
    }
    return null;
}

const FIELD_READ = {
    1: (dv, o) => dv.getInt8(o),
    2: (dv, o) => dv.getUint8(o),
    3: (dv, o, le) => dv.getInt16(o, le),
    4: (dv, o, le) => dv.getUint16(o, le),
    5: (dv, o, le) => dv.getInt32(o, le),
    6: (dv, o, le) => dv.getUint32(o, le),
    7: (dv, o, le) => dv.getFloat32(o, le),
    8: (dv, o, le) => dv.getFloat64(o, le)
};

// Indices of a uniform sample of `max` out of `total`, from a random start
function stride(total, max) {
    if (total <= max) return { step: 1, start: 0, n: total };
    const step = total / max;
    return { step, start: Math.random() * step, n: max };
}

/**
 * Decode the sampled points of one message into `out` (x, y, z in the
 * message frame).
 * @returns {{count: number, total: number}} points written, points the message carried
 */
export function decodePoints(kind, msg, out, max, rMin = 0, rMax = Infinity) {
    const cap = Math.min(max, Math.floor(out.length / 3));
    const r2min = rMin * rMin, r2max = rMax * rMax;
    let count = 0;
    const keep = (x, y, z) => {
        if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) return;
        const r2 = x * x + y * y + z * z;
        if (r2 === 0 || r2 < r2min || r2 > r2max) return;
        out[count * 3] = x; out[count * 3 + 1] = y; out[count * 3 + 2] = z;
        count++;
    };

    if (kind === 'range') {
        const r = msg.range;
        if (typeof r === 'number' && r >= (msg.min_range || 0) && r <= (msg.max_range || Infinity)) keep(r, 0, 0);
        return { count, total: 1 };
    }

    if (kind === 'scan') {
        const ranges = floatsOf(msg.ranges);
        if (!ranges) return { count: 0, total: 0 };
        const total = ranges.length;
        const lo = Math.max(msg.range_min || 0, rMin), hi = Math.min(msg.range_max || Infinity, rMax);
        const s = stride(total, cap);
        for (let k = 0; k < s.n; k++) {
            const idx = Math.floor(s.start + k * s.step);
            const r = ranges[idx];
            if (typeof r !== 'number' || !(r >= lo && r <= hi)) continue;
            const a = msg.angle_min + idx * msg.angle_increment;
            keep(r * Math.cos(a), r * Math.sin(a), 0);
        }
        return { count, total };
    }

    if (kind === 'cloud') {
        const bytes = bytesOf(msg.data);
        const total = (msg.width || 0) * (msg.height || 0);
        if (!bytes || !total || !msg.point_step) return { count: 0, total };
        const fields = {};
        for (const f of msg.fields || []) fields[f.name] = f;
        const fx = fields.x, fy = fields.y, fz = fields.z;
        if (!fx || !fy || !fz || !FIELD_READ[fx.datatype] || !FIELD_READ[fy.datatype] || !FIELD_READ[fz.datatype]) {
            return { count: 0, total };
        }
        const rx = FIELD_READ[fx.datatype], ry = FIELD_READ[fy.datatype], rz = FIELD_READ[fz.datatype];
        const le = !msg.is_bigendian;
        const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
        const width = msg.width, step = msg.point_step, rowStep = msg.row_step || width * step;
        const last = bytes.byteLength - step;
        const s = stride(total, cap);
        for (let k = 0; k < s.n; k++) {
            const idx = Math.floor(s.start + k * s.step);
            const o = Math.floor(idx / width) * rowStep + (idx % width) * step;
            if (o > last) break;
            keep(rx(dv, o + fx.offset, le), ry(dv, o + fy.offset, le), rz(dv, o + fz.offset, le));
        }
        return { count, total };
    }
    return { count: 0, total: 0 };
}

// Scratch for despike(), grown on demand
let spikeR = new Float32Array(0), spikeW = new Float32Array(0), spikeKeep = new Uint8Array(0);

function medianOf(a, n) {
    for (let i = 1; i < n; i++) {
        const v = a[i];
        let j = i - 1;
        while (j >= 0 && a[j] > v) { a[j + 1] = a[j]; j--; }
        a[j + 1] = v;
    }
    return n & 1 ? a[n >> 1] : 0.5 * (a[(n >> 1) - 1] + a[n >> 1]);
}

/**
 * Spikes in a scan profile, dropped in place. Points in the order the sensor
 * sent them — a sonar's beams across its swath, a LaserScan's sweep, a lidar's
 * rings — form a profile: a real surface is continuous from one beam to the
 * next but at a few edges, while aeration under the hull, a fish or a particle
 * is one beam, or a few, far off its neighbours. A point whose range is more
 * than max(ABS, REL × range) off the median of the 2·HALF + 1 points around
 * it is dropped: the median keeps edges (a wall, a wreck's side) and drops
 * what is narrower than HALF beams.
 *
 * Only on ordered profiles: in a cloud without order (or sampled too sparsely
 * to keep it) the points around one are not its neighbours, and nothing is
 * dropped. A profile is ordered when the median step from one point to the
 * next is small against the spread of its ranges.
 * @param {Float32Array} pts  x, y, z in the sensor frame (decodePoints), compacted in place
 * @returns {{count: number, removed: number, ordered: boolean}}
 */
export function despike(pts, count, opts = {}) {
    const half = opts.half ?? 4, abs = opts.abs ?? 0.5, rel = opts.rel ?? 0.08;
    const win = 2 * half + 1;
    if (count < win) return { count, removed: 0, ordered: false };
    if (spikeR.length < count) {
        spikeR = new Float32Array(count);
        spikeKeep = new Uint8Array(count);
        spikeW = new Float32Array(Math.max(win, count));
    }
    const r = spikeR, w = spikeW, keep = spikeKeep;
    for (let k = 0; k < count; k++) {
        const x = pts[k * 3], y = pts[k * 3 + 1], z = pts[k * 3 + 2];
        r[k] = Math.sqrt(x * x + y * y + z * z);
    }
    // Ordered? median step vs the 10–90 % spread (on a subsample: sorting is O(n²) here)
    const sub = Math.max(1, Math.floor(count / 256));
    let n = 0;
    for (let k = sub; k < count; k += sub) w[n++] = Math.abs(r[k] - r[k - 1]);
    const step = medianOf(w, n);
    n = 0;
    for (let k = 0; k < count; k += sub) w[n++] = r[k];
    medianOf(w, n);
    const spread = w[Math.floor(0.9 * (n - 1))] - w[Math.floor(0.1 * (n - 1))];
    if (step > Math.max(0.1 * spread, abs / 4)) return { count, removed: 0, ordered: false };

    for (let k = 0; k < count; k++) {
        const a = Math.max(0, Math.min(count - win, k - half));
        for (let i = 0; i < win; i++) w[i] = r[a + i];
        const m = medianOf(w, win);
        keep[k] = Math.abs(r[k] - m) <= Math.max(abs, rel * m) ? 1 : 0;
    }
    let out = 0;
    for (let k = 0; k < count; k++) {
        if (!keep[k]) continue;
        if (out !== k) { pts[out * 3] = pts[k * 3]; pts[out * 3 + 1] = pts[k * 3 + 1]; pts[out * 3 + 2] = pts[k * 3 + 2]; }
        out++;
    }
    return { count: out, removed: count - out, ordered: true };
}

// ============== GEOREFERENCING ==============
const M_PER_DEG = 111320;
const PROJECT_STEP_S = 0.02;

/**
 * Pose at time t (ms), projected from the last attitude sample with its body
 * rates (ZYX Euler kinematics, 20 ms steps)
 *   φ' = p + (q sinφ + r cosφ) tanθ,  θ' = q cosφ − r sinφ,  ψ' = (q sinφ + r cosφ) / cosθ
 * and from the last position sample with its NED velocity (m/s; lat/lon at
 * 111 320 m per degree, the app's convention).
 * @param {{t, roll, pitch, yaw, p, q, r}} a   attitude sample
 * @param {{t, lat, lon, alt, vn, ve, vd}} q   position sample
 * @param {number} t      time to project to, ms on the samples' clock
 * @param {number} [k=1]  autopilot seconds per second of that clock: the rates
 *                        and velocities are per autopilot second (a SITL at
 *                        speedup 20: k = 20)
 */
export function projectPose(a, q, t, k = 1) {
    let roll = a.roll, pitch = a.pitch, yaw = a.yaw;
    const dt = k * (t - a.t) / 1000, n = Math.max(1, Math.ceil(Math.abs(dt) / PROJECT_STEP_S)), h = dt / n;
    for (let k = 0; k < n; k++) {
        const sr = Math.sin(roll), cr = Math.cos(roll), qr = a.q * sr + a.r * cr;
        roll += h * (a.p + qr * Math.tan(pitch));
        pitch += h * (a.q * cr - a.r * sr);
        yaw += h * qr / Math.max(1e-3, Math.cos(pitch));
    }
    const dp = k * (t - q.t) / 1000;
    return {
        lat: q.lat + q.vn * dp / M_PER_DEG,
        lon: q.lon + q.ve * dp / (M_PER_DEG * Math.cos(q.lat * Math.PI / 180)),
        alt: q.alt - q.vd * dp,
        roll, pitch, yaw
    };
}


/**
 * ZYX Euler → rotation matrix (row-major 3×3) mapping the rotated frame into
 * its parent: body → NED for the attitude, sensor → body for the mount.
 */
export function eulerToMatrix(roll, pitch, yaw, out = new Float64Array(9)) {
    const cr = Math.cos(roll), sr = Math.sin(roll);
    const cp = Math.cos(pitch), sp = Math.sin(pitch);
    const cy = Math.cos(yaw), sy = Math.sin(yaw);
    out[0] = cy * cp; out[1] = cy * sp * sr - sy * cr; out[2] = cy * sp * cr + sy * sr;
    out[3] = sy * cp; out[4] = sy * sp * sr + cy * cr; out[5] = sy * sp * cr - cy * sr;
    out[6] = -sp;     out[7] = cp * sr;                out[8] = cp * cr;
    return out;
}

/**
 * Local metric frame at an anchor. Spherical (111 320 m per degree, longitude
 * scaled by the anchor's latitude): the convention of latLonToMeters() and of
 * the relative navigation's synthetic coordinates, so metres here are metres
 * in the 3D view in both modes.
 */
export function makeAnchor(lat, lon, alt) {
    return { lat, lon, alt, mPerLat: M_PER_DEG, mPerLon: M_PER_DEG * Math.cos(lat * Math.PI / 180) };
}

/** ENU of a lat/lon/alt from the anchor. */
export function toEnu(anchor, lat, lon, alt, out = [0, 0, 0]) {
    out[0] = (lon - anchor.lon) * anchor.mPerLon;
    out[1] = (lat - anchor.lat) * anchor.mPerLat;
    out[2] = alt - anchor.alt;
    return out;
}

/**
 * Sensor-frame points → ENU from the anchor.
 * @param {Float32Array} src   x, y, z in the sensor frame (FLU)
 * @param {Float64Array} Rm    mount, sensor → body (eulerToMatrix of the mount angles)
 * @param {number[]} lever     IMU → sensor, m, FRD
 * @param {Float64Array} Ra    attitude, body → NED
 * @param {number[]} veh       vehicle ENU from the anchor
 */
export function sensorToEnu(src, count, Rm, lever, Ra, veh, out) {
    const m = Rm, a = Ra, lx = lever[0], ly = lever[1], lz = lever[2];
    for (let k = 0; k < count; k++) {
        const fx = src[k * 3], fy = -src[k * 3 + 1], fz = -src[k * 3 + 2];
        const bx = m[0] * fx + m[1] * fy + m[2] * fz + lx;
        const by = m[3] * fx + m[4] * fy + m[5] * fz + ly;
        const bz = m[6] * fx + m[7] * fy + m[8] * fz + lz;
        const pn = a[0] * bx + a[1] * by + a[2] * bz;
        const pe = a[3] * bx + a[4] * by + a[5] * bz;
        const pd = a[6] * bx + a[7] * by + a[8] * bz;
        out[k * 3] = veh[0] + pe;
        out[k * 3 + 1] = veh[1] + pn;
        out[k * 3 + 2] = veh[2] - pd;
    }
}

/**
 * World-frame points → ENU from the anchor.
 * @param {boolean} ned   the frame is NED (x north, y east, z down), else ENU
 * @param {number[]} org  the frame origin's ENU from the anchor
 */
export function worldToEnu(src, count, ned, org, out) {
    for (let k = 0; k < count; k++) {
        const x = src[k * 3], y = src[k * 3 + 1], z = src[k * 3 + 2];
        out[k * 3] = org[0] + (ned ? y : x);
        out[k * 3 + 1] = org[1] + (ned ? x : y);
        out[k * 3 + 2] = org[2] + (ned ? -z : z);
    }
}
