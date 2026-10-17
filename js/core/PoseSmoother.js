/**
 * The pose the 3D view and the HUD draw the vehicle at.
 *
 * Telemetry writes STATE at its own rate (GLOBAL_POSITION_INT and ATTITUDE
 * at 25 Hz from ArduPilot, in bursts over a cellular link) while the view
 * renders at 60 fps. Drawn as it is, the picture stands still for one or two
 * frames and then jumps by a whole sample: flying at 70 m/s and 40 m above
 * the ground, 63 % of the frames did not move and the next one moved 2.8 m.
 *
 * Between samples the pose is projected forward, for at most 0.5 s:
 *  - position on the NED velocity of the sample and the acceleration seen
 *    between the last velocity samples: over the ground along the turn that
 *    acceleration describes, vertically v·dt + ½·a·dt²;
 *  - attitude on the angular rates: the body rates (p, q, r) converted to
 *    Euler angle rates, or, from a source that sends none, the rates seen
 *    between the last attitude samples.
 * When a sample arrives, the gap between the pose being drawn and the new
 * sample is blended out over `tau` seconds instead of jumped.
 *
 * Samples are timed by the vehicle's own clock (time_boot_ms) when it is
 * reported, mapped onto the GCS clock by the quickest arrival seen, so
 * arrival jitter and bursts do not show; otherwise by their arrival time.
 *
 * No DOM, no Three: STATE-shaped objects in, plain numbers out.
 */

const M_PER_DEG = 111320;
const TWO_PI = Math.PI * 2;
const HORIZON_S = 0.5;        // projection stops 0.5 s after the last sample: past that it would be guesswork
const CLOCK_HISTORY = 64;     // samples over which the quickest arrival is kept
const MIN_DIFF_S = 0.005;     // samples closer than this give no derivative
const MAX_DIFF_S = 1.0;       // nor samples further apart than this
const MAX_ACCEL = 30;         // m/s², ~3 g: anything above is a glitch
const MAX_RATE = 6;           // rad/s
const MAX_TURN = 1;           // rad/s of track change in the projection
const MIN_TURN_SPEED = 3;     // m/s: slower, the track is noise and the projection stays straight
const DERIV_BLEND = 0.5;      // weight of the newest derivative (noise vs lag)
const SNAP_M = 100;           // further than this is a seek or a new vehicle: jump
const SNAP_RAD = Math.PI / 3;

function wrapPi(a) {
    return Math.atan2(Math.sin(a), Math.cos(a));
}

function finiteOr(value, fallback) {
    return Number.isFinite(value) ? value : fallback;
}

/** Scale a 3-vector down to a maximum length. */
function clampNorm(v, max) {
    const n = Math.hypot(v[0], v[1], v[2]);
    if (n > max) for (let i = 0; i < 3; i++) v[i] *= max / n;
}

/**
 * One telemetry channel, position or attitude, as deviations from its last
 * sample: first and second derivative, the blend still to fade out, and the
 * timing of the samples.
 */
class Channel {
    constructor() {
        this.rate = new Float64Array(3);
        this.accel = new Float64Array(3);
        this.offset = new Float64Array(3);
        this.t = null;           // ms on the GCS clock, when the sample was taken
        this.stamp = null;       // arrival stamp of the last sample, to tell a new one
        this.boot = null;        // vehicle clock of the last sample (ms)
        this.lags = [];          // arrival - vehicle clock, for the clock map
        this.projection = new Float64Array(3);
    }

    /** Seconds of projection from the sample to time now (ms). */
    span(now) {
        return Math.min(Math.max(0, now - this.t) / 1000, HORIZON_S);
    }

    /** Projected deviation from the sample at time now (ms), without the blend: x + ½·a·t². */
    project(now) {
        const dt = this.span(now), out = this.projection;
        for (let i = 0; i < 3; i++) out[i] = this.rate[i] * dt + 0.5 * this.accel[i] * dt * dt;
        return out;
    }

    decay(now, tau) {
        return tau > 0 ? Math.exp(-Math.max(0, now - this.t) / 1000 / tau) : 0;
    }

    /**
     * When the new sample was taken, on the GCS clock (ms), and the time since
     * the previous one on the vehicle's clock where it has one (s, or null).
     */
    time(arrival, boot, now) {
        const fresh = Number.isFinite(arrival) && arrival > 0 && arrival !== this.stamp && arrival <= now && now - arrival < 1000;
        this.stamp = arrival;
        const at = fresh ? arrival : now;
        // The vehicle clock dates the sample only when it came with it: the
        // stamps a link or a replay leaves behind date nothing the demo
        // flight or the relative frame writes into STATE afterwards
        if (!fresh || !Number.isFinite(boot) || boot === this.boot) {
            this.lags.length = 0; this.boot = null;
            return { t: at, since: null };
        }
        let since = null;
        if (this.boot !== null) {
            since = (boot - this.boot) / 1000;
            // The vehicle rebooted or another one took over: start the clock map again
            if (!(since >= 0 && since < 5)) { this.lags.length = 0; since = null; }
        }
        this.boot = boot;
        this.lags.push(at - boot);
        if (this.lags.length > CLOCK_HISTORY) this.lags.shift();
        // Quickest arrival = least delayed: that delay is the link's own, the rest is jitter
        const t = Math.min(at, boot + Math.min(...this.lags));
        return { t, since };
    }

    /** Seconds from the previous sample to one taken at t: vehicle clock first. */
    gap(t, since) {
        return since !== null ? since : this.t !== null ? (t - this.t) / 1000 : Infinity;
    }
}

/**
 * Position: north, east, up. Over the ground the projection follows the
 * turn the acceleration describes — its part across the track is a turn
 * rate (ω = a⊥ / v), its part along the track a change of speed — so in a
 * turn the drawn vehicle stays on the arc instead of a straight ½·a·t²
 * parabola drifting outside it.
 */
class PositionChannel extends Channel {
    project(now) {
        const out = super.project(now);
        const vn = this.rate[0], ve = this.rate[1], speed = Math.hypot(vn, ve);
        if (speed < MIN_TURN_SPEED) return out;
        const dt = this.span(now), tn = vn / speed, te = ve / speed;
        const along = this.accel[0] * tn + this.accel[1] * te;
        const across = this.accel[1] * tn - this.accel[0] * te;   // + to the right
        const w = Math.max(-MAX_TURN, Math.min(MAX_TURN, across / speed));
        const angle = w * dt, travelled = speed * dt + 0.5 * along * dt * dt;
        // Distance along and to the side of the starting track, on the arc
        let ahead, side;
        if (Math.abs(angle) < 1e-6) { ahead = travelled; side = 0.5 * travelled * angle; }
        else { ahead = travelled * Math.sin(angle) / angle; side = travelled * (1 - Math.cos(angle)) / angle; }
        out[0] = ahead * tn - side * te;
        out[1] = ahead * te + side * tn;
        return out;
    }
}

export class PoseSmoother {
    constructor() {
        this.pose = { lat: 0, lon: 0, alt: 0, roll: 0, pitch: 0, yaw: 0 };
        this.reset();
    }

    /** Forget the samples: the next update draws STATE as it is. */
    reset() {
        this.pos = new PositionChannel();
        this.att = new Channel();     // roll, pitch, yaw (rad)
        this.raw = { lat: NaN, lon: NaN, alt: NaN, roll: NaN, pitch: NaN, yaw: NaN };
        this.velocity = null;         // NED + up of the last position sample, for the acceleration
    }

    /**
     * Pose to draw at time now.
     * @param {object} s    STATE (lat, lon, rawAlt, roll, pitch, yaw, vn, ve, vs, gs,
     *                      track, rollRate, pitchRate, yawRate, posTime, attTime,
     *                      posBootMs, attBootMs)
     * @param {number} now  Date.now() clock, the one posTime and attTime use
     * @param {number} tau  blend time constant (s); 0 draws STATE unchanged
     */
    update(s, now, tau) {
        const pose = this.pose;
        if (!(tau > 0)) {
            this.reset();
            pose.lat = s.lat; pose.lon = s.lon; pose.alt = s.rawAlt;
            pose.roll = s.roll; pose.pitch = s.pitch; pose.yaw = s.yaw;
            return pose;
        }
        const raw = this.raw;
        if (s.lat !== raw.lat || s.lon !== raw.lon || s.rawAlt !== raw.alt) this.takePosition(s, now, tau);
        if (s.roll !== raw.roll || s.pitch !== raw.pitch || s.yaw !== raw.yaw) this.takeAttitude(s, now, tau);

        const p = this.pos, kp = p.decay(now, tau), dp = p.project(now);
        const n = dp[0] + p.offset[0] * kp;
        const e = dp[1] + p.offset[1] * kp;
        const u = dp[2] + p.offset[2] * kp;
        pose.lat = raw.lat + n / M_PER_DEG;
        pose.lon = raw.lon + e / (M_PER_DEG * Math.max(0.01, Math.cos(raw.lat * Math.PI / 180)));
        pose.alt = raw.alt + u;

        const a = this.att, ka = a.decay(now, tau), da = a.project(now);
        pose.roll = raw.roll + da[0] + a.offset[0] * ka;
        pose.pitch = raw.pitch + da[1] + a.offset[1] * ka;
        const yaw = (raw.yaw + da[2] + a.offset[2] * ka) % TWO_PI;
        pose.yaw = yaw < 0 ? yaw + TWO_PI : yaw;
        return pose;
    }

    takePosition(s, now, tau) {
        const p = this.pos, raw = this.raw;
        const { t, since } = p.time(s.posTime, s.posBootMs, now);
        // Where the old sample puts the drawn vehicle at t, in metres from the new one
        let dn = 0, de = 0, du = 0, snapped = true;
        if (p.t !== null && Number.isFinite(raw.lat)) {
            const k = p.decay(t, tau), d = p.project(t);
            const mPerLon = M_PER_DEG * Math.max(0.01, Math.cos(s.lat * Math.PI / 180));
            dn = (raw.lat - s.lat) * M_PER_DEG + d[0] + p.offset[0] * k;
            de = (raw.lon - s.lon) * mPerLon + d[1] + p.offset[1] * k;
            du = (raw.alt - s.rawAlt) + d[2] + p.offset[2] * k;
            snapped = !(Math.hypot(dn, de, du) < SNAP_M);
            if (snapped) dn = de = du = 0;
        }
        // Velocity: the EKF's NED vector, or ground speed along the track
        // when only GPS_RAW_INT reports (it fills gs and track, not vn/ve)
        let vn = finiteOr(s.vn, 0), ve = finiteOr(s.ve, 0);
        if (vn === 0 && ve === 0 && s.gs > 0.5 && Number.isFinite(s.track)) {
            vn = s.gs * Math.cos(s.track); ve = s.gs * Math.sin(s.track);
        }
        const vu = finiteOr(s.vs, 0);
        // Acceleration: change of velocity since the previous sample (samples
        // landing together in a burst, without a vehicle clock, give none)
        const dt = p.gap(t, since);
        if (snapped || !this.velocity || dt > MAX_DIFF_S) p.accel.fill(0);
        else if (dt >= MIN_DIFF_S) {
            const measured = [(vn - this.velocity[0]) / dt, (ve - this.velocity[1]) / dt, (vu - this.velocity[2]) / dt];
            clampNorm(measured, MAX_ACCEL);
            for (let i = 0; i < 3; i++) p.accel[i] += (measured[i] - p.accel[i]) * DERIV_BLEND;
        }
        this.velocity = [vn, ve, vu];

        p.t = t;
        p.offset[0] = dn; p.offset[1] = de; p.offset[2] = du;
        p.rate[0] = vn; p.rate[1] = ve; p.rate[2] = vu;
        raw.lat = s.lat; raw.lon = s.lon; raw.alt = s.rawAlt;
    }

    takeAttitude(s, now, tau) {
        const a = this.att, raw = this.raw;
        const { t, since } = a.time(s.attTime, s.attBootMs, now);
        let dr = 0, dp = 0, dy = 0, snapped = true;
        if (a.t !== null && Number.isFinite(raw.roll)) {
            const k = a.decay(t, tau), d = a.project(t);
            dr = wrapPi(raw.roll + d[0] + a.offset[0] * k - s.roll);
            dp = wrapPi(raw.pitch + d[1] + a.offset[1] * k - s.pitch);
            dy = wrapPi(raw.yaw + d[2] + a.offset[2] * k - s.yaw);
            snapped = !(Math.abs(dr) < SNAP_RAD && Math.abs(dp) < SNAP_RAD && Math.abs(dy) < SNAP_RAD);
            if (snapped) dr = dp = dy = 0;
        }
        const p = finiteOr(s.rollRate, 0), q = finiteOr(s.pitchRate, 0), r = finiteOr(s.yawRate, 0);
        if (p !== 0 || q !== 0 || r !== 0) {
            // Body rates (p, q, r) to Euler angle rates
            const sr = Math.sin(s.roll), cr = Math.cos(s.roll), cp = Math.cos(s.pitch);
            const qr = q * sr + r * cr;
            a.rate[0] = p + Math.tan(s.pitch) * qr;
            a.rate[1] = q * cr - r * sr;
            a.rate[2] = qr / (Math.abs(cp) < 0.05 ? Math.sign(cp || 1) * 0.05 : cp);
        } else {
            // No rates from this source: the change since the previous sample
            const dt = a.gap(t, since);
            if (snapped || !Number.isFinite(raw.roll) || dt > MAX_DIFF_S) a.rate.fill(0);
            else if (dt >= MIN_DIFF_S) {
                const seen = [wrapPi(s.roll - raw.roll) / dt, wrapPi(s.pitch - raw.pitch) / dt, wrapPi(s.yaw - raw.yaw) / dt];
                for (let i = 0; i < 3; i++) a.rate[i] += (seen[i] - a.rate[i]) * DERIV_BLEND;
            }
        }
        for (let i = 0; i < 3; i++) {
            a.rate[i] = Number.isFinite(a.rate[i]) ? Math.max(-MAX_RATE, Math.min(MAX_RATE, a.rate[i])) : 0;
        }
        a.t = t;
        a.offset[0] = dr; a.offset[1] = dp; a.offset[2] = dy;
        raw.roll = s.roll; raw.pitch = s.pitch; raw.yaw = s.yaw;
    }
}

/** The app's smoother: main.js updates it once per rendered frame, the HUD and the 3D layers draw its pose. */
export const vehiclePose = new PoseSmoother();
