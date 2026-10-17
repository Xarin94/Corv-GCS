/**
 * EKF STATUS data: the last 20 s of the estimator's own health figures, kept
 * per source because an ArduPilot board and a CORV board report different
 * things.
 *
 * ArduPilot (EKF3), over MAVLink:
 *  - EKF_STATUS_REPORT (193): innovation test ratios of velocity, horizontal
 *    and vertical position, compass and airspeed — how far each measurement
 *    lands from the prediction, scaled by its gate (1 = rejected) — and the
 *    flags of the solutions the EKF trusts;
 *  - GPS_RAW_INT (24): the receiver's horizontal, vertical and speed accuracy;
 *  - AHRS2 (178) against ATTITUDE (30): the DCM backup estimator against the
 *    EKF, which agree within a degree or two while the attitude is sound;
 *  - VIBRATION (241): what spoils the accelerometers the EKF predicts with.
 *
 * CORV-2 (RBPF: the particles carry the attitude, each with its own EKF for
 * velocity and position, and one shared EKF holds the sensor biases):
 *  - Navigation (0x01, via 'corv-nav-status'): confidence, trace of the shared
 *    EKF covariance, status flags, the solution with the board clock;
 *  - Debug (0x02, 1 Hz): particle cloud spread, effective sample size, active
 *    particles, resamples, gyro biases, magnetometer quality, GPS accuracy;
 *  - Raw Sensor (0x03, 5 Hz): the raw GPS fix, set against the filtered
 *    position at the same board time: the GPS residual.
 *
 * No DOM: times are passed in (performance.now() in the app).
 */

export const EKF_WINDOW_MS = 20000;
const KEEP_MS = EKF_WINDOW_MS + 2000;
const STALE_MS = 3000;            // a source silent this long is no longer shown as live
const NAV_HISTORY_MS = 3000;      // filtered positions kept to meet the raw GPS fixes
const AHRS2_WAIT_MS = 250;        // longest wait for the ATTITUDE after an AHRS2
const M_PER_DEG = 111320;
const DEG = 180 / Math.PI;

/** EKF_STATUS_FLAGS (MAVLink common). */
export const AP_EKF_FLAGS = {
    ATTITUDE: 1, VELOCITY_HORIZ: 2, VELOCITY_VERT: 4, POS_HORIZ_REL: 8, POS_HORIZ_ABS: 16,
    POS_VERT_ABS: 32, POS_VERT_AGL: 64, CONST_POS_MODE: 128, PRED_POS_HORIZ_REL: 256,
    PRED_POS_HORIZ_ABS: 512, UNINITIALIZED: 1024, GPS_GLITCHING: 32768
};

/** Navigation packet status flags (BINARY_TELEMETRY.md). */
export const CORV_FLAGS = {
    CONVERGED: 1, INITIALIZED: 2, ZUPT_ACTIVE: 4, GPS_AVAILABLE: 8, IMU1_HEALTHY: 16,
    IMU2_HEALTHY: 32, BARO_HEALTHY: 64, MAG_HEALTHY: 128, GPS_BYPASS: 256,
    GPS_JAMMING: 2048, GPS_SPOOFING: 4096
};
export const CORV_AIDING = ['GPS+MAG', 'GPS', 'MAG', 'DEAD RECKONING'];

/** Time series trimmed to the chart window (plus a margin for the left edge). */
export class Series {
    constructor() { this.t = []; this.v = []; }
    push(t, v) {
        if (!Number.isFinite(v)) return;
        this.t.push(t); this.v.push(v);
        const cut = t - KEEP_MS;
        let i = 0;
        while (i < this.t.length && this.t[i] < cut) i++;
        if (i) { this.t.splice(0, i); this.v.splice(0, i); }
    }
    get length() { return this.t.length; }
    last() { return this.v.length ? this.v[this.v.length - 1] : null; }
    lastTime() { return this.t.length ? this.t[this.t.length - 1] : null; }
    /** Value in force at time t (the sample at or before it), or null. */
    at(t) {
        let lo = 0, hi = this.t.length - 1, best = -1;
        while (lo <= hi) { const mid = (lo + hi) >> 1; if (this.t[mid] <= t) { best = mid; lo = mid + 1; } else hi = mid - 1; }
        return best < 0 ? null : this.v[best];
    }
}

function view(bytes) {
    const u8 = bytes instanceof Uint8Array ? bytes : Uint8Array.from(bytes);
    return new DataView(u8.buffer, u8.byteOffset, u8.byteLength);
}

function wrapDeg(a) {
    return ((a + 540) % 360) - 180;
}

export class EkfMonitor {
    constructor() { this.reset(); }

    reset() {
        this.series = new Map();
        this.ap = { time: -Infinity, flags: null, terr: null, tas: null, fix: null, sats: null, hdop: null, vibe: null, clip: null };
        this.corv = { time: -Infinity, flags: null, fix: null, sats: null, hdop: null, n: null, ess: null, resamples: null,
            resampleRate: null, gyroBias: null, accelBias: null, baroBias: null, magQuality: null, trP: null };
        this.lastSource = null;
        this.attitude = [];        // last two ATTITUDE samples {t, r, p, y} (deg), arrival time
        this.pendingAhrs2 = null;
        this.nav = [];             // CORV filtered positions {ts, lat, lon, alt}, board clock
        this.lastResampleCount = null;
        this.lastDebugTime = null;
    }

    /** The series of that name, created on first use. */
    get(name) {
        let s = this.series.get(name);
        if (!s) { s = new Series(); this.series.set(name, s); }
        return s;
    }

    /** 'corv' or 'ardupilot' while one reports, else the last one that did (or null). */
    source(now) {
        if (now - this.corv.time < STALE_MS) return 'corv';
        if (now - this.ap.time < STALE_MS) return 'ardupilot';
        return this.lastSource;
    }

    /** True while the source on show is still reporting. */
    live(now) {
        const s = this.source(now);
        return s === 'corv' ? now - this.corv.time < STALE_MS : s === 'ardupilot' ? now - this.ap.time < STALE_MS : false;
    }

    // ── ArduPilot ────────────────────────────────────────────────────────

    ingestMavlink(msgId, d, now) {
        if (!d) return;
        switch (msgId) {
            case 193: {   // EKF_STATUS_REPORT
                if (!Number.isFinite(d.flags)) return;
                this.ap.time = now; this.lastSource = 'ardupilot';
                this.ap.flags = d.flags;
                this.get('apVel').push(now, d.velocityVariance);
                this.get('apPosH').push(now, d.posHorizVariance);
                this.get('apPosV').push(now, d.posVertVariance);
                this.get('apMag').push(now, d.compassVariance);
                this.ap.terr = Number.isFinite(d.terrainAltVariance) ? d.terrainAltVariance : null;
                // No pitot, no airspeed fusion: the ratio stays 0 and is not drawn
                if (Number.isFinite(d.airspeedVariance) && (d.airspeedVariance > 0 || this.get('apTas').length)) {
                    this.get('apTas').push(now, d.airspeedVariance);
                }
                break;
            }
            case 24: {    // GPS_RAW_INT: accuracies in mm and mm/s, 0 = not reported
                if (Number.isFinite(d.fixType)) this.ap.fix = d.fixType;
                if (Number.isFinite(d.satellitesVisible) && d.satellitesVisible !== 255) this.ap.sats = d.satellitesVisible;
                if (Number.isFinite(d.eph) && d.eph !== 65535) { this.ap.hdop = d.eph / 100; this.get('apHdop').push(now, d.eph / 100); }
                // Without a fix the receiver may still quote its last accuracy: not plotted
                if (!(d.fixType >= 2)) break;
                if (d.hAcc > 0) this.get('apHAcc').push(now, d.hAcc / 1000);
                if (d.vAcc > 0) this.get('apVAcc').push(now, d.vAcc / 1000);
                if (d.velAcc > 0) this.get('apVelAcc').push(now, d.velAcc / 1000);
                break;
            }
            case 30:      // ATTITUDE (the EKF's)
                if (!Number.isFinite(d.roll)) return;
                this.attitude.push({ t: now, r: d.roll * DEG, p: d.pitch * DEG, y: d.yaw * DEG });
                if (this.attitude.length > 2) this.attitude.shift();
                this.resolveAhrs2(now, false);
                break;
            case 178:     // AHRS2 (DCM): compared once the ATTITUDE after it is in
                if (!Number.isFinite(d.roll)) return;
                this.ap.time = now; this.lastSource = 'ardupilot';
                this.resolveAhrs2(now, true);
                this.pendingAhrs2 = { t: now, r: d.roll * DEG, p: d.pitch * DEG, y: d.yaw * DEG };
                this.resolveAhrs2(now, false);
                break;
            case 241:     // VIBRATION
                if (!Number.isFinite(d.vibrationX)) return;
                this.ap.vibe = [d.vibrationX, d.vibrationY, d.vibrationZ];
                this.ap.clip = (d.clipping0 || 0) + (d.clipping1 || 0) + (d.clipping2 || 0);
                break;
        }
    }

    /**
     * EKF − DCM at the AHRS2's arrival, the EKF attitude interpolated between
     * the ATTITUDE samples around it: at 25 Hz the nearest one alone would be up
     * to 20 ms off, 1.2° in a 60°/s roll.
     */
    resolveAhrs2(now, force) {
        const a = this.pendingAhrs2;
        if (!a || !this.attitude.length) return;
        const [p0, p1] = this.attitude.length === 2 ? this.attitude : [null, this.attitude[0]];
        let e;
        if (p0 && p0.t <= a.t && p1.t >= a.t && p1.t > p0.t) {
            const k = (a.t - p0.t) / (p1.t - p0.t);
            e = { r: p0.r + (p1.r - p0.r) * k, p: p0.p + (p1.p - p0.p) * k, y: p0.y + wrapDeg(p1.y - p0.y) * k };
        } else if (p1.t >= a.t || force || now - a.t > AHRS2_WAIT_MS) {
            e = p1;   // the ATTITUDE stream is late or gone: the latest one will do
        } else return;
        this.get('apDRoll').push(a.t, e.r - a.r);
        this.get('apDPitch').push(a.t, e.p - a.p);
        this.get('apDYaw').push(a.t, wrapDeg(e.y - a.y));
        this.pendingAhrs2 = null;
    }

    // ── CORV-2 ───────────────────────────────────────────────────────────

    ingestCorvNav(n, now) {
        if (!n || !Number.isFinite(n.confidence)) return;
        this.corv.time = now; this.lastSource = 'corv';
        this.corv.flags = n.flags;
        this.corv.fix = n.fixType; this.corv.sats = n.numSat;
        this.corv.hdop = Number.isFinite(n.hdop) ? n.hdop : null;
        this.corv.trP = n.covTrace;
        this.get('corvConf').push(now, n.confidence * 100);
        if (n.covTrace > 0) this.get('corvTrP').push(now, n.covTrace);
        if (Number.isFinite(n.ts) && Number.isFinite(n.lat) && Number.isFinite(n.lon)) {
            this.nav.push({ ts: n.ts, lat: n.lat, lon: n.lon, alt: n.alt });
            while (this.nav.length && this.nav[0].ts < n.ts - NAV_HISTORY_MS) this.nav.shift();
            // The board rebooted: its clock went back
            if (this.nav.length > 1 && this.nav[this.nav.length - 2].ts > n.ts) this.nav = [this.nav[this.nav.length - 1]];
        }
    }

    ingestCorvDebug(bytes, now) {
        if (!bytes || bytes.length < 92) return;
        const v = view(bytes);
        const f = o => v.getFloat32(o, true);
        this.corv.gyroBias = [f(4), f(8), f(12)];           // deg/s
        this.corv.accelBias = [f(16), f(20), f(24)];        // m/s²
        this.corv.baroBias = f(28);
        this.corv.magQuality = f(32);
        const vAcc = f(60), ess = f(76), spread = f(80);
        const resamples = v.getUint16(84, true), n = v.getUint16(86, true);
        this.corv.n = n; this.corv.ess = ess;
        if (vAcc > 0 && this.corv.fix >= 2) this.get('corvGpsV').push(now, vAcc);
        if (spread >= 0) this.get('corvSpread').push(now, spread);
        if (n > 0 && ess >= 0) this.get('corvEss').push(now, Math.min(100, ess / n * 100));
        // Resamples since the last debug packet (the counter wraps at 65535)
        if (this.lastResampleCount !== null) {
            const count = (resamples - this.lastResampleCount + 65536) % 65536;
            const dt = (now - this.lastDebugTime) / 1000;
            this.corv.resampleRate = dt > 0 ? count / dt : null;
            if (count > 0) this.get('corvResample').push(now, count);
        }
        this.lastResampleCount = resamples; this.lastDebugTime = now;
    }

    ingestCorvRaw(bytes, now) {
        if (!bytes || bytes.length < 74) return;
        const v = view(bytes);
        const ts = v.getUint32(0, true), lat = v.getFloat64(4, true), lon = v.getFloat64(12, true), alt = v.getFloat32(20, true);
        const fix = v.getUint8(36), hAcc = v.getFloat32(38, true);
        if (fix < 2 || !lat || !lon) return;
        if (hAcc > 0) this.get('corvGpsH').push(now, hAcc);
        const filtered = this.filteredAt(ts);
        if (!filtered) return;
        const mPerLon = M_PER_DEG * Math.cos(lat * Math.PI / 180);
        this.get('corvResH').push(now, Math.hypot((lat - filtered.lat) * M_PER_DEG, (lon - filtered.lon) * mPerLon));
        if (Number.isFinite(alt) && Number.isFinite(filtered.alt)) this.get('corvResV').push(now, Math.abs(alt - filtered.alt));
    }

    /** Filtered position at board time ts, between the navigation samples around it. */
    filteredAt(ts) {
        const nav = this.nav;
        for (let i = nav.length - 1; i > 0; i--) {
            const a = nav[i - 1], b = nav[i];
            if (a.ts <= ts && b.ts >= ts) {
                const k = b.ts > a.ts ? (ts - a.ts) / (b.ts - a.ts) : 0;
                return { lat: a.lat + (b.lat - a.lat) * k, lon: a.lon + (b.lon - a.lon) * k, alt: a.alt + (b.alt - a.alt) * k };
            }
        }
        // Just past the newest solution (the raw packet overtook it): close enough within 50 ms
        const last = nav[nav.length - 1];
        return last && ts >= last.ts && ts - last.ts <= 50 ? last : null;
    }
}
