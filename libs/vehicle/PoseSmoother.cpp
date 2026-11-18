#include "vehicle/PoseSmoother.h"

#include "core/Geo.h"
#include "vehicle/VehicleState.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace corv {

namespace {
constexpr double kMPerDeg = 111320;
constexpr double kTwoPi = 2 * kPi;
constexpr double kHorizonS = 0.5;
constexpr std::size_t kClockHistory = 64;
constexpr double kMinDiffS = 0.005;
constexpr double kMaxDiffS = 1.0;
constexpr double kMaxAccel = 30;
constexpr double kMaxRate = 6;
constexpr double kMaxTurn = 1;
constexpr double kMinTurnSpeed = 3;
constexpr double kDerivBlend = 0.5;
constexpr double kSnapM = 100;
constexpr double kSnapRad = kPi / 3;
const double kNaN = std::numeric_limits<double>::quiet_NaN();

double finiteOr(double v, double f) { return std::isfinite(v) ? v : f; }

void clampNorm(std::array<double, 3> &v, double max)
{
    const double n = std::hypot(v[0], v[1], v[2]);
    if (n > max)
        for (double &x : v) x *= max / n;
}
} // namespace

PoseSmoother::Sample PoseSmoother::fromState(const VehicleState &s)
{
    Sample o;
    o.lat = s.lat;
    o.lon = s.lon;
    o.rawAlt = s.rawAlt;
    o.roll = s.roll;
    o.pitch = s.pitch;
    o.yaw = s.yaw;
    o.vn = s.vn;
    o.ve = s.ve;
    o.vs = s.vs;
    o.gs = s.gs;
    o.track = s.track;
    o.rollRate = s.rollRate;
    o.pitchRate = s.pitchRate;
    o.yawRate = s.yawRate;
    o.posTime = double(s.posTime);
    o.attTime = double(s.attTime);
    if (s.posBootMs) o.posBootMs = double(*s.posBootMs);
    if (s.attBootMs) o.attBootMs = double(*s.attBootMs);
    return o;
}

double PoseSmoother::Channel::span(double now) const
{
    return std::min(std::max(0.0, now - *t) / 1000.0, kHorizonS);
}

std::array<double, 3> &PoseSmoother::Channel::projectLinear(double now)
{
    const double dt = span(now);
    for (int i = 0; i < 3; ++i) projection[i] = rate[i] * dt + 0.5 * accel[i] * dt * dt;
    return projection;
}

double PoseSmoother::Channel::decay(double now, double tau) const
{
    return tau > 0 ? std::exp(-std::max(0.0, now - *t) / 1000.0 / tau) : 0.0;
}

PoseSmoother::Channel::Timed PoseSmoother::Channel::time(double arrival, std::optional<double> bootIn, double now)
{
    const bool fresh = std::isfinite(arrival) && arrival > 0 && (!stamp || arrival != *stamp) && arrival <= now && now - arrival < 1000;
    stamp = arrival;
    const double at = fresh ? arrival : now;
    // The vehicle clock dates the sample only when it came with it
    if (!fresh || !bootIn || (boot && *bootIn == *boot)) {
        lags.clear();
        boot.reset();
        return {at, std::nullopt};
    }
    std::optional<double> since;
    if (boot) {
        since = (*bootIn - *boot) / 1000.0;
        // The vehicle rebooted or another one took over: start the clock map again
        if (!(*since >= 0 && *since < 5)) {
            lags.clear();
            since.reset();
        }
    }
    boot = bootIn;
    lags.push_back(at - *bootIn);
    if (lags.size() > kClockHistory) lags.pop_front();
    // Quickest arrival = least delayed: that delay is the link's own, the rest is jitter
    const double minLag = *std::min_element(lags.begin(), lags.end());
    return {std::min(at, *bootIn + minLag), since};
}

double PoseSmoother::Channel::gap(double tt, std::optional<double> since) const
{
    if (since) return *since;
    if (t) return (tt - *t) / 1000.0;
    return std::numeric_limits<double>::infinity();
}

void PoseSmoother::reset()
{
    m_pos = Channel{};
    m_att = Channel{};
    m_raw = {kNaN, kNaN, kNaN, kNaN, kNaN, kNaN};
    m_velocity.reset();
}

std::array<double, 3> &PoseSmoother::projectPosition(double now)
{
    auto &out = m_pos.projectLinear(now);
    const double vn = m_pos.rate[0], ve = m_pos.rate[1], speed = std::hypot(vn, ve);
    if (speed < kMinTurnSpeed) return out;
    const double dt = m_pos.span(now), tn = vn / speed, te = ve / speed;
    const double along = m_pos.accel[0] * tn + m_pos.accel[1] * te;
    const double across = m_pos.accel[1] * tn - m_pos.accel[0] * te;   // + to the right
    const double w = std::clamp(across / speed, -kMaxTurn, kMaxTurn);
    const double angle = w * dt, travelled = speed * dt + 0.5 * along * dt * dt;
    double ahead, side;
    if (std::abs(angle) < 1e-6) {
        ahead = travelled;
        side = 0.5 * travelled * angle;
    } else {
        ahead = travelled * std::sin(angle) / angle;
        side = travelled * (1 - std::cos(angle)) / angle;
    }
    out[0] = ahead * tn - side * te;
    out[1] = ahead * te + side * tn;
    return out;
}

const Pose &PoseSmoother::update(const Sample &s, double now, double tau)
{
    if (!(tau > 0)) {
        reset();
        m_pose = {s.lat, s.lon, s.rawAlt, s.roll, s.pitch, s.yaw};
        return m_pose;
    }
    if (s.lat != m_raw.lat || s.lon != m_raw.lon || s.rawAlt != m_raw.alt) takePosition(s, now, tau);
    if (s.roll != m_raw.roll || s.pitch != m_raw.pitch || s.yaw != m_raw.yaw) takeAttitude(s, now, tau);

    const double kp = m_pos.decay(now, tau);
    const auto dp = projectPosition(now);
    const double n = dp[0] + m_pos.offset[0] * kp;
    const double e = dp[1] + m_pos.offset[1] * kp;
    const double u = dp[2] + m_pos.offset[2] * kp;
    m_pose.lat = m_raw.lat + n / kMPerDeg;
    m_pose.lon = m_raw.lon + e / (kMPerDeg * std::max(0.01, std::cos(m_raw.lat * kDeg)));
    m_pose.alt = m_raw.alt + u;

    const double ka = m_att.decay(now, tau);
    const auto da = m_att.projectLinear(now);
    m_pose.roll = m_raw.roll + da[0] + m_att.offset[0] * ka;
    m_pose.pitch = m_raw.pitch + da[1] + m_att.offset[1] * ka;
    const double yaw = std::fmod(m_raw.yaw + da[2] + m_att.offset[2] * ka, kTwoPi);
    m_pose.yaw = yaw < 0 ? yaw + kTwoPi : yaw;
    return m_pose;
}

void PoseSmoother::takePosition(const Sample &s, double now, double tau)
{
    Channel &p = m_pos;
    const auto timed = p.time(s.posTime, s.posBootMs, now);
    double dn = 0, de = 0, du = 0;
    bool snapped = true;
    if (p.t && std::isfinite(m_raw.lat)) {
        const double k = p.decay(timed.t, tau);
        const auto d = projectPosition(timed.t);
        const double mPerLon = kMPerDeg * std::max(0.01, std::cos(s.lat * kDeg));
        dn = (m_raw.lat - s.lat) * kMPerDeg + d[0] + p.offset[0] * k;
        de = (m_raw.lon - s.lon) * mPerLon + d[1] + p.offset[1] * k;
        du = (m_raw.alt - s.rawAlt) + d[2] + p.offset[2] * k;
        snapped = !(std::hypot(dn, de, du) < kSnapM);
        if (snapped) dn = de = du = 0;
    }
    // Velocity: the EKF's NED vector, or ground speed along the track when only GPS_RAW_INT reports
    double vn = finiteOr(s.vn, 0), ve = finiteOr(s.ve, 0);
    if (vn == 0 && ve == 0 && s.gs > 0.5 && std::isfinite(s.track)) {
        vn = s.gs * std::cos(s.track);
        ve = s.gs * std::sin(s.track);
    }
    const double vu = finiteOr(s.vs, 0);
    const double dt = p.gap(timed.t, timed.since);
    if (snapped || !m_velocity || dt > kMaxDiffS) {
        p.accel.fill(0);
    } else if (dt >= kMinDiffS) {
        std::array<double, 3> measured{(vn - (*m_velocity)[0]) / dt, (ve - (*m_velocity)[1]) / dt, (vu - (*m_velocity)[2]) / dt};
        clampNorm(measured, kMaxAccel);
        for (int i = 0; i < 3; ++i) p.accel[i] += (measured[i] - p.accel[i]) * kDerivBlend;
    }
    m_velocity = std::array<double, 3>{vn, ve, vu};
    p.t = timed.t;
    p.offset = {dn, de, du};
    p.rate = {vn, ve, vu};
    m_raw.lat = s.lat;
    m_raw.lon = s.lon;
    m_raw.alt = s.rawAlt;
}

void PoseSmoother::takeAttitude(const Sample &s, double now, double tau)
{
    Channel &a = m_att;
    const auto timed = a.time(s.attTime, s.attBootMs, now);
    double dr = 0, dp = 0, dy = 0;
    bool snapped = true;
    if (a.t && std::isfinite(m_raw.roll)) {
        const double k = a.decay(timed.t, tau);
        const auto d = a.projectLinear(timed.t);
        dr = wrapPi(m_raw.roll + d[0] + a.offset[0] * k - s.roll);
        dp = wrapPi(m_raw.pitch + d[1] + a.offset[1] * k - s.pitch);
        dy = wrapPi(m_raw.yaw + d[2] + a.offset[2] * k - s.yaw);
        snapped = !(std::abs(dr) < kSnapRad && std::abs(dp) < kSnapRad && std::abs(dy) < kSnapRad);
        if (snapped) dr = dp = dy = 0;
    }
    const double p = finiteOr(s.rollRate, 0), q = finiteOr(s.pitchRate, 0), r = finiteOr(s.yawRate, 0);
    if (p != 0 || q != 0 || r != 0) {
        // Body rates (p, q, r) to Euler angle rates
        const double sr = std::sin(s.roll), cr = std::cos(s.roll), cp = std::cos(s.pitch);
        const double qr = q * sr + r * cr;
        a.rate[0] = p + std::tan(s.pitch) * qr;
        a.rate[1] = q * cr - r * sr;
        a.rate[2] = qr / (std::abs(cp) < 0.05 ? (cp < 0 ? -0.05 : 0.05) : cp);
    } else {
        // No rates from this source: the change since the previous sample
        const double dt = a.gap(timed.t, timed.since);
        if (snapped || !std::isfinite(m_raw.roll) || dt > kMaxDiffS) {
            a.rate.fill(0);
        } else if (dt >= kMinDiffS) {
            const std::array<double, 3> seen{wrapPi(s.roll - m_raw.roll) / dt, wrapPi(s.pitch - m_raw.pitch) / dt,
                                             wrapPi(s.yaw - m_raw.yaw) / dt};
            for (int i = 0; i < 3; ++i) a.rate[i] += (seen[i] - a.rate[i]) * kDerivBlend;
        }
    }
    for (double &v : a.rate) v = std::isfinite(v) ? std::clamp(v, -kMaxRate, kMaxRate) : 0.0;
    a.t = timed.t;
    a.offset = {dr, dp, dy};
    m_raw.roll = s.roll;
    m_raw.pitch = s.pitch;
    m_raw.yaw = s.yaw;
}

} // namespace corv
