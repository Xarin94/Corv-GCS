#include "vehicle/EkfMonitor.h"

#include <algorithm>
#include <cmath>
#include <cstring>

using namespace Qt::StringLiterals;

namespace corv {

namespace {
constexpr double kKeepMs = kEkfWindowMs + 2000;
constexpr double kStaleMs = 3000;           // a source silent this long is no longer shown as live
constexpr double kAhrs2WaitMs = 250;        // longest wait for the ATTITUDE after an AHRS2
constexpr double kDeg = 180 / M_PI;

double wrapDeg(double a) { return std::fmod(std::fmod(a + 540, 360) + 360, 360) - 180; }

} // namespace

void EkfSeries::push(double t, double v)
{
    if (!std::isfinite(v)) return;
    m_t.push_back(t);
    m_v.push_back(v);
    const double cut = t - kKeepMs;
    while (!m_t.empty() && m_t.front() < cut) {
        m_t.pop_front();
        m_v.pop_front();
    }
}

std::optional<double> EkfSeries::at(double t) const
{
    int lo = 0, hi = int(m_t.size()) - 1, best = -1;
    while (lo <= hi) {
        const int mid = (lo + hi) >> 1;
        if (m_t[std::size_t(mid)] <= t) {
            best = mid;
            lo = mid + 1;
        } else {
            hi = mid - 1;
        }
    }
    return best < 0 ? std::nullopt : std::optional<double>(m_v[std::size_t(best)]);
}

void EkfMonitor::reset()
{
    m_series.clear();
    m_ap = ArduPilot{};
    m_lastSource.clear();
    m_attitude.clear();
    m_pendingAhrs2.reset();
}

const EkfSeries &EkfMonitor::series(const QString &name) const
{
    static const EkfSeries empty;
    auto it = m_series.constFind(name);
    return it == m_series.constEnd() ? empty : *it;
}

QString EkfMonitor::source(double now) const
{
    if (now - m_ap.time < kStaleMs) return u"ardupilot"_s;
    return m_lastSource;
}

bool EkfMonitor::live(double now) const
{
    const QString s = source(now);
    if (s == "ardupilot"_L1) return now - m_ap.time < kStaleMs;
    return false;
}

// ── ArduPilot ────────────────────────────────────────────────────────────

void EkfMonitor::ingestMavlink(const mavlink_message_t &msg, double now)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_EKF_STATUS_REPORT: {
        mavlink_ekf_status_report_t d;
        mavlink_msg_ekf_status_report_decode(&msg, &d);
        m_ap.time = now;
        m_lastSource = u"ardupilot"_s;
        m_ap.flags = int(d.flags);
        get(u"apVel"_s).push(now, d.velocity_variance);
        get(u"apPosH"_s).push(now, d.pos_horiz_variance);
        get(u"apPosV"_s).push(now, d.pos_vert_variance);
        get(u"apMag"_s).push(now, d.compass_variance);
        m_ap.terr = std::isfinite(d.terrain_alt_variance) ? std::optional<double>(double(d.terrain_alt_variance)) : std::nullopt;
        // No pitot, no airspeed fusion: the ratio stays 0 and is not drawn
        if (std::isfinite(d.airspeed_variance) && (d.airspeed_variance > 0 || !get(u"apTas"_s).empty()))
            get(u"apTas"_s).push(now, d.airspeed_variance);
        break;
    }
    case MAVLINK_MSG_ID_GPS_RAW_INT: {   // accuracies in mm and mm/s, 0 = not reported
        mavlink_gps_raw_int_t d;
        mavlink_msg_gps_raw_int_decode(&msg, &d);
        m_ap.fix = int(d.fix_type);
        if (d.satellites_visible != 255) m_ap.sats = int(d.satellites_visible);
        if (d.eph != 65535) {
            m_ap.hdop = d.eph / 100.0;
            get(u"apHdop"_s).push(now, d.eph / 100.0);
        }
        // Without a fix the receiver may still quote its last accuracy: not plotted
        if (d.fix_type < 2) break;
        if (d.h_acc > 0) get(u"apHAcc"_s).push(now, d.h_acc / 1000.0);
        if (d.v_acc > 0) get(u"apVAcc"_s).push(now, d.v_acc / 1000.0);
        if (d.vel_acc > 0) get(u"apVelAcc"_s).push(now, d.vel_acc / 1000.0);
        break;
    }
    case MAVLINK_MSG_ID_ATTITUDE: {   // the EKF's
        mavlink_attitude_t d;
        mavlink_msg_attitude_decode(&msg, &d);
        if (!std::isfinite(d.roll)) return;
        m_attitude.push_back({now, d.roll * kDeg, d.pitch * kDeg, d.yaw * kDeg});
        if (m_attitude.size() > 2) m_attitude.erase(m_attitude.begin());
        resolveAhrs2(now, false);
        break;
    }
    case MAVLINK_MSG_ID_AHRS2: {      // DCM: compared once the ATTITUDE after it is in
        mavlink_ahrs2_t d;
        mavlink_msg_ahrs2_decode(&msg, &d);
        if (!std::isfinite(d.roll)) return;
        m_ap.time = now;
        m_lastSource = u"ardupilot"_s;
        resolveAhrs2(now, true);
        m_pendingAhrs2 = Att{now, d.roll * kDeg, d.pitch * kDeg, d.yaw * kDeg};
        resolveAhrs2(now, false);
        break;
    }
    case MAVLINK_MSG_ID_VIBRATION: {
        mavlink_vibration_t d;
        mavlink_msg_vibration_decode(&msg, &d);
        if (!std::isfinite(d.vibration_x)) return;
        m_ap.vibe = std::array<double, 3>{d.vibration_x, d.vibration_y, d.vibration_z};
        m_ap.clip = int(d.clipping_0 + d.clipping_1 + d.clipping_2);
        break;
    }
    default:
        break;
    }
}

// EKF − DCM at the AHRS2's arrival, the EKF attitude interpolated between the
// ATTITUDE samples around it: at 25 Hz the nearest one alone would be up to
// 20 ms off, 1.2° in a 60°/s roll.
void EkfMonitor::resolveAhrs2(double now, bool force)
{
    if (!m_pendingAhrs2 || m_attitude.empty()) return;
    const Att a = *m_pendingAhrs2;
    const Att *p0 = m_attitude.size() == 2 ? &m_attitude[0] : nullptr;
    const Att &p1 = m_attitude.back();
    Att e;
    if (p0 && p0->t <= a.t && p1.t >= a.t && p1.t > p0->t) {
        const double k = (a.t - p0->t) / (p1.t - p0->t);
        e = {a.t, p0->r + (p1.r - p0->r) * k, p0->p + (p1.p - p0->p) * k, p0->y + wrapDeg(p1.y - p0->y) * k};
    } else if (p1.t >= a.t || force || now - a.t > kAhrs2WaitMs) {
        e = p1;   // the ATTITUDE stream is late or gone: the latest one will do
    } else {
        return;
    }
    get(u"apDRoll"_s).push(a.t, e.r - a.r);
    get(u"apDPitch"_s).push(a.t, e.p - a.p);
    get(u"apDYaw"_s).push(a.t, wrapDeg(e.y - a.y));
    m_pendingAhrs2.reset();
}

} // namespace corv
