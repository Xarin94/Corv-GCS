#include "vehicle/RelativeNav.h"

#include "vehicle/VehicleState.h"

#include <cmath>
#include <set>

namespace corv {

namespace {
const std::set<int> kFixedWing = {1, 16, 19, 20, 21, 22, 23, 24, 25};
constexpr int kSubmarine = 12;
constexpr std::int64_t kFreshMs = 2000;
constexpr double kMaxStepS = 0.5;
constexpr double kStandardHpa = 1013.25;
constexpr double kSurfaceMaxHpa = 1100;

template <typename T>
bool fresh(const std::optional<T> &o, std::int64_t now) { return o && now - o->t < kFreshMs; }
} // namespace

void RelativeNav::setSettings(const Settings &s)
{
    const bool toggled = s.relative != m_settings.relative;
    m_settings = s;
    if (toggled) reset();
}

void RelativeNav::reset()
{
    m_n = m_e = m_d = 0;
    m_drSrc.clear();
    m_pAir0.reset();
    m_pAir.reset();
    m_pWater0.reset();
    m_pWater.reset();
    m_bootMs.reset();
}

void RelativeNav::onMessage(const MavMessage &mm, int vehicleType, std::int64_t now)
{
    m_vehicleType = vehicleType;
    const mavlink_message_t &m = mm.msg;
    switch (m.msgid) {
    case MAVLINK_MSG_ID_ATTITUDE: {
        mavlink_attitude_t a;
        mavlink_msg_attitude_decode(&m, &a);
        if (!std::isfinite(a.yaw)) return;
        m_yaw = a.yaw;
        step(a.time_boot_ms ? std::int64_t(a.time_boot_ms) : now, vehicleType, now);
        break;
    }
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
        mavlink_local_position_ned_t l;
        mavlink_msg_local_position_ned_decode(&m, &l);
        if (!std::isfinite(l.x)) return;
        m_local = Local{l.x, l.y, l.z, l.vx, l.vy, l.vz, now};
        break;
    }
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        mavlink_global_position_int_t g;
        mavlink_msg_global_position_int_decode(&m, &g);
        m_ekf = Ekf{g.vx / 100.0, g.vy / 100.0, g.vz / 100.0, now};
        if (g.lat != 0 && g.lon != 0 && !mm.synthetic) m_fix = Fix{g.relative_alt / 1000.0, now};
        break;
    }
    case MAVLINK_MSG_ID_VFR_HUD: {
        mavlink_vfr_hud_t v;
        mavlink_msg_vfr_hud_decode(&m, &v);
        m_vfr = Vfr{v.airspeed, v.groundspeed, double(v.heading), v.climb, now};
        break;
    }
    case MAVLINK_MSG_ID_SCALED_PRESSURE: {
        mavlink_scaled_pressure_t p;
        mavlink_msg_scaled_pressure_decode(&m, &p);
        if (!std::isfinite(p.press_abs)) return;
        m_pAir = p.press_abs;
        if (!m_pAir0) m_pAir0 = m_pAir;
        break;
    }
    case MAVLINK_MSG_ID_SCALED_PRESSURE2: {
        mavlink_scaled_pressure2_t p;
        mavlink_msg_scaled_pressure2_decode(&m, &p);
        if (!std::isfinite(p.press_abs)) return;
        m_pWater = p.press_abs;
        // The first reading is the surface, unless the GCS connected during a dive
        if (!m_pWater0) m_pWater0 = *m_pWater > kSurfaceMaxHpa ? kStandardHpa : *m_pWater;
        break;
    }
    default: break;
    }
}

std::optional<RelativeNav::Vel> RelativeNav::pickVelocity(int vehicleType, std::int64_t now) const
{
    std::optional<double> hdg = m_yaw;
    if (!hdg && fresh(m_vfr, now)) hdg = m_vfr->heading * kDeg;
    const double vd = fresh(m_ekf, now) ? m_ekf->vd : fresh(m_vfr, now) ? -m_vfr->climb : 0.0;
    auto along = [&](double speed, const char *src) -> std::optional<Vel> {
        if (!hdg || !std::isfinite(speed)) return std::nullopt;
        return Vel{speed * std::cos(*hdg), speed * std::sin(*hdg), vd, QString::fromLatin1(src)};
    };
    const QString &s = m_settings.velSource;
    if (s == QLatin1String("air")) return fresh(m_vfr, now) ? along(m_vfr->airspeed, "air") : std::nullopt;
    if (s == QLatin1String("ground")) return fresh(m_vfr, now) ? along(m_vfr->groundspeed, "ground") : std::nullopt;
    if (s == QLatin1String("ekf"))
        return fresh(m_ekf, now) ? std::optional<Vel>(Vel{m_ekf->vn, m_ekf->ve, m_ekf->vd, QStringLiteral("ekf")}) : std::nullopt;
    if (kFixedWing.count(vehicleType) && fresh(m_vfr, now) && m_vfr->airspeed > 3) return along(m_vfr->airspeed, "air");
    if (fresh(m_ekf, now)) return Vel{m_ekf->vn, m_ekf->ve, m_ekf->vd, QStringLiteral("ekf")};
    return fresh(m_vfr, now) ? along(m_vfr->groundspeed, "ground") : std::nullopt;
}

void RelativeNav::step(std::int64_t t, int vehicleType, std::int64_t now)
{
    const auto prev = m_bootMs;
    m_bootMs = t;
    if (!m_settings.relative) return;
    if (fresh(m_local, now)) {
        m_n = m_local->x;
        m_e = m_local->y;
        m_d = m_local->z;
        m_drSrc.clear();
        return;
    }
    if (!prev || t <= *prev) return;   // first sample, or the autopilot rebooted
    const double dt = (t - *prev) / 1000.0;
    if (dt > kMaxStepS) return;
    const auto v = pickVelocity(vehicleType, now);
    if (!v) return;
    m_drSrc = v->src;
    m_n += v->vn * dt;
    m_e += v->ve * dt;
    m_d += v->vd * dt;
}

double RelativeNav::relativeHeight(int vehicleType, std::int64_t now) const
{
    if (fresh(m_local, now)) return -m_local->z;
    if (fresh(m_fix, now)) return m_fix->relAlt;
    if (vehicleType == kSubmarine && m_pWater) {
        const double density = m_settings.water == QLatin1String("salt") ? 1025.0 : 997.0;
        return -((*m_pWater - *m_pWater0) * 100.0) / (density * 9.80665);
    }
    if (m_pAir && *m_pAir > 0 && m_pAir0 && *m_pAir0 > 0) return 44330.0 * (1 - std::pow(*m_pAir / *m_pAir0, 1 / 5.255));
    return -m_d;
}

void RelativeNav::apply(VehicleState &s, std::int64_t now) const
{
    s.relativeMode = m_settings.relative;
    if (!m_settings.relative) {
        s.relativeSource.clear();
        return;
    }
    const double n = fresh(m_local, now) ? m_local->x : m_n;
    const double e = fresh(m_local, now) ? m_local->y : m_e;
    const LatLon o = s.sceneOrigin;
    s.lat = o.lat + n / kMetersPerDegLat;
    s.lon = o.lon + e / (kMetersPerDegLat * std::cos(o.lat * kDeg));
    s.rawAlt = relativeHeight(s.vehicleType, now);
    s.homeLat = o.lat;
    s.homeLon = o.lon;
    s.homeAlt = 0.0;
    if (!fresh(m_local, now) && !fresh(m_fix, now)) {
        if (const auto v = pickVelocity(s.vehicleType, now)) {
            s.vn = v->vn;
            s.ve = v->ve;
            s.vd = v->vd;
            s.gs = std::hypot(v->vn, v->ve);
            s.vs = -v->vd;
        }
    }
    s.relativeSource = source(now);
    s.relativeVelSource = m_drSrc;
}

QString RelativeNav::source(std::int64_t now) const
{
    if (!m_settings.relative) return {};
    return fresh(m_local, now) ? QStringLiteral("local") : QStringLiteral("dead-reckoning");
}

} // namespace corv
