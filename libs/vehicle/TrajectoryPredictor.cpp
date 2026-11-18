#include "vehicle/TrajectoryPredictor.h"

#include "core/Geo.h"
#include "vehicle/VehicleState.h"

#include <algorithm>
#include <cmath>

namespace corv {

namespace {
constexpr double kG = 9.81;
constexpr double kMinSpeed = 2;              // m/s: below, no prediction
constexpr double kMinRollForTurn = 0.00873;  // ~0.5 deg
constexpr double kMinTurnRadius = 10;
constexpr double kMaxVertAccel = 5;
constexpr double kRateTau = 0.7;
constexpr double kRelaxTau = 2.0;
const double kMaxBank = 60 * kDeg;
constexpr std::int64_t kNavFreshMs = 3000;
constexpr double kVertAccelTau = 4;
constexpr double kLpAlpha = 0.88;

double omega(double roll, double gs)
{
    if (std::abs(roll) < kMinRollForTurn) return 0;
    const double tr = std::tan(roll);
    const double radius = gs * gs / (kG * std::abs(tr));
    if (radius < kMinTurnRadius) return gs / kMinTurnRadius * (tr < 0 ? -1 : 1);
    return kG * tr / gs;
}
} // namespace

double TrajectoryPredictor::predictionTime(double gs)
{
    constexpr double minT = 5, maxT = 20, minV = 5, maxV = 60;
    if (gs <= minV) return minT;
    if (gs >= maxV) return maxT;
    return minT + (gs - minV) / (maxV - minV) * (maxT - minT);
}

std::vector<TrajectoryPredictor::Point> TrajectoryPredictor::predict(const VehicleState &s, double lat0, double lon0, double alt0,
                                                                     int numPoints, double totalTimeSec, std::int64_t now)
{
    const double track = s.track != 0 ? s.track : s.yaw;
    if (!m_init) {
        m_gs = s.gs;
        m_roll = s.roll;
        m_rollRate = s.rollRate;
        m_vs = s.vs;
        m_vertAccel = 0;
        m_trackSin = std::sin(track);
        m_trackCos = std::cos(track);
        m_prevVd = s.vd;
        m_prevVdTime = now;
        m_init = true;
    } else {
        m_gs = kLpAlpha * m_gs + (1 - kLpAlpha) * s.gs;
        m_roll = kLpAlpha * m_roll + (1 - kLpAlpha) * s.roll;
        // Lighter on the gyro rate: it must catch roll-in and roll-out
        m_rollRate = 0.7 * m_rollRate + 0.3 * s.rollRate;
        m_vs = kLpAlpha * m_vs + (1 - kLpAlpha) * s.vs;
        m_trackSin = kLpAlpha * m_trackSin + (1 - kLpAlpha) * std::sin(track);
        m_trackCos = kLpAlpha * m_trackCos + (1 - kLpAlpha) * std::cos(track);
        const std::int64_t dtMs = now - m_prevVdTime;
        if (dtMs > 10) {
            const double raw = -(s.vd - m_prevVd) / (dtMs / 1000.0);
            m_vertAccel = kLpAlpha * m_vertAccel + (1 - kLpAlpha) * std::clamp(raw, -kMaxVertAccel, kMaxVertAccel);
            m_prevVd = s.vd;
            m_prevVdTime = now;
        }
    }

    std::vector<Point> out;
    if (m_gs < kMinSpeed || numPoints < 1) return out;
    const double dt = totalTimeSec / numPoints;

    // Bank profile: the measured rate carries it, then guidance pulls it to the commanded roll
    const bool navFresh = s.navDataTime > 0 && now - s.navDataTime < kNavFreshMs;
    const double targetBank = s.navRoll * kDeg;
    std::vector<double> banks(static_cast<std::size_t>(numPoints));
    double bank = m_roll;
    for (int i = 0; i < numPoints; ++i) {
        bank += m_rollRate * std::exp(-i * dt / kRateTau) * dt;
        if (navFresh) bank += (targetBank - bank) * std::min(1.0, dt / kRelaxTau);
        bank = std::clamp(bank, -kMaxBank, kMaxBank);
        banks[std::size_t(i)] = bank;
    }

    double heading = std::atan2(m_trackSin, m_trackCos);
    double lat = lat0, lon = lon0, alt = alt0, vs = m_vs;
    out.reserve(std::size_t(numPoints));
    for (int i = 0; i < numPoints; ++i) {
        const double t = (i + 1) * dt;
        heading += omega(banks[std::size_t(i)], m_gs) * dt;
        lat += m_gs * std::cos(heading) * dt / kMetersPerDegLat;
        lon += m_gs * std::sin(heading) * dt / (kMetersPerDegLat * std::cos(lat * kDeg));
        // A constant extrapolation of the vertical acceleration turns noise into parabolas: it decays
        vs += m_vertAccel * std::exp(-t / kVertAccelTau) * dt;
        alt += vs * dt;
        out.push_back({lat, lon, alt, t, banks[std::size_t(i)]});
    }
    return out;
}

} // namespace corv
