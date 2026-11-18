#include "vehicle/DemoFlight.h"

#include "vehicle/StateMapper.h"
#include "vehicle/VehicleState.h"

#include <algorithm>
#include <cmath>

namespace corv {

namespace {
// js/core/constants.js
constexpr double kCruiseSpeed = 28;
constexpr double kSpeedVariance = 4;
constexpr double kCruiseAgl = 50;
constexpr double kMinClearance = 35;
constexpr double kMinFloor = 15;      // the demo is never closer to the ground than this
constexpr double kMaxVs = 3.5;
const double kMaxBank = 28 * kDeg;
const double kRollRate = 22 * kDeg;
constexpr double kHdgGain = 2.2;
constexpr double kStraight = 1100;
constexpr double kTurnRadius = 340;
constexpr double kCaptureR = 130;
} // namespace

void DemoFlight::init(VehicleState &s)
{
    m_centerLat = s.lat;
    m_centerLon = s.lon;
    const double mLon = std::max(1.0, kMetersPerDegLat * std::cos(m_centerLat * kDeg));
    const double R = kTurnRadius, H = kStraight / 2;
    const int arcSteps = 6;
    m_wps.clear();
    auto push = [&](double n, double e) { m_wps.push_back({m_centerLat + n / kMetersPerDegLat, m_centerLon + e / mLon}); };
    push(-H, R); push(0, R); push(H, R);
    for (int i = 1; i < arcSteps; ++i) {
        const double th = kPi * i / arcSteps;
        push(H + R * std::sin(th), R * std::cos(th));
    }
    push(H, -R); push(0, -R); push(-H, -R);
    for (int i = 1; i < arcSteps; ++i) {
        const double th = kPi + kPi * i / arcSteps;
        push(-H + R * std::sin(th), R * std::cos(th));
    }
    s.lat = m_wps[0].lat;
    s.lon = m_wps[0].lon;
    s.yaw = 0;
    m_wp = 1;
    m_prevWpDist.reset();
    m_bank = 0;
    m_speed = m_speedTarget = kCruiseSpeed;
    const auto t = m_terrain ? m_terrain(m_centerLat, m_centerLon) : std::nullopt;
    m_baseAlt = t.value_or(600) + kCruiseAgl;
    m_baseAltLocked = t.has_value();
    s.rawAlt = m_baseAlt;
    m_windChange = 0;
    m_init = true;
}

void DemoFlight::step(VehicleState &s, double dtIn, double now)
{
    const double dt = std::clamp(dtIn, 1e-3, 0.1);
    if (!m_init) init(s);
    const std::optional<double> terrain = m_terrain ? m_terrain(s.lat, s.lon) : std::nullopt;
    s.terrainHeight = terrain;
    if (!m_baseAltLocked) {
        if (const auto c = m_terrain ? m_terrain(m_centerLat, m_centerLon) : std::nullopt) {
            m_baseAlt = *c + kCruiseAgl;
            s.rawAlt = m_baseAlt;
            m_baseAltLocked = true;
        }
    }
    const double mLon = std::max(1.0, kMetersPerDegLat * std::cos(m_centerLat * kDeg));

    // Waypoint guidance: advance on capture, or once the fix is behind
    const LatLon wp = m_wps[m_wp];
    const double dN = (wp.lat - s.lat) * kMetersPerDegLat, dE = (wp.lon - s.lon) * mLon;
    const double dist = std::hypot(dN, dE);
    if (dist < kCaptureR || (m_prevWpDist && dist > *m_prevWpDist && dist < kCaptureR * 2.5)) {
        m_wp = (m_wp + 1) % m_wps.size();
        m_prevWpDist.reset();
    } else {
        m_prevWpDist = dist;
    }
    const double desired = std::atan2(dE, dN);

    // Coordinated turn: bank commands the yaw rate
    const double hdgErr = wrapPi(desired - s.yaw);
    const double bankCmd = std::clamp(kHdgGain * hdgErr, -kMaxBank, kMaxBank);
    const double rollRate = std::clamp((bankCmd - m_bank) / 0.7, -kRollRate, kRollRate);
    m_bank += rollRate * dt;
    s.roll = m_bank;
    s.rollRate = rollRate;

    const double bankFrac = std::abs(m_bank) / kMaxBank;
    m_speedTarget = kCruiseSpeed - 3 * bankFrac + std::sin(now * 0.0003) * kSpeedVariance * 0.5;
    m_speed += (m_speedTarget - m_speed) * std::min(1.0, dt / 3);
    const double V = std::max(12.0, m_speed);

    const double omega = 9.81 * std::tan(m_bank) / V;
    s.yaw = std::fmod(std::fmod(s.yaw + omega * dt, 2 * kPi) + 2 * kPi, 2 * kPi);
    s.yawRate = omega;

    // Wind: a new gust every 20-40 s, 3-6 m/s
    if (now > m_windChange) {
        std::uniform_real_distribution<double> u(0, 1);
        m_windChange = now + 20000 + u(m_rng) * 20000;
        const double w = 3 + u(m_rng) * 3, from = u(m_rng) * 2 * kPi;
        m_windTargetN = -std::cos(from) * w;
        m_windTargetE = -std::sin(from) * w;
    }
    m_windN += (m_windTargetN - m_windN) * std::min(1.0, dt / 8);
    m_windE += (m_windTargetE - m_windE) * std::min(1.0, dt / 8);
    s.windSpeed = std::hypot(m_windN, m_windE);
    s.windDir = std::fmod(std::atan2(-m_windE, -m_windN) * kRad + 360, 360);
    s.windDataTime = std::int64_t(now);

    // Altitude: fixed cruise MSL, rate-limited terrain follow. The terrain is read
    // ahead along the nose too, so the climb starts before rising ground and not
    // on it, and the demo never flies into a slope it could not out-climb
    double terrainElev = terrain.value_or(m_baseAlt - kCruiseAgl);
    double ahead = terrainElev;
    if (m_terrain) {
        for (const double d : {150.0, 300.0, 600.0}) {
            const auto t = m_terrain(s.lat + std::cos(s.yaw) * d / kMetersPerDegLat, s.lon + std::sin(s.yaw) * d / mLon);
            if (t) ahead = std::max(ahead, *t);
        }
    }
    const double targetAlt = std::max(m_baseAlt, ahead + kMinClearance);
    if (!std::isfinite(s.rawAlt) || s.rawAlt <= 0) s.rawAlt = targetAlt;
    // Climb harder while below the minimum clearance
    const bool low = terrain && s.rawAlt - *terrain < kMinClearance;
    const double maxVs = low ? 3 * kMaxVs : kMaxVs;
    const double vsCmd = std::clamp((targetAlt - s.rawAlt) * 0.15, -kMaxVs, maxVs);
    s.vs += (vsCmd - s.vs) * std::min(1.0, dt / (low ? 0.5 : 1.5));
    s.rawAlt += s.vs * dt;
    // Hard floor: never under the ground, whatever the tile that just loaded says
    if (terrain && s.rawAlt < *terrain + kMinFloor) {
        s.rawAlt = *terrain + kMinFloor;
        s.vs = std::max(0.0, s.vs);
    }

    // Pitch: climb angle plus the load-factor pull in the turn
    const double climbAngle = std::asin(std::clamp(s.vs / V, -0.5, 0.5));
    const double pitchCmd = climbAngle + (1 / std::cos(m_bank) - 1) * 0.06;
    const double prevPitch = s.pitch;
    s.pitch += (pitchCmd - s.pitch) * std::min(1.0, dt / 0.6);
    s.pitchRate = (s.pitch - prevPitch) / dt;

    // Ground position: airspeed along the nose plus the wind
    const double airHoriz = V * dt * std::cos(s.pitch);
    const double north = std::cos(s.yaw) * airHoriz + m_windN * dt;
    const double east = std::sin(s.yaw) * airHoriz + m_windE * dt;
    s.lat += north / kMetersPerDegLat;
    s.lon += east / mLon;
    s.vn = north / dt;
    s.ve = east / dt;
    s.vd = -s.vs;
    const double gh = std::hypot(s.vn, s.ve);
    s.as = V;
    s.gs = gh;
    s.track = std::fmod(std::fmod(std::atan2(s.ve, s.vn), 2 * kPi) + 2 * kPi, 2 * kPi);
    s.gamma = std::atan2(s.vs, std::max(1.0, gh));
    StateMapper::computeAeroAngles(s, std::int64_t(now));

    if (terrain) s.rangefinderDist = std::max(0.01, s.rawAlt - *terrain);
    else s.rangefinderDist.reset();

    const double load = std::min(3.0, 1 / std::cos(m_bank));
    s.ax = 0;
    s.ay = 0;
    s.az = -9.81 * load;
    // Written every frame: PoseSmoother draws it as it is (no vehicle clock)
    s.posTime = s.attTime = 0;
    s.posBootMs.reset();
    s.attBootMs.reset();
}

} // namespace corv
