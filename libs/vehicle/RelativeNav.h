// RelativeNav.h - navigation without an absolute position (port of js/core/RelativeNav.js).
//
// With no GPS ever, ArduPilot sends no LOCAL_POSITION_NED and zeros in
// GLOBAL_POSITION_INT. In relative mode the position is LOCAL_POSITION_NED
// whenever the vehicle sends it (its EKF already fuses a DVL / USBL /
// odometry), otherwise it is dead-reckoned here from a velocity, starting from
// the last local position. Height: LOCAL_POSITION_NED, else the relative
// altitude of a real fix, else the pressure sensor against its first reading
// (water depth on a sub, the barometer otherwise). The frame is laid around
// the scene origin as synthetic lat/lon. Integration runs on the autopilot's
// ATTITUDE time_boot_ms, so a log replayed faster integrates the same way.
#pragma once

#include "link/MavMessage.h"

#include <QString>
#include <optional>

namespace corv {

struct VehicleState;

class RelativeNav
{
public:
    struct Settings {
        bool relative = false;
        QString velSource = QStringLiteral("auto");   // auto, air, ground, ekf
        QString water = QStringLiteral("fresh");      // fresh, salt
    };

    void setSettings(const Settings &s);
    const Settings &settings() const { return m_settings; }
    bool isRelative() const { return m_settings.relative; }
    void reset();

    /// Every message, before the mapper's own mapping. now = wall ms.
    void onMessage(const MavMessage &m, int vehicleType, std::int64_t now);
    /// Write the relative position into the state (once per message batch / frame).
    void apply(VehicleState &s, std::int64_t now) const;
    QString source(std::int64_t now) const;
    QString deadReckoningSource() const { return m_drSrc; }

private:
    struct Vel {
        double vn = 0, ve = 0, vd = 0;
        QString src;
    };
    std::optional<Vel> pickVelocity(int vehicleType, std::int64_t now) const;
    void step(std::int64_t t, int vehicleType, std::int64_t now);
    double relativeHeight(int vehicleType, std::int64_t now) const;

    Settings m_settings;
    std::optional<double> m_yaw;
    std::optional<std::int64_t> m_bootMs;
    struct Local { double x, y, z, vx, vy, vz; std::int64_t t; };
    struct Ekf { double vn, ve, vd; std::int64_t t; };
    struct Fix { double relAlt; std::int64_t t; };
    struct Vfr { double airspeed, groundspeed, heading, climb; std::int64_t t; };
    std::optional<Local> m_local;
    std::optional<Ekf> m_ekf;
    std::optional<Fix> m_fix;
    std::optional<Vfr> m_vfr;
    std::optional<double> m_pAir0, m_pAir, m_pWater0, m_pWater;
    double m_n = 0, m_e = 0, m_d = 0;
    QString m_drSrc;
    int m_vehicleType = 0;
};

} // namespace corv
