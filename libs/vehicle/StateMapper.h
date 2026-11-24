// StateMapper.h - MAVLink message -> VehicleState (port of MAVLinkStateMapper.js
// and the heartbeat bookkeeping of MAVLinkManager.handleMessage).
//
// Behaviours kept from the Electron version:
//  - position / velocity priority chain: GLOBAL_POSITION_INT first, VFR_HUD
//    ground speed and climb only while that is older than 2 s, GPS_RAW_INT last;
//  - boot-time nulls rejected on the exact raw integers (0, 0);
//  - mode tables per vehicle type, the link protocol (MSP) winning over MAV_TYPE;
//  - the sample timing (arrival + time_boot_ms) PoseSmoother projects from;
//  - AoA / SSA from the air-relative velocity (wind subtracted when fresh).
// Runs on the link thread; `now` is the message's arrival on the wall clock.
#pragma once

#include "link/MavMessage.h"
#include "vehicle/RelativeNav.h"
#include "vehicle/SceneOrigin.h"
#include "vehicle/VehicleState.h"

#include <QHash>
#include <functional>

namespace corv {

class StateMapper
{
public:
    struct Hooks {
        /// The first fix of a stream or a HOME_POSITION on a live link: remember as last home (GUI persists it).
        std::function<void(double lat, double lon)> rememberLastHome;
        /// COMMAND_ACK other than the internal polling ones, for the HUD toast.
        std::function<void(int command, int result)> commandAck;
        /// STATUSTEXT, for the HUD message list.
        std::function<void(int severity, const QString &text)> statusText;
        /// The water a point is on, else the ground there (elevation data); none while it is not in. Any thread.
        std::function<std::optional<double>(double lat, double lon)> surface;
    };

    StateMapper();

    void setHooks(Hooks h) { m_hooks = std::move(h); }
    SceneOrigin &origin() { return m_origin; }
    RelativeNav &relativeNav() { return m_relNav; }

    /// Battery range used when the vehicle reports no percentage (SYS CONFIG).
    void setBatteryRange(double vmin, double vmax) { m_batVMin = vmin; m_batVMax = vmax; }

    /// Map one message. Returns true when the state changed in a way readers care about.
    void apply(const MavMessage &m, VehicleState &s);

    /// After a burst of messages (or once per tick): relative frame, traffic purge.
    void finish(VehicleState &s, std::int64_t now);

    /// Clear what the replay or a new link must not inherit (resetReplayState).
    void resetTelemetry(VehicleState &s);

    static void computeAeroAngles(VehicleState &s, std::int64_t now);

private:
    void heartbeat(const mavlink_message_t &m, VehicleState &s);
    void sysStatus(const mavlink_message_t &m, VehicleState &s, std::int64_t now);
    void gpsRawInt(const MavMessage &mm, VehicleState &s, std::int64_t now);
    void scaledImu(const mavlink_message_t &m, VehicleState &s, bool primary, std::int64_t now);
    void attitude(const MavMessage &mm, VehicleState &s, std::int64_t now);
    void globalPositionInt(const MavMessage &mm, VehicleState &s, std::int64_t now);
    void vfrHud(const mavlink_message_t &m, VehicleState &s, std::int64_t now);
    void homePosition(const mavlink_message_t &m, VehicleState &s);
    void adsbVehicle(const mavlink_message_t &m, VehicleState &s, std::int64_t now);
    void cellInfo(const mavlink_message_t &m, VehicleState &s, std::int64_t now);
    void guidance(const mavlink_message_t &m, VehicleState &s, std::int64_t now);
    void sensors(const mavlink_message_t &m, VehicleState &s, std::int64_t now);
    void rotation(const mavlink_message_t &m, VehicleState &s, std::int64_t now);
    void reports(const mavlink_message_t &m, VehicleState &s, std::int64_t now);
    void firstFix(VehicleState &s);
    void subSurface(VehicleState &s, std::int64_t now);

    Hooks m_hooks;
    SceneOrigin m_origin;
    RelativeNav m_relNav;
    double m_batVMin = 9.6, m_batVMax = 12.6;
    std::int64_t m_lastGlobalPosTs = 0;
    bool m_imuPrimaryActive = false;
    std::int64_t m_imuPrimaryLastTime = 0;
    std::int64_t m_lastTrafficPurge = 0;
    QString m_subSurfaceKey;
    std::int64_t m_subSurfaceTried = 0;
};

} // namespace corv
