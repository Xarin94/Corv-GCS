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
#include "vehicle/GpsReceivers.h"
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

    /// Battery gauge for a vehicle that reports no percentage (SYS CONFIG): volts per
    /// cell when empty and full, and the cell count (0 = automatic).
    void setBatteryGauge(double cellEmpty, double cellFull, int cells);
    static constexpr double kCellEmptyV = 3.2, kCellFullV = 4.2;
    /// The most a cell holds (HV LiPo 4.35 V, and some margin): the automatic count is
    /// the pack voltage over this, rounded up.
    static constexpr double kCellMaxV = 4.4;
    /// Under this the board runs on USB or bench power: no pack to count or to gauge.
    static constexpr double kNoPackV = 2.5;

    /// A new link or vehicle: forget its GPS receivers and its battery.
    void newVehicle();

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
    void gpsRaw(const MavMessage &mm, VehicleState &s, std::int64_t now);
    void batteryStatus(const mavlink_message_t &m, VehicleState &s);
    void batteryCells(VehicleState &s);
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
    GpsReceivers m_gps;
    double m_cellEmpty = kCellEmptyV, m_cellFull = kCellFullV;
    int m_fixedCells = 0;      // SYS CONFIG; 0 = automatic
    int m_vehicleCells = 0;    // cells the vehicle measures (BATTERY_STATUS)
    int m_detectedCells = 0;   // from the highest pack voltage since the link opened
    std::int64_t m_lastGlobalPosTs = 0;
    bool m_imuPrimaryActive = false;
    std::int64_t m_imuPrimaryLastTime = 0;
    std::int64_t m_lastTrafficPurge = 0;
    QString m_subSurfaceKey;
    std::int64_t m_subSurfaceTried = 0;
};

} // namespace corv
