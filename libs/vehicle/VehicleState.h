// VehicleState.h - the vehicle as the GCS knows it (the telemetry part of STATE in
// js/core/state.js). Written only by the link thread (StateMapper), published
// as an immutable snapshot after every message; the render thread and the
// GUI read the latest snapshot without locks.
//
// Units as in the Electron STATE: radians for attitude, metres, m/s, degrees
// for lat/lon, wall-clock milliseconds for every *Time field.
#pragma once

#include "core/Geo.h"

#include <QString>
#include <array>
#include <cstdint>
#include <optional>
#include <vector>

namespace corv {

struct TrafficEntry {
    QString icao24, callsign;
    double lat = 0, lon = 0, alt = 0;      // alt m
    double velocity = 0, heading = 0;      // m/s, deg
    double vertRate = 0;                   // m/s
    bool onGround = false;
    std::int64_t ts = 0;                   // received
    std::int64_t posTs = 0;                // when the position was measured
};

struct CellNeighbour {
    int pci = 0;
    unsigned earfcn = 0;
    std::optional<int> rsrp;
};

struct CellInfo {
    bool valid = false;
    unsigned eci = 0, enb = 0, sector = 0;
    int mcc = 0, mnc = 0;
    unsigned tac = 0;
    int pci = 0;
    unsigned earfcn = 0;
    int band = 0;
    QString state;
    std::optional<int> rsrp, rssi, rsrq, sinr;
    std::optional<double> sinrAvg;
    unsigned handovers = 0;
    int neighbours = 0;
    std::vector<CellNeighbour> nb;
    std::int64_t receivedAt = 0, changedAt = 0;
};

struct VibrationSample {
    double x = 0, y = 0, z = 0;
    std::int64_t t = 0;
};

struct VehicleState {
    // Source / mode
    QString mode = QStringLiteral("LIVE");            // LIVE, REPLAY, FREEZE...
    bool connected = false;
    QString connectionType = QStringLiteral("none");  // none, mavlink-serial, -udp, -tcp, -lte, msp-serial, msp-tcp
    QString linkType;                                 // SERIAL | UDP | TCP | CORV | LTE
    double linkKbps = 0;
    std::int64_t lastLinkStatsTime = 0;

    // Attitude (rad) and rates (rad/s)
    double roll = 0, pitch = 0, yaw = 0;
    double rollRate = 0, pitchRate = 0, yawRate = 0;
    double aoa = 0, ssa = 0, gamma = 0, track = 0;

    // Position
    double lat = kDefaultOrigin.lat, lon = kDefaultOrigin.lon;
    double rawAlt = 0, offsetAlt = 0;
    std::optional<double> subSurfaceAlt;

    // Speeds
    double as = 0, gs = 0, vs = 0;
    double vn = 0, ve = 0, vd = 0;
    double ax = 0, ay = 0, az = 0;

    // Sample timing (PoseSmoother): arrival on the wall clock, vehicle clock or absent
    std::int64_t attTime = 0, posTime = 0;
    std::optional<std::uint32_t> attBootMs, posBootMs;

    // Vehicle identity and mode
    bool armed = false;
    QString flightMode = QStringLiteral("UNKNOWN");
    std::uint32_t flightModeNum = 0;
    int baseMode = 0;
    std::uint32_t customMode = 0;
    int autopilotType = 0;
    int vehicleType = 0;
    int systemStatus = 0;
    int systemId = 1, componentId = 1;
    std::int64_t heartbeatCount = 0;
    std::int64_t lastHeartbeatTime = 0;

    // Battery
    double batteryVoltage = 0, batteryCurrent = 0;
    int batteryRemaining = -1;

    // GPS
    int gpsFix = 0, gpsNumSat = 0;
    double gpsHdop = 99.9;
    std::int64_t gpsDataTime = 0;

    // Radio
    double linkQuality = 0;
    std::optional<int> rssi, remRssi;
    CellInfo cell;

    // Health
    std::uint32_t sensorsPresent = 0, sensorsEnabled = 0, sensorsHealth = 0;
    std::int64_t sysStatusTime = 0;
    int ekfFlags = 0;
    double ekfVariance = 0, ekfCompassVariance = 0;
    std::int64_t ekfDataTime = 0;
    int gnssJamming = 0, gnssSpoofing = 0;
    std::int64_t gnssIntegrityTime = 0;
    std::int64_t magCalTime = 0;

    std::optional<double> rangefinderDist;

    // NAV_CONTROLLER_OUTPUT
    double navRoll = 0, navPitch = 0, navBearing = 0, targetBearing = 0, wpDist = 0;
    double altError = 0, aspdError = 0, xtrackError = 0;
    std::int64_t navDataTime = 0;

    // Wind
    double windDir = 0, windSpeed = 0, windSpeedZ = 0;
    std::int64_t windDataTime = 0;
    double appWindDir = 0, appWindSpeed = 0;
    std::int64_t appWindTime = 0;

    // RTK
    int rtkIar = 0;
    double rtkBaseline = 0, rtkAccuracy = 0;

    // Vibration (last 120 samples, oldest first)
    double vibX = 0, vibY = 0, vibZ = 0;
    std::uint32_t vibClip0 = 0, vibClip1 = 0, vibClip2 = 0;
    std::vector<VibrationSample> vibHistory;

    // Home
    std::optional<double> homeLat, homeLon, homeAlt;

    // Channels
    std::array<std::uint16_t, 18> rcChannels{};
    std::array<std::uint16_t, 16> servoOutputs{};

    // Mission
    int missionCount = 0;
    int missionCurrentSeq = 0;

    // Status text (last)
    QString statusText;
    int statusSeverity = 0;
    std::int64_t statusTextTime = 0;

    // Last COMMAND_ACK
    int lastAckCommand = -1, lastAckResult = -1;

    // The vehicle's terrain database (TERRAIN_REPORT)
    int terrainPending = 0, terrainLoaded = 0;

    // ADS-B
    bool adsbEnabled = true;
    std::vector<TrafficEntry> traffic;

    // Scene origin: world metres count from here; epoch changes on every move
    LatLon sceneOrigin = kDefaultOrigin;
    int sceneOriginEpoch = 0;

    // Relative navigation (no absolute position)
    bool relativeMode = false;
    QString relativeSource;   // "local", "dead-reckoning", empty outside the mode
    QString relativeVelSource;  // what dead reckoning integrates: air, ground, ekf (empty before any)

    /// Total G (signed by the normal axis, NED az = -9.81 at rest -> +1 G).
    double gLoad() const
    {
        const double mag = std::hypot(ax, ay, az) / 9.81;
        return az > 0 ? -mag : mag;
    }
    /// Demo flight: LIVE without a link.
    bool isDemo() const { return mode == QLatin1String("LIVE") && !connected; }
};

} // namespace corv
