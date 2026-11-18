// Ltm.h - Lightweight Telemetry frames for the telemetry forward outputs
// (buildLTM*Frame of telforward-manager.js): $T, a function letter, a little-
// endian payload and the XOR of the payload. G and A at 5 Hz, S at 2 Hz, O at
// 0.5 Hz.
#pragma once

#include <QByteArray>
#include <QString>

namespace corv {
struct VehicleState;
}

namespace corv::ltm {

/// G: lat, lon (1e-7 deg), ground speed (m/s), altitude (cm), sats << 2 | fix.
QByteArray gFrame(const VehicleState &s);
/// A: pitch, roll, heading (degrees, heading 0-360).
QByteArray aFrame(const VehicleState &s);
/// S: battery (mV), capacity (0), RSSI, airspeed (m/s), armed | mode << 2.
QByteArray sFrame(const VehicleState &s);
/// O: home lat, lon (1e-7 deg), home altitude (cm MSL), fix, sats.
QByteArray oFrame(const VehicleState &s);

/// ArduPilot mode name → LTM flight mode (0 Manual ... 18 Unknown).
int flightMode(const QString &name);

} // namespace corv::ltm
