// GpsReceivers.h - the two GNSS receivers ArduPilot reports, and which one to show.
//
// GPS_RAW_INT is always the first receiver (GPS1_* parameters: the first
// serial port set to a GPS protocol) and GPS2_RAW the second, whichever the
// autopilot navigates on. With the only receiver on the second port
// GPS_RAW_INT says NO GPS, 0 satellites, while GPS2_RAW has a 3D fix and the
// vehicle uses it. What the GCS shows is the better of the receivers heard in
// the last 5 s: the higher fix, then more satellites, then the lower HDOP; the
// first one on a tie.
#pragma once

#include "link/Mavlink.h"

#include <array>
#include <cstdint>

namespace corv {

struct GpsReading {
    int fix = 0;                 // GPS_FIX_TYPE
    int sats = 0;                // 0 when not reported
    double hdop = 99.9;          // eph / 100; 99.9 when not reported
    double hAcc = 0, vAcc = 0;   // m; 0 = not reported
    double velAcc = 0;           // m/s; 0 = not reported
    std::int32_t lat = 0, lon = 0, altMm = 0;
    std::uint16_t vel = UINT16_MAX, cog = UINT16_MAX;   // cm/s, cdeg
    std::int64_t time = 0;       // arrival on the wall clock, ms; 0 = never
};

class GpsReceivers
{
public:
    static constexpr std::int64_t kStaleMs = 5000;

    /// A GPS_RAW_INT or GPS2_RAW: the receiver it reports (0 or 1); -1 for any other message.
    int feed(const mavlink_message_t &m, std::int64_t now);
    /// The receiver to show: -1 when none was heard in the last 5 s.
    int best(std::int64_t now) const;
    const GpsReading &at(int i) const { return m_rx[std::size_t(i)]; }
    void reset() { m_rx = {}; }

private:
    std::array<GpsReading, 2> m_rx;
};

} // namespace corv
