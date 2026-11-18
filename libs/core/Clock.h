// Clock.h - the two clocks the GCS uses.
//
// wallMs(): milliseconds since the Unix epoch, Date.now() of the Electron
// version. Arrival stamps of telemetry, tlog timestamps and "x ms ago" checks
// use it, so a replay and the debug log stay comparable with the old version.
// monoMs(): a steady clock for frame timing and timeouts.
#pragma once

#include <QDateTime>
#include <chrono>
#include <cstdint>

namespace corv {

inline std::int64_t wallMs() { return QDateTime::currentMSecsSinceEpoch(); }

inline double monoMs()
{
    using namespace std::chrono;
    return duration<double, std::milli>(steady_clock::now().time_since_epoch()).count();
}

} // namespace corv
