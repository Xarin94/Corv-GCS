// LogIndex.h - the index LOG REPLAY plays from (port of indexTlogFile in
// log-replay-manager.js and of log-replay-bin-parser.js).
//
// .tlog: the file is memory-mapped and the index holds only where each frame
// is (offset, length) and when (ms from the start): a 200 MB tlog costs tens of
// MB, not the 884 MB of heap the Electron index took.
//
// .bin: ArduPilot DataFlash. The whitelisted messages become real MAVLink v2
// frames (packed with the generated functions, so they parse, record and replay
// like any other) laid end to end in one buffer, indexed the same way:
//   ATT → ATTITUDE, AHR2 → GLOBAL_POSITION_INT (position, yaw; velocity carried
//   over from NKF1/XKF1, GPI or GPS), GPS → GPS_RAW_INT and VFR_HUD, BARO / ARSP
//   → VFR_HUD, BAT → SYS_STATUS, MODE → HEARTBEAT, MSG → STATUSTEXT, RCIN →
//   RC_CHANNELS_RAW, RCOU / RCO2 → SERVO_OUTPUT_RAW, VIBE → VIBRATION, ORGN →
//   GPS_GLOBAL_ORIGIN. VER / PARM identify the vehicle: every HEARTBEAT carries
//   the MAV_TYPE the autopilot would have sent. The IMU samples (GYR, ACC, IMU,
//   ISBH / ISBD) are not replayed: they go to LogSpectrum.
//
// Both: timestamps from 0, the full-flight track for the trail (a centred
// 5-point moving average, written back into the .bin positions).
#pragma once

#include "services/LogSpectrum.h"

#include <QByteArray>
#include <QString>
#include <cstdint>
#include <vector>

namespace corv {

struct LogEntry {
    qint64 offset = 0;      // of the frame's STX, in the mapped file (.tlog) or in frames (.bin)
    std::uint32_t tsMs = 0;
    std::uint32_t len = 0;
};

struct TrackPoint {
    double lat = 0, lon = 0, alt = 0;
};

struct ParsedLog {
    QString format;                 // "tlog" or "bin"
    QByteArray frames;              // .bin: the synthetic frames
    std::vector<LogEntry> index;
    std::uint32_t totalMs = 0;
    std::vector<TrackPoint> gpsTrack;
    int mavType = 1;
    int gapsCompressed = 0;
    ImuStreams imu;
};

/// Index a .tlog: 8-byte little-endian µs stamp, then the frame.
ParsedLog indexTlog(const std::uint8_t *data, qint64 size);
/// Parse a .bin into synthetic frames.
ParsedLog parseBinLog(const std::uint8_t *data, qint64 size);
/// Light, fixed smoothing of a track: centred moving average over ±2 samples.
std::vector<TrackPoint> smoothTrack(const std::vector<TrackPoint> &track);
/// msgid of the MAVLink frame at p (v1 or v2), or -1.
int frameMsgId(const std::uint8_t *p, std::uint32_t len);

} // namespace corv
