// EkfMonitor.h - EKF STATUS data: the last 20 s of the estimator's own health
// figures (port of js/core/EkfMonitor.js): ArduPilot's EKF3, over MAVLink:
//  - EKF_STATUS_REPORT (193): innovation test ratios of velocity, horizontal and
//    vertical position, compass and airspeed (1 = measurement rejected), and the
//    flags of the solutions the EKF trusts;
//  - GPS_RAW_INT (24): the receiver's horizontal, vertical and speed accuracy;
//  - AHRS2 (178) against ATTITUDE (30): the DCM backup estimator against the EKF;
//  - VIBRATION (241): what spoils the accelerometers the EKF predicts with.
//
// Times are passed in (monoMs() in the app). No Qt Quick; GUI thread.
#pragma once

#include "link/Mavlink.h"
#include "vehicle/GpsReceivers.h"

#include <QByteArray>
#include <QHash>
#include <QString>
#include <array>
#include <deque>
#include <optional>
#include <vector>

namespace corv {

constexpr double kEkfWindowMs = 20000;

/// EKF_STATUS_FLAGS (MAVLink common).
namespace ekfflags {
constexpr int Attitude = 1, VelocityHoriz = 2, VelocityVert = 4, PosHorizRel = 8, PosHorizAbs = 16, PosVertAbs = 32,
              PosVertAgl = 64, ConstPosMode = 128, PredPosHorizRel = 256, PredPosHorizAbs = 512, Uninitialized = 1024,
              GpsGlitching = 32768;
}

/// Time series trimmed to the chart window (plus a margin for the left edge).
class EkfSeries
{
public:
    void push(double t, double v);
    int size() const { return int(m_t.size()); }
    bool empty() const { return m_t.empty(); }
    double t(int i) const { return m_t[std::size_t(i)]; }
    double v(int i) const { return m_v[std::size_t(i)]; }
    std::optional<double> last() const { return m_v.empty() ? std::nullopt : std::optional<double>(m_v.back()); }
    std::optional<double> lastTime() const { return m_t.empty() ? std::nullopt : std::optional<double>(m_t.back()); }
    /// Value in force at time t (the sample at or before it).
    std::optional<double> at(double t) const;

private:
    std::deque<double> m_t, m_v;
};

class EkfMonitor
{
public:
    struct ArduPilot {
        double time = -1e300;
        std::optional<int> flags, fix, sats, clip;
        std::optional<double> terr, hdop;
        std::optional<std::array<double, 3>> vibe;
    };

    EkfMonitor() { reset(); }
    void reset();

    /// The series of that name, created on first use.
    EkfSeries &get(const QString &name) { return m_series[name]; }
    const EkfSeries &series(const QString &name) const;

    /// "ardupilot" while it reports, else the last source that did ("" for none).
    QString source(double now) const;
    /// True while the source on show is still reporting.
    bool live(double now) const;

    void ingestMavlink(const mavlink_message_t &msg, double now);

    const ArduPilot &ap() const { return m_ap; }

private:
    struct Att {
        double t, r, p, y;
    };
    void resolveAhrs2(double now, bool force);

    QHash<QString, EkfSeries> m_series;
    ArduPilot m_ap;
    GpsReceivers m_gps;                    // GPS_RAW_INT and GPS2_RAW: the better one is drawn
    QString m_lastSource;
    std::vector<Att> m_attitude;           // last two ATTITUDE samples (deg), arrival time
    std::optional<Att> m_pendingAhrs2;
};

} // namespace corv
