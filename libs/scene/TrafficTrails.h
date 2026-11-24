// TrafficTrails.h - the paths of the ADS-B aircraft in the 3D view (js/engine/Traffic3D.js).
//
// Reports are sparse (OpenSky every 30 s, ADSB_VEHICLE about once a second), so
// the trail is a cubic Hermite curve through them whose tangent at each report
// is that report's own velocity: it bends where the aircraft turned instead of
// cutting straight from fix to fix, and it fades with age over three minutes.
// Between reports the aircraft is dead-reckoned along its last velocity.
// Positions stay geographic: a new scene origin moves nothing here.
#pragma once

#include <QHash>
#include <QString>
#include <cstdint>
#include <vector>

namespace corv {

class TrafficTrails
{
public:
    static constexpr double kTrailSeconds = 180;
    static constexpr double kMaxDeadReckonS = 30;   // never further past the last report

    struct Report {
        QString id;              // ICAO address
        double lat = 0, lon = 0, alt = 0;
        double velocity = 0, heading = 0, vertRate = 0;   // m/s, deg true
        std::int64_t timeMs = 0;                          // when the position was measured
    };
    struct Vertex {
        double lat = 0, lon = 0, alt = 0;
        double alpha = 0;        // 1 at the aircraft, 0 three minutes back
    };

    /// The aircraft shown now: new reports become samples, the others are dropped.
    void update(const std::vector<Report> &reports);
    /// Where an aircraft is at `nowMs`; false when unknown.
    bool head(const QString &id, double nowMs, double *lat, double *lon, double *alt) const;
    /// Its path, oldest first, ending at the aircraft.
    std::vector<Vertex> trail(const QString &id, double nowMs) const;
    void clear() { m_tracks.clear(); }
    int count() const { return int(m_tracks.size()); }

private:
    struct Sample {
        double t = 0;                        // s
        double lat = 0, lon = 0, alt = 0;
        double vn = 0, ve = 0, vu = 0;       // m/s
    };
    struct Track {
        std::vector<Sample> samples;
        std::int64_t reportTs = 0;
    };
    QHash<QString, Track> m_tracks;
};

} // namespace corv
