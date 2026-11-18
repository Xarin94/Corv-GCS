// MagSamples.h - the magnetometer samples of a compass calibration, as the 3D
// view shows them (the data half of js/ui/MagCal3D.js).
//
// Every sample is a point in the vehicle's body frame (x nose, y right, z down).
// Turning the vehicle turns the Earth's field around it, so the points paint a
// sphere: offset and squashed before calibration, which is exactly what the
// calibration measures. A sphere fitted to each compass's points recentres and
// scales them, so every compass sits on the same unit sphere. The latest sample
// of the compass in focus is the marker; the nearest of the 80 sections still
// missing from its completion mask is the one to aim for.
// GUI thread; no Qt Quick.
#pragma once

#include "vehicle/GeodesicGrid.h"

#include <QVector3D>
#include <array>
#include <optional>
#include <vector>

namespace corv {

class MagSamples
{
public:
    static constexpr int kCompasses = 3;
    static constexpr int kMaxPoints = 2500;       // per compass, oldest dropped first
    static constexpr float kMinStep = 0.02f;      // closer than 2 % of the radius to the last kept one: nothing new
    static constexpr int kFitMinPoints = 30;
    static constexpr int kFitMinSections = 10;    // a fit through one patch of the sphere is a guess, not a sphere

    struct Fit {
        QVector3D center;
        float r = 1;
    };
    struct Cloud {
        std::vector<QVector3D> points;            // ring buffer, size() <= kMaxPoints
        int next = 0;
        std::optional<QVector3D> last;            // last kept
        std::optional<QVector3D> latest;          // last received
        std::optional<Fit> fit;
    };

    /// One sample, body frame, any unit (RAW_IMU sends mGauss). True when it was kept
    /// as a new point (the fits are then worth redoing). All-zero = compass absent.
    bool add(int compass, float x, float y, float z);
    /// Fit every cloud again (MagCal3D does it 400 ms after the last kept point).
    void refit();
    void clear();

    const Cloud &cloud(int compass) const { return m_clouds[std::size_t(compass)]; }
    int count(int compass) const { return int(m_clouds[std::size_t(compass)].points.size()); }

    /// Centre and radius that put a compass's points on the unit sphere. Before a
    /// fit: around the origin (RAW_IMU fields already carry the stored offsets).
    Fit frame(int compass) const;

    int focus() const { return m_focus; }
    void setFocus(int compass);
    /// Completion mask of the compass in focus; none when not calibrating.
    void setCoverage(const std::optional<geogrid::Mask> &mask);
    const std::optional<geogrid::Mask> &coverage() const { return m_mask; }

    /// The latest sample of the focused compass on the unit sphere (body frame).
    std::optional<QVector3D> marker() const;
    /// Section to aim for (nearest one missing from the mask), -1 when none.
    int target() const;
    /// Section the marker is in, -1 when none.
    int current() const;

    /// |B| of the focused compass: the fitted radius, else the latest sample's length.
    std::optional<float> field() const;
    bool fitted() const { return m_clouds[std::size_t(m_focus)].fit.has_value(); }
    /// Sections covered by the mask, -1 without one.
    int covered() const { return m_mask ? geogrid::maskCount(*m_mask) : -1; }

    /// Least-squares sphere |p|^2 = 2 c.p + d, r^2 = d + |c|^2; none when the points
    /// cover too little of the sphere or scatter too far from it.
    static std::optional<Fit> fitSphere(const std::vector<QVector3D> &pts);

private:
    std::array<Cloud, kCompasses> m_clouds;
    int m_focus = 0;
    std::optional<geogrid::Mask> m_mask;
};

} // namespace corv
