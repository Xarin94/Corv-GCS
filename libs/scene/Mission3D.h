// Mission3D.h - the shape of the mission route in the 3D view (js/engine/Mission3D.js).
//
// Drawn the way the flight plan page draws it, so the two views read alike:
//   - between segments and through plain waypoints, a solid line in the route
//     colour, with a symbol, a drop line and a label on every waypoint;
//   - a pattern (area scan, corridor, perimeter): its passes as a thinner dashed
//     line in the segment's colour, marked only where the vehicle enters and
//     leaves it (a survey compiles to hundreds of waypoints);
//   - a circle: the loiter ring, with arrows for the direction of turn. The route
//     meets the ring at the point nearest to where the vehicle comes from and
//     leaves it from there, as ArduPilot flies it, never through the centre;
//   - a landing: across at height, then straight down (a fixed wing's NAV_LAND
//     keeps its glide);
//   - return to launch: a dashed line back over home, then down;
//   - a point of interest: a target at the height the camera aims at, on a pole.
// Geometry only, in world metres (x east, y up, z south): no graphics API.
#pragma once

#include <QString>
#include <QVector3D>
#include <optional>
#include <vector>

namespace corv::mission3d {

/// A located navigation item.
struct Point {
    QVector3D pos;
    std::optional<float> ground;   // terrain height under it
    int seq = 0;
    QString kind;                  // home | takeoff | land | loiter | wp
    bool derived = false;          // a terrain-following vertex the compiler inserted: path only
    QString segId, segType;        // of the planner: area | corridor | perimeter | circle...
    double loiterRadius = 0;
    int turns = 0;
    bool ccw = false;
    bool glide = false;            // a landing flown down a slope
    double alt = 0, hold = 0;      // as the item states them
};

struct Poi {
    QVector3D pos;                 // at the height the camera aims at
    std::optional<float> ground;
    int seq = 0;
};

enum class Style { Route, Pattern, Ring, Return };

struct Line {
    QVector3D a, b;
    Style style = Style::Route;
    QString segType;               // of a Pattern line: its colour
    int seq = 0;                   // the item it leads to (the leg being flown)
};

enum class Shape { None, Square, Triangle, Diamond, Circle, Target };

/// A marked point: symbol, label, and the drop line to the ground under it.
struct Mark {
    QVector3D at;
    Shape shape = Shape::None;
    QString segType;               // colour: circle, area... "poi", or empty for the route's
    QString main, sub;
    bool plain = false;            // a plain waypoint: labelled one in n on a long list
    std::optional<float> ground;   // none: no drop line
    int seq = -1;
};

struct Drawing {
    std::vector<Line> lines;
    std::vector<Mark> marks;
};

/// `rtlSeq`: the item of a closing RETURN_TO_LAUNCH, if any.
Drawing build(const std::vector<Point> &points, const std::vector<Poi> &pois, std::optional<int> rtlSeq);

bool isPattern(const QString &segType);

} // namespace corv::mission3d
