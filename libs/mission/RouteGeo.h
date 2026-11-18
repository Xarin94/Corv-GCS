// RouteGeo.h - geometry shared by the compiler and the planner UI (the helpers
// of RouteModel.js and CameraFootprint.js).
#pragma once

#include "mission/RouteModel.h"

#include <QJsonObject>
#include <QList>
#include <optional>

namespace corv::mission {

struct XY {
    double x = 0, y = 0;
};

double haversine(const LL &a, const LL &b);
/// Initial bearing a→b in degrees [0, 360).
double bearing(const LL &a, const LL &b);

/// Local planar frame around `origin`: metres east (x) and north (y).
struct LocalFrame {
    LL origin;
    double kLat = 111320, kLng = 111320;
    explicit LocalFrame(const LL &o);
    XY toXY(const LL &p) const { return {(p.lng - origin.lng) * kLng, (p.lat - origin.lat) * kLat}; }
    LL toLL(const XY &q) const { return {origin.lat + q.y / kLat, origin.lng + q.x / kLng}; }
};

std::optional<LL> centroid(const QList<LL> &points);
/// Planar polygon area in m² (shoelace in the local frame).
double polygonArea(const QList<LL> &points);
double pathLength(const QList<LL> &points);

struct Fov { double fovH = 0, fovV = 0; };
/// Horizontal / vertical field of view in degrees.
Fov cameraFov(const QJsonObject &cam);
struct Footprint { double width = 0, height = 0; };
/// Nadir footprint on the ground at `alt` metres, from the camera profile.
Footprint cameraFootprint(double alt, const QJsonObject &cam);

struct Swath { double swath = 0; bool limitedByRange = false; };
/// Width a mapping sensor covers on a flat surface at `distance` (0 when out of range).
Swath sensorSwath(double apertureDeg, double range, double distance);

struct SurveyGeometry {
    double sideDistance = 0, triggerDistance = 0;
    std::optional<double> gsd;   // cm/px
    Footprint footprint;
    bool hasSensor = false;
    Swath sensor;
    double sensorDistance = 0;
};
/// Lane spacing, trigger distance and GSD of a survey segment.
SurveyGeometry surveyGeometry(const QJsonObject &seg, double alt, const QJsonObject &cam);

/// Camera footprint projected on flat ground: [nearLeft, nearRight, farRight, farLeft], or empty.
QList<LL> groundFootprint(const LL &pos, double alt, double azimuthDeg, double pitchDeg, const QJsonObject &cam,
                          double maxRange = -1);
struct Aim { double azimuth = 0, pitch = 0, distance = 0; };
/// Gimbal aim at a point of interest from a vehicle position.
Aim aimAt(const LL &from, double altAgl, const LL &to, double targetH = 0);
struct PhotoPos { double lat, lng, heading; };
/// Photo positions along a lane every `spacing` metres, starting at the lane start.
QList<PhotoPos> photosAlong(const QList<LL> &points, double spacing);

} // namespace corv::mission
