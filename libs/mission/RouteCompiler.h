// RouteCompiler.h - route → MAVLink mission items (port of js/mission/RouteCompiler.js
// and of the INAV encoding of InavMission.js).
//
// The "calculate route" step: every segment expands into navigation points
// (lawn-mower lanes for an area scan, offset passes for a corridor, a loiter
// for a circle…), the DO_/CONDITION_ items of the actions are interleaved and
// every point's AGL / AMSL / relative altitude is resolved against the
// elevation model. The vehicle flies straight from one waypoint to the next;
// only with terrain-following waypoints are legs subdivided so the straight
// segments stay within the AGL tolerance. It also produces the route card
// statistics and the warnings and errors that gate the upload.
//
// A pure function of (route, context): the controller feeds it a terrain
// lookup and the take-off point.
#pragma once

#include "mission/RouteModel.h"

#include <QHash>
#include <QJsonObject>
#include <QList>
#include <QString>
#include <functional>
#include <optional>

namespace corv::mission {

using TerrainFn = std::function<std::optional<double>(double lat, double lng)>;

struct Loiter {
    double r = 0;
    int turns = 1;
};

struct RoiTarget {
    double lat = 0, lng = 0, alt = 0;
    QString segId;
};

/// One mission item. `alt` is the height above ground once resolved (the legacy
/// shape the 3D scene and the library read); altMsl / altRel are the resolved
/// absolute and take-off-relative altitudes.
struct CompiledItem {
    int command = 16;
    double lat = 0, lng = 0;
    double alt = 0;
    bool loc = false;           // has a location (frame global)
    bool isHome = false, roi = false, landing = false, derived = false, firstLeg = false;
    QString kind;               // "takeoff" | "rtl" | ""
    QString segId, segType;
    double param1 = 0, param2 = 0, param3 = 0, param4 = 0;
    std::optional<Loiter> loiter;
    int segIdx = -1;
    QString lane;
    double gimbalPitch = -90, gimbalYaw = 0;
    std::optional<RoiTarget> roiTarget;
    double trigger = 0;
    std::optional<double> terrain;
    double altMsl = 0, altRel = 0;
    int seq = 0, frame = 0;

    LL ll() const { return {lat, lng}; }
};

struct Issue {
    QString level;   // error | warn | info
    QString text;
    QString segId;
};

struct SegStats {
    double lengthM = 0, durationS = 0;
    int waypoints = 0, photos = 0, lanes = 0;
    std::optional<double> gsd, swath;
    double area = 0;
};

struct NavPoint {
    double lat = 0, lng = 0, dist = 0, altMsl = 0;
    std::optional<double> agl, terrain;
    QString segId;
    int seq = 0;
    bool derived = false, isHome = false, landing = false;
    std::optional<Loiter> loiter;
    LL ll() const { return {lat, lng}; }
};

struct RouteStats {
    double lengthM = 0, durationS = 0;
    int items = 0, waypoints = 0, photos = 0;
    std::optional<double> minAgl, maxAgl, minAmsl, maxAmsl;
    double areaM2 = 0;
};

struct Photo {
    double lat = 0, lng = 0;
    QList<LL> poly;
    QString segId;
    bool shot = false;
};

struct Cone {
    LL from, target;
    QList<LL> poly;
    QString segId, poiId;
    bool onBasePoint = false;
};

struct CameraPreview {
    QList<Photo> photos;
    QList<Cone> cones;
    bool truncated = false;
};

struct InavWp {
    int index = 0, action = 1;
    qint32 lat = 0, lon = 0, alt = 0;   // 1e7 deg, cm
    int p1 = 0, p2 = 0, p3 = 0;
    int flag = 0;
    QString segId;
};

struct InavMission {
    QList<InavWp> wps;
    QList<Issue> issues;
};

struct Home {
    double lat = 0, lng = 0, elev = 0;
};

struct CompiledRoute {
    QList<CompiledItem> items;
    RouteStats stats;
    QList<Issue> issues;
    QList<NavPoint> navPath;
    QHash<QString, SegStats> segStats;
    std::optional<Home> home;
    CameraPreview camera;
    std::optional<InavMission> inav;
    QString platform;
};

struct CompileContext {
    TerrainFn terrain;
    std::optional<LL> home;     // take-off point when the route has none of its own
    double homeAlt = 0;         // vehicle home MSL, when no terrain under the take-off point
    int vehicleType = 0;        // MAV_TYPE
    QString platform = QStringLiteral("ardupilot");
    int itemLimit = 0;          // board-reported capacity (INAV), 0 = the catalogue figure
};

CompiledRoute compileRoute(const QJsonObject &route, const CompileContext &ctx);

/// Closed outline of a corridor band (for drawing), `width` metres wide around the centre line.
QList<LL> corridorOutline(const QList<LL> &line, double width);

/// Translate compiled mission items into an INAV waypoint list (InavMission.js).
InavMission toInavMission(const QList<CompiledItem> &items, bool amsl, int limit);

} // namespace corv::mission
