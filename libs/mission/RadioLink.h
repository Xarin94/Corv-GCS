// RadioLink.h - ground ↔ aircraft radio link over the terrain (port of js/mission/RadioLink.js).
//
// Pure link-budget and geometry maths. The Flight Plan page asks three questions:
//   evaluateLink()    one point: received power, margin, line of sight and
//                     first-Fresnel-zone clearance against the SRTM terrain
//   analyzeRoute()    the same along the calculated flight path, as runs of
//                     equal link quality for the map halo and the profile band
//   computeCoverage() a polar raster around the operator at the planned
//                     altitude, for the coverage overlay on the map
//
// Model: free-space path loss, single knife-edge diffraction (ITU-R P.526) on
// the sample that intrudes most into the first Fresnel zone, 4/3 earth radius,
// omnidirectional antennas.
#pragma once

#include "mission/RouteCompiler.h"

#include <QColor>
#include <QJsonObject>
#include <QList>
#include <atomic>
#include <functional>
#include <optional>
#include <vector>

namespace corv::mission {

enum class Link : int { None = 0, Marginal = 1, Degraded = 2, Good = 3, Unknown = 4 };

struct LinkStyle {
    QString label, shortName;
    QColor color, fill;
};
const LinkStyle &linkStyle(Link c);

double wavelength(double fMHz);
double fspl(double dM, double fMHz);
double fresnelRadius(double d1, double d2, double fMHz);
double knifeEdgeLoss(double nu);
double systemGain(const QJsonObject &radio);
/// Distance (m) at which a free-space path leaves exactly `marginDb` above the sensitivity.
double freeSpaceRange(const QJsonObject &radio, double marginDb = 0);

struct GroundStation {
    double lat = 0, lng = 0;
    std::optional<double> elev, antennaMsl;
    double height = 0;
};
GroundStation groundStation(const QJsonObject &radio, const LL &pos, const TerrainFn &terrain);

struct LinkProfile {
    int n = 0;
    double step = 0, h0 = 0, h1 = 0, D = 0, lambda = 0, freq = 0;
    std::vector<double> terr;
};
struct KnifeEdge {
    double dist = 0, elev = 0, line = 0, clearance = 0, r1 = 0;
};
struct LinkEval {
    double dist = 0;
    Link clazz = Link::Unknown;
    double rssi = 0, margin = 0, nu = 0, loss = 0, fspl = 0;
    bool obstructed = false, fresnelClear = false;
    std::optional<KnifeEdge> worst;
    std::optional<LinkProfile> profile;
};
LinkEval evaluateLink(const QJsonObject &radio, const GroundStation &gs, double lat, double lng, double altMsl,
                      const TerrainFn &terrain, double step = 30, int maxSamples = 600, bool keepProfile = false);

struct LinkSample {
    double dist = 0, lat = 0, lng = 0, altMsl = 0;
    QString segId;
    Link clazz = Link::Unknown;
    double margin = 0, rssi = 0, nu = 0, loss = 0, gsDist = 0;
};
struct LinkRun {
    Link clazz = Link::Unknown;
    double from = 0, to = 0;
    QList<LL> points;
};
struct LinkAnalysis {
    QList<LinkSample> samples;
    QList<LinkRun> runs;
    double lengths[5] = {0, 0, 0, 0, 0};
    std::optional<LinkSample> worst;
    double total = 0, maxGsDist = 0;
};
std::optional<LinkAnalysis> analyzeRoute(const QJsonObject &radio, const GroundStation &gs, const QList<NavPoint> &navPath,
                                         const TerrainFn &terrain, double profileStep = 30);
/// "no link 1.2 km · marginal 400 m", or "good link along the whole route".
QString summarizeLink(const LinkAnalysis &a);

struct Coverage {
    int rays = 0, count = 0;
    double step = 0, range = 0;
    std::vector<uint8_t> classes;
    LL gs;
};
/// Polar coverage around the antenna; `altAt(terrainElev)` is the planned MSL over a cell.
/// Runs synchronously (call it from a worker); `cancel` set abandons it (nullopt).
std::optional<Coverage> computeCoverage(const QJsonObject &radio, const GroundStation &gs, const std::function<double(double)> &altAt,
                                        const TerrainFn &terrain, double range, const std::atomic<bool> *cancel = nullptr);
Link coverageClassAt(const Coverage &cov, double xEast, double yNorth);

} // namespace corv::mission
