#include "mission/RouteGeo.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace corv::mission {

namespace {
constexpr double kD2R = M_PI / 180.0;
constexpr double kEarthR = 6371000.0;
} // namespace

double haversine(const LL &a, const LL &b)
{
    const double dLat = (b.lat - a.lat) * kD2R;
    const double dLon = (b.lng - a.lng) * kD2R;
    const double h = std::pow(std::sin(dLat / 2), 2) + std::cos(a.lat * kD2R) * std::cos(b.lat * kD2R) * std::pow(std::sin(dLon / 2), 2);
    return 2 * kEarthR * std::asin(std::sqrt(std::min(1.0, h)));
}

double bearing(const LL &a, const LL &b)
{
    const double p1 = a.lat * kD2R, p2 = b.lat * kD2R, dl = (b.lng - a.lng) * kD2R;
    const double y = std::sin(dl) * std::cos(p2);
    const double x = std::cos(p1) * std::sin(p2) - std::sin(p1) * std::cos(p2) * std::cos(dl);
    return std::fmod(std::atan2(y, x) / kD2R + 360.0, 360.0);
}

LocalFrame::LocalFrame(const LL &o) : origin(o), kLat(111320), kLng(111320 * std::cos(o.lat * kD2R)) {}

std::optional<LL> centroid(const QList<LL> &points)
{
    if (points.isEmpty()) return std::nullopt;
    LL c;
    for (const LL &p : points) { c.lat += p.lat; c.lng += p.lng; }
    c.lat /= points.size();
    c.lng /= points.size();
    return c;
}

double polygonArea(const QList<LL> &points)
{
    if (points.size() < 3) return 0;
    const LocalFrame f(*centroid(points));
    QList<XY> xy;
    for (const LL &p : points) xy.append(f.toXY(p));
    double a = 0;
    for (qsizetype i = 0, j = xy.size() - 1; i < xy.size(); j = i++) a += xy[j].x * xy[i].y - xy[i].x * xy[j].y;
    return std::abs(a) / 2;
}

double pathLength(const QList<LL> &points)
{
    double d = 0;
    for (qsizetype i = 1; i < points.size(); ++i) d += haversine(points[i - 1], points[i]);
    return d;
}

Fov cameraFov(const QJsonObject &cam)
{
    const double f = std::max(0.1, num(cam.value(QStringLiteral("focal")), 4.2) ? num(cam.value(QStringLiteral("focal")), 4.2) : 4.2);
    const double w = std::max(0.1, num(cam.value(QStringLiteral("sensorW")), 6.17) ? num(cam.value(QStringLiteral("sensorW")), 6.17) : 6.17);
    const double h = std::max(0.1, num(cam.value(QStringLiteral("sensorH")), 4.63) ? num(cam.value(QStringLiteral("sensorH")), 4.63) : 4.63);
    return {2 * std::atan(w / (2 * f)) / kD2R, 2 * std::atan(h / (2 * f)) / kD2R};
}

Footprint cameraFootprint(double alt, const QJsonObject &cam)
{
    const Fov fov = cameraFov(cam);
    return {2 * alt * std::tan(fov.fovH * kD2R / 2), 2 * alt * std::tan(fov.fovV * kD2R / 2)};
}

Swath sensorSwath(double apertureDeg, double range, double distance)
{
    const double d = std::max(0.1, distance);
    if (!(range > d)) return {0, true};
    const double byAngle = d * std::tan(std::min(179.0, apertureDeg) / 2 * kD2R);
    const double byRange = std::sqrt(range * range - d * d);
    return {2 * std::min(byAngle, byRange), byRange < byAngle};
}

SurveyGeometry surveyGeometry(const QJsonObject &seg, double alt, const QJsonObject &cam)
{
    const QJsonObject p = seg.value(QStringLiteral("params")).toObject();
    const QString mode = p.value(QStringLiteral("spacingMode")).toString();
    SurveyGeometry g;
    if (mode == QLatin1String("sensor")) {
        // A sonar or a LiDAR: lanes from its swath on the surface it maps
        const double sd = num(p.value(QStringLiteral("sensorDistance")));
        const double distance = sd > 0 ? sd : std::max(1.0, alt);
        const double ap = num(p.value(QStringLiteral("sensorAperture")));
        const double rg = num(p.value(QStringLiteral("sensorRange")));
        g.sensor = sensorSwath(ap ? ap : 90, rg ? rg : 90, distance);
        g.hasSensor = true;
        g.sensorDistance = distance;
        g.sideDistance = std::max(1.0, g.sensor.swath * (1 - num(p.value(QStringLiteral("sideOverlap"))) / 100));
        g.footprint = {g.sensor.swath, 0};
        return g;
    }
    g.footprint = cameraFootprint(std::max(1.0, alt), cam);
    const double side = num(p.value(QStringLiteral("sideDistance")));
    g.sideDistance = mode == QLatin1String("manual") ? std::max(1.0, side ? side : 30)
                                                     : std::max(1.0, g.footprint.width * (1 - num(p.value(QStringLiteral("sideOverlap"))) / 100));
    g.triggerDistance = std::max(0.5, g.footprint.height * (1 - num(p.value(QStringLiteral("forwardOverlap"))) / 100));
    const double imgW = num(cam.value(QStringLiteral("imgW")));
    if (imgW > 0) g.gsd = g.footprint.width / imgW * 100;
    return g;
}

QList<LL> groundFootprint(const LL &pos, double alt, double azimuthDeg, double pitchDeg, const QJsonObject &cam, double maxRange)
{
    if (!(alt > 0.5)) return {};
    if (maxRange < 0) maxRange = 8 * std::max(alt, 10.0);
    const Fov fov = cameraFov(cam);
    const double az = azimuthDeg * kD2R, p = std::clamp(pitchDeg, -90.0, 89.0) * kD2R;
    struct V { double x, y, z; };
    const V f{std::sin(az), std::cos(az), 0};
    const V r{std::cos(az), -std::sin(az), 0};
    const V d{f.x * std::cos(p), f.y * std::cos(p), std::sin(p)};
    const V v{-f.x * std::sin(p), -f.y * std::sin(p), std::cos(p)};
    const double th = std::tan(fov.fovH * kD2R / 2), tv = std::tan(fov.fovV * kD2R / 2);
    auto corner = [&](double sr, double sv, XY &out) {
        const V ray{d.x + sr * th * r.x + sv * tv * v.x, d.y + sr * th * r.y + sv * tv * v.y, d.z + sr * th * r.z + sv * tv * v.z};
        const double t = ray.z < -1e-6 ? alt / -ray.z : std::numeric_limits<double>::infinity();
        const double horiz = std::hypot(ray.x, ray.y);
        const double dist = std::min(t * horiz, maxRange);
        if (!(dist >= 0) || horiz < 1e-9) return false;
        out = {ray.x / horiz * dist, ray.y / horiz * dist};
        return true;
    };
    XY q[4];
    if (!corner(-1, -1, q[0]) || !corner(1, -1, q[1]) || !corner(1, 1, q[2]) || !corner(-1, 1, q[3])) return {};
    const LocalFrame frame(pos);
    return {frame.toLL(q[0]), frame.toLL(q[1]), frame.toLL(q[2]), frame.toLL(q[3])};
}

Aim aimAt(const LL &from, double altAgl, const LL &to, double targetH)
{
    const LocalFrame frame(from);
    const XY q = frame.toXY(to);
    const double horiz = std::hypot(q.x, q.y);
    return {std::fmod(std::atan2(q.x, q.y) / kD2R + 360, 360), -std::atan2(altAgl - targetH, horiz) / kD2R, horiz};
}

QList<PhotoPos> photosAlong(const QList<LL> &points, double spacing)
{
    QList<PhotoPos> out;
    if (points.size() < 2 || !(spacing > 0)) return out;
    double carry = 0;
    for (qsizetype i = 1; i < points.size(); ++i) {
        const LL &a = points[i - 1], &b = points[i];
        const double len = haversine(a, b);
        const double heading = bearing(a, b);
        if (i == 1) { out.append({a.lat, a.lng, heading}); carry = 0; }
        double pos = spacing - carry;
        while (pos <= len) {
            const double t = len > 0 ? pos / len : 0;
            out.append({a.lat + (b.lat - a.lat) * t, a.lng + (b.lng - a.lng) * t, heading});
            pos += spacing;
        }
        carry = len - (pos - spacing);
    }
    return out;
}

} // namespace corv::mission
