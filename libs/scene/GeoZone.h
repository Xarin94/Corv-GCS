// GeoZone.h - UAS geographical zones: the record, the geometry, the rules and
// the palette (port of the pure parts of geozones.js, js/safety/GeoZoneRules.js
// and js/safety/GeoZoneStyle.js).
//
// A zone is a restricted volume, floor and top each AGL or MSL, over one or
// more polygons of flat [lon, lat, ...] rings (outer first, then holes), kind
// "prohibited" (PROHIBITED, REQ_AUTHORISATION) or "conditional" (CONDITIONAL,
// part time, by NOTAM). A floor above the ground is the height that may be
// flown up to. The JSON form is the one Electron caches under
// <data root>/geozones, so the two versions share the cache.
//
// General height limit: 120 m above the ground in the EASA states (Regulation
// (EU) 2019/947, open category; the UK, Switzerland and Norway alike), 400 ft
// in the United States (14 CFR 107.51).
#pragma once

#include <QColor>
#include <QJsonArray>
#include <QJsonObject>
#include <QString>
#include <memory>
#include <optional>
#include <vector>

namespace corv {

struct GeoZone {
    using Ring = std::vector<double>;            // lon, lat, lon, lat... closed
    using Polygon = std::vector<Ring>;           // outer ring, then holes
    QString id, src, name, kind;                 // kind: prohibited | conditional
    double floor = 0;
    bool floorAgl = true;
    std::optional<double> top;                   // none: no top
    bool topAgl = true;
    std::vector<Polygon> polys;
    double bbox[4] = {180, 90, -180, -90};       // west, south, east, north
    QString info;

    bool prohibited() const { return kind == QLatin1String("prohibited"); }
    QJsonObject toJson() const;
    static std::optional<GeoZone> fromJson(const QJsonObject &o);
};

using GeoZoneList = std::vector<GeoZone>;
using GeoZonesPtr = std::shared_ptr<const GeoZoneList>;

namespace geozone {

constexpr double kFt = 0.3048;
constexpr double kUnlimitedM = 30000;            // a top above this is no top ("99999", FL999)
constexpr double kSimplifyDeg = 3e-5;            // ~3 m

// ── Geometry and normalisation (geozones.js) ──
/// Douglas-Peucker on [[lon, lat], ...], lon scaled by cos(lat); empty when fewer than 3 corners survive.
GeoZone::Ring simplifyRing(const QJsonArray &coords, double tol = kSimplifyDeg);
/// GeoJSON Polygon / MultiPolygon -> polygons of flat rings.
std::vector<GeoZone::Polygon> geojsonPolys(const QJsonObject &geometry);
std::vector<GeoZone::Polygon> circlePoly(double lon, double lat, double radiusM, int n = 48);
/// Metres from a value and its unit (M, FT, Feet...); nullopt when not a number.
std::optional<double> toMetres(const QJsonValue &value, const QString &uom);
/// true AGL (AGL, SFC, GND), false MSL (AMSL, WGS84, a flight level).
bool isAgl(const QString &ref);
/// ED-269 / ED-318 restriction -> kind; empty for NO_RESTRICTION.
QString restrictionKind(const QString &r);
/// A zone from its parts; nullopt without a kind or a polygon.
std::optional<GeoZone> makeZone(const QString &src, const QString &key, const QString &name, const QString &kind, double floor,
                                bool floorAgl, std::optional<double> top, bool topAgl, std::vector<GeoZone::Polygon> polys,
                                const QString &info);
/// An ED-269 file ({features: [{restriction, geometry: [...]}]}) or an ED-318 / GeoJSON one.
GeoZoneList parseEdZones(const QJsonValue &json, const QString &src);

// ── Rules (GeoZoneRules.js) ──
struct Limit {
    double metres = 0;
    QString label, rule;
};
/// "EU", "US" or empty (no general limit known here).
QString regionAt(double lat, double lon);
std::optional<Limit> generalLimit(const QString &region);
bool pointInRing(const GeoZone::Ring &ring, double lon, double lat);
bool pointInZone(const GeoZone &z, double lat, double lon);
/// A limit in metres above the ground at a point of height terrain (nullopt: unknown).
std::optional<double> heightAgl(std::optional<double> value, bool agl, std::optional<double> terrain);

struct Alert {
    QString id, level, icon, label, value, rule;   // level: warn | caut
    const GeoZone *zone = nullptr;
    double metres = 0;
};
struct Evaluation {
    QString region;
    std::optional<Limit> limit;
    std::vector<const GeoZone *> here;
    std::vector<Alert> alerts;
    bool overHeight = false;
};
/// What the zones and the general limit say about a vehicle agl metres above ground of height terrain.
Evaluation evaluate(const GeoZoneList &zones, double lat, double lon, std::optional<double> agl, std::optional<double> terrain,
                    bool airborne);
struct ArmAlert {
    QString level, title, detail;
    const GeoZone *zone = nullptr;
};
/// The alarm on arming among the zones over the vehicle; nullopt when none restricts it.
std::optional<ArmAlert> armingAlert(const std::vector<const GeoZone *> &here, std::optional<double> terrain, std::optional<Limit> limit);

// ── Palette (GeoZoneStyle.js) ──
constexpr double kCeilingMaxM = 150;
QColor prohibitedColor();
QColor conditionalColor();
/// Height that may be flown (m AGL) -> colour, red-orange at 15 m to yellow-green at 120 m.
QColor ceilingColor(double metres);
/// "FAA", "DFS DIPUL"... or "imported <file>".
QString sourceName(const QString &src);

} // namespace geozone
} // namespace corv
