#include "scene/GeoZone.h"

#include <QDateTime>
#include <QHash>
#include <QRegularExpression>
#include <algorithm>
#include <cmath>

using namespace Qt::StringLiterals;

namespace corv {

namespace {
constexpr double kPiD = 3.14159265358979323846;

QJsonArray polysToJson(const std::vector<GeoZone::Polygon> &polys)
{
    QJsonArray out;
    for (const auto &rings : polys) {
        QJsonArray r;
        for (const auto &ring : rings) {
            QJsonArray a;
            for (double v : ring) a.append(v);
            r.append(a);
        }
        out.append(r);
    }
    return out;
}

void computeBbox(GeoZone &z)
{
    double w = 180, s = 90, e = -180, n = -90;
    for (const auto &rings : z.polys) {
        if (rings.empty()) continue;
        const auto &r = rings[0];
        for (std::size_t i = 0; i + 1 < r.size(); i += 2) {
            w = std::min(w, r[i]); e = std::max(e, r[i]);
            s = std::min(s, r[i + 1]); n = std::max(n, r[i + 1]);
        }
    }
    z.bbox[0] = w; z.bbox[1] = s; z.bbox[2] = e; z.bbox[3] = n;
}

QString stripHtml(QString s)
{
    static const QRegularExpression tags(u"<[^>]*>"_s), spaces(u"\\s+"_s);
    s.replace(tags, u" "_s).replace(u"&nbsp;"_s, u" "_s).replace(spaces, u" "_s);
    return s.trimmed().left(300);
}

/// An ED-269 applicability (or ED-318 limitedApplicability) that has already ended.
bool expired(const QJsonValue &applicability)
{
    QJsonArray list;
    if (applicability.isArray()) list = applicability.toArray();
    else if (applicability.isObject()) list.append(applicability);
    if (list.isEmpty()) return false;
    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    for (const QJsonValue &v : list) {
        const QJsonObject a = v.toObject();
        if (a.value(u"permanent"_s).toString().toUpper() == "YES"_L1) return false;
        QString endText = a.value(u"endDateTime"_s).toString();
        if (endText.isEmpty()) endText = a.value(u"endDate"_s).toString();
        const QDateTime end = QDateTime::fromString(endText, Qt::ISODate);
        if (!end.isValid() || end.toMSecsSinceEpoch() >= now) return false;
    }
    return true;
}

QString str(const QJsonObject &o, const char *k) { return o.value(QLatin1String(k)).toVariant().toString(); }
} // namespace

QJsonObject GeoZone::toJson() const
{
    return QJsonObject{{u"id"_s, id}, {u"src"_s, src}, {u"name"_s, name},
                       {u"kind"_s, kind}, {u"floor"_s, floor},
                       {u"floorRef"_s, floorAgl ? u"AGL"_s : u"MSL"_s},
                       {u"top"_s, top ? QJsonValue(*top) : QJsonValue(QJsonValue::Null)},
                       {u"topRef"_s, topAgl ? u"AGL"_s : u"MSL"_s},
                       {u"polys"_s, polysToJson(polys)},
                       {u"bbox"_s, QJsonArray{bbox[0], bbox[1], bbox[2], bbox[3]}},
                       {u"info"_s, info}};
}

std::optional<GeoZone> GeoZone::fromJson(const QJsonObject &o)
{
    GeoZone z;
    z.id = o.value(u"id"_s).toString();
    z.src = o.value(u"src"_s).toString();
    z.name = o.value(u"name"_s).toString();
    z.kind = o.value(u"kind"_s).toString();
    z.floor = o.value(u"floor"_s).toDouble();
    z.floorAgl = o.value(u"floorRef"_s).toString() != "MSL"_L1;
    if (o.value(u"top"_s).isDouble()) z.top = o.value(u"top"_s).toDouble();
    z.topAgl = o.value(u"topRef"_s).toString() != "MSL"_L1;
    z.info = o.value(u"info"_s).toString();
    for (const QJsonValue &p : o.value(u"polys"_s).toArray()) {
        GeoZone::Polygon rings;
        for (const QJsonValue &r : p.toArray()) {
            GeoZone::Ring ring;
            const QJsonArray a = r.toArray();
            ring.reserve(std::size_t(a.size()));
            for (const QJsonValue &v : a) ring.push_back(v.toDouble());
            if (ring.size() >= 8) rings.push_back(std::move(ring));
        }
        if (!rings.empty()) z.polys.push_back(std::move(rings));
    }
    if (z.id.isEmpty() || z.kind.isEmpty() || z.polys.empty()) return std::nullopt;
    const QJsonArray b = o.value(u"bbox"_s).toArray();
    if (b.size() == 4) for (int i = 0; i < 4; ++i) z.bbox[i] = b[i].toDouble();
    else computeBbox(z);
    return z;
}

namespace geozone {

GeoZone::Ring simplifyRing(const QJsonArray &coords, double tol)
{
    const int n = int(coords.size());
    if (n < 4) return {};
    std::vector<double> xs(static_cast<std::size_t>(n)), ys(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        const QJsonArray p = coords[i].toArray();
        xs[std::size_t(i)] = p.size() > 0 ? p[0].toDouble() : 0;
        ys[std::size_t(i)] = p.size() > 1 ? p[1].toDouble() : 0;
    }
    const double k = std::cos(ys[0] * kPiD / 180);
    std::vector<char> keep(std::size_t(n), 0);
    keep[0] = keep[std::size_t(n - 1)] = 1;
    std::vector<std::pair<int, int>> stack{{0, n - 1}};
    while (!stack.empty()) {
        const auto [a, b] = stack.back();
        stack.pop_back();
        const double ax = xs[std::size_t(a)] * k, ay = ys[std::size_t(a)];
        const double dx = xs[std::size_t(b)] * k - ax, dy = ys[std::size_t(b)] - ay;
        const double len2 = dx * dx + dy * dy;
        int worst = -1;
        double worstD = tol * tol;
        for (int i = a + 1; i < b; ++i) {
            const double px = xs[std::size_t(i)] * k - ax, py = ys[std::size_t(i)] - ay;
            double d2;
            if (len2 == 0) {
                d2 = px * px + py * py;
            } else {
                const double t = std::clamp((px * dx + py * dy) / len2, 0.0, 1.0);
                const double ex = px - t * dx, ey = py - t * dy;
                d2 = ex * ex + ey * ey;
            }
            if (d2 > worstD) { worstD = d2; worst = i; }
        }
        if (worst > 0) {
            keep[std::size_t(worst)] = 1;
            stack.push_back({a, worst});
            stack.push_back({worst, b});
        }
    }
    GeoZone::Ring out;
    for (int i = 0; i < n; ++i)
        if (keep[std::size_t(i)]) {
            out.push_back(std::round(xs[std::size_t(i)] * 1e6) / 1e6);
            out.push_back(std::round(ys[std::size_t(i)] * 1e6) / 1e6);
        }
    return out.size() >= 8 ? out : GeoZone::Ring{};   // 3 corners and the closing point
}

std::vector<GeoZone::Polygon> geojsonPolys(const QJsonObject &geometry)
{
    std::vector<GeoZone::Polygon> polys;
    const QString type = geometry.value(u"type"_s).toString();
    QJsonArray list;
    if (type == "Polygon"_L1) list.append(geometry.value(u"coordinates"_s));
    else if (type == "MultiPolygon"_L1) list = geometry.value(u"coordinates"_s).toArray();
    for (const QJsonValue &rings : list) {
        GeoZone::Polygon flat;
        for (const QJsonValue &ring : rings.toArray()) {
            GeoZone::Ring r = ring.isArray() ? simplifyRing(ring.toArray()) : GeoZone::Ring{};
            if (!r.empty()) flat.push_back(std::move(r));
            else if (flat.empty()) break;   // no outer ring, no polygon
        }
        if (!flat.empty()) polys.push_back(std::move(flat));
    }
    return polys;
}

std::vector<GeoZone::Polygon> circlePoly(double lon, double lat, double radiusM, int n)
{
    GeoZone::Ring ring;
    const double dLat = radiusM / 111320, dLon = dLat / std::max(0.01, std::cos(lat * kPiD / 180));
    for (int i = 0; i <= n; ++i) {
        const double a = double(i % n) / n * 2 * kPiD;
        ring.push_back(std::round((lon + dLon * std::sin(a)) * 1e6) / 1e6);
        ring.push_back(std::round((lat + dLat * std::cos(a)) * 1e6) / 1e6);
    }
    return {GeoZone::Polygon{ring}};
}

std::optional<double> toMetres(const QJsonValue &value, const QString &uom)
{
    bool ok = false;
    const double v = value.isDouble() ? value.toDouble() : value.toVariant().toString().toDouble(&ok);
    if (!value.isDouble() && !ok) return std::nullopt;
    if (!std::isfinite(v)) return std::nullopt;
    return (uom.isEmpty() ? u"M"_s : uom).startsWith('f'_L1, Qt::CaseInsensitive) ? v * kFt : v;
}

bool isAgl(const QString &ref)
{
    static const QRegularExpression re(u"^(AGL|SFC|GND)"_s, QRegularExpression::CaseInsensitiveOption);
    return re.match(ref.isEmpty() ? u"AGL"_s : ref).hasMatch();
}

QString restrictionKind(const QString &r)
{
    const QString v = r.toUpper();
    if (v == "PROHIBITED"_L1 || v == "REQ_AUTHORISATION"_L1 || v == "REQ_AUTHORIZATION"_L1)
        return u"prohibited"_s;
    if (v == "NO_RESTRICTION"_L1) return {};
    return u"conditional"_s;
}

std::optional<GeoZone> makeZone(const QString &src, const QString &key, const QString &name, const QString &kind, double floor,
                                bool floorAgl, std::optional<double> top, bool topAgl, std::vector<GeoZone::Polygon> polys,
                                const QString &info)
{
    if (kind.isEmpty() || polys.empty()) return std::nullopt;
    if (top && *top >= kUnlimitedM) top.reset();
    GeoZone z;
    z.id = src + ':'_L1 + key;
    z.src = src;
    z.name = (name.trimmed().isEmpty() ? key : name).trimmed().left(120);
    z.kind = kind;
    z.floor = std::max(0.0, std::round(floor * 10) / 10);
    z.floorAgl = floorAgl;
    if (top) z.top = std::round(*top * 10) / 10;
    z.topAgl = topAgl;
    z.polys = std::move(polys);
    computeBbox(z);
    z.info = stripHtml(info);
    return z;
}

GeoZoneList parseEdZones(const QJsonValue &json, const QString &src)
{
    QJsonArray features;
    if (json.isArray()) features = json.toArray();
    else features = json.toObject().value(u"features"_s).toArray();
    GeoZoneList zones;
    int n = 0;
    for (const QJsonValue &fv : features) {
        ++n;
        const QJsonObject f = fv.toObject();
        if (f.isEmpty()) continue;
        if (f.value(u"geometry"_s).isArray()) {
            // ED-269: one volume per geometry element
            if (expired(f.value(u"applicability"_s))) continue;
            const QString kind = restrictionKind(str(f, "restriction"));
            QString base = str(f, "identifier");
            if (base.isEmpty()) base = str(f, "name");
            if (base.isEmpty()) base = u"z%1"_s.arg(n);
            const QJsonArray geoms = f.value(u"geometry"_s).toArray();
            QString info = str(f, "message");
            if (info.isEmpty() && f.value(u"restrictionConditions"_s).isArray()) {
                QStringList parts;
                for (const QJsonValue &c : f.value(u"restrictionConditions"_s).toArray()) parts << c.toVariant().toString();
                info = parts.join(' '_L1);
            }
            for (int i = 0; i < geoms.size(); ++i) {
                const QJsonObject g = geoms[i].toObject();
                const QJsonObject hp = g.value(u"horizontalProjection"_s).toObject();
                std::vector<GeoZone::Polygon> polys;
                if (hp.value(u"type"_s).toString() == "Circle"_L1 && hp.value(u"center"_s).isArray()) {
                    const QJsonArray c = hp.value(u"center"_s).toArray();
                    polys = circlePoly(c[0].toDouble(), c[1].toDouble(), hp.value(u"radius"_s).toDouble());
                } else {
                    polys = geojsonPolys(hp);
                }
                const QString uom = str(g, "uomDimensions");
                QString name = str(f, "name");
                if (name.isEmpty()) name = str(f, "identifier");
                auto z = makeZone(src, geoms.size() > 1 ? u"%1#%2"_s.arg(base).arg(i) : base, name, kind,
                                  toMetres(g.value(u"lowerLimit"_s), uom).value_or(0), isAgl(str(g, "lowerVerticalReference")),
                                  toMetres(g.value(u"upperLimit"_s), uom), isAgl(str(g, "upperVerticalReference")),
                                  std::move(polys), info);
                if (z) zones.push_back(std::move(*z));
            }
        } else {
            // ED-318 / GeoJSON
            const QJsonObject p = f.value(u"properties"_s).toObject();
            if (expired(p.contains(u"limitedApplicability"_s) ? p.value(u"limitedApplicability"_s)
                                                                         : p.value(u"applicability"_s)))
                continue;
            const QJsonObject v = p.value(u"verticalLayer"_s).isObject() ? p.value(u"verticalLayer"_s).toObject() : p;
            QString uom = str(v, "uom");
            if (uom.isEmpty()) uom = str(v, "uomDimensions");
            if (uom.isEmpty()) uom = u"M"_s;
            auto pick = [&](const char *a, const char *b) { return v.contains(QLatin1String(a)) ? v.value(QLatin1String(a)) : v.value(QLatin1String(b)); };
            QString key = str(p, "identifier");
            if (key.isEmpty()) key = f.value(u"id"_s).toVariant().toString();
            if (key.isEmpty()) key = u"z%1"_s.arg(n);
            QString name = str(p, "name");
            if (name.isEmpty()) name = str(p, "identifier");
            QString restriction = str(p, "restriction");
            if (restriction.isEmpty()) restriction = str(p, "type");
            QString info = str(p, "message");
            if (info.isEmpty()) info = str(p, "otherReasonInfo");
            auto z = makeZone(src, key, name, restrictionKind(restriction), toMetres(pick("lower", "lowerLimit"), uom).value_or(0),
                              isAgl(pick("lowerReference", "lowerVerticalReference").toString()),
                              toMetres(pick("upper", "upperLimit"), uom), isAgl(pick("upperReference", "upperVerticalReference").toString()),
                              geojsonPolys(f.value(u"geometry"_s).toObject()), info);
            if (z) zones.push_back(std::move(*z));
        }
    }
    return zones;
}

// ── Rules ────────────────────────────────────────────────────────────────────

namespace {
// EASA member states with the UK, Switzerland, Norway and Iceland (and the Balkans), [lon, lat]
const GeoZone::Ring &europe()
{
    static const GeoZone::Ring r = {
        -32, 32, -12, 32, -9.5, 35.5, -6.2, 35.85, -5.3, 35.95, -2, 36.25, 8, 38, 10.5, 37.7,
        11.6, 36.95, 11.8, 36.6, 12.2, 35.3, 15.5, 35.3, 23, 34.6, 26.5, 34.8, 32, 34.4, 34.7, 34.4,
        34.7, 35.75, 32.5, 35.75, 28.6, 36.2, 28.45, 36.55, 27.5, 36.72, 27.4, 36.95, 27, 37.75, 26.25, 38.3, 26.62, 39.05,
        25.95, 39.7, 25.9, 40.3, 26.05, 40.75, 26.35, 41.25, 26.6, 41.7, 28, 42, 29.8, 45.2,
        28.2, 45.45, 26.6, 48.25, 22.9, 48, 22.5, 49, 24.1, 50.6, 23.6, 51.5, 23.6, 53.9, 25, 54.2,
        26.6, 55.6, 28.2, 56.2, 27.7, 57.4, 28.2, 59.45, 27.7, 60.5, 29.5, 61.5, 31.6, 62.9,
        30, 65.5, 29, 67, 28.6, 68.5, 29.3, 69.7, 31.5, 69.9, 32, 72, -12, 72, -12, 67.5,
        -25, 67.5, -25, 63, -32, 40};
    return r;
}
bool inBox(const double b[4], double lon, double lat) { return lon >= b[0] && lon <= b[2] && lat >= b[1] && lat <= b[3]; }
constexpr double kHysteresisM = 1;
QString shortName(const GeoZone &z) { return z.name.size() > 18 ? z.name.left(17) + u"…"_s : z.name; }
QString fmtM(double m) { return u"%1 m"_s.arg(qRound(m)); }
} // namespace

QString regionAt(double lat, double lon)
{
    static const double us[4][4] = {{-125, 24.4, -66.8, 49.4}, {-170, 51, -129, 71.5}, {-161, 18.8, -154.7, 22.3}, {-67.4, 17.8, -65.2, 18.6}};
    for (const auto &b : us)
        if (inBox(b, lon, lat)) return u"US"_s;
    static const double canaries[4] = {-18.5, 27.5, -13.3, 29.5};
    if (inBox(canaries, lon, lat) || pointInRing(europe(), lon, lat)) return u"EU"_s;
    return {};
}

std::optional<Limit> generalLimit(const QString &region)
{
    if (region == "EU"_L1) return Limit{120, u"120 m"_s, u"EU open category (Reg. 2019/947): 120 m above the ground"_s};
    if (region == "US"_L1) return Limit{400 * kFt, u"400 ft"_s, u"FAA Part 107 / recreational: 400 ft above the ground"_s};
    return std::nullopt;
}

bool pointInRing(const GeoZone::Ring &ring, double lon, double lat)
{
    bool inside = false;
    const std::size_t n = ring.size();
    if (n < 6) return false;
    for (std::size_t i = 0, j = n - 2; i < n; j = i, i += 2) {
        const double xi = ring[i], yi = ring[i + 1], xj = ring[j], yj = ring[j + 1];
        if ((yi > lat) != (yj > lat) && lon < (xj - xi) * (lat - yi) / (yj - yi) + xi) inside = !inside;
    }
    return inside;
}

bool pointInZone(const GeoZone &z, double lat, double lon)
{
    if (!inBox(z.bbox, lon, lat)) return false;
    for (const auto &rings : z.polys) {
        if (rings.empty() || !pointInRing(rings[0], lon, lat)) continue;
        bool hole = false;
        for (std::size_t k = 1; k < rings.size() && !hole; ++k) hole = pointInRing(rings[k], lon, lat);
        if (!hole) return true;
    }
    return false;
}

std::optional<double> heightAgl(std::optional<double> value, bool agl, std::optional<double> terrain)
{
    if (!value) return std::nullopt;
    if (agl) return value;
    if (!terrain) return std::nullopt;
    return *value - *terrain;
}

Evaluation evaluate(const GeoZoneList &zones, double lat, double lon, std::optional<double> agl, std::optional<double> terrain,
                    bool airborne)
{
    Evaluation r;
    r.region = regionAt(lat, lon);
    r.limit = generalLimit(r.region);
    for (const GeoZone &z : zones)
        if (pointInZone(z, lat, lon)) r.here.push_back(&z);
    const GeoZone *noFly = nullptr, *cond = nullptr, *ceilZone = nullptr, *maxZone = nullptr;
    double ceilFloor = 0, maxFloor = 0;
    for (const GeoZone *z : r.here) {
        const auto floor = heightAgl(z->floor, z->floorAgl, terrain);
        const auto top = heightAgl(z->top, z->topAgl, terrain);
        if (!floor) continue;
        if (agl && top && *agl > *top) continue;   // over the zone
        if (*floor <= 0.5) {
            if (!airborne) continue;
            if (z->prohibited()) { if (!noFly) noFly = z; }
            else if (!cond) cond = z;
        } else if (agl && *agl > *floor + kHysteresisM) {
            // The broken ceiling shown: a prohibited zone before a conditional one, then the lowest
            const bool p = z->prohibited(), cp = ceilZone && ceilZone->prohibited();
            if (!ceilZone || (p && !cp) || (p == cp && *floor < ceilFloor)) { ceilZone = z; ceilFloor = *floor; }
        } else if (!maxZone || *floor < maxFloor) {
            maxZone = z;
            maxFloor = *floor;
        }
    }
    const bool overLimit = r.limit && agl && *agl > r.limit->metres + kHysteresisM;
    if (noFly) r.alerts.push_back({u"noFly"_s, u"warn"_s, u"zone"_s, u"NO-FLY"_s, shortName(*noFly), {}, noFly, 0});
    if (ceilZone)
        r.alerts.push_back({u"ceiling"_s, ceilZone->prohibited() ? u"warn"_s : u"caut"_s, u"ceiling"_s,
                            u"MAX "_s + fmtM(ceilFloor), shortName(*ceilZone), {}, ceilZone, ceilFloor});
    if (overLimit)
        r.alerts.push_back({u"altLimit"_s, u"warn"_s, u"ceiling"_s, u"MAX "_s + r.limit->label,
                            u"ALT LIMIT"_s, r.limit->rule, nullptr, r.limit->metres});
    if (cond) r.alerts.push_back({u"cond"_s, u"caut"_s, u"zone"_s, u"ZONE"_s, shortName(*cond), {}, cond, 0});
    if (maxZone && !ceilZone && (!r.limit || maxFloor < r.limit->metres))
        r.alerts.push_back({u"maxHere"_s, u"caut"_s, u"ceiling"_s, u"MAX "_s + fmtM(maxFloor),
                            shortName(*maxZone), {}, maxZone, maxFloor});
    r.overHeight = overLimit || (ceilZone && ceilZone->prohibited());
    return r;
}

std::optional<ArmAlert> armingAlert(const std::vector<const GeoZone *> &here, std::optional<double> terrain, std::optional<Limit> limit)
{
    const GeoZone *noFly = nullptr, *cond = nullptr, *ceil = nullptr;
    double ceilFloor = 0;
    for (const GeoZone *z : here) {
        const auto floor = heightAgl(z->floor, z->floorAgl, terrain);
        if (!floor) continue;
        if (*floor <= 0.5) {
            if (z->prohibited()) { if (!noFly) noFly = z; }
            else if (!cond) cond = z;
        } else if ((!limit || *floor < limit->metres) && (!ceil || *floor < ceilFloor)) {
            ceil = z;
            ceilFloor = *floor;
        }
    }
    if (noFly) return ArmAlert{u"warn"_s, u"ARMED IN A NO-FLY ZONE"_s, noFly->name + u" — no flight without authorisation"_s, noFly};
    if (cond) return ArmAlert{u"caut"_s, u"ARMED IN A RESTRICTED ZONE"_s, cond->name + u" — conditions apply"_s, cond};
    if (ceil)
        return ArmAlert{u"caut"_s, u"ARMED UNDER A %1 CEILING"_s.arg(fmtM(ceilFloor)),
                        u"%1 — max %2 above the ground"_s.arg(ceil->name, fmtM(ceilFloor)), ceil};
    return std::nullopt;
}

// ── Palette ──────────────────────────────────────────────────────────────────

QColor prohibitedColor() { return QColor(232, 35, 29); }
QColor conditionalColor() { return QColor(176, 96, 255); }

QColor ceilingColor(double m)
{
    static const struct { double m; int r, g, b; } stops[] = {{0, 255, 64, 24}, {15, 255, 84, 26}, {30, 255, 140, 26},
                                                              {60, 255, 196, 26}, {90, 226, 232, 30}, {120, 140, 222, 60}};
    if (m <= stops[0].m) return QColor(stops[0].r, stops[0].g, stops[0].b);
    for (std::size_t i = 1; i < std::size(stops); ++i) {
        if (m <= stops[i].m) {
            const double t = (m - stops[i - 1].m) / (stops[i].m - stops[i - 1].m);
            auto lerp = [t](int a, int b) { return int(std::lround(a + (b - a) * t)); };
            return QColor(lerp(stops[i - 1].r, stops[i].r), lerp(stops[i - 1].g, stops[i].g), lerp(stops[i - 1].b, stops[i].b));
        }
    }
    const auto &l = stops[std::size(stops) - 1];
    return QColor(l.r, l.g, l.b);
}

QString sourceName(const QString &src)
{
    static const QHash<QString, QString> names = {{u"faa"_s, u"FAA"_s}, {u"dipul"_s, u"DFS DIPUL"_s},
                                                  {u"ign"_s, u"IGN / DGAC"_s}, {u"enaire"_s, u"ENAIRE"_s},
                                                  {u"bazl"_s, u"FOCA / BAZL"_s}};
    if (names.contains(src)) return names.value(src);
    if (src.startsWith("import-"_L1)) return u"imported "_s + src.mid(7);
    return src;
}

} // namespace geozone
} // namespace corv
