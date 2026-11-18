#include "scene/GeoZone.h"

#include <QDateTime>
#include <QHash>
#include <QRegularExpression>
#include <algorithm>
#include <cmath>

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
    static const QRegularExpression tags(QStringLiteral("<[^>]*>")), spaces(QStringLiteral("\\s+"));
    s.replace(tags, QStringLiteral(" ")).replace(QStringLiteral("&nbsp;"), QStringLiteral(" ")).replace(spaces, QStringLiteral(" "));
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
        if (a.value(QStringLiteral("permanent")).toString().toUpper() == QLatin1String("YES")) return false;
        QString endText = a.value(QStringLiteral("endDateTime")).toString();
        if (endText.isEmpty()) endText = a.value(QStringLiteral("endDate")).toString();
        const QDateTime end = QDateTime::fromString(endText, Qt::ISODate);
        if (!end.isValid() || end.toMSecsSinceEpoch() >= now) return false;
    }
    return true;
}

QString str(const QJsonObject &o, const char *k) { return o.value(QLatin1String(k)).toVariant().toString(); }
} // namespace

QJsonObject GeoZone::toJson() const
{
    return QJsonObject{{QStringLiteral("id"), id}, {QStringLiteral("src"), src}, {QStringLiteral("name"), name},
                       {QStringLiteral("kind"), kind}, {QStringLiteral("floor"), floor},
                       {QStringLiteral("floorRef"), floorAgl ? QStringLiteral("AGL") : QStringLiteral("MSL")},
                       {QStringLiteral("top"), top ? QJsonValue(*top) : QJsonValue(QJsonValue::Null)},
                       {QStringLiteral("topRef"), topAgl ? QStringLiteral("AGL") : QStringLiteral("MSL")},
                       {QStringLiteral("polys"), polysToJson(polys)},
                       {QStringLiteral("bbox"), QJsonArray{bbox[0], bbox[1], bbox[2], bbox[3]}},
                       {QStringLiteral("info"), info}};
}

std::optional<GeoZone> GeoZone::fromJson(const QJsonObject &o)
{
    GeoZone z;
    z.id = o.value(QStringLiteral("id")).toString();
    z.src = o.value(QStringLiteral("src")).toString();
    z.name = o.value(QStringLiteral("name")).toString();
    z.kind = o.value(QStringLiteral("kind")).toString();
    z.floor = o.value(QStringLiteral("floor")).toDouble();
    z.floorAgl = o.value(QStringLiteral("floorRef")).toString() != QLatin1String("MSL");
    if (o.value(QStringLiteral("top")).isDouble()) z.top = o.value(QStringLiteral("top")).toDouble();
    z.topAgl = o.value(QStringLiteral("topRef")).toString() != QLatin1String("MSL");
    z.info = o.value(QStringLiteral("info")).toString();
    for (const QJsonValue &p : o.value(QStringLiteral("polys")).toArray()) {
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
    const QJsonArray b = o.value(QStringLiteral("bbox")).toArray();
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
    const QString type = geometry.value(QStringLiteral("type")).toString();
    QJsonArray list;
    if (type == QLatin1String("Polygon")) list.append(geometry.value(QStringLiteral("coordinates")));
    else if (type == QLatin1String("MultiPolygon")) list = geometry.value(QStringLiteral("coordinates")).toArray();
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
    return (uom.isEmpty() ? QStringLiteral("M") : uom).startsWith(QLatin1Char('f'), Qt::CaseInsensitive) ? v * kFt : v;
}

bool isAgl(const QString &ref)
{
    static const QRegularExpression re(QStringLiteral("^(AGL|SFC|GND)"), QRegularExpression::CaseInsensitiveOption);
    return re.match(ref.isEmpty() ? QStringLiteral("AGL") : ref).hasMatch();
}

QString restrictionKind(const QString &r)
{
    const QString v = r.toUpper();
    if (v == QLatin1String("PROHIBITED") || v == QLatin1String("REQ_AUTHORISATION") || v == QLatin1String("REQ_AUTHORIZATION"))
        return QStringLiteral("prohibited");
    if (v == QLatin1String("NO_RESTRICTION")) return {};
    return QStringLiteral("conditional");
}

std::optional<GeoZone> makeZone(const QString &src, const QString &key, const QString &name, const QString &kind, double floor,
                                bool floorAgl, std::optional<double> top, bool topAgl, std::vector<GeoZone::Polygon> polys,
                                const QString &info)
{
    if (kind.isEmpty() || polys.empty()) return std::nullopt;
    if (top && *top >= kUnlimitedM) top.reset();
    GeoZone z;
    z.id = src + QLatin1Char(':') + key;
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
    else features = json.toObject().value(QStringLiteral("features")).toArray();
    GeoZoneList zones;
    int n = 0;
    for (const QJsonValue &fv : features) {
        ++n;
        const QJsonObject f = fv.toObject();
        if (f.isEmpty()) continue;
        if (f.value(QStringLiteral("geometry")).isArray()) {
            // ED-269: one volume per geometry element
            if (expired(f.value(QStringLiteral("applicability")))) continue;
            const QString kind = restrictionKind(str(f, "restriction"));
            QString base = str(f, "identifier");
            if (base.isEmpty()) base = str(f, "name");
            if (base.isEmpty()) base = QStringLiteral("z%1").arg(n);
            const QJsonArray geoms = f.value(QStringLiteral("geometry")).toArray();
            QString info = str(f, "message");
            if (info.isEmpty() && f.value(QStringLiteral("restrictionConditions")).isArray()) {
                QStringList parts;
                for (const QJsonValue &c : f.value(QStringLiteral("restrictionConditions")).toArray()) parts << c.toVariant().toString();
                info = parts.join(QLatin1Char(' '));
            }
            for (int i = 0; i < geoms.size(); ++i) {
                const QJsonObject g = geoms[i].toObject();
                const QJsonObject hp = g.value(QStringLiteral("horizontalProjection")).toObject();
                std::vector<GeoZone::Polygon> polys;
                if (hp.value(QStringLiteral("type")).toString() == QLatin1String("Circle") && hp.value(QStringLiteral("center")).isArray()) {
                    const QJsonArray c = hp.value(QStringLiteral("center")).toArray();
                    polys = circlePoly(c[0].toDouble(), c[1].toDouble(), hp.value(QStringLiteral("radius")).toDouble());
                } else {
                    polys = geojsonPolys(hp);
                }
                const QString uom = str(g, "uomDimensions");
                QString name = str(f, "name");
                if (name.isEmpty()) name = str(f, "identifier");
                auto z = makeZone(src, geoms.size() > 1 ? QStringLiteral("%1#%2").arg(base).arg(i) : base, name, kind,
                                  toMetres(g.value(QStringLiteral("lowerLimit")), uom).value_or(0), isAgl(str(g, "lowerVerticalReference")),
                                  toMetres(g.value(QStringLiteral("upperLimit")), uom), isAgl(str(g, "upperVerticalReference")),
                                  std::move(polys), info);
                if (z) zones.push_back(std::move(*z));
            }
        } else {
            // ED-318 / GeoJSON
            const QJsonObject p = f.value(QStringLiteral("properties")).toObject();
            if (expired(p.contains(QStringLiteral("limitedApplicability")) ? p.value(QStringLiteral("limitedApplicability"))
                                                                         : p.value(QStringLiteral("applicability"))))
                continue;
            const QJsonObject v = p.value(QStringLiteral("verticalLayer")).isObject() ? p.value(QStringLiteral("verticalLayer")).toObject() : p;
            QString uom = str(v, "uom");
            if (uom.isEmpty()) uom = str(v, "uomDimensions");
            if (uom.isEmpty()) uom = QStringLiteral("M");
            auto pick = [&](const char *a, const char *b) { return v.contains(QLatin1String(a)) ? v.value(QLatin1String(a)) : v.value(QLatin1String(b)); };
            QString key = str(p, "identifier");
            if (key.isEmpty()) key = f.value(QStringLiteral("id")).toVariant().toString();
            if (key.isEmpty()) key = QStringLiteral("z%1").arg(n);
            QString name = str(p, "name");
            if (name.isEmpty()) name = str(p, "identifier");
            QString restriction = str(p, "restriction");
            if (restriction.isEmpty()) restriction = str(p, "type");
            QString info = str(p, "message");
            if (info.isEmpty()) info = str(p, "otherReasonInfo");
            auto z = makeZone(src, key, name, restrictionKind(restriction), toMetres(pick("lower", "lowerLimit"), uom).value_or(0),
                              isAgl(pick("lowerReference", "lowerVerticalReference").toString()),
                              toMetres(pick("upper", "upperLimit"), uom), isAgl(pick("upperReference", "upperVerticalReference").toString()),
                              geojsonPolys(f.value(QStringLiteral("geometry")).toObject()), info);
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
QString shortName(const GeoZone &z) { return z.name.size() > 18 ? z.name.left(17) + QStringLiteral("…") : z.name; }
QString fmtM(double m) { return QStringLiteral("%1 m").arg(qRound(m)); }
} // namespace

QString regionAt(double lat, double lon)
{
    static const double us[4][4] = {{-125, 24.4, -66.8, 49.4}, {-170, 51, -129, 71.5}, {-161, 18.8, -154.7, 22.3}, {-67.4, 17.8, -65.2, 18.6}};
    for (const auto &b : us)
        if (inBox(b, lon, lat)) return QStringLiteral("US");
    static const double canaries[4] = {-18.5, 27.5, -13.3, 29.5};
    if (inBox(canaries, lon, lat) || pointInRing(europe(), lon, lat)) return QStringLiteral("EU");
    return {};
}

std::optional<Limit> generalLimit(const QString &region)
{
    if (region == QLatin1String("EU")) return Limit{120, QStringLiteral("120 m"), QStringLiteral("EU open category (Reg. 2019/947): 120 m above the ground")};
    if (region == QLatin1String("US")) return Limit{400 * kFt, QStringLiteral("400 ft"), QStringLiteral("FAA Part 107 / recreational: 400 ft above the ground")};
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
    if (noFly) r.alerts.push_back({QStringLiteral("noFly"), QStringLiteral("warn"), QStringLiteral("zone"), QStringLiteral("NO-FLY"), shortName(*noFly), {}, noFly, 0});
    if (ceilZone)
        r.alerts.push_back({QStringLiteral("ceiling"), ceilZone->prohibited() ? QStringLiteral("warn") : QStringLiteral("caut"), QStringLiteral("ceiling"),
                            QStringLiteral("MAX ") + fmtM(ceilFloor), shortName(*ceilZone), {}, ceilZone, ceilFloor});
    if (overLimit)
        r.alerts.push_back({QStringLiteral("altLimit"), QStringLiteral("warn"), QStringLiteral("ceiling"), QStringLiteral("MAX ") + r.limit->label,
                            QStringLiteral("ALT LIMIT"), r.limit->rule, nullptr, r.limit->metres});
    if (cond) r.alerts.push_back({QStringLiteral("cond"), QStringLiteral("caut"), QStringLiteral("zone"), QStringLiteral("ZONE"), shortName(*cond), {}, cond, 0});
    if (maxZone && !ceilZone && (!r.limit || maxFloor < r.limit->metres))
        r.alerts.push_back({QStringLiteral("maxHere"), QStringLiteral("caut"), QStringLiteral("ceiling"), QStringLiteral("MAX ") + fmtM(maxFloor),
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
    if (noFly) return ArmAlert{QStringLiteral("warn"), QStringLiteral("ARMED IN A NO-FLY ZONE"), noFly->name + QStringLiteral(" — no flight without authorisation"), noFly};
    if (cond) return ArmAlert{QStringLiteral("caut"), QStringLiteral("ARMED IN A RESTRICTED ZONE"), cond->name + QStringLiteral(" — conditions apply"), cond};
    if (ceil)
        return ArmAlert{QStringLiteral("caut"), QStringLiteral("ARMED UNDER A %1 CEILING").arg(fmtM(ceilFloor)),
                        QStringLiteral("%1 — max %2 above the ground").arg(ceil->name, fmtM(ceilFloor)), ceil};
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
    static const QHash<QString, QString> names = {{QStringLiteral("faa"), QStringLiteral("FAA")}, {QStringLiteral("dipul"), QStringLiteral("DFS DIPUL")},
                                                  {QStringLiteral("ign"), QStringLiteral("IGN / DGAC")}, {QStringLiteral("enaire"), QStringLiteral("ENAIRE")},
                                                  {QStringLiteral("bazl"), QStringLiteral("FOCA / BAZL")}};
    if (names.contains(src)) return names.value(src);
    if (src.startsWith(QLatin1String("import-"))) return QStringLiteral("imported ") + src.mid(7);
    return src;
}

} // namespace geozone
} // namespace corv
