#include "ui/GeoZonePainter.h"

#include "terrain/Hgt.h"

#include <QPainter>
#include <QPainterPath>
#include <QPixmap>
#include <algorithm>
#include <vector>

namespace corv::geozone {

namespace {
constexpr int kMaxPolygons = 6000;

/// Floor above the ground (m), for the colour; nullopt when the ground is unknown.
std::optional<double> floorAgl(const GeoZone &z)
{
    if (z.floorAgl) return z.floor;
    const double lat = (z.bbox[1] + z.bbox[3]) / 2, lon = (z.bbox[0] + z.bbox[2]) / 2;
    const auto ground = HgtStore::instance().elevation(lat, lon);
    if (!ground) return std::nullopt;
    return z.floor - *ground;
}

QBrush hatch(const QColor &c, double dark)
{
    // A 10 px band pattern turned 45 degrees: the zone's colour with a black band
    QPixmap pm(10, 10);
    pm.fill(c);
    QPainter p(&pm);
    p.fillRect(QRectF(0, 0, dark, 10), Qt::black);
    p.end();
    QBrush b(pm);
    b.setTransform(QTransform().rotate(45));
    return b;
}

struct Styled {
    const GeoZone *z;
    double order, floor;
};
} // namespace

void paintZones(QPainter *p, const GeoZoneList &zones, const std::array<double, 4> &box,
                const std::function<QPointF(double lat, double lon)> &toScreen)
{
    std::vector<Styled> list;
    for (const GeoZone &z : zones) {
        if (z.bbox[0] > box[2] || z.bbox[2] < box[0] || z.bbox[1] > box[3] || z.bbox[3] < box[1]) continue;
        const auto floor = floorAgl(z);
        if (!floor) continue;
        if (*floor <= 0.5) list.push_back({&z, z.prohibited() ? -2.0 : -1.0, *floor});
        else if (*floor < kCeilingMaxM) list.push_back({&z, *floor, *floor});
    }
    std::sort(list.begin(), list.end(), [](const Styled &a, const Styled &b) { return a.order > b.order; });
    static const QBrush noFlyBrush = hatch(prohibitedColor(), 5);
    static const QBrush condBrush = hatch(conditionalColor(), 3.5);
    p->save();
    p->setRenderHint(QPainter::Antialiasing);
    int n = 0;
    for (const Styled &s : list) {
        const GeoZone &z = *s.z;
        const bool faaGrid = z.id.startsWith(QLatin1String("faa:uasfm"));
        QPainterPath path;
        path.setFillRule(Qt::OddEvenFill);
        for (const auto &rings : z.polys) {
            if (n++ >= kMaxPolygons) break;
            for (const auto &ring : rings) {
                if (ring.size() < 6) continue;
                path.moveTo(toScreen(ring[1], ring[0]));
                for (std::size_t i = 2; i + 1 < ring.size(); i += 2) path.lineTo(toScreen(ring[i + 1], ring[i]));
                path.closeSubpath();
            }
        }
        if (path.isEmpty()) continue;
        QColor line;
        if (s.floor <= 0.5) {
            const bool pr = z.prohibited();
            p->setOpacity(pr ? 0.45 : 0.35);
            p->fillPath(path, pr ? noFlyBrush : condBrush);
            line = pr ? prohibitedColor() : conditionalColor();
            line.setAlphaF(0.9f);
        } else {
            QColor fill = ceilingColor(s.floor);
            p->setOpacity(0.4);
            p->fillPath(path, fill);
            line = fill;
            line.setAlphaF(0.85f);
        }
        p->setOpacity(1.0);
        QPen pen(line, faaGrid ? 0.5 : 1.2);
        if (!z.prohibited()) pen.setDashPattern({5 / pen.widthF(), 4 / pen.widthF()});
        p->strokePath(path, pen);
        if (n >= kMaxPolygons) break;
    }
    p->restore();
}

QString describeAt(const GeoZoneList &zones, double lat, double lon)
{
    for (const GeoZone &z : zones) {
        if (!pointInZone(z, lat, lon)) continue;
        const auto floor = floorAgl(z);
        if (!floor || *floor >= kCeilingMaxM) continue;
        const QString what = *floor <= 0.5
                                 ? (z.prohibited() ? QStringLiteral("No-fly or authorisation required from the surface")
                                                   : QStringLiteral("Conditions apply from the surface"))
                                 : QStringLiteral("Max %1 m AGL%2").arg(qRound(*floor)).arg(z.prohibited() ? QString() : QStringLiteral(" (conditions above)"));
        auto lim = [](std::optional<double> v, bool agl) {
            return v ? QStringLiteral("%1 m %2").arg(qRound(*v)).arg(agl ? QStringLiteral("AGL") : QStringLiteral("MSL")) : QStringLiteral("unlimited");
        };
        return QStringLiteral("%1\n%2\nZone %3 – %4 · %5%6").arg(z.name, what, lim(z.floor, z.floorAgl), lim(z.top, z.topAgl), sourceName(z.src),
                                                             z.info.isEmpty() ? QString() : QStringLiteral("\n") + z.info.left(160));
    }
    return {};
}

} // namespace corv::geozone
