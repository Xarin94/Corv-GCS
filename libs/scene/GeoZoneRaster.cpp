#include "scene/GeoZoneRaster.h"

#include <QImage>
#include <QPainter>
#include <QPainterPath>
#include <QtCore/qfloat16.h>
#include <algorithm>
#include <cmath>
#include <cstring>

namespace corv {

std::shared_ptr<GeoZoneRaster> GeoZoneRaster::build(const GeoZoneList &zones, double centerLat, double centerLon, int version)
{
    constexpr int N = kSize;
    auto r = std::make_shared<GeoZoneRaster>();
    r->version = version;
    r->centerLat = centerLat;
    r->centerLon = centerLon;
    const double half = kWindowKm * 1000 / 2;
    const double dLat = half / 111320.0, dLon = dLat / std::max(0.05, std::cos(centerLat * M_PI / 180));
    r->lat0 = centerLat - dLat;
    r->lat1 = centerLat + dLat;
    r->lon0 = centerLon - dLon;
    r->lon1 = centerLon + dLon;
    const double sx = N / (r->lon1 - r->lon0), sy = N / (r->lat1 - r->lat0);
    auto toTexel = [&](double lon, double lat) { return QPointF((lon - r->lon0) * sx, (r->lat1 - lat) * sy); };

    QImage cov[3];
    QPainter cp[3];
    for (int i = 0; i < 3; ++i) {
        cov[i] = QImage(N, N, QImage::Format_ARGB32_Premultiplied);
        cov[i].fill(Qt::transparent);
        cp[i].begin(&cov[i]);
        cp[i].setRenderHint(QPainter::Antialiasing);
        cp[i].setPen(Qt::NoPen);
        cp[i].setBrush(Qt::white);
    }
    std::vector<float> floorAgl(std::size_t(N) * N, kNone), floorMsl(std::size_t(N) * N, kNone);
    QImage mask(N, N, QImage::Format_ARGB32_Premultiplied);
    mask.fill(Qt::transparent);

    for (const GeoZone &z : zones) {
        if (z.bbox[0] > r->lon1 || z.bbox[2] < r->lon0 || z.bbox[1] > r->lat1 || z.bbox[3] < r->lat0) continue;
        const bool surface = z.floorAgl && z.floor <= 0.5;
        // A ceiling this high is not drawn (the general limit is lower)
        if (z.floorAgl && !surface && z.floor >= geozone::kCeilingMaxM) continue;
        QPainterPath path;
        path.setFillRule(Qt::OddEvenFill);
        for (const auto &rings : z.polys)
            for (const auto &ring : rings) {
                if (ring.size() < 6) continue;
                path.moveTo(toTexel(ring[0], ring[1]));
                for (std::size_t i = 2; i + 1 < ring.size(); i += 2) path.lineTo(toTexel(ring[i], ring[i + 1]));
                path.closeSubpath();
            }
        if (path.isEmpty()) continue;
        r->empty = false;
        if (surface) {
            cp[z.prohibited() ? 0 : 1].drawPath(path);
            continue;
        }
        cp[2].drawPath(path);
        // Floors: the zone's mask grown by a texel, the lowest floor kept
        const QRect rect = path.boundingRect().toAlignedRect().adjusted(-2, -2, 2, 2).intersected(QRect(0, 0, N, N));
        if (rect.isEmpty()) continue;
        {
            QPainter mp(&mask);
            mp.setCompositionMode(QPainter::CompositionMode_Source);
            mp.fillRect(rect, Qt::transparent);
            mp.setCompositionMode(QPainter::CompositionMode_SourceOver);
            mp.setPen(QPen(Qt::white, 2.0));
            mp.setBrush(Qt::white);
            mp.drawPath(path);
        }
        const float f = float(z.floor);
        for (int y = rect.top(); y <= rect.bottom(); ++y) {
            const QRgb *line = reinterpret_cast<const QRgb *>(mask.constScanLine(y));
            for (int x = rect.left(); x <= rect.right(); ++x) {
                if (!qAlpha(line[x])) continue;
                const std::size_t i = std::size_t(y) * N + x;
                if (z.floorAgl) floorAgl[i] = std::min(floorAgl[i], f);
                else floorMsl[i] = std::min(floorMsl[i], f);
            }
        }
    }
    for (QPainter &p : cp) p.end();

    r->coverage.resize(std::size_t(N) * N * 4);
    r->floors.resize(std::size_t(N) * N * 4);
    const qfloat16 one(1.0f), zero(0.0f);
    for (int y = 0; y < N; ++y) {
        const QRgb *l0 = reinterpret_cast<const QRgb *>(cov[0].constScanLine(y));
        const QRgb *l1 = reinterpret_cast<const QRgb *>(cov[1].constScanLine(y));
        const QRgb *l2 = reinterpret_cast<const QRgb *>(cov[2].constScanLine(y));
        for (int x = 0; x < N; ++x) {
            const std::size_t i = std::size_t(y) * N + x;
            r->coverage[i * 4 + 0] = uint8_t(qAlpha(l0[x]));
            r->coverage[i * 4 + 1] = uint8_t(qAlpha(l1[x]));
            r->coverage[i * 4 + 2] = uint8_t(qAlpha(l2[x]));
            r->coverage[i * 4 + 3] = 255;
            const qfloat16 fa(floorAgl[i]), fm(floorMsl[i]);
            std::memcpy(&r->floors[i * 4 + 0], &fa, 2);
            std::memcpy(&r->floors[i * 4 + 1], &fm, 2);
            std::memcpy(&r->floors[i * 4 + 2], &zero, 2);
            std::memcpy(&r->floors[i * 4 + 3], &one, 2);
        }
    }
    return r;
}

} // namespace corv
