#include "ui/PlanMapItem.h"

#include "mission/Platforms.h"
#include "mission/RadioLink.h"
#include "mission/RouteGeo.h"
#include "ui/PlanController.h"
#include "ui/GeoZoneController.h"
#include "ui/GeoZonePainter.h"
#include "terrain/TileStore.h"
#include "vehicle/VehicleHub.h"

#include <QCursor>
#include <QFont>
#include <QPainter>
#include <QPainterPath>
#include <QtMath>
#include <cmath>

namespace corv {

using namespace mission;

namespace {

constexpr int kTile = 256;
constexpr double kMercR = 6378137.0;

QColor withAlpha(QColor c, double a)
{
    c.setAlphaF(std::clamp(a, 0.0, 1.0));
    return c;
}

QPen mkPen(const QColor &c, double w, double opacity = 1, bool round = true)
{
    QPen p(withAlpha(c, c.alphaF() * opacity), w);
    p.setCapStyle(round ? Qt::RoundCap : Qt::FlatCap);
    p.setJoinStyle(round ? Qt::RoundJoin : Qt::MiterJoin);
    return p;
}

/// Leaflet dashArray (px) → Qt dash pattern (pen widths)
void dash(QPen &p, double on, double off)
{
    const double w = std::max(0.5, p.widthF());
    p.setDashPattern({on / w, off / w});
}

QPolygonF poly(const QList<QPointF> &pts) { return QPolygonF(QVector<QPointF>(pts.begin(), pts.end())); }

void circleMarker(QPainter *p, QPointF c, double r, const QColor &stroke, double strokeW, const QColor &fill, bool dashed = false)
{
    QPen pen = mkPen(stroke, strokeW);
    if (dashed) dash(pen, 2, 2);
    p->setPen(strokeW > 0 ? pen : Qt::NoPen);
    p->setBrush(fill);
    p->drawEllipse(c, r, r);
}

/// .fp-arrow: a 8×9 triangle pointing along `headingDeg`, with a dark 1 px halo
void arrow(QPainter *p, QPointF at, double headingDeg, const QColor &color)
{
    p->save();
    p->translate(at);
    p->rotate(headingDeg);
    QPainterPath tri;
    tri.moveTo(0, -5);
    tri.lineTo(4, 4);
    tri.lineTo(-4, 4);
    tri.closeSubpath();
    p->setPen(QPen(QColor(0, 0, 0, 230), 1.2));
    p->setBrush(Qt::NoBrush);
    p->drawPath(tri);
    p->setPen(Qt::NoPen);
    p->setBrush(color);
    p->drawPath(tri);
    p->restore();
}

QFont font(const QString &family, double px, int weight = QFont::Normal)
{
    QFont f;
    f.setFamilies({family});
    f.setPixelSize(std::max(1, int(std::round(px))));
    f.setWeight(QFont::Weight(weight));
    return f;
}

double metersPerPixel(double lat, int zoom)
{
    return 2 * M_PI * kMercR * std::cos(lat * M_PI / 180) / (kTile * std::pow(2.0, zoom));
}

bool insidePolygon(const QPolygonF &poly, QPointF p) { return poly.containsPoint(p, Qt::OddEvenFill); }

double segDist(QPointF p, QPointF a, QPointF b)
{
    const QPointF ab = b - a;
    const double l2 = QPointF::dotProduct(ab, ab);
    const double t = l2 > 0 ? std::clamp(QPointF::dotProduct(p - a, ab) / l2, 0.0, 1.0) : 0;
    const QPointF q = a + ab * t;
    return std::hypot(p.x() - q.x(), p.y() - q.y());
}

double dist(QPointF a, QPointF b) { return std::hypot(a.x() - b.x(), a.y() - b.y()); }

} // namespace

PlanMapItem::PlanMapItem(QQuickItem *parent) : QQuickPaintedItem(parent)
{
    if (GeoZoneController::s_instance) connect(GeoZoneController::s_instance, &GeoZoneController::dataChanged, this, [this] { update(); });
    setAcceptedMouseButtons(Qt::LeftButton | Qt::RightButton);
    setAcceptHoverEvents(true);
    setAntialiasing(true);
    setFillColor(QColor(0xdd, 0xdd, 0xdd));   // the Leaflet container before the tiles arrive
    setCursor(Qt::OpenHandCursor);
    connect(&TileStore::instance(), &TileStore::tileReady, this, [this](const QString &, int z) {
        if (std::abs(z - m_zoom) <= 2) update();
    });
}

void PlanMapItem::componentComplete()
{
    QQuickPaintedItem::componentComplete();
    m_plan = PlanController::s_instance;
    if (!m_plan) return;
    connect(m_plan, &PlanController::mapChanged, this, [this] { update(); });
    connect(m_plan, &PlanController::layersChanged, this, [this] { update(); });
    connect(m_plan, &PlanController::fitRequested, this, &PlanMapItem::fitRoute);
    connect(m_plan, &PlanController::panRequested, this, [this](double lat, double lng) { setView(lat, lng, m_zoom); });
    connect(m_plan, &PlanController::centerRequested, this, [this](double lat, double lng, int z) { setView(lat, lng, z); });
    connect(m_plan, &PlanController::toolChanged, this, [this] {
        setCursor(m_plan->tool() == QLatin1String("select") ? Qt::OpenHandCursor : Qt::CrossCursor);
    });
    // Where an empty plan opens: the take-off point, else the demo aircraft
    QString kind;
    if (const auto t = m_plan->takeoffPoint(&kind)) {
        setView(t->lat, t->lng, 15);
    } else if (VehicleHub *hub = m_plan->hub()) {
        const auto s = hub->snapshot();
        setView(s->lat, s->lon, 13);
    }
}

// ── Projection ───────────────────────────────────────────────────────────────

QPointF PlanMapItem::project(double lat, double lng) const
{
    const double n = kTile * std::pow(2.0, m_zoom);
    const double s = std::sin(std::clamp(lat, -85.0511, 85.0511) * M_PI / 180);
    return {(lng + 180.0) / 360.0 * n, (0.5 - std::log((1 + s) / (1 - s)) / (4 * M_PI)) * n};
}

LL PlanMapItem::unproject(QPointF w) const
{
    const double n = kTile * std::pow(2.0, m_zoom);
    return {std::atan(std::sinh(M_PI * (1 - 2 * w.y() / n))) * 180 / M_PI, w.x() / n * 360.0 - 180.0};
}

QPointF PlanMapItem::origin() const
{
    return project(m_center.lat, m_center.lng) - QPointF(width() / 2, height() / 2);
}

void PlanMapItem::setView(double lat, double lng, int zoom)
{
    m_center = {lat, lng};
    m_zoom = std::clamp(zoom, 2, 20);
    emit viewChanged();
    update();
}

void PlanMapItem::zoomAt(int z, QPointF anchor)
{
    z = std::clamp(z, 2, 20);
    if (z == m_zoom) return;
    const LL ll = toLL(anchor);
    m_zoom = z;
    m_center = unproject(project(ll.lat, ll.lng) - (anchor - QPointF(width() / 2, height() / 2)));
    emit viewChanged();
    update();
}

void PlanMapItem::fitRoute()
{
    if (!m_plan) return;
    QList<LL> pts;
    for (const QJsonValue &v : m_plan->route().value(QStringLiteral("segments")).toArray()) pts.append(pointsOf(v.toObject()));
    if (m_plan->compiled().home) pts.append({m_plan->compiled().home->lat, m_plan->compiled().home->lng});
    if (pts.isEmpty()) return;
    if (pts.size() == 1) { setView(pts[0].lat, pts[0].lng, 15); return; }
    double minLat = 90, maxLat = -90, minLng = 180, maxLng = -180;
    for (const LL &p : pts) {
        minLat = std::min(minLat, p.lat); maxLat = std::max(maxLat, p.lat);
        minLng = std::min(minLng, p.lng); maxLng = std::max(maxLng, p.lng);
    }
    // The largest zoom (≤ 17) whose projected bounds fit with 40 px of padding
    int best = 2;
    for (int z = 17; z >= 2; --z) {
        m_zoom = z;
        const QPointF a = project(maxLat, minLng), b = project(minLat, maxLng);
        if (b.x() - a.x() <= width() - 80 && b.y() - a.y() <= height() - 80) { best = z; break; }
    }
    m_zoom = best;
    const LL c = unproject((project(maxLat, minLng) + project(minLat, maxLng)) / 2);
    setView(c.lat, c.lng, best);
}

// ── Paint ────────────────────────────────────────────────────────────────────

void PlanMapItem::drawTiles(QPainter *p, const QString &provider, int maxNative)
{
    const int z = std::min(m_zoom, maxNative);
    const double scale = std::pow(2.0, m_zoom - z);
    const double ts = kTile * scale;
    const QPointF o = origin();
    const int maxIndex = (1 << z) - 1;
    const int tx0 = int(std::floor(o.x() / ts)), ty0 = int(std::floor(o.y() / ts));
    const int tx1 = int(std::floor((o.x() + width()) / ts)), ty1 = int(std::floor((o.y() + height()) / ts));
    TileStore &store = TileStore::instance();
    for (int ty = ty0; ty <= ty1; ++ty) {
        if (ty < 0 || ty > maxIndex) continue;
        for (int tx = tx0; tx <= tx1; ++tx) {
            const int x = ((tx % (maxIndex + 1)) + maxIndex + 1) % (maxIndex + 1);
            const QRectF dst(tx * ts - o.x(), ty * ts - o.y(), ts, ts);
            const QImage img = store.tile(provider, z, x, ty);
            if (!img.isNull()) { p->drawImage(dst, img); continue; }
            // Missing: a parent tile, up to three levels up, scaled
            for (int up = 1; up <= 3 && z - up >= 0; ++up) {
                const int f = 1 << up;
                const QImage parent = store.tile(provider, z - up, x / f, ty / f);
                if (parent.isNull()) continue;
                const double sub = double(kTile) / f;
                p->drawImage(dst, parent, QRectF((x % f) * sub, (ty % f) * sub, sub, sub));
                break;
            }
        }
    }
}

void PlanMapItem::drawCoverage(QPainter *p)
{
    const auto cov = m_plan->coverage();
    if (!cov) return;
    if (m_covFor != cov.get()) {
        // Rasterise the polar coverage into a Web-Mercator image, as RadioLinkView.renderCoverage
        constexpr int kPx = 720;
        const double lat0 = cov->gs.lat * M_PI / 180;
        const double mx = kMercR * cov->gs.lng * M_PI / 180;
        const double my = kMercR * std::log(std::tan(M_PI / 4 + lat0 / 2));
        const double Rp = cov->range / std::cos(lat0);
        const double x0 = mx - Rp, yTop = my + Rp, px = 2 * Rp / kPx;
        const LocalFrame frame(cov->gs);
        QImage img(kPx, kPx, QImage::Format_ARGB32_Premultiplied);
        img.fill(Qt::transparent);
        static const QRgb colors[5] = {0, qPremultiply(qRgba(255, 60, 60, 84)), qPremultiply(qRgba(255, 170, 0, 78)), qPremultiply(qRgba(96, 255, 128, 66)), 0};
        for (int j = 0; j < kPx; ++j) {
            const double y = yTop - (j + 0.5) * px;
            const double lat = (2 * std::atan(std::exp(y / kMercR)) - M_PI / 2) * 180 / M_PI;
            auto *line = reinterpret_cast<QRgb *>(img.scanLine(j));
            for (int i = 0; i < kPx; ++i) {
                const double x = x0 + (i + 0.5) * px;
                const double lng = x / kMercR * 180 / M_PI;
                const XY q = frame.toXY({lat, lng});
                line[i] = colors[int(coverageClassAt(*cov, q.x, q.y))];
            }
        }
        m_covImage = img;
        m_covNW = {(2 * std::atan(std::exp(yTop / kMercR)) - M_PI / 2) * 180 / M_PI, x0 / kMercR * 180 / M_PI};
        m_covSE = {(2 * std::atan(std::exp((yTop - 2 * Rp) / kMercR)) - M_PI / 2) * 180 / M_PI, (x0 + 2 * Rp) / kMercR * 180 / M_PI};
        m_covFor = cov.get();
    }
    p->save();
    p->setRenderHint(QPainter::SmoothPixmapTransform);
    p->drawImage(QRectF(toScreen(m_covNW), toScreen(m_covSE)), m_covImage);
    p->restore();
}

void PlanMapItem::paint(QPainter *p)
{
    if (!m_plan) return;
    p->setRenderHint(QPainter::Antialiasing);
    const PlanController &P = *m_plan;
    const QJsonArray segs = P.route().value(QStringLiteral("segments")).toArray();
    const CompiledRoute &C = P.compiled();
    const QString sel = P.selectedId();
    const QString tool = P.tool();

    drawTiles(p, QStringLiteral("osm"), 19);
    if (P.showSat()) drawTiles(p, QStringLiteral("satellite"), 20);

    // UAS geographical zones over the tiles, under everything else
    if (const GeoZonesPtr zones = GeoZoneController::shownZones(); zones && !zones->empty()) {
        const LL nw = toLL(QPointF(0, 0)), se = toLL(QPointF(width(), height()));
        geozone::paintZones(p, *zones, {nw.lng, se.lat, se.lng, nw.lat}, [this](double lat, double lon) { return toScreen(lat, lon); });
    }

    // Radio coverage and the link halo under the route
    if (P.showLink()) {
        drawCoverage(p);
        if (const auto &A = P.linkAnalysis()) {
            for (const LinkRun &run : A->runs) {
                if (run.points.size() < 2) continue;
                const bool none = run.clazz == Link::None || run.clazz == Link::Unknown;
                QPen pen = mkPen(linkStyle(run.clazz).color, none ? 5 : 10, none ? 0.7 : 0.45, false);
                if (none) dash(pen, 2, 6);
                p->setPen(pen);
                p->setBrush(Qt::NoBrush);
                QList<QPointF> pts;
                for (const LL &q : run.points) pts.append(toScreen(q));
                p->drawPolyline(poly(pts));
            }
        }
    }

    // ── Segment figures ──
    for (qsizetype idx = 0; idx < segs.size(); ++idx) {
        const QJsonObject seg = segs[idx].toObject();
        const QString type = seg.value(QStringLiteral("type")).toString();
        const QColor color(segmentDef(type).value(QStringLiteral("color")).toString());
        const bool isSel = seg.value(QStringLiteral("id")).toString() == sel;
        const QList<LL> pts = pointsOf(seg);
        const QJsonObject sp = seg.value(QStringLiteral("params")).toObject();
        QPen base = mkPen(color, isSel ? 2.5 : 1.5, isSel ? 1 : 0.75);
        QList<QPointF> s;
        for (const LL &q : pts) s.append(toScreen(q));
        if (type == QLatin1String("area") && s.size() >= 3) {
            dash(base, 6, 4);
            p->setPen(base);
            p->setBrush(withAlpha(color, isSel ? 0.16 : 0.08));
            p->drawPolygon(poly(s));
        } else if (type == QLatin1String("perimeter") && s.size() >= 3) {
            p->setPen(base);
            p->setBrush(withAlpha(color, 0.02));
            p->drawPolygon(poly(s));
        } else if (type == QLatin1String("corridor") && pts.size() >= 2) {
            const double w = num(sp.value(QStringLiteral("width")));
            QList<QPointF> o;
            for (const LL &q : corridorOutline(pts, w ? w : 60)) o.append(toScreen(q));
            dash(base, 6, 4);
            p->setPen(base);
            p->setBrush(withAlpha(color, isSel ? 0.14 : 0.07));
            p->drawPolygon(poly(o));
            QPen axis = mkPen(color, 1, 0.5);
            dash(axis, 2, 4);
            p->setPen(axis);
            p->setBrush(Qt::NoBrush);
            p->drawPolyline(poly(s));
        } else if (type == QLatin1String("circle") && !pts.isEmpty()) {
            const double rr = num(sp.value(QStringLiteral("radius")));
            const double r = (rr ? rr : 50) / metersPerPixel(pts[0].lat, m_zoom);
            p->setPen(base);
            p->setBrush(withAlpha(color, isSel ? 0.08 : 0.03));
            p->drawEllipse(s[0], r, r);
        }
    }

    // ── Calculated flight path: home → …, ROI excluded ──
    const QList<NavPoint> &nav = C.navPath;
    if (nav.size() >= 2) {
        QList<QPointF> line;
        for (const NavPoint &n : nav) line.append(toScreen(n.lat, n.lng));
        p->setBrush(Qt::NoBrush);
        p->setPen(mkPen(QColor(0, 0, 0, 140), 5, 0.6));
        p->drawPolyline(poly(line));
        p->setPen(mkPen(QColor(0x44, 0xff, 0x44), 2, 0.95));
        p->drawPolyline(poly(line));
        if (!sel.isEmpty()) {
            // The selected segment's part of the path
            QList<QPointF> part;
            for (qsizetype i = 0; i < nav.size(); ++i)
                if (nav[i].segId == sel) { if (i > 0 && part.isEmpty()) part.append(line[i - 1]); part.append(line[i]); }
            if (part.size() >= 2) {
                p->setPen(mkPen(Qt::white, 3.5, 0.9));
                p->drawPolyline(poly(part));
            }
        }
    }

    // Calculated waypoints
    if (P.showCalc()) {
        QHash<QString, QString> typeOf;
        for (const QJsonValue &v : segs) typeOf.insert(v.toObject().value(QStringLiteral("id")).toString(), v.toObject().value(QStringLiteral("type")).toString());
        const QRectF vis = boundingRect().adjusted(-10, -10, 10, 10);
        for (const NavPoint &n : nav) {
            if (n.isHome) continue;
            const QJsonObject def = segmentDef(typeOf.value(n.segId));
            const bool single = def.value(QStringLiteral("points")).isDouble();
            if (single && !n.derived) continue;   // already drawn as the base point
            const QPointF q = toScreen(n.lat, n.lng);
            if (!vis.contains(q)) continue;
            const QColor fill = n.derived ? QColor(0x44, 0xff, 0x44) : QColor(def.value(QStringLiteral("color")).toString(QStringLiteral("#44ff44")));
            circleMarker(p, q, n.derived ? 2 : 3, n.derived ? QColor(255, 255, 255, 178) : QColor(Qt::white), 1, fill);
        }
    }

    // Draft lines (overlay pane)
    if (const auto &d = P.draft(); d && !d->points.isEmpty()) {
        const QJsonObject def = segmentDef(d->type);
        const QColor color(def.value(QStringLiteral("color")).toString());
        QList<QPointF> s;
        for (const LL &q : d->points) s.append(toScreen(q));
        if (def.value(QStringLiteral("points")).toString() == QLatin1String("poly") && s.size() >= 3) {
            p->setPen(Qt::NoPen);
            p->setBrush(withAlpha(color, 0.12));
            p->drawPolygon(poly(s));
        }
        QPen pen = mkPen(color, 2);
        dash(pen, 6, 4);
        p->setPen(pen);
        p->setBrush(Qt::NoBrush);
        p->drawPolyline(poly(s));
        if (m_mouseLL) {
            QPen rubber = mkPen(Qt::white, 1, 0.6);
            dash(rubber, 4, 4);
            p->setPen(rubber);
            p->drawLine(s.last(), toScreen(*m_mouseLL));
        }
    }

    // ── Camera: a red dot at every photo, pink where the camera aims at a POI ──
    if (P.showCam()) {
        const auto &photos = C.camera.photos;
        double radius = 4;
        if (photos.size() > 1) {
            const double px = dist(toScreen(photos[0].lat, photos[0].lng), toScreen(photos[1].lat, photos[1].lng));
            radius = std::max(1.5, std::min(4.0, px / 3));
        }
        const QColor red(0xff, 0x30, 0x30);
        const QRectF vis = boundingRect().adjusted(-10, -10, 10, 10);
        for (const Photo &ph : photos) {
            const QPointF q = toScreen(ph.lat, ph.lng);
            if (!vis.contains(q)) continue;
            circleMarker(p, q, ph.shot ? radius + 1.5 : radius, withAlpha(red, 0.9), 1, red);
        }
        for (const Cone &c : C.camera.cones)
            if (!c.onBasePoint) circleMarker(p, toScreen(c.from), 4.5, QColor(0xff, 0x66, 0xaa), 1, QColor(0xff, 0x66, 0xaa));
    }

    // ── Base points (not selected) ──
    auto drawBasePoints = [&](const QJsonObject &seg, bool isSel) {
        const QString type = seg.value(QStringLiteral("type")).toString();
        const QJsonObject def = segmentDef(type);
        const QColor color(def.value(QStringLiteral("color")).toString());
        const bool single = def.value(QStringLiteral("points")).isDouble();
        const double r = single ? 7 : 4;
        for (const LL &q : pointsOf(seg)) {
            circleMarker(p, toScreen(q), isSel ? r + 1 : r, isSel ? QColor(Qt::white) : QColor(255, 255, 255, 217), isSel ? 2 : 1.5,
                         withAlpha(color, single ? 1 : 0.9), type == QLatin1String("poi"));
        }
        if (type == QLatin1String("landing") && !pointsOf(seg).isEmpty()) {
            // The landing glyph over its point
            const QPointF c = toScreen(pointsOf(seg)[0]);
            p->save();
            p->translate(c - QPointF(9, 9));
            p->scale(18.0 / 24, 18.0 / 24);
            QPainterPath g;
            g.moveTo(12, 3); g.lineTo(12, 14);
            g.moveTo(7, 10); g.lineTo(12, 15); g.lineTo(17, 10);
            g.moveTo(4, 20); g.lineTo(20, 20);
            p->setBrush(Qt::NoBrush);
            p->setPen(mkPen(QColor(0, 0, 0, 160), 3.5));
            p->drawPath(g);
            p->setPen(mkPen(color, 2));
            p->drawPath(g);
            p->restore();
        }
    };
    for (const QJsonValue &v : segs)
        if (v.toObject().value(QStringLiteral("id")).toString() != sel) drawBasePoints(v.toObject(), false);

    // ── Markers: take-off point, operator, vehicle ──
    {
        QString kind;
        std::optional<LL> home = P.takeoffPoint(&kind);
        if (!home && C.home) home = LL{C.home->lat, C.home->lng};
        const bool custom = kind == QLatin1String("custom");
        if (m_drag == Drag::Home && m_dragMarker) home = m_dragMarker;
        if (home) {
            const QPointF c = toScreen(*home);
            p->setPen(Qt::NoPen);
            p->setBrush(QColor(0, 0, 0, 120));
            p->drawEllipse(c, 15, 15);
            QPen ring(Qt::white, 2);
            if (custom) ring.setStyle(Qt::DashLine);
            p->setPen(ring);
            p->setBrush(custom ? QColor(0xff, 0xb3, 0x47) : QColor(0xff, 0x88, 0x00));
            p->drawEllipse(c, 13, 13);
            p->setPen(Qt::black);
            p->setFont(font(QStringLiteral("Rajdhani"), 13, QFont::Bold));
            p->drawText(QRectF(c.x() - 12, c.y() - 12, 24, 24), Qt::AlignCenter, QStringLiteral("H"));
        }
        std::optional<LL> op;
        const QJsonObject jo = P.route().value(QStringLiteral("params")).toObject().value(QStringLiteral("operator")).toObject();
        if (!jo.isEmpty()) op = LL{num(jo.value(QStringLiteral("lat"))), num(jo.value(QStringLiteral("lng")))};
        if (m_drag == Drag::Operator && m_dragMarker) op = m_dragMarker;
        if (op) {
            const QPointF c = toScreen(*op);
            p->setPen(QPen(QColor(255, 255, 255, 153), 1));
            p->setBrush(QColor(0, 0, 0, 100));
            p->drawEllipse(c, 15, 15);
            p->setPen(QPen(QColor(0x1a, 0x24, 0x30), 2));
            p->setBrush(QColor(0xf4, 0xf7, 0xfa));
            p->drawEllipse(c, 13, 13);
            p->save();
            p->translate(c - QPointF(7.5, 7.5));
            p->scale(15.0 / 24, 15.0 / 24);
            QPainterPath a;
            a.moveTo(12, 22); a.lineTo(12, 13);
            a.addEllipse(QPointF(12, 10.5), 2.2, 2.2);
            a.moveTo(8.3, 6.8); a.arcTo(QRectF(12 - 5.2, 10.5 - 5.2, 10.4, 10.4), 135, 90);
            a.moveTo(15.7, 6.8); a.arcTo(QRectF(12 - 5.2, 10.5 - 5.2, 10.4, 10.4), 45, -90);
            a.moveTo(5.5, 4); a.arcTo(QRectF(12 - 9.2, 10.5 - 9.2, 18.4, 18.4), 135, 90);
            a.moveTo(18.5, 4); a.arcTo(QRectF(12 - 9.2, 10.5 - 9.2, 18.4, 18.4), 45, -90);
            p->setPen(mkPen(QColor(0x0a, 0x10, 0x18), 2));
            p->setBrush(Qt::NoBrush);
            p->drawPath(a);
            p->restore();
        }
        if (VehicleHub *hub = P.hub()) {
            const auto s = hub->snapshot();
            if (s->lat || s->lon) {
                p->save();
                p->translate(toScreen(s->lat, s->lon));
                p->rotate(s->yaw * 180 / M_PI);
                QPolygonF v({{0, -14}, {6, 12}, {0, 6}, {-6, 12}});
                p->setPen(QPen(QColor(0, 0, 0, 120), 4, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
                p->setBrush(Qt::NoBrush);
                p->drawPolygon(v);
                p->setPen(QPen(Qt::white, 1.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
                p->setBrush(QColor(0x00, 0xd2, 0xff));
                p->drawPolygon(v);
                p->restore();
            }
        }
    }

    // ── The hovered footprint / view wedge ──
    if (P.showCam()) {
        const auto &photos = C.camera.photos;
        const auto &cones = C.camera.cones;
        auto footprint = [&](const QList<LL> &q) {
            QList<QPointF> s;
            for (const LL &l : q) s.append(toScreen(l));
            p->setPen(mkPen(QColor(0xff, 0x40, 0x40), 1.5));
            p->setBrush(withAlpha(QColor(0xff, 0x30, 0x30), 0.18));
            p->drawPolygon(poly(s));
        };
        if (m_hoverPhoto >= 0 && m_hoverPhoto < photos.size()) {
            const Photo &ph = photos[m_hoverPhoto];
            footprint(ph.poly);
            circleMarker(p, toScreen(ph.lat, ph.lng), 4, Qt::white, 1.5, QColor(0xff, 0x30, 0x30));
        } else if (m_hoverCone >= 0 && m_hoverCone < cones.size()) {
            const Cone &c = cones[m_hoverCone];
            if (c.poly.size() == 4) {
                QPen wedge = mkPen(QColor(0xff, 0x80, 0xb0), 1);
                dash(wedge, 3, 3);
                p->setPen(wedge);
                p->setBrush(withAlpha(QColor(0xff, 0x66, 0xaa), 0.12));
                p->drawPolygon(poly({toScreen(c.from), toScreen(c.poly[3]), toScreen(c.poly[2])}));
                footprint(c.poly);
                QPen ray = mkPen(QColor(255, 102, 170, 178), 1);
                dash(ray, 1, 4);
                p->setPen(ray);
                p->drawLine(toScreen(c.from), toScreen(c.target));
            }
        }
    }

    // ── Labels near the first point, arrows along the path ──
    for (qsizetype idx = 0; idx < segs.size(); ++idx) {
        const QJsonObject seg = segs[idx].toObject();
        const QList<LL> pts = pointsOf(seg);
        if (pts.isEmpty()) continue;
        const QString type = seg.value(QStringLiteral("type")).toString();
        const QJsonObject def = segmentDef(type);
        const bool isSel = seg.value(QStringLiteral("id")).toString() == sel;
        // A plan read back from the vehicle is one waypoint per segment: past 40, every 5th
        if (segs.size() > 40 && type == QLatin1String("waypoint") && !isSel && (idx + 1) % 5 != 0) continue;
        const QColor color(def.value(QStringLiteral("color")).toString());
        const QString numText = QString::number(idx + 1);
        const QString chip = type == QLatin1String("waypoint") ? QString() : def.value(QStringLiteral("short")).toString();
        const QFont fNum = font(QStringLiteral("Roboto Mono"), 10, QFont::Bold);
        const QFont fChip = font(QStringLiteral("Roboto Mono"), 8, QFont::Medium);
        const double wNum = QFontMetricsF(fNum).horizontalAdvance(numText);
        const double wChip = chip.isEmpty() ? 0 : QFontMetricsF(fChip).horizontalAdvance(chip) + 3;
        const double w = wNum + wChip + 10 + 2, h = 15;
        const QPointF c = toScreen(pts[0]) + QPointF(9, -9);
        const QRectF box(c.x() - w / 2, c.y() - h / 2, w, h);
        if (isSel) {
            p->setPen(Qt::NoPen);
            p->setBrush(QColor(0, 210, 255, 60));
            p->drawRoundedRect(box.adjusted(-3, -3, 3, 3), 4, 4);
        }
        p->setPen(QPen(color, 1));
        p->setBrush(isSel ? QColor(0, 210, 255, 64) : QColor(5, 12, 20, 217));
        p->drawRoundedRect(box, 2, 2);
        p->setPen(Qt::white);
        p->setFont(fNum);
        p->drawText(QRectF(box.x() + 6, box.y(), wNum + 1, h), Qt::AlignVCenter | Qt::AlignLeft, numText);
        if (!chip.isEmpty()) {
            p->setPen(QColor(0xcf, 0xd8, 0xdd));
            p->setFont(fChip);
            p->drawText(QRectF(box.x() + 6 + wNum + 3, box.y() + 1, wChip, h), Qt::AlignVCenter | Qt::AlignLeft, chip);
        }
        // Circle: four arrows on the rim pointing along the turn
        if (type == QLatin1String("circle")) {
            const QJsonObject sp = seg.value(QStringLiteral("params")).toObject();
            const double rr = num(sp.value(QStringLiteral("radius")));
            const double r = rr ? rr : 50;
            const LocalFrame f(pts[0]);
            const double turn = sp.value(QStringLiteral("direction")).toString() == QLatin1String("ccw") ? -90 : 90;
            for (double a : {45.0, 135.0, 225.0, 315.0}) {
                const LL ll = f.toLL({r * std::sin(qDegreesToRadians(a)), r * std::cos(qDegreesToRadians(a))});
                arrow(p, toScreen(ll), std::fmod(a + turn + 360, 360), color);
            }
        }
    }
    // Direction arrows along the calculated path, every 70 px on screen
    if (nav.size() >= 2) {
        constexpr double kMinPx = 70;
        double carry = 0;
        for (qsizetype i = 1; i < nav.size(); ++i) {
            const QPointF a = toScreen(nav[i - 1].lat, nav[i - 1].lng), b = toScreen(nav[i].lat, nav[i].lng);
            const double len = dist(a, b);
            if (len < 4) continue;
            double pos = kMinPx / 2 - carry;
            const double brg = bearing(nav[i - 1].ll(), nav[i].ll());
            const bool isSel = !sel.isEmpty() && nav[i].segId == sel;
            while (pos < len) {
                const QPointF at = a + (b - a) * (pos / len);
                if (boundingRect().adjusted(-10, -10, 10, 10).contains(at))
                    arrow(p, at, brg, isSel ? QColor(Qt::white) : QColor(68, 255, 68, 230));
                pos += kMinPx;
            }
            carry = len - (pos - kMinPx);
        }
    }

    // ── Handles of the selected segment ──
    for (const QJsonValue &v : segs) {
        const QJsonObject seg = v.toObject();
        if (seg.value(QStringLiteral("id")).toString() != sel) continue;
        drawBasePoints(seg, true);
        const QString type = seg.value(QStringLiteral("type")).toString();
        const QJsonObject def = segmentDef(type);
        const QColor color(def.value(QStringLiteral("color")).toString());
        const QList<LL> pts = pointsOf(seg);
        const QString kind = def.value(QStringLiteral("points")).toString();
        if ((kind == QLatin1String("poly") || kind == QLatin1String("line")) && pts.size() >= 2) {
            const qsizetype n = pts.size();
            const qsizetype edges = kind == QLatin1String("poly") ? n : n - 1;
            for (qsizetype i = 0; i < edges; ++i) {
                const LL &a = pts[i], &b = pts[(i + 1) % n];
                circleMarker(p, toScreen((a.lat + b.lat) / 2, (a.lng + b.lng) / 2), 4, Qt::white, 1, withAlpha(color, 0.45));
            }
            const LL c = *centroid(pts);
            const QPointF cs = toScreen(c);
            p->setPen(Qt::NoPen);
            p->setBrush(QColor(0, 210, 255, 60));
            p->drawEllipse(cs, 11, 11);
            p->setPen(QPen(QColor(0x00, 0xd2, 0xff), 2));
            p->setBrush(QColor(0, 210, 255, 89));
            p->drawEllipse(cs, 8, 8);
        }
        if (type == QLatin1String("circle") && !pts.isEmpty()) {
            const double rr = num(seg.value(QStringLiteral("params")).toObject().value(QStringLiteral("radius")));
            const LL rim = LocalFrame(pts[0]).toLL({rr ? rr : 50, 0});
            circleMarker(p, toScreen(rim), 5, Qt::white, 1.5, color);
        }
    }

    // Draft vertices
    if (const auto &d = P.draft(); d && !d->points.isEmpty()) {
        const QColor color(segmentDef(d->type).value(QStringLiteral("color")).toString());
        for (qsizetype i = 0; i < d->points.size(); ++i) circleMarker(p, toScreen(d->points[i]), i == 0 ? 6 : 4, Qt::white, 1.5, color);
    }

    // Profile cursor on the map, and the line of sight to it
    if (const auto &h = P.profileHover()) {
        if (P.showLink() && P.groundStation() && P.profileHoverClass()) {
            QPen los = mkPen(linkStyle(*P.profileHoverClass()).color, 1.5, 0.9);
            dash(los, 5, 5);
            p->setPen(los);
            p->drawLine(toScreen(P.groundStation()->lat, P.groundStation()->lng), toScreen(*h));
        }
        circleMarker(p, toScreen(*h), 6, QColor(0x00, 0xd2, 0xff), 2, QColor(0, 210, 255, 102));
    }
}

// ── Hit testing ──────────────────────────────────────────────────────────────

PlanMapItem::Hit PlanMapItem::hitTest(QPointF s, bool forContext) const
{
    Hit h;
    if (!m_plan) return h;
    const QJsonArray segs = m_plan->route().value(QStringLiteral("segments")).toArray();
    const QString sel = m_plan->selectedId();
    // Handles of the selected segment (top-most)
    for (const QJsonValue &v : segs) {
        const QJsonObject seg = v.toObject();
        if (seg.value(QStringLiteral("id")).toString() != sel || forContext) continue;
        const QString type = seg.value(QStringLiteral("type")).toString();
        const QJsonObject def = segmentDef(type);
        const QList<LL> pts = pointsOf(seg);
        const QString kind = def.value(QStringLiteral("points")).toString();
        if (type == QLatin1String("circle") && !pts.isEmpty()) {
            const double rr = num(seg.value(QStringLiteral("params")).toObject().value(QStringLiteral("radius")));
            if (dist(s, toScreen(LocalFrame(pts[0]).toLL({rr ? rr : 50, 0}))) <= 7) return {Hit::Radius, sel, 0};
        }
        if ((kind == QLatin1String("poly") || kind == QLatin1String("line")) && pts.size() >= 2) {
            if (dist(s, toScreen(*centroid(pts))) <= 10) return {Hit::Centre, sel, -1};
            const qsizetype n = pts.size();
            const qsizetype edges = kind == QLatin1String("poly") ? n : n - 1;
            for (qsizetype i = 0; i < edges; ++i) {
                const LL &a = pts[i], &b = pts[(i + 1) % n];
                if (dist(s, toScreen((a.lat + b.lat) / 2, (a.lng + b.lng) / 2)) <= 6) return {Hit::Mid, sel, int(i)};
            }
        }
        const double r = (def.value(QStringLiteral("points")).isDouble() ? 7 : 4) + 4;
        for (qsizetype i = pts.size() - 1; i >= 0; --i)
            if (dist(s, toScreen(pts[i])) <= r) return {Hit::Vertex, sel, int(i)};
    }
    if (!forContext) {
        const QJsonObject jo = m_plan->route().value(QStringLiteral("params")).toObject().value(QStringLiteral("operator")).toObject();
        if (!jo.isEmpty() && dist(s, toScreen(num(jo.value(QStringLiteral("lat"))), num(jo.value(QStringLiteral("lng"))))) <= 14) return {Hit::Operator, {}, -1};
        QString kind;
        std::optional<LL> home = m_plan->takeoffPoint(&kind);
        if (!home && m_plan->compiled().home) home = LL{m_plan->compiled().home->lat, m_plan->compiled().home->lng};
        if (home && dist(s, toScreen(*home)) <= 14) return {Hit::Home, {}, -1};
    }
    // Base points, then figures, top-most first
    for (qsizetype k = segs.size() - 1; k >= 0; --k) {
        const QJsonObject seg = segs[k].toObject();
        const QJsonObject def = segmentDef(seg.value(QStringLiteral("type")).toString());
        const double r = (def.value(QStringLiteral("points")).isDouble() ? 7 : 4) + 3;
        const QList<LL> pts = pointsOf(seg);
        for (qsizetype i = pts.size() - 1; i >= 0; --i)
            if (dist(s, toScreen(pts[i])) <= r) return {Hit::Point, seg.value(QStringLiteral("id")).toString(), int(i)};
    }
    for (qsizetype k = segs.size() - 1; k >= 0; --k) {
        const QJsonObject seg = segs[k].toObject();
        const QString type = seg.value(QStringLiteral("type")).toString();
        const QString id = seg.value(QStringLiteral("id")).toString();
        const QList<LL> pts = pointsOf(seg);
        const QJsonObject sp = seg.value(QStringLiteral("params")).toObject();
        QList<QPointF> sc;
        for (const LL &q : pts) sc.append(toScreen(q));
        if ((type == QLatin1String("area") || type == QLatin1String("perimeter")) && sc.size() >= 3) {
            if (insidePolygon(poly(sc), s)) return {Hit::Figure, id, -1};
            for (qsizetype i = 0; i < sc.size(); ++i) if (segDist(s, sc[i], sc[(i + 1) % sc.size()]) <= 4) return {Hit::Figure, id, -1};
        } else if (type == QLatin1String("corridor") && pts.size() >= 2) {
            const double w = num(sp.value(QStringLiteral("width")));
            QList<QPointF> o;
            for (const LL &q : corridorOutline(pts, w ? w : 60)) o.append(toScreen(q));
            if (insidePolygon(poly(o), s)) return {Hit::Figure, id, -1};
        } else if (type == QLatin1String("circle") && !pts.isEmpty()) {
            const double rr = num(sp.value(QStringLiteral("radius")));
            if (dist(s, sc[0]) <= (rr ? rr : 50) / metersPerPixel(pts[0].lat, m_zoom) + 3) return {Hit::Figure, id, -1};
        }
    }
    return h;
}

// ── Mouse ────────────────────────────────────────────────────────────────────

void PlanMapItem::mousePressEvent(QMouseEvent *e)
{
    if (!m_plan) return;
    forceActiveFocus();
    const QPointF pos = e->position();
    if (e->button() == Qt::RightButton) {
        const LL ll = toLL(pos);
        QVariantList items;
        auto add = [&](const char *act, const QString &label, bool danger = false) {
            items.append(QVariantList{QString::fromLatin1(act), label, danger});
        };
        QString segId;
        int pointIdx = -1;
        if (m_plan->draft()) {
            add("finish", QStringLiteral("Finish drawing"));
            add("cancel", QStringLiteral("Cancel drawing"));
        } else {
            const Hit h = hitTest(pos, true);
            if (h.kind == Hit::Point || h.kind == Hit::Figure) {
                segId = h.segId;
                pointIdx = h.kind == Hit::Point ? h.index : -1;
                QJsonObject seg;
                for (const QJsonValue &v : m_plan->route().value(QStringLiteral("segments")).toArray())
                    if (v.toObject().value(QStringLiteral("id")).toString() == segId) seg = v.toObject();
                const QJsonObject def = segmentDef(seg.value(QStringLiteral("type")).toString());
                add("select", QStringLiteral("Select %1").arg(def.value(QStringLiteral("label")).toString().toLower()));
                const QString kind = def.value(QStringLiteral("points")).toString();
                if (pointIdx >= 0 && (kind == QLatin1String("poly") || kind == QLatin1String("line"))) {
                    add("insert", QStringLiteral("Insert vertex after"));
                    if (pointsOf(seg).size() > (kind == QLatin1String("poly") ? 3 : 2)) add("delvtx", QStringLiteral("Delete vertex"), true);
                }
                add("delete", QStringLiteral("Delete segment"), true);
            } else {
                add("wp", QStringLiteral("Add waypoint here"));
                add("poi", QStringLiteral("Add POI here"));
                add("home", QStringLiteral("Take-off point here"));
                add("operator", QStringLiteral("Operator here"));
                if (!m_plan->selectedId().isEmpty()) add("deselect", QStringLiteral("Deselect"));
            }
        }
        emit contextMenu(pos.x(), pos.y(), items, ll.lat, ll.lng, segId, pointIdx);
        e->accept();
        return;
    }
    emit clickedAway();
    m_press = pos;
    m_pressCenter = m_center;
    m_moved = false;
    m_pressHit = hitTest(pos, false);
    m_dragMarker.reset();
    switch (m_pressHit.kind) {
    case Hit::Vertex:
        m_drag = Drag::Vertex;
        m_gestureLabel = QStringLiteral("Move point");
        break;
    case Hit::Mid: {
        // Insert a vertex and drag it at once
        const QJsonObject seg = [&] {
            for (const QJsonValue &v : m_plan->route().value(QStringLiteral("segments")).toArray())
                if (v.toObject().value(QStringLiteral("id")).toString() == m_pressHit.segId) return v.toObject();
            return QJsonObject();
        }();
        const QList<LL> pts = pointsOf(seg);
        const LL a = pts[m_pressHit.index], b = pts[(m_pressHit.index + 1) % pts.size()];
        m_plan->insertPoint(m_pressHit.segId, m_pressHit.index, (a.lat + b.lat) / 2, (a.lng + b.lng) / 2);
        m_pressHit = {Hit::Vertex, m_pressHit.segId, m_pressHit.index + 1};
        m_drag = Drag::Vertex;
        m_gestureLabel = QStringLiteral("Insert vertex");
        m_moved = true;
        break;
    }
    case Hit::Centre: {
        m_drag = Drag::Centre;
        for (const QJsonValue &v : m_plan->route().value(QStringLiteral("segments")).toArray())
            if (v.toObject().value(QStringLiteral("id")).toString() == m_pressHit.segId) m_dragOriginal = pointsOf(v.toObject());
        m_dragStartLL = toLL(pos);
        m_gestureLabel = QStringLiteral("Move segment");
        break;
    }
    case Hit::Radius:
        m_drag = Drag::Radius;
        m_gestureLabel = QStringLiteral("Change radius");
        break;
    case Hit::Home: m_drag = Drag::Home; break;
    case Hit::Operator: m_drag = Drag::Operator; break;
    default: m_drag = Drag::Pan; break;
    }
    if (m_drag == Drag::Pan) setCursor(m_plan->tool() == QLatin1String("select") ? Qt::ClosedHandCursor : Qt::CrossCursor);
    else setCursor(Qt::ClosedHandCursor);
    e->accept();
}

void PlanMapItem::mouseMoveEvent(QMouseEvent *e)
{
    if (!m_plan) return;
    const QPointF pos = e->position();
    const QPointF d = pos - m_press;
    m_mouseLL = toLL(pos);
    switch (m_drag) {
    case Drag::Pan:
        if (!m_moved && std::hypot(d.x(), d.y()) < 3) return;
        m_moved = true;
        m_center = unproject(project(m_pressCenter.lat, m_pressCenter.lng) - d);
        update();
        break;
    case Drag::Vertex:
        m_moved = true;
        m_plan->movePoint(m_pressHit.segId, m_pressHit.index, m_mouseLL->lat, m_mouseLL->lng);
        break;
    case Drag::Centre:
        m_moved = true;
        m_plan->moveSegmentBy(m_pressHit.segId, m_dragOriginal, m_mouseLL->lat - m_dragStartLL.lat, m_mouseLL->lng - m_dragStartLL.lng);
        break;
    case Drag::Radius: {
        m_moved = true;
        for (const QJsonValue &v : m_plan->route().value(QStringLiteral("segments")).toArray()) {
            if (v.toObject().value(QStringLiteral("id")).toString() != m_pressHit.segId) continue;
            const QList<LL> pts = pointsOf(v.toObject());
            if (!pts.isEmpty()) m_plan->setCircleRadius(m_pressHit.segId, haversine(pts[0], *m_mouseLL));
        }
        break;
    }
    case Drag::Home:
    case Drag::Operator:
        if (!m_moved && std::hypot(d.x(), d.y()) < 3) return;
        m_moved = true;
        m_dragMarker = m_mouseLL;
        update();
        break;
    case Drag::None:
        break;
    }
}

void PlanMapItem::mouseReleaseEvent(QMouseEvent *e)
{
    if (!m_plan || e->button() != Qt::LeftButton) return;
    const QPointF pos = e->position();
    const Drag drag = m_drag;
    m_drag = Drag::None;
    setCursor(m_plan->tool() == QLatin1String("select") ? Qt::OpenHandCursor : Qt::CrossCursor);
    switch (drag) {
    case Drag::Vertex:
    case Drag::Centre:
    case Drag::Radius:
        if (m_moved) m_plan->endGesture(m_gestureLabel);
        break;
    case Drag::Home:
        if (m_moved && m_dragMarker) m_plan->setHome(m_dragMarker->lat, m_dragMarker->lng);
        else m_plan->openRouteSettingsFromMap();
        break;
    case Drag::Operator:
        if (m_moved && m_dragMarker) m_plan->setOperator(m_dragMarker);
        else m_plan->openRadioFromMap();
        break;
    case Drag::Pan:
        if (!m_moved) {
            const LL ll = toLL(pos);
            if (m_plan->tool() == QLatin1String("select") && (m_pressHit.kind == Hit::Point || m_pressHit.kind == Hit::Figure)) {
                m_plan->selectFromMap(m_pressHit.segId);
            } else {
                bool nearFirst = false;
                if (const auto &d = m_plan->draft(); d && !d->points.isEmpty()) nearFirst = dist(toScreen(d->points.first()), pos) < 12;
                m_plan->mapClick(ll.lat, ll.lng, nearFirst);
            }
        } else {
            emit viewChanged();
        }
        break;
    case Drag::None:
        break;
    }
    m_dragMarker.reset();
    update();
}

void PlanMapItem::mouseDoubleClickEvent(QMouseEvent *e)
{
    if (m_plan && e->button() == Qt::LeftButton) m_plan->mapDoubleClick();
    e->accept();
}

void PlanMapItem::hoverMoveEvent(QHoverEvent *e)
{
    if (!m_plan) return;
    const QPointF pos = e->position();
    m_mouseLL = toLL(pos);
    {
        const GeoZonesPtr zones = GeoZoneController::shownZones();
        const QString tip = zones ? geozone::describeAt(*zones, m_mouseLL->lat, m_mouseLL->lng) : QString();
        if (tip != m_zoneTip) {
            m_zoneTip = tip;
            emit zoneTipChanged();
        }
    }
    int photo = -1, cone = -1;
    if (m_plan->showCam()) {
        const auto &C = m_plan->compiled().camera;
        double best = 7;
        for (qsizetype i = 0; i < C.photos.size(); ++i) {
            const double d = dist(toScreen(C.photos[i].lat, C.photos[i].lng), pos);
            if (d < best) { best = d; photo = int(i); }
        }
        if (photo < 0) {
            // POI view points: their own dot, or a base point they sit on
            for (qsizetype i = 0; i < C.cones.size(); ++i)
                if (dist(toScreen(C.cones[i].from), pos) <= (C.cones[i].onBasePoint ? 9 : 6)) { cone = int(i); break; }
        }
    }
    const bool hoverChanged = photo != m_hoverPhoto || cone != m_hoverCone;
    m_hoverPhoto = photo;
    m_hoverCone = cone;
    if (m_plan->tool() == QLatin1String("select")) {
        const Hit h = hitTest(pos, false);
        setCursor(h.kind == Hit::None || h.kind == Hit::Figure ? Qt::OpenHandCursor
                  : h.kind == Hit::Centre                        ? Qt::SizeAllCursor
                  : h.kind == Hit::Mid                           ? Qt::DragCopyCursor
                                                                 : Qt::PointingHandCursor);
    }
    if (hoverChanged || m_plan->draft()) update();
}

void PlanMapItem::hoverLeaveEvent(QHoverEvent *)
{
    m_mouseLL.reset();
    m_hoverPhoto = m_hoverCone = -1;
    update();
}

void PlanMapItem::wheelEvent(QWheelEvent *e)
{
    zoomAt(m_zoom + (e->angleDelta().y() > 0 ? 1 : -1), e->position());
    e->accept();
}

} // namespace corv
