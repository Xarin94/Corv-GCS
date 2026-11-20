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

using namespace Qt::StringLiterals;

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

double segDist(QPointF p, QPointF a, QPointF b)
{
    const QPointF ab = b - a;
    const double l2 = QPointF::dotProduct(ab, ab);
    const double t = l2 > 0 ? std::clamp(QPointF::dotProduct(p - a, ab) / l2, 0.0, 1.0) : 0;
    const QPointF q = a + ab * t;
    return std::hypot(p.x() - q.x(), p.y() - q.y());
}

double dist(QPointF a, QPointF b) { return std::hypot(a.x() - b.x(), a.y() - b.y()); }

double paramOr(const QJsonObject &seg, const QString &key, double fallback)
{
    const double v = num(seg.value(u"params"_s).toObject().value(key));
    return v ? v : fallback;
}
double circleRadius(const QJsonObject &seg) { return paramOr(seg, u"radius"_s, 50); }
double corridorWidth(const QJsonObject &seg) { return paramOr(seg, u"width"_s, 60); }

/// The drag handle on the rim of a circle segment, east of its centre.
LL rimHandle(const QJsonObject &seg) { return LocalFrame(pointsOf(seg)[0]).toLL({circleRadius(seg), 0}); }

/// Midpoints of the edges of a polygon (closed) or a polyline.
QList<LL> edgeMidpoints(const QList<LL> &pts, bool closed)
{
    QList<LL> out;
    const qsizetype n = pts.size(), edges = closed ? n : n - 1;
    for (qsizetype i = 0; i < edges; ++i) {
        const LL &a = pts[i], &b = pts[(i + 1) % n];
        out.append({(a.lat + b.lat) / 2, (a.lng + b.lng) / 2});
    }
    return out;
}

void homeMarker(QPainter *p, QPointF c, bool custom)
{
    p->setPen(Qt::NoPen);
    p->setBrush(QColor(0, 0, 0, 120));
    p->drawEllipse(c, 15, 15);
    QPen ring(Qt::white, 2);
    if (custom) ring.setStyle(Qt::DashLine);
    p->setPen(ring);
    p->setBrush(custom ? QColor(0xff, 0xb3, 0x47) : QColor(0xff, 0x88, 0x00));
    p->drawEllipse(c, 13, 13);
    p->setPen(Qt::black);
    p->setFont(font(u"Rajdhani"_s, 13, QFont::Bold));
    p->drawText(QRectF(c.x() - 12, c.y() - 12, 24, 24), Qt::AlignCenter, u"H"_s);
}

/// A disc with the antenna glyph: a mast and two pairs of radio arcs.
void operatorMarker(QPainter *p, QPointF c)
{
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

void vehicleMarker(QPainter *p, QPointF at, double yawDeg)
{
    p->save();
    p->translate(at);
    p->rotate(yawDeg);
    const QPolygonF v({{0, -14}, {6, 12}, {0, 6}, {-6, 12}});
    p->setPen(QPen(QColor(0, 0, 0, 120), 4, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    p->setBrush(Qt::NoBrush);
    p->drawPolygon(v);
    p->setPen(QPen(Qt::white, 1.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    p->setBrush(QColor(0x00, 0xd2, 0xff));
    p->drawPolygon(v);
    p->restore();
}

/// The landing glyph (arrow down onto a line), 18 px, centred on `c`.
void landingGlyph(QPainter *p, QPointF c, const QColor &color)
{
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
    connect(m_plan, &PlanController::toolChanged, this, [this] { setCursor(toolCursor(Qt::OpenHandCursor)); });
    // Where an empty plan opens: the take-off point, else the demo aircraft
    if (const auto t = m_plan->takeoffPoint()) {
        setView(t->lat, t->lng, 15);
    } else if (VehicleHub *hub = m_plan->hub()) {
        const auto s = hub->snapshot();
        setView(s->lat, s->lon, 13);
    }
}

Qt::CursorShape PlanMapItem::toolCursor(Qt::CursorShape select) const
{
    return m_plan->tool() == "select"_L1 ? select : Qt::CrossCursor;
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

QPolygonF PlanMapItem::toScreen(const QList<LL> &pts) const
{
    QPolygonF out;
    out.reserve(pts.size());
    for (const LL &q : pts) out.append(toScreen(q));
    return out;
}

std::optional<LL> PlanMapItem::homePoint(bool *custom) const
{
    QString kind;
    std::optional<LL> home = m_plan->takeoffPoint(&kind);
    if (!home && m_plan->compiled().home) home = LL{m_plan->compiled().home->lat, m_plan->compiled().home->lng};
    if (custom) *custom = kind == "custom"_L1;
    return home;
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
    for (const QJsonValue &v : m_plan->route().value(u"segments"_s).toArray()) pts.append(pointsOf(v.toObject()));
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
    const double ts = kTile * std::pow(2.0, m_zoom - z);
    const QPointF o = origin();
    const int maxIndex = (1 << z) - 1;
    const int tx0 = int(std::floor(o.x() / ts)), ty0 = int(std::floor(o.y() / ts));
    const int tx1 = int(std::floor((o.x() + width()) / ts)), ty1 = int(std::floor((o.y() + height()) / ts));
    TileStore &store = TileStore::instance();
    for (int ty = std::max(ty0, 0); ty <= std::min(ty1, maxIndex); ++ty) {
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
        auto latOf = [](double y) { return (2 * std::atan(std::exp(y / kMercR)) - M_PI / 2) * 180 / M_PI; };
        auto lngOf = [](double x) { return x / kMercR * 180 / M_PI; };
        const LocalFrame frame(cov->gs);
        QImage img(kPx, kPx, QImage::Format_ARGB32_Premultiplied);
        img.fill(Qt::transparent);
        static const QRgb colors[5] = {0, qPremultiply(qRgba(255, 60, 60, 84)), qPremultiply(qRgba(255, 170, 0, 78)), qPremultiply(qRgba(96, 255, 128, 66)), 0};
        for (int j = 0; j < kPx; ++j) {
            const double lat = latOf(yTop - (j + 0.5) * px);
            auto *line = reinterpret_cast<QRgb *>(img.scanLine(j));
            for (int i = 0; i < kPx; ++i) {
                const XY q = frame.toXY({lat, lngOf(x0 + (i + 0.5) * px)});
                line[i] = colors[int(coverageClassAt(*cov, q.x, q.y))];
            }
        }
        m_covImage = img;
        m_covNW = {latOf(yTop), lngOf(x0)};
        m_covSE = {latOf(yTop - 2 * Rp), lngOf(x0 + 2 * Rp)};
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
    const QJsonArray segs = P.route().value(u"segments"_s).toArray();
    const QString sel = P.selectedId();

    drawTiles(p, u"osm"_s, 19);
    if (P.showSat()) drawTiles(p, u"satellite"_s, 20);
    // UAS geographical zones over the tiles, under everything else
    if (const GeoZonesPtr zones = GeoZoneController::shownZones(); zones && !zones->empty()) {
        const LL nw = toLL(QPointF(0, 0)), se = toLL(QPointF(width(), height()));
        geozone::paintZones(p, *zones, {nw.lng, se.lat, se.lng, nw.lat}, [this](double lat, double lon) { return toScreen(lat, lon); });
    }
    // Radio coverage and the link halo under the route
    if (P.showLink()) {
        drawCoverage(p);
        drawLinkHalo(p);
    }
    for (const QJsonValue &v : segs) drawFigure(p, v.toObject(), v.toObject().value(u"id"_s).toString() == sel);
    drawPath(p, sel);
    if (P.showCalc()) drawCalculatedPoints(p, segs);
    drawDraftLines(p);
    if (P.showCam()) drawPhotos(p);
    for (const QJsonValue &v : segs)
        if (v.toObject().value(u"id"_s).toString() != sel) drawBasePoints(p, v.toObject(), false);
    drawMarkers(p);
    if (P.showCam()) drawHoveredShot(p);
    drawLabels(p, segs, sel);
    drawPathArrows(p, sel);
    if (const QJsonObject seg = P.segment(sel); !seg.isEmpty()) drawHandles(p, seg);
    drawDraftVertices(p);
    drawProfileCursor(p);
}

void PlanMapItem::drawLinkHalo(QPainter *p)
{
    const auto &A = m_plan->linkAnalysis();
    if (!A) return;
    for (const LinkRun &run : A->runs) {
        if (run.points.size() < 2) continue;
        const bool none = run.clazz == Link::None || run.clazz == Link::Unknown;
        QPen pen = mkPen(linkStyle(run.clazz).color, none ? 5 : 10, none ? 0.7 : 0.45, false);
        if (none) dash(pen, 2, 6);
        p->setPen(pen);
        p->setBrush(Qt::NoBrush);
        p->drawPolyline(toScreen(run.points));
    }
}

void PlanMapItem::drawFigure(QPainter *p, const QJsonObject &seg, bool isSel)
{
    const QString type = seg.value(u"type"_s).toString();
    const QColor color(segmentDef(type).value(u"color"_s).toString());
    const QList<LL> pts = pointsOf(seg);
    const QPolygonF s = toScreen(pts);
    QPen base = mkPen(color, isSel ? 2.5 : 1.5, isSel ? 1 : 0.75);
    if (type == "area"_L1 && s.size() >= 3) {
        dash(base, 6, 4);
        p->setPen(base);
        p->setBrush(withAlpha(color, isSel ? 0.16 : 0.08));
        p->drawPolygon(s);
    } else if (type == "perimeter"_L1 && s.size() >= 3) {
        p->setPen(base);
        p->setBrush(withAlpha(color, 0.02));
        p->drawPolygon(s);
    } else if (type == "corridor"_L1 && pts.size() >= 2) {
        dash(base, 6, 4);
        p->setPen(base);
        p->setBrush(withAlpha(color, isSel ? 0.14 : 0.07));
        p->drawPolygon(toScreen(corridorOutline(pts, corridorWidth(seg))));
        QPen axis = mkPen(color, 1, 0.5);
        dash(axis, 2, 4);
        p->setPen(axis);
        p->setBrush(Qt::NoBrush);
        p->drawPolyline(s);
    } else if (type == "circle"_L1 && !pts.isEmpty()) {
        const double r = circleRadius(seg) / metersPerPixel(pts[0].lat, m_zoom);
        p->setPen(base);
        p->setBrush(withAlpha(color, isSel ? 0.08 : 0.03));
        p->drawEllipse(s[0], r, r);
    }
}

void PlanMapItem::drawPath(QPainter *p, const QString &sel)
{
    // The calculated flight path: home → …, ROI excluded
    const QList<NavPoint> &nav = m_plan->compiled().navPath;
    if (nav.size() < 2) return;
    QPolygonF line;
    for (const NavPoint &n : nav) line.append(toScreen(n.lat, n.lng));
    p->setBrush(Qt::NoBrush);
    p->setPen(mkPen(QColor(0, 0, 0, 140), 5, 0.6));
    p->drawPolyline(line);
    p->setPen(mkPen(QColor(0x44, 0xff, 0x44), 2, 0.95));
    p->drawPolyline(line);
    if (sel.isEmpty()) return;
    // The selected segment's part of the path
    QPolygonF part;
    for (qsizetype i = 0; i < nav.size(); ++i) {
        if (nav[i].segId != sel) continue;
        if (i > 0 && part.isEmpty()) part.append(line[i - 1]);
        part.append(line[i]);
    }
    if (part.size() >= 2) {
        p->setPen(mkPen(Qt::white, 3.5, 0.9));
        p->drawPolyline(part);
    }
}

void PlanMapItem::drawCalculatedPoints(QPainter *p, const QJsonArray &segs)
{
    QHash<QString, QString> typeOf;
    for (const QJsonValue &v : segs) typeOf.insert(v.toObject().value(u"id"_s).toString(), v.toObject().value(u"type"_s).toString());
    const QRectF vis = visibleArea();
    for (const NavPoint &n : m_plan->compiled().navPath) {
        if (n.isHome) continue;
        const QJsonObject def = segmentDef(typeOf.value(n.segId));
        if (def.value(u"points"_s).isDouble() && !n.derived) continue;   // already drawn as the base point
        const QPointF q = toScreen(n.lat, n.lng);
        if (!vis.contains(q)) continue;
        const QColor fill = n.derived ? QColor(0x44, 0xff, 0x44) : QColor(def.value(u"color"_s).toString(u"#44ff44"_s));
        circleMarker(p, q, n.derived ? 2 : 3, n.derived ? QColor(255, 255, 255, 178) : QColor(Qt::white), 1, fill);
    }
}

void PlanMapItem::drawDraftLines(QPainter *p)
{
    const auto &d = m_plan->draft();
    if (!d || d->points.isEmpty()) return;
    const QJsonObject def = segmentDef(d->type);
    const QColor color(def.value(u"color"_s).toString());
    const QPolygonF s = toScreen(d->points);
    if (def.value(u"points"_s).toString() == "poly"_L1 && s.size() >= 3) {
        p->setPen(Qt::NoPen);
        p->setBrush(withAlpha(color, 0.12));
        p->drawPolygon(s);
    }
    QPen pen = mkPen(color, 2);
    dash(pen, 6, 4);
    p->setPen(pen);
    p->setBrush(Qt::NoBrush);
    p->drawPolyline(s);
    if (m_mouseLL) {
        QPen rubber = mkPen(Qt::white, 1, 0.6);
        dash(rubber, 4, 4);
        p->setPen(rubber);
        p->drawLine(s.last(), toScreen(*m_mouseLL));
    }
}

void PlanMapItem::drawDraftVertices(QPainter *p)
{
    const auto &d = m_plan->draft();
    if (!d || d->points.isEmpty()) return;
    const QColor color(segmentDef(d->type).value(u"color"_s).toString());
    for (qsizetype i = 0; i < d->points.size(); ++i) circleMarker(p, toScreen(d->points[i]), i == 0 ? 6 : 4, Qt::white, 1.5, color);
}

void PlanMapItem::drawPhotos(QPainter *p)
{
    // A red dot at every photo, pink where the camera aims at a POI
    const CameraPreview &cam = m_plan->compiled().camera;
    const auto &photos = cam.photos;
    double radius = 4;
    if (photos.size() > 1) {
        const double px = dist(toScreen(photos[0].lat, photos[0].lng), toScreen(photos[1].lat, photos[1].lng));
        radius = std::max(1.5, std::min(4.0, px / 3));
    }
    const QColor red(0xff, 0x30, 0x30), pink(0xff, 0x66, 0xaa);
    const QRectF vis = visibleArea();
    for (const Photo &ph : photos) {
        const QPointF q = toScreen(ph.lat, ph.lng);
        if (vis.contains(q)) circleMarker(p, q, ph.shot ? radius + 1.5 : radius, withAlpha(red, 0.9), 1, red);
    }
    for (const Cone &c : cam.cones)
        if (!c.onBasePoint) circleMarker(p, toScreen(c.from), 4.5, pink, 1, pink);
}

void PlanMapItem::drawBasePoints(QPainter *p, const QJsonObject &seg, bool isSel)
{
    const QString type = seg.value(u"type"_s).toString();
    const QJsonObject def = segmentDef(type);
    const QColor color(def.value(u"color"_s).toString());
    const bool single = def.value(u"points"_s).isDouble();
    const double r = single ? 7 : 4;
    const QList<LL> pts = pointsOf(seg);
    for (const LL &q : pts) {
        circleMarker(p, toScreen(q), isSel ? r + 1 : r, isSel ? QColor(Qt::white) : QColor(255, 255, 255, 217), isSel ? 2 : 1.5,
                     withAlpha(color, single ? 1 : 0.9), type == "poi"_L1);
    }
    if (type == "landing"_L1 && !pts.isEmpty()) landingGlyph(p, toScreen(pts[0]), color);
}

void PlanMapItem::drawMarkers(QPainter *p)
{
    // Take-off point, operator, vehicle; a marker being dragged follows the mouse
    bool custom = false;
    std::optional<LL> home = homePoint(&custom);
    if (m_drag == Drag::Home && m_dragMarker) home = m_dragMarker;
    if (home) homeMarker(p, toScreen(*home), custom);
    std::optional<LL> op = m_plan->placedOperator();
    if (m_drag == Drag::Operator && m_dragMarker) op = m_dragMarker;
    if (op) operatorMarker(p, toScreen(*op));
    if (VehicleHub *hub = m_plan->hub()) {
        const auto s = hub->snapshot();
        if (s->lat || s->lon) vehicleMarker(p, toScreen(s->lat, s->lon), s->yaw * 180 / M_PI);
    }
}

void PlanMapItem::drawHoveredShot(QPainter *p)
{
    // The hovered photo's footprint, or the view wedge of a POI view point
    const CameraPreview &cam = m_plan->compiled().camera;
    auto footprint = [&](const QList<LL> &q) {
        p->setPen(mkPen(QColor(0xff, 0x40, 0x40), 1.5));
        p->setBrush(withAlpha(QColor(0xff, 0x30, 0x30), 0.18));
        p->drawPolygon(toScreen(q));
    };
    if (m_hoverPhoto >= 0 && m_hoverPhoto < cam.photos.size()) {
        const Photo &ph = cam.photos[m_hoverPhoto];
        footprint(ph.poly);
        circleMarker(p, toScreen(ph.lat, ph.lng), 4, Qt::white, 1.5, QColor(0xff, 0x30, 0x30));
    } else if (m_hoverCone >= 0 && m_hoverCone < cam.cones.size()) {
        const Cone &c = cam.cones[m_hoverCone];
        if (c.poly.size() != 4) return;
        QPen wedge = mkPen(QColor(0xff, 0x80, 0xb0), 1);
        dash(wedge, 3, 3);
        p->setPen(wedge);
        p->setBrush(withAlpha(QColor(0xff, 0x66, 0xaa), 0.12));
        p->drawPolygon(QPolygonF({toScreen(c.from), toScreen(c.poly[3]), toScreen(c.poly[2])}));
        footprint(c.poly);
        QPen ray = mkPen(QColor(255, 102, 170, 178), 1);
        dash(ray, 1, 4);
        p->setPen(ray);
        p->drawLine(toScreen(c.from), toScreen(c.target));
    }
}

void PlanMapItem::drawLabels(QPainter *p, const QJsonArray &segs, const QString &sel)
{
    // Number and type chip near the first point of every segment
    const QFont fNum = font(u"Roboto Mono"_s, 10, QFont::Bold);
    const QFont fChip = font(u"Roboto Mono"_s, 8, QFont::Medium);
    for (qsizetype idx = 0; idx < segs.size(); ++idx) {
        const QJsonObject seg = segs[idx].toObject();
        const QList<LL> pts = pointsOf(seg);
        if (pts.isEmpty()) continue;
        const QString type = seg.value(u"type"_s).toString();
        const QJsonObject def = segmentDef(type);
        const bool isSel = seg.value(u"id"_s).toString() == sel;
        // A plan read back from the vehicle is one waypoint per segment: past 40, every 5th
        if (segs.size() > 40 && type == "waypoint"_L1 && !isSel && (idx + 1) % 5 != 0) continue;
        const QColor color(def.value(u"color"_s).toString());
        const QString numText = QString::number(idx + 1);
        const QString chip = type == "waypoint"_L1 ? QString() : def.value(u"short"_s).toString();
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
        if (type == "circle"_L1) drawTurnArrows(p, seg, pts[0], color);
    }
}

void PlanMapItem::drawTurnArrows(QPainter *p, const QJsonObject &seg, const LL &centre, const QColor &color)
{
    // Four arrows on the rim of a circle, pointing along the turn
    const double r = circleRadius(seg);
    const LocalFrame f(centre);
    const double turn = seg.value(u"params"_s).toObject().value(u"direction"_s).toString() == "ccw"_L1 ? -90 : 90;
    for (double a : {45.0, 135.0, 225.0, 315.0}) {
        const LL ll = f.toLL({r * std::sin(qDegreesToRadians(a)), r * std::cos(qDegreesToRadians(a))});
        arrow(p, toScreen(ll), std::fmod(a + turn + 360, 360), color);
    }
}

void PlanMapItem::drawPathArrows(QPainter *p, const QString &sel)
{
    // Direction arrows along the calculated path, every 70 px on screen
    const QList<NavPoint> &nav = m_plan->compiled().navPath;
    constexpr double kMinPx = 70;
    const QRectF vis = visibleArea();
    double carry = 0;
    for (qsizetype i = 1; i < nav.size(); ++i) {
        const QPointF a = toScreen(nav[i - 1].lat, nav[i - 1].lng), b = toScreen(nav[i].lat, nav[i].lng);
        const double len = dist(a, b);
        if (len < 4) continue;
        double pos = kMinPx / 2 - carry;
        const double brg = bearing(nav[i - 1].ll(), nav[i].ll());
        const bool isSel = !sel.isEmpty() && nav[i].segId == sel;
        for (; pos < len; pos += kMinPx) {
            const QPointF at = a + (b - a) * (pos / len);
            if (vis.contains(at)) arrow(p, at, brg, isSel ? QColor(Qt::white) : QColor(68, 255, 68, 230));
        }
        carry = len - (pos - kMinPx);
    }
}

void PlanMapItem::drawHandles(QPainter *p, const QJsonObject &seg)
{
    // The selected segment: its points on top, a handle mid-edge and in the middle, the circle's rim
    drawBasePoints(p, seg, true);
    const QString type = seg.value(u"type"_s).toString();
    const QJsonObject def = segmentDef(type);
    const QColor color(def.value(u"color"_s).toString());
    const QList<LL> pts = pointsOf(seg);
    const QString kind = def.value(u"points"_s).toString();
    if ((kind == "poly"_L1 || kind == "line"_L1) && pts.size() >= 2) {
        for (const LL &m : edgeMidpoints(pts, kind == "poly"_L1)) circleMarker(p, toScreen(m), 4, Qt::white, 1, withAlpha(color, 0.45));
        const QPointF cs = toScreen(*centroid(pts));
        p->setPen(Qt::NoPen);
        p->setBrush(QColor(0, 210, 255, 60));
        p->drawEllipse(cs, 11, 11);
        p->setPen(QPen(QColor(0x00, 0xd2, 0xff), 2));
        p->setBrush(QColor(0, 210, 255, 89));
        p->drawEllipse(cs, 8, 8);
    }
    if (type == "circle"_L1 && !pts.isEmpty()) circleMarker(p, toScreen(rimHandle(seg)), 5, Qt::white, 1.5, color);
}

void PlanMapItem::drawProfileCursor(QPainter *p)
{
    // The profile's cursor on the map, and the line of sight to it
    const PlanController &P = *m_plan;
    const auto &h = P.profileHover();
    if (!h) return;
    if (P.showLink() && P.groundStation() && P.profileHoverClass()) {
        QPen los = mkPen(linkStyle(*P.profileHoverClass()).color, 1.5, 0.9);
        dash(los, 5, 5);
        p->setPen(los);
        p->drawLine(toScreen(P.groundStation()->lat, P.groundStation()->lng), toScreen(*h));
    }
    circleMarker(p, toScreen(*h), 6, QColor(0x00, 0xd2, 0xff), 2, QColor(0, 210, 255, 102));
}

// ── Hit testing ──────────────────────────────────────────────────────────────

PlanMapItem::Hit PlanMapItem::hitTest(QPointF s, bool forContext) const
{
    if (!m_plan) return {};
    if (!forContext) {
        // The selected segment's handles (top-most), then the operator and the take-off point
        const QString sel = m_plan->selectedId();
        if (const QJsonObject seg = m_plan->segment(sel); !seg.isEmpty()) {
            if (const Hit h = hitHandles(s, seg, sel); h.kind != Hit::None) return h;
        }
        if (const auto op = m_plan->placedOperator(); op && dist(s, toScreen(*op)) <= 14) return {Hit::Operator, {}, -1};
        if (const auto home = homePoint(); home && dist(s, toScreen(*home)) <= 14) return {Hit::Home, {}, -1};
    }
    // Base points, then figures, top-most first
    const QJsonArray segs = m_plan->route().value(u"segments"_s).toArray();
    for (qsizetype k = segs.size() - 1; k >= 0; --k) {
        const QJsonObject seg = segs[k].toObject();
        const double r = (segmentDef(seg.value(u"type"_s).toString()).value(u"points"_s).isDouble() ? 7 : 4) + 3;
        const QList<LL> pts = pointsOf(seg);
        for (qsizetype i = pts.size() - 1; i >= 0; --i)
            if (dist(s, toScreen(pts[i])) <= r) return {Hit::Point, seg.value(u"id"_s).toString(), int(i)};
    }
    for (qsizetype k = segs.size() - 1; k >= 0; --k)
        if (onFigure(s, segs[k].toObject())) return {Hit::Figure, segs[k].toObject().value(u"id"_s).toString(), -1};
    return {};
}

PlanMapItem::Hit PlanMapItem::hitHandles(QPointF s, const QJsonObject &seg, const QString &id) const
{
    const QString type = seg.value(u"type"_s).toString();
    const QJsonObject def = segmentDef(type);
    const QList<LL> pts = pointsOf(seg);
    const QString kind = def.value(u"points"_s).toString();
    if (type == "circle"_L1 && !pts.isEmpty() && dist(s, toScreen(rimHandle(seg))) <= 7) return {Hit::Radius, id, 0};
    if ((kind == "poly"_L1 || kind == "line"_L1) && pts.size() >= 2) {
        if (dist(s, toScreen(*centroid(pts))) <= 10) return {Hit::Centre, id, -1};
        const QList<LL> mids = edgeMidpoints(pts, kind == "poly"_L1);
        for (qsizetype i = 0; i < mids.size(); ++i)
            if (dist(s, toScreen(mids[i])) <= 6) return {Hit::Mid, id, int(i)};
    }
    const double r = (def.value(u"points"_s).isDouble() ? 7 : 4) + 4;
    for (qsizetype i = pts.size() - 1; i >= 0; --i)
        if (dist(s, toScreen(pts[i])) <= r) return {Hit::Vertex, id, int(i)};
    return {};
}

bool PlanMapItem::onFigure(QPointF s, const QJsonObject &seg) const
{
    const QString type = seg.value(u"type"_s).toString();
    const QList<LL> pts = pointsOf(seg);
    const QPolygonF sc = toScreen(pts);
    if ((type == "area"_L1 || type == "perimeter"_L1) && sc.size() >= 3) {
        if (sc.containsPoint(s, Qt::OddEvenFill)) return true;
        for (qsizetype i = 0; i < sc.size(); ++i)
            if (segDist(s, sc[i], sc[(i + 1) % sc.size()]) <= 4) return true;
        return false;
    }
    if (type == "corridor"_L1 && pts.size() >= 2) return toScreen(corridorOutline(pts, corridorWidth(seg))).containsPoint(s, Qt::OddEvenFill);
    if (type == "circle"_L1 && !pts.isEmpty()) return dist(s, sc[0]) <= circleRadius(seg) / metersPerPixel(pts[0].lat, m_zoom) + 3;
    return false;
}

// ── Mouse ────────────────────────────────────────────────────────────────────

void PlanMapItem::openContextMenu(QPointF pos)
{
    const LL ll = toLL(pos);
    QVariantList items;
    auto add = [&](const char *act, const QString &label, bool danger = false) { items.append(QVariantList{QString::fromLatin1(act), label, danger}); };
    QString segId;
    int pointIdx = -1;
    if (m_plan->draft()) {
        add("finish", u"Finish drawing"_s);
        add("cancel", u"Cancel drawing"_s);
    } else if (const Hit h = hitTest(pos, true); h.kind == Hit::Point || h.kind == Hit::Figure) {
        segId = h.segId;
        pointIdx = h.kind == Hit::Point ? h.index : -1;
        const QJsonObject seg = m_plan->segment(segId);
        const QJsonObject def = segmentDef(seg.value(u"type"_s).toString());
        add("select", u"Select %1"_s.arg(def.value(u"label"_s).toString().toLower()));
        const QString kind = def.value(u"points"_s).toString();
        if (pointIdx >= 0 && (kind == "poly"_L1 || kind == "line"_L1)) {
            add("insert", u"Insert vertex after"_s);
            if (pointsOf(seg).size() > (kind == "poly"_L1 ? 3 : 2)) add("delvtx", u"Delete vertex"_s, true);
        }
        add("delete", u"Delete segment"_s, true);
    } else {
        add("wp", u"Add waypoint here"_s);
        add("poi", u"Add POI here"_s);
        add("home", u"Take-off point here"_s);
        add("operator", u"Operator here"_s);
        if (!m_plan->selectedId().isEmpty()) add("deselect", u"Deselect"_s);
    }
    emit contextMenu(pos.x(), pos.y(), items, ll.lat, ll.lng, segId, pointIdx);
}

void PlanMapItem::mousePressEvent(QMouseEvent *e)
{
    if (!m_plan) return;
    forceActiveFocus();
    const QPointF pos = e->position();
    e->accept();
    if (e->button() == Qt::RightButton) {
        openContextMenu(pos);
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
        m_gestureLabel = u"Move point"_s;
        break;
    case Hit::Mid: {
        // Insert a vertex and drag it at once
        const QList<LL> pts = pointsOf(m_plan->segment(m_pressHit.segId));
        const LL a = pts[m_pressHit.index], b = pts[(m_pressHit.index + 1) % pts.size()];
        m_plan->insertPoint(m_pressHit.segId, m_pressHit.index, (a.lat + b.lat) / 2, (a.lng + b.lng) / 2);
        m_pressHit = {Hit::Vertex, m_pressHit.segId, m_pressHit.index + 1};
        m_drag = Drag::Vertex;
        m_gestureLabel = u"Insert vertex"_s;
        m_moved = true;
        break;
    }
    case Hit::Centre:
        m_drag = Drag::Centre;
        m_dragOriginal = pointsOf(m_plan->segment(m_pressHit.segId));
        m_dragStartLL = toLL(pos);
        m_gestureLabel = u"Move segment"_s;
        break;
    case Hit::Radius:
        m_drag = Drag::Radius;
        m_gestureLabel = u"Change radius"_s;
        break;
    case Hit::Home: m_drag = Drag::Home; break;
    case Hit::Operator: m_drag = Drag::Operator; break;
    default: m_drag = Drag::Pan; break;
    }
    setCursor(m_drag == Drag::Pan ? toolCursor(Qt::ClosedHandCursor) : Qt::ClosedHandCursor);
}

void PlanMapItem::mouseMoveEvent(QMouseEvent *e)
{
    if (!m_plan) return;
    const QPointF pos = e->position();
    const QPointF d = pos - m_press;
    const bool overThreshold = m_moved || std::hypot(d.x(), d.y()) >= 3;
    m_mouseLL = toLL(pos);
    switch (m_drag) {
    case Drag::Pan:
        if (!overThreshold) return;
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
    case Drag::Radius:
        m_moved = true;
        if (const QList<LL> pts = pointsOf(m_plan->segment(m_pressHit.segId)); !pts.isEmpty())
            m_plan->setCircleRadius(m_pressHit.segId, haversine(pts[0], *m_mouseLL));
        break;
    case Drag::Home:
    case Drag::Operator:
        if (!overThreshold) return;
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
    setCursor(toolCursor(Qt::OpenHandCursor));
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
        if (m_moved) {
            emit viewChanged();
        } else if (m_plan->tool() == "select"_L1 && (m_pressHit.kind == Hit::Point || m_pressHit.kind == Hit::Figure)) {
            m_plan->selectFromMap(m_pressHit.segId);
        } else {
            const LL ll = toLL(pos);
            const auto &d = m_plan->draft();
            m_plan->mapClick(ll.lat, ll.lng, d && !d->points.isEmpty() && dist(toScreen(d->points.first()), pos) < 12);
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
    const GeoZonesPtr zones = GeoZoneController::shownZones();
    if (const QString tip = zones ? geozone::describeAt(*zones, m_mouseLL->lat, m_mouseLL->lng) : QString(); tip != m_zoneTip) {
        m_zoneTip = tip;
        emit zoneTipChanged();
    }
    // The photo dot under the mouse, else a POI view point: its own dot, or a base point it sits on
    int photo = -1, cone = -1;
    if (m_plan->showCam()) {
        const auto &C = m_plan->compiled().camera;
        double best = 7;
        for (qsizetype i = 0; i < C.photos.size(); ++i) {
            const double d = dist(toScreen(C.photos[i].lat, C.photos[i].lng), pos);
            if (d < best) { best = d; photo = int(i); }
        }
        for (qsizetype i = 0; photo < 0 && cone < 0 && i < C.cones.size(); ++i)
            if (dist(toScreen(C.cones[i].from), pos) <= (C.cones[i].onBasePoint ? 9 : 6)) cone = int(i);
    }
    const bool hoverChanged = photo != m_hoverPhoto || cone != m_hoverCone;
    m_hoverPhoto = photo;
    m_hoverCone = cone;
    if (m_plan->tool() == "select"_L1) {
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
