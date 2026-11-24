#include "ui/MapItem.h"

#include "scene/CellOverlay.h"
#include "terrain/TileStore.h"
#include "ui/GeoZoneController.h"
#include "ui/GeoZonePainter.h"
#include "vehicle/VehicleHub.h"

#include <QNativeGestureEvent>
#include <QPainter>
#include <QFontMetricsF>
#include <QPainterPath>
#include <algorithm>
#include <cmath>

using namespace Qt::StringLiterals;

namespace corv {

VehicleHub *MapItem::s_hub = nullptr;
std::vector<LatLon> MapItem::s_frozen;
int MapItem::s_frozenEpoch = 0;
std::vector<MapItem::MissionPoint> MapItem::s_mission;
std::optional<LatLon> MapItem::s_target;

void MapItem::setFrozenTrail(std::vector<LatLon> points)
{
    s_frozen = std::move(points);
    ++s_frozenEpoch;
}

void MapItem::setMission(std::vector<MissionPoint> points) { s_mission = std::move(points); }
void MapItem::setTarget(std::optional<LatLon> target) { s_target = target; }

namespace {
constexpr int kTile = 256;
constexpr std::size_t kTrailMax = 3000;
constexpr double kTrailMaxJumpM = 2000;
constexpr double kMinZoom = 2, kMaxZoom = 20;
constexpr int kMaxTileZoom = 19;
constexpr double kMouseSlopPx = 4;
const QString kProvider = u"satellite"_s;
} // namespace

MapItem::MapItem(QQuickItem *parent) : QQuickPaintedItem(parent)
{
    setAcceptedMouseButtons(Qt::LeftButton);
    setAcceptTouchEvents(true);
    setAcceptHoverEvents(true);
    setAntialiasing(true);
    setFillColor(Qt::black);
    m_timer.setInterval(100);
    connect(&m_timer, &PacedTimer::timeout, this, &MapItem::sample);
    m_timer.start();
    connect(&TileStore::instance(), &TileStore::tileReady, this, [this](const QString &p, int z) {
        if (p == kProvider && std::abs(z - m_zoom) < 1.5) update();
    });
}

void MapItem::zoomAt(double z, QPointF anchor)
{
    z = std::clamp(z, kMinZoom, kMaxZoom);
    if (z == m_zoom) return;
    const QPointF fromCentre = m_follow ? QPointF() : anchor - QPointF(width() / 2, height() / 2);
    const LatLon fixed = unproject(project(m_center.lat, m_center.lon) + fromCentre);
    m_zoom = z;
    m_center = unproject(project(fixed.lat, fixed.lon) - fromCentre);
    emit zoomChanged();
    update();
}

void MapItem::zoomIn() { zoomAt(std::floor(m_zoom + 1e-6) + 1, QPointF(width() / 2, height() / 2)); }
void MapItem::zoomOut() { zoomAt(std::ceil(m_zoom - 1e-6) - 1, QPointF(width() / 2, height() / 2)); }

void MapItem::setFollow(bool f)
{
    if (f == m_follow) return;
    m_follow = f;
    if (f && s_hub) {
        const auto s = s_hub->snapshot();
        m_center = {s->lat, s->lon};
    }
    emit followChanged();
    update();
}

void MapItem::setPanEnabled(bool on)
{
    if (on == m_pan) return;
    m_pan = on;
    // Back in the corner panel: on the vehicle again
    if (!on) setFollow(true);
    emit panEnabledChanged();
}

QPointF MapItem::project(double lat, double lon) const
{
    const double n = kTile * std::pow(2.0, m_zoom);
    const double s = std::sin(std::clamp(lat, -85.0511, 85.0511) * kDeg);
    return {(lon + 180.0) / 360.0 * n, (0.5 - std::log((1 + s) / (1 - s)) / (4 * kPi)) * n};
}

LatLon MapItem::unproject(QPointF w) const
{
    const double n = kTile * std::pow(2.0, m_zoom);
    const double lon = w.x() / n * 360.0 - 180.0;
    const double lat = std::atan(std::sinh(kPi * (1 - 2 * w.y() / n))) * kRad;
    return {lat, lon};
}

void MapItem::sample()
{
    if (!s_hub) return;
    const auto s = s_hub->snapshot();
    const LatLon here{s->lat, s->lon};
    // Relative navigation starts a track of its own, and so does leaving it
    if (s->relativeMode != m_wasRelative) {
        m_wasRelative = s->relativeMode;
        m_trail.clear();
    }
    // LOG REPLAY: the recorded flight replaces the trail; unloaded, it starts again
    if (m_frozenEpoch != s_frozenEpoch) {
        m_frozenEpoch = s_frozenEpoch;
        m_trail.assign(s_frozen.begin(), s_frozen.end());
    }
    // A jump is no flight: a new connection after the demo, another vehicle
    if (s_frozen.empty() && !m_trail.empty() && flatDistance(m_trail.back().lat, m_trail.back().lon, here.lat, here.lon) > kTrailMaxJumpM) m_trail.clear();
    if (!s_frozen.empty()) {
        // frozen: the live samples do not add to it
    } else if (m_trail.empty() || flatDistance(m_trail.back().lat, m_trail.back().lon, here.lat, here.lon) > 2) {
        m_trail.push_back(here);
        if (m_trail.size() > kTrailMax) m_trail.pop_front();
    }
    if (m_follow) m_center = here;
    update();
}

void MapItem::paint(QPainter *p)
{
    const QPointF origin = project(m_center.lat, m_center.lon) - QPointF(width() / 2, height() / 2);
    const auto s = s_hub ? s_hub->snapshot() : nullptr;
    // Relative navigation: metres from a synthetic origin, a real map around them would lie
    const bool relative = s && s->relativeMode;
    if (!relative) drawTiles(p, origin);
    if (!s) return;
    auto toScreen = [&](double lat, double lon) { return project(lat, lon) - origin; };

    // UAS geographical zones under the track and the markers
    if (const GeoZonesPtr zones = GeoZoneController::shownZones(); !relative && zones && !zones->empty()) {
        const LatLon nw = unproject(origin), se = unproject(origin + QPointF(width(), height()));
        geozone::paintZones(p, *zones, {nw.lon, se.lat, se.lon, nw.lat}, toScreen);
    }
    drawCells(p, *s, toScreen);

    if (!relative) drawMission(p, toScreen);

    // Trail: red polyline
    if (m_trail.size() > 1) {
        QPainterPath path;
        path.moveTo(toScreen(m_trail.front().lat, m_trail.front().lon));
        for (std::size_t i = 1; i < m_trail.size(); ++i) path.lineTo(toScreen(m_trail[i].lat, m_trail[i].lon));
        p->setPen(QPen(QColor(255, 0, 0, 217), 2));
        p->setBrush(Qt::NoBrush);
        p->drawPath(path);
    }
    // Home
    if (s->homeLat && s->homeLon) {
        const QPointF h = toScreen(*s->homeLat, *s->homeLon);
        p->setPen(QPen(Qt::white, 1.5));
        p->setBrush(QColor(0, 210, 255, 180));
        p->drawEllipse(h, 6, 6);
        p->setPen(Qt::black);
        QFont f = p->font();
        f.setPixelSize(9);
        f.setBold(true);
        p->setFont(f);
        p->drawText(QRectF(h.x() - 6, h.y() - 6, 12, 12), Qt::AlignCenter, u"H"_s);
    }
    // Target (GCS OPTIONS -> TARGET): a red dot in a ring
    if (s_target && !relative) {
        const QPointF t = toScreen(s_target->lat, s_target->lon);
        p->setPen(QPen(QColor(255, 0, 0, 128), 2));
        p->setBrush(QColor(255, 0, 0, 38));
        p->drawEllipse(t, 14, 14);
        p->setPen(QPen(QColor(255, 0, 0), 2));
        p->setBrush(QColor(255, 0, 0, 230));
        p->drawEllipse(t, 6, 6);
    }
    if (!relative) drawTraffic(p, *s, toScreen);
    drawVehicle(p, toScreen(s->lat, s->lon), s->yaw);

    // Attribution and zoom
    p->setPen(QColor(255, 255, 255, 170));
    QFont f = p->font();
    f.setPixelSize(10);
    p->setFont(f);
    p->drawText(QRectF(4, height() - 16, width() - 8, 14), Qt::AlignRight | Qt::AlignVCenter,
                u"z%1%2"_s.arg(QString::number(m_zoom, 'f', std::abs(m_zoom - std::round(m_zoom)) < 0.05 ? 0 : 1),
                               m_follow ? QString() : u"  (double-click: follow)"_s));
}

void MapItem::drawTiles(QPainter *p, QPointF origin)
{
    // Tiles at 0.6 opacity over black, as the satellite layer of MapEngine.js;
    // a missing one borrows its parent at one zoom less, scaled
    // (between two zoom levels, the nearest one's tiles scaled)
    const int z = std::clamp(int(std::lround(m_zoom)), int(kMinZoom), kMaxTileZoom);
    const double ts = kTile * std::pow(2.0, m_zoom - z);
    const int maxIndex = (1 << z) - 1;
    // Edges on whole device pixels: no seams between scaled tiles
    const double dpr = p->device() ? p->device()->devicePixelRatioF() * p->worldTransform().m11() : 1.0;
    auto snap = [dpr](double v) { return std::round(v * dpr) / dpr; };
    p->setRenderHint(QPainter::SmoothPixmapTransform, ts != kTile);
    p->setOpacity(0.6);
    const int tx0 = int(std::floor(origin.x() / ts)), ty0 = int(std::floor(origin.y() / ts));
    const int tx1 = int(std::floor((origin.x() + width()) / ts)), ty1 = int(std::floor((origin.y() + height()) / ts));
    for (int ty = std::max(ty0, 0); ty <= std::min(ty1, maxIndex); ++ty) {
        for (int tx = tx0; tx <= tx1; ++tx) {
            const int x = ((tx % (maxIndex + 1)) + maxIndex + 1) % (maxIndex + 1);
            const QRectF dst(QPointF(snap(tx * ts - origin.x()), snap(ty * ts - origin.y())),
                             QPointF(snap((tx + 1) * ts - origin.x()), snap((ty + 1) * ts - origin.y())));
            if (const QImage img = TileStore::instance().tile(kProvider, z, x, ty); !img.isNull()) p->drawImage(dst, img);
            else if (const QImage parent = TileStore::instance().tile(kProvider, z - 1, x / 2, ty / 2); !parent.isNull())
                p->drawImage(dst, parent, QRectF((x % 2) * 128, (ty % 2) * 128, 128, 128));
        }
    }
    p->setOpacity(1.0);
}

void MapItem::drawCells(QPainter *p, const VehicleState &s, const std::function<QPointF(double, double)> &toScreen)
{
    // LTE cell sites (CellTowerLayer.js), over the tiles and under the track: the other
    // sites as dots (brighter on the serving network), the serving one as a ring in the
    // colour of the link quality with a dashed line from the vehicle to it
    const CellOverlayPtr cells = cellOverlay::get();
    if (!cells || !cells->live || s.relativeMode) return;
    for (std::size_t i = 0; i < cells->sites.size(); ++i) {
        if (int(i) == cells->serving) continue;
        const CellSite &site = cells->sites[i];
        const bool own = site.mcc == cells->ownMcc && site.mnc == cells->ownMnc;
        QColor stroke = own ? QColor(0xe8e8e8) : QColor(0x7a8ca0), fill = own ? QColor(0xffffff) : QColor(0x7a8ca0);
        stroke.setAlphaF(0.9f);
        fill.setAlphaF(own ? 0.8f : 0.5f);
        p->setPen(QPen(stroke, 1));
        p->setBrush(fill);
        const double r = own ? 3 : 2.5;
        p->drawEllipse(toScreen(site.lat, site.lon), r, r);
    }
    if (cells->serving < 0 || cells->serving >= int(cells->sites.size())) return;
    const CellSite &site = cells->sites[std::size_t(cells->serving)];
    const QPointF mast = toScreen(site.lat, site.lon);
    if (s.lat != 0 || s.lon != 0) {
        QPen dash(cells->linkColor, 2);
        dash.setDashPattern({4, 3});   // 8 px on, 6 off
        p->setPen(dash);
        p->drawLine(toScreen(s.lat, s.lon), mast);
    }
    QColor fill = cells->linkColor;
    fill.setAlphaF(0.35f);
    p->setPen(QPen(cells->linkColor, 2));
    p->setBrush(fill);
    p->drawEllipse(mast, 7, 7);
}

void MapItem::drawMission(QPainter *p, const std::function<QPointF(double, double)> &toScreen)
{
    // updateMissionOverlay of MapEngine.js: a thin dashed line, a dot on every point, home in orange
    if (s_mission.empty()) return;
    const QColor green(0x44, 0xff, 0x44);
    if (s_mission.size() >= 2) {
        QPolygonF line;
        for (const MissionPoint &m : s_mission) line << toScreen(m.at.lat, m.at.lon);
        QPen pen(QColor(green.red(), green.green(), green.blue(), 128), 1);
        pen.setDashPattern({4, 4});
        p->setPen(pen);
        p->setBrush(Qt::NoBrush);
        p->drawPolyline(line);
    }
    const QRectF view(-8, -8, width() + 16, height() + 16);
    for (const MissionPoint &m : s_mission) {
        const QPointF at = toScreen(m.at.lat, m.at.lon);
        if (!view.contains(at)) continue;
        const QColor c = m.home ? QColor(0xff, 0x88, 0x00) : green;
        p->setPen(QPen(c, 1));
        p->setBrush(QColor(c.red(), c.green(), c.blue(), m.home ? 230 : 204));
        p->drawEllipse(at, m.home ? 5 : 3, m.home ? 5 : 3);
    }
}

void MapItem::drawTraffic(QPainter *p, const VehicleState &s, const std::function<QPointF(double, double)> &toScreen)
{
    const TrafficEntry *under = nullptr;
    double best = 10;   // px
    for (const TrafficEntry &t : s.traffic) {
        const QPointF at = toScreen(t.lat, t.lon);
        p->setPen(QPen(QColor(255, 0, 0), 1));
        p->setBrush(QColor(255, 0, 0, 230));
        p->drawEllipse(at, 5, 5);
        if (!m_hover || t.callsign.trimmed().isEmpty()) continue;
        if (const double d = std::hypot(at.x() - m_hover->x(), at.y() - m_hover->y()); d < best) {
            best = d;
            under = &t;
        }
    }
    if (!under) return;
    // The callsign over the aircraft under the mouse (traffic-tooltip)
    const QPointF at = toScreen(under->lat, under->lon);
    QFont f = p->font();
    f.setPixelSize(11);
    f.setBold(true);
    p->setFont(f);
    const QString text = under->callsign.trimmed();
    const double w = QFontMetricsF(f).horizontalAdvance(text) + 12, h = 18;
    const QRectF box(std::clamp(at.x() - w / 2, 0.0, std::max(0.0, width() - w)), std::max(0.0, at.y() - h - 8), w, h);
    p->setPen(QPen(QColor(255, 0, 0, 200), 1));
    p->setBrush(QColor(0, 0, 0, 210));
    p->drawRoundedRect(box, 3, 3);
    p->setPen(QColor(255, 90, 90));
    p->drawText(box, Qt::AlignCenter, text);
}

void MapItem::drawVehicle(QPainter *p, QPointF at, double yaw)
{
    // The white aircraft of MapEngine.js (40 px, 24-unit SVG path, white glow)
    static const QPainterPath plane = [] {
        QPainterPath q;
        q.moveTo(21, 16); q.lineTo(21, 14); q.lineTo(13, 9); q.lineTo(13, 3.5);
        q.arcTo(QRectF(10, 2, 3, 3), 0, 180);
        q.lineTo(10, 9); q.lineTo(2, 14); q.lineTo(2, 16); q.lineTo(10, 13.5); q.lineTo(10, 19);
        q.lineTo(8, 20.5); q.lineTo(8, 22); q.lineTo(11.5, 21); q.lineTo(15, 22); q.lineTo(15, 20.5);
        q.lineTo(13, 19); q.lineTo(13, 13.5); q.closeSubpath();
        return q;
    }();
    p->save();
    p->translate(at);
    p->rotate(yaw * kRad);
    p->scale(40.0 / 24.0, 40.0 / 24.0);
    p->translate(-11.5, -12);
    p->setBrush(Qt::NoBrush);
    for (int i = 3; i >= 1; --i) {
        p->setPen(QPen(QColor(255, 255, 255, 40), i * 1.2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        p->drawPath(plane);
    }
    p->setPen(Qt::NoPen);
    p->setBrush(Qt::white);
    p->drawPath(plane);
    p->restore();
}

// ── Input ────────────────────────────────────────────────────────────────────

void MapItem::panFrom(QPointF press)
{
    m_press = press;
    m_pressCenter = m_center;
}

void MapItem::panTo(QPointF p)
{
    if (!m_pan) return;
    setFollow(false);
    m_center = unproject(project(m_pressCenter.lat, m_pressCenter.lon) - (p - m_press));
    update();
}

void MapItem::click(QPointF p)
{
    const LatLon ll = unproject(project(m_center.lat, m_center.lon) + p - QPointF(width() / 2, height() / 2));
    emit clicked(ll.lat, ll.lon);
}

void MapItem::wheelEvent(QWheelEvent *e)
{
    zoomAt(TouchGesture::wheelZoom(e, m_zoom), e->position());
    e->accept();
}

bool MapItem::event(QEvent *e)
{
    if (e->type() == QEvent::NativeGesture) {
        auto *g = static_cast<QNativeGestureEvent *>(e);
        if (double levels = 0; TouchGesture::nativeZoom(g, levels)) {
            zoomAt(m_zoom + levels, g->position());
            return true;
        }
    }
    return QQuickPaintedItem::event(e);
}

void MapItem::mousePressEvent(QMouseEvent *e)
{
    panFrom(e->position());
    m_pressScene = e->scenePosition();
    m_dragged = false;
    e->accept();
}

void MapItem::mouseMoveEvent(QMouseEvent *e)
{
    // In the window: the corner panel grows under the cursor, which is no drag
    const QPointF d = e->scenePosition() - m_pressScene;
    if (!m_dragged && std::hypot(d.x(), d.y()) < kMouseSlopPx) return;
    m_dragged = true;
    panTo(m_press + d);
}

void MapItem::mouseReleaseEvent(QMouseEvent *e)
{
    if (!m_dragged) click(e->position());
}

void MapItem::mouseDoubleClickEvent(QMouseEvent *)
{
    setFollow(true);
}

void MapItem::hoverMoveEvent(QHoverEvent *e)
{
    m_hover = e->position();
    e->ignore();   // the panel and its buttons see it too
}

void MapItem::hoverLeaveEvent(QHoverEvent *e)
{
    m_hover.reset();
    e->ignore();
}

void MapItem::touchDragBegin(QPointF press) { panFrom(press); }
void MapItem::touchDrag(QPointF p) { panTo(p); }
void MapItem::touchTap(QPointF p) { click(p); }
void MapItem::touchDoubleTap(QPointF) { setFollow(true); }

void MapItem::pinch(QPointF centre, QPointF moved, double scale)
{
    // Two fingers never take the map off the vehicle; away from it they move it too
    if (!m_follow) m_center = unproject(project(m_center.lat, m_center.lon) - moved);
    zoomAt(m_zoom + std::log2(scale), centre);
    update();
}

} // namespace corv
