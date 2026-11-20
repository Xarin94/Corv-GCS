#include "ui/MapItem.h"

#include "scene/CellOverlay.h"
#include "terrain/TileStore.h"
#include "ui/GeoZoneController.h"
#include "ui/GeoZonePainter.h"
#include "vehicle/VehicleHub.h"

#include <QPainter>
#include <QPainterPath>
#include <cmath>

using namespace Qt::StringLiterals;

namespace corv {

VehicleHub *MapItem::s_hub = nullptr;
std::vector<LatLon> MapItem::s_frozen;
int MapItem::s_frozenEpoch = 0;

void MapItem::setFrozenTrail(std::vector<LatLon> points)
{
    s_frozen = std::move(points);
    ++s_frozenEpoch;
}

namespace {
constexpr int kTile = 256;
constexpr std::size_t kTrailMax = 3000;
const QString kProvider = u"satellite"_s;
} // namespace

MapItem::MapItem(QQuickItem *parent) : QQuickPaintedItem(parent)
{
    setAcceptedMouseButtons(Qt::LeftButton);
    setAntialiasing(true);
    setFillColor(Qt::black);
    m_timer.setInterval(100);
    connect(&m_timer, &PacedTimer::timeout, this, &MapItem::sample);
    m_timer.start();
    connect(&TileStore::instance(), &TileStore::tileReady, this, [this](const QString &p, int z) {
        if (p == kProvider && z == m_zoom) update();
    });
}

void MapItem::setZoom(int z)
{
    z = std::clamp(z, 2, 19);
    if (z == m_zoom) return;
    m_zoom = z;
    update();
}

void MapItem::setFollow(bool f)
{
    if (f == m_follow) return;
    m_follow = f;
    update();
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
    // LOG REPLAY: the recorded flight replaces the trail; unloaded, it starts again
    if (m_frozenEpoch != s_frozenEpoch) {
        m_frozenEpoch = s_frozenEpoch;
        m_trail.assign(s_frozen.begin(), s_frozen.end());
    }
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
    drawTiles(p, origin);
    if (!s_hub) return;
    const auto s = s_hub->snapshot();
    auto toScreen = [&](double lat, double lon) { return project(lat, lon) - origin; };

    // UAS geographical zones under the track and the markers
    if (const GeoZonesPtr zones = GeoZoneController::shownZones(); zones && !zones->empty()) {
        const LatLon nw = unproject(origin), se = unproject(origin + QPointF(width(), height()));
        geozone::paintZones(p, *zones, {nw.lon, se.lat, se.lon, nw.lat}, toScreen);
    }
    drawCells(p, *s, toScreen);

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
    // ADS-B traffic
    for (const TrafficEntry &t : s->traffic) {
        p->setPen(Qt::NoPen);
        p->setBrush(QColor(255, 170, 0));
        p->drawEllipse(toScreen(t.lat, t.lon), 4, 4);
    }
    drawVehicle(p, toScreen(s->lat, s->lon), s->yaw);

    // Attribution and zoom
    p->setPen(QColor(255, 255, 255, 170));
    QFont f = p->font();
    f.setPixelSize(10);
    p->setFont(f);
    p->drawText(QRectF(4, height() - 16, width() - 8, 14), Qt::AlignRight | Qt::AlignVCenter,
                u"z%1%2"_s.arg(m_zoom).arg(m_follow ? QString() : u"  (double-click: follow)"_s));
}

void MapItem::drawTiles(QPainter *p, QPointF origin)
{
    // Tiles at 0.6 opacity over black, as the satellite layer of MapEngine.js;
    // a missing one borrows its parent at one zoom less, scaled
    const int maxIndex = (1 << m_zoom) - 1;
    p->setOpacity(0.6);
    const int tx0 = int(std::floor(origin.x() / kTile)), ty0 = int(std::floor(origin.y() / kTile));
    const int tx1 = int(std::floor((origin.x() + width()) / kTile)), ty1 = int(std::floor((origin.y() + height()) / kTile));
    for (int ty = std::max(ty0, 0); ty <= std::min(ty1, maxIndex); ++ty) {
        for (int tx = tx0; tx <= tx1; ++tx) {
            const int x = ((tx % (maxIndex + 1)) + maxIndex + 1) % (maxIndex + 1);
            const QRectF dst(tx * kTile - origin.x(), ty * kTile - origin.y(), kTile, kTile);
            if (const QImage img = TileStore::instance().tile(kProvider, m_zoom, x, ty); !img.isNull()) p->drawImage(dst, img);
            else if (const QImage parent = TileStore::instance().tile(kProvider, m_zoom - 1, x / 2, ty / 2); !parent.isNull())
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

void MapItem::wheelEvent(QWheelEvent *e)
{
    setZoom(m_zoom + (e->angleDelta().y() > 0 ? 1 : -1));
    e->accept();
}

void MapItem::mousePressEvent(QMouseEvent *e)
{
    m_press = e->position();
    m_pressCenter = m_center;
    m_dragged = false;
    e->accept();
}

void MapItem::mouseMoveEvent(QMouseEvent *e)
{
    const QPointF d = e->position() - m_press;
    if (!m_dragged && std::hypot(d.x(), d.y()) < 4) return;
    m_dragged = true;
    setFollow(false);
    m_center = unproject(project(m_pressCenter.lat, m_pressCenter.lon) - d);
    update();
}

void MapItem::mouseReleaseEvent(QMouseEvent *e)
{
    if (!m_dragged) {
        const QPointF c = project(m_center.lat, m_center.lon);
        const LatLon ll = unproject(c + e->position() - QPointF(width() / 2, height() / 2));
        emit clicked(ll.lat, ll.lon);
    }
}

void MapItem::mouseDoubleClickEvent(QMouseEvent *)
{
    setFollow(true);
}

} // namespace corv
