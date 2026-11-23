#include "ui/ProfileItem.h"

#include "mission/RouteGeo.h"
#include "ui/PlanController.h"

#include <QCursor>
#include <QFont>
#include <QFontMetricsF>
#include <QLinearGradient>
#include <QPainter>
#include <QPainterPath>
#include <QSet>
#include <cmath>
#include <limits>

using namespace Qt::StringLiterals;

namespace corv {

using namespace mission;

namespace {

const QColor kDim(0x6e, 0x7f, 0x8d);
const QColor kMuted(0x88, 0x99, 0x99);
constexpr double kBandH = 6;   // the radio link band under the header

QFont mono(double px, int weight = QFont::Normal)
{
    QFont f(u"Roboto Mono"_s);
    f.setPixelSize(int(std::round(px)));
    f.setWeight(QFont::Weight(weight));
    return f;
}

QFont ui(double px, int weight = QFont::Bold)
{
    QFont f(u"Rajdhani"_s);
    f.setPixelSize(int(std::round(px)));
    f.setWeight(QFont::Weight(weight));
    return f;
}

double niceStep(double range, double count)
{
    const double raw = range / std::max(1.0, count);
    const double mag = std::pow(10, std::floor(std::log10(raw > 0 ? raw : 1)));
    const double norm = raw / mag;
    return (norm < 1.5 ? 1 : norm < 3.5 ? 2 : norm < 7.5 ? 5 : 10) * mag;
}

/// canvas fillText with textAlign
void text(QPainter *p, double x, double y, const QString &s, Qt::Alignment align)
{
    const QFontMetricsF fm(p->font());
    const double w = fm.horizontalAdvance(s);
    double left = x;
    if (align & Qt::AlignRight) left = x - w;
    else if (align & Qt::AlignHCenter) left = x - w / 2;
    p->drawText(QPointF(left, y), s);
}

} // namespace

ProfileItem::ProfileItem(QQuickItem *parent) : QQuickPaintedItem(parent)
{
    setAcceptedMouseButtons(Qt::LeftButton);
    setAcceptTouchEvents(true);
    setAcceptHoverEvents(true);
    setAntialiasing(true);
    setCursor(Qt::CrossCursor);
}

void ProfileItem::componentComplete()
{
    QQuickPaintedItem::componentComplete();
    m_plan = PlanController::s_instance;
    if (!m_plan) return;
    connect(m_plan, &PlanController::compiledChanged, this, [this] { rebuildSamples(); update(); });
    connect(m_plan, &PlanController::selectionChanged, this, [this] { update(); });
    connect(m_plan, &PlanController::layersChanged, this, [this] { update(); });
    connect(m_plan, &PlanController::derivedChanged, this, [this] { update(); });
    connect(this, &QQuickItem::widthChanged, this, [this] { update(); });
}

void ProfileItem::rebuildSamples()
{
    if (!m_plan || m_samplesRev == m_plan->compileRevision()) return;
    m_samplesRev = m_plan->compileRevision();
    m_samples.clear();
    const QList<NavPoint> &nav = m_plan->compiled().navPath;
    if (nav.size() < 2) return;
    const double total = nav.last().dist > 0 ? nav.last().dist : 1;
    const TerrainFn terrain = m_plan->terrainFn();
    // Dense terrain samples along the calculated path
    const int kSamples = std::min(1200, std::max(200, int(std::lround(total / 15))));
    for (qsizetype i = 1; i < nav.size(); ++i) {
        const NavPoint &a = nav[i - 1], &b = nav[i];
        const double segLen = b.dist - a.dist;
        const int n = std::max(1, int(std::lround(kSamples * segLen / total)));
        for (int s = 0; s < n; ++s) {
            const double t = double(s) / n;
            // Climb-out and final descent are vertical manoeuvres, not low flying
            m_samples.push_back({a.dist + segLen * t, terrain(a.lat + (b.lat - a.lat) * t, a.lng + (b.lng - a.lng) * t),
                                 a.altMsl + (b.altMsl - a.altMsl) * t, a.isHome || b.landing});
        }
    }
    m_samples.push_back({total, nav.last().terrain, nav.last().altMsl, false});
}

bool ProfileItem::layout(Layout &L) const
{
    if (!m_plan) return false;
    const QList<NavPoint> &nav = m_plan->compiled().navPath;
    L.w = width();
    L.h = height();
    // With the radio link on, the right part of the panel is the line-of-sight cut
    L.linkOn = m_plan->showLink() && m_plan->groundStation().has_value() && nav.size() >= 2;
    L.insetW = L.linkOn ? std::max(240.0, std::min(420.0, std::round(L.w * 0.34))) : 0;
    L.padR = 16 + L.insetW;
    if (nav.size() < 2) return false;
    L.total = nav.last().dist > 0 ? nav.last().dist : 1;
    double lo = std::numeric_limits<double>::infinity(), hi = -lo;
    for (const Sample &s : m_samples) if (s.elev) { lo = std::min(lo, *s.elev); hi = std::max(hi, *s.elev); }
    for (const NavPoint &n : nav) { lo = std::min(lo, n.altMsl); hi = std::max(hi, n.altMsl); }
    L.lo = lo - 15;
    L.hi = hi + 25;
    L.range = (L.hi - L.lo) != 0 ? L.hi - L.lo : 100;
    L.plotW = L.w - L.padL - L.padR;
    L.plotH = L.h - L.padT - L.padB;
    L.zoom = m_zoom;
    L.scroll = m_scroll;
    return true;
}

void ProfileItem::clampScroll(const Layout &L)
{
    m_scroll = std::max(0.0, std::min(L.plotW * m_zoom - L.plotW, m_scroll));
}

// ── Interaction ──────────────────────────────────────────────────────────────

void ProfileItem::wheelEvent(QWheelEvent *e)
{
    Layout L;
    if (!layout(L)) return;
    if (e->modifiers() & (Qt::ControlModifier | Qt::MetaModifier)) {
        zoomAt(std::pow(1.2, e->angleDelta().y() / 120.0), e->position().x(), L);
    } else {
        // A wheel notch scrolls 100 px, as the browser did
        m_scroll -= (e->angleDelta().y() + e->angleDelta().x()) / 1.2;
    }
    clampScroll(L);
    updateHover();
    update();
    e->accept();
}

void ProfileItem::zoomAt(double factor, double x, const Layout &L)
{
    const double old = m_zoom;
    m_zoom = std::max(1.0, std::min(40.0, m_zoom * factor));
    const double mx = x - L.padL;
    const double ratio = (mx + m_scroll) / (L.plotW * old);
    m_scroll = ratio * L.plotW * m_zoom - mx;
}

void ProfileItem::touchDragBegin(QPointF press)
{
    m_dragging = true;
    m_moved = true;
    m_dragX = press.x();
    m_scrollStart = m_scroll;
}

void ProfileItem::touchTap(QPointF p)
{
    m_hover = p;
    updateHover();
    update();
    if (m_plan && !m_hoverSeg.isEmpty()) m_plan->selectFromMap(m_hoverSeg);
}

void ProfileItem::pinch(QPointF centre, QPointF moved, double scale)
{
    Layout L;
    if (!layout(L)) return;
    m_scroll -= moved.x();
    zoomAt(scale, centre.x(), L);
    clampScroll(L);
    update();
}

void ProfileItem::mousePressEvent(QMouseEvent *e)
{
    m_dragging = true;
    m_moved = false;
    m_dragX = e->position().x();
    m_scrollStart = m_scroll;
    e->accept();
}

void ProfileItem::mouseMoveEvent(QMouseEvent *e) { dragTo(e->position()); }

void ProfileItem::dragTo(QPointF pos)
{
    Layout L;
    if (!m_dragging || !layout(L)) return;
    const double dx = m_dragX - pos.x();
    if (std::abs(dx) > 2) m_moved = true;
    m_scroll = m_scrollStart + dx;
    clampScroll(L);
    m_hover = pos;
    updateHover();
    update();
}

void ProfileItem::mouseReleaseEvent(QMouseEvent *)
{
    m_dragging = false;
    if (!m_moved && m_plan && !m_hoverSeg.isEmpty()) m_plan->selectFromMap(m_hoverSeg);
}

void ProfileItem::hoverMoveEvent(QHoverEvent *e)
{
    m_hover = e->position();
    updateHover();
    update();
}

void ProfileItem::hoverLeaveEvent(QHoverEvent *)
{
    m_hover.reset();
    updateHover();
    update();
}

void ProfileItem::updateHover()
{
    m_hoverDist.reset();
    m_hoverLink.reset();
    m_hoverSeg.clear();
    Layout L;
    const bool ok = m_plan && layout(L);
    const bool inside = ok && m_hover && m_hover->x() >= L.padL && m_hover->x() <= L.padL + L.plotW;
    if (!inside) {
        if (m_plan) m_plan->setProfileHover(std::nullopt, std::nullopt);
        if (m_tipVisible) { m_tipVisible = false; emit tipChanged(); }
        return;
    }
    const QList<NavPoint> &nav = m_plan->compiled().navPath;
    const double d = std::max(0.0, std::min(L.total, (m_hover->x() - L.padL + m_scroll) / (L.plotW * m_zoom) * L.total));
    qsizetype i = 1;
    while (i < nav.size() - 1 && nav[i].dist < d) ++i;
    const NavPoint &a = nav[i - 1], &b = nav[i];
    const double t = b.dist > a.dist ? (d - a.dist) / (b.dist - a.dist) : 0;
    const double lat = a.lat + (b.lat - a.lat) * t, lng = a.lng + (b.lng - a.lng) * t;
    const double alt = a.altMsl + (b.altMsl - a.altMsl) * t;
    const auto g = m_plan->terrainFn()(lat, lng);
    m_hoverDist = d;
    m_hoverAlt = alt;
    m_hoverSeg = !b.segId.isEmpty() ? b.segId : a.segId;

    // Radio link at the hovered point: numbers in the tooltip, the cut in the inset, the ray on the map
    std::optional<Link> cls;
    if (L.linkOn) {
        const QJsonObject radio = m_plan->route().value(u"params"_s).toObject().value(u"radio"_s).toObject();
        m_hoverLink = evaluateLink(radio, *m_plan->groundStation(), lat, lng, alt, m_plan->terrainFn(), 30, 600, true);
        cls = m_hoverLink->clazz;
    }
    m_plan->setProfileHover(LL{lat, lng}, cls);

    // Tooltip
    QString segText;
    const QJsonArray segs = m_plan->route().value(u"segments"_s).toArray();
    for (qsizetype k = 0; k < segs.size(); ++k) {
        const QJsonObject s = segs[k].toObject();
        if (s.value(u"id"_s).toString() == m_hoverSeg)
            segText = u" · %1 %2"_s.arg(k + 1).arg(segmentDef(s.value(u"type"_s).toString()).value(u"label"_s).toString());
    }
    QString tip = u"<b>%1</b>%2<br>AMSL <b>%3 m</b>"_s.arg(fmtDist(d), segText).arg(qRound64(alt));
    if (g) tip += u" · AGL <b>%1 m</b> · terrain %2 m"_s.arg(qRound64(alt - *g)).arg(qRound64(*g));
    if (m_hoverLink) {
        const LinkStyle &st = linkStyle(m_hoverLink->clazz);
        if (m_hoverLink->clazz == Link::Unknown) {
            tip += u"<br><font color=\"%1\">link: no terrain data</font>"_s.arg(st.color.name());
        } else {
            tip += u"<br><font color=\"%1\">%2</font> · %3 dBm · margin <b>%4 dB</b> · %5 from GS%6"_s
                       .arg(st.color.name(), st.shortName)
                       .arg(m_hoverLink->rssi, 0, 'f', 0)
                       .arg(m_hoverLink->margin, 0, 'f', 0)
                       .arg(fmtDist(m_hoverLink->dist))
                       .arg(m_hoverLink->loss > 0 ? u" · terrain −%1 dB"_s.arg(m_hoverLink->loss, 0, 'f', 0) : QString());
        }
    }
    const double tipW = m_hoverLink ? 300 : 190;
    m_tipText = tip;
    m_tipX = std::max(L.padL, std::min(L.padL + L.plotW - tipW, m_hover->x() + 12));
    m_tipY = std::max(4.0, std::min(L.h - (m_hoverLink ? 60.0 : 46.0), m_hover->y() - 40));
    m_tipVisible = true;
    emit tipChanged();
}

// ── Paint ────────────────────────────────────────────────────────────────────

void ProfileItem::paint(QPainter *p)
{
    p->setRenderHint(QPainter::Antialiasing);
    const double w = width(), h = height();
    p->fillRect(QRectF(0, 0, w, h), QColor(5, 12, 20, 242));
    if (!m_plan) return;
    Layout L;
    if (!layout(L) || m_samples.empty()) {
        p->setPen(kDim);
        p->setFont(mono(11));
        text(p, w / 2, h / 2, u"Elevation profile — draw at least one segment"_s, Qt::AlignHCenter);
        return;
    }
    const QString sel = m_plan->selectedId();
    const double altStep = niceStep(L.range, 4);
    const double distStep = niceStep(L.total / m_zoom, std::max(4.0, std::floor(L.plotW / 90)));

    p->save();
    p->setClipRect(QRectF(L.padL, 0, L.plotW, h));
    drawGrid(p, L, altStep, distStep);
    if (L.linkOn && m_plan->linkAnalysis()) drawLinkBand(p, L);
    drawTerrain(p, L);
    drawRoute(p, L, sel);
    drawPoints(p, L, sel);
    if (m_hoverDist) {
        const double x = std::round(L.X(*m_hoverDist)) + 0.5;
        p->setPen(QPen(QColor(0, 210, 255, 204), 1));
        p->drawLine(QPointF(x, L.padT), QPointF(x, L.padT + L.plotH));
        p->setPen(Qt::NoPen);
        p->setBrush(QColor(0x00, 0xd2, 0xff));
        p->drawEllipse(QPointF(x, L.Y(m_hoverAlt)), 4, 4);
    }
    p->restore();

    // Line-of-sight cut: the hovered point, else the worst point of the route
    if (L.linkOn) {
        const auto &worst = m_plan->worstEval();
        const LinkEval *ev = m_hoverLink ? &*m_hoverLink : worst ? &*worst : nullptr;
        const QString title = m_hoverLink ? u"GS → cursor"_s
                            : (worst && m_plan->linkAnalysis() && m_plan->linkAnalysis()->worst)
                                ? u"GS → worst point, %1"_s.arg(fmtDist(m_plan->linkAnalysis()->worst->dist)) : QString();
        drawInset(p, QRectF(w - L.insetW - 8, 0, L.insetW + 8, h), ev, title);
        p->setPen(kDim);
        p->setFont(mono(8));
        text(p, L.padL - 6, L.bandY() + kBandH - 1, u"LINK"_s, Qt::AlignRight);
    }
    drawAxes(p, L, altStep, distStep);
}

void ProfileItem::drawGrid(QPainter *p, const Layout &L, double altStep, double distStep) const
{
    p->setPen(QPen(QColor(255, 255, 255, 15), 1));
    for (double a = std::ceil(L.lo / altStep) * altStep; a <= L.hi; a += altStep) {
        const double y = std::round(L.Y(a)) + 0.5;
        p->drawLine(QPointF(L.padL, y), QPointF(L.padL + L.plotW, y));
    }
    p->setPen(QPen(QColor(255, 255, 255, 10), 1));
    for (double d = 0; d <= L.total; d += distStep) {
        const double x = std::round(L.X(d)) + 0.5;
        if (L.inPlot(x)) p->drawLine(QPointF(x, L.padT), QPointF(x, L.padT + L.plotH));
    }
}

void ProfileItem::drawLinkBand(QPainter *p, const Layout &L) const
{
    // Radio link class along the route, as a band under the header
    const auto &s = m_plan->linkAnalysis()->samples;
    const double xMin = L.padL, xMax = L.padL + L.plotW, bandY = L.bandY();
    for (qsizetype i = 1; i < s.size(); ++i) {
        double x1 = L.X(s[i - 1].dist), x2 = L.X(s[i].dist);
        if (x2 < xMin || x1 > xMax) continue;
        x1 = std::max(xMin, x1);
        x2 = std::min(xMax, x2);
        const Link cls = s[i - 1].clazz;
        if (cls == Link::None) {
            p->fillRect(QRectF(x1, bandY, std::max(1.0, x2 - x1), kBandH), QColor(154, 164, 176, 64));
            for (double x = std::ceil(x1 / 4) * 4; x < x2; x += 4) p->fillRect(QRectF(x, bandY + kBandH / 2 - 0.5, 1.5, 1), QColor(0x9a, 0xa4, 0xb0));
        } else if (cls == Link::Unknown) {
            p->fillRect(QRectF(x1, bandY, std::max(1.0, x2 - x1), kBandH), QColor(74, 85, 96, 89));
        } else {
            QColor c = linkStyle(cls).color;
            c.setAlphaF(cls == Link::Good ? 0.55 : 0.85);
            p->fillRect(QRectF(x1, bandY, std::max(1.0, x2 - x1 + 0.5), kBandH), c);
        }
    }
}

void ProfileItem::drawTerrain(QPainter *p, const Layout &L) const
{
    std::vector<const Sample *> tPts;
    for (const Sample &s : m_samples) if (s.elev) tPts.push_back(&s);
    if (tPts.empty()) return;
    auto polyline = [&](double above) {
        QPainterPath path;
        path.moveTo(L.X(tPts.front()->dist), L.Y(*tPts.front()->elev + above));
        for (const Sample *s : tPts) path.lineTo(L.X(s->dist), L.Y(*s->elev + above));
        return path;
    };
    QPainterPath fill;
    fill.moveTo(L.X(tPts.front()->dist), L.Y(L.lo));
    for (const Sample *s : tPts) fill.lineTo(L.X(s->dist), L.Y(*s->elev));
    fill.lineTo(L.X(tPts.back()->dist), L.Y(L.lo));
    fill.closeSubpath();
    QLinearGradient g(0, L.Y(L.hi), 0, L.Y(L.lo));
    g.setColorAt(0, QColor(96, 128, 72, 140));
    g.setColorAt(1, QColor(40, 56, 32, 64));
    p->setPen(Qt::NoPen);
    p->setBrush(g);
    p->drawPath(fill);
    p->setBrush(Qt::NoBrush);
    p->setPen(QPen(QColor(140, 190, 110, 178), 1.2));
    p->drawPath(polyline(0));

    // Minimum-clearance floor and the stretches that dip under it
    const double minC = num(m_plan->route().value(u"params"_s).toObject().value(u"minClearance"_s));
    if (minC > 0) {
        QPen dp(QColor(255, 170, 0, 89), 1);
        dp.setDashPattern({3, 4});
        p->setPen(dp);
        p->drawPath(polyline(minC));
    }
    std::optional<std::pair<double, double>> run;
    auto flush = [&] {
        if (run) p->fillRect(QRectF(L.X(run->first), L.padT, std::max(1.0, L.X(run->second) - L.X(run->first)), L.plotH), QColor(255, 60, 60, 71));
        run.reset();
    };
    for (const Sample *s : tPts) {
        if (s->vertical || s->route - *s->elev >= minC || s->dist <= 0 || s->dist >= L.total) flush();
        else if (!run) run = std::make_pair(s->dist, s->dist);
        else run->second = s->dist;
    }
    flush();
}

void ProfileItem::drawRoute(QPainter *p, const Layout &L, const QString &sel) const
{
    const QList<NavPoint> &nav = m_plan->compiled().navPath;
    // Waypoint verticals, base points only
    p->setPen(QPen(QColor(255, 255, 255, 26), 1));
    for (const NavPoint &n : nav) {
        if (n.derived || n.isHome) continue;
        const double x = std::round(L.X(n.dist)) + 0.5;
        p->drawLine(QPointF(x, L.padT), QPointF(x, L.padT + L.plotH));
    }
    // Route line, the selected segment brighter
    auto at = [&](qsizetype i) { return QPointF(L.X(nav[i].dist), L.Y(nav[i].altMsl)); };
    QPainterPath route;
    route.moveTo(at(0));
    for (qsizetype i = 1; i < nav.size(); ++i) route.lineTo(at(i));
    p->setPen(QPen(QColor(0x44, 0xff, 0x44), 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    p->drawPath(route);
    if (sel.isEmpty()) return;
    QPainterPath part;
    bool started = false;
    for (qsizetype i = 0; i < nav.size(); ++i) {
        if (nav[i].segId != sel) {
            started = false;
        } else if (started) {
            part.lineTo(at(i));
        } else {
            part.moveTo(at(i > 0 ? i - 1 : i));
            if (i > 0) part.lineTo(at(i));
            started = true;
        }
    }
    p->setPen(QPen(Qt::white, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    p->drawPath(part);
}

void ProfileItem::drawPoints(QPainter *p, const Layout &L, const QString &sel) const
{
    const QList<NavPoint> &nav = m_plan->compiled().navPath;
    const QJsonArray segs = m_plan->route().value(u"segments"_s).toArray();
    QHash<QString, int> idxOf;
    QHash<QString, QString> colorOf;
    for (qsizetype k = 0; k < segs.size(); ++k) {
        const QJsonObject s = segs[k].toObject();
        idxOf.insert(s.value(u"id"_s).toString(), int(k));
        colorOf.insert(s.value(u"id"_s).toString(), segmentDef(s.value(u"type"_s).toString()).value(u"color"_s).toString());
    }
    for (const NavPoint &n : nav) {
        const double x = L.X(n.dist), y = L.Y(n.altMsl);
        if (!L.inPlot(x, 10)) continue;
        p->setBrush(n.isHome ? QColor(0xff, 0x88, 0x00) : QColor(colorOf.value(n.segId, u"#44ff44"_s)));
        p->setPen(n.derived ? QPen(Qt::NoPen) : QPen(Qt::white, 1.2));
        p->drawEllipse(QPointF(x, y), n.derived ? 2 : 4, n.derived ? 2 : 4);
    }
    // Segment numbers once, at the first point of each, never overlapping
    p->setFont(mono(9, QFont::Bold));
    QSet<QString> seen;
    double lastLabelX = -1e9;
    for (const NavPoint &n : nav) {
        if (n.segId.isEmpty() || n.derived || seen.contains(n.segId)) continue;
        seen.insert(n.segId);
        const double x = L.X(n.dist);
        if (!L.inPlot(x) || (x - lastLabelX < 16 && n.segId != sel)) continue;
        lastLabelX = x;
        p->setPen(n.segId == sel ? QColor(Qt::white) : QColor(0xcf, 0xd8, 0xdd));
        text(p, x, L.Y(n.altMsl) - 8, QString::number(idxOf.value(n.segId) + 1), Qt::AlignHCenter);
    }
    if (nav[0].isHome) {
        p->setPen(QColor(0xff, 0x88, 0x00));
        text(p, L.X(0), L.Y(nav[0].altMsl) - 8, u"H"_s, Qt::AlignHCenter);
    }
}

void ProfileItem::drawAxes(QPainter *p, const Layout &L, double altStep, double distStep) const
{
    p->setPen(kDim);
    p->setFont(mono(9));
    for (double a = std::ceil(L.lo / altStep) * altStep; a <= L.hi; a += altStep) text(p, L.padL - 6, L.Y(a) + 3, QString::number(qRound64(a)), Qt::AlignRight);
    for (double d = 0; d <= L.total; d += distStep)
        if (const double x = L.X(d); L.inPlot(x)) text(p, x, L.h - 8, fmtDist(d), Qt::AlignHCenter);
    // Header: title and the route figures
    p->setPen(QColor(0x00, 0xd2, 0xff));
    p->setFont(ui(10));
    text(p, L.padL, 15, u"ELEVATION PROFILE"_s, Qt::AlignLeft);
    const RouteStats &st = m_plan->compiled().stats;
    QStringList parts{fmtDist(st.lengthM), fmtTime(st.durationS), u"%1 WP"_s.arg(st.waypoints)};
    if (st.minAgl && st.maxAgl) parts << u"AGL %1–%2 m"_s.arg(qRound64(*st.minAgl)).arg(qRound64(*st.maxAgl));
    if (st.maxAmsl) parts << u"AMSL max %1 m"_s.arg(qRound64(*st.maxAmsl));
    const QString mode = m_plan->route().value(u"params"_s).toObject().value(u"altMode"_s).toString();
    parts << (mode == "amsl"_L1 ? u"AMSL mode"_s : mode == "rel"_L1 ? u"REL mode"_s : u"AGL mode"_s);
    p->setPen(kMuted);
    p->setFont(mono(9));
    text(p, L.padL + 130, 15, parts.join(u"   ·   "_s), Qt::AlignLeft);
    if (m_zoom > 1.05) text(p, L.w - L.padR, 15, u"%1×"_s.arg(m_zoom, 0, 'f', 1), Qt::AlignRight);
}

void ProfileItem::drawInset(QPainter *p, const QRectF &rect, const LinkEval *ev, const QString &title) const
{
    const double rx = rect.x(), ry = rect.y(), w = rect.width(), h = rect.height();
    p->save();
    p->setClipRect(rect);
    p->fillRect(rect, QColor(0, 0, 0, 56));
    p->setPen(QPen(QColor(0, 210, 255, 64), 1));
    p->drawLine(QPointF(rx + 0.5, ry), QPointF(rx + 0.5, ry + h));
    p->setPen(QColor(0x00, 0xd2, 0xff));
    p->setFont(ui(10));
    text(p, rx + 10, ry + 15, u"LINK PROFILE"_s, Qt::AlignLeft);
    if (!title.isEmpty()) {
        p->setPen(kMuted);
        p->setFont(mono(9));
        text(p, rx + 96, ry + 15, title, Qt::AlignLeft);
    }
    const double padL = 12, padR = 12, padT = 24, padB = 26;
    const double pw = w - padL - padR, ph = h - padT - padB;
    if (!ev || !ev->profile || !(ev->profile->D > 0) || ev->clazz == Link::Unknown) {
        p->setPen(kDim);
        p->setFont(mono(9.5));
        text(p, rx + w / 2, ry + h / 2 + 3, ev && ev->clazz == Link::Unknown ? u"no elevation data along the path"_s
                                                                            : u"hover the profile to see the line of sight"_s,
             Qt::AlignHCenter);
        p->restore();
        return;
    }
    const LinkProfile &pr = *ev->profile;
    const int n = pr.n;
    const double D = pr.D;
    auto bulge = [&](double d) { return d * (D - d) / (2 * (4.0 / 3.0) * 6371000); };
    auto line = [&](double d) { return pr.h0 + (pr.h1 - pr.h0) * (d / D); };
    auto fr = [&](double d) { return fresnelRadius(d, D - d, pr.freq); };
    double lo = std::min(pr.h0, pr.h1), hi = std::max(pr.h0, pr.h1);
    for (int j = 0; j <= n; ++j) {
        const double d = j * pr.step;
        lo = std::min({lo, pr.terr[j] + bulge(d), line(d) - fr(d)});
        hi = std::max({hi, pr.terr[j] + bulge(d), line(d) + fr(d)});
    }
    const double range = std::max(20.0, hi - lo);
    lo -= range * 0.08;
    hi += range * 0.12;
    auto X = [&](double d) { return rx + padL + d / D * pw; };
    auto Y = [&](double a) { return ry + padT + ph - (a - lo) / (hi - lo) * ph; };
    const LinkStyle &st = linkStyle(ev->clazz);

    // First Fresnel zone: outline of F1, filled 60 % core
    auto zone = [&](double k) {
        QPainterPath z;
        for (int j = 0; j <= n; ++j) { const double d = j * pr.step; const QPointF q(X(d), Y(line(d) + k * fr(d))); j ? z.lineTo(q) : z.moveTo(q); }
        for (int j = n; j >= 0; --j) { const double d = j * pr.step; z.lineTo(X(d), Y(line(d) - k * fr(d))); }
        z.closeSubpath();
        return z;
    };
    QPen zp(QColor(0, 210, 255, 89), 1);
    zp.setDashPattern({3, 3});
    p->setPen(zp);
    p->setBrush(Qt::NoBrush);
    p->drawPath(zone(1));
    p->setPen(Qt::NoPen);
    p->setBrush(QColor(0, 210, 255, 26));
    p->drawPath(zone(0.6));

    // Terrain with the earth bulge folded in
    QPainterPath ground;
    ground.moveTo(X(0), Y(lo));
    for (int j = 0; j <= n; ++j) ground.lineTo(X(j * pr.step), Y(pr.terr[j] + bulge(j * pr.step)));
    ground.lineTo(X(D), Y(lo));
    ground.closeSubpath();
    QLinearGradient g(0, Y(hi), 0, Y(lo));
    g.setColorAt(0, QColor(96, 128, 72, 178));
    g.setColorAt(1, QColor(40, 56, 32, 89));
    p->setBrush(g);
    p->drawPath(ground);
    QPainterPath gl;
    for (int j = 0; j <= n; ++j) { const QPointF q(X(j * pr.step), Y(pr.terr[j] + bulge(j * pr.step))); j ? gl.lineTo(q) : gl.moveTo(q); }
    p->setBrush(Qt::NoBrush);
    p->setPen(QPen(QColor(140, 190, 110, 204), 1.2));
    p->drawPath(gl);

    // Where the ground enters the 60 % zone: the intruding slice in red
    const double cw = std::max(1.5, pw / n);
    for (int j = 1; j < n; ++j) {
        const double d = j * pr.step;
        const double top = line(d) - 0.6 * fr(d);
        const double gr = pr.terr[j] + bulge(d);
        if (gr <= top) continue;
        const double yTop = Y(std::min(gr, line(d)));
        p->fillRect(QRectF(X(d) - cw / 2, yTop, cw, std::max(1.0, Y(top) - yTop)), QColor(255, 60, 60, 89));
    }
    QPainterPath red;
    bool inRun = false;
    for (int j = 1; j < n; ++j) {
        const double d = j * pr.step;
        const bool intrudes = pr.terr[j] + bulge(d) > line(d) - 0.6 * fr(d);
        const QPointF q(X(d), Y(pr.terr[j] + bulge(d)));
        if (intrudes) { inRun ? red.lineTo(q) : red.moveTo(q); inRun = true; }
        else inRun = false;
    }
    p->setPen(QPen(QColor(255, 70, 70, 242), 2));
    p->drawPath(red);

    // Line of sight, mast and aircraft
    p->setPen(QPen(st.color, 1.6));
    p->drawLine(QPointF(X(0), Y(pr.h0)), QPointF(X(D), Y(pr.h1)));
    p->setPen(QPen(Qt::white, 1.2));
    p->drawLine(QPointF(X(0) + 0.5, Y(pr.terr[0])), QPointF(X(0) + 0.5, Y(pr.h0)));
    p->setPen(Qt::NoPen);
    p->setBrush(Qt::white);
    p->drawEllipse(QPointF(X(0) + 0.5, Y(pr.h0)), 2.5, 2.5);
    p->setPen(QPen(Qt::white, 1));
    p->setBrush(st.color);
    p->drawEllipse(QPointF(X(D), Y(pr.h1)), 3.5, 3.5);

    // Worst knife edge
    if (ev->worst && ev->nu > -0.78) {
        const double wx = X(ev->worst->dist), wy = Y(ev->worst->elev + bulge(ev->worst->dist));
        QPainterPath tri;
        tri.moveTo(wx, wy - 9);
        tri.lineTo(wx - 4, wy - 2);
        tri.lineTo(wx + 4, wy - 2);
        tri.closeSubpath();
        p->setPen(Qt::NoPen);
        p->setBrush(ev->obstructed ? QColor(0xff, 0x3b, 0x3b) : QColor(0xff, 0xaa, 0x00));
        p->drawPath(tri);
        p->setPen(ev->obstructed ? QColor(0xff, 0x60, 0x60) : QColor(0xff, 0xc0, 0x40));
        p->setFont(mono(8.5, QFont::Bold));
        text(p, std::min(rx + w - 26, std::max(rx + 26, wx)), wy - 12, u"−%1 dB"_s.arg(ev->loss, 0, 'f', 1), Qt::AlignHCenter);
    }

    // Axis text and numbers
    p->setPen(kDim);
    p->setFont(mono(8.5));
    text(p, X(0) + 4, ry + padT + ph + 10, u"GS"_s, Qt::AlignLeft);
    text(p, X(D), ry + padT + ph + 10, fmtDist(D), Qt::AlignRight);
    p->setPen(QColor(0x55, 0x55, 0x66));
    text(p, rx + padL, Y(lo) - 2, u"%1 m"_s.arg(qRound64(lo)), Qt::AlignLeft);
    const QString status = ev->obstructed ? u"NLOS · diffraction"_s : ev->fresnelClear ? u"LOS · Fresnel clear"_s : u"LOS · Fresnel intruded"_s;
    p->setFont(mono(9));
    p->setPen(st.color);
    text(p, rx + padL, ry + h - 6, st.shortName, Qt::AlignLeft);
    p->setPen(QColor(0xcf, 0xd8, 0xdd));
    text(p, rx + padL + 62, ry + h - 6,
         u"%1 dBm · margin %2%3 dB · %4"_s.arg(ev->rssi, 0, 'f', 0).arg(ev->margin >= 0 ? u"+"_s : QString()).arg(ev->margin, 0, 'f', 0).arg(status),
         Qt::AlignLeft);
    p->restore();
}

} // namespace corv
