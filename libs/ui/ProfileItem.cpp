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

namespace corv {

using namespace mission;

namespace {

const QColor kDim(0x6e, 0x7f, 0x8d);
const QColor kMuted(0x88, 0x99, 0x99);

QFont mono(double px, int weight = QFont::Normal)
{
    QFont f(QStringLiteral("Roboto Mono"));
    f.setPixelSize(int(std::round(px)));
    f.setWeight(QFont::Weight(weight));
    return f;
}

QFont ui(double px, int weight = QFont::Bold)
{
    QFont f(QStringLiteral("Rajdhani"));
    f.setPixelSize(int(std::round(px)));
    f.setWeight(QFont::Weight(weight));
    return f;
}

QString fmtDist(double m)
{
    return m >= 1000 ? QStringLiteral("%1 km").arg(m / 1000, 0, 'f', m >= 10000 ? 0 : 1) : QStringLiteral("%1 m").arg(qRound64(m));
}

QString fmtTime(double s)
{
    if (!s) return QStringLiteral("--");
    const qint64 m = qRound64(s / 60);
    return m >= 60 ? QStringLiteral("%1h %2m").arg(m / 60).arg(m % 60, 2, 10, QLatin1Char('0')) : QStringLiteral("%1 min").arg(m);
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
        const double factor = e->angleDelta().y() > 0 ? 1.2 : 1 / 1.2;
        const double old = m_zoom;
        m_zoom = std::max(1.0, std::min(40.0, m_zoom * factor));
        const double mx = e->position().x() - L.padL;
        const double ratio = (mx + m_scroll) / (L.plotW * old);
        m_scroll = ratio * L.plotW * m_zoom - mx;
    } else {
        // A wheel notch scrolls 100 px, as the browser did
        m_scroll -= (e->angleDelta().y() + e->angleDelta().x()) / 1.2;
    }
    clampScroll(L);
    updateHover();
    update();
    e->accept();
}

void ProfileItem::mousePressEvent(QMouseEvent *e)
{
    m_dragging = true;
    m_moved = false;
    m_dragX = e->position().x();
    m_scrollStart = m_scroll;
    e->accept();
}

void ProfileItem::mouseMoveEvent(QMouseEvent *e)
{
    Layout L;
    if (!m_dragging || !layout(L)) return;
    const double dx = m_dragX - e->position().x();
    if (std::abs(dx) > 2) m_moved = true;
    m_scroll = m_scrollStart + dx;
    clampScroll(L);
    m_hover = e->position();
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
        const QJsonObject radio = m_plan->route().value(QStringLiteral("params")).toObject().value(QStringLiteral("radio")).toObject();
        m_hoverLink = evaluateLink(radio, *m_plan->groundStation(), lat, lng, alt, m_plan->terrainFn(), 30, 600, true);
        cls = m_hoverLink->clazz;
    }
    m_plan->setProfileHover(LL{lat, lng}, cls);

    // Tooltip
    QString segText;
    const QJsonArray segs = m_plan->route().value(QStringLiteral("segments")).toArray();
    for (qsizetype k = 0; k < segs.size(); ++k) {
        const QJsonObject s = segs[k].toObject();
        if (s.value(QStringLiteral("id")).toString() == m_hoverSeg)
            segText = QStringLiteral(" · %1 %2").arg(k + 1).arg(segmentDef(s.value(QStringLiteral("type")).toString()).value(QStringLiteral("label")).toString());
    }
    QString tip = QStringLiteral("<b>%1</b>%2<br>AMSL <b>%3 m</b>").arg(fmtDist(d), segText).arg(qRound64(alt));
    if (g) tip += QStringLiteral(" · AGL <b>%1 m</b> · terrain %2 m").arg(qRound64(alt - *g)).arg(qRound64(*g));
    if (m_hoverLink) {
        const LinkStyle &st = linkStyle(m_hoverLink->clazz);
        if (m_hoverLink->clazz == Link::Unknown) {
            tip += QStringLiteral("<br><font color=\"%1\">link: no terrain data</font>").arg(st.color.name());
        } else {
            tip += QStringLiteral("<br><font color=\"%1\">%2</font> · %3 dBm · margin <b>%4 dB</b> · %5 from GS%6")
                       .arg(st.color.name(), st.shortName)
                       .arg(m_hoverLink->rssi, 0, 'f', 0)
                       .arg(m_hoverLink->margin, 0, 'f', 0)
                       .arg(fmtDist(m_hoverLink->dist))
                       .arg(m_hoverLink->loss > 0 ? QStringLiteral(" · terrain −%1 dB").arg(m_hoverLink->loss, 0, 'f', 0) : QString());
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
    const QList<NavPoint> &nav = m_plan->compiled().navPath;
    const QJsonObject P = m_plan->route().value(QStringLiteral("params")).toObject();
    if (nav.size() < 2 || m_samples.empty()) {
        p->setPen(kDim);
        p->setFont(mono(11));
        text(p, w / 2, h / 2, QStringLiteral("Elevation profile — draw at least one segment"), Qt::AlignHCenter);
        return;
    }
    Layout L;
    layout(L);
    auto X = [&](double d) { return L.X(d, m_zoom, m_scroll); };
    auto Y = [&](double a) { return L.Y(a); };
    const QString sel = m_plan->selectedId();

    p->save();
    p->setClipRect(QRectF(L.padL, 0, L.plotW, h));

    // Grid
    const double altStep = niceStep(L.range, 4);
    p->setPen(QPen(QColor(255, 255, 255, 15), 1));
    for (double a = std::ceil(L.lo / altStep) * altStep; a <= L.hi; a += altStep) {
        const double y = std::round(Y(a)) + 0.5;
        p->drawLine(QPointF(L.padL, y), QPointF(L.padL + L.plotW, y));
    }
    const double distStep = niceStep(L.total / m_zoom, std::max(4.0, std::floor(L.plotW / 90)));
    p->setPen(QPen(QColor(255, 255, 255, 10), 1));
    for (double d = 0; d <= L.total; d += distStep) {
        const double x = std::round(X(d)) + 0.5;
        if (x < L.padL || x > L.padL + L.plotW) continue;
        p->drawLine(QPointF(x, L.padT), QPointF(x, L.padT + L.plotH));
    }

    // Radio link class along the route, as a band under the header
    const double bandY = L.padT - 11, bandH = 6;
    if (L.linkOn && m_plan->linkAnalysis()) {
        const auto &s = m_plan->linkAnalysis()->samples;
        const double xMin = L.padL, xMax = L.padL + L.plotW;
        for (qsizetype i = 1; i < s.size(); ++i) {
            double x1 = X(s[i - 1].dist), x2 = X(s[i].dist);
            if (x2 < xMin || x1 > xMax) continue;
            x1 = std::max(xMin, x1);
            x2 = std::min(xMax, x2);
            const Link cls = s[i - 1].clazz;
            if (cls == Link::None) {
                p->fillRect(QRectF(x1, bandY, std::max(1.0, x2 - x1), bandH), QColor(154, 164, 176, 64));
                for (double x = std::ceil(x1 / 4) * 4; x < x2; x += 4) p->fillRect(QRectF(x, bandY + bandH / 2 - 0.5, 1.5, 1), QColor(0x9a, 0xa4, 0xb0));
                continue;
            }
            if (cls == Link::Unknown) { p->fillRect(QRectF(x1, bandY, std::max(1.0, x2 - x1), bandH), QColor(74, 85, 96, 89)); continue; }
            QColor c = linkStyle(cls).color;
            c.setAlphaF(cls == Link::Good ? 0.55 : 0.85);
            p->fillRect(QRectF(x1, bandY, std::max(1.0, x2 - x1 + 0.5), bandH), c);
        }
    }

    // Terrain
    std::vector<const Sample *> tPts;
    for (const Sample &s : m_samples) if (s.elev) tPts.push_back(&s);
    if (!tPts.empty()) {
        QPainterPath fill;
        fill.moveTo(X(tPts.front()->dist), Y(L.lo));
        for (const Sample *s : tPts) fill.lineTo(X(s->dist), Y(*s->elev));
        fill.lineTo(X(tPts.back()->dist), Y(L.lo));
        fill.closeSubpath();
        QLinearGradient g(0, Y(L.hi), 0, Y(L.lo));
        g.setColorAt(0, QColor(96, 128, 72, 140));
        g.setColorAt(1, QColor(40, 56, 32, 64));
        p->setPen(Qt::NoPen);
        p->setBrush(g);
        p->drawPath(fill);
        QPainterPath line;
        line.moveTo(X(tPts.front()->dist), Y(*tPts.front()->elev));
        for (const Sample *s : tPts) line.lineTo(X(s->dist), Y(*s->elev));
        p->setBrush(Qt::NoBrush);
        p->setPen(QPen(QColor(140, 190, 110, 178), 1.2));
        p->drawPath(line);

        // Minimum-clearance floor and the stretches that dip under it
        const double minC = num(P.value(QStringLiteral("minClearance")));
        if (minC > 0) {
            QPainterPath floor;
            floor.moveTo(X(tPts.front()->dist), Y(*tPts.front()->elev + minC));
            for (const Sample *s : tPts) floor.lineTo(X(s->dist), Y(*s->elev + minC));
            QPen dp(QColor(255, 170, 0, 89), 1);
            dp.setDashPattern({3, 4});
            p->setPen(dp);
            p->drawPath(floor);
        }
        std::optional<std::pair<double, double>> run;
        auto flush = [&] {
            if (run) p->fillRect(QRectF(X(run->first), L.padT, std::max(1.0, X(run->second) - X(run->first)), L.plotH), QColor(255, 60, 60, 71));
            run.reset();
        };
        for (const Sample *s : tPts) {
            const bool bad = !s->vertical && s->route - *s->elev < minC && s->dist > 0 && s->dist < L.total;
            if (bad) { if (!run) run = std::make_pair(s->dist, s->dist); else run->second = s->dist; }
            else flush();
        }
        flush();
    }

    // Waypoint verticals, base points only
    p->setPen(QPen(QColor(255, 255, 255, 26), 1));
    for (const NavPoint &n : nav) {
        if (n.derived || n.isHome) continue;
        const double x = std::round(X(n.dist)) + 0.5;
        p->drawLine(QPointF(x, L.padT), QPointF(x, L.padT + L.plotH));
    }

    // Route line, the selected segment brighter
    {
        QPainterPath route;
        route.moveTo(X(nav[0].dist), Y(nav[0].altMsl));
        for (qsizetype i = 1; i < nav.size(); ++i) route.lineTo(X(nav[i].dist), Y(nav[i].altMsl));
        p->setPen(QPen(QColor(0x44, 0xff, 0x44), 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        p->drawPath(route);
        if (!sel.isEmpty()) {
            QPainterPath part;
            bool started = false;
            for (qsizetype i = 0; i < nav.size(); ++i) {
                if (nav[i].segId == sel) {
                    if (!started && i > 0) { part.moveTo(X(nav[i - 1].dist), Y(nav[i - 1].altMsl)); started = true; }
                    if (!started) { part.moveTo(X(nav[i].dist), Y(nav[i].altMsl)); started = true; }
                    else part.lineTo(X(nav[i].dist), Y(nav[i].altMsl));
                } else {
                    started = false;
                }
            }
            p->setPen(QPen(Qt::white, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
            p->drawPath(part);
        }
    }

    // Points
    const QJsonArray segs = m_plan->route().value(QStringLiteral("segments")).toArray();
    QHash<QString, int> idxOf;
    QHash<QString, QString> colorOf;
    for (qsizetype k = 0; k < segs.size(); ++k) {
        const QJsonObject s = segs[k].toObject();
        idxOf.insert(s.value(QStringLiteral("id")).toString(), int(k));
        colorOf.insert(s.value(QStringLiteral("id")).toString(), segmentDef(s.value(QStringLiteral("type")).toString()).value(QStringLiteral("color")).toString());
    }
    for (const NavPoint &n : nav) {
        const double x = X(n.dist), y = Y(n.altMsl);
        if (x < L.padL - 10 || x > L.padL + L.plotW + 10) continue;
        const QColor c = n.isHome ? QColor(0xff, 0x88, 0x00) : QColor(colorOf.value(n.segId, QStringLiteral("#44ff44")));
        p->setBrush(c);
        p->setPen(n.derived ? QPen(Qt::NoPen) : QPen(Qt::white, 1.2));
        p->drawEllipse(QPointF(x, y), n.derived ? 2 : 4, n.derived ? 2 : 4);
    }
    // Segment labels once, at the first point of each
    p->setFont(mono(9, QFont::Bold));
    QSet<QString> seen;
    double lastLabelX = -1e9;
    for (const NavPoint &n : nav) {
        if (n.segId.isEmpty() || n.derived || seen.contains(n.segId)) continue;
        seen.insert(n.segId);
        const double x = X(n.dist);
        if (x < L.padL || x > L.padL + L.plotW) continue;
        if (x - lastLabelX < 16 && n.segId != sel) continue;   // no overlapping numbers
        lastLabelX = x;
        p->setPen(n.segId == sel ? QColor(Qt::white) : QColor(0xcf, 0xd8, 0xdd));
        text(p, x, Y(n.altMsl) - 8, QString::number(idxOf.value(n.segId) + 1), Qt::AlignHCenter);
    }
    if (nav[0].isHome) {
        p->setPen(QColor(0xff, 0x88, 0x00));
        text(p, X(0), Y(nav[0].altMsl) - 8, QStringLiteral("H"), Qt::AlignHCenter);
    }

    // Hover cursor
    if (m_hoverDist) {
        const double x = std::round(X(*m_hoverDist)) + 0.5;
        p->setPen(QPen(QColor(0, 210, 255, 204), 1));
        p->drawLine(QPointF(x, L.padT), QPointF(x, L.padT + L.plotH));
        p->setPen(Qt::NoPen);
        p->setBrush(QColor(0x00, 0xd2, 0xff));
        p->drawEllipse(QPointF(x, Y(m_hoverAlt)), 4, 4);
    }
    p->restore();

    // Line-of-sight cut: the hovered point, else the worst point of the route
    if (L.linkOn) {
        const LinkEval *ev = m_hoverLink ? &*m_hoverLink : m_plan->worstEval() ? &*m_plan->worstEval() : nullptr;
        const QString title = m_hoverLink ? QStringLiteral("GS → cursor")
                            : (m_plan->worstEval() && m_plan->linkAnalysis() && m_plan->linkAnalysis()->worst)
                                ? QStringLiteral("GS → worst point, %1").arg(fmtDist(m_plan->linkAnalysis()->worst->dist)) : QString();
        drawInset(p, QRectF(w - L.insetW - 8, 0, L.insetW + 8, h), ev, title);
        p->setPen(kDim);
        p->setFont(mono(8));
        text(p, L.padL - 6, bandY + bandH - 1, QStringLiteral("LINK"), Qt::AlignRight);
    }

    // Axes
    p->setPen(kDim);
    p->setFont(mono(9));
    for (double a = std::ceil(L.lo / altStep) * altStep; a <= L.hi; a += altStep) text(p, L.padL - 6, Y(a) + 3, QString::number(qRound64(a)), Qt::AlignRight);
    for (double d = 0; d <= L.total; d += distStep) {
        const double x = X(d);
        if (x < L.padL || x > L.padL + L.plotW) continue;
        text(p, x, h - 8, fmtDist(d), Qt::AlignHCenter);
    }
    // Header
    p->setPen(QColor(0x00, 0xd2, 0xff));
    p->setFont(ui(10));
    text(p, L.padL, 15, QStringLiteral("ELEVATION PROFILE"), Qt::AlignLeft);
    const RouteStats &st = m_plan->compiled().stats;
    QStringList parts{fmtDist(st.lengthM), fmtTime(st.durationS), QStringLiteral("%1 WP").arg(st.waypoints)};
    if (st.minAgl && st.maxAgl) parts << QStringLiteral("AGL %1–%2 m").arg(qRound64(*st.minAgl)).arg(qRound64(*st.maxAgl));
    if (st.maxAmsl) parts << QStringLiteral("AMSL max %1 m").arg(qRound64(*st.maxAmsl));
    const QString mode = P.value(QStringLiteral("altMode")).toString();
    parts << (mode == QLatin1String("amsl") ? QStringLiteral("AMSL mode") : mode == QLatin1String("rel") ? QStringLiteral("REL mode") : QStringLiteral("AGL mode"));
    p->setPen(kMuted);
    p->setFont(mono(9));
    text(p, L.padL + 130, 15, parts.join(QStringLiteral("   ·   ")), Qt::AlignLeft);
    if (m_zoom > 1.05) text(p, w - L.padR, 15, QStringLiteral("%1×").arg(m_zoom, 0, 'f', 1), Qt::AlignRight);
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
    text(p, rx + 10, ry + 15, QStringLiteral("LINK PROFILE"), Qt::AlignLeft);
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
        text(p, rx + w / 2, ry + h / 2 + 3, ev && ev->clazz == Link::Unknown ? QStringLiteral("no elevation data along the path")
                                                                            : QStringLiteral("hover the profile to see the line of sight"),
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
        text(p, std::min(rx + w - 26, std::max(rx + 26, wx)), wy - 12, QStringLiteral("−%1 dB").arg(ev->loss, 0, 'f', 1), Qt::AlignHCenter);
    }

    // Axis text and numbers
    p->setPen(kDim);
    p->setFont(mono(8.5));
    text(p, X(0) + 4, ry + padT + ph + 10, QStringLiteral("GS"), Qt::AlignLeft);
    text(p, X(D), ry + padT + ph + 10, fmtDist(D), Qt::AlignRight);
    p->setPen(QColor(0x55, 0x55, 0x66));
    text(p, rx + padL, Y(lo) - 2, QStringLiteral("%1 m").arg(qRound64(lo)), Qt::AlignLeft);
    const QString status = ev->obstructed ? QStringLiteral("NLOS · diffraction") : ev->fresnelClear ? QStringLiteral("LOS · Fresnel clear") : QStringLiteral("LOS · Fresnel intruded");
    p->setFont(mono(9));
    p->setPen(st.color);
    text(p, rx + padL, ry + h - 6, st.shortName, Qt::AlignLeft);
    p->setPen(QColor(0xcf, 0xd8, 0xdd));
    text(p, rx + padL + 62, ry + h - 6,
         QStringLiteral("%1 dBm · margin %2%3 dB · %4").arg(ev->rssi, 0, 'f', 0).arg(ev->margin >= 0 ? QStringLiteral("+") : QString()).arg(ev->margin, 0, 'f', 0).arg(status),
         Qt::AlignLeft);
    p->restore();
}

} // namespace corv
