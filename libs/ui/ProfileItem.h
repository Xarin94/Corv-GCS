// ProfileItem.h - the elevation profile under the Flight Plan map (renderProfile
// of FlightPlanController.js, drawLinkBand / drawLinkInset of RadioLinkView.js).
//
// Terrain under the calculated path, the route line with its points, the
// minimum-clearance floor and the stretches under it, the radio link band and,
// with LINK on, the line-of-sight cut to the hovered (or worst) point.
// Ctrl+wheel zooms, wheel and drag scroll, a click selects the segment under
// the cursor. The tooltip is a QML Text fed by tipText / tipX / tipY.
#pragma once

#include "mission/RadioLink.h"

#include <QQuickPaintedItem>
#include <optional>
#include <vector>

namespace corv {

class PlanController;

class ProfileItem : public QQuickPaintedItem
{
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(bool tipVisible READ tipVisible NOTIFY tipChanged)
    Q_PROPERTY(QString tipText READ tipText NOTIFY tipChanged)
    Q_PROPERTY(qreal tipX READ tipX NOTIFY tipChanged)
    Q_PROPERTY(qreal tipY READ tipY NOTIFY tipChanged)

public:
    explicit ProfileItem(QQuickItem *parent = nullptr);
    void paint(QPainter *p) override;

    bool tipVisible() const { return m_tipVisible; }
    QString tipText() const { return m_tipText; }
    qreal tipX() const { return m_tipX; }
    qreal tipY() const { return m_tipY; }

signals:
    void tipChanged();

protected:
    void componentComplete() override;
    void wheelEvent(QWheelEvent *e) override;
    void mousePressEvent(QMouseEvent *e) override;
    void mouseMoveEvent(QMouseEvent *e) override;
    void mouseReleaseEvent(QMouseEvent *e) override;
    void hoverMoveEvent(QHoverEvent *e) override;
    void hoverLeaveEvent(QHoverEvent *e) override;

private:
    struct Sample { double dist; std::optional<double> elev; double route; bool vertical; };
    struct Layout {
        double w = 0, h = 0, padL = 52, padR = 16, padT = 34, padB = 24, insetW = 0;
        double plotW = 0, plotH = 0, lo = 0, hi = 0, range = 1, total = 1, zoom = 1, scroll = 0;
        bool linkOn = false;
        double X(double d) const { return padL + d / total * plotW * zoom - scroll; }
        double Y(double a) const { return padT + plotH - (a - lo) / range * plotH; }
        bool inPlot(double x, double margin = 0) const { return x >= padL - margin && x <= padL + plotW + margin; }
        double bandY() const { return padT - 11; }
    };
    bool layout(Layout &L) const;
    void rebuildSamples();
    void updateHover();
    void clampScroll(const Layout &L);
    void drawGrid(QPainter *p, const Layout &L, double altStep, double distStep) const;
    void drawLinkBand(QPainter *p, const Layout &L) const;
    void drawTerrain(QPainter *p, const Layout &L) const;
    void drawRoute(QPainter *p, const Layout &L, const QString &sel) const;
    void drawPoints(QPainter *p, const Layout &L, const QString &sel) const;
    void drawAxes(QPainter *p, const Layout &L, double altStep, double distStep) const;
    void drawInset(QPainter *p, const QRectF &rect, const mission::LinkEval *ev, const QString &title) const;

    PlanController *m_plan = nullptr;
    int m_samplesRev = -1;
    std::vector<Sample> m_samples;
    double m_zoom = 1, m_scroll = 0;
    bool m_dragging = false, m_moved = false;
    double m_dragX = 0, m_scrollStart = 0;
    std::optional<QPointF> m_hover;

    // Hover results (GUI thread, read by paint during sync)
    std::optional<double> m_hoverDist;
    double m_hoverAlt = 0;
    QString m_hoverSeg;
    std::optional<mission::LinkEval> m_hoverLink;

    bool m_tipVisible = false;
    QString m_tipText;
    qreal m_tipX = 0, m_tipY = 0;
};

} // namespace corv
