// PlanMapItem.h - the Flight Plan map (the Leaflet map of FlightPlanController.js).
//
// Layers, bottom to top as the Leaflet panes: OSM / satellite tiles, radio
// coverage, link halo, segment figures, calculated path, calculated points,
// photo dots, base points and markers, the hovered footprint, labels and
// arrows, the drag handles of the selected segment, the figure being drawn.
//
// Painting reads PlanController while the GUI thread is blocked in the sync
// step; every edit goes back through the controller on the GUI thread.
#pragma once

#include "mission/RouteModel.h"

#include <QImage>
#include <QPointF>
#include <QQuickPaintedItem>
#include <QVariantList>
#include <memory>
#include <optional>

namespace corv {

namespace mission { struct Coverage; }
class PlanController;

class PlanMapItem : public QQuickPaintedItem
{
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(int zoom READ zoom NOTIFY viewChanged)
    /// The UAS zone under the mouse, for a tooltip (empty: none).
    Q_PROPERTY(QString zoneTip READ zoneTip NOTIFY zoneTipChanged)

public:
    explicit PlanMapItem(QQuickItem *parent = nullptr);

    int zoom() const { return m_zoom; }
    Q_INVOKABLE void zoomIn() { zoomAt(m_zoom + 1, {width() / 2, height() / 2}); }
    Q_INVOKABLE void zoomOut() { zoomAt(m_zoom - 1, {width() / 2, height() / 2}); }
    Q_INVOKABLE void fitRoute();

    void paint(QPainter *p) override;

public:
    QString zoneTip() const { return m_zoneTip; }

signals:
    void zoneTipChanged();
    void viewChanged();
    /// Right click: items = [[act, label, danger]], to show at (x, y).
    void contextMenu(qreal x, qreal y, const QVariantList &items, double lat, double lng, const QString &segId, int pointIdx);
    void clickedAway();

protected:
    void componentComplete() override;
    void mousePressEvent(QMouseEvent *e) override;
    void mouseMoveEvent(QMouseEvent *e) override;
    void mouseReleaseEvent(QMouseEvent *e) override;
    void mouseDoubleClickEvent(QMouseEvent *e) override;
    void hoverMoveEvent(QHoverEvent *e) override;
    void hoverLeaveEvent(QHoverEvent *e) override;
    void wheelEvent(QWheelEvent *e) override;

private:
    QPointF project(double lat, double lng) const;          // world px at m_zoom
    mission::LL unproject(QPointF w) const;
    QPointF origin() const;
    QPointF toScreen(double lat, double lng) const { return project(lat, lng) - origin(); }
    QPointF toScreen(const mission::LL &p) const { return toScreen(p.lat, p.lng); }
    mission::LL toLL(QPointF s) const { return unproject(s + origin()); }
    void zoomAt(int z, QPointF anchor);
    void setView(double lat, double lng, int zoom);

    void drawTiles(QPainter *p, const QString &provider, int maxNative);
    void drawCoverage(QPainter *p);

    // Hit testing (GUI thread)
    struct Hit {
        enum Kind { None, Vertex, Mid, Centre, Radius, Home, Operator, Point, Figure } kind = None;
        QString segId;
        int index = -1;
    };
    Hit hitTest(QPointF s, bool forContext) const;

    PlanController *m_plan = nullptr;
    mission::LL m_center{47.2603, 11.3439};
    int m_zoom = 13;
    bool m_initialised = false;

    // Gestures
    enum class Drag { None, Pan, Vertex, Centre, Radius, Home, Operator };
    Drag m_drag = Drag::None;
    QPointF m_press;
    mission::LL m_pressCenter;
    bool m_moved = false;
    Hit m_pressHit;
    QList<mission::LL> m_dragOriginal;
    mission::LL m_dragStartLL;
    std::optional<mission::LL> m_dragMarker;      // home / operator while dragged
    QString m_gestureLabel;

    // Hover
    std::optional<mission::LL> m_mouseLL;
    int m_hoverPhoto = -1, m_hoverCone = -1;

    // Coverage image cache (render thread)
    const mission::Coverage *m_covFor = nullptr;
    QImage m_covImage;
    mission::LL m_covNW, m_covSE;
    QString m_zoneTip;
};

} // namespace corv
