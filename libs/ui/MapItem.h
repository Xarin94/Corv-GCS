// MapItem.h - 2D map in Web Mercator (the mini-map of MapEngine.js).
//
// Satellite tiles from TileStore, the red trail (3000 points at most), the mission,
// home, the target, the vehicle arrow, ADS-B traffic dots (the callsign under the mouse). Wheel, touchpad, pinch and the
// + / - buttons zoom, about the vehicle while the map follows it. As the main
// view (panEnabled) a drag moves the map off the vehicle, and a double click, a
// double tap or the centre button brings it back; in the corner panel it always
// follows. Drawn with QPainter: the layer split of the guide (tiles as scene
// graph nodes) comes with the FLIGHT PLAN page, where the map is large.
#pragma once

#include "core/Geo.h"
#include "gpu/FrameTicker.h"
#include "gpu/TouchGesture.h"

#include <QPointF>
#include <QQuickPaintedItem>
#include <QTimer>
#include <deque>
#include <functional>
#include <optional>
#include <vector>

namespace corv {

class VehicleHub;
struct VehicleState;

class MapItem : public QQuickPaintedItem, private TouchGesture::Client
{
    Q_OBJECT
    QML_ELEMENT
    /// Centred on the vehicle.
    Q_PROPERTY(bool follow READ follow NOTIFY followChanged)
    /// A drag moves the map (the main view); off, it stays on the vehicle.
    Q_PROPERTY(bool panEnabled READ panEnabled WRITE setPanEnabled NOTIFY panEnabledChanged)
    Q_PROPERTY(double zoom READ zoom NOTIFY zoomChanged)

public:
    explicit MapItem(QQuickItem *parent = nullptr);
    static void setHub(VehicleHub *hub) { s_hub = hub; }
    /// LOG REPLAY: the whole recorded flight drawn as the trail, live sampling
    /// paused; an empty list resumes the live trail from scratch.
    static void setFrozenTrail(std::vector<LatLon> points);
    /// The mission's points, in order (FLIGHT PLAN compiles it).
    struct MissionPoint {
        LatLon at;
        bool home = false;
    };
    static void setMission(std::vector<MissionPoint> points);
    /// GCS OPTIONS -> TARGET.
    static void setTarget(std::optional<LatLon> target);

    void paint(QPainter *p) override;
    Q_INVOKABLE void clearTrail() { m_trail.clear(); update(); }

    bool follow() const { return m_follow; }
    bool panEnabled() const { return m_pan; }
    void setPanEnabled(bool on);
    double zoom() const { return m_zoom; }
    /// The next whole zoom level in or out, about the centre.
    Q_INVOKABLE void zoomIn();
    Q_INVOKABLE void zoomOut();
    Q_INVOKABLE void recentre() { setFollow(true); }

signals:
    void followChanged();
    void panEnabledChanged();
    void zoomChanged();
    /// Click on the map (guided target in the Electron version).
    void clicked(double lat, double lon);

protected:
    void wheelEvent(QWheelEvent *e) override;
    void mousePressEvent(QMouseEvent *e) override;
    void mouseMoveEvent(QMouseEvent *e) override;
    void mouseReleaseEvent(QMouseEvent *e) override;
    void mouseDoubleClickEvent(QMouseEvent *e) override;
    void hoverMoveEvent(QHoverEvent *e) override;
    void hoverLeaveEvent(QHoverEvent *e) override;
    void touchEvent(QTouchEvent *e) override { m_touch.event(e); }
    void touchUngrabEvent() override { m_touch.cancel(); }
    bool event(QEvent *e) override;

private:
    void touchDragBegin(QPointF press) override;
    void touchDrag(QPointF p) override;
    void touchTap(QPointF p) override;
    void touchDoubleTap(QPointF p) override;
    void pinch(QPointF centre, QPointF moved, double scale) override;

    void panFrom(QPointF press);
    void panTo(QPointF p);
    void click(QPointF p);
    QPointF project(double lat, double lon) const;   // world pixels at the current zoom
    LatLon unproject(QPointF world) const;
    void sample();
    /// Zoom with the point under `anchor` fixed (the vehicle, while the map follows it).
    void zoomAt(double z, QPointF anchor);
    void setFollow(bool f);
    void drawTiles(QPainter *p, QPointF origin);
    void drawCells(QPainter *p, const VehicleState &s, const std::function<QPointF(double, double)> &toScreen);
    void drawMission(QPainter *p, const std::function<QPointF(double, double)> &toScreen);
    void drawTraffic(QPainter *p, const VehicleState &s, const std::function<QPointF(double, double)> &toScreen);
    void drawVehicle(QPainter *p, QPointF at, double yaw);

    static VehicleHub *s_hub;
    static std::vector<LatLon> s_frozen;
    static int s_frozenEpoch;
    static std::vector<MissionPoint> s_mission;
    static std::optional<LatLon> s_target;
    int m_frozenEpoch = 0;
    bool m_wasRelative = false;
    PacedTimer m_timer;   // on the 3D view's frames while it paces them (RENDER FPS CAP)
    double m_zoom = 13;
    bool m_follow = true;
    bool m_pan = false;
    TouchGesture m_touch{this};
    LatLon m_center = kDefaultOrigin;
    std::deque<LatLon> m_trail;
    std::optional<QPointF> m_hover;
    QPointF m_press, m_pressScene;
    LatLon m_pressCenter;
    bool m_dragged = false;
};

} // namespace corv
