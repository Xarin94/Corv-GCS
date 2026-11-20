// MapItem.h - 2D map in Web Mercator (the mini-map of MapEngine.js).
//
// Satellite tiles from TileStore, the red trail (3000 points at most),
// home, the vehicle arrow, ADS-B traffic dots. Follows the vehicle until it is
// dragged; a double click recentres. Wheel zooms. Drawn with QPainter: the
// layer split of the guide (tiles as scene graph nodes) comes with the FLIGHT
// PLAN page, where the map is large.
#pragma once

#include "core/Geo.h"
#include "gpu/FrameTicker.h"

#include <QPointF>
#include <QQuickPaintedItem>
#include <QTimer>
#include <deque>
#include <functional>
#include <vector>

namespace corv {

class VehicleHub;
struct VehicleState;

class MapItem : public QQuickPaintedItem
{
    Q_OBJECT
    QML_ELEMENT

public:
    explicit MapItem(QQuickItem *parent = nullptr);
    static void setHub(VehicleHub *hub) { s_hub = hub; }
    /// LOG REPLAY: the whole recorded flight drawn as the trail, live sampling
    /// paused; an empty list resumes the live trail from scratch.
    static void setFrozenTrail(std::vector<LatLon> points);

    void paint(QPainter *p) override;
    Q_INVOKABLE void clearTrail() { m_trail.clear(); update(); }

signals:
    /// Click on the map (guided target in the Electron version).
    void clicked(double lat, double lon);

protected:
    void wheelEvent(QWheelEvent *e) override;
    void mousePressEvent(QMouseEvent *e) override;
    void mouseMoveEvent(QMouseEvent *e) override;
    void mouseReleaseEvent(QMouseEvent *e) override;
    void mouseDoubleClickEvent(QMouseEvent *e) override;

private:
    QPointF project(double lat, double lon) const;   // world pixels at the current zoom
    LatLon unproject(QPointF world) const;
    void sample();
    void setZoom(int z);
    void setFollow(bool f);
    void drawTiles(QPainter *p, QPointF origin);
    void drawCells(QPainter *p, const VehicleState &s, const std::function<QPointF(double, double)> &toScreen);
    void drawVehicle(QPainter *p, QPointF at, double yaw);

    static VehicleHub *s_hub;
    static std::vector<LatLon> s_frozen;
    static int s_frozenEpoch;
    int m_frozenEpoch = 0;
    PacedTimer m_timer;   // on the 3D view's frames while it paces them (RENDER FPS CAP)
    int m_zoom = 13;
    bool m_follow = true;
    LatLon m_center = kDefaultOrigin;
    std::deque<LatLon> m_trail;
    QPointF m_press;
    LatLon m_pressCenter;
    bool m_dragged = false;
};

} // namespace corv
