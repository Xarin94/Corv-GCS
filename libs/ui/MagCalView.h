// MagCalView.h - the 3D view of the magnetometers during compass calibration
// (the drawing half of js/ui/MagCal3D.js).
//
// What Mission Planner's live compass calibration shows: every sample of
// Calibration.samples() as a point, recentred and scaled by its compass's fitted
// sphere; the 80 sections of the firmware's completion mask as a translucent
// sphere, green once covered; the latest sample as a white marker and the
// nearest section still missing in orange; the body axes and an arrowhead for
// the vehicle. Drag turns, wheel or pinch zooms, a double click resets.
//
// The guide planned a small QQuickRhiItem; a few thousand points and 1280
// triangles drawn on demand cost QPainter well under a millisecond, so it is a
// QQuickPaintedItem like the other instruments. It repaints when the samples,
// the coverage or the camera change, never on its own.
#pragma once

#include "gpu/TouchGesture.h"

#include <QQuickPaintedItem>

namespace corv {

class CalibrationController;

class MagCalView : public QQuickPaintedItem, private TouchGesture::Client
{
    Q_OBJECT
    QML_ELEMENT
public:
    explicit MagCalView(QQuickItem *parent = nullptr);
    void paint(QPainter *p) override;

    Q_INVOKABLE void resetView();

protected:
    void componentComplete() override;
    void mousePressEvent(QMouseEvent *e) override;
    void mouseMoveEvent(QMouseEvent *e) override;
    void mouseReleaseEvent(QMouseEvent *e) override;
    void mouseDoubleClickEvent(QMouseEvent *e) override;
    void wheelEvent(QWheelEvent *e) override;
    void touchEvent(QTouchEvent *e) override;
    void touchUngrabEvent() override { m_touch.cancel(); }

private:
    void touchDragBegin(QPointF press) override { m_drag = press; }
    void touchDrag(QPointF p) override { turnTo(p); }
    void touchDoubleTap(QPointF) override { resetView(); }
    void pinch(QPointF centre, QPointF moved, double scale) override;
    void turnTo(QPointF p);

    CalibrationController *m_cal = nullptr;
    double m_yaw = 0.65, m_pitch = 0.42, m_dist = 4.4;
    QPointF m_drag;
    bool m_dragging = false;
    TouchGesture m_touch{this};
};

} // namespace corv
