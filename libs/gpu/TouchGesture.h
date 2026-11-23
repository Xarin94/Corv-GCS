// TouchGesture.h - fingers on the maps and the 3D view.
//
// Qt hands a touch to an item that only knows the mouse as a mouse drag of the
// first finger, and drops the others: a pinch panned the map away from the
// aircraft and never zoomed. An item that takes its touch events here gets
//   one finger    press, then a drag once it has moved more than a finger
//                 trembles, else a tap, a double tap or a long press;
//   two fingers   a pinch: the scale and the movement of their midpoint since
//                 the last event. The finger left down after it does nothing.
// Nothing of the first finger's press reaches the item's drag if a second
// finger comes before it has moved: touchCancel(). Movement is measured in the
// window: an item that moves or grows under the finger (the corner panel of the
// flight screen) is not a drag. The positions handed over are the item's, as it
// was when the finger came down.
//
// Also the zoom a wheel asks for, which is not one level an event: a touchpad
// sends many small steps, and horizontal ones that are no zoom at all.
// GUI thread.
#pragma once

#include <QElapsedTimer>
#include <QPointF>
#include <QTimer>

QT_FORWARD_DECLARE_CLASS(QNativeGestureEvent)
QT_FORWARD_DECLARE_CLASS(QTouchEvent)
QT_FORWARD_DECLARE_CLASS(QWheelEvent)

namespace corv {

class TouchGesture
{
public:
    class Client
    {
    public:
        virtual ~Client() = default;
        virtual void touchPress(QPointF) {}
        /// The finger set off from `press`.
        virtual void touchDragBegin(QPointF press) { Q_UNUSED(press) }
        virtual void touchDrag(QPointF) {}
        virtual void touchDragEnd(QPointF) {}
        virtual void touchTap(QPointF) {}
        virtual void touchDoubleTap(QPointF p) { touchTap(p); }
        virtual void touchLongPress(QPointF) {}
        /// A second finger, or the system took the touch: what the first one began is over.
        virtual void touchCancel() {}
        virtual void pinch(QPointF centre, QPointF moved, double scale) { Q_UNUSED(centre) Q_UNUSED(moved) Q_UNUSED(scale) }
        virtual void pinchEnd() {}
    };

    static constexpr double kSlopPx = 12;         // a finger held still moves this much
    static constexpr int kLongPressMs = 600;
    static constexpr int kDoubleTapMs = 350;
    static constexpr double kDoubleTapPx = 40;

    explicit TouchGesture(Client *client);

    /// From touchEvent(); always accepted.
    void event(QTouchEvent *e);
    /// From touchUngrabEvent().
    void cancel();
    bool active() const { return m_mode != Mode::Idle; }

    /// Zoom levels a wheel asks for, from the level shown: whole levels for a mouse notch, fractions for a touchpad.
    static double wheelZoom(const QWheelEvent *e, double level);
    /// Zoom levels of a touchpad pinch (macOS); false when the gesture is another.
    static bool nativeZoom(const QNativeGestureEvent *e, double &levels);

private:
    enum class Mode { Idle, Press, Drag, Held, Pinch, Over };
    void startPinch(const QTouchEvent *e);
    /// Where a finger is, for the item as it was at the press.
    QPointF local(QPointF scene) const { return m_press + (scene - m_pressScene); }

    Client *m_client;
    Mode m_mode = Mode::Idle;
    QPointF m_press, m_pressScene, m_last;
    int m_id = -1, m_idA = -1, m_idB = -1;
    QPointF m_centre;   // in the window
    double m_span = 0;
    QTimer m_hold;
    QElapsedTimer m_tapClock;
    QPointF m_tapAt;
};

} // namespace corv
