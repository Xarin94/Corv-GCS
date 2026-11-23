#include "gpu/TouchGesture.h"

#include <QNativeGestureEvent>
#include <QTouchEvent>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>

namespace corv {

namespace {
double dist(QPointF a, QPointF b) { return std::hypot(a.x() - b.x(), a.y() - b.y()); }
constexpr double kMinSpanPx = 20;   // fingers closer than this give no steady scale
} // namespace

TouchGesture::TouchGesture(Client *client) : m_client(client)
{
    m_hold.setSingleShot(true);
    m_hold.setInterval(kLongPressMs);
    QObject::connect(&m_hold, &QTimer::timeout, &m_hold, [this] {
        if (m_mode != Mode::Press) return;
        m_mode = Mode::Held;
        m_client->touchLongPress(m_last);
    });
}

void TouchGesture::startPinch(const QTouchEvent *e)
{
    QList<const QEventPoint *> down;
    for (const QEventPoint &p : e->points())
        if (p.state() != QEventPoint::State::Released) down << &p;
    if (down.size() < 2) return;
    m_hold.stop();
    if (m_mode == Mode::Press || m_mode == Mode::Drag || m_mode == Mode::Held) m_client->touchCancel();
    m_mode = Mode::Pinch;
    m_idA = down[0]->id();
    m_idB = down[1]->id();
    m_centre = (down[0]->scenePosition() + down[1]->scenePosition()) / 2;
    m_span = std::max(kMinSpanPx, dist(down[0]->scenePosition(), down[1]->scenePosition()));
    m_tapClock.invalidate();
}

void TouchGesture::event(QTouchEvent *e)
{
    e->accept();
    if (e->type() == QEvent::TouchCancel) {
        cancel();
        return;
    }
    const auto &points = e->points();
    int down = 0;
    for (const QEventPoint &p : points)
        if (p.state() != QEventPoint::State::Released) ++down;

    if (m_mode == Mode::Idle && down == 1) {
        for (const QEventPoint &p : points) {
            if (p.state() != QEventPoint::State::Pressed) continue;
            m_mode = Mode::Press;
            m_id = p.id();
            m_press = m_last = p.position();
            m_pressScene = p.scenePosition();
            m_hold.start();
            m_client->touchPress(m_press);
        }
        return;
    }
    if (m_mode != Mode::Pinch && m_mode != Mode::Over && down >= 2) {
        if (m_mode == Mode::Idle) {   // both fingers at once
            m_press = points.first().position();
            m_pressScene = points.first().scenePosition();
        }
        startPinch(e);
        return;
    }

    switch (m_mode) {
    case Mode::Press:
    case Mode::Drag:
    case Mode::Held: {
        const QEventPoint *p = nullptr;
        for (const QEventPoint &q : points)
            if (q.id() == m_id) p = &q;
        if (!p) break;
        m_last = local(p->scenePosition());
        if (m_mode == Mode::Press && dist(m_last, m_press) > kSlopPx) {
            m_hold.stop();
            m_mode = Mode::Drag;
            m_client->touchDragBegin(m_press);
        }
        if (m_mode == Mode::Drag) m_client->touchDrag(m_last);
        if (p->state() != QEventPoint::State::Released) break;
        m_hold.stop();
        const Mode was = m_mode;
        m_mode = Mode::Idle;
        if (was == Mode::Drag) {
            m_client->touchDragEnd(m_last);
        } else if (was == Mode::Press) {
            const bool again = m_tapClock.isValid() && m_tapClock.elapsed() < kDoubleTapMs && dist(m_tapAt, m_last) < kDoubleTapPx;
            m_tapAt = m_last;
            if (again) m_tapClock.invalidate();   // a third tap is a first one
            else m_tapClock.start();
            if (again) m_client->touchDoubleTap(m_last);
            else m_client->touchTap(m_last);
        }
        break;
    }
    case Mode::Pinch: {
        const QEventPoint *a = nullptr, *b = nullptr;
        for (const QEventPoint &q : points) {
            if (q.id() == m_idA) a = &q;
            if (q.id() == m_idB) b = &q;
        }
        if (a && b) {
            const QPointF c = (a->scenePosition() + b->scenePosition()) / 2;
            const double span = std::max(kMinSpanPx, dist(a->scenePosition(), b->scenePosition()));
            const QPointF moved = c - m_centre;
            const double scale = span / m_span;
            m_centre = c;
            m_span = span;
            if (!moved.isNull() || scale != 1) m_client->pinch(local(c), moved, scale);
        }
        // A finger up ends it: the other one does not drag the map from wherever it is
        if (!a || !b || a->state() == QEventPoint::State::Released || b->state() == QEventPoint::State::Released) {
            m_client->pinchEnd();
            m_mode = Mode::Over;
        }
        break;
    }
    case Mode::Over:
    case Mode::Idle: break;
    }
    if (down == 0) {
        m_hold.stop();
        m_mode = Mode::Idle;
    }
}

void TouchGesture::cancel()
{
    m_hold.stop();
    const Mode was = m_mode;
    m_mode = Mode::Idle;
    if (was == Mode::Press || was == Mode::Drag || was == Mode::Held) m_client->touchCancel();
    else if (was == Mode::Pinch) m_client->pinchEnd();
}

double TouchGesture::wheelZoom(const QWheelEvent *e, double level)
{
    const int dy = e->angleDelta().y();
    if (dy == 0) return level;
    const double to = level + dy / 120.0;
    // A mouse steps from one whole level to the next
    return dy % 120 == 0 ? std::round(to) : to;
}

bool TouchGesture::nativeZoom(const QNativeGestureEvent *e, double &levels)
{
    if (e->gestureType() != Qt::ZoomNativeGesture) return false;
    levels = std::log2(std::max(0.2, 1 + e->value()));
    return true;
}

} // namespace corv
