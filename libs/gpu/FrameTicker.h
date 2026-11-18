// FrameTicker.h - the paced frames of the 3D view (RENDER FPS CAP) for the rest
// of the window.
//
// Capped, the 3D view asks for its frames on a vblank grid (FramePacer) and
// ticks just before each. Whatever else changes the window continuously must
// change it on those ticks: a change at any other moment asks for a frame of
// its own, and the window draws far more often than the cap. The HUD, moving
// inside every frame (afterAnimating), would keep it at every vblank; the
// readouts and the mini-map, polled on timers of their own, would add frames
// in between. The Electron version drew 3D and HUD in the same capped
// requestAnimationFrame callback, which is what this restores.
#pragma once

#include <QObject>
#include <QTimer>

namespace corv {

class FrameTicker : public QObject
{
    Q_OBJECT
public:
    static FrameTicker *instance();
signals:
    /// A frame was requested for the vblank at frameWallMs (wall clock, ms).
    void tick(double frameWallMs);
};

/// A timer that, while the 3D view paces the frames, fires on its ticks instead
/// (at least an interval apart, less half a 60 fps frame), so what it changes on
/// screen is drawn in frames that are drawn anyway. Without a tick for 250 ms
/// (the view hidden, no cap) it fires on its own interval again.
class PacedTimer : public QObject
{
    Q_OBJECT
public:
    explicit PacedTimer(QObject *parent = nullptr);
    void setInterval(int ms);
    void start();
    void stop();
    bool isActive() const { return m_on; }

signals:
    void timeout();

private:
    void fire(double nowMs);

    QTimer m_timer, m_watchdog;
    int m_interval = 0;
    bool m_on = false;
    double m_lastMs = -1e9;
};

} // namespace corv
