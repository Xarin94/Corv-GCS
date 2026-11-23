// FrameStats.h - how evenly the 3D view is drawn, for the debug log and the
// debug server ("stats").
//
// For every frame: the time since the previous one, what synchronize() and
// render() took on the render thread, what the end of the frame took there
// (texture uploads and the swap: the GPU), and how late the GUI thread answered
// the frame's "drawn" notice and the timer of the next one (its event loop was
// busy with something else that long). A frame is late when it came more than
// half an interval after it was due, the interval being the median of the
// window. Every 30 s a line goes to the debug log when frames were late, so a
// report of stutter says which thread held the frame up.
#pragma once

#include <QMutex>
#include <vector>

namespace corv {

class FrameStats
{
public:
    struct Summary {
        int frames = 0, late = 0, dropped = 0;            // dropped: frame slots missed, all late frames together
        double median = 0, p95 = 0, p99 = 0, worst = 0;   // interval between frames, ms
        double renderWorst = 0, syncWorst = 0, swapWorst = 0, guiWorst = 0;
        double renderP99 = 0, swapP99 = 0, guiP99 = 0;
    };

    static FrameStats &instance();

    /// Render thread, once per frame: when render() began and what it and synchronize() took.
    void frame(double startMs, double syncMs, double renderMs);
    /// Render thread, when the frame was presented.
    void swapped(double nowMs);
    /// GUI thread: how late it answered (the "drawn" notice, the frame timer).
    void gui(double waitMs);
    /// Since the last reset(), for the debug server.
    Summary summary() const;
    void reset();

private:
    struct Window {
        std::vector<float> interval, render, sync, swap, gui;
        void clear();
    };
    static Summary summarise(Window w);
    static constexpr std::size_t kMax = 20000;

    mutable QMutex m_lock;
    Window m_user, m_log;
    double m_last = -1, m_logStart = -1, m_renderEnd = -1;
};

} // namespace corv
