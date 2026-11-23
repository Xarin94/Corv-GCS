#include "gpu/FrameStats.h"

#include "core/DebugLog.h"

#include <algorithm>

namespace corv {

namespace {
constexpr double kLogMs = 30000;
constexpr double kPauseMs = 1000;   // a longer gap is a view that was hidden, not a late frame

double at(const std::vector<float> &sorted, double q)
{
    return sorted.empty() ? 0 : double(sorted[std::min(sorted.size() - 1, std::size_t(q * double(sorted.size())))]);
}
} // namespace

FrameStats &FrameStats::instance()
{
    static FrameStats s;
    return s;
}

void FrameStats::Window::clear()
{
    interval.clear();
    render.clear();
    sync.clear();
    swap.clear();
    gui.clear();
}

FrameStats::Summary FrameStats::summarise(Window w)
{
    for (auto *v : {&w.interval, &w.render, &w.sync, &w.swap, &w.gui}) std::sort(v->begin(), v->end());
    Summary s;
    s.frames = int(w.interval.size());
    s.median = at(w.interval, 0.5);
    s.p95 = at(w.interval, 0.95);
    s.p99 = at(w.interval, 0.99);
    s.worst = at(w.interval, 1);
    for (float v : w.interval) {
        if (s.median <= 0 || v <= 1.5 * s.median) continue;
        ++s.late;
        s.dropped += int(double(v) / s.median + 0.5) - 1;
    }
    s.renderWorst = at(w.render, 1);
    s.renderP99 = at(w.render, 0.99);
    s.syncWorst = at(w.sync, 1);
    s.swapWorst = at(w.swap, 1);
    s.swapP99 = at(w.swap, 0.99);
    s.guiWorst = at(w.gui, 1);
    s.guiP99 = at(w.gui, 0.99);
    return s;
}

void FrameStats::frame(double startMs, double syncMs, double renderMs)
{
    QMutexLocker l(&m_lock);
    const double dt = m_last >= 0 ? startMs - m_last : -1;
    m_last = startMs;
    m_renderEnd = startMs + renderMs;
    if (m_logStart < 0) m_logStart = startMs;
    for (Window *w : {&m_user, &m_log}) {
        if (w->render.size() >= kMax) continue;
        if (dt >= 0 && dt < kPauseMs) w->interval.push_back(float(dt));
        w->render.push_back(float(renderMs));
        w->sync.push_back(float(syncMs));
    }
    if (startMs - m_logStart < kLogMs) return;
    m_logStart = startMs;
    const Summary s = summarise(std::exchange(m_log, {}));
    l.unlock();
    if (s.late > 0)
        LOG_I("perf") << s.late << "late frames of" << s.frames << "in 30 s: worst" << QString::number(s.worst, 'f', 0) << "ms (median"
                      << QString::number(s.median, 'f', 1) << "), render" << QString::number(s.renderWorst, 'f', 1) << "ms, sync"
                      << QString::number(s.syncWorst, 'f', 1) << "ms, frame end" << QString::number(s.swapWorst, 'f', 1) << "ms, GUI thread" << QString::number(s.guiWorst, 'f', 1) << "ms";
}

void FrameStats::swapped(double nowMs)
{
    QMutexLocker l(&m_lock);
    if (m_renderEnd < 0) return;   // a frame without the 3D view
    for (Window *w : {&m_user, &m_log})
        if (w->swap.size() < kMax) w->swap.push_back(float(nowMs - m_renderEnd));
    m_renderEnd = -1;
}

void FrameStats::gui(double waitMs)
{
    QMutexLocker l(&m_lock);
    for (Window *w : {&m_user, &m_log})
        if (w->gui.size() < kMax) w->gui.push_back(float(waitMs));
}

FrameStats::Summary FrameStats::summary() const
{
    QMutexLocker l(&m_lock);
    const Window w = m_user;
    l.unlock();
    return summarise(w);
}

void FrameStats::reset()
{
    QMutexLocker l(&m_lock);
    m_user.clear();
}

} // namespace corv
