#include "scene/FramePacer.h"

#include "core/Clock.h"

#include <algorithm>
#include <cmath>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <dwmapi.h>
#endif

namespace corv {

double FramePacer::next(double nowMs, double refreshHz, const Vsync &vs)
{
    if (m_cap <= 0) {
        m_dueMs = nowMs;
        return nowMs;
    }
    const double period = vs.periodMs > 0 ? vs.periodMs : 1000.0 / (refreshHz > 1 ? refreshHz : 60.0);
    const int vblanks = std::max(1, int(std::lround(1000.0 / period / m_cap)));
    const double interval = vblanks * period;
    const double lead = period / 2;

    // One interval after the last frame was due; the first frame, or the first
    // after a stall (view hidden, a frame longer than the interval): at once
    double due = m_dueMs >= 0 && nowMs - m_dueMs < interval ? m_dueMs + interval : nowMs + lead;
    if (vs.periodMs > 0) due = vs.vblankMs + std::round((due - vs.vblankMs) / vs.periodMs) * vs.periodMs;
    m_dueMs = due;
    return std::max(nowMs, due - lead);
}

FramePacer::Vsync FramePacer::systemVsync()
{
#ifdef _WIN32
    DWM_TIMING_INFO ti{};
    ti.cbSize = sizeof ti;
    if (FAILED(DwmGetCompositionTimingInfo(nullptr, &ti)) || ti.qpcRefreshPeriod == 0) return {};
    LARGE_INTEGER freq, qpc;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&qpc);
    const double now = monoMs();
    const double toMs = 1000.0 / double(freq.QuadPart);
    return {double(ti.qpcRefreshPeriod) * toMs, now - double(qpc.QuadPart - LONGLONG(ti.qpcVBlank)) * toMs};
#else
    return {};
#endif
}

} // namespace corv
