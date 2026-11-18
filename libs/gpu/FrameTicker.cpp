#include "gpu/FrameTicker.h"

#include "core/Clock.h"

namespace corv {

namespace {
constexpr double kSlackMs = 8;        // half a 60 fps frame: 33 ms fires on every second tick
constexpr int kTicksLostMs = 250;     // no tick for this long: the timer of its own again
} // namespace

FrameTicker *FrameTicker::instance()
{
    static FrameTicker t;
    return &t;
}

PacedTimer::PacedTimer(QObject *parent) : QObject(parent)
{
    connect(&m_timer, &QTimer::timeout, this, [this] { fire(monoMs()); });
    m_watchdog.setSingleShot(true);
    m_watchdog.setInterval(kTicksLostMs);
    connect(&m_watchdog, &QTimer::timeout, this, [this] {
        if (m_on) m_timer.start();
    });
    connect(FrameTicker::instance(), &FrameTicker::tick, this, [this] {
        if (!m_on) return;
        m_timer.stop();
        m_watchdog.start();
        const double now = monoMs();
        if (now - m_lastMs >= m_interval - kSlackMs) fire(now);
    });
}

void PacedTimer::setInterval(int ms)
{
    m_interval = ms;
    m_timer.setInterval(ms);
}

void PacedTimer::start()
{
    m_on = true;
    m_timer.start();
}

void PacedTimer::stop()
{
    m_on = false;
    m_timer.stop();
    m_watchdog.stop();
}

void PacedTimer::fire(double nowMs)
{
    m_lastMs = nowMs;
    emit timeout();
}

} // namespace corv
