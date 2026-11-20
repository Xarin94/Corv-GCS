#include "vehicle/VehicleHub.h"

#include "core/Clock.h"
#include "core/DebugLog.h"
#include "link/LinkManager.h"

#include <QThread>
#include <QTimer>

using namespace Qt::StringLiterals;

namespace corv {

VehicleHub::VehicleHub(LinkManager *link, QObject *parent) : QObject(parent), m_link(link)
{
    m_snapshot = std::make_shared<const VehicleState>(m_state);

    StateMapper::Hooks hooks;
    hooks.rememberLastHome = [this](double lat, double lon) { emit lastHomeChanged(lat, lon); };
    hooks.commandAck = [this](int c, int r) { emit commandAck(c, r); };
    hooks.statusText = [this](int sev, const QString &t) { emit statusText(sev, t); };
    m_mapper.setHooks(hooks);
    m_mapper.origin().onMoved = [this](LatLon o) {
        m_state.sceneOrigin = o;
        m_state.sceneOriginEpoch = m_mapper.origin().epoch();
    };

    if (link) {
        // Same thread as the link manager: direct call, no copy through the event loop
        connect(link, &LinkManager::message, this, &VehicleHub::onMessage, Qt::DirectConnection);
        connect(link, &LinkManager::connected, this, [this](const QString &type) {
            m_state.connected = true;
            m_state.mode = u"LIVE"_s;
            m_state.connectionType = type;
            publish();
            emit connectionChanged(true, type);
        }, Qt::DirectConnection);
        connect(link, &LinkManager::connectionState, this, [this](const QString &st) {
            if (st != "DISCONNECTED"_L1) return;
            // Every DISCONNECTED clears the connection (the Electron renderer kept connected = true)
            const bool was = m_state.connected;
            m_state.connected = false;
            m_state.connectionType = u"none"_s;
            m_state.linkType.clear();
            m_state.linkKbps = 0;
            publish();
            if (was) emit connectionChanged(false, QString());
        }, Qt::DirectConnection);
        connect(link, &LinkManager::linkStats, this, [this](const QString &kind, double kbps, qint64) {
            m_state.linkType = kind;
            m_state.linkKbps = kbps;
            m_state.lastLinkStatsTime = wallMs();
            publish();
        }, Qt::DirectConnection);
    }
}

template <typename F>
void VehicleHub::post(F &&f)
{
    if (QThread::currentThread() == thread()) f();
    else QMetaObject::invokeMethod(this, std::forward<F>(f), Qt::QueuedConnection);
}

std::shared_ptr<const VehicleState> VehicleHub::snapshot() const
{
    QMutexLocker lock(&m_snapLock);
    return m_snapshot;
}

void VehicleHub::publish()
{
    auto snap = std::make_shared<const VehicleState>(m_state);
    {
        QMutexLocker lock(&m_snapLock);
        m_snapshot = std::move(snap);
    }
    emit published();
}

void VehicleHub::onMessage(const MavMessage &m)
{
    // A replay frame only counts while the mode is REPLAY; live frames while a log is loaded are ignored
    if (m.replay != (m_state.mode == "REPLAY"_L1)) return;
    m_mapper.apply(m, m_state);
    m_mapper.finish(m_state, m.arrivalMs);
    publish();
}

void VehicleHub::mutate(std::function<void(VehicleState &)> fn)
{
    post([this, fn = std::move(fn)] {
        fn(m_state);
        publish();
    });
}

void VehicleHub::setMode(const QString &mode)
{
    post([this, mode] {
        m_state.mode = mode;
        publish();
    });
}

void VehicleHub::setAdsbEnabled(bool on)
{
    post([this, on] {
        m_state.adsbEnabled = on;
        if (!on) m_state.traffic.clear();
        publish();
    });
}

void VehicleHub::setBatteryRange(double vmin, double vmax)
{
    post([this, vmin, vmax] { m_mapper.setBatteryRange(vmin, vmax); });
}

void VehicleHub::setRelativeNav(const RelativeNav::Settings &s)
{
    post([this, s] {
        m_mapper.relativeNav().setSettings(s);
        m_mapper.finish(m_state, wallMs());
        publish();
    });
}

void VehicleHub::resetRelativeNav()
{
    post([this] {
        m_mapper.relativeNav().reset();
        m_mapper.finish(m_state, wallMs());
        publish();
    });
}

void VehicleHub::resetTelemetry()
{
    post([this] {
        m_mapper.resetTelemetry(m_state);
        publish();
    });
}

void VehicleHub::awaitSceneOriginFix()
{
    post([this] { m_mapper.origin().awaitFix(); });
}

void VehicleHub::setSceneOrigin(double lat, double lon)
{
    post([this, lat, lon] {
        m_mapper.origin().set(lat, lon);
        publish();
    });
}

} // namespace corv

namespace corv {

void VehicleHub::startDemo(DemoFlight::TerrainFn terrain)
{
    post([this, terrain = std::move(terrain)] {
        m_demo = std::make_unique<DemoFlight>(terrain);
        m_demoTimer = new QTimer(this);
        m_demoTimer->setTimerType(Qt::PreciseTimer);
        m_demoTimer->setInterval(16);
        connect(m_demoTimer, &QTimer::timeout, this, [this] {
            const qint64 now = wallMs();
            const double dt = m_demoLast ? (now - m_demoLast) / 1000.0 : 0.016;
            m_demoLast = now;
            if (!m_state.isDemo()) return;
            m_demo->step(m_state, dt, double(now));
            publish();
        });
        m_demoTimer->start();
    });
}

} // namespace corv
