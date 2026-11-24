#include "vehicle/ConnectionManager.h"

#include "core/Clock.h"
#include "core/DebugLog.h"
#include "link/LinkManager.h"
#include "link/MessageBus.h"
#include "vehicle/CommandSender.h"
#include "vehicle/VehicleHub.h"

using namespace Qt::StringLiterals;

namespace corv {

namespace {
constexpr qint64 kStreamRequestTimeoutMs = 15000;
} // namespace

ConnectionManager::ConnectionManager(LinkManager *link, VehicleHub *hub, MessageBus *bus, CommandSender *cmd, QObject *parent)
    : QObject(parent), m_link(link), m_hub(hub), m_bus(bus), m_cmd(cmd)
{
    m_streamPoll.setInterval(200);
    connect(&m_streamPoll, &QTimer::timeout, this, [this] {
        const bool linked = m_hub->snapshot()->lastHeartbeatTime >= m_linkedSince;
        if (!linked && wallMs() - m_linkedSince < kStreamRequestTimeoutMs) return;
        m_streamPoll.stop();
        m_cmd->requestAllDataStreams();
        QTimer::singleShot(2000, this, [this] { m_cmd->requestHomePosition(); });
        // The firmware version, asked again once if the first answer is lost
        for (const int at : {1000, 6000})
            QTimer::singleShot(at, this, [this] {
                const auto s = m_hub->snapshot();
                if (s->connected && s->firmwareVersion.isEmpty()) m_cmd->requestAutopilotVersion();
            });
    });
    m_homePoll.setInterval(5000);
    connect(&m_homePoll, &QTimer::timeout, this, [this] {
        const auto s = m_hub->snapshot();
        if (wallMs() - s->lastHeartbeatTime >= 3000 || s->homeLat) {
            stopHomePolling();
            return;
        }
        m_cmd->requestHomePosition();
    });

    connect(m_link, &LinkManager::connected, this, [this](const QString &type) {
        LOG_I("conn") << "connect" << type << "done";
        // MSP speaks no MAVLink to the vehicle: no streams to ask for
        if (!type.startsWith("msp"_L1)) requestStreamsWhenLinked();
    });
    connect(m_link, &LinkManager::connectFailed, this, [this](const QString &type, const QString &error) {
        LOG_E("conn") << "connect" << type << "failed:" << error;
        m_type.clear();
        emit connectFailed(type, error);
    });
    connect(m_link, &LinkManager::connectionState, this, [this](const QString &st) {
        if (st != "DISCONNECTED"_L1) return;
        m_streamPoll.stop();
        stopHomePolling();
        m_type.clear();
    });

    // Home after arming: the autopilot sets a new one, ask until it comes
    m_bus->subscribe(MAVLINK_MSG_ID_HEARTBEAT, this, [this](const MavMessage &m) {
        if (m.compid() == 1) onHeartbeat();
    });
}

void ConnectionManager::connectLink(const QString &type, const QVariantMap &o)
{
    QStringList shown;
    for (auto it = o.cbegin(); it != o.cend(); ++it)
        if (!it.key().contains("key"_L1, Qt::CaseInsensitive) && !it.key().contains("pass"_L1, Qt::CaseInsensitive))
            shown << it.key() + '='_L1 + it.value().toString();
    LOG_I("conn") << "connect" << type << shown.join(' '_L1);
    m_type = type;
    m_hub->awaitSceneOriginFix();
    if (type == QLatin1String(LinkManager::kMavlinkSerial))
        m_link->connectSerial(o.value(u"port"_s).toString(), o.value(u"baudRate"_s, 57600).toInt());
    else if (type == QLatin1String(LinkManager::kMavlinkUdp))
        m_link->connectUdp(o.value(u"host"_s, u"127.0.0.1"_s).toString(), o.value(u"port"_s, 14550).toInt());
    else if (type == QLatin1String(LinkManager::kMavlinkTcp))
        m_link->connectTcp(o.value(u"host"_s, u"127.0.0.1"_s).toString(), o.value(u"port"_s, 5760).toInt());
    else if (type == QLatin1String(LinkManager::kMavlinkLte))
        m_link->connectLte(o);
    else if (type == QLatin1String(LinkManager::kMspSerial))
        m_link->connectMspSerial(o.value(u"port"_s).toString(), o.value(u"baudRate"_s, 115200).toInt(),
                                 o.value(u"profile"_s, u"normal"_s).toString());
    else if (type == QLatin1String(LinkManager::kMspTcp))
        m_link->connectMspTcp(o.value(u"host"_s, u"127.0.0.1"_s).toString(), o.value(u"port"_s, 5760).toInt(),
                              o.value(u"profile"_s, u"normal"_s).toString());
    else {
        LOG_E("conn") << "unknown connection type" << type;
        emit connectFailed(type, u"Unknown connection type: %1"_s.arg(type));
    }
}

void ConnectionManager::disconnectLink()
{
    LOG_I("conn") << "disconnect" << m_type;
    m_link->disconnectLink();
}

void ConnectionManager::requestStreamsWhenLinked()
{
    m_linkedSince = wallMs();
    m_streamPoll.start();
}

void ConnectionManager::startHomePolling()
{
    m_homePoll.start();
}

void ConnectionManager::stopHomePolling() { m_homePoll.stop(); }

void ConnectionManager::onHeartbeat()
{
    const auto s = m_hub->snapshot();
    if (s->connectionType.startsWith("msp"_L1)) {
        stopHomePolling();
        return;
    }
    if (s->armed && !m_prevArmed) {
        m_hub->mutate([](VehicleState &st) { st.homeLat.reset(); st.homeLon.reset(); st.homeAlt.reset(); });
        QTimer::singleShot(1500, this, [this] {
            m_cmd->requestHomePosition();
            startHomePolling();
        });
    }
    m_prevArmed = s->armed;
    if (!m_homePoll.isActive()) startHomePolling();
}

} // namespace corv
