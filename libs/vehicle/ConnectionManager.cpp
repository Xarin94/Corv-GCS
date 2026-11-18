#include "vehicle/ConnectionManager.h"

#include "core/Clock.h"
#include "core/DebugLog.h"
#include "link/LinkManager.h"
#include "link/MessageBus.h"
#include "vehicle/CommandSender.h"
#include "vehicle/VehicleHub.h"

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
        if (!type.startsWith(QLatin1String("msp"))) requestStreamsWhenLinked();
        emit vehicleConnected(type);
    });
    connect(m_link, &LinkManager::connectFailed, this, [this](const QString &type, const QString &error) {
        LOG_E("conn") << "connect" << type << "failed:" << error;
        m_type.clear();
        emit connectFailed(type, error);
    });
    connect(m_link, &LinkManager::connectionState, this, [this](const QString &st) {
        if (st != QLatin1String("DISCONNECTED")) return;
        m_streamPoll.stop();
        stopHomePolling();
        m_type.clear();
        emit disconnected();
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
        if (!it.key().contains(QLatin1String("key"), Qt::CaseInsensitive) && !it.key().contains(QLatin1String("pass"), Qt::CaseInsensitive))
            shown << it.key() + QLatin1Char('=') + it.value().toString();
    LOG_I("conn") << "connect" << type << shown.join(QLatin1Char(' '));
    m_type = type;
    m_hub->awaitSceneOriginFix();
    if (type == QLatin1String(LinkManager::kMavlinkSerial))
        m_link->connectSerial(o.value(QStringLiteral("port")).toString(), o.value(QStringLiteral("baudRate"), 57600).toInt());
    else if (type == QLatin1String(LinkManager::kMavlinkUdp))
        m_link->connectUdp(o.value(QStringLiteral("host"), QStringLiteral("127.0.0.1")).toString(), o.value(QStringLiteral("port"), 14550).toInt());
    else if (type == QLatin1String(LinkManager::kMavlinkTcp))
        m_link->connectTcp(o.value(QStringLiteral("host"), QStringLiteral("127.0.0.1")).toString(), o.value(QStringLiteral("port"), 5760).toInt());
    else if (type == QLatin1String(LinkManager::kMavlinkLte))
        m_link->connectLte(o);
    else if (type == QLatin1String(LinkManager::kMspSerial))
        m_link->connectMspSerial(o.value(QStringLiteral("port")).toString(), o.value(QStringLiteral("baudRate"), 115200).toInt(),
                                 o.value(QStringLiteral("profile"), QStringLiteral("normal")).toString());
    else if (type == QLatin1String(LinkManager::kMspTcp))
        m_link->connectMspTcp(o.value(QStringLiteral("host"), QStringLiteral("127.0.0.1")).toString(), o.value(QStringLiteral("port"), 5760).toInt(),
                              o.value(QStringLiteral("profile"), QStringLiteral("normal")).toString());
    else {
        LOG_E("conn") << "unknown connection type" << type;
        emit connectFailed(type, QStringLiteral("Unknown connection type: %1").arg(type));
    }
}

void ConnectionManager::disconnectLink()
{
    LOG_I("conn") << "disconnect" << m_type;
    m_link->disconnectLink();
}

bool ConnectionManager::isHeartbeatAlive() const
{
    const auto s = m_hub->snapshot();
    if (!s->connected) return false;
    if (s->connectionType.startsWith(QLatin1String("msp"))) return wallMs() - s->lastHeartbeatTime < 5000;
    return wallMs() - s->lastHeartbeatTime < 3000;
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
    if (s->connectionType.startsWith(QLatin1String("msp"))) {
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
