// ConnectionManager.h - connection lifecycle (port of js/mavlink/ConnectionManager.js).
//
// GUI thread. connect(type, options) opens the link through LinkManager, moves
// the scene origin to the stream's first fix, asks for the data streams once
// the first heartbeat is in (or after 15 s on a one-way link) and polls
// HOME_POSITION every 5 s until it arrives, again after every arming.
#pragma once

#include <QObject>
#include <QTimer>
#include <QVariantMap>

namespace corv {

class CommandSender;
class LinkManager;
class MessageBus;
class VehicleHub;

class ConnectionManager : public QObject
{
    Q_OBJECT
public:
    ConnectionManager(LinkManager *link, VehicleHub *hub, MessageBus *bus, CommandSender *cmd, QObject *parent = nullptr);

    /// type: mavlink-serial (port, baudRate), mavlink-udp (host, port), mavlink-tcp (host, port), mavlink-lte (moduleId, keyHex, host, port, certSha256).
    Q_INVOKABLE void connectLink(const QString &type, const QVariantMap &options);
    Q_INVOKABLE void disconnectLink();
    Q_INVOKABLE bool isHeartbeatAlive() const;
    QString currentType() const { return m_type; }

signals:
    void vehicleConnected(const QString &type);
    void connectFailed(const QString &type, const QString &error);
    void disconnected();

private:
    void requestStreamsWhenLinked();
    void startHomePolling();
    void stopHomePolling();
    void onHeartbeat();

    LinkManager *m_link;
    VehicleHub *m_hub;
    MessageBus *m_bus;
    CommandSender *m_cmd;
    QString m_type;
    QTimer m_streamPoll;
    QTimer m_homePoll;
    qint64 m_linkedSince = 0;
    bool m_prevArmed = false;
};

} // namespace corv
