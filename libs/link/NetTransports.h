// NetTransports.h - UDP and TCP links.
#pragma once

#include "link/Transport.h"

#include <QHostAddress>
#include <QTcpSocket>
#include <QUdpSocket>

namespace corv {

/// Listens on a UDP port; replies go to whoever sent the first packet.
class UdpTransport : public Transport
{
    Q_OBJECT
public:
    UdpTransport(QString host, quint16 port, QObject *parent = nullptr);
    ~UdpTransport() override;

    bool open(QString *error) override;
    void close() override;
    qint64 write(const QByteArray &bytes) override;
    bool canSend() const override { return m_hasRemote; }
    QString describe() const override { return QStringLiteral("MAVLink UDP listen :%1").arg(m_port); }
    QString kind() const override { return QStringLiteral("UDP"); }

private:
    QUdpSocket *m_sock = nullptr;
    QString m_host;
    quint16 m_port;
    QHostAddress m_remote;
    quint16 m_remotePort = 0;
    bool m_hasRemote = false;
};

/// TCP client (SITL on 5760): six attempts two seconds apart, SITL may still be binding.
class TcpTransport : public Transport
{
    Q_OBJECT
public:
    TcpTransport(QString host, quint16 port, QString label = QStringLiteral("TCP"), QObject *parent = nullptr);
    ~TcpTransport() override;

    bool open(QString *error) override;
    void close() override;
    qint64 write(const QByteArray &bytes) override;
    QString describe() const override { return QStringLiteral("MAVLink TCP %1:%2").arg(m_host).arg(m_port); }
    QString kind() const override { return m_label; }

    static constexpr int kAttempts = 6;
    static constexpr int kRetryMs = 2000;

private:
    QTcpSocket *m_sock = nullptr;
    QString m_host;
    quint16 m_port;
    QString m_label;
    bool m_closing = false;
};

} // namespace corv
