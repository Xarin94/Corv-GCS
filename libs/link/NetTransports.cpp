#include "link/NetTransports.h"

#include "core/DebugLog.h"

#include <QThread>

using namespace Qt::StringLiterals;

namespace corv {

UdpTransport::UdpTransport(QString host, quint16 port, QObject *parent)
    : Transport(parent), m_host(std::move(host)), m_port(port ? port : 14550)
{
}

UdpTransport::~UdpTransport() { close(); }

bool UdpTransport::open(QString *error)
{
    m_sock = new QUdpSocket(this);
    if (!m_sock->bind(QHostAddress::AnyIPv4, m_port, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint)) {
        if (error) *error = m_sock->errorString();
        delete m_sock;
        m_sock = nullptr;
        return false;
    }
    LOG_I("mavlink") << "UDP listening on port" << m_port;
    connect(m_sock, &QUdpSocket::readyRead, this, [this] {
        while (m_sock && m_sock->hasPendingDatagrams()) {
            QByteArray d;
            d.resize(int(m_sock->pendingDatagramSize()));
            QHostAddress from;
            quint16 fromPort = 0;
            m_sock->readDatagram(d.data(), d.size(), &from, &fromPort);
            if (!m_hasRemote) {
                m_remote = from;
                m_remotePort = fromPort;
                m_hasRemote = true;
                LOG_I("mavlink") << u"UDP first packet from %1:%2 (replies go there)"_s.arg(from.toString()).arg(fromPort);
            }
            emit received(d);
        }
    });
    return true;
}

void UdpTransport::close()
{
    if (!m_sock) return;
    m_sock->close();
    m_sock->deleteLater();
    m_sock = nullptr;
    emit closed(u"UDP socket closed"_s);
}

qint64 UdpTransport::write(const QByteArray &bytes)
{
    if (!m_sock || !m_hasRemote) return -1;
    return m_sock->writeDatagram(bytes, m_remote, m_remotePort);
}

TcpTransport::TcpTransport(QString host, quint16 port, QString label, QObject *parent)
    : Transport(parent), m_host(host.isEmpty() ? u"127.0.0.1"_s : std::move(host)),
      m_port(port ? port : 5760), m_label(std::move(label))
{
}

TcpTransport::~TcpTransport() { close(); }

bool TcpTransport::open(QString *error)
{
    for (int attempt = 1; attempt <= kAttempts; ++attempt) {
        m_sock = new QTcpSocket(this);
        m_sock->setSocketOption(QAbstractSocket::LowDelayOption, 1);
        m_sock->connectToHost(m_host, m_port);
        if (m_sock->waitForConnected(3000)) break;
        LOG_I("mavlink") << u"TCP attempt %1/%2 failed: %3"_s.arg(attempt).arg(kAttempts).arg(m_sock->errorString());
        if (attempt == kAttempts) {
            if (error) *error = m_sock->errorString();
            delete m_sock;
            m_sock = nullptr;
            return false;
        }
        delete m_sock;
        m_sock = nullptr;
        QThread::msleep(kRetryMs);
    }
    LOG_I("mavlink") << u"TCP connected to %1:%2"_s.arg(m_host).arg(m_port);
    connect(m_sock, &QTcpSocket::readyRead, this, [this] { emit received(m_sock->readAll()); });
    // The vehicle side closing the link must end the connection (fixes the
    // Electron version, where STATE.connected stayed true)
    connect(m_sock, &QTcpSocket::disconnected, this, [this] {
        if (!m_closing) emit closed(u"TCP closed"_s);
    });
    connect(m_sock, &QTcpSocket::errorOccurred, this, [this](QAbstractSocket::SocketError e) {
        if (m_closing || e == QAbstractSocket::RemoteHostClosedError) return;
        LOG_W("mavlink") << "TCP error:" << m_sock->errorString();
    });
    return true;
}

void TcpTransport::close()
{
    if (!m_sock) return;
    m_closing = true;
    m_sock->abort();
    m_sock->deleteLater();
    m_sock = nullptr;
}

qint64 TcpTransport::write(const QByteArray &bytes)
{
    return m_sock && m_sock->state() == QAbstractSocket::ConnectedState ? m_sock->write(bytes) : -1;
}

} // namespace corv
