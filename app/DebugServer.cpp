#include "DebugServer.h"

#include "TestDriver.h"
#include "core/DebugLog.h"

#include <QJsonDocument>
#include <QJsonObject>
#include <QTcpSocket>

using namespace Qt::StringLiterals;

namespace corv {

namespace {
constexpr qint64 kMaxLine = 1 << 20;   // a request longer than this closes the connection
} // namespace

DebugServer::DebugServer(TestDriver &driver, QObject *parent) : QObject(parent), m_driver(driver)
{
    connect(&m_server, &QTcpServer::newConnection, this, &DebugServer::onConnection);
}

bool DebugServer::listen(quint16 port)
{
    // Loopback only: the server drives the whole application
    if (!m_server.listen(QHostAddress::LocalHost, port)) {
        LOG_E("app") << "debug server: cannot listen on 127.0.0.1:" << port << m_server.errorString();
        return false;
    }
    LOG_W("app") << "debug server listening on 127.0.0.1:" << m_server.serverPort() << "(--debug-port: tests only)";
    return true;
}

void DebugServer::onConnection()
{
    while (QTcpSocket *s = m_server.nextPendingConnection()) {
        connect(s, &QTcpSocket::disconnected, s, &QObject::deleteLater);
        connect(s, &QTcpSocket::readyRead, this, [this, s] {
            while (s->canReadLine()) {
                const QByteArray line = s->readLine().trimmed();
                if (line.isEmpty()) continue;
                s->write(handle(line) + '\n');
            }
            if (s->bytesAvailable() > kMaxLine) s->abort();
        });
    }
}

QByteArray DebugServer::handle(const QByteArray &line)
{
    QJsonParseError pe;
    const QJsonDocument doc = QJsonDocument::fromJson(line, &pe);
    QJsonObject resp{{u"jsonrpc"_s, u"2.0"_s}};
    if (pe.error != QJsonParseError::NoError || !doc.isObject()) {
        resp.insert(u"id"_s, QJsonValue());
        resp.insert(u"error"_s, QJsonObject{{u"code"_s, -32700}, {u"message"_s, u"parse error"_s}});
        return QJsonDocument(resp).toJson(QJsonDocument::Compact);
    }
    const QJsonObject req = doc.object();
    resp.insert(u"id"_s, req.value(u"id"_s));
    const QString method = req.value(u"method"_s).toString();
    QString error;
    const QJsonValue result = m_driver.runJson(method, req.value(u"params"_s), &error);
    if (!error.isEmpty()) {
        const int code = error.startsWith("unknown method"_L1) ? -32601 : -32000;
        resp.insert(u"error"_s, QJsonObject{{u"code"_s, code}, {u"message"_s, error}});
    } else {
        resp.insert(u"result"_s, result);
    }
    LOG_D("app") << "debug server:" << method << (error.isEmpty() ? u"ok"_s : error);
    return QJsonDocument(resp).toJson(QJsonDocument::Compact);
}

} // namespace corv
