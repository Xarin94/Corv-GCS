#include "ui/RtkController.h"

#include "core/Clock.h"
#include "core/DebugLog.h"
#include "core/Settings.h"
#include "link/LinkManager.h"
#include "link/Mavlink.h"
#include "ui/DialogHub.h"
#include "vehicle/VehicleHub.h"
#include "vehicle/VehicleState.h"

#include <QJsonDocument>
#include <QJsonObject>
#include <QRegularExpression>
#include <QSerialPort>
#include <QSslSocket>
#include <algorithm>
#include <cmath>
#include <cstring>

namespace corv {

namespace {

const QString kNtripKey = QStringLiteral("ntrip-settings");
constexpr qint64 kRateWindowMs = 5000;
constexpr int kMaxTypes = 12;
constexpr int kReconnectMs = 5000;
constexpr int kIdleTimeoutMs = 20000;       // casters stream continuously
constexpr int kSourcetableTimeoutMs = 10000;

QString formatBytes(qint64 bytes)
{
    if (bytes > 1048576) return QString::number(bytes / 1048576.0, 'f', 1) + QStringLiteral(" MB");
    if (bytes > 1024) return QString::number(bytes / 1024.0, 'f', 1) + QStringLiteral(" KB");
    return QString::number(bytes) + QStringLiteral(" B");
}

QByteArray basicAuth(const QString &user, const QString &pass)
{
    return "Authorization: Basic " + (user + QLatin1Char(':') + pass).toUtf8().toBase64() + "\r\n";
}

} // namespace

RtkController *RtkController::s_instance = nullptr;

RtkController::RtkController(LinkManager *link, VehicleHub *hub, QObject *parent) : QObject(parent), m_link(link), m_hub(hub)
{
    s_instance = this;
    m_reconnectTimer.setSingleShot(true);
    m_reconnectTimer.setInterval(kReconnectMs);
    connect(&m_reconnectTimer, &QTimer::timeout, this, [this] {
        if (!m_userDisconnect && m_ntripCfg) ntripOpen();
    });
    connect(&m_ggaTimer, &QTimer::timeout, this, &RtkController::sendGga);
    m_idleTimer.setSingleShot(true);
    m_idleTimer.setInterval(kIdleTimeoutMs);
    connect(&m_idleTimer, &QTimer::timeout, this, [this] {
        LOG_E("ntrip") << "socket timeout";
        if (m_ntrip) m_ntrip->abort();
    });
    m_droneTimer.setInterval(250);
    connect(&m_droneTimer, &QTimer::timeout, this, &RtkController::updateDrone);
    m_droneTimer.start();
}

RtkController::~RtkController()
{
    // cleanup(): stop NTRIP, close the base station
    stopNtrip(true);
    closeSerial();
}

// ── Statistics and injection ────────────────────────────────────────────────

void RtkController::resetStats(const QString &source)
{
    m_connected = true;
    m_source = source;
    m_bytes = 0;
    m_msgCount = 0;
    m_msgPerSec = 0;
    m_window.clear();
    m_types.clear();
    m_lastNotify = wallMs();
}

void RtkController::handleData(const QByteArray &data)
{
    m_bytes += data.size();
    for (const rtcm::Frame &f : m_parser.parse(data)) {
        ++m_msgCount;
        // Rate over the last 5 s
        const qint64 now = wallMs();
        m_window.push_back(now);
        while (!m_window.empty() && now - m_window.front() >= kRateWindowMs) m_window.pop_front();
        m_msgPerSec = int(std::lround(m_window.size() / 5.0));
        // Last message types, unique, newest first
        for (qsizetype i = 0; i < m_types.size(); ++i)
            if (m_types[i].toMap().value(QStringLiteral("id")).toInt() == f.type) {
                m_types.removeAt(i);
                break;
            }
        m_types.prepend(QVariantMap{{QStringLiteral("id"), f.type}, {QStringLiteral("name"), rtcm::typeName(f.type)}});
        if (m_types.size() > kMaxTypes) m_types.removeLast();
        inject(f.raw);
    }
    notify();
}

/// GPS_RTCM_DATA (233) to the vehicle, fragmented above 180 bytes; the link
/// sets system / component id, sequence and CRC.
void RtkController::inject(const QByteArray &frame)
{
    for (const auto &[flags, chunk] : rtcm::fragments(frame, m_msgCount)) {
        uint8_t data[180] = {};
        std::memcpy(data, chunk.constData(), std::size_t(std::min<qsizetype>(chunk.size(), 180)));
        mavlink_message_t msg;
        mavlink_msg_gps_rtcm_data_pack(255, 190, &msg, flags, uint8_t(chunk.size()), data);
        m_link->send(msg);
    }
}

/// The page refreshes at most 4 times a second while data flows, at once on events.
void RtkController::notify(bool force)
{
    const qint64 now = wallMs();
    if (!force && now - m_lastNotify <= 250) return;
    m_lastNotify = now;
    emit statusChanged();
}

void RtkController::setConnStatus(const QString &text, bool active)
{
    m_connStatus = text;
    m_connStatusActive = active;
    notify(true);
}

QString RtkController::streamStatus() const
{
    if (!m_connected) return QStringLiteral("No data");
    return m_msgPerSec > 0 ? QStringLiteral("Streaming RTCM3") : QStringLiteral("Connected, waiting for data...");
}

QString RtkController::activeSource() const
{
    if (!m_connected) return QStringLiteral("---");
    return m_source == QLatin1String("ntrip") ? QStringLiteral("NTRIP Caster") : QStringLiteral("Serial Base");
}

QString RtkController::bytesText() const { return formatBytes(m_bytes); }

QString RtkController::ggaSent() const
{
    return m_source == QLatin1String("ntrip") && m_ntripCfg ? QString::number(m_ggaSentCount) : QStringLiteral("---");
}

QString RtkController::reconnects() const
{
    return m_source == QLatin1String("ntrip") && m_ntripCfg ? QString::number(m_reconnects) : QStringLiteral("---");
}

// ── Serial base station ─────────────────────────────────────────────────────

void RtkController::connectSerial(const QString &port, int baud)
{
    if (port.isEmpty()) {
        DialogHub::toast(QStringLiteral("Select a serial port first"));
        return;
    }
    closeSerial();
    stopNtrip(true);   // a single correction source at a time
    baud = baud > 0 ? baud : 115200;
    auto *sp = new QSerialPort(port, this);
    sp->setBaudRate(baud);
    if (!sp->open(QIODevice::ReadWrite)) {
        const QString err = sp->errorString();
        delete sp;
        LOG_E("rtk") << "connect failed:" << err;
        DialogHub::toast(QStringLiteral("RTK connect failed: ") + err);
        return;
    }
    m_serial = sp;
    m_portPath = port;
    m_baud = baud;
    resetStats(QStringLiteral("serial"));
    connect(sp, &QSerialPort::readyRead, this, [this, sp] { handleData(sp->readAll()); });
    connect(sp, &QSerialPort::errorOccurred, this, [this, sp](QSerialPort::SerialPortError e) {
        if (e == QSerialPort::NoError) return;
        LOG_E("rtk") << "serial error:" << sp->errorString();
        if (e == QSerialPort::ResourceError) {
            // the port went away: 'close'
            LOG_I("rtk") << "port closed";
            m_connected = false;
            setConnStatus(QStringLiteral("Not connected"), false);
        } else {
            notify(true);
        }
    });
    LOG_I("rtk") << QStringLiteral("connected to %1 at %2 baud").arg(port).arg(baud);
    setConnStatus(QStringLiteral("Serial — %1").arg(port), true);
}

void RtkController::closeSerial()
{
    if (!m_serial) return;
    m_serial->disconnect(this);
    m_serial->close();
    m_serial->deleteLater();
    m_serial = nullptr;
    if (m_source == QLatin1String("serial")) {
        m_connected = false;
        LOG_I("rtk") << "disconnected";
        setConnStatus(QStringLiteral("Not connected"), false);
    }
}

void RtkController::disconnectSerial()
{
    closeSerial();
    setConnStatus(QStringLiteral("Disconnected"), false);
}

// ── NTRIP client ────────────────────────────────────────────────────────────
// Mission Planner-style: HTTP GET with Basic auth against a caster, accepts both
// "ICY 200 OK" (v1) and "HTTP/1.x 200" (v2, optionally chunked), uploads GGA
// for VRS networks, reconnects by itself.

void RtkController::ntripConnect(const QVariantMap &cfg, const QString &source)
{
    NtripConfig c;
    c.host = cfg.value(QStringLiteral("host")).toString().trimmed();
    c.port = cfg.value(QStringLiteral("port")).toInt();
    if (c.port <= 0) c.port = 2101;
    c.mountpoint = cfg.value(QStringLiteral("mountpoint")).toString().trimmed();
    c.mountpoint.remove(QRegularExpression(QStringLiteral("^/")));
    c.username = cfg.value(QStringLiteral("username")).toString();
    c.password = cfg.value(QStringLiteral("password")).toString();
    c.tls = cfg.value(QStringLiteral("tls")).toBool();
    c.ggaMode = cfg.value(QStringLiteral("ggaMode"), QStringLiteral("off")).toString();
    bool ok = false;
    c.lat = cfg.value(QStringLiteral("lat")).toDouble(&ok);
    if (!ok) c.lat = qQNaN();
    c.lon = cfg.value(QStringLiteral("lon")).toDouble(&ok);
    if (!ok) c.lon = qQNaN();
    c.alt = cfg.value(QStringLiteral("alt")).toDouble();
    c.ggaInterval = std::max(1, cfg.value(QStringLiteral("ggaInterval")).toInt());
    if (!cfg.contains(QStringLiteral("ggaInterval"))) c.ggaInterval = 10;
    if (c.host.isEmpty() || c.mountpoint.isEmpty()) {
        DialogHub::toast(QStringLiteral("Caster host and mountpoint are required"));
        return;
    }
    closeSerial();   // a single correction source at a time
    stopNtrip(true);
    m_ntripCfg = c;
    m_userDisconnect = false;
    m_reconnects = 0;
    m_ggaSentCount = 0;
    m_firstAttempt = true;
    m_connecting = true;
    saveNtrip(c, source);   // Electron saves after a successful connect; the source choice goes with it
    setConnStatus(QStringLiteral("Connecting to caster..."), false);
    ntripOpen();
}

void RtkController::ntripOpen()
{
    if (!m_ntripCfg) return;
    const NtripConfig &c = *m_ntripCfg;
    m_headerParsed = false;
    m_headerBuf.clear();
    m_chunked = false;
    m_dechunk = rtcm::Dechunker();
    m_parser = rtcm::Parser();

    auto *sock = new QSslSocket(this);
    m_ntrip = sock;
    sock->setPeerVerifyMode(QSslSocket::VerifyNone);   // rejectUnauthorized: false
    const auto onConnect = [this, sock] {
        const NtripConfig &cc = *m_ntripCfg;
        QByteArray req = "GET /" + cc.mountpoint.toUtf8() + " HTTP/1.1\r\n" + "Host: " + cc.host.toUtf8() + ':' +
                         QByteArray::number(cc.port) + "\r\n" + "User-Agent: NTRIP CORV-GCS\r\n" + "Accept: */*\r\n";
        if (!cc.username.isEmpty() || !cc.password.isEmpty()) req += basicAuth(cc.username, cc.password);
        req += "Connection: close\r\n\r\n";
        sock->write(req);
        m_idleTimer.start();
    };
    if (c.tls) connect(sock, &QSslSocket::encrypted, this, onConnect);
    else connect(sock, &QSslSocket::connected, this, onConnect);
    connect(sock, &QSslSocket::sslErrors, sock, [sock](const QList<QSslError> &) { sock->ignoreSslErrors(); });
    connect(sock, &QSslSocket::readyRead, this, &RtkController::ntripOnData);
    connect(sock, &QSslSocket::errorOccurred, this, [this, sock](QAbstractSocket::SocketError e) {
        if (e == QAbstractSocket::RemoteHostClosedError && m_headerParsed) return;   // the close handles it
        LOG_E("ntrip") << "socket error:" << sock->errorString();
        if (m_firstAttempt) ntripFail(sock->errorString(), false);
    });
    connect(sock, &QSslSocket::disconnected, this, &RtkController::ntripOnClosed);
    connect(sock, &QSslSocket::errorOccurred, this, [this, sock] {
        // a connect that never got through emits no disconnected()
        if (sock->state() == QAbstractSocket::UnconnectedState) QTimer::singleShot(0, this, [this, sock] {
            if (m_ntrip == sock) ntripOnClosed();
        });
    });
    if (c.tls) sock->connectToHostEncrypted(c.host, quint16(c.port));
    else sock->connectToHost(c.host, quint16(c.port));
}

void RtkController::ntripFail(const QString &message, bool fatal)
{
    if (fatal) m_userDisconnect = true;   // auth / config error: do not retry
    if (m_firstAttempt) {
        m_firstAttempt = false;
        m_connecting = false;
        setConnStatus(QStringLiteral("Error: ") + message, false);
        DialogHub::toast(QStringLiteral("NTRIP connect failed: ") + message);
    }
}

void RtkController::ntripOnData()
{
    QSslSocket *sock = m_ntrip;
    if (!sock) return;
    const QByteArray data = sock->readAll();
    m_idleTimer.start();
    if (m_headerParsed) {
        const QByteArray payload = m_chunked ? m_dechunk.feed(data) : data;
        if (!payload.isEmpty()) handleData(payload);
        return;
    }
    m_headerBuf.append(data);
    const qsizetype idx = m_headerBuf.indexOf("\r\n\r\n");
    if (idx < 0) {
        if (m_headerBuf.size() > 16384) sock->abort();
        return;
    }
    const QString header = QString::fromLatin1(m_headerBuf.left(idx));
    const QByteArray body = m_headerBuf.mid(idx + 4);
    m_headerBuf.clear();
    const QString statusLine = header.section(QStringLiteral("\r\n"), 0, 0).trimmed();
    const auto rx = [](const char *p) { return QRegularExpression(QLatin1String(p), QRegularExpression::CaseInsensitiveOption); };

    if (rx("^SOURCETABLE").match(statusLine).hasMatch() || rx("Content-Type:\\s*gnss/sourcetable").match(header).hasMatch()) {
        ntripFail(QStringLiteral("Mountpoint \"%1\" not found (caster returned sourcetable)").arg(m_ntripCfg->mountpoint), true);
        sock->abort();
        return;
    }
    if (!rx("^(ICY 200|HTTP/\\d\\.\\d 200)").match(statusLine).hasMatch()) {
        ntripFail(rx("401|Unauthorized").match(statusLine).hasMatch() ? QStringLiteral("NTRIP authentication failed (check username/password)")
                                                                       : QStringLiteral("NTRIP caster error: ") + statusLine,
                  true);
        sock->abort();
        return;
    }
    m_chunked = rx("transfer-encoding:\\s*chunked").match(header).hasMatch();
    m_headerParsed = true;
    m_firstAttempt = false;
    m_connecting = false;
    resetStats(QStringLiteral("ntrip"));
    const NtripConfig &c = *m_ntripCfg;
    LOG_I("ntrip") << QStringLiteral("connected to %1:%2/%3%4").arg(c.host).arg(c.port).arg(c.mountpoint, m_chunked ? QStringLiteral(" (chunked)") : QString());
    // GGA upload
    m_ggaTimer.stop();
    if (c.ggaMode != QLatin1String("off")) {
        sendGga();
        m_ggaTimer.start(c.ggaInterval * 1000);
    }
    setConnStatus(QStringLiteral("NTRIP — %1:%2/%3").arg(c.host).arg(c.port).arg(c.mountpoint), true);
    if (!body.isEmpty()) {
        const QByteArray payload = m_chunked ? m_dechunk.feed(body) : body;
        if (!payload.isEmpty()) handleData(payload);
    }
}

void RtkController::ntripOnClosed()
{
    QSslSocket *sock = m_ntrip;
    if (!sock) return;
    m_ntrip = nullptr;
    sock->disconnect(this);
    sock->deleteLater();
    m_headerParsed = false;
    m_ggaTimer.stop();
    m_idleTimer.stop();
    if (m_source == QLatin1String("ntrip") && m_connected) {
        m_connected = false;
        setConnStatus(QStringLiteral("Not connected"), false);
    }
    if (m_firstAttempt) ntripFail(QStringLiteral("NTRIP connection closed"), false);
    // Reconnect unless the user disconnected or the configuration is invalid
    if (!m_userDisconnect && m_ntripCfg) {
        ++m_reconnects;
        LOG_I("ntrip") << QStringLiteral("connection lost — reconnecting in 5 s (attempt %1)").arg(m_reconnects);
        m_reconnectTimer.start();
    }
}

void RtkController::stopNtrip(bool userRequested)
{
    if (userRequested) m_userDisconnect = true;
    m_reconnectTimer.stop();
    m_ggaTimer.stop();
    m_idleTimer.stop();
    m_headerParsed = false;
    if (m_ntrip) {
        QSslSocket *sock = m_ntrip;
        m_ntrip = nullptr;
        sock->disconnect(this);
        sock->abort();
        sock->deleteLater();
    }
    if (userRequested) m_ntripCfg.reset();
    m_connecting = false;
    m_firstAttempt = false;
    if (m_source == QLatin1String("ntrip") && m_connected) {
        m_connected = false;
        setConnStatus(QStringLiteral("Not connected"), false);
    }
}

void RtkController::ntripDisconnect()
{
    stopNtrip(true);
    setConnStatus(QStringLiteral("Disconnected"), false);
}

// ── GGA upload (VRS networks) ───────────────────────────────────────────────

void RtkController::sendGga()
{
    if (!m_ntripCfg || !m_ntrip || !m_headerParsed) return;
    const NtripConfig &c = *m_ntripCfg;
    double lat = c.lat, lon = c.lon, alt = c.alt;
    if (c.ggaMode != QLatin1String("manual")) {
        // The vehicle position (fed at 1 Hz in Electron while NTRIP is up)
        const auto s = m_hub->snapshot();
        lat = s->lat;
        lon = s->lon;
        alt = s->rawAlt;
    }
    if (!std::isfinite(lat) || !std::isfinite(lon)) return;
    if (lat == 0 && lon == 0) return;
    m_ntrip->write(rtcm::buildGga(lat, lon, std::isfinite(alt) ? alt : 0));
    ++m_ggaSentCount;
}

// ── Sourcetable ─────────────────────────────────────────────────────────────

void RtkController::fetchSourcetable(const QVariantMap &cfg)
{
    const QString host = cfg.value(QStringLiteral("host")).toString().trimmed();
    int port = cfg.value(QStringLiteral("port")).toInt();
    if (port <= 0) port = 2101;
    if (host.isEmpty()) {
        DialogHub::toast(QStringLiteral("Enter the caster host first"));
        return;
    }
    if (m_table) m_table->abort();
    m_listing = true;
    emit mountsChanged();
    m_tableBuf.clear();
    m_tableDone = false;
    auto *sock = new QSslSocket(this);
    m_table = sock;
    sock->setPeerVerifyMode(QSslSocket::VerifyNone);
    const QString user = cfg.value(QStringLiteral("username")).toString(), pass = cfg.value(QStringLiteral("password")).toString();
    const bool tls = cfg.value(QStringLiteral("tls")).toBool();
    const auto onConnect = [sock, host, port, user, pass] {
        QByteArray req = "GET / HTTP/1.1\r\nHost: " + host.toUtf8() + ':' + QByteArray::number(port) +
                         "\r\nUser-Agent: NTRIP CORV-GCS\r\nAccept: */*\r\n";
        if (!user.isEmpty() || !pass.isEmpty()) req += basicAuth(user, pass);
        req += "Connection: close\r\n\r\n";
        sock->write(req);
    };
    if (tls) connect(sock, &QSslSocket::encrypted, sock, onConnect);
    else connect(sock, &QSslSocket::connected, sock, onConnect);
    connect(sock, &QSslSocket::sslErrors, sock, [sock](const QList<QSslError> &) { sock->ignoreSslErrors(); });

    const auto finish = [this, sock](const QString &error) {
        if (m_table != sock || m_tableDone) return;
        m_tableDone = true;
        m_listing = false;
        sock->disconnect(this);
        sock->abort();
        sock->deleteLater();
        QList<rtcm::Mount> list;
        QString err = error;
        if (err.isEmpty() && !rtcm::parseSourcetable(m_tableBuf, &list, &err)) {}
        if (!err.isEmpty()) {
            emit mountsChanged();
            DialogHub::toast(QStringLiteral("Sourcetable fetch failed: ") + err);
            return;
        }
        m_mounts.clear();
        for (const rtcm::Mount &m : list) {
            QStringList details;
            for (const QString &d : {m.format, m.navSystem, m.country})
                if (!d.isEmpty()) details << d;
            m_mounts << QVariantMap{{QStringLiteral("value"), m.mountpoint},
                                    {QStringLiteral("label"), m.mountpoint + (details.isEmpty() ? QString() : QStringLiteral(" (%1)").arg(details.join(QStringLiteral(" · "))))}};
        }
        emit mountsChanged();
        if (list.isEmpty()) DialogHub::toast(QStringLiteral("Caster returned an empty sourcetable"));
    };
    connect(sock, &QSslSocket::readyRead, this, [this, sock, finish] {
        m_tableBuf.append(sock->readAll());
        if (m_tableBuf.size() > 2 * 1024 * 1024 || m_tableBuf.contains("ENDSOURCETABLE")) finish(QString());
    });
    connect(sock, &QSslSocket::disconnected, this, [finish] { finish(QString()); });
    connect(sock, &QSslSocket::errorOccurred, this, [sock, finish](QAbstractSocket::SocketError e) {
        if (e == QAbstractSocket::RemoteHostClosedError) finish(QString());
        else finish(sock->errorString());
    });
    QTimer::singleShot(kSourcetableTimeoutMs, sock, [finish] { finish(QString()); });
    if (tls) sock->connectToHostEncrypted(host, quint16(port));
    else sock->connectToHost(host, quint16(port));
}

// ── Saved form ──────────────────────────────────────────────────────────────

QVariantMap RtkController::saved() const
{
    QJsonObject o = Settings::instance().json(kNtripKey).toObject();
    if (o.isEmpty()) {
        const QString raw = Settings::instance().value(kNtripKey).toString();
        if (!raw.isEmpty()) o = QJsonDocument::fromJson(raw.toUtf8()).object();
    }
    return o.toVariantMap();
}

void RtkController::saveNtrip(const NtripConfig &c, const QString &source) const
{
    QJsonObject o;
    o.insert(QStringLiteral("host"), c.host);
    o.insert(QStringLiteral("port"), c.port);
    o.insert(QStringLiteral("mountpoint"), c.mountpoint);
    o.insert(QStringLiteral("username"), c.username);
    o.insert(QStringLiteral("tls"), c.tls);
    o.insert(QStringLiteral("ggaMode"), c.ggaMode);
    o.insert(QStringLiteral("ggaInterval"), c.ggaInterval);
    o.insert(QStringLiteral("lat"), std::isfinite(c.lat) ? QJsonValue(c.lat) : QJsonValue());
    o.insert(QStringLiteral("lon"), std::isfinite(c.lon) ? QJsonValue(c.lon) : QJsonValue());
    o.insert(QStringLiteral("alt"), c.alt);
    o.insert(QStringLiteral("source"), source.isEmpty() ? QStringLiteral("ntrip") : source);
    Settings::instance().setJson(kNtripKey, o);
}

void RtkController::setSourceChoice(const QString &source)
{
    QVariantMap s = saved();
    s.insert(QStringLiteral("source"), source);
    Settings::instance().setJson(kNtripKey, QJsonObject::fromVariantMap(s));
}

// ── Vehicle RTK state (4 Hz) ────────────────────────────────────────────────

void RtkController::updateDrone()
{
    const auto s = m_hub->snapshot();
    static const char *names[] = {"No GPS", "No Fix", "2D Fix", "3D Fix", "DGPS", "RTK Float", "RTK Fixed"};
    const QString fix = s->gpsFix >= 0 && s->gpsFix < 7 ? QString::fromLatin1(names[s->gpsFix]) : QStringLiteral("Fix %1").arg(s->gpsFix);
    const QString color = s->gpsFix == 6 ? QStringLiteral("#00ff7f") : s->gpsFix == 5 ? QStringLiteral("#ffcc00")
                        : s->gpsFix >= 3 ? QStringLiteral("#00d2ff") : QStringLiteral("#ff3333");
    const QString sat = s->gpsNumSat ? QString::number(s->gpsNumSat) : QStringLiteral("---");
    const QString hdop = s->gpsHdop < 99 ? QString::number(s->gpsHdop, 'f', 1) : QStringLiteral("---");
    const QString baseline = s->rtkBaseline > 0 ? QString::number(s->rtkBaseline / 1000, 'f', 3) + QStringLiteral(" m") : QStringLiteral("---");
    const QString accuracy = s->rtkAccuracy > 0 ? QString::number(s->rtkAccuracy / 10, 'f', 1) + QStringLiteral(" mm") : QStringLiteral("---");
    const QString iar = s->rtkIar > 0 ? QString::number(s->rtkIar) : QStringLiteral("---");
    if (fix == m_droneFix && color == m_droneFixColor && sat == m_droneSat && hdop == m_droneHdop && baseline == m_droneBaseline &&
        accuracy == m_droneAccuracy && iar == m_droneIar)
        return;
    m_droneFix = fix;
    m_droneFixColor = color;
    m_droneSat = sat;
    m_droneHdop = hdop;
    m_droneBaseline = baseline;
    m_droneAccuracy = accuracy;
    m_droneIar = iar;
    emit droneChanged();
}

} // namespace corv
