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

using namespace Qt::StringLiterals;

namespace corv {

namespace {

const QString kNtripKey = u"ntrip-settings"_s;
constexpr qint64 kRateWindowMs = 5000;
constexpr int kMaxTypes = 12;
constexpr int kReconnectMs = 5000;
constexpr int kIdleTimeoutMs = 20000;       // casters stream continuously
constexpr int kSourcetableTimeoutMs = 10000;

QString formatBytes(qint64 bytes)
{
    if (bytes > 1048576) return QString::number(bytes / 1048576.0, 'f', 1) + u" MB"_s;
    if (bytes > 1024) return QString::number(bytes / 1024.0, 'f', 1) + u" KB"_s;
    return QString::number(bytes) + u" B"_s;
}

QByteArray basicAuth(const QString &user, const QString &pass)
{
    return "Authorization: Basic " + (user + ':'_L1 + pass).toUtf8().toBase64() + "\r\n";
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
            if (m_types[i].toMap().value(u"id"_s).toInt() == f.type) {
                m_types.removeAt(i);
                break;
            }
        m_types.prepend(QVariantMap{{u"id"_s, f.type}, {u"name"_s, rtcm::typeName(f.type)}});
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
    if (!m_connected) return u"No data"_s;
    return m_msgPerSec > 0 ? u"Streaming RTCM3"_s : u"Connected, waiting for data..."_s;
}

QString RtkController::activeSource() const
{
    if (!m_connected) return u"---"_s;
    return m_source == "ntrip"_L1 ? u"NTRIP Caster"_s : u"Serial Base"_s;
}

QString RtkController::bytesText() const { return formatBytes(m_bytes); }

QString RtkController::ggaSent() const
{
    return m_source == "ntrip"_L1 && m_ntripCfg ? QString::number(m_ggaSentCount) : u"---"_s;
}

QString RtkController::reconnects() const
{
    return m_source == "ntrip"_L1 && m_ntripCfg ? QString::number(m_reconnects) : u"---"_s;
}

// ── Serial base station ─────────────────────────────────────────────────────

void RtkController::connectSerial(const QString &port, int baud)
{
    if (port.isEmpty()) {
        DialogHub::toast(u"Select a serial port first"_s);
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
        DialogHub::toast(u"RTK connect failed: "_s + err);
        return;
    }
    m_serial = sp;
    m_portPath = port;
    m_baud = baud;
    resetStats(u"serial"_s);
    connect(sp, &QSerialPort::readyRead, this, [this, sp] { handleData(sp->readAll()); });
    connect(sp, &QSerialPort::errorOccurred, this, [this, sp](QSerialPort::SerialPortError e) {
        if (e == QSerialPort::NoError) return;
        LOG_E("rtk") << "serial error:" << sp->errorString();
        if (e == QSerialPort::ResourceError) {
            // the port went away: 'close'
            LOG_I("rtk") << "port closed";
            m_connected = false;
            setConnStatus(u"Not connected"_s, false);
        } else {
            notify(true);
        }
    });
    LOG_I("rtk") << u"connected to %1 at %2 baud"_s.arg(port).arg(baud);
    setConnStatus(u"Serial — %1"_s.arg(port), true);
}

void RtkController::closeSerial()
{
    if (!m_serial) return;
    m_serial->disconnect(this);
    m_serial->close();
    m_serial->deleteLater();
    m_serial = nullptr;
    if (m_source == "serial"_L1) {
        m_connected = false;
        LOG_I("rtk") << "disconnected";
        setConnStatus(u"Not connected"_s, false);
    }
}

void RtkController::disconnectSerial()
{
    closeSerial();
    setConnStatus(u"Disconnected"_s, false);
}

// ── NTRIP client ────────────────────────────────────────────────────────────
// Mission Planner-style: HTTP GET with Basic auth against a caster, accepts both
// "ICY 200 OK" (v1) and "HTTP/1.x 200" (v2, optionally chunked), uploads GGA
// for VRS networks, reconnects by itself.

void RtkController::ntripConnect(const QVariantMap &cfg, const QString &source)
{
    NtripConfig c;
    c.host = cfg.value(u"host"_s).toString().trimmed();
    c.port = cfg.value(u"port"_s).toInt();
    if (c.port <= 0) c.port = 2101;
    c.mountpoint = cfg.value(u"mountpoint"_s).toString().trimmed();
    c.mountpoint.remove(QRegularExpression(u"^/"_s));
    c.username = cfg.value(u"username"_s).toString();
    c.password = cfg.value(u"password"_s).toString();
    c.tls = cfg.value(u"tls"_s).toBool();
    c.ggaMode = cfg.value(u"ggaMode"_s, u"off"_s).toString();
    bool ok = false;
    c.lat = cfg.value(u"lat"_s).toDouble(&ok);
    if (!ok) c.lat = qQNaN();
    c.lon = cfg.value(u"lon"_s).toDouble(&ok);
    if (!ok) c.lon = qQNaN();
    c.alt = cfg.value(u"alt"_s).toDouble();
    c.ggaInterval = std::max(1, cfg.value(u"ggaInterval"_s).toInt());
    if (!cfg.contains(u"ggaInterval"_s)) c.ggaInterval = 10;
    if (c.host.isEmpty() || c.mountpoint.isEmpty()) {
        DialogHub::toast(u"Caster host and mountpoint are required"_s);
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
    setConnStatus(u"Connecting to caster..."_s, false);
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
        setConnStatus(u"Error: "_s + message, false);
        DialogHub::toast(u"NTRIP connect failed: "_s + message);
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
    const QString statusLine = header.section(u"\r\n"_s, 0, 0).trimmed();
    const auto rx = [](const char *p) { return QRegularExpression(QLatin1String(p), QRegularExpression::CaseInsensitiveOption); };

    if (rx("^SOURCETABLE").match(statusLine).hasMatch() || rx("Content-Type:\\s*gnss/sourcetable").match(header).hasMatch()) {
        ntripFail(u"Mountpoint \"%1\" not found (caster returned sourcetable)"_s.arg(m_ntripCfg->mountpoint), true);
        sock->abort();
        return;
    }
    if (!rx("^(ICY 200|HTTP/\\d\\.\\d 200)").match(statusLine).hasMatch()) {
        ntripFail(rx("401|Unauthorized").match(statusLine).hasMatch() ? u"NTRIP authentication failed (check username/password)"_s
                                                                       : u"NTRIP caster error: "_s + statusLine,
                  true);
        sock->abort();
        return;
    }
    m_chunked = rx("transfer-encoding:\\s*chunked").match(header).hasMatch();
    m_headerParsed = true;
    m_firstAttempt = false;
    m_connecting = false;
    resetStats(u"ntrip"_s);
    const NtripConfig &c = *m_ntripCfg;
    LOG_I("ntrip") << u"connected to %1:%2/%3%4"_s.arg(c.host).arg(c.port).arg(c.mountpoint, m_chunked ? u" (chunked)"_s : QString());
    // GGA upload
    m_ggaTimer.stop();
    if (c.ggaMode != "off"_L1) {
        sendGga();
        m_ggaTimer.start(c.ggaInterval * 1000);
    }
    setConnStatus(u"NTRIP — %1:%2/%3"_s.arg(c.host).arg(c.port).arg(c.mountpoint), true);
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
    if (m_source == "ntrip"_L1 && m_connected) {
        m_connected = false;
        setConnStatus(u"Not connected"_s, false);
    }
    if (m_firstAttempt) ntripFail(u"NTRIP connection closed"_s, false);
    // Reconnect unless the user disconnected or the configuration is invalid
    if (!m_userDisconnect && m_ntripCfg) {
        ++m_reconnects;
        LOG_I("ntrip") << u"connection lost — reconnecting in 5 s (attempt %1)"_s.arg(m_reconnects);
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
    if (m_source == "ntrip"_L1 && m_connected) {
        m_connected = false;
        setConnStatus(u"Not connected"_s, false);
    }
}

void RtkController::ntripDisconnect()
{
    stopNtrip(true);
    setConnStatus(u"Disconnected"_s, false);
}

// ── GGA upload (VRS networks) ───────────────────────────────────────────────

void RtkController::sendGga()
{
    if (!m_ntripCfg || !m_ntrip || !m_headerParsed) return;
    const NtripConfig &c = *m_ntripCfg;
    double lat = c.lat, lon = c.lon, alt = c.alt;
    if (c.ggaMode != "manual"_L1) {
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
    const QString host = cfg.value(u"host"_s).toString().trimmed();
    int port = cfg.value(u"port"_s).toInt();
    if (port <= 0) port = 2101;
    if (host.isEmpty()) {
        DialogHub::toast(u"Enter the caster host first"_s);
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
    const QString user = cfg.value(u"username"_s).toString(), pass = cfg.value(u"password"_s).toString();
    const bool tls = cfg.value(u"tls"_s).toBool();
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
            DialogHub::toast(u"Sourcetable fetch failed: "_s + err);
            return;
        }
        m_mounts.clear();
        for (const rtcm::Mount &m : list) {
            QStringList details;
            for (const QString &d : {m.format, m.navSystem, m.country})
                if (!d.isEmpty()) details << d;
            m_mounts << QVariantMap{{u"value"_s, m.mountpoint},
                                    {u"label"_s, m.mountpoint + (details.isEmpty() ? QString() : u" (%1)"_s.arg(details.join(u" · "_s)))}};
        }
        emit mountsChanged();
        if (list.isEmpty()) DialogHub::toast(u"Caster returned an empty sourcetable"_s);
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
    o.insert(u"host"_s, c.host);
    o.insert(u"port"_s, c.port);
    o.insert(u"mountpoint"_s, c.mountpoint);
    o.insert(u"username"_s, c.username);
    o.insert(u"tls"_s, c.tls);
    o.insert(u"ggaMode"_s, c.ggaMode);
    o.insert(u"ggaInterval"_s, c.ggaInterval);
    o.insert(u"lat"_s, std::isfinite(c.lat) ? QJsonValue(c.lat) : QJsonValue());
    o.insert(u"lon"_s, std::isfinite(c.lon) ? QJsonValue(c.lon) : QJsonValue());
    o.insert(u"alt"_s, c.alt);
    o.insert(u"source"_s, source.isEmpty() ? u"ntrip"_s : source);
    Settings::instance().setJson(kNtripKey, o);
}

void RtkController::setSourceChoice(const QString &source)
{
    QVariantMap s = saved();
    s.insert(u"source"_s, source);
    Settings::instance().setJson(kNtripKey, QJsonObject::fromVariantMap(s));
}

// ── Vehicle RTK state (4 Hz) ────────────────────────────────────────────────

void RtkController::updateDrone()
{
    const auto s = m_hub->snapshot();
    static const char *names[] = {"No GPS", "No Fix", "2D Fix", "3D Fix", "DGPS", "RTK Float", "RTK Fixed"};
    const QString fix = s->gpsFix >= 0 && s->gpsFix < 7 ? QString::fromLatin1(names[s->gpsFix]) : u"Fix %1"_s.arg(s->gpsFix);
    const QString color = s->gpsFix == 6 ? u"#00ff7f"_s : s->gpsFix == 5 ? u"#ffcc00"_s
                        : s->gpsFix >= 3 ? u"#00d2ff"_s : u"#ff3333"_s;
    const QString sat = s->gpsNumSat ? QString::number(s->gpsNumSat) : u"---"_s;
    const QString hdop = s->gpsHdop < 99 ? QString::number(s->gpsHdop, 'f', 1) : u"---"_s;
    const QString baseline = s->rtkBaseline > 0 ? QString::number(s->rtkBaseline / 1000, 'f', 3) + u" m"_s : u"---"_s;
    const QString accuracy = s->rtkAccuracy > 0 ? QString::number(s->rtkAccuracy / 10, 'f', 1) + u" mm"_s : u"---"_s;
    const QString iar = s->rtkIar > 0 ? QString::number(s->rtkIar) : u"---"_s;
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
