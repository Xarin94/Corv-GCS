#include "ui/TelForwardController.h"

#include "core/Clock.h"
#include "core/DebugLog.h"
#include "core/Settings.h"
#include "link/LinkManager.h"
#include "services/Ltm.h"
#include "vehicle/VehicleHub.h"
#include "vehicle/VehicleState.h"

#include <QJsonDocument>
#include <QHostInfo>
#include <QJsonObject>
#include <QSerialPort>
#include <QUdpSocket>
#include <cmath>

using namespace Qt::StringLiterals;

namespace corv {

namespace {

const QString kStorageKey = u"telfwd-settings"_s;
constexpr qint64 kPeerTtlMs = 60000;   // UDP-server peers are forgotten after this much silence
constexpr qint64 kRateWindowMs = 5000;

QString formatBytes(qint64 n)
{
    if (n >= 1048576) return QString::number(n / 1048576.0, 'f', 1) + u" MB"_s;
    if (n >= 1024) return QString::number(n / 1024.0, 'f', 1) + u" KB"_s;
    return QString::number(n) + u" B"_s;
}

} // namespace

TelForwardController *TelForwardController::s_instance = nullptr;

TelForwardController::TelForwardController(LinkManager *link, VehicleHub *hub, QObject *parent) : QObject(parent), m_link(link), m_hub(hub)
{
    s_instance = this;
    // Every received MAVLink frame fans out to the MAVLink outputs
    connect(m_link, &LinkManager::rawFrame, this, &TelForwardController::onRawFrame, Qt::QueuedConnection);
    m_status.setInterval(1000);
    connect(&m_status, &QTimer::timeout, this, [this] {
        const qint64 now = wallMs();
        for (auto &[id, o] : m_outputs) {
            while (!o->window.empty() && now - o->window.front() >= kRateWindowMs) o->window.pop_front();
            o->msgPerSec = int(std::lround(o->window.size() / 5.0));
        }
        emit outputsChanged();
    });
    m_ltmFast.setInterval(200);    // G + A at 5 Hz
    m_ltmSlow.setInterval(500);    // S at 2 Hz
    m_ltmOrigin.setInterval(2000); // O at 0.5 Hz
    connect(&m_ltmFast, &QTimer::timeout, this, [this] { ltmTick(0); });
    connect(&m_ltmSlow, &QTimer::timeout, this, [this] { ltmTick(1); });
    connect(&m_ltmOrigin, &QTimer::timeout, this, [this] { ltmTick(2); });
    m_currentTimer.setInterval(250);
    connect(&m_currentTimer, &QTimer::timeout, this, &TelForwardController::updateCurrent);
    m_currentTimer.start();
}

TelForwardController::~TelForwardController()
{
    for (auto &[id, o] : m_outputs) closeOutput(*o);
}

// ── Output lifecycle ────────────────────────────────────────────────────────

void TelForwardController::addOutput(const QVariantMap &cfg)
{
    auto o = std::make_unique<Output>();
    o->type = cfg.value(u"type"_s).toString();
    o->protocol = cfg.value(u"protocol"_s).toString() == "ltm"_L1 ? u"ltm"_s : u"mavlink"_s;
    o->writeAccess = o->protocol == "mavlink"_L1 && cfg.value(u"writeAccess"_s).toBool();
    const int baud = cfg.value(u"baudRate"_s).toInt() > 0 ? cfg.value(u"baudRate"_s).toInt() : 57600;
    Output *out = o.get();

    if (o->type == "serial"_L1) {
        const QString portPath = cfg.value(u"portPath"_s).toString();
        if (portPath.isEmpty()) return setAddStatus(u"Select a serial port first"_s, true);
        // Refuse a port already used by another output
        for (const auto &[id, other] : m_outputs)
            if (other->type == "serial"_L1 && other->portPath == portPath)
                return setAddStatus(u"Error: %1 is already in use by output #%2"_s.arg(portPath).arg(id), true);
        auto *sp = new QSerialPort(portPath, this);
        sp->setBaudRate(baud);
        if (!sp->open(QIODevice::ReadWrite)) {
            const QString err = sp->errorString();
            delete sp;
            return setAddStatus(u"Error: "_s + err, true);
        }
        connect(sp, &QSerialPort::errorOccurred, this, [this, out, sp](QSerialPort::SerialPortError e) {
            if (e == QSerialPort::NoError) return;
            LOG_E("telfwd") << u"#%1 serial error:"_s.arg(out->id) << sp->errorString();
            out->error = sp->errorString();
            if (e == QSerialPort::ResourceError) out->connected = false;
            emit outputsChanged();
        });
        if (o->writeAccess) connect(sp, &QSerialPort::readyRead, this, [this, out, sp] { received(*out, sp->readAll()); });
        else connect(sp, &QSerialPort::readyRead, sp, [sp] { sp->readAll(); });
        o->serial = sp;
        o->portPath = portPath;
        o->label = u"SERIAL %1 @ %2"_s.arg(portPath).arg(baud);
    } else if (o->type == "udp-client"_L1) {
        const QString host = cfg.value(u"host"_s).toString().trimmed();
        const int port = cfg.value(u"port"_s).toInt();
        if (host.isEmpty() || port <= 0) return setAddStatus(u"Enter UDP host and port"_s, true);
        // A host name is resolved once here, not for every datagram
        QHostAddress address(host);
        if (address.isNull()) {
            const QHostInfo info = QHostInfo::fromName(host);
            for (const QHostAddress &a : info.addresses())
                if (a.protocol() == QAbstractSocket::IPv4Protocol) { address = a; break; }
            if (address.isNull()) return setAddStatus(u"Error: cannot resolve %1"_s.arg(host), true);
        }
        o->address = address;
        auto *u = new QUdpSocket(this);
        u->bind(QHostAddress::AnyIPv4, 0);   // replies from the peer come back here
        connect(u, &QUdpSocket::readyRead, this, [this, out, u] {
            while (u->hasPendingDatagrams()) {
                QByteArray d(qsizetype(u->pendingDatagramSize()), '\0');
                u->readDatagram(d.data(), d.size());
                out->bytesRx += d.size();
                if (out->writeAccess) m_link->sendRaw(d);
            }
        });
        connect(u, &QUdpSocket::errorOccurred, this, [this, out, u] {
            LOG_E("telfwd") << u"#%1 UDP error:"_s.arg(out->id) << u->errorString();
            out->error = u->errorString();
            emit outputsChanged();
        });
        o->udp = u;
        o->host = host;
        o->udpPort = quint16(port);
        o->label = u"UDP CLIENT → %1:%2"_s.arg(host).arg(port);
    } else if (o->type == "udp-server"_L1) {
        const int listenPort = cfg.value(u"listenPort"_s).toInt();
        if (listenPort <= 0) return setAddStatus(u"Enter a listen port"_s, true);
        auto *u = new QUdpSocket(this);
        if (!u->bind(QHostAddress::AnyIPv4, quint16(listenPort))) {
            const QString err = u->errorString();
            delete u;
            return setAddStatus(u"Error: "_s + err, true);
        }
        connect(u, &QUdpSocket::readyRead, this, [this, out, u] {
            while (u->hasPendingDatagrams()) {
                QByteArray d(qsizetype(u->pendingDatagramSize()), '\0');
                QHostAddress from;
                quint16 fromPort = 0;
                u->readDatagram(d.data(), d.size(), &from, &fromPort);
                // Any peer that sends us data becomes a forward destination
                out->peers.insert(u"%1:%2"_s.arg(from.toString()).arg(fromPort), Peer{from, fromPort, wallMs()});
                out->bytesRx += d.size();
                if (out->writeAccess) m_link->sendRaw(d);
            }
        });
        o->udp = u;
        o->listenPort = quint16(listenPort);
        o->label = u"UDP SERVER :%1"_s.arg(listenPort);
    } else {
        return setAddStatus(u"Error: Unknown output type: %1"_s.arg(o->type), true);
    }

    o->connected = true;
    o->id = m_nextId++;
    LOG_I("telfwd") << u"output #%1 added: %2 (%3)"_s.arg(o->id).arg(o->label, o->protocol);
    m_outputs.emplace(o->id, std::move(o));
    ensureTimers();
    emit outputsChanged();
    setAddStatus(u"Output started"_s, false);
    QJsonObject s;
    s.insert(u"type"_s, out->type);
    s.insert(u"protocol"_s, out->protocol);
    s.insert(u"baudRate"_s, baud);
    s.insert(u"host"_s, cfg.value(u"host"_s).toString().trimmed());
    s.insert(u"port"_s, cfg.value(u"port"_s).toInt());
    s.insert(u"listenPort"_s, cfg.value(u"listenPort"_s).toInt());
    Settings::instance().setJson(kStorageKey, s);
}

void TelForwardController::removeOutput(int id)
{
    auto it = m_outputs.find(id);
    if (it == m_outputs.end()) return;
    closeOutput(*it->second);
    m_outputs.erase(it);
    ensureTimers();
    emit outputsChanged();
    LOG_I("telfwd") << u"output #%1 removed"_s.arg(id);
}

void TelForwardController::closeOutput(Output &o)
{
    if (o.serial) {
        o.serial->disconnect(this);
        o.serial->close();
        o.serial->deleteLater();
    }
    if (o.udp) {
        o.udp->disconnect(this);
        o.udp->close();
        o.udp->deleteLater();
    }
    o.serial = nullptr;
    o.udp = nullptr;
    o.connected = false;
}

void TelForwardController::received(Output &o, const QByteArray &data)
{
    o.bytesRx += data.size();
    // Write access: a secondary GCS can send commands through us
    m_link->sendRaw(data);
}

// ── Data fan-out ────────────────────────────────────────────────────────────

void TelForwardController::onRawFrame(const QByteArray &frame)
{
    for (auto &[id, o] : m_outputs)
        if (o->connected && o->protocol == "mavlink"_L1) writeOut(*o, frame);
}

void TelForwardController::writeOut(Output &o, const QByteArray &buf)
{
    if (o.type == "serial"_L1) {
        if (o.serial && o.serial->isOpen()) {
            o.serial->write(buf);
            track(o, buf.size());
        }
    } else if (o.type == "udp-client"_L1) {
        if (o.udp->writeDatagram(buf, o.address, o.udpPort) >= 0) track(o, buf.size());
    } else if (o.type == "udp-server"_L1) {
        const qint64 now = wallMs();
        for (auto it = o.peers.begin(); it != o.peers.end();) {
            if (now - it->lastSeen > kPeerTtlMs) {
                it = o.peers.erase(it);
                continue;
            }
            o.udp->writeDatagram(buf, it->address, it->port);
            ++it;
        }
        if (!o.peers.isEmpty()) track(o, buf.size());
    }
}

void TelForwardController::track(Output &o, qsizetype bytes)
{
    o.bytesSent += bytes;
    ++o.msgCount;
    o.window.push_back(wallMs());
    if (o.window.size() > 20000) o.window.pop_front();
}

// ── LTM ─────────────────────────────────────────────────────────────────────

void TelForwardController::ensureTimers()
{
    bool ltm = false;
    for (const auto &[id, o] : m_outputs) ltm = ltm || (o->protocol == "ltm"_L1 && o->connected);
    for (QTimer *t : {&m_ltmFast, &m_ltmSlow, &m_ltmOrigin}) {
        if (ltm && !t->isActive()) t->start();
        if (!ltm) t->stop();
    }
    if (!m_outputs.empty() && !m_status.isActive()) m_status.start();
    if (m_outputs.empty()) m_status.stop();
}

void TelForwardController::ltmTick(int which)
{
    const auto s = m_hub->snapshot();
    QList<QByteArray> frames;
    if (which == 0) frames << ltm::gFrame(*s) << ltm::aFrame(*s);
    else if (which == 1) frames << ltm::sFrame(*s);
    else frames << ltm::oFrame(*s);
    for (auto &[id, o] : m_outputs) {
        if (o->protocol != "ltm"_L1 || !o->connected) continue;
        for (const QByteArray &f : frames) writeOut(*o, f);
    }
}

// ── Status ──────────────────────────────────────────────────────────────────

QVariantList TelForwardController::outputs() const
{
    QVariantList list;
    for (const auto &[id, o] : m_outputs) {
        QStringList parts{o->protocol == "mavlink"_L1 ? u"MAVLink"_s : u"LTM"_s,
                          u"%1 msg/s"_s.arg(o->msgPerSec), formatBytes(o->bytesSent) + u" TX"_s};
        if (o->bytesRx > 0) parts << formatBytes(o->bytesRx) + u" RX"_s;
        if (o->type == "udp-server"_L1) parts << u"%1 peer%2"_s.arg(o->peers.size()).arg(o->peers.size() == 1 ? "" : "s");
        if (!o->connected) parts << u"DISCONNECTED"_s;
        if (!o->error.isEmpty()) parts << u"ERR: "_s + o->error;
        list << QVariantMap{{u"id"_s, id},
                            {u"label"_s, o->label},
                            {u"rw"_s, o->writeAccess},
                            {u"connected"_s, o->connected},
                            {u"stats"_s, parts.join(u" · "_s)}};
    }
    return list;
}

QVariantMap TelForwardController::saved() const
{
    QJsonObject o = Settings::instance().json(kStorageKey).toObject();
    if (o.isEmpty()) {
        const QString raw = Settings::instance().value(kStorageKey).toString();
        if (!raw.isEmpty()) o = QJsonDocument::fromJson(raw.toUtf8()).object();
    }
    return o.toVariantMap();
}

void TelForwardController::setAddStatus(const QString &text, bool error)
{
    m_addStatus = text;
    m_addError = error;
    emit addStatusChanged();
}

void TelForwardController::updateCurrent()
{
    const auto s = m_hub->snapshot();
    double hdg = s->yaw * 180 / M_PI;
    if (hdg < 0) hdg += 360;
    QVariantMap c{{u"lat"_s, s->lat ? QString::number(s->lat, 'f', 7) : u"---"_s},
                  {u"lon"_s, s->lon ? QString::number(s->lon, 'f', 7) : u"---"_s},
                  {u"alt"_s, QString::number(s->rawAlt, 'f', 1) + u" m"_s},
                  {u"heading"_s, QString::number(hdg, 'f', 0) + u"°"_s},
                  {u"gs"_s, QString::number(s->gs, 'f', 1) + u" m/s"_s}};
    if (c == m_current) return;
    m_current = c;
    emit currentChanged();
}

} // namespace corv
