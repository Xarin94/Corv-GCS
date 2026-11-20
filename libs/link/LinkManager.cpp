#include "link/LinkManager.h"

#include "core/Clock.h"
#include "core/DataRoot.h"
#include "core/DebugLog.h"
#include "link/LteLink.h"
#include "link/MavlinkParser.h"
#include "link/MessageBus.h"
#include "link/NetTransports.h"
#include "link/SerialTransport.h"

#include <QThread>

using namespace Qt::StringLiterals;

namespace corv {

LinkManager::LinkManager(MessageBus *bus, QObject *parent) : QObject(parent), m_bus(bus)
{
    qRegisterMetaType<corv::MavMessage>();
}

LinkManager::~LinkManager()
{
    closeTransport(u"shutdown"_s, false);
    m_tlog.stop();
}

template <typename F>
void LinkManager::post(F &&f)
{
    if (QThread::currentThread() == thread()) f();
    else QMetaObject::invokeMethod(this, std::forward<F>(f), Qt::QueuedConnection);
}

QString LinkManager::connectionType() const { return m_typeCopy; }

void LinkManager::connectSerial(const QString &port, int baud)
{
    post([=, this] {
        LOG_I("mavlink") << u"connect serial %1 @ %2 requested"_s.arg(port).arg(baud ? baud : 57600);
        openTransport(new SerialTransport(port, baud ? baud : 57600, u"SERIAL"_s), QLatin1String(kMavlinkSerial));
    });
}

void LinkManager::connectUdp(const QString &host, int port)
{
    post([=, this] {
        LOG_I("mavlink") << u"connect UDP %1:%2 requested"_s.arg(host).arg(port);
        openTransport(new UdpTransport(host, quint16(port ? port : 14550)), QLatin1String(kMavlinkUdp));
    });
}

void LinkManager::connectTcp(const QString &host, int port)
{
    post([=, this] {
        LOG_I("mavlink") << u"connect TCP %1:%2 requested"_s.arg(host).arg(port);
        openTransport(new TcpTransport(host, quint16(port ? port : 5760)), QLatin1String(kMavlinkTcp));
    });
}

void LinkManager::connectMspSerial(const QString &port, int baud, const QString &profile)
{
    post([=, this] {
        LOG_I("msp") << u"connect serial %1 @ %2 (%3) requested"_s.arg(port).arg(baud ? baud : 115200).arg(profile);
        m_mspProfile = profile;
        openTransport(new SerialTransport(port, baud ? baud : 115200, u"MSP"_s), QLatin1String(kMspSerial));
    });
}

void LinkManager::connectMspTcp(const QString &host, int port, const QString &profile)
{
    post([=, this] {
        LOG_I("msp") << u"connect TCP %1:%2 (%3) requested"_s.arg(host).arg(port ? port : 5760).arg(profile);
        m_mspProfile = profile;
        openTransport(new TcpTransport(host, quint16(port ? port : 5760)), QLatin1String(kMspTcp));
    });
}

void LinkManager::mspMissionUpload(const QList<MspWaypoint> &wps, bool save)
{
    post([this, wps, save] {
        if (!m_msp) {
            emit mspMissionDone(u"upload"_s, false, u"MSP link not connected"_s, {});
            return;
        }
        m_msp->missionUpload(wps, save);
    });
}

void LinkManager::mspMissionDownload()
{
    post([this] {
        if (!m_msp) {
            emit mspMissionDone(u"download"_s, false, u"MSP link not connected"_s, {});
            return;
        }
        m_msp->missionDownload();
    });
}

void LinkManager::disconnectLink()
{
    post([this] {
        LOG_I("mavlink") << "disconnect requested";
        closeTransport(u"disconnect"_s, true);
    });
}

void LinkManager::connectLte(const QVariantMap &o)
{
    post([=, this] {
        // Module id and relay only, never the key
        LOG_I("mavlink") << u"connect LTE module %1 via %2:%3 requested"_s
                                .arg(o.value(u"moduleId"_s).toString(), o.value(u"host"_s).toString())
                                .arg(o.value(u"port"_s).toInt());
        LteTransport::Options opt;
        opt.moduleId = o.value(u"moduleId"_s).toString();
        opt.keyHex = o.value(u"keyHex"_s).toString();
        opt.host = o.value(u"host"_s).toString();
        opt.port = o.value(u"port"_s).toInt();
        opt.certSha256 = o.value(u"certSha256"_s).toString();
        auto *t = new LteTransport(opt);
        connect(t, &LteTransport::status, this, &LinkManager::lteStatus);
        // The flight controller's MAVLink and the module's own share the stream
        openTransport(t, QLatin1String(kMavlinkLte));
    });
}

void LinkManager::openTransport(Transport *t, const QString &type)
{
    closeTransport(u"new connection"_s, true);
    t->setParent(this);
    QString error;
    if (!t->open(&error)) {
        LOG_E("mavlink") << type << "connect failed:" << error;
        delete t;
        emit connectFailed(type, error);
        return;
    }
    m_transport = t;
    m_type = type;
    m_typeCopy = type;
    m_bytesRx = m_bytesAtTick = 0;

    // Every link but MSP parses MAVLink
    m_parser = std::make_unique<MavlinkParser>([this](const mavlink_message_t &msg, const QByteArray &raw) {
        if (!m_replay) m_tlog.write(raw, wallMs());
        emit rawFrame(raw);
        MavMessage m;
        m.msg = msg;
        m.arrivalMs = wallMs();
        deliver(m);
    });
    const bool msp = type.startsWith("msp"_L1);
    if (msp) {
        // MSP: polled request / reply, decoded into synthetic MAVLink (msp-manager.js)
        m_msp = std::make_unique<MspSession>(
            [this](const QByteArray &b) { if (m_transport) m_transport->write(b); },
            [this](const mavlink_message_t &msg) {
                ++m_decodedPackets;   // decoded packets, for the link diagnosis
                MavMessage m;
                m.msg = msg;
                m.arrivalMs = wallMs();
                m.synthetic = true;
                deliver(m);
            },
            m_mspProfile);
        connect(m_msp.get(), &MspSession::missionProgress, this, &LinkManager::mspMissionProgress);
        connect(m_msp.get(), &MspSession::missionDone, this, &LinkManager::mspMissionDone);
    }
    m_connectedAt = wallMs();
    m_lastDiag = m_connectedAt;
    m_warnedNoData = m_warnedNoFrames = false;
    m_decodedPackets = 0;

    connect(t, &Transport::received, this, &LinkManager::onBytes);
    connect(t, &Transport::closed, this, [this](const QString &why) { closeTransport(why, true); });

    if (!m_stats) {
        m_stats = new QTimer(this);
        m_stats->setInterval(1000);
        connect(m_stats, &QTimer::timeout, this, &LinkManager::statsTick);
    }
    m_stats->start();
    m_trace.onConnect(t->describe());
    setConnectionState(u"CONNECTED"_s);
    emit connected(type);

    if (!msp) {
        if (!m_heartbeat) {
            m_heartbeat = new QTimer(this);
            m_heartbeat->setInterval(1000);
            connect(m_heartbeat, &QTimer::timeout, this, &LinkManager::heartbeat);
        }
        // TCP: give SITL time to finish initialising before the first heartbeat
        if (type == QLatin1String(kMavlinkTcp)) QTimer::singleShot(2000, m_heartbeat, [this] { if (m_transport) m_heartbeat->start(); });
        else m_heartbeat->start();
    }
    LOG_I("mavlink") << "connected:" << t->describe();
}

void LinkManager::closeTransport(const QString &why, bool notify)
{
    if (m_heartbeat) m_heartbeat->stop();
    if (m_stats) m_stats->stop();
    if (!m_transport) return;
    Transport *t = m_transport;
    m_transport = nullptr;
    t->disconnect(this);
    t->close();
    t->deleteLater();
    m_msp.reset();   // a transfer in flight hears that the link closed
    m_parser.reset();
    m_type.clear();
    m_typeCopy.clear();
    m_trace.onDisconnect(why);
    if (notify) setConnectionState(u"DISCONNECTED"_s);
}

void LinkManager::setConnectionState(const QString &state)
{
    // Auto-start / stop of the tlog recording on connection changes
    if (state == "CONNECTED"_L1 && !m_tlog.isOpen() && !m_replay && !m_type.startsWith("msp"_L1)) {
        if (!m_tlog.start(DataRoot::logsDir()).isEmpty()) m_tlogAuto = true;
    } else if (state == "DISCONNECTED"_L1 && m_tlogAuto) {
        m_tlog.stop();
        m_tlogAuto = false;
    }
    LOG_I("mavlink") << "connection state ->" << state;
    emit connectionState(state);
}

void LinkManager::onBytes(const QByteArray &b)
{
    m_bytesRx += b.size();
    m_trace.onBytes(b.size());
    if (m_msp) m_msp->push(b);
    else if (m_parser) m_parser->push(b);
}

void LinkManager::deliver(const MavMessage &m)
{
    if (!m_replay || m.replay) m_trace.onMessage(m);
    emit message(m);
    if (m_bus) m_bus->publish(m);
}

void LinkManager::injectReplay(const MavMessage &m)
{
    post([this, m] { deliver(m); });
}

void LinkManager::send(const mavlink_message_t &msg)
{
    post([this, msg] { sendNow(msg); });
}

void LinkManager::sendNow(mavlink_message_t msg)
{
    if (m_muted) {
        const int skipped = DebugLog::throttle(u"tx-muted"_s, 10000);
        if (skipped >= 0)
            LOG_W("mavlink") << "GCS output muted:" << mav::messageName(msg.msgid) << "not sent"
                             << (skipped ? u"(+%1 more)"_s.arg(skipped) : QString());
        return;
    }
    if (!m_transport || m_type.startsWith("msp"_L1)) return;
    if (!m_transport->canSend()) {
        const int skipped = DebugLog::throttle(u"udp-no-remote"_s, 10000);
        if (skipped >= 0)
            LOG_W("mavlink") << "no remote endpoint yet for UDP (nothing received):" << mav::messageName(msg.msgid) << "not sent";
        return;
    }
    // Our identity, our sequence, the CRC for both
    if (!mav::finalizeForSend(msg, m_seq)) return;
    ++m_seq;
    m_trace.onSend(msg);
    writeOut(MavlinkParser::serialize(msg), true);
}

void LinkManager::writeOut(const QByteArray &bytes, bool record)
{
    if (!m_transport) return;
    // The same bytes go to the tlog and the wire
    if (record && !m_replay) m_tlog.write(bytes, wallMs());
    m_transport->write(bytes);
}

void LinkManager::sendRaw(const QByteArray &bytes)
{
    post([this, bytes] {
        if (m_muted || !m_transport || !m_transport->canSend()) return;
        m_transport->write(bytes);
    });
}

void LinkManager::heartbeat()
{
    if (!m_transport) return;
    mavlink_message_t m;
    mavlink_msg_heartbeat_pack(mav::kGcsSystemId, mav::kGcsComponentId, &m, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
    sendNow(m);
}

void LinkManager::statsTick()
{
    const qint64 delta = m_bytesRx - m_bytesAtTick;
    m_bytesAtTick = m_bytesRx;
    const double kbps = std::round(delta * 8.0 / 1000.0 * 10.0) / 10.0;
    if (m_transport) emit linkStats(m_transport->kind(), kbps, m_bytesRx);
    m_trace.tick();
    if (m_transport) diagnose();
}

void LinkManager::diagnose()
{
    // What reaches the parsers, so "no data" and "wrong data" read apart in the
    // debug log and on the HUD (a wrong baud rate gives bytes and no frames)
    const qint64 now = wallMs();
    const quint64 mavOk = m_parser ? m_parser->framesOk() : 0;
    const quint64 mavBad = m_parser ? m_parser->framesBadCrc() : 0;
    const QString what = m_transport->describe();
    if (now - m_connectedAt >= 5000) {
        if (m_bytesRx == 0 && !m_warnedNoData) {
            m_warnedNoData = true;
            const QString text = u"No data from %1 in 5 s: check port, cable and that the vehicle is on"_s.arg(what);
            LOG_W("link") << text;
            emit linkWarning(text);
        } else if (m_bytesRx > 0 && mavOk == 0 && m_decodedPackets == 0 && !m_warnedNoFrames) {
            m_warnedNoFrames = true;
            const QString text = u"%1: %2 bytes but no valid MAVLink frame (wrong baud rate or protocol?)"_s
                                     .arg(what).arg(m_bytesRx);
            LOG_W("link") << text;
            emit linkWarning(text);
        }
    }
    if (now - m_lastDiag >= 10000) {
        m_lastDiag = now;
        LOG_I("link") << u"rx %1 bytes: MAVLink %2 ok / %3 bad CRC%4"_s
                             .arg(m_bytesRx).arg(mavOk).arg(mavBad)
                             .arg(m_msp ? u", MSP %1 decoded"_s.arg(m_decodedPackets) : QString());
    }
}

void LinkManager::setGcsMuted(bool muted)
{
    m_muted = muted;
    DebugLog::setHeader(u"GCS output"_s, muted ? u"MUTED (nothing is sent to the vehicle)"_s : QString());
    LOG_I("mavlink") << "GCS output" << (muted ? "MUTED" : "UNMUTED");
}

void LinkManager::setReplayActive(bool active) { m_replay = active; }

} // namespace corv
