#include "video/RtspClient.h"

#include "core/DebugLog.h"

#include <QPointer>
#include <QRandomGenerator>
#include <QRegularExpression>
#include <QTcpSocket>
#include <QTimer>
#include <QUdpSocket>
#include <algorithm>

using namespace Qt::StringLiterals;

namespace corv {

RtspClient::RtspClient(const QString &url, QObject *parent) : QObject(parent), m_url(url), m_parsed(url) {}

RtspClient::~RtspClient() { close(); }

void RtspClient::start()
{
    if (!m_parsed.isValid() || m_parsed.host().isEmpty()) {
        QTimer::singleShot(0, this, [this] { finish(u"Invalid address %1"_s.arg(m_url)); });
        return;
    }
    m_socket = new QTcpSocket(this);
    m_socket->setSocketOption(QAbstractSocket::LowDelayOption, 1);
    m_connectTimer = new QTimer(this);
    m_connectTimer->setSingleShot(true);
    connect(m_connectTimer, &QTimer::timeout, this, [this] { finish(u"RTSP connect timeout"_s); });
    m_connectTimer->start(m_timeoutMs);

    connect(m_socket, &QTcpSocket::readyRead, this, &RtspClient::onData);
    connect(m_socket, &QTcpSocket::errorOccurred, this, [this](QAbstractSocket::SocketError e) {
        finish(e == QAbstractSocket::RemoteHostClosedError ? QString() : m_socket->errorString());
    });
    connect(m_socket, &QTcpSocket::disconnected, this, [this] { finish(QString()); });
    connect(m_socket, &QTcpSocket::connected, this, [this] {
        m_connectTimer->stop();
        request(u"OPTIONS"_s, m_url, {}, [this](const Response &) {
            request(u"DESCRIBE"_s, m_url, {{u"Accept"_s, u"application/sdp"_s}}, [this](const Response &r) {
                const auto sdp = SdpVideo::parse(r.body);
                if (!sdp || sdp->codec.isEmpty()) return finish(u"No video track in SDP"_s);
                if (sdp->codec != "H264"_L1 && sdp->codec != "H265"_L1)
                    return finish(u"Codec %1 not decodable here"_s.arg(sdp->codec));
                m_sdp = *sdp;
                m_codec = sdp->codec == "H265"_L1 ? VideoCodec::H265 : VideoCodec::H264;
                m_base = r.headers.value(u"content-base"_s, r.headers.value(u"content-location"_s, m_url));
                m_trackUrl = rtspControlUrl(rtspControlUrl(m_base, m_sdp.sessionControl), m_sdp.control);
                m_depack = std::make_unique<Depacketizer>(m_codec, [this](const AccessUnit &au) {
                    if (m_closed || !m_depack->hasSps()) return;   // no SPS yet: nothing decodable
                    const quint32 ts = au.rtpTimestamp;
                    if (!m_tsBase) m_tsBase = ts;
                    if (ts < m_tsLast && m_tsLast - ts > 0x80000000u) ++m_tsWraps;
                    m_tsLast = ts;
                    const double ticks = double(ts) + double(m_tsWraps) * 4294967296.0 - double(*m_tsBase);
                    emit accessUnit(au, qint64(ticks * 1e6 / std::max(1, m_sdp.clockRate)));
                });
                for (const QByteArray &ps : std::as_const(m_sdp.paramSets)) m_depack->addParamSet(ps);
                m_receiver = std::make_unique<RtpReceiver>(m_sdp.payloadType, m_depack.get());
                setup();
            });
        });
    });
    m_socket->connectToHost(m_parsed.host(), quint16(m_parsed.port(554)));
}

void RtspClient::setup()
{
    request(u"SETUP"_s, m_trackUrl, {{u"Transport"_s, u"RTP/AVP/TCP;unicast;interleaved=0-1"_s}},
            [this](const Response &r) {
                m_transport = u"tcp"_s;
                static const QRegularExpression re(u"interleaved=(\\d+)"_s);
                const auto m = re.match(r.headers.value(u"transport"_s));
                m_rtpChannel = m.hasMatch() ? m.captured(1).toInt() : 0;
                play(r);
            },
            // 461 Unsupported Transport (or a 4xx from servers that do not say so)
            [this](const Response &) { setupUdp(); });
}

void RtspClient::setupUdp()
{
    if (!openUdp()) return finish(u"No free UDP port pair for RTP"_s);
    const int port = m_rtp->localPort();
    request(u"SETUP"_s, m_trackUrl, {{u"Transport"_s, u"RTP/AVP;unicast;client_port=%1-%2"_s.arg(port).arg(port + 1)}},
            [this](const Response &r) {
                m_transport = u"udp"_s;
                m_rtpChannel = -1;
                play(r);
            });
}

void RtspClient::play(const Response &setupResponse)
{
    const QString session = setupResponse.headers.value(u"session"_s);
    m_session = session.section(';'_L1, 0, 0).trimmed();
    static const QRegularExpression re(u"timeout=(\\d+)"_s);
    const auto m = re.match(session);
    const double timeout = m.hasMatch() ? m.captured(1).toInt() : 60;
    m_tsBase.reset();
    m_tsLast = 0;
    m_tsWraps = 0;
    request(u"PLAY"_s, rtspControlUrl(m_base, m_sdp.sessionControl), {{u"Range"_s, u"npt=0.000-"_s}}, [this, timeout](const Response &) {
        // Many servers drop a session that sends nothing for its timeout, even
        // while it is streaming over the same connection
        m_keepAlive = new QTimer(this);
        connect(m_keepAlive, &QTimer::timeout, this, [this] {
            request(u"GET_PARAMETER"_s, m_url, {}, [](const Response &) {}, [](const Response &) {});
        });
        m_keepAlive->start(int(std::clamp(timeout / 2, 5.0, 30.0) * 1000));
        emit playing();
    });
}

/// An even RTP port and the RTCP port above it.
bool RtspClient::openUdp()
{
    for (int attempt = 0; attempt < 20; ++attempt) {
        const quint16 port = quint16(50000 + 2 * QRandomGenerator::global()->bounded(5000));
        auto rtp = std::make_unique<QUdpSocket>();
        auto rtcp = std::make_unique<QUdpSocket>();
        if (!rtp->bind(QHostAddress::AnyIPv4, port) || !rtcp->bind(QHostAddress::AnyIPv4, quint16(port + 1))) continue;
        // Key frames arrive as bursts of a few hundred packets
        rtp->setSocketOption(QAbstractSocket::ReceiveBufferSizeSocketOption, 4 * 1024 * 1024);
        m_rtp = rtp.release();
        m_rtcp = rtcp.release();
        m_rtp->setParent(this);
        m_rtcp->setParent(this);
        connect(m_rtp, &QUdpSocket::readyRead, this, [this] {
            while (m_rtp && m_rtp->hasPendingDatagrams()) {
                QByteArray d(qMax<qint64>(0, m_rtp->pendingDatagramSize()), Qt::Uninitialized);
                const qint64 n = m_rtp->readDatagram(d.data(), d.size());
                if (n <= 0) continue;
                d.truncate(n);
                m_received += n;
                if (m_receiver && !m_closed) m_receiver->packet(d);
            }
        });
        connect(m_rtcp, &QUdpSocket::readyRead, this, [this] {
            while (m_rtcp && m_rtcp->hasPendingDatagrams()) m_rtcp->readDatagram(nullptr, 0);
        });
        return true;
    }
    return false;
}

void RtspClient::needKeyFrame()
{
    if (m_depack) m_depack->needKeyFrame();
}

int RtspClient::write(const QString &method, const QString &url, const Headers &headers)
{
    const int cseq = ++m_cseq;
    QString msg = u"%1 %2 RTSP/1.0\r\nCSeq: %3\r\nUser-Agent: CORV-GCS\r\n"_s.arg(method, url).arg(cseq);
    if (!m_session.isEmpty()) msg += u"Session: %1\r\n"_s.arg(m_session);
    for (const auto &[k, v] : headers) msg += k + u": "_s + v + u"\r\n"_s;
    m_socket->write((msg + u"\r\n"_s).toUtf8());
    return cseq;
}

void RtspClient::request(const QString &method, const QString &url, const Headers &headers, Handler then, Handler onError)
{
    if (m_closed) return;
    const int cseq = write(method, url, headers);
    auto *timer = new QTimer(this);
    timer->setSingleShot(true);
    connect(timer, &QTimer::timeout, this, [this, cseq, method, timer] {
        const Pending p = m_pending.take(cseq);
        timer->deleteLater();
        // A keep-alive the camera does not answer ends nothing: the video tells whether it is there
        if (method != "GET_PARAMETER"_L1) finish(u"%1 timed out"_s.arg(method));
    });
    timer->start(m_timeoutMs);
    m_pending.insert(cseq, {method, std::move(then), std::move(onError), timer});
}

void RtspClient::onData()
{
    if (m_closed) return;
    m_buf += m_socket->readAll();
    QPointer<RtspClient> self(this);
    qsizetype off = 0;
    const QByteArray buf = m_buf;   // shared, not copied: a handler may stop the client
    while (off < buf.size()) {
        const char c = buf[off];
        if (c == '$') {                        // interleaved binary frame
            if (buf.size() - off < 4) break;
            const int channel = quint8(buf[off + 1]);
            const qsizetype len = (quint8(buf[off + 2]) << 8) | quint8(buf[off + 3]);
            if (buf.size() - off < 4 + len) break;
            if (channel == m_rtpChannel && m_receiver) {
                m_received += len;
                m_receiver->packet(buf.mid(off + 4, len));
                if (!self || m_closed) return;
            }
            off += 4 + len;                    // RTCP (odd channel) ignored
        } else if (c == 'R') {                 // RTSP/1.0 response
            const qsizetype headerEnd = buf.indexOf("\r\n\r\n", off);
            if (headerEnd < 0) break;
            const QString head = QString::fromLatin1(buf.mid(off, headerEnd - off));
            static const QRegularExpression cl(u"\\r\\ncontent-length:\\s*(\\d+)"_s, QRegularExpression::CaseInsensitiveOption);
            const auto m = cl.match(head);
            const qsizetype bodyLen = m.hasMatch() ? m.captured(1).toLongLong() : 0;
            if (buf.size() < headerEnd + 4 + bodyLen) break;
            const QString body = QString::fromUtf8(buf.mid(headerEnd + 4, bodyLen));
            off = headerEnd + 4 + bodyLen;
            onResponse(head, body);
            if (!self || m_closed) return;
        } else {
            // Out of sync (or a server request we do not handle): skip to the next frame
            const qsizetype next = buf.indexOf('$', off + 1);
            off = next < 0 ? buf.size() : next;
        }
    }
    m_buf = off >= buf.size() ? QByteArray() : buf.mid(off);
}

void RtspClient::onResponse(const QString &head, const QString &body)
{
    const QStringList lines = head.split(u"\r\n"_s);
    static const QRegularExpression re(u"^RTSP/1\\.\\d\\s+(\\d+)\\s*(.*)$"_s);
    const auto m = re.match(lines.value(0));
    if (!m.hasMatch()) return;
    Response r;
    r.status = m.captured(1).toInt();
    r.reason = m.captured(2);
    r.body = body;
    for (const QString &line : lines.mid(1)) {
        const qsizetype i = line.indexOf(':'_L1);
        if (i > 0) r.headers.insert(line.left(i).trimmed().toLower(), line.mid(i + 1).trimmed());
    }
    const int cseq = r.headers.value(u"cseq"_s).toInt();
    const auto it = m_pending.find(cseq);
    if (it == m_pending.end()) return;
    const Pending p = it.value();
    m_pending.erase(it);
    p.timer->stop();
    p.timer->deleteLater();
    if (r.status >= 200 && r.status < 300) {
        if (p.then) p.then(r);
    } else if (p.onError) {
        p.onError(r);
    } else {
        finish(u"%1 failed: %2 %3"_s.arg(p.method).arg(r.status).arg(r.reason));
    }
}

void RtspClient::stop()
{
    if (m_closed) return;
    if (!m_session.isEmpty() && m_socket && m_socket->state() == QAbstractSocket::ConnectedState) {
        write(u"TEARDOWN"_s, m_url, {});
        m_socket->flush();
    }
    close();
}

void RtspClient::finish(const QString &error)
{
    if (m_closed) return;
    close();
    emit finished(error);
}

void RtspClient::close()
{
    if (m_closed) return;
    m_closed = true;
    if (m_connectTimer) m_connectTimer->stop();
    if (m_keepAlive) m_keepAlive->stop();
    for (const Pending &p : std::as_const(m_pending)) p.timer->stop();
    m_pending.clear();
    if (m_socket) {
        m_socket->disconnect(this);
        m_socket->abort();
    }
    for (QUdpSocket *s : {m_rtp, m_rtcp}) {
        if (!s) continue;
        s->disconnect(this);
        s->close();
    }
}

} // namespace corv
