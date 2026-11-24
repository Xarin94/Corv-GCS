// The FPV video without a camera: SDP, depacketizer and RTP ordering (rtsp-client.js
// of the Electron version), the RTSP session against a server in the test (TCP, the
// 461 fallback to UDP, refusals, a camera that goes away), and one picture of each
// codec through the decoder.
#include "video/FpvStream.h"
#include "video/RtspClient.h"
#include "video/VideoDecoder.h"

#include <QSignalSpy>
#include <QTcpServer>
#include <QTcpSocket>
#include <QTest>
#include <QUdpSocket>
#include <cmath>

using namespace corv;
using namespace Qt::StringLiterals;

namespace {
QByteArray hex(const char *s) { return QByteArray::fromHex(s); }
const QByteArray kStart("\0\0\0\1", 4);

// A 64 x 48 picture of one colour, Y 88 U 118 V 196
const QByteArray kSps264 = hex("6764000aacb423d8088000000300800000030147891350"), kPps264 = hex("68ef0f2c8b"),
                 kIdr264 = hex("65888404bffee8c9fccb2aa571d34834618c539a71cc23d1d23feb907664bc821c162d");
const QByteArray kVps265 = hex("40010c01ffff01600000030090000003000003001e928090"),
                 kSps265 = hex("42010101600000030090000003000003001ea0208316592ae4caf016808000000300800000030084"),
                 kPps265 = hex("4401c173d089"), kIdr265 = hex("2801af78f70403ffd34cffae2d2ef9591020f0001e");

QByteArray rtp(quint16 seq, quint32 ts, bool marker, const QByteArray &payload, int pt = 96)
{
    QByteArray b(12, '\0');
    b[0] = char(0x80);
    b[1] = char(pt | (marker ? 0x80 : 0));
    b[2] = char(seq >> 8);
    b[3] = char(seq);
    for (int i = 0; i < 4; ++i) b[4 + i] = char(ts >> (24 - 8 * i));
    return b + payload;
}

QString sdp264()
{
    return u"v=0\r\no=- 0 0 IN IP4 0\r\ns=x\r\na=control:*\r\nm=audio 0 RTP/AVP 0\r\na=control:audio\r\n"
           "m=video 0 RTP/AVP 96\r\na=rtpmap:96 H264/90000\r\na=fmtp:96 packetization-mode=1;sprop-parameter-sets=%1,%2\r\n"
           "a=control:track1\r\nm=application 0 RTP/AVP 107\r\na=control:meta\r\n"_s
        .arg(QString::fromLatin1(kSps264.toBase64()), QString::fromLatin1(kPps264.toBase64()));
}

/// A camera: answers the session and sends what the test gives it.
class Camera : public QTcpServer
{
public:
    bool udpOnly = false, mute = false;
    QString sdp = sdp264();
    QByteArray describeStatus = "200 OK";
    QStringList requests;
    QTcpSocket *peer = nullptr;
    QUdpSocket udp;
    quint16 clientPort = 0;
    bool playing = false;
    QByteArray split;   // sent in two parts, to arrive in odd chunks

    Camera()
    {
        listen(QHostAddress::LocalHost);
        connect(this, &QTcpServer::newConnection, this, [this] {
            peer = nextPendingConnection();
            connect(peer, &QTcpSocket::readyRead, this, [this] { read(); });
        });
    }
    QString url() const { return u"rtsp://127.0.0.1:%1/main.264"_s.arg(serverPort()); }
    void frame(const QByteArray &pkt)
    {
        QByteArray b = QByteArray("$\0", 2) + char(pkt.size() >> 8) + char(pkt.size()) + pkt;
        peer->write(b.left(3));
        peer->flush();
        peer->write(b.mid(3));
        peer->flush();
    }

private:
    void read()
    {
        m_buf += peer->readAll();
        for (qsizetype end; (end = m_buf.indexOf("\r\n\r\n")) >= 0;) {
            const QStringList lines = QString::fromLatin1(m_buf.left(end)).split(u"\r\n"_s);
            m_buf.remove(0, end + 4);
            const QString method = lines[0].section(' '_L1, 0, 0);
            requests << lines[0];
            QString cseq, transport;
            for (const QString &l : lines) {
                if (l.startsWith("CSeq: "_L1)) cseq = l.mid(6);
                if (l.startsWith("Transport: "_L1)) transport = l.mid(11);
            }
            if (mute) continue;
            auto reply = [&](const QByteArray &status, const QByteArray &headers = {}, const QByteArray &body = {}) {
                peer->write("RTSP/1.0 " + status + "\r\nCSeq: " + cseq.toLatin1() + "\r\n" + headers
                            + (body.isEmpty() ? QByteArray() : "Content-Length: " + QByteArray::number(body.size()) + "\r\n") + "\r\n" + body);
            };
            if (method == "DESCRIBE"_L1) reply(describeStatus, "Content-Type: application/sdp\r\n", sdp.toUtf8());
            else if (method == "SETUP"_L1 && transport.contains(u"TCP"_s) && udpOnly) reply("461 Unsupported Transport");
            else if (method == "SETUP"_L1) {
                if (udpOnly) clientPort = quint16(transport.section(u"client_port="_s, 1).section('-'_L1, 0, 0).toUInt());
                reply("200 OK", "Session: abc;timeout=60\r\nTransport: " + transport.toLatin1() + "\r\n");
            } else {
                reply("200 OK");
                if (method == "PLAY"_L1) playing = true;
            }
        }
    }
    QByteArray m_buf;
};
} // namespace

class TstVideo : public QObject
{
    Q_OBJECT
private slots:
    void sdp()
    {
        const auto s = SdpVideo::parse(sdp264());
        QVERIFY(s);
        QCOMPARE(s->codec, u"H264"_s);
        QCOMPARE(s->payloadType, 96);
        QCOMPARE(s->control, u"track1"_s);   // of the video, not of the audio or metadata tracks around it
        QCOMPARE(s->sessionControl, u"*"_s);
        QCOMPARE(s->paramSets, QList<QByteArray>({kSps264, kPps264}));
        const auto h = SdpVideo::parse(u"m=video 0 RTP/AVP 98\na=rtpmap:98 hevc/90000\na=fmtp:98 sprop-vps=%1; sprop-sps=%2; sprop-pps=%3\n"_s
                                           .arg(QString::fromLatin1(kVps265.toBase64()), QString::fromLatin1(kSps265.toBase64()),
                                                QString::fromLatin1(kPps265.toBase64())));
        QVERIFY(h);
        QCOMPARE(h->codec, u"H265"_s);
        QCOMPARE(h->paramSets.size(), 3);
        QVERIFY(!SdpVideo::parse(u"m=audio 0 RTP/AVP 0\r\n"_s));
        QCOMPARE(rtspControlUrl(u"rtsp://a/b"_s, u"track1"_s), u"rtsp://a/b/track1"_s);
        QCOMPARE(rtspControlUrl(u"rtsp://a/b/"_s, u"*"_s), u"rtsp://a/b/"_s);
        QCOMPARE(rtspControlUrl(u"rtsp://a/b"_s, u"RTSP://c/d"_s), u"RTSP://c/d"_s);
    }

    void h264Units()
    {
        QList<AccessUnit> out;
        Depacketizer d(VideoCodec::H264, [&](const AccessUnit &au) { out << au; });
        // A P picture before any key frame is nothing to show
        d.push(hex("419a"), 0, true);
        QCOMPARE(out.size(), 0);
        for (const QByteArray &ps : {kSps264, kPps264}) d.addParamSet(ps);
        QVERIFY(d.hasSps());
        // The key frame gets the parameter sets of the SDP in front
        d.push(kIdr264, 3000, true);
        QCOMPARE(out.size(), 1);
        QVERIFY(out[0].key);
        QCOMPARE(out[0].data, kStart + kSps264 + kStart + kPps264 + kStart + kIdr264);
        // One with its own (STAP-A) keeps them, once
        QByteArray stap("\x18", 1);
        for (const QByteArray &n : {kSps264, kPps264}) stap += QByteArray(1, char(n.size() >> 8)) + char(n.size()) + n;
        d.push(stap, 6000, false);
        // and the IDR in three fragments (FU-A)
        const QByteArray body = kIdr264.mid(1);
        d.push(hex("7c85") + body.left(10), 6000, false);
        d.push(hex("7c05") + body.mid(10, 10), 6000, false);
        d.push(hex("7c45") + body.mid(20), 6000, true);
        QCOMPARE(out.size(), 2);
        QCOMPARE(out[1].data, kStart + kSps264 + kStart + kPps264 + kStart + kIdr264);
        QCOMPARE(out[1].rtpTimestamp, 6000u);
        // A lost marker: the next timestamp ends the picture
        d.push(hex("419a01"), 9000, false);
        d.push(hex("419a02"), 12000, true);
        QCOMPARE(out.size(), 4);
        QVERIFY(!out[2].key);
        // A loss: nothing until the next key frame
        d.markLoss();
        d.push(hex("419a03"), 15000, true);
        d.push(hex("419a04"), 18000, true);
        QCOMPARE(out.size(), 4);
        d.push(kIdr264, 21000, true);
        QCOMPARE(out.size(), 5);
    }

    void h265Units()
    {
        QList<AccessUnit> out;
        Depacketizer d(VideoCodec::H265, [&](const AccessUnit &au) { out << au; });
        QByteArray ap = hex("6001");   // aggregation packet, type 48
        for (const QByteArray &n : {kVps265, kSps265, kPps265}) ap += QByteArray(1, char(n.size() >> 8)) + char(n.size()) + n;
        d.push(ap, 1, false);
        // The IDR (type 20) in two fragments
        const QByteArray body = kIdr265.mid(2);
        d.push(hex("6201") + char(0x80 | 20) + body.left(9), 1, false);
        d.push(hex("6201") + char(0x40 | 20) + body.mid(9), 1, true);
        QCOMPARE(out.size(), 1);
        QVERIFY(out[0].key);
        QCOMPARE(out[0].data, kStart + kVps265 + kStart + kSps265 + kStart + kPps265 + kStart + kIdr265);
    }

    void rtpOrder()
    {
        QList<AccessUnit> out;
        Depacketizer d(VideoCodec::H264, [&](const AccessUnit &au) { out << au; });
        RtpReceiver r(96, &d);
        auto p = [](int n) { return QByteArray("\x41\x9a", 2) + char(n); };
        r.packet(rtp(65534, 0, true, kIdr264));
        r.packet(rtp(0, 2, true, p(2)));        // ahead across the wrap: held
        QCOMPARE(out.size(), 1);
        r.packet(rtp(65535, 1, true, p(1)));
        QCOMPARE(out.size(), 3);
        r.packet(rtp(65535, 1, true, p(1)));    // a late duplicate
        r.packet(rtp(1, 3, true, p(3), 97));    // another payload type
        r.packet(QByteArray("\x80\x60\0", 3));  // too short
        QCOMPARE(out.size(), 3);
        // Extension and padding are not payload
        QByteArray ext = rtp(1, 3, true, hex("bede00010000ffff") + p(7) + hex("000003"));
        ext[0] = char(0x80 | 0x10 | 0x20);
        r.packet(ext);
        QCOMPARE(out.size(), 4);
        QCOMPARE(out[3].data, kStart + p(7));
        // Packet 2 never comes: given up after a window, then nothing until a key frame
        for (int i = 0; i <= RtpReceiver::kReorderWindow; ++i) r.packet(rtp(quint16(3 + i), quint32(10 + i), true, p(i)));
        QCOMPARE(out.size(), 4);
        r.packet(rtp(quint16(4 + RtpReceiver::kReorderWindow), 99, true, kIdr264));
        QCOMPARE(out.size(), 5);
        QVERIFY(out[4].key);
    }

    void sessionOverTcp()
    {
        Camera cam;
        RtspClient c(cam.url());
        QSignalSpy playing(&c, &RtspClient::playing), done(&c, &RtspClient::finished);
        QList<AccessUnit> out;
        QList<qint64> times;
        connect(&c, &RtspClient::accessUnit, &c, [&](const AccessUnit &au, qint64 t) { out << au; times << t; });
        c.start();
        QTRY_COMPARE(playing.count(), 1);
        QCOMPARE(c.transport(), u"tcp"_s);
        QCOMPARE(c.codec(), VideoCodec::H264);
        QCOMPARE(cam.requests[0], u"OPTIONS %1 RTSP/1.0"_s.arg(cam.url()));
        QVERIFY(cam.requests[2].startsWith(u"SETUP %1/track1 "_s.arg(cam.url())));
        cam.frame(rtp(1, 0xfffff000u, true, kIdr264));
        cam.peer->write("RTSP/1.0 200 OK\r\nCSeq: 999\r\n\r\n");   // an answer between two packets
        cam.frame(rtp(2, 0xfffff000u + 9000u, true, QByteArray("\x41\x9a\x01", 3)));   // across the timestamp's wrap
        QTRY_COMPARE(out.size(), 2);
        QCOMPARE(times, QList<qint64>({0, 100000}));
        // The camera goes away
        cam.peer->close();
        QTRY_COMPARE(done.count(), 1);
        QVERIFY(done[0][0].toString().isEmpty());
    }

    void sessionOverUdp()
    {
        Camera cam;
        cam.udpOnly = true;
        RtspClient c(cam.url());
        QSignalSpy playing(&c, &RtspClient::playing);
        QList<AccessUnit> out;
        connect(&c, &RtspClient::accessUnit, &c, [&](const AccessUnit &au, qint64) { out << au; });
        c.start();
        QTRY_COMPARE(playing.count(), 1);
        QCOMPARE(c.transport(), u"udp"_s);
        QVERIFY(cam.clientPort >= 50000 && cam.clientPort % 2 == 0);
        cam.udp.writeDatagram(rtp(7, 0, true, kIdr264), QHostAddress::LocalHost, cam.clientPort);
        QTRY_COMPARE(out.size(), 1);
        c.stop();
        QTRY_VERIFY(cam.requests.last().startsWith(u"TEARDOWN"_s));
    }

    void refusals()
    {
        {
            Camera cam;
            cam.describeStatus = "404 Not Found";
            RtspClient c(cam.url());
            QSignalSpy done(&c, &RtspClient::finished);
            c.start();
            QTRY_COMPARE(done.count(), 1);
            QCOMPARE(done[0][0].toString(), u"DESCRIBE failed: 404 Not Found"_s);
        }
        {
            Camera cam;
            cam.sdp = u"m=video 0 RTP/AVP 26\r\na=rtpmap:26 JPEG/90000\r\n"_s;
            RtspClient c(cam.url());
            QSignalSpy done(&c, &RtspClient::finished);
            c.start();
            QTRY_COMPARE(done.count(), 1);
            QCOMPARE(done[0][0].toString(), u"Codec JPEG not decodable here"_s);
        }
        {
            Camera cam;
            cam.mute = true;
            RtspClient c(cam.url());
            c.setTimeout(200);
            QSignalSpy done(&c, &RtspClient::finished);
            c.start();
            QTRY_COMPARE(done.count(), 1);
            QCOMPARE(done[0][0].toString(), u"OPTIONS timed out"_s);
        }
        {
            QTcpServer nobody;
            nobody.listen(QHostAddress::LocalHost);
            const quint16 port = nobody.serverPort();
            nobody.close();
            RtspClient c(u"rtsp://127.0.0.1:%1/x"_s.arg(port));
            QSignalSpy done(&c, &RtspClient::finished);
            c.start();
            QTRY_COMPARE(done.count(), 1);
            QVERIFY(!done[0][0].toString().isEmpty());
        }
    }

    void decoder_data()
    {
        QTest::addColumn<bool>("hevc");
        QTest::addColumn<bool>("hardware");
        QTest::newRow("H264 software") << false << false;
        QTest::newRow("H265 software") << true << false;
        QTest::newRow("H264 GPU if any") << false << true;
        QTest::newRow("H265 GPU if any") << true << true;
    }
    void decoder()
    {
        if (!VideoDecoder::available()) QSKIP("built without FFmpeg");
        QFETCH(bool, hevc);
        QFETCH(bool, hardware);
        VideoDecoder dec;
        QString error;
        QVERIFY2(dec.open(hevc ? VideoCodec::H265 : VideoCodec::H264, hardware, &error), qPrintable(error));
        QList<VideoFramePtr> out;
        const QByteArray au = hevc ? kStart + kVps265 + kStart + kSps265 + kStart + kPps265 + kStart + kIdr265
                                   : kStart + kSps264 + kStart + kPps264 + kStart + kIdr264;
        const bool ok = dec.decode(au, 40000, [&](VideoFramePtr f) { out << f; }, &error);
        // An engine may refuse so small a picture: the software decoder may not
        if (hardware && (!ok || out.isEmpty())) QSKIP("the GPU decoder refused a 64 x 48 picture");
        QVERIFY2(ok, qPrintable(error));
        QCOMPARE(out.size(), 1);   // at once, no reordering delay
        const VideoFrame &f = *out[0];
        QCOMPARE(f.size(), QSize(64, 48));
        QCOMPARE(f.ptsUs, 40000);
        QCOMPARE(f.y.size(), 64 * 48);
        QCOMPARE(QSize(f.chromaWidth, f.chromaHeight), QSize(32, 24));
        QCOMPARE(f.u.size(), 32 * 24 * (f.interleaved ? 2 : 1));
        auto near = [](char c, int want) { return std::abs(int(quint8(c)) - want) <= 2; };
        QVERIFY(near(f.y[0], 88) && near(f.y[64 * 48 - 1], 88));
        QVERIFY(near(f.u[0], 118));
        QVERIFY(near(f.interleaved ? f.u[1] : f.v[0], 196));
        QVERIFY(!f.bt709 && !f.fullRange);
        // Nonsense is refused, and the decoder still works
        dec.decode(kStart + QByteArray(40, '\x65'), 80000, [](VideoFramePtr) {}, &error);
        out.clear();
        dec.decode(au, 120000, [&](VideoFramePtr f2) { out << f2; }, &error);
        QCOMPARE(out.size(), 1);
    }

    // The whole stream: frames in the mailbox, TARGET FPS thins them out, the mailbox is
    // emptied at the end
    void stream()
    {
        if (!VideoDecoder::available()) QSKIP("built without FFmpeg");
        qputenv("CORV_VIDEO_HW", "0");
        Camera cam;
        FpvStream s;
        QSignalSpy started(&s, &FpvStream::started), ended(&s, &FpvStream::ended);
        s.start(cam.url(), 10);
        QTRY_VERIFY(cam.playing);
        quint16 seq = 1;
        quint32 ts = 0;
        quint64 first = 0, serial = 0;
        for (int i = 0; i < 30; ++i) {   // 30 pictures at 50 a second, by their time stamps
            cam.frame(rtp(seq++, ts, true, kIdr264));
            ts += 1800;
            QTest::qWait(20);
            if (i == 2) QTRY_COMPARE(started.count(), 1);
            if (i == 2) VideoMailbox::instance().latest(&first);
        }
        const VideoFramePtr f = VideoMailbox::instance().latest(&serial);
        QVERIFY(f);
        QCOMPARE(f->size(), QSize(64, 48));
        QCOMPARE(started[0][0].toString(), u"H264 · software"_s);
        QVERIFY2(serial - first >= 4 && serial - first <= 6, qPrintable(QString::number(serial - first)));   // 10 a second
        cam.peer->close();
        QTRY_COMPARE(ended.count(), 1);
        QCOMPARE(ended[0][0].toBool(), true);
        QVERIFY(!VideoMailbox::instance().latest(&serial));
    }
};

QTEST_GUILESS_MAIN(TstVideo)
#include "tst_video.moc"
