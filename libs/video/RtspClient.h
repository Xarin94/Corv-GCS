// RtspClient.h - minimal RTSP/RTP client for the FPV camera (port of rtsp-client.js).
//
// RTSP 1.0 without authentication, one video track. RTP is asked for over TCP
// (interleaved on the RTSP connection, immune to loss on the radio link) and over
// UDP when the server refuses that. Asynchronous: start(), then playing() once PLAY
// succeeded and accessUnit() for every picture, or finished() with what went wrong.
// Lives on the thread of its owner (FpvStream: the video thread).
#pragma once

#include "video/Rtp.h"

#include <QHash>
#include <QObject>
#include <QUrl>
#include <memory>

QT_FORWARD_DECLARE_CLASS(QTcpSocket)
QT_FORWARD_DECLARE_CLASS(QTimer)
QT_FORWARD_DECLARE_CLASS(QUdpSocket)

namespace corv {

class RtspClient : public QObject
{
    Q_OBJECT
public:
    explicit RtspClient(const QString &url, QObject *parent = nullptr);
    ~RtspClient() override;

    void setTimeout(int ms) { m_timeoutMs = ms; }
    void start();
    /// TEARDOWN and close, without finished().
    void stop();

    VideoCodec codec() const { return m_codec; }
    /// "tcp" or "udp" once playing.
    QString transport() const { return m_transport; }
    /// The decoder lost track: pictures again from the next key frame.
    void needKeyFrame();
    /// Bytes of video received so far.
    qint64 received() const { return m_received; }

signals:
    void playing();
    void accessUnit(const corv::AccessUnit &au, qint64 timestampUs);
    /// The connection could not be made, was refused or ended (error empty: closed by the camera).
    void finished(const QString &error);

private:
    struct Response {
        int status = 0;
        QString reason;
        QHash<QString, QString> headers;   // lower case names
        QString body;
    };
    using Handler = std::function<void(const Response &)>;
    using Headers = QList<std::pair<QString, QString>>;

    int write(const QString &method, const QString &url, const Headers &headers);
    /// then(): 2xx. Anything else goes to onError, or ends the client with "METHOD failed: code reason".
    void request(const QString &method, const QString &url, const Headers &headers, Handler then, Handler onError = {});
    void onData();
    void onResponse(const QString &head, const QString &body);
    void setup();
    void setupUdp();
    void play(const Response &setupResponse);
    bool openUdp();
    void finish(const QString &error);
    void close();

    QString m_url;
    QUrl m_parsed;
    int m_timeoutMs = 5000;
    QTcpSocket *m_socket = nullptr;
    QUdpSocket *m_rtp = nullptr, *m_rtcp = nullptr;
    QTimer *m_connectTimer = nullptr, *m_keepAlive = nullptr;
    QByteArray m_buf;
    int m_cseq = 0;
    struct Pending {
        QString method;
        Handler then, onError;
        QTimer *timer;
    };
    QHash<int, Pending> m_pending;
    QString m_session, m_base, m_trackUrl, m_transport;
    SdpVideo m_sdp;
    VideoCodec m_codec = VideoCodec::H264;
    int m_rtpChannel = -1;
    std::unique_ptr<Depacketizer> m_depack;
    std::unique_ptr<RtpReceiver> m_receiver;
    bool m_closed = false;
    qint64 m_received = 0;
    // 32-bit RTP timestamp -> monotonic microseconds
    std::optional<quint32> m_tsBase;
    quint32 m_tsLast = 0;
    qint64 m_tsWraps = 0;
};

} // namespace corv
