// Rtp.h - what the FPV camera sends, as data: the SDP of its stream and the RTP
// payloads put together into Annex-B access units (port of the Depacketizer and
// parseSdp of rtsp-client.js). No sockets: testable on its own.
#pragma once

#include <QByteArray>
#include <QHash>
#include <QList>
#include <QMap>
#include <QString>
#include <functional>
#include <optional>

namespace corv {

enum class VideoCodec { H264, H265 };

/// One picture as the decoder takes it (Annex B), parameter sets in front of the key frames.
struct AccessUnit {
    QByteArray data;
    bool key = false;
    quint32 rtpTimestamp = 0;
};

/// First video media of an SDP.
struct SdpVideo {
    int payloadType = -1;
    QString codec;             // "H264", "H265", or what the camera says
    int clockRate = 90000;
    QString control, sessionControl;
    QList<QByteArray> paramSets;   // sprop-*

    static std::optional<SdpVideo> parse(const QString &sdp);
};

/// The URL of a control attribute under a base.
QString rtspControlUrl(const QString &base, const QString &control);

/// Reassembles RTP payloads (RFC 6184 for H.264, RFC 7798 for H.265) into access units.
class Depacketizer
{
public:
    using Sink = std::function<void(const AccessUnit &)>;
    Depacketizer(VideoCodec codec, Sink sink) : m_hevc(codec == VideoCodec::H265), m_sink(std::move(sink)) {}

    /// Parameter sets announced in the SDP.
    void addParamSet(const QByteArray &nal);
    /// An RTP sequence gap: the current access unit cannot be decoded.
    void markLoss();
    void push(const QByteArray &payload, quint32 rtpTimestamp, bool marker);
    /// The decoder failed: nothing more until the next key frame.
    void needKeyFrame() { m_needKey = true; }
    /// SPS seen, in the SDP or in the stream: a decoder can be set up.
    bool hasSps() const { return m_paramSets.contains(m_hevc ? 33 : 7); }

private:
    int nalType(const QByteArray &nal) const;
    bool isParamSet(int type) const { return m_hevc ? type >= 32 && type <= 34 : type == 7 || type == 8; }
    bool isKey(int type) const { return m_hevc ? type >= 16 && type <= 21 : type == 5; }
    void pushH264(const QByteArray &p);
    void pushH265(const QByteArray &p);
    void aggregate(const QByteArray &p, int from);
    void addNal(const QByteArray &nal);
    void flush();

    bool m_hevc;
    Sink m_sink;
    QList<QByteArray> m_nals;                 // of the access unit being assembled
    std::optional<quint32> m_timestamp;
    std::optional<QByteArray> m_fragments;    // FU being reassembled
    bool m_broken = false;                    // a packet of this access unit was lost
    bool m_needKey = true;                    // after a loss, drop until the next key frame
    QMap<int, QByteArray> m_paramSets;        // by NAL type
};

/// RTP packets in sequence order: UDP may reorder them; a missing one is given up
/// once the window is full of later ones.
class RtpReceiver
{
public:
    static constexpr int kReorderWindow = 32;
    RtpReceiver(int payloadType, Depacketizer *out) : m_payloadType(payloadType), m_out(out) {}
    void packet(const QByteArray &pkt);

private:
    struct Packet {
        quint16 seq;
        quint32 ts;
        bool marker;
        QByteArray payload;
    };
    void deliver(const Packet &p);
    void drainHeld();

    int m_payloadType;
    Depacketizer *m_out;
    std::optional<quint16> m_expected;
    QHash<quint16, Packet> m_held;
};

} // namespace corv
