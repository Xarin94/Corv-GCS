#include "video/Rtp.h"

#include <QRegularExpression>
#include <QStringList>

using namespace Qt::StringLiterals;

namespace corv {

namespace {
const QByteArray kStartCode("\0\0\0\1", 4);

quint16 be16(const QByteArray &b, int at) { return quint16((quint8(b[at]) << 8) | quint8(b[at + 1])); }
quint32 be32(const QByteArray &b, int at) { return (quint32(be16(b, at)) << 16) | be16(b, at + 2); }
} // namespace

// ── SDP ──────────────────────────────────────────────────────────────────────

std::optional<SdpVideo> SdpVideo::parse(const QString &sdp)
{
    static const QRegularExpression eol(u"\\r?\\n"_s);
    static const QRegularExpression rtpmap(u"^rtpmap:(\\d+)\\s+([\\w-]+)/(\\d+)"_s);
    struct Media { bool video = false; int payloadType = -1; QStringList attrs; };
    std::optional<Media> media;
    QString sessionControl;
    for (const QString &line : sdp.split(eol)) {
        if (line.startsWith("m="_L1)) {
            if (media && media->video) break;
            const QStringList parts = line.mid(2).split(' '_L1);
            media = Media{parts.value(0) == "video"_L1, parts.value(3).toInt(), {}};
            continue;
        }
        if (!line.startsWith("a="_L1)) continue;
        if (!media) {
            if (line.startsWith("a=control:"_L1)) sessionControl = line.mid(10).trimmed();
            continue;
        }
        media->attrs << line.mid(2);
    }
    if (!media || !media->video) return std::nullopt;

    SdpVideo out;
    out.payloadType = media->payloadType;
    out.sessionControl = sessionControl;
    for (const QString &a : std::as_const(media->attrs)) {
        if (a.startsWith("rtpmap:"_L1)) {
            const auto m = rtpmap.match(a);
            if (m.hasMatch() && m.captured(1).toInt() == out.payloadType) {
                const QString codec = m.captured(2).toUpper();
                out.codec = codec == "HEVC"_L1 ? u"H265"_s : codec;
                out.clockRate = m.captured(3).toInt();
            }
        } else if (a.startsWith("control:"_L1)) {
            out.control = a.mid(8).trimmed();
        } else if (a.startsWith("fmtp:"_L1)) {
            const QStringList params = a.mid(a.indexOf(' '_L1) + 1).split(';'_L1);
            for (const QString &kv : params) {
                const qsizetype eq = kv.indexOf('='_L1);
                if (eq < 0) continue;
                const QString k = kv.left(eq).trimmed().toLower(), v = kv.mid(eq + 1).trimmed();
                if (k != "sprop-parameter-sets"_L1 && k != "sprop-vps"_L1 && k != "sprop-sps"_L1 && k != "sprop-pps"_L1) continue;
                for (const QString &b64 : v.split(','_L1, Qt::SkipEmptyParts)) out.paramSets << QByteArray::fromBase64(b64.toLatin1());
            }
        }
    }
    return out;
}

QString rtspControlUrl(const QString &base, const QString &control)
{
    if (control.isEmpty() || control == "*"_L1) return base;
    if (control.startsWith("rtsp://"_L1, Qt::CaseInsensitive)) return control;
    return base.endsWith('/'_L1) ? base + control : base + '/'_L1 + control;
}

// ── Depacketizer ─────────────────────────────────────────────────────────────

int Depacketizer::nalType(const QByteArray &nal) const
{
    const int b = quint8(nal[0]);
    return m_hevc ? (b >> 1) & 0x3f : b & 0x1f;
}

void Depacketizer::addParamSet(const QByteArray &nal)
{
    if (!nal.isEmpty()) m_paramSets.insert(nalType(nal), nal);
}

void Depacketizer::markLoss()
{
    m_broken = true;
    m_fragments.reset();
}

void Depacketizer::push(const QByteArray &payload, quint32 rtpTimestamp, bool marker)
{
    if (m_timestamp && rtpTimestamp != *m_timestamp) flush();   // the marker of the previous access unit was lost
    m_timestamp = rtpTimestamp;
    if (payload.size() >= (m_hevc ? 3 : 2)) {
        if (m_hevc) pushH265(payload);
        else pushH264(payload);
    }
    if (marker) flush();
}

void Depacketizer::aggregate(const QByteArray &p, int from)
{
    qsizetype off = from;
    while (off + 2 <= p.size()) {
        const int size = be16(p, int(off));
        off += 2;
        if (size == 0 || off + size > p.size()) break;
        addNal(p.mid(off, size));
        off += size;
    }
}

void Depacketizer::pushH264(const QByteArray &p)
{
    const int type = quint8(p[0]) & 0x1f;
    if (type >= 1 && type <= 23) {
        addNal(p);
    } else if (type == 24) {   // STAP-A
        aggregate(p, 1);
    } else if (type == 28) {   // FU-A
        const int fu = quint8(p[1]);
        if (fu & 0x80) m_fragments = QByteArray(1, char((quint8(p[0]) & 0xe0) | (fu & 0x1f))) + p.mid(2);
        else if (m_fragments) *m_fragments += p.mid(2);
        if ((fu & 0x40) && m_fragments) {
            addNal(*m_fragments);
            m_fragments.reset();
        }
    }
    // STAP-B / MTAP / FU-B are not used by cameras in non-interleaved mode
}

void Depacketizer::pushH265(const QByteArray &p)
{
    const int type = (quint8(p[0]) >> 1) & 0x3f;
    if (type == 48) {          // aggregation packet
        aggregate(p, 2);
    } else if (type == 49) {   // fragmentation unit
        const int fu = quint8(p[2]);
        if (fu & 0x80) {
            QByteArray head(2, '\0');
            head[0] = char((quint8(p[0]) & 0x81) | ((fu & 0x3f) << 1));
            head[1] = p[1];
            m_fragments = head + p.mid(3);
        } else if (m_fragments) {
            *m_fragments += p.mid(3);
        }
        if ((fu & 0x40) && m_fragments) {
            addNal(*m_fragments);
            m_fragments.reset();
        }
    } else if (type != 50) {   // 50 = PACI, not used by cameras
        addNal(p);
    }
}

void Depacketizer::addNal(const QByteArray &nal)
{
    if (nal.isEmpty()) return;
    const int type = nalType(nal);
    if (isParamSet(type)) m_paramSets.insert(type, nal);
    m_nals << nal;
}

void Depacketizer::flush()
{
    const QList<QByteArray> nals = std::exchange(m_nals, {});
    const bool broken = std::exchange(m_broken, false);
    m_fragments.reset();
    const quint32 ts = m_timestamp.value_or(0);
    m_timestamp.reset();
    if (nals.isEmpty()) {
        if (broken) m_needKey = true;
        return;
    }

    bool key = false;
    QList<int> present;
    qsizetype bytes = 0;
    for (const QByteArray &nal : nals) {
        const int type = nalType(nal);
        present << type;
        key = key || isKey(type);
        bytes += nal.size() + 4;
    }
    if (broken) {
        m_needKey = true;
        return;
    }
    if (m_needKey && !key) return;
    m_needKey = false;

    AccessUnit au;
    au.key = key;
    au.rtpTimestamp = ts;
    au.data.reserve(bytes + 512);
    // The decoder needs the parameter sets with every key frame in Annex B
    if (key)
        for (auto it = m_paramSets.cbegin(); it != m_paramSets.cend(); ++it)
            if (!present.contains(it.key())) au.data += kStartCode + it.value();
    for (const QByteArray &nal : nals) au.data += kStartCode + nal;
    m_sink(au);
}

// ── RTP ──────────────────────────────────────────────────────────────────────

void RtpReceiver::packet(const QByteArray &pkt)
{
    if (pkt.size() < 12 || (quint8(pkt[0]) >> 6) != 2) return;
    if ((quint8(pkt[1]) & 0x7f) != m_payloadType) return;
    qsizetype off = 12 + (quint8(pkt[0]) & 0x0f) * 4;
    if (quint8(pkt[0]) & 0x10) {   // header extension
        if (pkt.size() < off + 4) return;
        off += 4 + be16(pkt, int(off) + 2) * 4;
    }
    qsizetype end = pkt.size();
    if (quint8(pkt[0]) & 0x20) end -= quint8(pkt[pkt.size() - 1]);   // padding
    if (off >= end) return;
    const Packet p{be16(pkt, 2), be32(pkt, 4), (quint8(pkt[1]) & 0x80) != 0, pkt.mid(off, end - off)};

    if (!m_expected) m_expected = p.seq;
    const quint16 ahead = quint16(p.seq - *m_expected);
    if (ahead == 0) {
        deliver(p);
        drainHeld();
    } else if (ahead < 0x8000) {
        // A packet is missing: this one waits until the gap fills or the window
        // overflows, then the missing ones are given up
        m_held.insert(p.seq, p);
        if (m_held.size() > kReorderWindow) {
            quint16 next = p.seq;
            int best = 0x10000;
            for (auto it = m_held.cbegin(); it != m_held.cend(); ++it) {
                const int d = quint16(it.key() - *m_expected);
                if (d < best) {
                    best = d;
                    next = it.key();
                }
            }
            m_out->markLoss();
            m_expected = next;
            drainHeld();
        }
    }
    // else: a late duplicate of a packet delivered or given up
}

void RtpReceiver::deliver(const Packet &p)
{
    m_expected = quint16(p.seq + 1);
    m_out->push(p.payload, p.ts, p.marker);
}

void RtpReceiver::drainHeld()
{
    while (!m_held.isEmpty()) {
        const auto it = m_held.find(*m_expected);
        if (it == m_held.end()) return;
        const Packet p = it.value();
        m_held.erase(it);
        deliver(p);
    }
}

} // namespace corv
