#include "link/MavFtp.h"

#include "core/DebugLog.h"

#include <QRandomGenerator>
#include <QtEndian>
#include <algorithm>
#include <cstring>

namespace corv {

namespace ftp {

QByteArray buildPayload(const Request &r)
{
    QByteArray p(kPayloadLen, '\0');
    auto *b = reinterpret_cast<uchar *>(p.data());
    qToLittleEndian<quint16>(r.seq, b);
    b[2] = r.session;
    b[3] = r.opcode;
    b[5] = r.reqOpcode;
    b[6] = r.burstComplete ? 1 : 0;
    qToLittleEndian<quint32>(r.offset, b + 8);
    int n = 0;
    if (!r.data.isEmpty()) {
        n = int(std::min<qsizetype>(r.data.size(), kDataLen));
        std::memcpy(b + kHeaderLen, r.data.constData(), size_t(n));
    }
    b[4] = uint8_t(r.size >= 0 ? r.size : n);
    return p;
}

std::optional<Packet> parsePayload(const QByteArray &raw)
{
    if (raw.size() < kHeaderLen) return std::nullopt;
    const auto *b = reinterpret_cast<const uchar *>(raw.constData());
    Packet p;
    p.seq = qFromLittleEndian<quint16>(b);
    p.session = b[2];
    p.opcode = b[3];
    p.size = b[4];
    p.reqOpcode = b[5];
    p.burstComplete = b[6] != 0;
    p.offset = qFromLittleEndian<quint32>(b + 8);
    const int size = std::min<int>({int(b[4]), kDataLen, int(std::max<qsizetype>(0, raw.size() - kHeaderLen))});
    p.data = raw.mid(kHeaderLen, size);
    return p;
}

namespace {
constexpr quint16 kPckMagic = 0x671b;
constexpr quint16 kPckMagicDefaults = 0x671c;
int typeSize(int ptype)
{
    switch (ptype) {
    case 1: return 1;
    case 2: return 2;
    case 3:
    case 4: return 4;
    default: return 0;
    }
}
// AP_Param type -> MAV_PARAM_TYPE, as PARAM_VALUE reports it
int toMavType(int ptype)
{
    switch (ptype) {
    case 1: return 2;
    case 2: return 4;
    case 3: return 6;
    default: return 9;
    }
}
double readValue(const uchar *p, int ptype)
{
    switch (ptype) {
    case 1: return double(qint8(p[0]));
    case 2: return double(qFromLittleEndian<qint16>(p));
    case 3: return double(qFromLittleEndian<qint32>(p));
    default: return double(qFromLittleEndian<float>(p));
    }
}
} // namespace

ParamPack decodeParamPack(const QByteArray &raw)
{
    ParamPack out;
    const auto *b = reinterpret_cast<const uchar *>(raw.constData());
    const qsizetype len = raw.size();
    if (len < 6) {
        out.error = QStringLiteral("short header");
        return out;
    }
    const quint16 magic = qFromLittleEndian<quint16>(b);
    if (magic != kPckMagic && magic != kPckMagicDefaults) {
        out.error = QStringLiteral("bad magic 0x%1").arg(magic, 0, 16);
        return out;
    }
    const bool withDefaults = magic == kPckMagicDefaults;
    const int numParams = qFromLittleEndian<quint16>(b + 2);
    out.total = qFromLittleEndian<quint16>(b + 4);
    out.ok = true;
    qsizetype pos = 6;
    QByteArray lastName;
    while (pos < len) {
        while (pos < len && b[pos] == 0) ++pos;   // padding
        if (pos + 2 > len) break;
        const int ptype = b[pos] & 0x0F;
        const int flags = b[pos] >> 4;
        const int commonLen = b[pos + 1] & 0x0F;
        const int nameLen = (b[pos + 1] >> 4) + 1;
        const int tlen = typeSize(ptype);
        if (!tlen) {
            out.ok = false;
            out.error = QStringLiteral("bad type %1 at %2").arg(ptype).arg(pos);
            break;
        }
        if (commonLen > lastName.size()) {
            out.ok = false;
            out.error = QStringLiteral("bad common length at %1").arg(pos);
            break;
        }
        const bool hasDefault = withDefaults && (flags & 1);
        const qsizetype entryLen = 2 + nameLen + tlen + (hasDefault ? tlen : 0);
        if (pos + entryLen > len) break;   // the rest has not arrived yet
        const QByteArray name = lastName.left(commonLen) + QByteArray(reinterpret_cast<const char *>(b + pos + 2), nameLen);
        PackedParam p;
        p.name = QString::fromLatin1(name);
        p.value = readValue(b + pos + 2 + nameLen, ptype);
        p.type = toMavType(ptype);
        if (hasDefault) p.defaultValue = readValue(b + pos + 2 + nameLen + tlen, ptype);
        else if (withDefaults) p.defaultValue = p.value;
        out.params.append(p);
        lastName = name;
        pos += entryLen;
    }
    out.count = int(out.params.size());
    out.complete = out.ok && out.count == numParams && numParams > 0;
    return out;
}

QByteArray encodeParamPack(const QList<ParamToPack> &params, int packetLen)
{
    QByteArray out(6, '\0');
    qToLittleEndian<quint16>(kPckMagic, out.data());
    qToLittleEndian<quint16>(quint16(params.size()), out.data() + 2);
    qToLittleEndian<quint16>(quint16(params.size()), out.data() + 4);
    QByteArray lastName;
    for (const ParamToPack &p : params) {
        const QByteArray name = p.name.toLatin1();
        int common = 0;
        while (common < std::min<qsizetype>({15, lastName.size(), name.size()}) && lastName[common] == name[common]) ++common;
        QByteArray tail = name.mid(common);
        if (tail.isEmpty()) {
            --common;
            tail = name.mid(common);
        }
        const int tlen = typeSize(p.ptype);
        const qsizetype entry = 2 + tail.size() + tlen;
        const qsizetype pos = out.size();
        const qsizetype boundary = pos / packetLen;
        if ((pos + entry - 1) / packetLen != boundary) out.append(QByteArray((boundary + 1) * packetLen - pos, '\0'));
        QByteArray e(entry, '\0');
        e[0] = char(p.ptype & 0x0F);
        e[1] = char((common & 0x0F) | ((tail.size() - 1) << 4));
        std::memcpy(e.data() + 2, tail.constData(), size_t(tail.size()));
        uchar *v = reinterpret_cast<uchar *>(e.data()) + 2 + tail.size();
        switch (p.ptype) {
        case 1: v[0] = uchar(qint8(p.value)); break;
        case 2: qToLittleEndian<qint16>(qint16(p.value), v); break;
        case 3: qToLittleEndian<qint32>(qint32(p.value), v); break;
        default: qToLittleEndian<float>(float(p.value), v); break;
        }
        out.append(e);
        lastName = name;
    }
    return out;
}

} // namespace ftp

using namespace ftp;

struct MavFtpClient::Transfer {
    QString path;
    Options opts;
    std::function<void(const Progress &)> progress;
    Done done;
    uint8_t session = 0;
    std::map<uint32_t, QByteArray> chunks;        // offset -> data
    std::vector<std::pair<uint32_t, uint32_t>> gaps;   // [offset, length] still to read
    uint32_t expected = 0;                         // next offset the burst should deliver
    qint64 eof = -1;                               // file length once known
    quint32 sizeHint = 0;
    qint64 bytes = 0;
    int packets = 0;
    enum Phase { Open, Burst, Gaps, Finished } phase = Open;
    bool resetDone = false;
    // The outstanding request
    bool pending = false;
    Request pendingReq;
    int tries = 0;
};

MavFtpClient::MavFtpClient(Send send, QObject *parent)
    : QObject(parent), m_send(std::move(send)), m_seq(uint16_t(QRandomGenerator::global()->bounded(0xFFFF)))
{
    m_timer.setSingleShot(true);
    connect(&m_timer, &QTimer::timeout, this, &MavFtpClient::onTimeout);
}

MavFtpClient::~MavFtpClient() = default;

void MavFtpClient::onPacket(const QByteArray &payload)
{
    if (!m_t) return;
    if (const auto op = parsePayload(payload)) handle(*op);
}

void MavFtpClient::cancel(const QString &reason)
{
    if (m_t) abort(QStringLiteral("CANCELLED"), reason);
}

void MavFtpClient::readFile(const QString &path, const Options &opts, std::function<void(const Progress &)> progress, Done done)
{
    if (m_t) {
        if (done) done(false, {}, QStringLiteral("BUSY"), QStringLiteral("an FTP transfer is already running"));
        return;
    }
    m_t = std::make_unique<Transfer>();
    m_t->path = path;
    m_t->opts = opts;
    m_t->progress = std::move(progress);
    m_t->done = std::move(done);
    m_session = uint8_t(m_session + 1);
    m_t->session = m_session;
    sendOpen();
}

QByteArray MavFtpClient::partial() const
{
    QByteArray out;
    if (!m_t) return out;
    uint32_t pos = 0;
    for (;;) {
        auto it = m_t->chunks.find(pos);
        if (it == m_t->chunks.end() || it->second.isEmpty()) break;
        out.append(it->second);
        pos += uint32_t(it->second.size());
    }
    return out;
}

void MavFtpClient::abort(const QString &code, const QString &text)
{
    if (!m_t) return;
    m_timer.stop();
    LOG_W("ftp") << m_t->path << ":" << text;
    auto t = std::move(m_t);   // the transfer is over before its callback runs
    if (t->done) t->done(false, {}, code, text);
}

void MavFtpClient::finish()
{
    if (!m_t) return;
    m_timer.stop();
    QByteArray out(qsizetype(m_t->eof), '\0');
    for (const auto &[ofs, buf] : m_t->chunks) {
        if (qint64(ofs) >= m_t->eof) continue;
        const qint64 n = std::min<qint64>(buf.size(), m_t->eof - ofs);
        std::memcpy(out.data() + ofs, buf.constData(), size_t(n));
    }
    m_t->phase = Transfer::Finished;
    // Close the session; the ACK is not worth waiting for
    Request r;
    r.opcode = TerminateSession;
    sendRaw(r);
    auto t = std::move(m_t);
    if (t->done) t->done(true, out, {}, {});
}

void MavFtpClient::sendRaw(Request r)
{
    m_seq = uint16_t(m_seq + 1);
    r.seq = m_seq;
    r.session = m_t ? m_t->session : m_session;
    if (m_send) m_send(buildPayload(r), m_t ? m_t->opts.targetSystem : 1, m_t ? m_t->opts.targetComponent : 1);
}

void MavFtpClient::request(Request r, int tries)
{
    m_t->pending = true;
    m_t->pendingReq = r;
    m_t->tries = tries;
    sendRaw(r);
    m_timer.start(m_t->opts.timeoutMs);
}

void MavFtpClient::onTimeout()
{
    if (!m_t || !m_t->pending) return;
    if (m_t->tries >= m_t->opts.retries) {
        if (m_t->phase == Transfer::Open)
            abort(QStringLiteral("NOREPLY"), QStringLiteral("no FTP reply from the vehicle after %1 attempts").arg(m_t->opts.retries + 1));
        else
            abort(QStringLiteral("TIMEOUT"), QStringLiteral("FTP transfer stalled at %1 bytes").arg(m_t->bytes));
        return;
    }
    // A burst that went quiet is asked again from the first byte still missing
    if (m_t->pendingReq.opcode == BurstReadFile) {
        Request r;
        r.opcode = BurstReadFile;
        r.offset = m_t->gaps.empty() ? m_t->expected : m_t->gaps.front().first;
        r.size = kDataLen;
        request(r, m_t->tries + 1);
    } else {
        request(m_t->pendingReq, m_t->tries + 1);
    }
}

void MavFtpClient::sendOpen()
{
    m_t->phase = Transfer::Open;
    Request r;
    r.opcode = OpenFileRO;
    r.data = m_t->path.toUtf8();
    request(r);
}

void MavFtpClient::sendBurst(uint32_t offset)
{
    m_t->phase = Transfer::Burst;
    Request r;
    r.opcode = BurstReadFile;
    r.offset = offset;
    r.size = kDataLen;
    request(r);
}

void MavFtpClient::sendNextGap()
{
    if (m_t->gaps.empty()) {
        if (m_t->eof >= 0) finish();
        else sendBurst(m_t->expected);
        return;
    }
    m_t->phase = Transfer::Gaps;
    Request r;
    r.opcode = ReadFile;
    r.offset = m_t->gaps.front().first;
    r.size = int(std::min<uint32_t>(m_t->gaps.front().second, kDataLen));
    request(r);
}

void MavFtpClient::storeChunk(const Packet &op)
{
    if (op.data.isEmpty()) return;
    const uint32_t len = uint32_t(op.data.size());
    const uint32_t end = op.offset + len;
    auto known = m_t->chunks.find(op.offset);
    const qint64 had = known != m_t->chunks.end() ? known->second.size() : 0;
    if (had >= len) return;   // duplicate
    m_t->chunks[op.offset] = op.data;
    m_t->bytes += len - had;
    // Out of the gaps, whichever gap it falls in
    std::vector<std::pair<uint32_t, uint32_t>> gaps;
    for (const auto &[gofs, glen] : m_t->gaps) {
        const uint32_t gend = gofs + glen;
        if (end <= gofs || op.offset >= gend) {
            gaps.push_back({gofs, glen});
            continue;
        }
        if (op.offset > gofs) gaps.push_back({gofs, op.offset - gofs});
        if (end < gend) gaps.push_back({end, gend - end});
    }
    m_t->gaps = std::move(gaps);
    if (m_t->progress) m_t->progress({m_t->bytes, m_t->sizeHint, m_t->packets, m_t->eof});
}

void MavFtpClient::handle(const Packet &op)
{
    if (!m_t || op.session != m_t->session) return;   // another client, or an old session
    if (op.opcode != Ack && op.opcode != Nak) return;
    ++m_t->packets;
    auto nak = [&op]() {
        const int code = op.data.isEmpty() ? Fail : uchar(op.data[0]);
        static const char *names[] = {"None", "Fail", "FailErrno", "InvalidDataSize", "InvalidSession", "NoSessionsAvailable",
                                      "EndOfFile", "UnknownCommand", "FileExists", "FileProtected", "FileNotFound"};
        QString text = code >= 0 && code <= 10 ? QString::fromLatin1(names[code]) : QStringLiteral("error %1").arg(code);
        if (code == FailErrno && op.data.size() > 1) text += QStringLiteral(" errno %1").arg(uchar(op.data[1]));
        return std::make_pair(code, text);
    };

    switch (op.reqOpcode) {
    case OpenFileRO: {
        if (m_t->phase != Transfer::Open) return;
        if (op.opcode == Nak) {
            const auto [code, text] = nak();
            if (code == NoSessionsAvailable && !m_t->resetDone) {
                // A session left open by an interrupted transfer: free them all and try once more
                m_t->resetDone = true;
                LOG_I("ftp") << "no session available, resetting the vehicle sessions";
                Request r;
                r.opcode = ResetSessions;
                request(r);
                return;
            }
            abort(QStringLiteral("NAK"), QStringLiteral("open %1: %2").arg(m_t->path, text));
            return;
        }
        m_t->sizeHint = op.data.size() >= 4 ? qFromLittleEndian<quint32>(op.data.constData()) : 0;
        LOG_I("ftp") << m_t->path << "open, size hint" << m_t->sizeHint << "B";
        sendBurst(0);
        return;
    }
    case ResetSessions:
        if (m_t->phase == Transfer::Open) sendOpen();
        return;
    case BurstReadFile: {
        if (m_t->phase == Transfer::Gaps && op.opcode == Ack) {
            // A straggler from the burst: still useful if it fills the gap being read
            const qint64 want = m_t->pending ? qint64(m_t->pendingReq.offset) : -1;
            storeChunk(op);
            if (!m_t) return;   // cancelled from the progress callback
            const bool stillThere = std::any_of(m_t->gaps.begin(), m_t->gaps.end(), [want](const auto &g) { return qint64(g.first) == want; });
            if (want >= 0 && !stillThere) {
                m_timer.stop();
                m_t->pending = false;
                sendNextGap();
            }
            return;
        }
        if (m_t->phase != Transfer::Burst) return;
        if (op.opcode == Nak) {
            const auto [code, text] = nak();
            if (code == EndOfFile) {
                m_t->eof = m_t->expected;
                m_timer.stop();
                m_t->pending = false;
                sendNextGap();
                return;
            }
            abort(QStringLiteral("NAK"), QStringLiteral("burst read: %1").arg(text));
            return;
        }
        // Any ACK keeps the burst alive
        if (m_t->pending) m_timer.start(m_t->opts.timeoutMs);
        if (op.offset > m_t->expected) {
            // Packets lost between expected and this one
            m_t->gaps.push_back({m_t->expected, op.offset - m_t->expected});
            m_t->expected = op.offset;
        }
        storeChunk(op);
        if (!m_t) return;
        if (op.offset == m_t->expected) m_t->expected = op.offset + uint32_t(op.data.size());
        if (op.burstComplete) {
            if (op.data.size() < kDataLen) {
                // A short last packet is the end of the file
                m_t->eof = std::max<qint64>(m_t->expected, qint64(op.offset) + op.data.size());
                m_timer.stop();
                m_t->pending = false;
                sendNextGap();
            } else {
                // The firmware's burst is over; ask for the rest
                sendBurst(m_t->expected);
            }
        }
        return;
    }
    case ReadFile: {
        if (m_t->phase != Transfer::Gaps) return;
        if (op.opcode == Nak) {
            const auto [code, text] = nak();
            if (code == EndOfFile) {
                // The gap lies past the end of the file: nothing to fetch
                const uint32_t at = m_t->pendingReq.offset;
                std::erase_if(m_t->gaps, [at](const auto &g) { return g.first == at; });
                m_timer.stop();
                m_t->pending = false;
                sendNextGap();
                return;
            }
            abort(QStringLiteral("NAK"), QStringLiteral("read at %1: %2").arg(op.offset).arg(text));
            return;
        }
        storeChunk(op);
        if (!m_t) return;
        m_timer.stop();
        m_t->pending = false;
        sendNextGap();
        return;
    }
    default:
        return;
    }
}

} // namespace corv
