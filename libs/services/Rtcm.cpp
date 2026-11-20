#include "services/Rtcm.h"

#include <QHash>
#include <QRegularExpression>
#include <QStringList>
#include <QTime>
#include <algorithm>
#include <cmath>

using namespace Qt::StringLiterals;

namespace corv::rtcm {

QList<Frame> Parser::parse(const QByteArray &data)
{
    m_buf.append(data);
    QList<Frame> out;
    while (m_buf.size() >= 6) {   // minimum: 3 header + 0 payload + 3 CRC
        const qsizetype sync = m_buf.indexOf(char(0xD3));
        if (sync < 0) {
            m_buf.clear();
            break;
        }
        if (sync > 0) m_buf.remove(0, sync);
        if (m_buf.size() < 3) break;
        const auto *b = reinterpret_cast<const uint8_t *>(m_buf.constData());
        const int len = ((b[1] & 0x03) << 8) | b[2];
        const int frameLen = 3 + len + 3;   // header + payload + CRC
        if (m_buf.size() < frameLen) break;   // wait for more data
        Frame f;
        f.length = len;
        if (len >= 2) f.type = (b[3] << 4) | ((b[4] >> 4) & 0x0F);
        f.raw = m_buf.left(frameLen);
        m_buf.remove(0, frameLen);
        out.append(f);
    }
    // Keep the buffer from growing unbounded
    if (m_buf.size() > 8192) m_buf = m_buf.right(4096);
    return out;
}

QString typeName(int type)
{
    static const QHash<int, QString> names = {
        {1001, u"GPS L1 Obs"_s},
        {1002, u"GPS L1 Ext Obs"_s},
        {1003, u"GPS L1/L2 Obs"_s},
        {1004, u"GPS L1/L2 Ext Obs"_s},
        {1005, u"Base Position"_s},
        {1006, u"Base Position + Height"_s},
        {1007, u"Antenna Descriptor"_s},
        {1008, u"Antenna Serial"_s},
        {1009, u"GLONASS L1 Obs"_s},
        {1010, u"GLONASS L1 Ext Obs"_s},
        {1011, u"GLONASS L1/L2 Obs"_s},
        {1012, u"GLONASS L1/L2 Ext Obs"_s},
        {1033, u"Receiver/Antenna Info"_s},
        {1074, u"GPS MSM4"_s},
        {1077, u"GPS MSM7"_s},
        {1084, u"GLONASS MSM4"_s},
        {1087, u"GLONASS MSM7"_s},
        {1094, u"Galileo MSM4"_s},
        {1097, u"Galileo MSM7"_s},
        {1124, u"BeiDou MSM4"_s},
        {1127, u"BeiDou MSM7"_s},
        {1230, u"GLONASS Code-Phase Bias"_s},
        {4072, u"u-blox Proprietary"_s}};
    return names.value(type, u"Type %1"_s.arg(type));
}

QList<std::pair<uint8_t, QByteArray>> fragments(const QByteArray &frame, std::uint64_t frameCount)
{
    constexpr int kMaxPayload = 180;
    QList<std::pair<uint8_t, QByteArray>> out;
    if (frame.size() <= kMaxPayload) {
        out.append({uint8_t(0), frame});
        return out;
    }
    const int count = int((frame.size() + kMaxPayload - 1) / kMaxPayload);
    const uint8_t seqId = uint8_t(frameCount & 0x1F);
    for (int i = 0; i < count; ++i) {
        const uint8_t flags = uint8_t(1 | ((i & 0x03) << 1) | ((seqId & 0x1F) << 3));
        out.append({flags, frame.mid(qsizetype(i) * kMaxPayload, kMaxPayload)});
    }
    return out;
}

QByteArray buildGga(double lat, double lon, double alt, const QDateTime &utc)
{
    const QTime t = utc.toUTC().time();
    const QString time = u"%1%2%3.00"_s.arg(t.hour(), 2, 10, '0'_L1).arg(t.minute(), 2, 10, '0'_L1)
                             .arg(t.second(), 2, 10, '0'_L1);
    const double latAbs = std::abs(lat), lonAbs = std::abs(lon);
    const int latDeg = int(std::floor(latAbs)), lonDeg = int(std::floor(lonAbs));
    const QString latStr = u"%1%2"_s.arg(latDeg, 2, 10, '0'_L1).arg((latAbs - latDeg) * 60, 10, 'f', 7, '0'_L1);
    const QString lonStr = u"%1%2"_s.arg(lonDeg, 3, 10, '0'_L1).arg((lonAbs - lonDeg) * 60, 10, 'f', 7, '0'_L1);
    const QString body = u"GPGGA,%1,%2,%3,%4,%5,1,10,1.0,%6,M,0.0,M,,"_s
                             .arg(time, latStr, lat >= 0 ? u"N"_s : u"S"_s, lonStr,
                                  lon >= 0 ? u"E"_s : u"W"_s)
                             .arg(alt, 0, 'f', 1);
    uint8_t cs = 0;
    for (QChar c : body) cs ^= uint8_t(c.unicode());
    return u"$%1*%2\r\n"_s.arg(body, u"%1"_s.arg(cs, 2, 16, '0'_L1).toUpper()).toLatin1();
}

bool parseSourcetable(const QByteArray &reply, QList<Mount> *out, QString *error)
{
    const QString text = QString::fromUtf8(reply);
    const QString statusLine = text.section(u"\r\n"_s, 0, 0).trimmed();
    if (!statusLine.contains("200"_L1) && !text.contains("STR;"_L1)) {
        if (error) *error = u"Sourcetable request failed: %1"_s.arg(statusLine.isEmpty() ? u"no response"_s : statusLine);
        return false;
    }
    out->clear();
    static const QRegularExpression nl(u"\r?\n"_s);
    for (const QString &line : text.split(nl)) {
        if (!line.startsWith("STR;"_L1)) continue;
        const QStringList f = line.split(';'_L1);
        const auto at = [&](int i) { return i < f.size() ? f[i] : QString(); };
        Mount m;
        m.mountpoint = at(1);
        m.identifier = at(2);
        m.format = at(3);
        m.navSystem = at(6);
        m.country = at(8);
        m.lat = at(9).toDouble();
        m.lon = at(10).toDouble();
        out->append(m);
    }
    return true;
}

QByteArray Dechunker::feed(const QByteArray &data)
{
    m_buf.append(data);
    QByteArray out;
    while (!m_buf.isEmpty()) {
        if (m_remain > 0) {
            const qsizetype take = std::min<qsizetype>(m_remain, m_buf.size());
            out.append(m_buf.left(take));
            m_buf.remove(0, take);
            m_remain -= take;
            continue;
        }
        // Skip the CRLF that ends the previous chunk, then read a size line
        qsizetype start = 0;
        while (start < m_buf.size() && (m_buf[start] == '\r' || m_buf[start] == '\n')) ++start;
        const qsizetype lineEnd = m_buf.indexOf("\r\n", start);
        if (lineEnd < 0) {
            m_buf.remove(0, start);
            break;
        }
        bool ok = false;
        const qint64 size = m_buf.mid(start, lineEnd - start).trimmed().toLongLong(&ok, 16);
        m_buf.remove(0, lineEnd + 2);
        if (!ok || size <= 0) {
            m_buf.clear();
            break;
        }
        m_remain = size;
    }
    return out;
}

} // namespace corv::rtcm
