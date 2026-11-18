#include "services/Rtcm.h"

#include <QHash>
#include <QRegularExpression>
#include <QStringList>
#include <QTime>
#include <algorithm>
#include <cmath>

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
        {1001, QStringLiteral("GPS L1 Obs")},
        {1002, QStringLiteral("GPS L1 Ext Obs")},
        {1003, QStringLiteral("GPS L1/L2 Obs")},
        {1004, QStringLiteral("GPS L1/L2 Ext Obs")},
        {1005, QStringLiteral("Base Position")},
        {1006, QStringLiteral("Base Position + Height")},
        {1007, QStringLiteral("Antenna Descriptor")},
        {1008, QStringLiteral("Antenna Serial")},
        {1009, QStringLiteral("GLONASS L1 Obs")},
        {1010, QStringLiteral("GLONASS L1 Ext Obs")},
        {1011, QStringLiteral("GLONASS L1/L2 Obs")},
        {1012, QStringLiteral("GLONASS L1/L2 Ext Obs")},
        {1033, QStringLiteral("Receiver/Antenna Info")},
        {1074, QStringLiteral("GPS MSM4")},
        {1077, QStringLiteral("GPS MSM7")},
        {1084, QStringLiteral("GLONASS MSM4")},
        {1087, QStringLiteral("GLONASS MSM7")},
        {1094, QStringLiteral("Galileo MSM4")},
        {1097, QStringLiteral("Galileo MSM7")},
        {1124, QStringLiteral("BeiDou MSM4")},
        {1127, QStringLiteral("BeiDou MSM7")},
        {1230, QStringLiteral("GLONASS Code-Phase Bias")},
        {4072, QStringLiteral("u-blox Proprietary")}};
    return names.value(type, QStringLiteral("Type %1").arg(type));
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
    const QString time = QStringLiteral("%1%2%3.00").arg(t.hour(), 2, 10, QLatin1Char('0')).arg(t.minute(), 2, 10, QLatin1Char('0'))
                             .arg(t.second(), 2, 10, QLatin1Char('0'));
    const double latAbs = std::abs(lat), lonAbs = std::abs(lon);
    const int latDeg = int(std::floor(latAbs)), lonDeg = int(std::floor(lonAbs));
    const QString latStr = QStringLiteral("%1%2").arg(latDeg, 2, 10, QLatin1Char('0')).arg((latAbs - latDeg) * 60, 10, 'f', 7, QLatin1Char('0'));
    const QString lonStr = QStringLiteral("%1%2").arg(lonDeg, 3, 10, QLatin1Char('0')).arg((lonAbs - lonDeg) * 60, 10, 'f', 7, QLatin1Char('0'));
    const QString body = QStringLiteral("GPGGA,%1,%2,%3,%4,%5,1,10,1.0,%6,M,0.0,M,,")
                             .arg(time, latStr, lat >= 0 ? QStringLiteral("N") : QStringLiteral("S"), lonStr,
                                  lon >= 0 ? QStringLiteral("E") : QStringLiteral("W"))
                             .arg(alt, 0, 'f', 1);
    uint8_t cs = 0;
    for (QChar c : body) cs ^= uint8_t(c.unicode());
    return QStringLiteral("$%1*%2\r\n").arg(body, QStringLiteral("%1").arg(cs, 2, 16, QLatin1Char('0')).toUpper()).toLatin1();
}

bool parseSourcetable(const QByteArray &reply, QList<Mount> *out, QString *error)
{
    const QString text = QString::fromUtf8(reply);
    const QString statusLine = text.section(QStringLiteral("\r\n"), 0, 0).trimmed();
    if (!statusLine.contains(QLatin1String("200")) && !text.contains(QLatin1String("STR;"))) {
        if (error) *error = QStringLiteral("Sourcetable request failed: %1").arg(statusLine.isEmpty() ? QStringLiteral("no response") : statusLine);
        return false;
    }
    out->clear();
    static const QRegularExpression nl(QStringLiteral("\r?\n"));
    for (const QString &line : text.split(nl)) {
        if (!line.startsWith(QLatin1String("STR;"))) continue;
        const QStringList f = line.split(QLatin1Char(';'));
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
