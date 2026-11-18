#include "sensors/LivoxProtocol.h"

#include <QString>
#include <QStringList>
#include <QtEndian>
#include <array>

namespace corv::livox {

namespace {
const std::array<std::uint16_t, 256> kCrc16 = [] {
    std::array<std::uint16_t, 256> t{};
    for (int i = 0; i < 256; ++i) {
        std::uint32_t c = std::uint32_t(i) << 8;
        for (int k = 0; k < 8; ++k) c = (c & 0x8000) ? ((c << 1) ^ 0x1021) & 0xFFFF : (c << 1) & 0xFFFF;
        t[std::size_t(i)] = std::uint16_t(c);
    }
    return t;
}();
const std::array<std::uint32_t, 256> kCrc32 = [] {
    std::array<std::uint32_t, 256> t{};
    for (std::uint32_t i = 0; i < 256; ++i) {
        std::uint32_t c = i;
        for (int k = 0; k < 8; ++k) c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        t[i] = c;
    }
    return t;
}();
std::uint32_t g_seq = 1;
} // namespace

std::uint16_t crc16(const std::uint8_t *buf, int len)
{
    std::uint32_t crc = 0xFFFF;
    for (int i = 0; i < len; ++i) crc = ((crc << 8) & 0xFFFF) ^ kCrc16[((crc >> 8) ^ buf[i]) & 0xFF];
    return std::uint16_t(crc);
}

std::uint32_t crc32(const std::uint8_t *buf, int len)
{
    std::uint32_t crc = 0xFFFFFFFFu;
    for (int i = 0; i < len; ++i) crc = kCrc32[(crc ^ buf[i]) & 0xFF] ^ (crc >> 8);
    return crc ^ 0xFFFFFFFFu;
}

QByteArray packCommand(std::uint16_t cmdId, const QByteArray &data, std::optional<std::uint32_t> seq)
{
    QByteArray buf(kSdkHeader + data.size(), '\0');
    auto *b = reinterpret_cast<std::uint8_t *>(buf.data());
    b[0] = 0xAA;   // sof
    b[1] = 0;      // protocol version
    qToLittleEndian<std::uint16_t>(std::uint16_t(kSdkHeader + data.size()), b + 2);   // whole frame length
    qToLittleEndian<std::uint32_t>((seq ? *seq : g_seq++) & 0xFFFF, b + 4);            // seq_num
    qToLittleEndian<std::uint16_t>(cmdId, b + 8);
    b[10] = 0;     // cmd_type: request
    b[11] = 0;     // sender: host
    // rsvd[6] @12
    qToLittleEndian<std::uint16_t>(crc16(b, 18), b + 18);
    qToLittleEndian<std::uint32_t>(data.isEmpty() ? 0 : crc32(reinterpret_cast<const std::uint8_t *>(data.constData()), int(data.size())), b + 20);
    std::copy(data.cbegin(), data.cend(), buf.begin() + kSdkHeader);
    return buf;
}

std::optional<Command> parseCommand(const QByteArray &buf)
{
    const auto *b = reinterpret_cast<const std::uint8_t *>(buf.constData());
    if (buf.size() < kSdkHeader || b[0] != 0xAA) return std::nullopt;
    const int length = qFromLittleEndian<std::uint16_t>(b + 2);
    if (length > buf.size() || length < kSdkHeader) return std::nullopt;
    if (crc16(b, 18) != qFromLittleEndian<std::uint16_t>(b + 18)) return std::nullopt;
    Command c;
    c.data = buf.mid(kSdkHeader, length - kSdkHeader);
    if (!c.data.isEmpty() && crc32(reinterpret_cast<const std::uint8_t *>(c.data.constData()), int(c.data.size())) != qFromLittleEndian<std::uint32_t>(b + 20))
        return std::nullopt;
    c.seq = qFromLittleEndian<std::uint32_t>(b + 4);
    c.cmdId = qFromLittleEndian<std::uint16_t>(b + 8);
    c.cmdType = b[10];
    c.senderType = b[11];
    return c;
}

QByteArray buildKvRequest(const std::vector<std::pair<std::uint16_t, QByteArray>> &entries)
{
    int size = 4;
    for (const auto &e : entries) size += 4 + int(e.second.size());
    QByteArray buf(size, '\0');
    auto *b = reinterpret_cast<std::uint8_t *>(buf.data());
    qToLittleEndian<std::uint16_t>(std::uint16_t(entries.size()), b);
    int o = 4;
    for (const auto &[key, value] : entries) {
        qToLittleEndian<std::uint16_t>(key, b + o);
        qToLittleEndian<std::uint16_t>(std::uint16_t(value.size()), b + o + 2);
        std::copy(value.cbegin(), value.cend(), buf.begin() + o + 4);
        o += 4 + int(value.size());
    }
    return buf;
}

QByteArray hostIpValue(const QString &ip, quint16 hostPort, quint16 lidarPort)
{
    QByteArray v(8, '\0');
    auto *b = reinterpret_cast<std::uint8_t *>(v.data());
    const QStringList parts = ip.split(QLatin1Char('.'));
    for (int i = 0; i < 4; ++i) b[i] = std::uint8_t(parts.value(i).toInt() & 0xFF);
    qToLittleEndian<std::uint16_t>(hostPort, b + 4);
    qToLittleEndian<std::uint16_t>(lidarPort, b + 6);
    return v;
}

QString deviceName(int devType)
{
    switch (devType) {
    case 9: return QStringLiteral("Mid-360");
    case 35: return QStringLiteral("Mid-360S");
    case 15: return QStringLiteral("HAP");
    case 40: return QStringLiteral("Avia2");
    }
    return {};
}

} // namespace corv::livox
