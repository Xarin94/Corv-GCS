// DataFlash.h - ArduPilot DataFlash (.bin) records: the FMT description of a
// message type and reading its fields (format characters of AP_Logger).
//
// Record: 0xA3 0x95 <msgType:u8> <payload...>; self-describing via FMT records
// (msgType 128): { type:u8, length:u8, name:char[4], format:char[16], labels:char[64] }.
#pragma once

#include <QByteArray>
#include <QList>
#include <QString>
#include <QtEndian>
#include <cstdint>
#include <cstring>
#include <optional>
#include <vector>

namespace corv::df {

constexpr std::uint8_t kHead0 = 0xA3, kHead1 = 0x95;
constexpr std::uint8_t kFmtType = 128;
constexpr int kFmtLength = 89;

inline int typeSize(char c)
{
    switch (c) {
    case 'b': case 'B': case 'M': return 1;
    case 'h': case 'H': case 'c': case 'C': return 2;
    case 'i': case 'I': case 'f': case 'e': case 'E': case 'L': case 'n': return 4;
    case 'd': case 'q': case 'Q': return 8;
    case 'N': return 16;
    case 'Z': case 'a': return 64;
    default: return 0;
    }
}

struct Field {
    QByteArray name;
    char type = 0;
    int offset = 0;   // in the payload (after the 3-byte header)
};

struct Format {
    std::uint8_t type = 0;
    int length = 0;   // whole record, header included
    QByteArray name;
    QByteArray format;
    std::vector<Field> fields;

    const Field *field(const char *n) const
    {
        for (const Field &f : fields)
            if (f.name == n) return &f;
        return nullptr;
    }
    int offsetOf(const char *n) const
    {
        const Field *f = field(n);
        return f ? f->offset : -1;
    }
};

/// A numeric field, scaled as AP_Logger formats say (c, C, e, E: × 0.01).
inline std::optional<double> number(const std::uint8_t *p, const Field &f)
{
    const auto *b = p + f.offset;
    switch (f.type) {
    case 'b': return double(std::int8_t(b[0]));
    case 'B': case 'M': return double(b[0]);
    case 'h': return double(qFromLittleEndian<std::int16_t>(b));
    case 'H': return double(qFromLittleEndian<std::uint16_t>(b));
    case 'i': case 'L': return double(qFromLittleEndian<std::int32_t>(b));
    case 'I': return double(qFromLittleEndian<std::uint32_t>(b));
    case 'f': return double(qFromLittleEndian<float>(b));
    case 'd': return qFromLittleEndian<double>(b);
    case 'q': return double(qFromLittleEndian<std::int64_t>(b));
    case 'Q': return double(qFromLittleEndian<std::uint64_t>(b));
    case 'c': return qFromLittleEndian<std::int16_t>(b) * 0.01;
    case 'C': return qFromLittleEndian<std::uint16_t>(b) * 0.01;
    case 'e': return qFromLittleEndian<std::int32_t>(b) * 0.01;
    case 'E': return qFromLittleEndian<std::uint32_t>(b) * 0.01;
    default: return std::nullopt;
    }
}

inline QString text(const std::uint8_t *p, const Field &f)
{
    const int n = typeSize(f.type);
    if (f.type != 'n' && f.type != 'N' && f.type != 'Z') return {};
    const char *s = reinterpret_cast<const char *>(p + f.offset);
    int len = 0;
    while (len < n && s[len]) ++len;
    return QString::fromLatin1(s, len);
}

/// uint64 little-endian (µs since boot) as a double: exact below 2^53.
inline double u64(const std::uint8_t *p) { return double(qFromLittleEndian<std::uint64_t>(p)); }

} // namespace corv::df
