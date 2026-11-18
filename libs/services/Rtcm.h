// Rtcm.h - the byte-level half of rtk-manager.js: RTCM3 framing, the
// GPS_RTCM_DATA fragments, the NMEA GGA a VRS caster asks for, the NTRIP
// sourcetable and the chunked transfer of NTRIP v2 casters. No I/O here.
#pragma once

#include <QByteArray>
#include <QDateTime>
#include <QList>
#include <QString>
#include <cstdint>
#include <utility>

namespace corv::rtcm {

struct Frame {
    int type = 0;        // 12-bit message number, 0 with fewer than 2 payload bytes
    int length = 0;      // payload length
    QByteArray raw;      // the whole frame: D3, length, payload, CRC-24
};

/// RTCM3 frames out of a raw byte stream: 0xD3 | 6 reserved bits + 10-bit
/// length | payload | 24-bit CRC. The CRC is not checked (the autopilot does).
class Parser
{
public:
    QList<Frame> parse(const QByteArray &data);

private:
    QByteArray m_buf;
};

/// "GPS L1 Obs", "Base Position"..., else "Type N".
QString typeName(int type);

/// GPS_RTCM_DATA payloads for one frame: (flags, data) pairs of at most 180
/// bytes. A frame that fits goes alone with flags 0; a longer one is split,
/// flags = 1 (fragmented) | fragment index << 1 | sequence id << 3, where the
/// sequence id is the low 5 bits of the count of frames seen so far.
QList<std::pair<uint8_t, QByteArray>> fragments(const QByteArray &frame, std::uint64_t frameCount);

/// "$GPGGA,...*CS\r\n" for a position (fix 1, 10 satellites, HDOP 1.0).
QByteArray buildGga(double lat, double lon, double alt, const QDateTime &utc = QDateTime::currentDateTimeUtc());

struct Mount {
    QString mountpoint, identifier, format, navSystem, country;
    double lat = 0, lon = 0;
};

/// STR; lines of a caster sourcetable. False with *error when the reply is
/// neither a 200 nor carries any STR; line.
bool parseSourcetable(const QByteArray &reply, QList<Mount> *out, QString *error);

/// Incremental decoder of HTTP chunked transfer (NTRIP v2 casters).
class Dechunker
{
public:
    QByteArray feed(const QByteArray &data);

private:
    QByteArray m_buf;
    qint64 m_remain = 0;   // bytes of chunk payload still expected
};

} // namespace corv::rtcm
