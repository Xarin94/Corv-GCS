#include "services/Ltm.h"

#include "vehicle/VehicleState.h"

#include <QtEndian>
#include <algorithm>
#include <cmath>

namespace corv::ltm {

namespace {

constexpr double kRad = 180.0 / M_PI;

QByteArray frame(char fn, int payloadLen)
{
    QByteArray b(3 + payloadLen + 1, '\0');
    b[0] = '$';
    b[1] = 'T';
    b[2] = fn;
    return b;
}

void putI32(QByteArray &b, int at, double v) { qToLittleEndian<qint32>(qint32(std::llround(v)), b.data() + at); }
void putI16(QByteArray &b, int at, double v) { qToLittleEndian<qint16>(qint16(std::lround(v)), b.data() + at); }
void putU16(QByteArray &b, int at, double v) { qToLittleEndian<quint16>(quint16(std::clamp<long>(std::lround(v), 0, 65535)), b.data() + at); }
uint8_t u8(double v) { return uint8_t(std::clamp<long>(std::lround(v), 0, 255)); }
double fin(double v) { return std::isfinite(v) ? v : 0; }

/// XOR of the payload (bytes 3 .. end - 1) into the last byte
QByteArray seal(QByteArray b)
{
    uint8_t crc = 0;
    for (int i = 3; i < b.size() - 1; ++i) crc ^= uint8_t(b[i]);
    b[b.size() - 1] = char(crc);
    return b;
}

} // namespace

QByteArray gFrame(const VehicleState &s)
{
    QByteArray b = frame('G', 14);
    putI32(b, 3, fin(s.lat) * 1e7);
    putI32(b, 7, fin(s.lon) * 1e7);
    b[11] = char(u8(fin(s.gs)));
    putI32(b, 12, fin(s.rawAlt) * 100);
    const int sats = std::min(63, std::max(0, s.gpsNumSat));
    const int fix = std::min(3, std::max(0, s.gpsFix));
    b[16] = char((sats << 2) | fix);
    return seal(b);
}

QByteArray aFrame(const VehicleState &s)
{
    QByteArray b = frame('A', 6);
    putI16(b, 3, fin(s.pitch) * kRad);
    putI16(b, 5, fin(s.roll) * kRad);
    long hdg = std::lround(fin(s.yaw) * kRad);
    hdg = ((hdg % 360) + 360) % 360;
    putI16(b, 7, double(hdg));
    return seal(b);
}

QByteArray sFrame(const VehicleState &s)
{
    QByteArray b = frame('S', 7);
    putU16(b, 3, fin(s.batteryVoltage) * 1000);
    putU16(b, 5, 0);   // consumed capacity: not in the basic telemetry
    b[7] = char(u8(fin(s.linkQuality)));
    b[8] = char(u8(fin(s.as)));
    b[9] = char((s.armed ? 1 : 0) | (flightMode(s.flightMode) << 2));
    return seal(b);
}

QByteArray oFrame(const VehicleState &s)
{
    QByteArray b = frame('O', 14);
    putI32(b, 3, s.homeLat.value_or(0) * 1e7);
    putI32(b, 7, s.homeLon.value_or(0) * 1e7);
    putI32(b, 11, s.homeAlt.value_or(0) * 100);
    b[15] = char(std::min(3, std::max(0, s.gpsFix)));
    b[16] = char(std::min(255, std::max(0, s.gpsNumSat)));
    return seal(b);
}

int flightMode(const QString &modeName)
{
    const QString n = modeName.toUpper();
    const auto has = [&](const char *t) { return n.contains(QLatin1String(t)); };
    if (has("MANUAL")) return 0;
    if (has("ACRO")) return 4;
    if (has("STABILIZE") || has("STAB")) return 5;
    if (has("ALT_HOLD") || has("ALT HOLD")) return 7;
    if (has("LOITER")) return 8;
    if (has("AUTO")) return 9;
    if (has("CIRCLE")) return 11;
    if (has("RTL") || has("RTH")) return 12;
    if (has("LAND")) return 14;
    if (has("FBWA") || has("FLY BY WIRE A")) return 15;
    if (has("FBWB") || has("FLY BY WIRE B")) return 16;
    if (has("CRUISE")) return 17;
    if (has("GUIDED")) return 8;
    if (has("POSHOLD")) return 8;
    return 18;   // Unknown
}

} // namespace corv::ltm
