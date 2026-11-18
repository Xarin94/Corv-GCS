#include "services/LogIndex.h"

#include "link/Mavlink.h"
#include "link/MavlinkParser.h"

#include <QHash>
#include <QRegularExpression>
#include <algorithm>
#include <array>
#include <cmath>
#include <optional>

namespace corv {

namespace {
constexpr int kTlogTsBytes = 8;   // uint64 LE microseconds since the Unix epoch
constexpr std::uint8_t kV1Stx = 0xFE, kV2Stx = 0xFD;
// Gaps over a minute between consecutive messages are clamped to 100 ms
constexpr std::int64_t kGapCompressThresholdMs = 60000;
constexpr std::int64_t kGapCompressToMs = 100;
constexpr int kTrajectorySmoothWindow = 2;
} // namespace

int frameMsgId(const std::uint8_t *p, std::uint32_t len)
{
    if (len >= 10 && p[0] == kV2Stx) return int(p[7] | (p[8] << 8) | (p[9] << 16));
    if (len >= 6 && p[0] == kV1Stx) return p[5];
    return -1;
}

std::vector<TrackPoint> smoothTrack(const std::vector<TrackPoint> &track)
{
    const int n = int(track.size()), W = kTrajectorySmoothWindow;
    if (n < 2 * W + 1) return track;
    auto out = std::vector<TrackPoint>(std::size_t(n));
    for (int i = 0; i < n; ++i) {
        double lat = 0, lon = 0, alt = 0;
        int c = 0;
        for (int k = -W; k <= W; ++k) {
            const int j = i + k;
            if (j < 0 || j >= n) continue;
            lat += track[std::size_t(j)].lat;
            lon += track[std::size_t(j)].lon;
            alt += track[std::size_t(j)].alt;
            ++c;
        }
        out[std::size_t(i)] = {lat / c, lon / c, alt / c};
    }
    return out;
}

// ── TLOG ─────────────────────────────────────────────────────────────────

ParsedLog indexTlog(const std::uint8_t *buf, qint64 size)
{
    struct Raw {
        std::optional<double> tsUs;
        qint64 offset;
        std::uint32_t len;
    };
    std::vector<Raw> raw;
    std::vector<double> firstValid;   // for the median baseline
    std::vector<TrackPoint> track;    // GLOBAL_POSITION_INT positions
    // Garbage timestamps (pre-clock frames, a never-synced ~2001 clock,
    // corrupted blocks) are left out: ≈ 2017 … 2096
    constexpr double kTsMin = 1.5e15, kTsMax = 4e18;

    qint64 offset = 0;
    while (offset + kTlogTsBytes + 3 <= size) {
        const double tsUs = double(qFromLittleEndian<std::uint64_t>(buf + offset));
        const qint64 stx = offset + kTlogTsBytes;
        std::uint32_t pktLen = 0, payloadLen = 0;
        int msgId = 0;
        qint64 payloadStart = 0;
        if (buf[stx] == kV2Stx) {
            if (stx + 10 > size) break;
            payloadLen = buf[stx + 1];
            const int signedLen = (buf[stx + 2] & 0x01) ? 13 : 0;
            pktLen = 10 + payloadLen + 2 + std::uint32_t(signedLen);
            msgId = int(buf[stx + 7] | (buf[stx + 8] << 8) | (buf[stx + 9] << 16));
            payloadStart = stx + 10;
        } else if (buf[stx] == kV1Stx) {
            if (stx + 6 > size) break;
            payloadLen = buf[stx + 1];
            pktLen = 6 + payloadLen + 2;
            msgId = buf[stx + 5];
            payloadStart = stx + 6;
        } else {
            ++offset;   // not a valid STX: resync by one byte
            continue;
        }
        if (stx + pktLen > size) break;
        const bool valid = std::isfinite(tsUs) && tsUs > kTsMin && tsUs < kTsMax;
        if (valid && firstValid.size() < 20) firstValid.push_back(tsUs);
        raw.push_back({valid ? std::optional<double>(tsUs) : std::nullopt, stx, pktLen});

        // GLOBAL_POSITION_INT: time_boot_ms, lat, lon, alt, ... (a trimmed payload reads as zeros)
        if (msgId == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
            std::uint8_t p[16] = {};
            std::memcpy(p, buf + payloadStart, std::min<std::uint32_t>(payloadLen, 16));
            const std::int32_t latE7 = qFromLittleEndian<std::int32_t>(p + 4), lonE7 = qFromLittleEndian<std::int32_t>(p + 8);
            const std::int32_t altMm = qFromLittleEndian<std::int32_t>(p + 12);
            if (latE7 != 0 || lonE7 != 0) track.push_back({latE7 / 1e7, lonE7 / 1e7, altMm / 1000.0});
        }
        offset = stx + pktLen;
    }

    // Median of the first 20 valid timestamps: robust against one-off outliers
    double minUs = 0;
    if (!firstValid.empty()) {
        std::sort(firstValid.begin(), firstValid.end());
        minUs = firstValid[firstValid.size() / 2];
    }
    // From 0; invalid stamps take the previous one; absurd forward jumps (> 1 h) become 100 ms
    constexpr std::int64_t kMaxForwardJumpMs = 3600000;
    ParsedLog out;
    out.format = QStringLiteral("tlog");
    out.index.reserve(raw.size());
    std::int64_t prev = 0;
    std::vector<std::int64_t> ts(raw.size());
    for (std::size_t i = 0; i < raw.size(); ++i) {
        std::int64_t t;
        if (!raw[i].tsUs) {
            t = prev;
        } else {
            const double v = std::floor((*raw[i].tsUs - minUs) / 1000);
            t = std::isfinite(v) && v >= 0 ? std::int64_t(v) : prev;
            if (i > 0 && t < prev) t = prev + 1;
            if (i > 0 && t > prev + kMaxForwardJumpMs) t = prev + 100;
        }
        prev = t;
        ts[i] = t;
    }
    // Gaps over a minute shrink to 100 ms. The Electron version compared each
    // stamp with the previous one already shifted, so after the first long gap
    // every message looked like another one and the times went negative: here
    // the original stamps are compared.
    std::int64_t shift = 0;
    std::vector<std::int64_t> shifted(ts.size());
    for (std::size_t i = 0; i < ts.size(); ++i) {
        if (i > 0) {
            const std::int64_t dt = ts[i] - ts[i - 1];
            if (dt > kGapCompressThresholdMs) {
                shift += dt - kGapCompressToMs;
                ++out.gapsCompressed;
            }
        }
        shifted[i] = ts[i] - shift;
    }
    for (std::size_t i = 0; i < raw.size(); ++i)
        out.index.push_back({raw[i].offset, std::uint32_t(std::max<std::int64_t>(0, shifted[i])), raw[i].len});
    out.totalMs = out.index.empty() ? 0 : out.index.back().tsMs;
    out.gpsTrack = smoothTrack(track);
    return out;
}

// ── DataFlash ────────────────────────────────────────────────────────────

namespace {

const QHash<int, QString> &buildById()
{
    static const QHash<int, QString> m = {{1, QStringLiteral("rover")}, {2, QStringLiteral("copter")}, {3, QStringLiteral("plane")},
                                          {4, QStringLiteral("tracker")}, {7, QStringLiteral("sub")}, {12, QStringLiteral("blimp")},
                                          {13, QStringLiteral("heli")}};
    return m;
}

/// MAV_TYPE the autopilot that wrote the log announces, from the firmware and its frame parameters.
int resolveMavType(const QString &build, const QHash<QString, double> &params)
{
    if (build == QLatin1String("copter")) {
        if (!params.contains(QStringLiteral("FRAME_CLASS"))) return 2;
        static const QHash<int, int> byClass = {{0, 2}, {1, 2}, {2, 13}, {5, 13}, {3, 14}, {4, 14}, {6, 4}, {11, 4}, {13, 4},
                                                {7, 15}, {8, 3}, {9, 3}, {10, 3}, {12, 29}, {14, 27}};
        return byClass.value(int(params.value(QStringLiteral("FRAME_CLASS"))), 0);
    }
    if (build == QLatin1String("heli")) return 4;
    if (build == QLatin1String("plane")) {
        const double qe = params.value(QStringLiteral("Q_ENABLE"), 0), qt = params.value(QStringLiteral("Q_MAV_TYPE"), 0);
        return qe > 0 && qt > 0 ? int(qt) : 1;
    }
    if (build == QLatin1String("rover")) return params.value(QStringLiteral("FRAME_CLASS"), 0) == 2 ? 11 : 10;
    if (build == QLatin1String("sub")) return 12;
    if (build == QLatin1String("blimp")) return 7;
    if (build == QLatin1String("tracker")) return 5;
    return 1;
}

df::Format fmtOfFmt()
{
    df::Format f;
    f.type = df::kFmtType;
    f.length = df::kFmtLength;
    f.name = "FMT";
    f.format = "BBnNZ";
    f.fields = {{"Type", 'B', 0}, {"Length", 'B', 1}, {"Name", 'n', 2}, {"Format", 'N', 6}, {"Labels", 'Z', 22}};
    return f;
}

QByteArray cstr(const std::uint8_t *p, int n)
{
    int len = 0;
    while (len < n && p[len]) ++len;
    return QByteArray(reinterpret_cast<const char *>(p), len).trimmed();
}

} // namespace

ParsedLog parseBinLog(const std::uint8_t *buf, qint64 size)
{
    static const QSet<QByteArray> whitelist = {"ATT", "AHR2", "GPS", "BARO", "ARSP", "BAT", "MODE", "MSG", "RCIN", "RCOU", "RCO2",
                                               "VIBE", "ORGN", "VER", "PARM", "NKF1", "XKF1", "GPI"};
    std::array<std::optional<df::Format>, 256> fmts;
    fmts[df::kFmtType] = fmtOfFmt();

    // Items in log order: a frame already serialized, or a GLOBAL_POSITION_INT /
    // HEARTBEAT filled in after the scan (smoothed position, vehicle type)
    struct Item {
        double tsUs;
        int kind;          // 0 frame in tmp, 1 GPI, 2 HEARTBEAT
        std::uint32_t a, b;
    };
    struct Gpi {
        std::int32_t lat, lon, alt, relAlt;
        std::int16_t vx, vy, vz;
        std::uint16_t hdg;
    };
    std::vector<Item> items;
    QByteArray tmp;
    std::vector<Gpi> gpis;
    std::vector<std::uint32_t> heartbeats;   // custom modes
    std::vector<TrackPoint> trackRaw;
    ImuCollector imu;
    std::optional<double> minUs;

    const auto push = [&](double tsUs, const mavlink_message_t &m) {
        const QByteArray f = MavlinkParser::serialize(m);
        items.push_back({tsUs, 0, std::uint32_t(tmp.size()), std::uint32_t(f.size())});
        tmp.append(f);
    };

    // NED velocity carry-over (cm/s, vz down-positive): AHR2 has position and
    // attitude but no velocity. NKF1 / XKF1 / GPI (EKF, IMU rate) preferred, GPS
    // (~5 Hz, step-wise) only until one of them is seen.
    int lastVx = 0, lastVy = 0, lastVz = 0;
    bool hasEkfVel = false;
    // VFR_HUD from several sources: airspeed ← ARSP, groundspeed / climb ← GPS, altitude / climb ← BARO
    double vfrAirspeed = 0, vfrGroundspeed = 0, vfrClimb = 0, vfrAlt = 0;
    bool hasArsp = false, hasBaro = false;
    const auto pushVfrHud = [&](double tsUs) {
        mavlink_message_t m;
        mavlink_msg_vfr_hud_pack(1, 1, &m, float(hasArsp ? vfrAirspeed : vfrGroundspeed), float(vfrGroundspeed), 0, 0, float(vfrAlt), float(vfrClimb));
        push(tsUs, m);
    };
    int gpsFix = 0, gpsNumSat = 0;
    double gpsHdop = 99.9;
    QString build;
    QHash<QString, double> params;
    std::optional<std::uint32_t> firstModeNum;
    // RCOU carries outputs 1-14 and RCO2 15-16
    std::array<std::uint16_t, 16> servo{};

    qint64 offset = 0;
    const auto nextHead = [&](qint64 from) {
        for (qint64 i = from; i + 1 < size; ++i)
            if (buf[i] == df::kHead0 && buf[i + 1] == df::kHead1) return i;
        return size;
    };
    while (offset + 3 <= size) {
        if (buf[offset] != df::kHead0 || buf[offset + 1] != df::kHead1) {
            offset = nextHead(offset + 1);
            continue;
        }
        const std::uint8_t msgType = buf[offset + 2];
        const auto &entryOpt = fmts[msgType];
        if (!entryOpt) {
            offset = nextHead(offset + 1);
            continue;
        }
        const df::Format &entry = *entryOpt;
        const int len = entry.length;
        if (len < 3 || offset + len > size) break;
        const std::uint8_t *p = buf + offset + 3;

        if (msgType == df::kFmtType) {
            const std::uint8_t type = p[0];
            df::Format f;
            f.type = type;
            f.length = p[1];
            f.name = cstr(p + 2, 4);
            f.format = cstr(p + 6, 16);
            const QByteArray labels = cstr(p + 22, 64);
            const QList<QByteArray> names = labels.isEmpty() ? QList<QByteArray>() : labels.split(',');
            int off = 0;
            for (int i = 0; i < names.size() && i < f.format.size(); ++i) {
                f.fields.push_back({names[i], f.format[i], off});
                off += df::typeSize(f.format[i]);
            }
            if (type != df::kFmtType) fmts[type] = std::move(f);
            offset += len;
            continue;
        }
        if (ImuCollector::wants(entry.name)) {
            imu.take(entry, p);
            offset += len;
            continue;
        }
        if (!whitelist.contains(entry.name)) {
            offset += len;
            continue;
        }
        const auto num = [&](const char *name) -> std::optional<double> {
            const df::Field *f = entry.field(name);
            return f ? df::number(p, *f) : std::nullopt;
        };
        const auto numOr = [&](const char *name, double def) { return num(name).value_or(def); };
        const auto tsOpt = num("TimeUS");
        if (!tsOpt) {
            offset += len;
            continue;
        }
        const double tsUs = *tsOpt;
        if (!minUs || tsUs < *minUs) minUs = tsUs;
        mavlink_message_t m;
        const QByteArray &name = entry.name;

        if (name == "ATT") {
            const double D = M_PI / 180;
            mavlink_msg_attitude_pack(1, 1, &m, std::uint32_t(std::floor(tsUs / 1000)), float(numOr("Roll", 0) * D),
                                      float(numOr("Pitch", 0) * D), float(numOr("Yaw", 0) * D), 0, 0, 0);
            push(tsUs, m);
        } else if (name == "GPS") {
            if (auto v = num("Status")) gpsFix = int(*v);
            if (auto v = num("NSats")) gpsNumSat = int(*v);
            if (auto v = num("HDop")) gpsHdop = *v;
            const auto lat = num("Lat"), lon = num("Lng");
            const double altM = numOr("Alt", 0), spd = numOr("Spd", 0), gcrs = numOr("GCrs", 0);
            // GPS VZ in the log is NED down-positive, as GLOBAL_POSITION_INT.vz
            const double vz = numOr("VZ", 0);
            const double vx = spd * std::cos(gcrs * M_PI / 180), vy = spd * std::sin(gcrs * M_PI / 180);
            if (!hasEkfVel) {
                lastVx = int(std::lround(vx * 100));
                lastVy = int(std::lround(vy * 100));
                lastVz = int(std::lround(vz * 100));
            }
            vfrGroundspeed = spd;
            if (!hasBaro) vfrClimb = -vz;
            pushVfrHud(tsUs);
            mavlink_msg_gps_raw_int_pack(1, 1, &m, std::uint64_t(tsUs), std::uint8_t(gpsFix), lat ? std::int32_t(std::lround(*lat)) : 0,
                                         lon ? std::int32_t(std::lround(*lon)) : 0, std::int32_t(std::lround(altM * 1000)),
                                         std::uint16_t(std::lround((gpsHdop ? gpsHdop : 99.9) * 100)), UINT16_MAX,
                                         std::uint16_t(std::lround(spd * 100)), std::uint16_t(std::lround(gcrs * 100)), std::uint8_t(gpsNumSat),
                                         0, 0, 0, 0, 0, 0);
            push(tsUs, m);
        } else if (name == "AHR2") {
            // Trajectory source: AHR2, at IMU rate. Boot samples near (0, 0) are
            // skipped: |raw| ≤ 100, i.e. under 1e-5° (they would draw a line
            // from off west Africa to the flight site).
            const auto lat = num("Lat"), lon = num("Lng");
            const double altM = numOr("Alt", 0), yaw = numOr("Yaw", 0);
            if (lat && lon && std::abs(*lat) > 100 && std::abs(*lon) > 100) {
                gpis.push_back({std::int32_t(std::lround(*lat)), std::int32_t(std::lround(*lon)), std::int32_t(std::lround(altM * 1000)),
                                std::int32_t(std::lround(altM * 1000)), std::int16_t(lastVx), std::int16_t(lastVy), std::int16_t(lastVz),
                                std::uint16_t(std::lround(yaw * 100))});
                items.push_back({tsUs, 1, std::uint32_t(gpis.size() - 1), 0});
                trackRaw.push_back({*lat / 1e7, *lon / 1e7, altM});
            }
        } else if (name == "NKF1" || name == "XKF1") {
            // EKF NED velocity at IMU rate, primary core only
            if (numOr("C", -1) == 0) {
                hasEkfVel = true;
                lastVx = int(std::lround(numOr("VN", 0) * 100));
                lastVy = int(std::lround(numOr("VE", 0) * 100));
                lastVz = int(std::lround(numOr("VD", 0) * 100));
            }
        } else if (name == "GPI") {
            hasEkfVel = true;
            lastVx = int(std::lround(numOr("VN", 0) * 100));
            lastVy = int(std::lround(numOr("VE", 0) * 100));
            lastVz = int(std::lround(numOr("VD", 0) * 100));
        } else if (name == "BARO") {
            hasBaro = true;
            vfrAlt = numOr("Alt", vfrAlt);
            vfrClimb = numOr("CRt", vfrClimb);
            pushVfrHud(tsUs);
        } else if (name == "ARSP") {
            hasArsp = true;
            vfrAirspeed = numOr("Airspeed", vfrAirspeed);
            pushVfrHud(tsUs);
        } else if (name == "BAT") {
            mavlink_msg_sys_status_pack(1, 1, &m, 0, 0, 0, 0, std::uint16_t(std::lround(numOr("Volt", 0) * 1000)),
                                        std::int16_t(std::lround(numOr("Curr", 0) * 100)), -1, 0, 0, 0, 0, 0, 0, 0, 0, 0);
            push(tsUs, m);
        } else if (name == "MODE") {
            const auto mode = std::uint32_t(numOr("Mode", 0));
            if (!firstModeNum) firstModeNum = mode;
            heartbeats.push_back(mode);
            items.push_back({tsUs, 2, std::uint32_t(heartbeats.size() - 1), 0});
        } else if (name == "VER") {
            const int bu = int(numOr("BU", -1));
            if (buildById().contains(bu)) build = buildById().value(bu);
        } else if (name == "PARM") {
            // Only the first value counts: the vehicle cannot change frame mid-flight
            const df::Field *nf = entry.field("Name");
            const auto v = num("Value");
            if (nf && v && std::isfinite(*v)) {
                const QString pn = df::text(p, *nf);
                if (!pn.isEmpty() && !params.contains(pn)) params.insert(pn, *v);
            }
        } else if (name == "MSG") {
            const df::Field *mf = entry.field("Message");
            const QString text = mf ? df::text(p, *mf) : QString();
            if (build.isEmpty()) {
                static const std::pair<QRegularExpression, const char *> banners[] = {
                    {QRegularExpression(QStringLiteral("^ArduCopter"), QRegularExpression::CaseInsensitiveOption), "copter"},
                    {QRegularExpression(QStringLiteral("^ArduPlane"), QRegularExpression::CaseInsensitiveOption), "plane"},
                    {QRegularExpression(QStringLiteral("^(ArduRover|APMrover2|Rover)"), QRegularExpression::CaseInsensitiveOption), "rover"},
                    {QRegularExpression(QStringLiteral("^ArduSub"), QRegularExpression::CaseInsensitiveOption), "sub"},
                    {QRegularExpression(QStringLiteral("^Blimp"), QRegularExpression::CaseInsensitiveOption), "blimp"},
                    {QRegularExpression(QStringLiteral("^AntennaTracker"), QRegularExpression::CaseInsensitiveOption), "tracker"}};
                for (const auto &[re, b] : banners)
                    if (re.match(text).hasMatch()) {
                        build = QLatin1String(b);
                        break;
                    }
            }
            if (!text.isEmpty()) {
                char t[50] = {};
                const QByteArray l1 = text.toLatin1();
                std::memcpy(t, l1.constData(), std::min<qsizetype>(l1.size(), 50));
                mavlink_msg_statustext_pack(1, 1, &m, 6, t, 0, 0);
                push(tsUs, m);
            }
        } else if (name == "RCIN") {
            const auto c = [&](const char *n) { return std::uint16_t(numOr(n, 0)); };
            mavlink_msg_rc_channels_raw_pack(1, 1, &m, 0, 0, c("C1"), c("C2"), c("C3"), c("C4"), c("C5"), c("C6"), c("C7"), c("C8"), 0);
            push(tsUs, m);
        } else if (name == "RCOU") {
            for (int ch = 1; ch <= 14; ++ch) servo[std::size_t(ch - 1)] = std::uint16_t(numOr(QByteArray("C" + QByteArray::number(ch)).constData(), 0));
            mavlink_msg_servo_output_raw_pack(1, 1, &m, 0, 0, servo[0], servo[1], servo[2], servo[3], servo[4], servo[5], servo[6], servo[7],
                                              servo[8], servo[9], servo[10], servo[11], servo[12], servo[13], servo[14], servo[15]);
            push(tsUs, m);
        } else if (name == "RCO2") {
            servo[14] = std::uint16_t(numOr("C15", 0));
            servo[15] = std::uint16_t(numOr("C16", 0));
        } else if (name == "VIBE") {
            mavlink_msg_vibration_pack(1, 1, &m, std::uint64_t(tsUs), float(numOr("VibeX", 0)), float(numOr("VibeY", 0)), float(numOr("VibeZ", 0)),
                                       std::uint32_t(numOr("Clip0", 0)), std::uint32_t(numOr("Clip1", 0)), std::uint32_t(numOr("Clip2", 0)));
            push(tsUs, m);
        } else if (name == "ORGN") {
            const auto lat = num("Lat"), lon = num("Lng");
            if (lat && lon) {
                mavlink_msg_gps_global_origin_pack(1, 1, &m, std::int32_t(std::lround(*lat)), std::int32_t(std::lround(*lon)),
                                                   std::int32_t(std::lround(numOr("Alt", 0) * 1000)), 0);
                push(tsUs, m);
            }
        }
        offset += len;
    }

    const double t0 = minUs.value_or(0);
    ParsedLog out;
    out.format = QStringLiteral("bin");
    out.mavType = resolveMavType(build, params);
    // The vehicle type at the very start too: the first MODE can be minutes in
    items.push_back({t0, 2, std::uint32_t(heartbeats.size()), 0});
    heartbeats.push_back(firstModeNum.value_or(0));

    // Smoothed positions, written back into the replayed GLOBAL_POSITION_INT
    out.gpsTrack = smoothTrack(trackRaw);
    for (std::size_t i = 0; i < out.gpsTrack.size() && i < gpis.size(); ++i) {
        gpis[i].lat = std::int32_t(std::lround(out.gpsTrack[i].lat * 1e7));
        gpis[i].lon = std::int32_t(std::lround(out.gpsTrack[i].lon * 1e7));
        gpis[i].alt = std::int32_t(std::lround(out.gpsTrack[i].alt * 1000));
        gpis[i].relAlt = gpis[i].alt;
    }

    // From 0, stable by ms
    const auto msOf = [&](double tsUs) { return std::uint32_t(std::max(0.0, std::floor((tsUs - t0) / 1000))); };
    std::stable_sort(items.begin(), items.end(), [&](const Item &a, const Item &b) { return msOf(a.tsUs) < msOf(b.tsUs); });
    out.index.reserve(items.size());
    out.frames.reserve(tmp.size() + qsizetype(gpis.size() * 40 + heartbeats.size() * 21));
    for (const Item &it : items) {
        QByteArray f;
        if (it.kind == 0) {
            f = tmp.mid(it.a, it.b);
        } else {
            mavlink_message_t m;
            if (it.kind == 1) {
                const Gpi &g = gpis[it.a];
                mavlink_msg_global_position_int_pack(1, 1, &m, 0, g.lat, g.lon, g.alt, g.relAlt, g.vx, g.vy, g.vz, g.hdg);
            } else {
                // armed + custom mode, ACTIVE: as the replayed vehicle flies
                mavlink_msg_heartbeat_pack(1, 1, &m, std::uint8_t(out.mavType), MAV_AUTOPILOT_ARDUPILOTMEGA, 209, heartbeats[it.a], MAV_STATE_ACTIVE);
            }
            f = MavlinkParser::serialize(m);
        }
        out.index.push_back({qint64(out.frames.size()), msOf(it.tsUs), std::uint32_t(f.size())});
        out.frames.append(f);
    }
    out.totalMs = out.index.empty() ? 0 : out.index.back().tsMs;
    out.imu = imu.finish(t0, params);
    return out;
}

} // namespace corv
