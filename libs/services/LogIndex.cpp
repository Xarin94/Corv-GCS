#include "services/LogIndex.h"

#include "link/Mavlink.h"
#include "link/MavlinkParser.h"

#include <QHash>
#include <QRegularExpression>
#include <algorithm>
#include <array>
#include <cmath>
#include <optional>

using namespace Qt::StringLiterals;

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
    out.format = u"tlog"_s;
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
    static const QHash<int, QString> m = {{1, u"rover"_s}, {2, u"copter"_s}, {3, u"plane"_s},
                                          {4, u"tracker"_s}, {7, u"sub"_s}, {12, u"blimp"_s},
                                          {13, u"heli"_s}};
    return m;
}

/// MAV_TYPE the autopilot that wrote the log announces, from the firmware and its frame parameters.
int resolveMavType(const QString &build, const QHash<QString, double> &params)
{
    if (build == "copter"_L1) {
        if (!params.contains(u"FRAME_CLASS"_s)) return 2;
        static const QHash<int, int> byClass = {{0, 2}, {1, 2}, {2, 13}, {5, 13}, {3, 14}, {4, 14}, {6, 4}, {11, 4}, {13, 4},
                                                {7, 15}, {8, 3}, {9, 3}, {10, 3}, {12, 29}, {14, 27}};
        return byClass.value(int(params.value(u"FRAME_CLASS"_s)), 0);
    }
    if (build == "heli"_L1) return 4;
    if (build == "plane"_L1) {
        const double qe = params.value(u"Q_ENABLE"_s, 0), qt = params.value(u"Q_MAV_TYPE"_s, 0);
        return qe > 0 && qt > 0 ? int(qt) : 1;
    }
    if (build == "rover"_L1) return params.value(u"FRAME_CLASS"_s, 0) == 2 ? 11 : 10;
    if (build == "sub"_L1) return 12;
    if (build == "blimp"_L1) return 7;
    if (build == "tracker"_L1) return 5;
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

/// The format an FMT record defines.
df::Format parseFmt(const std::uint8_t *p)
{
    df::Format f;
    f.type = p[0];
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
    return f;
}

/// One record and its format: fields by name.
struct Record {
    const df::Format &fmt;
    const std::uint8_t *p;

    std::optional<double> num(const char *name) const
    {
        const df::Field *f = fmt.field(name);
        return f ? df::number(p, *f) : std::nullopt;
    }
    double num(const char *name, double fallback) const { return num(name).value_or(fallback); }
    QString text(const char *name) const
    {
        const df::Field *f = fmt.field(name);
        return f ? df::text(p, *f) : QString();
    }
};

int cm(double v) { return int(std::lround(v * 100)); }

/// DataFlash records → the MAVLink frames the replay feeds the vehicle model.
class BinConverter
{
public:
    ParsedLog run(const std::uint8_t *buf, qint64 size);

private:
    // Items in log order: a frame already serialized, or a GLOBAL_POSITION_INT /
    // HEARTBEAT filled in after the scan (smoothed position, vehicle type)
    enum Kind { Frame, Position, Heartbeat };
    struct Item {
        double tsUs;
        Kind kind;
        std::uint32_t a, b;   // Frame: offset and size in m_tmp; else the index in m_gpis / m_heartbeats
    };
    struct Gpi {
        std::int32_t lat, lon, alt, relAlt;
        std::int16_t vx, vy, vz;
        std::uint16_t hdg;
    };

    void record(const QByteArray &name, const Record &r, double tsUs);
    void push(double tsUs, const mavlink_message_t &m);
    void pushVfrHud(double tsUs);
    void onGps(const Record &r, double tsUs);
    void onAhr2(const Record &r, double tsUs);
    void onMsg(const Record &r, double tsUs);
    void setVelocity(int vx, int vy, int vz);
    ParsedLog finish();

    std::vector<Item> m_items;
    QByteArray m_tmp;
    std::vector<Gpi> m_gpis;
    std::vector<std::uint32_t> m_heartbeats;   // custom modes
    std::vector<TrackPoint> m_trackRaw;
    ImuCollector m_imu;
    std::optional<double> m_minUs;
    // NED velocity carry-over (cm/s, vz down-positive): AHR2 has position and
    // attitude but no velocity. NKF1 / XKF1 / GPI (EKF, IMU rate) preferred, GPS
    // (~5 Hz, step-wise) only until one of them is seen.
    int m_vx = 0, m_vy = 0, m_vz = 0;
    bool m_hasEkfVel = false;
    // VFR_HUD from several sources: airspeed ← ARSP, groundspeed / climb ← GPS, altitude / climb ← BARO
    double m_airspeed = 0, m_groundspeed = 0, m_climb = 0, m_alt = 0;
    bool m_hasArsp = false, m_hasBaro = false;
    int m_gpsFix = 0, m_gpsNumSat = 0;
    double m_gpsHdop = 99.9;
    QString m_build;
    QHash<QString, double> m_params;
    std::optional<std::uint32_t> m_firstMode;
    std::array<std::uint16_t, 16> m_servo{};   // RCOU carries outputs 1-14, RCO2 15-16
};

ParsedLog BinConverter::run(const std::uint8_t *buf, qint64 size)
{
    static const QSet<QByteArray> whitelist = {"ATT", "AHR2", "GPS", "BARO", "ARSP", "BAT", "MODE", "MSG", "RCIN", "RCOU", "RCO2",
                                               "VIBE", "ORGN", "VER", "PARM", "NKF1", "XKF1", "GPI"};
    std::array<std::optional<df::Format>, 256> fmts;
    fmts[df::kFmtType] = fmtOfFmt();
    const auto nextHead = [&](qint64 from) {
        for (qint64 i = from; i + 1 < size; ++i)
            if (buf[i] == df::kHead0 && buf[i + 1] == df::kHead1) return i;
        return size;
    };
    qint64 offset = 0;
    while (offset + 3 <= size) {
        const std::uint8_t msgType = buf[offset + 2];
        if (buf[offset] != df::kHead0 || buf[offset + 1] != df::kHead1 || !fmts[msgType]) {
            offset = nextHead(offset + 1);
            continue;
        }
        const df::Format &entry = *fmts[msgType];
        const int len = entry.length;
        if (len < 3 || offset + len > size) break;
        const std::uint8_t *p = buf + offset + 3;
        offset += len;
        if (msgType == df::kFmtType) {
            if (df::Format f = parseFmt(p); p[0] != df::kFmtType) fmts[p[0]] = std::move(f);
        } else if (ImuCollector::wants(entry.name)) {
            m_imu.take(entry, p);
        } else if (whitelist.contains(entry.name)) {
            const Record r{entry, p};
            if (const auto ts = r.num("TimeUS")) {
                if (!m_minUs || *ts < *m_minUs) m_minUs = *ts;
                record(entry.name, r, *ts);
            }
        }
    }
    return finish();
}

void BinConverter::push(double tsUs, const mavlink_message_t &m)
{
    const QByteArray f = MavlinkParser::serialize(m);
    m_items.push_back({tsUs, Frame, std::uint32_t(m_tmp.size()), std::uint32_t(f.size())});
    m_tmp.append(f);
}

void BinConverter::pushVfrHud(double tsUs)
{
    mavlink_message_t m;
    mavlink_msg_vfr_hud_pack(1, 1, &m, float(m_hasArsp ? m_airspeed : m_groundspeed), float(m_groundspeed), 0, 0, float(m_alt), float(m_climb));
    push(tsUs, m);
}

void BinConverter::setVelocity(int vx, int vy, int vz)
{
    m_vx = vx;
    m_vy = vy;
    m_vz = vz;
}

void BinConverter::record(const QByteArray &name, const Record &r, double tsUs)
{
    mavlink_message_t m;
    if (name == "ATT") {
        const double D = M_PI / 180;
        mavlink_msg_attitude_pack(1, 1, &m, std::uint32_t(std::floor(tsUs / 1000)), float(r.num("Roll", 0) * D), float(r.num("Pitch", 0) * D),
                                  float(r.num("Yaw", 0) * D), 0, 0, 0);
        push(tsUs, m);
    } else if (name == "GPS") {
        onGps(r, tsUs);
    } else if (name == "AHR2") {
        onAhr2(r, tsUs);
    } else if (name == "GPI" || ((name == "NKF1" || name == "XKF1") && r.num("C", -1) == 0)) {
        // EKF NED velocity at IMU rate, primary core only
        m_hasEkfVel = true;
        setVelocity(cm(r.num("VN", 0)), cm(r.num("VE", 0)), cm(r.num("VD", 0)));
    } else if (name == "BARO") {
        m_hasBaro = true;
        m_alt = r.num("Alt", m_alt);
        m_climb = r.num("CRt", m_climb);
        pushVfrHud(tsUs);
    } else if (name == "ARSP") {
        m_hasArsp = true;
        m_airspeed = r.num("Airspeed", m_airspeed);
        pushVfrHud(tsUs);
    } else if (name == "BAT") {
        mavlink_msg_sys_status_pack(1, 1, &m, 0, 0, 0, 0, std::uint16_t(std::lround(r.num("Volt", 0) * 1000)), std::int16_t(cm(r.num("Curr", 0))),
                                    -1, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        push(tsUs, m);
    } else if (name == "MODE") {
        const auto mode = std::uint32_t(r.num("Mode", 0));
        if (!m_firstMode) m_firstMode = mode;
        m_heartbeats.push_back(mode);
        m_items.push_back({tsUs, Heartbeat, std::uint32_t(m_heartbeats.size() - 1), 0});
    } else if (name == "VER") {
        if (const int bu = int(r.num("BU", -1)); buildById().contains(bu)) m_build = buildById().value(bu);
    } else if (name == "PARM") {
        // Only the first value counts: the vehicle cannot change frame mid-flight
        const auto v = r.num("Value");
        const QString pn = v && std::isfinite(*v) ? r.text("Name") : QString();
        if (!pn.isEmpty() && !m_params.contains(pn)) m_params.insert(pn, *v);
    } else if (name == "MSG") {
        onMsg(r, tsUs);
    } else if (name == "RCIN") {
        const auto c = [&](const char *n) { return std::uint16_t(r.num(n, 0)); };
        mavlink_msg_rc_channels_raw_pack(1, 1, &m, 0, 0, c("C1"), c("C2"), c("C3"), c("C4"), c("C5"), c("C6"), c("C7"), c("C8"), 0);
        push(tsUs, m);
    } else if (name == "RCOU") {
        for (int ch = 1; ch <= 14; ++ch) m_servo[std::size_t(ch - 1)] = std::uint16_t(r.num(QByteArray("C" + QByteArray::number(ch)).constData(), 0));
        const auto &s = m_servo;
        mavlink_msg_servo_output_raw_pack(1, 1, &m, 0, 0, s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7], s[8], s[9], s[10], s[11], s[12], s[13], s[14], s[15]);
        push(tsUs, m);
    } else if (name == "RCO2") {
        m_servo[14] = std::uint16_t(r.num("C15", 0));
        m_servo[15] = std::uint16_t(r.num("C16", 0));
    } else if (name == "VIBE") {
        mavlink_msg_vibration_pack(1, 1, &m, std::uint64_t(tsUs), float(r.num("VibeX", 0)), float(r.num("VibeY", 0)), float(r.num("VibeZ", 0)),
                                   std::uint32_t(r.num("Clip0", 0)), std::uint32_t(r.num("Clip1", 0)), std::uint32_t(r.num("Clip2", 0)));
        push(tsUs, m);
    } else if (name == "ORGN") {
        const auto lat = r.num("Lat"), lon = r.num("Lng");
        if (!lat || !lon) return;
        mavlink_msg_gps_global_origin_pack(1, 1, &m, std::int32_t(std::lround(*lat)), std::int32_t(std::lround(*lon)),
                                           std::int32_t(std::lround(r.num("Alt", 0) * 1000)), 0);
        push(tsUs, m);
    }
}

void BinConverter::onGps(const Record &r, double tsUs)
{
    if (auto v = r.num("Status")) m_gpsFix = int(*v);
    if (auto v = r.num("NSats")) m_gpsNumSat = int(*v);
    if (auto v = r.num("HDop")) m_gpsHdop = *v;
    const auto lat = r.num("Lat"), lon = r.num("Lng");
    const double altM = r.num("Alt", 0), spd = r.num("Spd", 0), gcrs = r.num("GCrs", 0);
    // GPS VZ in the log is NED down-positive, as GLOBAL_POSITION_INT.vz
    const double vz = r.num("VZ", 0);
    if (!m_hasEkfVel) setVelocity(cm(spd * std::cos(gcrs * M_PI / 180)), cm(spd * std::sin(gcrs * M_PI / 180)), cm(vz));
    m_groundspeed = spd;
    if (!m_hasBaro) m_climb = -vz;
    pushVfrHud(tsUs);
    mavlink_message_t m;
    mavlink_msg_gps_raw_int_pack(1, 1, &m, std::uint64_t(tsUs), std::uint8_t(m_gpsFix), lat ? std::int32_t(std::lround(*lat)) : 0,
                                 lon ? std::int32_t(std::lround(*lon)) : 0, std::int32_t(std::lround(altM * 1000)),
                                 std::uint16_t(std::lround((m_gpsHdop ? m_gpsHdop : 99.9) * 100)), UINT16_MAX, std::uint16_t(cm(spd)),
                                 std::uint16_t(cm(gcrs)), std::uint8_t(m_gpsNumSat), 0, 0, 0, 0, 0, 0);
    push(tsUs, m);
}

void BinConverter::onAhr2(const Record &r, double tsUs)
{
    // Trajectory source: AHR2, at IMU rate. Boot samples near (0, 0) are
    // skipped: |raw| ≤ 100, i.e. under 1e-5° (they would draw a line
    // from off west Africa to the flight site).
    const auto lat = r.num("Lat"), lon = r.num("Lng");
    if (!lat || !lon || std::abs(*lat) <= 100 || std::abs(*lon) <= 100) return;
    const double altM = r.num("Alt", 0);
    const auto altMm = std::int32_t(std::lround(altM * 1000));
    m_gpis.push_back({std::int32_t(std::lround(*lat)), std::int32_t(std::lround(*lon)), altMm, altMm, std::int16_t(m_vx), std::int16_t(m_vy),
                      std::int16_t(m_vz), std::uint16_t(cm(r.num("Yaw", 0)))});
    m_items.push_back({tsUs, Position, std::uint32_t(m_gpis.size() - 1), 0});
    m_trackRaw.push_back({*lat / 1e7, *lon / 1e7, altM});
}

void BinConverter::onMsg(const Record &r, double tsUs)
{
    const QString text = r.text("Message");
    // The firmware banner says which vehicle wrote the log, when VER does not
    if (m_build.isEmpty()) {
        static const std::pair<QRegularExpression, const char *> banners[] = {
            {QRegularExpression(u"^ArduCopter"_s, QRegularExpression::CaseInsensitiveOption), "copter"},
            {QRegularExpression(u"^ArduPlane"_s, QRegularExpression::CaseInsensitiveOption), "plane"},
            {QRegularExpression(u"^(ArduRover|APMrover2|Rover)"_s, QRegularExpression::CaseInsensitiveOption), "rover"},
            {QRegularExpression(u"^ArduSub"_s, QRegularExpression::CaseInsensitiveOption), "sub"},
            {QRegularExpression(u"^Blimp"_s, QRegularExpression::CaseInsensitiveOption), "blimp"},
            {QRegularExpression(u"^AntennaTracker"_s, QRegularExpression::CaseInsensitiveOption), "tracker"}};
        for (const auto &[re, b] : banners)
            if (re.match(text).hasMatch()) {
                m_build = QLatin1String(b);
                break;
            }
    }
    if (text.isEmpty()) return;
    char t[50] = {};
    const QByteArray l1 = text.toLatin1();
    std::memcpy(t, l1.constData(), std::min<qsizetype>(l1.size(), 50));
    mavlink_message_t m;
    mavlink_msg_statustext_pack(1, 1, &m, 6, t, 0, 0);
    push(tsUs, m);
}

ParsedLog BinConverter::finish()
{
    const double t0 = m_minUs.value_or(0);
    ParsedLog out;
    out.format = u"bin"_s;
    out.mavType = resolveMavType(m_build, m_params);
    // The vehicle type at the very start too: the first MODE can be minutes in
    m_items.push_back({t0, Heartbeat, std::uint32_t(m_heartbeats.size()), 0});
    m_heartbeats.push_back(m_firstMode.value_or(0));

    // Smoothed positions, written back into the replayed GLOBAL_POSITION_INT
    out.gpsTrack = smoothTrack(m_trackRaw);
    for (std::size_t i = 0; i < out.gpsTrack.size() && i < m_gpis.size(); ++i) {
        Gpi &g = m_gpis[i];
        g.lat = std::int32_t(std::lround(out.gpsTrack[i].lat * 1e7));
        g.lon = std::int32_t(std::lround(out.gpsTrack[i].lon * 1e7));
        g.alt = g.relAlt = std::int32_t(std::lround(out.gpsTrack[i].alt * 1000));
    }

    // From 0, stable by ms
    const auto msOf = [&](double tsUs) { return std::uint32_t(std::max(0.0, std::floor((tsUs - t0) / 1000))); };
    std::stable_sort(m_items.begin(), m_items.end(), [&](const Item &a, const Item &b) { return msOf(a.tsUs) < msOf(b.tsUs); });
    out.index.reserve(m_items.size());
    out.frames.reserve(m_tmp.size() + qsizetype(m_gpis.size() * 40 + m_heartbeats.size() * 21));
    for (const Item &it : m_items) {
        QByteArray f;
        if (it.kind == Frame) {
            f = m_tmp.mid(it.a, it.b);
        } else {
            mavlink_message_t m;
            if (it.kind == Position) {
                const Gpi &g = m_gpis[it.a];
                mavlink_msg_global_position_int_pack(1, 1, &m, 0, g.lat, g.lon, g.alt, g.relAlt, g.vx, g.vy, g.vz, g.hdg);
            } else {
                // armed + custom mode, ACTIVE: as the replayed vehicle flies
                mavlink_msg_heartbeat_pack(1, 1, &m, std::uint8_t(out.mavType), MAV_AUTOPILOT_ARDUPILOTMEGA, 209, m_heartbeats[it.a], MAV_STATE_ACTIVE);
            }
            f = MavlinkParser::serialize(m);
        }
        out.index.push_back({qint64(out.frames.size()), msOf(it.tsUs), std::uint32_t(f.size())});
        out.frames.append(f);
    }
    out.totalMs = out.index.empty() ? 0 : out.index.back().tsMs;
    out.imu = m_imu.finish(t0, m_params);
    return out;
}

} // namespace

ParsedLog parseBinLog(const std::uint8_t *buf, qint64 size)
{
    return BinConverter().run(buf, size);
}

} // namespace corv
