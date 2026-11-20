#include "link/MspSession.h"

#include "core/Clock.h"
#include "core/DebugLog.h"

#include <QHash>
#include <QtEndian>
#include <array>
#include <cmath>

using namespace Qt::StringLiterals;

namespace corv {

namespace {
// MSP command ids
constexpr int MSP_FC_VARIANT = 2;
constexpr int MSP_FC_VERSION = 3;
constexpr int MSP_RAW_IMU = 102;
constexpr int MSP_RC = 105;
constexpr int MSP_RAW_GPS = 106;
constexpr int MSP_COMP_GPS = 107;
constexpr int MSP_ATTITUDE = 108;
constexpr int MSP_ALTITUDE = 109;
constexpr int MSP_ANALOG = 110;
constexpr int MSP_BOXNAMES = 116;
constexpr int MSP_BOXIDS = 119;
constexpr int MSP_STATUS_EX = 150;
constexpr int MSP2_INAV_STATUS = 0x2000;
constexpr int MSP2_INAV_ANALOG = 0x2002;
constexpr int MSP2_INAV_AIR_SPEED = 0x2009;
constexpr int MSP_WP_MISSION_SAVE = 19;
constexpr int MSP_WP_GETINFO = 20;
constexpr int MSP_WP = 118;
constexpr int MSP_SET_WP = 209;

constexpr int kTickMs = 20;
constexpr int kMaxMisses = 3;
constexpr int kWpRecordLen = 21;
constexpr int kMissionTimeoutMs = 1500;
constexpr int kMissionTries = 3;
constexpr uint8_t kSys = 1, kComp = 1;
const double kDeg2Rad = M_PI / 180;

template <typename T>
T rd(const QByteArray &p, int off)
{
    return qFromLittleEndian<T>(p.constData() + off);
}

int clampI16(int v) { return std::clamp(v, -32768, 32767); }

/// Synthetic INAV mode numbers: the mode table maps custom_mode to a name, so they only need to be stable.
quint32 inavModeNumber(const QString &m)
{
    static const QHash<QString, quint32> n = {{u"MANUAL"_s, 0}, {u"ACRO"_s, 1}, {u"ANGLE"_s, 2},
                                              {u"HORIZON"_s, 3}, {u"ALTHOLD"_s, 4}, {u"POSHOLD"_s, 5},
                                              {u"COURSE HOLD"_s, 6}, {u"CRUISE"_s, 7}, {u"WAYPOINT"_s, 8},
                                              {u"RTH"_s, 9}, {u"FAILSAFE"_s, 10}, {u"ARMED"_s, 1},
                                              {u"DISARMED"_s, 0}};
    return n.value(m, 0);
}
} // namespace

uint8_t MspSession::crc8DvbS2(uint8_t crc, uint8_t byte)
{
    crc ^= byte;
    for (int i = 0; i < 8; ++i) crc = (crc & 0x80) ? uint8_t((crc << 1) ^ 0xD5) : uint8_t(crc << 1);
    return crc;
}

QByteArray MspSession::encodeRequest(int cmd, const QByteArray &payload)
{
    QByteArray b;
    if (cmd > 254 || payload.size() > 254) {
        // MSP v2
        b.append("$X<", 3);
        b.append(char(0));
        b.append(char(cmd & 0xff)).append(char((cmd >> 8) & 0xff));
        b.append(char(payload.size() & 0xff)).append(char((payload.size() >> 8) & 0xff));
        b.append(payload);
        uint8_t crc = 0;
        for (int i = 3; i < b.size(); ++i) crc = crc8DvbS2(crc, uint8_t(b[i]));
        b.append(char(crc));
        return b;
    }
    b.append("$M<", 3);
    b.append(char(payload.size()));
    b.append(char(cmd));
    b.append(payload);
    uint8_t x = uint8_t(payload.size()) ^ uint8_t(cmd);
    for (char c : payload) x ^= uint8_t(c);
    b.append(char(x));
    return b;
}

QByteArray MspSession::encodeWaypoint(const MspWaypoint &wp)
{
    QByteArray b(kWpRecordLen, '\0');
    char *d = b.data();
    d[0] = char(wp.index & 0xff);
    d[1] = char(wp.action & 0xff);
    qToLittleEndian<qint32>(wp.lat, d + 2);
    qToLittleEndian<qint32>(wp.lon, d + 6);
    qToLittleEndian<qint32>(wp.alt, d + 10);
    qToLittleEndian<qint16>(qint16(clampI16(wp.p1)), d + 14);
    qToLittleEndian<qint16>(qint16(clampI16(wp.p2)), d + 16);
    qToLittleEndian<qint16>(qint16(clampI16(wp.p3)), d + 18);
    d[20] = char(wp.flag & 0xff);
    return b;
}

bool MspSession::decodeWaypoint(const QByteArray &b, MspWaypoint *wp)
{
    if (b.size() < kWpRecordLen) return false;
    wp->index = uint8_t(b[0]);
    wp->action = uint8_t(b[1]);
    wp->lat = rd<qint32>(b, 2);
    wp->lon = rd<qint32>(b, 6);
    wp->alt = rd<qint32>(b, 10);
    wp->p1 = rd<qint16>(b, 14);
    wp->p2 = rd<qint16>(b, 16);
    wp->p3 = rd<qint16>(b, 18);
    wp->flag = uint8_t(b[20]);
    return true;
}

MspSession::MspSession(WriteFn write, MessageFn message, const QString &profile, QObject *parent)
    : QObject(parent), m_write(std::move(write)), m_message(std::move(message))
{
    const bool slow = profile == "slow"_L1;
    // Poll rates; "slow" for long-range radio links where 25 Hz attitude would use the whole budget
    if (slow) {
        m_schedule = {{MSP_ATTITUDE, 5}, {MSP_ALTITUDE, 2}, {MSP_RAW_GPS, 2}, {MSP2_INAV_STATUS, 1}, {MSP2_INAV_ANALOG, 0.5}, {MSP_COMP_GPS, 0.5}};
    } else {
        m_schedule = {{MSP_ATTITUDE, 25}, {MSP_ALTITUDE, 10}, {MSP_RAW_GPS, 5}, {MSP2_INAV_STATUS, 5}, {MSP_RAW_IMU, 5},
                      {MSP_RC, 5}, {MSP_COMP_GPS, 2}, {MSP2_INAV_ANALOG, 2}, {MSP2_INAV_AIR_SPEED, 2}};
    }
    // A radio link has real latency; USB and TCP answer in milliseconds
    m_replyTimeoutMs = slow ? 2000 : 500;
    // Identify the FC and learn the box names before polling: without them the mode cannot be resolved
    for (const int cmd : {MSP_FC_VARIANT, MSP_FC_VERSION, MSP_BOXNAMES, MSP_BOXIDS}) m_queue.push_back(Job{cmd});
    m_timeout.setSingleShot(true);
    connect(&m_timeout, &QTimer::timeout, this, &MspSession::onTimeout);
    m_tick.setInterval(kTickMs);
    connect(&m_tick, &QTimer::timeout, this, &MspSession::tick);
    m_tick.start();
}

MspSession::~MspSession()
{
    // Whoever waits on a transfer hears about the link going away
    std::vector<std::function<void(const QString &)>> orphans;
    for (const Job &j : m_queue)
        if (j.fail) orphans.push_back(j.fail);
    if (m_pending && m_pendingJob.fail) orphans.push_back(m_pendingJob.fail);
    m_queue.clear();
    m_pending = false;
    for (auto &f : orphans) f(u"MSP link closed during the transfer"_s);
}

void MspSession::push(const QByteArray &bytes)
{
    m_rx.append(bytes);
    const auto *b = reinterpret_cast<const uint8_t *>(m_rx.constData());
    const qsizetype n = m_rx.size();
    qsizetype i = 0;
    while (i < n) {
        if (b[i] != '$') { ++i; continue; }   // resynchronise on '$'
        if (i + 2 >= n) break;
        const uint8_t version = b[i + 1], dir = b[i + 2];
        if (version == 'M') {
            if (i + 5 > n) break;
            const int size = b[i + 3], cmd = b[i + 4];
            const qsizetype total = 6 + size;
            if (i + total > n) break;
            uint8_t x = uint8_t(size) ^ uint8_t(cmd);
            for (int k = 0; k < size; ++k) x ^= b[i + 5 + k];
            if (x == b[i + 5 + size]) {
                const QByteArray payload = m_rx.mid(i + 5, size);
                if (dir != '!') reply(cmd, payload);
                else rejected(cmd);
                i += total;
                continue;
            }
            ++i;   // bad checksum: the '$' was noise
            continue;
        }
        if (version == 'X') {
            if (i + 8 > n) break;
            const int cmd = b[i + 4] | (b[i + 5] << 8);
            const int size = b[i + 6] | (b[i + 7] << 8);
            const qsizetype total = 9 + size;
            if (i + total > n) break;
            uint8_t crc = 0;
            for (qsizetype k = i + 3; k < i + 8 + size; ++k) crc = crc8DvbS2(crc, b[k]);
            if (crc == b[i + 8 + size]) {
                const QByteArray payload = m_rx.mid(i + 8, size);
                if (dir != '!') reply(cmd, payload);
                else rejected(cmd);
                i += total;
                continue;
            }
            ++i;
            continue;
        }
        ++i;   // unknown version byte
    }
    if (i > 0) m_rx.remove(0, i);
    // Never let garbage pile up without a valid frame
    if (m_rx.size() > 8192) m_rx = m_rx.right(1024);
}

void MspSession::send(const Job &job)
{
    m_pending = true;
    m_pendingJob = job;
    m_timeout.start(job.timeout ? job.timeout : m_replyTimeoutMs);
    if (m_write) m_write(encodeRequest(job.cmd, job.payload));
}

void MspSession::tick()
{
    if (m_pending) return;
    // One-shot jobs (identification, mission transfer) first
    if (!m_queue.empty()) {
        const Job j = m_queue.front();
        m_queue.pop_front();
        send(j);
        return;
    }
    if (m_missionBusy) return;
    const qint64 now = wallMs();
    const Entry *best = nullptr;
    qint64 bestLate = -1;
    for (const Entry &e : m_schedule) {
        const qint64 late = now - m_nextDue.value(e.cmd, 0);
        if (late >= 0 && late > bestLate) { best = &e; bestLate = late; }
    }
    if (!best) return;
    m_nextDue[best->cmd] = now + qint64(1000 / best->hz);
    send(Job{best->cmd});
}

void MspSession::onTimeout()
{
    // A dropped reply must not wedge the scheduler
    if (!m_pending) return;
    m_pending = false;
    Job job = m_pendingJob;
    if (job.fail) {
        // A mission record is worth retrying: one frame lost on a radio must not cost the upload
        if (--job.tries > 0) {
            m_queue.push_front(job);
            return;
        }
        job.fail(u"No reply to MSP command %1"_s.arg(job.cmd));
        return;
    }
    const int n = ++m_misses[job.cmd];
    if (n >= kMaxMisses) {
        const qsizetype before = m_schedule.size();
        m_schedule.removeIf([&](const Entry &e) { return e.cmd == job.cmd; });
        if (m_schedule.size() != before)
            LOG_W("msp") << u"command 0x%1 unanswered %2x: removed from the poll schedule"_s.arg(job.cmd, 0, 16).arg(n);
    }
}

void MspSession::reply(int cmd, const QByteArray &payload)
{
    std::function<void(const QByteArray &)> ok;
    if (m_pending && m_pendingJob.cmd == cmd) {
        m_timeout.stop();
        m_pending = false;
        ok = m_pendingJob.ok;
    }
    m_misses[cmd] = 0;
    decode(cmd, payload);
    if (ok) ok(payload);
}

void MspSession::rejected(int cmd)
{
    if (!m_pending || m_pendingJob.cmd != cmd) return;
    m_timeout.stop();
    m_pending = false;
    if (m_pendingJob.fail) m_pendingJob.fail(u"Flight controller rejected MSP command %1"_s.arg(cmd));
}

void MspSession::request(int cmd, const QByteArray &payload, int timeoutMs, std::function<void(const QByteArray &)> ok,
                         std::function<void(const QString &)> fail)
{
    m_queue.push_back(Job{cmd, payload, std::move(ok), std::move(fail), kMissionTries, std::max(timeoutMs, m_replyTimeoutMs)});
}

void MspSession::decode(int cmd, const QByteArray &p)
{
    switch (cmd) {
    case MSP_FC_VARIANT:
        m_fcVariant = QString::fromLatin1(p.left(4));
        break;
    case MSP_FC_VERSION:
        if (p.size() >= 3) {
            m_fcVersion = u"%1.%2.%3"_s.arg(uint8_t(p[0])).arg(uint8_t(p[1])).arg(uint8_t(p[2]));
            LOG_I("msp") << "flight controller" << m_fcVariant << m_fcVersion;
        }
        break;
    case MSP_ATTITUDE:
        // roll / pitch in decidegrees, yaw in whole degrees
        if (p.size() < 6) return;
        m_roll = rd<qint16>(p, 0) / 10.0;
        m_pitch = rd<qint16>(p, 2) / 10.0;
        m_yaw = rd<qint16>(p, 4);
        emitAttitude();
        break;
    case MSP_ALTITUDE:
        if (p.size() < 6) return;
        m_altBaro = rd<qint32>(p, 0) / 100.0;
        m_climb = rd<qint16>(p, 4) / 100.0;
        emitVfrHud();
        break;
    case MSP_RAW_GPS:
        if (p.size() < 16) return;
        m_fixType = uint8_t(p[0]);
        m_numSat = uint8_t(p[1]);
        m_lat = rd<qint32>(p, 2);
        m_lon = rd<qint32>(p, 6);
        m_altGps = rd<qint16>(p, 10);
        m_groundspeed = rd<quint16>(p, 12) / 100.0;
        m_gpsCourse = rd<quint16>(p, 14) / 10.0;
        if (p.size() >= 18) m_hdop = rd<quint16>(p, 16);
        emitGps();
        emitGlobalPosition();
        break;
    case MSP_RAW_IMU:
        if (p.size() < 18) return;
        // INAV reports acceleration in 1/512 g
        m_accX = rd<qint16>(p, 0) / 512.0;
        m_accY = rd<qint16>(p, 2) / 512.0;
        m_accZ = rd<qint16>(p, 4) / 512.0;
        m_gyroX = rd<qint16>(p, 6);
        m_gyroY = rd<qint16>(p, 8);
        m_gyroZ = rd<qint16>(p, 10);
        emitImu();
        break;
    case MSP_ANALOG:
        if (p.size() < 7) return;
        m_voltage = uint8_t(p[0]) / 10.0;
        m_mahDrawn = rd<quint16>(p, 1);
        m_rssi = rd<quint16>(p, 3);
        m_current = rd<qint16>(p, 5) / 100.0;
        emitSysStatus();
        break;
    case MSP2_INAV_ANALOG:
        if (p.size() < 23) return;
        m_voltage = rd<quint16>(p, 1) / 100.0;
        m_current = rd<qint16>(p, 3) / 100.0;
        m_mahDrawn = rd<quint32>(p, 9);
        m_batteryPercent = uint8_t(p[21]);
        if (p.size() >= 24) m_rssi = rd<quint16>(p, 22);
        emitSysStatus();
        break;
    case MSP2_INAV_AIR_SPEED:
        if (p.size() < 4) return;
        m_airspeed = rd<quint32>(p, 0) / 100.0;
        emitVfrHud();
        break;
    case MSP_STATUS_EX:
    case MSP2_INAV_STATUS: {
        if (p.size() < 11) return;
        std::array<quint32, 4> boxes{};
        if (cmd == MSP2_INAV_STATUS && p.size() >= 25) {
            // cycleTime u16, i2cErr u16, sensors u16, cpuLoad u16, profile u8, armingFlags u32, boxModeFlags 4 x u32
            const quint32 armingFlags = rd<quint32>(p, 9);
            m_armed = (armingFlags & 0x04) != 0;   // bit 2 is ARMED, the others are reasons it cannot arm
            for (int i = 0; i < 4; ++i)
                if (13 + i * 4 + 4 <= p.size()) boxes[std::size_t(i)] = rd<quint32>(p, 13 + i * 4);
        } else {
            boxes[0] = rd<quint32>(p, 6);
            m_armed = (boxes[0] & 1) != 0;   // box 0 = ARM on Betaflight
        }
        m_mode = modeName(boxes);
        emitHeartbeat();
        break;
    }
    case MSP_BOXNAMES:
        m_boxNames = QString::fromLatin1(p).split(';'_L1, Qt::SkipEmptyParts);
        break;
    case MSP_RC: {
        m_rc.clear();
        for (int i = 0; i + 1 < p.size(); i += 2) m_rc << rd<quint16>(p, i);
        emitRc();
        break;
    }
    default:
        break;
    }
}

QString MspSession::modeName(const std::array<quint32, 4> &boxes) const
{
    // INAV / Betaflight have no mode enum: the first active box in the order the OSD shows them
    static const QList<QPair<QString, QString>> priority = {
        {u"FAILSAFE"_s, u"FAILSAFE"_s}, {u"NAV RTH"_s, u"RTH"_s},
        {u"NAV WP"_s, u"WAYPOINT"_s}, {u"NAV CRUISE"_s, u"CRUISE"_s},
        {u"NAV COURSE HOLD"_s, u"COURSE HOLD"_s}, {u"NAV POSHOLD"_s, u"POSHOLD"_s},
        {u"NAV ALTHOLD"_s, u"ALTHOLD"_s}, {u"ANGLE"_s, u"ANGLE"_s},
        {u"HORIZON"_s, u"HORIZON"_s}, {u"MANUAL"_s, u"MANUAL"_s},
        {u"ACRO"_s, u"ACRO"_s}};
    if (m_boxNames.isEmpty()) return m_armed ? u"ARMED"_s : u"DISARMED"_s;
    auto active = [&](const QString &name) {
        const int idx = int(m_boxNames.indexOf(name));
        if (idx < 0 || idx >= 128) return false;
        return ((boxes[std::size_t(idx / 32)] >> (idx % 32)) & 1u) != 0;
    };
    for (const auto &[box, label] : priority)
        if (active(box)) return label;
    return u"ACRO"_s;
}

// ── Synthetic MAVLink ────────────────────────────────────────────────────────

void MspSession::emitAttitude()
{
    mavlink_message_t m;
    mavlink_msg_attitude_pack(kSys, kComp, &m, 0, float(m_roll * kDeg2Rad), float(m_pitch * kDeg2Rad), float(m_yaw * kDeg2Rad), 0, 0, 0);
    if (m_message) m_message(m);
}

void MspSession::emitVfrHud()
{
    mavlink_message_t m;
    mavlink_msg_vfr_hud_pack(kSys, kComp, &m, float(m_airspeed > 0 ? m_airspeed : m_groundspeed), float(m_groundspeed),
                             int16_t(std::lround(m_yaw)), 0, float(m_altBaro), float(m_climb));
    if (m_message) m_message(m);
}

void MspSession::emitGps()
{
    // MSP fix: 0 none, 1 2D, 2 3D -> GPS_FIX_TYPE
    const uint8_t fix = m_fixType >= 2 ? 3 : m_fixType == 1 ? 2 : 1;
    mavlink_message_t m;
    mavlink_msg_gps_raw_int_pack(kSys, kComp, &m, 0, fix, m_lat, m_lon, qint32(std::lround(m_altGps * 1000)), uint16_t(m_hdop), UINT16_MAX,
                                 uint16_t(std::lround(m_groundspeed * 100)), uint16_t(std::lround(m_gpsCourse * 100)), uint8_t(m_numSat),
                                 0, 0, 0, 0, 0, 0);
    if (m_message) m_message(m);
}

void MspSession::emitGlobalPosition()
{
    // Ground track as NE velocity: the flight-path marker and the predictor read these
    const double c = m_gpsCourse * kDeg2Rad;
    mavlink_message_t m;
    mavlink_msg_global_position_int_pack(kSys, kComp, &m, 0, m_lat, m_lon, qint32(std::lround((m_altBaro != 0 ? m_altBaro : m_altGps) * 1000)),
                                         qint32(std::lround(m_altBaro * 1000)), int16_t(std::lround(m_groundspeed * std::cos(c) * 100)),
                                         int16_t(std::lround(m_groundspeed * std::sin(c) * 100)), int16_t(std::lround(-m_climb * 100)),
                                         uint16_t(std::lround(std::fmod(m_yaw + 360, 360) * 100)));
    if (m_message) m_message(m);
}

void MspSession::emitImu()
{
    mavlink_message_t m;
    mavlink_msg_raw_imu_pack(kSys, kComp, &m, 0, int16_t(std::lround(m_accX * 1000)), int16_t(std::lround(m_accY * 1000)),
                             int16_t(std::lround(m_accZ * 1000)), int16_t(m_gyroX), int16_t(m_gyroY), int16_t(m_gyroZ), 0, 0, 0, 0, 0);
    if (m_message) m_message(m);
}

void MspSession::emitSysStatus()
{
    mavlink_message_t m;
    mavlink_msg_sys_status_pack(kSys, kComp, &m, 0, 0, 0, 0, uint16_t(std::lround(m_voltage * 1000)), int16_t(std::lround(m_current * 100)),
                                int8_t(m_batteryPercent ? m_batteryPercent : -1), 0, 0, 0, 0, 0, 0, 0, 0, 0);
    if (m_message) m_message(m);
    uint16_t cells[10];
    std::fill(std::begin(cells), std::end(cells), UINT16_MAX);
    cells[0] = uint16_t(std::lround(m_voltage * 1000));
    const uint16_t ext[4] = {0, 0, 0, 0};
    mavlink_msg_battery_status_pack(kSys, kComp, &m, 0, 0, 0, INT16_MAX, cells, int16_t(std::lround(m_current * 100)), qint32(m_mahDrawn), -1,
                                    int8_t(m_batteryPercent ? m_batteryPercent : -1), 0, 0, ext, 0, 0);
    if (m_message) m_message(m);
}

void MspSession::emitRc()
{
    uint16_t c[18];
    for (int i = 0; i < 18; ++i) c[i] = i < m_rc.size() ? uint16_t(m_rc[i]) : UINT16_MAX;
    mavlink_message_t m;
    mavlink_msg_rc_channels_pack(kSys, kComp, &m, 0, uint8_t(m_rc.size()), c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7], c[8], c[9], c[10],
                                 c[11], c[12], c[13], c[14], c[15], c[16], c[17], uint8_t(std::min(255, m_rssi / 4)));
    if (m_message) m_message(m);
}

void MspSession::emitHeartbeat()
{
    mavlink_message_t m;
    mavlink_msg_heartbeat_pack(kSys, kComp, &m, MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_GENERIC, m_armed ? 209 : 81, inavModeNumber(m_mode),
                               m_armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY);
    if (m_message) m_message(m);
}

// ── INAV missions ────────────────────────────────────────────────────────────

void MspSession::missionUpload(const QList<MspWaypoint> &wpsIn, bool save)
{
    if (m_missionBusy) { emit missionDone(u"upload"_s, false, u"A mission transfer is already running"_s, {}); return; }
    if (wpsIn.isEmpty()) { emit missionDone(u"upload"_s, false, u"No waypoints to upload"_s, {}); return; }
    m_missionBusy = true;
    auto wps = std::make_shared<QList<MspWaypoint>>(wpsIn);
    // The board knows its own limit: refuse before writing half a mission
    request(MSP_WP_GETINFO, {}, kMissionTimeoutMs,
            [this, wps, save](const QByteArray &p) {
                const int maxWp = p.size() >= 4 ? uint8_t(p[1]) : 0;
                if (maxWp && wps->size() > maxWp) {
                    m_missionBusy = false;
                    emit missionDone(u"upload"_s, false,
                                     u"%1 waypoints — this board stores at most %2"_s.arg(wps->size()).arg(maxWp), {{u"maxWaypoints"_s, maxWp}});
                    return;
                }
                uploadNext(wps, 0, save, maxWp);
            },
            [this, wps, save](const QString &) { uploadNext(wps, 0, save, 0); });
}

void MspSession::uploadNext(std::shared_ptr<QList<MspWaypoint>> wps, int i, bool save, int maxWaypoints)
{
    auto fail = [this](const QString &e) {
        m_missionBusy = false;
        emit missionDone(u"upload"_s, false, e, {});
    };
    if (i >= wps->size()) {
        auto finish = [this, wps, save, maxWaypoints] {
            m_missionBusy = false;
            emit missionDone(u"upload"_s, true, {},
                             {{u"count"_s, int(wps->size())}, {u"saved"_s, save}, {u"maxWaypoints"_s, maxWaypoints}});
        };
        // The list lives in RAM until it is written to the board's storage
        if (save) request(MSP_WP_MISSION_SAVE, QByteArray(1, '\0'), 4000, [finish](const QByteArray &) { finish(); }, fail);
        else finish();
        return;
    }
    MspWaypoint wp = (*wps)[i];
    wp.index = i + 1;
    request(MSP_SET_WP, encodeWaypoint(wp), kMissionTimeoutMs,
            [this, wps, i, save, maxWaypoints](const QByteArray &) {
                emit missionProgress(u"upload"_s, i + 1, int(wps->size()));
                uploadNext(wps, i + 1, save, maxWaypoints);
            },
            fail);
}

void MspSession::missionDownload()
{
    if (m_missionBusy) { emit missionDone(u"download"_s, false, u"A mission transfer is already running"_s, {}); return; }
    m_missionBusy = true;
    request(MSP_WP_GETINFO, {}, kMissionTimeoutMs,
            [this](const QByteArray &p) {
                if (p.size() < 4) {
                    m_missionBusy = false;
                    emit missionDone(u"download"_s, false, u"Short MSP_WP_GETINFO reply"_s, {});
                    return;
                }
                downloadNext(std::make_shared<QVariantList>(), 1, uint8_t(p[3]), uint8_t(p[1]), uint8_t(p[2]) == 1);
            },
            [this](const QString &e) {
                m_missionBusy = false;
                emit missionDone(u"download"_s, false, e, {});
            });
}

void MspSession::downloadNext(std::shared_ptr<QVariantList> out, int i, int count, int maxWaypoints, bool valid)
{
    if (i > count) {
        m_missionBusy = false;
        emit missionDone(u"download"_s, true, {},
                         {{u"wps"_s, *out}, {u"count"_s, count}, {u"maxWaypoints"_s, maxWaypoints}, {u"valid"_s, valid}});
        return;
    }
    request(MSP_WP, QByteArray(1, char(i)), kMissionTimeoutMs,
            [this, out, i, count, maxWaypoints, valid](const QByteArray &p) {
                MspWaypoint w;
                if (!decodeWaypoint(p, &w)) {
                    m_missionBusy = false;
                    emit missionDone(u"download"_s, false, u"Short MSP_WP reply"_s, {});
                    return;
                }
                out->append(QVariantMap{{u"index"_s, w.index}, {u"action"_s, w.action}, {u"lat"_s, w.lat},
                                        {u"lon"_s, w.lon}, {u"alt"_s, w.alt}, {u"p1"_s, w.p1},
                                        {u"p2"_s, w.p2}, {u"p3"_s, w.p3}, {u"flag"_s, w.flag}});
                emit missionProgress(u"download"_s, i, count);
                downloadNext(out, i + 1, count, maxWaypoints, valid);
            },
            [this](const QString &e) {
                m_missionBusy = false;
                emit missionDone(u"download"_s, false, e, {});
            });
}

} // namespace corv
