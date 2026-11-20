#include "sensors/LidarCore.h"

#include "core/Clock.h"
#include "core/DataRoot.h"
#include "core/DebugLog.h"
#include "core/Geo.h"
#include "link/Mavlink.h"
#include "sensors/LivoxProtocol.h"

#include <QDateTime>
#include <QDir>
#include <QNetworkDatagram>
#include <QNetworkInterface>
#include <QUdpSocket>
#include <QtEndian>
#include <algorithm>
#include <cmath>

using namespace Qt::StringLiterals;

namespace corv {

namespace {
constexpr std::size_t kMaxQueue = 6000;
constexpr int kMapChunk = 262144;
constexpr int kBatch = 32768;
constexpr int kLiveBatch = 16384;
constexpr int kLivePerPacket = 16;   // of 96 -> ~33 kpts/s at the full rate
constexpr double kMaxExtrapMs = 400;
constexpr int kEkfAttitude = 1, kEkfVelocityHoriz = 2, kEkfPosHorizAbs = 16, kEkfPosVertAbs = 32;
constexpr int kEkfRequired = kEkfAttitude | kEkfVelocityHoriz | kEkfPosHorizAbs | kEkfPosVertAbs;
const char *const kCountField = "0000000000";

// ZYX Euler -> rotation matrix (row-major 3x3), maps the rotated frame into the
// parent frame: body->NED for the attitude, lidar->body for the mount.
void eulerToMatrix(double roll, double pitch, double yaw, std::array<double, 9> &out)
{
    const double cr = std::cos(roll), sr = std::sin(roll);
    const double cp = std::cos(pitch), sp = std::sin(pitch);
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    out[0] = cy * cp; out[1] = cy * sp * sr - sy * cr; out[2] = cy * sp * cr + sy * sr;
    out[3] = sy * cp; out[4] = sy * sp * sr + cy * cr; out[5] = sy * sp * cr - cy * sr;
    out[6] = -sp;     out[7] = cp * sr;                out[8] = cp * cr;
}

struct Mpd {
    double mPerLat, mPerLon;
};
Mpd metersPerDegree(double latDeg)
{
    const double f = latDeg * kDeg;
    return {111132.954 - 559.822 * std::cos(2 * f) + 1.175 * std::cos(4 * f),
            111412.84 * std::cos(f) - 93.5 * std::cos(3 * f) + 0.118 * std::cos(5 * f)};
}

double lerpAngle(double a, double b, double f)
{
    double d = b - a;
    while (d > kPi) d -= 2 * kPi;
    while (d < -kPi) d += 2 * kPi;
    return a + d * f;
}

int strideOf(int dataType) { return dataType == 1 ? 14 : dataType == 2 ? 8 : dataType == 3 ? 10 : 0; }

// Point decoding shared by the georeferenced and the live paths
struct Pt {
    double x, y, z;
    std::uint8_t refl, tag;
};
Pt readPoint(const std::uint8_t *b, int dataType)
{
    if (dataType == 1)
        return {qFromLittleEndian<qint32>(b) * 0.001, qFromLittleEndian<qint32>(b + 4) * 0.001, qFromLittleEndian<qint32>(b + 8) * 0.001, b[12], b[13]};
    if (dataType == 2)
        return {qFromLittleEndian<qint16>(b) * 0.01, qFromLittleEndian<qint16>(b + 2) * 0.01, qFromLittleEndian<qint16>(b + 4) * 0.01, b[6], b[7]};
    const double d = qFromLittleEndian<quint32>(b) * 0.001;
    const double theta = qFromLittleEndian<quint16>(b + 4) * 0.01 * kDeg;   // zenith
    const double phi = qFromLittleEndian<quint16>(b + 6) * 0.01 * kDeg;     // azimuth
    return {d * std::sin(theta) * std::cos(phi), d * std::sin(theta) * std::sin(phi), d * std::cos(theta), b[8], b[9]};
}

QString stamp() { return QDateTime::currentDateTimeUtc().toString(u"yyyy-MM-ddTHH-mm-ss"_s); }

QUdpSocket *bindUdp(QObject *parent, quint16 port, QString *error)
{
    auto *s = new QUdpSocket(parent);
    if (!s->bind(QHostAddress::AnyIPv4, port, QAbstractSocket::ShareAddress | QAbstractSocket::ReuseAddressHint)) {
        *error = u"%1: %2"_s.arg(port).arg(s->errorString());
        delete s;
        return nullptr;
    }
    s->setSocketOption(QAbstractSocket::ReceiveBufferSizeSocketOption, 4 * 1024 * 1024);
    return s;
}
} // namespace

// Index of the newest sample with t <= time, walking back from head; -1 when
// the buffer is empty or every sample is newer than `time`
int LidarCore::PoseRing::findBefore(double time) const
{
    for (int k = 1; k <= n; ++k) {
        const int i = (head - k + kCap) % kCap;
        if (t[std::size_t(i)] <= time) return i;
    }
    return -1;
}

LidarCore::LidarCore(QObject *parent) : QObject(parent)
{
    m_batchXyz.resize(kBatch * 3);
    m_batchInt.resize(kBatch);
    m_liveXyz.resize(kLiveBatch * 3);
    m_liveInt.resize(kLiveBatch);
    m_handshakeTimer = new QTimer(this);
    m_handshakeTimer->setInterval(1000);
    connect(m_handshakeTimer, &QTimer::timeout, this, &LidarCore::handshakeTick);
    m_processTimer = new QTimer(this);
    m_processTimer->setInterval(20);
    m_processTimer->setTimerType(Qt::PreciseTimer);
    connect(m_processTimer, &QTimer::timeout, this, [this] { processQueue(wallMs()); });
    m_statusTimer = new QTimer(this);
    m_statusTimer->setInterval(500);
    connect(m_statusTimer, &QTimer::timeout, this, [this] {
        rollStats();
        sendStatus();
    });
}

LidarCore::~LidarCore()
{
    if (m_rec) closePly(*m_rec);
}

QString LidarCore::lidarDir() { return DataRoot::lidarDir(); }

// ============== POSE ==============

std::optional<LidarCore::Pose> LidarCore::poseAt(double time) const
{
    if (m_att.n == 0 || m_pos.n == 0) return std::nullopt;
    Pose p{};
    // Attitude
    int i = m_att.findBefore(time);
    if (i < 0) return std::nullopt;
    int j = (i + 1) % PoseRing::kCap;
    const bool hasNext = m_att.n > 1 && j != m_att.head && m_att.t[std::size_t(j)] > m_att.t[std::size_t(i)];
    if (hasNext) {
        const double f = (time - m_att.t[std::size_t(i)]) / (m_att.t[std::size_t(j)] - m_att.t[std::size_t(i)]);
        p.roll = lerpAngle(m_attR[std::size_t(i)], m_attR[std::size_t(j)], f);
        p.pitch = lerpAngle(m_attP[std::size_t(i)], m_attP[std::size_t(j)], f);
        p.yaw = lerpAngle(m_attY[std::size_t(i)], m_attY[std::size_t(j)], f);
    } else {
        if (time - m_att.t[std::size_t(i)] > kMaxExtrapMs) return std::nullopt;
        p.roll = m_attR[std::size_t(i)];
        p.pitch = m_attP[std::size_t(i)];
        p.yaw = m_attY[std::size_t(i)];
    }
    // Position
    i = m_pos.findBefore(time);
    if (i < 0) return std::nullopt;
    j = (i + 1) % PoseRing::kCap;
    const bool hasNextP = m_pos.n > 1 && j != m_pos.head && m_pos.t[std::size_t(j)] > m_pos.t[std::size_t(i)];
    if (hasNextP) {
        const double f = (time - m_pos.t[std::size_t(i)]) / (m_pos.t[std::size_t(j)] - m_pos.t[std::size_t(i)]);
        p.lat = m_posLat[std::size_t(i)] + (m_posLat[std::size_t(j)] - m_posLat[std::size_t(i)]) * f;
        p.lon = m_posLon[std::size_t(i)] + (m_posLon[std::size_t(j)] - m_posLon[std::size_t(i)]) * f;
        p.alt = m_posAlt[std::size_t(i)] + (m_posAlt[std::size_t(j)] - m_posAlt[std::size_t(i)]) * f;
    } else {
        const double dt = (time - m_pos.t[std::size_t(i)]) / 1000;
        if (dt * 1000 > kMaxExtrapMs) return std::nullopt;
        // Dead-reckon on the EKF velocity for the gap after the last sample
        const Mpd mpd = metersPerDegree(m_posLat[std::size_t(i)]);
        p.lat = m_posLat[std::size_t(i)] + (m_posVn[std::size_t(i)] * dt) / mpd.mPerLat;
        p.lon = m_posLon[std::size_t(i)] + (m_posVe[std::size_t(i)] * dt) / mpd.mPerLon;
        p.alt = m_posAlt[std::size_t(i)] - m_posVd[std::size_t(i)] * dt;
    }
    return p;
}

// ============== MAVLINK TAP ==============

void LidarCore::onMavlink(const mavlink_message_t &msg, qint64 now)
{
    if (!m_connected) return;
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_ATTITUDE: {
        mavlink_attitude_t a;
        mavlink_msg_attitude_decode(&msg, &a);
        if (!std::isfinite(a.roll) || !std::isfinite(a.pitch) || !std::isfinite(a.yaw)) break;
        const int i = m_att.head;
        m_att.t[std::size_t(i)] = double(now);
        m_attR[std::size_t(i)] = a.roll;
        m_attP[std::size_t(i)] = a.pitch;
        m_attY[std::size_t(i)] = a.yaw;
        m_att.head = (i + 1) % PoseRing::kCap;
        if (m_att.n < PoseRing::kCap) ++m_att.n;
        break;
    }
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        mavlink_global_position_int_t g;
        mavlink_msg_global_position_int_decode(&msg, &g);
        if (g.lat == 0 || g.lon == 0) break;
        const int i = m_pos.head;
        m_pos.t[std::size_t(i)] = double(now);
        m_posLat[std::size_t(i)] = g.lat / 1e7;
        m_posLon[std::size_t(i)] = g.lon / 1e7;
        m_posAlt[std::size_t(i)] = g.alt / 1000.0;
        m_posVn[std::size_t(i)] = g.vx / 100.f;
        m_posVe[std::size_t(i)] = g.vy / 100.f;
        m_posVd[std::size_t(i)] = g.vz / 100.f;
        m_pos.head = (i + 1) % PoseRing::kCap;
        if (m_pos.n < PoseRing::kCap) ++m_pos.n;
        break;
    }
    case MAVLINK_MSG_ID_GPS_RAW_INT: {
        mavlink_gps_raw_int_t g;
        mavlink_msg_gps_raw_int_decode(&msg, &g);
        m_nav.fix = g.fix_type;
        m_nav.sats = g.satellites_visible == 255 ? 0 : g.satellites_visible;
        m_nav.hdop = g.eph != 65535 ? g.eph / 100.0 : 99;
        m_nav.gpsTime = now;
        break;
    }
    case MAVLINK_MSG_ID_EKF_STATUS_REPORT: {
        mavlink_ekf_status_report_t e;
        mavlink_msg_ekf_status_report_decode(&msg, &e);
        m_nav.ekfFlags = e.flags;
        m_nav.ekfPosVar = std::max(e.pos_horiz_variance, e.pos_vert_variance);
        m_nav.ekfVelVar = e.velocity_variance;
        m_nav.ekfTime = now;
        break;
    }
    default: break;
    }
}

LidarCore::Gate LidarCore::evaluateGate(qint64 now) const
{
    constexpr qint64 kFresh = 2000;
    if (!m_connected) return {false, u"NO LINK"_s};
    if (m_pos.n == 0 || now - m_pos.t[std::size_t(m_pos.newest())] > kFresh) return {false, u"NO POSITION"_s};
    if (m_att.n == 0 || now - m_att.t[std::size_t(m_att.newest())] > kFresh) return {false, u"NO ATTITUDE"_s};
    if (now - m_nav.gpsTime > 5000) return {false, u"NO GPS"_s};
    if (m_nav.fix < m_cfg.minFix) return {false, u"GPS FIX %1 < %2"_s.arg(m_nav.fix).arg(m_cfg.minFix)};
    if (m_nav.sats < m_cfg.minSats) return {false, u"SATS %1 < %2"_s.arg(m_nav.sats).arg(m_cfg.minSats)};
    if (m_nav.hdop > m_cfg.maxHdop) return {false, u"HDOP %1 > %2"_s.arg(m_nav.hdop, 0, 'f', 1).arg(m_cfg.maxHdop)};
    if (m_cfg.requireEkf) {
        if (m_nav.ekfTime == 0) return {false, u"NO EKF REPORT"_s};
        if (now - m_nav.ekfTime > 5000) return {false, u"EKF STALE"_s};
        if ((m_nav.ekfFlags & kEkfRequired) != kEkfRequired) return {false, u"EKF NOT CONVERGED"_s};
        if (m_nav.ekfPosVar > 0.5 || m_nav.ekfVelVar > 0.5) return {false, u"EKF VARIANCE HIGH"_s};
    }
    if (m_voxelCount >= m_cfg.maxPoints) return {false, u"MAP FULL"_s};
    return {true, u"ACCUMULATING"_s};
}

// ============== VOXEL MAP ==============

void LidarCore::resetMap()
{
    m_voxelKeys.clear();
    m_voxelCount = 0;
    m_mapChunks.clear();
    m_mapCount = 0;
    m_origin.reset();
    m_batchN = 0;
}

void LidarCore::ensureVoxelTable()
{
    if (!m_voxelKeys.empty()) return;
    std::uint64_t size = 1 << 16;
    while (double(size) < m_cfg.maxPoints * 2) size <<= 1;
    m_voxelKeys.assign(size, 0.0);
    m_voxelMask = size - 1;
}

// True when the voxel was free (and marks it). The key packs three 17-bit
// indices (+-16 km at 0.25 m) into 51 bits, safely below 2^53.
bool LidarCore::claimVoxel(double e, double n, double u)
{
    if (m_voxelKeys.empty()) ensureVoxelTable();
    const double inv = 1 / m_cfg.voxel;
    const double ix = std::floor(e * inv) + 65536, iy = std::floor(n * inv) + 65536, iz = std::floor(u * inv) + 65536;
    if (ix < 0 || iy < 0 || iz < 0 || ix >= 131072 || iy >= 131072 || iz >= 131072) return false;
    const double key = (ix * 131072 + iy) * 131072 + iz + 1;
    // Multiplicative hash on the low bits, linear probing
    const std::uint32_t hx = std::uint32_t(std::uint64_t(ix) * 73856093ull), hy = std::uint32_t(std::uint64_t(iy) * 19349663ull),
                        hz = std::uint32_t(std::uint64_t(iz) * 83492791ull);
    std::uint64_t h = (hx ^ hy ^ hz) & m_voxelMask;
    for (;;) {
        const double k = m_voxelKeys[h];
        if (k == 0) {
            m_voxelKeys[h] = key;
            ++m_voxelCount;
            return true;
        }
        if (k == key) return false;
        h = (h + 1) & m_voxelMask;
    }
}

void LidarCore::mapPush(float e, float n, float u, std::uint8_t intensity)
{
    if (m_mapChunks.empty() || m_mapChunks.back().n >= kMapChunk) {
        MapChunk c;
        c.xyz.resize(std::size_t(kMapChunk) * 3);
        c.i.resize(kMapChunk);
        m_mapChunks.push_back(std::move(c));
    }
    MapChunk &c = m_mapChunks.back();
    const std::size_t o = std::size_t(c.n) * 3;
    c.xyz[o] = e;
    c.xyz[o + 1] = n;
    c.xyz[o + 2] = u;
    c.i[std::size_t(c.n)] = intensity;
    ++c.n;
    ++m_mapCount;
}

void LidarCore::batchPush(float e, float n, float u, std::uint8_t intensity)
{
    if (m_batchN >= kBatch) flushBatch();
    const std::size_t o = std::size_t(m_batchN) * 3;
    m_batchXyz[o] = e;
    m_batchXyz[o + 1] = n;
    m_batchXyz[o + 2] = u;
    m_batchInt[std::size_t(m_batchN)] = intensity;
    ++m_batchN;
}

void LidarCore::flushBatch()
{
    m_lastFlush = m_now;
    if (m_batchN > 0) {
        emit points(m_epoch, QVector<float>(m_batchXyz.begin(), m_batchXyz.begin() + m_batchN * 3),
                    QByteArray(reinterpret_cast<const char *>(m_batchInt.data()), m_batchN));
        m_batchN = 0;
    }
    if (m_liveN > 0) {
        emit live(m_liveFrame, m_cfg.liveSeconds, QVector<float>(m_liveXyz.begin(), m_liveXyz.begin() + m_liveN * 3),
                  QByteArray(reinterpret_cast<const char *>(m_liveInt.data()), m_liveN));
        m_liveN = 0;
    }
}

// ============== POINT PACKETS ==============

void LidarCore::onPointPacket(const QByteArray &buf, qint64 rxTime)
{
    if (buf.size() < livox::kPacketHeader) return;
    const int dataType = std::uint8_t(buf[10]);
    if (dataType == 0) return;   // IMU packet on the wrong port
    ++m_stats.packets;
    m_stats.bytes += buf.size();
    m_lidar.lastPacket = rxTime;
    if (m_queue.size() >= kMaxQueue) m_queue.pop_front();
    m_queue.push_back({buf, rxTime});
}

void LidarCore::processQueue(qint64 now)
{
    m_now = now;
    const double hold = m_cfg.lagMs + 60;
    const Gate g = evaluateGate(now);
    if (g.ok != m_gate.ok || g.reason != m_gate.reason) {
        m_gate = g;
        sendStatus();
    }
    if (m_mountDirty) {
        eulerToMatrix(m_cfg.mountRoll * kDeg, m_cfg.mountPitch * kDeg, m_cfg.mountYaw * kDeg, m_mount);
        m_mountDirty = false;
    }

    int processed = 0;
    while (!m_queue.empty() && m_queue.front().t + hold <= now && processed < 600) {
        const Packet pkt = std::move(m_queue.front());
        m_queue.pop_front();
        ++processed;
        const auto *b = reinterpret_cast<const std::uint8_t *>(pkt.buf.constData());
        const int dotNum = qFromLittleEndian<quint16>(b + 5);
        const int dataType = b[10];
        m_stats.points += dotNum;
        if (!m_gate.ok) {
            m_stats.gated += dotNum;
            livePacket(pkt.buf, dotNum, dataType, now);
            continue;
        }
        // Pose at the packet's (corrected) measurement time
        const auto pose = poseAt(double(pkt.t) + m_cfg.lagMs);
        if (!pose) {
            m_stats.gated += dotNum;
            livePacket(pkt.buf, dotNum, dataType, now);
            continue;
        }
        if (!m_origin) {
            const Mpd mpd = metersPerDegree(pose->lat);
            m_origin = Origin{pose->lat, pose->lon, pose->alt, mpd.mPerLat, mpd.mPerLon, ++m_epoch};
            ensureVoxelTable();
            sendOrigin();
        }
        eulerToMatrix(pose->roll, pose->pitch, pose->yaw, m_attM);
        const double vehE = (pose->lon - m_origin->lon) * m_origin->mPerLon;
        const double vehN = (pose->lat - m_origin->lat) * m_origin->mPerLat;
        const double vehU = pose->alt - m_origin->alt;
        georeferencePacket(pkt.buf, dotNum, dataType, vehE, vehN, vehU, pkt.t);
    }

    if ((m_batchN > 0 || m_liveN > 0) && now - m_lastFlush >= 50) flushBatch();
}

// Gate closed: a subsample of the packet, vehicle-relative. With a fresh
// attitude the points are levelled into NED around the vehicle (the picture
// stays upright while the aircraft banks); without one they stay in the body
// frame and the renderer rotates them with the model.
void LidarCore::livePacket(const QByteArray &buf, int dotNum, int dataType, qint64 now)
{
    const int stride = strideOf(dataType);
    if (!stride) return;
    const int n = std::min(dotNum, int((buf.size() - livox::kPacketHeader) / stride));
    const int step = std::max(1, n / kLivePerPacket);
    const double rMin2 = m_cfg.minRange * m_cfg.minRange, rMax2 = m_cfg.maxRange * m_cfg.maxRange;
    const double lx = m_cfg.leverX, ly = m_cfg.leverY, lz = m_cfg.leverZ;
    const auto &m = m_mount;
    const int iAtt = m_att.n ? m_att.newest() : -1;
    const bool levelled = iAtt >= 0 && now - m_att.t[std::size_t(iAtt)] < 2000;
    if (levelled) eulerToMatrix(m_attR[std::size_t(iAtt)], m_attP[std::size_t(iAtt)], m_attY[std::size_t(iAtt)], m_attM);
    const QString frame = levelled ? u"ned"_s : u"body"_s;
    if (frame != m_liveFrame && m_liveN > 0) flushBatch();
    m_liveFrame = frame;
    const auto &a = m_attM;
    const auto *b = reinterpret_cast<const std::uint8_t *>(buf.constData());

    for (int k = 0; k < n; k += step) {
        const Pt pt = readPoint(b + livox::kPacketHeader + k * stride, dataType);
        const double r2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
        if (r2 < rMin2 || r2 > rMax2) continue;
        if (m_cfg.dropNoise && (pt.tag & 0x0F) != 0) continue;
        const double fx = pt.x, fy = -pt.y, fz = -pt.z;
        double bx = m[0] * fx + m[1] * fy + m[2] * fz + lx;
        double by = m[3] * fx + m[4] * fy + m[5] * fz + ly;
        double bz = m[6] * fx + m[7] * fy + m[8] * fz + lz;
        if (levelled) {
            const double pn = a[0] * bx + a[1] * by + a[2] * bz;
            const double pe = a[3] * bx + a[4] * by + a[5] * bz;
            const double pd = a[6] * bx + a[7] * by + a[8] * bz;
            bx = pn;
            by = pe;
            bz = pd;
        }
        if (m_liveN >= kLiveBatch) flushBatch();
        const std::size_t o = std::size_t(m_liveN) * 3;
        m_liveXyz[o] = float(bx);
        m_liveXyz[o + 1] = float(by);
        m_liveXyz[o + 2] = float(bz);
        m_liveInt[std::size_t(m_liveN)] = pt.refl;
        ++m_liveN;
        ++m_stats.live;
    }
}

void LidarCore::georeferencePacket(const QByteArray &buf, int dotNum, int dataType, double vehE, double vehN, double vehU, qint64 rxTime)
{
    const int stride = strideOf(dataType);
    if (!stride) return;
    const int n = std::min(dotNum, int((buf.size() - livox::kPacketHeader) / stride));
    const double rMin2 = m_cfg.minRange * m_cfg.minRange, rMax2 = m_cfg.maxRange * m_cfg.maxRange;
    const double lx = m_cfg.leverX, ly = m_cfg.leverY, lz = m_cfg.leverZ;
    const auto &m = m_mount, &a = m_attM;
    const double tSec = rxTime / 1000.0;
    const auto *b = reinterpret_cast<const std::uint8_t *>(buf.constData());

    for (int k = 0; k < n; ++k) {
        const Pt pt = readPoint(b + livox::kPacketHeader + k * stride, dataType);
        const double r2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
        if (r2 < rMin2 || r2 > rMax2) {   // also drops the (0,0,0) no-return points
            ++m_stats.filtered;
            continue;
        }
        if (m_cfg.dropNoise && (pt.tag & 0x0F) != 0) {
            ++m_stats.filtered;
            continue;
        }
        // Livox FLU -> FRD, mount rotation, lever arm -> body FRD
        const double fx = pt.x, fy = -pt.y, fz = -pt.z;
        const double bx = m[0] * fx + m[1] * fy + m[2] * fz + lx;
        const double by = m[3] * fx + m[4] * fy + m[5] * fz + ly;
        const double bz = m[6] * fx + m[7] * fy + m[8] * fz + lz;
        // body -> NED, then ENU from the vehicle position
        const double pn = a[0] * bx + a[1] * by + a[2] * bz;
        const double pe = a[3] * bx + a[4] * by + a[5] * bz;
        const double pd = a[6] * bx + a[7] * by + a[8] * bz;
        const double E = vehE + pe, N = vehN + pn, U = vehU - pd;

        if (m_rec) plyWrite(*m_rec, float(E), float(N), float(U), pt.refl, tSec);
        if (!claimVoxel(E, N, U)) continue;
        ++m_stats.accepted;
        mapPush(float(E), float(N), float(U), pt.refl);
        batchPush(float(E), float(N), float(U), pt.refl);
    }
}

// ============== PLY WRITER ==============
// Binary little-endian PLY, count patched in place at close. Points are ENU
// metres from the origin recorded in the header - CloudCompare / QGIS turn
// that into a projected CRS with a global shift.

std::optional<LidarCore::Ply> LidarCore::openPly(const QString &path, bool withTime) const
{
    QStringList header{u"ply"_s, u"format binary_little_endian 1.0"_s, u"comment Corv-GCS Livox point cloud"_s,
                       u"comment frame ENU metres, x=east y=north z=up, relative to origin"_s};
    header << (m_origin ? u"comment origin_lat %1 origin_lon %2 origin_alt_msl %3"_s
                              .arg(m_origin->lat, 0, 'f', 8).arg(m_origin->lon, 0, 'f', 8).arg(m_origin->alt, 0, 'f', 3)
                        : u"comment origin_lat unset origin_lon unset origin_alt_msl unset"_s);
    header << u"comment created "_s + QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
    header << u"comment lidar %1 %2"_s.arg(m_lidar.sn.isEmpty() ? u"unknown"_s : m_lidar.sn, livox::deviceName(m_lidar.devType));
    header << u"element vertex "_s + QLatin1String(kCountField);
    header << u"property float x"_s << u"property float y"_s << u"property float z"_s
           << u"property uchar intensity"_s;
    if (withTime) header << u"property double time"_s;
    header << u"end_header"_s;
    const QByteArray text = (header.join('\n'_L1) + '\n'_L1).toUtf8();
    Ply w;
    w.file = std::make_unique<QFile>(path);
    if (!w.file->open(QIODevice::WriteOnly | QIODevice::Truncate)) return std::nullopt;
    w.file->write(text);
    w.path = path;
    w.countPos = text.indexOf(kCountField);
    w.withTime = withTime;
    w.buf.reserve(1 << 16);
    return w;
}

void LidarCore::plyWrite(Ply &w, float e, float n, float u, std::uint8_t intensity, double tSec)
{
    const int need = w.withTime ? 21 : 13;
    if (w.buf.size() + need > (1 << 16)) plyFlush(w);
    char rec[21];
    qToLittleEndian<float>(e, rec);
    qToLittleEndian<float>(n, rec + 4);
    qToLittleEndian<float>(u, rec + 8);
    rec[12] = char(intensity);
    if (w.withTime) qToLittleEndian<double>(tSec, rec + 13);
    w.buf.append(rec, need);
    ++w.count;
}

void LidarCore::plyFlush(Ply &w)
{
    if (w.buf.isEmpty()) return;
    w.file->write(w.buf);
    w.buf.clear();
}

void LidarCore::closePly(Ply &w)
{
    plyFlush(w);
    w.file->seek(w.countPos);
    w.file->write(QString::number(w.count).rightJustified(int(qstrlen(kCountField)), '0'_L1).toLatin1());
    w.file->close();
}

QString LidarCore::startRecording()
{
    if (m_rec) return m_rec->path;
    const QString path = QDir(lidarDir()).filePath(u"raw-%1.ply"_s.arg(stamp()));
    m_rec = openPly(path, true);
    if (!m_rec) {
        m_lastError = u"cannot write %1"_s.arg(path);
        return {};
    }
    LOG_I("lidar") << "recording raw points to" << path;
    return path;
}

QString LidarCore::stopRecording()
{
    if (!m_rec) return {};
    const QString p = m_rec->path;
    closePly(*m_rec);
    LOG_I("lidar") << m_rec->count << "raw points recorded to" << p;
    m_rec.reset();
    return p;
}

QVariantMap LidarCore::saveMap()
{
    if (!m_origin || m_mapCount == 0) return {{u"success"_s, false}, {u"error"_s, u"Map is empty"_s}};
    const QString path = QDir(lidarDir()).filePath(u"map-%1.ply"_s.arg(stamp()));
    auto w = openPly(path, false);
    if (!w) return {{u"success"_s, false}, {u"error"_s, u"cannot write %1"_s.arg(path)}};
    for (const MapChunk &c : m_mapChunks)
        for (int k = 0; k < c.n; ++k)
            plyWrite(*w, c.xyz[std::size_t(k) * 3], c.xyz[std::size_t(k) * 3 + 1], c.xyz[std::size_t(k) * 3 + 2], c.i[std::size_t(k)], 0);
    closePly(*w);
    LOG_I("lidar") << m_mapCount << "points saved to" << path;
    return {{u"success"_s, true}, {u"path"_s, path}, {u"points"_s, m_mapCount}};
}

// ============== NETWORK ==============

QString LidarCore::pickHostIp(const QString &lidarIp) const
{
    if (!m_cfg.hostIp.isEmpty() && m_cfg.hostIp != "auto"_L1) return m_cfg.hostIp;
    const QString prefix = lidarIp.section('.'_L1, 0, 2) + '.'_L1;
    QString fallback;
    for (const QNetworkInterface &ni : QNetworkInterface::allInterfaces()) {
        if (!(ni.flags() & QNetworkInterface::IsUp) || (ni.flags() & QNetworkInterface::IsLoopBack)) continue;
        for (const QNetworkAddressEntry &e : ni.addressEntries()) {
            if (e.ip().protocol() != QAbstractSocket::IPv4Protocol) continue;
            const QString a = e.ip().toString();
            if (a.startsWith(prefix)) return a;
            if (fallback.isEmpty()) fallback = a;
        }
    }
    return fallback.isEmpty() ? u"0.0.0.0"_s : fallback;
}

QVariantList LidarCore::listInterfaces()
{
    QVariantList out;
    for (const QNetworkInterface &ni : QNetworkInterface::allInterfaces()) {
        if (ni.flags() & QNetworkInterface::IsLoopBack) continue;
        for (const QNetworkAddressEntry &e : ni.addressEntries())
            if (e.ip().protocol() == QAbstractSocket::IPv4Protocol)
                out << QVariantMap{{u"name"_s, ni.humanReadableName()}, {u"address"_s, e.ip().toString()}};
    }
    return out;
}

QVariantMap LidarCore::connectLidar(const QVariantMap &cfg)
{
    applyConfig(cfg);
    if (m_connected) return {{u"success"_s, true}};
    resetPose();
    m_queue.clear();
    m_stats = {};
    m_statsT0 = wallMs();
    m_lidar = {};
    m_lidar.ip = m_cfg.lidarIp;
    m_lastError.clear();

    QString err;
    // A host port held by Livox Viewer or a second GCS, or inside a range Windows
    // reserves for Hyper-V / WSL (netsh int ipv4 show excludedportrange): any free
    // port instead. Search and command replies come back to the sender's port; the
    // state / point / IMU ports go to the LiDAR in the configuration (sendConfig).
    auto bindOrAny = [this, &err](quint16 port) {
        if (QUdpSocket *s = bindUdp(this, port, &err)) return s;
        LOG_W("lidar") << "port" << port << "unavailable (" << err << "), using a free one";
        return bindUdp(this, 0, &err);
    };
    m_search = bindOrAny(livox::kHostSearch);
    if (m_search) m_cmd = bindOrAny(livox::kHostCmd);
    if (m_cmd) m_push = bindOrAny(livox::kHostPush);
    if (m_push) m_point = bindOrAny(livox::kHostPoint);
    if (m_point) m_imu = bindOrAny(livox::kHostImu);
    if (!m_imu) {
        m_lastError = err;
        closeSockets();
        return {{u"success"_s, false},
                {u"error"_s, u"UDP bind failed (%1) — is Livox Viewer or another GCS running?"_s.arg(err)}};
    }
    connect(m_search, &QUdpSocket::readyRead, this, [this] { onSearchReply(m_search); });
    connect(m_cmd, &QUdpSocket::readyRead, this, [this] { onCmdReply(m_cmd); });
    connect(m_point, &QUdpSocket::readyRead, this, [this] { onPointReady(m_point); });
    for (QUdpSocket *s : {m_push, m_imu}) connect(s, &QUdpSocket::readyRead, s, [s] { while (s->hasPendingDatagrams()) s->receiveDatagram(); });

    m_connected = true;
    m_mountDirty = true;
    if (m_cfg.recordRaw) startRecording();
    m_handshakeTimer->start();
    m_processTimer->start();
    m_statusTimer->start();
    handshakeTick();
    sendStatus();
    LOG_I("lidar") << "connecting to" << m_cfg.lidarIp << "host" << pickHostIp(m_cfg.lidarIp) << "ports" << m_cmd->localPort() << m_push->localPort()
                   << m_point->localPort() << m_imu->localPort();
    return {{u"success"_s, true}};
}

void LidarCore::closeSockets()
{
    for (QUdpSocket **s : {&m_search, &m_cmd, &m_push, &m_point, &m_imu}) {
        if (!*s) continue;
        (*s)->disconnect(this);
        (*s)->close();
        (*s)->deleteLater();
        *s = nullptr;
    }
}

QVariantMap LidarCore::disconnectLidar()
{
    if (!m_connected) return {{u"success"_s, true}};
    m_connected = false;
    m_handshakeTimer->stop();
    m_processTimer->stop();
    m_statusTimer->stop();
    m_now = wallMs();
    flushBatch();
    stopRecording();
    closeSockets();
    m_queue.clear();
    m_gate = {};
    sendStatus();
    LOG_I("lidar") << "disconnected";
    return {{u"success"_s, true}};
}

void LidarCore::resetPose()
{
    m_att.n = m_att.head = 0;
    m_pos.n = m_pos.head = 0;
    m_nav = {};
}

void LidarCore::setConnectedForTest(bool v)
{
    m_connected = v;
    if (v) {
        resetPose();
        m_mountDirty = true;
    }
}

// Discovery + configuration retried every second until the LiDAR acks and
// packets flow; re-sent if packets stop (LiDAR rebooted, link dropped)
void LidarCore::handshakeTick()
{
    if (!m_search) return;
    const qint64 now = wallMs();
    const bool streaming = m_lidar.lastPacket && now - m_lidar.lastPacket < 2000;
    if (m_lidar.configured && streaming) return;
    if (m_lidar.configured && !streaming && now - m_lidar.lastAck > 5000) m_lidar.configured = false;

    const QByteArray searchPkt = livox::packCommand(livox::kCmdSearch);
    m_search->writeDatagram(searchPkt, QHostAddress(m_cfg.lidarIp), livox::kLidarSearch);
    m_search->writeDatagram(searchPkt, QHostAddress::Broadcast, livox::kLidarSearch);
    if (!m_lidar.configured) sendConfig();
}

void LidarCore::sendConfig()
{
    const QString host = pickHostIp(m_cfg.lidarIp);
    if (host == "0.0.0.0"_L1) m_lastError = u"no local IPv4 interface found — set HOST IP manually"_s;
    const QHostAddress to(m_cfg.lidarIp);
    m_cmd->writeDatagram(livox::packCommand(livox::kCmdWorkMode, livox::buildKvRequest({
                                                {livox::kKeyStateHostIp, livox::hostIpValue(host, m_push->localPort(), livox::kLidarPush)},
                                                {livox::kKeyPointHostIp, livox::hostIpValue(host, m_point->localPort(), livox::kLidarPoint)},
                                                {livox::kKeyImuHostIp, livox::hostIpValue(host, m_imu->localPort(), livox::kLidarImu)},
                                            })),
                         to, livox::kLidarCmd);
    m_cmd->writeDatagram(livox::packCommand(livox::kCmdWorkMode, livox::buildKvRequest({{livox::kKeyPclDataType, QByteArray(1, char(m_cfg.dataType == 2 ? 2 : 1))}})),
                         to, livox::kLidarCmd);
    m_cmd->writeDatagram(livox::packCommand(livox::kCmdWorkMode, livox::buildKvRequest({{livox::kKeyWorkMode, QByteArray(1, char(livox::kWorkModeNormal))}})),
                         to, livox::kLidarCmd);
}

void LidarCore::onSearchReply(QUdpSocket *s)
{
    while (s && s->hasPendingDatagrams()) {
        const QNetworkDatagram d = s->receiveDatagram();
        const auto pkt = livox::parseCommand(d.data());
        if (!pkt || pkt->cmdId != livox::kCmdSearch || pkt->cmdType != 1 || pkt->data.size() < 24) continue;
        // DetectionData: ret_code u8, dev_type u8, sn[16], lidar_ip[4], cmd_port u16
        const auto *b = reinterpret_cast<const std::uint8_t *>(pkt->data.constData());
        if (b[0] != 0) continue;
        const QString ip = u"%1.%2.%3.%4"_s.arg(b[18]).arg(b[19]).arg(b[20]).arg(b[21]);
        QString from = d.senderAddress().toString();
        if (from.startsWith("::ffff:"_L1)) from = from.mid(7);
        // Another unit answered the broadcast - only the configured one is used
        if (ip != m_cfg.lidarIp && from != m_cfg.lidarIp) continue;
        m_lidar.devType = b[1];
        m_lidar.sn = QString::fromLatin1(pkt->data.mid(2, 16)).section(QChar(0), 0, 0);
        m_lidar.ip = ip;
    }
}

void LidarCore::onCmdReply(QUdpSocket *s)
{
    while (s && s->hasPendingDatagrams()) {
        const auto pkt = livox::parseCommand(s->receiveDatagram().data());
        if (!pkt || pkt->cmdType != 1) continue;
        if (pkt->cmdId == livox::kCmdWorkMode && pkt->data.size() >= 3) {
            const auto *b = reinterpret_cast<const std::uint8_t *>(pkt->data.constData());
            const int ret = b[0];
            const int errKey = qFromLittleEndian<quint16>(b + 1);
            m_lidar.lastAck = wallMs();
            if (ret == 0) m_lidar.configured = true;
            else m_lastError = u"config rejected (ret %1, key 0x%2)"_s.arg(ret).arg(errKey, 0, 16);
        }
    }
}

void LidarCore::onPointReady(QUdpSocket *s)
{
    while (s && s->hasPendingDatagrams()) {
        const QNetworkDatagram d = s->receiveDatagram();
        onPointPacket(d.data(), wallMs());
    }
}

// ============== STATUS ==============

void LidarCore::rollStats()
{
    const qint64 now = wallMs();
    const double dt = (now - m_statsT0) / 1000.0;
    if (dt < 0.4) return;
    auto r = [dt](qint64 v) { return qint64(std::lround(v / dt)); };
    m_statsShown = {{u"pps"_s, r(m_stats.packets)},
                    {u"pointsPs"_s, r(m_stats.points)},
                    {u"acceptedPs"_s, r(m_stats.accepted)},
                    {u"gatedPs"_s, r(m_stats.gated)},
                    {u"filteredPs"_s, r(m_stats.filtered)},
                    {u"livePs"_s, r(m_stats.live)},
                    {u"kbps"_s, qint64(std::lround(m_stats.bytes * 8 / dt / 1000))}};
    m_stats = {};
    m_statsT0 = now;
}

QVariantMap LidarCore::status() const
{
    const qint64 now = wallMs();
    const bool streaming = m_lidar.lastPacket && now - m_lidar.lastPacket < 2000;
    QString link = u"OFF"_s;
    if (m_connected)
        link = streaming ? u"STREAMING"_s : m_lidar.configured ? u"CONFIGURED"_s : !m_lidar.sn.isEmpty() ? u"FOUND"_s : u"SEARCHING"_s;
    QString model = livox::deviceName(m_lidar.devType);
    if (model.isEmpty() && m_lidar.devType) model = u"type %1"_s.arg(m_lidar.devType);
    QVariantMap st{{u"connected"_s, m_connected},
                   {u"link"_s, link},
                   {u"streaming"_s, streaming},
                   {u"lidarIp"_s, m_cfg.lidarIp},
                   {u"hostIp"_s, m_connected ? pickHostIp(m_cfg.lidarIp) : QString()},
                   {u"sn"_s, m_lidar.sn},
                   {u"model"_s, model},
                   {u"gateOk"_s, m_gate.ok},
                   {u"gateReason"_s, m_gate.reason},
                   {u"navFix"_s, m_nav.fix},
                   {u"navSats"_s, m_nav.sats},
                   {u"navHdop"_s, m_nav.hdop},
                   {u"navEkfFlags"_s, m_nav.ekfFlags},
                   {u"navEkfPosVar"_s, m_nav.ekfPosVar},
                   {u"mapPoints"_s, m_mapCount},
                   {u"maxPoints"_s, m_cfg.maxPoints},
                   {u"queue"_s, qint64(m_queue.size())},
                   {u"recording"_s, bool(m_rec)},
                   {u"recordPath"_s, m_rec ? m_rec->path : QString()},
                   {u"recordedPoints"_s, m_rec ? m_rec->count : 0},
                   {u"error"_s, m_lastError}};
    for (auto it = m_statsShown.cbegin(); it != m_statsShown.cend(); ++it) st.insert(it.key(), it.value());
    return st;
}

void LidarCore::sendOrigin()
{
    if (!m_origin) return;
    emit originChanged({{u"lat"_s, m_origin->lat},
                        {u"lon"_s, m_origin->lon},
                        {u"alt"_s, m_origin->alt},
                        {u"mPerLat"_s, m_origin->mPerLat},
                        {u"mPerLon"_s, m_origin->mPerLon},
                        {u"epoch"_s, m_origin->epoch}});
}

// ============== COMMANDS ==============

void LidarCore::applyConfig(const QVariantMap &patch)
{
    const double prevVoxel = m_cfg.voxel, prevMax = m_cfg.maxPoints;
    auto num = [&](const char *key, double &field) {
        const auto it = patch.constFind(QLatin1String(key));
        if (it == patch.cend()) return;
        bool ok = false;
        const double v = it->toDouble(&ok);
        if (ok && std::isfinite(v)) field = v;
    };
    auto integer = [&](const char *key, int &field) {
        double v = field;
        num(key, v);
        field = int(v);
    };
    auto flag = [&](const char *key, bool &field) {
        const auto it = patch.constFind(QLatin1String(key));
        if (it != patch.cend()) field = it->toBool();
    };
    auto text = [&](const char *key, QString &field) {
        const auto it = patch.constFind(QLatin1String(key));
        if (it != patch.cend()) field = it->toString();
    };
    text("lidarIp", m_cfg.lidarIp);
    text("hostIp", m_cfg.hostIp);
    integer("dataType", m_cfg.dataType);
    num("mountRoll", m_cfg.mountRoll);
    num("mountPitch", m_cfg.mountPitch);
    num("mountYaw", m_cfg.mountYaw);
    num("leverX", m_cfg.leverX);
    num("leverY", m_cfg.leverY);
    num("leverZ", m_cfg.leverZ);
    num("lagMs", m_cfg.lagMs);
    num("minRange", m_cfg.minRange);
    num("maxRange", m_cfg.maxRange);
    flag("dropNoise", m_cfg.dropNoise);
    num("voxel", m_cfg.voxel);
    num("maxPoints", m_cfg.maxPoints);
    integer("minFix", m_cfg.minFix);
    num("maxHdop", m_cfg.maxHdop);
    num("minSats", m_cfg.minSats);
    flag("requireEkf", m_cfg.requireEkf);
    num("liveSeconds", m_cfg.liveSeconds);
    flag("recordRaw", m_cfg.recordRaw);
    // Under 1 cm the voxel indices leave their 17 bits (+-655 m at 1 cm), and the
    // occupancy table is 2 x MAX POINTS doubles: a typo there must not size it past
    // memory (or overflow the power-of-two search in ensureVoxelTable)
    m_cfg.voxel = m_cfg.voxel > 0 ? std::max(0.01, m_cfg.voxel) : 0.25;
    m_cfg.maxPoints = std::clamp(m_cfg.maxPoints, 1000.0, 50000000.0);
    m_mountDirty = true;
    if (m_cfg.voxel != prevVoxel || m_cfg.maxPoints != prevMax) {
        // The occupancy table is keyed on the voxel size: rebuild it from the map
        m_voxelKeys.clear();
        m_voxelCount = 0;
        if (m_mapCount > 0) {
            ensureVoxelTable();
            for (const MapChunk &c : m_mapChunks)
                for (int k = 0; k < c.n; ++k) claimVoxel(c.xyz[std::size_t(k) * 3], c.xyz[std::size_t(k) * 3 + 1], c.xyz[std::size_t(k) * 3 + 2]);
        }
    }
    if (m_connected) {
        if (m_cfg.recordRaw && !m_rec) startRecording();
        if (!m_cfg.recordRaw && m_rec) stopRecording();
    }
}

void LidarCore::clear()
{
    resetMap();
    sendStatus();
}

} // namespace corv
