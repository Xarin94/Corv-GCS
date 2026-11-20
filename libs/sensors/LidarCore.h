// LidarCore.h - Livox Mid-360 / Mid-360S point-cloud client (port of
// lidar-core.js, lidar-worker.js and lidar-manager.js).
//
// Lives on a thread of its own ("lidar"): 2 000 UDP datagrams a second and
// the per-packet maths stay off the GUI and render threads.
//
// The GCS talks to the LiDAR directly over the IP link (a LAN bridge to the
// aircraft): MAVLink telemetry keeps flowing on its own channel, this client
// opens the Livox SDK2 UDP protocol next to it and georeferences every point
// with the vehicle pose from that telemetry.
//
//   LiDAR (192.168.1.1xx)                 host (this PC)
//     56000  <-- search (cmd 0x0000) --   56000   discovery, unicast + broadcast
//     56100  <-- config (cmd 0x0100) --   56101   host IP/ports, work mode, data type
//     56200  -- push msg (0x0102) -->     56201   lidar state (ignored)
//     56300  -- point packets -->         56301   ~2100 packets/s x 96 points
//     56400  -- IMU packets -->           56401   200 Hz (received, ignored)
//
// Point packets carry points in the LiDAR frame (X forward, Y left, Z up,
// origin at the optical centre). Each packet is turned into world points with
// the chain
//
//   p_frd  = (x, -y, -z)                       Livox FLU -> aircraft FRD
//   p_body = R_mount * p_frd + lever_arm       mount attitude + offset from the IMU
//   p_ned  = R_att(roll, pitch, yaw) * p_body  ATTITUDE (30) at the packet time
//   ENU    = pos(t) + (p_ned.e, p_ned.n, -p_ned.d)   GLOBAL_POSITION_INT (33)
//
// where pos(t) and att(t) are interpolated in time from a pose ring buffer,
// because telemetry runs at 3-10 Hz and packets at 2 kHz. This is the direct
// georeferencing every airborne LiDAR pipeline uses (DJI L1/L2 live preview,
// YellowScan LiveStation); the difference from a survey product is the time
// base: with no PPS into the Mid-360 both streams are aligned on GCS arrival
// time, so a configurable "telemetry lag" absorbs the radio delay.
//
// Accumulation is gated on navigation quality (GPS fix / sats / HDOP and the
// ArduPilot EKF flags + variances) and decimated through a voxel grid - one
// point per voxel, like the map downsampling in FAST-LIO - so a long flight
// stays within a few million points for the renderer and for the .ply export.
//
// Packets that cannot be georeferenced (gate closed: no GPS, EKF not
// converged, map full...) are not thrown away: a subsample goes to the
// renderer as LIVE points, relative to the vehicle (level NED frame when
// attitude is known, body frame otherwise), where they fade out after a few
// seconds. The operator always sees what the sensor sees; only the
// georeferenced points are kept.
//
// Commands are called on the client's thread (LidarController queues them);
// the pose messages come from the MessageBus on the same thread.
#pragma once

#include "link/MavMessage.h"

#include <QByteArray>
#include <QFile>
#include <QObject>
#include <QTimer>
#include <QVariantMap>
#include <QVector>
#include <array>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <vector>

QT_FORWARD_DECLARE_CLASS(QUdpSocket)

namespace corv {

class LidarCore : public QObject
{
    Q_OBJECT
public:
    struct Config {
        QString lidarIp = QStringLiteral("192.168.1.12");
        QString hostIp = QStringLiteral("auto");   // IPv4 the LiDAR must send to; 'auto' = interface on the LiDAR's /24
        int dataType = 1;                          // 1 = Cartesian 32-bit (mm), 2 = Cartesian 16-bit (cm) - halves the bandwidth
        // Mount attitude of the LiDAR relative to the autopilot body frame (FRD),
        // ZYX Euler in degrees, and the lever arm IMU -> LiDAR optical centre (m, FRD).
        double mountRoll = 0, mountPitch = 0, mountYaw = 0;
        double leverX = 0, leverY = 0, leverZ = 0;
        double lagMs = 0;          // telemetry arrives this much later than the point packets
        double minRange = 2.5;     // m - rejects the airframe: rudder, gear, struts, antennas in the FOV
        double maxRange = 70;      // m - Mid-360 rated range
        bool dropNoise = true;     // tag bits 0-3: spatial / intensity noise flags
        double voxel = 0.25;       // m - one point kept per voxel
        double maxPoints = 3000000;
        // Navigation-quality gate
        int minFix = 3;            // GPS_FIX_TYPE: 3 = 3D, 4 = DGPS, 5 = RTK float, 6 = RTK fixed
        double maxHdop = 2.0;
        double minSats = 8;
        bool requireEkf = true;    // EKF_STATUS_REPORT flags + variances (ArduPilot)
        double liveSeconds = 3;    // non-georeferenced points stay on screen this long
        bool recordRaw = false;    // stream every accepted point (pre-voxel) to a .ply
    };
    struct Gate {
        bool ok = false;
        QString reason = QStringLiteral("NO LINK");
    };
    struct MapChunk {
        std::vector<float> xyz;           // ENU metres from the origin
        std::vector<std::uint8_t> i;
        int n = 0;
    };
    struct Origin {
        double lat = 0, lon = 0, alt = 0, mPerLat = 0, mPerLon = 0;
        int epoch = 0;
    };

    explicit LidarCore(QObject *parent = nullptr);
    ~LidarCore() override;

    // ── Commands (on the client's thread) ──
    QVariantMap connectLidar(const QVariantMap &cfg);
    QVariantMap disconnectLidar();
    void applyConfig(const QVariantMap &patch);
    QVariantMap status() const;
    void clear();
    QVariantMap saveMap();
    /// The renderer started over while the link was up: the origin and the whole map again.
    /// Every IPv4 interface of this machine: [{name, address}]. Any thread.
    static QVariantList listInterfaces();
    static QString lidarDir();

    /// A decoded message (ATTITUDE, GLOBAL_POSITION_INT, GPS_RAW_INT, EKF_STATUS_REPORT feed the pose).
    void onMavlink(const mavlink_message_t &msg, qint64 now);

    // ── Offline tests drive the pipeline without sockets ──
    void onPointPacket(const QByteArray &buf, qint64 rxTime);
    void processQueue(qint64 now);
    Gate evaluateGate(qint64 now) const;
    void setConnectedForTest(bool v);
    void resetMap();
    const std::vector<MapChunk> &mapChunks() const { return m_mapChunks; }
    int mapCount() const { return m_mapCount; }
    std::optional<Origin> origin() const { return m_origin; }
    const Config &config() const { return m_cfg; }

signals:
    /// Georeferenced points: ENU metres from the origin of `epoch`, intensity 0-255.
    void points(int epoch, const QVector<float> &enu, const QByteArray &intensity);
    /// Points not georeferenced, relative to the vehicle: frame "ned" (levelled) or "body".
    void live(const QString &frame, double ttl, const QVector<float> &xyz, const QByteArray &intensity);
    /// The cloud's origin {lat, lon, alt, mPerLat, mPerLon, epoch}, empty when cleared.
    void originChanged(const QVariantMap &origin);
    void statusChanged(const QVariantMap &status);

private:
    struct PoseRing {
        static constexpr int kCap = 1024;
        std::array<double, kCap> t{};
        int n = 0, head = 0;
        int newest() const { return (head - 1 + kCap) % kCap; }
        int findBefore(double time) const;
    };
    struct Pose {
        double lat, lon, alt, roll, pitch, yaw;
    };
    struct Packet {
        QByteArray buf;
        qint64 t;
    };
    struct Ply {
        std::unique_ptr<QFile> file;
        QString path;
        qint64 count = 0;
        qint64 countPos = 0;
        bool withTime = false;
        QByteArray buf;
    };

    std::optional<Pose> poseAt(double time) const;
    void flushBatch();
    void batchPush(float e, float n, float u, std::uint8_t intensity);
    void mapPush(float e, float n, float u, std::uint8_t intensity);
    void ensureVoxelTable();
    bool claimVoxel(double e, double n, double u);
    void livePacket(const QByteArray &buf, int dotNum, int dataType, qint64 now);
    void georeferencePacket(const QByteArray &buf, int dotNum, int dataType, double vehE, double vehN, double vehU, qint64 rxTime);
    std::optional<Ply> openPly(const QString &path, bool withTime) const;
    static void plyWrite(Ply &w, float e, float n, float u, std::uint8_t intensity, double tSec);
    static void plyFlush(Ply &w);
    static void closePly(Ply &w);
    QString startRecording();
    QString stopRecording();
    QString pickHostIp(const QString &lidarIp) const;
    void handshakeTick();
    void sendConfig();
    void onSearchReply(QUdpSocket *s);
    void onCmdReply(QUdpSocket *s);
    void onPointReady(QUdpSocket *s);
    void rollStats();
    void sendStatus() { emit statusChanged(status()); }
    void sendOrigin();
    void resetPose();
    void closeSockets();

    Config m_cfg;
    bool m_connected = false;
    struct LidarInfo {
        QString ip, sn;
        int devType = 0;
        bool configured = false;
        qint64 lastAck = 0, lastPacket = 0;
    } m_lidar;
    QUdpSocket *m_search = nullptr, *m_cmd = nullptr, *m_push = nullptr, *m_point = nullptr, *m_imu = nullptr;
    QTimer *m_handshakeTimer = nullptr, *m_processTimer = nullptr, *m_statusTimer = nullptr;

    // Packets are held ~lag + 60 ms before georeferencing so the pose buffer has
    // samples on both sides of the measurement time (interpolation, not extrapolation).
    std::deque<Packet> m_queue;

    // Pose ring buffers (host arrival time in ms)
    PoseRing m_att, m_pos;
    std::array<float, PoseRing::kCap> m_attR{}, m_attP{}, m_attY{};
    std::array<double, PoseRing::kCap> m_posLat{}, m_posLon{}, m_posAlt{};
    std::array<float, PoseRing::kCap> m_posVn{}, m_posVe{}, m_posVd{};
    struct Nav {
        int fix = 0, sats = 0;
        double hdop = 99;
        qint64 gpsTime = 0;
        int ekfFlags = 0;
        double ekfPosVar = 0, ekfVelVar = 0;
        qint64 ekfTime = 0;
    } m_nav;

    // Cloud origin - set at the first accepted point, cleared with the map. The
    // epoch stamps every origin/points event so the renderer can drop a batch
    // that was already in flight when the map was cleared.
    std::optional<Origin> m_origin;
    int m_epoch = 0;

    // Voxel occupancy: open-addressing hash of packed voxel indices (17 bits per axis)
    std::vector<double> m_voxelKeys;   // 0 = empty, stored key + 1
    std::uint64_t m_voxelMask = 0;
    int m_voxelCount = 0;

    // Accumulated map (ENU metres from origin + intensity), chunked so it can
    // grow without copying; this is what SAVE MAP writes out.
    std::vector<MapChunk> m_mapChunks;
    int m_mapCount = 0;

    // Batch towards the renderer
    std::vector<float> m_batchXyz;
    std::vector<std::uint8_t> m_batchInt;
    int m_batchN = 0;
    qint64 m_lastFlush = 0;
    qint64 m_now = 0;   // the clock of the last processQueue() (tests run on a fake one)

    // Live (non-georeferenced) batch: vehicle-relative points, subsampled
    std::vector<float> m_liveXyz;
    std::vector<std::uint8_t> m_liveInt;
    int m_liveN = 0;
    QString m_liveFrame = QStringLiteral("body");   // "ned" when attitude is fresh, "body" otherwise

    std::array<double, 9> m_mount{}, m_attM{};
    bool m_mountDirty = true;

    // Stats (per second)
    struct Counts {
        qint64 packets = 0, points = 0, accepted = 0, gated = 0, filtered = 0, live = 0, bytes = 0;
    } m_stats;
    QVariantMap m_statsShown;
    qint64 m_statsT0 = 0;
    Gate m_gate;
    QString m_lastError;

    std::optional<Ply> m_rec;
};

} // namespace corv
