// tst_lidar - port of scripts/test-lidar-math.js: the Livox SDK2 framing and
// the georeferencing pipeline of LidarCore without sockets (fake MAVLink pose
// tap, fake point packets, a fake clock), plus the PointCloudData the 3D view
// draws (heights, live ring, epochs).
#include "link/Mavlink.h"
#include "scene/PointCloudData.h"
#include "sensors/LidarCore.h"
#include "sensors/LivoxProtocol.h"

#include <QTest>
#include <QtEndian>
#include <cmath>

using namespace corv;

namespace {
bool near(double a, double b, double tol) { return std::abs(a - b) <= tol; }

struct P {
    double lat = 45.0, lon = 9.0, alt = 200, roll = 0, pitch = 0, yaw = 0;
};

QByteArray makePacket(const std::vector<std::array<int, 3>> &pointsMm, int dataType = 1)
{
    const int stride = 14;
    QByteArray buf(36 + int(pointsMm.size()) * stride, '\0');
    auto *b = reinterpret_cast<uchar *>(buf.data());
    qToLittleEndian<quint16>(quint16(buf.size()), b + 1);
    qToLittleEndian<quint16>(quint16(pointsMm.size()), b + 5);
    b[10] = uchar(dataType);
    for (std::size_t k = 0; k < pointsMm.size(); ++k) {
        uchar *o = b + 36 + k * stride;
        qToLittleEndian<qint32>(pointsMm[k][0], o);
        qToLittleEndian<qint32>(pointsMm[k][1], o + 4);
        qToLittleEndian<qint32>(pointsMm[k][2], o + 8);
        o[12] = 100;
        o[13] = 0;
    }
    return buf;
}

class Rig
{
public:
    LidarCore core;
    qint64 now = 1'000'000;
    int pointBatches = 0, originEvents = 0, pointsInBatches = 0, batchEpoch = -1, originEpoch = -2;
    QString liveFrame;
    QVector<float> liveXyz;

    Rig()
    {
        QObject::connect(&core, &LidarCore::points, [this](int epoch, const QVector<float> &enu, const QByteArray &) {
            ++pointBatches;
            pointsInBatches += int(enu.size() / 3);
            batchEpoch = epoch;
        });
        QObject::connect(&core, &LidarCore::originChanged, [this](const QVariantMap &o) {
            ++originEvents;
            originEpoch = o.value(QStringLiteral("epoch")).toInt();
        });
        QObject::connect(&core, &LidarCore::live, [this](const QString &frame, double, const QVector<float> &xyz, const QByteArray &) {
            liveFrame = frame;
            liveXyz = xyz;
        });
    }
    void feedPose(const P &p)
    {
        mavlink_message_t m;
        mavlink_msg_attitude_pack(1, 1, &m, 0, float(p.roll), float(p.pitch), float(p.yaw), 0, 0, 0);
        core.onMavlink(m, now);
        mavlink_msg_global_position_int_pack(1, 1, &m, 0, qint32(std::lround(p.lat * 1e7)), qint32(std::lround(p.lon * 1e7)),
                                             qint32(std::lround(p.alt * 1000)), 0, 0, 0, 0, 0);
        core.onMavlink(m, now);
        mavlink_msg_gps_raw_int_pack(1, 1, &m, 0, 3, 0, 0, 0, 90, 65535, 65535, 65535, 12, 0, 0, 0, 0, 0, 0);
        core.onMavlink(m, now);
        mavlink_msg_ekf_status_report_pack(1, 1, &m, 0x1FF & ~0x80, 0.1f, 0.1f, 0.1f, 0.1f, 0, 0);
        core.onMavlink(m, now);
    }
    void reset(const QVariantMap &cfg)
    {
        core.setConnectedForTest(true);
        core.resetMap();
        pointBatches = originEvents = pointsInBatches = 0;
        liveFrame.clear();
        liveXyz.clear();
        core.applyConfig(cfg);
    }
    std::vector<std::array<double, 4>> run(const QVariantMap &cfg, const P &pose, const std::vector<std::array<int, 3>> &pointsMm)
    {
        reset(cfg);
        // Two pose samples around the packet time so interpolation is exercised
        feedPose(pose);
        now += 100;
        feedPose(pose);
        now += 50;
        core.onPointPacket(makePacket(pointsMm), now);
        now += 100;
        feedPose(pose);
        now += 200;
        core.processQueue(now);
        return points();
    }
    std::vector<std::array<double, 4>> points() const
    {
        std::vector<std::array<double, 4>> pts;
        for (const auto &c : core.mapChunks())
            for (int k = 0; k < c.n; ++k) pts.push_back({c.xyz[std::size_t(k) * 3], c.xyz[std::size_t(k) * 3 + 1], c.xyz[std::size_t(k) * 3 + 2], double(c.i[std::size_t(k)])});
        return pts;
    }
};

QVariantMap base()
{
    return {{QStringLiteral("voxel"), 0.1},      {QStringLiteral("minRange"), 0.5}, {QStringLiteral("maxRange"), 100},
            {QStringLiteral("dropNoise"), true}, {QStringLiteral("requireEkf"), true}, {QStringLiteral("minFix"), 3},
            {QStringLiteral("minSats"), 8},      {QStringLiteral("maxHdop"), 2},      {QStringLiteral("lagMs"), 0},
            {QStringLiteral("mountRoll"), 0},    {QStringLiteral("mountPitch"), 0},   {QStringLiteral("mountYaw"), 0},
            {QStringLiteral("leverX"), 0},       {QStringLiteral("leverY"), 0},       {QStringLiteral("leverZ"), 0}};
}
QVariantMap with(QVariantMap m, const char *k, const QVariant &v)
{
    m.insert(QLatin1String(k), v);
    return m;
}
} // namespace

class TstLidar : public QObject
{
    Q_OBJECT
private slots:
    void framing()
    {
        const QByteArray kv = livox::buildKvRequest({{0x001A, QByteArray(1, 1)}});
        QCOMPARE(kv, QByteArray::fromHex("010000001a00010001"));
        const QByteArray frame = livox::packCommand(0x0100, kv);
        QCOMPARE(int(qFromLittleEndian<quint16>(frame.constData() + 2)), int(frame.size()));
        QCOMPARE(frame.size(), 24 + kv.size());
        const auto parsed = livox::parseCommand(frame);
        QVERIFY(parsed);
        QCOMPARE(parsed->cmdId, 0x0100);
        QCOMPARE(parsed->data, kv);
        QByteArray bad = frame;
        bad[26] = char(bad[26] ^ 0xFF);
        QVERIFY(!livox::parseCommand(bad));
        const QByteArray check("123456789");
        QCOMPARE(livox::crc16(reinterpret_cast<const uchar *>(check.constData()), 9), quint16(0x29B1));
        QCOMPARE(livox::crc32(reinterpret_cast<const uchar *>(check.constData()), 9), quint32(0xCBF43926));
    }

    // *_HOST_IP value: IPv4, then the host port (the bound one, maybe not 56301) and the LiDAR port, LE
    void hostIpValue()
    {
        QCOMPARE(livox::hostIpValue(QStringLiteral("192.168.1.50"), 50123, livox::kLidarPoint), QByteArray::fromHex("c0a80132cbc3ecdb"));
    }

    // The occupancy table is 2 x MAX POINTS doubles: a typo must not size it past memory
    void configLimits()
    {
        LidarCore core;
        core.applyConfig({{QStringLiteral("maxPoints"), 1e20}, {QStringLiteral("voxel"), 0.0001}});
        QCOMPARE(core.config().maxPoints, 50000000.0);
        QCOMPARE(core.config().voxel, 0.01);
        core.applyConfig({{QStringLiteral("maxPoints"), -5}, {QStringLiteral("voxel"), 0}});
        QCOMPARE(core.config().maxPoints, 1000.0);
        QCOMPARE(core.config().voxel, 0.25);
    }

    // Level, heading north, upright mount: lidar +X -> north, +Y -> west, +Z -> up
    void identity()
    {
        Rig r;
        const auto pts = r.run(base(), P{}, {{10000, 0, 0}, {0, 10000, 0}, {0, 0, 10000}});
        QCOMPARE(int(pts.size()), 3);
        QVERIFY(near(pts[0][1], 10, 0.01) && near(pts[0][0], 0, 0.01) && near(pts[0][2], 0, 0.01));
        QVERIFY(near(pts[1][0], -10, 0.01) && near(pts[1][1], 0, 0.01));
        QVERIFY(near(pts[2][2], 10, 0.01));
    }

    void yaw90()
    {
        Rig r;
        P p;
        p.yaw = M_PI / 2;
        const auto pts = r.run(base(), p, {{10000, 0, 0}});
        QCOMPARE(int(pts.size()), 1);
        QVERIFY(near(pts[0][0], 10, 0.01) && near(pts[0][1], 0, 0.01));
    }

    // Inverted mount (roll 180): lidar +Z -> down, +Y -> east (right)
    void invertedMount()
    {
        Rig r;
        const auto pts = r.run(with(base(), "mountRoll", 180), P{}, {{0, 0, 10000}, {0, 10000, 0}});
        QCOMPARE(int(pts.size()), 2);
        QVERIFY(near(pts[0][2], -10, 0.01));
        QVERIFY(near(pts[1][0], 10, 0.01));
    }

    // Rolled 30 deg right, inverted mount, a point "up" the lidar (= body down):
    // the belly faces left, so the point is 5 m WEST and 8.66 m below
    void rollAndInverted()
    {
        Rig r;
        P p;
        p.roll = 30 * M_PI / 180;
        const auto pts = r.run(with(base(), "mountRoll", 180), p, {{0, 0, 10000}});
        QCOMPARE(int(pts.size()), 1);
        QVERIFY2(near(pts[0][0], -5, 0.02) && near(pts[0][2], -8.66, 0.02), qPrintable(QStringLiteral("%1 %2").arg(pts[0][0]).arg(pts[0][2])));
    }

    void leverArm()
    {
        Rig r;
        const auto pts = r.run(with(base(), "leverX", 1), P{}, {{10000, 0, 0}});
        QCOMPARE(int(pts.size()), 1);
        QVERIFY(near(pts[0][1], 11, 0.01));
    }

    // Range gate, noise tag, voxel dedup
    void filters()
    {
        Rig r;
        r.reset(with(with(base(), "minRange", 2), "maxRange", 20));
        r.feedPose(P{});
        r.now += 100;
        r.feedPose(P{});
        r.now += 50;
        r.core.onPointPacket(makePacket({{1000, 0, 0}, {30000, 0, 0}, {5000, 0, 0}, {5010, 0, 0}, {0, 0, 0}}), r.now);
        r.now += 300;
        r.feedPose(P{});
        r.core.processQueue(r.now);
        QCOMPARE(r.core.mapCount(), 1);
        QByteArray noisy = makePacket({{8000, 0, 0}});
        noisy[36 + 13] = 0x01;   // spatial noise tag
        r.core.onPointPacket(noisy, r.now);
        r.now += 300;
        r.feedPose(P{});
        r.core.processQueue(r.now);
        QCOMPARE(r.core.mapCount(), 1);
    }

    // RTK fixed required with only a 3D fix: nothing accumulates, the reason says why;
    // the packet still shows as live points, levelled (attitude fresh): heading east, +X east
    void gateAndLive()
    {
        Rig r;
        P p;
        p.yaw = M_PI / 2;
        r.reset(with(base(), "minFix", 6));
        r.feedPose(p);
        r.now += 100;
        r.feedPose(p);
        r.now += 50;
        r.core.onPointPacket(makePacket({{10000, 0, 0}}), r.now);
        r.now += 300;
        r.feedPose(p);
        r.core.processQueue(r.now);
        const LidarCore::Gate g = r.core.evaluateGate(r.now);
        QCOMPARE(r.core.mapCount(), 0);
        QVERIFY(!g.ok);
        QVERIFY2(g.reason.contains(QLatin1String("GPS FIX")), qPrintable(g.reason));
        QCOMPARE(r.liveFrame, QStringLiteral("ned"));
        QCOMPARE(int(r.liveXyz.size()), 3);
        QVERIFY(near(r.liveXyz[1], 10, 0.01) && near(r.liveXyz[0], 0, 0.01));
    }

    void batchMatchesMap()
    {
        Rig r;
        const auto pts = r.run(base(), P{}, {{10000, 0, 0}, {0, 10000, 0}});
        QVERIFY(r.pointBatches > 0 && r.originEvents == 1);
        QCOMPARE(r.pointsInBatches, int(pts.size()));
        QCOMPARE(r.batchEpoch, r.originEpoch);
    }

    // The pose changes between samples; with lagMs the packet takes the later pose
    void telemetryLag()
    {
        Rig r;
        P north, east;
        east.yaw = M_PI / 2;
        r.reset(with(with(base(), "lagMs", 200), "voxel", 0.05));
        r.feedPose(north);
        r.now += 50;
        r.core.onPointPacket(makePacket({{10000, 0, 0}}), r.now);
        r.now += 200;
        r.feedPose(east);
        r.now += 100;
        r.feedPose(east);
        r.now += 100;
        r.core.processQueue(r.now);
        const auto pts = r.points();
        QCOMPARE(int(pts.size()), 1);
        QVERIFY(near(pts[0][0], 10, 0.05));
    }

    // The cloud the 3D view draws: axes, the epoch guard, the live ring and the height ramp
    void pointCloudData()
    {
        PointCloudData &d = PointCloudData::instance();
        d.clear();
        d.setOrigin({45, 9, 200, 111132, 78847, 7});
        const float enu[] = {1, 2, 3, 4, 5, 6};
        const uchar in[] = {255, 0};
        d.appendMapped(6, enu, in, 2);   // an older epoch: dropped
        QCOMPARE(d.total, 0);
        d.appendMapped(7, enu, in, 2);
        QCOMPARE(d.total, 2);
        const float *p = d.chunks[0]->pts.get();
        QCOMPARE(p[0], 1.f);    // x east
        QCOMPARE(p[1], 3.f);    // y up
        QCOMPARE(p[2], -2.f);   // z south
        QCOMPARE(p[3], 1.f);
        const int gen = d.generation;
        d.setOrigin({45, 9, 200, 111132, 78847, 8});   // a new epoch clears the map
        QCOMPARE(d.total, 0);
        QCOMPARE(d.generation, gen + 1);
        const float ned[] = {10, 0, -5};   // 10 m north, 5 m up
        d.appendLive(ned, nullptr, 1, QStringLiteral("ned"), 4);
        QVERIFY(!d.liveBody);
        QCOMPARE(d.liveTtl, 4.0);
        const int slot = (d.liveHead - 1 + PointCloudData::kLiveCapacity) % PointCloudData::kLiveCapacity;
        QCOMPARE(d.livePts[std::size_t(slot) * 4 + 1], 5.f);
        QCOMPARE(d.livePts[std::size_t(slot) * 4 + 2], -10.f);
        std::vector<float> many;
        for (int i = 0; i < 100; ++i) many.insert(many.end(), {0.f, 0.f, float(i)});
        d.setOrigin({45, 9, 200, 111132, 78847, 9});
        d.appendMapped(9, many.data(), nullptr, 100);
        d.updateRange(1e9);
        QCOMPARE(d.heightRange[0], 1.f);
        QCOMPARE(d.heightRange[1], 97.f);
    }
};

QTEST_GUILESS_MAIN(TstLidar)
#include "tst_lidar.moc"
