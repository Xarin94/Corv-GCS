// tst_rossurface - port of the offline parts of scripts/test-ros-surface.js:
// SurfaceTiles averaging, budget and changes; RosPoints decoding of
// PointCloud2 / LaserScan / Range (CBOR byte strings, base64, typed arrays,
// JSON nulls), georeferencing and the projected pose; the spike filter; the
// roughness and fill levels of SurfaceRaster and the coverage of an area;
// SurfaceVolume meshing a wall; the RosSurfaceData blocks the 3D view draws.
#include "scene/RosSurfaceData.h"
#include "scene/SurfaceRaster.h"
#include "scene/SurfaceTiles.h"
#include "scene/SurfaceVolume.h"
#include "sensors/RosPoints.h"

#include <QCborArray>
#include <QJsonArray>
#include <QJsonObject>
#include <QRandomGenerator>
#include <QTest>
#include <QtEndian>
#include <algorithm>
#include <cmath>
#include <set>

using namespace corv;

namespace {
constexpr double DEG = M_PI / 180;
bool near(double a, double b, double tol) { return std::abs(a - b) <= tol; }
// Seeded: the same numbers on every run (a run of 5 aerated pings in a row,
// which a 9-beam median keeps by design, would otherwise turn up now and then)
QRandomGenerator g_rng(20261117);
double rnd() { return g_rng.generateDouble(); }
constexpr int TILE = SurfaceTiles::kTile;

struct Cell {
    double x, y, h, w;
};
std::vector<Cell> cellsOf(const SurfaceTiles &st)
{
    std::vector<Cell> out;
    const double c = st.cell();
    for (const auto &[key, t] : st.tiles())
        for (int k = 0; k < TILE * TILE; ++k) {
            if (!(t->w[std::size_t(k)] > 0)) continue;
            const int i = k % TILE, j = k / TILE;
            out.push_back({(t->tx * TILE + i + 0.5) * c, (t->ty * TILE + j + 0.5) * c, t->h[std::size_t(k)], t->w[std::size_t(k)]});
        }
    return out;
}

QCborMap livoxCloud(const std::vector<std::array<float, 3>> &points, bool asBase64)
{
    const int step = 26;
    QByteArray buf(int(points.size()) * step, '\0');
    auto *b = reinterpret_cast<uchar *>(buf.data());
    for (std::size_t k = 0; k < points.size(); ++k) {
        uchar *o = b + k * step;
        qToLittleEndian<float>(points[k][0], o);
        qToLittleEndian<float>(points[k][1], o + 4);
        qToLittleEndian<float>(points[k][2], o + 8);
        qToLittleEndian<float>(99, o + 12);
        o[16] = 7;
        qToLittleEndian<double>(1e18, o + 18);
    }
    QCborArray fields;
    auto f = [](const char *name, int offset, int datatype) {
        return QCborMap{{QStringLiteral("name"), QString::fromLatin1(name)}, {QStringLiteral("offset"), offset}, {QStringLiteral("datatype"), datatype}};
    };
    fields << f("x", 0, 7) << f("y", 4, 7) << f("z", 8, 7) << f("intensity", 12, 7) << f("tag", 16, 2) << f("timestamp", 18, 8);
    QCborMap m{{QStringLiteral("header"), QCborMap{{QStringLiteral("frame_id"), QStringLiteral("livox_frame")}}},
               {QStringLiteral("height"), 1},
               {QStringLiteral("width"), qint64(points.size())},
               {QStringLiteral("point_step"), step},
               {QStringLiteral("row_step"), qint64(step * points.size())},
               {QStringLiteral("is_bigendian"), false},
               {QStringLiteral("fields"), fields}};
    if (asBase64) m.insert(QStringLiteral("data"), QString::fromLatin1(buf.toBase64()));
    else m.insert(QStringLiteral("data"), buf);
    return m;
}

QCborValue floats32Tag(const std::vector<float> &v)
{
    QByteArray b(int(v.size()) * 4, '\0');
    for (std::size_t i = 0; i < v.size(); ++i) qToLittleEndian<float>(v[i], b.data() + i * 4);
    return QCborValue(QCborTag(85), b);
}
} // namespace

class TstRosSurface : public QObject
{
    Q_OBJECT
private slots:
    void tilesPlane()
    {
        SurfaceTiles st;
        st.configure(0.3, 20, NAN);
        const int N = 60000;
        std::vector<double> pts(std::size_t(N) * 3);
        for (int k = 0; k < N; ++k) {
            const double x = (rnd() - 0.5) * 20, y = (rnd() - 0.5) * 20;
            pts[std::size_t(k) * 3] = x;
            pts[std::size_t(k) * 3 + 1] = y;
            pts[std::size_t(k) * 3 + 2] = 5 + 0.1 * x - 0.05 * y;
        }
        const int used = st.add(pts.data(), N);
        double worst = 0;
        const auto cells = cellsOf(st);
        for (const Cell &p : cells) worst = std::max(worst, std::abs(p.h - (5 + 0.1 * p.x - 0.05 * p.y)));
        QVERIFY2(worst < 0.03 && used == N, qPrintable(QString::number(worst)));
        QCOMPARE(st.size(), 16);
        QCOMPARE(st.filled(), int(cells.size()));
    }

    void tilesIndexingAndChanges()
    {
        SurfaceTiles st;
        const double three[] = {-0.1, -0.1, 1, 8.99, 0.1, 2, 9.0, 0.1, 3};
        st.add(three, 3);
        const auto &tiles = st.tiles();
        const auto t00 = tiles.find(SurfaceTiles::tileKey(-1, -1)), t0 = tiles.find(SurfaceTiles::tileKey(0, 0)), t1 = tiles.find(SurfaceTiles::tileKey(1, 0));
        QVERIFY(t00 != tiles.end() && t00->second->w[29 * 30 + 29] > 0);
        QVERIFY(t0 != tiles.end() && t0->second->w[29] > 0);
        QVERIFY(t1 != tiles.end() && t1->second->w[0] > 0);
        QCOMPARE(st.size(), 3);
        const auto first = st.takeChanges(), second = st.takeChanges();
        QCOMPARE(int(first.tiles.size()), 3);
        QCOMPARE(int(second.tiles.size()), 0);
        const double one[] = {0.1, 0.1, 5};
        st.add(one, 1);
        const auto third = st.takeChanges();
        QCOMPARE(int(third.tiles.size()), 1);
        QCOMPARE(third.tiles[0].tx, 0);
        QVERIFY(near(third.tiles[0].h[0], 5, 1e-6));
        QVERIFY(near(st.tiles().at(SurfaceTiles::tileKey(0, 0))->w[0], 1.0 / 3, 1e-6));   // one point = 1/3 of a sample
        QVERIFY(st.configure(0.5, NAN, NAN) && st.size() == 0 && st.cell() == 0.5);        // a new cell size clears
    }

    // A cell follows a change once its memory is full: moving average
    void tilesMovingAverage()
    {
        SurfaceTiles st;
        st.configure(1, 10, NAN);
        auto feed = [&](double u) {
            const double p[] = {0.2, 0.2, u, 0.3, 0.3, u, 0.25, 0.25, u, 0.2, 0.3, u};
            st.add(p, 4);
        };
        for (int k = 0; k < 50; ++k) feed(0);
        const double settled = st.tiles().at(SurfaceTiles::tileKey(0, 0))->h[0];
        for (int k = 0; k < 30; ++k) feed(10);
        const double a = (2.0 / 3) / (10 + 2.0 / 3), expect = 10 * (1 - std::pow(1 - a, 30));
        QCOMPARE(settled, 0.0);
        QVERIFY(near(st.tiles().at(SurfaceTiles::tileKey(0, 0))->h[0], expect, 1e-6));
    }

    // 40 tiles in a row, room for 10: the ones nearest the vehicle stay
    void tilesBudget()
    {
        SurfaceTiles st;
        st.configure(0.3, NAN, 10);
        for (int k = 0; k < 40; ++k) {
            const double p[] = {k * 9 + 4.5, 4.5, 0};
            st.add(p, 1, 0, 0);
        }
        std::vector<int> kept;
        for (const auto &[key, t] : st.tiles()) kept.push_back(t->tx);
        std::sort(kept.begin(), kept.end());
        const auto ch = st.takeChanges();
        QVERIFY(st.size() <= 10);
        for (int i = 0; i < 9; ++i) QCOMPARE(kept[std::size_t(i)], i);
        QCOMPARE(int(ch.removed.size()), 40 - st.size());
    }

    void kindsAndFrames()
    {
        QCOMPARE(rospts::kindOf(QStringLiteral("sensor_msgs/PointCloud2")), QStringLiteral("cloud"));
        QCOMPARE(rospts::kindOf(QStringLiteral("sensor_msgs/msg/LaserScan")), QStringLiteral("scan"));
        QCOMPARE(rospts::kindOf(QStringLiteral("sensor_msgs/msg/Range")), QStringLiteral("range"));
        QVERIFY(rospts::kindOf(QStringLiteral("sensor_msgs/msg/Image")).isEmpty());
        QCOMPARE(rospts::frameOf(QStringLiteral("map")), QStringLiteral("world-enu"));
        QCOMPARE(rospts::frameOf(QStringLiteral("odom_ned")), QStringLiteral("world-ned"));
        QCOMPARE(rospts::frameOf(QStringLiteral("/odom")), QStringLiteral("world-enu"));
        QCOMPARE(rospts::frameOf(QStringLiteral("livox_frame")), QStringLiteral("sensor"));
        QCOMPARE(rospts::frameOf(QStringLiteral("base_link")), QStringLiteral("sensor"));
        QCOMPARE(rospts::frameOf(QStringLiteral("camera_init"), QStringLiteral("world")), QStringLiteral("world-enu"));
        QCOMPARE(rospts::frameOf(QStringLiteral("map"), QStringLiteral("sensor")), QStringLiteral("sensor"));
    }

    void pointCloud2()
    {
        std::vector<float> out(3000);
        const std::vector<std::array<float, 3>> pts{{10, 1, -2}, {0, 0, 0}, {1, 0, 0}, {NAN, 1, 1}, {300, 0, 0}, {-5, 5, 5}};
        const auto a = rospts::decodePoints(QStringLiteral("cloud"), livoxCloud(pts, false), out.data(), 1000, 100, 2, 100);
        QCOMPARE(a.total, 6);
        QCOMPARE(a.count, 2);
        QVERIFY(near(out[0], 10, 1e-5) && near(out[2], -2, 1e-5) && near(out[3], -5, 1e-5));
        const auto b = rospts::decodePoints(QStringLiteral("cloud"), livoxCloud(pts, true), out.data(), 1000, 100, 2, 100);
        QVERIFY(b.count == 2 && near(out[0], 10, 1e-5));
        std::vector<std::array<float, 3>> many;
        for (int k = 0; k < 1000; ++k) many.push_back({10 + k * 0.01f, 0, 0});
        const auto c = rospts::decodePoints(QStringLiteral("cloud"), livoxCloud(many, false), out.data(), 1000, 100, 0);
        bool increasing = true;
        for (int k = 1; k < c.count; ++k)
            if (!(out[std::size_t(k) * 3] > out[std::size_t(k - 1) * 3])) increasing = false;
        QVERIFY(c.count == 100 && increasing && out[std::size_t(c.count - 1) * 3] > 19.8);

        // float64 big endian, organized (height 2)
        QByteArray buf(64, '\0');
        const double v[2][3] = {{1.5, -2.5, 3.5}, {4, 5, 6}};
        for (int k = 0; k < 2; ++k)
            for (int i = 0; i < 3; ++i) qToBigEndian<double>(v[k][i], buf.data() + k * 32 + i * 8);
        QCborArray fields;
        for (int i = 0; i < 3; ++i)
            fields << QCborMap{{QStringLiteral("name"), QString(QChar(u'x' + i))}, {QStringLiteral("offset"), i * 8}, {QStringLiteral("datatype"), 8}};
        const QCborMap be{{QStringLiteral("height"), 2}, {QStringLiteral("width"), 1},         {QStringLiteral("point_step"), 32},
                          {QStringLiteral("row_step"), 32}, {QStringLiteral("is_bigendian"), true}, {QStringLiteral("fields"), fields},
                          {QStringLiteral("data"), buf}};
        const auto r = rospts::decodePoints(QStringLiteral("cloud"), be, out.data(), 1000, 10);
        QVERIFY(r.count == 2 && near(out[1], -2.5, 1e-6) && near(out[5], 6, 1e-6));
    }

    void laserScanAndRange()
    {
        std::vector<float> out(3000), times(1000);
        QCborMap scan{{QStringLiteral("angle_min"), -M_PI / 2}, {QStringLiteral("angle_increment"), M_PI / 2},
                      {QStringLiteral("range_min"), 0.1},       {QStringLiteral("range_max"), 30}};
        scan.insert(QStringLiteral("ranges"), floats32Tag({10, INFINITY, 20, 50, NAN}));
        const auto a = rospts::decodePoints(QStringLiteral("scan"), scan, out.data(), 1000, 100);
        QVERIFY(a.total == 5 && a.count == 2);
        QVERIFY(near(out[0], 0, 1e-5) && near(out[1], -10, 1e-5) && near(out[3], 0, 1e-5) && near(out[4], 20, 1e-5));
        // JSON array with nulls, through the JSON -> CBOR conversion the client does
        QJsonObject js{{QStringLiteral("angle_min"), -M_PI / 2}, {QStringLiteral("angle_increment"), M_PI / 2}, {QStringLiteral("range_min"), 0.1},
                       {QStringLiteral("range_max"), 30}, {QStringLiteral("ranges"), QJsonArray{10, QJsonValue(), 20, 50, QJsonValue()}}};
        const auto b = rospts::decodePoints(QStringLiteral("scan"), QCborValue::fromJsonValue(js).toMap(), out.data(), 1000, 100);
        QCOMPARE(b.count, 2);
        // A sector sonar's batch: one beam per ping, 66 ms apart, the head turning back (negative step)
        QCborMap sweep{{QStringLiteral("angle_min"), 30 * DEG}, {QStringLiteral("angle_increment"), -1.8 * DEG}, {QStringLiteral("time_increment"), 0.066},
                       {QStringLiteral("range_min"), 0.75},      {QStringLiteral("range_max"), 40}};
        sweep.insert(QStringLiteral("ranges"), floats32Tag({20, INFINITY, 19, 18.5f, 0.2f, 18}));
        const auto t = rospts::decodePoints(QStringLiteral("scan"), sweep, out.data(), 1000, 100, 0.5, 100, times.data());
        QCOMPARE(t.count, 4);
        QVERIFY(near(t.span, 5 * 0.066, 1e-9));
        QVERIFY(near(times[0], 0, 1e-9) && near(times[1], 0.132, 1e-6) && near(times[3], 0.33, 1e-6));
        QVERIFY(near(std::atan2(out[4], out[3]) / DEG, 30 - 2 * 1.8, 1e-4));
        sweep.insert(QStringLiteral("time_increment"), 0);
        QCOMPARE(rospts::decodePoints(QStringLiteral("scan"), sweep, out.data(), 1000, 100, 0.5, 100, times.data()).span, 0.0);

        auto range = [&](const QCborValue &r) {
            return rospts::decodePoints(QStringLiteral("range"), QCborMap{{QStringLiteral("range"), r}, {QStringLiteral("min_range"), 0.5}, {QStringLiteral("max_range"), 50}},
                                        out.data(), 1000, 10).count;
        };
        QCOMPARE(range(12.5), 1);
        QCOMPARE(range(double(INFINITY)), 0);
        QCOMPARE(range(0.2), 0);
        QCOMPARE(range(QCborValue(QCborValue::Null)), 0);
    }

    void georeferencing()
    {
        const auto anchor = rospts::makeAnchor(45.6, 10.67, 100);
        const auto veh = rospts::toEnu(anchor, 45.6, 10.67, 150);
        const auto Ra = rospts::eulerToMatrix(0, 0, 0);
        const auto down = rospts::eulerToMatrix(0, -90 * DEG, 0);
        double out[9];
        const float r50[] = {50, 0, 0};
        rospts::sensorToEnu(r50, 1, down, {0, 0, 0}, Ra, veh, out);
        QVERIFY(near(out[0], 0, 1e-6) && near(out[1], 0, 1e-6) && near(out[2], 0, 1e-6));   // a down-looking range 50 m from 50 m up: the ground
        const float z10[] = {0, 0, 10};
        rospts::sensorToEnu(z10, 1, rospts::eulerToMatrix(M_PI, 0, 0), {0, 0, 0}, Ra, veh, out);
        QVERIFY(near(out[2], 40, 1e-6) && near(out[0], 0, 1e-6));   // inverted Livox: +z is down
        const float x10[] = {10, 0, 0};
        rospts::sensorToEnu(x10, 1, rospts::eulerToMatrix(0, 0, 0), {0, 0, 0}, rospts::eulerToMatrix(0, 0, 90 * DEG), veh, out);
        QVERIFY(near(out[0], 10, 1e-6) && near(out[1], 0, 1e-6));   // heading east: forward is +E
        const float y10[] = {0, 10, 0};
        rospts::sensorToEnu(y10, 1, rospts::eulerToMatrix(0, 0, 0), {0, 0, 0}, Ra, veh, out);
        QVERIFY(near(out[0], -10, 1e-6));   // +y is left
        const float x5[] = {5, 0, 0};
        rospts::sensorToEnu(x5, 1, down, {1, 0.5, 0.3}, Ra, veh, out);
        QVERIFY(near(out[0], 0.5, 1e-6) && near(out[1], 1, 1e-6) && near(out[2], 50 - 0.3 - 5, 1e-6));   // lever arm in FRD
        rospts::sensorToEnu(r50, 1, down, {0, 0, 0}, rospts::eulerToMatrix(10 * DEG, 0, 0), veh, out);
        QVERIFY(out[0] < -8 && near(out[2], 50 - 50 * std::cos(10 * DEG), 1e-6));   // rolled right 10 deg: the beam lands west

        // 0.5 s ahead at 90 deg/s of yaw, level: 45 deg; 10 m/s north: 5 m
        const auto pr = rospts::projectPose({0, 0, 0, 0, 0, 0, 90 * DEG}, {0, 45, 10, 100, 10, 0, -2}, 500);
        QVERIFY(near(pr.yaw, 45 * DEG, 1e-9) && near((pr.lat - 45) * 111320, 5, 1e-6) && near(pr.alt, 101, 1e-9));
        // A SITL at speedup 20: 25 ms here are 0.5 s of the autopilot's rates and velocity
        const auto pk = rospts::projectPose({0, 0, 0, 0, 0, 0, 90 * DEG}, {0, 45, 10, 100, 10, 0, 0}, 25, 20);
        QVERIFY(near(pk.yaw, 45 * DEG, 1e-9) && near((pk.lat - 45) * 111320, 5, 1e-6));
        // Banked, constant body rates for 1 s, against the exact rotation R0 * exp([w]x t)
        const double w[3] = {5 * DEG, -3 * DEG, 20 * DEG};
        const auto pb = rospts::projectPose({0, 30 * DEG, 10 * DEG, 0, w[0], w[1], w[2]}, {0, 0, 0, 0, 0, 0, 0}, 1000);
        const auto R0 = rospts::eulerToMatrix(30 * DEG, 10 * DEG, 0);
        const double th = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]), k[3] = {w[0] / th, w[1] / th, w[2] / th};
        const double K[9] = {0, -k[2], k[1], k[2], 0, -k[0], -k[1], k[0], 0};
        double E[9], R1[9];
        for (int i = 0; i < 9; ++i) {
            const int r = i / 3, c = i % 3;
            const double k2 = K[r * 3] * K[c] + K[r * 3 + 1] * K[3 + c] + K[r * 3 + 2] * K[6 + c];
            E[i] = (i % 4 == 0 ? 1 : 0) + std::sin(th) * K[i] + (1 - std::cos(th)) * k2;
        }
        for (int i = 0; i < 9; ++i) {
            const int r = i / 3, c = i % 3;
            R1[i] = R0[std::size_t(r * 3)] * E[c] + R0[std::size_t(r * 3 + 1)] * E[3 + c] + R0[std::size_t(r * 3 + 2)] * E[6 + c];
        }
        const double err = std::max({std::abs(pb.roll - std::atan2(R1[7], R1[8])), std::abs(pb.pitch + std::asin(R1[6])), std::abs(pb.yaw - std::atan2(R1[3], R1[0]))});
        QVERIFY2(err / DEG < 0.2, qPrintable(QString::number(err / DEG)));

        const float w123[] = {1, 2, 3};
        rospts::worldToEnu(w123, 1, false, {10, 20, 30}, out);
        QVERIFY(out[0] == 11 && out[1] == 22 && out[2] == 33);
        rospts::worldToEnu(w123, 1, true, {10, 20, 30}, out);
        QVERIFY(out[0] == 12 && out[1] == 21 && out[2] == 27);
        const auto a2 = rospts::makeAnchor(47.2603, 11.3439, 0);
        const auto back = rospts::toEnu(a2, 47.2603 + 100 / 111320.0, 11.3439 + 100 / (111320 * std::cos(47.2603 * DEG)), 0);
        QVERIFY(near(back[0], 100, 1e-6) && near(back[1], 100, 1e-6));
    }

    void spikeFilter()
    {
        // A 256-beam fan over a 40 m flat bed: 12 % aeration (0.5-4.5 m), a few fish
        const int n = 256;
        std::vector<float> fan(std::size_t(n) * 3), orig;
        std::set<int> bad;
        for (int i = 0; i < n; ++i) {
            const double a = (-45 + 90 * (i + 0.5) / n) * DEG;
            double d = 40 / std::cos(a) + 0.05 * (rnd() - 0.5);
            // (at most 3 in any 9 beams: the median of 9 drops what is narrower than HALF = 4 beams)
            int recent = 0;
            for (int k = 1; k <= 8; ++k) recent += int(bad.count(i - k));
            if (rnd() < 0.12 && recent < 3) {
                d = 0.5 + 4 * rnd();
                bad.insert(i);
            } else if (i % 97 == 50) {
                d = 15;
                bad.insert(i);
            }
            fan[std::size_t(i) * 3] = float(std::cos(a) * d);
            fan[std::size_t(i) * 3 + 1] = float(std::sin(a) * d);
        }
        orig = fan;
        std::vector<float> times(static_cast<std::size_t>(n));
        for (int i = 0; i < n; ++i) times[std::size_t(i)] = float(i);
        const auto r = rospts::despike(fan.data(), n, 4, 0.5, 0.08, times.data());
        QVERIFY(r.ordered);
        std::set<int> kept;
        for (int k = 0; k < r.count; ++k) {
            const int i = int(times[std::size_t(k)]);
            kept.insert(i);
            QCOMPARE(fan[std::size_t(k) * 3], orig[std::size_t(i) * 3]);   // each kept point keeps its time
        }
        int badKept = 0, goodLost = 0;
        for (int i = 0; i < n; ++i) {
            if (bad.count(i) && kept.count(i)) ++badKept;
            if (!bad.count(i) && !kept.count(i)) ++goodLost;
        }
        QVERIFY2(badKept == 0 && goodLost <= 0.03 * (n - int(bad.size())), qPrintable(QStringLiteral("%1 kept, %2 lost").arg(badKept).arg(goodLost)));
        // The same fan shuffled has no order: nothing is dropped
        std::vector<float> sh = orig;
        for (int i = n - 1; i > 0; --i) {
            const int j = int(rnd() * (i + 1));
            for (int c = 0; c < 3; ++c) std::swap(sh[std::size_t(i) * 3 + std::size_t(c)], sh[std::size_t(j) * 3 + std::size_t(c)]);
        }
        const auto u = rospts::despike(sh.data(), n);
        QVERIFY(!u.ordered && u.removed == 0);
        // A cave profile: a near wall then a far one, the edge between them kept
        std::vector<float> cv(80 * 3);
        for (int i = 0; i < 80; ++i) {
            const double a = i / 80.0 * M_PI, d = i < 40 ? 3 : 15;
            cv[std::size_t(i) * 3] = float(std::cos(a) * d);
            cv[std::size_t(i) * 3 + 1] = float(std::sin(a) * d);
        }
        const auto e = rospts::despike(cv.data(), 80);
        QVERIFY(e.ordered && e.removed == 0);
    }

    void roughnessAndFill()
    {
        const int w = 60, h = 60;
        std::vector<float> view(std::size_t(w * h * 2), 0.f);
        std::vector<std::uint8_t> out(std::size_t(w * h));
        // 1 m cells: a bed sloping 5 % with 2 cm of noise, a 1.5 m boulder at (40, 20), a hole
        for (int j = 0; j < h; ++j)
            for (int i = 0; i < w; ++i)
                view[std::size_t(j * w + i) * 2] = float(-40 + 0.05 * i + 0.02 * j + 0.02 * (rnd() - 0.5) * 2 + 1.5 * std::exp(-((i - 40) * (i - 40) + (j - 20) * (j - 20)) / 4.5));
        for (int j = 45; j < 55; ++j)
            for (int i = 5; i < 15; ++i) view[std::size_t(j * w + i) * 2] = raster::kEmpty;
        raster::roughness(view.data(), w, h, out.data(), 1);
        auto at = [&](int i, int j) { return int(out[std::size_t(j * w + i)]); };
        QVERIFY2(at(10, 10) <= 3 && at(50, 50) <= 3 && at(40, 20) >= 25 && at(10, 50) == 0,
                 qPrintable(QStringLiteral("%1 %2 %3 %4").arg(at(10, 10)).arg(at(50, 50)).arg(at(40, 20)).arg(at(10, 50))));

        QCOMPARE(raster::fillLevelsFor(8, 0.5), 4);
        QCOMPARE(raster::fillLevelsFor(0, 0.3), 2);
        QCOMPARE(raster::fillLevelsFor(2, 0.5), 2);
        QCOMPARE(raster::fillLevelsFor(8, 0.3), 4);
        QCOMPARE(raster::fillLevelsFor(1, 0.5), 1);
        // A sector sonar's sweeps 5 m apart: lines by default, one surface when holes are filled up to 8 m
        const int n = 96;
        std::vector<float> data(std::size_t(n * n), raster::kEmpty), wts(std::size_t(n * n), 0.f), drawn(std::size_t(n * n * 2));
        for (int j = 0; j < n; j += 10)
            for (int i = 0; i < n; ++i) {
                data[std::size_t(j * n + i)] = float(-12 + 0.02 * i + 0.03 * j);
                wts[std::size_t(j * n + i)] = 1;
            }
        auto share = [&](int fillLevels, double *worst) {
            raster::Options o;
            o.cell = 0.5;
            o.fillLevels = fillLevels;
            raster::rasterize(data.data(), wts.data(), n, n, drawn.data(), o);
            int k = 0;
            *worst = 0;
            for (int j = 0; j <= 90; ++j)
                for (int i = 8; i < 88; ++i) {
                    const float v = drawn[std::size_t(j * n + i) * 2];
                    if (v > raster::kEmpty) {
                        ++k;
                        *worst = std::max(*worst, std::abs(v - (-12 + 0.02 * i + 0.03 * j)));
                    }
                }
            return k / (91.0 * 80);
        };
        double worstDef = 0, worstWide = 0;
        const double def = share(raster::kFillLevels, &worstDef), wide = share(raster::fillLevelsFor(8, 0.5), &worstWide);
        QVERIFY2(def < 0.5 && wide > 0.95 && worstWide < 0.15, qPrintable(QStringLiteral("%1 %2 %3").arg(def).arg(wide).arg(worstWide)));
        // ... and the coverage of an area is counted in the same blocks
        SurfaceTiles st;
        st.configure(0.5, NAN, NAN);
        std::vector<double> pts;
        for (double y = 0.25; y < 64; y += 5)
            for (double x = 0.25; x < 64; x += 0.5) pts.insert(pts.end(), {x, y, -12});
        st.add(pts.data(), int(pts.size() / 3));
        const std::vector<QPointF> sq{{0, 0}, {64, 0}, {64, 64}, {0, 64}};
        const double fine = coveredArea(st, sq) / 4096, blocks = coveredArea(st, sq, 16) / 4096;
        QVERIFY2(fine < 0.15 && blocks > 0.95, qPrintable(QStringLiteral("%1 %2").arg(fine).arg(blocks)));
        QCOMPARE(polygonAreaXY(sq), 4096.0);
    }

    // A wall seen from a sensor in front of it: the TSDF meshes a surface on the wall
    void volumeWall()
    {
        SurfaceVolume v;
        v.configure(0.3, 20, 1024);
        const double o[3] = {0, 0, 0};
        std::vector<double> pts;
        for (double y = -3; y <= 3; y += 0.1)
            for (double z = -3; z <= 3; z += 0.1) pts.insert(pts.end(), {5, y, z});   // a wall 5 m east
        for (int pass = 0; pass < 3; ++pass) QCOMPARE(v.integrate(o, pts.data(), int(pts.size() / 3)), int(pts.size() / 3));
        const auto ch = v.takeChanges();
        std::size_t vertices = 0;
        double worst = 0;
        for (const VolumeMesh &m : ch.meshes) {
            vertices += m.pos.size() / 3;
            for (std::size_t k = 0; k + 2 < m.pos.size(); k += 3)
                if (std::abs(m.pos[k + 1]) < 2.5 && std::abs(m.pos[k + 2]) < 2.5) worst = std::max(worst, std::abs(m.pos[k] - 5.0));
        }
        QVERIFY(vertices > 300);
        QVERIFY2(worst < 0.2, qPrintable(QString::number(worst)));
        QCOMPARE(int(v.takeChanges().meshes.size()), 0);
        // A flat wall: (nearly) no spread of the normals away from its edges
        const auto it = std::max_element(ch.meshes.begin(), ch.meshes.end(), [](const VolumeMesh &a, const VolumeMesh &b) { return a.pos.size() < b.pos.size(); });
        const auto spread = RosSurfaceData::normalSpread(it->pos);
        QCOMPARE(spread.size(), it->pos.size() / 3);
        std::vector<float> sorted = spread;
        std::sort(sorted.begin(), sorted.end());
        QVERIFY2(sorted[sorted.size() / 2] < 0.05f, qPrintable(QString::number(sorted[sorted.size() / 2])));
    }

    // The blocks of the 3D view: a tile goes into its block and into the apron of the block west of it
    void surfaceBlocks()
    {
        RosSurfaceData &d = RosSurfaceData::instance();
        d.clear();
        RosAnchor a;
        a.lat = 45;
        a.lon = 10;
        a.epoch = 1;
        TileUpdate t;
        t.tx = 8;   // the first tile of block 1: also the last column of block 0's apron
        t.ty = 0;
        t.h.assign(900, 2.f);
        t.w.assign(900, 1.f);
        TileUpdate t0 = t;
        t0.tx = 7;
        d.applyTiles(true, a, 0.3, {t0}, {});
        QCOMPARE(int(d.blocks.size()), 1);
        d.applyTiles(false, a, 0.3, {t}, {});
        QCOMPARE(int(d.blocks.size()), 2);
        d.fillDirty();
        const auto &b0 = *d.blocks.at(SurfaceTiles::tileKey(0, 0));
        // block 0: its last tile (7) in columns 210..239, the apron column 240 from tile 8
        QCOMPARE(b0.view[std::size_t(0 * RosSurfaceData::kTex + 240) * 2], 2.f);
        QCOMPARE(b0.view[std::size_t(0 * RosSurfaceData::kTex + 215) * 2], 2.f);
        QCOMPARE(b0.view[std::size_t(0 * RosSurfaceData::kTex + 100) * 2], raster::kEmpty);
        std::vector<std::uint16_t> idx(std::size_t(RosSurfaceData::kBlock * RosSurfaceData::kBlock * 6));
        const int n = raster::triangulate(b0.view.data(), RosSurfaceData::kTex, RosSurfaceData::kTex, 1, idx.data(), 0.3);
        QCOMPARE(n, 30 * 29 * 6);   // 30 x 30 cells of tile 7 and the apron column: 30 x 29 quads
        // A new epoch starts over
        a.epoch = 2;
        d.applyTiles(false, a, 0.3, {}, {});
        QVERIFY(d.blocks.empty());
    }
};

QTEST_GUILESS_MAIN(TstRosSurface)
#include "tst_rossurface.moc"
