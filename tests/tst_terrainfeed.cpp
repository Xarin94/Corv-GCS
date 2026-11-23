// The terrain feed (TerrainFeeder): what a TERRAIN_REQUEST gets back. Block and
// point layout as ArduPilot's AP_Terrain reads it, one block per message, the
// pace, no made-up heights, and the reason given when the vehicle stays pending.
#include "link/MessageBus.h"
#include "vehicle/TerrainFeeder.h"

#include <QSignalSpy>
#include <QTest>
#include <cmath>

using namespace corv;

namespace {
constexpr int32_t kLat = 470000000, kLon = 110000000;

/// A height that says where it was taken: metres north and east of the grid's corner, in grid steps.
double stepsNorth(double lat) { return (lat - kLat * 1e-7) * 1e7 / 89.83204953368922 / 100; }
double stepsEast(double lat, double lon)
{
    const double mid = (kLat * 1e-7 + lat) / 2;
    return (lon - kLon * 1e-7) * 1e7 / 89.83204953368922 * std::cos(mid * M_PI / 180) / 100;
}

mavlink_terrain_request_t request(quint64 mask, int32_t lat = kLat)
{
    mavlink_terrain_request_t r{};
    r.lat = lat;
    r.lon = kLon;
    r.grid_spacing = 100;
    r.mask = mask;
    return r;
}

struct Sink {
    QList<mavlink_terrain_data_t> got;
    TerrainFeeder::SendFn fn()
    {
        return [this](const mavlink_message_t &m) {
            QCOMPARE(m.msgid, uint32_t(MAVLINK_MSG_ID_TERRAIN_DATA));
            mavlink_terrain_data_t d;
            mavlink_msg_terrain_data_decode(&m, &d);
            got << d;
        };
    }
};
} // namespace

class TstTerrainFeed : public QObject
{
    Q_OBJECT
private slots:
    void layout()
    {
        // Bits 0 (corner), 9 (second row of blocks, second column) and 55 (north-east block)
        Sink sink;
        TerrainFeeder f(nullptr, [](double lat, double lon, double &h) {
            h = std::lround(stepsNorth(lat)) * 100 + std::lround(stepsEast(lat, lon));
            return TerrainFeeder::Lookup::Found;
        }, sink.fn());
        f.onRequest(request((quint64(1) << 0) | (quint64(1) << 9) | (quint64(1) << 55)));
        QTRY_COMPARE(sink.got.size(), 3);
        for (const auto &d : sink.got) {
            QCOMPARE(d.lat, kLat);
            QCOMPARE(d.lon, kLon);
            QCOMPARE(d.grid_spacing, uint16_t(100));
            const int north0 = d.gridbit / 8 * 4, east0 = d.gridbit % 8 * 4;
            for (int x = 0; x < 4; ++x)
                for (int y = 0; y < 4; ++y) QCOMPARE(int(d.data[x * 4 + y]), (north0 + x) * 100 + east0 + y);
        }
        QCOMPARE(sink.got[0].gridbit, uint8_t(0));
        QCOMPARE(sink.got[1].gridbit, uint8_t(9));
        QCOMPARE(sink.got[2].gridbit, uint8_t(55));
        QCOMPARE(f.stats().sent, 3);
        QCOMPARE(f.stats().queued, 0);
    }

    void onePerPaceAndNoDuplicates()
    {
        Sink sink;
        TerrainFeeder f(nullptr, [](double, double, double &h) { h = 500; return TerrainFeeder::Lookup::Found; }, sink.fn());
        constexpr int n = 24;
        const quint64 all = (quint64(1) << n) - 1;
        QElapsedTimer t;
        t.start();
        f.onRequest(request(all));
        f.onRequest(request(all));   // repeated while it is being answered
        QTRY_COMPARE_WITH_TIMEOUT(sink.got.size(), n, 10000);
        QVERIFY(t.elapsed() >= (n - 1) * TerrainFeeder::kSendIntervalMs * 8 / 10);
        // Answered a moment ago: the packets may still be on their way
        f.onRequest(request(all));
        QCOMPARE(f.stats().queued, 0);
        QTest::qWait(150);
        QCOMPARE(sink.got.size(), n);
        // Still missing later: they were lost
        QTest::qWait(TerrainFeeder::kResendAfterMs);
        f.onRequest(request(0b11));
        QTRY_COMPARE(sink.got.size(), n + 2);
    }

    void unknownHeightsAreNotSent()
    {
        Sink sink;
        auto what = TerrainFeeder::Lookup::Pending;
        TerrainFeeder f(nullptr, [&](double, double, double &h) { h = 7; return what; }, sink.fn());
        f.onRequest(request(0b1111));
        QTRY_COMPARE(f.stats().queued, 0);
        QCOMPARE(sink.got.size(), 0);
        QCOMPARE(f.stats().missing, 0);   // the tile is on its way, not missing
        what = TerrainFeeder::Lookup::Missing;
        f.onRequest(request(0b1111));
        QTRY_COMPARE(f.stats().missing, 4);
        QCOMPARE(sink.got.size(), 0);
        // The tile arrived: the next request is answered at once
        what = TerrainFeeder::Lookup::Found;
        f.onRequest(request(0b1111));
        QTRY_COMPARE(sink.got.size(), 4);
    }

    void mutedSendsNothing()
    {
        Sink sink;
        TerrainFeeder f(nullptr, [](double, double, double &h) { h = 1; return TerrainFeeder::Lookup::Found; }, sink.fn());
        f.setMuted(true);
        f.onRequest(request(0b11));
        QTest::qWait(150);
        QCOMPARE(sink.got.size(), 0);
        f.setMuted(false);
        f.onRequest(request(0b11));
        QTRY_COMPARE(sink.got.size(), 2);
    }

    void throughTheBus()
    {
        Sink sink;
        MessageBus bus;
        TerrainFeeder f(&bus, [](double, double, double &h) { h = 300; return TerrainFeeder::Lookup::Found; }, sink.fn());
        MavMessage m;
        mavlink_msg_terrain_request_pack(1, 1, &m.msg, kLat, kLon, 100, 0b101);
        m.replay = true;   // a log being replayed is not answered
        bus.publish(m);
        QTest::qWait(150);
        QCOMPARE(sink.got.size(), 0);
        m.replay = false;
        bus.publish(m);
        QTRY_COMPARE(sink.got.size(), 2);
        MavMessage r;
        mavlink_msg_terrain_report_pack(1, 1, &r.msg, kLat, kLon, 100, 300, 50, 56, 224);
        bus.publish(r);
        QTRY_COMPARE(f.stats().pending, 56);
        QCOMPARE(f.stats().loaded, 224);
        f.reset();
        QCOMPARE(f.stats().sent, 0);
        QCOMPARE(f.stats().pending, -1);
    }

    void offsetMatchesArduPilot()
    {
        // 100 m north and east at 47 N: 898.32 e-7 deg and that over cos(lat)
        double lat = 0, lon = 0;
        TerrainFeeder::offset(kLat, kLon, 100, 100, lat, lon);
        QVERIFY(std::abs(lat - 47.000898320495) < 1e-9);
        QVERIFY(std::abs(lon - (11 + 0.000898320495 / std::cos(47.00044916 * M_PI / 180))) < 1e-9);
        // Across the date line
        TerrainFeeder::offset(0, 1799999000, 0, 100, lat, lon);
        QVERIFY(lon < -179.99);
    }
};

QTEST_GUILESS_MAIN(TstTerrainFeed)
#include "tst_terrainfeed.moc"
