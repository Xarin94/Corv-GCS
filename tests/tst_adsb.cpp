// The OpenSky reply as AdsbClient reads it (fetchADSBData of ADSBManager.js):
//   - the columns of a state vector: icao24, trimmed callsign, lat / lon, the
//     barometric altitude or else the geometric one, speed, track, climb, ground
//   - entries with no position are dropped
//   - time_position is kept only when plausible (not in the future, not older
//     than the 60 s purge age), otherwise the reception time
//   - malformed JSON is an error, an empty sky is not
#include "services/AdsbClient.h"
#include "vehicle/VehicleState.h"

#include <QTest>

using namespace corv;

class TstAdsb : public QObject
{
    Q_OBJECT
private slots:
    void columns()
    {
        const std::int64_t received = 1'700'000'030'000;
        const QByteArray json = R"({"time":1700000030,"states":[
            ["3c6585","DLH505  ","Germany",1700000025,1700000029,11.35,47.30,6287.1,false,210.5,93.0,-4.2,null,6350.0,"1000",false,0],
            ["4b1805","SWR188  ","Switzerland",1700000040,1700000029,11.10,47.10,null,false,230.0,270.0,0.0,null,8252.0,"2000",false,0],
            ["abcdef","","Austria",1699999000,1700000029,null,47.00,1000,false,0,0,0,null,1000,"",false,0],
            ["4ca123",null,"Ireland",null,1700000029,11.50,47.50,null,true,0,0,0,null,null,"",false,0]
        ]})";
        QString error;
        const QList<TrafficEntry> t = AdsbClient::parseStates(json, received, &error);
        QVERIFY(error.isEmpty());
        QCOMPARE(t.size(), 3);   // the one with no longitude is dropped

        QCOMPARE(t[0].icao24, QStringLiteral("3c6585"));
        QCOMPARE(t[0].callsign, QStringLiteral("DLH505"));
        QCOMPARE(t[0].lat, 47.30);
        QCOMPARE(t[0].lon, 11.35);
        QCOMPARE(t[0].alt, 6287.1);       // barometric
        QCOMPARE(t[0].velocity, 210.5);
        QCOMPARE(t[0].heading, 93.0);
        QCOMPARE(t[0].vertRate, -4.2);
        QCOMPARE(t[0].onGround, false);
        QCOMPARE(t[0].ts, received);
        QCOMPARE(t[0].posTs, std::int64_t(1'700'000'025'000));   // 5 s old: kept

        QCOMPARE(t[1].alt, 8252.0);       // no barometric altitude: geometric
        QCOMPARE(t[1].posTs, received);   // in the future: reception time

        QCOMPARE(t[2].callsign, QString());
        QCOMPARE(t[2].alt, 0.0);          // neither altitude
        QCOMPARE(t[2].onGround, true);
        QCOMPARE(t[2].posTs, received);   // no time_position
    }

    void stalePositionTime()
    {
        const std::int64_t received = 1'700'000'100'000;
        const QByteArray json = R"({"states":[["a","X","",1700000000,0,11,47,100,false,0,0,0,null,100,"",false,0]]})";
        const auto t = AdsbClient::parseStates(json, received);
        QCOMPARE(t.size(), 1);
        QCOMPARE(t[0].posTs, received);   // 100 s old, past the purge age
    }

    void emptyAndInvalid()
    {
        QString error;
        QVERIFY(AdsbClient::parseStates(R"({"time":1,"states":null})", 0, &error).isEmpty());
        QVERIFY(error.isEmpty());
        QVERIFY(AdsbClient::parseStates("<html>rate limited</html>", 0, &error).isEmpty());
        QCOMPARE(error, QStringLiteral("Invalid JSON from OpenSky"));
    }
};

QTEST_GUILESS_MAIN(TstAdsb)
#include "tst_adsb.moc"
