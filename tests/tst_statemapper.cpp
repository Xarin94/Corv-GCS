// StateMapper: the behaviours the Electron mapper had and the port must keep.
#include "vehicle/FlightModes.h"
#include "vehicle/StateMapper.h"

#include <QTest>
#include <QtEndian>
#include <cstring>

using namespace corv;

namespace {
MavMessage msgAt(const mavlink_message_t &m, std::int64_t t)
{
    MavMessage mm;
    mm.msg = m;
    mm.arrivalMs = t;
    return mm;
}

mavlink_message_t gpi(int32_t lat, int32_t lon, int32_t altMm, int16_t vx, int16_t vy, int16_t vz, uint32_t boot = 1000)
{
    mavlink_message_t m;
    mavlink_msg_global_position_int_pack(1, 1, &m, boot, lat, lon, altMm, 5000, vx, vy, vz, 9000);
    return m;
}

} // namespace

class TestStateMapper : public QObject
{
    Q_OBJECT
private slots:
    void heartbeatModes()
    {
        StateMapper mapper;
        VehicleState s;
        mavlink_message_t m;
        mavlink_msg_heartbeat_pack(1, 1, &m, MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_ARDUPILOTMEGA, 209, 10, MAV_STATE_ACTIVE);
        mapper.apply(msgAt(m, 1000), s);
        QVERIFY(s.armed);
        QCOMPARE(s.flightMode, QStringLiteral("AUTO"));
        QCOMPARE(s.heartbeatCount, 1);
        // Another component's heartbeat does not count
        mavlink_msg_heartbeat_pack(1, 68, &m, MAV_TYPE_GCS, 0, 0, 0, 0);
        mapper.apply(msgAt(m, 1100), s);
        QCOMPARE(s.heartbeatCount, 1);
        // The link protocol wins: MSP uses the INAV table
        QCOMPARE(modes::name(8, 1, QStringLiteral("msp-serial")), QStringLiteral("WAYPOINT"));
        QCOMPARE(modes::name(8, 1, QStringLiteral("mavlink-udp")), QStringLiteral("AUTOTUNE"));
        QCOMPARE(modes::number(QStringLiteral("GUIDED"), 2, QStringLiteral("mavlink-udp")), 4);
    }

    void positionPriorityChain()
    {
        StateMapper mapper;
        VehicleState s;
        mapper.apply(msgAt(gpi(451234567, 101234567, 300000, 300, 400, -100), 1000), s);
        QCOMPARE(s.lat, 45.1234567);
        QCOMPARE(s.gs, 5.0);   // EKF horizontal velocity
        QCOMPARE(s.vs, 1.0);
        // VFR_HUD ground speed ignored while GLOBAL_POSITION_INT is fresh
        mavlink_message_t v;
        mavlink_msg_vfr_hud_pack(1, 1, &v, 22.f, 9.f, 90, 50, 300.f, 3.f);
        mapper.apply(msgAt(v, 1500), s);
        QCOMPARE(s.as, 22.0);
        QCOMPARE(s.gs, 5.0);
        // ... and taken once it is older than 2 s
        mapper.apply(msgAt(v, 3500), s);
        QCOMPARE(s.gs, 9.0);
        QCOMPARE(s.vs, 3.0);
        // GPS_RAW_INT position only when GLOBAL_POSITION_INT is stale
        mavlink_message_t g;
        mavlink_msg_gps_raw_int_pack(1, 1, &g, 0, 3, 460000000, 110000000, 500000, 80, 120, 1200, 9000, 12, 0, 0, 0, 0, 0, 0);
        mapper.apply(msgAt(g, 2000), s);
        QCOMPARE(s.lat, 45.1234567);
        QCOMPARE(s.gpsNumSat, 12);
        mapper.apply(msgAt(g, 5000), s);
        QCOMPARE(s.lat, 46.0);
        QCOMPARE(s.gs, 12.0);
    }

    void bootNullsRejected()
    {
        StateMapper mapper;
        VehicleState s;
        const double lat0 = s.lat;
        mapper.apply(msgAt(gpi(0, 101234567, 0, 0, 0, 0), 1000), s);
        QCOMPARE(s.lat, lat0);
        // A real fix on the Greenwich meridian is not a null
        mapper.apply(msgAt(gpi(515000000, 1, 0, 0, 0, 0), 1100), s);
        QCOMPARE(s.lat, 51.5);
    }

    void firstFixMovesOrigin()
    {
        StateMapper mapper;
        VehicleState s;
        mapper.origin().awaitFix();
        mapper.apply(msgAt(gpi(-338000000, 1512000000, 10000, 0, 0, 0), 1000), s);
        QCOMPARE(s.sceneOrigin.lat, -33.8);
        QCOMPARE(s.sceneOriginEpoch, 1);
        // Only the first fix of the stream
        mapper.apply(msgAt(gpi(-339000000, 1512000000, 10000, 0, 0, 0), 1100), s);
        QCOMPARE(s.sceneOrigin.lat, -33.8);
        // HOME_POSITION moves it again
        mavlink_message_t h;
        const float q[4] = {1, 0, 0, 0};
        mavlink_msg_home_position_pack(1, 1, &h, -340000000, 1510000000, 5000, 0, 0, 0, q, 0, 0, 0, 0);
        mapper.apply(msgAt(h, 1200), s);
        QCOMPARE(s.sceneOrigin.lat, -34.0);
        QVERIFY(s.homeLat.has_value());
        QCOMPARE(*s.homeAlt, 5.0);
    }
};

QTEST_GUILESS_MAIN(TestStateMapper)
#include "tst_statemapper.moc"
