// StateMapper: the behaviours the Electron mapper had and the port must keep.
#include "vehicle/FlightModes.h"
#include "vehicle/StateMapper.h"

#include <QTest>
#include <QtEndian>
#include <algorithm>
#include <cstring>
#include <iterator>

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
    void rpmSensors()
    {
        StateMapper mapper;
        VehicleState s;
        mavlink_message_t m;
        mavlink_msg_rpm_pack(1, 1, &m, 1485.5f, -1.f);
        mapper.apply(msgAt(m, 10000), s);
        QCOMPARE(s.sensorRpm(1, 10500).value_or(-9), 1485.5);
        QVERIFY(!s.sensorRpm(2, 10500));     // -1: not there
        QVERIFY(!s.sensorRpm(1, 14000));     // gone quiet
        QVERIFY(!s.sensorRpm(3, 10500));
    }

    void escTelemetry()
    {
        StateMapper mapper;
        VehicleState s;
        mavlink_message_t m;
        const uint8_t temp[4] = {};
        const uint16_t zero[4] = {};
        uint16_t rpm[4] = {4000, 4100, 4200, 0}, count[4] = {10, 20, 30, 0};
        mavlink_msg_esc_telemetry_1_to_4_pack(1, 1, &m, temp, zero, zero, zero, rpm, count);
        mapper.apply(msgAt(m, 10000), s);
        QCOMPARE(s.escRpm(1, 10100).value_or(-1), 4000.0);
        QCOMPARE(s.escRpm(3, 10100).value_or(-1), 4200.0);
        QVERIFY(!s.escRpm(4, 10100));        // nothing ever came from it
        QVERIFY(!s.escRpm(5, 10100));
        // ESC 2 dies: the message goes on with the others, its count stands still
        rpm[0] = 4050;
        count[0] = 11;
        count[2] = 31;
        mavlink_msg_esc_telemetry_1_to_4_pack(1, 1, &m, temp, zero, zero, zero, rpm, count);
        mapper.apply(msgAt(m, 14000), s);
        QCOMPARE(s.escRpm(1, 14100).value_or(-1), 4050.0);
        QVERIFY(!s.escRpm(2, 14100));
        // The next four
        mavlink_msg_esc_telemetry_5_to_8_pack(1, 1, &m, temp, zero, zero, zero, rpm, count);
        mapper.apply(msgAt(m, 14100), s);
        QCOMPARE(s.escRpm(5, 14200).value_or(-1), 4050.0);
        QVERIFY(!s.escRpm(0, 14200));
        QVERIFY(!s.escRpm(17, 14200));
    }

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

    void homeNearKeepsOrigin()
    {
        // ArduPlane moves its home where it stands every few seconds while disarmed
        StateMapper mapper;
        VehicleState s;
        mapper.origin().awaitFix();
        mapper.apply(msgAt(gpi(445832000, 109350630, 70000, 0, 0, 0), 1000), s);
        QCOMPARE(s.sceneOriginEpoch, 1);
        mavlink_message_t h;
        const float q[4] = {1, 0, 0, 0};
        mavlink_msg_home_position_pack(1, 1, &h, 445831949, 109350389, 72210, 0, 0, 0, q, 0, 0, 0, 0);
        mapper.apply(msgAt(h, 1200), s);
        mavlink_msg_home_position_pack(1, 1, &h, 445830927, 109349410, 66880, 0, 0, 0, q, 0, 0, 0, 0);
        mapper.apply(msgAt(h, 6200), s);
        QCOMPARE(s.sceneOriginEpoch, 1);            // no terrain rebuilt for metres
        QCOMPARE(s.sceneOrigin.lat, 44.5832);
        QCOMPARE(*s.homeLat, 44.5830927);           // the home itself is the vehicle's
        // A home kilometres away (another field) moves it
        mavlink_msg_home_position_pack(1, 1, &h, 446000000, 109350000, 50000, 0, 0, 0, q, 0, 0, 0, 0);
        mapper.apply(msgAt(h, 7000), s);
        QCOMPARE(s.sceneOriginEpoch, 2);
        QCOMPARE(s.sceneOrigin.lat, 44.6);
    }

    void secondGpsReceiver()
    {
        // The receiver on the second GPS port: GPS_RAW_INT says no GPS, GPS2_RAW has the fix
        StateMapper mapper;
        VehicleState s;
        mavlink_message_t g1, g2;
        mavlink_msg_gps_raw_int_pack(1, 1, &g1, 0, 0, 0, 0, 0, UINT16_MAX, UINT16_MAX, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        mavlink_msg_gps2_raw_pack(1, 1, &g2, 0, 3, 445832000, 109350630, 70000, 137, 200, 10, 0, 16, 0, 0, 0, 0, 0, 0, 0, 0);
        mapper.apply(msgAt(g1, 10000), s);
        QCOMPARE(s.gpsFix, 0);
        QCOMPARE(s.gpsReceiver, 1);
        mapper.apply(msgAt(g2, 10010), s);
        mapper.apply(msgAt(g1, 10500), s);           // the first one goes on saying nothing
        QCOMPARE(s.gpsFix, 3);
        QCOMPARE(s.gpsNumSat, 16);
        QCOMPARE(s.gpsHdop, 1.37);
        QCOMPARE(s.gpsReceiver, 2);
        QCOMPARE(s.gpsDataTime, std::int64_t(10010));
        // Position from it when there is no GLOBAL_POSITION_INT
        QCOMPARE(s.lat, 44.5832);
        // The second falls silent: after 5 s the first is shown again
        mapper.apply(msgAt(g1, 15100), s);
        QCOMPARE(s.gpsFix, 0);
        QCOMPARE(s.gpsReceiver, 1);
        // Both with a fix: the one with more satellites
        mavlink_msg_gps_raw_int_pack(1, 1, &g1, 0, 3, 445832000, 109350630, 70000, 90, 200, 10, 0, 12, 0, 0, 0, 0, 0, 0);
        mapper.apply(msgAt(g1, 16000), s);
        mapper.apply(msgAt(g2, 16010), s);
        QCOMPARE(s.gpsReceiver, 2);
        QCOMPARE(s.gpsNumSat, 16);
        // A new vehicle forgets both
        mapper.newVehicle();
        mapper.apply(msgAt(g1, 16100), s);
        QCOMPARE(s.gpsReceiver, 1);
        QCOMPARE(s.gpsNumSat, 12);
    }

    void batteryCellsFromVoltage()
    {
        StateMapper mapper;
        VehicleState s;
        mavlink_message_t m;
        auto sys = [&](uint16_t mv, std::int64_t t) {
            mavlink_msg_sys_status_pack(1, 1, &m, 0, 0, 0, 0, mv, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0);
            mapper.apply(msgAt(m, t), s);
        };
        // A charged 2S on the default 3.2-4.2 V per cell: no BATT LOW (the 3S range gave 0 %)
        sys(8342, 1000);
        QCOMPARE(s.batteryCells, 2);
        QCOMPARE(s.batteryCellsSource, 0);
        QCOMPARE(s.batteryRemaining, 97);
        // USB power only: no pack, no percentage
        StateMapper bench;
        VehicleState b;
        mavlink_msg_sys_status_pack(1, 1, &m, 0, 0, 0, 0, 120, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        bench.apply(msgAt(m, 1000), b);
        QCOMPARE(b.batteryCells, 0);
        QCOMPARE(b.batteryRemaining, -1);
        // A full 6S, then sagging in flight: the count stays
        mapper.newVehicle();
        sys(25200, 2000);
        QCOMPARE(s.batteryCells, 6);
        QCOMPARE(s.batteryRemaining, 100);
        sys(21000, 3000);
        QCOMPARE(s.batteryCells, 6);
        QCOMPARE(s.batteryRemaining, 30);
        // A high-voltage LiPo just off the charger is not one cell more
        mapper.newVehicle();
        sys(8700, 4000);
        QCOMPARE(s.batteryCells, 2);
        // A count set by hand wins
        mapper.setBatteryGauge(3.0, 4.2, 3);
        sys(12000, 5000);
        QCOMPARE(s.batteryCells, 3);
        QCOMPARE(s.batteryCellsSource, 2);
        QCOMPARE(s.batteryRemaining, 83);
    }

    void batteryCellsFromVehicle()
    {
        StateMapper mapper;
        VehicleState s;
        mavlink_message_t m;
        // ArduPilot without cell monitoring: the pack in the first slot only, no count
        uint16_t v[10];
        std::fill(std::begin(v), std::end(v), UINT16_MAX);
        const uint16_t ext[4] = {};
        v[0] = 15800;
        mavlink_msg_battery_status_pack(1, 1, &m, 0, 0, 0, INT16_MAX, v, -1, -1, -1, -1, 0, 0, ext, 0, 0);
        mapper.apply(msgAt(m, 1000), s);
        QCOMPARE(s.batteryCells, 0);
        // A smart battery with its cells: that count, even nearly flat (13.2 V would read 3S)
        v[0] = v[1] = v[2] = v[3] = 3300;
        mavlink_msg_battery_status_pack(1, 1, &m, 0, 0, 0, INT16_MAX, v, -1, -1, -1, -1, 0, 0, ext, 0, 0);
        mapper.apply(msgAt(m, 1100), s);
        mavlink_msg_sys_status_pack(1, 1, &m, 0, 0, 0, 0, 13200, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        mapper.apply(msgAt(m, 1200), s);
        QCOMPARE(s.batteryCells, 4);
        QCOMPARE(s.batteryCellsSource, 1);
        QCOMPARE(s.batteryRemaining, 10);
    }

    void firmwareVersion()
    {
        QCOMPARE(modes::firmwareVersion(0x040700ff), QStringLiteral("4.7.0"));
        QCOMPARE(modes::firmwareVersion(0x04080000), QStringLiteral("4.8.0-dev"));
        QCOMPARE(modes::firmwareVersion(0x04070081), QStringLiteral("4.7.0-beta1"));
        QCOMPARE(modes::firmwareVersion(0x040603c0), QStringLiteral("4.6.3-rc"));
        StateMapper mapper;
        VehicleState s;
        mavlink_message_t m;
        const uint8_t custom[8] = {}, uid2[18] = {};
        mavlink_msg_autopilot_version_pack(1, 1, &m, 0, 0x040603ff, 0, 0, 0, custom, custom, custom, 0, 0, 0, uid2);
        mapper.apply(msgAt(m, 1000), s);
        QCOMPARE(s.firmwareVersion, QStringLiteral("4.6.3"));
        // A companion's version is not the autopilot's
        mavlink_msg_autopilot_version_pack(1, 191, &m, 0, 0x010000ff, 0, 0, 0, custom, custom, custom, 0, 0, 0, uid2);
        mapper.apply(msgAt(m, 1100), s);
        QCOMPARE(s.firmwareVersion, QStringLiteral("4.6.3"));
    }

    void preArm()
    {
        using P = VehicleState::PreArm;
        StateMapper mapper;
        VehicleState s;
        QVERIFY(s.preArm() == P::Unknown);
        mavlink_message_t m;
        char text[50] = "PreArm: Compass not calibrated";
        mavlink_msg_statustext_pack(1, 1, &m, MAV_SEVERITY_CRITICAL, text, 0, 0);
        mapper.apply(msgAt(m, 1000), s);
        QVERIFY(s.preArm() == P::Failed);
        QCOMPARE(s.preArmText, QStringLiteral("PreArm: Compass not calibrated"));
        char good[50] = "PreArm: good";
        mavlink_msg_statustext_pack(1, 1, &m, MAV_SEVERITY_INFO, good, 0, 0);
        mapper.apply(msgAt(m, 1050), s);
        QVERIFY(s.preArm() == P::Passed);
        // The vehicle's own bit says more than its last message
        constexpr uint32_t bit = 0x10000000;
        mavlink_msg_sys_status_pack(1, 1, &m, bit | 1, bit | 1, bit, 0, 12600, -1, 90, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        mapper.apply(msgAt(m, 1100), s);
        QVERIFY(s.preArm() == P::Passed);
        mavlink_msg_sys_status_pack(1, 1, &m, bit | 1, bit | 1, 1, 0, 12600, -1, 90, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        mapper.apply(msgAt(m, 1200), s);
        QVERIFY(s.preArm() == P::Failed);
        s.armed = true;
        QVERIFY(s.preArm() == P::Passed);
    }

    void subOnALake()
    {
        StateMapper mapper;
        VehicleState s;
        bool loaded = false;
        mapper.setHooks({.surface = [&](double, double) { return loaded ? std::optional<double>(65) : std::nullopt; }});
        mavlink_message_t m;
        mavlink_msg_heartbeat_pack(1, 1, &m, MAV_TYPE_SUBMARINE, MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, MAV_STATE_ACTIVE);
        mapper.apply(msgAt(m, 1000), s);
        auto position = [&](std::int64_t t, int32_t relMm) {
            mavlink_msg_global_position_int_pack(1, 1, &m, 1000, 456420000, 106700000, relMm, relMm, 0, 0, 0, 9000);
            mapper.apply(msgAt(m, t), s);
            mapper.finish(s, t);
        };
        // The elevation tile is not in yet: as the vehicle says it
        position(2000, -8000);
        QCOMPARE(s.rawAlt, -8.0);
        QVERIFY(!s.subSurfaceAlt);
        // In: the depth is below the lake, 65 m above the sea
        loaded = true;
        position(2400, -8000);
        QCOMPARE(*s.subSurfaceAlt, 65.0);
        position(2500, -8000);
        QCOMPARE(s.rawAlt, 57.0);
        // Another vehicle has no such surface
        mavlink_msg_heartbeat_pack(1, 1, &m, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, MAV_STATE_ACTIVE);
        mapper.apply(msgAt(m, 3000), s);
        mapper.finish(s, 3000);
        QVERIFY(!s.subSurfaceAlt);
    }
};

QTEST_GUILESS_MAIN(TestStateMapper)
#include "tst_statemapper.moc"
