// Port of scripts/test-ekf-monitor.js: the EKF STATUS data fed what an
// ArduPilot sends (real MAVLink frames here): units, the EKF − DCM comparison
// at the same instant and the 20 s window.
#include "vehicle/EkfMonitor.h"

#include <QTest>
#include <QtEndian>
#include <cmath>

using namespace corv;

namespace {
constexpr double RAD = M_PI / 180;
bool near(std::optional<double> a, double b, double tol) { return a && std::abs(*a - b) <= tol; }

mavlink_message_t ekf(int flags, float vel, float posH, float posV, float mag, float terr = 0, float tas = 0)
{
    mavlink_message_t m;
    mavlink_msg_ekf_status_report_pack(1, 1, &m, std::uint16_t(flags), vel, posH, posV, mag, terr, tas);
    return m;
}
mavlink_message_t gps(int fix, int sats, int eph, unsigned hAcc, unsigned vAcc, unsigned velAcc)
{
    mavlink_message_t m;
    mavlink_msg_gps_raw_int_pack(1, 1, &m, 0, std::uint8_t(fix), 0, 0, 0, std::uint16_t(eph), 0, 0, 0, std::uint8_t(sats), 0, hAcc, vAcc, velAcc, 0, 0);
    return m;
}
mavlink_message_t att(double r, double p, double y)
{
    mavlink_message_t m;
    mavlink_msg_attitude_pack(1, 1, &m, 0, float(r), float(p), float(y), 0, 0, 0);
    return m;
}
mavlink_message_t ahrs2(double r, double p, double y)
{
    mavlink_message_t m;
    mavlink_msg_ahrs2_pack(1, 1, &m, float(r), float(p), float(y), 0, 0, 0);
    return m;
}

} // namespace

class TstEkfMonitor : public QObject
{
    Q_OBJECT
private slots:
    void ardupilot()
    {
        EkfMonitor m;
        QVERIFY(m.source(0).isEmpty() && !m.live(0));                         // no data: no source
        m.ingestMavlink(ekf(0x37f, 0.12f, 0.3f, 0.05f, 0.2f), 1000);
        QCOMPARE(m.source(1000), QStringLiteral("ardupilot"));
        QVERIFY(near(m.get(QStringLiteral("apPosH")).last(), 0.3, 1e-6) && m.ap().flags == 0x37f);
        QCOMPARE(m.get(QStringLiteral("apTas")).size(), 0);                    // no airspeed fusion: not drawn
        m.ingestMavlink(ekf(0x37f, 0.1f, 0.3f, 0.05f, 0.2f, 0, 0.4f), 1200);
        QVERIFY(near(m.get(QStringLiteral("apTas")).last(), 0.4, 1e-6));       // airspeed fusion: drawn

        m.ingestMavlink(gps(3, 14, 80, 1500, 2200, 300), 1300);
        QVERIFY(near(m.get(QStringLiteral("apHAcc")).last(), 1.5, 1e-9) && near(m.get(QStringLiteral("apVAcc")).last(), 2.2, 1e-9)
                && near(m.get(QStringLiteral("apVelAcc")).last(), 0.3, 1e-9) && near(m.ap().hdop, 0.8, 1e-9));
        m.ingestMavlink(gps(3, 14, 80, 0, 0, 0), 1400);
        QCOMPARE(m.get(QStringLiteral("apHAcc")).size(), 1);                   // accuracy 0 = not reported
        m.ingestMavlink(gps(1, 0, 9999, 300, 300, 0), 1450);
        QCOMPARE(m.get(QStringLiteral("apHAcc")).size(), 1);                   // no fix: not plotted

        // EKF ATTITUDE at 0 and 40 ms, rolling 100°/s; DCM at 20 ms one degree behind
        m.ingestMavlink(att(0, 0, 359 * RAD), 2000);
        m.ingestMavlink(ahrs2(1 * RAD, 0, 1 * RAD), 2020);
        QCOMPARE(m.get(QStringLiteral("apDRoll")).size(), 0);                  // waits for the next ATTITUDE
        m.ingestMavlink(att(4 * RAD, 0, 359 * RAD), 2040);
        QVERIFY2(near(m.get(QStringLiteral("apDRoll")).last(), 1, 1e-4), "EKF − DCM interpolated at the AHRS2 instant");
        QVERIFY2(near(m.get(QStringLiteral("apDYaw")).last(), -2, 1e-4), "EKF − DCM yaw across north");
        m.ingestMavlink(ahrs2(0, 0, 0), 2100);
        m.ingestMavlink(ahrs2(0, 0, 0), 2500);
        QCOMPARE(m.get(QStringLiteral("apDRoll")).size(), 2);                  // ATTITUDE gone: the latest one

        mavlink_message_t v;
        mavlink_msg_vibration_pack(1, 1, &v, 0, 12, 15, 31, 0, 2, 0);
        m.ingestMavlink(v, 2600);
        QVERIFY(m.ap().vibe && (*m.ap().vibe)[2] == 31 && m.ap().clip == 2);

        QVERIFY(m.source(6000) == QLatin1String("ardupilot") && !m.live(6000));   // silent 3 s
        m.ingestMavlink(ekf(0x37f, 0.1f, 0.3f, 0.05f, 0.2f), 1000 + kEkfWindowMs + 2500);
        QVERIFY(m.get(QStringLiteral("apPosH")).t(0) > 1000);                 // 20 s window
    }
};

QTEST_GUILESS_MAIN(TstEkfMonitor)
#include "tst_ekfmonitor.moc"
