// ServoMap: SERVOn_FUNCTION read into motors, throttles and flaps, and the flap
// deployment read back from the PWM the way ArduPilot writes it
// (SRV_Channel::pwm_from_range / pwm_from_angle, Plane::flaperon_update).
#include "vehicle/ServoMap.h"

#include <QHash>
#include <QTest>
#include <cmath>

using namespace corv;

namespace {

using Params = QHash<QString, double>;

ServoMap::Lookup lookupOf(const Params &p)
{
    return [p](const QString &n) -> std::optional<double> {
        const auto it = p.find(n);
        return it == p.end() ? std::nullopt : std::optional<double>(*it);
    };
}

/// SERVO1..16_FUNCTION all 0 (disabled), then the given ones.
Params functions(std::initializer_list<std::pair<int, int>> set)
{
    Params p;
    for (int ch = 1; ch <= 16; ++ch) p[QStringLiteral("SERVO%1_FUNCTION").arg(ch)] = 0;
    for (const auto &[ch, fn] : set) p[QStringLiteral("SERVO%1_FUNCTION").arg(ch)] = fn;
    return p;
}

// ArduPilot's own PWM for a range output (0..100) and an angle output (±4500)
std::uint16_t fromRange(double v, const OutputScale &s)
{
    v = std::clamp(v, 0.0, 100.0);
    if (s.reversed) v = 100 - v;
    return std::uint16_t(s.min + std::uint16_t(v * (s.max - s.min) / 100));
}
std::uint16_t fromAngle(double v, const OutputScale &s)
{
    if (s.reversed) v = -v;
    v = std::clamp(v, -4500.0, 4500.0);
    if (v > 0) return std::uint16_t(s.trim + std::uint16_t(v * (s.max - s.trim) / 4500));
    return std::uint16_t(s.trim - std::uint16_t(-v * (s.trim - s.min) / 4500));
}

} // namespace

class TestServoMap : public QObject
{
    Q_OBJECT
private slots:
    void motorNumbers()
    {
        QCOMPARE(servo::motorNumber(33), 1);
        QCOMPARE(servo::motorNumber(40), 8);
        QCOMPARE(servo::motorNumber(82), 9);
        QCOMPARE(servo::motorNumber(85), 12);
        QCOMPARE(servo::motorNumber(160), 13);
        QCOMPARE(servo::motorNumber(179), 32);
        QCOMPARE(servo::motorNumber(70), 0);
        QCOMPARE(servo::motorNumber(41), 0);
    }

    void partialUntilEveryFunction()
    {
        Params p = functions({{1, 33}});
        p.remove(QStringLiteral("SERVO9_FUNCTION"));
        const ServoMap m = ServoMap::read(lookupOf(p));
        QVERIFY(!m.complete());
        QCOMPARE(m.known(), 15);
        QCOMPARE(m.function(9), -1);
        QCOMPARE(m.motors().at(1), 1);
    }

    void quadplane()
    {
        // ArduPlane quadplane.parm: aileron, elevator, throttle, rudder, Motor1..4 on 5..8
        const ServoMap m = ServoMap::read(lookupOf(functions({{1, 4}, {2, 19}, {3, 70}, {4, 21}, {5, 33}, {6, 34}, {7, 35}, {8, 36}})));
        QVERIFY(m.complete());
        QCOMPARE(int(m.motors().size()), 4);
        QCOMPARE(m.motors().at(1), 5);
        QCOMPARE(m.motors().at(4), 8);
        const auto t = m.throttles();
        QCOMPARE(int(t.size()), 1);
        QCOMPARE(t[0].output, 3);
        QCOMPARE(m.flaps().kind, ServoMap::Flaps::None);
        QVERIFY(m.flapParams().isEmpty());
    }

    void tricopterAndTwin()
    {
        // Tricopter: motors 1, 2, 4 and the tail servo, which ArduPilot maps as Motor7
        const ServoMap tri = ServoMap::read(lookupOf(functions({{1, 33}, {2, 34}, {4, 36}, {7, 39}})));
        QCOMPARE(int(tri.motors().size()), 4);
        QCOMPARE(tri.motors().at(7), 7);
        QCOMPARE(tri.motors().count(3), std::size_t(0));
        QCOMPARE(tri.motors().at(4), 4);

        // Twin: right engine first on the outputs, drawn left to right
        const ServoMap twin = ServoMap::read(lookupOf(functions({{3, 74}, {4, 73}, {9, 74}})));
        const auto t = twin.throttles();
        QCOMPARE(int(t.size()), 2);
        QCOMPARE(t[0].output, 4);
        QCOMPARE(t[0].side, -1);
        QCOMPARE(t[1].output, 3);
        QCOMPARE(t[1].side, 1);
    }

    void flapPriority()
    {
        QCOMPARE(ServoMap::read(lookupOf(functions({{5, 2}}))).flaps().kind, ServoMap::Flaps::Range);
        // FlapAuto (the effective setting) before flaperons before manual Flap
        ServoMap::Flaps f = ServoMap::read(lookupOf(functions({{5, 2}, {6, 24}, {7, 25}, {8, 3}}))).flaps();
        QCOMPARE(f.kind, ServoMap::Flaps::Range);
        QCOMPARE(f.output, 8);
        f = ServoMap::read(lookupOf(functions({{5, 2}, {6, 24}, {7, 25}}))).flaps();
        QCOMPARE(f.kind, ServoMap::Flaps::Flaperons);
        QCOMPARE(f.left, 6);
        QCOMPARE(f.right, 7);
        const QStringList names = ServoMap::read(lookupOf(functions({{6, 24}, {7, 25}}))).flapParams();
        QVERIFY(names.contains(QStringLiteral("SERVO6_TRIM")));
        QVERIFY(names.contains(QStringLiteral("SERVO7_REVERSED")));
        QCOMPARE(names.size(), 8);
    }

    void rangeFlap_data()
    {
        QTest::addColumn<double>("pct");
        QTest::addColumn<bool>("reversed");
        for (double v : {0.0, 25.0, 50.0, 100.0})
            for (bool rev : {false, true}) QTest::addRow("%g%s", v, rev ? " rev" : "") << v << rev;
    }
    void rangeFlap()
    {
        QFETCH(double, pct);
        QFETCH(bool, reversed);
        Params p = functions({{5, 3}});
        p[QStringLiteral("SERVO5_MIN")] = 1000;
        p[QStringLiteral("SERVO5_MAX")] = 2000;
        p[QStringLiteral("SERVO5_REVERSED")] = reversed ? 1 : 0;
        const auto look = lookupOf(p);
        const ServoMap m = ServoMap::read(look);
        std::array<std::uint16_t, 16> pwm{};
        pwm[4] = fromRange(pct, ServoMap::scale(look, 5));
        const auto got = ServoMap::flapPercent(m.flaps(), pwm, [&](int ch) { return ServoMap::scale(look, ch); });
        QVERIFY(got);
        QVERIFY2(std::abs(*got - pct) < 0.2, qPrintable(QString::number(*got)));
    }

    void flaperons_data()
    {
        QTest::addColumn<double>("flap");
        QTest::addColumn<double>("aileron");
        QTest::addColumn<bool>("rightReversed");
        QTest::addRow("retracted") << 0.0 << 0.0 << false;
        QTest::addRow("half, rolling") << 50.0 << 1200.0 << false;
        QTest::addRow("full") << 100.0 << 0.0 << false;
        QTest::addRow("half, right servo reversed") << 50.0 << -800.0 << true;
        QTest::addRow("reflex") << -20.0 << 0.0 << false;
    }
    void flaperons()
    {
        QFETCH(double, flap);
        QFETCH(double, aileron);
        QFETCH(bool, rightReversed);
        Params p = functions({{1, 24}, {2, 25}});
        p[QStringLiteral("SERVO1_MIN")] = 1000;
        p[QStringLiteral("SERVO1_TRIM")] = 1520;
        p[QStringLiteral("SERVO1_MAX")] = 2000;
        p[QStringLiteral("SERVO2_REVERSED")] = rightReversed ? 1 : 0;
        const auto look = lookupOf(p);
        const ServoMap m = ServoMap::read(look);
        std::array<std::uint16_t, 16> pwm{};
        pwm[0] = fromAngle(std::clamp(aileron + flap * 45, -4500.0, 4500.0), ServoMap::scale(look, 1));
        pwm[1] = fromAngle(std::clamp(aileron - flap * 45, -4500.0, 4500.0), ServoMap::scale(look, 2));
        const auto got = ServoMap::flapPercent(m.flaps(), pwm, [&](int ch) { return ServoMap::scale(look, ch); });
        QVERIFY(got);
        QVERIFY2(std::abs(*got - flap) < 0.5, qPrintable(QString::number(*got)));
    }

    void helicopterOutputs()
    {
        // The defaults until the functions are read
        const ServoMap::Heli d = ServoMap::read(lookupOf({})).heli();
        QCOMPARE(d.swash, (std::vector<int>{1, 2, 3}));
        QCOMPARE(d.tail, 4);
        QCOMPARE(d.rsc, 8);
        // Moved about, with a tail motor
        const ServoMap::Heli h = ServoMap::read(lookupOf(functions({{5, 33}, {6, 34}, {7, 35}, {9, 31}, {10, 32}}))).heli();
        QCOMPARE(h.swash, (std::vector<int>{5, 6, 7}));
        QCOMPARE(h.tail, 10);
        QVERIFY(h.tailMotor);
        QCOMPARE(h.rsc, 9);
        // Four swashplate servos (H4)
        QCOMPARE(ServoMap::read(lookupOf(functions({{1, 33}, {2, 34}, {3, 35}, {4, 36}, {5, 37}, {8, 31}}))).heli().swash, (std::vector<int>{1, 2, 3, 5}));
    }

    void collective_data()
    {
        QTest::addColumn<double>("collective");
        QTest::addColumn<double>("roll");
        QTest::addColumn<double>("pitch");
        for (double c : {1300.0, 1500.0, 1680.0})
            for (double r : {-80.0, 0.0, 60.0})
                for (double p : {-50.0, 70.0}) QTest::addRow("%g %g %g", c, r, p) << c << r << p;
    }

    /// Whatever the cyclic, the collective comes out of the three servos, one of them mounted the other way round.
    void collective()
    {
        QFETCH(double, collective);
        QFETCH(double, roll);
        QFETCH(double, pitch);
        ServoMap::Heli h{{1, 2, 3}, 4, false, 8};
        const double az[3] = {-60, 60, 180};   // H3-120
        const OutputScale scale[3] = {{1000, 1500, 2000, false}, {1000, 1520, 2000, true}, {1000, 1490, 2000, false}};
        std::array<std::uint16_t, 16> pwm{};
        for (int i = 0; i < 3; ++i) {
            const double a = az[i] * M_PI / 180;
            const double v = collective + roll * std::cos(a + M_PI / 2) + pitch * std::cos(a);
            pwm[std::size_t(i)] = std::uint16_t(std::lround(scale[i].trim + (v - 1500) * (scale[i].reversed ? -1 : 1)));
        }
        const auto got = ServoMap::collectivePwm(h, 3, pwm, [&](int ch) { return scale[ch - 1]; });
        QVERIFY(got);
        QVERIFY2(std::abs(*got - collective) < 1, qPrintable(QString::number(*got)));
        // H1: the collective has a servo of its own
        QVERIFY(std::abs(*ServoMap::collectivePwm(h, 1, pwm, [&](int ch) { return scale[ch - 1]; }) - (1500 + (pwm[2] - 1490))) < 1e-9);
        pwm[1] = 0;
        QVERIFY(!ServoMap::collectivePwm(h, 3, pwm, [&](int ch) { return scale[ch - 1]; }));
    }

    void noPwmNoValue()
    {
        const auto look = lookupOf(functions({{5, 2}}));
        const ServoMap m = ServoMap::read(look);
        const std::array<std::uint16_t, 16> pwm{};
        QVERIFY(!ServoMap::flapPercent(m.flaps(), pwm, [&](int ch) { return ServoMap::scale(look, ch); }));
        QVERIFY(!ServoMap::flapPercent(ServoMap::Flaps{}, pwm, [&](int ch) { return ServoMap::scale(look, ch); }));
    }
};

QTEST_GUILESS_MAIN(TestServoMap)
#include "tst_servomap.moc"
