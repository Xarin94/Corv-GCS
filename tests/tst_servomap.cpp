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
