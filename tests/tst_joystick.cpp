// The RC mapping of the joystick (JoystickMath, port of JoystickManager.js):
// defaults, cleaning of a saved configuration, deadzone, expo, PWM, sources.
#include "input/JoystickMath.h"

#include <QtTest>
#include <cmath>

using namespace corv::joystick;

class TstJoystick : public QObject
{
    Q_OBJECT

private slots:
    void defaults()
    {
        // Roll, pitch, yaw, throttle on CH1, CH2, CH4, CH3; expo on the sticks only
        const int ch[] = {1, 2, 4, 3, 0, 0, 0, 0};
        for (int i = 0; i < 8; ++i) {
            const AxisConfig a = defaultAxis(i);
            QCOMPARE(a.channel, ch[i]);
            QCOMPARE(a.sourceIndex, i);
            QCOMPARE(a.source, AxisConfig::Source::Axis);
            QCOMPARE(a.deadzone, 0.05);
        }
        QCOMPARE(defaultAxis(0).expo, 0.3);
        QCOMPARE(defaultAxis(3).expo, 0.0);
    }

    void cleaning()
    {
        // A hand-edited configuration: out-of-range values fall back, never a NaN PWM
        const QJsonObject bad{{"sourceType", "weird"}, {"sourceIndex", 300}, {"channel", 25}, {"deadzone", 1.5}, {"expo", -2}, {"inverted", true}};
        const AxisConfig a = axisFromJson(bad, 5);
        QCOMPARE(a.source, AxisConfig::Source::Axis);
        QCOMPARE(a.sourceIndex, 5);
        QCOMPARE(a.channel, 0);
        QCOMPARE(a.deadzone, 0.9);
        QCOMPARE(a.expo, 0.0);
        QVERIFY(a.inverted);
        const AxisConfig b = axisFromJson(QJsonObject{{"sourceType", "button"}, {"sourceIndex", 7}, {"channel", 6}}, 0);
        QCOMPARE(b.source, AxisConfig::Source::Button);
        QCOMPARE(sourceKey(b), QStringLiteral("button:7"));
        // Round trip
        const AxisConfig c = axisFromJson(toJson(b), 0);
        QCOMPARE(c.channel, 6);
        QCOMPARE(c.sourceIndex, 7);
        const ServoConfig s = servoFromJson(QJsonObject{{"button", 2}, {"servo", 40}, {"percent", 150}}, 1);
        QCOMPARE(s.button, 2);
        QCOMPARE(s.servo, 10);       // 9 + index when out of 1..16
        QCOMPARE(s.percent, 100.0);
        QCOMPARE(servoPwm(s), 2000);
    }

    void shaping()
    {
        QCOMPARE(applyDeadzone(0.04, 0.05), 0.0);
        QCOMPARE(applyDeadzone(-0.04, 0.05), 0.0);
        QVERIFY(std::abs(applyDeadzone(0.525, 0.05) - 0.5) < 1e-12);   // the rest of the range rescaled
        QCOMPARE(applyDeadzone(1.0, 0.05), 1.0);
        QCOMPARE(applyExpo(0.5, 0), 0.5);
        QVERIFY(std::abs(applyExpo(0.5, 0.3) - (0.3 * 0.125 + 0.7 * 0.5)) < 1e-12);
        QCOMPARE(applyExpo(-1.0, 0.3), -1.0);
        QCOMPARE(axisToPwm(0), 1500);
        QCOMPARE(axisToPwm(1), 2000);
        QCOMPARE(axisToPwm(-1), 1000);
        QCOMPARE(axisToPwm(3), 2000);
    }

    void sources()
    {
        const std::vector<double> axes{0.5, -1, 0.02};
        const std::vector<double> buttons{0, 1, 0.5};
        AxisConfig a = defaultAxis(0);          // axis 0 → CH1, expo 0.3
        QCOMPARE(readSource(a, axes, buttons).value(), 0.5);
        a.expo = 0;
        a.deadzone = 0;
        QCOMPARE(channelPwm(a, readSource(a, axes, buttons)).value(), 1750);
        a.inverted = true;
        QCOMPARE(channelPwm(a, readSource(a, axes, buttons)).value(), 1250);
        // A button: released 1000, pressed 2000, analog in between
        AxisConfig b;
        b.source = AxisConfig::Source::Button;
        b.channel = 7;
        b.deadzone = 0;
        b.sourceIndex = 0;
        QCOMPARE(channelPwm(b, readSource(b, axes, buttons)).value(), 1000);
        b.sourceIndex = 1;
        QCOMPARE(channelPwm(b, readSource(b, axes, buttons)).value(), 2000);
        b.sourceIndex = 2;
        QCOMPARE(channelPwm(b, readSource(b, axes, buttons)).value(), 1500);
        // Unavailable or unmapped: the channel is released (no value)
        b.sourceIndex = 9;
        QVERIFY(!readSource(b, axes, buttons));
        QVERIFY(!channelPwm(b, readSource(b, axes, buttons)));
        AxisConfig none = defaultAxis(2);
        none.source = AxisConfig::Source::None;
        QVERIFY(!readSource(none, axes, buttons));
        AxisConfig unmapped = defaultAxis(6);   // channel 0
        QVERIFY(!channelPwm(unmapped, 0.3));
    }
};

QTEST_GUILESS_MAIN(TstJoystick)
#include "tst_joystick.moc"
