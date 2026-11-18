// JoystickMath.h - the mapping of JoystickManager.js: an input (an axis, or an
// analog button) to an RC channel, with inversion, deadzone, expo and the PWM
// range 1000-2000 around 1500; the servo buttons; the persisted configuration
// ('datad-joystick-config', the Electron key). No device, no Qt GUI: tested on
// its own.
#pragma once

#include <QJsonArray>
#include <QJsonObject>
#include <QString>
#include <optional>
#include <vector>

namespace corv::joystick {

constexpr int kMinInputs = 8;      // input rows always shown
constexpr int kChannels = 18;      // RC_CHANNELS_OVERRIDE
constexpr int kRelease = 0;        // 0: the channel goes back to the RC receiver

struct AxisConfig {
    enum class Source { Axis, Button, None };
    Source source = Source::Axis;
    int sourceIndex = 0;
    int channel = 0;          // 1..18, 0 unmapped
    bool inverted = false;
    double deadzone = 0.05;   // 0..0.9
    double expo = 0;          // 0..1
};

struct ServoConfig {
    int button = -1;          // -1 unmapped
    int servo = 9;            // 1..16
    double percent = 100;     // 0..100 % = 1000..2000 us
};

/// Defaults of input `index`: roll, pitch, yaw, throttle on CH1, CH2, CH4, CH3, then unmapped.
AxisConfig defaultAxis(int index);
/// A persisted entry, cleaned as normalizeAxis(): bad values fall back to the defaults.
AxisConfig axisFromJson(const QJsonValue &v, int index);
QJsonObject toJson(const AxisConfig &a);
ServoConfig servoFromJson(const QJsonValue &v, int index);
QJsonObject toJson(const ServoConfig &s);
QString sourceKey(const AxisConfig &a);   // "axis:0", "button:3", "none:0"

double applyDeadzone(double value, double deadzone);
double applyExpo(double value, double expo);
int axisToPwm(double value);
/// The input's value -1..1 from the device state; nothing when the source is unavailable.
/// A button reads -1 released, +1 pressed (analog in between).
std::optional<double> readSource(const AxisConfig &a, const std::vector<double> &axes, const std::vector<double> &buttons);
/// The PWM the channel gets, or nothing when the input is unmapped or unavailable.
std::optional<int> channelPwm(const AxisConfig &a, std::optional<double> raw);
/// Servo button position: 0..100 % = 1000..2000 us.
int servoPwm(const ServoConfig &s);

} // namespace corv::joystick
