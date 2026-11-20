#include "input/JoystickMath.h"

#include <algorithm>
#include <cmath>

using namespace Qt::StringLiterals;

namespace corv::joystick {

namespace {
bool validIndex(const QJsonValue &v) { return v.isDouble() && v.toDouble() == std::floor(v.toDouble()) && v.toDouble() >= 0 && v.toDouble() < 256; }
double clampOr(const QJsonValue &v, double lo, double hi, double fallback)
{
    return v.isDouble() && std::isfinite(v.toDouble()) ? std::clamp(v.toDouble(), lo, hi) : fallback;
}
} // namespace

AxisConfig defaultAxis(int index)
{
    AxisConfig a;
    a.sourceIndex = index;
    static const int channels[] = {1, 2, 4, 3};
    static const double expos[] = {0.3, 0.3, 0.3, 0.0};
    if (index >= 0 && index < 4) {
        a.channel = channels[index];
        a.expo = expos[index];
    }
    return a;
}

AxisConfig axisFromJson(const QJsonValue &v, int index)
{
    if (!v.isObject()) return defaultAxis(index);
    const QJsonObject o = v.toObject();
    AxisConfig a;
    const QString type = o.value(u"sourceType"_s).toString();
    a.source = type == "button"_L1 ? AxisConfig::Source::Button
             : type == "none"_L1   ? AxisConfig::Source::None
                                                : AxisConfig::Source::Axis;
    a.sourceIndex = validIndex(o.value(u"sourceIndex"_s)) ? o.value(u"sourceIndex"_s).toInt() : index;
    const QJsonValue ch = o.value(u"channel"_s);
    a.channel = ch.isDouble() && ch.toDouble() == std::floor(ch.toDouble()) && ch.toInt() >= 0 && ch.toInt() <= kChannels ? ch.toInt() : 0;
    a.inverted = o.value(u"inverted"_s).toBool();
    a.deadzone = clampOr(o.value(u"deadzone"_s), 0, 0.9, 0.05);
    a.expo = clampOr(o.value(u"expo"_s), 0, 1, 0);
    return a;
}

QJsonObject toJson(const AxisConfig &a)
{
    return {{u"sourceType"_s, a.source == AxisConfig::Source::Button ? u"button"_s
                                           : a.source == AxisConfig::Source::None ? u"none"_s
                                                                                  : u"axis"_s},
            {u"sourceIndex"_s, a.sourceIndex},
            {u"channel"_s, a.channel},
            {u"inverted"_s, a.inverted},
            {u"deadzone"_s, a.deadzone},
            {u"expo"_s, a.expo}};
}

ServoConfig servoFromJson(const QJsonValue &v, int index)
{
    ServoConfig s;
    s.servo = 9 + index;
    if (!v.isObject()) return s;
    const QJsonObject o = v.toObject();
    s.button = validIndex(o.value(u"button"_s)) ? o.value(u"button"_s).toInt() : -1;
    const QJsonValue sv = o.value(u"servo"_s);
    if (sv.isDouble() && sv.toDouble() == std::floor(sv.toDouble()) && sv.toInt() >= 1 && sv.toInt() <= 16) s.servo = sv.toInt();
    s.percent = clampOr(o.value(u"percent"_s), 0, 100, 100);
    return s;
}

QJsonObject toJson(const ServoConfig &s)
{
    return {{u"button"_s, s.button}, {u"servo"_s, s.servo}, {u"percent"_s, s.percent}};
}

QString sourceKey(const AxisConfig &a)
{
    switch (a.source) {
    case AxisConfig::Source::Axis: return u"axis:%1"_s.arg(a.sourceIndex);
    case AxisConfig::Source::Button: return u"button:%1"_s.arg(a.sourceIndex);
    case AxisConfig::Source::None: break;
    }
    return u"none:0"_s;
}

double applyDeadzone(double value, double deadzone)
{
    if (std::abs(value) < deadzone) return 0;
    // The rest of the range rescaled to 0..1
    const double sign = value > 0 ? 1 : -1;
    return sign * (std::abs(value) - deadzone) / (1 - deadzone);
}

double applyExpo(double value, double expo)
{
    if (expo == 0) return value;
    const double sign = value > 0 ? 1 : -1;
    const double a = std::abs(value);
    return sign * (expo * a * a * a + (1 - expo) * a);
}

int axisToPwm(double value)
{
    value = std::clamp(value, -1.0, 1.0);
    return int(std::lround(1500 + value * 500));
}

std::optional<double> readSource(const AxisConfig &a, const std::vector<double> &axes, const std::vector<double> &buttons)
{
    if (a.source == AxisConfig::Source::Axis) {
        if (a.sourceIndex < 0 || a.sourceIndex >= int(axes.size()) || !std::isfinite(axes[std::size_t(a.sourceIndex)])) return std::nullopt;
        return std::clamp(axes[std::size_t(a.sourceIndex)], -1.0, 1.0);
    }
    if (a.source == AxisConfig::Source::Button) {
        if (a.sourceIndex < 0 || a.sourceIndex >= int(buttons.size()) || !std::isfinite(buttons[std::size_t(a.sourceIndex)])) return std::nullopt;
        return std::clamp(buttons[std::size_t(a.sourceIndex)], 0.0, 1.0) * 2 - 1;
    }
    return std::nullopt;
}

std::optional<int> channelPwm(const AxisConfig &a, std::optional<double> raw)
{
    if (a.channel == 0 || !raw) return std::nullopt;
    double v = *raw;
    if (a.inverted) v = -v;
    v = applyDeadzone(v, a.deadzone);
    v = applyExpo(v, a.expo);
    return axisToPwm(v);
}

int servoPwm(const ServoConfig &s) { return int(std::lround(1000 + s.percent * 10)); }

} // namespace corv::joystick
