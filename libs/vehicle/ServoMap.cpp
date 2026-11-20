#include "vehicle/ServoMap.h"

#include <QStringList>
#include <algorithm>
#include <cmath>

using namespace Qt::StringLiterals;

namespace corv {

namespace servo {
int motorNumber(int f)
{
    if (f >= kMotor1 && f < kMotor1 + 8) return f - kMotor1 + 1;
    if (f >= kMotor9 && f < kMotor9 + 4) return f - kMotor9 + 9;
    if (f >= kMotor13 && f < kMotor13 + 20) return f - kMotor13 + 13;
    return 0;
}
} // namespace servo

namespace {
double clamp(double v, double lo, double hi) { return std::min(std::max(v, lo), hi); }
} // namespace

ServoMap ServoMap::read(const Lookup &param)
{
    ServoMap m;
    m.m_fn.fill(-1);
    for (int ch = 1; ch <= servo::kOutputs; ++ch) {
        const auto f = param(u"SERVO%1_FUNCTION"_s.arg(ch));
        if (!f) continue;
        const int fn = int(std::lround(*f));
        m.m_fn[std::size_t(ch - 1)] = fn;
        ++m.m_known;
        if (const int n = servo::motorNumber(fn); n && !m.m_motors.count(n)) m.m_motors[n] = ch;
    }
    return m;
}

OutputScale ServoMap::scale(const Lookup &param, int output)
{
    OutputScale s;
    const auto get = [&](const char *what) { return param(u"SERVO%1_%2"_s.arg(output).arg(QLatin1String(what))); };
    if (const auto v = get("MIN")) s.min = *v;
    if (const auto v = get("TRIM")) s.trim = *v;
    if (const auto v = get("MAX")) s.max = *v;
    if (const auto v = get("REVERSED")) s.reversed = *v >= 0.5;
    return s;
}

int ServoMap::function(int output) const
{
    if (output < 1 || output > servo::kOutputs) return -1;
    return m_fn[std::size_t(output - 1)];
}

int ServoMap::first(int fn) const
{
    for (int ch = 1; ch <= servo::kOutputs; ++ch)
        if (m_fn[std::size_t(ch - 1)] == fn) return ch;
    return 0;
}

std::vector<ServoMap::Engine> ServoMap::throttles() const
{
    std::vector<Engine> out;
    for (int ch = 1; ch <= servo::kOutputs; ++ch) {
        const int fn = m_fn[std::size_t(ch - 1)];
        // One gauge per engine: a second output on the same throttle is the
        // same engine (a servo Y-lead in software)
        const auto same = [&](const Engine &e) { return e.function == fn; };
        if (std::any_of(out.begin(), out.end(), same)) continue;
        if (fn == servo::kThrottle) out.push_back({ch, fn, 0});
        else if (fn == servo::kThrottleLeft) out.push_back({ch, fn, -1});
        else if (fn == servo::kThrottleRight) out.push_back({ch, fn, 1});
    }
    std::stable_sort(out.begin(), out.end(), [](const Engine &a, const Engine &b) { return a.side < b.side; });
    return out;
}

ServoMap::Flaps ServoMap::flaps() const
{
    Flaps f;
    if (const int ch = first(servo::kFlapAuto)) {
        f.kind = Flaps::Range;
        f.output = ch;
        return f;
    }
    f.left = first(servo::kFlaperonLeft);
    f.right = first(servo::kFlaperonRight);
    if (f.left || f.right) {
        f.kind = Flaps::Flaperons;
        return f;
    }
    if (const int ch = first(servo::kFlap)) {
        f.kind = Flaps::Range;
        f.output = ch;
    }
    return f;
}

QStringList ServoMap::flapParams() const
{
    QStringList names;
    const Flaps f = flaps();
    const auto add = [&](int ch, bool trim) {
        if (!ch) return;
        names << u"SERVO%1_MIN"_s.arg(ch) << u"SERVO%1_MAX"_s.arg(ch)
              << u"SERVO%1_REVERSED"_s.arg(ch);
        if (trim) names << u"SERVO%1_TRIM"_s.arg(ch);
    };
    if (f.kind == Flaps::Range) add(f.output, false);
    if (f.kind == Flaps::Flaperons) {
        add(f.left, true);
        add(f.right, true);
    }
    return names;
}

std::optional<double> ServoMap::flapPercent(const Flaps &src, const std::array<std::uint16_t, servo::kOutputs> &pwm,
                                            const std::function<OutputScale(int output)> &scale)
{
    const auto pwmOf = [&](int ch) -> std::optional<double> {
        if (ch < 1 || ch > servo::kOutputs) return std::nullopt;
        const double v = pwm[std::size_t(ch - 1)];
        return v > 0 ? std::optional<double>(v) : std::nullopt;
    };
    if (src.kind == Flaps::Range) {
        const auto v = pwmOf(src.output);
        if (!v) return std::nullopt;
        const OutputScale s = scale(src.output);
        if (s.max <= s.min) return std::nullopt;
        // SRV_Channel::pwm_from_range, inverted
        const double pct = clamp((*v - s.min) / (s.max - s.min), 0, 1) * 100;
        return s.reversed ? 100 - pct : pct;
    }
    if (src.kind == Flaps::Flaperons) {
        // SRV_Channel::pwm_from_angle, inverted: −1..+1 of full deflection
        const auto deflection = [&](int ch) -> std::optional<double> {
            const auto v = pwmOf(ch);
            if (!v) return std::nullopt;
            const OutputScale s = scale(ch);
            double n = 0;
            if (*v >= s.trim) n = s.max > s.trim ? (*v - s.trim) / (s.max - s.trim) : 0;
            else n = s.trim > s.min ? -(s.trim - *v) / (s.trim - s.min) : 0;
            n = clamp(n, -1, 1);
            return s.reversed ? -n : n;
        };
        const auto l = deflection(src.left), r = deflection(src.right);
        // With one side only the aileron stays mixed in; better than nothing
        if (l && r) return clamp((*l - *r) / 2, -1, 1) * 100;
        if (l) return *l * 100;
        if (r) return -*r * 100;
    }
    return std::nullopt;
}

} // namespace corv
