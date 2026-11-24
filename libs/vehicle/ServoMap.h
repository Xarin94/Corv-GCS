// ServoMap.h - what an ArduPilot drives on each servo output, from its
// SERVOn_FUNCTION parameters (SRV_Channel::Function), and the flap deployment
// read back from the PWM of those outputs.
//
// The ROTOR LOAD schematic draws the motors the outputs are assigned to (a
// quadplane's four lift motors and its pusher, a twin's left and right
// throttle) and the flap indicator above it is there only when an output
// carries flaps or flaperons.
//
// Flap and FlapAuto are range outputs: 0 % at SERVOn_MIN, 100 % at SERVOn_MAX
// (the other way round when SERVOn_REVERSED). Flaperons are angle outputs
// around SERVOn_TRIM that mix aileron and flap (Plane::flaperon_update):
// left = aileron + 45·flap %, right = aileron − 45·flap %, out of ±4500, so the
// flap is half the difference of the two normalised deflections. Pure logic,
// no Qt object: the parameters come in through a lookup function.
#pragma once

#include <QString>
#include <QStringList>
#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <vector>

namespace corv {

namespace servo {
constexpr int kOutputs = 16;   // SERVO_OUTPUT_RAW carries servo1..16

constexpr int kFlap = 2;
constexpr int kFlapAuto = 3;
constexpr int kFlaperonLeft = 24;
constexpr int kFlaperonRight = 25;
constexpr int kGroundSteering = 26;
constexpr int kHeliRsc = 31;
constexpr int kHeliTailRsc = 32;
constexpr int kMotor1 = 33;     // Motor1..8 = 33..40
constexpr int kThrottle = 70;
constexpr int kThrottleLeft = 73;
constexpr int kThrottleRight = 74;
constexpr int kMotor9 = 82;     // Motor9..12 = 82..85
constexpr int kMastRotation = 137;
constexpr int kMotor13 = 160;   // Motor13..32 = 160..179

/// Motor number (1..32) of a MotorN function, else 0.
int motorNumber(int function);
} // namespace servo

/// SERVOn_MIN / _TRIM / _MAX / _REVERSED of one output (ArduPilot's defaults
/// until they are read).
struct OutputScale {
    double min = 1100, trim = 1500, max = 1900;
    bool reversed = false;
};

class ServoMap
{
public:
    using Lookup = std::function<std::optional<double>(const QString &name)>;

    /// The functions of SERVO1..16 as far as `param` knows them.
    static ServoMap read(const Lookup &param);
    static OutputScale scale(const Lookup &param, int output);

    /// Every SERVOn_FUNCTION read: the motor list is final.
    bool complete() const { return m_known == servo::kOutputs; }
    int known() const { return m_known; }
    /// Function of a 1-based output, -1 not read yet.
    int function(int output) const;
    /// First 1-based output carrying `function`, 0 none.
    int first(int function) const;

    /// MotorN functions: motor number → 1-based output (the first one).
    const std::map<int, int> &motors() const { return m_motors; }

    struct Engine {
        int output;     // 1-based
        int function;   // Throttle, ThrottleLeft, ThrottleRight
        int side;       // -1 left, 0 centre, +1 right
    };
    /// Fixed-wing throttles, left to right.
    std::vector<Engine> throttles() const;

    /// Where the flap deployment is read: FlapAuto (the effective setting,
    /// manual or automatic), else the flaperon pair, else Flap (manual only).
    struct Flaps {
        enum Kind { None, Range, Flaperons } kind = None;
        int output = 0;            // Range
        int left = 0, right = 0;   // Flaperons (either may be missing)
    };
    Flaps flaps() const;
    /// Output parameters the flap reading needs, given flaps().
    QStringList flapParams() const;

    /// A helicopter (AP_MotorsHeli): the swashplate is on Motor1..3 (and 5 with four servos), the
    /// tail on Motor4, or on a motor of its own; the rotor is the RSC output.
    struct Heli {
        std::vector<int> swash;   // 1-based outputs, by motor number
        int tail = 0;             // 0 none
        bool tailMotor = false;   // HeliTailRSC: a throttle, not a servo around its centre
        int rsc = 0;
    };
    /// ArduPilot's default outputs until every function is read.
    Heli heli() const;
    /// The collective as the PWM AP_MotorsHeli asks of the swashplate (between H_COL_MIN and H_COL_MAX):
    /// the mean of its servos, each put back the right way round; the third alone on an H1 head
    /// (H_SW_TYPE 1: roll, pitch and collective on a servo each). None without PWM on all of them.
    static std::optional<double> collectivePwm(const Heli &h, int swashType, const std::array<std::uint16_t, servo::kOutputs> &pwm,
                                               const std::function<OutputScale(int output)> &scale);

    /// Flap deployment in percent (0 = retracted, 100 = full; negative is
    /// reflex, from flaperons only), or none with no source or no PWM.
    static std::optional<double> flapPercent(const Flaps &src, const std::array<std::uint16_t, servo::kOutputs> &pwm,
                                             const std::function<OutputScale(int output)> &scale);

private:
    std::array<int, servo::kOutputs> m_fn{};
    int m_known = 0;
    std::map<int, int> m_motors;
};

} // namespace corv
