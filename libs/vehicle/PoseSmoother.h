// PoseSmoother.h - the pose the 3D view and the HUD draw the vehicle at
// (port of js/core/PoseSmoother.js, the 1.8.5 fix for 25 Hz pose steps).
//
// Telemetry writes the state at its own rate while the view renders at 60 fps.
// Between samples the pose is projected forward for at most 0.5 s: position on
// the NED velocity and the acceleration seen between velocity samples (along
// the turn that acceleration describes), attitude on the angular rates and, for
// one sample interval, on how fast they were changing. When a sample arrives the
// gap between the drawn pose and the sample is blended out over tau seconds
// instead of jumped; in attitude the angular velocity too: the drawn one is
// carried into the one the new sample propagates on (critically damped, tau
// seconds), so the horizon does not change pace at every sample.
// Samples are timed by the vehicle's own clock (time_boot_ms) when reported,
// mapped onto the GCS clock by the quickest arrival seen.
//
// tau comes from SYS CONFIG -> MOTION SMOOTHING (tauFor): 0 draws the telemetry
// as it arrives, position and attitude, without any propagation.
#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <optional>

namespace corv {

struct VehicleState;

struct Pose {
    double lat = 0, lon = 0, alt = 0;
    double roll = 0, pitch = 0, yaw = 0;
};

class PoseSmoother
{
public:
    /// Inputs (a subset of VehicleState, so the class is testable on its own).
    struct Sample {
        double lat = 0, lon = 0, rawAlt = 0;
        double roll = 0, pitch = 0, yaw = 0;
        double vn = 0, ve = 0, vs = 0, gs = 0, track = 0;
        double rollRate = 0, pitchRate = 0, yawRate = 0;
        double posTime = 0, attTime = 0;                 // wall ms
        std::optional<double> posBootMs, attBootMs;
    };

    static Sample fromState(const VehicleState &s);

    /// MOTION SMOOTHING: the slider's 0..0.5, its default, and the blend time constant it stands for (s).
    static constexpr double kMaxSmoothing = 0.5;
    static constexpr double kDefaultSmoothing = 0.15;
    static constexpr double tauFor(double smoothing)
    {
        return (smoothing > 0 ? smoothing < kMaxSmoothing ? smoothing : kMaxSmoothing : 0) * 0.5;
    }

    PoseSmoother() { reset(); }
    void reset();
    /// Pose to draw at time now (wall ms). tau: blend time constant (s), 0 draws the sample unchanged.
    const Pose &update(const Sample &s, double now, double tau);
    const Pose &pose() const { return m_pose; }

private:
    struct Channel {
        std::array<double, 3> rate{}, accel{}, offset{}, projection{};
        std::array<double, 3> slip{};  // drawn rate - the sample's rate when it was taken (attitude)
        double hold = 0.5;             // s the acceleration acts for: then the rate it reached holds
        std::optional<double> t;       // GCS clock, ms, when the sample was taken
        std::optional<double> stamp;   // arrival stamp of the last sample
        std::optional<double> boot;    // vehicle clock of the last sample
        std::deque<double> lags;       // arrival - vehicle clock
        double span(double now) const;
        double rateAt(int i, double now) const;
        std::array<double, 3> &projectLinear(double now);
        double decay(double now, double tau) const;
        /// The part of the gap to the sample (angle and rate) still to fade out, and how fast it fades.
        double blend(int i, double now, double tau) const;
        double blendRate(int i, double now, double tau) const;
        struct Timed { double t; std::optional<double> since; };
        Timed time(double arrival, std::optional<double> boot, double now);
        double gap(double t, std::optional<double> since) const;
    };

    std::array<double, 3> &projectPosition(double now);
    void takePosition(const Sample &s, double now, double tau);
    void takeAttitude(const Sample &s, double now, double tau);

    Pose m_pose;
    Channel m_pos, m_att;
    struct Raw { double lat, lon, alt, roll, pitch, yaw; } m_raw{};
    std::optional<std::array<double, 3>> m_velocity;
};

} // namespace corv
