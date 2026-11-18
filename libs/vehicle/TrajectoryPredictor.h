// TrajectoryPredictor.h - the predicted flight path (port of js/engine/TrajectoryPredictor.js).
//
// From the current speed, turn and climb the path is integrated forward in
// steps: the bank evolves along it (the measured roll rate carries it at
// first, decaying over 0.7 s, then it relaxes toward the autopilot's commanded
// roll while NAV_CONTROLLER_OUTPUT is fresh), the turn rate follows the bank
// (g tan(bank) / V), the climb follows the vertical speed plus a decaying
// vertical acceleration from the NED vd derivative. The inputs go through a
// low-pass filter first, so the corridor does not jitter.
// The length of the prediction scales with ground speed: 5 s at 5 m/s up to
// 20 s at 60 m/s (getPredictionTime of TrajectoryCorridor3D.js).
#pragma once

#include <cstdint>
#include <vector>

namespace corv {

struct VehicleState;

class TrajectoryPredictor
{
public:
    struct Point {
        double lat = 0, lon = 0, alt = 0;   // alt MSL
        double t = 0;                       // seconds ahead
        double bank = 0;                    // predicted bank, rad
    };

    /// Seconds of prediction for a ground speed.
    static double predictionTime(double gs);

    /// Path from (lat, lon, alt), the drawn position, with the rest of the state's
    /// motion. nowMs is the wall clock (filter steps, guidance freshness).
    std::vector<Point> predict(const VehicleState &s, double lat, double lon, double alt, int numPoints, double totalTimeSec,
                               std::int64_t nowMs);

    void reset() { m_init = false; }

private:
    bool m_init = false;
    double m_gs = 0, m_roll = 0, m_rollRate = 0, m_vs = 0, m_vertAccel = 0;
    double m_trackSin = 0, m_trackCos = 1;
    double m_prevVd = 0;
    std::int64_t m_prevVdTime = 0;
};

} // namespace corv
