// DemoFlight.h - the demo flight with no link (port of the demo part of js/main.js).
//
// A fixed wing flies a closed racetrack (two 1100 m straights joined by 340 m
// radius turns) around where the vehicle stands, banking into coordinated
// turns (yaw rate = g tan(bank) / V), at 50 m above the circuit centre's
// terrain with gentle rate-limited climbs over rising ground, in a slowly
// varying wind. It writes the state like a telemetry source; VehicleHub runs
// it while the mode is LIVE and nothing is connected.
#pragma once

#include "core/Geo.h"

#include <functional>
#include <optional>
#include <random>
#include <vector>

namespace corv {

struct VehicleState;

class DemoFlight
{
public:
    using TerrainFn = std::function<std::optional<double>(double lat, double lon)>;

    explicit DemoFlight(TerrainFn terrain) : m_terrain(std::move(terrain)) {}

    /// Advance by dt seconds at wall time nowMs.
    void step(VehicleState &s, double dt, double nowMs);
    void reset() { m_init = false; }

private:
    void init(VehicleState &s);

    TerrainFn m_terrain;
    bool m_init = false;
    bool m_baseAltLocked = false;
    double m_centerLat = 0, m_centerLon = 0, m_baseAlt = 0;
    std::vector<LatLon> m_wps;
    std::size_t m_wp = 0;
    std::optional<double> m_prevWpDist;
    double m_bank = 0, m_speed = 0, m_speedTarget = 0;
    double m_windN = 0, m_windE = 0, m_windTargetN = 0, m_windTargetE = 0, m_windChange = 0;
    std::mt19937 m_rng{1234};
};

} // namespace corv
