// Geo.h - coordinate helpers shared by every library (port of js/core/utils.js).
//
// The 3D world is in metres from a geographic origin: X east, Y up, Z south,
// flat-earth with 111320 m per degree (the same convention as the Electron
// version, so terrain, trail and route line up the same way).
#pragma once

#include <cmath>

namespace corv {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr double kRad = 180.0 / kPi;
constexpr double kMetersPerDegLat = 111320.0;

struct LatLon {
    double lat = 0;
    double lon = 0;
    bool operator==(const LatLon &) const = default;
};

/// World metres on the ground plane: x east, z south.
struct GroundMeters {
    double x = 0;
    double z = 0;
};

/// Default scene origin before any vehicle position (Innsbruck, as in js/core/constants.js).
constexpr LatLon kDefaultOrigin{47.2603, 11.3439};

GroundMeters latLonToMeters(double lat, double lon, LatLon origin);
LatLon metersToLatLon(double x, double z, LatLon origin);

/// Flat-earth distance (calculateDistance in utils.js): metres per degree of longitude at lat1.
double flatDistance(double lat1, double lon1, double lat2, double lon2);
/// Great-circle distance on a 6371 km sphere (mission-store summaries, link budgets).
double haversine(double lat1, double lon1, double lat2, double lon2);

struct TileXY {
    int x = 0;
    int y = 0;
    int z = 0;
};

struct TileBounds {
    double latTop = 0;
    double latBottom = 0;
    double lonLeft = 0;
    double lonRight = 0;
};

TileXY latLonToTile(double lat, double lon, int zoom);
TileBounds tileToBounds(int x, int y, int z);

struct Rgb {
    float r = 0, g = 0, b = 0;
};

/// Terrain height palette of getHeightColor(): green below 700 m, then sand, orange, red, purple, blue.
Rgb heightColor(double height);

inline double wrapPi(double a) { return std::atan2(std::sin(a), std::cos(a)); }
inline double wrapDeg180(double a) { return std::fmod(std::fmod(a + 540.0, 360.0) + 360.0, 360.0) - 180.0; }

} // namespace corv
