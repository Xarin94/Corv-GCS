#include "core/Geo.h"

#include <algorithm>

namespace corv {

GroundMeters latLonToMeters(double lat, double lon, LatLon origin)
{
    const double metersPerLon = kMetersPerDegLat * std::cos(origin.lat * kDeg);
    return {(lon - origin.lon) * metersPerLon, -(lat - origin.lat) * kMetersPerDegLat};
}

LatLon metersToLatLon(double x, double z, LatLon origin)
{
    const double metersPerLon = kMetersPerDegLat * std::cos(origin.lat * kDeg);
    return {origin.lat - z / kMetersPerDegLat, origin.lon + x / metersPerLon};
}

double flatDistance(double lat1, double lon1, double lat2, double lon2)
{
    const double metersPerLon = kMetersPerDegLat * std::cos(lat1 * kDeg);
    const double dLat = (lat2 - lat1) * kMetersPerDegLat;
    const double dLon = (lon2 - lon1) * metersPerLon;
    return std::sqrt(dLat * dLat + dLon * dLon);
}

double haversine(double lat1, double lon1, double lat2, double lon2)
{
    constexpr double R = 6371000.0;
    const double dLat = (lat2 - lat1) * kDeg;
    const double dLon = (lon2 - lon1) * kDeg;
    const double h = std::pow(std::sin(dLat / 2), 2)
        + std::cos(lat1 * kDeg) * std::cos(lat2 * kDeg) * std::pow(std::sin(dLon / 2), 2);
    return 2 * R * std::asin(std::sqrt(h));
}

TileXY latLonToTile(double lat, double lon, int zoom)
{
    const double n = std::pow(2.0, zoom);
    const double latRad = lat * kDeg;
    return {int(std::floor((lon + 180.0) / 360.0 * n)),
            int(std::floor((1.0 - std::asinh(std::tan(latRad)) / kPi) / 2.0 * n)),
            zoom};
}

TileBounds tileToBounds(int x, int y, int z)
{
    const double n = std::pow(2.0, z);
    TileBounds b;
    b.lonLeft = x / n * 360.0 - 180.0;
    b.lonRight = (x + 1) / n * 360.0 - 180.0;
    b.latTop = std::atan(std::sinh(kPi * (1 - 2 * y / n))) * kRad;
    b.latBottom = std::atan(std::sinh(kPi * (1 - 2 * (y + 1) / n))) * kRad;
    return b;
}

Rgb heightColor(double h)
{
    if (h <= -100) return {0, 0, 0};
    static const float G[3] = {0.0431372549f, 0.4f, 0.137254902f};
    static const float Y[3] = {0.902f, 0.7647058824f, 0.3529411765f};
    static const float O[3] = {0.902f, 0.494f, 0.133f};
    static const float R[3] = {0.906f, 0.298f, 0.235f};
    static const float P[3] = {0.608f, 0.349f, 0.713f};
    static const float B[3] = {0.204f, 0.596f, 0.858f};
    auto mix = [](const float *a, const float *b, double t) {
        const float k = float(t);
        return Rgb{a[0] + (b[0] - a[0]) * k, a[1] + (b[1] - a[1]) * k, a[2] + (b[2] - a[2]) * k};
    };
    if (h <= 700) return {G[0], G[1], G[2]};
    if (h <= 1400) return mix(G, Y, (h - 700) / 700);
    if (h <= 2100) return mix(Y, O, (h - 1400) / 700);
    if (h <= 2800) return mix(O, R, (h - 2100) / 700);
    if (h <= 3500) return mix(R, P, (h - 2800) / 700);
    if (h <= 4000) return mix(P, B, (h - 3500) / 500);
    return {B[0], B[1], B[2]};
}

} // namespace corv
