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

} // namespace corv
