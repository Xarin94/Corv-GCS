// GeoZoneRaster.h - UAS geographical zones as textures for the 3D terrain (the
// rasterisation of js/terrain/GeoZoneTerrain.js), without GPU resources.
//
// Two textures over a square window (20 km) that follows the aircraft, laid
// out in lat/lon, which latLonToMeters() maps linearly to world x/z, so a new
// scene origin only changes where the window sits, never the raster:
//   coverage (RGBA8): R no-fly zones from the surface, G conditional zones
//     from the surface, B zones that start above the ground; the fraction of
//     each texel inside them (antialiased), so the shader cuts the outline at
//     one half along the polygon and not along the texels.
//   floors (RGBA16F): R the lowest floor AGL, G the lowest floor MSL (60000:
//     none), spread one texel past every texel a zone touches so the filtered
//     value is a real floor wherever the coverage says there is a zone.
// The terrain shader turns the floors into a height above its own ground,
// min(R, G - h): a floor above sea level is right on every slope, and one at
// or under the ground there is a no-fly zone. Built off the GUI thread.
#pragma once

#include "scene/GeoZone.h"

#include <cstdint>
#include <memory>
#include <vector>

namespace corv {

struct GeoZoneRaster {
    static constexpr int kSize = 1024;           // texels per side (~20 m)
    static constexpr double kWindowKm = 20;
    static constexpr double kRecenterKm = 4;     // built again this far from the window's centre
    static constexpr float kNone = 60000.f;      // no floor (half float max is 65504)

    double lat0 = 0, lat1 = 0, lon0 = 0, lon1 = 0;   // south, north, west, east
    double centerLat = 0, centerLon = 0;
    std::vector<uint8_t> coverage;               // RGBA8, row 0 north
    std::vector<uint16_t> floors;                // RGBA16F half floats, row 0 north
    int version = 0;
    bool empty = true;                           // no zone touches the window

    /// Rasterise the zones around a centre.
    static std::shared_ptr<GeoZoneRaster> build(const GeoZoneList &zones, double centerLat, double centerLon, int version);
};

} // namespace corv
