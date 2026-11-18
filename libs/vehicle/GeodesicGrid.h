// GeodesicGrid.h - the 80 sections ArduPilot's compass calibrator covers (port
// of js/ui/GeodesicGrid.js).
//
// MAG_CAL_PROGRESS.completion_mask is 80 bits, one per section of
// AP_GeodesicGrid (libraries/AP_Math/AP_GeodesicGrid.cpp): an icosahedron whose
// 20 triangles are each split into four, the section number being
// 4 * triangle + sub-triangle. To show which bits are set the GCS needs the same
// numbering, so the sections are built from the firmware's own tables rather
// than from an icosahedron of our own (whose triangle order would not match).
#pragma once

#include <QVector3D>
#include <array>
#include <cstdint>

namespace corv::geogrid {

constexpr int kSections = 80;

struct Section {
    QVector3D a, b, c;    // unit vectors, counter-clockwise seen from outside
    QVector3D center;     // unit vector through the centroid
};

/// The 80 sections, indexed by section number. Body frame (x nose, y right, z down).
const std::array<Section, kSections> &sections();

/// Section a direction falls in (AP_GeodesicGrid::section, done by testing the
/// 80 spherical triangles); -1 for the null vector.
int sectionOf(const QVector3D &v);

/// The 10-byte completion mask of MAG_CAL_PROGRESS.
using Mask = std::array<uint8_t, 10>;
inline bool maskHas(const Mask &m, int section)
{
    return section >= 0 && section < kSections && (m[std::size_t(section >> 3)] & (1u << (section & 7))) != 0;
}
int maskCount(const Mask &m);

} // namespace corv::geogrid
