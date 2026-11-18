// SurfaceRaster.h - from averaged cells to the heights and triangles drawn
// (port of js/ros/SurfaceRaster.js).
//
// Pure functions (no graphics) used on each block of the ROS surface and by
// the offline tests. A grid is w x h cells, row-major: `data` the measured
// height of each cell (kEmpty where none), `wts` its samples.
//
// Adaptive resolution (rasterize). The cells are also averaged into levels of
// 2, 4, 8, 16 cells (pull, weighted by samples), and each cell is drawn at the
// finest level with MIN SAMPLES (push, bilinear): its own height where the
// sensor saw it well, a wider average where it did not, so one stray return is
// never a vertex on its own.
//
// Discontinuities. A height field of a cave or of a terrain with a cliff has
// jumps: the lake bed at -2 m next to the bottom of a shaft at -11.5 m, a near
// wall next to a far one. Averaging or interpolating across a jump draws a
// surface where there is only water or air - a funnel instead of a shaft. So:
//   - a coarse cell whose cells span more than JUMP (max(0.4 m, 1.5 x its
//     size): steeper than 56 deg) is not an average of anything;
//   - a cell with data takes a coarser level only if that level agrees with
//     its own height within JUMP;
//   - an empty cell is filled only by interpolation - the four coarse cells
//     around it valid and agreeing, at most `fillLevels` (FILL_LEVELS: 4
//     cells, 1.2 m at 30 cm; up to 16 cells for a sensor that samples in
//     lines, a sector sonar's sweeps metres apart) - never extended past the
//     edge of the data;
//   - a triangle whose corners span more than JUMP over its size is not drawn.
// jumps = false turns these off (the behaviour before them; tests compare).
#pragma once

#include <array>
#include <cstdint>

namespace corv::raster {

constexpr float kEmpty = -1e9f;
constexpr int kLevels = 4;       // coarsest: 2^4 = 16 cells
constexpr int kFillLevels = 2;   // empty cells: interpolated from 2 or 4-cell levels only, by default

double jumpTolerance(double spanCells, double cell);

/// Levels holes are interpolated from for "fill holes up to `metres`" at a cell
/// size: the coarsest level no wider than that (1 ... kLevels); 0 or less: the
/// default kFillLevels. Also the block of cells the coverage of an area is
/// counted in (2^levels cells), so it measures what is drawn.
int fillLevelsFor(double metres, double cell);

struct Options {
    double minSamples = 1;
    double cell = 0.3;
    bool jumps = true;
    int fillLevels = kFillLevels;
};

/// Heights to draw: out[2p] = height (kEmpty: none), out[2p + 1] = level used.
/// Returns the cells drawn per level.
std::array<std::uint32_t, kLevels + 1> rasterize(const float *data, const float *wts, int w, int h, float *out, const Options &opts = {});

/// Roughness of the drawn surface: at each drawn cell, the RMS distance of the
/// heights around it (within `radius` m) from the plane that fits them best - the
/// variance about the local plane. A flat bed, level or sloping, is 0; ripples,
/// rocks, a wreck, a scarp, a bad average are not. Summed-area tables of the
/// fit's moments make it O(1) per cell whatever the radius.
/// view: rasterize()'s output; out: per cell the RMS in cm, 0-255 (255: 2.55 m or more).
void roughness(const float *view, int w, int h, std::uint8_t *out, double cell = 0.3, double radius = 1.5);

/// Triangles over a w x h raster at vertex spacing s (cells): two per quad with
/// four corners drawn, one where three are; vertex ids are row-major on the
/// (w - 1) / s + 1 grid. With jumps on, a triangle spanning more than JUMP over
/// its size is left out. Returns the indices written to out.
/// stride: floats per cell of view (2 for rasterize()'s output).
int triangulate(const float *view, int w, int h, int s, std::uint16_t *out, double cell = 0.3, bool jumps = true, int stride = 2);

} // namespace corv::raster
