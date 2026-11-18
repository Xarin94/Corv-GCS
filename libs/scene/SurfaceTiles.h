// SurfaceTiles.h - averaged height cells behind the ROS surface mesh (port of
// js/ros/SurfaceTiles.js).
//
// Points from a ROS sensor (terrain under an aircraft, the sea bed under a
// boat or a ROV) are not kept: each one only moves the mean height of the cell
// it falls in. Cells are small and fixed (CELL, 30 cm by default), so a rock,
// a step or a wreck survives the averaging; they are grouped in tiles of
// 30 x 30 cells (9 x 9 m at 30 cm), created only where data arrives, so the
// surface covers whatever is surveyed without a grid over the empty rest.
// A tile is the unit of the mesh: at most 29 x 29 quads, 1 682 triangles,
// and the unit sent to the renderer when something in it changes.
//
// Averaging. Every message gives each cell it touches one sample: the mean
// height of its points in that cell, weighted by how many there are (1 point
// = 1/3 of a sample, 10 points ~ 5/6), so a cell grazed by one return moves
// less than a cell the sensor saw well. A cell keeps a weighted running mean
// of at most MEMORY samples: a cumulative mean while it fills, a moving
// average after that, so a surface that changes (tide, a moved object, a bad
// first pass) is followed instead of being frozen by its history.
//
// Budget. Past MAX TILES the tiles farthest from the vehicle are dropped
// (2 048 tiles ~ 16 ha at 30 cm, ~16 MB here).
//
// Coordinates: metres east (x), north (y), up (heights) from the caller's
// anchor. Cell (i, j) of tile (tx, ty) spans x from (tx*30 + i)*CELL.
#pragma once

#include <QPointF>
#include <array>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace corv {

/// A changed tile as it leaves the averaging: heights (NaN where empty) and samples per cell.
struct TileUpdate {
    int tx = 0, ty = 0;
    std::vector<float> h, w;   // kTile * kTile, row-major (j * kTile + i)
};

class SurfaceTiles
{
public:
    static constexpr int kTile = 30;   // cells per tile side
    static std::int64_t tileKey(int tx, int ty);

    struct Tile {
        std::int64_t key = 0;
        int tx = 0, ty = 0;
        std::array<double, kTile * kTile> h{};
        std::array<float, kTile * kTile> w{};
        int filled = 0;
    };
    struct Changes {
        std::vector<TileUpdate> tiles;
        std::vector<std::pair<int, int>> removed;
    };

    SurfaceTiles();

    /// Apply settings (non-finite or out of range: unchanged); a new cell size clears the surface. True if it did.
    bool configure(double cell, double memory, double maxTiles);
    void reset();

    /// Average one message's points: x, y, up triples; vx, vy the vehicle (for the budget). Returns the points used.
    int add(const double *enu, int count, double vx = 0, double vy = 0);

    /// Tiles changed since the last call and tiles dropped.
    Changes takeChanges();
    /// Every tile, for a renderer that starts over.
    Changes takeAll();

    int size() const { return int(m_tiles.size()); }
    double cell() const { return m_cell; }
    double memory() const { return m_memory; }
    int maxTiles() const { return m_maxTiles; }
    int filled() const { return m_filled; }
    int version() const { return m_version; }
    const std::unordered_map<std::int64_t, std::unique_ptr<Tile>> &tiles() const { return m_tiles; }

private:
    Tile *tileAt(int tx, int ty);
    void evict(double vx, double vy);

    double m_cell = 0.3;
    double m_memory = 20;
    int m_maxTiles = 2048;
    int m_version = 0;
    int m_filled = 0;
    std::unordered_map<std::int64_t, std::unique_ptr<Tile>> m_tiles;
    std::unordered_set<std::int64_t> m_dirty;
    std::vector<std::pair<int, int>> m_removed;
    // Per-message aggregation, reused
    std::unordered_map<std::int64_t, int> m_slots;
    std::vector<double> m_sum;
    std::vector<std::uint32_t> m_cnt;
    std::vector<int> m_idx;
    std::vector<Tile *> m_slotTile;
};

/// Area of a polygon (x east, y north), m^2.
double polygonAreaXY(const std::vector<QPointF> &poly);

/// Area of a polygon (x east, y north, metres from the anchor) the surface
/// covers: filled cells whose centre is inside, m^2. A tile wholly inside counts
/// its filled cells at once; only the tiles on the edge are looked at cell by cell.
/// With block > 1 the area is counted in blocks of block x block cells - the
/// holes the mesh fills (fillLevelsFor) - a block covered when any of its cells
/// has data and its centre is inside: a sector sonar's sweeps metres apart
/// cover the area they are drawn over.
double coveredArea(const SurfaceTiles &surface, const std::vector<QPointF> &poly, int block = 1);

} // namespace corv
