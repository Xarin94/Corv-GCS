// RosSurfaceData.h - the averaged ROS surface as the 3D view draws it (the
// data side of js/ros/RosMesh3D.js and js/ros/RosVolume3D.js).
//
// Height field. The tiles RosClient averages (30 x 30 cells) are stored in
// blocks of 8 x 8 tiles (240 x 240 cells) plus the first row and column of
// the blocks east and north, so the blocks join (241 x 241). Each block keeps
// the measured heights and their samples, and what is drawn of them
// (raster::rasterize: adaptive level per cell, holes interpolated, nothing
// across a jump) with, in the ROUGHNESS colour mode, the roughness of every
// cell (raster::roughness). The renderer turns a block into a texture and, at
// the level of detail its distance asks for, an index list (raster::triangulate).
//
// Volume. In a cave (SURFACES 3D) the client sends one triangle soup per
// chunk of 16^3 voxels; each comes with the spread of the face normals around
// every vertex (1 - |area-weighted mean normal|): the roughness of a wall.
//
// One look for both: grey lines over a transparent body, or a solid surface
// coloured by distance from the vehicle or by roughness (the scales are kept
// here). Written by RosController on the GUI thread; the 3D view reads it in
// synchronize() while the GUI thread is blocked.
#pragma once

#include "scene/SurfaceTiles.h"
#include "scene/SurfaceVolume.h"

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace corv {

/// The local metric frame the client places points in (RosPoints makeAnchor), and the frame it came from.
struct RosAnchor {
    double lat = 0, lon = 0, alt = 0;
    double mPerLat = 111320, mPerLon = 111320;
    bool rel = false;   // relative navigation frame
    int epoch = 0;
};

class RosSurfaceData
{
public:
    static constexpr int kBlockTiles = 8;
    static constexpr int kBlock = kBlockTiles * SurfaceTiles::kTile;   // 240 cells
    static constexpr int kTex = kBlock + 1;                             // + the next blocks' first row / column
    enum Mode { Gray = 0, Distance = 1, Rough = 2 };

    struct TileRec {
        int tx = 0, ty = 0;
        std::vector<float> h, w;
        float minH = 0, maxH = -1;   // maxH < minH: no height
    };
    struct Block {
        int bx = 0, by = 0;
        std::vector<float> data, wts;   // measured heights (kEmpty where none), their samples
        std::vector<float> view;        // drawn: height, level per cell (kTex * kTex * 2)
        std::vector<std::uint8_t> rough;   // cm, ROUGHNESS mode only
        bool roughDone = false;
        double roughN = 0, roughSum = 0, roughSq = 0;
        std::unordered_set<std::int64_t> tiles;
        bool dirty = true;
        int revision = 0;               // bumped whenever view / rough change
        std::array<std::uint32_t, 5> levelCount{};
        float minH = 0, maxH = 0;
    };
    struct VolChunk {
        std::vector<float> pos;     // x y z per vertex, metres from the anchor (east, north, up)
        std::vector<float> rough;   // per vertex: spread of the face normals
        int revision = 0;
        float cx = 0, cy = 0, cz = 0, radius = 0;   // bounding sphere
        double n = 0, sum = 0, sq = 0;
    };
    struct View {
        bool visible = false;      // the ROS feature is on
        bool shown = true;         // the operator's VIEW ON / OFF
        bool overTerrain = true;   // drawn through the terrain
        double minSamples = 1;
        double fillM = 0;
        int mode = Gray;
        float fill = 0.1f;         // body opacity in the grey mode
        float nearM = 0, farM = 20;
        float roughMean = 0, roughFull = 0.05f;      // m RMS about the local plane
        float spreadMean = 0, spreadFull = 0.02f;    // 1 - |mean normal|
    };
    struct Stats {
        int tiles = 0, blocks = 0, triangles = 0;
        std::array<std::uint32_t, 5> levels{};
    };

    static RosSurfaceData &instance();

    /// Changed tiles from the client: reset starts over (also a new anchor epoch or cell size).
    void applyTiles(bool reset, const std::optional<RosAnchor> &anchor, double cell, const std::vector<TileUpdate> &tiles,
                    const std::vector<std::pair<int, int>> &removed);
    /// Changed chunk meshes of the 3D surface.
    void applyVolume(bool reset, const std::optional<RosAnchor> &anchor, double cell, const std::vector<VolumeMesh> &meshes,
                     const std::vector<std::int64_t> &removed);
    void clear();

    /// Rasterize the dirty blocks (and, in the roughness mode, their roughness). GUI thread.
    void fillDirty();
    /// Every block filled again (min samples, fill, colour mode changed).
    void markAllDirty();
    void setMode(int mode);
    /// Distance ramp and roughness scales, from the vehicle (ENU metres from the anchor).
    void updateScales(double vehE, double vehN, double vehU);

    Stats stats() const;
    int volumeTriangles() const;
    bool empty() const { return blocks.empty() && chunks.empty(); }

    // ── Read by the renderer ──
    std::optional<RosAnchor> anchor;     // height field
    double cell = 0.3;
    std::unordered_map<std::int64_t, TileRec> tiles;
    std::unordered_map<std::int64_t, std::unique_ptr<Block>> blocks;
    int generation = 0;                  // bumped when blocks go away all at once
    std::optional<RosAnchor> volAnchor;  // volume
    double volCell = 0.3;
    std::unordered_map<std::int64_t, std::shared_ptr<const VolChunk>> chunks;
    int volGeneration = 0;
    View view;
    /// What the 3D view drew last (it writes them in synchronize): blocks in view, triangles.
    int drawnBlocks = 0, drawnTriangles = 0;

    /// 1 - |area-weighted mean of the normals of the faces that share each vertex| (vertices shared by position).
    static std::vector<float> normalSpread(const std::vector<float> &pos);

private:
    RosSurfaceData() = default;
    Block *createBlock(int bx, int by);
    void removeTile(int tx, int ty);
    void writeTile(int tx, int ty, const TileRec *t);
    void fillBlock(Block &b);
    void updateRoughScale();
    void updateSpreadScale();
};

} // namespace corv
