// SurfaceVolume.h - 3D surface of a cave from sonar / LiDAR returns (TSDF;
// port of js/ros/SurfaceVolume.js).
//
// A height field holds one surface per column: right for terrain and sea bed,
// wrong in a cave, a shaft or under an overhang, where the floor, the walls
// and the ceiling share columns. Here the space is cut into voxels (CELL, 30 cm
// by default) holding a truncated signed distance to the surface, averaged
// over the returns - positive on the open side, negative behind the rock - and
// the mesh is where it changes sign. Every wall comes out with its own
// orientation, nothing has to be told which way it faces.
//
// Integration (KinectFusion's TSDF, as cave and ship-hull sonar mappers use
// it). For a return at range r from the sensor, the voxels the ray crosses
// from r - TRUNC to r + TRUNC get the sample clamp((r - t) / TRUNC, -1, 1),
// t their distance along the ray; each voxel keeps a weighted running mean of
// at most MEMORY samples, like the 2D cells (SurfaceTiles). Free water far
// in front of the surface is not carved: only the band around it matters.
//
// Meshing (surface nets). In each cube of 8 neighbouring voxels, all of them
// observed, whose signs differ, one vertex at the mean of its edges' zero
// crossings; each voxel edge that changes sign joins the four cubes around it
// into a quad, two triangles. Voxels are grouped in chunks of 16^3 (4.8 m at
// 30 cm); a chunk is meshed again when it or a neighbour it borders changes,
// and only those meshes travel to the renderer.
//
// Coordinates: metres east, north, up from the caller's anchor.
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace corv {

/// A chunk's mesh as it leaves the volume: triangles, x y z per vertex, three vertices a triangle.
struct VolumeMesh {
    std::int64_t key = 0;
    int cx = 0, cy = 0, cz = 0;
    std::vector<float> pos;
};

class SurfaceVolume
{
public:
    static constexpr int kChunk = 16;
    static constexpr int kN3 = kChunk * kChunk * kChunk;
    static std::int64_t chunkKey(int cx, int cy, int cz);

    struct Chunk {
        std::int64_t key = 0;
        int cx = 0, cy = 0, cz = 0;
        std::array<float, kN3> d{};
        std::array<std::uint8_t, kN3> w{};
    };
    struct Changes {
        std::vector<VolumeMesh> meshes;
        std::vector<std::int64_t> removed;
    };

    SurfaceVolume();

    /// cell: voxel size, m; memory: samples a voxel averages over (<= 255); maxChunks: chunks kept (16^3 voxels, 20 KB each).
    bool configure(double cell, double memory, double maxChunks);
    void reset();

    /// Fuse the returns of one message seen from sensor position o (e, n, u). Returns the returns used.
    int integrate(const double o[3], const double *enu, int count);

    /// Surface nets mesh of one chunk: x y z per vertex (metres from the anchor), not indexed.
    std::vector<float> meshChunk(const Chunk &c) const;

    Changes takeChanges();
    Changes takeAll();

    int size() const { return int(m_chunks.size()); }
    double cell() const { return m_cell; }
    int maxChunks() const { return m_maxChunks; }
    const std::unordered_map<std::int64_t, std::unique_ptr<Chunk>> &chunks() const { return m_chunks; }

private:
    Chunk *chunk(int cx, int cy, int cz, bool create);
    const Chunk *findChunk(int cx, int cy, int cz) const;
    bool voxel(int i, int j, int k, float *d) const;
    void touch(const Chunk &c, int li, int lj, int lk);
    void evict(const double o[3]);

    double m_cell = 0.3;
    int m_memory = 20;
    int m_maxChunks = 1024;
    std::unordered_map<std::int64_t, std::unique_ptr<Chunk>> m_chunks;
    std::unordered_set<std::int64_t> m_dirty;
    std::vector<std::int64_t> m_removed;
};

} // namespace corv
