// PointCloudData.h - the LiDAR point cloud the 3D view draws (port of
// js/render/PointCloudData.js and the transform part of js/lidar/LidarCloud.js).
//
// Two parts:
//   map   the georeferenced points, metres from the cloud's origin, in blocks
//         of 262 144 that only grow (the renderer uploads what is new in each);
//         a new origin epoch or CLEAR MAP starts a new generation.
//   live  the scan that could not be georeferenced (gate closed), relative to
//         the vehicle - level NED or body frame - in a ring of 262 144 with the
//         time each point arrived: it fades out after `ttl` seconds.
// Points are kept as the 3D view's axes (x east, y up, z south) and an
// intensity 0..1, so a block uploads as it is. The height ramp runs over the
// 2nd-98th percentile of the map's heights (a histogram of 1 m bins).
//
// Written by LidarController on the GUI thread; the 3D view reads it in
// synchronize(), while the GUI thread is blocked. Blocks are shared so a
// render that follows still reads memory a CLEAR MAP let go of.
#pragma once

#include <QString>
#include <array>
#include <cstdint>
#include <memory>
#include <vector>

namespace corv {

struct PointCloudOrigin {
    double lat = 0, lon = 0, alt = 0;
    double mPerLat = 111320, mPerLon = 111320;
    int epoch = 0;
};

class PointCloudData
{
public:
    static constexpr int kChunk = 262144;
    static constexpr int kLiveCapacity = 262144;

    struct Block {
        int id = 0;
        int count = 0;
        std::unique_ptr<float[]> pts;   // x, y, z, intensity per point (kChunk * 4)
    };
    using BlockPtr = std::shared_ptr<Block>;

    static PointCloudData &instance();
    /// Seconds on the clock live points are stamped with (the renderer fades them on it).
    static double nowSeconds();

    void setOrigin(const PointCloudOrigin &o);
    /// Georeferenced points, ENU metres from the origin of `epoch` (a batch of an older epoch is dropped).
    void appendMapped(int epoch, const float *enu, const std::uint8_t *intensity, int count);
    /// Vehicle-relative points: frame "ned" (levelled) or "body" (FRD).
    void appendLive(const float *xyz, const std::uint8_t *intensity, int count, const QString &frame, double ttl);
    void clearLive();
    void clear();
    /// The height ramp from the histogram, at most every 500 ms.
    void updateRange(double nowMs);

    // ── Read by the renderer ──
    bool hasOrigin = false;
    PointCloudOrigin origin;
    std::vector<BlockPtr> chunks;
    int total = 0;
    int generation = 0;
    int maxPoints = 3000000;
    // Display
    bool visible = false;
    bool intensityColors = false;   // false: height
    double pointSize = 2;
    bool overTerrain = false;
    std::array<float, 2> heightRange{-10, 50};
    // Live ring
    std::vector<float> livePts;     // x, y, z, intensity (kLiveCapacity * 4)
    std::vector<float> liveBirth;   // s, nowSeconds()
    int liveHead = 0, liveCount = 0;
    std::uint64_t liveWritten = 0;  // points ever written (the renderer uploads what is newer)
    int liveGeneration = 0;         // clearLive()
    bool liveBody = false;          // the live frame is the body (rotates with the model), else level NED
    double liveTtl = 3;

private:
    PointCloudData();
    double percentile(double q) const;

    std::vector<int> m_histogram;   // 1 m bins from -1000 m
    bool m_histogramDirty = false;
    double m_lastRange = 0;
};

} // namespace corv
