#include "scene/PointCloudData.h"

#include <algorithm>
#include <chrono>
#include <cmath>

using namespace Qt::StringLiterals;

namespace corv {

PointCloudData &PointCloudData::instance()
{
    static PointCloudData d;
    return d;
}

double PointCloudData::nowSeconds()
{
    using namespace std::chrono;
    static const steady_clock::time_point t0 = steady_clock::now();
    return duration<double>(steady_clock::now() - t0).count();
}

PointCloudData::PointCloudData() : livePts(std::size_t(kLiveCapacity) * 4, 0.f), liveBirth(std::size_t(kLiveCapacity), -1e9f), m_histogram(4000, 0) {}

void PointCloudData::setOrigin(const PointCloudOrigin &o)
{
    if (hasOrigin && origin.epoch != o.epoch) clear();
    origin = o;
    hasOrigin = true;
}

void PointCloudData::appendMapped(int epoch, const float *enu, const std::uint8_t *intensity, int count)
{
    if (!hasOrigin || !enu || epoch != origin.epoch) return;
    int k = 0;
    bool any = false;
    while (k < count && total < maxPoints) {
        BlockPtr block = chunks.empty() ? nullptr : chunks.back();
        if (!block || block->count >= kChunk) {
            block = std::make_shared<Block>();
            block->id = int(chunks.size());
            block->pts.reset(new float[std::size_t(kChunk) * 4]);
            chunks.push_back(block);
        }
        const int start = block->count;
        const int room = std::min({kChunk - start, count - k, maxPoints - total});
        for (int i = 0; i < room; ++i) {
            const float *src = enu + std::size_t(k + i) * 3;
            float *dst = block->pts.get() + std::size_t(start + i) * 4;
            const float up = src[2];
            dst[0] = src[0];
            dst[1] = up;
            dst[2] = -src[1];
            dst[3] = (intensity ? intensity[k + i] : 128) / 255.f;
            const int bin = int(std::floor(up + 1000));
            if (bin >= 0 && bin < int(m_histogram.size())) ++m_histogram[std::size_t(bin)];
        }
        block->count += room;
        total += room;
        k += room;
        any = any || room > 0;
    }
    if (any) m_histogramDirty = true;
}

void PointCloudData::appendLive(const float *xyz, const std::uint8_t *intensity, int count, const QString &frame, double ttl)
{
    if (!xyz) return;
    if (!frame.isEmpty()) liveBody = frame == "body"_L1;
    if (ttl > 0) liveTtl = ttl;
    const float now = float(nowSeconds());
    for (int k = 0; k < count; ++k) {
        const float *src = xyz + std::size_t(k) * 3;
        float *dst = livePts.data() + std::size_t(liveHead) * 4;
        // NED / FRD -> x east (right), y up, z south (aft)
        dst[0] = src[1];
        dst[1] = -src[2];
        dst[2] = -src[0];
        dst[3] = (intensity ? intensity[k] : 128) / 255.f;
        liveBirth[std::size_t(liveHead)] = now;
        liveHead = (liveHead + 1) % kLiveCapacity;
        liveCount = std::min(kLiveCapacity, liveCount + 1);
        ++liveWritten;
    }
}

void PointCloudData::clearLive()
{
    liveHead = 0;
    liveCount = 0;
    ++liveGeneration;
}

void PointCloudData::clear()
{
    chunks.clear();
    total = 0;
    std::fill(m_histogram.begin(), m_histogram.end(), 0);
    m_histogramDirty = true;
    ++generation;
}

void PointCloudData::updateRange(double nowMs)
{
    if (!m_histogramDirty || nowMs - m_lastRange <= 500) return;
    m_histogramDirty = false;
    m_lastRange = nowMs;
    const double lo = percentile(0.02), hi = percentile(0.98);
    if (hi > lo) {
        heightRange[0] = float(lo);
        heightRange[1] = float(hi);
    }
}

double PointCloudData::percentile(double q) const
{
    if (!total) return 0;
    const double target = q * total;
    double sum = 0;
    for (std::size_t i = 0; i < m_histogram.size(); ++i) {
        sum += m_histogram[i];
        if (sum >= target) return double(i) - 1000;
    }
    return 3000;
}

} // namespace corv
