// PointCloudRenderer.h - the LiDAR point cloud on QRhi (port of
// js/engine/three/PointCloudRenderer.js).
//
// The map: one vertex buffer per block of PointCloudData (262 144 points),
// uploaded only where the block grew; a new generation (CLEAR MAP, a new
// origin) drops them all. The live scan: one ring buffer with the arrival
// time of each point, uploaded where it was written; the shader clips points
// past their TTL and fades them over its last 40 %. Each point is a quad of a
// few pixels with a round mask (the point size is not available on every
// backend). Over the terrain (the default) the cloud is drawn without the
// depth test: SRTM is coarser than the LiDAR.
//
// sync() runs in the item's synchronize() (GUI blocked) and keeps what
// render() uploads: references to the map blocks, copies of the live slots.
#pragma once

#include "scene/PointCloudData.h"

#include <QMatrix4x4>
#include <rhi/qrhi.h>
#include <memory>
#include <vector>

namespace corv {

class PointCloudRenderer
{
public:
    explicit PointCloudRenderer(QRhi *rhi);
    ~PointCloudRenderer();

    /// mapModel: cloud origin -> world; liveModel: vehicle -> world. GUI blocked.
    void sync(const PointCloudData &d, const QMatrix4x4 &mapModel, const QMatrix4x4 &liveModel, float dpr);
    void prepare(const QMatrix4x4 &viewProj, const QSize &out, QRhiRenderPassDescriptor *rp, int sampleCount, QRhiResourceUpdateBatch *u);
    void draw(QRhiCommandBuffer *cb, const QSize &out);
    bool visible() const { return m_visible; }

private:
    struct Pass {
        std::unique_ptr<QRhiBuffer> ubuf;
        std::unique_ptr<QRhiShaderResourceBindings> srb;
    };
    struct GpuBlock {
        std::unique_ptr<QRhiBuffer> buf;
        int uploaded = 0;
    };
    struct BlockUpload {
        std::size_t index;
        PointCloudData::BlockPtr block;
        int from, to;
    };
    struct LiveUpload {
        int slot;
        std::vector<float> pts, birth;
    };
    void ensurePipelines(QRhiRenderPassDescriptor *rp, int sampleCount);
    std::unique_ptr<QRhiGraphicsPipeline> makePipeline(const QShader &vs, const QRhiVertexInputLayout &layout, QRhiShaderResourceBindings *srb,
                                                       bool depth, bool depthWrite);

    QRhi *m_rhi;
    QShader m_mapVs, m_liveVs, m_fs;
    QRhiRenderPassDescriptor *m_rp = nullptr;
    int m_sampleCount = 0;
    Pass m_map, m_live;
    std::unique_ptr<QRhiGraphicsPipeline> m_mapDepth, m_mapOver, m_liveDepth, m_liveOver;
    std::unique_ptr<QRhiBuffer> m_corners;
    bool m_cornersReady = false;
    std::vector<GpuBlock> m_blocks;
    std::unique_ptr<QRhiBuffer> m_livePts, m_liveBirth;

    // From sync()
    bool m_visible = false;
    int m_generation = -1;
    std::vector<int> m_blockCounts;
    std::vector<BlockUpload> m_blockUploads;
    bool m_dropBlocks = false;
    std::vector<LiveUpload> m_liveUploads;
    std::uint64_t m_liveWritten = 0;
    int m_liveGeneration = -1;
    int m_liveCount = 0;
    QMatrix4x4 m_mapModel, m_liveModel;
    float m_size = 2, m_mode = 0, m_hmin = -10, m_hmax = 50, m_ttl = 3, m_now = 0;
    bool m_overTerrain = false;
};

} // namespace corv
