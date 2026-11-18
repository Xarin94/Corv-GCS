// RosSurfaceRenderer.h - the averaged ROS surface on QRhi (the drawing part
// of js/ros/RosMesh3D.js and js/ros/RosVolume3D.js).
//
// Height field. Nothing here is per cell on the CPU, per frame:
//   - each block of RosSurfaceData (240 x 240 cells and the first row and
//     column of its neighbours) is one RGBA32F texture: height, level, roughness;
//   - the vertex shader builds each vertex from its index and the texture:
//     there is no vertex buffer, only an index buffer per block listing the
//     triangles whose three corners have a height;
//   - level of detail: far blocks are drawn on every 2nd ... 16th vertex, so a
//     triangle never shrinks below ~6 px; the index is rebuilt when a block's
//     data or detail changes, in view only, within a per-frame budget;
//   - the lines are not geometry: the fragment shader draws the triangle edges
//     from the cell coordinates;
//   - blocks outside the view frustum are skipped.
// Grey lines over a transparent body, drawn through the terrain (the default)
// or depth-tested with a fainter pass where the terrain hides them; or, in a
// colour mode, a solid surface: opaque, depth-tested and depth-writing, after
// a pass that resets the depth under its footprint to the far plane when it is
// drawn through the terrain (it still hides its own far side), before the
// vehicle model so the vehicle stays in front of the bed it floats over.
//
// Volume. One vertex buffer per chunk mesh (position, barycentric
// coordinates for the lines, spread of the normals), the same look.
//
// sync() runs in the item's synchronize() (GUI blocked): the levels of detail,
// the indices to build and the textures to upload are decided there.
#pragma once

#include "core/Geo.h"
#include "scene/RosSurfaceData.h"

#include <QMatrix4x4>
#include <QVector3D>
#include <rhi/qrhi.h>
#include <memory>
#include <unordered_map>
#include <vector>

namespace corv {

class RosSurfaceRenderer
{
public:
    explicit RosSurfaceRenderer(QRhi *rhi);
    ~RosSurfaceRenderer();

    struct Camera {
        QMatrix4x4 viewProj;   // for the frustum (clip-space correction left out)
        QVector3D pos;
        float focalPx = 1000;
    };
    /// vehicle: where the 3D view draws it (world). GUI blocked.
    void sync(RosSurfaceData &d, LatLon sceneOrigin, double offsetAlt, const QVector3D &vehicle, const Camera &cam);
    void prepare(const QMatrix4x4 &viewProj, QRhiRenderPassDescriptor *rp, int sampleCount, QRhiResourceUpdateBatch *u);
    /// The depth reset and the solid surfaces: before the vehicle.
    void drawOpaque(QRhiCommandBuffer *cb, const QSize &out);
    /// The grey lines (ghost first) and the volume's lines.
    void drawTranslucent(QRhiCommandBuffer *cb, const QSize &out);

private:
    struct GpuBlock {
        std::unique_ptr<QRhiTexture> tex;
        std::unique_ptr<QRhiBuffer> ubuf, ubufAlt, index;
        std::unique_ptr<QRhiShaderResourceBindings> srb, srbAlt;
        quint32 indexCap = 0;
        int indices = 0;
        int step = 0;
        int revision = -1;
        bool inView = false;
        QMatrix4x4 model;
        // Pending for render()
        std::vector<float> texData;
        std::vector<std::uint16_t> indexData;
        bool indexPending = false;
    };
    struct GpuChunk {
        std::unique_ptr<QRhiBuffer> vbuf;
        int vertices = 0;
        int revision = -1;
        std::vector<float> data;   // pending upload
    };
    void ensurePipelines(QRhiRenderPassDescriptor *rp, int sampleCount);
    std::unique_ptr<QRhiGraphicsPipeline> pipeline(const QShader &vs, const QShader &fs, const QRhiVertexInputLayout &layout, QRhiShaderResourceBindings *srb,
                                                   bool blend, bool depthTest, bool depthWrite, QRhiGraphicsPipeline::CompareOp op, bool colorWrite);
    void initBlock(GpuBlock &b);
    void fillUbuf(QRhiResourceUpdateBatch *u, QRhiBuffer *buf, const QMatrix4x4 &viewProj, const QMatrix4x4 &model, int step, float alpha, bool mask);

    QRhi *m_rhi;
    QShader m_meshVs, m_meshFs, m_volVs, m_volFs;
    QRhiRenderPassDescriptor *m_rp = nullptr;
    int m_sampleCount = 0;
    std::unique_ptr<QRhiSampler> m_sampler;
    std::unique_ptr<QRhiTexture> m_dummyTex;
    std::unique_ptr<QRhiBuffer> m_dummyUbuf, m_volUbuf;
    std::unique_ptr<QRhiShaderResourceBindings> m_layoutSrb, m_volSrb;
    std::unique_ptr<QRhiGraphicsPipeline> m_linesOver, m_linesDepth, m_ghost, m_solid, m_mask;
    std::unique_ptr<QRhiGraphicsPipeline> m_volLinesOver, m_volLinesDepth, m_volSolid;

    std::unordered_map<std::int64_t, GpuBlock> m_blocks;
    std::unordered_map<std::int64_t, GpuChunk> m_chunks;
    int m_generation = -1, m_volGeneration = -1;

    // From sync()
    bool m_show = false;
    RosSurfaceData::View m_view;
    float m_cell = 0.3f;
    QVector3D m_vehicle, m_cam;
    QMatrix4x4 m_volModel;
    bool m_hasVolume = false;
};

} // namespace corv
