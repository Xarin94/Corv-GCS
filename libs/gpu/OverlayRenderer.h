// OverlayRenderer.h - what the 3D view draws over the terrain (the overlay part
// of js/engine/Scene3D.js, ThickLine.js, TrajectoryCorridor3D.js, Water3D.js
// and the vehicle model of js/main.js), on QRhi.
//
//   lines      screen-space thick segments, widths in pixels: the trail, the
//              predicted corridor borders, home, the mission route, traffic and
//              the symbols (rings of segments around a world point, offset in
//              pixels). Drawn twice: a faint ghost without the depth test, then
//              the line itself depth-tested, so what is behind a ridge still shows.
//   trail      the same segments in a buffer of its own that only grows: a
//              long flight uploads only its new segments every frame.
//   fill       translucent triangles, the corridor ribbon.
//   vehicle    the model (ModelData), lit by the sun.
//   water      the grid on the surface around the vehicle.
//   particles  specks in the water while the camera is under the surface.
// Render thread only.
#pragma once

#include "scene/ModelData.h"

#include <QColor>
#include <QImage>
#include <QMatrix4x4>
#include <QVector3D>
#include <rhi/qrhi.h>
#include <memory>
#include <vector>

namespace corv {

struct LineSeg {
    float a[4] = {};       // world xyz, w unused
    float b[4] = {};
    float off[4] = {};     // pixel offsets of a (xy) and b (zw)
    float color[4] = {};   // rgba
    float style[4] = {};   // width px, ghost opacity (0: none), ghost width px

    static LineSeg make(const QVector3D &a, const QVector3D &b, const QColor &c, float width, float ghostOpacity = 0,
                        float ghostWidth = 0, float alpha = 1);
};

/// A text label: an image of the atlas at a world point, offset and sized in pixels.
struct LabelInst {
    float anchor[4] = {};   // world xyz
    float rect[4] = {};     // x, y offset of the lower-left corner (px, y up), width, height
    float uv[4] = {};       // u0, v0 (top), u1, v1 (bottom)
};

struct FillVertex {
    float p[3];
    float c[4];
};

struct OverlayFrame {
    std::vector<LineSeg> lines;
    std::vector<FillVertex> fill;      // triangles
    std::vector<LabelInst> labels;
    bool vehicle = false;
    QMatrix4x4 model;
    QVector3D sunDir{0.3f, 0.8f, 0.2f};
    // Water surface grid around the vehicle
    bool water = false;
    float waterX = 0, waterZ = 0, waterLevel = 0, waterRadius = 500, waterCell = 25, waterOpacity = 0.6f;
    bool waterTriangles = true;
    QColor waterColor{0x2a, 0x7f, 0xff};
    // Particles while the camera is under a water surface
    bool underwater = false;
    float underwaterLevel = 0;
    QVector3D camPos;
};

class OverlayRenderer
{
public:
    explicit OverlayRenderer(QRhi *rhi);
    ~OverlayRenderer();

    /// The vehicle model (null: none). Uploaded now.
    void setModel(const std::shared_ptr<const ModelData> &model, QRhiResourceUpdateBatch *u);
    /// The label atlas (RGBA8888 premultiplied), uploaded now; its size never changes.
    void setLabelAtlas(const QImage &atlas, QRhiResourceUpdateBatch *u);
    /// The trail: everything again (reset, new origin, downsampled) or new segments at the end.
    void trailReset(const std::vector<LineSeg> &segs, QRhiResourceUpdateBatch *u);
    void trailAppend(const std::vector<LineSeg> &segs, QRhiResourceUpdateBatch *u);

    void prepare(const OverlayFrame &f, const QMatrix4x4 &viewProj, const QSize &out, QRhiRenderPassDescriptor *rp, int sampleCount,
                 QRhiResourceUpdateBatch *u);
    /// The vehicle, before the translucent layers.
    void drawOpaque(QRhiCommandBuffer *cb, const QSize &out);
    /// Water, corridor, particles, ghost lines, lines.
    void drawTranslucent(QRhiCommandBuffer *cb, const QSize &out);

private:
    struct Pass {
        std::unique_ptr<QRhiBuffer> ubuf;
        std::unique_ptr<QRhiShaderResourceBindings> srb;
        std::unique_ptr<QRhiGraphicsPipeline> pipeline;
    };
    void ensurePipelines(QRhiRenderPassDescriptor *rp, int sampleCount);
    void makePass(Pass &p, const QShader &vs, const QShader &fs, const QRhiVertexInputLayout &layout, quint32 ubufSize,
                  QRhiGraphicsPipeline::Topology topo, bool blend, bool depthTest, bool depthWrite, int depthBias = 0);
    bool growDynamic(std::unique_ptr<QRhiBuffer> &buf, quint32 &capacity, quint32 need, QRhiBuffer::UsageFlags usage);
    void drawLines(QRhiCommandBuffer *cb, Pass &p);

    QRhi *m_rhi;
    QShader m_lineVs, m_lineFs, m_meshVs, m_meshFs, m_fillVs, m_fillFs, m_waterVs, m_waterFs, m_partVs, m_partFs;
    QRhiRenderPassDescriptor *m_rp = nullptr;
    int m_sampleCount = 0;

    Pass m_lineMain, m_lineGhost, m_mesh, m_fill, m_water, m_particles, m_label;
    QShader m_labelVs, m_labelFs;
    std::unique_ptr<QRhiTexture> m_atlas;
    std::unique_ptr<QRhiSampler> m_atlasSampler;
    std::unique_ptr<QRhiBuffer> m_labelBuf;
    quint32 m_labelCap = 0, m_labelCount = 0;

    std::unique_ptr<QRhiBuffer> m_corners;       // 4 corners of a segment quad
    std::unique_ptr<QRhiBuffer> m_partCorners;   // 4 corners of a particle quad
    std::unique_ptr<QRhiBuffer> m_lattice;       // particle positions in the unit cube
    bool m_staticReady = false;

    std::unique_ptr<QRhiBuffer> m_lines;
    quint32 m_linesCap = 0, m_lineCount = 0;
    std::unique_ptr<QRhiBuffer> m_trail;
    quint32 m_trailCap = 0, m_trailCount = 0;
    std::vector<LineSeg> m_trailCpu;
    std::unique_ptr<QRhiBuffer> m_fillBuf;
    quint32 m_fillCap = 0, m_fillCount = 0;
    std::unique_ptr<QRhiBuffer> m_waterBuf;
    std::unique_ptr<QRhiBuffer> m_modelBuf;
    int m_modelVertices = 0;

    bool m_vehicle = false, m_waterOn = false, m_underwater = false;
};

} // namespace corv
