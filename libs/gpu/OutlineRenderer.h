// OutlineRenderer.h - the outlines of the schematic 3D view (renderSchematic of
// js/engine/Scene3D.js), on QRhi.
//
//   1. the terrain is drawn into a target of its own: colour (RGBA8) and depth
//      (D32F, sampled), one sample;
//   2. a full-screen pass into the view's target draws that colour with a line
//      on every ridge that hides the ground behind it, a brighter one where the
//      ground meets the sky, fills the one-pixel cracks between chunks of
//      different detail, and writes the depth back (none on a water surface);
//   3. the overlays (trail, route, symbols, the vehicle) follow in the same
//      pass, depth-tested against that ground and multisampled. They are not
//      outlined themselves; the vehicle is not either, unlike the Electron
//      version, where it was part of the world layer.
//
// Without a sampleable D32F format the view is drawn as before, without lines.
// Render thread only.
#pragma once

#include <QMatrix4x4>
#include <QVector3D>
#include <rhi/qrhi.h>
#include <memory>

namespace corv {

class OutlineRenderer
{
public:
    explicit OutlineRenderer(QRhi *rhi);
    ~OutlineRenderer();

    bool supported() const { return m_supported; }
    /// The target the terrain is drawn into, at the view's size (made again when it changes).
    QRhiTextureRenderTarget *target(const QSize &size);

    struct Params {
        QMatrix4x4 viewProj;
        QVector3D camPos;
        float fogDensity = 0;
        float nearPlane = 1, farPlane = 300000;
        float brightness = 0.85f;    // MAP BRIGHTNESS, 0.85 draws the lines as designed
        bool light = false;          // light UI theme: dark lines on green ground, a pale blue sky
        float dpr = 1;
    };
    void prepare(const Params &p, QRhiRenderPassDescriptor *rp, int sampleCount, QRhiResourceUpdateBatch *u);
    /// Pass 2, first thing in the view's pass.
    void draw(QRhiCommandBuffer *cb, const QSize &out);

private:
    void makeBindings();

    QRhi *m_rhi;
    bool m_supported = false;
    QShader m_vs, m_fs;
    QSize m_size;
    std::unique_ptr<QRhiTexture> m_color, m_depth;
    std::unique_ptr<QRhiTextureRenderTarget> m_rt;
    std::unique_ptr<QRhiRenderPassDescriptor> m_rtPass;
    std::unique_ptr<QRhiSampler> m_sampler;
    std::unique_ptr<QRhiBuffer> m_ubuf, m_tri;
    std::unique_ptr<QRhiShaderResourceBindings> m_srb;
    std::unique_ptr<QRhiGraphicsPipeline> m_pipeline;
    QRhiRenderPassDescriptor *m_rp = nullptr;
    int m_sampleCount = 0;
    bool m_triReady = false;
};

} // namespace corv
