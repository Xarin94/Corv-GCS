// VideoRenderer.h - the FPV camera image in the 3D view, on QRhi (guide, "Video
// FPV": copied planes, converted to RGB in the shader).
//
// CAMERA: the frame alone, opaque, covering the view (nothing else is drawn).
// AR: the 3D view first, on black, and the frame over it with a "screen" blend, so
// that what is black lets the camera through and the route, the symbols and the
// outlines of the terrain stay on top - the mix-blend-mode of the Electron version.
// Render thread only.
#pragma once

#include "scene/VideoFrame.h"

#include <rhi/qrhi.h>
#include <memory>

namespace corv {

class VideoRenderer
{
public:
    explicit VideoRenderer(QRhi *rhi);
    ~VideoRenderer();

    /// The frame to show, uploaded by prepare() when it is a new one; null drops the textures.
    void sync(VideoFramePtr frame, quint64 serial);
    void prepare(const QSize &out, bool screenBlend, QRhiRenderPassDescriptor *rp, int sampleCount, QRhiResourceUpdateBatch *u);
    void draw(QRhiCommandBuffer *cb, const QSize &out);

private:
    bool m_supported = false;
    QRhi *m_rhi;
    QShader m_vs, m_fs;
    VideoFramePtr m_frame;
    quint64 m_serial = 0, m_uploaded = 0;
    QSize m_lumaSize, m_chromaSize;
    bool m_interleaved = false;
    std::unique_ptr<QRhiTexture> m_y, m_u, m_v;
    std::unique_ptr<QRhiSampler> m_sampler;
    std::unique_ptr<QRhiBuffer> m_ubuf, m_tri;
    std::unique_ptr<QRhiShaderResourceBindings> m_srb;
    std::unique_ptr<QRhiGraphicsPipeline> m_pipeline;
    QRhiRenderPassDescriptor *m_rp = nullptr;
    int m_sampleCount = 0;
    bool m_screen = false;
    bool m_triReady = false;
};

} // namespace corv
