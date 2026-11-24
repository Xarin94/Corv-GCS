#include "gpu/VideoRenderer.h"

#include "core/DebugLog.h"

#include <QFile>
#include <QMatrix4x4>
#include <cstring>

using namespace Qt::StringLiterals;

namespace corv {

namespace {
QShader loadShader(const QString &name)
{
    QFile f(name);
    if (!f.open(QIODevice::ReadOnly)) {
        LOG_E("gpu") << "shader missing:" << name;
        return {};
    }
    return QShader::fromSerialized(f.readAll());
}

/// A resource the frame's commands may still use: destroyed at the end of the frame.
template <class T>
void retire(std::unique_ptr<T> &p)
{
    if (p) p.release()->deleteLater();
}

struct Ubuf {
    float clipCorr[16];
    float crop[4];
    float mode[4];
};
} // namespace

VideoRenderer::VideoRenderer(QRhi *rhi) : m_rhi(rhi)
{
    m_supported = m_rhi->isTextureFormatSupported(QRhiTexture::R8) && m_rhi->isTextureFormatSupported(QRhiTexture::RG8);
    if (!m_supported) {
        LOG_W("gpu") << "no R8 / RG8 textures: no FPV video";
        return;
    }
    m_vs = loadShader(u":/shaders/video.vert.qsb"_s);
    m_fs = loadShader(u":/shaders/video.frag.qsb"_s);
    m_sampler.reset(m_rhi->newSampler(QRhiSampler::Linear, QRhiSampler::Linear, QRhiSampler::None, QRhiSampler::ClampToEdge,
                                      QRhiSampler::ClampToEdge));
    m_sampler->create();
    m_ubuf.reset(m_rhi->newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer, m_rhi->ubufAligned(sizeof(Ubuf))));
    m_ubuf->create();
    m_tri.reset(m_rhi->newBuffer(QRhiBuffer::Immutable, QRhiBuffer::VertexBuffer, 6 * sizeof(float)));
    m_tri->create();
}

VideoRenderer::~VideoRenderer() = default;

void VideoRenderer::sync(VideoFramePtr frame, quint64 serial)
{
    m_frame = std::move(frame);
    m_serial = serial;
}

void VideoRenderer::prepare(const QSize &out, bool screenBlend, QRhiRenderPassDescriptor *rp, int sampleCount, QRhiResourceUpdateBatch *u)
{
    if (!m_supported) return;
    if (!m_frame) {
        retire(m_pipeline);
        retire(m_srb);
        retire(m_y);
        retire(m_u);
        retire(m_v);
        m_lumaSize = {};
        m_uploaded = 0;
        return;
    }
    const VideoFrame &f = *m_frame;
    const QSize luma(f.width, f.height), chroma(f.chromaWidth, f.chromaHeight);
    if (luma != m_lumaSize || chroma != m_chromaSize || f.interleaved != m_interleaved || !m_y) {
        retire(m_pipeline);
        retire(m_srb);
        retire(m_y);
        retire(m_u);
        retire(m_v);
        m_y.reset(m_rhi->newTexture(QRhiTexture::R8, luma));
        m_u.reset(m_rhi->newTexture(f.interleaved ? QRhiTexture::RG8 : QRhiTexture::R8, chroma));
        m_v.reset(m_rhi->newTexture(QRhiTexture::R8, f.interleaved ? QSize(1, 1) : chroma));
        if (!m_y->create() || !m_u->create() || !m_v->create()) {
            LOG_W("gpu") << "video textures" << luma << chroma << "not created";
            retire(m_y);
            return;
        }
        if (f.interleaved) u->uploadTexture(m_v.get(), QRhiTextureUploadEntry(0, 0, QRhiTextureSubresourceUploadDescription(QByteArray(1, '\x80'))));
        m_lumaSize = luma;
        m_chromaSize = chroma;
        m_interleaved = f.interleaved;
        m_uploaded = 0;
        m_srb.reset(m_rhi->newShaderResourceBindings());
        m_srb->setBindings({
            QRhiShaderResourceBinding::uniformBuffer(0, QRhiShaderResourceBinding::VertexStage | QRhiShaderResourceBinding::FragmentStage, m_ubuf.get()),
            QRhiShaderResourceBinding::sampledTexture(1, QRhiShaderResourceBinding::FragmentStage, m_y.get(), m_sampler.get()),
            QRhiShaderResourceBinding::sampledTexture(2, QRhiShaderResourceBinding::FragmentStage, m_u.get(), m_sampler.get()),
            QRhiShaderResourceBinding::sampledTexture(3, QRhiShaderResourceBinding::FragmentStage, m_v.get(), m_sampler.get()),
        });
        m_srb->create();
    }
    if (m_uploaded != m_serial) {
        auto upload = [u](QRhiTexture *t, const QByteArray &data, int bytesPerRow) {
            QRhiTextureSubresourceUploadDescription d(data);
            d.setDataStride(quint32(bytesPerRow));
            u->uploadTexture(t, QRhiTextureUploadEntry(0, 0, d));
        };
        upload(m_y.get(), f.y, f.width);
        upload(m_u.get(), f.u, f.chromaWidth * (f.interleaved ? 2 : 1));
        if (!f.interleaved) upload(m_v.get(), f.v, f.chromaWidth);
        m_uploaded = m_serial;
    }
    if (!m_triReady) {
        const float tri[6] = {-1, -1, 3, -1, -1, 3};
        u->uploadStaticBuffer(m_tri.get(), tri);
        m_triReady = true;
    }
    if (!m_pipeline || rp != m_rp || sampleCount != m_sampleCount || screenBlend != m_screen) {
        retire(m_pipeline);
        m_pipeline.reset(m_rhi->newGraphicsPipeline());
        m_pipeline->setShaderStages({{QRhiShaderStage::Vertex, m_vs}, {QRhiShaderStage::Fragment, m_fs}});
        QRhiVertexInputLayout layout;
        layout.setBindings({{2 * sizeof(float)}});
        layout.setAttributes({{0, 0, QRhiVertexInputAttribute::Float2, 0}});
        m_pipeline->setVertexInputLayout(layout);
        m_pipeline->setShaderResourceBindings(m_srb.get());
        m_pipeline->setRenderPassDescriptor(rp);
        m_pipeline->setSampleCount(sampleCount);
        m_pipeline->setCullMode(QRhiGraphicsPipeline::None);
        if (screenBlend) {
            // 1 - (1 - scene)(1 - video)
            QRhiGraphicsPipeline::TargetBlend b;
            b.enable = true;
            b.srcColor = QRhiGraphicsPipeline::OneMinusDstColor;
            b.dstColor = QRhiGraphicsPipeline::One;
            b.srcAlpha = QRhiGraphicsPipeline::One;
            b.dstAlpha = QRhiGraphicsPipeline::One;
            m_pipeline->setTargetBlends({b});
        }
        m_pipeline->create();
        m_rp = rp;
        m_sampleCount = sampleCount;
        m_screen = screenBlend;
    }

    Ubuf ub{};
    const QMatrix4x4 corr = m_rhi->clipSpaceCorrMatrix();
    std::memcpy(ub.clipCorr, corr.constData(), sizeof ub.clipCorr);
    // Cover: centre crop, aspect preserved
    const double ia = double(f.width) / f.height, ca = double(std::max(1, out.width())) / std::max(1, out.height());
    ub.crop[0] = ia > ca ? float(ca / ia) : 1.f;
    ub.crop[1] = ia > ca ? 1.f : float(ia / ca);
    ub.mode[0] = f.interleaved;
    ub.mode[1] = f.fullRange;
    ub.mode[2] = f.bt709;
    u->updateDynamicBuffer(m_ubuf.get(), 0, sizeof(Ubuf), &ub);
}

void VideoRenderer::draw(QRhiCommandBuffer *cb, const QSize &out)
{
    if (!m_pipeline || !m_srb || !m_y) return;
    cb->setGraphicsPipeline(m_pipeline.get());
    cb->setViewport({0, 0, float(out.width()), float(out.height())});
    cb->setShaderResources(m_srb.get());
    const QRhiCommandBuffer::VertexInput in{m_tri.get(), 0};
    cb->setVertexInput(0, 1, &in);
    cb->draw(3);
}

} // namespace corv
