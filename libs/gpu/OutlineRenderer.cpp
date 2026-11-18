#include "gpu/OutlineRenderer.h"

#include "core/DebugLog.h"

#include <QColor>
#include <QFile>
#include <algorithm>
#include <cmath>
#include <cstring>

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
    float invViewProj[16];
    float camPos[4];
    float planes[4];
    float widths[4];
    float profileColor[4];
    float skylineColor[4];
    float skyHorizon[4];
    float skyZenith[4];
};

void put(float *dst, const QColor &c)
{
    dst[0] = c.redF();
    dst[1] = c.greenF();
    dst[2] = c.blueF();
    dst[3] = 1;
}
} // namespace

OutlineRenderer::OutlineRenderer(QRhi *rhi) : m_rhi(rhi)
{
    m_supported = m_rhi->isTextureFormatSupported(QRhiTexture::D32F);
    if (!m_supported) {
        LOG_W("gpu") << "no sampleable D32F depth: the schematic view is drawn without outlines";
        return;
    }
    m_vs = loadShader(QStringLiteral(":/shaders/outline.vert.qsb"));
    m_fs = loadShader(QStringLiteral(":/shaders/outline.frag.qsb"));
    m_sampler.reset(m_rhi->newSampler(QRhiSampler::Nearest, QRhiSampler::Nearest, QRhiSampler::None, QRhiSampler::ClampToEdge,
                                      QRhiSampler::ClampToEdge));
    m_sampler->create();
    m_ubuf.reset(m_rhi->newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer, m_rhi->ubufAligned(sizeof(Ubuf))));
    m_ubuf->create();
    m_tri.reset(m_rhi->newBuffer(QRhiBuffer::Immutable, QRhiBuffer::VertexBuffer, 6 * sizeof(float)));
    m_tri->create();
}

OutlineRenderer::~OutlineRenderer() = default;

QRhiTextureRenderTarget *OutlineRenderer::target(const QSize &sizeIn)
{
    const QSize size(std::max(1, sizeIn.width()), std::max(1, sizeIn.height()));
    if (m_rt && size == m_size) return m_rt.get();
    m_size = size;
    retire(m_color);
    retire(m_depth);
    m_color.reset(m_rhi->newTexture(QRhiTexture::RGBA8, size, 1, QRhiTexture::RenderTarget));
    m_color->create();
    m_depth.reset(m_rhi->newTexture(QRhiTexture::D32F, size, 1, QRhiTexture::RenderTarget));
    m_depth->create();
    QRhiTextureRenderTargetDescription desc{QRhiColorAttachment(m_color.get())};
    desc.setDepthTexture(m_depth.get());
    if (!m_rt) {
        m_rt.reset(m_rhi->newTextureRenderTarget(desc));
        m_rtPass.reset(m_rt->newCompatibleRenderPassDescriptor());
        m_rt->setRenderPassDescriptor(m_rtPass.get());
    } else {
        m_rt->setDescription(desc);
    }
    m_rt->create();
    makeBindings();
    return m_rt.get();
}

void OutlineRenderer::makeBindings()
{
    retire(m_pipeline);   // made again with the new binding set
    retire(m_srb);
    m_srb.reset(m_rhi->newShaderResourceBindings());
    m_srb->setBindings({
        QRhiShaderResourceBinding::uniformBuffer(0, QRhiShaderResourceBinding::FragmentStage, m_ubuf.get()),
        QRhiShaderResourceBinding::sampledTexture(1, QRhiShaderResourceBinding::FragmentStage, m_color.get(), m_sampler.get()),
        QRhiShaderResourceBinding::sampledTexture(2, QRhiShaderResourceBinding::FragmentStage, m_depth.get(), m_sampler.get()),
    });
    m_srb->create();
}

void OutlineRenderer::prepare(const Params &p, QRhiRenderPassDescriptor *rp, int sampleCount, QRhiResourceUpdateBatch *u)
{
    if (!m_supported || !m_srb) return;
    if (!m_triReady) {
        const float tri[6] = {-1, -1, 3, -1, -1, 3};
        u->uploadStaticBuffer(m_tri.get(), tri);
        m_triReady = true;
    }
    if (!m_pipeline || rp != m_rp || sampleCount != m_sampleCount) {
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
        // The depth is written whatever is there: the test passes always
        m_pipeline->setDepthTest(true);
        m_pipeline->setDepthOp(QRhiGraphicsPipeline::Always);
        m_pipeline->setDepthWrite(true);
        m_pipeline->create();
        m_rp = rp;
        m_sampleCount = sampleCount;
    }

    Ubuf ub{};
    const QMatrix4x4 inv = p.viewProj.inverted();
    std::memcpy(ub.invViewProj, inv.constData(), sizeof(ub.invViewProj));
    ub.camPos[0] = p.camPos.x(); ub.camPos[1] = p.camPos.y(); ub.camPos[2] = p.camPos.z(); ub.camPos[3] = p.fogDensity;
    ub.planes[0] = p.nearPlane;
    ub.planes[1] = p.farPlane;
    // Line widths in device pixels
    ub.widths[0] = float(std::max(1.0, std::round(1.3 * p.dpr)));
    ub.widths[1] = float(std::max(2.0, std::round(2.4 * p.dpr)));
    ub.widths[2] = std::clamp(p.brightness, 0.3f, 1.6f) / 0.85f;
    if (p.light) {
        put(ub.profileColor, QColor::fromRgbF(0.12f, 0.24f, 0.17f));
        put(ub.skylineColor, QColor::fromRgbF(0.05f, 0.19f, 0.29f));
        put(ub.skyHorizon, QColor(0xe2eef7));
        put(ub.skyZenith, QColor(0x86bbe8));
    } else {
        put(ub.profileColor, QColor::fromRgbF(0.86f, 0.90f, 0.93f));
        put(ub.skylineColor, QColor::fromRgbF(0.97f, 0.98f, 1.00f));
        put(ub.skyHorizon, QColor(0, 0, 0));
        put(ub.skyZenith, QColor(0, 0, 0));
    }
    u->updateDynamicBuffer(m_ubuf.get(), 0, sizeof(Ubuf), &ub);
}

void OutlineRenderer::draw(QRhiCommandBuffer *cb, const QSize &out)
{
    if (!m_pipeline || !m_srb) return;
    cb->setGraphicsPipeline(m_pipeline.get());
    cb->setViewport({0, 0, float(out.width()), float(out.height())});
    cb->setShaderResources(m_srb.get());
    const QRhiCommandBuffer::VertexInput in{m_tri.get(), 0};
    cb->setVertexInput(0, 1, &in);
    cb->draw(3);
}

} // namespace corv
