#include "gpu/OverlayRenderer.h"

#include "core/DebugLog.h"

#include <QFile>
#include <cstring>
#include <random>

using namespace Qt::StringLiterals;

namespace corv {

namespace {
constexpr int kParticleCount = 2500;
constexpr float kParticleBoxM = 60;   // particles fill a cube this wide around the camera

QShader loadShader(const QString &name)
{
    QFile f(name);
    if (!f.open(QIODevice::ReadOnly)) {
        LOG_E("gpu") << "shader missing:" << name;
        return {};
    }
    return QShader::fromSerialized(f.readAll());
}

template <class T>
void retire(std::unique_ptr<T> &p)
{
    if (p) p.release()->deleteLater();
}

struct LineUbuf {
    float viewProj[16];
    float viewport[4];
};
struct MeshUbuf {
    float mvp[16];
    float model[16];
    float light[4];
    float params[4];
};
struct FillUbuf {
    float viewProj[16];
};
struct WaterUbuf {
    float viewProj[16];
    float plane[4];
    float color[4];
    float style[4];
};
struct LabelUbuf {
    float viewProj[16];
    float viewport[4];
};
struct PartUbuf {
    float viewProj[16];
    float cam[4];
    float params[4];
    float color[4];
};
} // namespace

LineSeg LineSeg::make(const QVector3D &a, const QVector3D &b, const QColor &c, float width, float ghostOpacity, float ghostWidth, float alpha)
{
    LineSeg s;
    s.a[0] = a.x(); s.a[1] = a.y(); s.a[2] = a.z();
    s.b[0] = b.x(); s.b[1] = b.y(); s.b[2] = b.z();
    s.color[0] = float(c.redF()); s.color[1] = float(c.greenF()); s.color[2] = float(c.blueF()); s.color[3] = float(c.alphaF()) * alpha;
    s.style[0] = width;
    s.style[1] = ghostOpacity;
    s.style[2] = ghostWidth;
    return s;
}

OverlayRenderer::OverlayRenderer(QRhi *rhi) : m_rhi(rhi)
{
    m_lineVs = loadShader(u":/shaders/line.vert.qsb"_s);
    m_lineFs = loadShader(u":/shaders/line.frag.qsb"_s);
    m_meshVs = loadShader(u":/shaders/mesh.vert.qsb"_s);
    m_meshFs = loadShader(u":/shaders/mesh.frag.qsb"_s);
    m_fillVs = loadShader(u":/shaders/fill.vert.qsb"_s);
    m_fillFs = loadShader(u":/shaders/fill.frag.qsb"_s);
    m_waterVs = loadShader(u":/shaders/water.vert.qsb"_s);
    m_waterFs = loadShader(u":/shaders/water.frag.qsb"_s);
    m_partVs = loadShader(u":/shaders/particles.vert.qsb"_s);
    m_partFs = loadShader(u":/shaders/particles.frag.qsb"_s);
    m_labelVs = loadShader(u":/shaders/label.vert.qsb"_s);
    m_labelFs = loadShader(u":/shaders/label.frag.qsb"_s);
    m_atlas.reset(m_rhi->newTexture(QRhiTexture::RGBA8, QSize(1024, 512)));
    m_atlas->create();
    m_atlasSampler.reset(m_rhi->newSampler(QRhiSampler::Linear, QRhiSampler::Linear, QRhiSampler::None,
                                           QRhiSampler::ClampToEdge, QRhiSampler::ClampToEdge));
    m_atlasSampler->create();

    m_corners.reset(m_rhi->newBuffer(QRhiBuffer::Immutable, QRhiBuffer::VertexBuffer, 8 * sizeof(float)));
    m_corners->create();
    m_partCorners.reset(m_rhi->newBuffer(QRhiBuffer::Immutable, QRhiBuffer::VertexBuffer, 8 * sizeof(float)));
    m_partCorners->create();
    m_lattice.reset(m_rhi->newBuffer(QRhiBuffer::Immutable, QRhiBuffer::VertexBuffer, kParticleCount * 3 * sizeof(float)));
    m_lattice->create();
    m_waterBuf.reset(m_rhi->newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::VertexBuffer, 6 * 3 * sizeof(float)));
    m_waterBuf->create();
}

OverlayRenderer::~OverlayRenderer() = default;

bool OverlayRenderer::growDynamic(std::unique_ptr<QRhiBuffer> &buf, quint32 &capacity, quint32 need, QRhiBuffer::UsageFlags usage)
{
    if (buf && need <= capacity) return false;
    capacity = std::max<quint32>(need * 2, 4096);
    retire(buf);
    buf.reset(m_rhi->newBuffer(QRhiBuffer::Dynamic, usage, capacity));
    buf->create();
    return true;
}

void OverlayRenderer::setModel(const std::shared_ptr<const ModelData> &model, QRhiResourceUpdateBatch *u)
{
    retire(m_modelBuf);
    m_modelVertices = 0;
    if (!model || model->vertices.empty()) return;
    const quint32 bytes = quint32(model->vertices.size() * sizeof(float));
    m_modelBuf.reset(m_rhi->newBuffer(QRhiBuffer::Immutable, QRhiBuffer::VertexBuffer, bytes));
    m_modelBuf->create();
    u->uploadStaticBuffer(m_modelBuf.get(), model->vertices.data());
    m_modelVertices = model->vertexCount();
}

void OverlayRenderer::setLabelAtlas(const QImage &atlas, QRhiResourceUpdateBatch *u)
{
    if (atlas.isNull()) return;
    QImage img = atlas.convertToFormat(QImage::Format_RGBA8888_Premultiplied);
    if (img.size() != m_atlas->pixelSize()) img = img.copy(0, 0, m_atlas->pixelSize().width(), m_atlas->pixelSize().height());
    u->uploadTexture(m_atlas.get(), img);
}

void OverlayRenderer::trailReset(const std::vector<LineSeg> &segs, QRhiResourceUpdateBatch *u)
{
    m_trailCpu = segs;
    m_trailCount = 0;
    if (segs.empty()) return;
    const quint32 need = quint32(segs.size());
    if (!m_trail || need > m_trailCap) {
        m_trailCap = std::max<quint32>(need * 2, 4096);
        retire(m_trail);
        m_trail.reset(m_rhi->newBuffer(QRhiBuffer::Static, QRhiBuffer::VertexBuffer, m_trailCap * sizeof(LineSeg)));
        m_trail->create();
    }
    u->uploadStaticBuffer(m_trail.get(), 0, need * sizeof(LineSeg), segs.data());
    m_trailCount = need;
}

void OverlayRenderer::trailAppend(const std::vector<LineSeg> &segs, QRhiResourceUpdateBatch *u)
{
    if (segs.empty()) return;
    m_trailCpu.insert(m_trailCpu.end(), segs.begin(), segs.end());
    if (!m_trail || m_trailCpu.size() > m_trailCap) {
        // Grown: everything again into a buffer twice the size
        const std::vector<LineSeg> all = m_trailCpu;
        trailReset(all, u);
        return;
    }
    u->uploadStaticBuffer(m_trail.get(), m_trailCount * sizeof(LineSeg), quint32(segs.size() * sizeof(LineSeg)), segs.data());
    m_trailCount = quint32(m_trailCpu.size());
}

void OverlayRenderer::makePass(Pass &p, const QShader &vs, const QShader &fs, const QRhiVertexInputLayout &layout, quint32 ubufSize,
                               QRhiGraphicsPipeline::Topology topo, bool blend, bool depthTest, bool depthWrite, int depthBias)
{
    if (!p.ubuf) {
        p.ubuf.reset(m_rhi->newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer, m_rhi->ubufAligned(ubufSize)));
        p.ubuf->create();
        p.srb.reset(m_rhi->newShaderResourceBindings());
        p.srb->setBindings({QRhiShaderResourceBinding::uniformBuffer(
            0, QRhiShaderResourceBinding::VertexStage | QRhiShaderResourceBinding::FragmentStage, p.ubuf.get())});
        p.srb->create();
    }
    retire(p.pipeline);
    p.pipeline.reset(m_rhi->newGraphicsPipeline());
    p.pipeline->setShaderStages({{QRhiShaderStage::Vertex, vs}, {QRhiShaderStage::Fragment, fs}});
    p.pipeline->setVertexInputLayout(layout);
    p.pipeline->setShaderResourceBindings(p.srb.get());
    p.pipeline->setRenderPassDescriptor(m_rp);
    p.pipeline->setSampleCount(m_sampleCount);
    p.pipeline->setTopology(topo);
    p.pipeline->setCullMode(QRhiGraphicsPipeline::None);
    p.pipeline->setDepthTest(depthTest);
    p.pipeline->setDepthWrite(depthWrite);
    p.pipeline->setDepthOp(QRhiGraphicsPipeline::LessOrEqual);
    if (depthBias) {
        p.pipeline->setDepthBias(depthBias);
        p.pipeline->setSlopeScaledDepthBias(-1.0f);
    }
    if (blend) {
        QRhiGraphicsPipeline::TargetBlend b;
        b.enable = true;
        b.srcColor = QRhiGraphicsPipeline::One;            // premultiplied in the shaders
        b.dstColor = QRhiGraphicsPipeline::OneMinusSrcAlpha;
        b.srcAlpha = QRhiGraphicsPipeline::One;
        b.dstAlpha = QRhiGraphicsPipeline::OneMinusSrcAlpha;
        p.pipeline->setTargetBlends({b});
    }
    p.pipeline->create();
}

void OverlayRenderer::ensurePipelines(QRhiRenderPassDescriptor *rp, int sampleCount)
{
    if (m_lineMain.pipeline && rp == m_rp && sampleCount == m_sampleCount) return;
    m_rp = rp;
    m_sampleCount = sampleCount;

    QRhiVertexInputLayout lines;
    lines.setBindings({{2 * sizeof(float)}, {sizeof(LineSeg), QRhiVertexInputBinding::PerInstance}});
    lines.setAttributes({{0, 0, QRhiVertexInputAttribute::Float2, 0},
                         {1, 1, QRhiVertexInputAttribute::Float4, 0},
                         {1, 2, QRhiVertexInputAttribute::Float4, 16},
                         {1, 3, QRhiVertexInputAttribute::Float4, 32},
                         {1, 4, QRhiVertexInputAttribute::Float4, 48},
                         {1, 5, QRhiVertexInputAttribute::Float4, 64}});
    makePass(m_lineGhost, m_lineVs, m_lineFs, lines, sizeof(LineUbuf), QRhiGraphicsPipeline::TriangleStrip, true, false, false);
    makePass(m_lineMain, m_lineVs, m_lineFs, lines, sizeof(LineUbuf), QRhiGraphicsPipeline::TriangleStrip, true, true, false);

    QRhiVertexInputLayout mesh;
    mesh.setBindings({{9 * sizeof(float)}});
    mesh.setAttributes({{0, 0, QRhiVertexInputAttribute::Float3, 0},
                        {0, 1, QRhiVertexInputAttribute::Float3, 3 * sizeof(float)},
                        {0, 2, QRhiVertexInputAttribute::Float3, 6 * sizeof(float)}});
    makePass(m_mesh, m_meshVs, m_meshFs, mesh, sizeof(MeshUbuf), QRhiGraphicsPipeline::Triangles, false, true, true);

    QRhiVertexInputLayout fill;
    fill.setBindings({{sizeof(FillVertex)}});
    fill.setAttributes({{0, 0, QRhiVertexInputAttribute::Float3, 0}, {0, 1, QRhiVertexInputAttribute::Float4, 3 * sizeof(float)}});
    makePass(m_fill, m_fillVs, m_fillFs, fill, sizeof(FillUbuf), QRhiGraphicsPipeline::Triangles, true, true, false);

    QRhiVertexInputLayout water;
    water.setBindings({{3 * sizeof(float)}});
    water.setAttributes({{0, 0, QRhiVertexInputAttribute::Float3, 0}});
    // Over a lake the plane lies on the terrain's own surface: pulled forward
    makePass(m_water, m_waterVs, m_waterFs, water, sizeof(WaterUbuf), QRhiGraphicsPipeline::Triangles, true, true, false, -4);

    {
        QRhiVertexInputLayout labels;
        labels.setBindings({{2 * sizeof(float)}, {sizeof(LabelInst), QRhiVertexInputBinding::PerInstance}});
        labels.setAttributes({{0, 0, QRhiVertexInputAttribute::Float2, 0},
                              {1, 1, QRhiVertexInputAttribute::Float4, 0},
                              {1, 2, QRhiVertexInputAttribute::Float4, 16},
                              {1, 3, QRhiVertexInputAttribute::Float4, 32}});
        if (!m_label.ubuf) {
            m_label.ubuf.reset(m_rhi->newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer, m_rhi->ubufAligned(sizeof(LabelUbuf))));
            m_label.ubuf->create();
            m_label.srb.reset(m_rhi->newShaderResourceBindings());
            m_label.srb->setBindings({QRhiShaderResourceBinding::uniformBuffer(0, QRhiShaderResourceBinding::VertexStage | QRhiShaderResourceBinding::FragmentStage, m_label.ubuf.get()),
                                      QRhiShaderResourceBinding::sampledTexture(1, QRhiShaderResourceBinding::FragmentStage, m_atlas.get(), m_atlasSampler.get())});
            m_label.srb->create();
        }
        retire(m_label.pipeline);
        m_label.pipeline.reset(m_rhi->newGraphicsPipeline());
        m_label.pipeline->setShaderStages({{QRhiShaderStage::Vertex, m_labelVs}, {QRhiShaderStage::Fragment, m_labelFs}});
        m_label.pipeline->setVertexInputLayout(labels);
        m_label.pipeline->setShaderResourceBindings(m_label.srb.get());
        m_label.pipeline->setRenderPassDescriptor(m_rp);
        m_label.pipeline->setSampleCount(m_sampleCount);
        m_label.pipeline->setTopology(QRhiGraphicsPipeline::TriangleStrip);
        m_label.pipeline->setCullMode(QRhiGraphicsPipeline::None);
        QRhiGraphicsPipeline::TargetBlend b;
        b.enable = true;
        b.srcColor = QRhiGraphicsPipeline::One;
        b.dstColor = QRhiGraphicsPipeline::OneMinusSrcAlpha;
        b.srcAlpha = QRhiGraphicsPipeline::One;
        b.dstAlpha = QRhiGraphicsPipeline::OneMinusSrcAlpha;
        m_label.pipeline->setTargetBlends({b});
        m_label.pipeline->create();
    }

    QRhiVertexInputLayout part;
    part.setBindings({{2 * sizeof(float)}, {3 * sizeof(float), QRhiVertexInputBinding::PerInstance}});
    part.setAttributes({{0, 0, QRhiVertexInputAttribute::Float2, 0}, {1, 1, QRhiVertexInputAttribute::Float3, 0}});
    makePass(m_particles, m_partVs, m_partFs, part, sizeof(PartUbuf), QRhiGraphicsPipeline::TriangleStrip, true, true, false);
}

void OverlayRenderer::prepare(const OverlayFrame &f, const QMatrix4x4 &viewProj, const QSize &out, QRhiRenderPassDescriptor *rp,
                              int sampleCount, QRhiResourceUpdateBatch *u)
{
    if (!m_staticReady) {
        const float corners[8] = {0, -1, 0, 1, 1, -1, 1, 1};
        u->uploadStaticBuffer(m_corners.get(), corners);
        const float pc[8] = {-1, -1, -1, 1, 1, -1, 1, 1};
        u->uploadStaticBuffer(m_partCorners.get(), pc);
        std::mt19937 rng(7);
        std::uniform_real_distribution<float> d(0.f, 1.f);
        std::vector<float> lattice(kParticleCount * 3);
        for (float &v : lattice) v = d(rng);
        u->uploadStaticBuffer(m_lattice.get(), lattice.data());
        m_staticReady = true;
    }
    ensurePipelines(rp, sampleCount);

    // Lines: both passes share the instances, the uniform tells them apart
    LineUbuf lu{};
    std::memcpy(lu.viewProj, viewProj.constData(), sizeof(lu.viewProj));
    lu.viewport[0] = float(out.width());
    lu.viewport[1] = float(out.height());
    lu.viewport[2] = 0;
    lu.viewport[3] = 1;
    u->updateDynamicBuffer(m_lineMain.ubuf.get(), 0, sizeof(LineUbuf), &lu);
    lu.viewport[2] = 1;
    u->updateDynamicBuffer(m_lineGhost.ubuf.get(), 0, sizeof(LineUbuf), &lu);
    m_lineCount = quint32(f.lines.size());
    if (m_lineCount) {
        growDynamic(m_lines, m_linesCap, m_lineCount * sizeof(LineSeg), QRhiBuffer::VertexBuffer);
        u->updateDynamicBuffer(m_lines.get(), 0, m_lineCount * sizeof(LineSeg), f.lines.data());
    }

    m_labelCount = quint32(f.labels.size());
    if (m_labelCount) {
        LabelUbuf lb{};
        std::memcpy(lb.viewProj, viewProj.constData(), sizeof(lb.viewProj));
        lb.viewport[0] = float(out.width());
        lb.viewport[1] = float(out.height());
        lb.viewport[2] = 1;
        lb.viewport[3] = m_rhi->isYUpInNDC() ? 1.f : -1.f;
        u->updateDynamicBuffer(m_label.ubuf.get(), 0, sizeof(LabelUbuf), &lb);
        growDynamic(m_labelBuf, m_labelCap, m_labelCount * sizeof(LabelInst), QRhiBuffer::VertexBuffer);
        u->updateDynamicBuffer(m_labelBuf.get(), 0, m_labelCount * sizeof(LabelInst), f.labels.data());
    }

    m_fillCount = quint32(f.fill.size());
    if (m_fillCount) {
        FillUbuf fu{};
        std::memcpy(fu.viewProj, viewProj.constData(), sizeof(fu.viewProj));
        u->updateDynamicBuffer(m_fill.ubuf.get(), 0, sizeof(FillUbuf), &fu);
        growDynamic(m_fillBuf, m_fillCap, m_fillCount * sizeof(FillVertex), QRhiBuffer::VertexBuffer);
        u->updateDynamicBuffer(m_fillBuf.get(), 0, m_fillCount * sizeof(FillVertex), f.fill.data());
    }

    m_vehicle = f.vehicle && m_modelBuf && m_modelVertices > 0;
    if (m_vehicle) {
        MeshUbuf mu{};
        const QMatrix4x4 mvp = viewProj * f.model;
        std::memcpy(mu.mvp, mvp.constData(), sizeof(mu.mvp));
        std::memcpy(mu.model, f.model.constData(), sizeof(mu.model));
        const QVector3D l = f.sunDir.normalized();
        mu.light[0] = l.x(); mu.light[1] = l.y(); mu.light[2] = l.z();
        mu.params[0] = 0.5f;
        mu.params[1] = 0.6f;
        u->updateDynamicBuffer(m_mesh.ubuf.get(), 0, sizeof(MeshUbuf), &mu);
    }

    m_waterOn = f.water;
    if (m_waterOn) {
        const float x0 = f.waterX - f.waterRadius, x1 = f.waterX + f.waterRadius;
        const float z0 = f.waterZ - f.waterRadius, z1 = f.waterZ + f.waterRadius, y = f.waterLevel;
        const float quad[18] = {x0, y, z0, x1, y, z0, x1, y, z1, x0, y, z0, x1, y, z1, x0, y, z1};
        u->updateDynamicBuffer(m_waterBuf.get(), 0, sizeof(quad), quad);
        WaterUbuf wu{};
        std::memcpy(wu.viewProj, viewProj.constData(), sizeof(wu.viewProj));
        wu.plane[0] = f.waterX; wu.plane[1] = f.waterZ; wu.plane[2] = f.waterRadius; wu.plane[3] = f.waterCell;
        wu.color[0] = float(f.waterColor.redF()); wu.color[1] = float(f.waterColor.greenF()); wu.color[2] = float(f.waterColor.blueF());
        wu.color[3] = f.waterOpacity;
        wu.style[0] = f.waterTriangles ? 1.f : 0.f;
        u->updateDynamicBuffer(m_water.ubuf.get(), 0, sizeof(WaterUbuf), &wu);
    }

    m_underwater = f.underwater;
    if (m_underwater) {
        PartUbuf pu{};
        std::memcpy(pu.viewProj, viewProj.constData(), sizeof(pu.viewProj));
        pu.cam[0] = f.camPos.x(); pu.cam[1] = f.camPos.y(); pu.cam[2] = f.camPos.z(); pu.cam[3] = kParticleBoxM;
        pu.params[0] = f.underwaterLevel;
        pu.params[1] = float(out.width());
        pu.params[2] = float(out.height());
        pu.color[0] = 0xa8 / 255.f; pu.color[1] = 0xd8 / 255.f; pu.color[2] = 1.f; pu.color[3] = 1.f;
        u->updateDynamicBuffer(m_particles.ubuf.get(), 0, sizeof(PartUbuf), &pu);
    }
}

void OverlayRenderer::drawOpaque(QRhiCommandBuffer *cb, const QSize &out)
{
    if (!m_vehicle) return;
    cb->setGraphicsPipeline(m_mesh.pipeline.get());
    cb->setViewport({0, 0, float(out.width()), float(out.height())});
    cb->setShaderResources(m_mesh.srb.get());
    const QRhiCommandBuffer::VertexInput in(m_modelBuf.get(), 0);
    cb->setVertexInput(0, 1, &in);
    cb->draw(quint32(m_modelVertices));
}

void OverlayRenderer::drawLines(QRhiCommandBuffer *cb, Pass &p)
{
    cb->setGraphicsPipeline(p.pipeline.get());
    cb->setShaderResources(p.srb.get());
    if (m_trail && m_trailCount) {
        const QRhiCommandBuffer::VertexInput in[] = {{m_corners.get(), 0}, {m_trail.get(), 0}};
        cb->setVertexInput(0, 2, in);
        cb->draw(4, m_trailCount);
    }
    if (m_lines && m_lineCount) {
        const QRhiCommandBuffer::VertexInput in[] = {{m_corners.get(), 0}, {m_lines.get(), 0}};
        cb->setVertexInput(0, 2, in);
        cb->draw(4, m_lineCount);
    }
}

void OverlayRenderer::drawTranslucent(QRhiCommandBuffer *cb, const QSize &out)
{
    if (!m_lineMain.pipeline) return;
    cb->setViewport({0, 0, float(out.width()), float(out.height())});
    if (m_waterOn) {
        cb->setGraphicsPipeline(m_water.pipeline.get());
        cb->setShaderResources(m_water.srb.get());
        const QRhiCommandBuffer::VertexInput in(m_waterBuf.get(), 0);
        cb->setVertexInput(0, 1, &in);
        cb->draw(6);
    }
    if (m_fillCount && m_fillBuf) {
        cb->setGraphicsPipeline(m_fill.pipeline.get());
        cb->setShaderResources(m_fill.srb.get());
        const QRhiCommandBuffer::VertexInput in(m_fillBuf.get(), 0);
        cb->setVertexInput(0, 1, &in);
        cb->draw(m_fillCount);
    }
    if (m_underwater) {
        cb->setGraphicsPipeline(m_particles.pipeline.get());
        cb->setShaderResources(m_particles.srb.get());
        const QRhiCommandBuffer::VertexInput in[] = {{m_partCorners.get(), 0}, {m_lattice.get(), 0}};
        cb->setVertexInput(0, 2, in);
        cb->draw(4, kParticleCount);
    }
    if ((m_trail && m_trailCount) || m_lineCount) {
        drawLines(cb, m_lineGhost);
        drawLines(cb, m_lineMain);
    }
    // Labels last, over everything
    if (m_labelCount && m_labelBuf) {
        cb->setGraphicsPipeline(m_label.pipeline.get());
        cb->setShaderResources(m_label.srb.get());
        const QRhiCommandBuffer::VertexInput in[] = {{m_corners.get(), 0}, {m_labelBuf.get(), 0}};
        cb->setVertexInput(0, 2, in);
        cb->draw(4, m_labelCount);
    }
}

} // namespace corv
