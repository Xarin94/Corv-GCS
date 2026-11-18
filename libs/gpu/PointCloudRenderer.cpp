#include "gpu/PointCloudRenderer.h"

#include "core/DebugLog.h"

#include <QFile>
#include <cstring>

namespace corv {

namespace {
struct Ubuf {
    float mvp[16];
    float params[4];
    float viewport[4];
};
constexpr quint32 kPointBytes = 4 * sizeof(float);

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
} // namespace

PointCloudRenderer::PointCloudRenderer(QRhi *rhi) : m_rhi(rhi)
{
    m_mapVs = loadShader(QStringLiteral(":/shaders/points.vert.qsb"));
    m_liveVs = loadShader(QStringLiteral(":/shaders/livepoints.vert.qsb"));
    m_fs = loadShader(QStringLiteral(":/shaders/points.frag.qsb"));
    m_corners.reset(m_rhi->newBuffer(QRhiBuffer::Immutable, QRhiBuffer::VertexBuffer, 8 * sizeof(float)));
    m_corners->create();
    for (Pass *p : {&m_map, &m_live}) {
        p->ubuf.reset(m_rhi->newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer, m_rhi->ubufAligned(sizeof(Ubuf))));
        p->ubuf->create();
        p->srb.reset(m_rhi->newShaderResourceBindings());
        p->srb->setBindings({QRhiShaderResourceBinding::uniformBuffer(0, QRhiShaderResourceBinding::VertexStage | QRhiShaderResourceBinding::FragmentStage, p->ubuf.get())});
        p->srb->create();
    }
    m_livePts.reset(m_rhi->newBuffer(QRhiBuffer::Static, QRhiBuffer::VertexBuffer, PointCloudData::kLiveCapacity * kPointBytes));
    m_livePts->create();
    m_liveBirth.reset(m_rhi->newBuffer(QRhiBuffer::Static, QRhiBuffer::VertexBuffer, PointCloudData::kLiveCapacity * sizeof(float)));
    m_liveBirth->create();
}

PointCloudRenderer::~PointCloudRenderer() = default;

void PointCloudRenderer::sync(const PointCloudData &d, const QMatrix4x4 &mapModel, const QMatrix4x4 &liveModel, float dpr)
{
    m_visible = d.visible;
    m_mapModel = mapModel;
    m_liveModel = liveModel;
    m_size = float(std::max(1.0, d.pointSize) * dpr);
    m_mode = d.intensityColors ? 1.f : 0.f;
    m_hmin = d.heightRange[0];
    m_hmax = d.heightRange[1];
    m_ttl = float(d.liveTtl);
    m_now = float(PointCloudData::nowSeconds());
    m_overTerrain = d.overTerrain;

    // Map: what each block gained since the last upload
    if (d.generation != m_generation) {
        m_generation = d.generation;
        m_dropBlocks = true;
        m_blockCounts.clear();
        m_blockUploads.clear();
    }
    const std::size_t known = m_blockCounts.size();
    m_blockCounts.resize(d.chunks.size(), 0);
    for (std::size_t i = 0; i < d.chunks.size(); ++i) {
        const int was = i < known ? m_blockCounts[i] : 0;
        const int now = d.chunks[i]->count;
        if (now > was) {
            m_blockUploads.push_back({i, d.chunks[i], was, now});
            m_blockCounts[i] = now;
        }
    }

    // Live ring: the slots written since the last upload, copied (the GUI rewrites them)
    if (d.liveGeneration != m_liveGeneration) {
        m_liveGeneration = d.liveGeneration;
        m_liveUploads.clear();
    }
    const std::uint64_t delta = d.liveWritten - m_liveWritten;
    m_liveWritten = d.liveWritten;
    const int cap = PointCloudData::kLiveCapacity;
    if (delta > 0) {
        int n = int(std::min<std::uint64_t>(delta, std::uint64_t(cap)));
        int from = ((d.liveHead - n) % cap + cap) % cap;
        while (n > 0) {
            const int run = std::min(n, cap - from);
            LiveUpload up;
            up.slot = from;
            up.pts.assign(d.livePts.begin() + std::ptrdiff_t(from) * 4, d.livePts.begin() + std::ptrdiff_t(from + run) * 4);
            up.birth.assign(d.liveBirth.begin() + from, d.liveBirth.begin() + from + run);
            m_liveUploads.push_back(std::move(up));
            n -= run;
            from = (from + run) % cap;
        }
    }
    m_liveCount = d.liveCount;
}

std::unique_ptr<QRhiGraphicsPipeline> PointCloudRenderer::makePipeline(const QShader &vs, const QRhiVertexInputLayout &layout,
                                                                       QRhiShaderResourceBindings *srb, bool depth, bool depthWrite)
{
    std::unique_ptr<QRhiGraphicsPipeline> p(m_rhi->newGraphicsPipeline());
    p->setShaderStages({{QRhiShaderStage::Vertex, vs}, {QRhiShaderStage::Fragment, m_fs}});
    p->setVertexInputLayout(layout);
    p->setShaderResourceBindings(srb);
    p->setRenderPassDescriptor(m_rp);
    p->setSampleCount(m_sampleCount);
    p->setTopology(QRhiGraphicsPipeline::TriangleStrip);
    p->setCullMode(QRhiGraphicsPipeline::None);
    p->setDepthTest(depth);
    p->setDepthWrite(depthWrite);
    p->setDepthOp(QRhiGraphicsPipeline::LessOrEqual);
    QRhiGraphicsPipeline::TargetBlend b;
    b.enable = true;
    b.srcColor = QRhiGraphicsPipeline::One;   // premultiplied
    b.dstColor = QRhiGraphicsPipeline::OneMinusSrcAlpha;
    b.srcAlpha = QRhiGraphicsPipeline::One;
    b.dstAlpha = QRhiGraphicsPipeline::OneMinusSrcAlpha;
    p->setTargetBlends({b});
    p->create();
    return p;
}

void PointCloudRenderer::ensurePipelines(QRhiRenderPassDescriptor *rp, int sampleCount)
{
    if (m_mapDepth && rp == m_rp && sampleCount == m_sampleCount) return;
    m_rp = rp;
    m_sampleCount = sampleCount;
    QRhiVertexInputLayout map;
    map.setBindings({{2 * sizeof(float)}, {kPointBytes, QRhiVertexInputBinding::PerInstance}});
    map.setAttributes({{0, 0, QRhiVertexInputAttribute::Float2, 0}, {1, 1, QRhiVertexInputAttribute::Float4, 0}});
    QRhiVertexInputLayout live;
    live.setBindings({{2 * sizeof(float)}, {kPointBytes, QRhiVertexInputBinding::PerInstance}, {sizeof(float), QRhiVertexInputBinding::PerInstance}});
    live.setAttributes({{0, 0, QRhiVertexInputAttribute::Float2, 0}, {1, 1, QRhiVertexInputAttribute::Float4, 0}, {2, 2, QRhiVertexInputAttribute::Float, 0}});
    for (auto *p : {&m_mapDepth, &m_mapOver, &m_liveDepth, &m_liveOver}) retire(*p);
    m_mapDepth = makePipeline(m_mapVs, map, m_map.srb.get(), true, true);
    m_mapOver = makePipeline(m_mapVs, map, m_map.srb.get(), false, false);
    m_liveDepth = makePipeline(m_liveVs, live, m_live.srb.get(), true, false);
    m_liveOver = makePipeline(m_liveVs, live, m_live.srb.get(), false, false);
}

void PointCloudRenderer::prepare(const QMatrix4x4 &viewProj, const QSize &out, QRhiRenderPassDescriptor *rp, int sampleCount, QRhiResourceUpdateBatch *u)
{
    if (!m_cornersReady) {
        const float corners[8] = {-1, -1, -1, 1, 1, -1, 1, 1};
        u->uploadStaticBuffer(m_corners.get(), corners);
        m_cornersReady = true;
    }
    ensurePipelines(rp, sampleCount);

    if (m_dropBlocks) {
        for (GpuBlock &b : m_blocks) retire(b.buf);
        m_blocks.clear();
        m_dropBlocks = false;
    }
    for (const BlockUpload &up : m_blockUploads) {
        if (up.index >= m_blocks.size()) m_blocks.resize(up.index + 1);
        GpuBlock &g = m_blocks[up.index];
        if (!g.buf) {
            g.buf.reset(m_rhi->newBuffer(QRhiBuffer::Static, QRhiBuffer::VertexBuffer, PointCloudData::kChunk * kPointBytes));
            g.buf->create();
        }
        u->uploadStaticBuffer(g.buf.get(), quint32(up.from) * kPointBytes, quint32(up.to - up.from) * kPointBytes,
                              up.block->pts.get() + std::size_t(up.from) * 4);
        g.uploaded = up.to;
    }
    m_blockUploads.clear();
    for (const LiveUpload &up : m_liveUploads) {
        u->uploadStaticBuffer(m_livePts.get(), quint32(up.slot) * kPointBytes, quint32(up.birth.size()) * kPointBytes, up.pts.data());
        u->uploadStaticBuffer(m_liveBirth.get(), quint32(up.slot) * sizeof(float), quint32(up.birth.size() * sizeof(float)), up.birth.data());
    }
    m_liveUploads.clear();
    if (!m_visible) return;

    Ubuf ub{};
    const QMatrix4x4 mapMvp = viewProj * m_mapModel, liveMvp = viewProj * m_liveModel;
    std::memcpy(ub.mvp, mapMvp.constData(), sizeof(ub.mvp));
    ub.params[0] = m_size;
    ub.params[1] = m_mode;
    ub.params[2] = m_hmin;
    ub.params[3] = m_hmax;
    ub.viewport[0] = float(out.width());
    ub.viewport[1] = float(out.height());
    ub.viewport[2] = m_now;
    ub.viewport[3] = m_ttl;
    u->updateDynamicBuffer(m_map.ubuf.get(), 0, sizeof(Ubuf), &ub);
    std::memcpy(ub.mvp, liveMvp.constData(), sizeof(ub.mvp));
    u->updateDynamicBuffer(m_live.ubuf.get(), 0, sizeof(Ubuf), &ub);
}

void PointCloudRenderer::draw(QRhiCommandBuffer *cb, const QSize &out)
{
    if (!m_visible || !m_mapDepth) return;
    cb->setViewport({0, 0, float(out.width()), float(out.height())});
    cb->setGraphicsPipeline(m_overTerrain ? m_mapOver.get() : m_mapDepth.get());
    cb->setShaderResources(m_map.srb.get());
    for (const GpuBlock &b : m_blocks) {
        if (!b.buf || b.uploaded <= 0) continue;
        const QRhiCommandBuffer::VertexInput in[] = {{m_corners.get(), 0}, {b.buf.get(), 0}};
        cb->setVertexInput(0, 2, in);
        cb->draw(4, quint32(b.uploaded));
    }
    if (m_liveCount > 0) {
        cb->setGraphicsPipeline(m_overTerrain ? m_liveOver.get() : m_liveDepth.get());
        cb->setShaderResources(m_live.srb.get());
        const QRhiCommandBuffer::VertexInput in[] = {{m_corners.get(), 0}, {m_livePts.get(), 0}, {m_liveBirth.get(), 0}};
        cb->setVertexInput(0, 3, in);
        cb->draw(4, quint32(m_liveCount));
    }
}

} // namespace corv
