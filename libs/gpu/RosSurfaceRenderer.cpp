#include "gpu/RosSurfaceRenderer.h"

#include "core/DebugLog.h"
#include "scene/SurfaceRaster.h"

#include <QFile>
#include <algorithm>
#include <cmath>
#include <cstring>

using namespace Qt::StringLiterals;

namespace corv {

namespace {
constexpr int kTex = RosSurfaceData::kTex;
constexpr int kBlock = RosSurfaceData::kBlock;
constexpr int kSteps[] = {1, 2, 4, 8, 16};   // vertex spacing per level of detail, cells
constexpr double kMinTriPx = 6;
constexpr int kRebuildQuadsPerFrame = 70000;
constexpr float kGhostAlpha = 0.35f;
constexpr float kRelief = 2;                 // shading of a solid surface: heights x2
const QColor kGrey(0xa6, 0xa6, 0xa6);

struct MeshUbuf {
    float viewProj[16];
    float model[16];
    float vehicle[4];
    float color[4];
    float params[4];
    float rough[4];
    float flags[4];
    float cam[4];
};
struct VolUbuf {
    float viewProj[16];
    float model[16];
    float vehicle[4];
    float color[4];
    float params[4];
    float spread[4];
    float cam[4];
};

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

// Is a box (8 world corners) wholly outside one side of the view?
bool outsideFrustum(const QMatrix4x4 &vp, const QVector3D *c)
{
    int out[5] = {0, 0, 0, 0, 0};
    for (int i = 0; i < 8; ++i) {
        const QVector4D p = vp * QVector4D(c[i], 1);
        if (p.x() < -p.w()) ++out[0];
        if (p.x() > p.w()) ++out[1];
        if (p.y() < -p.w()) ++out[2];
        if (p.y() > p.w()) ++out[3];
        if (p.w() <= 0) ++out[4];
    }
    return std::any_of(std::begin(out), std::end(out), [](int n) { return n == 8; });
}

double boxDistance(const QVector3D &p, const QVector3D &lo, const QVector3D &hi)
{
    const double dx = std::max({double(lo.x()) - p.x(), 0.0, double(p.x()) - hi.x()});
    const double dy = std::max({double(lo.y()) - p.y(), 0.0, double(p.y()) - hi.y()});
    const double dz = std::max({double(lo.z()) - p.z(), 0.0, double(p.z()) - hi.z()});
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}
} // namespace

RosSurfaceRenderer::RosSurfaceRenderer(QRhi *rhi) : m_rhi(rhi)
{
    m_meshVs = loadShader(u":/shaders/rosmesh.vert.qsb"_s);
    m_meshFs = loadShader(u":/shaders/rosmesh.frag.qsb"_s);
    m_volVs = loadShader(u":/shaders/rosvolume.vert.qsb"_s);
    m_volFs = loadShader(u":/shaders/rosvolume.frag.qsb"_s);
    if (!m_rhi->isTextureFormatSupported(QRhiTexture::RGBA32F)) LOG_W("gpu") << "RGBA32F textures not supported: no ROS surface";
    m_sampler.reset(m_rhi->newSampler(QRhiSampler::Nearest, QRhiSampler::Nearest, QRhiSampler::None, QRhiSampler::ClampToEdge, QRhiSampler::ClampToEdge));
    m_sampler->create();
    m_dummyTex.reset(m_rhi->newTexture(QRhiTexture::RGBA32F, QSize(1, 1)));
    m_dummyTex->create();
    m_dummyUbuf.reset(m_rhi->newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer, m_rhi->ubufAligned(sizeof(MeshUbuf))));
    m_dummyUbuf->create();
    m_layoutSrb.reset(m_rhi->newShaderResourceBindings());
    m_layoutSrb->setBindings({QRhiShaderResourceBinding::uniformBuffer(0, QRhiShaderResourceBinding::VertexStage | QRhiShaderResourceBinding::FragmentStage, m_dummyUbuf.get()),
                              QRhiShaderResourceBinding::sampledTexture(1, QRhiShaderResourceBinding::VertexStage, m_dummyTex.get(), m_sampler.get())});
    m_layoutSrb->create();
    m_volUbuf.reset(m_rhi->newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer, m_rhi->ubufAligned(sizeof(VolUbuf))));
    m_volUbuf->create();
    m_volSrb.reset(m_rhi->newShaderResourceBindings());
    m_volSrb->setBindings({QRhiShaderResourceBinding::uniformBuffer(0, QRhiShaderResourceBinding::VertexStage | QRhiShaderResourceBinding::FragmentStage, m_volUbuf.get())});
    m_volSrb->create();
}

RosSurfaceRenderer::~RosSurfaceRenderer() = default;

void RosSurfaceRenderer::sync(RosSurfaceData &d, LatLon sceneOrigin, double offsetAlt, const QVector3D &vehicle, const Camera &cam)
{
    const RosSurfaceData::View &v = d.view;
    m_view = v;
    m_show = v.visible && v.shown && m_rhi->isTextureFormatSupported(QRhiTexture::RGBA32F);
    m_cell = float(d.cell);
    m_vehicle = vehicle;
    m_cam = cam.pos;
    const double kOriginLon = kMetersPerDegLat * std::cos(sceneOrigin.lat * kDeg);

    // ── Height field ──
    if (d.generation != m_generation) {
        m_generation = d.generation;
        for (auto &[key, b] : m_blocks) {
            retire(b.tex);
            retire(b.ubuf);
            retire(b.ubufAlt);
            retire(b.index);
            retire(b.srb);
            retire(b.srbAlt);
        }
        m_blocks.clear();
    }
    for (auto it = m_blocks.begin(); it != m_blocks.end();) {
        if (d.blocks.count(it->first)) {
            ++it;
            continue;
        }
        GpuBlock &b = it->second;
        retire(b.tex);
        retire(b.ubuf);
        retire(b.ubufAlt);
        retire(b.index);
        retire(b.srb);
        retire(b.srbAlt);
        it = m_blocks.erase(it);
    }
    int drawn = 0, triangles = 0;
    if (d.anchor) {
        const RosAnchor &a = *d.anchor;
        const GroundMeters base = latLonToMeters(a.lat, a.lon, sceneOrigin);
        const double kx = kOriginLon / a.mPerLon, kz = kMetersPerDegLat / a.mPerLat;
        const double span = kBlock * d.cell;
        struct Todo {
            double dist;
            std::int64_t key;
            int step;
        };
        std::vector<Todo> todo;
        for (const auto &[key, blk] : d.blocks) {
            GpuBlock &g = m_blocks[key];
            QMatrix4x4 m;
            m.translate(float(base.x + blk->bx * span * kx), float(a.alt + offsetAlt), float(base.z - blk->by * span * kz));
            m.scale(float(kx), 1, float(kz));
            g.model = m;
            if (blk->revision != g.revision && !blk->dirty) {
                g.revision = blk->revision;
                g.texData.resize(std::size_t(kTex) * kTex * 4);
                for (int p = 0; p < kTex * kTex; ++p) {
                    g.texData[std::size_t(p) * 4] = blk->view[std::size_t(p) * 2];
                    g.texData[std::size_t(p) * 4 + 1] = blk->view[std::size_t(p) * 2 + 1];
                    g.texData[std::size_t(p) * 4 + 2] = blk->roughDone ? blk->rough[std::size_t(p)] / 100.f : 0.f;
                    g.texData[std::size_t(p) * 4 + 3] = 0;
                }
                g.step = 0;   // the index again
            }
            // In view, and the step its distance asks for
            const float ext = float((kBlock + 1) * d.cell);
            QVector3D corners[8];
            QVector3D lo(1e30f, 1e30f, 1e30f), hi(-1e30f, -1e30f, -1e30f);
            int c = 0;
            for (float x : {0.f, ext})
                for (float y : {blk->minH, blk->maxH})
                    for (float z : {0.f, -ext}) {
                        corners[c] = m.map(QVector3D(x, y, z));
                        lo = QVector3D(std::min(lo.x(), corners[c].x()), std::min(lo.y(), corners[c].y()), std::min(lo.z(), corners[c].z()));
                        hi = QVector3D(std::max(hi.x(), corners[c].x()), std::max(hi.y(), corners[c].y()), std::max(hi.z(), corners[c].z()));
                        ++c;
                    }
            g.inView = m_show && !outsideFrustum(cam.viewProj, corners);
            if (g.inView && g.revision >= 0) {
                const double dist = std::max(1.0, boxDistance(cam.pos, lo, hi));
                int s = 1;
                for (int st : kSteps) {
                    s = st;
                    if (d.cell * st * cam.focalPx / dist >= kMinTriPx) break;
                }
                if (s != g.step) todo.push_back({dist, key, s});
            }
        }
        // Nearest first, within the budget (a full-resolution block is 57 600 quads)
        std::sort(todo.begin(), todo.end(), [](const Todo &x, const Todo &y) { return x.dist < y.dist; });
        int budget = kRebuildQuadsPerFrame;
        static std::vector<std::uint16_t> scratch(std::size_t(kBlock) * kBlock * 6);
        for (const Todo &t : todo) {
            if (budget <= 0) break;
            const auto &blk = d.blocks.at(t.key);
            GpuBlock &g = m_blocks[t.key];
            const int n = raster::triangulate(blk->view.data(), kTex, kTex, t.step, scratch.data(), d.cell);
            g.indexData.assign(scratch.begin(), scratch.begin() + n);
            g.indexPending = true;
            g.step = t.step;
            const int W = kBlock / t.step + 1;
            budget -= (W - 1) * (W - 1);
        }
        for (const auto &[key, g] : m_blocks) {
            if (!g.inView) continue;
            const int n = g.indexPending ? int(g.indexData.size()) : g.indices;
            if (n > 0) {
                ++drawn;
                triangles += n / 3;
            }
        }
    }
    d.drawnBlocks = drawn;
    d.drawnTriangles = triangles;

    // ── Volume ──
    if (d.volGeneration != m_volGeneration) {
        m_volGeneration = d.volGeneration;
        for (auto &[key, c] : m_chunks) retire(c.vbuf);
        m_chunks.clear();
    }
    for (auto it = m_chunks.begin(); it != m_chunks.end();) {
        if (d.chunks.count(it->first)) {
            ++it;
            continue;
        }
        retire(it->second.vbuf);
        it = m_chunks.erase(it);
    }
    m_hasVolume = d.volAnchor && !d.chunks.empty();
    if (d.volAnchor) {
        const RosAnchor &a = *d.volAnchor;
        const GroundMeters base = latLonToMeters(a.lat, a.lon, sceneOrigin);
        const double kx = kOriginLon / a.mPerLon, kz = kMetersPerDegLat / a.mPerLat;
        // (east, north, up) -> (x = east, y = up, z = -north), then to the anchor
        m_volModel = QMatrix4x4(float(kx), 0, 0, float(base.x), 0, 0, 1, float(a.alt + offsetAlt), 0, float(-kz), 0, float(base.z), 0, 0, 0, 1);
        for (const auto &[key, ch] : d.chunks) {
            GpuChunk &g = m_chunks[key];
            if (g.revision == ch->revision) continue;
            g.revision = ch->revision;
            const std::size_t nv = ch->pos.size() / 3;
            g.data.resize(nv * 7);
            for (std::size_t i = 0; i < nv; ++i) {
                float *o = g.data.data() + i * 7;
                o[0] = ch->pos[i * 3];
                o[1] = ch->pos[i * 3 + 1];
                o[2] = ch->pos[i * 3 + 2];
                o[3] = i % 3 == 0 ? 1.f : 0.f;
                o[4] = i % 3 == 1 ? 1.f : 0.f;
                o[5] = i % 3 == 2 ? 1.f : 0.f;
                o[6] = i < ch->rough.size() ? ch->rough[i] : 0.f;
            }
        }
    }
}

std::unique_ptr<QRhiGraphicsPipeline> RosSurfaceRenderer::pipeline(const QShader &vs, const QShader &fs, const QRhiVertexInputLayout &layout,
                                                                   QRhiShaderResourceBindings *srb, bool blend, bool depthTest, bool depthWrite,
                                                                   QRhiGraphicsPipeline::CompareOp op, bool colorWrite)
{
    std::unique_ptr<QRhiGraphicsPipeline> p(m_rhi->newGraphicsPipeline());
    p->setShaderStages({{QRhiShaderStage::Vertex, vs}, {QRhiShaderStage::Fragment, fs}});
    p->setVertexInputLayout(layout);
    p->setShaderResourceBindings(srb);
    p->setRenderPassDescriptor(m_rp);
    p->setSampleCount(m_sampleCount);
    p->setTopology(QRhiGraphicsPipeline::Triangles);
    p->setCullMode(QRhiGraphicsPipeline::None);
    p->setDepthTest(depthTest);
    p->setDepthWrite(depthWrite);
    p->setDepthOp(op);
    QRhiGraphicsPipeline::TargetBlend b;
    if (blend) {
        b.enable = true;
        b.srcColor = QRhiGraphicsPipeline::One;   // premultiplied
        b.dstColor = QRhiGraphicsPipeline::OneMinusSrcAlpha;
        b.srcAlpha = QRhiGraphicsPipeline::One;
        b.dstAlpha = QRhiGraphicsPipeline::OneMinusSrcAlpha;
    }
    if (!colorWrite) b.colorWrite = {};
    p->setTargetBlends({b});
    p->create();
    return p;
}

void RosSurfaceRenderer::ensurePipelines(QRhiRenderPassDescriptor *rp, int sampleCount)
{
    if (m_solid && rp == m_rp && sampleCount == m_sampleCount) return;
    m_rp = rp;
    m_sampleCount = sampleCount;
    for (auto *p : {&m_linesOver, &m_linesDepth, &m_ghost, &m_solid, &m_mask, &m_volLinesOver, &m_volLinesDepth, &m_volSolid}) retire(*p);
    const QRhiVertexInputLayout none;
    using C = QRhiGraphicsPipeline;
    m_linesOver = pipeline(m_meshVs, m_meshFs, none, m_layoutSrb.get(), true, false, false, C::LessOrEqual, true);
    m_linesDepth = pipeline(m_meshVs, m_meshFs, none, m_layoutSrb.get(), true, true, false, C::LessOrEqual, true);
    m_ghost = pipeline(m_meshVs, m_meshFs, none, m_layoutSrb.get(), true, true, false, C::Greater, true);
    m_solid = pipeline(m_meshVs, m_meshFs, none, m_layoutSrb.get(), false, true, true, C::LessOrEqual, true);
    // Depth test on with Always, not off: OpenGL writes no depth with the test disabled
    m_mask = pipeline(m_meshVs, m_meshFs, none, m_layoutSrb.get(), false, true, true, C::Always, false);
    QRhiVertexInputLayout vol;
    vol.setBindings({{7 * sizeof(float)}});
    vol.setAttributes({{0, 0, QRhiVertexInputAttribute::Float3, 0},
                       {0, 1, QRhiVertexInputAttribute::Float3, 3 * sizeof(float)},
                       {0, 2, QRhiVertexInputAttribute::Float, 6 * sizeof(float)}});
    m_volLinesOver = pipeline(m_volVs, m_volFs, vol, m_volSrb.get(), true, false, false, C::LessOrEqual, true);
    m_volLinesDepth = pipeline(m_volVs, m_volFs, vol, m_volSrb.get(), true, true, false, C::LessOrEqual, true);
    m_volSolid = pipeline(m_volVs, m_volFs, vol, m_volSrb.get(), false, true, true, C::LessOrEqual, true);
}

void RosSurfaceRenderer::initBlock(GpuBlock &b)
{
    b.tex.reset(m_rhi->newTexture(QRhiTexture::RGBA32F, QSize(kTex, kTex)));
    b.tex->create();
    for (auto *u : {&b.ubuf, &b.ubufAlt}) {
        u->reset(m_rhi->newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer, m_rhi->ubufAligned(sizeof(MeshUbuf))));
        (*u)->create();
    }
    auto srb = [&](QRhiBuffer *ub) {
        auto *s = m_rhi->newShaderResourceBindings();
        s->setBindings({QRhiShaderResourceBinding::uniformBuffer(0, QRhiShaderResourceBinding::VertexStage | QRhiShaderResourceBinding::FragmentStage, ub),
                        QRhiShaderResourceBinding::sampledTexture(1, QRhiShaderResourceBinding::VertexStage, b.tex.get(), m_sampler.get())});
        s->create();
        return s;
    };
    b.srb.reset(srb(b.ubuf.get()));
    b.srbAlt.reset(srb(b.ubufAlt.get()));
}

void RosSurfaceRenderer::fillUbuf(QRhiResourceUpdateBatch *u, QRhiBuffer *buf, const QMatrix4x4 &viewProj, const QMatrix4x4 &model, int step, float alpha, bool mask)
{
    MeshUbuf ub{};
    std::memcpy(ub.viewProj, viewProj.constData(), sizeof(ub.viewProj));
    std::memcpy(ub.model, model.constData(), sizeof(ub.model));
    ub.vehicle[0] = m_vehicle.x(); ub.vehicle[1] = m_vehicle.y(); ub.vehicle[2] = m_vehicle.z(); ub.vehicle[3] = m_cell;
    ub.color[0] = float(kGrey.redF()); ub.color[1] = float(kGrey.greenF()); ub.color[2] = float(kGrey.blueF()); ub.color[3] = m_view.fill;
    ub.params[0] = float(m_view.mode); ub.params[1] = m_view.nearM; ub.params[2] = m_view.farM; ub.params[3] = alpha;
    ub.rough[0] = m_view.roughMean; ub.rough[1] = m_view.roughFull; ub.rough[2] = float(step); ub.rough[3] = float(kBlock / std::max(1, step) + 1);
    ub.flags[0] = mask ? 1.f : 0.f; ub.flags[1] = kRelief;
    ub.cam[0] = m_cam.x(); ub.cam[1] = m_cam.y(); ub.cam[2] = m_cam.z();
    u->updateDynamicBuffer(buf, 0, sizeof(MeshUbuf), &ub);
}

void RosSurfaceRenderer::prepare(const QMatrix4x4 &viewProj, QRhiRenderPassDescriptor *rp, int sampleCount, QRhiResourceUpdateBatch *u)
{
    ensurePipelines(rp, sampleCount);
    const bool solid = m_view.mode != RosSurfaceData::Gray;
    for (auto &[key, g] : m_blocks) {
        if (!g.texData.empty()) {
            if (!g.tex) initBlock(g);
            QRhiTextureSubresourceUploadDescription sub(g.texData.data(), quint32(g.texData.size() * sizeof(float)));
            u->uploadTexture(g.tex.get(), QRhiTextureUploadDescription({0, 0, sub}));
            g.texData.clear();
            g.texData.shrink_to_fit();
        }
        if (g.indexPending) {
            const quint32 bytes = quint32(g.indexData.size() * sizeof(std::uint16_t));
            if (bytes > 0) {
                if (!g.index || bytes > g.indexCap) {
                    retire(g.index);
                    g.indexCap = std::max<quint32>(bytes * 2, 4096);
                    g.index.reset(m_rhi->newBuffer(QRhiBuffer::Static, QRhiBuffer::IndexBuffer, g.indexCap));
                    g.index->create();
                }
                u->uploadStaticBuffer(g.index.get(), 0, bytes, g.indexData.data());
            }
            g.indices = int(g.indexData.size());
            g.indexData.clear();
            g.indexPending = false;
        }
        if (!g.tex || !g.inView || !g.indices || !m_show) continue;
        fillUbuf(u, g.ubuf.get(), viewProj, g.model, g.step, 1.f, false);
        // The second pass: the faint ghost (grey, depth-tested) or the depth reset (solid, through the terrain)
        fillUbuf(u, g.ubufAlt.get(), viewProj, g.model, g.step, solid ? 1.f : kGhostAlpha, solid);
    }
    for (auto &[key, c] : m_chunks) {
        if (c.data.empty()) continue;
        const quint32 bytes = quint32(c.data.size() * sizeof(float));
        retire(c.vbuf);
        c.vbuf.reset(m_rhi->newBuffer(QRhiBuffer::Immutable, QRhiBuffer::VertexBuffer, bytes));
        c.vbuf->create();
        u->uploadStaticBuffer(c.vbuf.get(), c.data.data());
        c.vertices = int(c.data.size() / 7);
        c.data.clear();
        c.data.shrink_to_fit();
    }
    if (m_hasVolume && m_show) {
        VolUbuf vb{};
        std::memcpy(vb.viewProj, viewProj.constData(), sizeof(vb.viewProj));
        std::memcpy(vb.model, m_volModel.constData(), sizeof(vb.model));
        vb.vehicle[0] = m_vehicle.x(); vb.vehicle[1] = m_vehicle.y(); vb.vehicle[2] = m_vehicle.z();
        vb.color[0] = float(kGrey.redF()); vb.color[1] = float(kGrey.greenF()); vb.color[2] = float(kGrey.blueF()); vb.color[3] = m_view.fill;
        vb.params[0] = float(m_view.mode); vb.params[1] = m_view.nearM; vb.params[2] = m_view.farM;
        vb.spread[0] = m_view.spreadMean; vb.spread[1] = m_view.spreadFull;
        vb.cam[0] = m_cam.x(); vb.cam[1] = m_cam.y(); vb.cam[2] = m_cam.z();
        u->updateDynamicBuffer(m_volUbuf.get(), 0, sizeof(VolUbuf), &vb);
    }
}

void RosSurfaceRenderer::drawOpaque(QRhiCommandBuffer *cb, const QSize &out)
{
    if (!m_show || !m_solid || m_view.mode == RosSurfaceData::Gray) return;
    cb->setViewport({0, 0, float(out.width()), float(out.height())});
    auto drawBlocks = [&](QRhiGraphicsPipeline *p, bool alt) {
        cb->setGraphicsPipeline(p);
        for (auto &[key, g] : m_blocks) {
            if (!g.tex || !g.inView || !g.indices || !g.index) continue;
            cb->setShaderResources(alt ? g.srbAlt.get() : g.srb.get());
            cb->setVertexInput(0, 0, nullptr, g.index.get(), 0, QRhiCommandBuffer::IndexUInt16);
            cb->drawIndexed(quint32(g.indices));
        }
    };
    if (m_view.overTerrain) drawBlocks(m_mask.get(), true);
    drawBlocks(m_solid.get(), false);
    // A cave's surface: meant for the inside, depth-tested whenever solid
    if (m_hasVolume) {
        cb->setGraphicsPipeline(m_volSolid.get());
        cb->setShaderResources(m_volSrb.get());
        for (auto &[key, c] : m_chunks) {
            if (!c.vbuf || !c.vertices) continue;
            const QRhiCommandBuffer::VertexInput in(c.vbuf.get(), 0);
            cb->setVertexInput(0, 1, &in);
            cb->draw(quint32(c.vertices));
        }
    }
}

void RosSurfaceRenderer::drawTranslucent(QRhiCommandBuffer *cb, const QSize &out)
{
    if (!m_show || !m_solid || m_view.mode != RosSurfaceData::Gray) return;
    cb->setViewport({0, 0, float(out.width()), float(out.height())});
    auto drawBlocks = [&](QRhiGraphicsPipeline *p, bool alt) {
        cb->setGraphicsPipeline(p);
        for (auto &[key, g] : m_blocks) {
            if (!g.tex || !g.inView || !g.indices || !g.index) continue;
            cb->setShaderResources(alt ? g.srbAlt.get() : g.srb.get());
            cb->setVertexInput(0, 0, nullptr, g.index.get(), 0, QRhiCommandBuffer::IndexUInt16);
            cb->drawIndexed(quint32(g.indices));
        }
    };
    if (!m_view.overTerrain) drawBlocks(m_ghost.get(), true);
    drawBlocks(m_view.overTerrain ? m_linesOver.get() : m_linesDepth.get(), false);
    if (m_hasVolume) {
        cb->setGraphicsPipeline(m_view.overTerrain ? m_volLinesOver.get() : m_volLinesDepth.get());
        cb->setShaderResources(m_volSrb.get());
        for (auto &[key, c] : m_chunks) {
            if (!c.vbuf || !c.vertices) continue;
            const QRhiCommandBuffer::VertexInput in(c.vbuf.get(), 0);
            cb->setVertexInput(0, 1, &in);
            cb->draw(quint32(c.vertices));
        }
    }
}

} // namespace corv
