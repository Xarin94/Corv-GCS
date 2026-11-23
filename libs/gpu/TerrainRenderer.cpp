#include "gpu/TerrainRenderer.h"

#include "core/DebugLog.h"

#include <QFile>
#include <QtCore/qfloat16.h>
#include <array>
#include <cmath>
#include <cstring>

using namespace Qt::StringLiterals;

namespace corv {

namespace {
constexpr int kInitialLayers = 512;   // 15 MB of heights: the default radius without growing
constexpr int kSteps[4] = {1, 2, 4, 8};
constexpr float kLodTargetPixels = 6;
constexpr float kSrtm1SpacingM = 30.9f;
constexpr std::size_t kMaxUploadsPerFrame = 24;
constexpr qsizetype kImageBytesPerFrame = 2 * 1024 * 1024;
constexpr std::size_t kMaxQueuedImages = 4;   // taken from the manager and not on the GPU yet

QShader loadShader(const QString &name)
{
    QFile f(name);
    if (!f.open(QIODevice::ReadOnly)) {
        LOG_E("gpu") << "shader missing:" << name;
        return {};
    }
    return QShader::fromSerialized(f.readAll());
}

struct Ubuf {
    float viewProj[16];
    float camPos[4];
    float sunDir[4];
    float params[4];
    float extra[4];
    float zone[4];
    float zone2[4];
    float detailMap[4][4];
    float detailWin[4][4];
    float detailOn[4];
};

struct Instance {
    float chunk[4];     // x0, z0, dx, dz
    float layer;        // heights layer
    float gridMax;
    float sat;          // satellite layer, -1: height palette / schematic
    float pad;
};

int stepIndex(int step)
{
    switch (step) {
    case 1: return 0;
    case 2: return 1;
    case 4: return 2;
    default: return 3;
    }
}

/// Frustum planes from a view-projection matrix (D3D-style depth 0..1 after the clip correction).
std::array<QVector4D, 6> frustumPlanes(const QMatrix4x4 &m)
{
    const QVector4D r0 = m.row(0), r1 = m.row(1), r2 = m.row(2), r3 = m.row(3);
    std::array<QVector4D, 6> p = {r3 + r0, r3 - r0, r3 + r1, r3 - r1, r2, r3 - r2};
    for (auto &v : p) v /= QVector3D(v.x(), v.y(), v.z()).length();
    return p;
}

bool boxVisible(const std::array<QVector4D, 6> &planes, const QVector3D &lo, const QVector3D &hi)
{
    for (const QVector4D &p : planes) {
        // The box corner furthest along the plane normal
        const QVector3D v(p.x() >= 0 ? hi.x() : lo.x(), p.y() >= 0 ? hi.y() : lo.y(), p.z() >= 0 ? hi.z() : lo.z());
        if (p.x() * v.x() + p.y() * v.y() + p.z() * v.z() + p.w() < 0) return false;
    }
    return true;
}

/// Replace a resource that this frame may still use: the resource update batch can
/// hold uploads to it (chunks added earlier in the same sync) and the previous
/// frame's commands bound it. deleteLater() destroys it at the end of the frame;
/// an immediate delete left the batch pointing at freed memory (the intermittent
/// 0xC0000409 in Qt6Gui.dll right after "terrain grid grown").
template <class T>
void retire(std::unique_ptr<T> &p)
{
    if (p) p.release()->deleteLater();
}

template <class Map>
void retireAll(Map &m)
{
    for (auto &kv : m) retire(kv.second);
    m.clear();
}

/// Layers of a satellite page: about 20 MB (BC1) or 40 MB (RGBA8) at most.
int pageLayers(int size, bool bc1)
{
    if (size >= 4096) return bc1 ? 2 : 1;
    if (size >= 2048) return bc1 ? 8 : 2;
    if (size >= 1024) return bc1 ? 16 : 8;
    if (size >= 512) return 32;
    return 64;
}

/// Bytes of a row of a mip level (a row of blocks for BC1).
qsizetype rowBytes(int w, bool bc1) { return bc1 ? qsizetype((w + 3) / 4) * 8 : qsizetype(w) * 4; }
int rowCount(int h, bool bc1) { return bc1 ? (h + 3) / 4 : h; }
} // namespace

TerrainRenderer::TerrainRenderer(QRhi *rhi) : m_rhi(rhi)
{
    m_vs = loadShader(u":/shaders/terrain.vert.qsb"_s);
    m_fs = loadShader(u":/shaders/terrain.frag.qsb"_s);
    if (!m_rhi->isTextureFormatSupported(QRhiTexture::R16)) {
        m_heightFormat = QRhiTexture::R32F;
        LOG_W("gpu") << "R16 textures not supported, heights in R32F";
    }
    // BC1 in a texture array comes out black on the OpenGL backend (Mesa radeonsi,
    // Qt 6.12: the layers sample as black although the uploads succeed), while
    // Direct3D, Metal and Vulkan draw it: on OpenGL the imagery goes in RGBA8,
    // four times the memory. CORV_SAT_FORMAT=bc1 / rgba8 forces one (diagnosis).
    const QString satFormat = qEnvironmentVariable("CORV_SAT_FORMAT").toLower();
    m_bc1 = m_rhi->isTextureFormatSupported(QRhiTexture::BC1) && m_rhi->backend() != QRhi::OpenGLES2;
    if (satFormat == "bc1"_L1) m_bc1 = m_rhi->isTextureFormatSupported(QRhiTexture::BC1);
    else if (satFormat == "rgba8"_L1) m_bc1 = false;
    m_maxLayers = std::max(64, m_rhi->resourceLimit(QRhi::TextureArraySizeMax));
    LOG_I("gpu") << "satellite imagery in" << (m_bc1 ? "BC1" : "RGBA8") << "texture arrays, up to" << m_maxLayers << "layers";
    m_ubuf.reset(m_rhi->newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer, m_rhi->ubufAligned(sizeof(Ubuf))));
    m_ubuf->create();
    m_sampler.reset(m_rhi->newSampler(QRhiSampler::Nearest, QRhiSampler::Nearest, QRhiSampler::None,
                                      QRhiSampler::ClampToEdge, QRhiSampler::ClampToEdge));
    m_sampler->create();
    m_satSampler.reset(m_rhi->newSampler(QRhiSampler::Linear, QRhiSampler::Linear, QRhiSampler::Linear,
                                         QRhiSampler::ClampToEdge, QRhiSampler::ClampToEdge));
    m_satSampler->create();
    m_noSat.reset(m_rhi->newTextureArray(QRhiTexture::RGBA8, 1, QSize(1, 1)));
    m_noSat->create();
    // UAS zones: the raster's size from the start, so the binding sets never change
    m_zoneCov.reset(m_rhi->newTexture(QRhiTexture::RGBA8, QSize(GeoZoneRaster::kSize, GeoZoneRaster::kSize)));
    m_zoneCov->create();
    const bool halfFloat = m_rhi->isTextureFormatSupported(QRhiTexture::RGBA16F);
    m_zoneFloor.reset(m_rhi->newTexture(halfFloat ? QRhiTexture::RGBA16F : QRhiTexture::RGBA32F, QSize(GeoZoneRaster::kSize, GeoZoneRaster::kSize)));
    m_zoneFloor->create();
    m_zoneSampler.reset(m_rhi->newSampler(QRhiSampler::Linear, QRhiSampler::Linear, QRhiSampler::None,
                                          QRhiSampler::ClampToEdge, QRhiSampler::ClampToEdge));
    m_zoneSampler->create();
    // Satellite detail: a placeholder bound for the levels not in use (never sampled)
    m_noDetail.reset(m_rhi->newTexture(QRhiTexture::RGBA8, QSize(1, 1)));
    m_noDetail->create();
    m_detailSampler.reset(m_rhi->newSampler(QRhiSampler::Linear, QRhiSampler::Linear, QRhiSampler::Linear,
                                            QRhiSampler::Repeat, QRhiSampler::Repeat));
    m_detailSampler->create();
    m_zeroTile.reset(m_rhi->newTexture(QRhiTexture::RGBA8, QSize(SatelliteDetail::kTile, SatelliteDetail::kTile), 1, QRhiTexture::UsedAsTransferSource));
    m_zeroTile->create();
    m_layoutSrb = makeSrb(m_noDetail.get(), m_noSat.get());
}

void TerrainRenderer::applyDetail(const std::vector<DetailOp> &ops, QRhiResourceUpdateBatch *u)
{
    if (ops.empty() && !(m_mipsDirty[0] || m_mipsDirty[1] || m_mipsDirty[2] || m_mipsDirty[3])) return;
    constexpr int L = SatelliteDetail::kLevels, T = SatelliteDetail::kTile, N = SatelliteDetail::kTiles;
    // A texture given back in this batch takes no upload before it: nothing may be pending on it
    int lastRelease[L];
    std::fill(std::begin(lastRelease), std::end(lastRelease), -1);
    for (int i = 0; i < int(ops.size()); ++i)
        if (ops[std::size_t(i)].kind == DetailOp::Release) lastRelease[ops[std::size_t(i)].level] = i;

    if (!m_zeroReady) {
        const QByteArray zero(T * T * 4, '\0');
        u->uploadTexture(m_zeroTile.get(), QRhiTextureUploadDescription(QRhiTextureUploadEntry(0, 0, QRhiTextureSubresourceUploadDescription(zero))));
        m_zeroReady = true;
    }

    bool bindingsChanged = false;
    bool *mips = m_mipsDirty;
    // A cleared slot is a copy on the GPU: the zeroes of a whole level are 16 MB to upload
    auto clearSlot = [&](QRhiTexture *tex, int slot) {
        QRhiTextureCopyDescription d;
        d.setPixelSize(QSize(T, T));
        d.setDestinationTopLeft(QPoint((slot % N) * T, (slot / N) * T));
        u->copyTexture(tex, m_zeroTile.get(), d);
    };
    for (int i = 0; i < int(ops.size()); ++i) {
        const DetailOp &op = ops[std::size_t(i)];
        if (op.level < 0 || op.level >= L) continue;
        auto &tex = m_detail[op.level];
        switch (op.kind) {
        case DetailOp::Allocate:
            if (!tex) {
                tex.reset(m_rhi->newTexture(QRhiTexture::RGBA8, QSize(SatelliteDetail::kSize, SatelliteDetail::kSize), 1,
                                            QRhiTexture::MipMapped | QRhiTexture::UsedWithGenerateMips));
                tex->create();
                bindingsChanged = true;
            }
            break;
        case DetailOp::Release:
            if (tex) {
                mips[op.level] = false;
                retire(tex);
                bindingsChanged = true;
            }
            break;
        case DetailOp::Clear:
            if (tex && i > lastRelease[op.level]) {
                for (int slot = 0; slot < N * N; ++slot) clearSlot(tex.get(), slot);
                mips[op.level] = true;
            }
            break;
        case DetailOp::ClearSlot:
            if (tex && i > lastRelease[op.level]) {
                clearSlot(tex.get(), op.slot);
                mips[op.level] = true;
            }
            break;
        case DetailOp::Upload:
            if (tex && i > lastRelease[op.level] && op.image.size() == QSize(T, T)) {
                QRhiTextureSubresourceUploadDescription d(op.image);
                d.setDestinationTopLeft(QPoint((op.slot % N) * T, (op.slot / N) * T));
                u->uploadTexture(tex.get(), QRhiTextureUploadDescription(QRhiTextureUploadEntry(0, 0, d)));
                mips[op.level] = true;
            }
            break;
        }
    }
    // One mipmap rebuild a frame (22 MB of texture each), the levels in turn
    for (int k = 0; k < L; ++k) {
        const int l = (m_nextMips + k) % L;
        if (!mips[l]) continue;
        mips[l] = false;
        if (!m_detail[l]) continue;
        u->generateMips(m_detail[l].get());
        m_nextMips = (l + 1) % L;
        break;
    }
    // The binding sets name the textures: made again (the pipelines know only their layout)
    if (bindingsChanged)
        for (auto &[size, g] : m_grids) retireAll(g.srbs);
}

void TerrainRenderer::setGeoZones(const std::shared_ptr<const GeoZoneRaster> &raster, QRhiResourceUpdateBatch *u)
{
    if (!raster || raster->coverage.empty()) return;
    const int n = GeoZoneRaster::kSize;
    u->uploadTexture(m_zoneCov.get(), QRhiTextureUploadDescription(QRhiTextureUploadEntry(
        0, 0, QRhiTextureSubresourceUploadDescription(raster->coverage.data(), quint32(raster->coverage.size())))));
    if (m_zoneFloor->format() == QRhiTexture::RGBA16F) {
        u->uploadTexture(m_zoneFloor.get(), QRhiTextureUploadDescription(QRhiTextureUploadEntry(
            0, 0, QRhiTextureSubresourceUploadDescription(raster->floors.data(), quint32(raster->floors.size() * 2)))));
    } else {
        std::vector<float> f(std::size_t(n) * n * 4);
        for (std::size_t i = 0; i < f.size(); ++i) {
            qfloat16 h;
            std::memcpy(&h, &raster->floors[i], 2);
            f[i] = float(h);
        }
        u->uploadTexture(m_zoneFloor.get(), QRhiTextureUploadDescription(QRhiTextureUploadEntry(
            0, 0, QRhiTextureSubresourceUploadDescription(f.data(), quint32(f.size() * 4)))));
    }
    m_zonesReady = true;
}

TerrainRenderer::~TerrainRenderer() = default;

TerrainRenderer::Grid *TerrainRenderer::gridFor(int size, QRhiResourceUpdateBatch *u)
{
    auto it = m_grids.find(size);
    if (it != m_grids.end()) return &it->second;
    Grid &g = m_grids[size];
    g.size = size;
    g.capacity = kInitialLayers;
    g.heights.reset(m_rhi->newTextureArray(m_heightFormat, g.capacity, QSize(size, size), 1, QRhiTexture::UsedAsTransferSource));
    g.heights->create();
    for (int i = g.capacity - 1; i >= 0; --i) g.freeLayers.push_back(i);

    // Vertices: (column, row) of every grid point
    std::vector<float> verts;
    verts.reserve(std::size_t(size) * size * 2);
    for (int r = 0; r < size; ++r)
        for (int c = 0; c < size; ++c) {
            verts.push_back(float(c));
            verts.push_back(float(r));
        }
    g.vbuf.reset(m_rhi->newBuffer(QRhiBuffer::Immutable, QRhiBuffer::VertexBuffer, quint32(verts.size() * sizeof(float))));
    g.vbuf->create();
    u->uploadStaticBuffer(g.vbuf.get(), verts.data());

    // One index buffer per decimation step over the same vertices
    for (int si = 0; si < 4; ++si) {
        const int step = kSteps[si];
        std::vector<quint16> idx;
        if ((size - 1) % step == 0) {
            for (int r = 0; r + step < size; r += step)
                for (int c = 0; c + step < size; c += step) {
                    const quint16 a = quint16(r * size + c), b = quint16(r * size + c + step);
                    const quint16 d = quint16((r + step) * size + c), e = quint16((r + step) * size + c + step);
                    idx.insert(idx.end(), {a, d, b, b, d, e});
                }
        }
        if (idx.empty()) idx = {0, 0, 0};
        g.indexCount[si] = quint32(idx.size());
        g.ibuf[si].reset(m_rhi->newBuffer(QRhiBuffer::Immutable, QRhiBuffer::IndexBuffer, quint32(idx.size() * sizeof(quint16))));
        g.ibuf[si]->create();
        u->uploadStaticBuffer(g.ibuf[si].get(), idx.data());
    }
    LOG_I("gpu") << "terrain grid" << size << "x" << size << ":" << g.capacity << "layers";
    return &g;
}

void TerrainRenderer::releaseChunk(Chunk &c)
{
    if (c.layer < 0) return;
    auto it = m_grids.find(c.grid);
    if (it != m_grids.end()) it->second.freeLayers.push_back(c.layer);
    c.layer = -1;
}

void TerrainRenderer::growHeights(Grid &g, QRhiResourceUpdateBatch *u)
{
    // Twice the layers: the old ones are copied over on the GPU
    const int oldCapacity = g.capacity;
    g.capacity *= 2;
    std::unique_ptr<QRhiTexture> old = std::move(g.heights);
    g.heights.reset(m_rhi->newTextureArray(m_heightFormat, g.capacity, QSize(g.size, g.size), 1, QRhiTexture::UsedAsTransferSource));
    g.heights->create();
    for (const auto &[key, c] : m_chunks) {
        if (c.grid != g.size || c.layer < 0) continue;
        QRhiTextureCopyDescription d;
        d.setSourceLayer(c.layer);
        d.setDestinationLayer(c.layer);
        u->copyTexture(g.heights.get(), old.get(), d);
    }
    retire(old);
    retireAll(g.srbs);
    for (int i = g.capacity - 1; i >= oldCapacity; --i) g.freeLayers.push_back(i);
    LOG_I("gpu") << "terrain grid" << g.size << "grown to" << g.capacity << "layers";
}

bool TerrainRenderer::allocateLayer(ChunkSat &s)
{
    const int size = s.image->size;
    const bool bc1 = s.image->bc1;
    auto take = [&s](int key, SatArray &a) {
        s.array = key;
        s.layer = a.freeLayers.back();
        a.freeLayers.pop_back();
        return true;
    };
    // A page with room, else the first page not there
    const int first = satKey(size, bc1, 0);
    int unused = first;
    for (auto it = m_sat.lower_bound(first); it != m_sat.end() && it->first < first + kMaxPages; ++it) {
        if (!it->second.freeLayers.empty()) return take(it->first, it->second);
        if (it->first == unused) ++unused;
    }
    if (unused >= first + kMaxPages) return false;
    SatArray &a = m_sat[unused];
    a.size = size;
    a.bc1 = bc1;
    a.layers = std::min(m_maxLayers, pageLayers(size, bc1));
    a.tex.reset(m_rhi->newTextureArray(bc1 ? QRhiTexture::BC1 : QRhiTexture::RGBA8, a.layers, QSize(size, size), 1, QRhiTexture::MipMapped));
    if (!a.tex->create()) {
        if (DebugLog::throttle(u"sat-pages"_s, 30000) >= 0) LOG_W("gpu") << "no memory for a satellite page of" << size << "px";
        m_sat.erase(unused);
        return false;
    }
    for (int i = a.layers - 1; i >= 0; --i) a.freeLayers.push_back(i);
    LOG_D("gpu") << "satellite page" << size << "x" << size << ":" << a.layers << "layers," << m_sat.size() << "pages";
    return take(unused, a);
}

void TerrainRenderer::releaseLayer(ChunkSat &s)
{
    if (s.layer < 0) return;
    auto it = m_sat.find(s.array);
    if (it != m_sat.end()) it->second.freeLayers.push_back(s.layer);
    s.layer = -1;
}

void TerrainRenderer::removeImage(quint64 chunk)
{
    cancelUpload(chunk);
    auto it = m_images.find(chunk);
    if (it == m_images.end()) return;
    m_imageBytes -= it->second.image->bytes();
    releaseLayer(it->second);
    m_images.erase(it);
}

void TerrainRenderer::cancelUpload(quint64 chunk)
{
    std::erase_if(m_uploads, [&](SatUpload &up) {
        if (up.sat.image->chunk != chunk) return false;
        releaseLayer(up.sat);
        return true;
    });
}

void TerrainRenderer::addImage(const std::shared_ptr<const ChunkImage> &img)
{
    if (!img || img->mips.empty() || (img->bc1 && !m_bc1)) return;
    cancelUpload(img->chunk);
    SatUpload up;
    up.sat.image = img;
    if (!allocateLayer(up.sat)) return;
    up.level = std::min(int(img->mips.size()), m_rhi->mipLevelsForSize(QSize(img->size, img->size))) - 1;
    m_uploads.push_back(std::move(up));
}

void TerrainRenderer::pumpUploads(QRhiResourceUpdateBatch *u)
{
    qsizetype budget = kImageBytesPerFrame;
    while (budget > 0 && !m_uploads.empty()) {
        SatUpload &up = m_uploads.front();
        const ChunkImage &img = *up.sat.image;
        const QByteArray &mip = img.mips[std::size_t(up.level)];
        const int w = std::max(1, img.size >> up.level), h = w;
        const qsizetype rb = rowBytes(w, img.bc1);
        const int rows = rowCount(h, img.bc1), n = int(std::clamp<qsizetype>(budget / rb, 1, rows - up.row));
        QRhiTextureSubresourceUploadDescription d;
        if (n == rows) {
            d.setData(mip);
        } else {
            // A strip. The image outlives the frame (m_uploads, then m_images), so its bytes need no copy
            const int px = img.bc1 ? 4 : 1;
            d.setData(QByteArray::fromRawData(mip.constData() + up.row * rb, n * rb));
            d.setSourceSize(QSize(w, std::min(n * px, h - up.row * px)));
            d.setDestinationTopLeft(QPoint(0, up.row * px));
        }
        u->uploadTexture(m_sat[up.sat.array].tex.get(), QRhiTextureUploadDescription(QRhiTextureUploadEntry(up.sat.layer, up.level, d)));
        budget -= n * rb;
        up.row += n;
        if (up.row < rows) continue;
        up.row = 0;
        if (--up.level >= 0) continue;
        // Complete: the chunk's old image (band change) was drawn until now
        const quint64 chunk = img.chunk;
        ChunkSat done = std::move(up.sat);
        m_uploads.pop_front();
        auto old = m_images.find(chunk);
        if (old != m_images.end()) {
            m_imageBytes -= old->second.image->bytes();
            releaseLayer(old->second);
        }
        m_imageBytes += done.image->bytes();
        m_images[chunk] = std::move(done);
    }
}

void TerrainRenderer::trimPages()
{
    // An empty page goes back when the other pages of its size have room for half a page
    for (auto it = m_sat.begin(); it != m_sat.end();) {
        const SatArray &a = it->second;
        std::size_t room = 0;
        if (int(a.freeLayers.size()) == a.layers)
            for (const auto &[key, b] : m_sat)
                if (key != it->first && b.size == a.size && b.bc1 == a.bc1) room += b.freeLayers.size();
        if (room == 0 || int(room) < a.layers / 2) {
            ++it;
            continue;
        }
        for (auto &[size, g] : m_grids) {
            auto srb = g.srbs.find(it->first);
            if (srb == g.srbs.end()) continue;
            retire(srb->second);
            g.srbs.erase(srb);
        }
        retire(it->second.tex);
        it = m_sat.erase(it);
    }
}

void TerrainRenderer::sync(TerrainChanges &changes, QRhiResourceUpdateBatch *u)
{
    if (!m_noSatReady) {
        // Never sampled (layer -1 picks the palette), but bound: one grey texel
        const quint32 grey = 0xff808080u;
        u->uploadTexture(m_noSat.get(), QRhiTextureUploadDescription(QRhiTextureUploadEntry(0, 0, QRhiTextureSubresourceUploadDescription(&grey, 4))));
        m_noSatReady = true;
    }

    std::vector<std::shared_ptr<const ChunkData>> added;
    std::vector<quint64> removed;
    std::vector<std::shared_ptr<const ChunkImage>> images;
    std::vector<quint64> imagesRemoved;
    bool clear = false, clearImages = false;
    changes.take(added, removed, clear, kMaxUploadsPerFrame, images, imagesRemoved, clearImages,
                 m_uploads.size() < kMaxQueuedImages ? kImageBytesPerFrame : 0);
    if (clear) {
        for (auto &kv : m_chunks) releaseChunk(kv.second);
        m_chunks.clear();
    }
    if (clear || clearImages) {
        for (SatUpload &up : m_uploads) releaseLayer(up.sat);
        m_uploads.clear();
        for (auto &kv : m_images) releaseLayer(kv.second);
        m_images.clear();
        m_imageBytes = 0;
    }
    for (quint64 k : removed) {
        auto it = m_chunks.find(k);
        if (it != m_chunks.end()) {
            releaseChunk(it->second);
            m_chunks.erase(it);
        }
        removeImage(k);   // the chunk's imagery goes with it
    }
    for (quint64 k : imagesRemoved) removeImage(k);
    for (auto &c : added) {
        Grid *g = gridFor(c->grid, u);
        const quint64 key = c->key.packed();
        Chunk &slot = m_chunks[key];
        releaseChunk(slot);
        if (g->freeLayers.empty()) growHeights(*g, u);
        slot.data = c;
        slot.grid = c->grid;
        slot.layer = g->freeLayers.back();
        g->freeLayers.pop_back();
        if (m_heightFormat == QRhiTexture::R16) {
            std::vector<quint16> px(c->heights.size());
            for (std::size_t i = 0; i < px.size(); ++i) px[i] = quint16(int(c->heights[i]) + 32768);
            QRhiTextureSubresourceUploadDescription d(px.data(), quint32(px.size() * 2));
            u->uploadTexture(g->heights.get(), QRhiTextureUploadDescription(QRhiTextureUploadEntry(slot.layer, 0, d)));
        } else {
            std::vector<float> px(c->heights.size());
            for (std::size_t i = 0; i < px.size(); ++i) px[i] = (float(c->heights[i]) + 32768.f) / 65535.f;
            QRhiTextureSubresourceUploadDescription d(px.data(), quint32(px.size() * 4));
            u->uploadTexture(g->heights.get(), QRhiTextureUploadDescription(QRhiTextureUploadEntry(slot.layer, 0, d)));
        }
    }
    for (const auto &img : images) addImage(img);
    pumpUploads(u);
    trimPages();
}

std::unique_ptr<QRhiShaderResourceBindings> TerrainRenderer::makeSrb(QRhiTexture *heights, QRhiTexture *sat)
{
    std::unique_ptr<QRhiShaderResourceBindings> p(m_rhi->newShaderResourceBindings());
    p->setBindings({
        QRhiShaderResourceBinding::uniformBuffer(0, QRhiShaderResourceBinding::VertexStage | QRhiShaderResourceBinding::FragmentStage, m_ubuf.get()),
        QRhiShaderResourceBinding::sampledTexture(1, QRhiShaderResourceBinding::VertexStage, heights, m_sampler.get()),
        QRhiShaderResourceBinding::sampledTexture(2, QRhiShaderResourceBinding::FragmentStage, sat, m_satSampler.get()),
        QRhiShaderResourceBinding::sampledTexture(3, QRhiShaderResourceBinding::FragmentStage, m_zoneCov.get(), m_zoneSampler.get()),
        QRhiShaderResourceBinding::sampledTexture(4, QRhiShaderResourceBinding::FragmentStage, m_zoneFloor.get(), m_zoneSampler.get()),
        QRhiShaderResourceBinding::sampledTexture(5, QRhiShaderResourceBinding::FragmentStage, m_detail[0] ? m_detail[0].get() : m_noDetail.get(), m_detailSampler.get()),
        QRhiShaderResourceBinding::sampledTexture(6, QRhiShaderResourceBinding::FragmentStage, m_detail[1] ? m_detail[1].get() : m_noDetail.get(), m_detailSampler.get()),
        QRhiShaderResourceBinding::sampledTexture(7, QRhiShaderResourceBinding::FragmentStage, m_detail[2] ? m_detail[2].get() : m_noDetail.get(), m_detailSampler.get()),
        QRhiShaderResourceBinding::sampledTexture(8, QRhiShaderResourceBinding::FragmentStage, m_detail[3] ? m_detail[3].get() : m_noDetail.get(), m_detailSampler.get()),
    });
    p->create();
    return p;
}

QRhiShaderResourceBindings *TerrainRenderer::srbFor(Grid &g, int key)
{
    auto &p = g.srbs[key];
    if (p) return p.get();
    QRhiTexture *sat = m_noSat.get();
    if (key) {
        auto it = m_sat.find(key);
        if (it != m_sat.end() && it->second.tex) sat = it->second.tex.get();
    }
    p = makeSrb(g.heights.get(), sat);
    return p.get();
}

void TerrainRenderer::ensurePipeline(Grid &g, QRhiRenderPassDescriptor *rp, int sampleCount)
{
    if (g.pipeline && m_rp == rp && m_sampleCount == sampleCount) return;
    retire(g.pipeline);
    g.pipeline.reset(m_rhi->newGraphicsPipeline());
    g.pipeline->setShaderStages({{QRhiShaderStage::Vertex, m_vs}, {QRhiShaderStage::Fragment, m_fs}});
    QRhiVertexInputLayout layout;
    layout.setBindings({{2 * sizeof(float)}, {sizeof(Instance), QRhiVertexInputBinding::PerInstance}});
    layout.setAttributes({{0, 0, QRhiVertexInputAttribute::Float2, 0},
                          {1, 1, QRhiVertexInputAttribute::Float4, 0},
                          {1, 2, QRhiVertexInputAttribute::Float2, 4 * sizeof(float)},
                          {1, 3, QRhiVertexInputAttribute::Float, 6 * sizeof(float)}});
    g.pipeline->setVertexInputLayout(layout);
    // Every binding set has this layout
    g.pipeline->setShaderResourceBindings(m_layoutSrb.get());
    g.pipeline->setRenderPassDescriptor(rp);
    g.pipeline->setSampleCount(sampleCount);
    g.pipeline->setDepthTest(true);
    g.pipeline->setDepthWrite(true);
    g.pipeline->setCullMode(QRhiGraphicsPipeline::None);
    g.pipeline->create();
}

void TerrainRenderer::prepare(const TerrainFrame &f, QRhiRenderPassDescriptor *rp, int sampleCount, QRhiResourceUpdateBatch *u)
{
    Ubuf ub{};
    std::memcpy(ub.viewProj, f.viewProj.constData(), sizeof(ub.viewProj));
    ub.camPos[0] = f.camPos.x(); ub.camPos[1] = f.camPos.y(); ub.camPos[2] = f.camPos.z(); ub.camPos[3] = f.fogDensity;
    ub.sunDir[0] = f.sunDir.x(); ub.sunDir[1] = f.sunDir.y(); ub.sunDir[2] = f.sunDir.z(); ub.sunDir[3] = f.sunlight ? 1.f : 0.f;
    ub.extra[0] = f.triGrid; ub.extra[1] = f.triCenter.x(); ub.extra[2] = f.triCenter.z(); ub.extra[3] = f.lightTheme ? 1.f : 0.f;
    ub.params[1] = f.brightness;
    ub.params[2] = f.schematicRadius;
    ub.params[3] = f.schematic ? 1.f : 0.f;
    ub.zone[0] = f.zoneMap.x(); ub.zone[1] = f.zoneMap.y(); ub.zone[2] = f.zoneMap.z(); ub.zone[3] = f.zoneMap.w();
    ub.zone2[0] = f.zonesOn && m_zonesReady ? 1.f : 0.f;
    for (int l = 0; l < SatelliteDetail::kLevels; ++l) {
        const QVector4D &m = f.detail.map[std::size_t(l)], &w = f.detail.win[std::size_t(l)];
        ub.detailMap[l][0] = m.x(); ub.detailMap[l][1] = m.y(); ub.detailMap[l][2] = m.z(); ub.detailMap[l][3] = m.w();
        ub.detailWin[l][0] = w.x(); ub.detailWin[l][1] = w.y(); ub.detailWin[l][2] = w.z(); ub.detailWin[l][3] = w.w();
        ub.detailOn[l] = f.detail.on[l] > 0.5f && m_detail[l] ? 1.f : 0.f;
    }
    u->updateDynamicBuffer(m_ubuf.get(), 0, sizeof(Ubuf), &ub);

    const auto planes = frustumPlanes(f.viewProj);
    const float lodPixelScale = 2.f * std::tan(f.fovDeg * float(M_PI) / 360.f) / std::max(1.f, f.viewportHeight);
    m_visible = 0;
    m_triangles = 0;

    for (auto &[size, g] : m_grids) {
        ensurePipeline(g, rp, sampleCount);
        // satellite array -> step -> instances
        std::map<int, std::array<std::vector<Instance>, 4>> groups;
        for (const auto &[key, c] : m_chunks) {
            if (c.grid != size || c.layer < 0) continue;
            const ChunkData &d = *c.data;
            const float w = float(d.dx * (d.grid - 1)), h = float(d.dz * (d.grid - 1));
            const QVector3D lo(float(d.x0), float(d.minH), float(d.z0));
            const QVector3D hi(float(d.x0) + w, float(d.maxH), float(d.z0) + h);
            if (!boxVisible(planes, lo, hi)) continue;
            // Distance to the nearest point of the box
            const QVector3D p = f.camPos;
            const QVector3D q(std::clamp(p.x(), lo.x(), hi.x()), std::clamp(p.y(), lo.y(), hi.y()), std::clamp(p.z(), lo.z(), hi.z()));
            const float dist = (p - q).length();
            const float mpp = std::max(1e-3f, dist) * lodPixelScale;
            // Ground spacing of the samples: 30.9 m for SRTM1, ~93 m for SRTM3
            const float spacing = std::max(kSrtm1SpacingM * 0.9f, float(d.dz));
            const float ideal = kLodTargetPixels * mpp / spacing;
            int step = 1;
            if (ideal > 1) step = std::min(8, 1 << int(std::floor(std::log2(ideal))));
            if ((size - 1) % step) step = 1;
            int sat = 0;
            float satLayer = -1;
            if (!f.schematic) {
                auto im = m_images.find(key);
                if (im != m_images.end() && im->second.layer >= 0) {
                    sat = im->second.array;
                    satLayer = float(im->second.layer);
                }
            }
            groups[sat][std::size_t(stepIndex(step))].push_back(
                {{float(d.x0), float(d.z0), float(d.dx), float(d.dz)}, float(c.layer), float(size - 1), satLayer, 0});
        }
        int total = 0;
        for (auto &[sat, steps] : groups)
            for (auto &gr : steps) total += int(gr.size());
        if (total > g.instanceCapacity) {
            g.instanceCapacity = std::max(256, total * 2);
            retire(g.instances);
            g.instances.reset(m_rhi->newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::VertexBuffer, quint32(g.instanceCapacity * sizeof(Instance))));
            g.instances->create();
        }
        g.draws.clear();
        std::vector<Instance> all;
        all.reserve(total);
        for (auto &[sat, steps] : groups) {
            srbFor(g, sat);   // binding sets are made outside the pass
            for (int si = 0; si < 4; ++si) {
                const auto &gr = steps[std::size_t(si)];
                if (gr.empty()) continue;
                g.draws.push_back({sat, si, int(all.size()), int(gr.size())});
                all.insert(all.end(), gr.begin(), gr.end());
                m_triangles += qint64(g.indexCount[si] / 3) * qint64(gr.size());
            }
        }
        m_visible += total;
        if (total) u->updateDynamicBuffer(g.instances.get(), 0, quint32(total * sizeof(Instance)), all.data());
    }
    m_rp = rp;
    m_sampleCount = sampleCount;
}

void TerrainRenderer::draw(QRhiCommandBuffer *cb, const QSize &outputSize)
{
    m_drawCalls = 0;
    for (auto &[size, g] : m_grids) {
        if (!g.pipeline || !g.instances || g.draws.empty()) continue;
        cb->setGraphicsPipeline(g.pipeline.get());
        cb->setViewport({0, 0, float(outputSize.width()), float(outputSize.height())});
        for (const Draw &d : g.draws) {
            cb->setShaderResources(g.srbs[d.sat].get());
            const QRhiCommandBuffer::VertexInput inputs[] = {{g.vbuf.get(), 0}, {g.instances.get(), quint32(d.first * sizeof(Instance))}};
            cb->setVertexInput(0, 2, inputs, g.ibuf[d.step].get(), 0, QRhiCommandBuffer::IndexUInt16);
            cb->drawIndexed(g.indexCount[d.step], quint32(d.count));
            ++m_drawCalls;
        }
    }
}

} // namespace corv
