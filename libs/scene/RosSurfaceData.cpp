#include "scene/RosSurfaceData.h"

#include "scene/SurfaceRaster.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace corv {

namespace {
constexpr int kT = SurfaceTiles::kTile;
constexpr double kRoughSigmas = 3;        // full red this many standard deviations above the mean roughness
constexpr double kRoughMinSpan = 0.05;    // m: ... and at least this far above it
constexpr double kSpreadMinSpan = 0.02;   // 1 - |mean normal|

bool sameAnchor(const std::optional<RosAnchor> &a, const std::optional<RosAnchor> &b)
{
    return a && b && a->epoch == b->epoch;
}

double boxDistance(double x, double y, double z, double x0, double y0, double z0, double x1, double y1, double z1)
{
    const double dx = std::max({x0 - x, 0.0, x - x1}), dy = std::max({y0 - y, 0.0, y - y1}), dz = std::max({z0 - z, 0.0, z - z1});
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}
} // namespace

RosSurfaceData &RosSurfaceData::instance()
{
    static RosSurfaceData d;
    return d;
}

void RosSurfaceData::clear()
{
    blocks.clear();
    tiles.clear();
    anchor.reset();
    ++generation;
    chunks.clear();
    volAnchor.reset();
    ++volGeneration;
}

void RosSurfaceData::applyTiles(bool reset, const std::optional<RosAnchor> &a, double c, const std::vector<TileUpdate> &upd,
                                const std::vector<std::pair<int, int>> &removed)
{
    if (reset || !sameAnchor(a, anchor) || c != cell) {
        blocks.clear();
        tiles.clear();
        anchor.reset();
        ++generation;
    }
    anchor = a;
    cell = c;
    if (!anchor) return;
    for (const auto &[tx, ty] : removed) removeTile(tx, ty);
    for (const TileUpdate &t : upd) {
        TileRec rec;
        rec.tx = t.tx;
        rec.ty = t.ty;
        rec.h = t.h;
        rec.w = t.w;
        float minH = std::numeric_limits<float>::infinity(), maxH = -std::numeric_limits<float>::infinity();
        for (float v : t.h)
            if (!std::isnan(v)) {
                minH = std::min(minH, v);
                maxH = std::max(maxH, v);
            }
        rec.minH = minH;
        rec.maxH = maxH;
        const std::int64_t key = SurfaceTiles::tileKey(t.tx, t.ty);
        tiles[key] = std::move(rec);
        const int bx = int(std::floor(double(t.tx) / kBlockTiles)), by = int(std::floor(double(t.ty) / kBlockTiles));
        auto it = blocks.find(SurfaceTiles::tileKey(bx, by));
        Block *b = it != blocks.end() ? it->second.get() : createBlock(bx, by);
        b->tiles.insert(key);
        writeTile(t.tx, t.ty, &tiles[key]);
    }
}

void RosSurfaceData::removeTile(int tx, int ty)
{
    const std::int64_t key = SurfaceTiles::tileKey(tx, ty);
    if (!tiles.erase(key)) return;
    writeTile(tx, ty, nullptr);
    const std::int64_t bk = SurfaceTiles::tileKey(int(std::floor(double(tx) / kBlockTiles)), int(std::floor(double(ty) / kBlockTiles)));
    auto it = blocks.find(bk);
    if (it != blocks.end()) {
        it->second->tiles.erase(key);
        if (it->second->tiles.empty()) blocks.erase(it);
    }
}

// Copy a tile's heights and weights (null: empty) into its block and into
// the aprons of the blocks west / south that end on its first column / row
void RosSurfaceData::writeTile(int tx, int ty, const TileRec *t)
{
    const int gi0 = tx * kT, gj0 = ty * kT;
    const int bx = int(std::floor(double(gi0) / kBlock)), by = int(std::floor(double(gj0) / kBlock));
    static const int offs[4][2] = {{0, 0}, {-1, 0}, {0, -1}, {-1, -1}};
    for (const auto &o : offs) {
        auto it = blocks.find(SurfaceTiles::tileKey(bx + o[0], by + o[1]));
        if (it == blocks.end()) continue;
        Block &b = *it->second;
        const int li0 = gi0 - (bx + o[0]) * kBlock, lj0 = gj0 - (by + o[1]) * kBlock;
        const int i1 = std::min(kT, kTex - li0), j1 = std::min(kT, kTex - lj0);
        if (i1 <= 0 || j1 <= 0) continue;
        for (int j = 0; j < j1; ++j) {
            const int row = (lj0 + j) * kTex + li0;
            for (int i = 0; i < i1; ++i) {
                const float v = t ? t->h[std::size_t(j * kT + i)] : std::numeric_limits<float>::quiet_NaN();
                const bool ok = !std::isnan(v);
                b.data[std::size_t(row + i)] = ok ? v : raster::kEmpty;
                b.wts[std::size_t(row + i)] = ok ? (t->w.empty() ? 1.f : t->w[std::size_t(j * kT + i)]) : 0.f;
            }
        }
        b.dirty = true;
    }
}

RosSurfaceData::Block *RosSurfaceData::createBlock(int bx, int by)
{
    auto b = std::make_unique<Block>();
    b->bx = bx;
    b->by = by;
    b->data.assign(std::size_t(kTex) * kTex, raster::kEmpty);
    b->wts.assign(std::size_t(kTex) * kTex, 0.f);
    b->view.assign(std::size_t(kTex) * kTex * 2, 0.f);
    b->rough.assign(std::size_t(kTex) * kTex, 0);
    Block *p = b.get();
    blocks.emplace(SurfaceTiles::tileKey(bx, by), std::move(b));
    // The aprons: first column / row of the tiles east and north, and the corner
    static const int aprons[3][2] = {{kBlockTiles, 0}, {0, kBlockTiles}, {kBlockTiles, kBlockTiles}};
    for (const auto &o : aprons) {
        const int n = o[0] && o[1] ? 1 : kBlockTiles;
        for (int k = 0; k < n; ++k) {
            const int tx = bx * kBlockTiles + (o[0] ? o[0] : k), ty = by * kBlockTiles + (o[1] ? o[1] : k);
            auto it = tiles.find(SurfaceTiles::tileKey(tx, ty));
            if (it != tiles.end()) writeTile(tx, ty, &it->second);
        }
    }
    return p;
}

void RosSurfaceData::fillBlock(Block &b)
{
    raster::Options o;
    o.minSamples = view.minSamples;
    o.cell = cell;
    o.fillLevels = raster::fillLevelsFor(view.fillM, cell);
    const auto lc = raster::rasterize(b.data.data(), b.wts.data(), kTex, kTex, b.view.data(), o);
    std::copy(lc.begin(), lc.end(), b.levelCount.begin());
    // Roughness only while it is shown; switching to it refills the blocks
    b.roughDone = view.mode == Rough;
    if (b.roughDone) {
        raster::roughness(b.view.data(), kTex, kTex, b.rough.data(), cell);
        // The block's share of the mean and spread the colours are scaled to (cm)
        double n = 0, sum = 0, sq = 0;
        for (int p = 0; p < kTex * kTex; ++p) {
            if (!(b.view[std::size_t(p) * 2] > raster::kEmpty)) continue;
            const double r = b.rough[std::size_t(p)];
            ++n;
            sum += r;
            sq += r * r;
        }
        b.roughN = n;
        b.roughSum = sum;
        b.roughSq = sq;
    }
    float minH = std::numeric_limits<float>::infinity(), maxH = -std::numeric_limits<float>::infinity();
    for (const std::int64_t key : b.tiles) {
        auto it = tiles.find(key);
        if (it == tiles.end() || !(it->second.maxH >= it->second.minH)) continue;
        minH = std::min(minH, it->second.minH);
        maxH = std::max(maxH, it->second.maxH);
    }
    if (minH > maxH) minH = maxH = 0;
    b.minH = minH;
    b.maxH = maxH;
    b.dirty = false;
    static int revision = 0;   // unique across blocks: a block dropped and made again is uploaded again
    b.revision = ++revision;
}

void RosSurfaceData::fillDirty()
{
    for (auto &[key, b] : blocks)
        if (b->dirty) fillBlock(*b);
}

void RosSurfaceData::markAllDirty()
{
    for (auto &[key, b] : blocks) b->dirty = true;
}

void RosSurfaceData::setMode(int mode)
{
    if (mode == view.mode) return;
    view.mode = mode;
    if (mode == Rough)
        for (auto &[key, b] : blocks)
            if (!b->roughDone) b->dirty = true;
}

void RosSurfaceData::updateRoughScale()
{
    double n = 0, sum = 0, sq = 0;
    for (const auto &[key, b] : blocks) {
        if (!b->roughDone || !b->roughN) continue;
        n += b->roughN;
        sum += b->roughSum;
        sq += b->roughSq;
    }
    if (!n) return;
    const double mean = sum / n, sd = std::sqrt(std::max(0.0, sq / n - mean * mean));
    view.roughMean = float(mean / 100);
    view.roughFull = float((mean + std::max(kRoughSigmas * sd, kRoughMinSpan * 100)) / 100);
}

void RosSurfaceData::updateSpreadScale()
{
    double n = 0, sum = 0, sq = 0;
    for (const auto &[key, c] : chunks) {
        n += c->n;
        sum += c->sum;
        sq += c->sq;
    }
    if (!n) return;
    const double mean = sum / n, sd = std::sqrt(std::max(0.0, sq / n - mean * mean));
    view.spreadMean = float(mean);
    view.spreadFull = float(mean + std::max(kRoughSigmas * sd, kSpreadMinSpan));
}

void RosSurfaceData::updateScales(double vehE, double vehN, double vehU)
{
    if (view.mode == Rough) updateRoughScale();
    if (view.mode != Distance) return;
    double nearM = std::numeric_limits<double>::infinity();
    // Height field: the nearest tile's bounding box
    if (anchor) {
        const double span = kT * cell;
        for (const auto &[key, t] : tiles) {
            if (!(t.maxH >= t.minH)) continue;
            nearM = std::min(nearM, boxDistance(vehE, vehN, vehU, t.tx * span, t.ty * span, t.minH, (t.tx + 1) * span, (t.ty + 1) * span, t.maxH));
        }
    }
    // Volume: the nearest vertex, among the chunks whose bounds come closest
    if (volAnchor) {
        for (const auto &[key, c] : chunks) {
            const double dc = std::sqrt((c->cx - vehE) * (c->cx - vehE) + (c->cy - vehN) * (c->cy - vehN) + (c->cz - vehU) * (c->cz - vehU));
            if (dc - c->radius > nearM) continue;
            for (std::size_t k = 0; k + 2 < c->pos.size(); k += 3) {
                const double dx = c->pos[k] - vehE, dy = c->pos[k + 1] - vehN, dz = c->pos[k + 2] - vehU;
                nearM = std::min(nearM, std::sqrt(dx * dx + dy * dy + dz * dz));
            }
        }
    }
    if (!std::isfinite(nearM)) return;
    view.nearM = float(nearM);
    view.farM = float(nearM + std::max(20.0, 2 * nearM));
}

RosSurfaceData::Stats RosSurfaceData::stats() const
{
    Stats s;
    s.tiles = int(tiles.size());
    s.blocks = int(blocks.size());
    for (const auto &[key, b] : blocks)
        for (int k = 0; k < 5; ++k) s.levels[std::size_t(k)] += b->levelCount[std::size_t(k)];
    return s;
}

int RosSurfaceData::volumeTriangles() const
{
    std::size_t n = 0;
    for (const auto &[key, c] : chunks) n += c->pos.size() / 9;
    return int(n);
}

void RosSurfaceData::applyVolume(bool reset, const std::optional<RosAnchor> &a, double c, const std::vector<VolumeMesh> &meshes,
                                 const std::vector<std::int64_t> &removed)
{
    if (reset || !sameAnchor(a, volAnchor) || c != volCell) {
        chunks.clear();
        volAnchor.reset();
        ++volGeneration;
    }
    volAnchor = a;
    volCell = c;
    if (!volAnchor) return;
    for (const std::int64_t key : removed) chunks.erase(key);
    for (const VolumeMesh &m : meshes) {
        chunks.erase(m.key);
        if (m.pos.empty()) continue;
        auto ch = std::make_shared<VolChunk>();
        ch->pos = m.pos;
        ch->rough = normalSpread(m.pos);
        static int revision = 0;
        ch->revision = ++revision;
        // Bounding sphere: the centre of the bounds and the farthest vertex
        float lo[3] = {1e30f, 1e30f, 1e30f}, hi[3] = {-1e30f, -1e30f, -1e30f};
        for (std::size_t k = 0; k + 2 < m.pos.size(); k += 3)
            for (int d = 0; d < 3; ++d) {
                lo[d] = std::min(lo[d], m.pos[k + std::size_t(d)]);
                hi[d] = std::max(hi[d], m.pos[k + std::size_t(d)]);
            }
        ch->cx = (lo[0] + hi[0]) / 2;
        ch->cy = (lo[1] + hi[1]) / 2;
        ch->cz = (lo[2] + hi[2]) / 2;
        ch->radius = std::sqrt((hi[0] - lo[0]) * (hi[0] - lo[0]) + (hi[1] - lo[1]) * (hi[1] - lo[1]) + (hi[2] - lo[2]) * (hi[2] - lo[2])) / 2;
        for (float r : ch->rough) {
            ch->sum += r;
            ch->sq += double(r) * r;
        }
        ch->n = double(ch->rough.size());
        chunks[m.key] = std::move(ch);
    }
    updateSpreadScale();
}

std::vector<float> RosSurfaceData::normalSpread(const std::vector<float> &pos)
{
    const std::size_t n = pos.size() / 3;
    struct Key {
        std::uint32_t x, y, z;
        bool operator==(const Key &o) const { return x == o.x && y == o.y && z == o.z; }
    };
    struct Hash {
        std::size_t operator()(const Key &k) const { return (std::size_t(k.x) * 73856093u) ^ (std::size_t(k.y) * 19349663u) ^ (std::size_t(k.z) * 83492791u); }
    };
    auto bits = [](float f) {
        if (f == 0) f = 0;   // -0 and +0 are one position
        std::uint32_t u;
        std::memcpy(&u, &f, 4);
        return u;
    };
    std::unordered_map<Key, int, Hash> index;
    std::vector<int> ids(n);
    int u = 0;
    for (std::size_t v = 0; v < n; ++v) {
        const Key key{bits(pos[v * 3]), bits(pos[v * 3 + 1]), bits(pos[v * 3 + 2])};
        auto [it, inserted] = index.emplace(key, u);
        if (inserted) ++u;
        ids[v] = it->second;
    }
    std::vector<double> sum(std::size_t(u) * 3, 0.0), area(std::size_t(u), 0.0);
    for (std::size_t t = 0; t + 2 < n; t += 3) {
        const double ax = pos[t * 3], ay = pos[t * 3 + 1], az = pos[t * 3 + 2];
        const double ux = pos[t * 3 + 3] - ax, uy = pos[t * 3 + 4] - ay, uz = pos[t * 3 + 5] - az;
        const double vx = pos[t * 3 + 6] - ax, vy = pos[t * 3 + 7] - ay, vz = pos[t * 3 + 8] - az;
        const double cx = uy * vz - uz * vy, cy = uz * vx - ux * vz, cz = ux * vy - uy * vx;
        const double a = std::sqrt(cx * cx + cy * cy + cz * cz);   // 2 x area; (cx, cy, cz) = normal x 2 x area
        if (!(a > 0)) continue;
        for (int k = 0; k < 3; ++k) {
            const std::size_t id = std::size_t(ids[t + std::size_t(k)]);
            sum[id * 3] += cx;
            sum[id * 3 + 1] += cy;
            sum[id * 3 + 2] += cz;
            area[id] += a;
        }
    }
    std::vector<float> out(n);
    for (std::size_t v = 0; v < n; ++v) {
        const std::size_t id = std::size_t(ids[v]);
        out[v] = area[id] > 0 ? float(1 - std::sqrt(sum[id * 3] * sum[id * 3] + sum[id * 3 + 1] * sum[id * 3 + 1] + sum[id * 3 + 2] * sum[id * 3 + 2]) / area[id]) : 0.f;
    }
    return out;
}

} // namespace corv
