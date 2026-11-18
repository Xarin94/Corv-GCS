#include "scene/SurfaceVolume.h"

#include <algorithm>
#include <cmath>
#include <optional>

namespace corv {

namespace {
constexpr int kBias = 32768;
constexpr int kTruncCells = 3;   // truncation band: +-3 voxels

inline int floorDiv(int a, int b) { return int(std::floor(double(a) / b)); }

// The 12 edges of a cube, corners numbered n = x + 2y + 4z
constexpr int kEdges[12][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}, {0, 2}, {1, 3}, {4, 6}, {5, 7}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
} // namespace

std::int64_t SurfaceVolume::chunkKey(int cx, int cy, int cz)
{
    return (std::int64_t(cx + kBias) * 65536 + (cy + kBias)) * 65536 + (cz + kBias);
}

SurfaceVolume::SurfaceVolume() { reset(); }

bool SurfaceVolume::configure(double cell, double memory, double maxChunks)
{
    bool relayout = false;
    if (std::isfinite(cell) && cell > 0 && cell != m_cell) {
        m_cell = cell;
        relayout = true;
    }
    if (std::isfinite(memory) && memory >= 1) m_memory = std::min(255, int(std::lround(memory)));
    if (std::isfinite(maxChunks) && maxChunks >= 8) m_maxChunks = int(std::lround(maxChunks));
    if (relayout) reset();
    return relayout;
}

void SurfaceVolume::reset()
{
    m_chunks.clear();
    m_dirty.clear();
    m_removed.clear();
}

SurfaceVolume::Chunk *SurfaceVolume::chunk(int cx, int cy, int cz, bool create)
{
    const std::int64_t key = chunkKey(cx, cy, cz);
    auto it = m_chunks.find(key);
    if (it != m_chunks.end()) return it->second.get();
    if (!create) return nullptr;
    auto c = std::make_unique<Chunk>();
    c->key = key;
    c->cx = cx;
    c->cy = cy;
    c->cz = cz;
    Chunk *p = c.get();
    m_chunks.emplace(key, std::move(c));
    return p;
}

const SurfaceVolume::Chunk *SurfaceVolume::findChunk(int cx, int cy, int cz) const
{
    auto it = m_chunks.find(chunkKey(cx, cy, cz));
    return it == m_chunks.end() ? nullptr : it->second.get();
}

// Distance of voxel (i, j, k); false if never observed
bool SurfaceVolume::voxel(int i, int j, int k, float *d) const
{
    const int cx = floorDiv(i, kChunk), cy = floorDiv(j, kChunk), cz = floorDiv(k, kChunk);
    const Chunk *c = findChunk(cx, cy, cz);
    if (!c) return false;
    const int p = ((k - cz * kChunk) * kChunk + (j - cy * kChunk)) * kChunk + (i - cx * kChunk);
    if (c->w[std::size_t(p)] == 0) return false;
    *d = c->d[std::size_t(p)];
    return true;
}

int SurfaceVolume::integrate(const double o[3], const double *enu, int count)
{
    const double v = m_cell, mu = kTruncCells * v, step = v / 2;
    const int mem = m_memory;
    int used = 0;
    for (int q = 0; q < count; ++q) {
        const double dx = enu[q * 3] - o[0], dy = enu[q * 3 + 1] - o[1], dz = enu[q * 3 + 2] - o[2];
        const double r = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (!(r > 1e-3)) continue;
        const double ux = dx / r, uy = dy / r, uz = dz / r;
        std::int64_t lastKey = -1;
        int lastP = -1;
        for (double t = std::max(0.0, r - mu); t <= r + mu; t += step) {
            const int i = int(std::floor((o[0] + ux * t) / v)), j = int(std::floor((o[1] + uy * t) / v)), k = int(std::floor((o[2] + uz * t) / v));
            const int cx = floorDiv(i, kChunk), cy = floorDiv(j, kChunk), cz = floorDiv(k, kChunk);
            Chunk *c = chunk(cx, cy, cz, true);
            const int li = i - cx * kChunk, lj = j - cy * kChunk, lk = k - cz * kChunk;
            const int p = (lk * kChunk + lj) * kChunk + li;
            if (c->key == lastKey && p == lastP) continue;
            lastKey = c->key;
            lastP = p;
            // Signed distance of the voxel centre along the ray
            const double along = ((i + 0.5) * v - o[0]) * ux + ((j + 0.5) * v - o[1]) * uy + ((k + 0.5) * v - o[2]) * uz;
            const double sdf = std::max(-1.0, std::min(1.0, (r - along) / mu));
            const int W = c->w[std::size_t(p)] + 1;
            c->d[std::size_t(p)] += float((sdf - c->d[std::size_t(p)]) / W);
            c->w[std::size_t(p)] = std::uint8_t(std::min(W, mem));
            touch(*c, li, lj, lk);
        }
        ++used;
    }
    if (int(m_chunks.size()) > m_maxChunks) evict(o);
    return used;
}

// A voxel on a chunk's face also changes the mesh of the chunk beyond that
// face: the cubes straddling the two chunks use voxels of both
void SurfaceVolume::touch(const Chunk &c, int li, int lj, int lk)
{
    m_dirty.insert(c.key);
    auto side = [](int l, int *out) {
        out[0] = 0;
        if (l == 0) { out[1] = -1; return 2; }
        if (l == kChunk - 1) { out[1] = 1; return 2; }
        return 1;
    };
    int sx[2], sy[2], sz[2];
    const int nx = side(li, sx), ny = side(lj, sy), nz = side(lk, sz);
    if (nx + ny + nz == 3) return;
    for (int a = 0; a < nx; ++a)
        for (int b = 0; b < ny; ++b)
            for (int d = 0; d < nz; ++d)
                if (sx[a] || sy[b] || sz[d]) m_dirty.insert(chunkKey(c.cx + sx[a], c.cy + sy[b], c.cz + sz[d]));
}

void SurfaceVolume::evict(const double o[3])
{
    const double span = kChunk * m_cell;
    std::vector<std::pair<Chunk *, double>> list;
    list.reserve(m_chunks.size());
    for (const auto &[key, c] : m_chunks) {
        const double dx = (c->cx + 0.5) * span - o[0], dy = (c->cy + 0.5) * span - o[1], dz = (c->cz + 0.5) * span - o[2];
        list.emplace_back(c.get(), dx * dx + dy * dy + dz * dz);
    }
    std::sort(list.begin(), list.end(), [](const auto &a, const auto &b) { return a.second < b.second; });
    for (std::size_t k = std::size_t(std::floor(m_maxChunks * 0.95)); k < list.size(); ++k) {
        const std::int64_t key = list[k].first->key;
        m_dirty.erase(key);
        m_removed.push_back(key);
        m_chunks.erase(key);
    }
}

std::vector<float> SurfaceVolume::meshChunk(const Chunk &c) const
{
    const double v = m_cell;
    const int i0 = c.cx * kChunk, j0 = c.cy * kChunk, k0 = c.cz * kChunk;
    constexpr int E = kChunk + 2;
    // Vertex of the cube whose lowest voxel is (i, j, k): 0 not computed, 1 none, 2 computed
    std::vector<std::uint8_t> state(std::size_t(E * E * E), 0);
    std::vector<std::array<float, 3>> verts(std::size_t(E * E * E));
    auto cubeVertex = [&](int i, int j, int k) -> const std::array<float, 3> * {
        const int key = ((i - i0 + 1) * E + (j - j0 + 1)) * E + (k - k0 + 1);
        if (key < 0 || key >= E * E * E) return nullptr;
        if (state[std::size_t(key)] == 1) return nullptr;
        if (state[std::size_t(key)] == 2) return &verts[std::size_t(key)];
        float corner[8];
        int mask = 0;
        bool ok = true;
        for (int n = 0; n < 8; ++n) {
            if (!voxel(i + (n & 1), j + ((n >> 1) & 1), k + ((n >> 2) & 1), &corner[n])) {
                ok = false;
                break;
            }
            if (corner[n] < 0) mask |= 1 << n;
        }
        if (!ok || mask == 0 || mask == 255) {
            state[std::size_t(key)] = 1;
            return nullptr;
        }
        double sx = 0, sy = 0, sz = 0;
        int m = 0;
        for (const auto &e : kEdges) {
            const int a = e[0], b = e[1];
            const float da = corner[a], db = corner[b];
            if ((da < 0) == (db < 0)) continue;
            const double t = da / (da - db);
            sx += (a & 1) + t * ((b & 1) - (a & 1));
            sy += ((a >> 1) & 1) + t * (((b >> 1) & 1) - ((a >> 1) & 1));
            sz += ((a >> 2) & 1) + t * (((b >> 2) & 1) - ((a >> 2) & 1));
            ++m;
        }
        verts[std::size_t(key)] = {float((i + 0.5 + sx / m) * v), float((j + 0.5 + sy / m) * v), float((k + 0.5 + sz / m) * v)};
        state[std::size_t(key)] = 2;
        return &verts[std::size_t(key)];
    };
    std::vector<float> tris;
    auto quad = [&](const std::array<float, 3> *a, const std::array<float, 3> *b, const std::array<float, 3> *cc, const std::array<float, 3> *d) {
        if (!a || !b || !cc || !d) return;
        for (const auto *p : {a, b, cc, a, cc, d}) tris.insert(tris.end(), p->begin(), p->end());
    };
    // Every voxel edge of this chunk along +x, +y, +z that changes sign
    for (int k = k0; k < k0 + kChunk; ++k)
        for (int j = j0; j < j0 + kChunk; ++j)
            for (int i = i0; i < i0 + kChunk; ++i) {
                float d0, dxv, dyv, dzv;
                if (!voxel(i, j, k, &d0)) continue;
                if (voxel(i + 1, j, k, &dxv) && (d0 < 0) != (dxv < 0))
                    quad(cubeVertex(i, j - 1, k - 1), cubeVertex(i, j, k - 1), cubeVertex(i, j, k), cubeVertex(i, j - 1, k));
                if (voxel(i, j + 1, k, &dyv) && (d0 < 0) != (dyv < 0))
                    quad(cubeVertex(i - 1, j, k - 1), cubeVertex(i, j, k - 1), cubeVertex(i, j, k), cubeVertex(i - 1, j, k));
                if (voxel(i, j, k + 1, &dzv) && (d0 < 0) != (dzv < 0))
                    quad(cubeVertex(i - 1, j - 1, k), cubeVertex(i, j - 1, k), cubeVertex(i, j, k), cubeVertex(i - 1, j, k));
            }
    return tris;
}

SurfaceVolume::Changes SurfaceVolume::takeChanges()
{
    Changes out;
    for (const std::int64_t key : m_dirty) {
        auto it = m_chunks.find(key);
        if (it == m_chunks.end()) continue;
        const Chunk &c = *it->second;
        out.meshes.push_back({key, c.cx, c.cy, c.cz, meshChunk(c)});
    }
    out.removed = std::move(m_removed);
    m_dirty.clear();
    m_removed.clear();
    return out;
}

SurfaceVolume::Changes SurfaceVolume::takeAll()
{
    for (const auto &[key, c] : m_chunks) m_dirty.insert(key);
    m_removed.clear();
    return takeChanges();
}

} // namespace corv
