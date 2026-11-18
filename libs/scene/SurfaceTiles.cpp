#include "scene/SurfaceTiles.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace corv {

namespace {
constexpr int kBias = 32768;   // tile coordinates +-32 768 tiles (+-295 km at 30 cm)
constexpr int kCells = SurfaceTiles::kTile * SurfaceTiles::kTile;

bool inPolygon(double x, double y, const std::vector<QPointF> &poly)
{
    bool inside = false;
    for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
        const double xi = poly[i].x(), yi = poly[i].y(), xj = poly[j].x(), yj = poly[j].y();
        if ((yi > y) != (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi) inside = !inside;
    }
    return inside;
}
} // namespace

std::int64_t SurfaceTiles::tileKey(int tx, int ty) { return std::int64_t(tx + kBias) * 65536 + (ty + kBias); }

SurfaceTiles::SurfaceTiles() { reset(); }

bool SurfaceTiles::configure(double cell, double memory, double maxTiles)
{
    bool relayout = false;
    if (std::isfinite(cell) && cell > 0 && cell != m_cell) {
        m_cell = cell;
        relayout = true;
    }
    if (std::isfinite(memory) && memory >= 1) m_memory = memory;
    if (std::isfinite(maxTiles) && maxTiles >= 1) m_maxTiles = int(std::lround(maxTiles));
    if (relayout) reset();
    return relayout;
}

void SurfaceTiles::reset()
{
    m_tiles.clear();
    m_dirty.clear();
    m_removed.clear();
    m_filled = 0;
    ++m_version;
}

SurfaceTiles::Tile *SurfaceTiles::tileAt(int tx, int ty)
{
    const std::int64_t key = tileKey(tx, ty);
    auto it = m_tiles.find(key);
    if (it != m_tiles.end()) return it->second.get();
    auto t = std::make_unique<Tile>();
    t->key = key;
    t->tx = tx;
    t->ty = ty;
    Tile *p = t.get();
    m_tiles.emplace(key, std::move(t));
    return p;
}

int SurfaceTiles::add(const double *enu, int count, double vx, double vy)
{
    const double c = m_cell;
    if (int(m_sum.size()) < count) {
        m_sum.resize(std::size_t(count));
        m_cnt.resize(std::size_t(count));
        m_idx.resize(std::size_t(count));
        m_slotTile.resize(std::size_t(count));
    }
    int ns = 0, used = 0;
    m_slots.clear();
    for (int k = 0; k < count; ++k) {
        const double u = enu[k * 3 + 2];
        if (!std::isfinite(u)) continue;
        const int gi = int(std::floor(enu[k * 3] / c)), gj = int(std::floor(enu[k * 3 + 1] / c));
        const int tx = int(std::floor(double(gi) / kTile)), ty = int(std::floor(double(gj) / kTile));
        const int idx = (gj - ty * kTile) * kTile + (gi - tx * kTile);
        const std::int64_t key = tileKey(tx, ty) * 1024 + idx;
        auto it = m_slots.find(key);
        int s;
        if (it == m_slots.end()) {
            s = ns++;
            m_slots.emplace(key, s);
            m_sum[std::size_t(s)] = 0;
            m_cnt[std::size_t(s)] = 0;
            m_idx[std::size_t(s)] = idx;
            m_slotTile[std::size_t(s)] = tileAt(tx, ty);
        } else {
            s = it->second;
        }
        m_sum[std::size_t(s)] += u;
        ++m_cnt[std::size_t(s)];
        ++used;
    }
    const double mem = m_memory;
    for (int s = 0; s < ns; ++s) {
        Tile *t = m_slotTile[std::size_t(s)];
        const int i = m_idx[std::size_t(s)];
        const double n = m_cnt[std::size_t(s)];
        const double wObs = n / (n + 2);
        const double W = t->w[std::size_t(i)] + wObs;
        if (t->w[std::size_t(i)] == 0) {
            t->h[std::size_t(i)] = 0;
            ++t->filled;
            ++m_filled;
        }
        t->h[std::size_t(i)] += (m_sum[std::size_t(s)] / n - t->h[std::size_t(i)]) * wObs / W;
        t->w[std::size_t(i)] = float(std::min(W, mem));
        m_dirty.insert(t->key);
    }
    if (ns) ++m_version;
    if (int(m_tiles.size()) > m_maxTiles) evict(vx, vy);
    return used;
}

SurfaceTiles::Changes SurfaceTiles::takeChanges()
{
    Changes out;
    out.tiles.reserve(m_dirty.size());
    for (const std::int64_t key : m_dirty) {
        auto it = m_tiles.find(key);
        if (it == m_tiles.end()) continue;
        const Tile &t = *it->second;
        TileUpdate u;
        u.tx = t.tx;
        u.ty = t.ty;
        u.h.resize(kCells);
        u.w.resize(kCells);
        for (int k = 0; k < kCells; ++k) {
            u.h[std::size_t(k)] = t.w[std::size_t(k)] > 0 ? float(t.h[std::size_t(k)]) : std::numeric_limits<float>::quiet_NaN();
            u.w[std::size_t(k)] = t.w[std::size_t(k)];
        }
        out.tiles.push_back(std::move(u));
    }
    out.removed = std::move(m_removed);
    m_dirty.clear();
    m_removed.clear();
    return out;
}

SurfaceTiles::Changes SurfaceTiles::takeAll()
{
    for (const auto &[key, t] : m_tiles) m_dirty.insert(key);
    m_removed.clear();
    return takeChanges();
}

// Over budget: drop the tiles farthest from the vehicle, 5 % below the
// budget so this does not run on every message
void SurfaceTiles::evict(double vx, double vy)
{
    const double span = kTile * m_cell;
    std::vector<std::pair<Tile *, double>> list;
    list.reserve(m_tiles.size());
    for (const auto &[key, t] : m_tiles) {
        const double dx = (t->tx + 0.5) * span - vx, dy = (t->ty + 0.5) * span - vy;
        list.emplace_back(t.get(), dx * dx + dy * dy);
    }
    std::sort(list.begin(), list.end(), [](const auto &a, const auto &b) { return a.second < b.second; });
    const std::size_t keep = std::size_t(std::floor(m_maxTiles * 0.95));
    for (std::size_t k = keep; k < list.size(); ++k) {
        Tile *t = list[k].first;
        m_filled -= t->filled;
        m_removed.emplace_back(t->tx, t->ty);
        m_dirty.erase(t->key);
        m_tiles.erase(t->key);
    }
}

double polygonAreaXY(const std::vector<QPointF> &poly)
{
    if (poly.size() < 3) return 0;
    double a = 0;
    for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) a += poly[j].x() * poly[i].y() - poly[i].x() * poly[j].y();
    return std::abs(a) / 2;
}

double coveredArea(const SurfaceTiles &surface, const std::vector<QPointF> &poly, int block)
{
    if (poly.size() < 3) return 0;
    const int T = SurfaceTiles::kTile;
    const double c = surface.cell(), span = T * c, cell2 = c * c;
    double x0 = 1e300, y0 = 1e300, x1 = -1e300, y1 = -1e300;
    for (const QPointF &p : poly) {
        x0 = std::min(x0, p.x());
        y0 = std::min(y0, p.y());
        x1 = std::max(x1, p.x());
        y1 = std::max(y1, p.y());
    }
    if (block > 1) {
        std::unordered_set<std::int64_t> seen;
        const double bs = block * c;
        int n = 0;
        for (const auto &[key, t] : surface.tiles()) {
            if (!t->filled) continue;
            const double tx0 = t->tx * span, ty0 = t->ty * span;
            if (tx0 + span < x0 - bs || tx0 > x1 + bs || ty0 + span < y0 - bs || ty0 > y1 + bs) continue;
            for (int k = 0; k < T * T; ++k) {
                if (!(t->w[std::size_t(k)] > 0)) continue;
                const int i = k % T, j = k / T;
                const std::int64_t bi = std::int64_t(std::floor(double(t->tx * T + i) / block));
                const std::int64_t bj = std::int64_t(std::floor(double(t->ty * T + j) / block));
                const std::int64_t bkey = bi * 1048576 + bj;
                if (!seen.insert(bkey).second) continue;
                if (inPolygon((double(bi) + 0.5) * bs, (double(bj) + 0.5) * bs, poly)) ++n;
            }
        }
        return n * bs * bs;
    }
    double covered = 0;
    for (const auto &[key, t] : surface.tiles()) {
        if (!t->filled) continue;
        const double tx0 = t->tx * span, ty0 = t->ty * span, tx1 = tx0 + span, ty1 = ty0 + span;
        if (tx1 < x0 || tx0 > x1 || ty1 < y0 || ty0 > y1) continue;
        const bool corners = inPolygon(tx0, ty0, poly) && inPolygon(tx1, ty0, poly) && inPolygon(tx1, ty1, poly) && inPolygon(tx0, ty1, poly);
        const bool vertexInside = std::any_of(poly.begin(), poly.end(), [&](const QPointF &p) {
            return p.x() > tx0 && p.x() < tx1 && p.y() > ty0 && p.y() < ty1;
        });
        if (corners && !vertexInside) {
            covered += t->filled * cell2;
            continue;
        }
        for (int k = 0; k < T * T; ++k) {
            if (!(t->w[std::size_t(k)] > 0)) continue;
            const int i = k % T, j = k / T;
            if (inPolygon(tx0 + (i + 0.5) * c, ty0 + (j + 0.5) * c, poly)) covered += cell2;
        }
    }
    return covered;
}

} // namespace corv
