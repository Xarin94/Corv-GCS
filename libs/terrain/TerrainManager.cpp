#include "terrain/TerrainManager.h"

#include "core/DebugLog.h"
#include "terrain/Hgt.h"
#include "terrain/HgtDownloader.h"

#include <QSet>
#include <algorithm>
#include <cmath>

namespace corv {

TerrainManager::TerrainManager(QObject *parent) : QObject(parent)
{
    // A tile arriving makes its chunks buildable
    connect(&HgtStore::instance(), &HgtStore::tileLoaded, this, [this] { m_dirty = true; });
    // A tile downloaded (autoDownloadSRTM): HgtStore forgot it was missing, read it now
    connect(&HgtDownloader::instance(), &HgtDownloader::tileDone, this, [this](int, int, int result) {
        if (result == HgtDownloader::Downloaded) m_dirty = true;
    });
}

void TerrainManager::setRadius(double metres)
{
    if (metres == m_radius) return;
    m_radius = metres;
    m_dirty = true;
}

std::shared_ptr<ChunkData> TerrainManager::build(const HgtTile &tile, const ChunkKey &k) const
{
    const int span = (tile.size - 1) / kChunksPerTileAxis;   // 120 (SRTM1) or 40 (SRTM3)
    auto c = std::make_shared<ChunkData>();
    c->key = k;
    c->grid = span + 1;
    c->heights.resize(std::size_t(c->grid) * c->grid);
    const int r0 = k.cy * span, c0 = k.cx * span;
    int16_t lo = INT16_MAX, hi = INT16_MIN;
    int16_t last = 0;
    for (int r = 0; r < c->grid; ++r) {
        for (int col = 0; col < c->grid; ++col) {
            int16_t h = tile.at(r0 + r, c0 + col);
            // Voids: the previous good sample (the shader has no notion of a hole)
            if (h == kHgtVoid) h = last;
            else last = h;
            c->heights[std::size_t(r) * c->grid + col] = h;
            lo = std::min(lo, h);
            hi = std::max(hi, h);
        }
    }
    c->minH = lo;
    c->maxH = hi;
    const double lat0 = k.latBase + 1 - double(k.cy) / kChunksPerTileAxis;
    const double lon0 = k.lonBase + double(k.cx) / kChunksPerTileAxis;
    const GroundMeters p = latLonToMeters(lat0, lon0, m_origin);
    const double mPerLon = kMetersPerDegLat * std::cos(m_origin.lat * kDeg);
    c->x0 = p.x;
    c->z0 = p.z;
    c->dx = mPerLon / (tile.size - 1);
    c->dz = kMetersPerDegLat / (tile.size - 1);
    c->originEpoch = m_epoch;
    return c;
}

void TerrainManager::update(double lat, double lon, double groundSpeed, LatLon origin, int originEpoch)
{
    if (originEpoch != m_epoch) {
        // New world metres: everything resident is rebuilt where it now belongs
        m_epoch = originEpoch;
        m_origin = origin;
        m_active.clear();
        m_changes.clearAll();
        m_imagery.resetResident();
        m_dirty = true;
    }
    const double moved = flatDistance(m_lastLat, m_lastLon, lat, lon);
    if (m_dirty || moved >= 500) updateChunks(lat, lon);
    m_imagery.refresh(lat, lon, groundSpeed, m_active);
}

void TerrainManager::updateChunks(double lat, double lon)
{
    m_dirty = false;
    m_lastLat = lat;
    m_lastLon = lon;

    const double mPerLon = kMetersPerDegLat * std::max(0.01, std::cos(lat * kDeg));
    const double dLat = m_radius / kMetersPerDegLat, dLon = m_radius / mPerLon;
    const double step = 1.0 / kChunksPerTileAxis;

    // Wanted: chunks whose centre is within the radius
    QHash<quint64, ChunkKey> wanted;
    const int y0 = int(std::floor((lat - dLat) / step)), y1 = int(std::floor((lat + dLat) / step));
    const int x0 = int(std::floor((lon - dLon) / step)), x1 = int(std::floor((lon + dLon) / step));
    for (int yi = y0; yi <= y1; ++yi) {
        for (int xi = x0; xi <= x1; ++xi) {
            const double cLat = (yi + 0.5) * step, cLon = (xi + 0.5) * step;
            if (flatDistance(lat, lon, cLat, cLon) > m_radius) continue;
            ChunkKey k;
            k.latBase = int(std::floor(yi * step + 1e-9));
            k.lonBase = int(std::floor(xi * step + 1e-9));
            k.cx = xi - k.lonBase * kChunksPerTileAxis;
            // Rows count south from the north edge
            k.cy = kChunksPerTileAxis - 1 - (yi - k.latBase * kChunksPerTileAxis);
            wanted.insert(k.packed(), k);
        }
    }

    // Drop what left the cleanup radius
    for (auto it = m_active.begin(); it != m_active.end();) {
        const ChunkKey &k = it.value();
        const double cLat = k.latBase + 1 - (k.cy + 0.5) * step, cLon = k.lonBase + (k.cx + 0.5) * step;
        if (flatDistance(lat, lon, cLat, cLon) > m_radius * kCleanupMargin) {
            m_changes.remove(it.key());
            m_imagery.chunkRemoved(it.key());
            it = m_active.erase(it);
        } else {
            ++it;
        }
    }

    // Build what is missing, nearest first; a tile still loading leaves its chunks for the next pass
    QList<ChunkKey> todo;
    for (auto it = wanted.cbegin(); it != wanted.cend(); ++it)
        if (!m_active.contains(it.key())) todo << it.value();
    std::sort(todo.begin(), todo.end(), [&](const ChunkKey &a, const ChunkKey &b) {
        auto d = [&](const ChunkKey &k) {
            return flatDistance(lat, lon, k.latBase + 1 - (k.cy + 0.5) * step, k.lonBase + (k.cx + 0.5) * step);
        };
        return d(a) < d(b);
    });
    bool waiting = false, added = false;
    QSet<int> asked;
    for (const ChunkKey &k : todo) {
        const auto tile = HgtStore::instance().get(k.latBase, k.lonBase);
        if (!tile) {
            if (!HgtStore::instance().missing(k.latBase, k.lonBase)) {
                waiting = true;
            } else {
                // No folder has it: fetch it from the AWS terrain tiles (nearest
                // tiles first, as the chunks are sorted). A tile the bucket does
                // not have (open sea) stays missing for the session.
                const int tk = (k.latBase + 90) * 1000 + (k.lonBase + 180);
                if (!asked.contains(tk)) {
                    asked.insert(tk);
                    HgtDownloader::instance().ensure(k.latBase, k.lonBase);
                }
                if (HgtDownloader::instance().busy(k.latBase, k.lonBase)) waiting = true;
            }
            continue;
        }
        m_changes.add(build(*tile, k));
        m_active.insert(k.packed(), k);
        added = true;
    }
    // New chunks inside the satellite radius want imagery now, not at the next pass
    if (added) m_imagery.touch();
    if (waiting) m_dirty = true;
    emit chunksChanged(int(m_active.size()));
}

} // namespace corv
