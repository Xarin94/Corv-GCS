#include "terrain/ChunkImagery.h"

#include "core/DebugLog.h"
#include "core/Geo.h"
#include "terrain/Bc1.h"
#include "terrain/TileStore.h"

#include <QPainter>
#include <QtConcurrent/QtConcurrentRun>
#include <algorithm>
#include <cmath>

namespace corv {

namespace {

struct Band {
    double maxDist;
    int zoom;
    int cap;   // largest image side the band may allocate
};
// ZOOM_BANDS of TerrainManager.js
constexpr Band kBands[] = {
    {3000, 17, 4096},
    {6000, 16, 2048},
    {12000, 15, 1024},
    {22000, 14, 512},
    {1e18, 13, 256},
};
constexpr int kBandCount = int(std::size(kBands));
constexpr int kBaseBand = 2;          // first pass: zoom 15 at most, the view fills quickly
constexpr int kMinZoom = 13;
constexpr double kHysteresis = 0.1;
constexpr int kTile = 256;
constexpr int kMaxActiveJobs = 6;     // jobs asking for tiles at the same time, nearest first
constexpr int kMaxComposing = 2;      // the two compression workers of the Electron version
constexpr qint64 kTileWaitMs = 10000;
constexpr int kChunksPerTileAxis = 30;

int bandFor(double dist)
{
    for (int i = 0; i < kBandCount; ++i)
        if (dist <= kBands[i].maxDist) return i;
    return kBandCount - 1;
}

double tileXf(double lon, int z) { return (lon + 180.0) / 360.0 * double(1 << z); }

double mercY(double lat)
{
    const double r = std::clamp(lat, -85.0511, 85.0511) * kDeg;
    return (1 - std::log(std::tan(r) + 1 / std::cos(r)) / kPi) / 2;
}

double tileYf(double lat, int z) { return mercY(lat) * double(1 << z); }

struct Bounds {
    double latTop, latBottom, lonLeft, lonRight;
};

Bounds boundsOf(const ChunkKey &k)
{
    const double step = 1.0 / kChunksPerTileAxis;
    const double latTop = k.latBase + 1 - k.cy * step;
    const double lonLeft = k.lonBase + k.cx * step;
    return {latTop, latTop - step, lonLeft, lonLeft + step};
}

struct ComposeInput {
    quint64 chunk = 0;
    int band = 0, zoom = 0, size = 0;
    bool bc1 = true;
    Bounds b{};
    int tx0 = 0, ty0 = 0, tilesX = 0, tilesY = 0;
    std::vector<QImage> tiles;     // null: missing
    std::vector<QImage> parents;   // for a missing tile: its parent at zoom - 1, when in memory
};

/// Thread pool: tile mosaic -> the chunk's lat/lon rectangle at size x size -> mips -> BC1.
std::shared_ptr<ChunkImage> compose(const ComposeInput &in)
{
    QImage mosaic(in.tilesX * kTile, in.tilesY * kTile, QImage::Format_RGB32);
    // Missing tiles show a neutral patch, not a black hole
    mosaic.fill(QColor(128, 128, 128));
    {
        QPainter p(&mosaic);
        p.setRenderHint(QPainter::SmoothPixmapTransform, true);
        for (int ty = 0; ty < in.tilesY; ++ty) {
            for (int tx = 0; tx < in.tilesX; ++tx) {
                const std::size_t i = std::size_t(ty) * in.tilesX + tx;
                const QRect dst(tx * kTile, ty * kTile, kTile, kTile);
                if (!in.tiles[i].isNull()) {
                    p.drawImage(dst, in.tiles[i]);
                } else if (!in.parents[i].isNull()) {
                    // The quarter of the parent tile that covers this one, scaled up
                    const QImage &par = in.parents[i];
                    const int pw = par.width() / 2, ph = par.height() / 2;
                    p.drawImage(dst, par, QRect(((in.tx0 + tx) & 1) * pw, ((in.ty0 + ty) & 1) * ph, pw, ph));
                }
            }
        }
    }

    // Web Mercator mosaic -> equirectangular chunk image, bilinear
    const int S = in.size;
    std::vector<uint8_t> rgba(std::size_t(S) * S * 4);
    const double world = double(1 << in.zoom) * kTile;
    const double mx0 = (in.b.lonLeft + 180) / 360 * world - in.tx0 * kTile;
    const double mx1 = (in.b.lonRight + 180) / 360 * world - in.tx0 * kTile;
    const auto *src = reinterpret_cast<const uint32_t *>(mosaic.constBits());
    const int stride = int(mosaic.bytesPerLine() / 4), W = mosaic.width(), H = mosaic.height();
    for (int j = 0; j < S; ++j) {
        const double lat = in.b.latTop - (j + 0.5) / S * (in.b.latTop - in.b.latBottom);
        const double Y = mercY(lat) * world - in.ty0 * kTile - 0.5;
        const int y0 = std::clamp(int(std::floor(Y)), 0, H - 1), y1 = std::min(y0 + 1, H - 1);
        const double fy = std::clamp(Y - std::floor(Y), 0.0, 1.0);
        const uint32_t *r0 = src + std::size_t(y0) * stride, *r1 = src + std::size_t(y1) * stride;
        uint8_t *o = rgba.data() + std::size_t(j) * S * 4;
        for (int i = 0; i < S; ++i, o += 4) {
            const double X = mx0 + (i + 0.5) / S * (mx1 - mx0) - 0.5;
            const int x0 = std::clamp(int(std::floor(X)), 0, W - 1), x1 = std::min(x0 + 1, W - 1);
            const double fx = std::clamp(X - std::floor(X), 0.0, 1.0);
            const uint32_t a = r0[x0], b = r0[x1], c = r1[x0], d = r1[x1];
            for (int ch = 0; ch < 3; ++ch) {
                const int sh = 16 - 8 * ch;   // R, G, B of 0xffRRGGBB
                const double top = ((a >> sh) & 255) * (1 - fx) + ((b >> sh) & 255) * fx;
                const double bot = ((c >> sh) & 255) * (1 - fx) + ((d >> sh) & 255) * fx;
                o[ch] = uint8_t(std::lround(top * (1 - fy) + bot * fy));
            }
            o[3] = 255;
        }
    }

    auto img = std::make_shared<ChunkImage>();
    img->chunk = in.chunk;
    img->size = S;
    img->band = in.band;
    img->zoom = in.zoom;
    img->bc1 = in.bc1;
    img->mips = bc1::mipChain(rgba.data(), S, S, in.bc1);
    return img;
}

} // namespace

ChunkImagery::ChunkImagery(TerrainChanges &changes, QObject *parent) : QObject(parent), m_changes(changes)
{
    m_clock.start();
    m_sinceRefresh.start();
    m_logClock.start();
    TileStore &store = TileStore::instance();
    connect(&store, &TileStore::tileReady, this, [this](const QString &p, int z, int x, int y) { onTile(p, z, x, y, true); });
    connect(&store, &TileStore::tileFailed, this, [this](const QString &p, int z, int x, int y) { onTile(p, z, x, y, false); });
}

ChunkImagery::~ChunkImagery()
{
    // Compositions still running post back to this object: let them end first
    m_futures.waitForFinished();
}

void ChunkImagery::setEnabled(bool on)
{
    if (on == m_enabled) return;
    m_enabled = on;
    if (!on) {
        // Satellite off: drop the images, the jobs and the queue (setTerrainSatelliteEnabled)
        m_jobs.clear();
        m_queue.clear();
        m_tileWaiters.clear();
        m_resident.clear();
        m_bytes = 0;
        m_changes.clearImages();
        m_initialDone = false;
        m_firstPassQueued = false;
    }
    m_force = true;
    LOG_I("terrain") << "satellite imagery" << (on ? "on" : "off") << "- radius" << m_radius << "m";
}

void ChunkImagery::setRadius(double metres)
{
    if (metres == m_radius) return;
    m_radius = metres;
    m_force = true;
}

void ChunkImagery::resetResident()
{
    // The renderer dropped everything with the chunks; the images themselves do not
    // depend on the scene origin, so the ones already built go again as they are
    for (const auto &[chunk, r] : m_resident)
        if (r.image) m_changes.addImage(r.image);
    m_force = true;
}

void ChunkImagery::chunkRemoved(quint64 chunk)
{
    dropJob(chunk);
    m_queue.removeIf([chunk](const auto &q) { return q.second.first.packed() == chunk; });
    auto it = m_resident.find(chunk);
    if (it != m_resident.end()) {
        m_bytes -= it->second.bytes;
        m_resident.erase(it);
        m_changes.removeImage(chunk);
    }
}

void ChunkImagery::dropJob(quint64 chunk)
{
    // Its tile waits stay in m_tileWaiters and are ignored when they come
    m_jobs.erase(chunk);
}

void ChunkImagery::refresh(double lat, double lon, double groundSpeed, const QHash<quint64, ChunkKey> &chunks)
{
    if (!m_enabled) return;
    if (m_logClock.elapsed() > 30000) {
        m_logClock.restart();
        LOG_I("terrain") << "satellite:" << m_resident.size() << "chunk images," << QString::number(m_bytes / 1048576.0, 'f', 1) << "MB,"
                         << m_jobs.size() << "jobs," << m_queue.size() << "queued";
    }
    // A pass after ~10 s of flight (500 m .. 5 km), every 30 s, or when something changed
    const double refreshDist = std::clamp((std::isfinite(groundSpeed) ? groundSpeed : 0) * 10, 500.0, 5000.0);
    const bool due = m_force || flatDistance(m_lastLat, m_lastLon, lat, lon) >= refreshDist || m_sinceRefresh.elapsed() > 30000;
    if (due) {
        m_force = false;
        m_lastLat = lat;
        m_lastLon = lon;
        m_sinceRefresh.restart();
        // The first pass is over once the work it queued is done, even if chunks
        // entering the radius meanwhile are still waiting (a moving vehicle always has some)
        if (!m_initialDone && m_firstPassQueued && m_jobs.empty() && m_queue.isEmpty()) m_initialDone = true;

        QList<std::pair<double, std::pair<ChunkKey, int>>> load, upgrade, downgrade;
        for (auto it = chunks.cbegin(); it != chunks.cend(); ++it) {
            const quint64 chunk = it.key();
            const Bounds b = boundsOf(it.value());
            const double dist = flatDistance(lat, lon, (b.latTop + b.latBottom) / 2, (b.lonLeft + b.lonRight) / 2);
            auto res = m_resident.find(chunk);
            if (dist > m_radius) {
                // Out of the satellite radius: back to the height palette
                dropJob(chunk);
                if (res != m_resident.end()) {
                    m_bytes -= res->second.bytes;
                    m_resident.erase(res);
                    m_changes.removeImage(chunk);
                }
                continue;
            }
            if (m_jobs.count(chunk)) continue;   // already being built
            const int band = bandFor(dist);
            if (res == m_resident.end()) {
                load.append({dist, {it.value(), m_initialDone ? band : std::max(band, kBaseBand)}});
            } else if (m_initialDone) {
                const int current = res->second.band;
                if (bandFor(dist * (1 + kHysteresis)) < current) upgrade.append({dist, {it.value(), band}});
                else if (bandFor(dist * (1 - kHysteresis)) > current) downgrade.append({dist, {it.value(), band}});
            }
        }
        auto nearFirst = [](const auto &a, const auto &b) { return a.first < b.first; };
        std::sort(load.begin(), load.end(), nearFirst);
        std::sort(upgrade.begin(), upgrade.end(), nearFirst);
        std::sort(downgrade.begin(), downgrade.end(), [](const auto &a, const auto &b) { return a.first > b.first; });
        m_queue = load + upgrade + downgrade;
        if (!m_queue.isEmpty()) m_firstPassQueued = true;
        if (!m_queue.isEmpty())
            LOG_D("terrain") << "satellite pass:" << load.size() << "to load," << upgrade.size() << "finer," << downgrade.size() << "coarser"
                             << (m_initialDone ? "" : "(first pass)");
    }
    // Start what the queue allows, and compose what is ready or waited long enough
    while (int(m_jobs.size()) < kMaxActiveJobs && !m_queue.isEmpty()) {
        const auto q = m_queue.takeFirst();
        startJob(q.second.first, q.second.second, q.first);
    }
    pumpCompose();
}

void ChunkImagery::startJob(const ChunkKey &key, int band, double dist)
{
    const quint64 chunk = key.packed();
    auto job = std::make_unique<Job>();
    job->chunk = chunk;
    job->band = band;
    job->dist = dist;
    const Bounds b = boundsOf(key);
    job->latTop = b.latTop;
    job->latBottom = b.latBottom;
    job->lonLeft = b.lonLeft;
    job->lonRight = b.lonRight;

    // The band's zoom, stepped down until the tiles fit the band's cap
    const int cap = kBands[band].cap;
    int z = kBands[band].zoom;
    auto span = [&](int zz, int &tx0, int &ty0, int &nx, int &ny) {
        tx0 = int(std::floor(tileXf(b.lonLeft, zz)));
        ty0 = int(std::floor(tileYf(b.latTop, zz)));
        nx = int(std::floor(tileXf(b.lonRight, zz) - 1e-9)) - tx0 + 1;
        ny = int(std::floor(tileYf(b.latBottom, zz) - 1e-9)) - ty0 + 1;
    };
    int tx0 = 0, ty0 = 0, nx = 0, ny = 0;
    for (;;) {
        span(z, tx0, ty0, nx, ny);
        if (z <= kMinZoom || (nx * kTile <= cap && ny * kTile <= cap)) break;
        --z;
    }
    job->zoom = z;
    job->tx0 = tx0;
    job->ty0 = ty0;
    job->tilesX = nx;
    job->tilesY = ny;
    // Square power-of-two image nearest to the chunk's own pixel size at that zoom
    const double w = (tileXf(b.lonRight, z) - tileXf(b.lonLeft, z)) * kTile;
    const double h = (tileYf(b.latBottom, z) - tileYf(b.latTop, z)) * kTile;
    job->size = std::clamp(1 << int(std::lround(std::log2(std::max({w, h, 1.0})))), 64, cap);
    job->tiles.assign(std::size_t(nx) * ny, QImage());
    job->resolved.assign(std::size_t(nx) * ny, 0);
    job->started = m_clock.elapsed();
    job->generation = ++m_generation;

    TileStore &store = TileStore::instance();
    for (int ty = 0; ty < ny; ++ty) {
        for (int tx = 0; tx < nx; ++tx) {
            const std::size_t i = std::size_t(ty) * nx + tx;
            const int x = tx0 + tx, y = ty0 + ty;
            const QImage img = store.tile(m_provider, z, x, y);
            if (!img.isNull()) {
                job->tiles[i] = img;
                job->resolved[i] = 1;
            } else if (store.hasFailed(m_provider, z, x, y)) {
                job->resolved[i] = 1;
            } else {
                m_tileWaiters.insert(tileKey(z, x, y), chunk);
                ++job->waiting;
            }
        }
    }
    m_jobs[chunk] = std::move(job);
}

void ChunkImagery::onTile(const QString &provider, int z, int x, int y, bool ok)
{
    if (provider != m_provider) return;
    const quint64 tk = tileKey(z, x, y);
    const QList<quint64> waiters = m_tileWaiters.values(tk);
    if (waiters.isEmpty()) return;
    m_tileWaiters.remove(tk);
    const QImage img = ok ? TileStore::instance().cached(provider, z, x, y) : QImage();
    bool ready = false;
    for (quint64 chunk : waiters) {
        auto it = m_jobs.find(chunk);
        if (it == m_jobs.end()) continue;
        Job &job = *it->second;
        if (job.zoom != z || job.composing) continue;
        const int tx = x - job.tx0, ty = y - job.ty0;
        if (tx < 0 || ty < 0 || tx >= job.tilesX || ty >= job.tilesY) continue;
        const std::size_t i = std::size_t(ty) * job.tilesX + tx;
        if (job.resolved[i]) continue;
        job.resolved[i] = 1;
        job.tiles[i] = img;
        if (--job.waiting == 0) ready = true;
    }
    if (ready) pumpCompose();
}

void ChunkImagery::pumpCompose()
{
    const qint64 now = m_clock.elapsed();
    while (m_composing < kMaxComposing) {
        // The nearest job whose tiles are all in, or that waited long enough
        Job *pick = nullptr;
        for (auto &[chunk, job] : m_jobs) {
            if (job->composing) continue;
            if (job->waiting > 0 && now - job->started < kTileWaitMs) continue;
            if (!pick || job->dist < pick->dist) pick = job.get();
        }
        if (!pick) break;

        ComposeInput in;
        in.chunk = pick->chunk;
        in.band = pick->band;
        in.zoom = pick->zoom;
        in.size = pick->size;
        in.bc1 = m_bc1;
        in.b = {pick->latTop, pick->latBottom, pick->lonLeft, pick->lonRight};
        in.tx0 = pick->tx0;
        in.ty0 = pick->ty0;
        in.tilesX = pick->tilesX;
        in.tilesY = pick->tilesY;
        in.tiles = pick->tiles;
        in.parents.assign(in.tiles.size(), QImage());
        for (int ty = 0; ty < pick->tilesY; ++ty)
            for (int tx = 0; tx < pick->tilesX; ++tx) {
                const std::size_t i = std::size_t(ty) * pick->tilesX + tx;
                if (in.tiles[i].isNull() && pick->zoom > 0)
                    in.parents[i] = TileStore::instance().cached(m_provider, pick->zoom - 1, (pick->tx0 + tx) / 2, (pick->ty0 + ty) / 2);
            }
        pick->composing = true;
        ++m_composing;
        const quint64 chunk = pick->chunk, gen = pick->generation;
        m_futures.addFuture(QtConcurrent::run([this, in = std::move(in), chunk, gen] {
            std::shared_ptr<ChunkImage> img = compose(in);
            QMetaObject::invokeMethod(this, [this, chunk, gen, img] { finished(chunk, gen, img); }, Qt::QueuedConnection);
        }));
    }
    // Drop finished futures now and then
    if (m_futures.futures().size() > 64) {
        QFutureSynchronizer<void> keep;
        for (const QFuture<void> &f : m_futures.futures())
            if (!f.isFinished()) keep.addFuture(f);
        m_futures.clearFutures();
        for (const QFuture<void> &f : keep.futures()) m_futures.addFuture(f);
    }
}

void ChunkImagery::finished(quint64 chunk, quint64 generation, std::shared_ptr<ChunkImage> img)
{
    --m_composing;
    if (!m_initialDone && m_firstPassQueued && m_jobs.size() <= 1 && m_queue.isEmpty()) m_force = true;
    auto it = m_jobs.find(chunk);
    if (it != m_jobs.end() && it->second->generation == generation) {
        m_jobs.erase(it);
        if (m_enabled && img) {
            Resident &r = m_resident[chunk];
            m_bytes += img->bytes() - r.bytes;
            r.band = img->band;
            r.bytes = img->bytes();
            r.image = img;
            m_changes.addImage(img);
        }
    }
    // Room for the next job
    while (int(m_jobs.size()) < kMaxActiveJobs && !m_queue.isEmpty()) {
        const auto q = m_queue.takeFirst();
        startJob(q.second.first, q.second.second, q.first);
    }
    pumpCompose();
}

} // namespace corv
