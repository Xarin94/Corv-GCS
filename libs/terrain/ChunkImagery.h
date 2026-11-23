// ChunkImagery.h - satellite imagery on the terrain chunks (the texture half of
// js/terrain/TerrainManager.js: ZOOM_BANDS, createChunkTexture,
// refreshNearbyChunkTextures, and TextureCompressWorker.js).
//
// Resolution follows distance. Each band pairs a tile zoom with the largest
// image it may allocate; when a chunk would need more pixels the zoom steps
// down (chunks are taller than wide at high latitude):
//
//   distance    zoom   cap      (near chunks step down to zoom 16, 1.6 m/px at 47 N)
//   <= 3 km      17    4096
//   <= 6 km      16    2048
//   <= 12 km     15    1024
//   <= 22 km     14     512
//   beyond       13     256
//
// Only chunks within the satellite radius (SYS CONFIG, 5-30 km, default 10)
// get imagery; the others keep the height palette. The first pass builds every
// chunk at zoom 15 at most, so the view fills quickly, then nearer chunks are
// rebuilt finer. A chunk is re-textured only when it is 10 % inside the next
// band or 10 % outside its own (hysteresis: a chunk on a band edge would
// otherwise be recomposed on every pass), and the pass runs again after a move
// of 10 s of flight (500 m to 5 km) or every 30 s.
//
// A job asks TileStore for its tiles (memory, then the SQLite store, then the
// network), waits for them (a tile that fails, or 10 s, ends the wait: a
// missing tile takes the quarter of its parent tile when that is in memory,
// grey otherwise), then composes on a low-priority pool: the tile mosaic is
// resampled from Web Mercator to the chunk's lat/lon rectangle at a square
// power-of-two size, the mip chain is built and BC1-compressed. Two jobs
// compose at a time. The result goes to the renderer through TerrainChanges;
// the old image of a chunk stays drawn until the new one replaces it.
// GUI thread.
#pragma once

#include "scene/TerrainData.h"

#include <QElapsedTimer>
#include <QHash>
#include <QImage>
#include <QObject>
#include <QThreadPool>
#include <memory>
#include <unordered_map>

namespace corv {

class ChunkImagery : public QObject
{
    Q_OBJECT
public:
    static constexpr double kDefaultRadius = 10000;

    explicit ChunkImagery(TerrainChanges &changes, QObject *parent = nullptr);
    ~ChunkImagery() override;

    void setEnabled(bool on);
    bool enabled() const { return m_enabled; }
    void setRadius(double metres);
    double radius() const { return m_radius; }
    /// BC1 when the GPU has it (set by the renderer's answer; RGBA8 otherwise).
    void setCompressed(bool on) { m_bc1 = on; }

    /// Bring the imagery up to date around a position; `chunks` are the resident chunks.
    void refresh(double lat, double lon, double groundSpeed, const QHash<quint64, ChunkKey> &chunks);
    /// A chunk left the terrain: forget its image and its job.
    void chunkRemoved(quint64 chunk);
    /// The chunk set changed: run a pass at the next refresh.
    void touch() { m_force = true; }
    /// Every chunk was dropped (new scene origin): images must be sent again.
    void resetResident();


private:
    struct Job {
        quint64 chunk = 0;
        int band = 0, zoom = 0, size = 0;
        double latTop = 0, latBottom = 0, lonLeft = 0, lonRight = 0;
        int tx0 = 0, ty0 = 0, tilesX = 0, tilesY = 0;
        std::vector<QImage> tiles;      // row-major, null while missing
        std::vector<char> resolved;     // arrived or failed
        int waiting = 0;
        double dist = 0;
        bool composing = false;
        qint64 started = 0;
        quint64 generation = 0;
    };
    struct Resident {
        int band = 0;
        qint64 bytes = 0;
        std::shared_ptr<const ChunkImage> image;   // shared with the renderer
    };

    void startJob(const ChunkKey &key, int band, double dist);
    void onTile(const QString &provider, int z, int x, int y, bool ok);
    void pumpCompose();
    void finished(quint64 chunk, quint64 generation, std::shared_ptr<ChunkImage> img);
    void dropJob(quint64 chunk);
    static quint64 tileKey(int z, int x, int y) { return (quint64(z) << 48) | (quint64(quint32(x)) << 24) | quint64(quint32(y)); }

    TerrainChanges &m_changes;
    bool m_enabled = false;
    bool m_bc1 = true;
    double m_radius = kDefaultRadius;
    QString m_provider = QStringLiteral("satellite");
    std::unordered_map<quint64, Resident> m_resident;
    qint64 m_bytes = 0;
    std::unordered_map<quint64, std::unique_ptr<Job>> m_jobs;
    QMultiHash<quint64, quint64> m_tileWaiters;   // tile -> chunks waiting for it
    QList<std::pair<double, std::pair<ChunkKey, int>>> m_queue;   // (distance, (chunk, band)) not started yet
    int m_composing = 0;
    quint64 m_generation = 0;
    bool m_initialDone = false;
    bool m_firstPassQueued = false;
    bool m_force = true;
    double m_lastLat = 1e9, m_lastLon = 1e9;
    QElapsedTimer m_sinceRefresh;
    QElapsedTimer m_clock;
    QElapsedTimer m_logClock;
    QThreadPool m_pool;
};

} // namespace corv
