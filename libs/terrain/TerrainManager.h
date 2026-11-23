// TerrainManager.h - which chunks exist around the vehicle (the chunk part of
// js/terrain/TerrainManager.js).
//
// A chunk is 1/30 degree of an HGT tile (CHUNKS_PER_TILE_AXIS = 30: about
// 3.7 x 2.5 km at mid latitudes). Chunks whose centre lies within the terrain
// radius (35 / 50 / 70 km, SYS CONFIG) are built, those beyond 1.05 x radius
// dropped. Heights go to the renderer at full resolution; the renderer picks the
// decimation per frame from the screen-space error. When the scene origin moves
// every chunk is rebuilt in the new world metres. The chunks are cut out of the
// tile on a low-priority pool, a few at a time: a tile that arrives makes some
// hundred of them, too many for one turn of the GUI thread.
// Chunks within the satellite radius also get satellite imagery (ChunkImagery)
// while the satellite view is on.
// GUI thread; tiles load on the pool (HgtStore).
#pragma once

#include "core/Geo.h"
#include "terrain/ChunkImagery.h"
#include "scene/TerrainData.h"

#include <QHash>
#include <QObject>
#include <QSet>
#include <QThreadPool>
#include <memory>
#include <vector>

namespace corv {

struct HgtTile;

class TerrainManager : public QObject
{
    Q_OBJECT
public:
    static constexpr int kChunksPerTileAxis = 30;
    static constexpr double kCleanupMargin = 1.05;

    explicit TerrainManager(QObject *parent = nullptr);
    ~TerrainManager() override;

    TerrainChanges &changes() { return m_changes; }

    void setRadius(double metres);
    double radius() const { return m_radius; }

    /// Bring the chunk set (and its imagery) up to date around a position. Cheap when nothing changed.
    void update(double lat, double lon, double groundSpeed, LatLon origin, int originEpoch);

    /// Satellite view on: imagery on the chunks within the satellite radius.
    void setSatellite(bool on) { m_imagery.setEnabled(on); }
    void setSatelliteRadius(double metres) { m_imagery.setRadius(metres); }
    double satelliteRadius() const { return m_imagery.radius(); }
    ChunkImagery &imagery() { return m_imagery; }

    int activeChunks() const { return int(m_active.size()); }

private:
    void updateChunks(double lat, double lon);
    static std::shared_ptr<ChunkData> build(const HgtTile &tile, const ChunkKey &k, LatLon origin, int epoch);
    void built(const std::vector<std::shared_ptr<ChunkData>> &chunks, int epoch);
    static double distanceTo(const ChunkKey &k, double lat, double lon);

    TerrainChanges m_changes;
    ChunkImagery m_imagery{m_changes};
    QHash<quint64, ChunkKey> m_active;
    QSet<quint64> m_building;
    QThreadPool m_pool;
    double m_radius = 35000;
    LatLon m_origin = kDefaultOrigin;
    int m_epoch = -1;
    double m_lastLat = 1e9, m_lastLon = 1e9;
    bool m_dirty = true;
};

} // namespace corv
