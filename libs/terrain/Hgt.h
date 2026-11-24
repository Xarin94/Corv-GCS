// Hgt.h - SRTM .hgt elevation tiles (from TerrainWorker.js and the topography
// lookup of main.js).
//
// A tile is 1x1 degree, 3601x3601 (SRTM1) or 1201x1201 (SRTM3) big-endian
// int16 samples, row 0 on the north edge; -32768 is a void. Tiles are looked
// up, ignoring the case of the name, in topo/ and topography/ next to the
// installation, then in <data root>/terrain where downloads are saved, then in
// CORV_TOPO_DIRS (development; ';'-separated). A tile found in the install
// folder wins over a downloaded copy.
#pragma once

#include <QHash>
#include <QMutex>
#include <QObject>
#include <QSet>
#include <QStringList>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace corv {

constexpr int16_t kHgtVoid = -32768;

struct HgtTile {
    int latBase = 0, lonBase = 0;   // south-west corner
    int size = 0;                   // samples per side
    std::vector<int16_t> data;      // host order, voids kept

    int16_t at(int row, int col) const { return data[std::size_t(row) * size + col]; }
    /// Bilinear height at a point inside the tile; nullopt on a void.
    std::optional<double> elevation(double lat, double lon) const;

    /// "N47E011" for the tile containing (lat, lon) or with that corner.
    static QString name(int latBase, int lonBase);
    /// Decode a file. Null when the size is not an SRTM size.
    static std::shared_ptr<HgtTile> load(const QString &path, int latBase, int lonBase, QString *error = nullptr);
};

/// The tiles the GCS is using, loaded on the thread pool. 26 MB each: past kMaxTiles the ones
/// not asked for in the last kKeepMs go, the oldest first (read again if they are needed).
class HgtStore : public QObject
{
    Q_OBJECT
public:
    static HgtStore &instance();
    static constexpr int kMaxTiles = 20;
    static constexpr double kKeepMs = 60000;

    QStringList searchDirs() const;
    /// Path of a tile, or empty when no folder has it.
    QString find(int latBase, int lonBase) const;

    /// The tile if loaded; otherwise starts loading it and returns null.
    std::shared_ptr<const HgtTile> get(int latBase, int lonBase);
    /// True when no folder has the tile (it will never load).
    bool missing(int latBase, int lonBase) const;
    /// True while the tile is being read from disk.
    bool loading(int latBase, int lonBase) const;
    /// Tiles being read from disk now, and in memory (the loading screen).
    int loadingCount() const;
    int loadedCount() const;
    /// A tile that was missing may exist now (just downloaded): look again next time.
    void forget(int latBase, int lonBase);
    /// SYS CONFIG → SELECT HGT FOLDER: one more folder to search.
    void addSearchDir(const QString &dir);
    /// .hgt files in all the search folders ("232 HGT AVAILABLE (lazy)").
    int availableCount() const;

    /// Height at a point from the loaded tiles; nullopt when the tile is not loaded or a void.
    std::optional<double> elevation(double lat, double lon);

    /// Water under a point from the elevation data alone (getWaterSurfaceAt of
    /// TerrainManager.js): SRTM flattens every lake to one height and the sea to
    /// 0, so a dozen equal samples around the point mark a water surface; below
    /// 0 the tile carries bathymetry, the sea bed under a surface at 0.
    struct Water {
        double level = 0;
        std::optional<double> bed;
    };
    std::optional<Water> waterAt(double lat, double lon);

signals:
    void tileLoaded(int latBase, int lonBase);

private:
    HgtStore() = default;
    static int key(int latBase, int lonBase) { return (latBase + 90) * 1000 + (lonBase + 180); }

    mutable QMutex m_lock;
    struct Loaded {
        std::shared_ptr<const HgtTile> tile;
        double lastUse = 0;   // monoMs()
    };
    void evict();   // m_lock held
    QHash<int, Loaded> m_tiles;
    QSet<int> m_loading;
    QSet<int> m_missing;
    QStringList m_extraDirs;
};

} // namespace corv
