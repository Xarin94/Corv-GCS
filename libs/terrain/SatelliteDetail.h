// SatelliteDetail.h - satellite imagery at full resolution around the aircraft
// (port of js/terrain/SatelliteDetail.js).
//
// The chunk textures (ChunkImagery) end at zoom 16-17 next to the aircraft,
// 1.6 m per pixel: flying 50 m over the ground a screen pixel covers 6 cm and
// the ground is a blur that the 2D map, at zoom 20, shows sharp.
//
// Four square textures follow the aircraft, one per zoom (17, 18, 19, 20),
// each 8 x 8 tiles of 256 px: 1.7 km, 830 m, 420 m and 210 m across at mid
// latitudes. The terrain shader draws them over the chunk textures, a finer
// one over a coarser one, each fading out over its outer half tile: a clipmap.
// A level is used only while the camera is close enough for it to add detail
// (a screen pixel there covers less than two of its texels). SYS CONFIG ->
// SATELLITE DETAIL sets the finest level: 18 by default (0.4 m), up to 20
// (0.1 m), or 16 for the chunk textures alone.
//
// Each texture is toroidal: tile (x, y) lives in slot (x mod 8, y mod 8), so
// when the window moves only the tiles entering it are loaded, into the slots
// of the ones that left; the shader samples with repeat wrapping. A slot is
// cleared to transparent the moment it changes tile, so the chunk texture
// shows until the new tile arrives (for good when it never comes). Tiles come
// from TileStore: memory, the MBTiles file, then the network.
//
// GUI thread object; update() runs in the 3D view's synchronize (GUI blocked)
// and hands the renderer the texture operations in order, at most 4 tile
// uploads a frame, and each level's map from world x/z to its window.
#pragma once

#include "core/Geo.h"

#include <QElapsedTimer>
#include <QImage>
#include <QObject>
#include <QTimer>
#include <QVector3D>
#include <QVector4D>
#include <array>
#include <optional>
#include <vector>

namespace corv {

struct DetailOp {
    enum Kind : quint8 { Allocate, Release, Clear, ClearSlot, Upload };
    Kind kind = Clear;
    int level = 0;
    int slot = 0;
    QImage image;       // Upload: 256 x 256 RGBA8888 premultiplied
};

struct DetailFrame {
    QVector4D on;                         // 1 for a level in use
    std::array<QVector4D, 4> map{};       // window centre in world x, z; texture px per metre in x, z
    std::array<QVector4D, 4> win{};       // centre px in the window; the window's origin in the texture
};

class SatelliteDetail : public QObject   // a context for TileStore's signals (lambdas): no Q_OBJECT
{
public:
    static constexpr int kLevels = 4;
    static constexpr int kTile = 256;
    static constexpr int kTiles = 8;
    static constexpr int kSize = kTile * kTiles;
    static constexpr int kDefaultMaxZoom = 18;
    static int zoomOf(int level) { return 17 + level; }

    explicit SatelliteDetail(QObject *parent = nullptr);

    /// The sharpest zoom drawn, 16 (chunk textures only) to 20. Levels above it give their texture back.
    void setMaxZoom(int zoom);
    int maxZoom() const { return m_maxZoom; }

    /// Once per frame. air: the aircraft in world metres; groundY: terrain under it;
    /// pixelAngle: vertical FOV / viewport height (rad); enabled: satellite imagery drawn.
    /// Appends the texture operations to `ops` and writes the uniforms to `frame`.
    void update(double lat, double lon, LatLon origin, const QVector3D &cam, const QVector3D &air, std::optional<double> groundY,
                double pixelAngle, bool enabled, std::vector<DetailOp> &ops, DetailFrame &frame);

    struct LevelStats { int zoom = 0; bool active = false, allocated = false; int ready = 0, loading = 0, missing = 0; };
    std::array<LevelStats, kLevels> stats() const;

private:
    enum class SlotState : quint8 { Empty, Loading, Ready, Missing };
    struct Slot {
        int tx = 0, ty = 0;
        SlotState state = SlotState::Empty;
    };
    struct Level {
        int index = 0, zoom = 17;
        bool active = false, allocated = false, placed = false;
        int tx0 = 0, ty0 = 0;           // window origin, tiles
        std::array<Slot, kTiles * kTiles> cells{};
    };
    struct Copy {
        int level = 0, slot = 0, tx = 0, ty = 0;
        QImage image;
    };

    void release(Level &l, std::vector<DetailOp> &ops);
    void placeWindow(Level &l, double lat, double lon, std::vector<DetailOp> &ops);
    void retryMissing();
    void request(Level &l, int slot);
    void onTile(const QString &provider, int z, int x, int y, bool ok);
    void writeUniforms(const Level &l, LatLon origin, DetailFrame &frame) const;
    void flushCopies(std::vector<DetailOp> &ops);

    std::array<Level, kLevels> m_levels;
    std::vector<Copy> m_copies;
    QTimer m_retryTimer;
    int m_maxZoom = kDefaultMaxZoom;
    QString m_provider = QStringLiteral("satellite");
    QElapsedTimer m_statsClock;
    QString m_lastStats;
};

} // namespace corv
