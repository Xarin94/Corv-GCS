// The clipmap of satellite imagery around the aircraft (SatelliteDetail): which
// levels are used, how world x/z map into a level's window, what a move of one
// tile and a lower SATELLITE DETAIL setting ask of the renderer. No GPU, no
// network: the tiles are only requested.
#include "terrain/SatelliteDetail.h"
#include "terrain/TileStore.h"

#include <QTemporaryDir>
#include <QtTest>
#include <cmath>

using namespace corv;

namespace {
constexpr double kLat = 47.26, kLon = 11.35;
constexpr double kMPerPxZ0 = 156543.03392;

int count(const std::vector<DetailOp> &ops, DetailOp::Kind k, int level)
{
    return int(std::count_if(ops.begin(), ops.end(), [&](const DetailOp &o) { return o.kind == k && o.level == level; }));
}
} // namespace

class TstSatDetail : public QObject
{
    Q_OBJECT
    QTemporaryDir m_dir;
    const LatLon m_origin{kLat, kLon};
    const QVector3D m_air{0, 600, 0};
    const QVector3D m_cam{60, 660, 60};      // chase camera 116 m from the ground point
    const double m_pixelAngle = 60 * kDeg / 1080;

private slots:
    void initTestCase()
    {
        qputenv("CORV_DATA_ROOT", m_dir.path().toUtf8());
        TileStore::instance().setNetworkEnabled(false);
    }

    void levelsFollowSettingAndDistance()
    {
        SatelliteDetail d;
        d.setMaxZoom(18);
        std::vector<DetailOp> ops;
        DetailFrame f;
        d.update(kLat, kLon, m_origin, m_cam, m_air, 560.0, m_pixelAngle, true, ops, f);
        QCOMPARE(f.on, QVector4D(1, 1, 0, 0));
        QCOMPARE(count(ops, DetailOp::Allocate, 0), 1);
        QCOMPARE(count(ops, DetailOp::Clear, 0), 1);
        QCOMPARE(count(ops, DetailOp::Allocate, 1), 1);
        QCOMPARE(count(ops, DetailOp::Allocate, 2), 0);
        // 64 tiles asked per level, none in memory yet
        const auto st = d.stats();
        QCOMPARE(st[0].loading, 64);
        QCOMPARE(st[1].loading, 64);
        QCOMPARE(st[2].loading, 0);

        // 1000 m from the ground only zoom 17 and 18 add detail (SatelliteDetail.js), whatever the setting
        d.setMaxZoom(20);
        ops.clear();
        d.update(kLat, kLon, m_origin, m_cam + QVector3D(0, 900, 0), m_air, 560.0, m_pixelAngle, true, ops, f);
        QCOMPARE(f.on, QVector4D(1, 1, 0, 0));
        // 3 km away none does
        ops.clear();
        d.update(kLat, kLon, m_origin, m_cam + QVector3D(0, 3000, 0), m_air, 560.0, m_pixelAngle, true, ops, f);
        QCOMPARE(f.on, QVector4D());
    }

    void windowMapsTheAircraft()
    {
        SatelliteDetail d;
        d.setMaxZoom(18);
        std::vector<DetailOp> ops;
        DetailFrame f;
        d.update(kLat, kLon, m_origin, m_cam, m_air, 560.0, m_pixelAngle, true, ops, f);
        for (int l = 0; l < 2; ++l) {
            const QVector4D m = f.map[std::size_t(l)], w = f.win[std::size_t(l)];
            // Texture pixels per metre: one over the ground resolution of the zoom
            const double res = kMPerPxZ0 * std::cos(kLat * kDeg) / std::ldexp(1.0, SatelliteDetail::zoomOf(l));
            QVERIFY2(std::abs(m.z() * res - 1) < 0.01, qPrintable(QString::number(m.z() * res)));
            QVERIFY2(std::abs(m.w() * res - 1) < 0.01, qPrintable(QString::number(m.w() * res)));
            // The aircraft (world 0, 0) within half a tile of the window's centre
            const float px = (m_air.x() - m.x()) * m.z() + w.x();
            const float pz = (m_air.z() - m.y()) * m.w() + w.y();
            QVERIFY2(std::abs(px - SatelliteDetail::kSize / 2) <= SatelliteDetail::kTile / 2 + 2, qPrintable(QString::number(px)));
            QVERIFY2(std::abs(pz - SatelliteDetail::kSize / 2) <= SatelliteDetail::kTile / 2 + 2, qPrintable(QString::number(pz)));
        }
    }

    void oneTileEastLoadsOneColumn()
    {
        SatelliteDetail d;
        d.setMaxZoom(17);
        std::vector<DetailOp> ops;
        DetailFrame f;
        d.update(kLat, kLon, m_origin, m_cam, m_air, 560.0, m_pixelAngle, true, ops, f);
        // Positions in tiles of zoom 17 east of the window's centre (a tile corner)
        const double n = std::ldexp(1.0, 17), tileDeg = 360.0 / n;
        const double centre = std::round((kLon + 180) / 360 * n);
        auto lonAt = [&](double fx) { return fx / n * 360 - 180; };
        // Half a tile off the centre: inside the window's slack (0.75 tile), nothing moves
        ops.clear();
        d.update(kLat, lonAt(centre + 0.5), m_origin, m_cam, m_air, 560.0, m_pixelAngle, true, ops, f);
        QCOMPARE(count(ops, DetailOp::ClearSlot, 0), 0);
        // Past the slack: the window moves one tile east, one column enters, its 8 slots
        // are cleared, not the texture
        ops.clear();
        d.update(kLat, lonAt(centre + 0.8), m_origin, m_cam, m_air, 560.0, m_pixelAngle, true, ops, f);
        QCOMPARE(count(ops, DetailOp::ClearSlot, 0), SatelliteDetail::kTiles);
        QCOMPARE(count(ops, DetailOp::Clear, 0), 0);
        // Far away: a jump clears the whole texture once
        ops.clear();
        d.update(kLat, kLon + 20 * tileDeg, m_origin, m_cam, m_air, 560.0, m_pixelAngle, true, ops, f);
        QCOMPARE(count(ops, DetailOp::Clear, 0), 1);
        QCOMPARE(count(ops, DetailOp::ClearSlot, 0), 0);
    }

    void lowerSettingReleases()
    {
        SatelliteDetail d;
        d.setMaxZoom(19);
        std::vector<DetailOp> ops;
        DetailFrame f;
        d.update(kLat, kLon, m_origin, m_cam, m_air, 560.0, m_pixelAngle, true, ops, f);
        QCOMPARE(count(ops, DetailOp::Allocate, 2), 1);
        d.setMaxZoom(17);
        ops.clear();
        d.update(kLat, kLon, m_origin, m_cam, m_air, 560.0, m_pixelAngle, true, ops, f);
        QCOMPARE(count(ops, DetailOp::Release, 1), 1);
        QCOMPARE(count(ops, DetailOp::Release, 2), 1);
        QCOMPARE(count(ops, DetailOp::Release, 0), 0);
        QCOMPARE(f.on, QVector4D(1, 0, 0, 0));
        // Satellite imagery off (schematic view): no level drawn, nothing given back
        ops.clear();
        d.update(kLat, kLon, m_origin, m_cam, m_air, 560.0, m_pixelAngle, false, ops, f);
        QCOMPARE(f.on, QVector4D());
        QVERIFY(ops.empty());
    }
};

QTEST_GUILESS_MAIN(TstSatDetail)
#include "tst_satdetail.moc"
