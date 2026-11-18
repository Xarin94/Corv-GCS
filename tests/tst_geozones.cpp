// UAS geographical zones (the cases of scripts/test-geozones.js): ED-269 and
// ED-318 parsing, polygons with holes, the general limit by region, the height
// rules and the arming alarm, and the raster for the terrain.
#include "scene/GeoZone.h"
#include "scene/GeoZoneRaster.h"

#include <QJsonDocument>
#include <QTest>

using namespace corv;

namespace {
/// A square ring of half-size d degrees around lon, lat, as GeoJSON coordinates.
QJsonArray square(double lon, double lat, double d)
{
    return QJsonArray{QJsonArray{lon - d, lat - d}, QJsonArray{lon + d, lat - d}, QJsonArray{lon + d, lat + d},
                      QJsonArray{lon - d, lat + d}, QJsonArray{lon - d, lat - d}};
}
/// [ring]: a one-element array (QJsonArray{ring} would copy the ring, not nest it)
QJsonArray one(const QJsonArray &ring)
{
    QJsonArray a;
    a.append(ring);
    return a;
}
} // namespace

class TstGeoZones : public QObject
{
    Q_OBJECT
private slots:
    void ed269()
    {
        const QJsonObject f1{{"identifier", "LIRP-CTR"}, {"name", "Pisa CTR"}, {"restriction", "REQ_AUTHORISATION"},
                             {"geometry", QJsonArray{QJsonObject{{"uomDimensions", "M"}, {"lowerLimit", 0}, {"lowerVerticalReference", "AGL"},
                                                                 {"upperLimit", 1000}, {"upperVerticalReference", "AMSL"},
                                                                 {"horizontalProjection", QJsonObject{{"type", "Polygon"}, {"coordinates", one(square(10.4, 43.7, 0.05))}}}}}}};
        const QJsonObject f2{{"identifier", "PARK"}, {"restriction", "CONDITIONAL"},
                             {"geometry", QJsonArray{QJsonObject{{"uomDimensions", "FT"}, {"lowerLimit", 200}, {"lowerVerticalReference", "AGL"},
                                                                 {"horizontalProjection", QJsonObject{{"type", "Circle"}, {"center", QJsonArray{10.5, 43.8}}, {"radius", 1000}}}}}}};
        const QJsonObject gone{{"identifier", "OLD"}, {"restriction", "PROHIBITED"},
                               {"applicability", QJsonObject{{"endDateTime", "2001-01-01T00:00:00Z"}}},
                               {"geometry", QJsonArray{QJsonObject{{"horizontalProjection", QJsonObject{{"type", "Polygon"}, {"coordinates", one(square(10, 43, 0.1))}}}}}}};
        const QJsonObject none{{"identifier", "FREE"}, {"restriction", "NO_RESTRICTION"},
                               {"geometry", QJsonArray{QJsonObject{{"horizontalProjection", QJsonObject{{"type", "Polygon"}, {"coordinates", one(square(10, 43, 0.1))}}}}}}};
        const GeoZoneList z = geozone::parseEdZones(QJsonObject{{"features", QJsonArray{f1, f2, gone, none}}}, "import-it");
        QCOMPARE(int(z.size()), 2);
        QCOMPARE(z[0].id, QStringLiteral("import-it:LIRP-CTR"));
        QCOMPARE(z[0].kind, QStringLiteral("prohibited"));
        QVERIFY(z[0].top && std::abs(*z[0].top - 1000) < 1e-9);
        QVERIFY(!z[0].topAgl);
        QCOMPARE(z[1].kind, QStringLiteral("conditional"));
        QVERIFY(std::abs(z[1].floor - 61) < 0.1);   // 200 ft
        QVERIFY(geozone::pointInZone(z[1], 43.8, 10.5));
        QVERIFY(!geozone::pointInZone(z[1], 43.82, 10.5));
        // The JSON round trip of the cache
        const auto back = GeoZone::fromJson(z[0].toJson());
        QVERIFY(back);
        QCOMPARE(back->polys.size(), z[0].polys.size());
        QCOMPARE(back->name, z[0].name);
    }

    void ed318AndHoles()
    {
        const QJsonObject f{{"type", "Feature"},
                            {"properties", QJsonObject{{"identifier", "ES1"}, {"restriction", "PROHIBITED"},
                                                       {"verticalLayer", QJsonObject{{"lower", 50}, {"lowerReference", "AGL"}, {"upper", 120}, {"uom", "m"}}}}},
                            {"geometry", QJsonObject{{"type", "Polygon"}, {"coordinates", QJsonArray{square(-3.7, 40.4, 0.1), square(-3.7, 40.4, 0.02)}}}}};
        const GeoZoneList z = geozone::parseEdZones(QJsonObject{{"type", "FeatureCollection"}, {"features", QJsonArray{f}}}, "enaire");
        QCOMPARE(int(z.size()), 1);
        QCOMPARE(z[0].floor, 50.0);
        QVERIFY(geozone::pointInZone(z[0], 40.45, -3.7));    // in the ring
        QVERIFY(!geozone::pointInZone(z[0], 40.4, -3.7));    // in the hole
    }

    void regions()
    {
        QCOMPARE(geozone::regionAt(45.46, 9.19), QStringLiteral("EU"));     // Milan
        QCOMPARE(geozone::regionAt(28.1, -15.4), QStringLiteral("EU"));     // Gran Canaria
        QCOMPARE(geozone::regionAt(40.7, -74.0), QStringLiteral("US"));
        QVERIFY(geozone::regionAt(-33.9, 151.2).isEmpty());                 // Sydney
        QCOMPARE(geozone::generalLimit("EU")->metres, 120.0);
    }

    void rules()
    {
        GeoZoneList zones;
        auto add = [&](const QString &key, const QString &kind, double floor, bool agl, double lon, double lat) {
            QJsonObject geom{{"type", "Polygon"}, {"coordinates", one(square(lon, lat, 0.05))}};
            zones.push_back(*geozone::makeZone("t", key, key, kind, floor, agl, std::nullopt, true, geozone::geojsonPolys(geom), {}));
        };
        add("nofly", "prohibited", 0, true, 9.0, 45.0);
        add("ceil60", "prohibited", 60, true, 10.0, 45.0);
        add("cond", "conditional", 0, true, 11.0, 45.0);
        add("msl", "prohibited", 300, false, 12.0, 45.0);

        // In a no-fly zone, airborne: NO-FLY; on the ground nothing from the surface
        auto e = geozone::evaluate(zones, 45.0, 9.0, 30, 100, true);
        QCOMPARE(e.alerts.front().id, QStringLiteral("noFly"));
        e = geozone::evaluate(zones, 45.0, 9.0, 0, 100, false);
        QVERIFY(e.alerts.empty());
        // Under a 60 m ceiling: MAX 60 m (caution); above it: a broken ceiling, over height
        e = geozone::evaluate(zones, 45.0, 10.0, 40, 100, true);
        QCOMPARE(e.alerts.front().id, QStringLiteral("maxHere"));
        QVERIFY(!e.overHeight);
        e = geozone::evaluate(zones, 45.0, 10.0, 70, 100, true);
        QCOMPARE(e.alerts.front().id, QStringLiteral("ceiling"));
        QCOMPARE(e.alerts.front().level, QStringLiteral("warn"));
        QVERIFY(e.overHeight);
        // A floor above sea level, 300 m MSL over ground at 250 m: 50 m above the ground
        e = geozone::evaluate(zones, 45.0, 12.0, 60, 250, true);
        QCOMPARE(e.alerts.front().id, QStringLiteral("ceiling"));
        QCOMPARE(e.alerts.front().metres, 50.0);
        // The general limit of 120 m in Europe, outside every zone
        e = geozone::evaluate(zones, 45.0, 13.0, 130, 100, true);
        QCOMPARE(e.alerts.front().id, QStringLiteral("altLimit"));
        QVERIFY(e.overHeight);
        // Arming alarms
        e = geozone::evaluate(zones, 45.0, 11.0, 0, 100, false);
        auto a = geozone::armingAlert(e.here, 100, e.limit);
        QVERIFY(a);
        QCOMPARE(a->title, QStringLiteral("ARMED IN A RESTRICTED ZONE"));
        e = geozone::evaluate(zones, 45.0, 9.0, 0, 100, false);
        QCOMPARE(geozone::armingAlert(e.here, 100, e.limit)->level, QStringLiteral("warn"));
    }

    void raster()
    {
        GeoZoneList zones;
        QJsonObject geom{{"type", "Polygon"}, {"coordinates", one(square(11.35, 47.26, 0.01))}};
        zones.push_back(*geozone::makeZone("t", "a", "a", "prohibited", 0, true, std::nullopt, true, geozone::geojsonPolys(geom), {}));
        QJsonObject geom2{{"type", "Polygon"}, {"coordinates", one(square(11.40, 47.26, 0.01))}};
        zones.push_back(*geozone::makeZone("t", "b", "b", "prohibited", 80, true, std::nullopt, true, geozone::geojsonPolys(geom2), {}));
        const auto r = GeoZoneRaster::build(zones, 47.26, 11.37, 1);
        QVERIFY(!r->empty);
        const int n = GeoZoneRaster::kSize;
        auto texel = [&](double lat, double lon) {
            const int x = int((lon - r->lon0) / (r->lon1 - r->lon0) * n), y = int((r->lat1 - lat) / (r->lat1 - r->lat0) * n);
            return std::size_t(y) * n + x;
        };
        QCOMPARE(int(r->coverage[texel(47.26, 11.35) * 4 + 0]), 255);   // no-fly from the surface
        QCOMPARE(int(r->coverage[texel(47.26, 11.35) * 4 + 2]), 0);
        QCOMPARE(int(r->coverage[texel(47.26, 11.40) * 4 + 2]), 255);   // a ceiling
        QCOMPARE(int(r->coverage[texel(47.30, 11.37) * 4 + 0]), 0);     // outside
    }
};

QTEST_GUILESS_MAIN(TstGeoZones)
#include "tst_geozones.moc"
