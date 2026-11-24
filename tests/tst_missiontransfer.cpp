// A mission read back from the vehicle: the route it becomes compiles into the same items.
#include "mission/MissionTransfer.h"
#include "mission/RouteCompiler.h"
#include "mission/RouteModel.h"

#include <QTest>

using namespace corv::mission;
using namespace Qt::StringLiterals;

namespace {
const LL kHome{45.05, 9.70};

QJsonObject routeWith(const QJsonArray &segments, double defaultSpeed = 0)
{
    QJsonObject r = createRoute();
    QJsonObject p = defaultRouteParams();
    p.insert(u"takeoff"_s, true);
    p.insert(u"takeoffAlt"_s, 15);
    p.insert(u"altMode"_s, u"amsl"_s);
    p.insert(u"defaultAlt"_s, 130);
    p.insert(u"defaultSpeed"_s, defaultSpeed);
    p.insert(u"endAction"_s, u"rtl"_s);
    p.insert(u"home"_s, QJsonObject{{u"lat"_s, kHome.lat}, {u"lng"_s, kHome.lng}});
    r.insert(u"params"_s, p);
    r.insert(u"segments"_s, segments);
    return r;
}

QJsonObject wp(double north, double speed = 0, const QJsonArray &actions = {})
{
    QJsonObject s = createSegment(u"waypoint"_s, {{kHome.lat + north / 111319.5, kHome.lng}});
    QJsonObject p = s.value(u"params"_s).toObject();
    p.insert(u"speed"_s, speed);
    s.insert(u"params"_s, p);
    s.insert(u"actions"_s, actions);
    return s;
}

QList<RawItem> itemsOf(const QJsonObject &route)
{
    CompileContext ctx;
    ctx.terrain = [](double, double) -> std::optional<double> { return 100.0; };
    ctx.home = kHome;
    ctx.homeAlt = 100;
    ctx.vehicleType = 2;
    return uploadItems(compileRoute(route, ctx).items);
}

/// What flies: command and speed, trigger or position of each item.
QStringList flown(const QList<RawItem> &items)
{
    QStringList out;
    for (const RawItem &i : items)
        out << u"%1/%2/%3/%4/%5"_s.arg(i.command).arg(i.param1, 0, 'f', 1).arg(i.command == 178 ? i.param2 : 0, 0, 'f', 1)
                   .arg(i.param3, 0, 'f', 1).arg(i.alt, 0, 'f', 1);
    return out;
}

QStringList roundTrip(const QJsonObject &route)
{
    return flown(itemsOf(itemsToRoute(itemsOf(route), u"From vehicle"_s, true)));
}
} // namespace

class TestMissionTransfer : public QObject
{
    Q_OBJECT
private slots:
    void speedsStayOnTheirLegs_data()
    {
        QTest::addColumn<QList<double>>("speeds");
        QTest::addColumn<double>("routeSpeed");
        QTest::newRow("each its own") << QList<double>{5, 10, 3} << 0.0;
        QTest::newRow("one, then the same") << QList<double>{5, 0, 0} << 0.0;
        QTest::newRow("from the second on") << QList<double>{0, 8, 0, 4} << 0.0;
        QTest::newRow("over a route speed") << QList<double>{0, 12, 0, 6} << 7.0;
        QTest::newRow("route speed alone") << QList<double>{0, 0, 0} << 7.0;
        QTest::newRow("the last one") << QList<double>{0, 0, 9} << 0.0;
    }

    void speedsStayOnTheirLegs()
    {
        QFETCH(QList<double>, speeds);
        QFETCH(double, routeSpeed);
        QJsonArray segs;
        for (qsizetype i = 0; i < speeds.size(); ++i) segs.append(wp(100.0 * double(i + 1), speeds[i]));
        const QJsonObject route = routeWith(segs, routeSpeed);
        QCOMPARE(roundTrip(route), flown(itemsOf(route)));
    }

    void actionsKeepTheirOrder()
    {
        const QJsonArray acts{createAction(u"speed"_s, QJsonObject{{u"speed"_s, 6}}), createAction(u"wait"_s, QJsonObject{{u"seconds"_s, 2}}),
                              createAction(u"yaw"_s, QJsonObject{{u"heading"_s, 90}}), createAction(u"shot"_s)};
        const QJsonObject route = routeWith({wp(100, 0, acts), wp(200), wp(300, 4, {createAction(u"speed"_s, QJsonObject{{u"speed"_s, 2}})})});
        QCOMPARE(roundTrip(route), flown(itemsOf(route)));
    }

    void cameraTriggerStopsWithoutAPhoto()
    {
        const QJsonArray pts{QJsonObject{{u"lat"_s, 45.05}, {u"lng"_s, 9.70}}, QJsonObject{{u"lat"_s, 45.05}, {u"lng"_s, 9.702}},
                             QJsonObject{{u"lat"_s, 45.048}, {u"lng"_s, 9.702}}, QJsonObject{{u"lat"_s, 45.048}, {u"lng"_s, 9.70}}};
        QJsonObject area = createSegment(u"area"_s, {}, QJsonObject{{u"spacingMode"_s, u"manual"_s}, {u"sideDistance"_s, 40}, {u"trigger"_s, true}});
        area.insert(u"points"_s, pts);
        const QJsonObject route = routeWith({wp(100), area, wp(300)});
        const QList<RawItem> once = itemsOf(route);
        QCOMPARE(std::count_if(once.cbegin(), once.cend(), [](const RawItem &i) { return i.command == 206; }), 2);
        QCOMPARE(roundTrip(route), flown(once));
    }

    void landingAndReturnComeBack()
    {
        QJsonObject land = createSegment(u"landing"_s, {{45.0502, 9.7003}});
        const QJsonObject route = routeWith({wp(100, 5), land});
        QCOMPARE(roundTrip(route), flown(itemsOf(route)));
        const QJsonObject back = itemsToRoute(itemsOf(routeWith({wp(100), wp(200)})), u"x"_s, true);
        QCOMPARE(back.value(u"params"_s).toObject().value(u"endAction"_s).toString(), u"rtl"_s);
        QCOMPARE(back.value(u"params"_s).toObject().value(u"takeoffAlt"_s).toDouble(), 15.0);
    }

    void pointsOfInterest()
    {
        QJsonObject poi = createSegment(u"poi"_s, {{45.051, 9.701}});
        QJsonObject clear = createSegment(u"poi"_s, {{45.051, 9.701}}, QJsonObject{{u"mode"_s, u"clear"_s}});
        QJsonObject land = createSegment(u"landing"_s, {{45.0502, 9.7003}});
        for (const QJsonArray &segs : {QJsonArray{wp(100), poi, wp(200)}, QJsonArray{wp(100), poi, wp(200), clear, wp(300)},
                                       QJsonArray{wp(100), poi, wp(200), land}}) {
            const QJsonObject route = routeWith(segs);
            QCOMPARE(roundTrip(route), flown(itemsOf(route)));
        }
    }

    void wplText()
    {
        const QList<RawItem> items = itemsOf(routeWith({wp(100, 5), wp(200, 9)}));
        QList<RawItem> read;
        QString err;
        QVERIFY(parseWpl(toWpl(items), read, &err));
        QCOMPARE(flown(read), flown(items));
        QVERIFY(!parseWpl(u"not a mission"_s, read, &err));
        QVERIFY(!err.isEmpty());
    }
};

QTEST_GUILESS_MAIN(TestMissionTransfer)
#include "tst_missiontransfer.moc"
