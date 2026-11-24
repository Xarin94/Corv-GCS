// The mission route and the traffic trails of the 3D view: Mission3D.js and Traffic3D.js
// as geometry.
#include "scene/Mission3D.h"
#include "scene/TrafficTrails.h"

#include <QTest>
#include <algorithm>
#include <cmath>

using namespace corv;
using namespace corv::mission3d;
using namespace Qt::StringLiterals;

namespace {
Point at(float x, float y, float z, int seq, const QString &kind = u"wp"_s)
{
    Point p;
    p.pos = {x, y, z};
    p.seq = seq;
    p.kind = kind;
    p.ground = 0.f;
    p.alt = y;
    return p;
}
int count(const Drawing &d, Style s)
{
    return int(std::count_if(d.lines.begin(), d.lines.end(), [s](const Line &l) { return l.style == s; }));
}
const Mark *mark(const Drawing &d, int seq)
{
    for (const Mark &m : d.marks)
        if (m.seq == seq) return &m;
    return nullptr;
}
} // namespace

class TestMission3D : public QObject
{
    Q_OBJECT
private slots:
    void route()
    {
        const Drawing d = build({at(0, 0, 0, 0, u"home"_s), at(0, 50, 0, 1, u"takeoff"_s), at(100, 50, 0, 2)}, {}, std::nullopt);
        QCOMPARE(d.lines.size(), std::size_t(2));
        QCOMPARE(d.lines[1].seq, 2);
        QCOMPARE(d.lines[1].b, QVector3D(100, 50, 0));
        // Home carries its own marker
        QCOMPARE(d.marks.size(), std::size_t(2));
        QCOMPARE(mark(d, 1)->main, u"1 T/O"_s);
        QCOMPARE(mark(d, 1)->shape, Shape::Triangle);
        QCOMPARE(mark(d, 2)->sub, u"50 m"_s);
        QVERIFY(mark(d, 2)->plain);
    }

    void patternIsDashedAndMarkedAtItsEnds()
    {
        std::vector<Point> pts{at(0, 0, 0, 0, u"home"_s)};
        for (int i = 0; i < 6; ++i) {
            Point p = at(float(i * 10), 40, float(i % 2 * 100), i + 1);
            p.segId = u"a"_s;
            p.segType = u"area"_s;
            pts.push_back(p);
        }
        // A terrain-following point of the leg that leads to the pattern, and one inside it
        Point before = at(-5, 40, 0, 100);
        before.derived = true;
        before.segId = u"a"_s;
        before.segType = u"area"_s;
        pts.insert(pts.begin() + 1, before);
        Point inside = before;
        inside.pos = {25, 45, 50};
        inside.seq = 101;
        pts.insert(pts.begin() + 4, inside);

        const Drawing d = build(pts, {}, std::nullopt);
        QCOMPARE(count(d, Style::Route), 2);      // home -> derived -> entry
        QCOMPARE(count(d, Style::Pattern), 6);    // five passes, one split by the derived point
        QCOMPARE(d.marks.size(), std::size_t(2));
        QCOMPARE(d.marks[0].main, u"1 AREA"_s);
        QCOMPARE(d.marks[1].main, u"6"_s);
        for (const Line &l : d.lines)
            if (l.style == Style::Pattern) QCOMPARE(l.segType, u"area"_s);
    }

    void circleIsEnteredAndLeftOnItsRing()
    {
        Point c = at(1000, 80, 0, 2, u"loiter"_s);
        c.loiterRadius = 100;
        c.turns = 2;
        const Drawing d = build({at(0, 80, 0, 1), c, at(2000, 80, 0, 3)}, {}, std::nullopt);
        // From the west: entry at the westernmost point, and the next leg leaves from there
        const Line &in = d.lines[0];
        QVERIFY(qAbs(in.b.x() - 900) < 0.1f && qAbs(in.b.z()) < 0.1f);
        const auto out = std::find_if(d.lines.begin(), d.lines.end(), [](const Line &l) { return l.seq == 3; });
        QVERIFY((out->a - in.b).length() < 0.1f);
        QCOMPARE(count(d, Style::Ring), 72 + 8);   // and four arrows
        for (const Line &l : d.lines)
            if (l.style == Style::Ring && l.seq == 2) QVERIFY((l.a - c.pos).length() < 135);
        QCOMPARE(mark(d, 2)->main, u"2 CIRC"_s);
        QCOMPARE(mark(d, 2)->sub, u"80 m · 2×"_s);
        // The arrows point the way it turns: clockwise seen from above, so at the north-east one
        // the tip is farther clockwise than its tail
        auto bearing = [&](const QVector3D &v) { return std::atan2(v.x() - c.pos.x(), c.pos.z() - v.z()); };
        const Line &arrow = d.lines[std::size_t(1 + 72)];
        QVERIFY(bearing(arrow.b) > bearing(arrow.a));
        Point ccw = c;
        ccw.ccw = true;
        const Drawing e = build({at(0, 80, 0, 1), ccw}, {}, std::nullopt);
        const Line &back = e.lines[std::size_t(1 + 72)];
        QVERIFY(bearing(back.b) < bearing(back.a));
    }

    void landing()
    {
        Point land = at(100, 0, 0, 2, u"land"_s);
        const Drawing d = build({at(0, 60, 0, 1), land}, {}, std::nullopt);
        QCOMPARE(d.lines.size(), std::size_t(2));   // across at height, then straight down
        QCOMPARE(d.lines[0].b, QVector3D(100, 60, 0));
        QCOMPARE(d.lines[1].b, QVector3D(100, 0, 0));
        QVERIFY(mark(d, 2)->sub.isEmpty());
        land.glide = true;
        const Drawing g = build({at(0, 60, 0, 1), land}, {}, std::nullopt);
        QCOMPARE(g.lines.size(), std::size_t(1));   // a fixed wing's glide slope
    }

    void returnToLaunchAndPoi()
    {
        Poi poi;
        poi.pos = {50, 30, 50};
        poi.ground = 10.f;
        poi.seq = 7;
        const Drawing d = build({at(0, 5, 0, 0, u"home"_s), at(300, 60, 0, 1)}, {poi}, 2);
        QCOMPARE(count(d, Style::Return), 2);
        const auto r = std::find_if(d.lines.begin(), d.lines.end(), [](const Line &l) { return l.style == Style::Return; });
        QCOMPARE(r->b, QVector3D(0, 60, 0));
        QCOMPARE(r->seq, 2);
        QCOMPARE(mark(d, 7)->main, u"7 POI"_s);
        QCOMPARE(mark(d, 7)->sub, u"20 m"_s);
        QCOMPARE(mark(d, 7)->shape, Shape::Target);
        QVERIFY(std::any_of(d.marks.begin(), d.marks.end(), [](const Mark &m) { return m.main == u"RTL"_s && !m.ground; }));
        // Nothing to come back to without a home
        QCOMPARE(count(build({at(300, 60, 0, 1)}, {}, 2), Style::Return), 0);
    }

    void trafficIsDeadReckoned()
    {
        TrafficTrails t;
        t.update({{u"4ca1"_s, 45, 10, 3000, 200, 90, 5, 100000}});
        double lat = 0, lon = 0, alt = 0;
        QVERIFY(t.head(u"4ca1"_s, 110000, &lat, &lon, &alt));
        QCOMPARE(alt, 3050.0);
        QCOMPARE(lat, 45.0);
        QVERIFY(qAbs((lon - 10) * 111320 * std::cos(45 * M_PI / 180) - 2000) < 5);
        // Never further than 30 s past the last report
        double later = 0;
        t.head(u"4ca1"_s, 200000, &lat, &later, &alt);
        QCOMPARE(alt, 3150.0);
        // Gone from the list: gone
        t.update({});
        QCOMPARE(t.count(), 0);
    }

    void trafficTrailBendsAndFades()
    {
        TrafficTrails t;
        // North for a while, then east: the curve passes the corner outside the chord
        const double d = 4000 / 111320.0, e = d / std::cos(45 * M_PI / 180);
        t.update({{u"a"_s, 45, 10, 1000, 200, 0, 0, 100000}});
        t.update({{u"a"_s, 45 + d, 10 + e, 1000, 200, 90, 0, 130000}});
        const auto trail = t.trail(u"a"_s, 130000);
        QVERIFY(trail.size() > 10);
        QVERIFY(trail.back().alpha > 0.99);
        QVERIFY(trail.front().alpha < trail.back().alpha);
        // Half way along the straight chord both are at half; the curve has gone north first
        const auto &mid = trail[trail.size() / 2];
        QVERIFY((mid.lat - 45) / d > (mid.lon - 10) / e + 0.1);
        // The same position repeated with a newer time is no new sample
        t.update({{u"a"_s, 45 + d, 10 + e, 1000, 200, 90, 0, 160000}});
        QCOMPARE(t.trail(u"a"_s, 130000).size(), trail.size());
        // Three minutes later nothing of it shows
        double sum = 0;
        for (const auto &v : t.trail(u"a"_s, 130000 + 215000)) sum += v.alpha;
        QVERIFY(sum < 1e-6);
    }
};

QTEST_GUILESS_MAIN(TestMission3D)
#include "tst_mission3d.moc"
