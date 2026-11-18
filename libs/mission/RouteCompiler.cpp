#include "mission/RouteCompiler.h"

#include "mission/Platforms.h"
#include "mission/RouteGeo.h"

#include <QJsonArray>
#include <QSet>
#include <algorithm>
#include <cmath>
#include <limits>

namespace corv::mission {

namespace {

constexpr double kD2R = M_PI / 180.0;
constexpr double kClimbRate = 2.5;     // m/s, for the duration estimate only
constexpr double kDescentRate = 1.5;
constexpr int kMaxPhotos = 4000;

QString str(const QJsonObject &o, const char *k) { return o.value(QLatin1String(k)).toString(); }
double n(const QJsonObject &o, const char *k) { return num(o.value(QLatin1String(k))); }

// ── Survey patterns ──────────────────────────────────────────────────────────

struct Rotator {
    double c = 1, s = 0;
    explicit Rotator(double headingDeg)
    {
        // Lane heading clockwise from north; rotate so the lane runs along +x (east)
        const double a = -std::atan2(std::cos(headingDeg * kD2R), std::sin(headingDeg * kD2R));
        c = std::cos(a);
        s = std::sin(a);
    }
    XY fwd(const XY &p) const { return {p.x * c - p.y * s, p.x * s + p.y * c}; }
    XY inv(const XY &p) const { return {p.x * c + p.y * s, -p.x * s + p.y * c}; }
};

struct LanePoint {
    double lat = 0, lng = 0;
    QString lane;
};
struct Pattern {
    QList<LanePoint> points;
    int count = 0;
};

/// Boustrophedon lanes across a polygon, centred so the outermost lanes sit half a
/// spacing from the boundary, entered at the corner nearest to `startNear`.
Pattern lawnmower(const QList<LL> &polyLL, double headingDeg, double spacing, double overshoot, const std::optional<LL> &startNear)
{
    const LocalFrame f(*centroid(polyLL));
    const Rotator rot(headingDeg);
    QList<XY> Q;
    for (const LL &p : polyLL) Q.append(rot.fwd(f.toXY(p)));
    double minY = std::numeric_limits<double>::infinity(), maxY = -minY;
    for (const XY &q : Q) { minY = std::min(minY, q.y); maxY = std::max(maxY, q.y); }
    const double extent = maxY - minY;
    const int nLanes = std::max(1, int(std::ceil(extent / spacing)));
    const double y0 = minY + (extent - (nLanes - 1) * spacing) / 2;

    struct Lane { double y; QList<std::pair<double, double>> spans; };
    QList<Lane> lanes;
    for (int k = 0; k < nLanes; ++k) {
        const double y = y0 + k * spacing;
        QList<double> xs;
        for (qsizetype i = 0, j = Q.size() - 1; i < Q.size(); j = i++) {
            const XY &a = Q[j], &b = Q[i];
            if ((a.y <= y && b.y > y) || (b.y <= y && a.y > y)) xs.append(a.x + (y - a.y) * (b.x - a.x) / (b.y - a.y));
        }
        std::sort(xs.begin(), xs.end());
        Lane lane{y, {}};
        for (qsizetype c = 0; c + 1 < xs.size(); c += 2) {
            if (xs[c + 1] - xs[c] < 0.5) continue;
            lane.spans.append({xs[c] - overshoot, xs[c + 1] + overshoot});
        }
        if (!lane.spans.isEmpty()) lanes.append(lane);
    }
    if (lanes.isEmpty()) return {};

    struct P { XY q; QString lane; };
    auto build = [&](bool reverseLanes, bool startRight) {
        QList<P> pts;
        for (qsizetype idx = 0; idx < lanes.size(); ++idx) {
            const Lane &lane = lanes[reverseLanes ? lanes.size() - 1 - idx : idx];
            const bool rightward = (idx % 2 == 0) != startRight;
            QList<std::pair<double, double>> spans = lane.spans;
            if (!rightward) std::reverse(spans.begin(), spans.end());
            for (const auto &[x0, x1] : spans) {
                const QString laneId = QStringLiteral("%1:%2").arg(lane.y, 0, 'f', 2).arg(x0, 0, 'f', 1);
                if (rightward) { pts.append({{x0, lane.y}, laneId}); pts.append({{x1, lane.y}, laneId}); }
                else { pts.append({{x1, lane.y}, laneId}); pts.append({{x0, lane.y}, laneId}); }
            }
        }
        return pts;
    };
    std::optional<XY> near;
    if (startNear) near = rot.fwd(f.toXY(*startNear));
    QList<P> best;
    double bestD = std::numeric_limits<double>::infinity();
    for (bool rl : {false, true}) {
        bool done = false;
        for (bool sr : {false, true}) {
            QList<P> pts = build(rl, sr);
            const double d = near ? std::hypot(pts[0].q.x - near->x, pts[0].q.y - near->y) : 0;
            if (d < bestD) { bestD = d; best = pts; }
            if (!near) { done = true; break; }
        }
        if (done) break;
    }
    Pattern out;
    out.count = int(lanes.size());
    for (const P &p : best) {
        const LL ll = f.toLL(rot.inv(p.q));
        out.points.append({ll.lat, ll.lng, p.lane});
    }
    return out;
}

/// Offset a planar polyline by `d` metres (positive = left of travel), mitred corners.
QList<XY> offsetPolyline(const QList<XY> &P, double d)
{
    const qsizetype cnt = P.size();
    QList<XY> out;
    auto norm = [&](qsizetype i, qsizetype j) {
        const double dx = P[j].x - P[i].x, dy = P[j].y - P[i].y;
        double l = std::hypot(dx, dy);
        if (l == 0) l = 1;
        return XY{-dy / l, dx / l};
    };
    for (qsizetype i = 0; i < cnt; ++i) {
        XY nv;
        if (i == 0) nv = norm(0, 1);
        else if (i == cnt - 1) nv = norm(cnt - 2, cnt - 1);
        else {
            const XY a = norm(i - 1, i), b = norm(i, i + 1);
            nv = {a.x + b.x, a.y + b.y};
            const double l = std::hypot(nv.x, nv.y);
            if (l < 1e-6) nv = a;
            else {
                nv.x /= l; nv.y /= l;
                const double cosHalf = std::max(0.33, nv.x * a.x + nv.y * a.y);   // mitre limit ≈ 3
                nv.x /= cosHalf; nv.y /= cosHalf;
            }
        }
        out.append({P[i].x + nv.x * d, P[i].y + nv.y * d});
    }
    return out;
}

/// Parallel passes along a polyline, covering `width` centred on it.
Pattern corridorPasses(const QList<LL> &lineLL, double width, double spacing, const std::optional<LL> &startNear)
{
    const LocalFrame f(*centroid(lineLL));
    QList<XY> P;
    for (const LL &p : lineLL) P.append(f.toXY(p));
    const int cnt = std::max(1, int(std::lround(width / spacing)));
    QList<QList<XY>> passes;
    for (int k = 0; k < cnt; ++k) passes.append(offsetPolyline(P, (k - (cnt - 1) / 2.0) * spacing));

    struct Q { XY q; QString lane; };
    auto build = [&](bool reverse, bool startBack) {
        QList<Q> pts;
        for (int idx = 0; idx < cnt; ++idx) {
            const int passIdx = reverse ? cnt - 1 - idx : idx;
            const bool back = (idx % 2 == 1) != startBack;
            QList<XY> seq = passes[passIdx];
            if (back) std::reverse(seq.begin(), seq.end());
            for (const XY &q : seq) pts.append({q, QStringLiteral("p%1").arg(passIdx)});
        }
        return pts;
    };
    std::optional<XY> near;
    if (startNear) near = f.toXY(*startNear);
    QList<Q> best;
    double bestD = std::numeric_limits<double>::infinity();
    for (bool rv : {false, true}) {
        bool done = false;
        for (bool sb : {false, true}) {
            QList<Q> pts = build(rv, sb);
            const double d = near ? std::hypot(pts[0].q.x - near->x, pts[0].q.y - near->y) : 0;
            if (d < bestD) { bestD = d; best = pts; }
            if (!near) { done = true; break; }
        }
        if (done) break;
    }
    Pattern out;
    out.count = cnt;
    for (const Q &q : best) {
        const LL ll = f.toLL(q.q);
        out.points.append({ll.lat, ll.lng, q.lane});
    }
    return out;
}

// ── Terrain following ────────────────────────────────────────────────────────

void splitLeg(const CompiledItem &a, const CompiledItem &b, const TerrainFn &terrain, double tol, QList<CompiledItem> &acc, int depth)
{
    if (depth > 7) return;
    const double len = haversine(a.ll(), b.ll());
    if (len < 40) return;
    const int cnt = std::min(300, std::max(4, int(std::ceil(len / 20))));
    const auto ta = terrain(a.lat, a.lng), tb = terrain(b.lat, b.lng);
    if (!ta || !tb) return;
    const double mslA = *ta + a.alt, mslB = *tb + b.alt;
    double worst = 0, worstT = -1;
    for (int s = 1; s < cnt; ++s) {
        const double t = double(s) / cnt;
        const auto g = terrain(a.lat + (b.lat - a.lat) * t, a.lng + (b.lng - a.lng) * t);
        if (!g) continue;
        const double wanted = *g + (a.alt + (b.alt - a.alt) * t);
        const double straight = mslA + (mslB - mslA) * t;
        const double dev = std::abs(wanted - straight);
        if (dev > worst) { worst = dev; worstT = t; }
    }
    if (worst <= tol || worstT < 0) return;
    CompiledItem mid;
    mid.command = b.command == 82 ? 82 : 16;
    mid.loc = true;
    mid.derived = true;
    mid.segId = b.segId;
    mid.lat = a.lat + (b.lat - a.lat) * worstT;
    mid.lng = a.lng + (b.lng - a.lng) * worstT;
    mid.alt = a.alt + (b.alt - a.alt) * worstT;
    splitLeg(a, mid, terrain, tol, acc, depth + 1);
    acc.append(mid);
    splitLeg(mid, b, terrain, tol, acc, depth + 1);
}

/// Insert derived waypoints wherever the straight line between two AGL points would
/// deviate more than `tol` metres from the terrain-following altitude (Douglas-Peucker).
QList<CompiledItem> subdivideForTerrain(const QList<CompiledItem> &items, double tol, const TerrainFn &terrain)
{
    QList<CompiledItem> result;
    const CompiledItem *prev = nullptr;
    for (const CompiledItem &it : items) {
        if (it.loc && !it.roi && !it.isHome && !it.landing && prev && !prev->landing) {
            QList<CompiledItem> inserted;
            splitLeg(*prev, it, terrain, tol, inserted, 0);
            result.append(inserted);
        }
        result.append(it);
        if (it.loc && !it.roi) prev = &it;
    }
    return result;
}

/// Minimum clearance along a straight leg (m), or nullopt without terrain data.
std::optional<double> legClearance(const CompiledItem &a, const CompiledItem &b, const TerrainFn &terrain)
{
    const double len = haversine(a.ll(), b.ll());
    const int cnt = std::min(200, std::max(2, int(std::ceil(len / 25))));
    std::optional<double> mn;
    for (int s = 1; s < cnt; ++s) {
        const double t = double(s) / cnt;
        const auto g = terrain(a.lat + (b.lat - a.lat) * t, a.lng + (b.lng - a.lng) * t);
        if (!g) continue;
        const double c = a.altMsl + (b.altMsl - a.altMsl) * t - *g;
        if (!mn || c < *mn) mn = c;
    }
    return mn;
}

// ── Camera preview ───────────────────────────────────────────────────────────

struct Shot {
    int itemIndex;     // index into navLocated
    QString segId;
    double pitch, yaw;
};

CameraPreview buildCameraPreview(const QJsonObject &route, const QList<const CompiledItem *> &navLocated, const QList<Shot> &shots,
                                 QHash<QString, int> &photosBySeg)
{
    const QJsonObject cam = route.value(QStringLiteral("params")).toObject().value(QStringLiteral("camera")).toObject();
    CameraPreview out;
    auto aglOf = [](const CompiledItem &it) { return it.terrain ? it.altMsl - *it.terrain : it.altRel; };

    // Trigger-by-distance along survey lanes: consecutive base points sharing a lane id
    struct Lane { QString id, segId; double trigger, pitch, yaw, agl; QList<LL> pts; };
    std::optional<Lane> lane;
    auto flushLane = [&] {
        if (!lane || lane->pts.size() < 2) { lane.reset(); return; }
        for (const PhotoPos &ph : photosAlong(lane->pts, lane->trigger)) {
            if (out.photos.size() >= kMaxPhotos) break;
            const QList<LL> poly = groundFootprint({ph.lat, ph.lng}, lane->agl, ph.heading + lane->yaw, lane->pitch, cam);
            if (!poly.isEmpty()) out.photos.append({ph.lat, ph.lng, poly, lane->segId, false});
            photosBySeg[lane->segId] += 1;
        }
        lane.reset();
    };
    for (const CompiledItem *it : navLocated) {
        if (it->derived || it->lane.isEmpty() || !(it->trigger > 0)) { flushLane(); continue; }
        if (lane && (lane->segId != it->segId || lane->id != it->lane)) flushLane();
        if (!lane) lane = Lane{it->lane, it->segId, it->trigger, it->gimbalPitch, it->gimbalYaw, std::max(1.0, aglOf(*it)), {}};
        lane->pts.append(it->ll());
    }
    flushLane();

    // Single shots: at the waypoint just reached, looking along the next leg
    for (const Shot &sh : shots) {
        const CompiledItem &it = *navLocated[sh.itemIndex];
        const CompiledItem *next = sh.itemIndex + 1 < navLocated.size() ? navLocated[sh.itemIndex + 1]
                                 : sh.itemIndex > 0 ? navLocated[sh.itemIndex - 1] : nullptr;
        const double heading = next ? bearing(it.ll(), next->ll()) : 0;
        const QList<LL> poly = groundFootprint(it.ll(), std::max(1.0, aglOf(it)), heading + sh.yaw, sh.pitch, cam);
        if (!poly.isEmpty()) out.photos.append({it.lat, it.lng, poly, sh.segId, true});
        photosBySeg[sh.segId] += 1;
    }

    // POI: from every base point flown while the ROI is active, the wedge the camera sees
    for (const CompiledItem *pit : navLocated) {
        const CompiledItem &it = *pit;
        if (!it.roiTarget || it.derived || it.isHome || it.landing || !it.lane.isEmpty()) continue;
        const double agl = std::max(1.0, aglOf(it));
        const RoiTarget &t = *it.roiTarget;
        const double targetH = t.alt - (it.altRel - agl);
        QList<LL> spots;
        if (it.loiter) {
            const LocalFrame f(it.ll());
            for (int k = 0; k < 6; ++k) {
                const double a = k * M_PI / 3;
                spots.append(f.toLL({it.loiter->r * std::sin(a), it.loiter->r * std::cos(a)}));
            }
        } else {
            spots.append(it.ll());
        }
        for (const LL &pos : spots) {
            const Aim aim = aimAt(pos, agl, {t.lat, t.lng}, targetH);
            const QList<LL> poly = groundFootprint(pos, agl, aim.azimuth, aim.pitch, cam, std::min(aim.distance * 1.4, aim.distance + 4 * agl));
            if (!poly.isEmpty()) out.cones.append({pos, {t.lat, t.lng}, poly, it.segId, t.segId, !it.loiter.has_value()});
        }
    }
    out.truncated = out.photos.size() >= kMaxPhotos;
    return out;
}

// ── Statistics ───────────────────────────────────────────────────────────────

RouteStats buildStats(const QJsonObject &P, const QList<CompiledItem> &items, const QList<const CompiledItem *> &navLocated,
                      const QHash<QString, SegStats> &segStats, double homeElev, const LL &home)
{
    RouteStats s;
    s.items = int(items.size());
    for (const CompiledItem *it : navLocated) if (!it->isHome) ++s.waypoints;
    for (const CompiledItem *it : navLocated) {
        if (it->isHome || it->landing) continue;
        if (it->terrain) {
            const double agl = it->altMsl - *it->terrain;
            s.minAgl = s.minAgl ? std::min(*s.minAgl, agl) : agl;
            s.maxAgl = s.maxAgl ? std::max(*s.maxAgl, agl) : agl;
        }
        s.minAmsl = s.minAmsl ? std::min(*s.minAmsl, it->altMsl) : it->altMsl;
        s.maxAmsl = s.maxAmsl ? std::max(*s.maxAmsl, it->altMsl) : it->altMsl;
    }
    for (const SegStats &st : segStats) { s.photos += st.photos; s.areaM2 += st.area; }

    // Length & duration follow the item sequence so speed changes are honoured
    double speed = 0, len = 0, dur = 0;
    const CompiledItem *prev = nullptr;
    for (const CompiledItem &it : items) if (it.firstLeg) speed = it.param2;
    for (const CompiledItem &it : items) if (it.kind == QLatin1String("takeoff")) { const double ta = n(P, "takeoffAlt"); dur += (ta ? ta : 30) / kClimbRate; break; }
    for (const CompiledItem &it : items) {
        if (it.command == 178) { speed = it.param2; continue; }
        if (it.command == 93) { dur += it.param1; continue; }
        if (!it.loc || it.roi) continue;
        if (prev) {
            const double d = haversine(prev->ll(), it.ll());
            len += d;
            dur += speed > 0 ? d / speed : d / 10;
        }
        if (it.loiter) {
            const double c = 2 * M_PI * it.loiter->r * it.loiter->turns;
            len += c;
            dur += speed > 0 ? c / speed : c / 10;
        }
        if (it.landing) dur += std::max(0.0, (prev ? prev->altMsl : it.altMsl) - it.altMsl) / kDescentRate;
        prev = &it;
    }
    if (!items.isEmpty() && items.last().kind == QLatin1String("rtl") && prev) {
        const double d = haversine(prev->ll(), home);
        len += d;
        dur += (speed > 0 ? d / speed : d / 10) + std::max(0.0, prev->altMsl - homeElev) / kDescentRate;
    }
    s.lengthM = len;
    s.durationS = dur;
    return s;
}

} // namespace

// ── Public ───────────────────────────────────────────────────────────────────

QList<LL> corridorOutline(const QList<LL> &line, double width)
{
    if (line.size() < 2) return {};
    const LocalFrame f(*centroid(line));
    QList<XY> P;
    for (const LL &p : line) P.append(f.toXY(p));
    QList<XY> left = offsetPolyline(P, width / 2);
    QList<XY> right = offsetPolyline(P, -width / 2);
    std::reverse(right.begin(), right.end());
    QList<LL> out;
    for (const XY &q : left) out.append(f.toLL(q));
    for (const XY &q : right) out.append(f.toLL(q));
    return out;
}

CompiledRoute compileRoute(const QJsonObject &route, const CompileContext &ctx)
{
    const QJsonObject P = route.value(QStringLiteral("params")).toObject();
    const QJsonArray segments = route.value(QStringLiteral("segments")).toArray();
    const Platform &plat = platform(ctx.platform);
    const bool isInav = plat.id == QLatin1String("inav");
    const int itemLimit = ctx.itemLimit > 0 ? ctx.itemLimit : plat.itemLimit;
    const TerrainFn terrain = ctx.terrain ? ctx.terrain : TerrainFn([](double, double) { return std::optional<double>(); });

    CompiledRoute R;
    R.platform = plat.id;
    auto addIssue = [&](const QString &level, const QString &text, const QString &segId = {}) { R.issues.append({level, text, segId}); };

    // ── Take-off point ──
    std::optional<LL> firstPt;
    for (const QJsonValue &v : segments) {
        const QList<LL> pts = pointsOf(v.toObject());
        if (!pts.isEmpty()) { firstPt = pts.first(); break; }
    }
    std::optional<LL> home;
    const QJsonObject ph = P.value(QStringLiteral("home")).toObject();
    if (!ph.isEmpty()) home = LL{n(ph, "lat"), n(ph, "lng")};
    else if (ctx.home) home = ctx.home;
    else home = firstPt;
    if (!home || segments.isEmpty()) {
        if (segments.isEmpty()) addIssue(QStringLiteral("info"), QStringLiteral("Pick a tool on the map and start drawing"));
        if (home) {
            const auto t = terrain(home->lat, home->lng);
            R.home = Home{home->lat, home->lng, t ? *t : ctx.homeAlt};
        }
        return R;
    }
    const auto homeTerrain = terrain(home->lat, home->lng);
    const double homeElev = homeTerrain ? *homeTerrain : ctx.homeAlt;
    if (!homeTerrain) addIssue(QStringLiteral("warn"), QStringLiteral("No elevation data at the take-off point — vehicle home altitude assumed"));

    const bool isPlane = ctx.vehicleType == 1;
    // A rover, a boat or a sub has nothing to take off from
    const bool onSurface = ctx.vehicleType == 10 || ctx.vehicleType == 11 || ctx.vehicleType == 12;

    QList<CompiledItem> out;
    auto push = [&](const CompiledItem &it) -> int { out.append(it); return int(out.size()) - 1; };
    auto doItem = [&](int command, double p1, double p2, double p3, double p4, const QString &segId, double alt = 0) {
        CompiledItem it;
        it.command = command;
        it.segId = segId;
        it.param1 = p1; it.param2 = p2; it.param3 = p3; it.param4 = p4;
        it.alt = alt;
        return push(it);
    };

    {
        CompiledItem h;
        h.command = 16;
        h.lat = home->lat; h.lng = home->lng;
        h.loc = true;
        h.isHome = true;
        push(h);
    }
    // INAV has no take-off waypoint at all; the operator is told so
    const bool takeoff = truthy(P.value(QStringLiteral("takeoff")));
    if (takeoff && isInav) addIssue(QStringLiteral("warn"), QStringLiteral("INAV has no take-off waypoint — arm and take off in a flight mode, then switch to WP"));
    if (takeoff && !isInav && !onSurface) {
        CompiledItem t;
        t.command = 22;
        t.lat = home->lat; t.lng = home->lng;
        const double ta = n(P, "takeoffAlt");
        t.alt = ta ? ta : 30;
        t.loc = true;
        t.param1 = isPlane ? 15 : 0;
        t.kind = QStringLiteral("takeoff");
        push(t);
    }

    double currentSpeed = 0;
    auto setSpeed = [&](double v, const QString &segId) {
        if (v > 0 && std::abs(v - currentSpeed) > 0.01) {
            doItem(178, 1, v, -1, 0, segId);
            currentSpeed = v;
        }
    };
    if (n(P, "defaultSpeed") > 0) setSpeed(n(P, "defaultSpeed"), {});

    LL lastNav = *home;
    int lastNavItem = -1;
    bool roiActive = false;
    std::optional<RoiTarget> roiTarget;
    const QJsonValue gp = P.value(QStringLiteral("gimbalPitch"));
    double gimbalPitch = std::isfinite(num(gp, NAN)) ? num(gp, NAN) : -90;
    double gimbalYaw = n(P, "gimbalYaw");
    // The nav item a photo is taken at, by identity: the boat reorder and the terrain
    // subdivision below move items around
    struct PendingShot { QString navSegId; int navSegIdx; double lat, lng; QString segId; double pitch, yaw; };
    QList<PendingShot> pendingShots;

    // ── Segments ──
    for (const QJsonValue &sv : segments) {
        const QJsonObject seg = sv.toObject();
        const QString type = str(seg, "type");
        if (!segmentTypes().contains(type)) continue;
        const QString segId = str(seg, "id");
        const QJsonObject sp = seg.value(QStringLiteral("params")).toObject();
        const QList<LL> pts = pointsOf(seg);
        const double alt = segmentAlt(seg, P);
        SegStats &stats = R.segStats[segId];
        stats = SegStats{};
        const double speed = n(sp, "speed") > 0 ? n(sp, "speed") : n(P, "defaultSpeed");

        if (type == QLatin1String("poi")) {
            if (str(sp, "mode") == QLatin1String("clear")) {
                doItem(197, 0, 0, 0, 0, segId);
                roiActive = false;
                roiTarget.reset();
            } else {
                if (pts.isEmpty()) continue;
                CompiledItem it;
                it.command = 201;
                it.lat = pts[0].lat; it.lng = pts[0].lng;
                it.alt = n(sp, "alt");
                it.loc = true;
                it.roi = true;
                it.segId = segId;
                push(it);
                roiActive = true;
                roiTarget = RoiTarget{pts[0].lat, pts[0].lng, n(sp, "alt"), segId};
            }
            continue;
        }

        struct NavPt { double lat, lng, alt; int command; double p1 = 0, p2 = 0, p3 = 0; std::optional<Loiter> loiter; bool landing = false; QString lane; };
        QList<NavPt> navPts;
        double navTrigger = 0;
        const QString turn = str(sp, "turn");
        const int cmdWp = ((turn == QLatin1String("route") || turn.isEmpty() ? str(P, "turnType") : turn) == QLatin1String("spline")) ? 82 : 16;

        if (type == QLatin1String("waypoint")) {
            if (!pts.isEmpty()) navPts.append({pts[0].lat, pts[0].lng, alt, cmdWp, n(sp, "delay"), n(sp, "acceptRadius")});
        } else if (type == QLatin1String("circle")) {
            if (!pts.isEmpty()) {
                const double rr = n(sp, "radius");
                const double r = std::max(5.0, rr ? rr : 50);
                const double tt = n(sp, "turns");
                const int turns = std::max(1, int(std::lround(tt ? tt : 1)));
                NavPt np{pts[0].lat, pts[0].lng, alt, 18, double(turns), 0, str(sp, "direction") == QLatin1String("ccw") ? -r : r};
                np.loiter = Loiter{r, turns};
                navPts.append(np);
            }
        } else if (type == QLatin1String("perimeter")) {
            if (pts.size() < 3) {
                addIssue(QStringLiteral("error"), QStringLiteral("Perimeter needs at least 3 vertices"), segId);
            } else {
                QList<LL> ring = pts;
                if (str(sp, "direction") == QLatin1String("ccw")) {
                    QList<LL> rev{ring[0]};
                    for (qsizetype i = ring.size() - 1; i >= 1; --i) rev.append(ring[i]);
                    ring = rev;
                }
                if (truthy(sp.value(QStringLiteral("closed")))) ring.append(ring[0]);
                for (const LL &p : ring) navPts.append({p.lat, p.lng, alt, cmdWp});
            }
        } else if (type == QLatin1String("area")) {
            if (pts.size() < 3) {
                addIssue(QStringLiteral("error"), QStringLiteral("Area scan needs at least 3 vertices"), segId);
            } else {
                const SurveyGeometry g = surveyGeometry(seg, alt, P.value(QStringLiteral("camera")).toObject());
                if (g.hasSensor && !g.sensor.swath) {
                    addIssue(QStringLiteral("error"), QStringLiteral("The surface (%1 m) is beyond the sensor's range").arg(g.sensorDistance), segId);
                } else {
                    stats.gsd = g.gsd;
                    stats.area = polygonArea(pts);
                    if (g.hasSensor) stats.swath = g.sensor.swath;
                    const double angle = n(sp, "angle");
                    Pattern lanes = lawnmower(pts, angle, g.sideDistance, n(sp, "overshoot"), lastNav);
                    stats.lanes = lanes.count;
                    if (truthy(sp.value(QStringLiteral("doubleGrid")))) {
                        const std::optional<LL> from = lanes.points.isEmpty() ? std::optional<LL>(lastNav)
                                                                              : std::optional<LL>(LL{lanes.points.last().lat, lanes.points.last().lng});
                        Pattern cross = lawnmower(pts, angle + 90, g.sideDistance, n(sp, "overshoot"), from);
                        for (LanePoint &q : cross.points) q.lane = QStringLiteral("x") + q.lane;
                        lanes.points.append(cross.points);
                        stats.lanes += cross.count;
                    }
                    if (lanes.points.isEmpty()) {
                        addIssue(QStringLiteral("error"), QStringLiteral("Area too small for the lane spacing — reduce side distance or overlap"), segId);
                    } else {
                        for (const LanePoint &q : lanes.points) { NavPt np{q.lat, q.lng, alt, 16}; np.lane = q.lane; navPts.append(np); }
                        if (truthy(sp.value(QStringLiteral("trigger"))) && !g.hasSensor) navTrigger = g.triggerDistance;
                    }
                }
            }
        } else if (type == QLatin1String("corridor")) {
            if (pts.size() < 2) {
                addIssue(QStringLiteral("error"), QStringLiteral("Corridor needs at least 2 points"), segId);
            } else {
                const SurveyGeometry g = surveyGeometry(seg, alt, P.value(QStringLiteral("camera")).toObject());
                if (g.hasSensor && !g.sensor.swath) {
                    addIssue(QStringLiteral("error"), QStringLiteral("The surface (%1 m) is beyond the sensor's range").arg(g.sensorDistance), segId);
                } else {
                    stats.gsd = g.gsd;
                    const double w = n(sp, "width");
                    const Pattern passes = corridorPasses(pts, std::max(1.0, w ? w : 60), g.sideDistance, lastNav);
                    stats.lanes = passes.count;
                    for (const LanePoint &q : passes.points) { NavPt np{q.lat, q.lng, alt, 16}; np.lane = q.lane; navPts.append(np); }
                    if (truthy(sp.value(QStringLiteral("trigger"))) && !g.hasSensor) navTrigger = g.triggerDistance;
                }
            }
        } else if (type == QLatin1String("landing")) {
            if (!pts.isEmpty()) {
                NavPt np{pts[0].lat, pts[0].lng, 0, truthy(sp.value(QStringLiteral("vtol"))) ? 85 : 21, n(sp, "abortAlt")};
                np.landing = true;
                navPts.append(np);
            }
        }
        if (navPts.isEmpty()) continue;

        setSpeed(speed, segId);

        const QJsonArray actions = seg.value(QStringLiteral("actions")).toArray();
        auto actionsAt = [&](int idx) {
            if (idx > 0 && str(P, "actionExec") != QLatin1String("every")) return;
            for (const QJsonValue &av : actions) {
                const QJsonObject a = av.toObject();
                const QString at = str(a, "type");
                if (at == QLatin1String("trigger_dist")) doItem(206, n(a, "distance"), 0, 1, 0, segId);
                else if (at == QLatin1String("shot")) doItem(203, 0, 0, 0, 0, segId);
                else if (at == QLatin1String("gimbal")) doItem(205, n(a, "pitch"), 0, n(a, "yaw"), 0, segId, 2);   // z = MAV_MOUNT_MODE_MAVLINK_TARGETING
                else if (at == QLatin1String("yaw")) doItem(115, n(a, "heading"), 0, 1, 0, segId);
                else if (at == QLatin1String("wait")) doItem(93, n(a, "seconds"), -1, -1, 0, segId);
                else if (at == QLatin1String("speed")) doItem(178, 1, n(a, "speed"), -1, 0, segId);
                else if (at == QLatin1String("servo")) doItem(183, n(a, "channel"), n(a, "pwm"), 0, 0, segId);
                else if (at == QLatin1String("relay")) doItem(181, n(a, "relay"), n(a, "state") != 0 ? 1 : 0, 0, 0, segId);
                else if (at == QLatin1String("raw")) doItem(int(n(a, "command")), n(a, "param1"), n(a, "param2"), n(a, "param3"), n(a, "param4"), segId);
                // A DO_ command runs once the previous waypoint is reached: that is where the camera is
                if (at == QLatin1String("gimbal")) { gimbalPitch = n(a, "pitch"); gimbalYaw = n(a, "yaw"); }
                if (at == QLatin1String("shot") && lastNavItem >= 0) {
                    const CompiledItem &nav = out[lastNavItem];
                    pendingShots.append({nav.segId, nav.segIdx, nav.lat, nav.lng, segId, gimbalPitch, gimbalYaw});
                }
            }
        };
        // Actions come AFTER the point they belong to: ArduPilot runs a DO_ command once
        // the preceding NAV item is reached, which is what "at the segment start" means.
        for (qsizetype i = 0; i < navPts.size(); ++i) {
            const NavPt &np = navPts[i];
            CompiledItem it;
            it.command = np.command;
            it.lat = np.lat; it.lng = np.lng; it.alt = np.alt;
            it.loc = true;
            it.segId = segId;
            it.segType = type;
            it.param1 = np.p1; it.param2 = np.p2; it.param3 = np.p3;
            it.loiter = np.loiter;
            it.landing = np.landing;
            it.segIdx = int(i);
            it.lane = np.lane;
            it.gimbalPitch = gimbalPitch;
            it.gimbalYaw = gimbalYaw;
            if (roiActive) it.roiTarget = roiTarget;
            it.trigger = navTrigger;
            const int idx = push(it);
            stats.waypoints++;
            stats.lengthM += haversine(lastNav, {np.lat, np.lng});
            if (np.loiter) stats.lengthM += 2 * M_PI * np.loiter->r * np.loiter->turns;
            lastNav = {np.lat, np.lng};
            lastNavItem = idx;
            if (i == 0 && navTrigger > 0) doItem(206, navTrigger, 0, 1, 0, segId);
            actionsAt(int(i));
        }
        if (navTrigger > 0) doItem(206, 0, 0, 0, 0, segId);
        if (speed > 0) stats.durationS = stats.lengthM / speed;
        for (const QJsonValue &av : actions)
            if (str(av.toObject(), "type") == QLatin1String("wait")) stats.durationS += n(av.toObject(), "seconds");
    }

    // ── End of route ──
    const bool endsWithLanding = !segments.isEmpty() && str(segments.last().toObject(), "type") == QLatin1String("landing");
    if (roiActive) doItem(197, 0, 0, 0, 0, {});
    const QString endAction = str(P, "endAction");
    const bool lastNavIsHome = std::abs(lastNav.lat - home->lat) < 1e-12 && std::abs(lastNav.lng - home->lng) < 1e-12 && lastNavItem < 0;
    if (!endsWithLanding && endAction == QLatin1String("rtl")) {
        const int i = doItem(20, 0, 0, 0, 0, {});
        out[i].kind = QStringLiteral("rtl");
    } else if (!endsWithLanding && endAction == QLatin1String("land") && !lastNavIsHome) {
        CompiledItem l;
        l.command = 21;
        l.lat = lastNav.lat; l.lng = lastNav.lng;
        l.loc = true;
        l.landing = true;
        push(l);
    }

    // ── A boat's first speed ──
    // ArduRover starts AUTO on a boat in loiter: a DO_CHANGE_SPEED before the first
    // waypoint goes to the loiter. After the first navigation item it runs as that leg starts.
    if (ctx.vehicleType == 11) {
        auto isNav = [](const CompiledItem &it) { return it.loc && !it.isHome && !it.roi; };
        qsizetype first = -1;
        for (qsizetype i = 0; i < out.size(); ++i) if (isNav(out[i])) { first = i; break; }
        QList<CompiledItem> early;
        if (first > 0) {
            QList<CompiledItem> kept;
            for (qsizetype i = 0; i < out.size(); ++i) {
                if (i < first && out[i].command == 178) { CompiledItem e = out[i]; e.firstLeg = true; early.append(e); }
                else kept.append(out[i]);
            }
            out = kept;
        }
        if (!early.isEmpty()) {
            qsizetype f2 = 0;
            for (qsizetype i = 0; i < out.size(); ++i) if (isNav(out[i])) { f2 = i; break; }
            for (qsizetype k = 0; k < early.size(); ++k) out.insert(f2 + 1 + k, early[k]);
        }
    }

    // ── Terrain-following waypoints (AGL mode, opt-in) ──
    QList<CompiledItem> items = out;
    const QString altMode = str(P, "altMode");
    if (altMode == QLatin1String("agl") && truthy(P.value(QStringLiteral("terrainWaypoints")))) {
        const double tol = n(P, "aglTolerance");
        items = subdivideForTerrain(out, tol ? tol : 10, terrain);
    }

    // ── Resolve altitudes ──
    int missingTerrain = 0;
    for (CompiledItem &it : items) {
        if (!it.loc) continue;
        const auto t = terrain(it.lat, it.lng);
        it.terrain = t;
        if (it.isHome) { it.altMsl = homeElev; it.altRel = 0; it.alt = 0; continue; }
        if (it.roi) { it.altMsl = homeElev + it.alt; it.altRel = it.alt; continue; }
        if (it.landing) { it.altMsl = t ? *t : homeElev; it.altRel = it.altMsl - homeElev; it.alt = 0; continue; }
        if (it.kind == QLatin1String("takeoff")) { it.altMsl = homeElev + it.alt; it.altRel = it.alt; continue; }
        if (!t && !it.derived) ++missingTerrain;
        const double a = it.alt;
        if (altMode == QLatin1String("amsl")) {
            it.altMsl = a;
            it.altRel = a - homeElev;
            it.alt = t ? a - *t : a - homeElev;
        } else if (altMode == QLatin1String("rel")) {
            it.altMsl = homeElev + a;
            it.altRel = a;
            it.alt = t ? it.altMsl - *t : a;
        } else {
            it.altMsl = (t ? *t : homeElev) + a;
            it.altRel = it.altMsl - homeElev;
            it.alt = a;
        }
    }
    if (missingTerrain)
        addIssue(QStringLiteral("warn"), QStringLiteral("No elevation data under %1 point%2 — take-off elevation assumed")
                                             .arg(missingTerrain).arg(missingTerrain > 1 ? QStringLiteral("s") : QString()));

    // Sequence numbers now: the pointers below must not see the list change again
    for (qsizetype i = 0; i < items.size(); ++i) { items[i].seq = int(i); items[i].frame = items[i].loc ? 0 : 3; }

    // ── Validation ──
    QList<const CompiledItem *> navLocated;
    for (const CompiledItem &it : items) if (it.loc && !it.roi) navLocated.append(&it);
    QList<const CompiledItem *> flying;
    for (const CompiledItem *it : navLocated) if (!it->isHome && !it->landing) flying.append(it);
    const double minClear = n(P, "minClearance");
    const double maxAgl = n(P, "maxAgl");
    QList<QString> below, low, high;   // ordered sets, as the JS Sets
    auto addOnce = [](QList<QString> &l, const QString &id) { if (!l.contains(id)) l.append(id); };
    if (!onSurface) {
        for (const CompiledItem *it : flying) {
            if (!it->terrain) continue;
            const double agl = it->altMsl - *it->terrain;
            if (agl <= 0) addOnce(below, it->segId);
            else if (agl < minClear) addOnce(low, it->segId);
            if (maxAgl > 0 && agl > maxAgl) addOnce(high, it->segId);
        }
    }
    // Legs: the straight line between two safe waypoints may clip a ridge
    std::optional<double> legMinAgl;
    for (qsizetype i = 1; i < (onSurface ? 0 : navLocated.size()); ++i) {
        const CompiledItem &a = *navLocated[i - 1], &b = *navLocated[i];
        if (a.landing || b.landing || b.isHome || a.isHome) continue;
        const auto hit = legClearance(a, b, terrain);
        if (!hit) continue;
        if (!legMinAgl || *hit < *legMinAgl) legMinAgl = hit;
        if (*hit <= 0) addOnce(below, b.segId);
        else if (*hit < minClear) addOnce(low, b.segId);
    }
    for (const QString &id : below) addIssue(QStringLiteral("error"), QStringLiteral("Path goes below the terrain"), id);
    for (const QString &id : low) if (!below.contains(id)) addIssue(QStringLiteral("warn"), QStringLiteral("Clearance under %1 m").arg(minClear), id);
    for (const QString &id : high) addIssue(QStringLiteral("warn"), QStringLiteral("Above the %1 m AGL ceiling").arg(maxAgl), id);

    if (!isInav && itemLimit > 0 && items.size() > itemLimit)
        addIssue(QStringLiteral("error"), QStringLiteral("%1 items — %2 stores at most %3").arg(items.size()).arg(plat.label).arg(itemLimit));
    if (!takeoff && !isPlane && !isInav && !onSurface && !segments.isEmpty())
        addIssue(QStringLiteral("warn"), QStringLiteral("No automatic take-off — the vehicle must already be airborne when AUTO starts"));
    if (!segments.isEmpty() && !flying.isEmpty()) {
        const double far = haversine(*home, flying[0]->ll());
        if (far > 5000) addIssue(QStringLiteral("warn"), QStringLiteral("First waypoint is %1 km from the take-off point").arg(far / 1000, 0, 'f', 1));
    }
    for (const QJsonValue &sv : segments) {
        const QJsonObject seg = sv.toObject();
        double spd = n(seg.value(QStringLiteral("params")).toObject(), "speed");
        if (!spd) spd = n(P, "defaultSpeed");
        if (spd > 40) addIssue(QStringLiteral("warn"), QStringLiteral("%1 m/s is fast for a multirotor").arg(spd), str(seg, "id"));
    }

    // ── Camera preview ──
    QList<Shot> shots;
    for (const PendingShot &ps : pendingShots) {
        const CompiledItem *target = nullptr;
        for (const CompiledItem *it : navLocated)
            if (!it->derived && it->segId == ps.navSegId && it->segIdx == ps.navSegIdx && it->lat == ps.lat && it->lng == ps.lng) { target = it; break; }
        if (target) shots.append({int(navLocated.indexOf(target)), ps.segId, ps.pitch, ps.yaw});
    }
    QHash<QString, int> photosBySeg;
    R.camera = buildCameraPreview(route, navLocated, shots, photosBySeg);
    for (auto it = photosBySeg.begin(); it != photosBySeg.end(); ++it)
        if (R.segStats.contains(it.key())) R.segStats[it.key()].photos = it.value();

    // ── Profile path, stats ──
    double dist = 0;
    const CompiledItem *prev = nullptr;
    for (const CompiledItem *it : navLocated) {
        if (prev) dist += haversine(prev->ll(), it->ll());
        NavPoint p;
        p.lat = it->lat; p.lng = it->lng; p.dist = dist; p.altMsl = it->altMsl;
        if (it->terrain) p.agl = it->altMsl - *it->terrain;
        p.terrain = it->terrain;
        p.segId = it->segId;
        p.seq = it->seq;
        p.derived = it->derived; p.isHome = it->isHome; p.landing = it->landing;
        p.loiter = it->loiter;
        R.navPath.append(p);
        prev = it;
    }
    R.stats = buildStats(P, items, navLocated, R.segStats, homeElev, *home);
    // The lowest point of the flight can be in the middle of a straight leg
    if (legMinAgl && (!R.stats.minAgl || *legMinAgl < *R.stats.minAgl)) R.stats.minAgl = legMinAgl;

    // ── INAV encoding ──
    if (isInav) {
        R.inav = toInavMission(items, truthy(P.value(QStringLiteral("inavAltAmsl"))), itemLimit);
        R.issues.append(R.inav->issues);
    }
    R.items = items;
    R.home = Home{home->lat, home->lng, homeElev};
    return R;
}

// ── INAV ─────────────────────────────────────────────────────────────────────

InavMission toInavMission(const QList<CompiledItem> &items, bool amsl, int limit)
{
    enum { WAYPOINT = 1, HOLD_TIME = 3, RTH = 4, SET_POI = 5, JUMP = 6, SET_HEAD = 7, LAND = 8 };
    if (limit <= 0) limit = 60;
    InavMission m;
    auto add = [&](const QString &level, const QString &text, const QString &segId = {}) { m.issues.append({level, text, segId}); };
    auto clampI16 = [](double v) { return int(std::clamp<double>(std::lround(v), -32768, 32767)); };
    auto altCm = [&](const CompiledItem &it) { return qint32(std::lround((amsl ? it.altMsl : it.altRel) * 100)); };
    const int p3Alt = amsl ? 1 : 0;
    auto push = [&](int action, const CompiledItem *it, double p1, double p2, double p3) {
        InavWp w;
        w.action = action;
        w.lat = it ? qint32(std::lround(it->lat * 1e7)) : 0;
        w.lon = it ? qint32(std::lround(it->lng * 1e7)) : 0;
        w.alt = it ? altCm(*it) : 0;
        w.p1 = clampI16(p1); w.p2 = clampI16(p2); w.p3 = clampI16(p3);
        if (it) w.segId = it->segId;
        m.wps.append(w);
    };
    int speedCms = 0;
    double holdS = 0;
    auto holdAt = [&](double seconds) {
        if (!m.wps.isEmpty() && m.wps.last().action == WAYPOINT) {
            InavWp &prev = m.wps.last();
            prev.action = HOLD_TIME;
            prev.p2 = prev.p1;
            prev.p1 = clampI16(seconds);
            return;
        }
        if (!m.wps.isEmpty() && m.wps.last().action == HOLD_TIME) {
            m.wps.last().p1 = clampI16(m.wps.last().p1 + std::lround(seconds));
            return;
        }
        holdS = std::max(holdS, seconds);
    };
    bool splineSeen = false, takeoffSeen = false;
    QList<QString> loiterSegs;
    QList<std::pair<int, int>> dropped;   // command → count, in first-seen order
    for (const CompiledItem &it : items) {
        if (it.isHome) continue;
        switch (it.command) {
        case 16: case 82: {
            if (it.command == 82) splineSeen = true;
            const long hold = std::lround(std::max(holdS, it.param1));
            holdS = 0;
            if (hold > 0) push(HOLD_TIME, &it, double(hold), speedCms, p3Alt);
            else push(WAYPOINT, &it, speedCms, 0, p3Alt);
            break;
        }
        case 22: case 84: takeoffSeen = true; break;
        case 17: case 18: case 19: case 31: if (!loiterSegs.contains(it.segId)) loiterSegs.append(it.segId); break;
        case 20: push(RTH, nullptr, 0, 0, 0); break;
        case 21: case 85: push(LAND, &it, 0, 0, p3Alt); break;
        case 201: case 195: push(SET_POI, &it, 0, 0, 0); break;
        case 197: break;
        case 115: push(SET_HEAD, nullptr, double(std::lround(std::fmod(std::fmod(it.param1, 360) + 360, 360))), 0, 0); break;
        case 178: speedCms = std::max(0, int(std::lround(it.param2 * 100))); break;
        case 93: case 112: holdAt(std::max(0.0, it.param1)); break;
        case 177: push(JUMP, nullptr, double(std::lround(it.param1)), double(std::lround(it.param2)), 0); break;
        default: {
            auto f = std::find_if(dropped.begin(), dropped.end(), [&](const auto &p) { return p.first == it.command; });
            if (f != dropped.end()) ++f->second; else dropped.append({it.command, 1});
        }
        }
    }
    for (qsizetype i = 0; i < m.wps.size(); ++i) { m.wps[i].index = int(i) + 1; m.wps[i].flag = 0; }
    if (!m.wps.isEmpty()) m.wps.last().flag = 0xA5;

    if (takeoffSeen) add(QStringLiteral("warn"), QStringLiteral("INAV has no take-off waypoint — automatic take-off is not uploaded"));
    if (splineSeen) add(QStringLiteral("warn"), QStringLiteral("INAV flies straight legs — spline waypoints upload as ordinary waypoints"));
    for (const QString &segId : loiterSegs) add(QStringLiteral("warn"), QStringLiteral("A circle cannot be uploaded to INAV — the segment is skipped"), segId);
    for (const auto &[cmd, cnt] : dropped) add(QStringLiteral("warn"), QStringLiteral("%1 × command %2 has no INAV equivalent — not uploaded").arg(cnt).arg(cmd));
    if (m.wps.isEmpty()) {
        add(QStringLiteral("error"), QStringLiteral("Nothing in this route can be uploaded to INAV"));
    } else {
        if (m.wps.size() > limit) add(QStringLiteral("error"), QStringLiteral("%1 waypoints — INAV stores at most %2").arg(m.wps.size()).arg(limit));
        const int last = m.wps.last().action;
        if (last != WAYPOINT && last != HOLD_TIME && last != RTH && last != LAND) {
            static const QHash<int, QString> names{{1, QStringLiteral("WAYPOINT")}, {3, QStringLiteral("HOLD")}, {4, QStringLiteral("RTH")},
                                                   {5, QStringLiteral("POI")}, {6, QStringLiteral("JUMP")}, {7, QStringLiteral("HEADING")}, {8, QStringLiteral("LAND")}};
            add(QStringLiteral("warn"), QStringLiteral("The mission ends on %1 — INAV expects a navigation waypoint last")
                                            .arg(names.value(last, QStringLiteral("ACTION %1").arg(last))));
        }
    }
    return m;
}

} // namespace corv::mission
