#include "mission/RouteCompiler.h"

#include "mission/Platforms.h"
#include "mission/RouteGeo.h"

#include <QJsonArray>
#include <QSet>
#include <algorithm>
#include <cmath>
#include <limits>

using namespace Qt::StringLiterals;

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

/// Of the four ways to walk a pattern (build(reversed order, other side first)), the one
/// starting nearest to `near`; without it, the first.
template <typename Build>
auto nearestStart(const Build &build, const std::optional<XY> &near)
{
    decltype(build(false, false)) best;
    double bestD = std::numeric_limits<double>::infinity();
    for (bool a : {false, true}) {
        for (bool b : {false, true}) {
            auto pts = build(a, b);
            if (!near) return pts;
            const double d = std::hypot(pts[0].q.x - near->x, pts[0].q.y - near->y);
            if (d < bestD) { bestD = d; best = pts; }
        }
    }
    return best;
}

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
                const QString laneId = u"%1:%2"_s.arg(lane.y, 0, 'f', 2).arg(x0, 0, 'f', 1);
                if (rightward) { pts.append({{x0, lane.y}, laneId}); pts.append({{x1, lane.y}, laneId}); }
                else { pts.append({{x1, lane.y}, laneId}); pts.append({{x0, lane.y}, laneId}); }
            }
        }
        return pts;
    };
    Pattern out;
    out.count = int(lanes.size());
    for (const P &p : nearestStart(build, startNear ? std::optional<XY>(rot.fwd(f.toXY(*startNear))) : std::nullopt)) {
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
            for (const XY &q : seq) pts.append({q, u"p%1"_s.arg(passIdx)});
        }
        return pts;
    };
    Pattern out;
    out.count = cnt;
    for (const Q &q : nearestStart(build, startNear ? std::optional<XY>(f.toXY(*startNear)) : std::nullopt)) {
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
    const QJsonObject cam = route.value(u"params"_s).toObject().value(u"camera"_s).toObject();
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
    for (const CompiledItem &it : items) if (it.kind == "takeoff"_L1) { const double ta = n(P, "takeoffAlt"); dur += (ta ? ta : 30) / kClimbRate; break; }
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
    if (!items.isEmpty() && items.last().kind == "rtl"_L1 && prev) {
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

namespace {

/// A navigation point a segment expands into, before it becomes a mission item.
struct NavPt {
    double lat, lng, alt;
    int command;
    double p1 = 0, p2 = 0, p3 = 0;
    std::optional<Loiter> loiter;
    bool landing = false;
    QString lane;
};

/// The nav item a photo is taken at, by identity: the boat reorder and the terrain
/// subdivision move items around.
struct PendingShot { QString navSegId; int navSegIdx; double lat, lng; QString segId; double pitch, yaw; };

double nonZero(double v, double fallback) { return v ? v : fallback; }

/// One run of compileRoute: the items as the segments emit them, then resolved and checked.
struct Compilation {
    const CompileContext &ctx;
    const QJsonObject route, P;
    const QJsonArray segments;
    const Platform &plat;
    const bool isInav, isPlane, onSurface;
    const int itemLimit;
    const TerrainFn terrain;

    CompiledRoute R;
    LL home;
    double homeElev = 0;
    bool takeoff = false;
    QList<CompiledItem> out;
    double currentSpeed = 0;
    LL lastNav;
    int lastNavItem = -1;
    bool roiActive = false;
    std::optional<RoiTarget> roiTarget;
    double gimbalPitch = -90, gimbalYaw = 0;
    QList<PendingShot> pendingShots;

    Compilation(const QJsonObject &r, const CompileContext &c)
        : ctx(c), route(r), P(r.value(u"params"_s).toObject()), segments(r.value(u"segments"_s).toArray()), plat(platform(c.platform)),
          isInav(plat.id == "inav"_L1), isPlane(c.vehicleType == 1),
          // A rover, a boat or a sub has nothing to take off from
          onSurface(c.vehicleType == 10 || c.vehicleType == 11 || c.vehicleType == 12),
          itemLimit(c.itemLimit > 0 ? c.itemLimit : plat.itemLimit),
          terrain(c.terrain ? c.terrain : TerrainFn([](double, double) { return std::optional<double>(); }))
    {
        const double gp = num(P.value(u"gimbalPitch"_s), NAN);
        gimbalPitch = std::isfinite(gp) ? gp : -90;
        gimbalYaw = n(P, "gimbalYaw");
    }

    void issue(const QString &level, const QString &text, const QString &segId = {}) { R.issues.append({level, text, segId}); }
    int push(const CompiledItem &it) { out.append(it); return int(out.size()) - 1; }
    int doItem(int command, double p1, double p2, double p3, double p4, const QString &segId, double alt = 0)
    {
        CompiledItem it;
        it.command = command;
        it.segId = segId;
        it.param1 = p1; it.param2 = p2; it.param3 = p3; it.param4 = p4;
        it.alt = alt;
        return push(it);
    }
    static CompiledItem located(int command, const LL &at, double alt)
    {
        CompiledItem it;
        it.command = command;
        it.lat = at.lat; it.lng = at.lng; it.alt = alt;
        it.loc = true;
        return it;
    }
    void setSpeed(double v, const QString &segId)
    {
        if (v <= 0 || std::abs(v - currentSpeed) <= 0.01) return;
        doItem(178, 1, v, -1, 0, segId);
        currentSpeed = v;
    }

    CompiledRoute run();
    std::optional<LL> takeoffPoint() const;
    void begin();
    void addSegment(const QJsonObject &seg);
    void addPoi(const QJsonObject &sp, const QList<LL> &pts, const QString &segId);
    QList<NavPt> navPoints(const QJsonObject &seg, const QString &type, SegStats &stats, double &trigger);
    Pattern survey(const QJsonObject &seg, bool area, double alt, SegStats &stats, double &trigger);
    void addNavItem(const NavPt &np, const QString &type, const QString &segId, int idx, double trigger, SegStats &stats);
    void addActions(const QJsonArray &actions, const QString &segId);
    void finish();
    void boatFirstSpeed();
    void resolveAltitudes(QList<CompiledItem> &items);
    std::optional<double> validate(const QList<const CompiledItem *> &navLocated, qsizetype itemCount);
    QList<Shot> shotsAt(const QList<const CompiledItem *> &navLocated) const;
};

CompiledRoute Compilation::run()
{
    R.platform = plat.id;
    const std::optional<LL> start = takeoffPoint();
    if (!start || segments.isEmpty()) {
        if (segments.isEmpty()) issue(u"info"_s, u"Pick a tool on the map and start drawing"_s);
        if (start) {
            const auto t = terrain(start->lat, start->lng);
            R.home = Home{start->lat, start->lng, t ? *t : ctx.homeAlt};
        }
        return R;
    }
    home = lastNav = *start;
    const auto homeTerrain = terrain(home.lat, home.lng);
    homeElev = homeTerrain ? *homeTerrain : ctx.homeAlt;
    if (!homeTerrain) issue(u"warn"_s, u"No elevation data at the take-off point — vehicle home altitude assumed"_s);

    begin();
    for (const QJsonValue &sv : segments) addSegment(sv.toObject());
    finish();
    if (ctx.vehicleType == 11) boatFirstSpeed();

    // Terrain-following waypoints (AGL mode, opt-in)
    QList<CompiledItem> items = out;
    if (str(P, "altMode") == "agl"_L1 && truthy(P.value(u"terrainWaypoints"_s)))
        items = subdivideForTerrain(out, nonZero(n(P, "aglTolerance"), 10), terrain);
    resolveAltitudes(items);
    // Sequence numbers now: the pointers below must not see the list change again
    for (qsizetype i = 0; i < items.size(); ++i) { items[i].seq = int(i); items[i].frame = items[i].loc ? 0 : 3; }

    QList<const CompiledItem *> navLocated;
    for (const CompiledItem &it : items) if (it.loc && !it.roi) navLocated.append(&it);
    const std::optional<double> legMinAgl = validate(navLocated, items.size());

    QHash<QString, int> photosBySeg;
    R.camera = buildCameraPreview(route, navLocated, shotsAt(navLocated), photosBySeg);
    for (auto it = photosBySeg.begin(); it != photosBySeg.end(); ++it)
        if (R.segStats.contains(it.key())) R.segStats[it.key()].photos = it.value();

    // Profile path, stats
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
    R.stats = buildStats(P, items, navLocated, R.segStats, homeElev, home);
    // The lowest point of the flight can be in the middle of a straight leg
    if (legMinAgl && (!R.stats.minAgl || *legMinAgl < *R.stats.minAgl)) R.stats.minAgl = legMinAgl;

    if (isInav) {
        R.inav = toInavMission(items, truthy(P.value(u"inavAltAmsl"_s)), itemLimit);
        R.issues.append(R.inav->issues);
    }
    R.items = items;
    R.home = Home{home.lat, home.lng, homeElev};
    return R;
}

std::optional<LL> Compilation::takeoffPoint() const
{
    const QJsonObject ph = P.value(u"home"_s).toObject();
    if (!ph.isEmpty()) return LL{n(ph, "lat"), n(ph, "lng")};
    if (ctx.home) return ctx.home;
    for (const QJsonValue &v : segments)
        if (const QList<LL> pts = pointsOf(v.toObject()); !pts.isEmpty()) return pts.first();
    return std::nullopt;
}

void Compilation::begin()
{
    CompiledItem h = located(16, home, 0);
    h.isHome = true;
    push(h);
    // INAV has no take-off waypoint at all; the operator is told so
    takeoff = truthy(P.value(u"takeoff"_s));
    if (takeoff && isInav) issue(u"warn"_s, u"INAV has no take-off waypoint — arm and take off in a flight mode, then switch to WP"_s);
    if (takeoff && !isInav && !onSurface) {
        CompiledItem t = located(22, home, nonZero(n(P, "takeoffAlt"), 30));
        t.param1 = isPlane ? 15 : 0;
        t.kind = u"takeoff"_s;
        push(t);
    }
    setSpeed(n(P, "defaultSpeed"), {});
}

void Compilation::addSegment(const QJsonObject &seg)
{
    const QString type = str(seg, "type");
    if (!segmentTypes().contains(type)) return;
    const QString segId = str(seg, "id");
    const QJsonObject sp = seg.value(u"params"_s).toObject();
    SegStats &stats = R.segStats[segId];
    stats = SegStats{};
    if (type == "poi"_L1) return addPoi(sp, pointsOf(seg), segId);

    double trigger = 0;
    const QList<NavPt> navPts = navPoints(seg, type, stats, trigger);
    if (navPts.isEmpty()) return;
    const double speed = n(sp, "speed") > 0 ? n(sp, "speed") : n(P, "defaultSpeed");
    setSpeed(speed, segId);
    // Actions come AFTER the point they belong to: ArduPilot runs a DO_ command once
    // the preceding NAV item is reached, which is what "at the segment start" means.
    const QJsonArray actions = seg.value(u"actions"_s).toArray();
    for (qsizetype i = 0; i < navPts.size(); ++i) {
        addNavItem(navPts[i], type, segId, int(i), trigger, stats);
        if (i == 0 && trigger > 0) doItem(206, trigger, 0, 1, 0, segId);
        if (i == 0 || str(P, "actionExec") == "every"_L1) addActions(actions, segId);
    }
    if (trigger > 0) doItem(206, 0, 0, 0, 0, segId);
    if (speed > 0) stats.durationS = stats.lengthM / speed;
    for (const QJsonValue &av : actions)
        if (str(av.toObject(), "type") == "wait"_L1) stats.durationS += n(av.toObject(), "seconds");
}

void Compilation::addPoi(const QJsonObject &sp, const QList<LL> &pts, const QString &segId)
{
    if (str(sp, "mode") == "clear"_L1) {
        doItem(197, 0, 0, 0, 0, segId);
        roiActive = false;
        roiTarget.reset();
        return;
    }
    if (pts.isEmpty()) return;
    CompiledItem it = located(201, pts[0], n(sp, "alt"));
    it.roi = true;
    it.segId = segId;
    push(it);
    roiActive = true;
    roiTarget = RoiTarget{pts[0].lat, pts[0].lng, n(sp, "alt"), segId};
}

QList<NavPt> Compilation::navPoints(const QJsonObject &seg, const QString &type, SegStats &stats, double &trigger)
{
    const QString segId = str(seg, "id");
    const QJsonObject sp = seg.value(u"params"_s).toObject();
    const QList<LL> pts = pointsOf(seg);
    const double alt = segmentAlt(seg, P);
    const QString turn = str(sp, "turn");
    const int cmdWp = ((turn == "route"_L1 || turn.isEmpty() ? str(P, "turnType") : turn) == "spline"_L1) ? 82 : 16;
    QList<NavPt> navPts;
    if (type == "waypoint"_L1) {
        if (!pts.isEmpty()) navPts.append({pts[0].lat, pts[0].lng, alt, cmdWp, n(sp, "delay"), n(sp, "acceptRadius")});
    } else if (type == "circle"_L1) {
        if (!pts.isEmpty()) {
            const double r = std::max(5.0, nonZero(n(sp, "radius"), 50));
            const int turns = std::max(1, int(std::lround(nonZero(n(sp, "turns"), 1))));
            NavPt np{pts[0].lat, pts[0].lng, alt, 18, double(turns), 0, str(sp, "direction") == "ccw"_L1 ? -r : r};
            np.loiter = Loiter{r, turns};
            navPts.append(np);
        }
    } else if (type == "perimeter"_L1) {
        if (pts.size() < 3) {
            issue(u"error"_s, u"Perimeter needs at least 3 vertices"_s, segId);
        } else {
            QList<LL> ring = pts;
            if (str(sp, "direction") == "ccw"_L1) std::reverse(ring.begin() + 1, ring.end());   // same start, walked the other way
            if (truthy(sp.value(u"closed"_s))) ring.append(ring[0]);
            for (const LL &p : ring) navPts.append({p.lat, p.lng, alt, cmdWp});
        }
    } else if (type == "area"_L1 || type == "corridor"_L1) {
        for (const LanePoint &q : survey(seg, type == "area"_L1, alt, stats, trigger).points) {
            NavPt np{q.lat, q.lng, alt, 16};
            np.lane = q.lane;
            navPts.append(np);
        }
    } else if (type == "landing"_L1) {
        if (!pts.isEmpty()) {
            NavPt np{pts[0].lat, pts[0].lng, 0, truthy(sp.value(u"vtol"_s)) ? 85 : 21, n(sp, "abortAlt")};
            np.landing = true;
            navPts.append(np);
        }
    }
    return navPts;
}

Pattern Compilation::survey(const QJsonObject &seg, bool area, double alt, SegStats &stats, double &trigger)
{
    // The lanes of an area scan or the passes of a corridor; none, with the error raised, when they cannot be laid out
    const QString segId = str(seg, "id");
    const QJsonObject sp = seg.value(u"params"_s).toObject();
    const QList<LL> pts = pointsOf(seg);
    if (pts.size() < (area ? 3 : 2)) {
        issue(u"error"_s, area ? u"Area scan needs at least 3 vertices"_s : u"Corridor needs at least 2 points"_s, segId);
        return {};
    }
    const SurveyGeometry g = surveyGeometry(seg, alt, P.value(u"camera"_s).toObject());
    if (g.hasSensor && !g.sensor.swath) {
        issue(u"error"_s, u"The surface (%1 m) is beyond the sensor's range"_s.arg(g.sensorDistance), segId);
        return {};
    }
    stats.gsd = g.gsd;
    Pattern pat;
    if (area) {
        stats.area = polygonArea(pts);
        if (g.hasSensor) stats.swath = g.sensor.swath;
        const double angle = n(sp, "angle");
        pat = lawnmower(pts, angle, g.sideDistance, n(sp, "overshoot"), lastNav);
        stats.lanes = pat.count;
        if (truthy(sp.value(u"doubleGrid"_s))) {
            const LL from = pat.points.isEmpty() ? lastNav : LL{pat.points.last().lat, pat.points.last().lng};
            Pattern cross = lawnmower(pts, angle + 90, g.sideDistance, n(sp, "overshoot"), from);
            for (LanePoint &q : cross.points) q.lane = u"x"_s + q.lane;
            pat.points.append(cross.points);
            stats.lanes += cross.count;
        }
        if (pat.points.isEmpty()) {
            issue(u"error"_s, u"Area too small for the lane spacing — reduce side distance or overlap"_s, segId);
            return {};
        }
    } else {
        pat = corridorPasses(pts, std::max(1.0, nonZero(n(sp, "width"), 60)), g.sideDistance, lastNav);
        stats.lanes = pat.count;
    }
    if (truthy(sp.value(u"trigger"_s)) && !g.hasSensor) trigger = g.triggerDistance;
    return pat;
}

void Compilation::addNavItem(const NavPt &np, const QString &type, const QString &segId, int idx, double trigger, SegStats &stats)
{
    CompiledItem it = located(np.command, {np.lat, np.lng}, np.alt);
    it.segId = segId;
    it.segType = type;
    it.param1 = np.p1; it.param2 = np.p2; it.param3 = np.p3;
    it.loiter = np.loiter;
    it.landing = np.landing;
    it.segIdx = idx;
    it.lane = np.lane;
    it.gimbalPitch = gimbalPitch;
    it.gimbalYaw = gimbalYaw;
    if (roiActive) it.roiTarget = roiTarget;
    it.trigger = trigger;
    lastNavItem = push(it);
    stats.waypoints++;
    stats.lengthM += haversine(lastNav, {np.lat, np.lng});
    if (np.loiter) stats.lengthM += 2 * M_PI * np.loiter->r * np.loiter->turns;
    lastNav = {np.lat, np.lng};
}

void Compilation::addActions(const QJsonArray &actions, const QString &segId)
{
    for (const QJsonValue &av : actions) {
        const QJsonObject a = av.toObject();
        const QString at = str(a, "type");
        if (at == "trigger_dist"_L1) doItem(206, n(a, "distance"), 0, 1, 0, segId);
        else if (at == "shot"_L1) doItem(203, 0, 0, 0, 0, segId);
        else if (at == "gimbal"_L1) doItem(205, n(a, "pitch"), 0, n(a, "yaw"), 0, segId, 2);   // z = MAV_MOUNT_MODE_MAVLINK_TARGETING
        else if (at == "yaw"_L1) doItem(115, n(a, "heading"), 0, 1, 0, segId);
        else if (at == "wait"_L1) doItem(93, n(a, "seconds"), -1, -1, 0, segId);
        else if (at == "speed"_L1) doItem(178, 1, n(a, "speed"), -1, 0, segId);
        else if (at == "servo"_L1) doItem(183, n(a, "channel"), n(a, "pwm"), 0, 0, segId);
        else if (at == "relay"_L1) doItem(181, n(a, "relay"), n(a, "state") != 0 ? 1 : 0, 0, 0, segId);
        else if (at == "raw"_L1) doItem(int(n(a, "command")), n(a, "param1"), n(a, "param2"), n(a, "param3"), n(a, "param4"), segId);
        // A DO_ command runs once the previous waypoint is reached: that is where the camera is
        if (at == "gimbal"_L1) { gimbalPitch = n(a, "pitch"); gimbalYaw = n(a, "yaw"); }
        if (at == "shot"_L1 && lastNavItem >= 0) {
            const CompiledItem &nav = out[lastNavItem];
            pendingShots.append({nav.segId, nav.segIdx, nav.lat, nav.lng, segId, gimbalPitch, gimbalYaw});
        }
    }
}

void Compilation::finish()
{
    if (roiActive) doItem(197, 0, 0, 0, 0, {});
    if (str(segments.last().toObject(), "type") == "landing"_L1) return;
    const QString endAction = str(P, "endAction");
    if (endAction == "rtl"_L1) {
        out[doItem(20, 0, 0, 0, 0, {})].kind = u"rtl"_s;
    } else if (endAction == "land"_L1 && lastNavItem >= 0) {
        CompiledItem l = located(21, lastNav, 0);
        l.landing = true;
        push(l);
    }
}

void Compilation::boatFirstSpeed()
{
    // ArduRover starts AUTO on a boat in loiter: a DO_CHANGE_SPEED before the first
    // waypoint goes to the loiter. After the first navigation item it runs as that leg starts.
    auto isNav = [](const CompiledItem &it) { return it.loc && !it.isHome && !it.roi; };
    const qsizetype first = std::find_if(out.cbegin(), out.cend(), isNav) - out.cbegin();
    QList<CompiledItem> early, kept;
    for (qsizetype i = 0; i < out.size(); ++i) {
        if (i < first && out[i].command == 178) {
            early.append(out[i]);
            early.last().firstLeg = true;
        } else {
            kept.append(out[i]);
        }
    }
    if (early.isEmpty()) return;
    const qsizetype after = first - early.size() + 1;   // right after the first navigation item
    for (qsizetype k = 0; k < early.size(); ++k) kept.insert(after + k, early[k]);
    out = kept;
}

void Compilation::resolveAltitudes(QList<CompiledItem> &items)
{
    const QString altMode = str(P, "altMode");
    int missingTerrain = 0;
    for (CompiledItem &it : items) {
        if (!it.loc) continue;
        const auto t = terrain(it.lat, it.lng);
        it.terrain = t;
        if (it.isHome) { it.altMsl = homeElev; it.altRel = 0; it.alt = 0; continue; }
        if (it.roi || it.kind == "takeoff"_L1) { it.altMsl = homeElev + it.alt; it.altRel = it.alt; continue; }
        if (it.landing) { it.altMsl = t ? *t : homeElev; it.altRel = it.altMsl - homeElev; it.alt = 0; continue; }
        if (!t && !it.derived) ++missingTerrain;
        const double a = it.alt;
        if (altMode == "amsl"_L1) {
            it.altMsl = a;
            it.altRel = a - homeElev;
            it.alt = t ? a - *t : a - homeElev;
        } else if (altMode == "rel"_L1) {
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
        issue(u"warn"_s, u"No elevation data under %1 point%2 — take-off elevation assumed"_s.arg(missingTerrain).arg(missingTerrain > 1 ? u"s"_s : QString()));
}

std::optional<double> Compilation::validate(const QList<const CompiledItem *> &navLocated, qsizetype itemCount)
{
    // Returns the lowest clearance along the legs, which can sit between two waypoints
    QList<const CompiledItem *> flying;
    for (const CompiledItem *it : navLocated) if (!it->isHome && !it->landing) flying.append(it);
    const double minClear = n(P, "minClearance");
    const double maxAgl = n(P, "maxAgl");
    QList<QString> below, low, high;   // ordered sets, as the JS Sets
    auto addOnce = [](QList<QString> &l, const QString &id) { if (!l.contains(id)) l.append(id); };
    std::optional<double> legMinAgl;
    if (!onSurface) {
        for (const CompiledItem *it : flying) {
            if (!it->terrain) continue;
            const double agl = it->altMsl - *it->terrain;
            if (agl <= 0) addOnce(below, it->segId);
            else if (agl < minClear) addOnce(low, it->segId);
            if (maxAgl > 0 && agl > maxAgl) addOnce(high, it->segId);
        }
        // Legs: the straight line between two safe waypoints may clip a ridge
        for (qsizetype i = 1; i < navLocated.size(); ++i) {
            const CompiledItem &a = *navLocated[i - 1], &b = *navLocated[i];
            if (a.landing || b.landing || b.isHome || a.isHome) continue;
            const auto hit = legClearance(a, b, terrain);
            if (!hit) continue;
            if (!legMinAgl || *hit < *legMinAgl) legMinAgl = hit;
            if (*hit <= 0) addOnce(below, b.segId);
            else if (*hit < minClear) addOnce(low, b.segId);
        }
    }
    for (const QString &id : below) issue(u"error"_s, u"Path goes below the terrain"_s, id);
    for (const QString &id : low) if (!below.contains(id)) issue(u"warn"_s, u"Clearance under %1 m"_s.arg(minClear), id);
    for (const QString &id : high) issue(u"warn"_s, u"Above the %1 m AGL ceiling"_s.arg(maxAgl), id);

    if (!isInav && itemLimit > 0 && itemCount > itemLimit)
        issue(u"error"_s, u"%1 items — %2 stores at most %3"_s.arg(itemCount).arg(plat.label).arg(itemLimit));
    if (!takeoff && !isPlane && !isInav && !onSurface)
        issue(u"warn"_s, u"No automatic take-off — the vehicle must already be airborne when AUTO starts"_s);
    if (!flying.isEmpty()) {
        const double far = haversine(home, flying[0]->ll());
        if (far > 5000) issue(u"warn"_s, u"First waypoint is %1 km from the take-off point"_s.arg(far / 1000, 0, 'f', 1));
    }
    for (const QJsonValue &sv : segments) {
        const QJsonObject seg = sv.toObject();
        const double spd = nonZero(n(seg.value(u"params"_s).toObject(), "speed"), n(P, "defaultSpeed"));
        if (spd > 40) issue(u"warn"_s, u"%1 m/s is fast for a multirotor"_s.arg(spd), str(seg, "id"));
    }
    return legMinAgl;
}

QList<Shot> Compilation::shotsAt(const QList<const CompiledItem *> &navLocated) const
{
    QList<Shot> shots;
    for (const PendingShot &ps : pendingShots) {
        for (qsizetype i = 0; i < navLocated.size(); ++i) {
            const CompiledItem *it = navLocated[i];
            if (it->derived || it->segId != ps.navSegId || it->segIdx != ps.navSegIdx || it->lat != ps.lat || it->lng != ps.lng) continue;
            shots.append({int(i), ps.segId, ps.pitch, ps.yaw});
            break;
        }
    }
    return shots;
}

} // namespace

CompiledRoute compileRoute(const QJsonObject &route, const CompileContext &ctx)
{
    return Compilation(route, ctx).run();
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

    if (takeoffSeen) add(u"warn"_s, u"INAV has no take-off waypoint — automatic take-off is not uploaded"_s);
    if (splineSeen) add(u"warn"_s, u"INAV flies straight legs — spline waypoints upload as ordinary waypoints"_s);
    for (const QString &segId : loiterSegs) add(u"warn"_s, u"A circle cannot be uploaded to INAV — the segment is skipped"_s, segId);
    for (const auto &[cmd, cnt] : dropped) add(u"warn"_s, u"%1 × command %2 has no INAV equivalent — not uploaded"_s.arg(cnt).arg(cmd));
    if (m.wps.isEmpty()) {
        add(u"error"_s, u"Nothing in this route can be uploaded to INAV"_s);
    } else {
        if (m.wps.size() > limit) add(u"error"_s, u"%1 waypoints — INAV stores at most %2"_s.arg(m.wps.size()).arg(limit));
        const int last = m.wps.last().action;
        if (last != WAYPOINT && last != HOLD_TIME && last != RTH && last != LAND) {
            static const QHash<int, QString> names{{1, u"WAYPOINT"_s}, {3, u"HOLD"_s}, {4, u"RTH"_s},
                                                   {5, u"POI"_s}, {6, u"JUMP"_s}, {7, u"HEADING"_s}, {8, u"LAND"_s}};
            add(u"warn"_s, u"The mission ends on %1 — INAV expects a navigation waypoint last"_s
                                            .arg(names.value(last, u"ACTION %1"_s.arg(last))));
        }
    }
    return m;
}

} // namespace corv::mission
