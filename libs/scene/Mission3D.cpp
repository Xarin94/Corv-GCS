#include "scene/Mission3D.h"

#include <QHash>
#include <algorithm>
#include <cmath>
#include <numbers>

using namespace Qt::StringLiterals;

namespace corv::mission3d {

namespace {
constexpr int kRingSegments = 72;
constexpr int kRingArrows = 4;
constexpr double kTau = 2 * std::numbers::pi;

float across(const QVector3D &a, const QVector3D &b) { return std::hypot(a.x() - b.x(), a.z() - b.z()); }

Shape shapeOf(const QString &kind)
{
    if (kind == "takeoff"_L1) return Shape::Triangle;
    if (kind == "land"_L1) return Shape::Diamond;
    if (kind == "loiter"_L1) return Shape::Circle;
    return Shape::Square;
}

/// The ring of a loiter from its entry point, the point of the ring nearest to
/// `from` (north of the centre when there is none), with arrows along the turn.
struct Ring {
    QVector3D entry;
    std::vector<std::pair<QVector3D, QVector3D>> pairs;
};
Ring loiterRing(const Point &p, const QVector3D *from)
{
    const double r = p.loiterRadius;
    // Angles are bearings from the centre
    double a0 = 0;
    if (from) {
        const double east = from->x() - p.pos.x(), north = p.pos.z() - from->z();
        if (std::hypot(east, north) > 0.5) a0 = std::atan2(east, north);
    }
    auto at = [&](double a) { return p.pos + QVector3D(float(r * std::sin(a)), 0, float(-r * std::cos(a))); };
    const double sign = p.ccw ? -1 : 1;
    Ring out;
    out.entry = at(a0);
    QVector3D prev = out.entry;
    for (int k = 1; k <= kRingSegments; ++k) {
        const QVector3D q = at(a0 + sign * k * kTau / kRingSegments);
        out.pairs.emplace_back(prev, q);
        prev = q;
    }
    // At the bearings of the flight plan page's arrows, sized with the ring
    const double len = std::clamp(r * 0.28, 4.0, 120.0);
    for (int k = 0; k < kRingArrows; ++k) {
        const double a = (k + 0.5) * kTau / kRingArrows;
        const QVector3D tip = at(a);
        const double te = sign * std::cos(a), tn = -sign * std::sin(a);   // the way it turns (east, north)
        const double re = std::sin(a), rn = std::cos(a);                  // outward
        for (const int side : {1, -1}) {
            const double e = -te * len + re * side * len * 0.6, n = -tn * len + rn * side * len * 0.6;
            out.pairs.emplace_back(tip + QVector3D(float(e), 0, float(-n)), tip);
        }
    }
    return out;
}
} // namespace

bool isPattern(const QString &t) { return t == "area"_L1 || t == "corridor"_L1 || t == "perimeter"_L1; }

Drawing build(const std::vector<Point> &pts, const std::vector<Poi> &pois, std::optional<int> rtlSeq)
{
    Drawing out;
    // A pattern runs from the first to the last base point of its segment; the
    // terrain-following points before the first belong to the transit leg
    struct Span {
        std::size_t first = 0, last = 0;
    };
    QHash<QString, Span> spans;
    for (std::size_t i = 0; i < pts.size(); ++i) {
        const Point &p = pts[i];
        if (!isPattern(p.segType) || p.derived) continue;
        if (auto it = spans.find(p.segId); it != spans.end()) it->last = i;
        else spans.insert(p.segId, {i, i});
    }
    static const QHash<QString, QString> segTags{{u"area"_s, u"AREA"_s}, {u"corridor"_s, u"CORR"_s}, {u"perimeter"_s, u"PERIM"_s}};
    static const QHash<QString, QString> kindTags{{u"takeoff"_s, u"T/O"_s}, {u"land"_s, u"LAND"_s}, {u"loiter"_s, u"LOIT"_s}};

    std::optional<QVector3D> from, home;   // where the vehicle sets off for the next item
    for (std::size_t i = 0; i < pts.size(); ++i) {
        const Point &p = pts[i];
        if (p.kind == "home"_L1) {
            if (!home) home = p.pos;
            from = p.pos;
            continue;
        }
        const auto span = isPattern(p.segType) ? spans.constFind(p.segId) : spans.cend();
        const bool hasSpan = span != spans.cend();
        const bool inPattern = hasSpan && i > span->first && i <= span->last;
        const Style style = inPattern ? Style::Pattern : Style::Route;
        auto leg = [&](const QVector3D &a, const QVector3D &b) {
            out.lines.push_back({a, b, style, inPattern ? p.segType : QString(), p.seq});
        };
        QVector3D exit = p.pos;
        const bool circle = p.kind == "loiter"_L1 && p.loiterRadius > 0;
        if (circle) {
            const Ring ring = loiterRing(p, from ? &*from : nullptr);
            exit = ring.entry;
            if (from) leg(*from, exit);
            for (const auto &[a, b] : ring.pairs) out.lines.push_back({a, b, Style::Ring, u"circle"_s, p.seq});
        } else if (p.kind == "land"_L1 && !p.glide && from) {
            const QVector3D over(p.pos.x(), std::max(from->y(), p.pos.y()), p.pos.z());
            if (across(over, *from) > 0.5f) leg(*from, over);
            leg(over, p.pos);
        } else if (from) {
            leg(*from, p.pos);
        }
        from = exit;

        // Symbol and label. Inside a pattern only its ends are marked
        if (p.derived) continue;
        const bool first = hasSpan && i == span->first, last = hasSpan && i == span->last;
        if (hasSpan && !first && !last) continue;
        const QString seq = QString::number(p.seq), alt = u"%1 m"_s.arg(std::lround(p.alt));
        Mark m;
        m.at = p.pos;
        m.ground = p.ground;
        m.seq = p.seq;
        if (circle) {
            m.shape = Shape::Circle;
            m.segType = u"circle"_s;
            m.main = seq + u" CIRC"_s;
            m.sub = alt + (p.turns > 1 ? u" · %1×"_s.arg(p.turns) : QString());
        } else if (hasSpan) {
            m.shape = Shape::Square;
            m.segType = p.segType;
            m.main = first ? seq + ' '_L1 + segTags.value(p.segType) : seq;
            if (first) m.sub = alt;
        } else {
            m.shape = shapeOf(p.kind);
            const QString tag = kindTags.value(p.kind);
            m.main = tag.isEmpty() ? seq : seq + ' '_L1 + tag;
            if (p.kind != "land"_L1) m.sub = alt + (p.hold > 0 ? u" · %1 s"_s.arg(std::lround(p.hold)) : QString());
            m.plain = p.kind == "wp"_L1;
        }
        out.marks.push_back(m);
    }

    // Return to launch: back over home at the height of the last leg, then down
    if (rtlSeq && home && from && *from != *home) {
        const QVector3D over(home->x(), std::max(from->y(), home->y()), home->z());
        if (across(over, *from) > 0.5f) out.lines.push_back({*from, over, Style::Return, {}, *rtlSeq});
        out.lines.push_back({over, *home, Style::Return, {}, *rtlSeq});
        Mark m;
        m.at = QVector3D((from->x() + over.x()) / 2, over.y(), (from->z() + over.z()) / 2);
        m.main = u"RTL"_s;
        out.marks.push_back(m);
    }

    // POIs are not flown: not in the route
    for (const Poi &poi : pois) {
        const float ground = poi.ground.value_or(poi.pos.y());
        Mark m;
        m.at = QVector3D(poi.pos.x(), std::max(poi.pos.y(), ground), poi.pos.z());
        m.shape = Shape::Target;
        m.segType = u"poi"_s;
        m.main = u"%1 POI"_s.arg(poi.seq);
        const long h = std::lround(m.at.y() - ground);
        if (h >= 1) m.sub = u"%1 m"_s.arg(h);
        m.ground = ground;
        m.seq = poi.seq;
        out.marks.push_back(m);
    }
    return out;
}

} // namespace corv::mission3d
