#include "mission/RadioLink.h"

#include "mission/RouteGeo.h"

#include <algorithm>
#include <cmath>
#include <limits>

using namespace Qt::StringLiterals;

namespace corv::mission {

namespace {
constexpr double kC = 299792458.0;
constexpr double kEarth43 = (4.0 / 3.0) * 6371000.0;   // effective earth radius (standard refraction)
constexpr double kNuClear = -0.78;                     // ITU-R P.526: J(ν) ≈ 0 dB below this

double r(const QJsonObject &o, const char *k) { return num(o.value(QLatin1String(k))); }

QString fmtKm(double m)
{
    return m >= 1000 ? u"%1 km"_s.arg(m / 1000, 0, 'f', 1) : u"%1 m"_s.arg(std::lround(m));
}

struct Edge { double nu; int j; };
Edge worstKnifeEdge(const std::vector<double> &terr, int n, double step, double D, double h0, double h1, double lambda)
{
    const double slope = (h1 - h0) / D;
    const double c = 2 / lambda;
    const double bulgeK = 1 / (2 * kEarth43);
    double best = -std::numeric_limits<double>::infinity();
    int bj = -1;
    for (int j = 1; j < n; ++j) {
        const double d1 = j * step, d2 = D - d1;
        if (d2 <= 0) break;
        const double h = terr[j] + d1 * d2 * bulgeK - (h0 + slope * d1);
        const double nu = h * std::sqrt(c * (1 / d1 + 1 / d2));
        if (nu > best) { best = nu; bj = j; }
    }
    return {best, bj};
}

Link classify(const QJsonObject &radio, double margin, double nu)
{
    if (!std::isfinite(margin)) return Link::Unknown;
    if (margin < 0) return Link::None;
    if (margin < r(radio, "minMargin")) return Link::Marginal;
    if (nu > kNuClear) return Link::Degraded;
    return Link::Good;
}
} // namespace

const LinkStyle &linkStyle(Link c)
{
    static const LinkStyle styles[5] = {
        {u"no link"_s, u"NO LINK"_s, QColor(0x9a, 0xa4, 0xb0), QColor(0, 0, 0, 0)},
        {u"below safety margin"_s, u"MARGINAL"_s, QColor(0xff, 0x3b, 0x3b), QColor(255, 60, 60, int(0.32 * 255))},
        {u"degraded link"_s, u"DEGRADED"_s, QColor(0xff, 0xaa, 0x00), QColor(255, 170, 0, int(0.30 * 255))},
        {u"good link"_s, u"GOOD"_s, QColor(0x60, 0xff, 0x80), QColor(96, 255, 128, int(0.26 * 255))},
        {u"no terrain data"_s, u"UNKNOWN"_s, QColor(0x4a, 0x55, 0x60), QColor(0, 0, 0, 0)},
    };
    return styles[std::clamp(int(c), 0, 4)];
}

double wavelength(double fMHz) { return kC / (std::max(1.0, fMHz ? fMHz : 1) * 1e6); }

double fspl(double dM, double fMHz)
{
    const double km = std::max(1.0, dM) / 1000;
    return 20 * std::log10(km) + 20 * std::log10(std::max(1.0, fMHz ? fMHz : 1)) + 32.44;
}

double fresnelRadius(double d1, double d2, double fMHz)
{
    if (d1 <= 0 || d2 <= 0) return 0;
    return std::sqrt(wavelength(fMHz) * d1 * d2 / (d1 + d2));
}

double knifeEdgeLoss(double nu)
{
    if (!(nu > kNuClear)) return 0;
    const double t = nu - 0.1;
    return 6.9 + 20 * std::log10(std::sqrt(t * t + 1) + t);
}

double systemGain(const QJsonObject &radio)
{
    return r(radio, "txPower") + r(radio, "gainGround") + r(radio, "gainAir") - r(radio, "losses");
}

double freeSpaceRange(const QJsonObject &radio, double marginDb)
{
    const double budget = systemGain(radio) - r(radio, "minRssi") - marginDb;
    const double f = r(radio, "freq");
    const double km = std::pow(10, (budget - 20 * std::log10(std::max(1.0, f ? f : 1)) - 32.44) / 20);
    return km * 1000;
}

GroundStation groundStation(const QJsonObject &radio, const LL &pos, const TerrainFn &terrain)
{
    GroundStation gs;
    gs.lat = pos.lat;
    gs.lng = pos.lng;
    gs.elev = terrain(pos.lat, pos.lng);
    gs.height = std::max(0.0, r(radio, "groundHeight"));
    if (gs.elev) gs.antennaMsl = *gs.elev + gs.height;
    return gs;
}

LinkEval evaluateLink(const QJsonObject &radio, const GroundStation &gs, double lat, double lng, double altMsl,
                      const TerrainFn &terrain, double step, int maxSamples, bool keepProfile)
{
    const LocalFrame f({gs.lat, gs.lng});
    const XY xy = f.toXY({lat, lng});
    const double D = std::hypot(xy.x, xy.y);
    LinkEval unknown;
    unknown.dist = D;
    if (!gs.antennaMsl || !std::isfinite(altMsl)) return unknown;
    const double h0 = *gs.antennaMsl, h1 = altMsl;
    const int n = std::max(2, std::min(maxSamples, int(std::ceil(D / step))));
    const double ds = D / n;
    std::vector<double> terr(std::size_t(n) + 1);
    for (int j = 0; j <= n; ++j) {
        const double t = double(j) / n;
        const LL ll = f.toLL({xy.x * t, xy.y * t});
        const auto g = terrain(ll.lat, ll.lng);
        if (!g) return unknown;
        terr[std::size_t(j)] = *g;
    }
    const double freq = r(radio, "freq");
    const double lambda = wavelength(freq);
    const Edge e = D > 0 ? worstKnifeEdge(terr, n, ds, D, h0, h1, lambda) : Edge{-std::numeric_limits<double>::infinity(), -1};
    LinkEval res;
    res.dist = D;
    res.nu = e.nu;
    res.loss = knifeEdgeLoss(e.nu);
    res.fspl = fspl(D, freq);
    res.rssi = systemGain(radio) - res.fspl - res.loss;
    res.margin = res.rssi - r(radio, "minRssi");
    res.clazz = classify(radio, res.margin, e.nu);
    res.obstructed = e.nu > 0;
    res.fresnelClear = !(e.nu > kNuClear);
    if (e.j >= 0) {
        const double d1 = e.j * ds, d2 = D - d1;
        const double line = h0 + (h1 - h0) * (d1 / D);
        res.worst = KnifeEdge{d1, terr[std::size_t(e.j)], line, line - terr[std::size_t(e.j)] - d1 * d2 / (2 * kEarth43), fresnelRadius(d1, d2, freq)};
    }
    if (keepProfile) res.profile = LinkProfile{n, ds, h0, h1, D, lambda, freq, std::move(terr)};
    return res;
}

std::optional<LinkAnalysis> analyzeRoute(const QJsonObject &radio, const GroundStation &gs, const QList<NavPoint> &navPath,
                                         const TerrainFn &terrain, double profileStep)
{
    if (navPath.size() < 2) return std::nullopt;
    LinkAnalysis A;
    A.total = navPath.last().dist;
    const double stepM = std::max(20.0, std::min(100.0, A.total / 600));
    auto eval1 = [&](double lat, double lng, double altMsl, double dist, const QString &segId) {
        const LinkEval ev = evaluateLink(radio, gs, lat, lng, altMsl, terrain, profileStep);
        A.samples.append({dist, lat, lng, altMsl, segId, ev.clazz, ev.margin, ev.rssi, ev.nu, ev.loss, ev.dist});
    };
    for (qsizetype i = 1; i < navPath.size(); ++i) {
        const NavPoint &a = navPath[i - 1], &b = navPath[i];
        const double len = b.dist - a.dist;
        const int nSub = std::max(1, int(std::ceil(len / stepM)));
        for (int s = 0; s < nSub; ++s) {
            const double t = double(s) / nSub;
            eval1(a.lat + (b.lat - a.lat) * t, a.lng + (b.lng - a.lng) * t, a.altMsl + (b.altMsl - a.altMsl) * t, a.dist + len * t,
                  b.segId.isEmpty() ? a.segId : b.segId);
        }
    }
    const NavPoint &last = navPath.last();
    eval1(last.lat, last.lng, last.altMsl, last.dist, last.segId);

    // Runs of equal class; each run repeats the first point of the next so the halo is continuous
    for (const LinkSample &s : A.samples) {
        if (A.runs.isEmpty() || A.runs.last().clazz != s.clazz) {
            if (!A.runs.isEmpty()) { A.runs.last().points.append({s.lat, s.lng}); A.runs.last().to = s.dist; }
            A.runs.append(LinkRun{s.clazz, s.dist, s.dist, {{s.lat, s.lng}}});
        } else {
            A.runs.last().points.append({s.lat, s.lng});
            A.runs.last().to = s.dist;
        }
    }
    for (const LinkRun &run : A.runs) A.lengths[int(run.clazz)] += run.to - run.from;
    for (const LinkSample &s : A.samples) {
        if (s.clazz == Link::Unknown) continue;
        if (!A.worst || int(s.clazz) < int(A.worst->clazz) || (s.clazz == A.worst->clazz && s.margin < A.worst->margin)) A.worst = s;
    }
    for (const LinkSample &s : A.samples) A.maxGsDist = std::max(A.maxGsDist, s.gsDist);
    return A;
}

QString summarizeLink(const LinkAnalysis &a)
{
    QStringList parts;
    if (a.lengths[int(Link::None)] > 0) parts << u"no link %1"_s.arg(fmtKm(a.lengths[int(Link::None)]));
    if (a.lengths[int(Link::Marginal)] > 0) parts << u"marginal %1"_s.arg(fmtKm(a.lengths[int(Link::Marginal)]));
    if (a.lengths[int(Link::Degraded)] > 0) parts << u"degraded %1"_s.arg(fmtKm(a.lengths[int(Link::Degraded)]));
    if (a.lengths[int(Link::Unknown)] > 0) parts << u"no terrain %1"_s.arg(fmtKm(a.lengths[int(Link::Unknown)]));
    return parts.isEmpty() ? u"good link along the whole route"_s : parts.join(u" · "_s);
}

std::optional<Coverage> computeCoverage(const QJsonObject &radio, const GroundStation &gs, const std::function<double(double)> &altAt,
                                        const TerrainFn &terrain, double rangeIn, const std::atomic<bool> *cancel)
{
    Coverage cov;
    cov.range = std::max(200.0, rangeIn > 0 ? rangeIn : 15000);
    cov.rays = 720;
    cov.count = std::max(8, std::min(400, int(std::ceil(cov.range / 30))));
    cov.step = cov.range / cov.count;
    cov.classes.assign(std::size_t(cov.rays) * cov.count, uint8_t(Link::Unknown));
    cov.gs = {gs.lat, gs.lng};
    if (!gs.antennaMsl || !gs.elev) return cov;

    const LocalFrame f({gs.lat, gs.lng});
    const double h0 = *gs.antennaMsl;
    const double freq = r(radio, "freq");
    const double c = 2 / wavelength(freq);
    const double bulgeK = 1 / (2 * kEarth43);
    const double gain = systemGain(radio);
    const double minRssi = r(radio, "minRssi");
    const double minMargin = r(radio, "minMargin");
    std::vector<double> terr(std::size_t(cov.count) + 1), invD(std::size_t(cov.count) + 1);
    for (int j = 1; j <= cov.count; ++j) invD[std::size_t(j)] = 1 / (j * cov.step);

    for (int ray = 0; ray < cov.rays; ++ray) {
        if (cancel && cancel->load()) return std::nullopt;
        const double az = double(ray) / cov.rays * 2 * M_PI;
        const double sx = std::sin(az), cy = std::cos(az);
        terr[0] = *gs.elev;
        int known = cov.count;   // the first unknown sample ends the ray
        for (int j = 1; j <= cov.count; ++j) {
            const double d = j * cov.step;
            const LL ll = f.toLL({d * sx, d * cy});
            const auto g = terrain(ll.lat, ll.lng);
            if (!g) { known = j - 1; break; }
            terr[std::size_t(j)] = *g;
        }
        const std::size_t base = std::size_t(ray) * cov.count;
        for (int k = 1; k <= known; ++k) {
            const double D = k * cov.step;
            const double h1 = altAt(terr[std::size_t(k)]);
            if (!std::isfinite(h1)) continue;
            Link clazz;
            if (h1 < terr[std::size_t(k)]) {
                clazz = Link::None;   // planned altitude under the ground here
            } else {
                const double slope = (h1 - h0) / D;
                double best = -std::numeric_limits<double>::infinity();
                for (int j = 1; j < k; ++j) {
                    const double d1 = j * cov.step, d2 = D - d1;
                    const double h = terr[std::size_t(j)] + d1 * d2 * bulgeK - (h0 + slope * d1);
                    const double nu = h * std::sqrt(c * (invD[std::size_t(j)] + 1 / d2));
                    if (nu > best) best = nu;
                }
                const double margin = gain - fspl(D, freq) - knifeEdgeLoss(best) - minRssi;
                clazz = margin < 0 ? Link::None : margin < minMargin ? Link::Marginal : best > kNuClear ? Link::Degraded : Link::Good;
            }
            cov.classes[base + std::size_t(k) - 1] = uint8_t(clazz);
        }
    }
    return cov;
}

Link coverageClassAt(const Coverage &cov, double xEast, double yNorth)
{
    const double d = std::hypot(xEast, yNorth);
    if (d > cov.range || cov.rays == 0) return Link::Unknown;
    double az = std::atan2(xEast, yNorth);
    if (az < 0) az += 2 * M_PI;
    const int ray = int(std::lround(az / (2 * M_PI) * cov.rays)) % cov.rays;
    const int k = std::min(cov.count - 1, int(std::floor(d / cov.step)));
    return Link(cov.classes[std::size_t(ray) * cov.count + std::size_t(k)]);
}

} // namespace corv::mission
