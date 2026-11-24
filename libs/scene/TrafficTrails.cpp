#include "scene/TrafficTrails.h"

#include "core/Geo.h"

#include <QSet>
#include <algorithm>
#include <cmath>

namespace corv {

namespace {
constexpr std::size_t kMaxSamples = 90;   // reports kept per aircraft (a 1 Hz feed: 90 s)
constexpr double kStepM = 60;             // resolution of the curve
constexpr std::size_t kMaxPoints = 512;

struct Vec {
    double n = 0, e = 0, u = 0;
};
double dot(const Vec &a, const Vec &b) { return a.n * b.n + a.e * b.e + a.u * b.u; }
double norm(const Vec &a) { return std::sqrt(dot(a, a)); }
} // namespace

void TrafficTrails::update(const std::vector<Report> &reports)
{
    QSet<QString> listed;
    for (const Report &r : reports) {
        if (r.id.isEmpty()) continue;
        listed.insert(r.id);
        Track &tr = m_tracks[r.id];
        if (r.timeMs == tr.reportTs) continue;
        tr.reportTs = r.timeMs;
        Sample s;
        s.t = r.timeMs / 1000.0;
        s.lat = r.lat;
        s.lon = r.lon;
        s.alt = r.alt;
        const double h = r.heading * kDeg;
        s.vn = r.velocity * std::cos(h);
        s.ve = r.velocity * std::sin(h);
        s.vu = r.vertRate;
        if (!tr.samples.empty()) {
            const Sample &last = tr.samples.back();
            // OpenSky repeats a state vector until the aircraft reports again: the same
            // position with a newer time would only restart the extrapolation
            if (s.t <= last.t || std::hypot(flatDistance(last.lat, last.lon, s.lat, s.lon), s.alt - last.alt) < 1) continue;
        }
        tr.samples.push_back(s);
        const double cutoff = s.t - kTrailSeconds - 30;
        std::size_t drop = 0;
        while (tr.samples.size() - drop > 2 && (tr.samples.size() - drop > kMaxSamples || tr.samples[drop + 1].t < cutoff)) ++drop;
        tr.samples.erase(tr.samples.begin(), tr.samples.begin() + std::ptrdiff_t(drop));
    }
    m_tracks.removeIf([&](const QHash<QString, Track>::iterator it) { return !listed.contains(it.key()); });
}

bool TrafficTrails::head(const QString &id, double nowMs, double *lat, double *lon, double *alt) const
{
    const auto it = m_tracks.constFind(id);
    if (it == m_tracks.cend() || it->samples.empty()) return false;
    const Sample &s = it->samples.back();
    const double dt = std::clamp(nowMs / 1000.0 - s.t, 0.0, kMaxDeadReckonS);
    *lat = s.lat + s.vn * dt / kMetersPerDegLat;
    *lon = s.lon + s.ve * dt / (kMetersPerDegLat * std::max(0.01, std::cos(s.lat * kDeg)));
    *alt = s.alt + s.vu * dt;
    return true;
}

std::vector<TrafficTrails::Vertex> TrafficTrails::trail(const QString &id, double nowMs) const
{
    std::vector<Vertex> out;
    const auto it = m_tracks.constFind(id);
    if (it == m_tracks.cend() || it->samples.empty()) return out;
    const std::vector<Sample> &s = it->samples;
    const double now = nowMs / 1000.0;
    // Fading by age, the square of what is left
    auto emitAt = [&](double lat, double lon, double alt, double t) {
        if (out.size() >= kMaxPoints) return;
        const double a = std::max(0.0, 1 - std::max(0.0, now - t) / kTrailSeconds);
        out.push_back({lat, lon, alt, a * a});
    };
    // Reports that have entirely faded are skipped; a dense feed is stepped over
    std::size_t first = 0;
    while (first + 1 < s.size() && now - s[first + 1].t > kTrailSeconds) ++first;
    const std::size_t stride = std::max<std::size_t>(1, (s.size() - first + kMaxPoints / 4 - 1) / (kMaxPoints / 4));

    emitAt(s[first].lat, s[first].lon, s[first].alt, s[first].t);
    for (std::size_t i = first; i + 1 < s.size();) {
        const std::size_t j = std::min(i + stride, s.size() - 1);
        const Sample &a = s[i], &b = s[j];
        // In metres north, east, up from a
        const double mLon = kMetersPerDegLat * std::max(0.01, std::cos(a.lat * kDeg));
        const Vec chord{(b.lat - a.lat) * kMetersPerDegLat, (b.lon - a.lon) * mLon, b.alt - a.alt};
        const double len = norm(chord), dt = std::max(1e-3, b.t - a.t);
        // The reported velocity over the segment's duration; the straight chord when it is
        // missing or implausible (pointing back, overshooting by far)
        auto tangent = [&](const Sample &q) {
            Vec m{q.vn * dt, q.ve * dt, q.vu * dt};
            if (dot(m, chord) <= 0) return chord;
            if (const double l = norm(m); l > 1.5 * len && l > 0) m = {m.n * 1.5 * len / l, m.e * 1.5 * len / l, m.u * 1.5 * len / l};
            return m;
        };
        const Vec m0 = tangent(a), m1 = tangent(b);
        const int steps = std::clamp(int(std::ceil(len / kStepM)), 1, 32);
        for (int k = 1; k <= steps; ++k) {
            const double u = double(k) / steps, u2 = u * u, u3 = u2 * u;
            const double h10 = u3 - 2 * u2 + u, h01 = -2 * u3 + 3 * u2, h11 = u3 - u2;
            const Vec p{h10 * m0.n + h01 * chord.n + h11 * m1.n, h10 * m0.e + h01 * chord.e + h11 * m1.e,
                        h10 * m0.u + h01 * chord.u + h11 * m1.u};
            emitAt(a.lat + p.n / kMetersPerDegLat, a.lon + p.e / mLon, a.alt + p.u, a.t + u * dt);
        }
        i = j;
    }
    double lat = 0, lon = 0, alt = 0;
    if (head(id, nowMs, &lat, &lon, &alt)) emitAt(lat, lon, alt, s.back().t + std::clamp(now - s.back().t, 0.0, kMaxDeadReckonS));
    return out;
}

} // namespace corv
