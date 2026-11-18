#include "vehicle/MagSamples.h"

#include <cmath>
#include <set>
#include <utility>

namespace corv {

bool MagSamples::add(int compass, float x, float y, float z)
{
    if (compass < 0 || compass >= kCompasses) return false;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return false;
    if (x == 0 && y == 0 && z == 0) return false;   // compass absent: the fields stay 0
    Cloud &c = m_clouds[std::size_t(compass)];
    const QVector3D p(x, y, z);
    c.latest = p;
    const float r = c.fit ? c.fit->r : p.length();
    if (c.last && (p - *c.last).length() < kMinStep * r) return false;
    if (int(c.points.size()) < kMaxPoints) c.points.push_back(p);
    else c.points[std::size_t(c.next)] = p;
    c.next = (c.next + 1) % kMaxPoints;
    c.last = p;
    return true;
}

void MagSamples::refit()
{
    for (Cloud &c : m_clouds) c.fit = fitSphere(c.points);
}

void MagSamples::clear()
{
    for (Cloud &c : m_clouds) c = Cloud{};
    m_mask.reset();
}

MagSamples::Fit MagSamples::frame(int compass) const
{
    const Cloud &c = m_clouds[std::size_t(compass)];
    if (c.fit) return *c.fit;
    const float r = c.latest ? c.latest->length() : 1.f;
    return {QVector3D(), r > 0 ? r : 1.f};
}

void MagSamples::setFocus(int compass)
{
    if (compass >= 0 && compass < kCompasses) m_focus = compass;
}

void MagSamples::setCoverage(const std::optional<geogrid::Mask> &mask) { m_mask = mask; }

std::optional<QVector3D> MagSamples::marker() const
{
    const Cloud &c = m_clouds[std::size_t(m_focus)];
    if (!c.latest) return std::nullopt;
    const Fit f = frame(m_focus);
    return (*c.latest - f.center) / f.r;
}

int MagSamples::current() const
{
    const auto m = marker();
    return m ? geogrid::sectionOf(*m) : -1;
}

int MagSamples::target() const
{
    const auto m = marker();
    if (!m || !m_mask) return -1;
    const float l = m->length() > 0 ? m->length() : 1.f;
    int best = -1;
    float bestDot = -1e30f;
    const auto &secs = geogrid::sections();
    for (int s = 0; s < geogrid::kSections; ++s) {
        if (geogrid::maskHas(*m_mask, s)) continue;
        const float d = QVector3D::dotProduct(secs[std::size_t(s)].center, *m) / l;
        if (d > bestDot) {
            bestDot = d;
            best = s;
        }
    }
    return best;
}

std::optional<float> MagSamples::field() const
{
    const Cloud &c = m_clouds[std::size_t(m_focus)];
    if (c.fit) return c.fit->r;
    if (c.latest) return c.latest->length();
    return std::nullopt;
}

namespace {

/// Gaussian elimination with partial pivoting on [A | B]; false when singular.
bool solve4(double m[4][5], double out[4])
{
    for (int c = 0; c < 4; ++c) {
        int p = c;
        for (int r = c + 1; r < 4; ++r)
            if (std::abs(m[r][c]) > std::abs(m[p][c])) p = r;
        if (std::abs(m[p][c]) < 1e-9) return false;
        if (p != c)
            for (int k = 0; k < 5; ++k) std::swap(m[c][k], m[p][k]);
        for (int r = 0; r < 4; ++r) {
            if (r == c) continue;
            const double f = m[r][c] / m[c][c];
            for (int k = c; k < 5; ++k) m[r][k] -= f * m[c][k];
        }
    }
    for (int i = 0; i < 4; ++i) out[i] = m[i][4] / m[i][i];
    return true;
}

} // namespace

std::optional<MagSamples::Fit> MagSamples::fitSphere(const std::vector<QVector3D> &pts)
{
    const std::size_t n = pts.size();
    if (int(n) < kFitMinPoints) return std::nullopt;
    double m[4][5] = {};
    for (const QVector3D &p : pts) {
        const double row[4] = {2.0 * p.x(), 2.0 * p.y(), 2.0 * p.z(), 1.0};
        const double rhs = double(p.x()) * p.x() + double(p.y()) * p.y() + double(p.z()) * p.z();
        for (int j = 0; j < 4; ++j) {
            m[j][4] += row[j] * rhs;
            for (int k = 0; k < 4; ++k) m[j][k] += row[j] * row[k];
        }
    }
    double sol[4];
    if (!solve4(m, sol)) return std::nullopt;
    const double r2 = sol[3] + sol[0] * sol[0] + sol[1] * sol[1] + sol[2] * sol[2];
    if (!(r2 > 0)) return std::nullopt;
    const double r = std::sqrt(r2);
    const QVector3D center{float(sol[0]), float(sol[1]), float(sol[2])};

    std::set<int> sections;
    double sq = 0;
    for (const QVector3D &p : pts) {
        const QVector3D d = p - center;
        const double e = double(d.length()) - r;
        sq += e * e;
        sections.insert(geogrid::sectionOf(d));
    }
    if (int(sections.size()) < kFitMinSections || std::sqrt(sq / double(n)) > 0.25 * r) return std::nullopt;
    return Fit{center, float(r)};
}

} // namespace corv
