#include "vehicle/GeodesicGrid.h"

#include <cmath>
#include <utility>

namespace corv::geogrid {

namespace {

using M3 = std::array<std::array<double, 3>, 3>;

// AP_GeodesicGrid::_inverses: a vector to its coordinates in the basis of
// triangle T_i's vertices. T_{i+10} = -T_i.
constexpr M3 kInverses[10] = {
    {{{-0.309017, 0.500000, 0.190983}, {0.000000, 0.000000, -0.618034}, {-0.309017, -0.500000, 0.190983}}},
    {{{-0.190983, 0.309017, -0.500000}, {-0.500000, -0.190983, 0.309017}, {0.309017, -0.500000, -0.190983}}},
    {{{-0.618034, 0.000000, 0.000000}, {0.190983, -0.309017, -0.500000}, {0.190983, -0.309017, 0.500000}}},
    {{{-0.500000, 0.190983, -0.309017}, {0.000000, -0.618034, 0.000000}, {0.500000, 0.190983, -0.309017}}},
    {{{-0.190983, -0.309017, -0.500000}, {-0.190983, -0.309017, 0.500000}, {0.618034, 0.000000, 0.000000}}},
    {{{-0.309017, -0.500000, -0.190983}, {0.190983, 0.309017, -0.500000}, {0.500000, -0.190983, 0.309017}}},
    {{{0.309017, -0.500000, 0.190983}, {0.000000, 0.000000, -0.618034}, {0.309017, 0.500000, 0.190983}}},
    {{{0.190983, -0.309017, -0.500000}, {0.500000, 0.190983, 0.309017}, {-0.309017, 0.500000, -0.190983}}},
    {{{0.500000, -0.190983, -0.309017}, {0.000000, 0.618034, 0.000000}, {-0.500000, -0.190983, -0.309017}}},
    {{{0.309017, 0.500000, -0.190983}, {-0.500000, 0.190983, 0.309017}, {-0.190983, -0.309017, -0.500000}}},
};

// AP_GeodesicGrid::_mid_inverses: the same for the triangle of T_i's edge
// midpoints; a negative x, y or z coordinate means sub-triangle 3, 1 or 2,
// none the middle one, 0.
constexpr M3 kMidInverses[10] = {
    {{{0.000000, 1.000000, -0.618034}, {0.000000, -1.000000, -0.618034}, {-0.618034, 0.000000, 1.000000}}},
    {{{-1.000000, 0.618034, 0.000000}, {0.000000, -1.000000, 0.618034}, {0.618034, 0.000000, -1.000000}}},
    {{{-0.618034, 0.000000, -1.000000}, {1.000000, -0.618034, 0.000000}, {-0.618034, 0.000000, 1.000000}}},
    {{{-1.000000, -0.618034, 0.000000}, {1.000000, -0.618034, 0.000000}, {0.000000, 1.000000, -0.618034}}},
    {{{-1.000000, -0.618034, 0.000000}, {0.618034, 0.000000, 1.000000}, {0.618034, 0.000000, -1.000000}}},
    {{{-0.618034, 0.000000, -1.000000}, {1.000000, 0.618034, 0.000000}, {0.000000, -1.000000, 0.618034}}},
    {{{0.000000, -1.000000, -0.618034}, {0.000000, 1.000000, -0.618034}, {0.618034, 0.000000, 1.000000}}},
    {{{1.000000, -0.618034, 0.000000}, {0.000000, 1.000000, 0.618034}, {-0.618034, 0.000000, -1.000000}}},
    {{{1.000000, 0.618034, 0.000000}, {-1.000000, 0.618034, 0.000000}, {0.000000, -1.000000, -0.618034}}},
    {{{0.000000, 1.000000, 0.618034}, {-1.000000, -0.618034, 0.000000}, {0.618034, 0.000000, -1.000000}}},
};

struct V3 {
    double x = 0, y = 0, z = 0;
};

V3 mul(const M3 &m, const V3 &v)
{
    return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z, m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
}

M3 invert(const M3 &m)
{
    const double a = m[0][0], b = m[0][1], c = m[0][2], d = m[1][0], e = m[1][1], f = m[1][2], g = m[2][0], h = m[2][1], i = m[2][2];
    const double A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g;
    const double det = a * A + b * B + c * C;
    return {{{A / det, -(b * i - c * h) / det, (b * f - c * e) / det},
             {B / det, (a * i - c * g) / det, -(a * f - c * d) / det},
             {C / det, -(a * h - b * g) / det, (a * e - b * d) / det}}};
}

V3 norm(const V3 &v)
{
    const double l = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    return l > 0 ? V3{v.x / l, v.y / l, v.z / l} : v;
}
V3 mid(const V3 &p, const V3 &q) { return {(p.x + q.x) / 2, (p.y + q.y) / 2, (p.z + q.z) / 2}; }
V3 centroid(const V3 &p, const V3 &q, const V3 &r) { return {(p.x + q.x + r.x) / 3, (p.y + q.y + r.y) / 3, (p.z + q.z + r.z) / 3}; }
double dot(const V3 &p, const V3 &q) { return p.x * q.x + p.y * q.y + p.z * q.z; }
V3 cross(const V3 &p, const V3 &q) { return {p.y * q.z - p.z * q.y, p.z * q.x - p.x * q.z, p.x * q.y - p.y * q.x}; }
V3 sub(const V3 &p, const V3 &q) { return {p.x - q.x, p.y - q.y, p.z - q.z}; }
QVector3D q3(const V3 &v) { return QVector3D(float(v.x), float(v.y), float(v.z)); }

/// AP_GeodesicGrid::_subtriangle_index for a vector known to cross T_triangle
int subtriangleIndex(int triangle, const V3 &v)
{
    V3 w = mul(kMidInverses[triangle % 10], v);
    if (triangle > 9) w = {-w.x, -w.y, -w.z};
    if (w.x < 0) return 3;
    if (w.y < 0) return 1;
    if (w.z < 0) return 2;
    return 0;
}

struct Built {
    std::array<Section, kSections> sec;
    // Double precision copies for the point-in-triangle test
    std::array<std::array<V3, 4>, kSections> d;   // a, b, c, center
};

const Built &built()
{
    static const Built b = [] {
        Built out;
        for (int t = 0; t < 20; ++t) {
            const M3 m = invert(kInverses[t % 10]);
            const double sign = t > 9 ? -1 : 1;
            // Columns of the inverse are the triangle's vertices
            V3 v[3];
            for (int col = 0; col < 3; ++col) v[col] = norm({m[0][col] * sign, m[1][col] * sign, m[2][col] * sign});
            const V3 &a = v[0], &bb = v[1], &c = v[2];
            const V3 ab = mid(a, bb), bc = mid(bb, c), ca = mid(c, a);
            const V3 tris[4][3] = {{a, ab, ca}, {ab, bb, bc}, {ca, bc, c}, {ab, bc, ca}};
            for (const auto &tri : tris) {
                const int s = 4 * t + subtriangleIndex(t, centroid(tri[0], tri[1], tri[2]));
                V3 p = norm(tri[0]), q = norm(tri[1]), r = norm(tri[2]);
                if (dot(cross(sub(q, p), sub(r, p)), p) < 0) std::swap(q, r);
                const V3 cen = norm(centroid(p, q, r));
                out.sec[std::size_t(s)] = {q3(p), q3(q), q3(r), q3(cen)};
                out.d[std::size_t(s)] = {p, q, r, cen};
            }
        }
        return out;
    }();
    return b;
}

} // namespace

const std::array<Section, kSections> &sections() { return built().sec; }

int sectionOf(const QVector3D &vf)
{
    const V3 v{vf.x(), vf.y(), vf.z()};
    if (v.x == 0 && v.y == 0 && v.z == 0) return -1;
    const Built &b = built();
    int best = -1;
    double bestDot = -1e300;
    for (int s = 0; s < kSections; ++s) {
        const auto &t = b.d[std::size_t(s)];
        if (dot(cross(t[0], t[1]), v) >= 0 && dot(cross(t[1], t[2]), v) >= 0 && dot(cross(t[2], t[0]), v) >= 0) return s;
        const double d = dot(t[3], v);
        if (d > bestDot) {
            bestDot = d;
            best = s;
        }
    }
    return best;   // on an edge, within rounding: the nearest section
}

int maskCount(const Mask &m)
{
    int n = 0;
    for (int s = 0; s < kSections; ++s) n += maskHas(m, s) ? 1 : 0;
    return n;
}

} // namespace corv::geogrid
