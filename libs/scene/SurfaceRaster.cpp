#include "scene/SurfaceRaster.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <vector>

namespace corv::raster {

namespace {
constexpr double kJumpMin = 0.4;     // m
constexpr double kJumpSlope = 1.5;   // tan 56 deg

// Scratch levels per grid size: sum, weight, min, max of the cells below
struct Level {
    int w = 0, h = 0;
    std::vector<double> sum, wt;
    std::vector<float> lo, hi;
    std::vector<std::uint8_t> ok;
};

std::vector<Level> &levelsFor(int w, int h)
{
    thread_local std::map<std::pair<int, int>, std::vector<Level>> cache;
    auto &L = cache[{w, h}];
    if (L.empty()) {
        int lw = w, lh = h;
        for (int k = 1; k <= kLevels; ++k) {
            lw = (lw + 1) / 2;
            lh = (lh + 1) / 2;
            Level l;
            l.w = lw;
            l.h = lh;
            const std::size_t n = std::size_t(lw) * std::size_t(lh);
            l.sum.resize(n);
            l.wt.resize(n);
            l.lo.resize(n);
            l.hi.resize(n);
            l.ok.resize(n);
            L.push_back(std::move(l));
        }
    }
    return L;
}

constexpr int kSatN = 10;
} // namespace

double jumpTolerance(double spanCells, double cell) { return std::max(kJumpMin, kJumpSlope * spanCells * cell); }

int fillLevelsFor(double metres, double cell)
{
    if (!(metres > 0)) return kFillLevels;
    return std::max(1, std::min(kLevels, int(std::floor(std::log2(metres / cell) + 1e-9))));
}

std::array<std::uint32_t, kLevels + 1> rasterize(const float *data, const float *wts, int w, int h, float *out, const Options &opts)
{
    const double minW = opts.minSamples, cell = opts.cell;
    const bool guard = opts.jumps;
    const int fillLevels = opts.fillLevels;
    auto &levels = levelsFor(w, h);
    // Pull
    const Level *fine = nullptr;
    int fw = w, fh = h;
    for (int k = 0; k < kLevels; ++k) {
        Level &L = levels[std::size_t(k)];
        const double tol = jumpTolerance(2 << k, cell);
        std::fill(L.sum.begin(), L.sum.end(), 0.0);
        std::fill(L.wt.begin(), L.wt.end(), 0.0);
        std::fill(L.lo.begin(), L.lo.end(), std::numeric_limits<float>::infinity());
        std::fill(L.hi.begin(), L.hi.end(), -std::numeric_limits<float>::infinity());
        for (int j = 0; j < fh; ++j) {
            for (int i = 0; i < fw; ++i) {
                const int p = j * fw + i;
                double s, wt;
                float lo, hi;
                if (fine) {
                    wt = fine->wt[std::size_t(p)];
                    if (!wt) continue;
                    s = fine->sum[std::size_t(p)];
                    lo = fine->lo[std::size_t(p)];
                    hi = fine->hi[std::size_t(p)];
                } else {
                    wt = wts[p];
                    if (!wt) continue;
                    const float v = data[p];
                    s = double(v) * wt;
                    lo = hi = v;
                }
                const std::size_t q = std::size_t((j >> 1) * L.w + (i >> 1));
                L.sum[q] += s;
                L.wt[q] += wt;
                if (lo < L.lo[q]) L.lo[q] = lo;
                if (hi > L.hi[q]) L.hi[q] = hi;
            }
        }
        for (std::size_t q = 0; q < L.ok.size(); ++q) L.ok[q] = L.wt[q] > 0 && (!guard || L.hi[q] - L.lo[q] <= tol) ? 1 : 0;
        fine = &L;
        fw = L.w;
        fh = L.h;
    }
    // Push
    std::array<std::uint32_t, kLevels + 1> count{};
    for (int j = 0; j < h; ++j) {
        for (int i = 0; i < w; ++i) {
            const int p = j * w + i;
            const bool own = wts[p] > 0;
            const float v = data[p];
            out[p * 2] = kEmpty;
            out[p * 2 + 1] = 0;
            if (own && wts[p] >= minW) {
                out[p * 2] = v;
                ++count[0];
                continue;
            }
            const int top = own || !guard ? kLevels : fillLevels;
            for (int k = 0; k < top; ++k) {
                const Level &L = levels[std::size_t(k)];
                const int f = 2 << k;
                const double tol = jumpTolerance(f, cell);
                const double u = (i + 0.5) / f - 0.5, t = (j + 0.5) / f - 0.5;
                const int i0 = int(std::floor(u)), j0 = int(std::floor(t));
                const double fu = u - i0, ft = t - j0;
                double s = 0, sw = 0, lo = std::numeric_limits<double>::infinity(), hi = -std::numeric_limits<double>::infinity();
                int corners = 0;
                for (int dj = 0; dj < 2; ++dj) {
                    const int jj = j0 + dj;
                    if (jj < 0 || jj >= L.h) continue;
                    for (int di = 0; di < 2; ++di) {
                        const int ii = i0 + di;
                        if (ii < 0 || ii >= L.w) continue;
                        const std::size_t q = std::size_t(jj * L.w + ii);
                        if (!(guard ? L.ok[q] != 0 : L.wt[q] > 0)) continue;
                        const double bw = (di ? fu : 1 - fu) * (dj ? ft : 1 - ft);
                        const double m = L.sum[q] / L.wt[q];
                        s += bw * L.sum[q];
                        sw += bw * L.wt[q];
                        ++corners;
                        if (m < lo) lo = m;
                        if (m > hi) hi = m;
                    }
                }
                if (sw < minW) continue;
                const double val = s / sw;
                if (guard) {
                    if (!own && corners < 4) continue;   // interpolate holes, never extend edges
                    if (hi - lo > jumpTolerance(2 * f, cell)) continue;
                    if (own && std::abs(val - v) > tol) continue;
                }
                out[p * 2] = float(val);
                out[p * 2 + 1] = float(k + 1);
                ++count[std::size_t(k + 1)];
                break;
            }
            // Under-sampled, and no coarser level agrees with it: its own height, still real
            if (own && out[p * 2] == kEmpty && guard) {
                out[p * 2] = v;
                ++count[0];
            }
        }
    }
    return count;
}

void roughness(const float *view, int w, int h, std::uint8_t *out, double cell, double radius)
{
    const int r = std::max(1, int(std::lround(radius / cell)));
    const int W = w + 1;
    thread_local std::map<std::pair<int, int>, std::vector<double>> satCache;
    std::vector<double> &sat = satCache[{w, h}];
    if (sat.empty()) sat.resize(std::size_t(W) * std::size_t(h + 1) * kSatN);
    // Heights from a reference in the block: the moments stay small
    double z0 = std::numeric_limits<double>::quiet_NaN();
    for (int p = 0; p < w * h && std::isnan(z0); ++p)
        if (view[p * 2] > kEmpty) z0 = view[p * 2];
    std::fill(out, out + std::size_t(w) * std::size_t(h), std::uint8_t(0));
    if (std::isnan(z0)) return;
    double m[kSatN];
    for (int j = 0; j <= h; ++j) {
        for (int i = 0; i <= w; ++i) {
            const std::size_t o = std::size_t(j * W + i) * kSatN;
            if (!i || !j) {
                for (int c = 0; c < kSatN; ++c) sat[o + std::size_t(c)] = 0;
                continue;
            }
            const float zr = view[((j - 1) * w + (i - 1)) * 2];
            std::fill(m, m + kSatN, 0.0);
            if (zr > kEmpty) {
                const double x = i - 1, y = j - 1, z = zr - z0;
                m[0] = 1; m[1] = x; m[2] = y; m[3] = z; m[4] = x * x; m[5] = y * y; m[6] = x * y; m[7] = x * z; m[8] = y * z; m[9] = z * z;
            }
            const std::size_t a = std::size_t((j - 1) * W + i) * kSatN, b = std::size_t(j * W + i - 1) * kSatN,
                              d = std::size_t((j - 1) * W + i - 1) * kSatN;
            for (int c = 0; c < kSatN; ++c) sat[o + std::size_t(c)] = m[c] + sat[a + std::size_t(c)] + sat[b + std::size_t(c)] - sat[d + std::size_t(c)];
        }
    }
    const double minN = std::max(4.0, std::ceil((2.0 * r + 1) * (2.0 * r + 1) / 4));
    for (int j = 0; j < h; ++j) {
        const int j0 = std::max(0, j - r), j1 = std::min(h, j + r + 1);
        for (int i = 0; i < w; ++i) {
            if (!(view[(j * w + i) * 2] > kEmpty)) continue;
            const int i0 = std::max(0, i - r), i1 = std::min(w, i + r + 1);
            const std::size_t A = std::size_t(j1 * W + i1) * kSatN, B = std::size_t(j0 * W + i1) * kSatN, C = std::size_t(j1 * W + i0) * kSatN,
                              D = std::size_t(j0 * W + i0) * kSatN;
            const double n = sat[A] - sat[B] - sat[C] + sat[D];
            if (n < minN) continue;
            auto S = [&](int c) { return (sat[A + std::size_t(c)] - sat[B + std::size_t(c)] - sat[C + std::size_t(c)] + sat[D + std::size_t(c)]) / n; };
            const double mx = S(1), my = S(2), mz = S(3);
            const double cxx = S(4) - mx * mx, cyy = S(5) - my * my, cxy = S(6) - mx * my;
            const double cxz = S(7) - mx * mz, cyz = S(8) - my * mz, czz = S(9) - mz * mz;
            const double det = cxx * cyy - cxy * cxy;
            double v;
            if (det > 1e-6) {
                const double bx = (cxz * cyy - cyz * cxy) / det, by = (cyz * cxx - cxz * cxy) / det;
                v = czz - bx * cxz - by * cyz;
            } else {
                v = czz - (cxx > 1e-6 ? cxz * cxz / cxx : 0) - (cyy > 1e-6 ? cyz * cyz / cyy : 0);
            }
            out[j * w + i] = std::uint8_t(std::min(255.0, std::round(100 * std::sqrt(std::max(0.0, v)))));
        }
    }
}

int triangulate(const float *view, int w, int h, int s, std::uint16_t *out, double cell, bool jumps, int stride)
{
    const int W = (w - 1) / s + 1, Hh = (h - 1) / s + 1;
    const double tol = jumpTolerance(s * std::sqrt(2.0), cell);
    auto z = [&](int vi, int vj) { return view[std::size_t(vj * s * w + vi * s) * std::size_t(stride)]; };
    auto ok3 = [&](float x, float y, float q) { return !jumps || std::max({x, y, q}) - std::min({x, y, q}) <= tol; };
    int n = 0;
    for (int vj = 0; vj < Hh - 1; ++vj) {
        for (int vi = 0; vi < W - 1; ++vi) {
            const float za = z(vi, vj), zb = z(vi + 1, vj), zc = z(vi, vj + 1), zd = z(vi + 1, vj + 1);
            const bool ha = za > kEmpty, hb = zb > kEmpty, hc = zc > kEmpty, hd = zd > kEmpty;
            if (int(ha) + int(hb) + int(hc) + int(hd) < 3) continue;
            const int a = vj * W + vi, b = a + 1, c = a + W, d = c + 1;
            if (ha && hb && hd && ok3(za, zb, zd)) { out[n++] = std::uint16_t(a); out[n++] = std::uint16_t(b); out[n++] = std::uint16_t(d); }
            if (ha && hd && hc && ok3(za, zd, zc)) { out[n++] = std::uint16_t(a); out[n++] = std::uint16_t(d); out[n++] = std::uint16_t(c); }
            if (!hd && ha && hb && hc && ok3(za, zb, zc)) { out[n++] = std::uint16_t(a); out[n++] = std::uint16_t(b); out[n++] = std::uint16_t(c); }
            if (!ha && hb && hd && hc && ok3(zb, zd, zc)) { out[n++] = std::uint16_t(b); out[n++] = std::uint16_t(d); out[n++] = std::uint16_t(c); }
        }
    }
    return n;
}

} // namespace corv::raster
