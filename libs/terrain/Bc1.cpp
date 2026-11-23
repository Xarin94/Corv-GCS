#include "terrain/Bc1.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace corv::bc1 {

namespace {

inline uint16_t to565(int r, int g, int b)
{
    return uint16_t(((r * 31 + 127) / 255) << 11 | ((g * 63 + 127) / 255) << 5 | ((b * 31 + 127) / 255));
}

inline void from565(uint16_t c, int &r, int &g, int &b)
{
    r = (c >> 11) & 31;
    g = (c >> 5) & 63;
    b = c & 31;
    r = (r << 3) | (r >> 2);
    g = (g << 2) | (g >> 4);
    b = (b << 3) | (b >> 2);
}

/// One 4x4 block: px holds 16 RGB triples.
void encodeBlock(const uint8_t px[16][3], uint8_t out[8])
{
    // Mean and covariance of the colours
    float mu[3] = {0, 0, 0};
    for (int i = 0; i < 16; ++i)
        for (int c = 0; c < 3; ++c) mu[c] += px[i][c];
    for (float &m : mu) m /= 16.f;
    float cov[6] = {0, 0, 0, 0, 0, 0};
    int lo[3] = {255, 255, 255}, hi[3] = {0, 0, 0};
    for (int i = 0; i < 16; ++i) {
        const float r = px[i][0] - mu[0], g = px[i][1] - mu[1], b = px[i][2] - mu[2];
        cov[0] += r * r; cov[1] += r * g; cov[2] += r * b;
        cov[3] += g * g; cov[4] += g * b; cov[5] += b * b;
        for (int c = 0; c < 3; ++c) {
            lo[c] = std::min<int>(lo[c], px[i][c]);
            hi[c] = std::max<int>(hi[c], px[i][c]);
        }
    }
    // Principal axis by power iteration, from the bounding box diagonal
    float v[3] = {float(hi[0] - lo[0]), float(hi[1] - lo[1]), float(hi[2] - lo[2])};
    for (int it = 0; it < 4; ++it) {
        const float x = v[0] * cov[0] + v[1] * cov[1] + v[2] * cov[2];
        const float y = v[0] * cov[1] + v[1] * cov[3] + v[2] * cov[4];
        const float z = v[0] * cov[2] + v[1] * cov[4] + v[2] * cov[5];
        v[0] = x; v[1] = y; v[2] = z;
    }
    const float mag = std::max({std::abs(v[0]), std::abs(v[1]), std::abs(v[2])});
    if (mag < 4.f) {   // flat block: luminance axis
        v[0] = 0.299f; v[1] = 0.587f; v[2] = 0.114f;
    }
    // Extreme pixels along the axis are the endpoints
    int iMin = 0, iMax = 0;
    float dMin = 1e30f, dMax = -1e30f;
    for (int i = 0; i < 16; ++i) {
        const float d = px[i][0] * v[0] + px[i][1] * v[1] + px[i][2] * v[2];
        if (d < dMin) { dMin = d; iMin = i; }
        if (d > dMax) { dMax = d; iMax = i; }
    }
    uint16_t c0 = to565(px[iMax][0], px[iMax][1], px[iMax][2]);
    uint16_t c1 = to565(px[iMin][0], px[iMin][1], px[iMin][2]);
    uint32_t indices = 0;
    if (c0 != c1) {
        if (c0 < c1) std::swap(c0, c1);   // c0 > c1: four-colour mode
        int pal[4][3];
        from565(c0, pal[0][0], pal[0][1], pal[0][2]);
        from565(c1, pal[1][0], pal[1][1], pal[1][2]);
        for (int c = 0; c < 3; ++c) {
            pal[2][c] = (2 * pal[0][c] + pal[1][c]) / 3;
            pal[3][c] = (pal[0][c] + 2 * pal[1][c]) / 3;
        }
        for (int i = 15; i >= 0; --i) {
            int best = 0, bestD = 1 << 30;
            for (int k = 0; k < 4; ++k) {
                const int dr = px[i][0] - pal[k][0], dg = px[i][1] - pal[k][1], db = px[i][2] - pal[k][2];
                const int d = dr * dr * 3 + dg * dg * 4 + db * db * 2;
                if (d < bestD) { bestD = d; best = k; }
            }
            indices = (indices << 2) | uint32_t(best);
        }
    }
    out[0] = uint8_t(c0 & 0xff); out[1] = uint8_t(c0 >> 8);
    out[2] = uint8_t(c1 & 0xff); out[3] = uint8_t(c1 >> 8);
    out[4] = uint8_t(indices & 0xff); out[5] = uint8_t((indices >> 8) & 0xff);
    out[6] = uint8_t((indices >> 16) & 0xff); out[7] = uint8_t(indices >> 24);
}

} // namespace

QByteArray compress(const uint8_t *rgba, int w, int h, int stride)
{
    const int bw = (w + 3) / 4, bh = (h + 3) / 4;
    QByteArray out(levelBytes(w, h), Qt::Uninitialized);
    auto *dst = reinterpret_cast<uint8_t *>(out.data());
    uint8_t px[16][3];
    for (int by = 0; by < bh; ++by) {
        for (int bx = 0; bx < bw; ++bx) {
            // Partial blocks on the edge repeat the last row / column
            for (int y = 0; y < 4; ++y) {
                const uint8_t *row = rgba + std::size_t(std::min(by * 4 + y, h - 1)) * stride;
                for (int x = 0; x < 4; ++x) {
                    const uint8_t *p = row + std::size_t(std::min(bx * 4 + x, w - 1)) * 4;
                    px[y * 4 + x][0] = p[0];
                    px[y * 4 + x][1] = p[1];
                    px[y * 4 + x][2] = p[2];
                }
            }
            encodeBlock(px, dst);
            dst += 8;
        }
    }
    return out;
}

std::vector<uint8_t> halve(const uint8_t *rgba, int w, int h, int &outW, int &outH)
{
    outW = std::max(1, w / 2);
    outH = std::max(1, h / 2);
    std::vector<uint8_t> out(std::size_t(outW) * outH * 4);
    for (int y = 0; y < outH; ++y) {
        const int y0 = std::min(2 * y, h - 1), y1 = std::min(2 * y + 1, h - 1);
        for (int x = 0; x < outW; ++x) {
            const int x0 = std::min(2 * x, w - 1), x1 = std::min(2 * x + 1, w - 1);
            const uint8_t *a = rgba + (std::size_t(y0) * w + x0) * 4, *b = rgba + (std::size_t(y0) * w + x1) * 4;
            const uint8_t *c = rgba + (std::size_t(y1) * w + x0) * 4, *d = rgba + (std::size_t(y1) * w + x1) * 4;
            uint8_t *o = out.data() + (std::size_t(y) * outW + x) * 4;
            for (int k = 0; k < 4; ++k) o[k] = uint8_t((a[k] + b[k] + c[k] + d[k] + 2) / 4);
        }
    }
    return out;
}

std::vector<QByteArray> mipChain(const uint8_t *rgba, int w, int h, bool compressed)
{
    std::vector<QByteArray> levels;
    std::vector<uint8_t> cur;   // the halved levels
    for (const uint8_t *px = rgba;;) {
        if (compressed) levels.push_back(compress(px, w, h, w * 4));
        else levels.emplace_back(reinterpret_cast<const char *>(px), qsizetype(w) * h * 4);
        if (w == 1 && h == 1) break;
        int nw = 0, nh = 0;
        cur = halve(px, w, h, nw, nh);
        px = cur.data();
        w = nw;
        h = nh;
    }
    return levels;
}

} // namespace corv::bc1
