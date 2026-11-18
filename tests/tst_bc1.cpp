// tst_bc1 - the BC1 encoder of the satellite chunk textures (terrain/Bc1.h).
//
// Encodes images, decodes them with a reference BC1 decoder written here from
// the format definition, and checks the error: flat colours exact to 565
// precision, gradients and two-colour blocks within a few levels, block
// layout and mip chain sizes as the GPU expects them.
#include "terrain/Bc1.h"

#include <QtTest>
#include <cmath>
#include <random>

using namespace corv;

namespace {

void expand565(uint16_t c, int rgb[3])
{
    rgb[0] = ((c >> 11) & 31) * 255 / 31;
    rgb[1] = ((c >> 5) & 63) * 255 / 63;
    rgb[2] = (c & 31) * 255 / 31;
}

/// Reference decoder: RGBA8 out, w x h.
std::vector<uint8_t> decode(const QByteArray &blocks, int w, int h)
{
    std::vector<uint8_t> out(std::size_t(w) * h * 4, 0);
    const auto *b = reinterpret_cast<const uint8_t *>(blocks.constData());
    const int bw = (w + 3) / 4, bh = (h + 3) / 4;
    for (int by = 0; by < bh; ++by)
        for (int bx = 0; bx < bw; ++bx, b += 8) {
            const uint16_t c0 = uint16_t(b[0] | (b[1] << 8)), c1 = uint16_t(b[2] | (b[3] << 8));
            const uint32_t idx = uint32_t(b[4]) | (uint32_t(b[5]) << 8) | (uint32_t(b[6]) << 16) | (uint32_t(b[7]) << 24);
            int pal[4][3];
            expand565(c0, pal[0]);
            expand565(c1, pal[1]);
            for (int k = 0; k < 3; ++k) {
                if (c0 > c1) {
                    pal[2][k] = (2 * pal[0][k] + pal[1][k]) / 3;
                    pal[3][k] = (pal[0][k] + 2 * pal[1][k]) / 3;
                } else {
                    pal[2][k] = (pal[0][k] + pal[1][k]) / 2;
                    pal[3][k] = 0;
                }
            }
            for (int i = 0; i < 16; ++i) {
                const int x = bx * 4 + i % 4, y = by * 4 + i / 4;
                if (x >= w || y >= h) continue;
                const int sel = int((idx >> (2 * i)) & 3);
                uint8_t *o = out.data() + (std::size_t(y) * w + x) * 4;
                o[0] = uint8_t(pal[sel][0]);
                o[1] = uint8_t(pal[sel][1]);
                o[2] = uint8_t(pal[sel][2]);
                o[3] = 255;
            }
        }
    return out;
}

double maxError(const std::vector<uint8_t> &a, const std::vector<uint8_t> &b)
{
    int m = 0;
    for (std::size_t i = 0; i < a.size(); i += 4)
        for (int k = 0; k < 3; ++k) m = std::max(m, std::abs(int(a[i + k]) - int(b[i + k])));
    return m;
}

double meanError(const std::vector<uint8_t> &a, const std::vector<uint8_t> &b)
{
    double s = 0;
    std::size_t n = 0;
    for (std::size_t i = 0; i < a.size(); i += 4, ++n)
        for (int k = 0; k < 3; ++k) s += std::abs(int(a[i + k]) - int(b[i + k]));
    return s / (3.0 * double(n));
}

} // namespace

class TestBc1 : public QObject
{
    Q_OBJECT
private slots:
    void sizes()
    {
        QCOMPARE(bc1::levelBytes(4, 4), qsizetype(8));
        QCOMPARE(bc1::levelBytes(1, 1), qsizetype(8));
        QCOMPARE(bc1::levelBytes(2048, 2048), qsizetype(2048 / 4 * 2048 / 4 * 8));
        QCOMPARE(bc1::levelBytes(6, 5), qsizetype(2 * 2 * 8));
    }

    void flatColourExact()
    {
        const int w = 8, h = 8;
        std::vector<uint8_t> img(std::size_t(w) * h * 4);
        for (std::size_t i = 0; i < img.size(); i += 4) {
            img[i] = 255; img[i + 1] = 0; img[i + 2] = 255; img[i + 3] = 255;   // exact in 565
        }
        const QByteArray c = bc1::compress(img.data(), w, h, w * 4);
        QCOMPARE(c.size(), bc1::levelBytes(w, h));
        QCOMPARE(maxError(img, decode(c, w, h)), 0.0);
    }

    void twoColourBlockKeepsBothColours()
    {
        // Half black, half white: both endpoints, every pixel on one of them
        const int w = 4, h = 4;
        std::vector<uint8_t> img(std::size_t(w) * h * 4);
        for (int i = 0; i < 16; ++i) {
            const uint8_t v = i % 4 < 2 ? 0 : 255;
            img[std::size_t(i) * 4] = img[std::size_t(i) * 4 + 1] = img[std::size_t(i) * 4 + 2] = v;
            img[std::size_t(i) * 4 + 3] = 255;
        }
        QCOMPARE(maxError(img, decode(bc1::compress(img.data(), w, h, w * 4), w, h)), 0.0);
    }

    void gradientWithinTolerance()
    {
        const int w = 64, h = 64;
        std::vector<uint8_t> img(std::size_t(w) * h * 4);
        for (int y = 0; y < h; ++y)
            for (int x = 0; x < w; ++x) {
                uint8_t *p = img.data() + (std::size_t(y) * w + x) * 4;
                p[0] = uint8_t(x * 4);
                p[1] = uint8_t(y * 4);
                p[2] = uint8_t(128 + (x - y));
                p[3] = 255;
            }
        const auto out = decode(bc1::compress(img.data(), w, h, w * 4), w, h);
        QVERIFY2(meanError(img, out) < 4.0, qPrintable(QString::number(meanError(img, out))));
        QVERIFY2(maxError(img, out) < 16.0, qPrintable(QString::number(maxError(img, out))));
    }

    void noisyImageReasonable()
    {
        // Satellite-like texture: random blobs over a base colour
        const int w = 128, h = 128;
        std::vector<uint8_t> img(std::size_t(w) * h * 4);
        std::mt19937 rng(7);
        std::uniform_int_distribution<int> d(-30, 30);
        for (std::size_t i = 0; i < img.size(); i += 4) {
            img[i] = uint8_t(std::clamp(90 + d(rng), 0, 255));
            img[i + 1] = uint8_t(std::clamp(110 + d(rng), 0, 255));
            img[i + 2] = uint8_t(std::clamp(70 + d(rng), 0, 255));
            img[i + 3] = 255;
        }
        const auto out = decode(bc1::compress(img.data(), w, h, w * 4), w, h);
        QVERIFY2(meanError(img, out) < 16.0, qPrintable(QString::number(meanError(img, out))));
    }

    void partialBlocksOnTheEdge()
    {
        const int w = 6, h = 3;
        std::vector<uint8_t> img(std::size_t(w) * h * 4, 200);
        const QByteArray c = bc1::compress(img.data(), w, h, w * 4);
        QCOMPARE(c.size(), qsizetype(2 * 1 * 8));
        QVERIFY(maxError(img, decode(c, w, h)) <= 4.0);
    }

    void mipChain()
    {
        const int s = 64;
        std::vector<uint8_t> img(std::size_t(s) * s * 4, 255);
        const auto levels = bc1::mipChain(img.data(), s, s, true);
        QCOMPARE(int(levels.size()), 7);   // 64, 32, 16, 8, 4, 2, 1
        int dim = s;
        for (const QByteArray &l : levels) {
            QCOMPARE(l.size(), bc1::levelBytes(dim, dim));
            dim = std::max(1, dim / 2);
        }
        const auto raw = bc1::mipChain(img.data(), s, s, false);
        QCOMPARE(raw.back().size(), qsizetype(4));
        QCOMPARE(raw[1].size(), qsizetype(32 * 32 * 4));
    }

    void halveAverages()
    {
        const uint8_t px[2 * 2 * 4] = {0, 0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0, 0, 255};
        int w = 0, h = 0;
        const auto out = bc1::halve(px, 2, 2, w, h);
        QCOMPARE(w, 1);
        QCOMPARE(h, 1);
        QCOMPARE(int(out[0]), 128);
    }
};

QTEST_GUILESS_MAIN(TestBc1)
#include "tst_bc1.moc"
