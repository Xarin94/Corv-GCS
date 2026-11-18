// Bc1.h - BC1 (DXT1) compression with a mip chain (port of js/terrain/TextureCompressWorker.js).
//
// Satellite chunk textures go to the GPU as BC1: 4 bits per pixel, an eighth
// of RGBA8, so the ~40 chunk textures inside a 10 km satellite radius stay
// around 25 MB of VRAM. Each 4x4 block is fitted along the principal axis of
// its colours (the extreme pixels on that axis become the two 565
// endpoints, as stb_dxt does without its refinement pass), and every pixel
// takes the nearest of the four palette colours.
// Pure functions; called on the thread pool.
#pragma once

#include <QByteArray>
#include <cstdint>
#include <vector>

namespace corv::bc1 {

/// Bytes of one level: 8 per 4x4 block, partial blocks rounded up.
inline qsizetype levelBytes(int w, int h) { return qsizetype((w + 3) / 4) * ((h + 3) / 4) * 8; }

/// Compress one RGBA8 image (R, G, B, A byte order, `stride` bytes per row). Alpha is ignored.
QByteArray compress(const uint8_t *rgba, int w, int h, int stride);

/// Halve an RGBA8 image with a 2x2 box filter (odd sizes keep the last row / column).
std::vector<uint8_t> halve(const uint8_t *rgba, int w, int h, int &outW, int &outH);

/// Every level from w x h down to 1 x 1, each either BC1 (`compressed`) or raw RGBA8.
std::vector<QByteArray> mipChain(const uint8_t *rgba, int w, int h, bool compressed);

} // namespace corv::bc1
