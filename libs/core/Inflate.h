// Inflate.h - gzip / raw deflate decoding (RFC 1951/1952) for the .hgt.gz
// elevation tiles. Qt's own zlib is private; qUncompress only reads its zlib
// format. A small canonical-Huffman decoder after Mark Adler's puff.c.
#pragma once

#include <QByteArray>

namespace corv {

/// Decompress a gzip member; empty on a malformed stream. `expectedSize` reserves the output.
QByteArray gunzip(const QByteArray &gz, qsizetype expectedSize = 0);
/// Decompress a raw deflate stream starting at `offset`; false on error.
bool inflateRaw(const QByteArray &in, qsizetype offset, QByteArray &out);

} // namespace corv
