#include "core/Inflate.h"

#include <cstdint>
#include <cstring>

namespace corv {

namespace {

constexpr int kMaxBits = 15;
constexpr int kMaxLCodes = 286;
constexpr int kMaxDCodes = 30;
constexpr int kFixLCodes = 288;

struct State {
    const uint8_t *in = nullptr;
    qsizetype inLen = 0, inPos = 0;
    uint32_t bitBuf = 0;
    int bitCnt = 0;
    QByteArray *out = nullptr;
    bool error = false;

    int bits(int need)
    {
        uint32_t val = bitBuf;
        while (bitCnt < need) {
            if (inPos >= inLen) { error = true; return 0; }
            val |= uint32_t(in[inPos++]) << bitCnt;
            bitCnt += 8;
        }
        bitBuf = val >> need;
        bitCnt -= need;
        return int(val & ((1u << need) - 1));
    }
};

struct Huffman {
    short count[kMaxBits + 1];
    short symbol[kFixLCodes];
};

int decode(State &s, const Huffman &h)
{
    int code = 0, first = 0, index = 0;
    for (int len = 1; len <= kMaxBits; ++len) {
        code |= s.bits(1);
        if (s.error) return -1;
        const int count = h.count[len];
        if (code - count < first) return h.symbol[index + (code - first)];
        index += count;
        first += count;
        first <<= 1;
        code <<= 1;
    }
    return -1;
}

int construct(Huffman &h, const short *length, int n)
{
    std::memset(h.count, 0, sizeof(h.count));
    for (int symbol = 0; symbol < n; ++symbol) h.count[length[symbol]]++;
    if (h.count[0] == n) return 0;
    int left = 1;
    for (int len = 1; len <= kMaxBits; ++len) {
        left <<= 1;
        left -= h.count[len];
        if (left < 0) return left;
    }
    short offs[kMaxBits + 1];
    offs[1] = 0;
    for (int len = 1; len < kMaxBits; ++len) offs[len + 1] = short(offs[len] + h.count[len]);
    for (int symbol = 0; symbol < n; ++symbol)
        if (length[symbol] != 0) h.symbol[offs[length[symbol]]++] = short(symbol);
    return left;
}

bool codes(State &s, const Huffman &lencode, const Huffman &distcode)
{
    static const short lbase[29] = {3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31, 35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258};
    static const short lext[29] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0};
    static const short dbase[30] = {1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193, 257, 385, 513, 769,
                                    1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577};
    static const short dext[30] = {0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13};
    for (;;) {
        int symbol = decode(s, lencode);
        if (symbol < 0) return false;
        if (symbol < 256) {
            s.out->append(char(symbol));
        } else if (symbol == 256) {
            return true;
        } else {
            symbol -= 257;
            if (symbol >= 29) return false;
            const int len = lbase[symbol] + s.bits(lext[symbol]);
            const int dsym = decode(s, distcode);
            if (dsym < 0 || dsym >= 30) return false;
            const qsizetype dist = dbase[dsym] + s.bits(dext[dsym]);
            if (s.error || dist > s.out->size()) return false;
            const qsizetype from = s.out->size() - dist;
            for (int i = 0; i < len; ++i) s.out->append(s.out->at(from + i));   // may overlap: byte by byte
        }
    }
}

bool stored(State &s)
{
    s.bitBuf = 0;
    s.bitCnt = 0;
    if (s.inPos + 4 > s.inLen) return false;
    const unsigned len = s.in[s.inPos] | (unsigned(s.in[s.inPos + 1]) << 8);
    const unsigned nlen = s.in[s.inPos + 2] | (unsigned(s.in[s.inPos + 3]) << 8);
    s.inPos += 4;
    if (len != (~nlen & 0xffff)) return false;
    if (s.inPos + len > s.inLen) return false;
    s.out->append(reinterpret_cast<const char *>(s.in + s.inPos), len);
    s.inPos += len;
    return true;
}

struct FixedTables {
    Huffman lencode, distcode;
    FixedTables()
    {
        short lengths[kFixLCodes];
        int symbol = 0;
        for (; symbol < 144; ++symbol) lengths[symbol] = 8;
        for (; symbol < 256; ++symbol) lengths[symbol] = 9;
        for (; symbol < 280; ++symbol) lengths[symbol] = 7;
        for (; symbol < kFixLCodes; ++symbol) lengths[symbol] = 8;
        construct(lencode, lengths, kFixLCodes);
        for (symbol = 0; symbol < kMaxDCodes; ++symbol) lengths[symbol] = 5;
        construct(distcode, lengths, kMaxDCodes);
    }
};

bool fixed(State &s)
{
    static const FixedTables t;   // thread-safe initialisation: downloads decode on the pool
    return codes(s, t.lencode, t.distcode);
}

bool dynamic(State &s)
{
    static const short order[19] = {16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15};
    short lengths[kMaxLCodes + kMaxDCodes];
    const int nlen = s.bits(5) + 257;
    const int ndist = s.bits(5) + 1;
    const int ncode = s.bits(4) + 4;
    if (s.error || nlen > kMaxLCodes || ndist > kMaxDCodes) return false;
    int index = 0;
    for (; index < ncode; ++index) lengths[order[index]] = short(s.bits(3));
    for (; index < 19; ++index) lengths[order[index]] = 0;
    Huffman lencode, distcode;
    if (construct(lencode, lengths, 19) != 0) return false;
    index = 0;
    while (index < nlen + ndist) {
        int symbol = decode(s, lencode);
        if (symbol < 0) return false;
        if (symbol < 16) {
            lengths[index++] = short(symbol);
        } else {
            int len = 0;
            if (symbol == 16) {
                if (index == 0) return false;
                len = lengths[index - 1];
                symbol = 3 + s.bits(2);
            } else if (symbol == 17) {
                symbol = 3 + s.bits(3);
            } else {
                symbol = 11 + s.bits(7);
            }
            if (index + symbol > nlen + ndist) return false;
            while (symbol--) lengths[index++] = short(len);
        }
    }
    if (lengths[256] == 0) return false;
    const int err = construct(lencode, lengths, nlen);
    if (err < 0 || (err > 0 && nlen - lencode.count[0] != 1)) return false;
    const int derr = construct(distcode, lengths + nlen, ndist);
    if (derr < 0 || (derr > 0 && ndist - distcode.count[0] != 1)) return false;
    return codes(s, lencode, distcode);
}

} // namespace

bool inflateRaw(const QByteArray &in, qsizetype offset, QByteArray &out)
{
    State s;
    s.in = reinterpret_cast<const uint8_t *>(in.constData());
    s.inLen = in.size();
    s.inPos = offset;
    s.out = &out;
    int last = 0;
    do {
        last = s.bits(1);
        const int type = s.bits(2);
        if (s.error) return false;
        bool ok = false;
        if (type == 0) ok = stored(s);
        else if (type == 1) ok = fixed(s);
        else if (type == 2) ok = dynamic(s);
        if (!ok || s.error) return false;
    } while (!last);
    return true;
}

QByteArray gunzip(const QByteArray &gz, qsizetype expectedSize)
{
    const auto *p = reinterpret_cast<const uint8_t *>(gz.constData());
    if (gz.size() < 18 || p[0] != 0x1f || p[1] != 0x8b || p[2] != 8) return {};
    const uint8_t flags = p[3];
    qsizetype pos = 10;
    if (flags & 0x04) {   // FEXTRA
        if (pos + 2 > gz.size()) return {};
        pos += 2 + (p[pos] | (p[pos + 1] << 8));
    }
    if (flags & 0x08) { while (pos < gz.size() && p[pos]) ++pos; ++pos; }   // FNAME
    if (flags & 0x10) { while (pos < gz.size() && p[pos]) ++pos; ++pos; }   // FCOMMENT
    if (flags & 0x02) pos += 2;                                               // FHCRC
    if (pos >= gz.size()) return {};
    QByteArray out;
    // The ISIZE trailer gives the size (mod 2^32), good enough to reserve
    const qsizetype isize = qsizetype(p[gz.size() - 4]) | (qsizetype(p[gz.size() - 3]) << 8) | (qsizetype(p[gz.size() - 2]) << 16)
                            | (qsizetype(p[gz.size() - 1]) << 24);
    out.reserve(expectedSize > 0 ? expectedSize : isize);
    if (!inflateRaw(gz, pos, out)) return {};
    return out;
}

} // namespace corv
