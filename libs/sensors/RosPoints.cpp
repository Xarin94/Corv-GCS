#include "sensors/RosPoints.h"

#include "core/Geo.h"

#include <QCborArray>
#include <QRandomGenerator>
#include <QRegularExpression>
#include <QtEndian>
#include <algorithm>
#include <cmath>
#include <vector>

using namespace Qt::StringLiterals;

namespace corv::rospts {

namespace {
constexpr double kMPerDeg = 111320;
constexpr double kProjectStepS = 0.02;

// REP-105 world-fixed frames
const QRegularExpression kWorldFrame(u"(^|/)(map|odom|world|earth|local_origin)(_ned|_enu)?$"_s, QRegularExpression::CaseInsensitiveOption);
const QRegularExpression kNedFrame(u"_ned$"_s, QRegularExpression::CaseInsensitiveOption);

QCborValue field(const QCborMap &m, const char *name) { return m.value(QLatin1String(name)); }

// uint8[] as it may arrive: CBOR byte string (or RFC 8746 uint8 typed array), base64 in JSON, an array of numbers
QByteArray bytesOf(const QCborValue &v)
{
    if (v.isByteArray()) return v.toByteArray();
    if (v.isTag() && v.taggedValue().isByteArray()) return v.taggedValue().toByteArray();
    if (v.isString()) return QByteArray::fromBase64(v.toString().toLatin1());
    if (v.isArray()) {
        const QCborArray a = v.toArray();
        QByteArray out(a.size(), '\0');
        for (qsizetype i = 0; i < a.size(); ++i) out[i] = char(int(num(a.at(i))) & 0xFF);
        return out;
    }
    return {};
}

// float32[] as it may arrive: JSON array (null = non-finite), or an RFC 8746
// typed array (tag 85 little-endian, 81 big-endian float32; 86 / 82 float64)
std::vector<double> floatsOf(const QCborValue &v, bool *ok)
{
    std::vector<double> out;
    *ok = true;
    if (v.isArray()) {
        const QCborArray a = v.toArray();
        out.reserve(std::size_t(a.size()));
        for (const QCborValue &x : a) out.push_back(x.isInteger() || x.isDouble() ? num(x) : std::numeric_limits<double>::quiet_NaN());
        return out;
    }
    if (v.isTag() && v.taggedValue().isByteArray()) {
        const QByteArray b = v.taggedValue().toByteArray();
        const auto tag = quint64(v.tag());
        const auto *p = reinterpret_cast<const uchar *>(b.constData());
        if (tag == 85 || tag == 81) {
            const qsizetype n = b.size() / 4;
            out.resize(std::size_t(n));
            for (qsizetype i = 0; i < n; ++i) out[std::size_t(i)] = tag == 85 ? qFromLittleEndian<float>(p + i * 4) : qFromBigEndian<float>(p + i * 4);
            return out;
        }
        if (tag == 86 || tag == 82) {
            const qsizetype n = b.size() / 8;
            out.resize(std::size_t(n));
            for (qsizetype i = 0; i < n; ++i) out[std::size_t(i)] = tag == 86 ? qFromLittleEndian<double>(p + i * 8) : qFromBigEndian<double>(p + i * 8);
            return out;
        }
    }
    *ok = false;
    return out;
}

// Indices of a uniform sample of `max` out of `total`, from a random start
struct Stride {
    double step, start;
    int n;
};
Stride stride(int total, int max)
{
    if (total <= max) return {1, 0, total};
    const double step = double(total) / max;
    return {step, QRandomGenerator::global()->generateDouble() * step, max};
}

double readField(const uchar *p, int datatype, bool le)
{
    switch (datatype) {
    case 1: return double(qint8(p[0]));
    case 2: return double(p[0]);
    case 3: return le ? qFromLittleEndian<qint16>(p) : qFromBigEndian<qint16>(p);
    case 4: return le ? qFromLittleEndian<quint16>(p) : qFromBigEndian<quint16>(p);
    case 5: return le ? qFromLittleEndian<qint32>(p) : qFromBigEndian<qint32>(p);
    case 6: return le ? qFromLittleEndian<quint32>(p) : qFromBigEndian<quint32>(p);
    case 7: return le ? qFromLittleEndian<float>(p) : qFromBigEndian<float>(p);
    case 8: return le ? qFromLittleEndian<double>(p) : qFromBigEndian<double>(p);
    }
    return std::numeric_limits<double>::quiet_NaN();
}

int fieldSize(int datatype) { return datatype <= 2 ? 1 : datatype <= 4 ? 2 : datatype <= 7 ? 4 : 8; }

double medianOf(std::vector<float> &a, int n)
{
    for (int i = 1; i < n; ++i) {
        const float v = a[std::size_t(i)];
        int j = i - 1;
        while (j >= 0 && a[std::size_t(j)] > v) {
            a[std::size_t(j + 1)] = a[std::size_t(j)];
            --j;
        }
        a[std::size_t(j + 1)] = v;
    }
    return n & 1 ? a[std::size_t(n >> 1)] : 0.5 * (a[std::size_t((n >> 1) - 1)] + a[std::size_t(n >> 1)]);
}
} // namespace

double num(const QCborValue &v)
{
    if (v.isInteger()) return double(v.toInteger());
    if (v.isDouble()) return v.toDouble();
    if (v.isFalse()) return 0;
    if (v.isTrue()) return 1;
    return std::numeric_limits<double>::quiet_NaN();
}

QString kindOf(const QString &type)
{
    QString t = type;
    t.replace("/msg/"_L1, "/"_L1);
    if (t == "sensor_msgs/PointCloud2"_L1) return u"cloud"_s;
    if (t == "sensor_msgs/LaserScan"_L1) return u"scan"_s;
    if (t == "sensor_msgs/Range"_L1) return u"range"_s;
    return {};
}

QString frameOf(const QString &frameId, const QString &forced)
{
    const bool ned = kNedFrame.match(frameId).hasMatch();
    if (forced == "world"_L1) return ned ? u"world-ned"_s : u"world-enu"_s;
    if (forced == "sensor"_L1) return u"sensor"_s;
    return kWorldFrame.match(frameId).hasMatch() ? (ned ? u"world-ned"_s : u"world-enu"_s) : u"sensor"_s;
}

Decoded decodePoints(const QString &kind, const QCborMap &msg, float *out, int outCap, int max, double rMin, double rMax, float *times)
{
    const int cap = std::min(max, outCap);
    const double r2min = rMin * rMin, r2max = rMax * rMax;
    Decoded d;
    auto keep = [&](double x, double y, double z, double t) {
        if (d.count >= cap) return;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return;
        const double r2 = x * x + y * y + z * z;
        if (r2 == 0 || r2 < r2min || r2 > r2max) return;
        out[d.count * 3] = float(x);
        out[d.count * 3 + 1] = float(y);
        out[d.count * 3 + 2] = float(z);
        if (times) times[d.count] = float(t);
        ++d.count;
    };

    if (kind == "range"_L1) {
        const double r = num(field(msg, "range"));
        const double lo = num(field(msg, "min_range")), hi = num(field(msg, "max_range"));
        if (std::isfinite(r) && r >= (lo > 0 ? lo : 0) && r <= (hi > 0 ? hi : std::numeric_limits<double>::infinity())) keep(r, 0, 0, 0);
        d.total = 1;
        return d;
    }

    if (kind == "scan"_L1) {
        bool ok = false;
        const std::vector<double> ranges = floatsOf(field(msg, "ranges"), &ok);
        if (!ok) return d;
        const int total = int(ranges.size());
        const double rmin = num(field(msg, "range_min")), rmax = num(field(msg, "range_max"));
        const double lo = std::max(rmin > 0 ? rmin : 0, rMin), hi = std::min(rmax > 0 ? rmax : std::numeric_limits<double>::infinity(), rMax);
        double dt = std::abs(num(field(msg, "time_increment")));
        if (!(dt > 0)) dt = 0;
        const double a0 = num(field(msg, "angle_min")), da = num(field(msg, "angle_increment"));
        const Stride s = stride(total, cap);
        for (int k = 0; k < s.n; ++k) {
            const int idx = int(std::floor(s.start + k * s.step));
            if (idx >= total) break;
            const double r = ranges[std::size_t(idx)];
            if (!(r >= lo && r <= hi)) continue;
            const double a = a0 + idx * da;
            keep(r * std::cos(a), r * std::sin(a), 0, idx * dt);
        }
        d.total = total;
        d.span = std::isfinite(dt) ? dt * std::max(0, total - 1) : 0;
        return d;
    }

    if (kind == "cloud"_L1) {
        const QByteArray bytes = bytesOf(field(msg, "data"));
        const double width = num(field(msg, "width")), height = num(field(msg, "height"));
        const int total = std::isfinite(width) && std::isfinite(height) ? int(width * height) : 0;
        const int step = int(num(field(msg, "point_step")));
        d.total = total;
        if (bytes.isEmpty() || total <= 0 || step <= 0) return d;
        struct F {
            int offset = -1, datatype = 0;
        } fx, fy, fz;
        for (const QCborValue &fv : field(msg, "fields").toArray()) {
            const QCborMap f = fv.toMap();
            const QString name = f.value("name"_L1).toString();
            F *dst = name == "x"_L1 ? &fx : name == "y"_L1 ? &fy : name == "z"_L1 ? &fz : nullptr;
            if (!dst) continue;
            dst->offset = int(num(f.value("offset"_L1)));
            dst->datatype = int(num(f.value("datatype"_L1)));
        }
        auto valid = [](const F &f) { return f.offset >= 0 && f.datatype >= 1 && f.datatype <= 8; };
        if (!valid(fx) || !valid(fy) || !valid(fz)) return d;
        const QCborValue be = field(msg, "is_bigendian");
        const bool le = !(be.isTrue() || num(be) == 1);
        const int w = int(width);
        const double rs = num(field(msg, "row_step"));
        const qint64 rowStep = rs > 0 ? qint64(rs) : qint64(w) * step;
        const qint64 last = bytes.size() - step;
        const auto *b = reinterpret_cast<const uchar *>(bytes.constData());
        const int need = std::max({fx.offset + fieldSize(fx.datatype), fy.offset + fieldSize(fy.datatype), fz.offset + fieldSize(fz.datatype)});
        if (need > step) return d;
        const Stride s = stride(total, cap);
        for (int k = 0; k < s.n; ++k) {
            const qint64 idx = qint64(std::floor(s.start + k * s.step));
            const qint64 o = (idx / w) * rowStep + (idx % w) * step;
            if (o > last) break;
            keep(readField(b + o + fx.offset, fx.datatype, le), readField(b + o + fy.offset, fy.datatype, le), readField(b + o + fz.offset, fz.datatype, le), 0);
        }
        return d;
    }
    return d;
}

Despiked despike(float *pts, int count, int half, double absM, double rel, float *times)
{
    const int win = 2 * half + 1;
    if (count < win) return {count, 0, false};
    thread_local std::vector<float> r, w;
    thread_local std::vector<std::uint8_t> keep;
    if (int(r.size()) < count) {
        r.resize(std::size_t(count));
        keep.resize(std::size_t(count));
    }
    if (int(w.size()) < std::max(win, count)) w.resize(std::size_t(std::max(win, count)));
    for (int k = 0; k < count; ++k) {
        const float x = pts[k * 3], y = pts[k * 3 + 1], z = pts[k * 3 + 2];
        r[std::size_t(k)] = std::sqrt(x * x + y * y + z * z);
    }
    // Ordered? median step vs the 10-90 % spread (on a subsample: sorting is O(n^2) here)
    const int sub = std::max(1, count / 256);
    int n = 0;
    for (int k = sub; k < count; k += sub) w[std::size_t(n++)] = std::abs(r[std::size_t(k)] - r[std::size_t(k - sub)]);
    const double step = medianOf(w, n);
    n = 0;
    for (int k = 0; k < count; k += sub) w[std::size_t(n++)] = r[std::size_t(k)];
    medianOf(w, n);
    const double spread = w[std::size_t(std::floor(0.9 * (n - 1)))] - w[std::size_t(std::floor(0.1 * (n - 1)))];
    if (step > std::max(0.1 * spread, absM / 4)) return {count, 0, false};

    for (int k = 0; k < count; ++k) {
        const int a = std::max(0, std::min(count - win, k - half));
        for (int i = 0; i < win; ++i) w[std::size_t(i)] = r[std::size_t(a + i)];
        const double m = medianOf(w, win);
        keep[std::size_t(k)] = std::abs(r[std::size_t(k)] - m) <= std::max(absM, rel * m) ? 1 : 0;
    }
    int outN = 0;
    for (int k = 0; k < count; ++k) {
        if (!keep[std::size_t(k)]) continue;
        if (outN != k) {
            pts[outN * 3] = pts[k * 3];
            pts[outN * 3 + 1] = pts[k * 3 + 1];
            pts[outN * 3 + 2] = pts[k * 3 + 2];
            if (times) times[outN] = times[k];
        }
        ++outN;
    }
    return {outN, count - outN, true};
}

Pose projectPose(const AttSample &a, const PosSample &q, double t, double k)
{
    double roll = a.roll, pitch = a.pitch, yaw = a.yaw;
    const double dt = k * (t - a.t) / 1000;
    const int n = std::max(1, int(std::ceil(std::abs(dt) / kProjectStepS)));
    const double h = dt / n;
    for (int i = 0; i < n; ++i) {
        const double sr = std::sin(roll), cr = std::cos(roll), qr = a.q * sr + a.r * cr;
        roll += h * (a.p + qr * std::tan(pitch));
        pitch += h * (a.q * cr - a.r * sr);
        yaw += h * qr / std::max(1e-3, std::cos(pitch));
    }
    const double dp = k * (t - q.t) / 1000;
    return {q.lat + q.vn * dp / kMPerDeg, q.lon + q.ve * dp / (kMPerDeg * std::cos(q.lat * kPi / 180)), q.alt - q.vd * dp, roll, pitch, yaw};
}

std::array<double, 9> eulerToMatrix(double roll, double pitch, double yaw)
{
    const double cr = std::cos(roll), sr = std::sin(roll);
    const double cp = std::cos(pitch), sp = std::sin(pitch);
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    return {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, -sp, cp * sr, cp * cr};
}

Anchor makeAnchor(double lat, double lon, double alt) { return {lat, lon, alt, kMPerDeg, kMPerDeg * std::cos(lat * kPi / 180)}; }

std::array<double, 3> toEnu(const Anchor &a, double lat, double lon, double alt)
{
    return {(lon - a.lon) * a.mPerLon, (lat - a.lat) * a.mPerLat, alt - a.alt};
}

void sensorToEnu(const float *src, int count, const std::array<double, 9> &m, const std::array<double, 3> &lever, const std::array<double, 9> &a,
                 const std::array<double, 3> &veh, double *out)
{
    const double lx = lever[0], ly = lever[1], lz = lever[2];
    for (int k = 0; k < count; ++k) {
        const double fx = src[k * 3], fy = -double(src[k * 3 + 1]), fz = -double(src[k * 3 + 2]);
        const double bx = m[0] * fx + m[1] * fy + m[2] * fz + lx;
        const double by = m[3] * fx + m[4] * fy + m[5] * fz + ly;
        const double bz = m[6] * fx + m[7] * fy + m[8] * fz + lz;
        const double pn = a[0] * bx + a[1] * by + a[2] * bz;
        const double pe = a[3] * bx + a[4] * by + a[5] * bz;
        const double pd = a[6] * bx + a[7] * by + a[8] * bz;
        out[k * 3] = veh[0] + pe;
        out[k * 3 + 1] = veh[1] + pn;
        out[k * 3 + 2] = veh[2] - pd;
    }
}

void worldToEnu(const float *src, int count, bool ned, const std::array<double, 3> &org, double *out)
{
    for (int k = 0; k < count; ++k) {
        const double x = src[k * 3], y = src[k * 3 + 1], z = src[k * 3 + 2];
        out[k * 3] = org[0] + (ned ? y : x);
        out[k * 3 + 1] = org[1] + (ned ? x : y);
        out[k * 3 + 2] = org[2] + (ned ? -z : z);
    }
}

} // namespace corv::rospts
