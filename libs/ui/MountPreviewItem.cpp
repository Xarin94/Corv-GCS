#include "ui/MountPreviewItem.h"

#include <QPainter>
#include <QPainterPath>
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <vector>

using namespace Qt::StringLiterals;

namespace corv {

namespace {
using V3 = std::array<double, 3>;
constexpr double kDeg = 3.14159265358979323846 / 180;
constexpr double kPi = 3.14159265358979323846;
constexpr double kZoneR = 0.8;    // m, schematic
constexpr double kAxisL = 0.6;
constexpr double kExtent = 1.2;   // m from the IMU to the canvas edge (the shorter side)
const QColor kColX(0xff, 0x5a, 0x4f), kColY(0x3d, 0xdc, 0x84), kColZ(0x4d, 0x8d, 0xff);

V3 norm(const V3 &a)
{
    const double l = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    return {a[0] / l, a[1] / l, a[2] / l};
}
V3 cross(const V3 &a, const V3 &b) { return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]}; }
double dot(const V3 &a, const V3 &b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

// Orthographic camera from behind-right-above, in the view frame (x fwd, y left, z up)
struct Cam {
    V3 v, right, up;
};
const Cam &cam()
{
    static const Cam c = [] {
        const V3 toCam = norm({-1.0, -0.85, 1.0});   // towards the camera: ~37 deg above, near isometric
        Cam k;
        k.v = {-toCam[0], -toCam[1], -toCam[2]};
        k.right = norm(cross(k.v, {0, 0, 1}));
        k.up = cross(k.right, k.v);
        return k;
    }();
    return c;
}

std::array<double, 9> eulerToMatrix(double roll, double pitch, double yaw)
{
    const double cr = std::cos(roll), sr = std::sin(roll), cp = std::cos(pitch), sp = std::sin(pitch), cy = std::cos(yaw), sy = std::sin(yaw);
    return {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, -sp, cp * sr, cp * cr};
}

// A sensor-frame direction (FLU) as the view frame sees it, through the mount:
// FLU -> FRD (x, -y, -z), R_mount, then body FRD -> view (x, -y, -z)
V3 sensorToView(const std::array<double, 9> &R, double x, double y, double z)
{
    const double fx = x, fy = -y, fz = -z;
    const double bx = R[0] * fx + R[1] * fy + R[2] * fz;
    const double by = R[3] * fx + R[4] * fy + R[5] * fz;
    const double bz = R[6] * fx + R[7] * fy + R[8] * fz;
    return {bx, -by, -bz};
}

double num(const QVariant &v, double def = 0)
{
    bool ok = false;
    const double d = v.toDouble(&ok);
    return ok && std::isfinite(d) ? d : def;
}
} // namespace

MountPreviewItem::MountPreviewItem(QQuickItem *parent) : QQuickPaintedItem(parent)
{
    setAntialiasing(true);
}

void MountPreviewItem::paint(QPainter *g)
{
    const double W = width(), H = height();
    if (W <= 0 || H <= 0) return;
    g->setRenderHint(QPainter::Antialiasing);
    const Cam &C = cam();
    const double scale = std::min(W, H) / (2 * kExtent);
    const double cx = W * 0.5, cy = H * 0.52;
    auto P = [&](const V3 &p) { return QPointF(cx + dot(p, C.right) * scale, cy - dot(p, C.up) * scale); };
    auto depth = [&](const V3 &p) { return dot(p, C.v); };
    auto poly = [&](const std::vector<V3> &pts, const QColor &fill, const QColor &stroke, double width) {
        QPainterPath path;
        for (std::size_t i = 0; i < pts.size(); ++i) {
            if (i) path.lineTo(P(pts[i]));
            else path.moveTo(P(pts[i]));
        }
        path.closeSubpath();
        if (fill.isValid()) g->fillPath(path, fill);
        if (stroke.isValid()) g->strokePath(path, QPen(stroke, width));
    };
    auto line = [&](const V3 &a, const V3 &b, const QColor &stroke, double width, bool dash) {
        QPen pen(stroke, width);
        if (dash) pen.setDashPattern({3 / width, 3 / width});
        g->setPen(pen);
        g->drawLine(P(a), P(b));
    };
    QFont labelFont(u"sans-serif"_s);
    labelFont.setPixelSize(10);
    labelFont.setWeight(QFont::DemiBold);
    auto label = [&](const V3 &p, const QString &text, const QColor &color, double dx = 4, double dy = -4) {
        g->setFont(labelFont);
        g->setPen(color);
        g->drawText(P(p) + QPointF(dx, dy), text);
    };
    const QColor ink = m_ink;

    // Reference: level ground circle under the vehicle, faint
    {
        std::vector<V3> ring;
        for (int k = 0; k < 48; ++k) {
            const double a = k / 48.0 * 2 * kPi;
            ring.push_back({1.05 * std::cos(a), 1.05 * std::sin(a), 0});
        }
        g->setOpacity(0.18);
        poly(ring, QColor(), ink, 1);
        g->setOpacity(1);
    }

    // The vehicle: one thin triangle, nose forward, with a small fin
    poly({{0.62, 0, 0}, {-0.5, 0.3, 0}, {-0.5, -0.3, 0}}, QColor(200, 206, 214, 140), ink, 1.2);
    poly({{-0.5, 0, 0}, {-0.24, 0, 0}, {-0.5, 0, 0.24}}, QColor(200, 206, 214, 204), ink, 1.2);
    label({0.62, 0, 0}, u"FWD"_s, ink, 6, 4);
    g->setPen(Qt::NoPen);
    g->setBrush(ink);
    g->drawEllipse(P({0, 0, 0}), 2.2, 2.2);

    // The sensor at its lever arm (FRD -> view)
    const auto R = eulerToMatrix(num(m_mount.value(0)) * kDeg, num(m_mount.value(1)) * kDeg, num(m_mount.value(2)) * kDeg);
    const V3 S{num(m_lever.value(0)), -num(m_lever.value(1)), -num(m_lever.value(2))};
    auto at = [&](const V3 &d, double r) { return V3{S[0] + d[0] * r, S[1] + d[1] * r, S[2] + d[2] * r}; };

    // The zone, far faces first
    const QString type = m_zone.value(u"type"_s).toString();
    if (!type.isEmpty()) {
        struct Face {
            std::vector<V3> pts;
            double alpha, z;
        };
        std::vector<Face> faces;
        std::vector<std::vector<V3>> outline;
        auto centre = [](const std::vector<V3> &pts) {
            V3 c{0, 0, 0};
            for (const V3 &p : pts)
                for (int i = 0; i < 3; ++i) c[std::size_t(i)] += p[std::size_t(i)] / double(pts.size());
            return c;
        };
        auto quad = [&](const V3 &a, const V3 &b, const V3 &c, const V3 &d, double alpha) {
            const V3 mid{(a[0] + c[0]) / 2, (a[1] + c[1]) / 2, (a[2] + c[2]) / 2};
            faces.push_back({{a, b, c, d}, alpha, depth(mid)});
        };
        auto tri = [&](const V3 &a, const V3 &b, const V3 &c, double alpha) { faces.push_back({{a, b, c}, alpha, depth(centre({a, b, c}))}); };
        auto dirAt = [&](double az, double el) { return sensorToView(R, std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el)); };
        if (type == "band"_L1) {
            const int n = 48;
            const double e0 = num(m_zone.value(u"elMin"_s)) * kDeg, e1 = num(m_zone.value(u"elMax"_s)) * kDeg;
            std::vector<V3> lo, hi;
            for (int k = 0; k <= n; ++k) {
                const double a = k / double(n) * 2 * kPi;
                lo.push_back(at(dirAt(a, e0), kZoneR));
                hi.push_back(at(dirAt(a, e1), kZoneR));
            }
            for (int k = 0; k < n; ++k) quad(lo[std::size_t(k)], lo[std::size_t(k + 1)], hi[std::size_t(k + 1)], hi[std::size_t(k)], 0.13);
            outline.push_back(lo);
            outline.push_back(hi);
        } else if (type == "cone"_L1) {
            const int n = 36;
            const double h = std::max(1.0, num(m_zone.value(u"half"_s))) * kDeg;
            std::vector<V3> rim;
            for (int k = 0; k <= n; ++k) {
                const double a = k / double(n) * 2 * kPi;
                rim.push_back(at(sensorToView(R, std::cos(h), std::sin(h) * std::cos(a), std::sin(h) * std::sin(a)), kZoneR));
            }
            for (int k = 0; k < n; ++k) tri(S, rim[std::size_t(k)], rim[std::size_t(k + 1)], 0.13);
            outline.push_back(rim);
            for (int k : {0, 9, 18, 27}) outline.push_back({S, rim[std::size_t(k)]});
        } else if (type == "fan"_L1) {
            const int n = 40;
            const double a0 = num(m_zone.value(u"min"_s)) * kDeg, a1 = num(m_zone.value(u"max"_s)) * kDeg;
            std::vector<V3> arc;
            for (int k = 0; k <= n; ++k) arc.push_back(at(dirAt(a0 + (a1 - a0) * k / n, 0), kZoneR));
            for (int k = 0; k < n; ++k) tri(S, arc[std::size_t(k)], arc[std::size_t(k + 1)], 0.16);
            outline.push_back(arc);
            outline.push_back({S, arc.front()});
            outline.push_back({S, arc.back()});
        } else if (type == "dirs"_L1) {
            const int az = std::max(1, int(num(m_zone.value(u"az"_s), 36))), el = std::max(1, int(num(m_zone.value(u"el"_s), 18)));
            const double da = 2 * kPi / az, de = kPi / el;
            for (const QVariant &bv : m_zone.value(u"bins"_s).toList()) {
                const int b = bv.toInt(), i = b % az, j = b / az;
                const double a0 = -kPi + i * da, e0 = -kPi / 2 + j * de;
                quad(at(dirAt(a0, e0), kZoneR), at(dirAt(a0 + da, e0), kZoneR), at(dirAt(a0 + da, e0 + de), kZoneR), at(dirAt(a0, e0 + de), kZoneR), 0.3);
            }
        }
        std::sort(faces.begin(), faces.end(), [](const Face &p, const Face &q) { return p.z > q.z; });
        for (const Face &f : faces) poly(f.pts, QColor(32, 210, 200, int(std::round(f.alpha * 255))), QColor(), 0);
        for (const auto &o : outline) {
            QPainterPath path;
            for (std::size_t i = 0; i < o.size(); ++i) {
                if (i) path.lineTo(P(o[i]));
                else path.moveTo(P(o[i]));
            }
            g->strokePath(path, QPen(QColor(32, 210, 200, 191), 1));
        }
    }

    // Lever arm, then the sensor's axes on top
    if (std::sqrt(dot(S, S)) > 1e-3) line({0, 0, 0}, S, ink, 1, true);
    struct Axis {
        QString name;
        QColor color;
        V3 d;
    };
    std::vector<Axis> axes{{u"X"_s, kColX, sensorToView(R, 1, 0, 0)},
                           {u"Y"_s, kColY, sensorToView(R, 0, 1, 0)},
                           {u"Z"_s, kColZ, sensorToView(R, 0, 0, 1)}};
    std::sort(axes.begin(), axes.end(), [&](const Axis &p, const Axis &q) { return depth(p.d) > depth(q.d); });
    for (const Axis &a : axes) {
        const V3 tip = at(a.d, kAxisL);
        line(S, tip, a.color, 2.6, false);
        g->setPen(Qt::NoPen);
        g->setBrush(a.color);
        g->drawEllipse(P(tip), 2.4, 2.4);
        label(tip, a.name, a.color);
    }
    g->setPen(QPen(Qt::black, 1));
    g->setBrush(Qt::white);
    g->drawEllipse(P(S), 3, 3);

    // Captions
    QFont capFont(u"sans-serif"_s);
    capFont.setPixelSize(10);
    g->setFont(capFont);
    g->setOpacity(0.8);
    g->setPen(ink);
    g->drawText(QPointF(6, 13), u"sensor X Y Z (REP-103) · zone not to scale"_s);
    if (!m_caption.isEmpty()) g->drawText(QPointF(6, H - 7), m_caption);
    g->setOpacity(1);
}

} // namespace corv
