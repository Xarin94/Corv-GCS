#include "ui/MagCalView.h"

#include "ui/CalibrationController.h"
#include "vehicle/GeodesicGrid.h"

#include <QCursor>
#include <QFont>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QRadialGradient>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

namespace corv {

namespace {

constexpr double kFovDeg = 36;
constexpr double kDefaultYaw = 0.65, kDefaultPitch = 0.42, kDefaultDist = 4.4;

const QColor kIdle(0x24, 0x38, 0x4c);       // no calibration running
const QColor kMissing(0x3a, 0x2a, 0x36);    // not covered yet
const QColor kCovered(0x1f, 0xbf, 0x62);
const QColor kTarget(0xff, 0xa0, 0x20);

// Body frame (x nose, y right, z down) -> scene (x right, y up, z towards the tail)
QVector3D toScene(const QVector3D &b) { return {b.y(), -b.z(), -b.x()}; }

struct Tessellation {
    // Per section: its 16 sub-triangles on the unit sphere (scene frame), so a
    // section bulges with the sphere instead of cutting a flat facet through it
    std::vector<std::vector<std::array<QVector3D, 3>>> tris;
    // Section outlines along great circles, slightly outside the sphere
    std::vector<std::pair<QVector3D, QVector3D>> lines;
};

const Tessellation &tessellation()
{
    static const Tessellation t = [] {
        Tessellation out;
        constexpr int N = 4;
        const auto &secs = geogrid::sections();
        const auto lerp = [](const QVector3D &a, const QVector3D &b, const QVector3D &c, float u, float v) {
            const float w = 1 - u - v;
            return toScene((a * w + b * u + c * v).normalized());
        };
        out.tris.resize(geogrid::kSections);
        for (int s = 0; s < geogrid::kSections; ++s) {
            const auto &sec = secs[std::size_t(s)];
            for (int i = 0; i < N; ++i)
                for (int j = 0; j < N - i; ++j) {
                    const auto P = [&](int a, int b) { return lerp(sec.a, sec.b, sec.c, float(a) / N, float(b) / N); };
                    out.tris[std::size_t(s)].push_back({P(i, j), P(i + 1, j), P(i, j + 1)});
                    if (j < N - i - 1) out.tris[std::size_t(s)].push_back({P(i + 1, j), P(i + 1, j + 1), P(i, j + 1)});
                }
            const auto arc = [&](const QVector3D &p, const QVector3D &q) {
                for (int k = 0; k < 6; ++k) {
                    const auto at = [&](float f) { return toScene((p + (q - p) * f).normalized() * 1.002f); };
                    out.lines.push_back({at(k / 6.f), at((k + 1) / 6.f)});
                }
            };
            arc(sec.a, sec.b);
            arc(sec.b, sec.c);
            arc(sec.c, sec.a);
        }
        return out;
    }();
    return t;
}

struct Camera {
    QMatrix4x4 mvp;
    QVector3D eye;
    double w = 1, h = 1, focal = 1;

    /// Screen position and depth (distance along the view axis); none behind the eye
    std::optional<std::pair<QPointF, double>> project(const QVector3D &v) const
    {
        const QVector4D c = mvp * QVector4D(v, 1);
        if (c.w() <= 1e-4f) return std::nullopt;
        return std::make_pair(QPointF((c.x() / c.w() + 1) * 0.5 * w, (1 - c.y() / c.w()) * 0.5 * h), double(c.w()));
    }
};

QColor withAlpha(QColor c, double a)
{
    c.setAlphaF(float(a));
    return c;
}

} // namespace

MagCalView::MagCalView(QQuickItem *parent) : QQuickPaintedItem(parent)
{
    setAcceptedMouseButtons(Qt::LeftButton);
    setAntialiasing(true);
    setCursor(Qt::OpenHandCursor);
}

void MagCalView::componentComplete()
{
    QQuickPaintedItem::componentComplete();
    m_cal = CalibrationController::s_instance;
    if (m_cal) connect(m_cal, &CalibrationController::magViewChanged, this, [this] { update(); });
}

void MagCalView::resetView()
{
    m_yaw = kDefaultYaw;
    m_pitch = kDefaultPitch;
    m_dist = kDefaultDist;
    update();
}

void MagCalView::paint(QPainter *p)
{
    const double W = width(), H = height();
    if (W < 2 || H < 2) return;
    p->setRenderHint(QPainter::Antialiasing, true);

    // The view keeps a dark instrument background in both themes
    const QPointF c0(W * 0.5, H * 0.45);
    const double far = std::hypot(std::max(c0.x(), W - c0.x()), std::max(c0.y(), H - c0.y()));
    QRadialGradient bg(c0, far);
    bg.setColorAt(0, QColor(0x13, 0x20, 0x2d));
    bg.setColorAt(0.75, QColor(0x07, 0x0b, 0x10));
    bg.setColorAt(1, QColor(0x07, 0x0b, 0x10));
    p->setPen(QPen(QColor(255, 255, 255, 20), 1));
    p->setBrush(bg);
    p->drawRoundedRect(QRectF(0.5, 0.5, W - 1, H - 1), 4, 4);
    if (!m_cal) return;
    const MagSamples &s = m_cal->samples();
    p->setClipRect(QRectF(1, 1, W - 2, H - 2));

    Camera cam;
    cam.w = W;
    cam.h = H;
    cam.eye = QVector3D(float(m_dist * std::cos(m_pitch) * std::sin(m_yaw)), float(m_dist * std::sin(m_pitch)),
                        float(m_dist * std::cos(m_pitch) * std::cos(m_yaw)));
    QMatrix4x4 proj, view;
    proj.perspective(float(kFovDeg), float(W / H), 0.05f, 50.f);
    view.lookAt(cam.eye, QVector3D(), QVector3D(0, 1, 0));
    cam.mvp = proj * view;
    cam.focal = H * 0.5 / std::tan(kFovDeg * M_PI / 360);

    // Section colours: idle without a calibration, else covered / missing / aim here
    const auto &mask = s.coverage();
    const int target = s.target();
    const auto sectionColor = [&](int sec) {
        if (!mask) return kIdle;
        if (sec == target) return kTarget;
        return geogrid::maskHas(*mask, sec) ? kCovered : kMissing;
    };

    // The sphere: far half faint first, near half last; each section one path so
    // its sub-triangles leave no seams between them
    const Tessellation &tess = tessellation();
    std::vector<QPainterPath> back(geogrid::kSections), front(geogrid::kSections);
    for (int sec = 0; sec < geogrid::kSections; ++sec) {
        back[std::size_t(sec)].setFillRule(Qt::WindingFill);
        front[std::size_t(sec)].setFillRule(Qt::WindingFill);
        for (const auto &tri : tess.tris[std::size_t(sec)]) {
            const QVector3D centre = (tri[0] + tri[1] + tri[2]) / 3;
            QPolygonF poly;
            for (const QVector3D &v : tri) {
                const auto pr = cam.project(v);
                if (!pr) break;
                poly << pr->first;
            }
            if (poly.size() != 3) continue;
            const bool facing = QVector3D::dotProduct(centre, cam.eye - centre) > 0;
            (facing ? front : back)[std::size_t(sec)].addPolygon(poly);
        }
    }
    p->setPen(Qt::NoPen);
    for (int sec = 0; sec < geogrid::kSections; ++sec)
        p->fillPath(back[std::size_t(sec)], withAlpha(sectionColor(sec), 0.16));

    // The samples, each compass on its own fitted sphere; the one in focus brighter
    for (int c = 0; c < MagSamples::kCompasses; ++c) {
        const auto &cloud = s.cloud(c);
        if (cloud.points.empty()) continue;
        const MagSamples::Fit f = s.frame(c);
        QPolygonF pts;
        pts.reserve(qsizetype(cloud.points.size()));
        for (const QVector3D &pt : cloud.points)
            if (const auto pr = cam.project(toScene((pt - f.center) / f.r))) pts << pr->first;
        QPen pen(withAlpha(QColor(CalibrationController::magColorList()[c]), c == s.focus() ? 0.95 : 0.35), 3);
        pen.setCapStyle(Qt::RoundCap);
        p->setPen(pen);
        p->drawPoints(pts);
    }

    p->setPen(Qt::NoPen);
    for (int sec = 0; sec < geogrid::kSections; ++sec)
        p->fillPath(front[std::size_t(sec)], withAlpha(sectionColor(sec), 0.38));
    {
        QVector<QLineF> lines;
        lines.reserve(qsizetype(tess.lines.size()));
        for (const auto &[a, b] : tess.lines) {
            const auto pa = cam.project(a), pb = cam.project(b);
            if (pa && pb) lines << QLineF(pa->first, pb->first);
        }
        p->setPen(QPen(withAlpha(QColor(0x9f, 0xd8, 0xff), 0.13), 1));
        p->drawLines(lines);
    }

    // Body axes and the vehicle at the centre: an arrowhead pointing at the nose
    struct Axis {
        QVector3D dir;
        QColor color;
        const char *label;
    };
    const Axis axes[] = {{{1, 0, 0}, QColor(0xff, 0x5d, 0x5d), "NOSE"}, {{0, 1, 0}, QColor(0x5d, 0xdc, 0x6a), "RIGHT"},
                         {{0, 0, 1}, QColor(0x5d, 0x9d, 0xff), "DOWN"}};
    const auto origin = cam.project(QVector3D());
    for (const Axis &a : axes) {
        const auto end = cam.project(toScene(a.dir * 1.45f));
        if (origin && end) {
            p->setPen(QPen(withAlpha(a.color, 0.85), 1.2));
            p->drawLine(origin->first, end->first);
        }
    }
    {
        QPolygonF arrow;
        for (const QVector3D &v : {QVector3D(0.3f, 0, 0), QVector3D(-0.18f, -0.17f, 0), QVector3D(-0.18f, 0.17f, 0)})
            if (const auto pr = cam.project(toScene(v))) arrow << pr->first;
        if (arrow.size() == 3) {
            p->setPen(Qt::NoPen);
            p->setBrush(withAlpha(QColor(0xdf, 0xe8, 0xf2), 0.85));
            p->drawPolygon(arrow);
        }
    }

    // The latest sample of the compass in focus, and the line from the centre to it
    if (const auto m = s.marker()) {
        const QVector3D ms = toScene(*m);
        const auto pm = cam.project(ms);
        if (origin && pm) {
            p->setPen(QPen(withAlpha(Qt::white, 0.55), 1));
            p->drawLine(origin->first, pm->first);
            const double r = std::max(2.0, 0.045 * cam.focal / pm->second);
            p->setPen(Qt::NoPen);
            p->setBrush(Qt::white);
            p->drawEllipse(pm->first, r, r);
        }
    }

    // Axis labels on top, sized as the sprites of the Electron view
    for (const Axis &a : axes) {
        const auto at = cam.project(toScene(a.dir * 1.45f * 1.12f));
        if (!at) continue;
        QFont f(QStringLiteral("Rajdhani"));
        f.setBold(true);
        f.setPixelSize(std::clamp(int(std::lround(0.103 * cam.focal / at->second)), 8, 30));
        p->setFont(f);
        p->setPen(a.color);
        const QRectF box(at->first.x() - 60, at->first.y() - 20, 120, 40);
        p->drawText(box, Qt::AlignCenter, QLatin1String(a.label));
    }
}

void MagCalView::mousePressEvent(QMouseEvent *e)
{
    m_drag = e->position();
    m_dragging = true;
    setKeepMouseGrab(true);   // a drag turns the sphere, it does not scroll the page
    setCursor(Qt::ClosedHandCursor);
    e->accept();
}

void MagCalView::mouseMoveEvent(QMouseEvent *e)
{
    if (!m_dragging) return;
    const QPointF d = e->position() - m_drag;
    m_drag = e->position();
    m_yaw -= d.x() * 0.008;
    m_pitch = std::clamp(m_pitch + d.y() * 0.008, -1.45, 1.45);
    update();
}

void MagCalView::mouseReleaseEvent(QMouseEvent *)
{
    m_dragging = false;
    setKeepMouseGrab(false);
    setCursor(Qt::OpenHandCursor);
}

void MagCalView::mouseDoubleClickEvent(QMouseEvent *) { resetView(); }

void MagCalView::wheelEvent(QWheelEvent *e)
{
    // The browser's deltaY: about 100 per notch, positive towards the user
    const double deltaY = -e->angleDelta().y() / 1.2;
    m_dist = std::clamp(m_dist * (1 + deltaY * 0.001), 2.2, 10.0);
    update();
    e->accept();
}

} // namespace corv
