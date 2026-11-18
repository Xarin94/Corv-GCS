#include "ui/FlapIndicatorItem.h"

#include "ui/HudPaint.h"

#include <QFontMetricsF>
#include <QPainter>
#include <QPainterPath>
#include <algorithm>
#include <cmath>

namespace corv {

using namespace hudpaint;

namespace {

constexpr double kMinDeg = -10;    // reflex, flap up
constexpr double kMaxDeg = 30;     // full flap, down
constexpr double kFullDeg = 30;    // the angle drawn for 100 %

// Same stroke weight as the rotor gauges: arcs and ticks share it
constexpr double kLineW = 4;

// The wing: a cambered NACA 4-digit section (a wing, not a rudder), the flap
// the aft third of it
constexpr double kThickness = 0.14;
constexpr double kCamber = 0.03;
constexpr double kCamberAt = 0.4;
constexpr double kFlapChord = 0.32;
// Scale radius in flap lengths: the flap tip stops inside the ticks
constexpr double kScaleR = 1.5;
// Ticks as the rotor gauges', in scale radii: fixed at zero, moving at the value
constexpr double kZeroTick = 0.22;
constexpr double kValueTick = 0.3;
constexpr int kSteps = 20;

double clampd(double v, double lo, double hi) { return std::min(std::max(v, lo), hi); }

/// NACA 4-digit mean line at x (0..1 of the chord).
double camber(double x)
{
    const double m = kCamber, p = kCamberAt;
    return x < p ? m / (p * p) * (2 * p * x - x * x) : m / ((1 - p) * (1 - p)) * ((1 - 2 * p) + 2 * p * x - x * x);
}

QString percentText(double v)
{
    const long n = std::lround(v);
    return (n < 0 ? QStringLiteral("−") : QString()) + QString::number(std::abs(n)) + QStringLiteral("%");
}

} // namespace

FlapIndicatorItem::FlapIndicatorItem(QQuickItem *parent) : QQuickPaintedItem(parent), m_percent(qQNaN())
{
    setAntialiasing(true);
}

void FlapIndicatorItem::setPercent(qreal v)
{
    const bool same = std::isnan(v) ? std::isnan(m_percent) : !std::isnan(m_percent) && std::abs(v - m_percent) < 0.01;
    if (same) return;
    m_percent = v;
    emit percentChanged();
    update();
}

double FlapIndicatorItem::angleFor(double percent)
{
    return clampd(percent / 100 * kFullDeg, kMinDeg, kMaxDeg);
}

void FlapIndicatorItem::paint(QPainter *p)
{
    const double w = width(), h = height();
    if (w <= 0 || h <= 0) return;
    p->setRenderHint(QPainter::Antialiasing);
    p->setRenderHint(QPainter::TextAntialiasing);
    // Shadow pass, then the drawing itself
    for (bool shadow : {true, false}) {
        p->save();
        if (shadow) p->translate(kShadowOffset, kShadowOffset);
        draw(p, w, h, shadow);
        p->restore();
    }
}

/// The rotor gauges' grammar on a 40° sector about the flap hinge: a bare
/// scale arc, a fixed tick at zero, a moving tick and a filled arc that follow
/// the deployment, all in one stroke weight, and no digits on the scale. As on
/// the FLAP display of an Airbus E/WD the wing is a fixed reference, the flap
/// moves down and aft over detent dots (10°, 20°, full), and the setting is
/// written under it, in the colour of its zone: blue retracted (the colour of
/// a stopped motor), green deployed, grey with no data.
void FlapIndicatorItem::draw(QPainter *p, double w, double h, bool shadow)
{
    const bool live = !std::isnan(m_percent);
    const double deg = live ? angleFor(m_percent) : 0;
    const bool out = live && deg > 0.15;
    const QColor color = !live ? kNoData : out ? kGreen : kIdle;
    const double spread = shadow ? kShadowSpread : 0;
    const double lw = kLineW;
    const double thin = std::max(1.5, lw * 0.5);   // outlines, as the sail foils
    const QFont valFont = uiFont(clampd(std::round(h * 0.3), 9, 22));
    const QFontMetricsF vfm(valFont);
    const double dotR = lw * 0.45, dotOut = lw * 1.4;   // detent dots, just outside the arc

    // The chord c that fits the width (wing and scale with its dots) and the
    // height (top of the arc and the wing above the hinge; the arc at full
    // flap and the readout under the wing below it)
    const double xh = 1 - kFlapChord;   // hinge, chord fraction
    const double rk = kScaleR * kFlapChord;   // scale radius, in chords
    const double edge = lw / 2 + spread + 1;
    const auto above = [&](double c) { return std::max(0.08 * c, rk * c * std::sin(-kMinDeg * M_PI / 180) + lw / 2) + edge; };
    const auto below = [&](double c) {
        return std::max((rk * c + dotOut + dotR) * std::sin(kMaxDeg * M_PI / 180), 0.075 * c + lw + vfm.height()) + edge;
    };
    double c = (w - 2 * edge - dotOut - dotR) / (xh + rk);
    while (c > 10 && above(c) + below(c) > h) c *= 0.95;
    if (c <= 10) return;
    const double R = rk * c;
    const double le = (w - (c * (xh + rk) + dotOut + dotR)) / 2;
    const double hx = le + xh * c;
    const double hy = (h - above(c) - below(c)) / 2 + above(c);

    const auto pen = [&](const QColor &col, double width) {
        return QPen(ink(col, shadow), width + spread, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    };
    const auto radial = [&](double d, double from, double to, const QColor &col) {
        const double a = d * M_PI / 180;
        p->setPen(pen(col, lw));
        p->drawLine(QPointF(hx + std::cos(a) * from, hy + std::sin(a) * from), QPointF(hx + std::cos(a) * to, hy + std::sin(a) * to));
    };

    // Scale arc, and the coloured part riding exactly on it from zero to the value
    p->setBrush(Qt::NoBrush);
    p->setPen(pen(kScale, lw));
    p->drawPath(arcPath(hx, hy, R, kMinDeg, kMaxDeg));
    if (live && std::abs(deg) > 0.15) {
        p->setPen(pen(color, lw));
        p->drawPath(arcPath(hx, hy, R, std::min(0.0, deg), std::max(0.0, deg)));
    }
    // Detent dots
    p->setPen(Qt::NoPen);
    p->setBrush(ink(kTick, shadow));
    for (double d : {10.0, 20.0, kMaxDeg}) {
        const double a = d * M_PI / 180, rr = R + dotOut;
        p->drawEllipse(QPointF(hx + std::cos(a) * rr, hy + std::sin(a) * rr), dotR + spread / 2, dotR + spread / 2);
    }
    // Fixed tick at zero, moving tick at the value
    radial(0, R * (1 - kZeroTick), R, kTick);
    radial(deg, R * (1 - kValueTick), R, live ? color : kNoData);

    // The wing ahead of the flap, outline only: upper surface to the cut and
    // the lower surface back, cosine spacing where the nose curves; the cut
    // leaves a slot round the flap nose
    const double yh = camber(xh);
    const double rN = nacaHalfThickness(xh, kThickness);   // flap nose radius, chords
    const double xb = xh - rN - (thin + 1) / c;
    const auto pt = [&](double x, double y) { return QPointF(le + x * c, hy - (y - yh) * c); };
    QPainterPath body;
    for (int i = 0; i <= kSteps; ++i) {
        const double x = xb * (1 - std::cos(M_PI * i / kSteps)) / 2;
        const QPointF q = pt(x, camber(x) + nacaHalfThickness(x, kThickness));
        if (i == 0) body.moveTo(q);
        else body.lineTo(q);
    }
    for (int i = kSteps; i >= 0; --i) {
        const double x = xb * (1 - std::cos(M_PI * i / kSteps)) / 2;
        body.lineTo(pt(x, camber(x) - nacaHalfThickness(x, kThickness)));
    }
    body.closeSubpath();
    p->setBrush(Qt::NoBrush);
    p->setPen(pen(kTick, thin));
    p->drawPath(body);

    // The flap about its hinge, drawn like the sail foils: the aft part of the
    // same section with a round nose, light fill and outline in the zone colour
    QPainterPath flap;
    const auto lp = [&](double x, double y) { return QPointF((x - xh) * c, -(y - yh) * c); };
    for (int i = 0; i <= kSteps; ++i) {
        const double x = xh + kFlapChord * i / kSteps;
        const QPointF q = lp(x, camber(x) + nacaHalfThickness(x, kThickness));
        if (i == 0) flap.moveTo(q);
        else flap.lineTo(q);
    }
    for (int i = kSteps; i >= 0; --i) {
        const double x = xh + kFlapChord * i / kSteps;
        flap.lineTo(lp(x, camber(x) - nacaHalfThickness(x, kThickness)));
    }
    flap.arcTo(QRectF(-rN * c, -rN * c, 2 * rN * c, 2 * rN * c), 270, -180);
    flap.closeSubpath();
    p->save();
    p->translate(hx, hy);
    p->rotate(deg);   // clockwise on screen: trailing edge down
    QColor fill = color;
    fill.setAlphaF(0.3);
    p->setBrush(live ? QBrush(ink(fill, shadow)) : QBrush(Qt::NoBrush));
    p->setPen(pen(color, thin));
    p->drawPath(flap);
    p->restore();

    // The setting under the wing, in the zone colour like the motor labels
    p->setFont(valFont);
    p->setPen(ink(color, shadow));
    p->drawText(QRectF(le, hy + 0.075 * c + lw, xh * c, vfm.height()), Qt::AlignHCenter | Qt::AlignTop,
                live ? percentText(m_percent) : QStringLiteral("---"));
}

} // namespace corv
