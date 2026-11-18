// HudPaint.h - what the borderless schematics over the 3D scene share (ROTOR
// LOAD, SAIL · RUDDER, FLAPS): colours, the HUD drop shadow, canvas arcs, the
// UI font and the NACA 00xx thickness.
#pragma once

#include <QColor>
#include <QFont>
#include <QPainterPath>
#include <QRectF>
#include <algorithm>
#include <cmath>

namespace corv::hudpaint {

inline const QColor kIdle{0x2f, 0x9f, 0xff};   // blue: at or below idle, motor stopped
inline const QColor kGreen{0x44, 0xff, 0x44};
inline const QColor kOrange{0xff, 0xaa, 0x00};
inline const QColor kRed{0xff, 0x30, 0x30};
inline const QColor kNoData{0x55, 0x65, 0x6f};
inline const QColor kScale{200, 220, 235, int(0.3 * 255 + 0.5)};
inline const QColor kTick{200, 220, 235, int(0.7 * 255 + 0.5)};
inline const QColor kText{200, 220, 235, int(0.85 * 255 + 0.5)};
inline const QColor kTravel{47, 159, 255, int(0.6 * 255 + 0.5)};
inline const QColor kFoilFill{68, 255, 68, int(0.3 * 255 + 0.5)};

// The panels are borderless over the 3D scene, so everything gets the same drop
// shadow the rest of the HUD uses to stay readable over bright terrain:
// rgba(0, 0, 0, 0.9), 1 px down-right, 2 px blur (drawn as a wider stroke)
constexpr double kShadowAlpha = 0.9;
constexpr double kShadowOffset = 1.0;
constexpr double kShadowSpread = 1.0;

/// The colour itself, or the drop shadow's for the shadow pass.
inline QColor ink(const QColor &c, bool shadow)
{
    if (!shadow) return c;
    return QColor(0, 0, 0, int(std::lround(255 * kShadowAlpha * c.alphaF())));
}

/// Canvas arc (angles in degrees clockwise from +x, y down) as a path.
inline QPainterPath arcPath(double cx, double cy, double r, double startDeg, double endDeg)
{
    const QRectF rect(cx - r, cy - r, 2 * r, 2 * r);
    QPainterPath path;
    path.arcMoveTo(rect, -startDeg);
    path.arcTo(rect, -startDeg, -(endDeg - startDeg));
    return path;
}

inline QFont uiFont(double px)
{
    QFont f(QStringLiteral("Rajdhani"));
    f.setPixelSize(std::max(1, int(std::lround(px))));
    return f;
}

/// Half thickness of a NACA 00xx section at x (0..1 of the chord), closed trailing edge.
inline double nacaHalfThickness(double x, double t)
{
    return 5 * t * (0.2969 * std::sqrt(x) - 0.1260 * x - 0.3516 * x * x + 0.2843 * x * x * x - 0.1036 * x * x * x * x);
}

} // namespace corv::hudpaint
