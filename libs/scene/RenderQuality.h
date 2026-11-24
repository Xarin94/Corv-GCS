// RenderQuality.h - SYS CONFIG -> 3D VIEW -> RESOLUTION: how many pixels the 3D
// scene is rendered at per point of the window (port of js/engine/RenderQuality.js).
// HUD and text keep the display's resolution whatever is chosen.
#pragma once

#include <QString>
#include <algorithm>
#include <cmath>
#include <limits>

namespace corv::renderQuality {

/// native | balanced | eco; anything else is the default.
inline QString normalize(const QString &value)
{
    return value == QLatin1String("native") || value == QLatin1String("eco") ? value : QStringLiteral("balanced");
}

/// Pixels per point: the display's, up to 1.5 (balanced) or 1 (eco).
inline double pixelRatio(const QString &quality, double displayRatio)
{
    const double ratio = std::isfinite(displayRatio) && displayRatio > 0 ? displayRatio : 1.0;
    const QString q = normalize(quality);
    const double cap = q == QLatin1String("native") ? std::numeric_limits<double>::infinity() : q == QLatin1String("eco") ? 1.0 : 1.5;
    return std::min(ratio, cap);
}

} // namespace corv::renderQuality
