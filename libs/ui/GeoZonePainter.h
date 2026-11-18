// GeoZonePainter.h - UAS geographical zones on a 2D map (js/maps/GeoZoneLayer.js),
// for the mini-map and the FLIGHT PLAN map.
//
// No-fly red with black bands, conditions violet with bands, a ceiling
// coloured by the height that may be flown (its floor above the ground; for a
// floor given above sea level, at the ground under the zone's centre); zones
// whose floor is 150 m or more above the ground are not drawn. Ceilings go
// highest first, so a lower one covers it, no-fly zones last, on top. Only the
// zones that touch the view are drawn, 6000 polygons at most.
#pragma once

#include "scene/GeoZone.h"

#include <QPointF>
#include <array>
#include <functional>

QT_FORWARD_DECLARE_CLASS(QPainter)

namespace corv::geozone {

/// viewBox: west, south, east, north of what is on screen; toScreen: lat, lon -> pixels.
void paintZones(QPainter *p, const GeoZoneList &zones, const std::array<double, 4> &viewBox,
                const std::function<QPointF(double lat, double lon)> &toScreen);

/// The zone under a point and a one-line description of it, for a tooltip; empty when none.
QString describeAt(const GeoZoneList &zones, double lat, double lon);

} // namespace corv::geozone
