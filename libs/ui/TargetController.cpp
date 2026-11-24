#include "ui/TargetController.h"

#include "gpu/SceneItem.h"
#include "ui/DialogHub.h"
#include "ui/MapItem.h"
#include "ui/VehicleModel.h"

#include <cmath>

using namespace Qt::StringLiterals;

namespace corv {

TargetController *TargetController::s_instance = nullptr;

TargetController::TargetController(VehicleModel *vehicle, QObject *parent) : QObject(parent), m_vehicle(vehicle)
{
    s_instance = this;
    connect(vehicle, &VehicleModel::updated, this, [this] {
        if (m_at) emit moved();
    });
}

bool TargetController::set(const QString &latText, const QString &lonText)
{
    auto number = [](QString t, bool *ok) { return t.trimmed().replace(','_L1, '.'_L1).toDouble(ok); };
    bool okLat = false, okLon = false;
    const double lat = number(latText, &okLat), lon = number(lonText, &okLon);
    if (!okLat || !okLon || std::abs(lat) > 90 || std::abs(lon) > 180) {
        DialogHub::toast(u"Target: latitude and longitude in decimal degrees"_s);
        return false;
    }
    m_at = LatLon{lat, lon};
    apply();
    return true;
}

void TargetController::clear()
{
    if (!m_at) return;
    m_at.reset();
    apply();
}

void TargetController::apply()
{
    SceneItem::setTarget(m_at);
    MapItem::setTarget(m_at);
    emit changed();
    emit moved();
}

double TargetController::distance() const
{
    return m_at ? flatDistance(m_vehicle->lat(), m_vehicle->lon(), m_at->lat, m_at->lon) : 0;
}

double TargetController::relativeBearing() const
{
    if (!m_at) return 0;
    const double north = m_at->lat - m_vehicle->lat();
    const double east = (m_at->lon - m_vehicle->lon()) * std::cos(m_vehicle->lat() * kDeg);
    const double rel = std::atan2(east, north) * kRad - m_vehicle->headingDeg();
    return std::remainder(rel, 360.0);
}

} // namespace corv
