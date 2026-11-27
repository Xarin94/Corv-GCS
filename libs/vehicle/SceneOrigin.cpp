#include "vehicle/SceneOrigin.h"

#include "core/DebugLog.h"

#include <cmath>

using namespace Qt::StringLiterals;

namespace corv {

bool SceneOrigin::set(double lat, double lon)
{
    if (!std::isfinite(lat) || !std::isfinite(lon)) return false;
    if (lat == m_origin.lat && lon == m_origin.lon) return false;
    m_origin = {lat, lon};
    ++m_epoch;
    LOG_I("origin") << u"scene origin moved to %1, %2"_s.arg(lat, 0, 'f', 7).arg(lon, 0, 'f', 7);
    if (onMoved) onMoved(m_origin);
    return true;
}

bool SceneOrigin::fix(double lat, double lon)
{
    if (!m_awaitingFix || !std::isfinite(lat) || !std::isfinite(lon)) return false;
    m_awaitingFix = false;
    set(lat, lon);
    return true;
}

void SceneOrigin::home(double lat, double lon)
{
    if (!std::isfinite(lat) || !std::isfinite(lon)) return;
    // A home re-set where the vehicle stands keeps the origin (and the terrain built on it)
    const bool near = haversine(m_origin.lat, m_origin.lon, lat, lon) < kHomeKeepM;
    if (near && !m_awaitingFix) return;
    m_awaitingFix = false;
    set(lat, lon);
}

} // namespace corv
