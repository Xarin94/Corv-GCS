#include "ui/VehicleModel.h"

#include "core/Clock.h"
#include "core/Geo.h"
#include "link/Mavlink.h"
#include "vehicle/FlightModes.h"
#include "terrain/Hgt.h"
#include "vehicle/VehicleHub.h"

#include <algorithm>
#include <vector>

using namespace Qt::StringLiterals;

namespace corv {

VehicleModel *VehicleModel::s_instance = nullptr;

VehicleModel::VehicleModel(VehicleHub *hub, QObject *parent) : QObject(parent), m_hub(hub)
{
    s_instance = this;
    m_s = hub->snapshot();
    m_timer.setInterval(33);
    connect(&m_timer, &PacedTimer::timeout, this, &VehicleModel::poll);
    m_timer.start();
}

void VehicleModel::poll()
{
    auto s = m_hub->snapshot();
    const qint64 now = wallMs();
    // Flight timer: runs while armed
    if (s->armed && !m_armedSince) m_armedSince = now;
    if (!s->armed && m_armedSince) {
        m_flightMs += now - m_armedSince;
        m_armedSince = 0;
    }
    if (s == m_s && !s->armed) return;
    m_s = std::move(s);
    emit updated();
}

bool VehicleModel::heartbeatAlive() const
{
    if (!m_s->connected) return false;
    return wallMs() - m_s->lastHeartbeatTime < 3000;
}

QString VehicleModel::modeClass() const { return modes::modeClass(m_s->flightMode); }

QStringList VehicleModel::availableModes() const
{
    QStringList out;
    for (const auto &m : modes::available(m_s->vehicleType, m_s->connectionType)) out << m.second;
    return out;
}

QString VehicleModel::vehicleTypeName() const { return modes::vehicleTypeName(m_s->vehicleType); }
double VehicleModel::rollDeg() const { return m_s->roll * kRad; }
double VehicleModel::pitchDeg() const { return m_s->pitch * kRad; }

double VehicleModel::headingDeg() const
{
    double h = std::fmod(m_s->yaw * kRad, 360.0);
    return h < 0 ? h + 360 : h;
}

double VehicleModel::homeDistance() const
{
    if (!m_s->homeLat || !m_s->homeLon) return 0;
    return flatDistance(*m_s->homeLat, *m_s->homeLon, m_s->lat, m_s->lon);
}

QString VehicleModel::gpsFixName() const { return modes::gpsFixName(m_s->gpsFix); }

bool VehicleModel::gpsStale() const
{
    // A GPS_RAW_INT older than 5 s shows as no GPS: a vehicle without GPS sends none
    return m_s->connected && wallMs() - m_s->gpsDataTime > 5000;
}

QString VehicleModel::flightTime() const
{
    qint64 ms = m_flightMs + (m_armedSince ? wallMs() - m_armedSince : 0);
    const qint64 s = ms / 1000;
    return u"%1:%2:%3"_s.arg(s / 3600, 2, 10, '0'_L1).arg((s / 60) % 60, 2, 10, '0'_L1).arg(s % 60, 2, 10, '0'_L1);
}

double VehicleModel::terrainAgl() const
{
    if (m_s->terrainHeight) return m_s->rawAlt - *m_s->terrainHeight;
    if (const auto h = HgtStore::instance().elevation(m_s->lat, m_s->lon)) return m_s->rawAlt - *h;
    return std::numeric_limits<double>::quiet_NaN();
}

/// The 4 nearest traffic entries, sorted by distance from the vehicle, as the
/// rows of the traffic table: callsign (else ICAO address), altitude in whole
/// metres, distance in km with one decimal (getNearestTraffic + updateUI).
QVariantList VehicleModel::nearestTraffic() const
{
    QVariantList out;
    if (m_s->traffic.empty()) return out;
    std::vector<std::pair<double, const TrafficEntry *>> list;
    list.reserve(m_s->traffic.size());
    for (const TrafficEntry &t : m_s->traffic) list.emplace_back(haversine(m_s->lat, m_s->lon, t.lat, t.lon), &t);
    std::sort(list.begin(), list.end(), [](const auto &a, const auto &b) { return a.first < b.first; });
    for (std::size_t i = 0; i < list.size() && i < 4; ++i) {
        const TrafficEntry &t = *list[i].second;
        out.append(QVariantMap{{u"callsign"_s, t.callsign.isEmpty() ? t.icao24 : t.callsign},
                               {u"alt"_s, QString::number(std::lround(t.alt))},
                               {u"dist"_s, QString::number(list[i].first / 1000, 'f', 1)}});
    }
    return out;
}

QString VehicleModel::autopilotName() const
{
    switch (m_s->autopilotType) {
    case MAV_AUTOPILOT_ARDUPILOTMEGA: return u"ArduPilot"_s;
    case MAV_AUTOPILOT_PX4: return u"PX4"_s;
    case MAV_AUTOPILOT_GENERIC: return u"Generic"_s;
    default: return u"Autopilot %1"_s.arg(m_s->autopilotType);
    }
}

} // namespace corv
