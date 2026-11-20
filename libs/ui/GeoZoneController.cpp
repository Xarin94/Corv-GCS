#include "ui/GeoZoneController.h"

#include "core/Clock.h"
#include "core/DataRoot.h"
#include "core/DebugLog.h"
#include "core/Settings.h"
#include "gpu/SceneItem.h"
#include "services/GeoZoneStore.h"
#include "terrain/Hgt.h"
#include "ui/PlanController.h"
#include "vehicle/FlightModes.h"
#include "vehicle/VehicleHub.h"

#include <QDateTime>
#include <QFileInfo>
#include <QJsonArray>
#include <cmath>

using namespace Qt::StringLiterals;

namespace corv {

GeoZoneController *GeoZoneController::s_instance = nullptr;

namespace {
constexpr double kRadiusKm = 50;
constexpr double kRequeryKm = 5;
constexpr double kMissionSpacingKm = 10;   // mission points closer than this to one already asked share its circle
constexpr qint64 kRefreshMs = 10 * 60 * 1000;
constexpr double kAirborneAgl = 3;
constexpr int kArmAlertMs = 10000;

GeoZonesPtr g_shown;
int g_shownVersion = 0;

double kmBetween(double lat1, double lon1, double lat2, double lon2)
{
    const double k = std::cos(lat1 * M_PI / 180);
    return std::hypot((lat1 - lat2) * 111.32, (lon1 - lon2) * 111.32 * k);
}

QString alertTitle(const geozone::Alert &a)
{
    QStringList lines;
    if (a.zone) {
        lines << a.zone->name;
        if (a.id == "noFly"_L1) lines << u"No-fly or authorisation required from the surface"_s;
        else if (a.id == "cond"_L1) lines << u"Conditions apply in this zone"_s;
        else lines << u"Height that may be flown here: %1 m above the ground"_s.arg(qRound(a.metres));
        lines << u"Source: "_s + geozone::sourceName(a.zone->src);
        if (!a.zone->info.isEmpty()) lines << (a.zone->info.size() > 200 ? a.zone->info.left(199) + u"…"_s : a.zone->info);
    } else if (!a.rule.isEmpty()) {
        lines << a.rule;
    }
    lines << u"Click to silence until the next connection"_s;
    return lines.join('\n'_L1);
}
} // namespace

GeoZonesPtr GeoZoneController::shownZones() { return g_shown; }

GeoZoneController::GeoZoneController(VehicleHub *hub, PlanController *plan, QObject *parent)
    : QObject(parent), m_hub(hub), m_plan(plan), m_store(new GeoZoneStore(DataRoot::geozonesDir(), this))
{
    s_instance = this;
    m_enabled = Settings::instance().value(u"geozones-enabled"_s, u"0"_s).toString() == "1"_L1;
    connect(m_store, &GeoZoneStore::changed, this, &GeoZoneController::query);
    m_evalTimer.setInterval(250);
    connect(&m_evalTimer, &QTimer::timeout, this, &GeoZoneController::evaluate);
    m_evalTimer.start();
    m_missionTimer.setSingleShot(true);
    m_missionTimer.setInterval(1500);
    connect(&m_missionTimer, &QTimer::timeout, this, &GeoZoneController::query);
    if (m_plan) connect(m_plan, &PlanController::routeChanged, &m_missionTimer, qOverload<>(&QTimer::start));
    m_armTimer.setSingleShot(true);
    m_armTimer.setInterval(kArmAlertMs);
    connect(&m_armTimer, &QTimer::timeout, this, &GeoZoneController::closeArmAlert);
    if (m_hub) {
        connect(m_hub, &VehicleHub::connectionChanged, this, [this] {
            // A new link: an armed vehicle met on connecting is not an arming; silenced alerts speak again
            m_wasArmed = -1;
            m_silenced.clear();
        }, Qt::QueuedConnection);
    }
    QTimer::singleShot(0, this, [this] {
        if (m_enabled) query();
        notify();
    });
}

GeoZoneController::~GeoZoneController()
{
    g_shown.reset();
    if (s_instance == this) s_instance = nullptr;
}

void GeoZoneController::setEnabled(bool on)
{
    if (on == m_enabled) return;
    m_enabled = on;
    Settings::instance().setValue(u"geozones-enabled"_s, on ? u"1"_s : u"0"_s);
    LOG_I("geozones") << (on ? "on" : "off");
    emit enabledChanged();
    if (on) {
        query();
    } else {
        m_zones.reset();
        m_sourcesText.clear();
        m_pending = 0;
        m_alerts.clear();
        m_overHeight = false;
        emit statusChanged();
        notify();
    }
}

bool GeoZoneController::vehiclePosition(double *lat, double *lon) const
{
    if (!m_hub) return false;
    const auto s = m_hub->snapshot();
    // The vehicle's position, when there is a real one: a link or a log, not the demo nor the relative frame
    if (s->isDemo() || !s->relativeSource.isEmpty()) return false;
    if (!s->connected && s->mode != "REPLAY"_L1) return false;
    if (!std::isfinite(s->lat) || !std::isfinite(s->lon) || (s->lat == 0 && s->lon == 0) || s->sceneOriginEpoch <= 0) return false;
    *lat = s->lat;
    *lon = s->lon;
    return true;
}

void GeoZoneController::query()
{
    if (!m_enabled) return;
    m_lastQueryAt = wallMs();
    QList<GeoZoneStore::Circle> circles;
    double vlat = 0, vlon = 0;
    m_haveCentre = vehiclePosition(&vlat, &vlon);
    if (m_haveCentre) {
        m_centreLat = vlat;
        m_centreLon = vlon;
        circles.append({vlat, vlon, kRadiusKm});
    }
    // One circle per mission point not already within 10 km of an earlier one
    if (m_plan) {
        for (const QJsonValue &seg : m_plan->route().value(u"segments"_s).toArray()) {
            for (const QJsonValue &pv : seg.toObject().value(u"points"_s).toArray()) {
                const QJsonObject p = pv.toObject();
                const double lat = p.value(u"lat"_s).toDouble(NAN), lon = p.value(u"lng"_s).toDouble(NAN);
                if (!std::isfinite(lat) || !std::isfinite(lon)) continue;
                bool near = false;
                for (const auto &c : circles)
                    if (kmBetween(c.lat, c.lon, lat, lon) < kMissionSpacingKm) { near = true; break; }
                if (!near) circles.append({lat, lon, kRadiusKm});
            }
        }
    }
    const GeoZoneStore::Result r = m_store->query(circles);
    m_zones = r.zones;
    m_pending = r.pending;
    QStringList lines;
    for (const auto &src : r.sources) {
        QString line = u"%1 (%2): %3 / %4 %5"_s.arg(src.name, src.country).arg(src.ready).arg(src.tiles).arg(src.tiles == 1 && src.id == "bazl"_L1 ? u"file"_s : u"tiles"_s);
        if (src.pending) line += u", %1 loading"_s.arg(src.pending);
        if (!src.error.isEmpty()) line += u" — "_s + src.error.left(80);
        lines << line;
    }
    if (lines.isEmpty()) lines << (circles.isEmpty() ? u"No vehicle and no mission: nothing to ask"_s : u"No online source covers this area: import the state's ED-269 file"_s);
    m_sourcesText = lines.join('\n'_L1);
    notify();
}

void GeoZoneController::notify()
{
    ++m_version;
    g_shown = m_enabled ? m_zones : nullptr;
    g_shownVersion = m_version;
    SceneItem::setGeoZones(m_enabled ? m_zones : nullptr, m_version, m_enabled);
    emit dataChanged();
}

QVariantList GeoZoneController::imports() const
{
    QVariantList out;
    for (const auto &i : m_store->imports())
        out << QVariantMap{{u"file"_s, i.file}, {u"name"_s, i.name}, {u"count"_s, i.count},
                           {u"date"_s, QDateTime::fromMSecsSinceEpoch(i.at).toString(u"yyyy-MM-dd"_s)}};
    return out;
}

void GeoZoneController::importFile(const QUrl &file)
{
    const QString path = file.isLocalFile() ? file.toLocalFile() : file.toString();
    QString error;
    const int n = m_store->importFile(path, &error);
    if (n < 0) {
        LOG_W("geozones") << "import of" << path << "failed:" << error;
        emit toast(u"Import failed: "_s + error);
        return;
    }
    emit toast(u"%1: %2 zones imported"_s.arg(QFileInfo(path).fileName()).arg(n));
    query();
}

void GeoZoneController::removeImport(const QString &file)
{
    m_store->removeImport(file);
    query();
}

void GeoZoneController::refresh()
{
    m_store->clear();
    query();
}

void GeoZoneController::silence(const QString &id)
{
    m_silenced.insert(id);
    evaluate();
}

void GeoZoneController::closeArmAlert()
{
    if (m_armAlert.isEmpty()) return;
    m_armAlert.clear();
    m_armTimer.stop();
    emit armAlertChanged();
}

void GeoZoneController::evaluate()
{
    auto clear = [this] {
        if (m_alerts.isEmpty() && !m_overHeight) return;
        m_alerts.clear();
        m_overHeight = false;
        emit statusChanged();
    };
    if (!m_enabled || !m_hub) {
        clear();
        m_wasArmed = -1;
        return;
    }
    double lat = 0, lon = 0;
    const bool have = vehiclePosition(&lat, &lon);
    // Follow the vehicle, refresh now and then
    if (have ? (!m_haveCentre || kmBetween(lat, lon, m_centreLat, m_centreLon) > kRequeryKm) : m_haveCentre) query();
    else if (wallMs() - m_lastQueryAt > kRefreshMs) query();

    const auto s = m_hub->snapshot();
    const QString type = modes::vehicleTypeName(s->vehicleType);
    if (!have || !m_zones || (type != "Copter"_L1 && type != "Plane"_L1)) {
        clear();
        m_wasArmed = -1;
        return;
    }
    const double alt = s->rawAlt + s->offsetAlt;
    std::optional<double> terrain = s->terrainHeight;
    if (!terrain) terrain = HgtStore::instance().elevation(lat, lon);
    std::optional<double> agl;
    if (terrain) agl = alt - *terrain;
    else if (s->homeAlt) agl = alt - *s->homeAlt;
    // A floor above sea level needs the ground under the vehicle: home's when the terrain is not loaded
    const std::optional<double> ground = terrain ? terrain : s->homeAlt;
    const geozone::Evaluation e = geozone::evaluate(*m_zones, lat, lon, agl, ground, s->armed || (agl && *agl > kAirborneAgl));

    QVariantList alerts;
    for (const geozone::Alert &a : e.alerts) {
        if (m_silenced.contains(a.id)) continue;
        alerts << QVariantMap{{u"id"_s, a.id}, {u"level"_s, a.level}, {u"icon"_s, a.icon},
                              {u"label"_s, a.label}, {u"value"_s, a.value}, {u"title"_s, alertTitle(a)}};
    }
    if (alerts != m_alerts || e.overHeight != m_overHeight) {
        m_alerts = alerts;
        m_overHeight = e.overHeight;
        emit statusChanged();
    }

    // Armed inside a zone: the banner and the HUD line
    if (m_wasArmed == 0 && s->armed) {
        if (const auto a = geozone::armingAlert(e.here, ground, e.limit)) {
            m_armAlert = QVariantMap{{u"level"_s, a->level}, {u"title"_s, a->title}, {u"detail"_s, a->detail}};
            m_armTimer.start();
            emit armAlertChanged();
            emit hudMessage(a->title + u": "_s + a->detail, a->level == "warn"_L1 ? u"error"_s : u"warning"_s);
            if (a->level == "warn"_L1) LOG_W("geozones") << a->title << ":" << a->detail;
            else LOG_I("geozones") << a->title << ":" << a->detail;
        }
    }
    m_wasArmed = s->armed ? 1 : 0;
}

} // namespace corv
