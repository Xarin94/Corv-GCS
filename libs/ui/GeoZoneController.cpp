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
        if (a.id == QLatin1String("noFly")) lines << QStringLiteral("No-fly or authorisation required from the surface");
        else if (a.id == QLatin1String("cond")) lines << QStringLiteral("Conditions apply in this zone");
        else lines << QStringLiteral("Height that may be flown here: %1 m above the ground").arg(qRound(a.metres));
        lines << QStringLiteral("Source: ") + geozone::sourceName(a.zone->src);
        if (!a.zone->info.isEmpty()) lines << (a.zone->info.size() > 200 ? a.zone->info.left(199) + QStringLiteral("…") : a.zone->info);
    } else if (!a.rule.isEmpty()) {
        lines << a.rule;
    }
    lines << QStringLiteral("Click to silence until the next connection");
    return lines.join(QLatin1Char('\n'));
}
} // namespace

GeoZonesPtr GeoZoneController::shownZones() { return g_shown; }
int GeoZoneController::shownVersion() { return g_shownVersion; }

GeoZoneController::GeoZoneController(VehicleHub *hub, PlanController *plan, QObject *parent)
    : QObject(parent), m_hub(hub), m_plan(plan), m_store(new GeoZoneStore(DataRoot::geozonesDir(), this))
{
    s_instance = this;
    m_enabled = Settings::instance().value(QStringLiteral("geozones-enabled"), QStringLiteral("0")).toString() == QLatin1String("1");
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
    Settings::instance().setValue(QStringLiteral("geozones-enabled"), on ? QStringLiteral("1") : QStringLiteral("0"));
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
    if (!s->connected && s->mode != QLatin1String("REPLAY")) return false;
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
        for (const QJsonValue &seg : m_plan->route().value(QStringLiteral("segments")).toArray()) {
            for (const QJsonValue &pv : seg.toObject().value(QStringLiteral("points")).toArray()) {
                const QJsonObject p = pv.toObject();
                const double lat = p.value(QStringLiteral("lat")).toDouble(NAN), lon = p.value(QStringLiteral("lng")).toDouble(NAN);
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
        QString line = QStringLiteral("%1 (%2): %3 / %4 %5").arg(src.name, src.country).arg(src.ready).arg(src.tiles).arg(src.tiles == 1 && src.id == QLatin1String("bazl") ? QStringLiteral("file") : QStringLiteral("tiles"));
        if (src.pending) line += QStringLiteral(", %1 loading").arg(src.pending);
        if (!src.error.isEmpty()) line += QStringLiteral(" — ") + src.error.left(80);
        lines << line;
    }
    if (lines.isEmpty()) lines << (circles.isEmpty() ? QStringLiteral("No vehicle and no mission: nothing to ask") : QStringLiteral("No online source covers this area: import the state's ED-269 file"));
    m_sourcesText = lines.join(QLatin1Char('\n'));
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
        out << QVariantMap{{QStringLiteral("file"), i.file}, {QStringLiteral("name"), i.name}, {QStringLiteral("count"), i.count},
                           {QStringLiteral("date"), QDateTime::fromMSecsSinceEpoch(i.at).toString(QStringLiteral("yyyy-MM-dd"))}};
    return out;
}

void GeoZoneController::importFile(const QUrl &file)
{
    const QString path = file.isLocalFile() ? file.toLocalFile() : file.toString();
    QString error;
    const int n = m_store->importFile(path, &error);
    if (n < 0) {
        LOG_W("geozones") << "import of" << path << "failed:" << error;
        emit toast(QStringLiteral("Import failed: ") + error);
        return;
    }
    emit toast(QStringLiteral("%1: %2 zones imported").arg(QFileInfo(path).fileName()).arg(n));
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
    if (!have || !m_zones || (type != QLatin1String("Copter") && type != QLatin1String("Plane"))) {
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
        alerts << QVariantMap{{QStringLiteral("id"), a.id}, {QStringLiteral("level"), a.level}, {QStringLiteral("icon"), a.icon},
                              {QStringLiteral("label"), a.label}, {QStringLiteral("value"), a.value}, {QStringLiteral("title"), alertTitle(a)}};
    }
    if (alerts != m_alerts || e.overHeight != m_overHeight) {
        m_alerts = alerts;
        m_overHeight = e.overHeight;
        emit statusChanged();
    }

    // Armed inside a zone: the banner and the HUD line
    if (m_wasArmed == 0 && s->armed) {
        if (const auto a = geozone::armingAlert(e.here, ground, e.limit)) {
            m_armAlert = QVariantMap{{QStringLiteral("level"), a->level}, {QStringLiteral("title"), a->title}, {QStringLiteral("detail"), a->detail}};
            m_armTimer.start();
            emit armAlertChanged();
            emit hudMessage(a->title + QStringLiteral(": ") + a->detail, a->level == QLatin1String("warn") ? QStringLiteral("error") : QStringLiteral("warning"));
            if (a->level == QLatin1String("warn")) LOG_W("geozones") << a->title << ":" << a->detail;
            else LOG_I("geozones") << a->title << ":" << a->detail;
        }
    }
    m_wasArmed = s->armed ? 1 : 0;
}

} // namespace corv
