#include "ui/GcsController.h"

#include "services/AdsbClient.h"
#include "ui/RtkController.h"
#include "ui/TelForwardController.h"
#include "ui/UpdateController.h"
#include "ui/LteController.h"
#include "ui/JoystickController.h"
#include "ui/SetupController.h"
#include "ui/LidarController.h"
#include "ui/RosController.h"
#include "ui/SitlController.h"

#include "core/DebugLog.h"
#include "core/Settings.h"
#include "link/LinkManager.h"
#include "link/MessageBus.h"
#include "link/SerialTransport.h"
#include "gpu/SceneItem.h"
#include "terrain/Hgt.h"
#include "terrain/HgtDownloader.h"
#include "terrain/TileStore.h"
#include "services/LogReplay.h"
#include "ui/AnnunciatorController.h"
#include "ui/GeoZoneController.h"
#include "ui/CalibrationController.h"
#include "ui/EkfController.h"
#include "ui/FftController.h"
#include "ui/FpvController.h"
#include "ui/ReplayController.h"
#include "ui/DialogHub.h"
#include "ui/HudItem.h"
#include "ui/MapItem.h"
#include "ui/LogsController.h"
#include "ui/ParamsController.h"
#include "ui/PlanController.h"
#include "ui/RtlOptions.h"
#include "ui/SysConfig.h"
#include "ui/TargetController.h"
#include "ui/VehicleModel.h"
#include "vehicle/CommandSender.h"
#include "vehicle/ConnectionManager.h"
#include "vehicle/FlightModes.h"
#include "vehicle/ParamStore.h"
#include "vehicle/TerrainFeeder.h"
#include "vehicle/VehicleHub.h"

#include <QJsonObject>
#include <QTime>
#include <cmath>

using namespace Qt::StringLiterals;

namespace corv {

GcsController *GcsController::s_instance = nullptr;

GcsController::GcsController(QObject *parent) : QObject(parent), m_bus(std::make_unique<MessageBus>())
{
    s_instance = this;
    startLink();
    createViewModels();
    createTerrainFeed();
    wireViewModels();
    wireLink();
}

void GcsController::startLink()
{
    // The link manager and the vehicle model own the link thread
    m_linkThread.setObjectName(u"link"_s);
    m_link = new LinkManager(m_bus.get());
    m_hub = new VehicleHub(m_link);
    m_link->moveToThread(&m_linkThread);
    m_hub->moveToThread(&m_linkThread);
    connect(&m_linkThread, &QThread::finished, m_link, &QObject::deleteLater);
    connect(&m_linkThread, &QThread::finished, m_hub, &QObject::deleteLater);
    m_linkThread.start(QThread::HighPriority);

    m_cmd = new CommandSender(m_link, m_bus.get(), m_hub, this);
    m_conn = new ConnectionManager(m_link, m_hub, m_bus.get(), m_cmd, this);
    m_vehicle = new VehicleModel(m_hub, this);
    HudItem::setHub(m_hub);
    SceneItem::setHub(m_hub);
    MapItem::setHub(m_hub);
    // CORV_DEMO_ORIGIN=lat,lon: the demo somewhere else than Innsbruck (tests: water, sea)
    if (const QStringList o = qEnvironmentVariable("CORV_DEMO_ORIGIN").split(','_L1); o.size() == 2) {
        const double lat = o[0].toDouble(), lon = o[1].toDouble();
        m_hub->mutate([lat, lon](VehicleState &s) { s.lat = lat; s.lon = lon; });
        m_hub->setSceneOrigin(lat, lon);
    }
    // GCS output muted as it was left (gcs-muted, as Electron kept it)
    if (Settings::instance().value(u"gcs-muted"_s).toString() == "1"_L1) m_link->setGcsMuted(true);
    // What a sub's depth is measured from: the lake or the sea there, else the ground
    m_hub->setSurface([](double lat, double lon) -> std::optional<double> {
        if (const auto w = HgtStore::instance().waterAt(lat, lon)) return w->level;
        return HgtStore::instance().elevation(lat, lon);
    });
    // No link: the demo racetrack around the scene origin, on the real terrain
    m_hub->startDemo([](double lat, double lon) { return HgtStore::instance().elevation(lat, lon); });
    // Battery gauge when the vehicle reports no percentage (SYS CONFIG -> BATTERY GAUGE)
    SysConfig::loadBatteryGauge(m_hub);
}

void GcsController::createViewModels()
{
    // The FLIGHT PLAN and SYS CONFIG pages (QML singletons Plan and SysConfig)
    m_sysConfig = new SysConfig(m_hub, m_cmd, this);
    m_plan = new PlanController(m_hub, m_cmd, this);
    // In-page dialogs for every view model, and SETUP → PARAMETERS (QML singletons DialogHub, Params)
    m_dialogs = new DialogHub(this);
    m_params = new ParamStore(m_link, m_bus.get(), m_cmd, m_hub, this);
    m_paramsUi = new ParamsController(m_params, m_hub, this);
    // SETUP → LOG DOWNLOAD: the DataFlash logs on the vehicle's SD card (QML singleton Logs)
    m_logs = new LogsController(m_link, m_bus.get(), m_cmd, m_hub, this);
    // FLIGHT MODES, FAILSAFE, PID / EXT TUNING, RADIO CAL, VIBRATIONS, SERVO / RELAY (QML singleton SetupCfg)
    m_setup = new SetupController(m_params, m_cmd, m_hub, this);
    // SETUP → SIMULATION: the ArduPilot SITL launcher (QML singleton Sitl)
    m_sitl = new SitlController(m_conn, m_sysConfig, this);
    // SETUP → ACCELEROMETER, COMPASS, GYRO / BARO (QML singleton Calibration)
    m_calibration = new CalibrationController(m_bus.get(), m_cmd, m_params, m_hub, this);
    // The flight screen: EKF STATUS and FFT bars, annunciators (QML singletons Ekf, Fft, Annunciators)
    m_ekf = new EkfController(m_bus.get(), m_link, m_cmd, m_hub, this);
    m_fft = new FftController(m_bus.get(), this);
    m_annun = new AnnunciatorController(m_hub, this);
    // UAS geographical zones: maps, terrain, height warnings (QML singleton GeoZones)
    m_geo = new GeoZoneController(m_hub, m_plan, this);
    // SETUP → RTK / GPS INJECT: RTCM corrections to the vehicle (QML singleton Rtk)
    m_rtk = new RtkController(m_link, m_hub, this);
    // SETUP → TELEMETRY FORWARD: MAVLink mirror and LTM outputs (QML singleton TelForward)
    m_telForward = new TelForwardController(m_link, m_hub, this);
    // SETUP → CELLULAR LINK and the LTE cell readouts (QML singleton Lte)
    m_lte = new LteController(m_link, m_conn, m_cmd, m_hub, this);
    m_joystick = new JoystickController(m_cmd, m_hub, this);
    // SETUP → LIDAR: the Livox point cloud (QML singleton Lidar); SETUP → ROS: the
    // rosbridge surface (QML singleton Ros). Each client on a thread of its own
    m_lidar = new LidarController(m_bus.get(), m_hub, this);
    m_ros = new RosController(m_hub, m_bus.get(), m_plan, this);
    // ADS-B traffic from OpenSky while SHOW ADS-B TRAFFIC is on (on by default, adsb-enabled)
    m_adsb = new AdsbClient(m_hub, this);
    m_adsb->setEnabled(m_sysConfig->adsbEnabled());
    // LOG REPLAY: the engine on the log thread (QML singleton Replay)
    m_logThread.setObjectName(u"log"_s);
    m_replayEngine = new LogReplay(m_link, m_hub);
    m_replayEngine->moveToThread(&m_logThread);
    connect(&m_logThread, &QThread::finished, m_replayEngine, &QObject::deleteLater);
    m_logThread.start();
    m_replay = new ReplayController(m_replayEngine, m_fft, this);
    // GCS OPTIONS: the reference point and the RTL parameters (QML singletons Target, Rtl)
    m_target = new TargetController(m_vehicle, this);
    m_rtl = new RtlOptions(m_params, m_hub, this);
    // FPV camera: the RTSP stream on the video thread (QML singleton Fpv)
    m_fpv = new FpvController(this);
    // The update check at start and the one-click install (QML singleton Updates)
    m_updates = new UpdateController(m_hub, this);
}

void GcsController::createTerrainFeed()
{
    // The vehicle's own terrain database (TERRAIN_ENABLE): its requests are answered from the SRTM tiles
    auto height = [](double lat, double lon, double &metres) {
        HgtStore &store = HgtStore::instance();
        const int la = int(std::floor(lat)), lo = int(std::floor(lon));
        if (const auto tile = store.get(la, lo)) {
            const auto h = tile->elevation(lat, lon);
            if (!h) return TerrainFeeder::Lookup::Missing;   // a void
            metres = *h;
            return TerrainFeeder::Lookup::Found;
        }
        if (!store.missing(la, lo)) return TerrainFeeder::Lookup::Pending;   // being read
        HgtDownloader::instance().ensure(la, lo);
        return HgtDownloader::instance().busy(la, lo) ? TerrainFeeder::Lookup::Pending : TerrainFeeder::Lookup::Missing;
    };
    m_terrainFeed = new TerrainFeeder(m_bus.get(), height, [link = m_link](const mavlink_message_t &m) { link->send(m); }, this);
    m_terrainFeed->setMuted(gcsMuted());
    connect(this, &GcsController::gcsMutedChanged, m_terrainFeed, [this] { m_terrainFeed->setMuted(gcsMuted()); });
    connect(m_link, &LinkManager::connected, m_terrainFeed, &TerrainFeeder::reset);
    connect(m_terrainFeed, &TerrainFeeder::hudMessage, this, &GcsController::hudMessage);
    auto show = [this] {
        const TerrainFeeder::Stats &s = m_terrainFeed->stats();
        QString t = u"No request from the vehicle"_s;
        if (s.sent || s.missing || s.queued || s.pending > 0 || s.loaded > 0) {
            t = u"%1 blocks sent"_s.arg(s.sent);
            if (s.missing) t += u" · %1 without elevation data"_s.arg(s.missing);
            if (s.pending >= 0) t += u" · vehicle: %1 pending, %2 loaded"_s.arg(s.pending).arg(s.loaded);
        }
        if (t == m_terrainText) return;
        m_terrainText = t;
        DebugLog::setHeader(u"Terrain feed"_s, t);
        emit terrainFeedChanged();
    };
    connect(m_terrainFeed, &TerrainFeeder::statsChanged, this, show);
    show();
}

void GcsController::wireViewModels()
{
    connect(m_geo, &GeoZoneController::hudMessage, this, &GcsController::hudMessage);
    connect(m_lidar, &LidarController::hudMessage, this, &GcsController::hudMessage);
    connect(m_ros, &RosController::hudMessage, this, &GcsController::hudMessage);
    connect(m_sysConfig, &SysConfig::adsbChanged, this, [this] {
        const bool on = m_sysConfig->adsbEnabled();
        if (on == m_adsb->enabled()) return;
        m_adsb->setEnabled(on);
        emit hudMessage(on ? u"ADS-B traffic enabled"_s : u"ADS-B traffic disabled"_s, u"info"_s);
    });
    // No answer from the tile servers: only the cached imagery until they are back (enterCacheOnlyMode)
    connect(&TileStore::instance(), &TileStore::networkLost, this, [this] {
        emit hudMessage(u"[WARNING] CONNECTION LOST — satellite from offline cache only, uncached areas shown without imagery"_s, u"warning"_s);
    });
    connect(&TileStore::instance(), &TileStore::networkRestored, this, [this] {
        emit hudMessage(u"Connection restored — satellite maps back online"_s, u"info"_s);
    });
    // One bar at a time on the left: opening one closes the other (FFT wins at start, as in Electron)
    if (m_ekf->enabled() && m_fft->enabled()) m_ekf->setEnabled(false);
    connect(m_ekf, &EkfController::enabledChanged, this, [this] { if (m_ekf->enabled()) m_fft->setEnabled(false); });
    connect(m_fft, &FftController::enabledChanged, this, [this] { if (m_fft->enabled()) m_ekf->setEnabled(false); });
    connect(m_sysConfig, &SysConfig::jsonFieldChanged, this, [this](const QString &key) {
        if (key == "log-fft-cfg"_L1) m_replay->applyFftConfig();
        if (key == "fpv-settings"_L1) m_fpv->settingsChanged();
    });
}

void GcsController::wireLink()
{
    connect(m_link, &LinkManager::connectionState, this, [this](const QString &st) {
        m_state = st;
        if (st == "CONNECTED"_L1) {
            m_error.clear();
            // A new link is a new vehicle: the charts and spectra start empty
            m_ekf->reset();
            m_fft->reset();
        }
        m_replay->setConnectionState(st);
        emit connectionStateChanged();
    });
    connect(m_conn, &ConnectionManager::connectFailed, this, [this](const QString &, const QString &e) {
        m_state = u"DISCONNECTED"_s;
        m_error = e;
        m_replay->setConnectionState(m_state);
        emit connectionStateChanged();
        emit hudMessage(u"Connection failed: %1"_s.arg(e), u"error"_s);
    });
    // No bytes, or bytes that never make a frame: said on the HUD, not only in the log
    connect(m_link, &LinkManager::linkWarning, this, [this](const QString &text) { emit hudMessage(text, u"warning"_s); });
    connect(m_hub, &VehicleHub::commandAck, this, [this](int cmd, int result) {
        static const QHash<int, const char *> names = {{22, "TAKEOFF"}, {21, "LAND"}, {176, "SET_MODE"}, {400, "ARM/DISARM"},
                                                       {178, "CHANGE_SPEED"}, {179, "SET_HOME"}, {246, "REBOOT"}, {241, "CALIBRATION"},
                                                       {20, "RTL"}, {42424, "START_MAG_CAL"}, {42425, "ACCEPT_MAG_CAL"},
                                                       {42426, "CANCEL_MAG_CAL"}, {42429, "ACCELCAL_POSITION"}, {42006, "FIXED_MAG_CAL_YAW"}};
        const QString n = names.contains(cmd) ? QString::fromLatin1(names.value(cmd)) : u"CMD_%1"_s.arg(cmd);
        emit hudMessage(u"%1: %2"_s.arg(n, mav::resultName(uint8_t(result))), result == 0 ? u"info"_s : u"warning"_s);
    });
    connect(m_hub, &VehicleHub::statusText, this, [this](int sev, const QString &t) {
        emit hudMessage(t, sev <= 3 ? u"error"_s : sev <= 4 ? u"warning"_s : u"info"_s);
        m_log << QTime::currentTime().toString(u"HH:mm:ss "_s) + (sev <= 3 ? u"[ERR] "_s : sev <= 4 ? u"[WRN] "_s : QString()) + t;
        while (m_log.size() > 2000) m_log.removeFirst();
        emit messageLogChanged();
    });
    connect(m_hub, &VehicleHub::lastHomeChanged, this, [](double lat, double lon) {
        Settings::instance().setJson(u"corv.lastHome"_s, QJsonObject{{u"lat"_s, lat}, {u"lng"_s, lon}});
    });
}

GcsController::~GcsController()
{
    // The replay injects into the link: its thread goes first
    m_logThread.quit();
    m_logThread.wait(3000);
    m_link->disconnectLink();
    m_linkThread.quit();
    m_linkThread.wait(3000);
}

QString GcsController::version() const { return QStringLiteral(CORV_VERSION); }
bool GcsController::gcsMuted() const { return m_link->gcsMuted(); }

void GcsController::setGcsMuted(bool m)
{
    if (m == gcsMuted()) return;
    m_link->setGcsMuted(m);
    Settings::instance().setValue(u"gcs-muted"_s, m ? u"1"_s : u"0"_s);
    emit gcsMutedChanged();
    emit hudMessage(m ? u"GCS output MUTED — no messages will be sent"_s : u"GCS output UNMUTED"_s, m ? u"warning"_s : u"info"_s);
}

void GcsController::clearMessageLog()
{
    m_log.clear();
    emit messageLogChanged();
}

QVariantList GcsController::serialPorts() const
{
    QVariantList out;
    for (const auto &p : SerialTransport::listPorts())
        out << QVariantMap{{u"path"_s, p.path},
                           {u"label"_s, p.friendlyName.isEmpty() ? p.path : u"%1  %2"_s.arg(p.path, p.friendlyName)}};
    return out;
}

void GcsController::connectLink(const QString &type, const QString &address, int portOrBaud)
{
    m_state = u"CONNECTING"_s;
    m_error.clear();
    m_replay->setConnectionState(m_state);
    emit connectionStateChanged();
    QVariantMap o;
    if (type == "TCP"_L1 || type == "UDP"_L1) {
        o[u"host"_s] = address;
        o[u"port"_s] = portOrBaud;
        m_conn->connectLink(type == "TCP"_L1 ? QLatin1String(LinkManager::kMavlinkTcp) : QLatin1String(LinkManager::kMavlinkUdp), o);
    } else {
        o[u"port"_s] = address;
        o[u"baudRate"_s] = portOrBaud;
        m_conn->connectLink(QLatin1String(LinkManager::kMavlinkSerial), o);
    }
}

void GcsController::disconnectLink() { m_conn->disconnectLink(); }

void GcsController::connectType(const QString &type, const QString &address, int portOrBaud)
{
    if (type == "mavlink-tcp"_L1) connectLink(u"TCP"_s, address, portOrBaud);
    else if (type == "mavlink-udp"_L1) connectLink(u"UDP"_s, address, portOrBaud);
    else if (type == "mavlink-serial"_L1) connectLink(u"SERIAL"_s, address, portOrBaud);
    else if (type == "msp-serial"_L1 || type == "msp-tcp"_L1) {
        // MSP (INAV / Betaflight); the poll profile from SETUP -> CONNECTION (msp-profile: normal | slow)
        m_state = u"CONNECTING"_s;
        m_error.clear();
        m_replay->setConnectionState(m_state);
        emit connectionStateChanged();
        QVariantMap o;
        o[u"profile"_s] = Settings::instance().value(u"msp-profile"_s, u"normal"_s).toString();
        if (type == "msp-tcp"_L1) {
            o[u"host"_s] = address;
            o[u"port"_s] = portOrBaud;
        } else {
            o[u"port"_s] = address;
            o[u"baudRate"_s] = portOrBaud;
        }
        m_conn->connectLink(type, o);
    } else {
        m_error = u"%1 is not ported yet (phase 6)"_s.arg(type);
        emit connectionStateChanged();
    }
}

QString GcsController::setting(const QString &key, const QString &fallback) const
{
    return Settings::instance().value(key, fallback).toString();
}

void GcsController::setSetting(const QString &key, const QString &value) { Settings::instance().setValue(key, value); }

void GcsController::setHomeHere()
{
    DialogHub::confirm(u"Set HOME to the vehicle's current position?"_s, [this](bool ok) {
        if (!ok) return;
        m_cmd->setHome(std::nullopt, std::nullopt, std::nullopt, [](bool got, const CommandSender::Ack &ack) {
            if (!got) DialogHub::toast(u"Set HOME: no answer from the autopilot"_s);
            else if (ack.result == MAV_RESULT_ACCEPTED) DialogHub::toast(u"HOME set to the current position"_s);
            else DialogHub::toast(u"Set HOME rejected by the autopilot (%1): it needs a 3D GPS fix and a healthy EKF"_s.arg(ack.resultName));
        });
    });
}

void GcsController::rebootAutopilot()
{
    if (m_hub->snapshot()->armed) {
        DialogHub::toast(u"The vehicle is armed: no reboot"_s);
        return;
    }
    DialogHub::confirm(u"Reboot the autopilot? The link drops and comes back when it has restarted."_s, [this](bool ok) {
        if (ok && !m_hub->snapshot()->armed) m_cmd->rebootAutopilot();
    });
}

void GcsController::setMissionSpeed(double speed) { m_cmd->setMissionSpeed(speed); }
void GcsController::arm(bool force) { m_cmd->armVehicle(force); }
void GcsController::disarm(bool force) { m_cmd->disarmVehicle(force); }

void GcsController::setMode(const QString &mode)
{
    if (!m_cmd->setFlightMode(mode)) DialogHub::toast(u"%1 is not a mode of this vehicle"_s.arg(mode));
}


void GcsController::takeoff(double altitude)
{
    m_cmd->takeoff(altitude, [](bool ok, const QString &why) {
        if (!ok) DialogHub::toast(u"Take-off: %1"_s.arg(why));
    });
}

void GcsController::askTakeoff()
{
    const auto s = m_hub->snapshot();
    const modes::Table t = modes::tableFor(s->vehicleType, s->connectionType);
    if (t != modes::Table::Copter && t != modes::Table::Plane) {
        DialogHub::toast(u"TAKEOFF: not for this vehicle"_s);
        return;
    }
    const bool fixedWing = t == modes::Table::Plane && !modes::isVtolType(s->vehicleType);
    DialogHub::prompt(fixedWing ? u"Take-off height above ground (m). The vehicle goes to TAKEOFF, arms and starts its run at once."_s
                                : u"Take-off height above ground (m). The vehicle goes to GUIDED, arms and climbs at once."_s, u"10"_s,
                      [this](bool ok, const QString &text) {
                          if (!ok) return;
                          bool number = false;
                          const double alt = QString(text).trimmed().replace(','_L1, '.'_L1).toDouble(&number);
                          if (number && alt > 0 && alt <= 1000) takeoff(alt);
                          else DialogHub::toast(u"Take-off height not valid: "_s + text);
                      });
}

void GcsController::land()
{
    // What lands it: a mode on a copter or a VTOL plane, the landing sequence of the mission
    // on a fixed wing (denied when there is none), the surface for a ROV
    const auto s = m_hub->snapshot();
    switch (modes::tableFor(s->vehicleType, s->connectionType)) {
    case modes::Table::Plane:
        if (modes::isVtolType(s->vehicleType)) setMode(u"QLAND"_s);
        else m_cmd->landSequence();
        break;
    case modes::Table::Sub: setMode(u"SURFACE"_s); break;
    default: setMode(u"LAND"_s); break;
    }
}

void GcsController::returnToLaunch() { setMode(u"RTL"_s); }

} // namespace corv
