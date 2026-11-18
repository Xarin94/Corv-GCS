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

#include "core/DataRoot.h"
#include "core/DebugLog.h"
#include "core/Settings.h"
#include "link/LinkManager.h"
#include "link/MessageBus.h"
#include "link/SerialTransport.h"
#include "gpu/SceneItem.h"
#include "terrain/Hgt.h"
#include "services/LogReplay.h"
#include "ui/AnnunciatorController.h"
#include "ui/GeoZoneController.h"
#include "ui/CalibrationController.h"
#include "ui/EkfController.h"
#include "ui/FftController.h"
#include "ui/ReplayController.h"
#include "ui/DialogHub.h"
#include "ui/HudItem.h"
#include "ui/MapItem.h"
#include "ui/LogsController.h"
#include "ui/ParamsController.h"
#include "ui/PlanController.h"
#include "ui/SysConfig.h"
#include "ui/VehicleModel.h"
#include "vehicle/CommandSender.h"
#include "vehicle/ConnectionManager.h"
#include "vehicle/ParamStore.h"
#include "vehicle/VehicleHub.h"

#include <QJsonObject>
#include <QTime>

namespace corv {

GcsController *GcsController::s_instance = nullptr;

GcsController::GcsController(QObject *parent) : QObject(parent), m_bus(std::make_unique<MessageBus>())
{
    s_instance = this;
    m_linkThread.setObjectName(QStringLiteral("link"));
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
    if (const QStringList o = qEnvironmentVariable("CORV_DEMO_ORIGIN").split(QLatin1Char(',')); o.size() == 2) {
        const double lat = o[0].toDouble(), lon = o[1].toDouble();
        m_hub->mutate([lat, lon](VehicleState &s) { s.lat = lat; s.lon = lon; });
        m_hub->setSceneOrigin(lat, lon);
    }
    // GCS output muted as it was left (gcs-muted, as Electron kept it)
    if (Settings::instance().value(QStringLiteral("gcs-muted")).toString() == QLatin1String("1")) m_link->setGcsMuted(true);
    // No link: the demo racetrack around the scene origin, on the real terrain
    m_hub->startDemo([](double lat, double lon) { return HgtStore::instance().elevation(lat, lon); });

    // Battery range when the vehicle reports no percentage (SYS CONFIG; localStorage bat-vmin / bat-vmax)
    m_hub->setBatteryRange(Settings::instance().value(QStringLiteral("bat-vmin"), 9.6).toDouble(),
                           Settings::instance().value(QStringLiteral("bat-vmax"), 12.6).toDouble());
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
    connect(m_geo, &GeoZoneController::hudMessage, this, &GcsController::hudMessage);
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
    connect(m_lidar, &LidarController::hudMessage, this, &GcsController::hudMessage);
    m_ros = new RosController(m_hub, m_bus.get(), m_plan, this);
    connect(m_ros, &RosController::hudMessage, this, &GcsController::hudMessage);
    // ADS-B traffic from OpenSky while SHOW ADS-B TRAFFIC is on (on by default, adsb-enabled)
    m_adsb = new AdsbClient(m_hub, this);
    m_adsb->setEnabled(m_sysConfig->adsbEnabled());
    connect(m_sysConfig, &SysConfig::adsbChanged, this, [this] {
        const bool on = m_sysConfig->adsbEnabled();
        if (on == m_adsb->enabled()) return;
        m_adsb->setEnabled(on);
        emit hudMessage(on ? QStringLiteral("ADS-B traffic enabled") : QStringLiteral("ADS-B traffic disabled"), QStringLiteral("info"));
    });
    // One bar at a time on the left: opening one closes the other (FFT wins at start, as in Electron)
    if (m_ekf->enabled() && m_fft->enabled()) m_ekf->setEnabled(false);
    connect(m_ekf, &EkfController::enabledChanged, this, [this] { if (m_ekf->enabled()) m_fft->setEnabled(false); });
    connect(m_fft, &FftController::enabledChanged, this, [this] { if (m_fft->enabled()) m_ekf->setEnabled(false); });
    // LOG REPLAY: the engine on the log thread (QML singleton Replay)
    m_logThread.setObjectName(QStringLiteral("log"));
    m_replayEngine = new LogReplay(m_link, m_hub);
    m_replayEngine->moveToThread(&m_logThread);
    connect(&m_logThread, &QThread::finished, m_replayEngine, &QObject::deleteLater);
    m_logThread.start();
    m_replay = new ReplayController(m_replayEngine, m_fft, this);
    // The update check at start and the one-click install (QML singleton Updates)
    m_updates = new UpdateController(m_hub, this);
    connect(m_sysConfig, &SysConfig::jsonFieldChanged, this, [this](const QString &key) {
        if (key == QLatin1String("log-fft-cfg")) m_replay->applyFftConfig();
    });

    connect(m_link, &LinkManager::connectionState, this, [this](const QString &st) {
        m_state = st;
        if (st == QLatin1String("CONNECTED")) {
            m_error.clear();
            // A new link is a new vehicle: the charts and spectra start empty
            m_ekf->reset();
            m_fft->reset();
        }
        m_replay->setConnectionState(st);
        emit connectionStateChanged();
    });
    connect(m_conn, &ConnectionManager::connectFailed, this, [this](const QString &, const QString &e) {
        m_state = QStringLiteral("DISCONNECTED");
        m_error = e;
        m_replay->setConnectionState(m_state);
        emit connectionStateChanged();
        emit hudMessage(QStringLiteral("Connection failed: %1").arg(e), QStringLiteral("error"));
    });
    // No bytes, or bytes that never make a frame: said on the HUD, not only in the log
    connect(m_link, &LinkManager::linkWarning, this, [this](const QString &text) { emit hudMessage(text, QStringLiteral("warning")); });
    connect(m_link, &LinkManager::tlogChanged, this, [this](const QString &p) {
        m_tlog = p;
        emit tlogPathChanged();
    });
    connect(m_hub, &VehicleHub::commandAck, this, [this](int cmd, int result) {
        static const QHash<int, const char *> names = {{22, "TAKEOFF"}, {21, "LAND"}, {176, "SET_MODE"}, {400, "ARM/DISARM"},
                                                       {178, "CHANGE_SPEED"}, {179, "SET_HOME"}, {246, "REBOOT"}, {241, "CALIBRATION"},
                                                       {20, "RTL"}, {42424, "START_MAG_CAL"}, {42425, "ACCEPT_MAG_CAL"},
                                                       {42426, "CANCEL_MAG_CAL"}, {42429, "ACCELCAL_POSITION"}, {42006, "FIXED_MAG_CAL_YAW"}};
        const QString n = names.contains(cmd) ? QString::fromLatin1(names.value(cmd)) : QStringLiteral("CMD_%1").arg(cmd);
        emit hudMessage(QStringLiteral("%1: %2").arg(n, mav::resultName(uint8_t(result))), result == 0 ? QStringLiteral("info") : QStringLiteral("warning"));
    });
    connect(m_hub, &VehicleHub::statusText, this, [this](int sev, const QString &t) {
        emit hudMessage(t, sev <= 3 ? QStringLiteral("error") : sev <= 4 ? QStringLiteral("warning") : QStringLiteral("info"));
        m_log << QTime::currentTime().toString(QStringLiteral("HH:mm:ss ")) + t;
        while (m_log.size() > 200) m_log.removeFirst();
        emit messageLogChanged();
    });
    connect(m_hub, &VehicleHub::lastHomeChanged, this, [](double lat, double lon) {
        Settings::instance().setJson(QStringLiteral("corv.lastHome"), QJsonObject{{QStringLiteral("lat"), lat}, {QStringLiteral("lng"), lon}});
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
QString GcsController::dataRoot() const { return DataRoot::root(); }
bool GcsController::gcsMuted() const { return m_link->gcsMuted(); }

void GcsController::setGcsMuted(bool m)
{
    if (m == gcsMuted()) return;
    m_link->setGcsMuted(m);
    Settings::instance().setValue(QStringLiteral("gcs-muted"), m ? QStringLiteral("1") : QStringLiteral("0"));
    emit gcsMutedChanged();
}

QVariantList GcsController::serialPorts() const
{
    QVariantList out;
    for (const auto &p : SerialTransport::listPorts())
        out << QVariantMap{{QStringLiteral("path"), p.path},
                           {QStringLiteral("label"), p.friendlyName.isEmpty() ? p.path : QStringLiteral("%1  %2").arg(p.path, p.friendlyName)}};
    return out;
}

void GcsController::connectLink(const QString &type, const QString &address, int portOrBaud)
{
    m_state = QStringLiteral("CONNECTING");
    m_error.clear();
    m_replay->setConnectionState(m_state);
    emit connectionStateChanged();
    QVariantMap o;
    if (type == QLatin1String("TCP") || type == QLatin1String("UDP")) {
        o[QStringLiteral("host")] = address;
        o[QStringLiteral("port")] = portOrBaud;
        m_conn->connectLink(type == QLatin1String("TCP") ? QLatin1String(LinkManager::kMavlinkTcp) : QLatin1String(LinkManager::kMavlinkUdp), o);
    } else {
        o[QStringLiteral("port")] = address;
        o[QStringLiteral("baudRate")] = portOrBaud;
        m_conn->connectLink(QLatin1String(LinkManager::kMavlinkSerial), o);
    }
}

void GcsController::disconnectLink() { m_conn->disconnectLink(); }

void GcsController::connectType(const QString &type, const QString &address, int portOrBaud)
{
    if (type == QLatin1String("mavlink-tcp")) connectLink(QStringLiteral("TCP"), address, portOrBaud);
    else if (type == QLatin1String("mavlink-udp")) connectLink(QStringLiteral("UDP"), address, portOrBaud);
    else if (type == QLatin1String("mavlink-serial")) connectLink(QStringLiteral("SERIAL"), address, portOrBaud);
    else if (type == QLatin1String("msp-serial") || type == QLatin1String("msp-tcp")) {
        // MSP (INAV / Betaflight); the poll profile from SETUP -> CONNECTION (msp-profile: normal | slow)
        m_state = QStringLiteral("CONNECTING");
        m_error.clear();
        m_replay->setConnectionState(m_state);
        emit connectionStateChanged();
        QVariantMap o;
        o[QStringLiteral("profile")] = Settings::instance().value(QStringLiteral("msp-profile"), QStringLiteral("normal")).toString();
        if (type == QLatin1String("msp-tcp")) {
            o[QStringLiteral("host")] = address;
            o[QStringLiteral("port")] = portOrBaud;
        } else {
            o[QStringLiteral("port")] = address;
            o[QStringLiteral("baudRate")] = portOrBaud;
        }
        m_conn->connectLink(type, o);
    } else {
        m_error = QStringLiteral("%1 is not ported yet (phase 6)").arg(type);
        emit connectionStateChanged();
    }
}

QString GcsController::setting(const QString &key, const QString &fallback) const
{
    return Settings::instance().value(key, fallback).toString();
}

void GcsController::setSetting(const QString &key, const QString &value) { Settings::instance().setValue(key, value); }
void GcsController::setHomeHere() { m_cmd->setHome(std::nullopt, std::nullopt, std::nullopt); }
void GcsController::rebootAutopilot() { m_cmd->rebootAutopilot(); }
void GcsController::setMissionSpeed(double speed) { m_cmd->setMissionSpeed(speed); }
void GcsController::arm(bool force) { m_cmd->armVehicle(force); }
void GcsController::disarm(bool force) { m_cmd->disarmVehicle(force); }
void GcsController::setMode(const QString &mode) { m_cmd->setFlightMode(mode); }
void GcsController::takeoff(double altitude) { m_cmd->takeoff(altitude); }
void GcsController::land() { m_cmd->land(); }
void GcsController::returnToLaunch() { m_cmd->returnToLaunch(); }
void GcsController::setGuidedTarget(double lat, double lon, double alt) { m_cmd->setGuidedTarget(lat, lon, alt); }

} // namespace corv
