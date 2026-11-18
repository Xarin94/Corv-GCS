#include "ui/LidarController.h"

#include "core/Clock.h"
#include "core/DebugLog.h"
#include "core/Geo.h"
#include "core/Settings.h"
#include "link/Mavlink.h"
#include "link/MessageBus.h"
#include "scene/PointCloudData.h"
#include "sensors/LidarCore.h"
#include "terrain/Hgt.h"
#include "vehicle/VehicleHub.h"

#include <QJsonDocument>
#include <QJsonObject>
#include <QLocale>
#include <QRandomGenerator>
#include <cmath>

namespace corv {

namespace {
const QString kStorageKey = QStringLiteral("lidar-config");

// Every setting with its default (the values of the Electron page)
const QVariantMap &defaults()
{
    static const QVariantMap d{{QStringLiteral("lidarIp"), QStringLiteral("192.168.1.12")},
                               {QStringLiteral("hostIp"), QStringLiteral("auto")},
                               {QStringLiteral("dataType"), 1},
                               {QStringLiteral("mountRoll"), 0.0},
                               {QStringLiteral("mountPitch"), 0.0},
                               {QStringLiteral("mountYaw"), 0.0},
                               {QStringLiteral("leverX"), 0.0},
                               {QStringLiteral("leverY"), 0.0},
                               {QStringLiteral("leverZ"), 0.0},
                               {QStringLiteral("lagMs"), 0.0},
                               {QStringLiteral("minRange"), 2.5},
                               {QStringLiteral("maxRange"), 70.0},
                               {QStringLiteral("voxel"), 0.25},
                               {QStringLiteral("maxPoints"), 3000000.0},
                               {QStringLiteral("dropNoise"), true},
                               {QStringLiteral("minFix"), 3},
                               {QStringLiteral("minSats"), 8.0},
                               {QStringLiteral("maxHdop"), 2.0},
                               {QStringLiteral("requireEkf"), true},
                               {QStringLiteral("liveSeconds"), 3.0},
                               {QStringLiteral("colorMode"), QStringLiteral("height")},
                               {QStringLiteral("pointSize"), 2.0},
                               {QStringLiteral("overTerrain"), true},
                               {QStringLiteral("recordRaw"), false}};
    return d;
}
// Display-only keys stay here; the others go to the client
const QSet<QString> kLocal{QStringLiteral("colorMode"), QStringLiteral("pointSize"), QStringLiteral("overTerrain")};

QString fmt(qint64 n) { return QLocale(QLocale::English).toString(n); }
QString esc(const QString &s) { return s.toHtmlEscaped(); }
QString span(const QString &cls, const QString &text)
{
    const char *c = cls == QLatin1String("ok") ? "#44ff44" : cls == QLatin1String("warn") ? "#ffaa00" : "#ff4444";
    return QStringLiteral("<font color=\"%1\">%2</font>").arg(QLatin1String(c), text);
}

// Demo sensor: a Mid-360 band with its spin axis along the fuselage
constexpr double kElMin = -7 * kDeg, kElMax = 52 * kDeg;
constexpr double kRange = 150;            // m
constexpr double kPointsPerSec = 20000;
constexpr int kMaxPerFrame = 700;
constexpr double kDemoVoxel = 0.5;        // m
constexpr double kStepCoarse = 10;        // m, ray march against the terrain
constexpr double kBudgetMs = 2.5;         // per frame, whatever the ray count

double rnd() { return QRandomGenerator::global()->generateDouble(); }

// False echoes off the airframe, the reason MIN RANGE exists: with the spin
// axis along the fuselage the fan grazes the wing roots (sideways, in the
// plane of the disc) and the landing gear (down, slightly aft). The range of
// such a hit for this ray, or -1.
double airframeHit(double az, double el)
{
    const double side = std::abs(std::abs(az) - kPi / 2);   // 0 = along a wing
    if (el < 3 * kDeg && side < 0.5 && rnd() < 0.35) return 0.8 + rnd() * 1.6;           // wing root / strut
    if (el < 0 && std::abs(az) < 0.35 && rnd() < 0.25) return 0.5 + rnd() * 0.5;        // landing gear
    if (el < -3 * kDeg && std::abs(az) > 2.6 && rnd() < 0.3) return 1.2 + rnd() * 1.2;   // fin / rudder
    return -1;
}
} // namespace

LidarController *LidarController::s_instance = nullptr;

LidarController::LidarController(MessageBus *bus, VehicleHub *hub, QObject *parent) : QObject(parent), m_bus(bus), m_hub(hub)
{
    s_instance = this;
    m_cfg = defaults();
    QJsonObject saved = Settings::instance().json(kStorageKey).toObject();
    if (saved.isEmpty()) saved = QJsonDocument::fromJson(Settings::instance().value(kStorageKey).toString().toUtf8()).object();
    for (auto it = saved.constBegin(); it != saved.constEnd(); ++it)
        if (m_cfg.contains(it.key())) m_cfg.insert(it.key(), it.value().toVariant());

    m_thread.setObjectName(QStringLiteral("lidar"));
    m_core = new LidarCore;
    m_core->moveToThread(&m_thread);
    connect(&m_thread, &QThread::finished, m_core, &QObject::deleteLater);
    m_thread.start();
    connect(m_core, &LidarCore::originChanged, this, [](const QVariantMap &o) {
        if (o.isEmpty()) return;
        PointCloudData::instance().setOrigin({o.value(QStringLiteral("lat")).toDouble(), o.value(QStringLiteral("lon")).toDouble(),
                                              o.value(QStringLiteral("alt")).toDouble(), o.value(QStringLiteral("mPerLat")).toDouble(),
                                              o.value(QStringLiteral("mPerLon")).toDouble(), o.value(QStringLiteral("epoch")).toInt()});
    });
    connect(m_core, &LidarCore::points, this, [this](int epoch, const QVector<float> &enu, const QByteArray &intensity) {
        PointCloudData::instance().appendMapped(epoch, enu.constData(), reinterpret_cast<const std::uint8_t *>(intensity.constData()), int(enu.size() / 3));
        updateCount();
    });
    connect(m_core, &LidarCore::live, this, [this](const QString &frame, double ttl, const QVector<float> &xyz, const QByteArray &intensity) {
        if (m_enabled)
            PointCloudData::instance().appendLive(xyz.constData(), reinterpret_cast<const std::uint8_t *>(intensity.constData()), int(xyz.size() / 3), frame, ttl);
    });
    connect(m_core, &LidarCore::statusChanged, this, &LidarController::onStatus);
    // The pose messages of the vehicle link (not of a replay), on the client's thread
    if (m_bus)
        for (uint32_t id : {MAVLINK_MSG_ID_GPS_RAW_INT, MAVLINK_MSG_ID_ATTITUDE, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, MAVLINK_MSG_ID_EKF_STATUS_REPORT})
            m_bus->subscribe(id, m_core, [core = m_core](const MavMessage &m) {
                if (!m.replay) core->onMavlink(m.msg, m.arrivalMs);
            });

    // A vehicle link ends the demo flight, and with it the synthetic scan
    if (m_hub)
        connect(m_hub, &VehicleHub::connectionChanged, this, [this](bool connected) {
            if (!connected || !m_demoRunning) return;
            stopDemo();
            m_enabled = false;
            PointCloudData::instance().visible = false;
            setActive(false);
            emit enabledChanged();
            emit hudMessage(QStringLiteral("LIDAR: demo scan stopped — link up, enable again for the sensor"), QStringLiteral("info"));
        });

    m_rangeTimer.setInterval(250);
    connect(&m_rangeTimer, &QTimer::timeout, this, [] { PointCloudData::instance().updateRange(monoMs()); });
    m_demoTimer.setInterval(16);
    m_demoTimer.setTimerType(Qt::PreciseTimer);
    connect(&m_demoTimer, &QTimer::timeout, this, &LidarController::demoTick);
    m_demoCountTimer.setInterval(250);
    connect(&m_demoCountTimer, &QTimer::timeout, this, &LidarController::updateCount);
    applyDisplayConfig();
    refreshInterfaces();
}

LidarController::~LidarController()
{
    if (m_core) QMetaObject::invokeMethod(m_core, [core = m_core] { core->disconnectLidar(); }, Qt::BlockingQueuedConnection);
    m_thread.quit();
    m_thread.wait();
}

void LidarController::call(std::function<QVariantMap(LidarCore *)> fn, std::function<void(const QVariantMap &)> done)
{
    QMetaObject::invokeMethod(m_core, [this, core = m_core, fn = std::move(fn), done = std::move(done)] {
        const QVariantMap r = fn(core);
        if (done) QMetaObject::invokeMethod(this, [done, r] { done(r); }, Qt::QueuedConnection);
    }, Qt::QueuedConnection);
}

// ============== SETTINGS ==============

QVariantMap LidarController::coreConfig() const
{
    QVariantMap out;
    for (auto it = m_cfg.cbegin(); it != m_cfg.cend(); ++it)
        if (!kLocal.contains(it.key())) out.insert(it.key(), it.value());
    return out;
}

void LidarController::saveConfig() { Settings::instance().setJson(kStorageKey, QJsonObject::fromVariantMap(m_cfg)); }

void LidarController::applyDisplayConfig()
{
    PointCloudData &d = PointCloudData::instance();
    d.intensityColors = m_cfg.value(QStringLiteral("colorMode")).toString() == QLatin1String("intensity");
    d.pointSize = std::max(1.0, m_cfg.value(QStringLiteral("pointSize")).toDouble());
    d.maxPoints = int(std::clamp(m_cfg.value(QStringLiteral("maxPoints")).toDouble(), 1000.0, 50000000.0));   // as LidarCore::applyConfig
    d.overTerrain = m_cfg.value(QStringLiteral("overTerrain")).toBool();
}

void LidarController::setField(const QString &key, const QVariant &value)
{
    if (!defaults().contains(key)) return;
    const QVariant def = defaults().value(key);
    QVariant v;
    if (def.typeId() == QMetaType::Bool) {
        v = value.toBool();
    } else if (def.typeId() == QMetaType::Int) {
        bool ok = false;
        const int n = int(value.toString().toDouble(&ok));
        v = ok && n ? n : def.toInt();
    } else if (def.typeId() == QMetaType::Double) {
        bool ok = false;
        const double n = value.toString().trimmed().toDouble(&ok);
        v = ok && std::isfinite(n) ? n : def.toDouble();
    } else {
        v = key == QLatin1String("lidarIp") ? value.toString().trimmed() : value.toString();
    }
    m_cfg.insert(key, v);
    saveConfig();
    applyDisplayConfig();
    emit cfgChanged();
    if (!kLocal.contains(key)) {
        const QVariantMap c = coreConfig();
        call([c](LidarCore *core) {
            core->applyConfig(c);
            return QVariantMap{};
        });
    }
}

void LidarController::applyPreset(const QString &rpy)
{
    const QStringList p = rpy.split(QLatin1Char(','));
    if (p.size() != 3) return;
    m_cfg.insert(QStringLiteral("mountRoll"), p[0].toDouble());
    m_cfg.insert(QStringLiteral("mountPitch"), p[1].toDouble());
    m_cfg.insert(QStringLiteral("mountYaw"), p[2].toDouble());
    saveConfig();
    emit cfgChanged();
    const QVariantMap c = coreConfig();
    call([c](LidarCore *core) {
        core->applyConfig(c);
        return QVariantMap{};
    });
}

void LidarController::refreshInterfaces()
{
    // Host interface list: AUTO + every IPv4 on this machine
    m_hostOptions = {QVariantMap{{QStringLiteral("value"), QStringLiteral("auto")}, {QStringLiteral("label"), QStringLiteral("AUTO (interface on the LiDAR subnet)")}}};
    for (const QVariant &v : LidarCore::listInterfaces()) {
        const QVariantMap ni = v.toMap();
        m_hostOptions << QVariantMap{{QStringLiteral("value"), ni.value(QStringLiteral("address"))},
                                     {QStringLiteral("label"), QStringLiteral("%1 (%2)").arg(ni.value(QStringLiteral("address")).toString(), ni.value(QStringLiteral("name")).toString())}};
    }
    emit hostOptionsChanged();
}

// ============== CONNECTION ==============

void LidarController::setEnabled(bool on)
{
    const bool demoMode = m_hub && m_hub->snapshot()->isDemo();
    if (on && demoMode) {
        saveConfig();
        applyDisplayConfig();
        startDemo();
        return;
    }
    if (on) {
        if (m_enabled) return;
        saveConfig();
        applyDisplayConfig();
        m_enabled = true;   // the checkbox stays on while the client starts
        emit enabledChanged();
        const QVariantMap c = coreConfig();
        const QString ip = m_cfg.value(QStringLiteral("lidarIp")).toString();
        call([c](LidarCore *core) { return core->connectLidar(c); },
             [this, ip](const QVariantMap &res) {
                 if (!res.value(QStringLiteral("success")).toBool()) {
                     const QString err = res.value(QStringLiteral("error")).toString();
                     m_enabled = false;
                     emit enabledChanged();
                     emit hudMessage(QStringLiteral("LIDAR: %1").arg(err.isEmpty() ? QStringLiteral("connection failed") : err), QStringLiteral("error"));
                     renderStatus({{QStringLiteral("connected"), false}, {QStringLiteral("link"), QStringLiteral("OFF")}, {QStringLiteral("error"), err}});
                     return;
                 }
                 m_lastGateOk = -1;
                 PointCloudData::instance().visible = true;
                 m_rangeTimer.start();
                 setStrip(QStringLiteral("search"), QStringLiteral("SEARCHING"));
                 emit hudMessage(QStringLiteral("LIDAR: connecting to %1").arg(ip), QStringLiteral("info"));
             });
    } else {
        m_enabled = false;
        emit enabledChanged();
        if (m_demoRunning) stopDemo();
        else call([](LidarCore *core) { return core->disconnectLidar(); });
        PointCloudData::instance().visible = false;
        PointCloudData::instance().clearLive();
        m_rangeTimer.stop();
        setActive(false);
    }
}

void LidarController::clearMap()
{
    if (m_demoRunning) {
        m_demoOrigin = false;
        m_demoVoxels.clear();
        m_demoTotal = 0;
    } else {
        call([](LidarCore *core) {
            core->clear();
            return QVariantMap{};
        });
    }
    PointCloudData::instance().clear();
    PointCloudData::instance().clearLive();
    emit hudMessage(QStringLiteral("LIDAR: map cleared"), QStringLiteral("info"));
    updateStrip(m_lastStatus);
    updateCount();
}

void LidarController::saveMap()
{
    if (m_demoRunning) {
        emit hudMessage(QStringLiteral("LIDAR: demo scan — nothing to save"), QStringLiteral("warning"));
        return;
    }
    call([](LidarCore *core) { return core->saveMap(); },
         [this](const QVariantMap &res) {
             if (res.value(QStringLiteral("success")).toBool())
                 emit hudMessage(QStringLiteral("LIDAR: %1 pts saved → %2").arg(fmt(res.value(QStringLiteral("points")).toLongLong()), res.value(QStringLiteral("path")).toString()),
                                 QStringLiteral("info"));
             else
                 emit hudMessage(QStringLiteral("LIDAR: save failed — %1").arg(res.value(QStringLiteral("error")).toString()), QStringLiteral("error"));
         });
}

// ============== STATUS ==============

void LidarController::renderStatus(const QVariantMap &st)
{
    QString text;
    if (!st.value(QStringLiteral("connected")).toBool()) {
        const QString err = st.value(QStringLiteral("error")).toString();
        text = err.isEmpty() ? QStringLiteral("OFF") : span(QStringLiteral("err"), esc(err));
    } else {
        QStringList lines;
        auto n = [&](const char *k) { return fmt(st.value(QLatin1String(k)).toLongLong()); };
        const bool streaming = st.value(QStringLiteral("streaming")).toBool();
        const QString model = st.value(QStringLiteral("model")).toString(), sn = st.value(QStringLiteral("sn")).toString();
        lines << QStringLiteral("%1 %2 %3").arg(span(streaming ? QStringLiteral("ok") : QStringLiteral("warn"), st.value(QStringLiteral("link")).toString()), esc(model),
                                               sn.isEmpty() ? QString() : QStringLiteral("S/N ") + esc(sn)).trimmed();
        const QString host = st.value(QStringLiteral("hostIp")).toString();
        lines << QStringLiteral("LiDAR %1 → host %2").arg(esc(st.value(QStringLiteral("lidarIp")).toString()), esc(host.isEmpty() ? QStringLiteral("?") : host));
        lines << QStringLiteral("%1 pkt/s · %2 pts/s · %3 kbit/s").arg(n("pps"), n("pointsPs"), n("kbps"));
        const bool gateOk = st.value(QStringLiteral("gateOk")).toBool();
        lines << QStringLiteral("%1 · fix %2 · %3 sats · HDOP %4 · EKF var %5")
                     .arg(span(gateOk ? QStringLiteral("ok") : QStringLiteral("warn"), esc(st.value(QStringLiteral("gateReason")).toString())))
                     .arg(st.value(QStringLiteral("navFix")).toInt())
                     .arg(st.value(QStringLiteral("navSats")).toInt())
                     .arg(st.value(QStringLiteral("navHdop")).toDouble(), 0, 'f', 1)
                     .arg(st.value(QStringLiteral("navEkfPosVar")).toDouble(), 0, 'f', 2);
        lines << QStringLiteral("map %1 / %2 pts · +%3/s · filtered %4/s · live %5/s").arg(n("mapPoints"), n("maxPoints"), n("acceptedPs"), n("filteredPs"), n("livePs"));
        if (st.value(QStringLiteral("recording")).toBool())
            lines << QStringLiteral("%1 %2 pts → %3").arg(span(QStringLiteral("ok"), QStringLiteral("REC")), n("recordedPoints"), esc(st.value(QStringLiteral("recordPath")).toString()));
        const QString err = st.value(QStringLiteral("error")).toString();
        if (!err.isEmpty()) lines << span(QStringLiteral("err"), esc(err));
        text = lines.join(QStringLiteral("<br>"));
    }
    if (text == m_statusText) return;
    m_statusText = text;
    emit statusChanged();
}

void LidarController::setStrip(const QString &state, const QString &label)
{
    if (state == m_stripState && label == m_stripLabel) return;
    m_stripState = state;
    m_stripLabel = label;
    emit stripChanged();
}

void LidarController::updateCount()
{
    const QString c = QStringLiteral("%1 pts").arg(fmt(PointCloudData::instance().total));
    if (c == m_stripCount) return;
    m_stripCount = c;
    emit stripChanged();
}

void LidarController::updateStrip(const QVariantMap &st)
{
    if (!m_enabled || m_demoRunning) return;
    QString state = QStringLiteral("search"), label = QStringLiteral("SEARCHING");
    if (!st.isEmpty()) {
        const bool streaming = st.value(QStringLiteral("streaming")).toBool();
        const QString reason = st.value(QStringLiteral("gateReason")).toString();
        if (!st.value(QStringLiteral("error")).toString().isEmpty() && !streaming) {
            state = QStringLiteral("error");
            label = QStringLiteral("ERROR");
        } else if (!streaming) {
            label = st.value(QStringLiteral("link")).toString();
        } else if (st.value(QStringLiteral("gateOk")).toBool()) {
            state = QStringLiteral("accum");
            label = QStringLiteral("ACCUMULATING");
        } else if (reason == QLatin1String("MAP FULL")) {
            state = QStringLiteral("full");
            label = QStringLiteral("MAP FULL");
        } else {
            state = QStringLiteral("hold");
            label = QStringLiteral("LIVE ONLY · ") + reason;
        }
    }
    setStrip(state, label);
    updateCount();
}

void LidarController::setActive(bool a)
{
    if (a == m_active) return;
    m_active = a;
    emit statusChanged();
}

void LidarController::onStatus(const QVariantMap &st)
{
    m_lastStatus = st;
    if (m_demoRunning) {
        setActive(true);
        return;
    }
    renderStatus(st);
    updateStrip(st);
    setActive(st.value(QStringLiteral("connected")).toBool());
    if (!m_enabled || !st.value(QStringLiteral("streaming")).toBool()) return;
    // Announce gate transitions on the HUD, once per change
    const int gateOk = st.value(QStringLiteral("gateOk")).toBool() ? 1 : 0;
    if (m_lastGateOk != gateOk) {
        if (m_lastGateOk != -1 || gateOk)
            emit hudMessage(gateOk ? QStringLiteral("LIDAR: georeferenced — accumulating")
                                   : QStringLiteral("LIDAR: live only, not kept — ") + st.value(QStringLiteral("gateReason")).toString(),
                            gateOk ? QStringLiteral("info") : QStringLiteral("warning"));
        m_lastGateOk = gateOk;
    }
}

// ============== DEMO (no link) ==============

void LidarController::startDemo()
{
    m_enabled = true;
    PointCloudData::instance().clear();
    m_demoOrigin = false;
    m_demoVoxels.clear();
    m_demoTotal = 0;
    m_demoRunning = true;
    m_demoClock.start();
    m_demoLastT = 0;
    m_demoCarry = 0;
    m_demoTimer.start();
    m_demoCountTimer.start();
    m_rangeTimer.start();
    PointCloudData::instance().visible = true;
    setStrip(QStringLiteral("accum"), QStringLiteral("DEMO · SYNTHETIC SCAN"));
    m_statusText = span(QStringLiteral("ok"), QStringLiteral("DEMO")) +
                   QStringLiteral(" synthetic scan of the SRTM terrain under the demo flight — no LiDAR link.<br>Connect a vehicle and enable again for the real sensor.");
    setActive(true);
    emit statusChanged();
    emit enabledChanged();
    emit hudMessage(QStringLiteral("LIDAR: demo — synthetic terrain scan"), QStringLiteral("info"));
}

void LidarController::stopDemo()
{
    m_demoTimer.stop();
    m_demoCountTimer.stop();
    m_demoRunning = false;
    m_demoOrigin = false;
    m_demoVoxels.clear();
    m_demoTotal = 0;
    PointCloudData::instance().clear();
    m_statusText = QStringLiteral("OFF");
    emit statusChanged();
    emit enabledChanged();
}

// Per frame: as many rays as the point rate and the time budget allow -
// terrain by marching the heightfield, nearest hit wins
void LidarController::demoTick()
{
    if (!m_demoRunning || !m_hub) return;
    const auto s = m_hub->snapshot();
    HgtStore &hgt = HgtStore::instance();
    if (!s->isDemo() || !hgt.elevation(s->lat, s->lon)) return;
    const double t0 = m_demoClock.nsecsElapsed() / 1e6;
    const double dt = std::min(0.1, (t0 - m_demoLastT) / 1000);
    m_demoLastT = t0;
    PointCloudData &cloud = PointCloudData::instance();

    if (!m_demoOrigin) {
        const double f = s->lat * kDeg;
        m_demoOLat = s->lat;
        m_demoOLon = s->lon;
        m_demoOAlt = s->rawAlt;
        m_demoMPerLat = 111132.954 - 559.822 * std::cos(2 * f) + 1.175 * std::cos(4 * f);
        m_demoMPerLon = 111412.84 * std::cos(f) - 93.5 * std::cos(3 * f);
        m_demoOrigin = true;
        cloud.setOrigin({m_demoOLat, m_demoOLon, m_demoOAlt, m_demoMPerLat, m_demoMPerLon, ++m_demoEpoch});
    }

    m_demoCarry += kPointsPerSec * dt;
    const int rays = std::min(kMaxPerFrame, int(std::floor(m_demoCarry)));
    m_demoCarry -= rays;
    if (rays <= 0) return;

    const double cr = std::cos(s->roll), sr = std::sin(s->roll), cp = std::cos(s->pitch), sp = std::sin(s->pitch), cy = std::cos(s->yaw), sy = std::sin(s->yaw);
    const double R[9] = {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, -sp, cp * sr, cp * cr};
    const double vehE = (s->lon - m_demoOLon) * m_demoMPerLon;
    const double vehN = (s->lat - m_demoOLat) * m_demoMPerLat;
    const double vehU = s->rawAlt - m_demoOAlt;
    const double minRange = std::max(0.0, m_cfg.value(QStringLiteral("minRange")).toDouble());
    auto terrainAt = [&](double e, double n) { return hgt.elevation(m_demoOLat + n / m_demoMPerLat, m_demoOLon + e / m_demoMPerLon); };
    const double inv = 1 / kDemoVoxel;
    std::vector<float> xyz;
    std::vector<std::uint8_t> refl;
    xyz.reserve(std::size_t(rays) * 3);
    refl.reserve(std::size_t(rays));

    for (int i = 0; i < rays; ++i) {
        if ((i & 31) == 31 && m_demoClock.nsecsElapsed() / 1e6 - t0 > kBudgetMs) break;
        // Spin axis = body X: az sweeps around the fuselage (0 = straight
        // down, pi = straight up), el leans the fan forward
        const double az = rnd() * 2 * kPi;
        const double el = kElMin + rnd() * (kElMax - kElMin);
        const double ce = std::cos(el);
        const double bx = std::sin(el), by = ce * std::sin(az), bz = ce * std::cos(az);   // body FRD
        const double dn = R[0] * bx + R[1] * by + R[2] * bz;
        const double de = R[3] * bx + R[4] * by + R[5] * bz;
        const double dd = R[6] * bx + R[7] * by + R[8] * bz;
        const double du = -dd;

        double tHit = -1;
        int kind = 0;
        const double tAir = airframeHit(az, el);
        if (tAir > 0) {
            tHit = tAir;   // the airframe itself, closer than anything else
            kind = 5;
        } else {
            if (du > 0.35) continue;   // steeply up: sky, nothing to hit
            // Terrain: coarse march until the ray is under the heightfield, then bisect
            if (dd > 0.02) {
                double tPrev = 0, hit = -1;
                for (double t = kStepCoarse; t <= kRange; t += kStepCoarse) {
                    const auto h = terrainAt(vehE + de * t, vehN + dn * t);
                    if (!h) break;
                    if (vehU - dd * t <= *h - m_demoOAlt) {
                        hit = t;
                        break;
                    }
                    tPrev = t;
                }
                if (hit > 0) {
                    double lo = tPrev, hi = hit;
                    for (int k = 0; k < 5; ++k) {
                        const double mid = (lo + hi) * 0.5;
                        const auto h = terrainAt(vehE + de * mid, vehN + dn * mid);
                        if (!h) break;
                        if (vehU - dd * mid <= *h - m_demoOAlt) hi = mid;
                        else lo = mid;
                    }
                    tHit = (lo + hi) * 0.5;
                }
            }
        }
        if (tHit < 0) continue;
        tHit += (rnd() - 0.5) * 0.06;
        if (tHit < minRange) continue;   // the MIN RANGE filter, as in the client
        const double E = vehE + de * tHit, N = vehN + dn * tHit, U = vehU + du * tHit;
        const qint64 key = ((qint64(std::floor(E * inv)) + 65536) * 131072 + (qint64(std::floor(N * inv)) + 65536)) * 131072 + (qint64(std::floor(U * inv)) + 65536);
        if (m_demoVoxels.contains(key)) continue;
        m_demoVoxels.insert(key);
        xyz.insert(xyz.end(), {float(E), float(N), float(U)});
        // Reflectivity by material: the airframe bright, the ground by incidence
        const double r = kind == 5 ? 150 + rnd() * 60 : 60 + (1 - dd) * 100 + (rnd() - 0.5) * 30;
        refl.push_back(std::uint8_t(std::clamp(r, 0.0, 255.0)));
    }
    if (!refl.empty()) {
        m_demoTotal += qint64(refl.size());
        cloud.appendMapped(m_demoEpoch, xyz.data(), refl.data(), int(refl.size()));
    }
}

} // namespace corv
