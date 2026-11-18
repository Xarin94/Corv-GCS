#include "ui/RosController.h"

#include "core/DebugLog.h"
#include "core/Geo.h"
#include "core/Settings.h"
#include "scene/RosSurfaceData.h"
#include "sensors/RosClient.h"
#include "ui/PlanController.h"
#include "vehicle/VehicleHub.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLocale>
#include <QRegularExpression>
#include <cmath>

namespace corv {

namespace {
const QString kStorageKey = QStringLiteral("ros-config");

const QVariantMap &defaults()
{
    static const QVariantMap d{{QStringLiteral("url"), QStringLiteral("ws://127.0.0.1:9090")},
                               {QStringLiteral("transport"), QStringLiteral("cbor")},
                               {QStringLiteral("profile"), QStringLiteral("aerial")},
                               {QStringLiteral("frame"), QStringLiteral("auto")},
                               {QStringLiteral("rateHz"), 5.0},
                               {QStringLiteral("maxPoints"), 5000.0},
                               {QStringLiteral("mountRoll"), 0.0},
                               {QStringLiteral("mountPitch"), 0.0},
                               {QStringLiteral("mountYaw"), 0.0},
                               {QStringLiteral("leverX"), 0.0},
                               {QStringLiteral("leverY"), 0.0},
                               {QStringLiteral("leverZ"), 0.0},
                               {QStringLiteral("minRange"), 2.5},
                               {QStringLiteral("maxRange"), 100.0},
                               {QStringLiteral("despike"), QStringLiteral("on")},
                               {QStringLiteral("lagMs"), 0.0},
                               {QStringLiteral("timeBase"), QStringLiteral("arrival")},
                               {QStringLiteral("vertical"), QStringLiteral("vehicle")},
                               {QStringLiteral("cell"), 0.3},
                               {QStringLiteral("memory"), 20.0},
                               {QStringLiteral("maxTiles"), 2048.0},
                               {QStringLiteral("minSamples"), 1.0},
                               {QStringLiteral("fillM"), 0.0},
                               {QStringLiteral("layers"), QStringLiteral("floor")},
                               {QStringLiteral("fillPct"), 10.0},
                               {QStringLiteral("overTerrain"), true},
                               {QStringLiteral("colorMode"), QStringLiteral("gray")},
                               {QStringLiteral("shown"), true},
                               {QStringLiteral("topic"), QString()},
                               {QStringLiteral("topicType"), QString()}};
    return d;
}
// Kept here, not sent to the client
const QSet<QString> kLocal{QStringLiteral("url"),     QStringLiteral("profile"),  QStringLiteral("minSamples"), QStringLiteral("fillPct"),
                           QStringLiteral("overTerrain"), QStringLiteral("colorMode"), QStringLiteral("shown"),   QStringLiteral("topic"),
                           QStringLiteral("topicType")};

// What choosing a sensor fills in: an aerial Livox inverted under the belly;
// a down-looking echo sounder or sonar (Range / LaserScan measure along x); a
// mechanical sector-scanning sonar, head down and sweeping across the track,
// one beam per ping (its LaserScans carry time_increment: each beam is placed
// at its own time). 30 cm cells need points: ~10 000 a second to fill a
// 100 m swath at 8 m/s.
QVariantMap profile(const QString &name)
{
    if (name == QLatin1String("aerial"))
        return {{QStringLiteral("mountRoll"), 180.0}, {QStringLiteral("mountPitch"), 0.0}, {QStringLiteral("mountYaw"), 0.0}, {QStringLiteral("minRange"), 2.5},
                {QStringLiteral("maxRange"), 100.0},  {QStringLiteral("rateHz"), 10.0},    {QStringLiteral("maxPoints"), 5000.0}, {QStringLiteral("fillM"), 0.0}};
    if (name == QLatin1String("seabed"))
        return {{QStringLiteral("mountRoll"), 0.0}, {QStringLiteral("mountPitch"), -90.0}, {QStringLiteral("mountYaw"), 0.0}, {QStringLiteral("minRange"), 0.5},
                {QStringLiteral("maxRange"), 100.0}, {QStringLiteral("rateHz"), 10.0},     {QStringLiteral("maxPoints"), 1000.0}, {QStringLiteral("fillM"), 0.0}};
    // its sweeps 5-10 m apart along the track: holes filled up to 8 m
    if (name == QLatin1String("sector"))
        return {{QStringLiteral("mountRoll"), 0.0}, {QStringLiteral("mountPitch"), -90.0}, {QStringLiteral("mountYaw"), 0.0}, {QStringLiteral("minRange"), 0.75},
                {QStringLiteral("maxRange"), 100.0}, {QStringLiteral("rateHz"), 10.0},     {QStringLiteral("maxPoints"), 1000.0}, {QStringLiteral("fillM"), 8.0}};
    return {};
}

QString fmt(double n, int d = 0) { return QLocale(QLocale::English).toString(n, 'f', d).remove(QRegularExpression(d ? QStringLiteral("\\.?0+$") : QStringLiteral("$^"))); }
QString fmtInt(double n) { return QLocale(QLocale::English).toString(qint64(std::llround(n))); }
QString esc(const QString &s) { return s.toHtmlEscaped(); }
QString span(const QString &cls, const QString &text)
{
    const char *c = cls == QLatin1String("ok") ? "#44ff44" : cls == QLatin1String("warn") ? "#ffaa00" : "#ff4444";
    return QStringLiteral("<font color=\"%1\">%2</font>").arg(QLatin1String(c), text);
}
QString shortType(const QString &t)
{
    QString s = t;
    s.replace(QLatin1String("/msg/"), QLatin1String("/"));
    if (s.startsWith(QLatin1String("sensor_msgs/"))) s = s.mid(12);
    return s;
}
int modeOf(const QString &m) { return m == QLatin1String("distance") ? RosSurfaceData::Distance : m == QLatin1String("rough") ? RosSurfaceData::Rough : RosSurfaceData::Gray; }
} // namespace

RosController *RosController::s_instance = nullptr;

RosController::RosController(VehicleHub *hub, MessageBus *bus, PlanController *plan, QObject *parent)
    : QObject(parent), m_hub(hub), m_bus(bus), m_plan(plan)
{
    s_instance = this;
    m_cfg = defaults();
    QJsonObject saved = Settings::instance().json(kStorageKey).toObject();
    if (saved.isEmpty()) saved = QJsonDocument::fromJson(Settings::instance().value(kStorageKey).toString().toUtf8()).object();
    for (auto it = saved.constBegin(); it != saved.constEnd(); ++it)
        if (m_cfg.contains(it.key())) m_cfg.insert(it.key(), it.value().toVariant());
    if (!saved.contains(QStringLiteral("profile"))) applyProfile(QStringLiteral("aerial"));   // first run: the aerial defaults

    m_thread.setObjectName(QStringLiteral("ros"));
    m_client = new RosClient(hub, bus);
    m_client->moveToThread(&m_thread);
    connect(&m_thread, &QThread::finished, m_client, &QObject::deleteLater);
    m_thread.start();
    post([](RosClient *c) { c->start(); });
    connect(m_client, &RosClient::statusChanged, this, &RosController::onStatus);
    connect(m_client, &RosClient::topicsChanged, this, &RosController::onTopics);
    connect(m_client, &RosClient::tilesChanged, this, &RosController::onTiles);
    connect(m_client, &RosClient::volumeChanged, this, &RosController::onVolume);
    pushConfig();

    m_topicOptions = {QVariantMap{{QStringLiteral("value"), QString()}, {QStringLiteral("label"), QStringLiteral("— connect to list the topics —")}}};
    if (!m_cfg.value(QStringLiteral("topic")).toString().isEmpty()) onTopics({});

    // The frame the points were placed in is gone: start the surface again
    if (m_hub)
        connect(m_hub, &VehicleHub::connectionChanged, this, [this](bool connected) {
            if (m_enabled && connected) clear(false);
        });
    m_tick.setInterval(250);
    connect(&m_tick, &QTimer::timeout, this, &RosController::tick);
}

RosController::~RosController()
{
    if (m_client) QMetaObject::invokeMethod(m_client, [c = m_client] { c->disconnectLink(); }, Qt::BlockingQueuedConnection);
    m_thread.quit();
    m_thread.wait();
}

void RosController::post(std::function<void(RosClient *)> fn)
{
    QMetaObject::invokeMethod(m_client, [c = m_client, fn = std::move(fn)] { fn(c); }, Qt::QueuedConnection);
}

// ============== SETTINGS ==============

QVariantMap RosController::workerConfig() const
{
    QVariantMap out;
    for (auto it = m_cfg.cbegin(); it != m_cfg.cend(); ++it)
        if (!kLocal.contains(it.key())) out.insert(it.key(), it.value());
    return out;
}

void RosController::saveConfig() { Settings::instance().setJson(kStorageKey, QJsonObject::fromVariantMap(m_cfg)); }

QString RosController::colorMode() const { return m_cfg.value(QStringLiteral("colorMode")).toString(); }
QString RosController::colorLabel() const
{
    const QString m = colorMode();
    return m == QLatin1String("distance") ? QStringLiteral("DIST") : m == QLatin1String("rough") ? QStringLiteral("ROUGH") : QStringLiteral("GRAY");
}
bool RosController::shown() const { return m_cfg.value(QStringLiteral("shown")).toBool(); }

void RosController::applyView()
{
    RosSurfaceData &d = RosSurfaceData::instance();
    d.view.fill = float(std::clamp(m_cfg.value(QStringLiteral("fillPct")).toDouble() / 100, 0.0, 1.0));
    d.view.overTerrain = m_cfg.value(QStringLiteral("overTerrain")).toBool();
    const double minSamples = std::max(0.1, m_cfg.value(QStringLiteral("minSamples")).toDouble() > 0 ? m_cfg.value(QStringLiteral("minSamples")).toDouble() : 1.0);
    const double fillM = std::max(0.0, m_cfg.value(QStringLiteral("fillM")).toDouble());
    if (minSamples != d.view.minSamples || fillM != d.view.fillM) {
        d.view.minSamples = minSamples;
        d.view.fillM = fillM;
        d.markAllDirty();
    }
    d.setMode(modeOf(colorMode()));
    d.view.shown = shown();
}

void RosController::pushConfig()
{
    applyView();
    const QVariantMap c = workerConfig();
    post([c](RosClient *client) { client->applyConfig(c); });
}

void RosController::applyProfile(const QString &name)
{
    const QVariantMap p = profile(name);
    for (auto it = p.cbegin(); it != p.cend(); ++it) m_cfg.insert(it.key(), it.value());
}

void RosController::setField(const QString &key, const QVariant &value)
{
    if (!defaults().contains(key)) return;
    const QVariant def = defaults().value(key);
    QVariant v;
    if (def.typeId() == QMetaType::Bool) {
        v = value.toBool();
    } else if (def.typeId() == QMetaType::Double) {
        bool ok = false;
        const double n = value.toString().trimmed().toDouble(&ok);
        v = ok && std::isfinite(n) ? n : def.toDouble();
    } else if (key == QLatin1String("despike")) {
        v = value.toString() == QLatin1String("off") ? QStringLiteral("off") : QStringLiteral("on");
    } else if (key == QLatin1String("vertical")) {
        v = value.toString() == QLatin1String("water") ? QStringLiteral("water") : QStringLiteral("vehicle");
    } else {
        v = key == QLatin1String("url") ? value.toString().trimmed() : value.toString();
    }
    if (key == QLatin1String("profile")) applyProfile(v.toString());
    m_cfg.insert(key, v);
    saveConfig();
    pushConfig();
    emit cfgChanged();
    updatePreview();
    if (key == QLatin1String("url") && m_enabled) {
        const QString url = v.toString();
        post([url](RosClient *c) { c->connectTo(url); });
    }
}

void RosController::applyPreset(const QString &rpy)
{
    const QStringList p = rpy.split(QLatin1Char(','));
    if (p.size() != 3) return;
    m_cfg.insert(QStringLiteral("mountRoll"), p[0].toDouble());
    m_cfg.insert(QStringLiteral("mountPitch"), p[1].toDouble());
    m_cfg.insert(QStringLiteral("mountYaw"), p[2].toDouble());
    saveConfig();
    pushConfig();
    emit cfgChanged();
}

// ============== CONNECTION ==============

void RosController::setEnabled(bool on)
{
    saveConfig();
    if (on) {
        const QString url = m_cfg.value(QStringLiteral("url")).toString();
        if (!QRegularExpression(QStringLiteral("^wss?://"), QRegularExpression::CaseInsensitiveOption).match(url).hasMatch()) {
            emit enabledChanged();   // the checkbox goes back off
            emit hudMessage(QStringLiteral("ROS: the rosbridge URL must start with ws:// or wss://"), QStringLiteral("error"));
            return;
        }
        m_enabled = true;
        pushConfig();
        const QString topic = m_cfg.value(QStringLiteral("topic")).toString(), type = m_cfg.value(QStringLiteral("topicType")).toString();
        post([url, topic, type](RosClient *c) {
            c->clear();
            c->connectTo(url);
            if (!topic.isEmpty()) c->select(topic, type);
        });
        m_sentAreas.clear();
        sendAreas();
        RosSurfaceData::instance().view.visible = true;
        m_tick.start();
        m_lastState.clear();
        m_wasRelative = m_hub && m_hub->snapshot()->relativeMode;
        emit hudMessage(QStringLiteral("ROS: connecting to %1").arg(url), QStringLiteral("info"));
    } else {
        m_enabled = false;
        post([](RosClient *c) {
            c->disconnectLink();
            emit c->statusChanged(c->status());
        });
        RosSurfaceData::instance().view.visible = false;
        RosSurfaceData::instance().clear();
        m_tick.stop();
        m_active = false;
        emit statusChanged();
    }
    emit enabledChanged();
    emit stripChanged();
}

void RosController::clear(bool announce)
{
    post([](RosClient *c) { c->clear(); });
    RosSurfaceData::instance().clear();
    if (announce) emit hudMessage(QStringLiteral("ROS: surface cleared"), QStringLiteral("info"));
}

void RosController::clearSurface() { clear(true); }

void RosController::cycleColor()
{
    const QString m = colorMode();
    m_cfg.insert(QStringLiteral("colorMode"), m == QLatin1String("gray") ? QStringLiteral("distance") : m == QLatin1String("distance") ? QStringLiteral("rough") : QStringLiteral("gray"));
    saveConfig();
    applyView();
    emit cfgChanged();
    updateStrip(m_lastStatus);
}

void RosController::toggleView()
{
    m_cfg.insert(QStringLiteral("shown"), !shown());
    saveConfig();
    applyView();
    emit cfgChanged();
}

// ============== TOPICS ==============

void RosController::onTopics(const QVariantList &list)
{
    m_topics = list;
    QVariantList usable;
    for (const QVariant &v : list)
        if (!v.toMap().value(QStringLiteral("kind")).toString().isEmpty()) usable << v;
    std::sort(usable.begin(), usable.end(), [](const QVariant &a, const QVariant &b) {
        return a.toMap().value(QStringLiteral("name")).toString() < b.toMap().value(QStringLiteral("name")).toString();
    });
    const qsizetype others = list.size() - usable.size();
    m_topicOptions = {QVariantMap{{QStringLiteral("value"), QString()},
                                  {QStringLiteral("label"), usable.isEmpty() ? QStringLiteral("— no PointCloud2 / LaserScan / Range topic —") : QStringLiteral("— choose a topic —")}}};
    for (const QVariant &v : std::as_const(usable)) {
        const QVariantMap t = v.toMap();
        m_topicOptions << QVariantMap{{QStringLiteral("value"), t.value(QStringLiteral("name"))},
                                      {QStringLiteral("type"), t.value(QStringLiteral("type"))},
                                      {QStringLiteral("label"), QStringLiteral("%1 · %2").arg(t.value(QStringLiteral("name")).toString(), shortType(t.value(QStringLiteral("type")).toString()))}};
    }
    // A saved topic nobody publishes yet stays selected (rosbridge subscribes ahead of the publisher)
    const QString topic = m_cfg.value(QStringLiteral("topic")).toString();
    if (!topic.isEmpty() && std::none_of(usable.cbegin(), usable.cend(), [&](const QVariant &v) { return v.toMap().value(QStringLiteral("name")).toString() == topic; })) {
        const QString type = m_cfg.value(QStringLiteral("topicType")).toString();
        m_topicOptions << QVariantMap{{QStringLiteral("value"), topic},
                                      {QStringLiteral("type"), type},
                                      {QStringLiteral("label"), QStringLiteral("%1 · %2 (not published now)").arg(topic, shortType(type))}};
    }
    m_topicTip = others ? QStringLiteral("%1 other topics of types this view does not draw").arg(others) : QString();
    emit topicsChanged();
}

void RosController::selectTopic(const QString &name)
{
    QString type;
    for (const QVariant &v : std::as_const(m_topicOptions))
        if (v.toMap().value(QStringLiteral("value")).toString() == name) type = v.toMap().value(QStringLiteral("type")).toString();
    m_cfg.insert(QStringLiteral("topic"), name);
    m_cfg.insert(QStringLiteral("topicType"), name.isEmpty() ? QString() : type);
    saveConfig();
    emit cfgChanged();
    clear(false);
    post([name, type](RosClient *c) { c->select(name, type); });
}

void RosController::refreshTopics()
{
    post([](RosClient *c) { c->listTopics(); });
}

// The planner's survey areas, for the coverage the client measures against them
void RosController::sendAreas()
{
    if (!m_plan) return;
    QVariantList areas;
    for (const QJsonValue &sv : m_plan->route().value(QStringLiteral("segments")).toArray()) {
        const QJsonObject seg = sv.toObject();
        const QJsonArray pts = seg.value(QStringLiteral("points")).toArray();
        if (seg.value(QStringLiteral("type")).toString() != QLatin1String("area") || pts.size() < 3) continue;
        areas << QVariantMap{{QStringLiteral("name"), QStringLiteral("area %1").arg(areas.size() + 1)}, {QStringLiteral("points"), pts.toVariantList()}};
    }
    const QString key = QString::fromUtf8(QJsonDocument(QJsonArray::fromVariantList(areas)).toJson(QJsonDocument::Compact));
    if (key == m_sentAreas) return;
    m_sentAreas = key;
    post([areas](RosClient *c) { c->setAreas(areas); });
}

// ============== SURFACE ==============

void RosController::onTiles(const RosTilesMsg &m)
{
    if (!m_enabled) return;
    RosSurfaceData::instance().applyTiles(m.reset, m.anchor, m.cell, m.tiles, m.removed);
}

void RosController::onVolume(const RosVolumeMsg &m)
{
    if (!m_enabled) return;
    RosSurfaceData::instance().applyVolume(m.reset, m.anchor, m.cell, m.meshes, m.removed);
}

// Four times a second: the drawn heights of the blocks that changed, the colour scales
void RosController::tick()
{
    RosSurfaceData &d = RosSurfaceData::instance();
    if (!m_hub) return;
    const auto s = m_hub->snapshot();
    // The relative navigation switched: the frame the points were placed in is gone
    if (s->relativeMode != m_wasRelative) {
        m_wasRelative = s->relativeMode;
        clear(false);
        return;
    }
    d.fillDirty();
    const std::optional<RosAnchor> &a = d.anchor ? d.anchor : d.volAnchor;
    if (a)
        d.updateScales((s->lon - a->lon) * a->mPerLon, (s->lat - a->lat) * a->mPerLat, s->rawAlt - a->alt);
}

// ============== STATUS ==============

void RosController::updatePreview()
{
    const QVariantMap &st = m_lastStatus;
    const QVariantMap z = st.value(QStringLiteral("zone")).toMap();
    const QString kind = st.value(QStringLiteral("kind")).toString(), frame = st.value(QStringLiteral("frame")).toString();
    QVariantMap zone;
    QString caption;
    if (kind.isEmpty()) {
        caption = QStringLiteral("choose a topic: its zone shows here");
    } else if (!frame.isEmpty() && frame != QLatin1String("sensor")) {
        caption = QStringLiteral("points in the %1 frame: no sensor zone").arg(st.value(QStringLiteral("frameId")).toString());
    } else if (kind == QLatin1String("range")) {
        const double f = z.value(QStringLiteral("fov")).toDouble();
        const double fov = f > 0 ? f / kDeg : 30;
        zone = {{QStringLiteral("type"), QStringLiteral("cone")}, {QStringLiteral("half"), fov / 2}};
        caption = QStringLiteral("Range · %1° cone along X").arg(fmt(fov));
    } else if (kind == QLatin1String("scan")) {
        double a0 = -135, a1 = 135;
        if (z.contains(QStringLiteral("min")) && std::isfinite(z.value(QStringLiteral("min")).toDouble())) {
            a0 = z.value(QStringLiteral("min")).toDouble() / kDeg;
            a1 = z.value(QStringLiteral("max")).toDouble() / kDeg;
        }
        zone = {{QStringLiteral("type"), QStringLiteral("fan")}, {QStringLiteral("min"), a0}, {QStringLiteral("max"), a1}};
        caption = QStringLiteral("LaserScan · %1…%2° in the X–Y plane").arg(fmt(a0), fmt(a1));
    } else {
        const QVariantMap dirs = z.value(QStringLiteral("dirs")).toMap();
        if (!dirs.value(QStringLiteral("bins")).toList().isEmpty()) {
            zone = dirs;
            zone.insert(QStringLiteral("type"), QStringLiteral("dirs"));
            caption = QStringLiteral("PointCloud2 · where the points came from, last 10 s");
        } else if (m_cfg.value(QStringLiteral("profile")).toString() == QLatin1String("aerial")) {
            zone = {{QStringLiteral("type"), QStringLiteral("band")}, {QStringLiteral("elMin"), -7}, {QStringLiteral("elMax"), 52}};
            caption = QStringLiteral("PointCloud2 · Mid-360 band until points arrive");
        } else {
            caption = QStringLiteral("PointCloud2 · the zone shows when points arrive");
        }
    }
    m_previewZone = zone;
    m_previewCaption = caption;
}

void RosController::renderStatus(const QVariantMap &st)
{
    const QString link = st.value(QStringLiteral("link")).toString();
    if (st.isEmpty() || link == QLatin1String("OFF")) {
        const QString err = st.value(QStringLiteral("error")).toString();
        m_statusText = err.isEmpty() ? QStringLiteral("OFF") : span(QStringLiteral("err"), esc(err));
        return;
    }
    const QString state = st.value(QStringLiteral("state")).toString();
    QStringList lines;
    lines << QStringLiteral("%1 %2").arg(span(link == QLatin1String("CONNECTED") ? QStringLiteral("ok") : QStringLiteral("warn"), esc(link)), esc(st.value(QStringLiteral("url")).toString()));
    const QString topic = st.value(QStringLiteral("topic")).toString(), frameId = st.value(QStringLiteral("frameId")).toString(), frame = st.value(QStringLiteral("frame")).toString();
    if (!topic.isEmpty()) {
        const QString fl = frame == QLatin1String("sensor") ? QStringLiteral("SENSOR · placed with the telemetry pose")
                         : frame == QLatin1String("world-enu") ? QStringLiteral("WORLD ENU")
                         : frame == QLatin1String("world-ned") ? QStringLiteral("WORLD NED") : QString();
        lines << QStringLiteral("%1 · %2%3").arg(esc(topic), esc(shortType(st.value(QStringLiteral("type")).toString())),
                                                 frameId.isEmpty() ? QString() : QStringLiteral(" · frame %1 → %2").arg(esc(frameId), fl));
    }
    if (st.value(QStringLiteral("timedSpan")).toDouble() > 0 && frame == QLatin1String("sensor"))
        lines << QStringLiteral("beams measured over %1 s a message (time_increment): each placed with the pose of its own time").arg(fmt(st.value(QStringLiteral("timedSpan")).toDouble(), 2));
    if (st.value(QStringLiteral("stampOff")).isValid())
        lines << span(QStringLiteral("warn"), QStringLiteral("header.stamp %1 s from this clock: not synced, arrival time used").arg(fmt(st.value(QStringLiteral("stampOff")).toDouble() / 1000, 1)));
    const QVariantMap r = st.value(QStringLiteral("rate")).toMap();
    auto rn = [&](const char *k) { return r.value(QLatin1String(k)).toDouble(); };
    const QString despike = st.value(QStringLiteral("despike")).toString();
    QString rateLine = QStringLiteral("%1 msg/s · %2 pts/s in · %3 sampled · %4 averaged").arg(fmtInt(rn("msgs")), fmtInt(rn("pointsIn")), fmtInt(rn("sampled")), fmtInt(rn("used")));
    if (despike == QLatin1String("on"))
        rateLine += QStringLiteral(" · %1 spikes dropped (%2 %)").arg(fmtInt(rn("spikes")), fmt(100 * rn("spikes") / std::max(1.0, rn("sampled")), 1));
    else if (despike == QLatin1String("unordered"))
        rateLine += QStringLiteral(" · spike filter idle: the scan has no order");
    lines << rateLine;
    QStringList drops;
    if (rn("noPose") > 0) {
        const QVariantMap m = st.value(QStringLiteral("poseMiss")).toMap();
        QString d = QStringLiteral("%1 msg/s without pose").arg(fmtInt(rn("noPose")));
        // (points later than the newest telemetry: the telemetry lags; earlier than the oldest kept: the points do)
        if (m.value(QStringLiteral("att")).isValid()) {
            const double att = m.value(QStringLiteral("att")).toDouble();
            d += QStringLiteral(" (points %1, %2 ms after the oldest kept, clock ×%3)")
                     .arg(att >= 0 ? QStringLiteral("%1 ms after the newest attitude").arg(fmtInt(att)) : QStringLiteral("%1 ms before it").arg(fmtInt(-att)),
                          fmtInt(m.value(QStringLiteral("oldest")).toDouble()), fmt(m.value(QStringLiteral("clockRate")).toDouble(), 1));
        }
        drops << d;
    }
    if (rn("noHome") > 0) drops << QStringLiteral("%1 msg/s without home").arg(fmtInt(rn("noHome")));
    if (!drops.isEmpty()) lines << span(QStringLiteral("warn"), drops.join(QStringLiteral(" · ")));
    const QVariantMap g = st.value(QStringLiteral("surface")).toMap();
    auto gn = [&](const char *k) { return g.value(QLatin1String(k)).toDouble(); };
    const RosSurfaceData &d = RosSurfaceData::instance();
    if (g.value(QStringLiteral("layers")).toString() == QLatin1String("cave")) {
        lines << QStringLiteral("3D surface: %1 / %2 chunks of 16³ voxels of %3 m · %4 triangles")
                     .arg(fmtInt(gn("chunks")), fmtInt(gn("maxChunks")), fmt(gn("cell"), 2), fmtInt(d.volumeTriangles()));
    } else {
        const double cell = gn("cell");
        lines << QStringLiteral("surface %1 / %2 tiles of %3×%3 · cell %4 m · %5 cells (%6 m²)")
                     .arg(fmtInt(gn("tiles")), fmtInt(gn("maxTiles")), fmtInt(gn("tileCells")), fmt(cell, 2), fmtInt(gn("filled")), fmtInt(gn("filled") * cell * cell));
        for (const QVariant &cv : g.value(QStringLiteral("coverage")).toList()) {
            const QVariantMap c = cv.toMap();
            const double area = c.value(QStringLiteral("area")).toDouble(), covered = c.value(QStringLiteral("covered")).toDouble();
            lines << QStringLiteral("planned %1: %2 km² · %3")
                         .arg(esc(c.value(QStringLiteral("name")).toString()), fmt(area / 1e6, 2),
                              span(area > 0 && covered / area > 0.9 ? QStringLiteral("ok") : QStringLiteral("warn"),
                                   QStringLiteral("%1 % covered").arg(fmt(area > 0 ? 100 * covered / area : 0, 1))));
        }
        const RosSurfaceData::Stats ms = d.stats();
        double lvTotal = 0;
        for (auto n : ms.levels) lvTotal += n;
        QStringList res;
        for (int k = 0; k < 5; ++k)
            if (ms.levels[std::size_t(k)])
                res << QStringLiteral("%1 cm %2%").arg(fmtInt(cell * 100 * (1 << k))).arg(std::lround(100 * ms.levels[std::size_t(k)] / lvTotal));
        lines << QStringLiteral("drawn %1 / %2 blocks · %3 triangles · resolution %4")
                     .arg(fmtInt(d.drawnBlocks), fmtInt(ms.blocks), fmtInt(d.drawnTriangles), res.isEmpty() ? QStringLiteral("—") : res.join(QStringLiteral(" · ")));
    }
    const bool rel = m_hub && m_hub->snapshot()->relativeMode;
    QString last = QStringLiteral("%1 · pose %2").arg(span(state == QLatin1String("ACCUMULATING") ? QStringLiteral("ok") : QStringLiteral("warn"), esc(state)),
                                                     rel ? QStringLiteral("relative (local frame)") : QStringLiteral("absolute (GPS)"));
    if (st.value(QStringLiteral("vertical")).toString() == QLatin1String("water"))
        last += st.value(QStringLiteral("waterAlt")).isValid() ? QStringLiteral(" · height: the water surface at home (%1 m)").arg(fmt(st.value(QStringLiteral("waterAlt")).toDouble(), 1))
                                                              : QStringLiteral(" · height: the water surface at home, no home yet");
    const double clock = st.value(QStringLiteral("clockRate")).toDouble();
    if (clock > 0 && std::abs(clock - 1) > 0.1) last += QStringLiteral(" · autopilot clock ×%1 (SITL speedup)").arg(fmt(clock, 1));
    lines << last;
    const QString err = st.value(QStringLiteral("error")).toString();
    if (!err.isEmpty() && link != QLatin1String("CONNECTED")) lines << span(QStringLiteral("err"), esc(err));
    m_statusText = lines.join(QStringLiteral("<br>"));
}

void RosController::updateStrip(const QVariantMap &st)
{
    if (!m_enabled || st.isEmpty()) return;
    const QString s = st.value(QStringLiteral("state")).toString();
    m_stripState = s == QLatin1String("ACCUMULATING") ? QStringLiteral("accum")
                 : (s == QLatin1String("CONNECTING") || s == QLatin1String("NO ROSBRIDGE")) ? QStringLiteral("search") : QStringLiteral("hold");
    m_stripLabel = s;
    const QVariantMap g = st.value(QStringLiteral("surface")).toMap();
    const bool cave = g.value(QStringLiteral("layers")).toString() == QLatin1String("cave");
    const double have = cave ? g.value(QStringLiteral("chunks")).toDouble() : g.value(QStringLiteral("filled")).toDouble();
    const double cell = g.value(QStringLiteral("cell")).toDouble();
    const RosSurfaceData &d = RosSurfaceData::instance();
    QString text = !have ? QStringLiteral("—")
                 : cave  ? QStringLiteral("3D · %1 triangles · %2 cm").arg(fmtInt(d.volumeTriangles()), fmtInt(cell * 100))
                         : QStringLiteral("%1 cells · %2 cm").arg(fmtInt(g.value(QStringLiteral("filled")).toDouble()), fmtInt(cell * 100));
    const QString mode = colorMode();
    if (mode == QLatin1String("distance") && have) {
        text += QStringLiteral(" · %1–%2 m").arg(fmtInt(d.view.nearM), fmtInt(d.view.farM));
    } else if (mode == QLatin1String("rough") && have) {
        if (cave) {
            auto deg = [](double s) { return std::acos(std::clamp(1 - s, -1.0, 1.0)) / kDeg; };
            text += QStringLiteral(" · blue ≤ %1° · red ≥ %2° of spread").arg(fmtInt(deg(d.view.spreadMean)), fmtInt(deg(d.view.spreadFull)));
        } else {
            text += QStringLiteral(" · blue ≤ %1 cm · red ≥ %2 cm off-plane").arg(fmtInt(d.view.roughMean * 100.0), fmtInt(d.view.roughFull * 100.0));
        }
    }
    const QVariantList cov = g.value(QStringLiteral("coverage")).toList();
    if (!cave && !cov.isEmpty()) {
        double a = 0, c = 0;
        for (const QVariant &v : cov) {
            a += v.toMap().value(QStringLiteral("area")).toDouble();
            c += v.toMap().value(QStringLiteral("covered")).toDouble();
        }
        if (a > 0) text += QStringLiteral(" · %1 % of the area").arg(fmtInt(100 * c / a));
    }
    m_stripText = text;
    emit stripChanged();
}

void RosController::onStatus(const QVariantMap &st)
{
    m_lastStatus = st;
    updatePreview();
    sendAreas();
    renderStatus(st);
    updateStrip(st);
    m_active = m_enabled && st.value(QStringLiteral("link")).toString() == QLatin1String("CONNECTED");
    emit statusChanged();
    const QString state = st.value(QStringLiteral("state")).toString();
    if (!m_enabled || state == m_lastState) return;
    if (state == QLatin1String("ACCUMULATING"))
        emit hudMessage(QStringLiteral("ROS: averaging %1").arg(st.value(QStringLiteral("topic")).toString()), QStringLiteral("info"));
    else if (m_lastState == QLatin1String("ACCUMULATING"))
        emit hudMessage(QStringLiteral("ROS: %1").arg(state.toLower()), QStringLiteral("warning"));
    else if (state == QLatin1String("NO ROSBRIDGE") && m_lastState == QLatin1String("CONNECTING"))
        emit hudMessage(QStringLiteral("ROS: no rosbridge at %1").arg(st.value(QStringLiteral("url")).toString()), QStringLiteral("warning"));
    m_lastState = state;
}

} // namespace corv
