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

using namespace Qt::StringLiterals;

namespace corv {

namespace {
const QString kStorageKey = u"ros-config"_s;

const QVariantMap &defaults()
{
    static const QVariantMap d{{u"url"_s, u"ws://127.0.0.1:9090"_s},
                               {u"transport"_s, u"cbor"_s},
                               {u"profile"_s, u"aerial"_s},
                               {u"frame"_s, u"auto"_s},
                               {u"rateHz"_s, 5.0},
                               {u"maxPoints"_s, 5000.0},
                               {u"mountRoll"_s, 0.0},
                               {u"mountPitch"_s, 0.0},
                               {u"mountYaw"_s, 0.0},
                               {u"leverX"_s, 0.0},
                               {u"leverY"_s, 0.0},
                               {u"leverZ"_s, 0.0},
                               {u"minRange"_s, 2.5},
                               {u"maxRange"_s, 100.0},
                               {u"despike"_s, u"on"_s},
                               {u"lagMs"_s, 0.0},
                               {u"timeBase"_s, u"arrival"_s},
                               {u"vertical"_s, u"vehicle"_s},
                               {u"cell"_s, 0.3},
                               {u"memory"_s, 20.0},
                               {u"maxTiles"_s, 2048.0},
                               {u"minSamples"_s, 1.0},
                               {u"fillM"_s, 0.0},
                               {u"layers"_s, u"floor"_s},
                               {u"fillPct"_s, 10.0},
                               {u"overTerrain"_s, true},
                               {u"colorMode"_s, u"gray"_s},
                               {u"shown"_s, true},
                               {u"topic"_s, QString()},
                               {u"topicType"_s, QString()}};
    return d;
}
// Kept here, not sent to the client
const QSet<QString> kLocal{u"url"_s,     u"profile"_s,  u"minSamples"_s, u"fillPct"_s,
                           u"overTerrain"_s, u"colorMode"_s, u"shown"_s,   u"topic"_s,
                           u"topicType"_s};

// What choosing a sensor fills in: an aerial Livox inverted under the belly;
// a down-looking echo sounder or sonar (Range / LaserScan measure along x); a
// mechanical sector-scanning sonar, head down and sweeping across the track,
// one beam per ping (its LaserScans carry time_increment: each beam is placed
// at its own time). 30 cm cells need points: ~10 000 a second to fill a
// 100 m swath at 8 m/s.
QVariantMap profile(const QString &name)
{
    if (name == "aerial"_L1)
        return {{u"mountRoll"_s, 180.0}, {u"mountPitch"_s, 0.0}, {u"mountYaw"_s, 0.0}, {u"minRange"_s, 2.5},
                {u"maxRange"_s, 100.0},  {u"rateHz"_s, 10.0},    {u"maxPoints"_s, 5000.0}, {u"fillM"_s, 0.0}};
    if (name == "seabed"_L1)
        return {{u"mountRoll"_s, 0.0}, {u"mountPitch"_s, -90.0}, {u"mountYaw"_s, 0.0}, {u"minRange"_s, 0.5},
                {u"maxRange"_s, 100.0}, {u"rateHz"_s, 10.0},     {u"maxPoints"_s, 1000.0}, {u"fillM"_s, 0.0}};
    // its sweeps 5-10 m apart along the track: holes filled up to 8 m
    if (name == "sector"_L1)
        return {{u"mountRoll"_s, 0.0}, {u"mountPitch"_s, -90.0}, {u"mountYaw"_s, 0.0}, {u"minRange"_s, 0.75},
                {u"maxRange"_s, 100.0}, {u"rateHz"_s, 10.0},     {u"maxPoints"_s, 1000.0}, {u"fillM"_s, 8.0}};
    return {};
}

QString fmt(double n, int d = 0) { return QLocale(QLocale::English).toString(n, 'f', d).remove(QRegularExpression(d ? u"\\.?0+$"_s : u"$^"_s)); }
QString fmtInt(double n) { return QLocale(QLocale::English).toString(qint64(std::llround(n))); }
QString esc(const QString &s) { return s.toHtmlEscaped(); }
QString span(const QString &cls, const QString &text)
{
    const char *c = cls == "ok"_L1 ? "#44ff44" : cls == "warn"_L1 ? "#ffaa00" : "#ff4444";
    return u"<font color=\"%1\">%2</font>"_s.arg(QLatin1String(c), text);
}
QString shortType(const QString &t)
{
    QString s = t;
    s.replace("/msg/"_L1, "/"_L1);
    if (s.startsWith("sensor_msgs/"_L1)) s = s.mid(12);
    return s;
}
int modeOf(const QString &m) { return m == "distance"_L1 ? RosSurfaceData::Distance : m == "rough"_L1 ? RosSurfaceData::Rough : RosSurfaceData::Gray; }
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
    if (!saved.contains(u"profile"_s)) applyProfile(u"aerial"_s);   // first run: the aerial defaults

    m_thread.setObjectName(u"ros"_s);
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

    m_topicOptions = {QVariantMap{{u"value"_s, QString()}, {u"label"_s, u"— connect to list the topics —"_s}}};
    if (!m_cfg.value(u"topic"_s).toString().isEmpty()) onTopics({});

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

QString RosController::colorMode() const { return m_cfg.value(u"colorMode"_s).toString(); }
QString RosController::colorLabel() const
{
    const QString m = colorMode();
    return m == "distance"_L1 ? u"DIST"_s : m == "rough"_L1 ? u"ROUGH"_s : u"GRAY"_s;
}
bool RosController::shown() const { return m_cfg.value(u"shown"_s).toBool(); }

void RosController::applyView()
{
    RosSurfaceData &d = RosSurfaceData::instance();
    d.view.fill = float(std::clamp(m_cfg.value(u"fillPct"_s).toDouble() / 100, 0.0, 1.0));
    d.view.overTerrain = m_cfg.value(u"overTerrain"_s).toBool();
    const double minSamples = std::max(0.1, m_cfg.value(u"minSamples"_s).toDouble() > 0 ? m_cfg.value(u"minSamples"_s).toDouble() : 1.0);
    const double fillM = std::max(0.0, m_cfg.value(u"fillM"_s).toDouble());
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
    } else if (key == "despike"_L1) {
        v = value.toString() == "off"_L1 ? u"off"_s : u"on"_s;
    } else if (key == "vertical"_L1) {
        v = value.toString() == "water"_L1 ? u"water"_s : u"vehicle"_s;
    } else {
        v = key == "url"_L1 ? value.toString().trimmed() : value.toString();
    }
    if (key == "profile"_L1) applyProfile(v.toString());
    m_cfg.insert(key, v);
    saveConfig();
    pushConfig();
    emit cfgChanged();
    updatePreview();
    if (key == "url"_L1 && m_enabled) {
        const QString url = v.toString();
        post([url](RosClient *c) { c->connectTo(url); });
    }
}

void RosController::applyPreset(const QString &rpy)
{
    const QStringList p = rpy.split(','_L1);
    if (p.size() != 3) return;
    m_cfg.insert(u"mountRoll"_s, p[0].toDouble());
    m_cfg.insert(u"mountPitch"_s, p[1].toDouble());
    m_cfg.insert(u"mountYaw"_s, p[2].toDouble());
    saveConfig();
    pushConfig();
    emit cfgChanged();
}

// ============== CONNECTION ==============

void RosController::setEnabled(bool on)
{
    saveConfig();
    if (on) {
        const QString url = m_cfg.value(u"url"_s).toString();
        if (!QRegularExpression(u"^wss?://"_s, QRegularExpression::CaseInsensitiveOption).match(url).hasMatch()) {
            emit enabledChanged();   // the checkbox goes back off
            emit hudMessage(u"ROS: the rosbridge URL must start with ws:// or wss://"_s, u"error"_s);
            return;
        }
        m_enabled = true;
        pushConfig();
        const QString topic = m_cfg.value(u"topic"_s).toString(), type = m_cfg.value(u"topicType"_s).toString();
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
        emit hudMessage(u"ROS: connecting to %1"_s.arg(url), u"info"_s);
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
    if (announce) emit hudMessage(u"ROS: surface cleared"_s, u"info"_s);
}

void RosController::clearSurface() { clear(true); }

void RosController::cycleColor()
{
    const QString m = colorMode();
    m_cfg.insert(u"colorMode"_s, m == "gray"_L1 ? u"distance"_s : m == "distance"_L1 ? u"rough"_s : u"gray"_s);
    saveConfig();
    applyView();
    emit cfgChanged();
    updateStrip(m_lastStatus);
}

void RosController::toggleView()
{
    m_cfg.insert(u"shown"_s, !shown());
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
        if (!v.toMap().value(u"kind"_s).toString().isEmpty()) usable << v;
    std::sort(usable.begin(), usable.end(), [](const QVariant &a, const QVariant &b) {
        return a.toMap().value(u"name"_s).toString() < b.toMap().value(u"name"_s).toString();
    });
    const qsizetype others = list.size() - usable.size();
    m_topicOptions = {QVariantMap{{u"value"_s, QString()},
                                  {u"label"_s, usable.isEmpty() ? u"— no PointCloud2 / LaserScan / Range topic —"_s : u"— choose a topic —"_s}}};
    for (const QVariant &v : std::as_const(usable)) {
        const QVariantMap t = v.toMap();
        m_topicOptions << QVariantMap{{u"value"_s, t.value(u"name"_s)},
                                      {u"type"_s, t.value(u"type"_s)},
                                      {u"label"_s, u"%1 · %2"_s.arg(t.value(u"name"_s).toString(), shortType(t.value(u"type"_s).toString()))}};
    }
    // A saved topic nobody publishes yet stays selected (rosbridge subscribes ahead of the publisher)
    const QString topic = m_cfg.value(u"topic"_s).toString();
    if (!topic.isEmpty() && std::none_of(usable.cbegin(), usable.cend(), [&](const QVariant &v) { return v.toMap().value(u"name"_s).toString() == topic; })) {
        const QString type = m_cfg.value(u"topicType"_s).toString();
        m_topicOptions << QVariantMap{{u"value"_s, topic},
                                      {u"type"_s, type},
                                      {u"label"_s, u"%1 · %2 (not published now)"_s.arg(topic, shortType(type))}};
    }
    m_topicTip = others ? u"%1 other topics of types this view does not draw"_s.arg(others) : QString();
    emit topicsChanged();
}

void RosController::selectTopic(const QString &name)
{
    QString type;
    for (const QVariant &v : std::as_const(m_topicOptions))
        if (v.toMap().value(u"value"_s).toString() == name) type = v.toMap().value(u"type"_s).toString();
    m_cfg.insert(u"topic"_s, name);
    m_cfg.insert(u"topicType"_s, name.isEmpty() ? QString() : type);
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
    for (const QJsonValue &sv : m_plan->route().value(u"segments"_s).toArray()) {
        const QJsonObject seg = sv.toObject();
        const QJsonArray pts = seg.value(u"points"_s).toArray();
        if (seg.value(u"type"_s).toString() != "area"_L1 || pts.size() < 3) continue;
        areas << QVariantMap{{u"name"_s, u"area %1"_s.arg(areas.size() + 1)}, {u"points"_s, pts.toVariantList()}};
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
    const QVariantMap z = st.value(u"zone"_s).toMap();
    const QString kind = st.value(u"kind"_s).toString(), frame = st.value(u"frame"_s).toString();
    QVariantMap zone;
    QString caption;
    if (kind.isEmpty()) {
        caption = u"choose a topic: its zone shows here"_s;
    } else if (!frame.isEmpty() && frame != "sensor"_L1) {
        caption = u"points in the %1 frame: no sensor zone"_s.arg(st.value(u"frameId"_s).toString());
    } else if (kind == "range"_L1) {
        const double f = z.value(u"fov"_s).toDouble();
        const double fov = f > 0 ? f / kDeg : 30;
        zone = {{u"type"_s, u"cone"_s}, {u"half"_s, fov / 2}};
        caption = u"Range · %1° cone along X"_s.arg(fmt(fov));
    } else if (kind == "scan"_L1) {
        double a0 = -135, a1 = 135;
        if (z.contains(u"min"_s) && std::isfinite(z.value(u"min"_s).toDouble())) {
            a0 = z.value(u"min"_s).toDouble() / kDeg;
            a1 = z.value(u"max"_s).toDouble() / kDeg;
        }
        zone = {{u"type"_s, u"fan"_s}, {u"min"_s, a0}, {u"max"_s, a1}};
        caption = u"LaserScan · %1…%2° in the X–Y plane"_s.arg(fmt(a0), fmt(a1));
    } else {
        const QVariantMap dirs = z.value(u"dirs"_s).toMap();
        if (!dirs.value(u"bins"_s).toList().isEmpty()) {
            zone = dirs;
            zone.insert(u"type"_s, u"dirs"_s);
            caption = u"PointCloud2 · where the points came from, last 10 s"_s;
        } else if (m_cfg.value(u"profile"_s).toString() == "aerial"_L1) {
            zone = {{u"type"_s, u"band"_s}, {u"elMin"_s, -7}, {u"elMax"_s, 52}};
            caption = u"PointCloud2 · Mid-360 band until points arrive"_s;
        } else {
            caption = u"PointCloud2 · the zone shows when points arrive"_s;
        }
    }
    m_previewZone = zone;
    m_previewCaption = caption;
}

void RosController::renderStatus(const QVariantMap &st)
{
    const QString link = st.value(u"link"_s).toString();
    if (st.isEmpty() || link == "OFF"_L1) {
        const QString err = st.value(u"error"_s).toString();
        m_statusText = err.isEmpty() ? u"OFF"_s : span(u"err"_s, esc(err));
        return;
    }
    const QString state = st.value(u"state"_s).toString();
    QStringList lines;
    lines << u"%1 %2"_s.arg(span(link == "CONNECTED"_L1 ? u"ok"_s : u"warn"_s, esc(link)), esc(st.value(u"url"_s).toString()));
    const QString topic = st.value(u"topic"_s).toString(), frameId = st.value(u"frameId"_s).toString(), frame = st.value(u"frame"_s).toString();
    if (!topic.isEmpty()) {
        const QString fl = frame == "sensor"_L1 ? u"SENSOR · placed with the telemetry pose"_s
                         : frame == "world-enu"_L1 ? u"WORLD ENU"_s
                         : frame == "world-ned"_L1 ? u"WORLD NED"_s : QString();
        lines << u"%1 · %2%3"_s.arg(esc(topic), esc(shortType(st.value(u"type"_s).toString())),
                                                 frameId.isEmpty() ? QString() : u" · frame %1 → %2"_s.arg(esc(frameId), fl));
    }
    if (st.value(u"timedSpan"_s).toDouble() > 0 && frame == "sensor"_L1)
        lines << u"beams measured over %1 s a message (time_increment): each placed with the pose of its own time"_s.arg(fmt(st.value(u"timedSpan"_s).toDouble(), 2));
    if (st.value(u"stampOff"_s).isValid())
        lines << span(u"warn"_s, u"header.stamp %1 s from this clock: not synced, arrival time used"_s.arg(fmt(st.value(u"stampOff"_s).toDouble() / 1000, 1)));
    const QVariantMap r = st.value(u"rate"_s).toMap();
    auto rn = [&](const char *k) { return r.value(QLatin1String(k)).toDouble(); };
    const QString despike = st.value(u"despike"_s).toString();
    QString rateLine = u"%1 msg/s · %2 pts/s in · %3 sampled · %4 averaged"_s.arg(fmtInt(rn("msgs")), fmtInt(rn("pointsIn")), fmtInt(rn("sampled")), fmtInt(rn("used")));
    if (despike == "on"_L1)
        rateLine += u" · %1 spikes dropped (%2 %)"_s.arg(fmtInt(rn("spikes")), fmt(100 * rn("spikes") / std::max(1.0, rn("sampled")), 1));
    else if (despike == "unordered"_L1)
        rateLine += u" · spike filter idle: the scan has no order"_s;
    lines << rateLine;
    QStringList drops;
    if (rn("noPose") > 0) {
        const QVariantMap m = st.value(u"poseMiss"_s).toMap();
        QString d = u"%1 msg/s without pose"_s.arg(fmtInt(rn("noPose")));
        // (points later than the newest telemetry: the telemetry lags; earlier than the oldest kept: the points do)
        if (m.value(u"att"_s).isValid()) {
            const double att = m.value(u"att"_s).toDouble();
            d += u" (points %1, %2 ms after the oldest kept, clock ×%3)"_s
                     .arg(att >= 0 ? u"%1 ms after the newest attitude"_s.arg(fmtInt(att)) : u"%1 ms before it"_s.arg(fmtInt(-att)),
                          fmtInt(m.value(u"oldest"_s).toDouble()), fmt(m.value(u"clockRate"_s).toDouble(), 1));
        }
        drops << d;
    }
    if (rn("noHome") > 0) drops << u"%1 msg/s without home"_s.arg(fmtInt(rn("noHome")));
    if (!drops.isEmpty()) lines << span(u"warn"_s, drops.join(u" · "_s));
    const QVariantMap g = st.value(u"surface"_s).toMap();
    auto gn = [&](const char *k) { return g.value(QLatin1String(k)).toDouble(); };
    const RosSurfaceData &d = RosSurfaceData::instance();
    if (g.value(u"layers"_s).toString() == "cave"_L1) {
        lines << u"3D surface: %1 / %2 chunks of 16³ voxels of %3 m · %4 triangles"_s
                     .arg(fmtInt(gn("chunks")), fmtInt(gn("maxChunks")), fmt(gn("cell"), 2), fmtInt(d.volumeTriangles()));
    } else {
        const double cell = gn("cell");
        lines << u"surface %1 / %2 tiles of %3×%3 · cell %4 m · %5 cells (%6 m²)"_s
                     .arg(fmtInt(gn("tiles")), fmtInt(gn("maxTiles")), fmtInt(gn("tileCells")), fmt(cell, 2), fmtInt(gn("filled")), fmtInt(gn("filled") * cell * cell));
        for (const QVariant &cv : g.value(u"coverage"_s).toList()) {
            const QVariantMap c = cv.toMap();
            const double area = c.value(u"area"_s).toDouble(), covered = c.value(u"covered"_s).toDouble();
            lines << u"planned %1: %2 km² · %3"_s
                         .arg(esc(c.value(u"name"_s).toString()), fmt(area / 1e6, 2),
                              span(area > 0 && covered / area > 0.9 ? u"ok"_s : u"warn"_s,
                                   u"%1 % covered"_s.arg(fmt(area > 0 ? 100 * covered / area : 0, 1))));
        }
        const RosSurfaceData::Stats ms = d.stats();
        double lvTotal = 0;
        for (auto n : ms.levels) lvTotal += n;
        QStringList res;
        for (int k = 0; k < 5; ++k)
            if (ms.levels[std::size_t(k)])
                res << u"%1 cm %2%"_s.arg(fmtInt(cell * 100 * (1 << k))).arg(std::lround(100 * ms.levels[std::size_t(k)] / lvTotal));
        lines << u"drawn %1 / %2 blocks · %3 triangles · resolution %4"_s
                     .arg(fmtInt(d.drawnBlocks), fmtInt(ms.blocks), fmtInt(d.drawnTriangles), res.isEmpty() ? u"—"_s : res.join(u" · "_s));
    }
    const bool rel = m_hub && m_hub->snapshot()->relativeMode;
    QString last = u"%1 · pose %2"_s.arg(span(state == "ACCUMULATING"_L1 ? u"ok"_s : u"warn"_s, esc(state)),
                                                     rel ? u"relative (local frame)"_s : u"absolute (GPS)"_s);
    if (st.value(u"vertical"_s).toString() == "water"_L1)
        last += st.value(u"waterAlt"_s).isValid() ? u" · height: the water surface at home (%1 m)"_s.arg(fmt(st.value(u"waterAlt"_s).toDouble(), 1))
                                                              : u" · height: the water surface at home, no home yet"_s;
    const double clock = st.value(u"clockRate"_s).toDouble();
    if (clock > 0 && std::abs(clock - 1) > 0.1) last += u" · autopilot clock ×%1 (SITL speedup)"_s.arg(fmt(clock, 1));
    lines << last;
    const QString err = st.value(u"error"_s).toString();
    if (!err.isEmpty() && link != "CONNECTED"_L1) lines << span(u"err"_s, esc(err));
    m_statusText = lines.join(u"<br>"_s);
}

void RosController::updateStrip(const QVariantMap &st)
{
    if (!m_enabled || st.isEmpty()) return;
    const QString s = st.value(u"state"_s).toString();
    m_stripState = s == "ACCUMULATING"_L1 ? u"accum"_s
                 : (s == "CONNECTING"_L1 || s == "NO ROSBRIDGE"_L1) ? u"search"_s : u"hold"_s;
    m_stripLabel = s;
    const QVariantMap g = st.value(u"surface"_s).toMap();
    const bool cave = g.value(u"layers"_s).toString() == "cave"_L1;
    const double have = cave ? g.value(u"chunks"_s).toDouble() : g.value(u"filled"_s).toDouble();
    const double cell = g.value(u"cell"_s).toDouble();
    const RosSurfaceData &d = RosSurfaceData::instance();
    QString text = !have ? u"—"_s
                 : cave  ? u"3D · %1 triangles · %2 cm"_s.arg(fmtInt(d.volumeTriangles()), fmtInt(cell * 100))
                         : u"%1 cells · %2 cm"_s.arg(fmtInt(g.value(u"filled"_s).toDouble()), fmtInt(cell * 100));
    const QString mode = colorMode();
    if (mode == "distance"_L1 && have) {
        text += u" · %1–%2 m"_s.arg(fmtInt(d.view.nearM), fmtInt(d.view.farM));
    } else if (mode == "rough"_L1 && have) {
        if (cave) {
            auto deg = [](double s) { return std::acos(std::clamp(1 - s, -1.0, 1.0)) / kDeg; };
            text += u" · blue ≤ %1° · red ≥ %2° of spread"_s.arg(fmtInt(deg(d.view.spreadMean)), fmtInt(deg(d.view.spreadFull)));
        } else {
            text += u" · blue ≤ %1 cm · red ≥ %2 cm off-plane"_s.arg(fmtInt(d.view.roughMean * 100.0), fmtInt(d.view.roughFull * 100.0));
        }
    }
    const QVariantList cov = g.value(u"coverage"_s).toList();
    if (!cave && !cov.isEmpty()) {
        double a = 0, c = 0;
        for (const QVariant &v : cov) {
            a += v.toMap().value(u"area"_s).toDouble();
            c += v.toMap().value(u"covered"_s).toDouble();
        }
        if (a > 0) text += u" · %1 % of the area"_s.arg(fmtInt(100 * c / a));
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
    m_active = m_enabled && st.value(u"link"_s).toString() == "CONNECTED"_L1;
    emit statusChanged();
    const QString state = st.value(u"state"_s).toString();
    if (!m_enabled || state == m_lastState) return;
    if (state == "ACCUMULATING"_L1)
        emit hudMessage(u"ROS: averaging %1"_s.arg(st.value(u"topic"_s).toString()), u"info"_s);
    else if (m_lastState == "ACCUMULATING"_L1)
        emit hudMessage(u"ROS: %1"_s.arg(state.toLower()), u"warning"_s);
    else if (state == "NO ROSBRIDGE"_L1 && m_lastState == "CONNECTING"_L1)
        emit hudMessage(u"ROS: no rosbridge at %1"_s.arg(st.value(u"url"_s).toString()), u"warning"_s);
    m_lastState = state;
}

} // namespace corv
