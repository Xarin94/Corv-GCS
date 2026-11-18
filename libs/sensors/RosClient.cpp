#include "sensors/RosClient.h"

#include "core/Clock.h"
#include "core/DebugLog.h"
#include "core/Geo.h"
#include "link/Mavlink.h"
#include "link/MessageBus.h"
#include "scene/SurfaceRaster.h"
#include "vehicle/VehicleHub.h"

#include <QCborArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QWebSocket>
#include <algorithm>
#include <cmath>

namespace corv {

namespace {
constexpr int kReconnectMs = 2000;
constexpr int kTilesMs = 250;
constexpr std::size_t kPoseCap = 1024;   // samples kept: 40 s at 25 Hz, 2 s of a SITL at speedup 20
constexpr double kClockSpanMs = 1500;    // samples spanning at least this give the autopilot's clock rate
constexpr double kMaxProjectMs = 500;    // a pose is projected at most this far from its sample
constexpr double kStampTrustMs = 2000;
constexpr int kMaxPoints = 20000;
constexpr double kGroupS = 0.005;        // beams this close in time share a pose
constexpr int kTail = 8;                 // beams of the last timed message the next one's spike filter sees
constexpr qint64 kTailMs = 2000;
constexpr qint64 kDeferMs = 1500;        // a message ahead of the telemetry waits this long for it
constexpr std::size_t kPendingCap = 64;
constexpr int kDirAz = 36, kDirEl = 18;
constexpr qint64 kDirKeepMs = 10000;
constexpr int kDirSamples = 200;
constexpr int kCoverageMs = 2000;
constexpr int kPoseMs = 50;

rospts::Anchor toPts(const RosAnchor &a) { return {a.lat, a.lon, a.alt, a.mPerLat, a.mPerLon}; }

QVariantMap anchorMap(const std::optional<RosAnchor> &a)
{
    if (!a) return {};
    return {{QStringLiteral("lat"), a->lat}, {QStringLiteral("lon"), a->lon}, {QStringLiteral("alt"), a->alt},
            {QStringLiteral("rel"), a->rel}, {QStringLiteral("epoch"), a->epoch}};
}
} // namespace

RosClient::RosClient(VehicleHub *hub, MessageBus *bus, QObject *parent) : QObject(parent), m_hub(hub), m_bus(bus)
{
    qRegisterMetaType<corv::RosTilesMsg>();
    qRegisterMetaType<corv::RosVolumeMsg>();
    m_pts.resize(std::size_t(kMaxPoints + kTail) * 3);
    m_times.resize(std::size_t(kMaxPoints + kTail));
    m_tailPts.resize(std::size_t(kTail) * 3);
    m_enu.resize(std::size_t(kMaxPoints) * 3);
    m_dirSeen.assign(std::size_t(kDirAz * kDirEl), 0);
    m_reconnect = new QTimer(this);
    m_reconnect->setSingleShot(true);
    m_reconnect->setInterval(kReconnectMs);
    connect(m_reconnect, &QTimer::timeout, this, &RosClient::open);
    applyConfig({});
}

RosClient::~RosClient()
{
    if (m_ws) m_ws->abort();
}

void RosClient::start()
{
    m_tilesTimer = new QTimer(this);
    m_tilesTimer->setInterval(kTilesMs);
    connect(m_tilesTimer, &QTimer::timeout, this, [this] {
        postTiles();
        drainPending();
    });
    m_tilesTimer->start();
    m_statusTimer = new QTimer(this);
    m_statusTimer->setInterval(1000);
    connect(m_statusTimer, &QTimer::timeout, this, [this] {
        m_lastRate = m_rate;
        m_rate = {};
        postStatus();
    });
    m_statusTimer->start();
    m_coverageTimer = new QTimer(this);
    m_coverageTimer->setInterval(kCoverageMs);
    connect(m_coverageTimer, &QTimer::timeout, this, &RosClient::updateCoverage);
    m_coverageTimer->start();
    m_poseTimer = new QTimer(this);
    m_poseTimer->setInterval(kPoseMs);
    connect(m_poseTimer, &QTimer::timeout, this, &RosClient::snapshotTick);
    if (m_bus) {
        m_bus->subscribe(MAVLINK_MSG_ID_ATTITUDE, this, [this](const MavMessage &m) { onBusMessage(m); });
        m_bus->subscribe(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, this, [this](const MavMessage &m) { onBusMessage(m); });
    }
}

// ============== CONNECTION ==============

void RosClient::connectTo(const QString &url)
{
    disconnectLink();
    m_url = url;
    m_link = QStringLiteral("CONNECTING");
    open();
    if (m_poseTimer) m_poseTimer->start();
    postStatus();
}

void RosClient::open()
{
    m_reconnect->stop();
    if (m_ws) {
        m_ws->disconnect(this);
        m_ws->abort();
        m_ws->deleteLater();
    }
    auto *ws = new QWebSocket(QString(), QWebSocketProtocol::VersionLatest, this);
    m_ws = ws;
    connect(ws, &QWebSocket::connected, this, [this, ws] {
        if (m_ws != ws) return;
        m_link = QStringLiteral("CONNECTED");
        m_lastError.clear();
        LOG_I("ros") << "rosbridge connected:" << m_url;
        listTopics();
        subscribe();
        postStatus();
    });
    connect(ws, &QWebSocket::errorOccurred, this, [this, ws](QAbstractSocket::SocketError) {
        if (m_ws != ws) return;
        m_lastError = QStringLiteral("cannot reach %1").arg(m_url);
        if (ws->state() != QAbstractSocket::ConnectedState) scheduleReconnect();
    });
    connect(ws, &QWebSocket::disconnected, this, [this, ws] {
        if (m_ws != ws) return;
        scheduleReconnect();
    });
    connect(ws, &QWebSocket::textMessageReceived, this, &RosClient::onText);
    connect(ws, &QWebSocket::binaryMessageReceived, this, &RosClient::onBinary);
    ws->open(QUrl(m_url));
}

void RosClient::scheduleReconnect()
{
    if (m_link == QLatin1String("OFF")) return;
    if (!m_subId.isEmpty()) m_subId.clear();
    if (m_link != QLatin1String("RECONNECTING")) LOG_W("ros") << "rosbridge link lost:" << m_url << m_lastError;
    m_link = QStringLiteral("RECONNECTING");
    if (!m_reconnect->isActive()) m_reconnect->start();
    postStatus();
}

void RosClient::disconnectLink()
{
    m_reconnect->stop();
    unsubscribe();
    if (m_ws) {
        QWebSocket *ws = m_ws;
        m_ws = nullptr;
        ws->disconnect(this);
        ws->close();
        ws->deleteLater();
    }
    if (m_poseTimer) m_poseTimer->stop();
    m_link = QStringLiteral("OFF");
    m_topics.clear();
}

void RosClient::sendJson(const QVariantMap &m)
{
    if (!m_ws || m_ws->state() != QAbstractSocket::ConnectedState) return;
    m_ws->sendTextMessage(QString::fromUtf8(QJsonDocument(QJsonObject::fromVariantMap(m)).toJson(QJsonDocument::Compact)));
}

void RosClient::listTopics()
{
    if (!m_ws || m_ws->state() != QAbstractSocket::ConnectedState) return;
    m_topicsCallId = QStringLiteral("call_service:/rosapi/topics:%1").arg(m_nextId++);
    sendJson({{QStringLiteral("op"), QStringLiteral("call_service")},
              {QStringLiteral("id"), m_topicsCallId},
              {QStringLiteral("service"), QStringLiteral("/rosapi/topics")},
              {QStringLiteral("type"), QStringLiteral("rosapi/Topics")},
              {QStringLiteral("args"), QVariantMap{}}});
}

void RosClient::subscribe()
{
    unsubscribe();
    if (!m_ws || m_ws->state() != QAbstractSocket::ConnectedState || !m_sel || m_sel->kind.isEmpty()) return;
    m_subId = QStringLiteral("subscribe:%1:%2").arg(m_sel->topic).arg(m_nextId++);
    sendJson({{QStringLiteral("op"), QStringLiteral("subscribe")},
              {QStringLiteral("id"), m_subId},
              {QStringLiteral("type"), m_sel->type},
              {QStringLiteral("topic"), m_sel->topic},
              {QStringLiteral("compression"), m_cfg.transport == QLatin1String("json") ? QStringLiteral("none") : QStringLiteral("cbor")},
              {QStringLiteral("throttle_rate"), int(std::lround(1000 / std::max(0.1, m_cfg.rateHz)))},
              {QStringLiteral("queue_length"), 1}});
    LOG_I("ros") << "subscribed to" << m_sel->topic << m_sel->type << "at" << m_cfg.rateHz << "Hz," << m_cfg.transport;
}

void RosClient::unsubscribe()
{
    if (m_subId.isEmpty()) return;
    if (m_sel) sendJson({{QStringLiteral("op"), QStringLiteral("unsubscribe")}, {QStringLiteral("id"), m_subId}, {QStringLiteral("topic"), m_sel->topic}});
    m_subId.clear();
}

void RosClient::onText(const QString &text)
{
    const QJsonDocument doc = QJsonDocument::fromJson(text.toUtf8());
    if (!doc.isObject()) return;
    onRosMessage(QCborValue::fromJsonValue(doc.object()).toMap());
}

void RosClient::onBinary(const QByteArray &data)
{
    const QCborValue v = QCborValue::fromCbor(data);
    if (v.isMap()) onRosMessage(v.toMap());
}

void RosClient::onRosMessage(const QCborMap &m)
{
    const QString op = m.value(QLatin1String("op")).toString();
    if (op == QLatin1String("publish")) {
        if (!m_sel || m.value(QLatin1String("topic")).toString() != m_sel->topic || m_subId.isEmpty()) return;
        onMessage(m.value(QLatin1String("msg")).toMap(), wallMs());
    } else if (op == QLatin1String("service_response")) {
        if (m.value(QLatin1String("id")).toString() != m_topicsCallId) return;
        const QCborValue values = m.value(QLatin1String("values"));
        const QCborValue result = m.value(QLatin1String("result"));
        if (result.isFalse() || !values.isMap()) {
            m_lastError = QStringLiteral("rosapi: %1 — is rosapi running (rosbridge_websocket_launch)?")
                              .arg(values.isString() ? values.toString() : QStringLiteral("no topics"));
            postStatus();
            return;
        }
        const QCborArray names = values.toMap().value(QLatin1String("topics")).toArray();
        const QCborArray types = values.toMap().value(QLatin1String("types")).toArray();
        m_topics.clear();
        for (qsizetype i = 0; i < names.size(); ++i) {
            const QString type = types.at(i).toString();
            m_topics << QVariantMap{{QStringLiteral("name"), names.at(i).toString()}, {QStringLiteral("type"), type}, {QStringLiteral("kind"), rospts::kindOf(type)}};
        }
        emit topicsChanged(m_topics);
    }
}

// ============== POSE ==============

// What the state says, every 50 ms: the position and attitude now (the
// relative frame's position, a replay), or why there is none to place points with
void RosClient::snapshotTick()
{
    if (!m_hub) return;
    const auto s = m_hub->snapshot();
    PoseSnap p;
    const qint64 now = wallMs();
    if (s->isDemo()) {
        p.reason = QStringLiteral("NO VEHICLE");
    } else if (!std::isfinite(s->lat) || !std::isfinite(s->lon) || (!s->relativeMode && s->lat == 0 && s->lon == 0)) {
        p.reason = QStringLiteral("NO POSITION");
    } else {
        p.valid = true;
        p.rel = s->relativeMode;
        if (p.rel) {
            p.home = RosAnchor{s->sceneOrigin.lat, s->sceneOrigin.lon, 0};
        } else if (s->homeLat && s->homeLon) {
            const double alt = s->subSurfaceAlt ? *s->subSurfaceAlt : s->homeAlt.value_or(0);
            p.home = RosAnchor{*s->homeLat, *s->homeLon, alt};
        }
        // The relative position is recomputed every frame: it is as of now
        p.tA = s->attTime ? double(s->attTime) : double(now);
        p.tP = p.rel ? double(now) : (s->posTime ? double(s->posTime) : double(now));
        p.lat = s->lat;
        p.lon = s->lon;
        p.alt = s->rawAlt;
        p.roll = s->roll;
        p.pitch = s->pitch;
        p.yaw = s->yaw;
        p.p = s->rollRate;
        p.q = s->pitchRate;
        p.r = s->yawRate;
        p.vn = s->vn;
        p.ve = s->ve;
        p.vd = s->vd;
    }
    std::vector<rospts::AttSample> att;
    std::vector<double> attBoot, posBoot;
    std::vector<rospts::PosSample> pos;
    if (p.valid) {
        for (const AttS &a : m_newAtt) {
            att.push_back({a.t, a.roll, a.pitch, a.yaw, a.p, a.q, a.r});
            attBoot.push_back(a.b);
        }
        if (!p.rel)
            for (const PosS &q : m_newPos) {
                pos.push_back({q.t, q.lat, q.lon, q.alt, q.vn, q.ve, q.vd});
                posBoot.push_back(q.b);
            }
    }
    m_newAtt.clear();
    m_newPos.clear();
    pushPose(p, att, attBoot, pos, posBoot);
    drainPending();
}

// Every ATTITUDE / GLOBAL_POSITION_INT, kept until the next snapshot
void RosClient::onBusMessage(const MavMessage &m)
{
    if (m_link == QLatin1String("OFF") || !m_hub) return;
    const auto s = m_hub->snapshot();
    if (m.id() == MAVLINK_MSG_ID_ATTITUDE) {
        mavlink_attitude_t a;
        mavlink_msg_attitude_decode(&m.msg, &a);
        if (!std::isfinite(a.roll) || !std::isfinite(a.pitch) || !std::isfinite(a.yaw)) return;
        const bool own = !m.synthetic;
        m_newAtt.push_back({double(m.arrivalMs), a.time_boot_ms ? double(a.time_boot_ms) : std::nan(""), a.roll, a.pitch, a.yaw,
                            own ? a.rollspeed : s->rollRate, own ? a.pitchspeed : s->pitchRate, own ? a.yawspeed : s->yawRate});
        if (m_newAtt.size() > kPoseCap) m_newAtt.erase(m_newAtt.begin());
    } else if (m.id() == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
        mavlink_global_position_int_t g;
        mavlink_msg_global_position_int_decode(&m.msg, &g);
        if (s->relativeMode || g.lat == 0 || g.lon == 0) return;
        // A sub reports its depth below the surface it started on (as StateMapper)
        const double alt = s->vehicleType == MAV_TYPE_SUBMARINE && s->subSurfaceAlt && !m.synthetic ? *s->subSurfaceAlt + g.relative_alt / 1000.0 : g.alt / 1000.0;
        m_newPos.push_back({double(m.arrivalMs), g.time_boot_ms ? double(g.time_boot_ms) : std::nan(""), g.lat / 1e7, g.lon / 1e7, alt,
                            g.vx / 100.0, g.vy / 100.0, g.vz / 100.0, false, std::nullopt});
        if (m_newPos.size() > kPoseCap) m_newPos.erase(m_newPos.begin());
    }
}

// The snapshot (p), and every ATTITUDE / GLOBAL_POSITION_INT that arrived
// since the last one (att, pos: absolute navigation only); a snapshot value
// is used where there are none (the relative position, a replay)
void RosClient::pushPose(const PoseSnap &p, const std::vector<rospts::AttSample> &att, const std::vector<double> &attBoot,
                         const std::vector<rospts::PosSample> &pos, const std::vector<double> &posBoot)
{
    if (!p.valid) {
        m_poseReason = p.reason.isEmpty() ? QStringLiteral("NO POSE") : p.reason;
        return;
    }
    auto pushAtt = [this](const AttS &s) {
        // The autopilot rebooted (a SITL relaunched): its clock starts over
        if (!m_attRing.empty() && !std::isnan(s.b) && !std::isnan(m_attRing.back().b) && s.b < m_attRing.back().b) m_attRing.clear();
        m_attRing.push_back(s);
        if (m_attRing.size() > kPoseCap) m_attRing.pop_front();
    };
    auto pushPos = [this](const PosS &s) {
        if (!m_posRing.empty() && !std::isnan(s.b) && !std::isnan(m_posRing.back().b) && s.b < m_posRing.back().b) m_posRing.clear();
        m_posRing.push_back(s);
        if (m_posRing.size() > kPoseCap) m_posRing.pop_front();
    };
    if (!att.empty()) {
        for (std::size_t i = 0; i < att.size(); ++i) {
            const auto &a = att[i];
            pushAtt({a.t, i < attBoot.size() ? attBoot[i] : std::nan(""), a.roll, a.pitch, a.yaw, a.p, a.q, a.r});
        }
    } else if (m_attRing.empty() || p.tA != m_attRing.back().t) {
        pushAtt({p.tA, std::nan(""), p.roll, p.pitch, p.yaw, p.p, p.q, p.r});
    }
    if (!pos.empty()) {
        for (std::size_t i = 0; i < pos.size(); ++i) {
            const auto &q = pos[i];
            pushPos({q.t, i < posBoot.size() ? posBoot[i] : std::nan(""), q.lat, q.lon, q.alt, q.vn, q.ve, q.vd, p.rel, p.home});
        }
    } else if (m_posRing.empty() || p.tP != m_posRing.back().t || p.rel != m_posRing.back().rel) {
        pushPos({p.tP, std::nan(""), p.lat, p.lon, p.alt, p.vn, p.ve, p.vd, p.rel, p.home});
    }
    autopilotClock();
}

// The autopilot's clock against this one: through the least delayed attitude
// sample of the oldest and of the newest third of those with a time_boot_ms;
// until they span kClockSpanMs, the arrival times
void RosClient::autopilotClock()
{
    std::vector<const AttS *> s;
    for (const AttS &a : m_attRing)
        if (!std::isnan(a.b)) s.push_back(&a);
    if (s.size() < 6 || s.back()->t - s.front()->t < kClockSpanMs) {
        m_bootOff.reset();
        return;
    }
    const std::size_t third = s.size() / 3;
    auto least = [&](std::size_t from, std::size_t to) {
        const AttS *best = s[from];
        double bv = 1e300;
        for (std::size_t i = from; i < to; ++i) {
            const double v = s[i]->t - s[i]->b / m_clockRate;
            if (v < bv) {
                bv = v;
                best = s[i];
            }
        }
        return best;
    };
    const AttS *a = least(0, third), *z = least(s.size() - third, s.size());
    if (z->t - a->t >= kClockSpanMs && z->b > a->b) m_clockRate = std::max(0.05, std::min(200.0, (z->b - a->b) / (z->t - a->t)));
    m_bootOff = z->t - z->b / m_clockRate;
}

// A telemetry sample's time on this clock: its boot time mapped, else its arrival
double RosClient::timeOf(double t, double b) const { return !std::isnan(b) && m_bootOff ? b / m_clockRate + *m_bootOff : t; }

// Pose at time t (ms, this clock), projected from the telemetry samples;
// none when the nearest sample is more than kMaxProjectMs away
std::optional<RosClient::Pose> RosClient::poseAt(double t) const
{
    if (m_attRing.empty() || m_posRing.empty()) return std::nullopt;
    auto attAt = [&]() -> const AttS & {
        for (auto it = m_attRing.rbegin(); it != m_attRing.rend(); ++it)
            if (timeOf(it->t, it->b) <= t) return *it;
        return m_attRing.front();
    };
    auto posAt = [&]() -> const PosS & {
        for (auto it = m_posRing.rbegin(); it != m_posRing.rend(); ++it)
            if (timeOf(it->t, it->b) <= t) return *it;
        return m_posRing.front();
    };
    const AttS &a = attAt();
    const PosS &q = posAt();
    const double ta = timeOf(a.t, a.b), tq = timeOf(q.t, q.b);
    if (std::abs(t - ta) > kMaxProjectMs || std::abs(t - tq) > kMaxProjectMs) return std::nullopt;
    Pose p;
    p.p = rospts::projectPose({ta, a.roll, a.pitch, a.yaw, a.p, a.q, a.r}, {tq, q.lat, q.lon, q.alt, q.vn, q.ve, q.vd}, t, m_clockRate);
    p.rel = q.rel;
    p.home = q.home;
    // A boat at the water surface (VERTICAL 'water'): no home yet, no height
    if (m_cfg.vertical == QLatin1String("water")) {
        if (!q.home) return std::nullopt;
        p.p.alt = q.home->alt;
    }
    return p;
}

// Time of a message on this clock (see the header)
double RosClient::messageTime(const QCborMap &msg, double arrival)
{
    if (m_cfg.timeBase == QLatin1String("stamp")) {
        const QCborMap st = msg.value(QLatin1String("header")).toMap().value(QLatin1String("stamp")).toMap();
        if (!st.isEmpty()) {
            double sec = rospts::num(st.value(QLatin1String("sec")));
            if (!std::isfinite(sec)) sec = rospts::num(st.value(QLatin1String("secs")));
            double ns = rospts::num(st.value(QLatin1String("nanosec")));
            if (!std::isfinite(ns)) ns = rospts::num(st.value(QLatin1String("nsecs")));
            if (!std::isfinite(ns)) ns = 0;
            if (std::isfinite(sec)) {
                const double ts = sec * 1000 + ns / 1e6;
                if (std::abs(ts - arrival) <= kStampTrustMs) {
                    m_stampOff.reset();
                    m_timeRef = QStringLiteral("stamp");
                    return ts + m_cfg.lagMs;
                }
                m_stampOff = ts - arrival;
            }
        }
    }
    m_timeRef = QStringLiteral("arrival");
    return arrival - m_cfg.lagMs;
}

// Sensor origin at a pose: the vehicle plus the lever arm (FRD -> NED -> ENU),
// with m_ra already the pose's attitude
void RosClient::sensorOrigin(double out[3]) const
{
    const auto &R = m_ra;
    const double ln = R[0] * m_lever[0] + R[1] * m_lever[1] + R[2] * m_lever[2];
    const double le = R[3] * m_lever[0] + R[4] * m_lever[1] + R[5] * m_lever[2];
    const double ld = R[6] * m_lever[0] + R[7] * m_lever[1] + R[8] * m_lever[2];
    out[0] = m_veh[0] + le;
    out[1] = m_veh[1] + ln;
    out[2] = m_veh[2] - ld;
}

// Sensor-frame points, each with the pose of its time (t0: the first beam,
// ms here; m_times in sensor seconds), compacted into m_enu. Beams with no
// pose (the telemetry stopped) are dropped. Returns how many were placed.
int RosClient::placeTimed(int count, double t0, bool cave)
{
    int out = 0;
    for (int a = 0; a < count;) {
        int b = a + 1;
        while (b < count && m_times[std::size_t(b)] - m_times[std::size_t(a)] < kGroupS) ++b;
        const auto p = poseAt(t0 + 500 * (double(m_times[std::size_t(a)]) + m_times[std::size_t(b - 1)]) / m_clockRate);
        if (p && p->rel == m_anchor->rel) {
            m_veh = rospts::toEnu(toPts(*m_anchor), p->p.lat, p->p.lon, p->p.alt);
            m_ra = rospts::eulerToMatrix(p->p.roll, p->p.pitch, p->p.yaw);
            double *dst = m_enu.data() + std::size_t(out) * 3;
            rospts::sensorToEnu(m_pts.data() + std::size_t(a) * 3, b - a, m_rm, m_lever, m_ra, m_veh, dst);
            if (cave) {
                sensorOrigin(m_sensorPos);
                m_rate.used += m_volume.integrate(m_sensorPos, dst, b - a);
            }
            out += b - a;
        }
        a = b;
    }
    return out;
}

// ============== MESSAGES ==============
// In order: a message waits while its points are ahead of the newest
// telemetry (even its last beam past what a pose may be projected to)
void RosClient::onMessage(const QCborMap &msg, qint64 arrival)
{
    m_lastMsgAt = arrival;
    if (!m_pending.empty() || telemetryBehind(msg, double(arrival))) {
        m_pending.emplace_back(msg, arrival);
        if (m_pending.size() > kPendingCap) {
            const auto [m, at] = m_pending.front();
            m_pending.pop_front();
            handle(m, at);
        }
        return;
    }
    handle(msg, arrival);
}

bool RosClient::telemetryBehind(const QCborMap &msg, double arrival)
{
    if (m_attRing.empty()) return false;
    const AttS &la = m_attRing.back();
    return messageTime(msg, arrival) > timeOf(la.t, la.b) + kMaxProjectMs;
}

// The waiting messages whose telemetry has come, or that waited long enough
void RosClient::drainPending()
{
    const qint64 now = wallMs();
    while (!m_pending.empty()) {
        const auto &[msg, arrival] = m_pending.front();
        if (telemetryBehind(msg, double(arrival)) && now - arrival < kDeferMs) break;
        const auto item = m_pending.front();
        m_pending.pop_front();
        handle(item.first, item.second);
    }
}

void RosClient::handle(const QCborMap &msg, qint64 now)
{
    if (!m_sel) return;
    ++m_totalMsgs;
    ++m_rate.msgs;
    const QString frameId = msg.value(QLatin1String("header")).toMap().value(QLatin1String("frame_id")).toString();
    const QString frame = rospts::frameOf(frameId, m_cfg.frame);
    m_lastFrameId = frameId;
    m_lastFrame = frame;

    const bool sensor = frame == QLatin1String("sensor");
    const int maxPts = std::min(int(m_cfg.maxPoints), kMaxPoints);
    const rospts::Decoded decoded = rospts::decodePoints(m_sel->kind, msg, m_pts.data(), kMaxPoints, maxPts, sensor ? m_cfg.minRange : 0,
                                                         sensor ? m_cfg.maxRange : std::numeric_limits<double>::infinity(), m_times.data());
    const int total = decoded.total;
    int count = decoded.count;
    // Beams measured over time are placed one by one (in a world frame the
    // points are already placed)
    const bool timed = sensor && decoded.span > 0;
    m_timedSpan = decoded.span;
    m_rate.pointsIn += total;
    m_rate.sampled += count;
    // Spikes along the scan, in the sensor frame where the order is the beams'
    if (sensor && m_cfg.despike != QLatin1String("off") && m_sel->kind != QLatin1String("range")) {
        // A sweep sent in parts: the previous part's last beams lead in, and
        // this part's last beams are kept for the next (before filtering)
        int lead = 0;
        if (timed && m_tailN && now - m_tailAt < kTailMs) {
            std::copy_backward(m_pts.begin(), m_pts.begin() + count * 3, m_pts.begin() + (m_tailN + count) * 3);
            std::copy_backward(m_times.begin(), m_times.begin() + count, m_times.begin() + m_tailN + count);
            std::copy(m_tailPts.begin(), m_tailPts.begin() + m_tailN * 3, m_pts.begin());
            std::fill(m_times.begin(), m_times.begin() + m_tailN, -1.f);
            lead = m_tailN;
        }
        if (timed) {
            m_tailN = std::min(kTail, count);
            std::copy(m_pts.begin() + (lead + count - m_tailN) * 3, m_pts.begin() + (lead + count) * 3, m_tailPts.begin());
            m_tailAt = now;
        } else {
            m_tailN = 0;
        }
        // (a sweep's pings in rough water: aeration comes in runs - a wider window)
        const rospts::Despiked d = rospts::despike(m_pts.data(), lead + count, timed ? 6 : 4, 0.5, 0.08, timed ? m_times.data() : nullptr);
        int kept = d.count;
        if (lead) {
            // only this message's beams go on
            int out = 0;
            for (int k = 0; k < kept; ++k) {
                if (m_times[std::size_t(k)] < 0) continue;
                if (out != k) {
                    m_pts[std::size_t(out) * 3] = m_pts[std::size_t(k) * 3];
                    m_pts[std::size_t(out) * 3 + 1] = m_pts[std::size_t(k) * 3 + 1];
                    m_pts[std::size_t(out) * 3 + 2] = m_pts[std::size_t(k) * 3 + 2];
                    m_times[std::size_t(out)] = m_times[std::size_t(k)];
                }
                ++out;
            }
            kept = out;
        }
        m_rate.spikes += count - kept;
        count = kept;
        m_unordered = !d.ordered;
    }
    if (m_sel->kind == QLatin1String("range")) {
        m_zoneMeta = {{QStringLiteral("fov"), rospts::num(msg.value(QLatin1String("field_of_view")))}};
    } else if (m_sel->kind == QLatin1String("scan")) {
        // The angles of the last 10 s: a sector sonar sends its sweep in parts
        const double a0 = rospts::num(msg.value(QLatin1String("angle_min"))), a1 = rospts::num(msg.value(QLatin1String("angle_max")));
        m_scanSpans.push_back({std::min(a0, a1), std::max(a0, a1), double(now)});
        while (!m_scanSpans.empty() && now - m_scanSpans.front()[2] > kDirKeepMs) m_scanSpans.pop_front();
        double lo = 1e300, hi = -1e300;
        for (const auto &z : m_scanSpans) {
            lo = std::min(lo, z[0]);
            hi = std::max(hi, z[1]);
        }
        m_zoneMeta = {{QStringLiteral("min"), lo}, {QStringLiteral("max"), hi}};
    }
    if (sensor && count) {
        const int step = std::max(1, count / kDirSamples);
        for (int k = 0; k < count; k += step) {
            const double x = m_pts[std::size_t(k) * 3], y = m_pts[std::size_t(k) * 3 + 1], z = m_pts[std::size_t(k) * 3 + 2];
            const int az = std::min(kDirAz - 1, int(std::floor((std::atan2(y, x) + kPi) / (2 * kPi) * kDirAz)));
            const int el = std::min(kDirEl - 1, int(std::floor((std::atan2(z, std::hypot(x, y)) + kPi / 2) / kPi * kDirEl)));
            m_dirSeen[std::size_t(el * kDirAz + az)] = now;
        }
    }
    // (decoded first: the zone shows on the bench, with no vehicle yet)
    double t = messageTime(msg, double(now));
    if (timed && m_timeRef == QLatin1String("arrival")) t -= 1000 * decoded.span / m_clockRate;   // the first beam
    const auto pose = poseAt(t);
    if (!pose) {
        ++m_rate.noPose;
        PoseMiss miss;
        miss.at = now;
        if (!m_attRing.empty()) {
            miss.att = std::round(t - timeOf(m_attRing.back().t, m_attRing.back().b));
            miss.oldest = std::round(t - timeOf(m_attRing.front().t, m_attRing.front().b));
        }
        if (!m_posRing.empty()) miss.pos = std::round(t - timeOf(m_posRing.back().t, m_posRing.back().b));
        miss.clockRate = m_clockRate;
        m_poseMiss = miss;
        const bool noHome = m_cfg.vertical == QLatin1String("water") && !m_posRing.empty() && !m_posRing.back().home;
        m_blocked = noHome ? QStringLiteral("NO HOME (water surface)") : m_poseReason;
        return;
    }
    if (!count) {
        m_blocked = total ? QStringLiteral("NO VALID POINTS") : QStringLiteral("EMPTY MESSAGES");
        return;
    }

    if (!m_anchor) {
        const rospts::Anchor a = rospts::makeAnchor(pose->p.lat, pose->p.lon, pose->p.alt);
        m_anchor = RosAnchor{a.lat, a.lon, a.alt, a.mPerLat, a.mPerLon, pose->rel, ++m_epoch};
    } else if (m_anchor->rel != pose->rel) {
        ++m_rate.noPose;
        m_blocked = QStringLiteral("FRAME CHANGED · CLEAR");
        return;
    }
    const bool cave = m_cfg.layers == QLatin1String("cave");

    if (timed) {
        count = placeTimed(count, t, cave);
        if (!cave) m_rate.used += m_surface.add(m_enu.data(), count, m_veh[0], m_veh[1]);
        m_blocked.clear();
        return;
    }
    m_veh = rospts::toEnu(toPts(*m_anchor), pose->p.lat, pose->p.lon, pose->p.alt);
    if (sensor) {
        m_ra = rospts::eulerToMatrix(pose->p.roll, pose->p.pitch, pose->p.yaw);
        rospts::sensorToEnu(m_pts.data(), count, m_rm, m_lever, m_ra, m_veh, m_enu.data());
    } else {
        if (!pose->home) {
            ++m_rate.noHome;
            m_blocked = QStringLiteral("NO HOME (world frame)");
            return;
        }
        m_org = rospts::toEnu(toPts(*m_anchor), pose->home->lat, pose->home->lon, pose->home->alt);
        rospts::worldToEnu(m_pts.data(), count, frame == QLatin1String("world-ned"), m_org, m_enu.data());
    }
    if (cave) {
        // Rays from the sensor
        m_ra = rospts::eulerToMatrix(pose->p.roll, pose->p.pitch, pose->p.yaw);
        sensorOrigin(m_sensorPos);
        m_rate.used += m_volume.integrate(m_sensorPos, m_enu.data(), count);
    } else {
        m_rate.used += m_surface.add(m_enu.data(), count, m_veh[0], m_veh[1]);
    }
    m_blocked.clear();
}

// ============== OUTPUT ==============

void RosClient::postTiles()
{
    const SurfaceTiles::Changes t = m_resetPending ? m_surface.takeAll() : m_surface.takeChanges();
    if (m_resetPending || !t.tiles.empty() || !t.removed.empty())
        emit tilesChanged({m_resetPending, m_anchor, m_surface.cell(), t.tiles, t.removed});
    const SurfaceVolume::Changes v = m_resetPending ? m_volume.takeAll() : m_volume.takeChanges();
    if (m_resetPending || !v.meshes.empty() || !v.removed.empty())
        emit volumeChanged({m_resetPending, m_anchor, m_volume.cell(), v.meshes, v.removed});
    m_resetPending = false;
}

QString RosClient::state() const
{
    if (m_link == QLatin1String("OFF")) return QStringLiteral("OFF");
    if (m_link != QLatin1String("CONNECTED")) return m_lastError.isEmpty() ? QStringLiteral("CONNECTING") : QStringLiteral("NO ROSBRIDGE");
    if (!m_sel || m_sel->kind.isEmpty()) return QStringLiteral("NO TOPIC");
    if (wallMs() - m_lastMsgAt > 3000) return QStringLiteral("NO DATA");
    return m_blocked.isEmpty() ? QStringLiteral("ACCUMULATING") : m_blocked;
}

// The mission's survey areas (lat/lng polygons from the planner) and how much
// of each the surface covers - absolute navigation only: a relative frame has
// no place on the map
void RosClient::updateCoverage()
{
    m_coverage.clear();
    if (!m_anchor || m_anchor->rel || m_areas.isEmpty()) return;
    const rospts::Anchor a = toPts(*m_anchor);
    // in the blocks the mesh fills holes over: what is drawn
    const int block = 1 << raster::fillLevelsFor(m_cfg.fillM, m_surface.cell());
    for (const QVariant &v : std::as_const(m_areas)) {
        const QVariantMap area = v.toMap();
        std::vector<QPointF> poly;
        for (const QVariant &pv : area.value(QStringLiteral("points")).toList()) {
            const QVariantMap p = pv.toMap();
            const auto e = rospts::toEnu(a, p.value(QStringLiteral("lat")).toDouble(), p.value(QStringLiteral("lng")).toDouble(), a.alt);
            poly.emplace_back(e[0], e[1]);
        }
        const double areaM2 = polygonAreaXY(poly);
        m_coverage << QVariantMap{{QStringLiteral("name"), area.value(QStringLiteral("name"))},
                                  {QStringLiteral("area"), areaM2},
                                  {QStringLiteral("covered"), std::min(areaM2, coveredArea(m_surface, poly, block))}};
    }
}

QVariantMap RosClient::status() const
{
    const qint64 now = wallMs();
    QVariantList bins;
    for (std::size_t b = 0; b < m_dirSeen.size(); ++b)
        if (now - m_dirSeen[b] < kDirKeepMs) bins << int(b);
    QVariantMap zone = m_zoneMeta;
    zone.insert(QStringLiteral("dirs"), QVariantMap{{QStringLiteral("az"), kDirAz}, {QStringLiteral("el"), kDirEl}, {QStringLiteral("bins"), bins}});
    QVariantMap miss;
    if (m_poseMiss && now - m_poseMiss->at < 3000) {
        auto opt = [](const std::optional<double> &v) { return v ? QVariant(*v) : QVariant(); };
        miss = {{QStringLiteral("att"), opt(m_poseMiss->att)}, {QStringLiteral("pos"), opt(m_poseMiss->pos)},
                {QStringLiteral("oldest"), opt(m_poseMiss->oldest)}, {QStringLiteral("clockRate"), m_poseMiss->clockRate}};
    }
    const QVariantMap rate{{QStringLiteral("msgs"), m_lastRate.msgs},       {QStringLiteral("pointsIn"), m_lastRate.pointsIn},
                           {QStringLiteral("sampled"), m_lastRate.sampled}, {QStringLiteral("used"), m_lastRate.used},
                           {QStringLiteral("spikes"), m_lastRate.spikes},   {QStringLiteral("noPose"), m_lastRate.noPose},
                           {QStringLiteral("noHome"), m_lastRate.noHome}};
    const QVariantMap surface{{QStringLiteral("cell"), m_surface.cell()},     {QStringLiteral("maxTiles"), m_surface.maxTiles()},
                              {QStringLiteral("tileCells"), SurfaceTiles::kTile}, {QStringLiteral("layers"), m_cfg.layers},
                              {QStringLiteral("tiles"), m_surface.size()},     {QStringLiteral("filled"), m_surface.filled()},
                              {QStringLiteral("chunks"), m_volume.size()},     {QStringLiteral("maxChunks"), m_volume.maxChunks()},
                              {QStringLiteral("coverage"), m_coverage}};
    QVariant waterAlt;
    if (!m_posRing.empty() && m_posRing.back().home) waterAlt = m_posRing.back().home->alt;
    return {{QStringLiteral("link"), m_link},
            {QStringLiteral("url"), m_url},
            {QStringLiteral("error"), m_lastError},
            {QStringLiteral("state"), state()},
            {QStringLiteral("topic"), m_sel ? m_sel->topic : QString()},
            {QStringLiteral("type"), m_sel ? m_sel->type : QString()},
            {QStringLiteral("kind"), m_sel ? m_sel->kind : QString()},
            {QStringLiteral("frameId"), m_lastFrameId},
            {QStringLiteral("frame"), m_lastFrame},
            {QStringLiteral("timeBase"), m_cfg.timeBase},
            {QStringLiteral("stampOff"), m_stampOff ? QVariant(*m_stampOff) : QVariant()},
            {QStringLiteral("clockRate"), m_clockRate},
            {QStringLiteral("timedSpan"), m_timedSpan},
            {QStringLiteral("poseMiss"), miss},
            {QStringLiteral("vertical"), m_cfg.vertical},
            {QStringLiteral("waterAlt"), waterAlt},
            {QStringLiteral("despike"), m_cfg.despike != QLatin1String("off") ? (m_unordered ? QStringLiteral("unordered") : QStringLiteral("on")) : QStringLiteral("off")},
            {QStringLiteral("zone"), zone},
            {QStringLiteral("rate"), rate},
            {QStringLiteral("totalMsgs"), m_totalMsgs},
            {QStringLiteral("surface"), surface},
            {QStringLiteral("anchor"), anchorMap(m_anchor)}};
}

// ============== CONFIG ==============

void RosClient::applyConfig(const QVariantMap &c)
{
    auto num = [&](const char *key, double &field, bool *changed = nullptr) {
        const auto it = c.constFind(QLatin1String(key));
        if (it == c.cend()) return;
        bool ok = false;
        const double v = it->toDouble(&ok);
        if (!ok || !std::isfinite(v)) return;
        if (changed && v != field) *changed = true;
        field = v;
    };
    auto text = [&](const char *key, QString &field, bool *changed = nullptr) {
        const auto it = c.constFind(QLatin1String(key));
        if (it == c.cend()) return;
        if (changed && it->toString() != field) *changed = true;
        field = it->toString();
    };
    bool resub = false;
    text("transport", m_cfg.transport, &resub);
    num("rateHz", m_cfg.rateHz, &resub);
    num("maxPoints", m_cfg.maxPoints);
    text("frame", m_cfg.frame);
    num("lagMs", m_cfg.lagMs);
    text("timeBase", m_cfg.timeBase);
    num("minRange", m_cfg.minRange);
    num("maxRange", m_cfg.maxRange);
    text("despike", m_cfg.despike);
    num("mountRoll", m_cfg.mountRoll);
    num("mountPitch", m_cfg.mountPitch);
    num("mountYaw", m_cfg.mountYaw);
    num("leverX", m_cfg.leverX);
    num("leverY", m_cfg.leverY);
    num("leverZ", m_cfg.leverZ);
    num("cell", m_cfg.cell);
    num("memory", m_cfg.memory);
    num("maxTiles", m_cfg.maxTiles);
    text("layers", m_cfg.layers);
    num("fillM", m_cfg.fillM);
    text("vertical", m_cfg.vertical);
    m_rm = rospts::eulerToMatrix(m_cfg.mountRoll * kDeg, m_cfg.mountPitch * kDeg, m_cfg.mountYaw * kDeg);
    m_lever = {m_cfg.leverX, m_cfg.leverY, m_cfg.leverZ};
    if (m_surface.configure(m_cfg.cell, m_cfg.memory, m_cfg.maxTiles)) m_resetPending = true;
    // A chunk (16^3 voxels) costs as much memory as ~2 tiles
    if (m_volume.configure(m_cfg.cell, m_cfg.memory, std::max(64.0, std::round(m_cfg.maxTiles / 2)))) m_resetPending = true;
    if (resub && !m_subId.isEmpty()) subscribe();
}

void RosClient::clear()
{
    m_pending.clear();
    m_surface.reset();
    m_volume.reset();
    m_anchor.reset();
    m_blocked.clear();
    m_resetPending = true;
    postTiles();
}

void RosClient::select(const QString &topic, const QString &type)
{
    if (!topic.isEmpty()) m_sel = Sel{topic, type, rospts::kindOf(type)};
    else m_sel.reset();
    m_zoneMeta.clear();
    m_scanSpans.clear();
    m_tailN = 0;
    m_pending.clear();
    std::fill(m_dirSeen.begin(), m_dirSeen.end(), 0);
    subscribe();
    postStatus();
}

void RosClient::setAreas(const QVariantList &areas) { m_areas = areas; }

} // namespace corv
