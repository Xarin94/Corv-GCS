// RosClient.h - rosbridge client, sampling and averaging off the GUI and render
// threads (port of js/ros/RosWorker.js and of the pose feed of RosController.js).
//
// Lives on a thread of its own ("ros"). A QWebSocket to rosbridge_server
// carries the rosbridge v2 protocol (what roslib did): the rosapi service that
// lists the topics, and one subscription. Load on the link is cut at the
// source: the subscription asks rosbridge for at most MAX RATE messages a
// second (throttle_rate) and keeps only the newest (queue_length 1), in CBOR
// so binary arrays are not base64-inflated. Each message then contributes at
// most POINTS PER MESSAGE sampled points. All that leaves this thread are the
// averaged tiles that changed (30 x 30 heights each, see SurfaceTiles) and,
// in a cave, the chunk meshes that changed, at most four times a second.
//
// The pose is the vehicle's own: every ATTITUDE and GLOBAL_POSITION_INT from
// the MessageBus (absolute navigation only for the position), and a snapshot
// of the state every 50 ms - the position the 3D view already draws the
// vehicle at, absolute (GPS) or in the relative navigation frame, so the mesh
// lands where the vehicle is in either mode. Each sample carries the arrival
// time of the message it comes from, the body rates and the NED velocity; the
// pose at the time of a message is PROJECTED from the last attitude before it
// with the gyro rates (Euler angle kinematics) and from the last position with
// the velocity. Without that, a copter yawing 90 deg/s with an attitude 50 ms
// old places a return 80 m away 6 m off: spikes wherever the scan crosses a slope.
//
// Time of a telemetry sample: not its arrival. Whatever holds the samples up
// on their way here delays the ones that came in meanwhile and bunches them at
// its end: a point measured during the hold finds no pose near its time. The
// autopilot's clock (time_boot_ms) is not delayed: each sample's time is its
// boot time mapped onto this clock by the line through the least delayed
// samples of the oldest and the newest third of those kept (a hold only ever
// adds delay) - which also gives the autopilot's clock rate, 1 on a vehicle,
// the speed-up in a SITL.
//
// A message whose points are newer than the telemetry the client has (the
// telemetry held up behind them) waits for it, up to DEFER_MS, in order.
//
// Time of a message: its arrival minus LAG (how much later than the
// telemetry the points reach the GCS), or - TIME BASE 'stamp', when the
// companion computer's clock is synced with this one (chrony / NTP / GPS) -
// its header.stamp plus LAG (then: how late the telemetry arrives). A stamp
// more than 2 s from this clock is not trusted: arrival is used and the
// status says so.
//
// Beams measured one after another. A LaserScan with a time_increment spans
// time - seconds for a mechanical sector-scanning sonar, one beam per ping -
// and each beam is placed with the pose of its own time (runs of beams within
// 5 ms share one). The stamp is the first beam's (sensor_msgs/LaserScan); a
// message arrives after its last, so with arrival time the first beam was
// measured `span` earlier. The sensor's seconds are taken as the autopilot's:
// the same on a vehicle, and in a simulation where both run on its clock.
// Such a sweep arrives a few pings at a time (one, from some drivers): the
// spike filter runs over the last beams of the previous message too, so a
// short message still has neighbours, and over 13 beams instead of 9 - in a
// chop, aeration comes in runs of pings.
//
// Vertical reference. The sensor's height is the vehicle's altitude, or - a
// boat, VERTICAL 'water' - the water surface at home: a boat is always at the
// surface, while its EKF altitude drifts with the barometer (decimetres in a
// survey, metres over a day) and would lift or sink the bed by as much, lane
// by lane.
//
// Surfaces. Over open ground or water the points average into a height per
// cell (SurfaceTiles). In a cave, a shaft, under a pier (SURFACES 'cave') a
// height per cell cannot hold floor, walls and ceiling together: there the
// returns are fused into a 3D volume of signed distances along their rays and
// meshed where it changes sign (SurfaceVolume).
#pragma once

#include "link/MavMessage.h"
#include "scene/RosSurfaceData.h"
#include "scene/SurfaceTiles.h"
#include "scene/SurfaceVolume.h"
#include "sensors/RosPoints.h"

#include <QCborMap>
#include <QObject>
#include <QPointer>
#include <QTimer>
#include <QVariantMap>
#include <deque>
#include <optional>
#include <vector>

QT_FORWARD_DECLARE_CLASS(QWebSocket)

namespace corv {

class MessageBus;
class VehicleHub;

/// Changed tiles of the height field: reset means the renderer starts over.
struct RosTilesMsg {
    bool reset = false;
    std::optional<RosAnchor> anchor;
    double cell = 0.3;
    std::vector<TileUpdate> tiles;
    std::vector<std::pair<int, int>> removed;
};
/// Changed chunk meshes of the 3D surface (triangles, metres from the anchor).
struct RosVolumeMsg {
    bool reset = false;
    std::optional<RosAnchor> anchor;
    double cell = 0.3;
    std::vector<VolumeMesh> meshes;
    std::vector<std::int64_t> removed;
};

class RosClient : public QObject
{
    Q_OBJECT
public:
    struct Config {
        QString transport = QStringLiteral("cbor");
        double rateHz = 5;
        double maxPoints = 400;
        QString frame = QStringLiteral("auto");
        double lagMs = 0;
        QString timeBase = QStringLiteral("arrival");
        double minRange = 0.5, maxRange = 100;
        QString despike = QStringLiteral("on");
        double mountRoll = 0, mountPitch = 0, mountYaw = 0;
        double leverX = 0, leverY = 0, leverZ = 0;
        double cell = 0.3, memory = 20, maxTiles = 2048;
        QString layers = QStringLiteral("floor");
        double fillM = 0;
        QString vertical = QStringLiteral("vehicle");
    };
    /// What the 50 ms state snapshot gives (RosController.poseSample).
    struct PoseSnap {
        bool valid = false;
        QString reason;
        double tA = 0, tP = 0;
        double lat = 0, lon = 0, alt = 0, roll = 0, pitch = 0, yaw = 0, p = 0, q = 0, r = 0, vn = 0, ve = 0, vd = 0;
        bool rel = false;
        std::optional<RosAnchor> home;   // lat, lon, alt only
    };

    /// hub and bus may be null (tests): no pose feed then, pushPose() drives it.
    RosClient(VehicleHub *hub, MessageBus *bus, QObject *parent = nullptr);
    ~RosClient() override;

    // ── Commands (on the client's thread) ──
    /// Timers and the pose feed; call once the object is on its thread.
    void start();
    void connectTo(const QString &url);
    void disconnectLink();
    void listTopics();
    void select(const QString &topic, const QString &type);
    void applyConfig(const QVariantMap &c);
    void clear();
    /// The planner's survey areas: [{name, points: [{lat, lng}]}].
    void setAreas(const QVariantList &areas);

    // ── Offline tests ──
    void pushPose(const PoseSnap &p, const std::vector<rospts::AttSample> &att, const std::vector<double> &attBoot,
                  const std::vector<rospts::PosSample> &pos, const std::vector<double> &posBoot);
    void onMessage(const QCborMap &msg, qint64 arrival);
    void drainPending();
    void postTiles();
    QVariantMap status() const;
    const SurfaceTiles &surface() const { return m_surface; }
    const SurfaceVolume &volume() const { return m_volume; }
    double clockRate() const { return m_clockRate; }

signals:
    void statusChanged(const QVariantMap &st);
    /// [{name, type, kind}]
    void topicsChanged(const QVariantList &list);
    void tilesChanged(const corv::RosTilesMsg &m);
    void volumeChanged(const corv::RosVolumeMsg &m);

private:
    struct AttS {
        double t, b;   // arrival, time_boot_ms (NaN: none)
        double roll, pitch, yaw, p, q, r;
    };
    struct PosS {
        double t, b;
        double lat, lon, alt, vn, ve, vd;
        bool rel;
        std::optional<RosAnchor> home;
    };
    struct Pose {
        rospts::Pose p;
        bool rel = false;
        std::optional<RosAnchor> home;
    };
    struct PoseMiss {
        qint64 at = 0;
        std::optional<double> att, pos, oldest;
        double clockRate = 1;
    };

    void open();
    void scheduleReconnect();
    void subscribe();
    void unsubscribe();
    void sendJson(const QVariantMap &m);
    void onText(const QString &text);
    void onBinary(const QByteArray &data);
    void onRosMessage(const QCborMap &m);
    void snapshotTick();
    void onBusMessage(const MavMessage &m);
    void autopilotClock();
    double timeOf(double t, double b) const;
    std::optional<Pose> poseAt(double t) const;
    double messageTime(const QCborMap &msg, double arrival);
    bool telemetryBehind(const QCborMap &msg, double arrival);
    void handle(const QCborMap &msg, qint64 now);
    int placeTimed(int count, double t0, bool cave);
    void sensorOrigin(double out[3]) const;
    void updateCoverage();
    QString state() const;
    void postStatus() { emit statusChanged(status()); }

    VehicleHub *m_hub;
    MessageBus *m_bus;
    Config m_cfg;
    QPointer<QWebSocket> m_ws;
    QTimer *m_reconnect = nullptr;
    QString m_url;
    QString m_link = QStringLiteral("OFF");
    QString m_lastError;
    QVariantList m_topics;
    struct Sel {
        QString topic, type, kind;
    };
    std::optional<Sel> m_sel;
    QString m_subId;            // the live subscription's id, empty when none
    QString m_topicsCallId;
    int m_nextId = 1;
    qint64 m_lastMsgAt = 0;
    QString m_lastFrameId, m_lastFrame;

    SurfaceTiles m_surface;
    SurfaceVolume m_volume;
    std::optional<RosAnchor> m_anchor;
    int m_epoch = 0;
    bool m_resetPending = true;

    std::deque<AttS> m_attRing;
    std::deque<PosS> m_posRing;
    double m_clockRate = 1;
    std::optional<double> m_bootOff;
    QString m_poseReason = QStringLiteral("NO VEHICLE");
    std::optional<double> m_stampOff;
    std::vector<AttS> m_newAtt;   // bus samples since the last snapshot
    std::vector<PosS> m_newPos;

    std::array<double, 9> m_rm{}, m_ra{};
    std::array<double, 3> m_lever{}, m_veh{}, m_org{};
    std::vector<float> m_pts, m_times, m_tailPts;
    int m_tailN = 0;
    qint64 m_tailAt = 0;
    std::vector<double> m_enu;
    double m_sensorPos[3] = {0, 0, 0};

    // Per-second counters, and totals since the last clear
    struct Rate {
        qint64 msgs = 0, pointsIn = 0, sampled = 0, used = 0, spikes = 0, noPose = 0, noHome = 0;
    } m_rate, m_lastRate;
    bool m_unordered = false;
    double m_timedSpan = 0;
    QString m_timeRef = QStringLiteral("arrival");
    qint64 m_totalMsgs = 0;
    QString m_blocked;
    std::optional<PoseMiss> m_poseMiss;
    std::deque<std::pair<QCborMap, qint64>> m_pending;

    // Where the sensor collects from, for the mount preview: a Range's cone, a
    // LaserScan's fan, and for clouds in the sensor frame the directions (10 x
    // 10 deg bins of azimuth and elevation) points came from in the last 10 s
    std::vector<qint64> m_dirSeen;
    QVariantMap m_zoneMeta;
    std::deque<std::array<double, 3>> m_scanSpans;

    QVariantList m_areas;
    QVariantList m_coverage;
    QTimer *m_tilesTimer = nullptr, *m_statusTimer = nullptr, *m_poseTimer = nullptr, *m_coverageTimer = nullptr;
};

} // namespace corv

Q_DECLARE_METATYPE(corv::RosTilesMsg)
Q_DECLARE_METATYPE(corv::RosVolumeMsg)
