#include "vehicle/CommandSender.h"

#include "core/DebugLog.h"
#include "link/LinkManager.h"
#include "link/MessageBus.h"
#include "vehicle/FlightModes.h"
#include "vehicle/VehicleHub.h"

#include <QDateTime>
#include <QPointer>
#include <QTimer>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>

using namespace Qt::StringLiterals;

namespace corv {

namespace {
constexpr int kPreflightCalTimeoutMs = 30000;
} // namespace

CommandSender::CommandSender(LinkManager *link, MessageBus *bus, VehicleHub *hub, QObject *parent)
    : QObject(parent), m_link(link), m_bus(bus), m_hub(hub)
{
}

int CommandSender::targetSystem() const { return m_hub ? m_hub->snapshot()->systemId : 1; }
int CommandSender::targetComponent() const { return m_hub ? m_hub->snapshot()->componentId : 1; }

void CommandSender::sendMsg(const mavlink_message_t &m)
{
    if (m_link) m_link->send(m);
}

void CommandSender::commandLong(uint16_t command, float p1, float p2, float p3, float p4, float p5, float p6, float p7)
{
    mavlink_message_t m;
    mavlink_msg_command_long_pack(mav::kGcsSystemId, mav::kGcsComponentId, &m, uint8_t(targetSystem()),
                                  uint8_t(targetComponent()), command, 0, p1, p2, p3, p4, p5, p6, p7);
    sendMsg(m);
}

void CommandSender::commandInt(uint16_t command, uint8_t frame, float p1, float p2, float p3, float p4, int32_t x,
                               int32_t y, float z)
{
    mavlink_message_t m;
    mavlink_msg_command_int_pack(mav::kGcsSystemId, mav::kGcsComponentId, &m, uint8_t(targetSystem()),
                                 uint8_t(targetComponent()), frame, command, 0, 0, p1, p2, p3, p4, x, y, z);
    sendMsg(m);
}

void CommandSender::commandWithAck(uint16_t command, float p1, float p2, float p3, float p4, float p5, float p6,
                                   float p7, int timeoutMs, AckDone done)
{
    struct Wait {
        int token = 0;
        bool finished = false;
        QTimer timer;
    };
    auto w = std::make_shared<Wait>();
    QPointer<CommandSender> self(this);
    auto finish = [w, self, done](bool got, const Ack &a) {
        if (w->finished) return;
        w->finished = true;
        w->timer.stop();
        if (self && self->m_bus) self->m_bus->unsubscribe(w->token);
        if (done) done(got, a);
    };
    w->token = m_bus->subscribe(MAVLINK_MSG_ID_COMMAND_ACK, this, [command, finish](const MavMessage &mm) {
        mavlink_command_ack_t a;
        mavlink_msg_command_ack_decode(&mm.msg, &a);
        if (a.command != command || a.result == MAV_RESULT_IN_PROGRESS) return;
        finish(true, Ack{a.result, mav::resultName(a.result)});
    });
    w->timer.setSingleShot(true);
    connect(&w->timer, &QTimer::timeout, this, [finish, command] {
        LOG_W("cmd") << "no COMMAND_ACK for command" << command << "(timeout)";
        finish(false, Ack{});
    });
    w->timer.start(timeoutMs);
    commandLong(command, p1, p2, p3, p4, p5, p6, p7);
}

void CommandSender::armVehicle(bool force) { commandLong(MAV_CMD_COMPONENT_ARM_DISARM, 1, force ? 21196 : 0); }
void CommandSender::disarmVehicle(bool force) { commandLong(MAV_CMD_COMPONENT_ARM_DISARM, 0, force ? 21196 : 0); }

bool CommandSender::setFlightMode(const QString &modeName)
{
    const auto s = m_hub->snapshot();
    const int num = modes::number(modeName, s->vehicleType, s->connectionType);
    if (num < 0) {
        LOG_W("cmd") << "unknown mode" << modeName;
        return false;
    }
    setFlightModeNum(num);
    return true;
}

void CommandSender::setFlightModeNum(int modeNum)
{
    mavlink_message_t m;
    mavlink_msg_set_mode_pack(mav::kGcsSystemId, mav::kGcsComponentId, &m, uint8_t(targetSystem()), 209, uint32_t(modeNum));
    sendMsg(m);
}

void CommandSender::waitFor(std::function<bool(const VehicleState &)> cond, int timeoutMs, std::function<void(bool)> then)
{
    auto *t = new QTimer(this);
    t->setInterval(100);
    auto left = std::make_shared<int>(timeoutMs / 100);
    connect(t, &QTimer::timeout, this, [this, t, left, cond = std::move(cond), then = std::move(then)] {
        const bool ok = cond(*m_hub->snapshot());
        if (!ok && --*left > 0) return;
        t->deleteLater();
        t->stop();
        then(ok);
    });
    t->start();
}

void CommandSender::takeoff(double altitude, Done done)
{
    // ArduCopter (and a VTOL plane) needs GUIDED for the takeoff command, then armed. A fixed
    // wing has a mode for it, with the height in a parameter: GUIDED would fly it to its last
    // guided point from the ground. Each step waits for the one before: on a slow link the
    // command came before the mode or the arming it needs.
    const auto s = m_hub->snapshot();
    const bool fixedWing = modes::tableFor(s->vehicleType, s->connectionType) == modes::Table::Plane && !modes::isVtolType(s->vehicleType);
    const QString mode = fixedWing ? u"TAKEOFF"_s : u"GUIDED"_s;
    auto end = [done](bool ok, const QString &why) { if (done) done(ok, why); };
    if (!setFlightMode(mode)) return end(false, u"%1 is not a mode of this vehicle"_s.arg(mode));
    if (fixedWing) setParameter(u"TKOFF_ALT"_s, float(altitude), MAV_PARAM_TYPE_REAL32, [](bool, float) {});
    waitFor([mode](const VehicleState &v) { return v.flightMode == mode; }, 4000, [this, mode, altitude, fixedWing, end](bool ok) {
        if (!ok) return end(false, u"the vehicle did not go to %1"_s.arg(mode));
        auto go = [this, altitude, fixedWing, end] {
            if (!fixedWing) commandLong(MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, float(altitude));
            end(true, QString());
        };
        if (m_hub->snapshot()->armed) return go();
        armVehicle();
        waitFor([](const VehicleState &v) { return v.armed; }, 6000, [go, end](bool armed) {
            if (armed) go();
            else end(false, u"the vehicle did not arm"_s);
        });
    });
}

void CommandSender::land() { commandLong(MAV_CMD_NAV_LAND); }
void CommandSender::landSequence() { commandLong(MAV_CMD_DO_LAND_START); }
void CommandSender::returnToLaunch() { setFlightMode(u"RTL"_s); }
void CommandSender::setMissionSpeed(double speed, int speedType) { commandLong(MAV_CMD_DO_CHANGE_SPEED, float(speedType), float(speed), -1); }

void CommandSender::setHome(std::optional<double> lat, std::optional<double> lon, std::optional<double> alt, AckDone done)
{
    const bool current = !lat || !lon;
    // COMMAND_INT: int32 degE7 positions, no float32 precision loss
    QPointer<CommandSender> self(this);
    struct Holder { int token = 0; bool done = false; QTimer timer; };
    auto h = std::make_shared<Holder>();
    auto finish = [h, self, done](bool got, const Ack &a) {
        if (h->done) return;
        h->done = true;
        h->timer.stop();
        if (self) self->m_bus->unsubscribe(h->token);
        if (got && a.result == MAV_RESULT_ACCEPTED && self) {
            // Drop the stale home, then pull the new one
            self->m_hub->mutate([](VehicleState &s) { s.homeLat.reset(); s.homeLon.reset(); s.homeAlt.reset(); });
            QTimer::singleShot(300, self, [self] { if (self) self->requestHomePosition(); });
        }
        if (done) done(got, a);
    };
    h->token = m_bus->subscribe(MAVLINK_MSG_ID_COMMAND_ACK, this, [finish](const MavMessage &mm) {
        mavlink_command_ack_t a;
        mavlink_msg_command_ack_decode(&mm.msg, &a);
        if (a.command != MAV_CMD_DO_SET_HOME || a.result == MAV_RESULT_IN_PROGRESS) return;
        finish(true, Ack{a.result, mav::resultName(a.result)});
    });
    h->timer.setSingleShot(true);
    connect(&h->timer, &QTimer::timeout, this, [finish] { finish(false, Ack{}); });
    h->timer.start(3000);
    commandInt(MAV_CMD_DO_SET_HOME, MAV_FRAME_GLOBAL, current ? 1.f : 0.f, 0, 0, 0,
               current ? 0 : int32_t(std::llround(*lat * 1e7)), current ? 0 : int32_t(std::llround(*lon * 1e7)),
               current ? 0.f : float(alt.value_or(0)));
}

void CommandSender::requestHomePosition() { commandLong(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_HOME_POSITION); }
void CommandSender::requestAutopilotVersion() { commandLong(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_AUTOPILOT_VERSION); }
void CommandSender::rebootAutopilot() { commandLong(MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1); }



void CommandSender::sendRcChannelsOverride(const std::array<uint16_t, 18> &in)
{
    // 0 releases a channel; the extension channels 9-18 need UINT16_MAX-1 for that
    std::array<uint16_t, 18> ch = in;
    for (int i = 8; i < 18; ++i)
        if (ch[i] == 0) ch[i] = 65534;
    mavlink_message_t m;
    mavlink_msg_rc_channels_override_pack(mav::kGcsSystemId, mav::kGcsComponentId, &m, uint8_t(targetSystem()),
                                          uint8_t(targetComponent()), ch[0], ch[1], ch[2], ch[3], ch[4], ch[5], ch[6],
                                          ch[7], ch[8], ch[9], ch[10], ch[11], ch[12], ch[13], ch[14], ch[15], ch[16], ch[17]);
    sendMsg(m);
}

void CommandSender::sendServoTest(int servo, int pwm) { commandLong(MAV_CMD_DO_SET_SERVO, float(servo), float(pwm)); }
void CommandSender::sendRelayToggle(int relay, int state) { commandLong(MAV_CMD_DO_SET_RELAY, float(relay), float(state)); }

void CommandSender::calibrateAccel(AckDone d) { commandWithAck(MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0, 0, 0, 1, 0, 0, kPreflightCalTimeoutMs, std::move(d)); }
void CommandSender::calibrateLevel(AckDone d) { commandWithAck(MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0, 0, 0, 2, 0, 0, kPreflightCalTimeoutMs, std::move(d)); }
void CommandSender::calibrateAccelSimple(AckDone d) { commandWithAck(MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0, 0, 0, 4, 0, 0, kPreflightCalTimeoutMs, std::move(d)); }
void CommandSender::calibrateGyro(AckDone d) { commandWithAck(MAV_CMD_PREFLIGHT_CALIBRATION, 1, 0, 0, 0, 0, 0, 0, kPreflightCalTimeoutMs, std::move(d)); }
void CommandSender::calibrateBaro(AckDone d) { commandWithAck(MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0, 1, 0, 0, 0, 0, kPreflightCalTimeoutMs, std::move(d)); }
void CommandSender::sendAccelCalPosition(int pos, AckDone d) { commandWithAck(MAV_CMD_ACCELCAL_VEHICLE_POS, float(pos), 0, 0, 0, 0, 0, 0, 3000, std::move(d)); }

void CommandSender::startMagCal(int mask, bool retry, bool autosave, float delay, bool autoreboot, AckDone d)
{
    commandWithAck(MAV_CMD_DO_START_MAG_CAL, float(mask), retry ? 1.f : 0.f, autosave ? 1.f : 0.f, delay,
                   autoreboot ? 1.f : 0.f, 0, 0, 3000, std::move(d));
}
void CommandSender::acceptMagCal(int mask, AckDone d) { commandWithAck(MAV_CMD_DO_ACCEPT_MAG_CAL, float(mask), 0, 0, 0, 0, 0, 0, 3000, std::move(d)); }
void CommandSender::cancelMagCal(int mask, AckDone d) { commandWithAck(MAV_CMD_DO_CANCEL_MAG_CAL, float(mask), 0, 0, 0, 0, 0, 0, 3000, std::move(d)); }
void CommandSender::magCalFixedYaw(float yawDeg, int mask, float lat, float lon, AckDone d)
{
    commandWithAck(MAV_CMD_FIXED_MAG_CAL_YAW, yawDeg, float(mask), lat, lon, 0, 0, 0, 3000, std::move(d));
}

void CommandSender::requestDataStream(int streamId, int rate, int start)
{
    mavlink_message_t m;
    mavlink_msg_request_data_stream_pack(mav::kGcsSystemId, mav::kGcsComponentId, &m, uint8_t(targetSystem()),
                                         uint8_t(targetComponent()), uint8_t(streamId), uint16_t(rate), uint8_t(start));
    sendMsg(m);
}

void CommandSender::setMessageInterval(uint32_t msgId, double rateHz)
{
    const float intervalUs = rateHz > 0 ? float(std::lround(1e6 / rateHz)) : -1.f;
    commandLong(MAV_CMD_SET_MESSAGE_INTERVAL, float(msgId), intervalUs, 0, 0, 0, 0, 0);
}

void CommandSender::resetMessageInterval(uint32_t msgId) { commandLong(MAV_CMD_SET_MESSAGE_INTERVAL, float(msgId), 0); }

void CommandSender::requestAllDataStreams()
{
    // MAV_DATA_STREAM ids and rates of the Electron version
    const int streams[][2] = {{1, 4}, {2, 2}, {3, 10}, {6, 25}, {10, 25}, {11, 20}, {12, 2}};
    for (const auto &s : streams) requestDataStream(s[0], s[1]);
    // Reliable backstop: the per-message rates the EXTRA streams are supposed to give
    const double msgs[][2] = {{30, 25}, {74, 20}, {31, 10}, {27, 10}, {241, 2}, {133, 1}, {136, 1},
                              {226, 4}, {11030, 4}};   // RPM and the ESC telemetry: where the vehicle has them
    for (const auto &m : msgs) setMessageInterval(uint32_t(m[0]), m[1]);
}

void CommandSender::setParameter(const QString &id, float value, int type, std::function<void(bool, float)> done)
{
    struct State {
        int attempt = 0;
        int token = 0;
        bool finished = false;
        QTimer timer;
    };
    auto st = std::make_shared<State>();
    QPointer<CommandSender> self(this);
    const QByteArray name = id.trimmed().toLatin1();
    auto sendOnce = [self, name, value, type] {
        if (!self) return;
        char pid[16] = {};
        std::memcpy(pid, name.constData(), std::min<qsizetype>(16, name.size()));
        mavlink_message_t m;
        mavlink_msg_param_set_pack(mav::kGcsSystemId, mav::kGcsComponentId, &m, uint8_t(self->targetSystem()),
                                   uint8_t(self->targetComponent()), pid, value, uint8_t(type));
        self->sendMsg(m);
    };
    auto finish = [st, self, done](bool ok, float v) {
        if (st->finished) return;
        st->finished = true;
        st->timer.stop();
        if (self) self->m_bus->unsubscribe(st->token);
        if (done) done(ok, v);
    };
    st->token = m_bus->subscribe(MAVLINK_MSG_ID_PARAM_VALUE, this, [name, finish](const MavMessage &mm) {
        mavlink_param_value_t pv;
        mavlink_msg_param_value_decode(&mm.msg, &pv);
        if (mav::fieldString(pv.param_id, 16).trimmed().toLatin1() == name) finish(true, pv.param_value);
    });
    st->timer.setInterval(2000);
    connect(&st->timer, &QTimer::timeout, this, [st, sendOnce, finish, id] {
        if (++st->attempt >= 3) {
            LOG_W("param") << "parameter" << id << "not acknowledged after 3 attempts";
            finish(false, 0);
            return;
        }
        LOG_W("param") << u"no ACK for %1 (attempt %2/3)"_s.arg(id).arg(st->attempt);
        sendOnce();
    });
    st->timer.start();
    sendOnce();
}

void CommandSender::requestAllParameters()
{
    mavlink_message_t m;
    mavlink_msg_param_request_list_pack(mav::kGcsSystemId, mav::kGcsComponentId, &m, uint8_t(targetSystem()), uint8_t(targetComponent()));
    sendMsg(m);
}

void CommandSender::requestParameter(const QString &id)
{
    char pid[16] = {};
    const QByteArray n = id.trimmed().toUpper().toLatin1();
    std::memcpy(pid, n.constData(), std::min<qsizetype>(16, n.size()));
    mavlink_message_t m;
    mavlink_msg_param_request_read_pack(mav::kGcsSystemId, mav::kGcsComponentId, &m, uint8_t(targetSystem()),
                                        uint8_t(targetComponent()), pid, -1);
    sendMsg(m);
}

void CommandSender::requestParameterIndex(int index)
{
    char pid[16] = {};
    mavlink_message_t m;
    mavlink_msg_param_request_read_pack(mav::kGcsSystemId, mav::kGcsComponentId, &m, uint8_t(targetSystem()),
                                        uint8_t(targetComponent()), pid, int16_t(index));
    sendMsg(m);
}

void CommandSender::fetchParameter(const QString &id, int timeoutMs, int retries, std::function<void(bool, float, int)> done)
{
    const QString target = id.trimmed().toUpper();
    if (target.isEmpty() || target.size() > 16) {
        if (done) done(false, 0, 0);
        return;
    }
    struct State { int attempt = 0; int token = 0; bool finished = false; QTimer timer; };
    auto st = std::make_shared<State>();
    QPointer<CommandSender> self(this);
    auto finish = [st, self, done](bool ok, float v, int t) {
        if (st->finished) return;
        st->finished = true;
        st->timer.stop();
        if (self) self->m_bus->unsubscribe(st->token);
        if (done) done(ok, v, t);
    };
    st->token = m_bus->subscribe(MAVLINK_MSG_ID_PARAM_VALUE, this, [target, finish](const MavMessage &mm) {
        mavlink_param_value_t pv;
        mavlink_msg_param_value_decode(&mm.msg, &pv);
        if (mav::fieldString(pv.param_id, 16).trimmed().toUpper() == target) finish(true, pv.param_value, pv.param_type);
    });
    st->timer.setInterval(timeoutMs);
    connect(&st->timer, &QTimer::timeout, this, [st, self, target, retries, finish] {
        if (++st->attempt > retries) {
            finish(false, 0, 0);
            return;
        }
        if (self) self->requestParameter(target);
    });
    st->timer.start();
    requestParameter(target);
}

void CommandSender::uploadMission(const QList<MissionItem> &items, Done done)
{
    if (items.isEmpty()) {
        if (done) done(false, u"No mission items to upload"_s);
        return;
    }
    LOG_I("mission") << "upload of" << items.size() << "items to" << targetSystem() << "/" << targetComponent();
    // The vehicle asks for the items one by one and asks again for the ones it did not get. What
    // it cannot mend is done here: a MISSION_COUNT that was lost is sent again, and a lost last
    // MISSION_ACK is replaced by asking how many items it holds. There is no limit on the whole
    // transfer, only on the silence: a long mission on a slow link takes its time.
    struct State {
        QList<int> tokens;
        bool finished = false;
        int lastSeq = -1, highest = -1;
        int counts = 0, verifies = 0;
        qint64 t0 = 0, heardAt = 0;
        QTimer timer;
    };
    auto st = std::make_shared<State>();
    st->t0 = st->heardAt = QDateTime::currentMSecsSinceEpoch();
    QPointer<CommandSender> self(this);
    const int n = int(items.size());
    auto finish = [st, self, done](bool ok, const QString &text) {
        if (st->finished) return;
        st->finished = true;
        st->timer.stop();
        if (self)
            for (int t : st->tokens) self->m_bus->unsubscribe(t);
        if (done) done(ok, text);
    };
    auto sendCount = [st, self, n] {
        if (!self) return;
        ++st->counts;
        mavlink_message_t m;
        mavlink_msg_mission_count_pack(mav::kGcsSystemId, mav::kGcsComponentId, &m, uint8_t(self->targetSystem()),
                                       uint8_t(self->targetComponent()), uint16_t(n), MAV_MISSION_TYPE_MISSION);
        self->sendMsg(m);
    };
    auto onRequest = [st, self, items](int seq) {
        if (!self) return;
        st->heardAt = QDateTime::currentMSecsSinceEpoch();
        if (seq >= items.size()) {
            LOG_W("mission") << "vehicle asked for item" << seq << "the mission has" << items.size();
            return;
        }
        st->lastSeq = seq;
        st->highest = std::max(st->highest, seq);
        const MissionItem &it = items[seq];
        mavlink_message_t m;
        mavlink_msg_mission_item_int_pack(mav::kGcsSystemId, mav::kGcsComponentId, &m, uint8_t(self->targetSystem()),
                                          uint8_t(self->targetComponent()), uint16_t(seq), uint8_t(it.frame),
                                          uint16_t(it.command), seq == 0 ? 1 : 0, 1, it.param1, it.param2, it.param3,
                                          it.param4, int32_t(std::llround(it.lat * 1e7)), int32_t(std::llround(it.lng * 1e7)),
                                          it.alt, MAV_MISSION_TYPE_MISSION);
        self->sendMsg(m);
    };
    st->tokens << m_bus->subscribe(MAVLINK_MSG_ID_MISSION_REQUEST_INT, this, [onRequest](const MavMessage &mm) {
        mavlink_mission_request_int_t r;
        mavlink_msg_mission_request_int_decode(&mm.msg, &r);
        if (r.mission_type == MAV_MISSION_TYPE_MISSION) onRequest(r.seq);
    });
    st->tokens << m_bus->subscribe(MAVLINK_MSG_ID_MISSION_REQUEST, this, [onRequest](const MavMessage &mm) {
        mavlink_mission_request_t r;
        mavlink_msg_mission_request_decode(&mm.msg, &r);
        if (r.mission_type == MAV_MISSION_TYPE_MISSION) onRequest(r.seq);
    });
    st->tokens << m_bus->subscribe(MAVLINK_MSG_ID_MISSION_ACK, this, [st, finish, n](const MavMessage &mm) {
        mavlink_mission_ack_t a;
        mavlink_msg_mission_ack_decode(&mm.msg, &a);
        if (a.mission_type != MAV_MISSION_TYPE_MISSION) return;
        if (a.type == MAV_MISSION_ACCEPTED) {
            LOG_I("mission") << u"upload accepted: %1 items in %2 ms"_s.arg(n).arg(QDateTime::currentMSecsSinceEpoch() - st->t0);
            finish(true, u"%1 items"_s.arg(n));
        } else if (a.type == MAV_MISSION_INVALID_SEQUENCE) {
            // A second copy of an item: the vehicle had asked again before the first came (an
            // answer slower than its patience). It drops the copy and goes on
            LOG_W("mission") << "a copy of item" << st->lastSeq << "dropped by the vehicle";
            st->heardAt = QDateTime::currentMSecsSinceEpoch();
        } else if (st->verifies && a.type == MAV_MISSION_DENIED) {
            // Still receiving: it will ask for the item it misses
        } else {
            LOG_E("mission") << u"upload rejected: MISSION_ACK type %1 after item %2"_s.arg(a.type).arg(st->lastSeq);
            finish(false, u"Mission ACK error: type=%1"_s.arg(a.type));
        }
    });
    st->tokens << m_bus->subscribe(MAVLINK_MSG_ID_MISSION_COUNT, this, [st, finish, n](const MavMessage &mm) {
        mavlink_mission_count_t c;
        mavlink_msg_mission_count_decode(&mm.msg, &c);
        if (!st->verifies || c.mission_type != MAV_MISSION_TYPE_MISSION) return;
        LOG_I("mission") << "no MISSION_ACK; the vehicle holds" << c.count << "items of" << n;
        if (c.count == n) finish(true, u"%1 items"_s.arg(n));
        else finish(false, u"the vehicle holds %1 items of %2"_s.arg(c.count).arg(n));
    });
    st->timer.setInterval(500);
    connect(&st->timer, &QTimer::timeout, this, [st, self, finish, sendCount, n] {
        if (!self) return;
        const qint64 quiet = QDateTime::currentMSecsSinceEpoch() - st->heardAt;
        if (quiet > 15000) {
            LOG_E("mission") << u"upload: silence after item %1 of %2"_s.arg(st->lastSeq).arg(n);
            finish(false, st->highest < 0 ? u"the vehicle did not ask for the mission"_s : u"Mission upload timeout"_s);
        } else if (st->highest < 0) {
            // Nobody asked for anything: the count was lost
            if (quiet >= 2500 * st->counts) sendCount();
        } else if (st->highest == n - 1 && quiet >= 3000 * (st->verifies + 1)) {
            // Everything was asked for and it went quiet: the acknowledgement was lost
            ++st->verifies;
            mavlink_message_t m;
            mavlink_msg_mission_request_list_pack(mav::kGcsSystemId, mav::kGcsComponentId, &m, uint8_t(self->targetSystem()),
                                                  uint8_t(self->targetComponent()), MAV_MISSION_TYPE_MISSION);
            self->sendMsg(m);
        }
    });
    st->timer.start();
    sendCount();
}

void CommandSender::downloadMission(std::function<void(int, int)> progress,
                                    std::function<void(bool, const QString &, const QList<MissionItem> &)> finished)
{
    struct State {
        QList<int> tokens;
        QList<MissionItem> items;
        int total = -1;
        bool finished = false;
        int tries = 0;              // of the request out now
        qint64 askedAt = 0;
        int patience = 2500;        // ms for an answer: grows with the link's own delay
        QTimer timer;
    };
    constexpr int kTries = 6;
    auto st = std::make_shared<State>();
    QPointer<CommandSender> self(this);
    const uint8_t sys = uint8_t(targetSystem()), comp = uint8_t(targetComponent());
    LOG_I("mission") << "download from" << sys << "/" << comp;

    auto cleanup = [st, self] {
        st->finished = true;
        st->timer.stop();
        if (self)
            for (int t : st->tokens) self->m_bus->unsubscribe(t);
    };
    auto fail = [st, cleanup, finished](const QString &msg) {
        if (st->finished) return;
        LOG_E("mission") << u"download failed: %1 (%2/%3 items)"_s.arg(msg).arg(st->items.size())
                                .arg(st->total < 0 ? u"?"_s : QString::number(st->total));
        cleanup();
        if (finished) finished(false, msg, {});
    };
    // A request that is not answered goes out again: on a radio either of the two gets lost
    auto ask = [st, self, sys, comp] {
        if (!self || st->finished) return;
        st->askedAt = QDateTime::currentMSecsSinceEpoch();
        st->timer.start(st->patience);
        mavlink_message_t m;
        if (st->total < 0)
            mavlink_msg_mission_request_list_pack(mav::kGcsSystemId, mav::kGcsComponentId, &m, sys, comp, MAV_MISSION_TYPE_MISSION);
        else
            mavlink_msg_mission_request_int_pack(mav::kGcsSystemId, mav::kGcsComponentId, &m, sys, comp, uint16_t(st->items.size()), MAV_MISSION_TYPE_MISSION);
        self->sendMsg(m);
    };
    auto request = [st, ask] {
        // The answer to a first request says how slow the link is
        if (st->tries == 0 && st->askedAt)
            st->patience = int(std::clamp<qint64>(3 * (QDateTime::currentMSecsSinceEpoch() - st->askedAt), 1200, 5000));
        st->tries = 0;
        ask();
    };
    auto finish = [st, self, cleanup, finished, sys, comp] {
        if (st->finished) return;
        LOG_I("mission") << "download complete:" << st->items.size() << "items";
        cleanup();
        if (self) {
            mavlink_message_t m;
            mavlink_msg_mission_ack_pack(mav::kGcsSystemId, mav::kGcsComponentId, &m, sys, comp, MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
            self->sendMsg(m);
        }
        if (finished) finished(true, {}, st->items);
    };
    auto onItem = [st, request, finish, progress](int seq, const MissionItem &it) {
        if (st->finished || seq != st->items.size()) return;   // duplicate or out of order: the timer decides
        st->items.append(it);
        if (progress) progress(int(st->items.size()), st->total);
        if (st->items.size() >= st->total) finish();
        else request();
    };

    st->tokens << m_bus->subscribe(MAVLINK_MSG_ID_MISSION_COUNT, this, [st, request, finish, progress](const MavMessage &mm) {
        if (st->total >= 0 || st->finished) return;
        mavlink_mission_count_t c;
        mavlink_msg_mission_count_decode(&mm.msg, &c);
        if (c.mission_type != MAV_MISSION_TYPE_MISSION) return;
        st->total = c.count;
        LOG_I("mission") << "download: vehicle has" << st->total << "items";
        if (progress) progress(0, st->total);
        if (st->total == 0) finish();
        else request();
    });
    st->tokens << m_bus->subscribe(MAVLINK_MSG_ID_MISSION_ITEM_INT, this, [onItem](const MavMessage &mm) {
        mavlink_mission_item_int_t r;
        mavlink_msg_mission_item_int_decode(&mm.msg, &r);
        if (r.mission_type != MAV_MISSION_TYPE_MISSION) return;
        MissionItem it;
        it.command = r.command;
        it.frame = r.frame;
        it.param1 = r.param1; it.param2 = r.param2; it.param3 = r.param3; it.param4 = r.param4;
        it.lat = r.x / 1e7;
        it.lng = r.y / 1e7;
        it.alt = r.z;
        onItem(r.seq, it);
    });
    st->tokens << m_bus->subscribe(MAVLINK_MSG_ID_MISSION_ITEM, this, [onItem](const MavMessage &mm) {
        // Legacy autopilots answer with float coordinates
        mavlink_mission_item_t r;
        mavlink_msg_mission_item_decode(&mm.msg, &r);
        if (r.mission_type != MAV_MISSION_TYPE_MISSION) return;
        MissionItem it;
        it.command = r.command;
        it.frame = r.frame;
        it.param1 = r.param1; it.param2 = r.param2; it.param3 = r.param3; it.param4 = r.param4;
        it.lat = r.x;
        it.lng = r.y;
        it.alt = r.z;
        onItem(r.seq, it);
    });
    st->timer.setSingleShot(true);
    connect(&st->timer, &QTimer::timeout, this, [st, ask, fail] {
        if (++st->tries >= kTries) fail(u"Vehicle did not answer"_s);
        else ask();
    });
    ask();
}

} // namespace corv
