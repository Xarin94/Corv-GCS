#include "link/LinkTrace.h"

#include "core/Clock.h"
#include "core/DebugLog.h"

#include <QSet>
#include <algorithm>

using namespace Qt::StringLiterals;

namespace corv {

namespace {
constexpr qint64 kSnapshotMs = 10000;
constexpr qint64 kHbTimeoutMs = 3500;
constexpr int kTopEvery = 3;
const QSet<uint32_t> kTxCountOnly = {MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE, MAVLINK_MSG_ID_TERRAIN_DATA,
                                     MAVLINK_MSG_ID_HEARTBEAT, MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT,
                                     MAVLINK_MSG_ID_GPS_RTCM_DATA};
} // namespace

void LinkTrace::onConnect(const QString &what)
{
    m_connected = true;
    m_what = what;
    m_bytes = m_bytesAtSnap = m_msgs = m_msgsAtSnap = 0;
    m_lastSnap = wallMs();
    m_snaps = 0;
    m_comps.clear();
    m_counts.clear();
    m_txCounts.clear();
    m_baseMode = m_sysStatus = m_fix = m_ekfFlags = -1;
    m_customMode = -1;
    LOG_I("link") << "connected:" << what;
}

void LinkTrace::onDisconnect(const QString &why)
{
    if (!m_connected) return;
    m_connected = false;
    LOG_I("link") << "disconnected:" << why << u"(%1 messages, %2 kB)"_s.arg(m_msgs).arg(m_bytes / 1024);
}

void LinkTrace::onMessage(const MavMessage &mm)
{
    const mavlink_message_t &m = mm.msg;
    ++m_msgs;
    ++m_counts[m.msgid];
    const quint16 key = quint16((m.sysid << 8) | m.compid);
    const qint64 now = wallMs();
    auto it = m_comps.find(key);
    if (it == m_comps.end()) {
        it = m_comps.insert(key, Comp{});
        LOG_I("link") << u"component %1/%2 appeared (%3)"_s.arg(m.sysid).arg(m.compid).arg(mav::messageName(m.msgid));
    }
    Comp &c = it.value();
    if (!mm.synthetic) {
        if (c.lastSeq >= 0) {
            const int gap = (int(m.seq) - c.lastSeq - 1 + 256) % 256;
            if (gap < 128) c.lost += gap;
        }
        c.lastSeq = m.seq;
    }
    ++c.received;

    switch (m.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: {
        if (c.hbLost) LOG_I("link") << u"heartbeat of %1/%2 back"_s.arg(m.sysid).arg(m.compid);
        c.hbLost = false;
        c.lastHb = now;
        if (m.compid != 1) break;
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(&m, &hb);
        const bool armed = hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED;
        if (m_baseMode >= 0 && armed != bool(m_baseMode & MAV_MODE_FLAG_SAFETY_ARMED))
            LOG_I("vehicle") << (armed ? "ARMED" : "DISARMED");
        if (m_customMode >= 0 && (long long)hb.custom_mode != m_customMode)
            LOG_I("vehicle") << "mode" << m_customMode << "->" << hb.custom_mode;
        if (m_sysStatus >= 0 && hb.system_status != m_sysStatus)
            LOG_I("vehicle") << "system status" << m_sysStatus << "->" << hb.system_status;
        m_baseMode = hb.base_mode;
        m_customMode = hb.custom_mode;
        m_sysStatus = hb.system_status;
        break;
    }
    case MAVLINK_MSG_ID_STATUSTEXT: {
        mavlink_statustext_t st;
        mavlink_msg_statustext_decode(&m, &st);
        const QString text = mav::fieldString(st.text, 50);
        const auto lvl = st.severity <= MAV_SEVERITY_ERROR ? DebugLog::Error
                         : st.severity <= MAV_SEVERITY_WARNING ? DebugLog::Warning : DebugLog::Info;
        DebugLog::write(lvl, u"main"_s, u"[statustext] %1/%2 sev %3: %4"_s.arg(m.sysid).arg(m.compid).arg(st.severity).arg(text));
        break;
    }
    case MAVLINK_MSG_ID_COMMAND_ACK: {
        mavlink_command_ack_t a;
        mavlink_msg_command_ack_decode(&m, &a);
        if (a.command == MAV_CMD_REQUEST_MESSAGE || a.command == MAV_CMD_SET_MESSAGE_INTERVAL) {
            if (DebugLog::throttle(u"ack-polled"_s, 30000) < 0) break;
        }
        LOG_I("mav") << "COMMAND_ACK" << mav::commandName(a.command) << mav::resultName(a.result);
        break;
    }
    case MAVLINK_MSG_ID_MISSION_ACK: {
        mavlink_mission_ack_t a;
        mavlink_msg_mission_ack_decode(&m, &a);
        LOG_I("mav") << "MISSION_ACK type" << a.type << "mission_type" << a.mission_type;
        break;
    }
    case MAVLINK_MSG_ID_GPS_RAW_INT: {
        if (mm.synthetic) break;
        mavlink_gps_raw_int_t g;
        mavlink_msg_gps_raw_int_decode(&m, &g);
        if (m_fix >= 0 && g.fix_type != m_fix) LOG_I("vehicle") << "GPS fix" << m_fix << "->" << g.fix_type << "sats" << g.satellites_visible;
        m_fix = g.fix_type;
        break;
    }
    case MAVLINK_MSG_ID_EKF_STATUS_REPORT: {
        mavlink_ekf_status_report_t e;
        mavlink_msg_ekf_status_report_decode(&m, &e);
        if (m_ekfFlags >= 0 && e.flags != m_ekfFlags)
            LOG_I("vehicle") << u"EKF flags 0x%1 -> 0x%2"_s.arg(m_ekfFlags, 0, 16).arg(e.flags, 0, 16);
        m_ekfFlags = e.flags;
        break;
    }
    default: break;
    }
}

void LinkTrace::onSend(const mavlink_message_t &m)
{
    ++m_txCounts[m.msgid];
    if (kTxCountOnly.contains(m.msgid)) return;
    if (m.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
        mavlink_command_long_t c;
        mavlink_msg_command_long_decode(&m, &c);
        if ((c.command == MAV_CMD_REQUEST_MESSAGE || c.command == MAV_CMD_SET_MESSAGE_INTERVAL)
            && DebugLog::throttle(u"tx-polled-%1"_s.arg(c.command), 30000) < 0)
            return;
        LOG_I("tx") << "COMMAND_LONG" << mav::commandName(c.command)
                    << u"p1=%1 p2=%2 p7=%3"_s.arg(c.param1).arg(c.param2).arg(c.param7);
        return;
    }
    if (m.msgid == MAVLINK_MSG_ID_MISSION_ITEM_INT || m.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_READ
        || m.msgid == MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL || m.msgid == MAVLINK_MSG_ID_MISSION_REQUEST_INT) {
        if (DebugLog::throttle(u"tx-burst-%1"_s.arg(m.msgid), 2000) < 0) return;
    }
    LOG_I("tx") << mav::messageName(m.msgid);
}

void LinkTrace::tick()
{
    if (!m_connected) return;
    const qint64 now = wallMs();
    for (auto it = m_comps.begin(); it != m_comps.end(); ++it) {
        Comp &c = it.value();
        if (c.lastHb && !c.hbLost && now - c.lastHb > kHbTimeoutMs) {
            c.hbLost = true;
            LOG_W("link") << u"heartbeat of %1/%2 lost"_s.arg(it.key() >> 8).arg(it.key() & 0xFF);
        }
    }
    if (now - m_lastSnap < kSnapshotMs) return;
    const double s = (now - m_lastSnap) / 1000.0;
    qint64 received = 0, lost = 0;
    for (const Comp &c : std::as_const(m_comps)) { received += c.received; lost += c.lost; }
    const double lossPct = received + lost ? 100.0 * lost / double(received + lost) : 0.0;
    LOG_I("link") << u"%1: %2 kB/s, %3 msg/s, loss %4%"_s
                         .arg(m_what)
                         .arg((m_bytes - m_bytesAtSnap) / 1024.0 / s, 0, 'f', 1)
                         .arg((m_msgs - m_msgsAtSnap) / s, 0, 'f', 0)
                         .arg(lossPct, 0, 'f', 1);
    m_bytesAtSnap = m_bytes;
    m_msgsAtSnap = m_msgs;
    m_lastSnap = now;
    if (++m_snaps % kTopEvery == 0) {
        QList<QPair<qint64, uint32_t>> top;
        for (auto it = m_counts.cbegin(); it != m_counts.cend(); ++it) top << qMakePair(it.value(), it.key());
        std::sort(top.begin(), top.end(), [](auto &a, auto &b) { return a.first > b.first; });
        QStringList parts;
        for (int i = 0; i < std::min<qsizetype>(8, top.size()); ++i)
            parts << u"%1 %2"_s.arg(mav::messageName(top[i].second)).arg(top[i].first);
        LOG_I("link") << "top messages:" << parts.join(u", "_s);
        m_counts.clear();
    }
}

} // namespace corv
