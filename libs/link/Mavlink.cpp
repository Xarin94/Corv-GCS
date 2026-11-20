// Message names come from the generated info tables, compiled in this file only
#include <cstddef>
#include <cstring>
#define MAVLINK_USE_MESSAGE_INFO
#include "link/Mavlink.h"
#include <mavlink_get_info.h>

#include <QHash>

using namespace Qt::StringLiterals;

namespace corv::mav {

QString messageName(uint32_t msgid)
{
    const mavlink_message_info_t *info = mavlink_get_message_info_by_id(msgid);
    return info ? QString::fromLatin1(info->name) : u"MSG_%1"_s.arg(msgid);
}

QString commandName(uint32_t cmd)
{
    static const QHash<uint32_t, const char *> names = {
        {16, "NAV_WAYPOINT"}, {20, "NAV_RETURN_TO_LAUNCH"}, {21, "NAV_LAND"}, {22, "NAV_TAKEOFF"},
        {176, "DO_SET_MODE"}, {178, "DO_CHANGE_SPEED"}, {179, "DO_SET_HOME"}, {181, "DO_SET_RELAY"},
        {183, "DO_SET_SERVO"}, {186, "DO_CHANGE_ALTITUDE"}, {241, "PREFLIGHT_CALIBRATION"},
        {246, "PREFLIGHT_REBOOT_SHUTDOWN"}, {400, "COMPONENT_ARM_DISARM"}, {511, "SET_MESSAGE_INTERVAL"},
        {512, "REQUEST_MESSAGE"}, {42006, "FIXED_MAG_CAL_YAW"}, {42424, "DO_START_MAG_CAL"},
        {42425, "DO_ACCEPT_MAG_CAL"}, {42426, "DO_CANCEL_MAG_CAL"}, {42429, "ACCELCAL_VEHICLE_POS"},
    };
    const auto it = names.constFind(cmd);
    return it != names.constEnd() ? u"MAV_CMD_%1(%2)"_s.arg(QLatin1String(*it)).arg(cmd)
                                  : u"CMD_%1"_s.arg(cmd);
}

QString resultName(uint8_t r)
{
    static const char *names[] = {"ACCEPTED", "TEMPORARILY_REJECTED", "DENIED", "UNSUPPORTED",
                                  "FAILED", "IN_PROGRESS", "CANCELLED"};
    return r < 7 ? QString::fromLatin1(names[r]) : u"RESULT_%1"_s.arg(r);
}

bool finalizeForSend(mavlink_message_t &msg, uint8_t seq)
{
    const mavlink_msg_entry_t *e = mavlink_get_msg_entry(msg.msgid);
    if (!e) return false;
    if (msg.len < e->max_msg_len)
        std::memset(reinterpret_cast<uint8_t *>(_MAV_PAYLOAD_NON_CONST(&msg)) + msg.len, 0, std::size_t(e->max_msg_len - msg.len));
    mavlink_status_t st{};
    st.current_tx_seq = seq;
    mavlink_finalize_message_buffer(&msg, kGcsSystemId, kGcsComponentId, &st, e->min_msg_len, e->max_msg_len, e->crc_extra);
    return true;
}

QString fieldString(const char *chars, int maxLen)
{
    int n = 0;
    while (n < maxLen && chars[n]) ++n;
    return QString::fromLatin1(chars, n);
}

} // namespace corv::mav
