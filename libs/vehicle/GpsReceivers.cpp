#include "vehicle/GpsReceivers.h"

namespace corv {

namespace {
template <typename T>
void fill(GpsReading &r, const T &g, std::int64_t now)
{
    r.fix = g.fix_type;
    r.sats = g.satellites_visible == 255 ? 0 : g.satellites_visible;
    r.hdop = g.eph != UINT16_MAX ? g.eph / 100.0 : 99.9;
    // Extensions: zero when the autopilot does not send them
    r.hAcc = g.h_acc / 1000.0;
    r.vAcc = g.v_acc / 1000.0;
    r.velAcc = g.vel_acc / 1000.0;
    r.lat = g.lat;
    r.lon = g.lon;
    r.altMm = g.alt;
    r.vel = g.vel;
    r.cog = g.cog;
    r.time = now;
}
} // namespace

int GpsReceivers::feed(const mavlink_message_t &m, std::int64_t now)
{
    switch (m.msgid) {
    case MAVLINK_MSG_ID_GPS_RAW_INT: {
        mavlink_gps_raw_int_t g;
        mavlink_msg_gps_raw_int_decode(&m, &g);
        fill(m_rx[0], g, now);
        return 0;
    }
    case MAVLINK_MSG_ID_GPS2_RAW: {
        mavlink_gps2_raw_t g;
        mavlink_msg_gps2_raw_decode(&m, &g);
        fill(m_rx[1], g, now);
        return 1;
    }
    default: return -1;
    }
}

int GpsReceivers::best(std::int64_t now) const
{
    int out = -1;
    for (int i = 0; i < int(m_rx.size()); ++i) {
        const GpsReading &r = m_rx[std::size_t(i)];
        if (!r.time || now - r.time > kStaleMs) continue;
        if (out < 0) { out = i; continue; }
        const GpsReading &b = m_rx[std::size_t(out)];
        const bool better = r.fix != b.fix ? r.fix > b.fix
                          : r.sats != b.sats ? r.sats > b.sats
                          : r.hdop < b.hdop;
        if (better) out = i;
    }
    return out;
}

} // namespace corv
