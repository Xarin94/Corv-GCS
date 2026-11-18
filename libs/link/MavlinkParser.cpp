#include "link/MavlinkParser.h"

namespace corv {

void MavlinkParser::push(const uint8_t *data, qsizetype len)
{
    for (qsizetype i = 0; i < len; ++i) {
        mavlink_message_t out{};
        mavlink_status_t outStatus{};
        const uint8_t r = mavlink_frame_char_buffer(&m_rx, &m_status, data[i], &out, &outStatus);
        if (r == MAVLINK_FRAMING_OK) {
            ++m_ok;
            if (m_onFrame) m_onFrame(out, serialize(out));
        } else if (r == MAVLINK_FRAMING_BAD_CRC || r == MAVLINK_FRAMING_BAD_SIGNATURE) {
            ++m_badCrc;
        }
    }
}

void MavlinkParser::reset()
{
    m_rx = {};
    m_status = {};
}

QByteArray MavlinkParser::serialize(const mavlink_message_t &msg)
{
    QByteArray buf(MAVLINK_MAX_PACKET_LEN, Qt::Uninitialized);
    const uint16_t n = mavlink_msg_to_send_buffer(reinterpret_cast<uint8_t *>(buf.data()), &msg);
    buf.resize(n);
    return buf;
}

} // namespace corv
