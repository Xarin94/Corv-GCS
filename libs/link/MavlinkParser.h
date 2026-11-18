// MavlinkParser.h - bytes in, whole CRC-checked MAVLink frames out.
//
// Wraps mavlink_frame_char_buffer with its own state (no global channel), so
// any number of parsers can run (live link, replay, tests). The raw bytes of
// each frame are rebuilt with mavlink_msg_to_send_buffer: identical to what
// arrived, signature included, which is what the tlog records.
#pragma once

#include "link/Mavlink.h"

#include <QByteArray>
#include <functional>

namespace corv {

class MavlinkParser
{
public:
    using FrameFn = std::function<void(const mavlink_message_t &msg, const QByteArray &raw)>;

    explicit MavlinkParser(FrameFn onFrame) : m_onFrame(std::move(onFrame)) {}

    void push(const QByteArray &bytes) { push(reinterpret_cast<const uint8_t *>(bytes.constData()), bytes.size()); }
    void push(const uint8_t *data, qsizetype len);
    void reset();

    uint64_t framesOk() const { return m_ok; }
    uint64_t framesBadCrc() const { return m_badCrc; }

    static QByteArray serialize(const mavlink_message_t &msg);

private:
    FrameFn m_onFrame;
    mavlink_message_t m_rx{};
    mavlink_status_t m_status{};
    uint64_t m_ok = 0;
    uint64_t m_badCrc = 0;
};

} // namespace corv
