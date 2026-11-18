// MavMessage.h - the one shape every source hands on (ARCHITECTURE.md 5.8).
//
// Live MAVLink, MSP and log replay all produce MavMessage: a real
// mavlink_message_t (packed with the generated pack functions when the source
// is not MAVLink) plus when it arrived. Consumers decode it with the generated
// mavlink_msg_<name>_decode of the message they want.
//
// Fields a synthetic source does not report are left at their "unknown"
// encoding: time_boot_ms 0, vel/cog/eph 65535, hdg 65535. StateMapper treats
// those as absent, which is what the Electron mapper did with undefined.
#pragma once

#include "link/Mavlink.h"

#include <QMetaType>
#include <cstdint>

namespace corv {

struct MavMessage {
    mavlink_message_t msg{};
    std::int64_t arrivalMs = 0;   // wall clock (Date.now() of the old version)
    bool synthetic = false;       // built by MSP / .bin replay, never on the wire
    bool replay = false;          // comes from a log being replayed

    uint32_t id() const { return msg.msgid; }
    uint8_t sysid() const { return msg.sysid; }
    uint8_t compid() const { return msg.compid; }
};

} // namespace corv

Q_DECLARE_METATYPE(corv::MavMessage)
