// Mavlink.h - the one include of the generated MAVLink headers.
//
// Dialect: tools/mavlink/corv.xml = ardupilotmega + development + CORV_CELL_INFO.
// The CRC_EXTRA table covers every message of those dialects; a frame of an
// unknown msgid fails its CRC and is dropped, as node-mavlink did.
#pragma once

#if defined(_MSC_VER)
#pragma warning(push, 0)
#pragma warning(disable : 4200 4244 4267 4996)
#elif defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#pragma GCC diagnostic ignored "-Wcast-align"
#endif

#include <corv/mavlink.h>

#if defined(_MSC_VER)
#pragma warning(pop)
#elif defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

#include <QString>

namespace corv::mav {

/// The GCS identity on the link: Mission Planner compatible.
constexpr uint8_t kGcsSystemId = 255;
constexpr uint8_t kGcsComponentId = 190;

/// CRC_EXTRA of a message id, -1 when the dialect does not know it.
int crcExtra(uint32_t msgid);
/// "ATTITUDE", or "MSG_123" for an unknown id.
QString messageName(uint32_t msgid);
/// "MAV_CMD_NAV_TAKEOFF(22)"-style name for the debug log.
QString commandName(uint32_t cmd);
QString resultName(uint8_t result);

/// Re-stamp a message made by a _pack with the GCS identity, sequence `seq` and
/// its CRC, MAVLink 2. False when the dialect does not know the msgid.
/// The _pack already trimmed the payload and wrote the CRC right after it: that
/// tail goes back to the zeros it stood for before trimming again, or the two
/// CRC bytes become payload (a FTP file name two bytes too long, a COMMAND_LONG
/// confirmation that is not 0).
bool finalizeForSend(mavlink_message_t &msg, uint8_t seq);

/// CRC-16/MCRF4XX (X.25) as MAVLink uses it.
uint16_t x25(const uint8_t *data, std::size_t len, uint16_t crc = 0xFFFF);

/// A MAVLink string field (char[N], not always NUL-terminated).
QString fieldString(const char *chars, int maxLen);

} // namespace corv::mav
