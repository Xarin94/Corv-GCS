// LivoxProtocol.h - the Livox SDK2 UDP control frames (the protocol part of
// lidar-core.js): search and key/value configuration commands.
//
// A command frame is the 24-byte SdkPacket header - sof 0xAA, version, length,
// seq, cmd_id, cmd_type (0 request, 1 ack), sender (0 host, 1 lidar), CRC-16
// of the first 18 bytes, CRC-32 of the payload - followed by the payload.
// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) and CRC-32 (IEEE, reflected):
// the two FastCRC routines SDK2 uses.
#pragma once

#include <QByteArray>
#include <QString>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace corv::livox {

// Ports: LiDAR side and host side
constexpr quint16 kLidarSearch = 56000, kLidarCmd = 56100, kLidarPush = 56200, kLidarPoint = 56300, kLidarImu = 56400;
constexpr quint16 kHostSearch = 56000, kHostCmd = 56101, kHostPush = 56201, kHostPoint = 56301, kHostImu = 56401;

constexpr std::uint16_t kCmdSearch = 0x0000;
constexpr std::uint16_t kCmdWorkMode = 0x0100;   // "LidarWorkModeControl": key/value parameter set

constexpr std::uint16_t kKeyPclDataType = 0x0000;
constexpr std::uint16_t kKeyStateHostIp = 0x0005;
constexpr std::uint16_t kKeyPointHostIp = 0x0006;
constexpr std::uint16_t kKeyImuHostIp = 0x0007;
constexpr std::uint16_t kKeyWorkMode = 0x001A;
constexpr std::uint8_t kWorkModeNormal = 0x01;

constexpr int kPacketHeader = 36;   // LivoxLidarEthernetPacket header
constexpr int kSdkHeader = 24;      // SdkPacket header

std::uint16_t crc16(const std::uint8_t *buf, int len);
std::uint32_t crc32(const std::uint8_t *buf, int len);

/// A request frame (seq increments per call unless given).
QByteArray packCommand(std::uint16_t cmdId, const QByteArray &data = {}, std::optional<std::uint32_t> seq = std::nullopt);

struct Command {
    std::uint32_t seq = 0;
    std::uint16_t cmdId = 0;
    std::uint8_t cmdType = 0;     // 0 request, 1 ack
    std::uint8_t senderType = 0;  // 0 host, 1 lidar
    QByteArray data;
};
/// A frame whose header and payload CRCs check out, else nothing.
std::optional<Command> parseCommand(const QByteArray &buf);

/// Key/value parameter block: u16 key_num, u16 rsvd, then {u16 key, u16 len, bytes}*.
QByteArray buildKvRequest(const std::vector<std::pair<std::uint16_t, QByteArray>> &entries);
/// The 8-byte value of a *_HOST_IP key: IPv4, host port, LiDAR port.
QByteArray hostIpValue(const QString &ip, quint16 hostPort, quint16 lidarPort);

/// Model name of a device type (Mid-360, Mid-360S, HAP, Avia2), empty if unknown.
QString deviceName(int devType);

} // namespace corv::livox
