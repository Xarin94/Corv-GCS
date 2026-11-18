// MavFtp.h - MAVLink FTP client and the ArduPilot packed parameter file
// (port of mavlink-ftp.js).
//
// ArduPilot serves its parameters as one file, "@PARAM/param.pck", over the
// MAVLink file transfer protocol (FILE_TRANSFER_PROTOCOL, msg 110): a few
// dozen 239-byte packets instead of one PARAM_VALUE per parameter (1000-1500),
// so a full download takes seconds on a telemetry radio instead of minutes.
// Protocol and file format follow the firmware: GCS_MAVLink/GCS_FTP.cpp and
// AP_Filesystem/AP_Filesystem_Param.cpp.
//
// The client is deliberately small: open, burst read, fill the gaps with
// single reads, terminate. One request outstanding at a time, repeated after
// `timeoutMs` of silence, `retries` times; a burst that went quiet is asked
// again from the first byte still missing.
//
// FTP payload (251 bytes inside the MAVLink message):
//   0  u16 seq_number        3  u8 opcode     5  u8 req_opcode   8  u32 offset
//   2  u8  session           4  u8 size       6  u8 burst_complete 12 data[239]
//
// GUI thread (QTimer). The owner sends what `send` is given and feeds every
// FILE_TRANSFER_PROTOCOL payload received to onPacket().
#pragma once

#include <QByteArray>
#include <QList>
#include <QObject>
#include <QString>
#include <QTimer>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <vector>

namespace corv {

namespace ftp {

constexpr int kPayloadLen = 251;
constexpr int kHeaderLen = 12;
constexpr int kDataLen = kPayloadLen - kHeaderLen;   // 239

enum Op : uint8_t {
    None = 0, TerminateSession = 1, ResetSessions = 2, ListDirectory = 3, OpenFileRO = 4, ReadFile = 5,
    CreateFile = 6, WriteFile = 7, RemoveFile = 8, CreateDirectory = 9, RemoveDirectory = 10, OpenFileWO = 11,
    TruncateFile = 12, Rename = 13, CalcFileCRC32 = 14, BurstReadFile = 15, Ack = 128, Nak = 129
};

enum Error : uint8_t {
    ErrNone = 0, Fail = 1, FailErrno = 2, InvalidDataSize = 3, InvalidSession = 4, NoSessionsAvailable = 5,
    EndOfFile = 6, UnknownCommand = 7, FileExists = 8, FileProtected = 9, FileNotFound = 10
};

struct Packet {
    uint16_t seq = 0;
    uint8_t session = 0, opcode = 0, size = 0, reqOpcode = 0;
    bool burstComplete = false;
    uint32_t offset = 0;
    QByteArray data;
};

struct Request {
    uint16_t seq = 0;
    uint8_t session = 0, opcode = 0;
    int size = -1;              // -1: the data length
    uint8_t reqOpcode = 0;
    bool burstComplete = false;
    uint32_t offset = 0;
    QByteArray data;
};

/// One 251-byte payload.
QByteArray buildPayload(const Request &r);
std::optional<Packet> parsePayload(const QByteArray &raw);

// ── @PARAM/param.pck ─────────────────────────────────────────────────────────
//   header: u16 magic (0x671b, 0x671c with defaults), u16 num_params, u16 total_params
//   entry:  u8 type:4 (1 INT8, 2 INT16, 3 INT32, 4 FLOAT) | flags:4 (bit 0: a default follows)
//           u8 common_len:4 (name bytes shared with the previous entry) | name_len-1:4
//           name[name_len], value[type size], [default[type size]]
//   Zero bytes between entries are padding (an entry never straddles a 239-byte packet).

struct PackedParam {
    QString name;
    double value = 0;
    int type = 9;                    // MAV_PARAM_TYPE as PARAM_VALUE reports it
    std::optional<double> defaultValue;
};

struct ParamPack {
    bool ok = false;
    QString error;
    int total = 0, count = 0;
    bool complete = false;
    QList<PackedParam> params;
};

/// Decode a packed parameter file, whole or the part received so far.
ParamPack decodeParamPack(const QByteArray &raw);

struct ParamToPack {
    QString name;
    double value = 0;
    int ptype = 4;                   // 1 INT8, 2 INT16, 3 INT32, 4 FLOAT
};
/// Encode as the firmware does (tests, fake vehicles): padding so no entry straddles `packetLen`.
QByteArray encodeParamPack(const QList<ParamToPack> &params, int packetLen = kDataLen);

} // namespace ftp

class MavFtpClient : public QObject
{
    Q_OBJECT
public:
    using Send = std::function<void(const QByteArray &payload, int targetSystem, int targetComponent)>;
    struct Options {
        int targetSystem = 1, targetComponent = 1;
        int timeoutMs = 3000;   // silence before a request is repeated
        int retries = 4;        // repeats of one request before giving up
    };
    struct Progress {
        qint64 bytes = 0;
        quint32 sizeHint = 0;
        int packets = 0;
        qint64 eof = -1;
    };
    /// ok, the file, and on failure a code (NOREPLY, NAK, TIMEOUT, CANCELLED, BUSY) and a text.
    using Done = std::function<void(bool ok, const QByteArray &data, const QString &code, const QString &text)>;

    explicit MavFtpClient(Send send, QObject *parent = nullptr);
    ~MavFtpClient() override;

    bool busy() const { return bool(m_t); }
    void onPacket(const QByteArray &payload);
    /// Abort the transfer in progress (code CANCELLED).
    void cancel(const QString &reason = QStringLiteral("cancelled"));
    void readFile(const QString &path, const Options &opts, std::function<void(const Progress &)> progress, Done done);
    /// The bytes received so far, contiguous from the start (progress decoding).
    QByteArray partial() const;

private:
    struct Transfer;
    void sendRaw(ftp::Request r);
    void request(ftp::Request r, int tries = 0);
    void onTimeout();
    void handle(const ftp::Packet &op);
    void sendOpen();
    void sendBurst(uint32_t offset);
    void sendNextGap();
    void storeChunk(const ftp::Packet &op);
    void abort(const QString &code, const QString &text);
    void finish();

    Send m_send;
    uint16_t m_seq;
    uint8_t m_session = 0;
    std::unique_ptr<Transfer> m_t;
    QTimer m_timer;
};

} // namespace corv
