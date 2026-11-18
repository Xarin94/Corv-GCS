// What LinkManager puts on the wire: a message made by a _pack, re-stamped by
// mav::finalizeForSend with the GCS identity and the link sequence, must parse
// back to the same fields. Before the fix the CRC the _pack had written after
// the trimmed payload was taken for payload: ArduPilot answered the FTP open of
// "@PARAM/param.pck" with InvalidDataSize, and COMMAND_LONG went out with a
// confirmation that was not 0.
#include "link/Mavlink.h"
#include "link/MavlinkParser.h"

#include <QTest>
#include <cstring>

using namespace corv;

class TstMavSend : public QObject
{
    Q_OBJECT

    /// Serialize, then decode the frame as a vehicle would (CRC checked).
    static mavlink_message_t roundTrip(mavlink_message_t msg, uint8_t seq)
    {
        if (!mav::finalizeForSend(msg, seq)) return {};
        const QByteArray wire = MavlinkParser::serialize(msg);
        mavlink_message_t out{};
        mavlink_status_t st{};
        for (char c : wire)
            if (mavlink_frame_char_buffer(&out, &st, uint8_t(c), &out, &st) == MAVLINK_FRAMING_OK) return out;
        return {};
    }

private slots:
    void ftpFileName()
    {
        // The FTP OpenFileRO of ParamStore: header, size 16, the name, zeros
        uint8_t payload[251] = {};
        payload[0] = 0x34; payload[1] = 0x12;   // seq
        payload[2] = 1;                         // session
        payload[3] = 4;                         // OpenFileRO
        payload[4] = 16;                        // size
        std::memcpy(payload + 12, "@PARAM/param.pck", 16);
        mavlink_message_t m;
        mavlink_msg_file_transfer_protocol_pack(mav::kGcsSystemId, mav::kGcsComponentId, &m, 0, 1, 1, payload);

        const mavlink_message_t rx = roundTrip(m, 7);
        QCOMPARE(int(rx.msgid), int(MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL));
        QCOMPARE(int(rx.seq), 7);
        QCOMPARE(int(rx.sysid), int(mav::kGcsSystemId));
        mavlink_file_transfer_protocol_t f;
        mavlink_msg_file_transfer_protocol_decode(&rx, &f);
        // What GCS_FTP::Session::check_name_len compares with the size field
        QCOMPARE(int(strnlen(reinterpret_cast<const char *>(f.payload + 12), 239)), 16);
        QCOMPARE(std::memcmp(f.payload, payload, sizeof payload), 0);
    }

    void commandLongConfirmation()
    {
        mavlink_message_t m;
        mavlink_msg_command_long_pack(mav::kGcsSystemId, mav::kGcsComponentId, &m, 1, 1, MAV_CMD_PREFLIGHT_CALIBRATION,
                                      0, 0, 0, 0, 0, 1, 0, 0);
        const mavlink_message_t rx = roundTrip(m, 200);
        mavlink_command_long_t c;
        mavlink_msg_command_long_decode(&rx, &c);
        QCOMPARE(int(c.command), int(MAV_CMD_PREFLIGHT_CALIBRATION));
        QCOMPARE(c.param5, 1.0f);
        QCOMPARE(int(c.target_system), 1);
        QCOMPARE(int(c.target_component), 1);
        QCOMPARE(int(c.confirmation), 0);
    }

    void sentTwice()
    {
        // A retry sends the same packed message again: same bytes but the sequence
        mavlink_message_t m;
        mavlink_msg_param_request_read_pack(mav::kGcsSystemId, mav::kGcsComponentId, &m, 1, 1, "COMPASS_USE", -1);
        mavlink_message_t a = m, b = m;
        QVERIFY(mav::finalizeForSend(a, 1));
        QVERIFY(mav::finalizeForSend(b, 1));
        QVERIFY(mav::finalizeForSend(b, 1));
        QCOMPARE(MavlinkParser::serialize(a), MavlinkParser::serialize(b));
        const mavlink_message_t rx = roundTrip(m, 2);
        mavlink_param_request_read_t r;
        mavlink_msg_param_request_read_decode(&rx, &r);
        QCOMPARE(mav::fieldString(r.param_id, 16), QStringLiteral("COMPASS_USE"));
        QCOMPARE(int(r.param_index), -1);
    }
};

QTEST_GUILESS_MAIN(TstMavSend)
#include "tst_mavsend.moc"
