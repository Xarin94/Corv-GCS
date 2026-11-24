// CommandSender.h - high-level autopilot commands (port of js/mavlink/CommandSender.js).
//
// Lives on the GUI thread. Every command is packed here and queued to the link
// thread (LinkManager::send); answers (COMMAND_ACK, PARAM_VALUE, MISSION_*)
// come back through the MessageBus. Asynchronous results go to a callback
// called on the GUI thread exactly once: ok, plus a short text for the HUD.
#pragma once

#include "link/MavMessage.h"

#include <QObject>
#include <functional>

namespace corv {

class LinkManager;
class MessageBus;
class VehicleHub;

struct MissionItem {
    int command = MAV_CMD_NAV_WAYPOINT;
    int frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
    float param1 = 0, param2 = 0, param3 = 0, param4 = 0;
    double lat = 0, lng = 0;
    float alt = 0;
};

class CommandSender : public QObject
{
    Q_OBJECT
public:
    using Done = std::function<void(bool ok, const QString &text)>;
    struct Ack {
        int result = -1;
        QString resultName;
    };
    using AckDone = std::function<void(bool gotAck, const Ack &ack)>;

    CommandSender(LinkManager *link, MessageBus *bus, VehicleHub *hub, QObject *parent = nullptr);

    // Fire and forget, as in the Electron version
    void commandLong(uint16_t command, float p1 = 0, float p2 = 0, float p3 = 0, float p4 = 0, float p5 = 0,
                     float p6 = 0, float p7 = 0);
    void commandInt(uint16_t command, uint8_t frame, float p1, float p2, float p3, float p4, int32_t x, int32_t y, float z);
    /// COMMAND_LONG and wait for its COMMAND_ACK (IN_PROGRESS is not final). The listener goes in first.
    void commandWithAck(uint16_t command, float p1, float p2, float p3, float p4, float p5, float p6, float p7,
                        int timeoutMs, AckDone done);

    Q_INVOKABLE void armVehicle(bool force = false);
    Q_INVOKABLE void disarmVehicle(bool force = false);
    Q_INVOKABLE bool setFlightMode(const QString &modeName);
    Q_INVOKABLE void setFlightModeNum(int modeNum);
    void takeoff(double altitude, Done done = {});
    Q_INVOKABLE void land();
    Q_INVOKABLE void returnToLaunch();
    Q_INVOKABLE void setMissionSpeed(double speed, int speedType = 1);
    void setHome(std::optional<double> lat, std::optional<double> lon, std::optional<double> alt, AckDone done = {});
    Q_INVOKABLE void requestHomePosition();
    /// AUTOPILOT_VERSION, for the firmware version.
    void requestAutopilotVersion();
    Q_INVOKABLE void rebootAutopilot();
    void sendRcChannelsOverride(const std::array<uint16_t, 18> &channels);
    Q_INVOKABLE void sendServoTest(int servo, int pwm);
    Q_INVOKABLE void sendRelayToggle(int relay, int state);

    // Calibration (Mission Planner procedures)
    void calibrateAccel(AckDone done);
    void calibrateLevel(AckDone done);
    void calibrateAccelSimple(AckDone done);
    void calibrateGyro(AckDone done);
    void calibrateBaro(AckDone done);
    void sendAccelCalPosition(int position, AckDone done);
    void startMagCal(int mask, bool retry, bool autosave, float delay, bool autoreboot, AckDone done);
    void acceptMagCal(int mask, AckDone done);
    void cancelMagCal(int mask, AckDone done);
    void magCalFixedYaw(float yawDeg, int mask, float lat, float lon, AckDone done);

    // Streams
    void requestDataStream(int streamId, int rate, int start = 1);
    void setMessageInterval(uint32_t msgId, double rateHz);
    void resetMessageInterval(uint32_t msgId);
    Q_INVOKABLE void requestAllDataStreams();

    // Parameters
    void setParameter(const QString &id, float value, int type, std::function<void(bool ok, float value)> done);
    void requestAllParameters();
    void requestParameter(const QString &id);
    void fetchParameter(const QString &id, int timeoutMs, int retries, std::function<void(bool ok, float value, int type)> done);

    // Mission upload (MISSION_COUNT -> REQUEST_INT/REQUEST -> ITEM_INT -> ACK)
    void uploadMission(const QList<MissionItem> &items, Done done);
    /// Mission download (REQUEST_LIST -> COUNT -> REQUEST_INT -> ITEM_INT / ITEM -> ACK), as MissionTransfer.js.
    /// `progress(done, total)` after every item; `finished` once (items in degrees).
    void downloadMission(std::function<void(int done, int total)> progress,
                         std::function<void(bool ok, const QString &error, const QList<MissionItem> &items)> finished);

    int targetSystem() const;
    int targetComponent() const;

private:
    void sendMsg(const mavlink_message_t &m);

    LinkManager *m_link;
    MessageBus *m_bus;
    VehicleHub *m_hub;
};

} // namespace corv
