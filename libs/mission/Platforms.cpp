#include "mission/Platforms.h"

#include "core/Settings.h"

namespace corv::mission {

namespace {

const char *kSettingsKey = "corv.platform";

QHash<QString, QString> mspSetupLimits(const QString &configurator)
{
    return {
        {QStringLiteral("tel-forward"), QStringLiteral("The telemetry mirror forwards MAVLink packets — an MSP link carries none")},
        {QStringLiteral("lte-link"), QStringLiteral("The cellular module relays the MAVLink stream of the flight controller — an MSP link carries none")},
        {QStringLiteral("rtk-gps"), QStringLiteral("RTCM corrections are injected as MAVLink GPS_RTCM_DATA — feed the base corrections to the receiver directly")},
        {QStringLiteral("accel-cal"), QStringLiteral("Sensor calibration lives in the %1").arg(configurator)},
        {QStringLiteral("compass-cal"), QStringLiteral("Sensor calibration lives in the %1").arg(configurator)},
        {QStringLiteral("sensor-cal"), QStringLiteral("Sensor calibration lives in the %1").arg(configurator)},
        {QStringLiteral("radio-cal"), QStringLiteral("RC calibration lives in the %1").arg(configurator)},
        {QStringLiteral("joystick"), QStringLiteral("The gamepad override sends MAVLink RC_CHANNELS_OVERRIDE")},
        {QStringLiteral("flight-modes"), QStringLiteral("Mode switches are set in the %1").arg(configurator)},
        {QStringLiteral("failsafe"), QStringLiteral("Failsafe is configured in the %1").arg(configurator)},
        {QStringLiteral("servo-relay"), QStringLiteral("Outputs are configured in the %1").arg(configurator)},
        {QStringLiteral("pid-tuning"), QStringLiteral("PID tuning lives in the %1").arg(configurator)},
        {QStringLiteral("ext-tuning"), QStringLiteral("Tuning lives in the %1").arg(configurator)},
        {QStringLiteral("vibration"), QStringLiteral("The vibration panel reads the MAVLink VIBRATION message, which MSP does not provide")},
        {QStringLiteral("parameters"), QStringLiteral("The parameter editor speaks the MAVLink parameter protocol — settings live in the %1 and the CLI").arg(configurator)},
        {QStringLiteral("simulation"), QStringLiteral("The SITL launcher runs ArduPilot binaries")},
    };
}

QHash<QString, Platform> buildPlatforms()
{
    QHash<QString, Platform> out;

    Platform ap;
    ap.id = QStringLiteral("ardupilot");
    ap.label = QStringLiteral("ArduPilot");
    ap.shortName = QStringLiteral("AP");
    ap.blurb = QStringLiteral("Copter · Plane · Rover — MAVLink");
    ap.transport = QStringLiteral("mavlink");
    ap.itemLimit = 700;
    ap.connType = QStringLiteral("mavlink-serial");
    ap.connBaud = 57600;
    ap.hint = QStringLiteral("Full MAVLink mission: every segment type, every action, terrain-following waypoints.");
    out.insert(ap.id, ap);

    Platform inav;
    inav.id = QStringLiteral("inav");
    inav.label = QStringLiteral("INAV");
    inav.shortName = QStringLiteral("INAV");
    inav.blurb = QStringLiteral("Waypoint missions over MSP");
    inav.transport = QStringLiteral("msp");
    inav.configurator = QStringLiteral("INAV Configurator");
    inav.itemLimit = 60;
    inav.connType = QStringLiteral("msp-serial");
    inav.connBaud = 115200;
    inav.hint = QStringLiteral("INAV waypoint mission over MSP: navigation, hold, POI, heading, speed, land and RTH. "
                               "Upload with the aircraft disarmed, then fly it in WP mode.");
    const QString noCam = QStringLiteral("INAV missions carry no camera commands — set the trigger up on the camera itself");
    const QString noTakeoff = QStringLiteral("INAV has no take-off waypoint — arm, take off in a flight mode, then switch to WP");
    const QString noSpline = QStringLiteral("INAV flies straight legs — there is no spline waypoint");
    inav.unsupported = {
        {QStringLiteral("segment"), {{QStringLiteral("circle"), QStringLiteral("An INAV waypoint has no loiter radius, direction or turn count — a circle cannot be expressed")}}},
        {QStringLiteral("action"), {{QStringLiteral("trigger_dist"), noCam},
                                    {QStringLiteral("shot"), noCam},
                                    {QStringLiteral("gimbal"), QStringLiteral("INAV missions carry no gimbal commands")},
                                    {QStringLiteral("servo"), QStringLiteral("INAV missions carry no servo commands (programming framework only)")},
                                    {QStringLiteral("relay"), QStringLiteral("INAV missions carry no relay commands (programming framework only)")},
                                    {QStringLiteral("raw"), QStringLiteral("A raw MAVLink command has no INAV waypoint equivalent")}}},
        {QStringLiteral("routeParam"), {{QStringLiteral("takeoff"), noTakeoff}, {QStringLiteral("takeoffAlt"), noTakeoff}, {QStringLiteral("turnType"), noSpline}}},
        {QStringLiteral("segField"), {{QStringLiteral("waypoint.turn"), noSpline},
                                      {QStringLiteral("waypoint.acceptRadius"), QStringLiteral("INAV uses the nav_wp_radius parameter for every waypoint")},
                                      {QStringLiteral("landing.vtol"), QStringLiteral("INAV has no VTOL landing waypoint")},
                                      {QStringLiteral("landing.abortAlt"), QStringLiteral("INAV has no landing abort altitude in the mission")},
                                      {QStringLiteral("area.trigger"), QStringLiteral("INAV missions carry no camera trigger — the lanes upload as plain waypoints")},
                                      {QStringLiteral("corridor.trigger"), QStringLiteral("INAV missions carry no camera trigger — the passes upload as plain waypoints")},
                                      {QStringLiteral("poi.mode.clear"), QStringLiteral("INAV cannot cancel a POI from the mission — it holds until the mission ends")}}},
        {QStringLiteral("setup"), mspSetupLimits(inav.configurator)},
    };
    out.insert(inav.id, inav);

    Platform bf;
    bf.id = QStringLiteral("betaflight");
    bf.label = QStringLiteral("Betaflight");
    bf.shortName = QStringLiteral("BF");
    bf.blurb = QStringLiteral("Acro / racing — no navigation");
    bf.transport = QStringLiteral("msp");
    bf.missions = false;
    bf.configurator = QStringLiteral("Betaflight Configurator");
    bf.itemLimit = 0;
    bf.connType = QStringLiteral("msp-serial");
    bf.connBaud = 115200;
    bf.hint = QStringLiteral("Betaflight has no navigation stack: telemetry and configuration only, no waypoint missions.");
    bf.noMissionReason = QStringLiteral("Betaflight has no navigation stack — it cannot fly a waypoint mission");
    bf.unsupported = {{QStringLiteral("setup"), mspSetupLimits(bf.configurator)}};
    out.insert(bf.id, bf);
    return out;
}

const QHash<QString, Platform> &platforms()
{
    static const QHash<QString, Platform> p = buildPlatforms();
    return p;
}

} // namespace

const QStringList &platformOrder()
{
    static const QStringList o{QStringLiteral("ardupilot"), QStringLiteral("inav"), QStringLiteral("betaflight")};
    return o;
}

const Platform &platform(const QString &id)
{
    const auto &p = platforms();
    auto it = p.constFind(id);
    return it != p.constEnd() ? *it : *p.constFind(QStringLiteral("ardupilot"));
}

QString activePlatformId()
{
    const QString saved = Settings::instance().value(QLatin1String(kSettingsKey)).toString();
    return platforms().contains(saved) ? saved : QStringLiteral("ardupilot");
}

bool setActivePlatform(const QString &id)
{
    if (!platforms().contains(id) || id == activePlatformId()) return false;
    Settings::instance().setValue(QLatin1String(kSettingsKey), id);
    return true;
}

bool missionsSupported(const QString &id) { return platform(id).missions; }

bool usesMspMissions(const QString &id)
{
    return platform(id).transport == QLatin1String("msp") && missionsSupported(id);
}

QString unsupportedReason(const QString &kind, const QString &key, const QString &id)
{
    const Platform &p = platform(id);
    // A stack with no navigation cannot fly any part of a mission; SETUP keeps its own table
    if (!p.missions && kind != QLatin1String("setup"))
        return p.noMissionReason.isEmpty() ? QStringLiteral("%1 cannot fly missions").arg(p.label) : p.noMissionReason;
    return p.unsupported.value(kind).value(key);
}

} // namespace corv::mission
