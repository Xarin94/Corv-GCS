#include "mission/Platforms.h"

#include "core/Settings.h"

using namespace Qt::StringLiterals;

namespace corv::mission {

namespace {

const char *kSettingsKey = "corv.platform";

QHash<QString, QString> mspSetupLimits(const QString &configurator)
{
    return {
        {u"tel-forward"_s, u"The telemetry mirror forwards MAVLink packets — an MSP link carries none"_s},
        {u"lte-link"_s, u"The cellular module relays the MAVLink stream of the flight controller — an MSP link carries none"_s},
        {u"rtk-gps"_s, u"RTCM corrections are injected as MAVLink GPS_RTCM_DATA — feed the base corrections to the receiver directly"_s},
        {u"accel-cal"_s, u"Sensor calibration lives in the %1"_s.arg(configurator)},
        {u"compass-cal"_s, u"Sensor calibration lives in the %1"_s.arg(configurator)},
        {u"sensor-cal"_s, u"Sensor calibration lives in the %1"_s.arg(configurator)},
        {u"radio-cal"_s, u"RC calibration lives in the %1"_s.arg(configurator)},
        {u"joystick"_s, u"The gamepad override sends MAVLink RC_CHANNELS_OVERRIDE"_s},
        {u"flight-modes"_s, u"Mode switches are set in the %1"_s.arg(configurator)},
        {u"failsafe"_s, u"Failsafe is configured in the %1"_s.arg(configurator)},
        {u"servo-relay"_s, u"Outputs are configured in the %1"_s.arg(configurator)},
        {u"pid-tuning"_s, u"PID tuning lives in the %1"_s.arg(configurator)},
        {u"ext-tuning"_s, u"Tuning lives in the %1"_s.arg(configurator)},
        {u"vibration"_s, u"The vibration panel reads the MAVLink VIBRATION message, which MSP does not provide"_s},
        {u"parameters"_s, u"The parameter editor speaks the MAVLink parameter protocol — settings live in the %1 and the CLI"_s.arg(configurator)},
        {u"simulation"_s, u"The SITL launcher runs ArduPilot binaries"_s},
    };
}

QHash<QString, Platform> buildPlatforms()
{
    QHash<QString, Platform> out;

    Platform ap;
    ap.id = u"ardupilot"_s;
    ap.label = u"ArduPilot"_s;
    ap.shortName = u"AP"_s;
    ap.blurb = u"Copter · Plane · Rover — MAVLink"_s;
    ap.transport = u"mavlink"_s;
    ap.itemLimit = 700;
    ap.connType = u"mavlink-serial"_s;
    ap.connBaud = 57600;
    ap.hint = u"Full MAVLink mission: every segment type, every action, terrain-following waypoints."_s;
    out.insert(ap.id, ap);

    Platform inav;
    inav.id = u"inav"_s;
    inav.label = u"INAV"_s;
    inav.shortName = u"INAV"_s;
    inav.blurb = u"Waypoint missions over MSP"_s;
    inav.transport = u"msp"_s;
    inav.configurator = u"INAV Configurator"_s;
    inav.itemLimit = 60;
    inav.connType = u"msp-serial"_s;
    inav.connBaud = 115200;
    inav.hint = QStringLiteral("INAV waypoint mission over MSP: navigation, hold, POI, heading, speed, land and RTH. "
                               "Upload with the aircraft disarmed, then fly it in WP mode.");
    const QString noCam = u"INAV missions carry no camera commands — set the trigger up on the camera itself"_s;
    const QString noTakeoff = u"INAV has no take-off waypoint — arm, take off in a flight mode, then switch to WP"_s;
    const QString noSpline = u"INAV flies straight legs — there is no spline waypoint"_s;
    inav.unsupported = {
        {u"segment"_s, {{u"circle"_s, u"An INAV waypoint has no loiter radius, direction or turn count — a circle cannot be expressed"_s}}},
        {u"action"_s, {{u"trigger_dist"_s, noCam},
                                    {u"shot"_s, noCam},
                                    {u"gimbal"_s, u"INAV missions carry no gimbal commands"_s},
                                    {u"servo"_s, u"INAV missions carry no servo commands (programming framework only)"_s},
                                    {u"relay"_s, u"INAV missions carry no relay commands (programming framework only)"_s},
                                    {u"raw"_s, u"A raw MAVLink command has no INAV waypoint equivalent"_s}}},
        {u"routeParam"_s, {{u"takeoff"_s, noTakeoff}, {u"takeoffAlt"_s, noTakeoff}, {u"turnType"_s, noSpline}}},
        {u"segField"_s, {{u"waypoint.turn"_s, noSpline},
                                      {u"waypoint.acceptRadius"_s, u"INAV uses the nav_wp_radius parameter for every waypoint"_s},
                                      {u"landing.vtol"_s, u"INAV has no VTOL landing waypoint"_s},
                                      {u"landing.abortAlt"_s, u"INAV has no landing abort altitude in the mission"_s},
                                      {u"area.trigger"_s, u"INAV missions carry no camera trigger — the lanes upload as plain waypoints"_s},
                                      {u"corridor.trigger"_s, u"INAV missions carry no camera trigger — the passes upload as plain waypoints"_s},
                                      {u"poi.mode.clear"_s, u"INAV cannot cancel a POI from the mission — it holds until the mission ends"_s}}},
        {u"setup"_s, mspSetupLimits(inav.configurator)},
    };
    out.insert(inav.id, inav);

    Platform bf;
    bf.id = u"betaflight"_s;
    bf.label = u"Betaflight"_s;
    bf.shortName = u"BF"_s;
    bf.blurb = u"Acro / racing — no navigation"_s;
    bf.transport = u"msp"_s;
    bf.missions = false;
    bf.configurator = u"Betaflight Configurator"_s;
    bf.itemLimit = 0;
    bf.connType = u"msp-serial"_s;
    bf.connBaud = 115200;
    bf.hint = u"Betaflight has no navigation stack: telemetry and configuration only, no waypoint missions."_s;
    bf.noMissionReason = u"Betaflight has no navigation stack — it cannot fly a waypoint mission"_s;
    bf.unsupported = {{u"setup"_s, mspSetupLimits(bf.configurator)}};
    out.insert(bf.id, bf);
    return out;
}

const QHash<QString, Platform> &platforms()
{
    static const QHash<QString, Platform> p = buildPlatforms();
    return p;
}

} // namespace

const Platform &platform(const QString &id)
{
    const auto &p = platforms();
    auto it = p.constFind(id);
    return it != p.constEnd() ? *it : *p.constFind(u"ardupilot"_s);
}

QString activePlatformId()
{
    const QString saved = Settings::instance().value(QLatin1String(kSettingsKey)).toString();
    return platforms().contains(saved) ? saved : u"ardupilot"_s;
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
    return platform(id).transport == "msp"_L1 && missionsSupported(id);
}

QString unsupportedReason(const QString &kind, const QString &key, const QString &id)
{
    const Platform &p = platform(id);
    // A stack with no navigation cannot fly any part of a mission; SETUP keeps its own table
    if (!p.missions && kind != "setup"_L1)
        return p.noMissionReason.isEmpty() ? u"%1 cannot fly missions"_s.arg(p.label) : p.noMissionReason;
    return p.unsupported.value(kind).value(key);
}

} // namespace corv::mission
