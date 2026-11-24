#include "vehicle/StateMapper.h"

#include "core/DebugLog.h"
#include "vehicle/FlightModes.h"

#include <QRegularExpression>
#include <algorithm>
#include <cmath>

using namespace Qt::StringLiterals;

namespace corv {

namespace {
constexpr std::int64_t kGlobalPosStaleMs = 2000;
constexpr std::int64_t kAdsbMaxAgeMs = 60000;
constexpr std::size_t kVibHistory = 120;
} // namespace

StateMapper::StateMapper()
{
    m_origin.onMoved = nullptr;
}

void StateMapper::apply(const MavMessage &mm, VehicleState &s)
{
    const mavlink_message_t &m = mm.msg;
    const std::int64_t now = mm.arrivalMs;

    // Heartbeat bookkeeping (MAVLinkManager.handleMessage): only the autopilot (compid 1)
    if (m.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        if (m.compid != 1) return;
        ++s.heartbeatCount;
        s.lastHeartbeatTime = now;
        s.systemId = m.sysid;
        s.componentId = m.compid;
    }

    // Relative navigation sees every message first
    m_relNav.onMessage(mm, s.vehicleType, now);

    switch (m.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: heartbeat(m, s); break;
    case MAVLINK_MSG_ID_SYS_STATUS: sysStatus(m, s, now); break;
    case MAVLINK_MSG_ID_GPS_RAW_INT: gpsRawInt(mm, s, now); break;
    case MAVLINK_MSG_ID_SCALED_IMU:
    case MAVLINK_MSG_ID_RAW_IMU: scaledImu(m, s, true, now); break;
    case MAVLINK_MSG_ID_SCALED_IMU2:
    case MAVLINK_MSG_ID_SCALED_IMU3: scaledImu(m, s, false, now); break;
    case MAVLINK_MSG_ID_ATTITUDE: attitude(mm, s, now); break;
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: globalPositionInt(mm, s, now); break;
    case MAVLINK_MSG_ID_VFR_HUD: vfrHud(m, s, now); break;
    case MAVLINK_MSG_ID_CORV_CELL_INFO: cellInfo(m, s, now); break;
    case MAVLINK_MSG_ID_HOME_POSITION: homePosition(m, s); break;
    case MAVLINK_MSG_ID_ADSB_VEHICLE: adsbVehicle(m, s, now); break;
    default:
        guidance(m, s, now);
        sensors(m, s, now);
        reports(m, s, now);
    }
}

/// Mission progress, navigation controller output, RC inputs and servo outputs.
void StateMapper::guidance(const mavlink_message_t &m, VehicleState &s, std::int64_t now)
{
    switch (m.msgid) {
    case MAVLINK_MSG_ID_MISSION_CURRENT: {
        mavlink_mission_current_t c;
        mavlink_msg_mission_current_decode(&m, &c);
        s.missionCurrentSeq = c.seq;
        // total is an extension: 0 when the autopilot does not send it
        if (m.len > 2 && c.total != UINT16_MAX && c.total) s.missionCount = c.total;
        break;
    }
    case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT: {
        mavlink_nav_controller_output_t n;
        mavlink_msg_nav_controller_output_decode(&m, &n);
        s.navRoll = n.nav_roll;
        s.navPitch = n.nav_pitch;
        s.navBearing = n.nav_bearing;
        s.targetBearing = n.target_bearing;
        s.wpDist = n.wp_dist;
        s.altError = n.alt_error;
        // ArduPilot sends airspeed_error * 100 here (historical quirk)
        s.aspdError = n.aspd_error / 100.0;
        s.xtrackError = n.xtrack_error;
        s.navDataTime = now;
        break;
    }
    case MAVLINK_MSG_ID_RC_CHANNELS_RAW: {
        mavlink_rc_channels_raw_t r;
        mavlink_msg_rc_channels_raw_decode(&m, &r);
        const uint16_t v[8] = {r.chan1_raw, r.chan2_raw, r.chan3_raw, r.chan4_raw, r.chan5_raw, r.chan6_raw, r.chan7_raw, r.chan8_raw};
        for (int i = 0; i < 8; ++i) s.rcChannels[i] = v[i];
        break;
    }
    case MAVLINK_MSG_ID_RC_CHANNELS: {
        mavlink_rc_channels_t r;
        mavlink_msg_rc_channels_decode(&m, &r);
        const uint16_t v[18] = {r.chan1_raw, r.chan2_raw, r.chan3_raw, r.chan4_raw, r.chan5_raw, r.chan6_raw,
                                r.chan7_raw, r.chan8_raw, r.chan9_raw, r.chan10_raw, r.chan11_raw, r.chan12_raw,
                                r.chan13_raw, r.chan14_raw, r.chan15_raw, r.chan16_raw, r.chan17_raw, r.chan18_raw};
        for (int i = 0; i < 18; ++i) s.rcChannels[i] = v[i];
        break;
    }
    case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: {
        mavlink_servo_output_raw_t r;
        mavlink_msg_servo_output_raw_decode(&m, &r);
        const uint16_t v[16] = {r.servo1_raw, r.servo2_raw, r.servo3_raw, r.servo4_raw, r.servo5_raw, r.servo6_raw,
                                r.servo7_raw, r.servo8_raw, r.servo9_raw, r.servo10_raw, r.servo11_raw, r.servo12_raw,
                                r.servo13_raw, r.servo14_raw, r.servo15_raw, r.servo16_raw};
        for (int i = 0; i < 16; ++i) s.servoOutputs[i] = v[i];
        break;
    }
    default: break;
    }
}

/// RTK baseline, rangefinder, terrain, wind, EKF, vibration, GNSS integrity.
void StateMapper::sensors(const mavlink_message_t &m, VehicleState &s, std::int64_t now)
{
    switch (m.msgid) {
    case MAVLINK_MSG_ID_GPS_RTK: {
        mavlink_gps_rtk_t r;
        mavlink_msg_gps_rtk_decode(&m, &r);
        s.rtkIar = r.iar_num_hypotheses;
        s.rtkBaseline = std::sqrt(double(r.baseline_a_mm) * r.baseline_a_mm + double(r.baseline_b_mm) * r.baseline_b_mm
                                  + double(r.baseline_c_mm) * r.baseline_c_mm);
        s.rtkAccuracy = r.accuracy;
        break;
    }
    case MAVLINK_MSG_ID_DISTANCE_SENSOR: {
        mavlink_distance_sensor_t d;
        mavlink_msg_distance_sensor_decode(&m, &d);
        if (d.orientation == MAV_SENSOR_ROTATION_PITCH_270) s.rangefinderDist = d.current_distance / 100.0;
        break;
    }
    case MAVLINK_MSG_ID_TERRAIN_REPORT: {
        mavlink_terrain_report_t t;
        mavlink_msg_terrain_report_decode(&m, &t);
        s.terrainPending = t.pending;
        s.terrainLoaded = t.loaded;
        break;
    }
    case MAVLINK_MSG_ID_WIND: {
        mavlink_wind_t w;
        mavlink_msg_wind_decode(&m, &w);
        s.windDir = w.direction;
        s.windSpeed = w.speed;
        s.windSpeedZ = w.speed_z;
        s.windDataTime = now;
        break;
    }
    case MAVLINK_MSG_ID_EKF_STATUS_REPORT: {
        mavlink_ekf_status_report_t e;
        mavlink_msg_ekf_status_report_decode(&m, &e);
        s.ekfFlags = e.flags;
        // Largest of the variances Mission Planner grades (airspeed left out)
        s.ekfVariance = std::max({0.0f, e.velocity_variance, e.pos_horiz_variance, e.pos_vert_variance,
                                  e.compass_variance, e.terrain_alt_variance});
        s.ekfCompassVariance = e.compass_variance;
        s.ekfDataTime = now;
        break;
    }
    case MAVLINK_MSG_ID_VIBRATION: {
        mavlink_vibration_t v;
        mavlink_msg_vibration_decode(&m, &v);
        s.vibX = v.vibration_x;
        s.vibY = v.vibration_y;
        s.vibZ = v.vibration_z;
        s.vibClip0 = v.clipping_0;
        s.vibClip1 = v.clipping_1;
        s.vibClip2 = v.clipping_2;
        s.vibHistory.push_back({s.vibX, s.vibY, s.vibZ, now});
        if (s.vibHistory.size() > kVibHistory) s.vibHistory.erase(s.vibHistory.begin());
        break;
    }
    case MAVLINK_MSG_ID_GNSS_INTEGRITY: {
        mavlink_gnss_integrity_t g;
        mavlink_msg_gnss_integrity_decode(&m, &g);
        s.gnssJamming = g.jamming_state;
        s.gnssSpoofing = g.spoofing_state;
        s.gnssIntegrityTime = now;
        break;
    }
    default: break;
    }
}

/// Command ACKs, named values, status texts, radio RSSI, compass calibration.
void StateMapper::reports(const mavlink_message_t &m, VehicleState &s, std::int64_t now)
{
    switch (m.msgid) {
    case MAVLINK_MSG_ID_COMMAND_ACK: {
        mavlink_command_ack_t a;
        mavlink_msg_command_ack_decode(&m, &a);
        s.lastAckCommand = a.command;
        s.lastAckResult = a.result;
        // No toast for the internal polling commands (REQUEST_MESSAGE, SET_MESSAGE_INTERVAL)
        if (a.command != MAV_CMD_REQUEST_MESSAGE && a.command != MAV_CMD_SET_MESSAGE_INTERVAL && m_hooks.commandAck)
            m_hooks.commandAck(a.command, a.result);
        break;
    }
    case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT: {
        mavlink_named_value_float_t n;
        mavlink_msg_named_value_float_decode(&m, &n);
        const QString name = mav::fieldString(n.name, 10).trimmed();
        if (name == "AppWndDir"_L1) {
            s.appWindDir = n.value;
            s.appWindTime = now;
        } else if (name == "AppWndSpd"_L1) {
            s.appWindSpeed = n.value;
        }
        break;
    }
    case MAVLINK_MSG_ID_STATUSTEXT: {
        mavlink_statustext_t t;
        mavlink_msg_statustext_decode(&m, &t);
        s.statusText = mav::fieldString(t.text, 50);
        s.statusSeverity = t.severity;
        s.statusTextTime = now;
        // ArduPilot says "PreArm: <check>" for what fails; some firmwares say when all is well
        if (s.statusText.startsWith("PreArm"_L1, Qt::CaseInsensitive)) {
            // (whole words: "PreArm: Compass not calibrated" has a "pass" in it)
            static const QRegularExpression passed(u"\\b(good|ok|pass|passed)\\b"_s, QRegularExpression::CaseInsensitiveOption);
            const bool good = passed.match(s.statusText).hasMatch();
            s.preArmReport = good ? VehicleState::PreArm::Passed : VehicleState::PreArm::Failed;
            s.preArmText = s.statusText;
        }
        if (m_hooks.statusText) m_hooks.statusText(t.severity, s.statusText);
        break;
    }
    case MAVLINK_MSG_ID_RADIO_STATUS: {
        mavlink_radio_status_t r;
        mavlink_msg_radio_status_decode(&m, &r);
        // 0-254 are valid RSSI counts; 255 = invalid / unknown
        s.rssi = r.rssi < 255 ? std::optional<int>(r.rssi) : std::nullopt;
        s.remRssi = r.remrssi < 255 ? std::optional<int>(r.remrssi) : std::nullopt;
        break;
    }
    case MAVLINK_MSG_ID_AUTOPILOT_VERSION: {
        mavlink_autopilot_version_t v;
        mavlink_msg_autopilot_version_decode(&m, &v);
        if (m.compid == 1 && v.flight_sw_version) s.firmwareVersion = modes::firmwareVersion(v.flight_sw_version);
        break;
    }
    case MAVLINK_MSG_ID_MAG_CAL_PROGRESS: s.magCalTime = now; break;
    case MAVLINK_MSG_ID_MAG_CAL_REPORT: s.magCalTime = 0; break;
    default: break;
    }
}

void StateMapper::heartbeat(const mavlink_message_t &m, VehicleState &s)
{
    mavlink_heartbeat_t hb;
    mavlink_msg_heartbeat_decode(&m, &hb);
    s.baseMode = hb.base_mode;
    s.customMode = hb.custom_mode;
    s.autopilotType = hb.autopilot;
    s.vehicleType = hb.type;
    s.armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
    s.systemStatus = hb.system_status;
    s.flightModeNum = hb.custom_mode;
    s.flightMode = modes::name(hb.custom_mode, hb.type, s.connectionType);
}

void StateMapper::sysStatus(const mavlink_message_t &m, VehicleState &s, std::int64_t now)
{
    mavlink_sys_status_t st;
    mavlink_msg_sys_status_decode(&m, &st);
    if (st.voltage_battery != UINT16_MAX) s.batteryVoltage = st.voltage_battery / 1000.0;
    if (st.current_battery >= 0) s.batteryCurrent = st.current_battery / 100.0;
    const int rawPct = st.battery_remaining;
    if (rawPct < 0 || rawPct > 100) {
        // No percentage from the vehicle: from the voltage in the configured range
        double vmin = m_batVMin, vmax = m_batVMax;
        if (!(vmax > vmin)) { vmin = 9.6; vmax = 12.6; }
        const double v = s.batteryVoltage;
        s.batteryRemaining = v > 0 ? std::clamp(int(std::lround((v - vmin) / (vmax - vmin) * 100)), 0, 100) : -1;
    } else {
        s.batteryRemaining = rawPct;
    }
    s.linkQuality = std::clamp(100.0 - st.drop_rate_comm / 100.0, 0.0, 100.0);
    s.sensorsPresent = st.onboard_control_sensors_present;
    s.sensorsEnabled = st.onboard_control_sensors_enabled;
    s.sensorsHealth = st.onboard_control_sensors_health;
    s.sysStatusTime = now;
}

void StateMapper::firstFix(VehicleState &s)
{
    if (m_origin.fix(s.lat, s.lon)) {
        s.sceneOrigin = m_origin.origin();
        s.sceneOriginEpoch = m_origin.epoch();
        if (!s.homeLat && s.mode != "REPLAY"_L1 && m_hooks.rememberLastHome) m_hooks.rememberLastHome(s.lat, s.lon);
    }
}

void StateMapper::gpsRawInt(const MavMessage &mm, VehicleState &s, std::int64_t now)
{
    mavlink_gps_raw_int_t g;
    mavlink_msg_gps_raw_int_decode(&mm.msg, &g);
    s.gpsFix = g.fix_type;
    s.gpsDataTime = now;
    s.gpsNumSat = g.satellites_visible;
    if (g.eph != UINT16_MAX) s.gpsHdop = g.eph / 100.0;

    // Raw GPS fills position / velocity only while no GLOBAL_POSITION_INT arrived recently
    if (now - m_lastGlobalPosTs <= kGlobalPosStaleMs) return;
    if (m_relNav.isRelative()) return;
    // Reject boot-time nulls on the exact raw integers
    if (g.lat != 0 && g.lon != 0) {
        s.lat = g.lat / 1e7;
        s.lon = g.lon / 1e7;
        if (g.fix_type >= 2) firstFix(s);
    }
    if (!mm.synthetic) s.rawAlt = g.alt / 1000.0;
    if (g.vel != UINT16_MAX) s.gs = g.vel / 100.0;
    if (g.cog != UINT16_MAX) s.track = (g.cog / 100.0) * kDeg;
}

void StateMapper::scaledImu(const mavlink_message_t &m, VehicleState &s, bool primary, std::int64_t now)
{
    int16_t xacc, yacc, zacc;
    switch (m.msgid) {
    case MAVLINK_MSG_ID_RAW_IMU: {
        mavlink_raw_imu_t r;
        mavlink_msg_raw_imu_decode(&m, &r);
        xacc = r.xacc; yacc = r.yacc; zacc = r.zacc;
        break;
    }
    case MAVLINK_MSG_ID_SCALED_IMU2: {
        mavlink_scaled_imu2_t r;
        mavlink_msg_scaled_imu2_decode(&m, &r);
        xacc = r.xacc; yacc = r.yacc; zacc = r.zacc;
        break;
    }
    case MAVLINK_MSG_ID_SCALED_IMU3: {
        mavlink_scaled_imu3_t r;
        mavlink_msg_scaled_imu3_decode(&m, &r);
        xacc = r.xacc; yacc = r.yacc; zacc = r.zacc;
        break;
    }
    default: {
        mavlink_scaled_imu_t r;
        mavlink_msg_scaled_imu_decode(&m, &r);
        xacc = r.xacc; yacc = r.yacc; zacc = r.zacc;
        break;
    }
    }
    if (primary) {
        m_imuPrimaryActive = true;
        m_imuPrimaryLastTime = now;
    } else if (m_imuPrimaryActive && now - m_imuPrimaryLastTime < 3000) {
        return;   // primary is active, skip IMU2/3
    }
    // mG -> m/s^2
    s.ax = xacc / 1000.0 * 9.81;
    s.ay = yacc / 1000.0 * 9.81;
    s.az = zacc / 1000.0 * 9.81;
}

void StateMapper::attitude(const MavMessage &mm, VehicleState &s, std::int64_t now)
{
    mavlink_attitude_t a;
    mavlink_msg_attitude_decode(&mm.msg, &a);
    if (!std::isfinite(a.roll) || !std::isfinite(a.pitch) || !std::isfinite(a.yaw)) return;
    s.roll = a.roll;
    s.pitch = a.pitch;
    s.yaw = a.yaw;
    s.attTime = now;
    // time_boot_ms 0: the source does not report the vehicle clock
    s.attBootMs = a.time_boot_ms ? std::optional<std::uint32_t>(a.time_boot_ms) : std::nullopt;
    if (!mm.synthetic) {
        s.rollRate = a.rollspeed;
        s.pitchRate = a.pitchspeed;
        s.yawRate = a.yawspeed;
    }
    computeAeroAngles(s, now);
}

void StateMapper::globalPositionInt(const MavMessage &mm, VehicleState &s, std::int64_t now)
{
    mavlink_global_position_int_t g;
    mavlink_msg_global_position_int_decode(&mm.msg, &g);
    m_lastGlobalPosTs = now;
    s.posTime = now;
    s.posBootMs = g.time_boot_ms ? std::optional<std::uint32_t>(g.time_boot_ms) : std::nullopt;

    if (!m_relNav.isRelative()) {
        if (g.lat != 0 && g.lon != 0) {
            s.lat = g.lat / 1e7;
            s.lon = g.lon / 1e7;
            firstFix(s);
        }
        // A sub reports its depth below the surface it started on
        if (modes::isSubType(s.vehicleType) && s.subSurfaceAlt && !mm.synthetic)
            s.rawAlt = *s.subSurfaceAlt + g.relative_alt / 1000.0;
        else
            s.rawAlt = g.alt / 1000.0;
    }
    s.vs = -g.vz / 100.0;
    s.vn = g.vx / 100.0;
    s.ve = g.vy / 100.0;
    s.vd = g.vz / 100.0;
    // Ground speed from the EKF horizontal velocity: the authoritative source
    const double vh = std::hypot(s.vn, s.ve);
    s.gs = vh;
    s.gamma = std::atan2(-s.vd, vh);
    s.track = std::atan2(s.ve, s.vn);
    computeAeroAngles(s, now);
}

void StateMapper::computeAeroAngles(VehicleState &s, std::int64_t now)
{
    double vn = s.vn, ve = s.ve, vd = s.vd;
    if (s.windDataTime > 0 && now - s.windDataTime < 5000) {
        // windDir is where the wind comes FROM; the air mass moves the other way
        const double from = s.windDir * kDeg;
        vn -= -std::cos(from) * s.windSpeed;
        ve -= -std::sin(from) * s.windSpeed;
        vd -= s.windSpeedZ;
    }
    const double cr = std::cos(s.roll), sr = std::sin(s.roll);
    const double cp = std::cos(s.pitch), sp = std::sin(s.pitch);
    const double cy = std::cos(s.yaw), sy = std::sin(s.yaw);
    const double vbx = cp * cy * vn + cp * sy * ve - sp * vd;
    const double vby = (sr * sp * cy - cr * sy) * vn + (sr * sp * sy + cr * cy) * ve + sr * cp * vd;
    const double vbz = (cr * sp * cy + sr * sy) * vn + (cr * sp * sy - sr * cy) * ve + cr * cp * vd;
    if (std::abs(vbx) > 0.5) {
        s.aoa = std::atan2(vbz, vbx);
        s.ssa = std::atan2(vby, vbx);
    } else {
        s.aoa = s.ssa = 0;
    }
}

void StateMapper::vfrHud(const mavlink_message_t &m, VehicleState &s, std::int64_t now)
{
    mavlink_vfr_hud_t v;
    mavlink_msg_vfr_hud_decode(&m, &v);
    s.as = v.airspeed;   // the only airspeed source
    if (now - m_lastGlobalPosTs > kGlobalPosStaleMs) {
        s.gs = v.groundspeed;
        s.vs = v.climb;
    }
}

void StateMapper::homePosition(const mavlink_message_t &m, VehicleState &s)
{
    mavlink_home_position_t h;
    mavlink_msg_home_position_decode(&m, &h);
    const double lat = h.latitude / 1e7, lon = h.longitude / 1e7;
    if (std::abs(lat) > 90 || std::abs(lon) > 180) return;
    if (std::abs(lat) < 0.01 && std::abs(lon) < 0.01) return;   // home not set yet
    if (m_relNav.isRelative()) return;                           // home is the zero of the local frame
    s.homeLat = lat;
    s.homeLon = lon;
    s.homeAlt = h.altitude / 1000.0;
    m_origin.home(lat, lon);
    s.sceneOrigin = m_origin.origin();
    s.sceneOriginEpoch = m_origin.epoch();
    if (s.mode != "REPLAY"_L1 && m_hooks.rememberLastHome) m_hooks.rememberLastHome(lat, lon);
}

void StateMapper::adsbVehicle(const mavlink_message_t &m, VehicleState &s, std::int64_t now)
{
    if (!s.adsbEnabled) return;
    mavlink_adsb_vehicle_t a;
    mavlink_msg_adsb_vehicle_decode(&m, &a);
    TrafficEntry e;
    e.icao24 = QString::number(a.ICAO_address, 16).rightJustified(6, '0'_L1);
    e.callsign = mav::fieldString(a.callsign, 9).trimmed();
    e.lat = a.lat / 1e7;
    e.lon = a.lon / 1e7;
    e.alt = a.altitude / 1000.0;
    e.velocity = a.hor_velocity / 100.0;
    e.heading = a.heading / 100.0;
    e.vertRate = a.ver_velocity / 100.0;
    if (std::abs(e.lat) < 0.01 && std::abs(e.lon) < 0.01) return;
    e.ts = now;
    // tslc: seconds since the receiver last heard this aircraft
    e.posTs = a.tslc > 0 && a.tslc < 60 ? now - std::int64_t(a.tslc) * 1000 : now;
    auto it = std::find_if(s.traffic.begin(), s.traffic.end(), [&](const TrafficEntry &t) { return t.icao24 == e.icao24; });
    if (it != s.traffic.end()) *it = e;
    else s.traffic.push_back(e);
}

void StateMapper::cellInfo(const mavlink_message_t &m, VehicleState &s, std::int64_t now)
{
    mavlink_corv_cell_info_t c;
    mavlink_msg_corv_cell_info_decode(&m, &c);
    static const char *states[] = {"?", "SEARCH", "LIMITED", "CAMPED", "CONNECTED"};
    auto dbm = [](int v) { return v != 32767 ? std::optional<int>(v) : std::nullopt; };
    auto db = [](int v) { return v != 127 ? std::optional<int>(v) : std::nullopt; };
    const CellInfo prev = s.cell;
    CellInfo n;
    n.valid = true;
    n.eci = c.eci;
    n.enb = c.eci / 256;
    n.sector = c.eci & 255;
    n.mcc = c.mcc;
    n.mnc = c.mnc;
    n.tac = c.tac;
    n.pci = c.pci;
    n.earfcn = c.earfcn;
    n.band = c.band;
    n.state = QString::fromLatin1(c.state < 5 ? states[c.state] : "?");
    n.rsrp = dbm(c.rsrp);
    n.rssi = dbm(c.rssi);
    n.rsrq = db(c.rsrq);
    n.sinr = db(c.sinr);
    for (int i = 0; i < 4; ++i) {
        if (c.nb_pci[i] == 65535) continue;
        n.nb.push_back({c.nb_pci[i], c.nb_earfcn[i], dbm(c.nb_rsrp[i])});
    }
    const bool changed = prev.valid && prev.eci && c.eci && prev.eci != c.eci;
    // SINR jumps by 10 dB between readings: the link colour follows an average
    if (!n.sinr) n.sinrAvg = prev.valid ? prev.sinrAvg : std::nullopt;
    else n.sinrAvg = prev.valid && prev.sinrAvg ? *prev.sinrAvg * 0.7 + *n.sinr * 0.3 : double(*n.sinr);
    if (changed)
        LOG_I("cell") << u"cell change: eNB %1/%2 PCI %3 -> eNB %4/%5 PCI %6"_s
                             .arg(prev.enb).arg(prev.sector).arg(prev.pci).arg(n.enb).arg(n.sector).arg(n.pci);
    n.handovers = c.handovers;
    n.neighbours = c.n_neighbours;
    n.receivedAt = now;
    n.changedAt = changed ? now : prev.changedAt;
    s.cell = n;
}

void StateMapper::finish(VehicleState &s, std::int64_t now)
{
    m_relNav.apply(s, now);
    subSurface(s, now);
    if (now - m_lastTrafficPurge > 1000) {
        m_lastTrafficPurge = now;
        s.traffic.erase(std::remove_if(s.traffic.begin(), s.traffic.end(),
                                       [&](const TrafficEntry &t) { return now - t.ts >= kAdsbMaxAgeMs; }),
                        s.traffic.end());
    }
}

/// The surface a sub's depth is measured from (updateSubSurface of main.js): ArduSub
/// puts its origin, on the surface, at 0 m. It is the lake or the sea at home, else the
/// ground there (a quarry the elevation data does not flatten). Again when home moves.
void StateMapper::subSurface(VehicleState &s, std::int64_t now)
{
    if (!modes::isSubType(s.vehicleType) || m_relNav.isRelative() || !m_hooks.surface) {
        s.subSurfaceAlt.reset();
        m_subSurfaceKey.clear();
        return;
    }
    const double lat = s.homeLat.value_or(s.lat), lon = s.homeLon.value_or(s.lon);
    if (lat == 0 && lon == 0) return;
    const QString key = QString::number(lat, 'f', 6) + ','_L1 + QString::number(lon, 'f', 6);
    if (key == m_subSurfaceKey && s.subSurfaceAlt) return;
    if (now - m_subSurfaceTried < 250) return;   // the tile is being read
    m_subSurfaceTried = now;
    if (const auto level = m_hooks.surface(lat, lon)) {
        s.subSurfaceAlt = *level;
        m_subSurfaceKey = key;
    }
}

void StateMapper::resetTelemetry(VehicleState &s)
{
    VehicleState fresh;
    // Keep connection, preferences and the origin
    fresh.mode = s.mode;
    fresh.connected = s.connected;
    fresh.connectionType = s.connectionType;
    fresh.linkType = s.linkType;
    fresh.adsbEnabled = s.adsbEnabled;
    fresh.sceneOrigin = s.sceneOrigin;
    fresh.sceneOriginEpoch = s.sceneOriginEpoch;
    fresh.lat = s.sceneOrigin.lat;
    fresh.lon = s.sceneOrigin.lon;
    s = fresh;
    m_lastGlobalPosTs = 0;
    m_imuPrimaryActive = false;
    m_subSurfaceKey.clear();
}

} // namespace corv
