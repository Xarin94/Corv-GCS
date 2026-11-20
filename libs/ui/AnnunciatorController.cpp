#include "ui/AnnunciatorController.h"

#include "core/Clock.h"
#include "core/DebugLog.h"
#include "vehicle/VehicleHub.h"

#include <QRandomGenerator>
#include <QRegularExpression>
#include <algorithm>
#include <cmath>

using namespace Qt::StringLiterals;

namespace corv {

AnnunciatorController *AnnunciatorController::s_instance = nullptr;

namespace {

QVariantMap part(const char *d, bool filled, bool stroked, const char *dash = nullptr)
{
    QVariantMap m{{u"d"_s, QLatin1String(d)}, {u"fill"_s, filled},
                  {u"stroke"_s, stroked ? 1.4 : 0.0}, {u"cap"_s, u"round"_s}};
    if (dash) {
        QVariantList l;
        for (const QString &s : QString::fromLatin1(dash).split(' '_L1, Qt::SkipEmptyParts)) l << s.toDouble();
        m[u"dash"_s] = l;
    }
    return m;
}

// Generated from the annunciator glyph sheet of html/index.html (24 × 24, stroke 1.4, round caps).
// clang-format off
const QHash<QString, QVariantList> &glyphTable()
{
    static const QHash<QString, QVariantList> t = [] {
        QHash<QString, QVariantList> g;
        g.insert(u"failsafe"_s, QVariantList{part("M12 3l9.5 17h-19z", false, true), part("M12 9.5v5M12 17.5v.5", false, true)});
        g.insert(u"ekf"_s, QVariantList{part("M5 12a7 7 0 1 0 14 0a7 7 0 1 0 -14 0z", false, true, "3 2.6"), part("M12 2v4M12 18v4M2 12h4M18 12h4", false, true), part("M10.8 12a1.2 1.2 0 1 0 2.4 0a1.2 1.2 0 1 0 -2.4 0z", true, false)});
        g.insert(u"vibe"_s, QVariantList{part("M2 12l2.5-6 3 12 3-14 3 14 3-14 3 12L22 12", false, true)});
        g.insert(u"imu"_s, QVariantList{part("M3 12a9 9 0 1 0 18 0a9 9 0 1 0 -18 0z", false, true), part("M3 12a9 3.4 0 1 0 18 0a9 3.4 0 1 0 -18 0z", false, true), part("M12 3v18", false, true), part("M10.8 12a1.2 1.2 0 1 0 2.4 0a1.2 1.2 0 1 0 -2.4 0z", true, false)});
        g.insert(u"mag"_s, QVariantList{part("M3 12a9 9 0 1 0 18 0a9 9 0 1 0 -18 0z", false, true), part("M12 5l2.6 7-2.6 7-2.6-7z", false, true), part("M12 5l2.6 7h-5.2z", true, false)});
        g.insert(u"baro"_s, QVariantList{part("M4.5 17a9 9 0 1 1 15 0", false, true), part("M12 16l4.5-6", false, true), part("M10.8 16a1.2 1.2 0 1 0 2.4 0a1.2 1.2 0 1 0 -2.4 0z", true, false), part("M12 21h0", false, true)});
        g.insert(u"gps"_s, QVariantList{part("M17 3.5l3.5 3.5L17 10.5 13.5 7zM12 8.5l3.5 3.5-3.5 3.5L8.5 12zM7 13.5l3.5 3.5L7 20.5 3.5 17z", false, true), part("M15 15l3.5 3.5", false, true), part("M3 7a4 4 0 0 1 4-4", false, true)});
        g.insert(u"jam"_s, QVariantList{part("M12 22V10", false, true), part("M10 8a2 2 0 1 0 4 0a2 2 0 1 0 -4 0z", false, true), part("M7.5 3.5a7 7 0 0 0 0 9M16.5 3.5a7 7 0 0 1 0 9", false, true), part("M3 21L21 3", false, true)});
        g.insert(u"spoof"_s, QVariantList{part("M12 22s-7-6.3-7-12a7 7 0 0 1 14 0c0 5.7-7 12-7 12z", false, true), part("M9.8 8.3a2.2 2.2 0 1 1 3.1 2c-.6.3-.9.7-.9 1.4", false, true), part("M12 14.2v.4", false, true)});
        g.insert(u"glitch"_s, QVariantList{part("M17 3.5l3.5 3.5L17 10.5 13.5 7zM12 8.5l3.5 3.5-3.5 3.5L8.5 12z", false, true), part("M2 21h3l1.5-4 2 8 2-8 1.5 4h3", false, true)});
        g.insert(u"ahrs"_s, QVariantList{part("M3 12a9 9 0 1 0 18 0a9 9 0 1 0 -18 0z", false, true), part("M4.4 15.6l15.2-7.2", false, true), part("M8.5 12.5h2.5M13 10.5h2.5", false, true), part("M10.8 12a1.2 1.2 0 1 0 2.4 0a1.2 1.2 0 1 0 -2.4 0z", true, false)});
        g.insert(u"rc"_s, QVariantList{part("M6 9h12a2 2 0 0 1 2 2v7a2 2 0 0 1 -2 2h-12a2 2 0 0 1 -2 -2v-7a2 2 0 0 1 2 -2z", false, true), part("M8 9V3", false, true), part("M7.4 14.5a1.6 1.6 0 1 0 3.2 0a1.6 1.6 0 1 0 -3.2 0z", false, true), part("M13.4 14.5a1.6 1.6 0 1 0 3.2 0a1.6 1.6 0 1 0 -3.2 0z", false, true), part("M2 22L22 2", false, true)});
        g.insert(u"batt"_s, QVariantList{part("M4 7h13a1.5 1.5 0 0 1 1.5 1.5v7a1.5 1.5 0 0 1 -1.5 1.5h-13a1.5 1.5 0 0 1 -1.5 -1.5v-7a1.5 1.5 0 0 1 1.5 -1.5z", false, true), part("M21.5 10v4", false, true), part("M5.5 10v4", false, true), part("M12 9.5v3M12 14.5v.4", false, true)});
        g.insert(u"motor"_s, QVariantList{part("M12 12C10 8 9 4 12 2c3 2 2 6 0 10zM12 12c2 4 3 8 0 10-3-2-2-6 0-10zM12 12c4-2 8-3 10 0-2 3-6 2-10 0zM12 12c-4 2-8 3-10 0 2-3 6-2 10 0z", false, true), part("M10.5 12a1.5 1.5 0 1 0 3 0a1.5 1.5 0 1 0 -3 0z", true, false)});
        g.insert(u"fence"_s, QVariantList{part("M4.5 3.5h15a1 1 0 0 1 1 1v15a1 1 0 0 1 -1 1h-15a1 1 0 0 1 -1 -1v-15a1 1 0 0 1 1 -1z", false, true, "3 2.4"), part("M11 13l8-8M15 5h4v4", false, true)});
        g.insert(u"aspd"_s, QVariantList{part("M2 15h9a2 2 0 0 0 2-2V6", false, true), part("M13 6h-3M14 12h8M19 9.5l3 2.5-3 2.5", false, true)});
        g.insert(u"terrain"_s, QVariantList{part("M2 20L8.5 8l4 7 2.5-4L22 20z", false, true), part("M6.5 11.7L8.5 8l2 3.7", false, true)});
        g.insert(u"log"_s, QVariantList{part("M6 3h9l4 4v14H6z", false, true), part("M9 15h6M9 18h6", false, true), part("M3 21L21 3", false, true)});
        g.insert(u"lidar"_s, QVariantList{part("M12 3v13", false, true), part("M8 12l4 4 4-4", false, true), part("M3 20h18", false, true)});
        g.insert(u"flow"_s, QVariantList{part("M9 12a3 3 0 1 0 6 0a3 3 0 1 0 -6 0z", false, true), part("M12 2v3M12 19v3M2 12h3M19 12h3M5 5l2 2M17 17l2 2M5 19l2-2M17 7l2-2", false, true)});
        g.insert(u"prox"_s, QVariantList{part("M10.5 19a1.5 1.5 0 1 0 3 0a1.5 1.5 0 1 0 -3 0z", true, false), part("M6.5 14.5a7.5 7.5 0 0 1 11 0", false, true), part("M3 10.5a12.5 12.5 0 0 1 18 0", false, true)});
        g.insert(u"link"_s, QVariantList{part("M4 20v-4M9 20v-8M14 20v-12M19 20V4", false, true), part("M2 22L22 2", false, true)});
        g.insert(u"cal"_s, QVariantList{part("M20 12a8 8 0 1 1-2.4-5.7", false, true), part("M20 3v4.5h-4.5", false, true), part("M10.8 12a1.2 1.2 0 1 0 2.4 0a1.2 1.2 0 1 0 -2.4 0z", true, false)});
        g.insert(u"zone"_s, QVariantList{part("M3 12a9 9 0 1 0 18 0a9 9 0 1 0 -18 0z", false, true), part("M5.6 5.6l12.8 12.8", false, true), part("M12 3v3M12 18v3", false, true)});
        g.insert(u"ceiling"_s, QVariantList{part("M3 4.5h18", false, true), part("M3 4.5l3-2.5M9 4.5l3-2.5M15 4.5l3-2.5", false, true), part("M12 21V9", false, true), part("M7.5 13.5L12 9l4.5 4.5", false, true)});
        g.insert(u"prearm"_s, QVariantList{part("M7 11h10a2 2 0 0 1 2 2v6a2 2 0 0 1 -2 2h-10a2 2 0 0 1 -2 -2v-6a2 2 0 0 1 2 -2z", false, true), part("M8 11V7a4 4 0 0 1 8 0v4", false, true), part("M12 15v2.5", false, true)});
        return g;
    }();
    return t;
}
// clang-format on

enum Level { Warn, Caut };
struct Alert {
    const char *id;
    Level lvl;
    const char *icon, *label, *title;
    int val = 0;                 // 0 none, 1 link, 2 GPS sats + HDOP, 3 HDOP
    const char *demoVal = nullptr;
};

// Display order within each list: a fault always appears in the same slot
const Alert kAlerts[] = {
    {"failsafe", Warn, "failsafe", "FAILSAFE", "Autopilot in failsafe (system status CRITICAL / EMERGENCY)"},
    {"linkLost", Warn, "link", "LINK", "No heartbeat for 3 s", 1, "RSSI --"},
    {"rc", Warn, "rc", "RC", "RC receiver unhealthy / radio failsafe"},
    {"ekf", Warn, "ekf", "EKF", "EKF variance > 0.8"},
    {"gpsGlitch", Warn, "glitch", "GPS GLITCH", "EKF reports GPS glitching", 2, "7 SAT\nHDOP 2.8"},
    {"gpsJam", Warn, "jam", "GPS JAM", "GNSS receiver reports signal jamming", 2, "4 SAT\nHDOP 4.6"},
    {"gpsSpoof", Warn, "spoof", "SPOOF", "GNSS receiver reports signal spoofing", 2, "12 SAT\nHDOP 0.9"},
    {"gps", Warn, "gps", "GPS", "GPS unhealthy or no fix", 2, "3 SAT\nHDOP 9.9"},
    {"vibe", Warn, "vibe", "VIBE", "Vibration > 60 or accelerometer clipping"},
    {"imu", Warn, "imu", "IMU", "Gyro / accelerometer unhealthy"},
    {"mag", Warn, "mag", "COMPASS", "Compass unhealthy"},
    {"baro", Warn, "baro", "BARO", "Barometer unhealthy"},
    {"ahrs", Warn, "ahrs", "AHRS", "Attitude estimate unhealthy"},
    {"aspd", Warn, "aspd", "AIRSPD", "Airspeed sensor unhealthy"},
    {"motor", Warn, "motor", "MOTOR", "Motor output unhealthy"},
    {"batt", Warn, "batt", "BATT", "Battery unhealthy / critical / failsafe"},
    {"fence", Warn, "fence", "FENCE", "Geofence breach"},

    {"ekfVar", Caut, "ekf", "EKF", "EKF variance > 0.5"},
    {"vibeHi", Caut, "vibe", "VIBE", "Vibration > 30"},
    {"gps2d", Caut, "gps", "2D FIX", "GPS 2D fix only", 2, "5 SAT\nHDOP 3.1"},
    {"hdop", Caut, "gps", "HDOP", "GPS HDOP > 2.0", 3, "HDOP 2.6"},
    {"prearm", Caut, "prearm", "PREARM", "Pre-arm checks failing"},
    {"magCal", Caut, "cal", "MAG CAL", "Compass calibration in progress"},
    {"imuCal", Caut, "cal", "IMU CAL", "Accelerometer / gyro calibration in progress"},
    {"baroCal", Caut, "cal", "BARO CAL", "Barometer calibration in progress"},
    {"battLow", Caut, "batt", "BATT LOW", "Battery low"},
    {"linkWeak", Caut, "link", "LINK", "Telemetry link quality < 50% or RSSI low", 1, "RSSI 41"},
    {"terrain", Caut, "terrain", "TERRAIN", "Terrain data unhealthy / missing"},
    {"logging", Caut, "log", "LOG", "Onboard logging unhealthy"},
    {"lidar", Caut, "lidar", "LIDAR", "Rangefinder unhealthy"},
    {"flow", Caut, "flow", "OPT FLOW", "Optical flow unhealthy"},
    {"prox", Caut, "prox", "PROX", "Proximity sensor unhealthy"},
};

// MAV_SYS_STATUS_SENSOR bits
namespace S {
constexpr std::uint32_t GYRO = 1, ACCEL = 2, MAG = 4, BARO = 8, DPRESS = 16, GPS = 32, FLOW = 64, LASER = 256, MOTOR = 32768,
                        RC = 65536, GYRO2 = 1u << 17, ACCEL2 = 1u << 18, MAG2 = 1u << 19, FENCE = 1u << 20, AHRS = 1u << 21,
                        TERRAIN = 1u << 22, LOGGING = 1u << 24, BATTERY = 1u << 25, PROX = 1u << 26, PREARM = 1u << 28;
}
constexpr int kEkfGpsGlitching = 1 << 15;

// Thresholds (Mission Planner's HUD rules)
constexpr double kEkfWarn = 0.8, kEkfCaut = 0.5, kVibeWarn = 60, kVibeCaut = 30, kHdopCaut = 2.0;
constexpr int kBattLowPct = 20, kLinkWeakPct = 50, kRssiWeak = 60;   // RADIO_STATUS rssi is 0-254
constexpr std::int64_t kStaleMs = 5000, kHeartbeatLostMs = 3000, kClipHoldMs = 10000, kTextHoldMs = 10000;
constexpr std::int64_t kDemoOnMs = 3200, kDemoOffMs = 1000;

// STATUSTEXT → alert, first match wins; "cleared" texts are excluded up front
const std::vector<std::pair<QRegularExpression, const char *>> &textRules()
{
    static const auto ci = QRegularExpression::CaseInsensitiveOption;
    static const std::vector<std::pair<QRegularExpression, const char *>> r = {
        {QRegularExpression(u"jam"_s, ci), "gpsJam"},
        {QRegularExpression(u"spoof"_s, ci), "gpsSpoof"},
        {QRegularExpression(u"gps glitch"_s, ci), "gpsGlitch"},
        {QRegularExpression(u"radio failsafe|rc failsafe|throttle failsafe"_s, ci), "rc"},
        {QRegularExpression(u"battery.*(critical|failsafe)|crit batt"_s, ci), "batt"},
        {QRegularExpression(u"battery.*low|low battery"_s, ci), "battLow"},
        {QRegularExpression(u"ekf.*failsafe|ekf variance"_s, ci), "ekf"},
        {QRegularExpression(u"fence breach|fence.*fail"_s, ci), "fence"},
        {QRegularExpression(u"failsafe"_s, ci), "failsafe"},
        {QRegularExpression(u"prearm"_s, ci), "prearm"},
        {QRegularExpression(u"calibrating (accel|gyro)|accel.*cal|gyro.*cal|place vehicle"_s, ci), "imuCal"},
        {QRegularExpression(u"calibrating baro|baro.*cal"_s, ci), "baroCal"},
        {QRegularExpression(u"compass.*cal|mag.*cal"_s, ci), "magCal"},
        {QRegularExpression(u"terrain.*(missing|unavailable|fail|disabled)"_s, ci), "terrain"},
        {QRegularExpression(u"log(ging)? fail|no logging|bad logging"_s, ci), "logging"},
    };
    return r;
}
const QRegularExpression &textCleared()
{
    static const QRegularExpression re(u"cleared|complete|success|ok\\b|passed|recovered"_s, QRegularExpression::CaseInsensitiveOption);
    return re;
}

// Warning → its caution twin(s): the same condition at a lower level
const QHash<QString, QStringList> &twins()
{
    static const QHash<QString, QStringList> t = {{u"ekf"_s, {u"ekfVar"_s}},
                                                  {u"vibe"_s, {u"vibeHi"_s}},
                                                  {u"gps"_s, {u"gps2d"_s, u"hdop"_s}},
                                                  {u"batt"_s, {u"battLow"_s}},
                                                  {u"linkLost"_s, {u"linkWeak"_s}}};
    return t;
}

QString valueOf(int kind, const VehicleState &s)
{
    switch (kind) {
    case 1: return s.rssi ? u"RSSI %1"_s.arg(*s.rssi) : u"LQ %1%"_s.arg(std::lround(s.linkQuality));
    case 2: return u"%1 SAT\nHDOP %2"_s.arg(s.gpsNumSat).arg(s.gpsHdop, 0, 'f', 1);
    case 3: return u"HDOP %1"_s.arg(s.gpsHdop, 0, 'f', 1);
    default: return {};
    }
}

} // namespace

AnnunciatorController::AnnunciatorController(VehicleHub *hub, QObject *parent) : QObject(parent), m_hub(hub)
{
    s_instance = this;
    connect(m_hub, &VehicleHub::statusText, this, [this](int, const QString &t) { onStatusText(t); });
    // A new vehicle: everything silenced comes back
    connect(m_hub, &VehicleHub::connectionChanged, this, [this](bool connected, const QString &) {
        if (!connected) return;
        m_silenced.clear();
        update();
    });
    m_timer.setInterval(100);
    connect(&m_timer, &PacedTimer::timeout, this, &AnnunciatorController::update);
    m_timer.start();
}

QVariantList AnnunciatorController::glyph(const QString &icon) const { return glyphTable().value(icon); }

void AnnunciatorController::onStatusText(const QString &text)
{
    if (text.isEmpty() || textCleared().match(text).hasMatch()) return;
    for (const auto &[re, id] : textRules())
        if (re.match(text).hasMatch()) {
            m_timed[QLatin1String(id)] = wallMs() + kTextHoldMs;
            return;
        }
}

void AnnunciatorController::silence(const QString &id)
{
    if (id.isEmpty()) return;
    LOG_I("annun") << "silenced by the operator:" << id;
    m_silenced.insert(id);
    for (const QString &t : twins().value(id)) m_silenced.insert(t);
    if (id == m_demoId) {
        m_demoId.clear();
        m_demoNextAt = wallMs() + kDemoOffMs;
    }
    update();
}

QSet<QString> AnnunciatorController::evaluate(std::int64_t now)
{
    const auto snap = m_hub->snapshot();
    const VehicleState &s = *snap;
    QSet<QString> on;
    for (auto it = m_timed.begin(); it != m_timed.end();) {
        if (it.value() > now) {
            on.insert(it.key());
            ++it;
        } else {
            it = m_timed.erase(it);
        }
    }
    const auto unsilenced = [&] {
        for (const QString &id : m_silenced) on.remove(id);
        return on;
    };
    if (s.isDemo()) {
        if (!m_demoId.isEmpty()) on.insert(m_demoId);
        return unsilenced();
    }
    if (!s.connected) return unsilenced();

    const bool hbFresh = now - s.lastHeartbeatTime < kHeartbeatLostMs;
    if (s.lastHeartbeatTime && !hbFresh) on.insert(u"linkLost"_s);
    if (s.systemStatus == 5 || s.systemStatus == 6) on.insert(u"failsafe"_s);

    const auto bad = [&](std::uint32_t bit) { return (s.sensorsPresent & bit) && (s.sensorsEnabled & bit) && !(s.sensorsHealth & bit); };
    if (now - s.sysStatusTime < kStaleMs) {
        if (bad(S::GYRO) || bad(S::ACCEL) || bad(S::GYRO2) || bad(S::ACCEL2)) on.insert(u"imu"_s);
        if (bad(S::MAG) || bad(S::MAG2)) on.insert(u"mag"_s);
        const std::pair<std::uint32_t, const char *> single[] = {{S::BARO, "baro"}, {S::DPRESS, "aspd"}, {S::GPS, "gps"}, {S::AHRS, "ahrs"},
                                                                 {S::RC, "rc"}, {S::MOTOR, "motor"}, {S::BATTERY, "batt"}, {S::FENCE, "fence"},
                                                                 {S::TERRAIN, "terrain"}, {S::LOGGING, "logging"}, {S::LASER, "lidar"},
                                                                 {S::FLOW, "flow"}, {S::PROX, "prox"}};
        for (const auto &[bit, id] : single)
            if (bad(bit)) on.insert(QLatin1String(id));
        // Pre-arm only matters on the ground
        if (!s.armed && bad(S::PREARM)) on.insert(u"prearm"_s);
    }
    if (now - s.ekfDataTime < kStaleMs) {
        if (s.ekfVariance > kEkfWarn) on.insert(u"ekf"_s);
        else if (s.ekfVariance > kEkfCaut) on.insert(u"ekfVar"_s);
        if (s.ekfFlags & kEkfGpsGlitching) on.insert(u"gpsGlitch"_s);
    }
    // Vibration: level thresholds, plus clipping (an increase is held red a while)
    const std::int64_t clipSum = std::int64_t(s.vibClip0) + s.vibClip1 + s.vibClip2;
    if (m_lastClipSum >= 0 && clipSum > m_lastClipSum) m_lastClipChange = now;
    m_lastClipSum = clipSum;
    const double vibMax = std::max({s.vibX, s.vibY, s.vibZ});
    if (vibMax > kVibeWarn || now - m_lastClipChange < kClipHoldMs) on.insert(u"vibe"_s);
    else if (vibMax > kVibeCaut) on.insert(u"vibeHi"_s);

    if (s.gpsDataTime && now - s.gpsDataTime < kStaleMs) {
        if (s.gpsFix < 2) on.insert(u"gps"_s);
        else if (s.gpsFix == 2) on.insert(u"gps2d"_s);
        else if (s.gpsHdop > kHdopCaut) on.insert(u"hdop"_s);
    }
    if (now - s.gnssIntegrityTime < kStaleMs) {
        if (s.gnssJamming >= 2) on.insert(u"gpsJam"_s);   // mitigated or detected
        if (s.gnssSpoofing >= 2) on.insert(u"gpsSpoof"_s);
    }
    if (s.magCalTime && now - s.magCalTime < kStaleMs) on.insert(u"magCal"_s);
    if (s.batteryVoltage > 0 && s.batteryRemaining >= 0 && s.batteryRemaining <= kBattLowPct) on.insert(u"battLow"_s);
    // Link quality only exists where SYS_STATUS does (MSP never sends it)
    const bool lqWeak = now - s.sysStatusTime < kStaleMs && s.linkQuality < kLinkWeakPct;
    if (hbFresh && (lqWeak || (s.rssi && *s.rssi < kRssiWeak))) on.insert(u"linkWeak"_s);

    // Warning beats its caution twin
    for (auto it = twins().cbegin(); it != twins().cend(); ++it)
        if (on.contains(it.key()))
            for (const QString &t : it.value()) on.remove(t);
    return unsilenced();
}

void AnnunciatorController::stepDemo(std::int64_t now)
{
    if (!m_demoId.isEmpty() && now >= m_demoUntil) {
        m_demoId.clear();
        m_demoNextAt = now + kDemoOffMs;
    }
    if (m_demoId.isEmpty() && now >= m_demoNextAt) {
        QStringList pool;
        for (const Alert &a : kAlerts)
            if (!m_silenced.contains(QLatin1String(a.id))) pool << QLatin1String(a.id);
        if (pool.isEmpty()) return;
        m_demoId = pool[QRandomGenerator::global()->bounded(int(pool.size()))];
        m_demoUntil = now + kDemoOnMs;
    }
}

void AnnunciatorController::update()
{
    const std::int64_t now = wallMs();
    const auto snap = m_hub->snapshot();
    const bool demo = snap->isDemo();
    if (demo) stepDemo(now);
    else if (!m_demoId.isEmpty()) {
        m_demoId.clear();
        m_demoNextAt = 0;
    }
    const QSet<QString> on = evaluate(now);
    QVariantList warns, cauts;
    for (const Alert &a : kAlerts) {
        const QString id = QLatin1String(a.id);
        if (!on.contains(id)) continue;
        const QVariantMap item{{u"id"_s, id},
                               {u"label"_s, QLatin1String(a.label)},
                               {u"icon"_s, QLatin1String(a.icon)},
                               {u"title"_s, u"%1\nClick to silence until the next connection"_s.arg(QLatin1String(a.title))},
                               {u"value"_s, a.val ? (demo ? QString::fromUtf8(a.demoVal ? a.demoVal : "") : valueOf(a.val, *snap)) : QString()}};
        (a.lvl == Warn ? warns : cauts) << item;
    }
    // Changes go to the debug log (not the demo's)
    if (!demo) {
        for (const Alert &a : kAlerts) {
            const QString id = QLatin1String(a.id);
            const bool was = m_loggedOn.contains(id), is = on.contains(id);
            if (was == is) continue;
            const QString lvl = a.lvl == Warn ? u"warning"_s : u"caution"_s;
            if (is) {
                const QString val = a.val ? u" (%1)"_s.arg(valueOf(a.val, *snap).replace('\n'_L1, ' '_L1)) : QString();
                if (a.lvl == Warn) LOG_W("annun") << "WARNING on:" << a.label << val << "—" << a.title;
                else LOG_I("annun") << "caution on:" << a.label << val << "—" << a.title;
            } else {
                LOG_I("annun") << lvl << "off:" << a.label;
            }
        }
        m_loggedOn = on;
    }
    if (warns != m_warnings || cauts != m_cautions) {
        m_warnings = warns;
        m_cautions = cauts;
        emit listChanged();
    }
}

} // namespace corv
