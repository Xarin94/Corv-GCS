#include "ui/RotorLoadItem.h"

#include "core/Clock.h"
#include "ui/GcsController.h"
#include "ui/HudPaint.h"
#include "ui/SysConfig.h"
#include "ui/VehicleModel.h"
#include "vehicle/CommandSender.h"
#include "vehicle/ParamStore.h"
#include "vehicle/VehicleState.h"

#include <QFont>
#include <QFontMetricsF>
#include <QPainter>
#include <QPainterPath>
#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <vector>

using namespace Qt::StringLiterals;

namespace corv {

using namespace hudpaint;

namespace {

const QString kCfgKey = u"rotor-load-cfg"_s;

double clamp(double v, double lo, double hi) { return std::min(std::max(v, lo), hi); }
double wrap180(double d) { return std::fmod(std::fmod(d + 180, 360) + 360, 360) - 180; }

// ── Airframe geometry ───────────────────────────────────────────────────

// Motor positions as ArduPilot defines them in AP_MotorsMatrix: angle in
// degrees clockwise from the nose, indexed by motor number. `outputs` is the
// 1-based servo output each motor drives (tricopters skip output 3).
struct Frame {
    std::vector<double> angles;
    std::vector<int> outputs;
};

const std::map<int, Frame> &frames()
{
    static const std::map<int, Frame> f = {
        {3, {{60, 300, 180}, {1, 2, 4}}},
        {4, {{45, 225, 315, 135}, {1, 2, 3, 4}}},
        {6, {{30, 210, 270, 90, 330, 150}, {1, 2, 3, 4, 5, 6}}},
        {8, {{22.5, 202.5, 67.5, 157.5, 337.5, 292.5, 247.5, 112.5}, {1, 2, 3, 4, 5, 6, 7, 8}}}};
    return f;
}

// MAV_TYPE → number of rotors on the schematic. Plane and helicopter show the
// single main rotor only, as does anything we have no matrix for.
int rotorsByMavType(int t)
{
    switch (t) {
    case 1: return 1;    // FIXED_WING
    case 2: return 4;    // QUADROTOR
    case 3: return 1;    // COAXIAL
    case 4: return 1;    // HELICOPTER
    case 13: return 6;   // HEXAROTOR
    case 14: return 8;   // OCTOROTOR
    case 15: return 3;   // TRICOPTER
    case 16: return 1;   // FLAPPING_WING (delta wings)
    case 19: case 20: case 21: case 22: case 23: case 24: case 25: return 4;   // VTOL variants: lift quad
    case 27: return 10;  // DECAROTOR
    case 29: return 12;  // DODECAROTOR
    }
    return 0;
}

// Vehicles with no rotor worth drawing
bool hiddenMavType(int t) { return t == 5 || t == 6 || t == 10 || t == 11 || t == 12 || t == 17 || t == 18; }

/// Evenly spaced ring of `count` motors, numbered the way ArduPilot numbers
/// its matrices: alternate opposite arms.
Frame evenRing(int count)
{
    Frame f;
    const double step = 360.0 / count;
    for (int i = 0; i < count; ++i) {
        const int k = (i % 2 == 0) ? i / 2 : (count + i - 1) / 2;
        f.angles.push_back(std::fmod(step / 2 + k * step, 360.0));
        f.outputs.push_back(i + 1);
    }
    return f;
}

/// The {angles, outputs} table for a rotor count, generating an evenly spaced
/// ring for the counts with no explicit ArduPilot table (deca, dodeca).
Frame frameFor(int count)
{
    const auto it = frames().find(count);
    return it != frames().end() ? it->second : evenRing(count);
}

// Some MotorN outputs are servos, not motors: a helicopter's Motor1..4 are its
// swashplate and tail servos (the rotor is the HeliRSC output), a single or
// coaxial copter's Motor1..4 its fins (the rotors are Motor5 and 6), and a
// tricopter's Motor7 its tail servo (AP_MotorsTri).
constexpr int kMavHelicopter = 4;
constexpr int kMavCoaxial = 3;
constexpr int kMavTricopter = 15;

/// The motor outputs that drive rotors, by motor number.
std::map<int, int> rotorMotors(const std::map<int, int> &motors, int mavType)
{
    std::map<int, int> out;
    for (const auto &[n, ch] : motors) {
        if (mavType == kMavCoaxial && n <= 4) continue;
        if (mavType == kMavTricopter && n == 7) continue;
        out[n] = ch;
    }
    return out;
}

constexpr int kAutopilotPx4 = 12;   // MAV_AUTOPILOT_PX4: no SERVOn_FUNCTION

// ── Sailboat: wing and rudder ───────────────────────────────────────────
//
// A Rover with SAIL_ENABLE or a MastRotation output draws two dials in place
// of the rotor gauges. ArduPilot spreads ±100 % of an angle output from
// SERVOn_TRIM to SERVOn_MIN / _MAX, so the commanded angle follows from
// SERVO_OUTPUT_RAW and those three parameters: times SAIL_ANGLE_MAX for the
// mast, times rudderMax for the rudder (full steering). The measured position
// of a DroneCAN servo (actuator.Status) never reaches MAVLink — ArduPilot only
// logs it, in CSRV — so the dials show the commanded angle.

bool roverMavType(int t) { return t == 10 || t == 11; }   // ground rover, surface boat

// The parameters the schematic needs (SERVOn_FUNCTION, the sail settings, the
// flap outputs' range) are read one by one when a vehicle connects and the
// schematic is on: a full download is not needed and may never happen on a
// slow link. Each name goes out once, then a few rounds ask again for what
// never arrived.
constexpr std::int64_t kProbeGapMs = 120;
constexpr std::int64_t kProbeRoundMs = 4000;
constexpr int kProbeRetries = 2;

/// Arc the trailing edge sweeps over the full travel, as [lo, hi] bearings
/// around the stern, or none for a full turn (nothing to mark).
std::optional<std::pair<double, double>> travelDeg(double maxDeg)
{
    if (maxDeg >= 179.5) return std::nullopt;
    return std::make_pair(180 - maxDeg, 180 + maxDeg);
}

constexpr double kGaugeStart = 135;   // lower-left, degrees clockwise from +x (canvas)
constexpr double kGaugeSweep = 270;   // clockwise
constexpr double kLineW = 4;          // arcs and ticks share one stroke weight

// Wing and rudder drawn as a symmetric NACA 00xx section: thick enough that
// the round leading edge and the sharp trailing edge read at dial size, and
// short enough that the trailing edge stays inside the 90° ticks
constexpr double kFoilThickness = 0.18;   // NACA 0018
constexpr double kFoilChord = 1.05;       // of the dial radius
constexpr double kFoilPivot = 0.3;        // mast / rudder stock, chord fraction from the leading edge
constexpr int kFoilSteps = 24;

QString signedDeg(double d)
{
    const QString sign = d > 0.05 ? u"+"_s : d < -0.05 ? u"−"_s : QString();
    return sign + QString::number(std::abs(d), 'f', 0) + u"°"_s;
}

} // namespace

RotorLoadItem::RotorLoadItem(QQuickItem *parent) : QQuickPaintedItem(parent), m_flapPct(qQNaN())
{
    setAntialiasing(true);
}

void RotorLoadItem::componentComplete()
{
    QQuickPaintedItem::componentComplete();
    loadConfig();
    if (SysConfig::s_instance)
        connect(SysConfig::s_instance, &SysConfig::jsonFieldChanged, this, [this](const QString &key) {
            if (key != kCfgKey) return;
            loadConfig();
            refresh();
        });
    if (VehicleModel::s_instance) connect(VehicleModel::s_instance, &VehicleModel::updated, this, &RotorLoadItem::refresh);
    // SERVOn_FUNCTION is read again only when the parameters change
    if (ParamStore::s_instance) connect(ParamStore::s_instance, &ParamStore::changed, this, [this] { m_servosDirty = true; });
    refresh();
}

// ── Configuration (SYS CONFIG > ROTOR LOAD, persisted) ──────────────────

void RotorLoadItem::loadConfig()
{
    Config d;
    auto num = [](const QVariant &v, double def) {
        bool ok = false;
        const double x = v.toDouble(&ok);
        return ok && std::isfinite(x) ? x : def;
    };
    if (auto *sc = SysConfig::s_instance) {
        d.enabled = sc->jsonField(kCfgKey, u"enabled"_s, true).toBool();
        d.pwmMin = num(sc->jsonField(kCfgKey, u"pwmMin"_s, 1000), 1000);
        d.pwmMax = num(sc->jsonField(kCfgKey, u"pwmMax"_s, 2000), 2000);
        d.thGreen = num(sc->jsonField(kCfgKey, u"thGreen"_s, 1100), 1100);
        d.thOrange = num(sc->jsonField(kCfgKey, u"thOrange"_s, 1500), 1500);
        d.thRed = num(sc->jsonField(kCfgKey, u"thRed"_s, 1700), 1700);
        d.frame = sc->jsonField(kCfgKey, u"frame"_s, u"auto"_s).toString();
        d.singleChannel = int(num(sc->jsonField(kCfgKey, u"singleChannel"_s, 3), 3));
        d.rudderMax = num(sc->jsonField(kCfgKey, u"rudderMax"_s, 35), 35);
    }
    // Thresholds that cross over each other would make the colours and the
    // scale meaningless, so they are forced back into ascending order.
    d.pwmMin = std::min(std::max(d.pwmMin, 500.0), 2500.0);
    d.pwmMax = std::min(std::max(d.pwmMax, d.pwmMin + 100), 2500.0);
    d.thGreen = std::min(std::max(d.thGreen, d.pwmMin), d.pwmMax);
    d.thOrange = std::min(std::max(d.thOrange, d.thGreen), d.pwmMax);
    d.thRed = std::min(std::max(d.thRed, d.thOrange), d.pwmMax);
    d.singleChannel = std::min(std::max(d.singleChannel, 1), 16);
    d.rudderMax = std::min(std::max(d.rudderMax, 1.0), 180.0);
    if (d.frame.isEmpty()) d.frame = u"auto"_s;
    m_cfg = d;
}

// ── Vehicle ─────────────────────────────────────────────────────────────

std::optional<double> RotorLoadItem::param(const QString &name) const
{
    const ParamValue *p = ParamStore::s_instance ? ParamStore::s_instance->find(name) : nullptr;
    if (!p || !std::isfinite(p->value)) return std::nullopt;
    return p->value;
}

/// Rotor count currently on screen: manual override, else MAV_TYPE, else 1.
int RotorLoadItem::activeRotorCount() const
{
    if (m_cfg.frame != "auto"_L1) {
        bool ok = false;
        const int n = m_cfg.frame.toInt(&ok);
        if (ok && n >= 1) return n;
    }
    const int n = m_s ? rotorsByMavType(m_s->vehicleType) : 0;
    return n ? n : 1;
}

bool RotorLoadItem::isHiddenVehicle() const
{
    return m_cfg.frame == "auto"_L1 && m_s && hiddenMavType(m_s->vehicleType);
}

/// The servo map applies to a vehicle on a link or in a replay; the demo has
/// no parameters of its own.
bool RotorLoadItem::servosApply() const
{
    return m_s && !m_s->isDemo();
}

/// The gauges and where they go. On AUTO, the motors the outputs drive once
/// every SERVOn_FUNCTION is known: MotorN on the ring at ArduPilot's matrix
/// position for that motor (labelled with the motor number, as the autopilot
/// names it), the fixed-wing throttles in the middle left to right (labelled
/// with their output, as the single gauge always was), a helicopter's RSC
/// output alone. Before that, and with FRAME picked by hand, MAV_TYPE or the
/// manual rotor count on outputs 1..n.
RotorLoadItem::Rotors RotorLoadItem::rotors() const
{
    Rotors r;
    if (m_cfg.frame == "auto"_L1 && servosApply() && m_servos.complete()) {
        const int type = m_s->vehicleType;
        const int rsc = m_servos.first(servo::kHeliRsc);
        if ((type == kMavHelicopter || type == kMavCoaxial) && rsc) {
            r.centre.push_back({rsc, u"CH%1"_s.arg(rsc), std::nullopt});
            return r;
        }
        if (type != kMavHelicopter) {
            const std::map<int, int> motors = rotorMotors(m_servos.motors(), type);
            if (type == kMavCoaxial || motors.size() == 1) {
                // Coaxial rotors share the axis: side by side in the middle
                for (const auto &[n, ch] : motors) r.centre.push_back({ch, u"M%1"_s.arg(n), std::nullopt});
            } else if (motors.size() > 1) {
                std::vector<int> nums;
                for (const auto &m : motors) nums.push_back(m.first);
                // ArduPilot's own position when the motors are numbered as in its
                // matrix (a tricopter's 1, 2, 4), evenly round the ring otherwise
                const Frame f = frameFor(int(nums.size()));
                std::vector<int> table = f.outputs;
                std::sort(table.begin(), table.end());
                const Frame even = evenRing(int(nums.size()));
                for (std::size_t i = 0; i < nums.size(); ++i) {
                    double angle = even.angles[i];
                    if (table == nums) {
                        const auto it = std::find(f.outputs.begin(), f.outputs.end(), nums[i]);
                        angle = f.angles[std::size_t(it - f.outputs.begin())];
                    }
                    r.ring.push_back({motors.at(nums[i]), u"M%1"_s.arg(nums[i]), angle});
                }
            }
            for (const ServoMap::Engine &e : m_servos.throttles())
                r.centre.push_back({e.output, u"CH%1"_s.arg(e.output), std::nullopt});
            if (!r.ring.empty() || !r.centre.empty()) return r;
        }
    }
    const int count = activeRotorCount();
    if (count == 1) {
        r.centre.push_back({m_cfg.singleChannel, u"CH%1"_s.arg(m_cfg.singleChannel), std::nullopt});
        return r;
    }
    // Labelled by output channel, not by index — a tricopter's tail motor is
    // M4 on output 4, and the label has to match what the autopilot says
    const Frame f = frameFor(count);
    for (std::size_t i = 0; i < f.angles.size(); ++i)
        r.ring.push_back({f.outputs[i], u"M%1"_s.arg(f.outputs[i]), f.angles[i]});
    return r;
}

/// Wing and rudder outputs and the mast scale, or none for a vehicle with no sail.
std::optional<RotorLoadItem::SailLayout> RotorLoadItem::sailLayout() const
{
    const int sail = m_servos.first(servo::kMastRotation);
    const int rudder = m_servos.first(servo::kGroundSteering);
    const auto enable = param(u"SAIL_ENABLE"_s);
    if (m_cfg.frame != "sail"_L1 && !sail && !(enable && *enable >= 1)) return std::nullopt;
    // Until the parameters arrive: ArduPilot's defaults, and AutoSail's mast on 4
    return SailLayout{sail ? sail : 4, rudder ? rudder : 1, param(u"SAIL_ANGLE_MAX"_s).value_or(90)};
}

/// The sail layout when the dials replace the rotor gauges, else none.
std::optional<RotorLoadItem::SailLayout> RotorLoadItem::sailMode() const
{
    if (m_cfg.frame == "sail"_L1) return sailLayout();
    if (m_cfg.frame != "auto"_L1 || !m_s || !roverMavType(m_s->vehicleType)) return std::nullopt;
    return sailLayout();
}

QStringList RotorLoadItem::wantedParams() const
{
    QStringList names;
    for (int ch = 1; ch <= servo::kOutputs; ++ch) names << u"SERVO%1_FUNCTION"_s.arg(ch);
    // The sail settings only where a sail can be: a Rover, or FRAME on SAILBOAT
    if (m_cfg.frame == "sail"_L1 || (m_s && roverMavType(m_s->vehicleType))) {
        names << u"SAIL_ENABLE"_s << u"SAIL_ANGLE_MAX"_s;
        if (const auto lay = sailLayout()) {
            for (int ch : {lay->sail, lay->rudder})
                names << u"SERVO%1_MIN"_s.arg(ch) << u"SERVO%1_TRIM"_s.arg(ch)
                      << u"SERVO%1_MAX"_s.arg(ch);
        }
    }
    names << m_servos.flapParams();
    return names;
}

void RotorLoadItem::probeParams(std::int64_t now)
{
    const VehicleState &s = *m_s;
    if (!s.connected || s.mode != "LIVE"_L1 || !s.connectionType.startsWith("mavlink"_L1)) {
        // The next link starts again
        if (m_probe.sys != -1) m_probe = Probe{};
        return;
    }
    // Nothing goes out while the schematic is off
    if (!m_cfg.enabled) return;
    if (s.autopilotType == kAutopilotPx4) return;
    // Nothing to draw on a tracker, a GCS or a sub; a Rover may still have a sail
    if (m_cfg.frame == "auto"_L1 && hiddenMavType(s.vehicleType) && !roverMavType(s.vehicleType)) return;
    // Another vehicle on the link, or the link restarted
    if (s.heartbeatCount < m_probe.hb || s.systemId != m_probe.sys) {
        m_probe = Probe{};
        m_probe.sys = s.systemId;
    }
    m_probe.hb = s.heartbeatCount;
    if (now < m_probe.next) return;
    // Each name goes out once on every link, known or not: the store may still
    // hold another vehicle's values. An output found to carry flaps adds its
    // range here, right after the functions.
    const QStringList wanted = wantedParams();
    for (const QString &n : wanted)
        if (!m_probe.asked.contains(n) && !m_probe.queue.contains(n)) m_probe.queue << n;
    if (m_probe.queue.isEmpty()) {
        // Then a few rounds, a while apart, for what never arrived
        if (m_probe.rounds >= kProbeRetries || now - m_probe.lastSent < kProbeRoundMs) return;
        for (const QString &n : wanted)
            if (!param(n)) m_probe.queue << n;
        if (m_probe.queue.isEmpty()) return;
        ++m_probe.rounds;
    }
    CommandSender *cmd = GcsController::s_instance ? GcsController::s_instance->commands() : nullptr;
    const QString name = m_probe.queue.takeFirst();
    m_probe.asked.insert(name);
    if (cmd) cmd->requestParameter(name);
    m_probe.lastSent = now;
    m_probe.next = now + kProbeGapMs;
}

/// Angle commanded on a 1-based output, or none with no SERVO_OUTPUT_RAW: the
/// PWM as a fraction of the way from SERVOn_TRIM to _MAX (or _MIN), times the
/// angle at full output. ArduPilot's defaults stand in until they are read.
std::optional<double> RotorLoadItem::commandedDeg(int output, double maxDeg) const
{
    const double pwm = m_s->servoOutputs[std::size_t(output - 1)];
    if (pwm <= 0) return std::nullopt;
    const double lo = param(u"SERVO%1_MIN"_s.arg(output)).value_or(1100);
    const double trim = param(u"SERVO%1_TRIM"_s.arg(output)).value_or(1500);
    const double hi = param(u"SERVO%1_MAX"_s.arg(output)).value_or(1900);
    if (pwm >= trim) return hi > trim ? clamp((pwm - trim) / (hi - trim), 0, 1) * maxDeg : 0;
    return trim > lo ? -clamp((trim - pwm) / (trim - lo), 0, 1) * maxDeg : 0;
}

/// Apparent wind from the bow (deg, + = from starboard), or none. AppWndDir is
/// the vane's own reading. WIND alone carries the true wind; with no wind speed
/// sensor ArduPilot makes it apparent + heading, so the heading comes off again.
std::optional<double> RotorLoadItem::apparentWindDeg(std::int64_t now) const
{
    const VehicleState &s = *m_s;
    if (s.appWindTime && now - s.appWindTime < 3000) return wrap180(s.appWindDir);
    if (s.windDataTime && now - s.windDataTime < 5000) return wrap180(s.windDir - s.yaw * 180 / M_PI);
    return std::nullopt;
}

// ── PWM sourcing ────────────────────────────────────────────────────────

/// Live PWM for a 1-based output channel, or none when nothing has been
/// received. In demo mode there is no autopilot, so the values are synthesised
/// from the demo attitude to keep the schematic alive.
std::optional<double> RotorLoadItem::pwmFor(int output, std::optional<double> angleDeg) const
{
    if (m_s->isDemo()) return demoPwm(angleDeg);
    const double v = m_s->servoOutputs[std::size_t(output - 1)];
    if (v > 0) return v;
    return std::nullopt;
}

double RotorLoadItem::demoPwm(std::optional<double> angleDeg) const
{
    const VehicleState &s = *m_s;
    if (!angleDeg) {
        // Single rotor: throttle tracks airspeed around a cruise setting
        return clamp(1350 + s.as * 12, m_cfg.pwmMin, m_cfg.pwmMax);
    }
    const double a = *angleDeg * M_PI / 180;
    // Roll lifts the outboard arms, pitch the aft ones — same sign convention
    // as a real mixer, just scaled to something readable.
    const double roll = s.roll * 260;
    const double pitch = s.pitch * 260;
    const double base = 1480 - s.vs * 20;
    return clamp(base - std::sin(a) * roll + std::cos(a) * pitch, 1050, 1950);
}

QColor RotorLoadItem::zoneColor(std::optional<double> pwm) const
{
    if (!pwm) return kNoData;
    if (*pwm < m_cfg.thGreen) return kIdle;
    if (*pwm < m_cfg.thOrange) return kGreen;
    if (*pwm < m_cfg.thRed) return kOrange;
    return kRed;
}

// ── Panel lifecycle ─────────────────────────────────────────────────────

void RotorLoadItem::applyVisibility(const std::optional<SailLayout> &lay)
{
    const bool sail = lay.has_value();
    const bool show = m_cfg.enabled && (sail || !isHiddenVehicle());
    // FLAPS: part of the schematic, there only when an output carries flaps
    const bool flaps = show && !sail && servosApply() && m_servos.flaps().kind != ServoMap::Flaps::None;
    if (m_applied && sail == m_sailShown && show == m_shown && flaps == m_flapShown) return;
    m_applied = true;
    m_sailShown = sail;
    m_shown = show;
    m_flapShown = flaps;
    emit layoutChanged();
}

/// Flap deployment read back from the outputs, NaN while hidden or with no PWM.
void RotorLoadItem::updateFlaps()
{
    std::optional<double> pct;
    if (m_flapShown) {
        const ServoMap::Lookup lookup = [this](const QString &n) { return param(n); };
        pct = ServoMap::flapPercent(m_servos.flaps(), m_s->servoOutputs, [&](int ch) { return ServoMap::scale(lookup, ch); });
    }
    const qreal v = pct ? qreal(*pct) : qQNaN();
    if (std::isnan(v) ? std::isnan(m_flapPct) : std::abs(v - m_flapPct) < 0.01) return;
    m_flapPct = v;
    emit flapChanged();
}

/// Redraw the schematic: every VehicleModel update while on screen.
void RotorLoadItem::refresh()
{
    if (!VehicleModel::s_instance) return;
    m_s = VehicleModel::s_instance->state();
    if (!m_s) return;
    if (m_servosDirty || !ParamStore::s_instance) {
        m_servos = ServoMap::read([this](const QString &n) { return param(n); });
        m_servosDirty = false;
    }
    // A Rover is hidden until its parameters say it has a sail, so they are
    // asked for before the visibility check
    probeParams(wallMs());
    m_lay = sailMode();
    applyVisibility(m_lay);
    updateFlaps();
    if (!m_shown) return;
    if (!m_lay) m_rotors = rotors();
    if (isVisible()) update();
}

// ── Rendering ───────────────────────────────────────────────────────────

QColor RotorLoadItem::ink(const QColor &c) const
{
    return hudpaint::ink(c, m_shadow);
}

void RotorLoadItem::paint(QPainter *p)
{
    if (!m_s || !m_shown) return;
    const double w = width(), h = height();
    if (w <= 0 || h <= 0) return;
    p->setRenderHint(QPainter::Antialiasing);
    p->setRenderHint(QPainter::TextAntialiasing);
    // Shadow pass, then the drawing itself
    for (bool shadow : {true, false}) {
        m_shadow = shadow;
        p->save();
        if (shadow) p->translate(kShadowOffset, kShadowOffset);
        if (m_lay) drawSail(p, w, h, *m_lay);
        else drawRotors(p, w, h);
        p->restore();
    }
    m_shadow = false;
}

void RotorLoadItem::drawRotors(QPainter *p, double w, double h)
{
    const double cx = w / 2, cy = h / 2;
    const std::vector<Gauge> &ring = m_rotors.ring;
    const std::vector<Gauge> &centre = m_rotors.centre;
    const auto draw = [&](const Gauge &g, double x, double y, double r) {
        drawGauge(p, x, y, r, pwmFor(g.output, g.angle), g.label);
    };

    if (ring.empty()) {
        // One big gauge, or the engines side by side, left to right
        const std::size_t k = centre.size();
        if (k == 1) draw(centre[0], cx, cy, std::min(w, h) * 0.4);
        if (k < 2) return;
        const double cell = w / double(k);
        const double r = std::max(8.0, std::min(cell / 2 - 3, h * 0.4));
        for (std::size_t i = 0; i < k; ++i) draw(centre[i], cell * (double(i) + 0.5), cy, r);
        return;
    }

    // Only the gauges are drawn — no airframe body, no arms. Their positions
    // already say which motor is which, so the outline would be pure clutter.
    // The ring is a true circle, not an ellipse: a quad has to look square.
    const int count = int(ring.size());
    const double avail = std::min(w, h) / 2 - 2;
    // Largest gauge whose neighbours on the ring still clear each other:
    // spacing 2·ring·sin(π/n) ≥ 2·gr with ring = avail − gr, minus a small gap
    // so neighbouring circles stay visually separate.
    const double sep = std::sin(M_PI / count) * 0.9;
    double gr = clamp(avail * sep / (1 + sep), 8, 58);
    // The forward engines of a quadplane go in the middle, a size smaller than
    // the arms: the arm gauges shrink until the middle row clears them
    // (ring − gr − half row ≥ gap, the half row in middle-gauge radii)
    constexpr double kCentreScale = 0.75;
    const std::size_t k = centre.size();
    const double halfRow = 1 + 1.1 * (double(k) - 1);
    if (k) gr = std::max(6.0, std::min(gr, (avail - 3) / (2 + kCentreScale * halfRow)));
    const double ringR = avail - gr;

    for (const Gauge &g : ring) {
        const double a = (g.angle.value_or(0) - 90) * M_PI / 180;
        draw(g, cx + std::cos(a) * ringR, cy + std::sin(a) * ringR, gr);
    }
    const double gc = gr * kCentreScale;
    for (std::size_t i = 0; i < k; ++i) draw(centre[i], cx + (double(i) - (double(k) - 1) / 2) * 2.2 * gc, cy, gc);
}

/// One EIS-style round gauge, in the spirit of a Boeing/Airbus N1 indicator:
/// a bare scale circle with an inward tick at zero, an inward moving tick at
/// the current value, and a filled arc between the two. No bezel, no needle,
/// no face, no digits — only the motor label sits in the middle.
void RotorLoadItem::drawGauge(QPainter *p, double cx, double cy, double r, std::optional<double> pwm, const QString &label)
{
    const QColor color = zoneColor(pwm);
    const double span = m_cfg.pwmMax - m_cfg.pwmMin;
    const double frac = !pwm ? 0 : clamp((*pwm - m_cfg.pwmMin) / span, 0, 1);
    const double valueAngle = kGaugeStart + frac * kGaugeSweep;
    const double scaleR = r * 0.88;
    const double lw = kLineW + (m_shadow ? kShadowSpread : 0);

    // Scale circle. The coloured part rides exactly on it — same radius, same
    // width, rounded ends — so the two read as one continuous line with no
    // step where they meet.
    p->setBrush(Qt::NoBrush);
    p->setPen(QPen(ink(kScale), lw, Qt::SolidLine, Qt::RoundCap));
    p->drawPath(arcPath(cx, cy, scaleR, kGaugeStart, kGaugeStart + kGaugeSweep));

    if (pwm && frac > 0.001) {
        p->setPen(QPen(ink(color), lw, Qt::SolidLine, Qt::RoundCap));
        p->drawPath(arcPath(cx, cy, scaleR, kGaugeStart, valueAngle));
    }

    // Fixed tick at zero
    tick(p, cx, cy, kGaugeStart, scaleR, r, 0.34, kTick);
    // Moving tick at the current value
    tick(p, cx, cy, valueAngle, scaleR, r, 0.44, !pwm ? kNoData : color);

    // Motor label in the middle — the arc carries the value
    if (!label.isEmpty()) {
        p->setFont(uiFont(clamp(std::round(r * 0.42), 9, 30)));
        p->setPen(ink(color));
        p->drawText(QRectF(cx - r, cy - r, 2 * r, 2 * r), Qt::AlignCenter, label);
    }
}

/// Radial tick growing inward from the scale circle.
void RotorLoadItem::tick(QPainter *p, double cx, double cy, double angle, double scaleR, double r, double len, const QColor &color)
{
    const double a = angle * M_PI / 180;
    const double dx = std::cos(a), dy = std::sin(a);
    const double inner = scaleR - r * len;
    const double outer = scaleR;
    p->setPen(QPen(ink(color), kLineW + (m_shadow ? kShadowSpread : 0), Qt::SolidLine, Qt::RoundCap));
    p->drawLine(QPointF(cx + dx * inner, cy + dy * inner), QPointF(cx + dx * outer, cy + dy * outer));
}

/// The foil, leading edge towards bearing `le`, turning about (cx, cy).
void RotorLoadItem::drawFoil(QPainter *p, double cx, double cy, double le, double chord, double lw)
{
    const double a = (le - 90) * M_PI / 180;
    const double ux = std::cos(a), uy = std::sin(a);   // along the chord, towards the leading edge
    QPainterPath path;
    // Upper surface leading → trailing edge, lower surface back; cosine
    // spacing puts the points where the nose curves
    for (int i = 0; i <= 2 * kFoilSteps; ++i) {
        const int k = i <= kFoilSteps ? i : 2 * kFoilSteps - i;
        const double side = i <= kFoilSteps ? 1 : -1;
        const double x = (1 - std::cos(M_PI * k / kFoilSteps)) / 2;
        const double s = (kFoilPivot - x) * chord;
        const double hh = side * nacaHalfThickness(x, kFoilThickness) * chord;
        const QPointF pt(cx + ux * s - uy * hh, cy + uy * s + ux * hh);
        if (i == 0) path.moveTo(pt);
        else path.lineTo(pt);
    }
    path.closeSubpath();
    p->setBrush(ink(kFoilFill));
    p->setPen(QPen(ink(kGreen), lw + (m_shadow ? kShadowSpread : 0), Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    p->drawPath(path);
    p->setBrush(Qt::NoBrush);
}

/// One round dial, seen from above with the bow up and angles clockwise, the
/// same convention as the outputs and the wind vane: a scale ring with a tick
/// every 30°, the arc the trailing edge sweeps over the full travel (none for
/// a full turn), the foil turning about its stock, and the apparent wind as a
/// red stroke in from the rim on the side it comes from.
void RotorLoadItem::drawDial(QPainter *p, double cx, double cy, double r, std::optional<double> le,
                             std::optional<std::pair<double, double>> travel, std::optional<double> windDeg)
{
    const double lw = std::max(2.0, r * 0.07);
    const double spread = m_shadow ? kShadowSpread : 0;
    const auto pt = [&](double d, double rr) {
        const double a = (d - 90) * M_PI / 180;
        return QPointF(cx + std::cos(a) * rr, cy + std::sin(a) * rr);
    };
    const auto line = [&](QPointF p1, QPointF p2, double width, const QColor &color) {
        p->setPen(QPen(ink(color), width + spread, Qt::SolidLine, Qt::RoundCap));
        p->drawLine(p1, p2);
    };
    p->setBrush(Qt::NoBrush);
    p->setPen(QPen(ink(kScale), lw + spread, Qt::SolidLine, Qt::RoundCap));
    p->drawEllipse(QPointF(cx, cy), r, r);
    if (travel) {
        p->setPen(QPen(ink(kTravel), lw + spread, Qt::SolidLine, Qt::RoundCap));
        p->drawPath(arcPath(cx, cy, r, travel->first - 90, travel->second - 90));
    }
    for (int d = 0; d < 360; d += 30) {
        const bool major = d % 90 == 0;
        line(pt(d, r * (major ? 0.74 : 0.86)), pt(d, r), major ? lw : lw * 0.6, kTick);
    }

    if (windDeg) line(pt(*windDeg, r * 1.12), pt(*windDeg, r * 0.62), lw, kRed);

    if (le) drawFoil(p, cx, cy, *le, r * kFoilChord, std::max(1.5, lw * 0.5));
    p->setPen(Qt::NoPen);
    p->setBrush(ink(!le ? kNoData : kText));
    p->drawEllipse(QPointF(cx, cy), lw * 0.8, lw * 0.8);
    p->setBrush(Qt::NoBrush);
}

/// Wing (left) and rudder (right), each with its name and commanded angle underneath.
void RotorLoadItem::drawSail(QPainter *p, double w, double h, const SailLayout &lay)
{
    const double fs = clamp(std::round(h * 0.11), 9, 18);
    const double textH = fs * 2.3;
    const double r = std::max(8.0, std::min(w / 4 - 6, (h - textH) / 2 - 6));
    const double cy = (h - textH) / 2;
    const std::int64_t now = wallMs();
    const auto sailDeg = commandedDeg(lay.sail, lay.sailMax);
    const auto rudderDeg = commandedDeg(lay.rudder, m_cfg.rudderMax);
    // The wing's angle is where its leading edge points (into the wind). Steering
    // to starboard swings the rudder's trailing edge to starboard, so its
    // leading edge goes the other way.
    struct Dial {
        double cx;
        QString name;
        std::optional<double> deg, le;
        std::optional<std::pair<double, double>> travel;
        std::optional<double> wind;
    };
    const std::array<Dial, 2> dials{
        Dial{w * 0.25, u"SAIL"_s, sailDeg, sailDeg, travelDeg(lay.sailMax), apparentWindDeg(now)},
        Dial{w * 0.75, u"RUDDER"_s, rudderDeg, rudderDeg ? std::optional<double>(-*rudderDeg) : std::nullopt,
             travelDeg(m_cfg.rudderMax), std::nullopt}};
    const QFont font = uiFont(fs);
    const QFontMetricsF fm(font);
    const auto centred = [&](const QString &text, double x, double y, const QColor &c) {
        p->setFont(font);
        p->setPen(ink(c));
        p->drawText(QPointF(x - fm.horizontalAdvance(text) / 2, y), text);
    };
    for (const Dial &d : dials) {
        drawDial(p, d.cx, cy, r, d.le, d.travel, d.wind);
        const double y = cy + r + fs * 1.15;
        centred(d.name + ' '_L1 + (d.deg ? signedDeg(*d.deg) : u"---"_s), d.cx, y,
                d.deg ? kText : kNoData);
        if (d.wind) centred(u"WIND "_s + signedDeg(*d.wind), d.cx, y + fs * 1.1, kRed);
    }
}

} // namespace corv
