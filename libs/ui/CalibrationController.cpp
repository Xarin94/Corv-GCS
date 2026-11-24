#include "ui/CalibrationController.h"

#include "core/Clock.h"
#include "core/DebugLog.h"
#include "link/MessageBus.h"
#include "ui/DialogHub.h"
#include "vehicle/CommandSender.h"
#include "vehicle/ParamStore.h"
#include "vehicle/VehicleHub.h"

#include <QLocale>
#include <QRegularExpression>
#include <algorithm>
#include <cmath>
#include <iterator>

using namespace Qt::StringLiterals;

namespace corv {

CalibrationController *CalibrationController::s_instance = nullptr;

namespace {

constexpr int kAccelSuccess = 16777215;   // ACCELCAL_VEHICLE_POS_SUCCESS
constexpr int kAccelFailed = 16777216;    // ACCELCAL_VEHICLE_POS_FAILED

QString resultText(int result)
{
    switch (result) {
    case 1: return u"temporarily rejected — try again"_s;
    case 2: return u"denied"_s;
    case 3: return u"not supported by this firmware"_s;
    case 4: return u"failed"_s;
    case 5: return u"still in progress"_s;
    case 6: return u"cancelled"_s;
    default: return u"result %1"_s.arg(result);
    }
}

QString noAnswer(int command)
{
    return u"No answer from the autopilot (No COMMAND_ACK for command %1 (timeout))."_s.arg(command);
}

// Autopilot texts worth repeating next to a calibration (the rest is chatter)
const QRegularExpression &calText()
{
    static const QRegularExpression re(u"cal|trim|baro|gyro|accel|compass|mag|disarm|level|sample|still|place|reboot|offset|fit"_s,
                                       QRegularExpression::CaseInsensitiveOption);
    return re;
}

bool matches(const QString &text, const char *pattern)
{
    return QRegularExpression(QLatin1String(pattern), QRegularExpression::CaseInsensitiveOption).match(text).hasMatch();
}

// ACCELCAL_VEHICLE_POS 1..6
struct PositionInfo {
    const char *title;
    const char *say;
};
constexpr PositionInfo kPositions[6] = {
    {"LEVEL", "LEVEL"},
    {"LEFT SIDE", "on its LEFT side"},
    {"RIGHT SIDE", "on its RIGHT side"},
    {"NOSE DOWN", "NOSE DOWN"},
    {"NOSE UP", "NOSE UP"},
    {"BACK", "on its BACK (upside down)"},
};

// Parameters shown on these pages
struct AccelImu {
    QString id;
    QStringList ofs, scal;
};
const QList<AccelImu> &accelImus()
{
    static const QList<AccelImu> l = [] {
        QList<AccelImu> out;
        for (const QString n : {QString(), u"2"_s, u"3"_s}) {
            AccelImu i;
            i.id = u"INS_ACC%1_ID"_s.arg(n);
            for (const char *a : {"X", "Y", "Z"}) {
                i.ofs << u"INS_ACC%1OFFS_%2"_s.arg(n, QLatin1String(a));
                i.scal << u"INS_ACC%1SCAL_%2"_s.arg(n, QLatin1String(a));
            }
            out << i;
        }
        return out;
    }();
    return l;
}
const QStringList &accelParams()
{
    static const QStringList l = [] {
        QStringList out;
        for (const AccelImu &i : accelImus()) out << i.id << i.ofs << i.scal;
        out << u"AHRS_TRIM_X"_s << u"AHRS_TRIM_Y"_s;
        return out;
    }();
    return l;
}

// Device, orientation, mounting and offsets are stored per detected slot;
// "use for yaw" per priority (COMPASS_PRIOn_ID says which device that is).
struct CompassSlot {
    QString dev, ext, orient;
    QStringList ofs;
};
struct CompassPrio {
    QString id, use;
};
const QList<CompassSlot> &compassSlots()
{
    static const QList<CompassSlot> l = [] {
        QList<CompassSlot> out;
        const QString n[3] = {QString(), u"2"_s, u"3"_s};
        const char *ext[3] = {"COMPASS_EXTERNAL", "COMPASS_EXTERN2", "COMPASS_EXTERN3"};
        for (int i = 0; i < 3; ++i) {
            CompassSlot s{u"COMPASS_DEV_ID%1"_s.arg(n[i]), QLatin1String(ext[i]), u"COMPASS_ORIENT%1"_s.arg(n[i]), {}};
            for (const char *a : {"X", "Y", "Z"}) s.ofs << u"COMPASS_OFS%1_%2"_s.arg(n[i], QLatin1String(a));
            out << s;
        }
        return out;
    }();
    return l;
}
const QList<CompassPrio> &compassPrios()
{
    static const QList<CompassPrio> l = {{u"COMPASS_PRIO1_ID"_s, u"COMPASS_USE"_s},
                                         {u"COMPASS_PRIO2_ID"_s, u"COMPASS_USE2"_s},
                                         {u"COMPASS_PRIO3_ID"_s, u"COMPASS_USE3"_s}};
    return l;
}
const QStringList &compassParams()
{
    static const QStringList l = [] {
        QStringList out;
        for (const CompassSlot &s : compassSlots()) out << s.dev << s.ext << s.orient << s.ofs;
        for (const CompassPrio &p : compassPrios()) out << p.id << p.use;
        out << u"COMPASS_CAL_FIT"_s;
        return out;
    }();
    return l;
}

// AP_HAL::Device bus types and AP_Compass_Backend::DevTypes
QString busName(int t)
{
    switch (t) {
    case 1: return u"I2C"_s;
    case 2: return u"SPI"_s;
    case 3: return u"DroneCAN"_s;
    case 4: return u"SITL"_s;
    case 5: return u"MSP"_s;
    case 6: return u"Serial"_s;
    case 7: return u"WSPI"_s;
    default: return u"bus %1"_s.arg(t);
    }
}
QString compassTypeName(int t)
{
    static const QHash<int, const char *> names = {
        {0x01, "HMC5883 (old)"}, {0x07, "HMC5883"}, {0x02, "LSM303D"}, {0x04, "AK8963"}, {0x05, "BMM150"}, {0x06, "LSM9DS1"},
        {0x08, "LIS3MDL"}, {0x09, "AK09916"}, {0x0A, "IST8310"}, {0x0B, "ICM20948"}, {0x0C, "MMC3416"}, {0x0D, "QMC5883L"},
        {0x0E, "MAG3110"}, {0x0F, "SITL"}, {0x10, "IST8308"}, {0x11, "RM3100"}, {0x12, "RM3100"}, {0x13, "MMC5983"},
        {0x14, "AK09918"}, {0x15, "AK09915"}, {0x16, "QMC5883P"}, {0x17, "BMM350"}, {0x18, "IIS2MDC"}, {0x1A, "AF9838"}};
    const auto hex = [](int n) { return u"0x%1"_s.arg(n, 2, 16, '0'_L1).toUpper().replace("0X"_L1, "0x"_L1); };
    return names.contains(t) ? QLatin1String(names.value(t)) : u"type %1"_s.arg(hex(t));
}
QString hex2(int n) { return u"0x"_s + u"%1"_s.arg(n, 2, 16, '0'_L1).toUpper(); }

// ArduPilot's Rotation enum, in order (rotations.h)
QString rotationName(int v)
{
    static const char *rot[] = {
        "None", "Yaw45", "Yaw90", "Yaw135", "Yaw180", "Yaw225", "Yaw270", "Yaw315", "Roll180", "Roll180Yaw45",
        "Roll180Yaw90", "Roll180Yaw135", "Pitch180", "Roll180Yaw225", "Roll180Yaw270", "Roll180Yaw315", "Roll90",
        "Roll90Yaw45", "Roll90Yaw90", "Roll90Yaw135", "Roll270", "Roll270Yaw45", "Roll270Yaw90", "Roll270Yaw135",
        "Pitch90", "Pitch270", "Pitch180Yaw90", "Pitch180Yaw270", "Roll90Pitch90", "Roll180Pitch90", "Roll270Pitch90",
        "Roll90Pitch180", "Roll270Pitch180", "Roll90Pitch270", "Roll180Pitch270", "Roll270Pitch270",
        "Roll90Pitch180Yaw90", "Roll90Yaw270", "Roll90Pitch68Yaw293", "Pitch315", "Roll90Pitch315", "Pitch7",
        "Roll45", "Roll315"};
    if (v < 0) return u"—"_s;
    if (v >= 100) return v - 100 ? u"Custom %1"_s.arg(v - 100) : u"Custom"_s;
    return v < int(std::size(rot)) ? QLatin1String(rot[v]) : u"#%1"_s.arg(v);
}

// MAG_CAL_STATUS, extended with ArduPilot's CompassCalibrator::Status
QString magStatusName(int s)
{
    static const char *n[] = {"NOT STARTED", "WAITING", "STEP 1", "STEP 2", "SUCCESS", "FAILED",
                              "BAD ORIENTATION", "BAD RADIUS", "BAD OFFSETS", "BAD SCALING", "RESIDUALS HIGH"};
    return s >= 0 && s <= 10 ? QLatin1String(n[s]) : QString::number(s);
}
QString magFailHint(int s)
{
    switch (s) {
    case 5: return u"the samples do not fit a sphere — turn the vehicle more slowly through every attitude, away from metal"_s;
    case 6: return u"the compass orientation does not match the autopilot — check COMPASS_ORIENT"_s;
    case 7: return u"the field strength is out of range — metal or magnets nearby, or the wrong compass setup"_s;
    case 8: return u"the offsets are too large (COMPASS_OFFS_MAX) — interference close to the compass"_s;
    case 9: return u"the scale factors are out of range — interference or a damaged sensor"_s;
    case 10: return u"the fit residuals are too high — try FITNESS Relaxed, or move away from interference"_s;
    default: return {};
    }
}
bool isMagRunning(int s) { return s >= 1 && s <= 3; }
bool isMagFinal(int s) { return s >= 4; }

QString fixed(double v, int d) { return QString::number(v, 'f', d); }

} // namespace

const QStringList &CalibrationController::magColorList()
{
    static const QStringList c = {u"#00d2ff"_s, u"#ffa020"_s, u"#ff4fd8"_s};
    return c;
}

CalibrationController::CalibrationController(MessageBus *bus, CommandSender *cmd, ParamStore *params, VehicleHub *hub, QObject *parent)
    : QObject(parent), m_bus(bus), m_cmd(cmd), m_params(params), m_hub(hub)
{
    s_instance = this;

    connect(m_hub, &VehicleHub::statusText, this, [this](int, const QString &t) {
        const QString text = QString(t).remove(QChar(0)).trimmed();
        if (!text.isEmpty()) onStatusText(text);
    });
    // ACCELCAL_VEHICLE_POS arrives as a COMMAND_LONG from the vehicle
    m_bus->subscribe(MAVLINK_MSG_ID_COMMAND_LONG, this, [this](const MavMessage &mm) {
        mavlink_command_long_t c;
        mavlink_msg_command_long_decode(&mm.msg, &c);
        if (c.command == MAV_CMD_ACCELCAL_VEHICLE_POS) onAccelPosition(int(std::lround(c.param1)));
    });
    // The magnetometers: RAW_IMU carries the first compass by priority, SCALED_IMU2 / 3 the others
    m_bus->subscribe(MAVLINK_MSG_ID_RAW_IMU, this, [this](const MavMessage &mm) {
        mavlink_raw_imu_t d;
        mavlink_msg_raw_imu_decode(&mm.msg, &d);
        onMagSample(0, d.xmag, d.ymag, d.zmag);
    });
    m_bus->subscribe(MAVLINK_MSG_ID_SCALED_IMU2, this, [this](const MavMessage &mm) {
        mavlink_scaled_imu2_t d;
        mavlink_msg_scaled_imu2_decode(&mm.msg, &d);
        onMagSample(1, d.xmag, d.ymag, d.zmag);
    });
    m_bus->subscribe(MAVLINK_MSG_ID_SCALED_IMU3, this, [this](const MavMessage &mm) {
        mavlink_scaled_imu3_t d;
        mavlink_msg_scaled_imu3_decode(&mm.msg, &d);
        onMagSample(2, d.xmag, d.ymag, d.zmag);
    });
    m_bus->subscribe(MAVLINK_MSG_ID_MAG_CAL_PROGRESS, this, [this](const MavMessage &mm) { onMagProgress(mm); });
    m_bus->subscribe(MAVLINK_MSG_ID_MAG_CAL_REPORT, this, [this](const MavMessage &mm) { onMagReport(mm); });

    // Parameter tables follow the store (it already coalesces bursts of PARAM_VALUE)
    connect(m_params, &ParamStore::changed, this, &CalibrationController::rebuildParamViews);

    m_accelTimer.setSingleShot(true);
    connect(&m_accelTimer, &QTimer::timeout, this, [this] {
        if (m_accel.phase == Waiting)
            failAccel(u"The autopilot accepted but never asked for a position (firmware without MAVLink accelerometer calibration?)."_s);
    });
    // With retries on, a failure can be followed by a new attempt: decide once
    // nothing has been running for a moment.
    m_magFinish.setSingleShot(true);
    m_magFinish.setInterval(1500);
    connect(&m_magFinish, &QTimer::timeout, this, [this] {
        if (!m_mag.running) return;
        for (int i : calibratingIds())
            if (!m_mag.progress[std::size_t(i)].has || !isMagFinal(m_mag.progress[std::size_t(i)].status)) return;
        finishMag();
    });
    m_statsTimer.setInterval(250);
    connect(&m_statsTimer, &QTimer::timeout, this, &CalibrationController::updateStats);
    m_fitTimer.setSingleShot(true);
    m_fitTimer.setInterval(400);
    connect(&m_fitTimer, &QTimer::timeout, this, [this] {
        m_samples.refit();
        emit magViewChanged();
    });

    rebuildParamViews();
}

bool CalibrationController::linked() const { return m_hub->snapshot()->connected; }

double CalibrationController::param(const QString &name, bool *has) const
{
    const ParamValue *p = m_params->find(name);
    const bool ok = p && std::isfinite(p->value);
    if (has) *has = ok;
    return ok ? p->value : 0;
}

/// Ask for the parameters a page shows; only the ones not loaded yet, unless forced
void CalibrationController::requestParams(const QStringList &names, bool force)
{
    if (!linked()) return;
    QStringList want;
    for (const QString &n : names)
        if (force || !m_params->find(n)) want << n;
    if (!want.isEmpty()) m_params->enqueueFetch(want);
}

void CalibrationController::onStatusText(const QString &text)
{
    // Six positions: the last texts under the steps, and a fallback for the end
    if (accelActive()) {
        m_accelLog << text;
        while (m_accelLog.size() > 5) m_accelLog.removeFirst();
        if (m_accel.pos > 0 && matches(text, "calibration successful")) finishAccel(true);
        else if (m_accel.pos > 0 && matches(text, "calibration (failed|cancelled)")) finishAccel(false, text);
        else emit accelChanged();
    }
    // One-command calibrations
    if (calText().match(text).hasMatch()) {
        for (auto it = m_oneShotSaid.begin(); it != m_oneShotSaid.end(); ++it) {
            it.value() = text;
            const QString running = m_results.value(it.key()).toMap().value(u"running"_s).toString();
            setResult(it.key(), running + '\n'_L1 + text, u"running"_s, true);
        }
    }
    // "Disarm to allow compass calibration" and the like arrive before the refusal
    if ((m_mag.starting || m_mag.running) && matches(text, "fail|bad|disarm|healthy|enable")) m_mag.said = text;
}

void CalibrationController::rebuildParamViews()
{
    rebuildAccelTable();
    rebuildCompassRows();
    rebuildFitOptions();
    emit paramsChanged();
}

// ════════════════════════════════════════════════════════════════════════════
// ACCELEROMETER
// ════════════════════════════════════════════════════════════════════════════

bool CalibrationController::accelActive() const
{
    return m_accel.phase == Starting || m_accel.phase == Waiting || m_accel.phase == Position || m_accel.phase == Sampling;
}

QStringList CalibrationController::accelSteps() const
{
    const bool active = accelActive();
    QStringList out;
    for (int pos = 1; pos <= 6; ++pos) {
        if (active && pos == m_accel.pos && m_accel.phase == Position) out << u"current"_s;
        else if (active && pos == m_accel.pos && m_accel.phase == Sampling) out << u"sampling"_s;
        else if (m_accel.done.contains(pos) && !(active && pos == m_accel.pos)) out << u"done"_s;
        else out << QString();
    }
    return out;
}

QString CalibrationController::accelButton() const
{
    switch (m_accel.phase) {
    case Position: return u"CONTINUE (%1)"_s.arg(QLatin1String(kPositions[m_accel.pos - 1].title));
    case Sampling: return u"SAMPLING…"_s;
    case Starting:
    case Waiting: return u"STARTING…"_s;
    default: return u"CALIBRATE ACCEL"_s;
    }
}

bool CalibrationController::accelButtonEnabled() const { return !accelActive() || m_accel.phase == Position; }

void CalibrationController::setAccelText(const QString &text)
{
    m_accelText = text;
    m_accelKind = m_accel.phase == Done ? u"ok"_s : m_accel.phase == Failed ? u"error"_s
                  : accelActive() ? u"running"_s : QString();
}

void CalibrationController::renderAccel(const QString *instruction)
{
    if (instruction) setAccelText(*instruction);
    emit accelChanged();
}

void CalibrationController::accelPrimary()
{
    if (m_accel.phase == Position) continueAccel();
    else if (!accelActive()) startAccel();
}

void CalibrationController::startAccel()
{
    if (!linked()) {
        m_accelText = u"Not connected to a vehicle."_s;
        m_accelKind = u"error"_s;
        emit accelChanged();
        return;
    }
    m_accel = Accel{};
    const int run = ++m_accelRun;
    m_accel.phase = Starting;
    m_accelLog.clear();
    const QString t = u"Measuring the gyros first — keep the vehicle still…"_s;
    renderAccel(&t);
    m_cmd->calibrateAccel([this, run](bool got, const CommandSender::Ack &ack) {
        if (run != m_accelRun) return;
        if (!got) return failAccel(noAnswer(MAV_CMD_PREFLIGHT_CALIBRATION));
        if (m_accel.phase != Starting) return;          // the first position came first
        if (ack.result != 0) return failAccel(u"The autopilot refused to start: %1."_s.arg(resultText(ack.result)));
        m_accel.phase = Waiting;
        const QString w = u"Waiting for the autopilot to ask for the first position…"_s;
        renderAccel(&w);
        m_accelTimer.start(8000);
    });
}

void CalibrationController::onAccelPosition(int pos)
{
    const qint64 now = wallMs();
    if (pos == kAccelSuccess || pos == kAccelFailed) {
        // Repeated every second after a calibration ends, until the next one
        // starts: only an end of the run this page has seen counts.
        if (accelActive() && m_accel.pos > 0) finishAccel(pos == kAccelSuccess);
        return;
    }
    if (pos < 1 || pos > 6) return;
    // The repeat of a request our answer crossed on the way
    if (pos == m_accel.confirmedPos && now - m_accel.confirmedAt < 4000) return;
    if (!accelActive()) {
        if (now - m_accelEndedAt < 4000) return;        // a late request from the run that just ended
        // Started elsewhere (another GCS, or before this page was opened): follow it
        m_accel = Accel{};
        ++m_accelRun;
        m_accel.phase = Waiting;
    }
    if (pos == m_accel.pos && m_accel.phase == Position) return;

    m_accelTimer.stop();
    for (int p = 1; p < pos; ++p) m_accel.done.insert(p);
    m_accel.pos = pos;
    m_accel.phase = Position;
    const QString t = u"Place the vehicle %1 and hold it still, then press CONTINUE."_s.arg(QLatin1String(kPositions[pos - 1].say));
    renderAccel(&t);
}

void CalibrationController::continueAccel()
{
    const int pos = m_accel.pos;
    const int run = m_accelRun;
    m_accel.phase = Sampling;
    m_accel.confirmedPos = pos;
    m_accel.confirmedAt = wallMs();
    const QString t = u"Sampling — hold still…"_s;
    renderAccel(&t);
    m_cmd->sendAccelCalPosition(pos, [this, pos, run](bool got, const CommandSender::Ack &ack) {
        if (run != m_accelRun || m_accel.phase != Sampling || m_accel.pos != pos) return;
        const int result = got ? ack.result : -1;
        if (result == 0) return;                         // the next position, or the end, comes as a command
        m_accel.phase = Position;
        m_accel.confirmedPos = 0;
        const QString why = result == -1 ? u"no answer"_s : resultText(result);
        const QString w = u"The autopilot did not take position %1 (%2). Hold it still and press CONTINUE again."_s.arg(pos).arg(why);
        renderAccel(&w);
    });
}

void CalibrationController::finishAccel(bool ok, const QString &why)
{
    if (!accelActive()) return;
    m_accelTimer.stop();
    m_accelEndedAt = wallMs();
    QString t;
    if (ok) {
        for (int p = 1; p <= 6; ++p) m_accel.done.insert(p);
        m_accel.phase = Done;
        t = u"Calibration successful — offsets saved. Reboot the autopilot before arming."_s;
        QTimer::singleShot(1000, this, [this] { requestParams(accelParams(), true); });
    } else {
        m_accel.phase = Failed;
        const QString extra = !why.isEmpty() && !matches(why, "failed") ? u" — "_s + why : QString();
        t = u"Calibration FAILED%1. Hold the vehicle still and square in every position, then start again."_s.arg(extra);
    }
    renderAccel(&t);
}

void CalibrationController::failAccel(const QString &text)
{
    m_accelTimer.stop();
    m_accelEndedAt = wallMs();
    m_accel.phase = Failed;
    renderAccel(&text);
}

void CalibrationController::accelAbort()
{
    if (!accelActive()) return;
    DialogHub::confirm(u"MAVLink cannot cancel an accelerometer calibration: the autopilot keeps waiting for the positions until it is rebooted.\n\nReboot the autopilot now?"_s,
                       [this](bool reboot) {
                           m_accelTimer.stop();
                           m_accel = Accel{};
                           ++m_accelRun;
                           m_accelEndedAt = wallMs();
                           if (reboot) m_cmd->rebootAutopilot();
                           const QString t = reboot ? u"Aborted — autopilot rebooting."_s
                                                    : u"Aborted here — the autopilot is still waiting until it reboots."_s;
                           renderAccel(&t);
                       });
}

void CalibrationController::rebuildAccelTable()
{
    QVariantList rows;
    const auto fmt = [](bool has, double v, int d) { return has ? fixed(v, d) : u"—"_s; };
    int i = 0;
    for (const AccelImu &imu : accelImus()) {
        const int n = i++;
        bool hasId = false;
        const double id = param(imu.id, &hasId);
        if (n > 0 && !id) continue;                       // no second / third IMU
        QStringList ofs, scal;
        bool anyMissing = false, untouched = true;
        for (const QString &p : imu.ofs) {
            bool h = false;
            const double v = param(p, &h);
            anyMissing |= !h;
            untouched &= h && v == 0;
            ofs << fmt(h, v, 3);
        }
        for (const QString &p : imu.scal) {
            bool h = false;
            const double v = param(p, &h);
            untouched &= h && (v == 0 || v == 1);
            scal << fmt(h, v, 3);
        }
        rows << QVariantMap{{u"cells"_s, QStringList{u"IMU %1"_s.arg(n + 1), ofs.join(" / "_L1),
                                                                  scal.join(" / "_L1),
                                                                  anyMissing ? u"not read"_s : untouched ? u"not calibrated"_s : u"calibrated"_s}},
                            {u"warn"_s, untouched}};
    }
    bool hx = false, hy = false;
    const double tx = param(u"AHRS_TRIM_X"_s, &hx), ty = param(u"AHRS_TRIM_Y"_s, &hy);
    const auto deg = [](bool h, double v) { return h ? fixed(v * 180 / M_PI, 2) + u"°"_s : u"—"_s; };
    rows << QVariantMap{{u"cells"_s, QStringList{u"LEVEL"_s, u"roll trim %1 · pitch trim %2"_s.arg(deg(hx, tx), deg(hy, ty))}},
                        {u"span"_s, true}};
    m_accelTable = rows;
}

void CalibrationController::readAccelParams() { requestParams(accelParams(), true); }

void CalibrationController::accelPageShown() { requestParams(accelParams(), false); }

// ── One-command calibrations ─────────────────────────────────────────────────

void CalibrationController::setResult(const QString &id, const QString &text, const QString &kind, bool busy)
{
    QVariantMap r = m_results.value(id).toMap();
    r[u"text"_s] = text;
    r[u"kind"_s] = kind;
    r[u"busy"_s] = busy;
    m_results[id] = r;
    emit resultsChanged();
}

void CalibrationController::runOneShot(const QString &id, const AckCommand &command, const QString &running, const QString &done,
                                       std::function<void()> after)
{
    if (m_oneShotSaid.contains(id)) return;
    if (!linked()) {
        setResult(id, u"Not connected to a vehicle."_s, u"error"_s, false);
        return;
    }
    QVariantMap r = m_results.value(id).toMap();
    r[u"running"_s] = running;
    m_results[id] = r;
    m_oneShotSaid.insert(id, QString());
    setResult(id, running, u"running"_s, true);
    command([this, id, done, after](bool got, int result) {
        const QString said = m_oneShotSaid.take(id);
        if (got && result == 0) {
            static const QRegularExpression chatter(u"^(calibrating|updating)"_s, QRegularExpression::CaseInsensitiveOption);
            setResult(id, !said.isEmpty() && !chatter.match(said).hasMatch() ? done + '\n'_L1 + said : done, u"ok"_s, false);
            if (after) after();
        } else if (got) {
            setResult(id, u"Refused: %1.%2"_s.arg(resultText(result), said.isEmpty() ? QString() : '\n'_L1 + said),
                      u"error"_s, false);
        } else {
            setResult(id, noAnswer(m_lastOneShotCommand.value(id)), u"error"_s, false);
        }
    });
}

namespace {
/// The CommandSender call of a one-command calibration, as runOneShot takes it
template <typename F>
auto ackCommand(F f)
{
    return [f](std::function<void(bool, int)> cb) { f([cb](bool got, const CommandSender::Ack &a) { cb(got, a.result); }); };
}
} // namespace

void CalibrationController::calibrateLevel()
{
    m_lastOneShotCommand[u"level"_s] = MAV_CMD_PREFLIGHT_CALIBRATION;
    runOneShot(u"level"_s, ackCommand([this](CommandSender::AckDone d) { m_cmd->calibrateLevel(std::move(d)); }),
               u"Levelling — keep the vehicle still…"_s, u"Level saved (AHRS_TRIM_X / Y)."_s,
               [this] { QTimer::singleShot(500, this, [this] { requestParams({u"AHRS_TRIM_X"_s, u"AHRS_TRIM_Y"_s}, true); }); });
}

void CalibrationController::calibrateSimple()
{
    m_lastOneShotCommand[u"simple"_s] = MAV_CMD_PREFLIGHT_CALIBRATION;
    runOneShot(u"simple"_s, ackCommand([this](CommandSender::AckDone d) { m_cmd->calibrateAccelSimple(std::move(d)); }),
               u"Calibrating — level and still…"_s, u"Accelerometers calibrated. Reboot the autopilot before arming."_s,
               [this] { QTimer::singleShot(500, this, [this] { requestParams(accelParams(), true); }); });
}

void CalibrationController::calibrateGyro()
{
    m_lastOneShotCommand[u"gyro"_s] = MAV_CMD_PREFLIGHT_CALIBRATION;
    runOneShot(u"gyro"_s, ackCommand([this](CommandSender::AckDone d) { m_cmd->calibrateGyro(std::move(d)); }),
               u"Measuring the gyros — keep the vehicle completely still…"_s, u"Gyro offsets updated."_s);
}

void CalibrationController::calibrateBaro()
{
    m_lastOneShotCommand[u"baro"_s] = MAV_CMD_PREFLIGHT_CALIBRATION;
    runOneShot(u"baro"_s, ackCommand([this](CommandSender::AckDone d) { m_cmd->calibrateBaro(std::move(d)); }),
               u"Sampling ground pressure…"_s, u"Barometer zeroed: the altitude now reads 0 here."_s);
}

void CalibrationController::calibrateFromHeading(const QString &yawText)
{
    bool ok = false;
    const double yaw = QLocale::c().toDouble(yawText.trimmed(), &ok);
    if (!ok || !std::isfinite(yaw) || yaw < 0 || yaw >= 360) {
        setResult(u"fixed"_s, u"Enter a heading from 0 to 359.9°."_s, u"error"_s, false);
        return;
    }
    m_lastOneShotCommand[u"fixed"_s] = MAV_CMD_FIXED_MAG_CAL_YAW;
    runOneShot(u"fixed"_s,
               ackCommand([this, yaw](CommandSender::AckDone d) { m_cmd->magCalFixedYaw(float(yaw), 0, 0, 0, std::move(d)); }),
               u"Calibrating from a %1° heading…"_s.arg(QString::number(yaw)),
               u"Compass offsets set from the heading. Reboot the autopilot to use them."_s,
               [this] { QTimer::singleShot(800, this, [this] { requestParams(compassParams(), true); }); });
}

// ════════════════════════════════════════════════════════════════════════════
// COMPASS
// ════════════════════════════════════════════════════════════════════════════

void CalibrationController::setMagButtons(bool start, bool accept, bool cancel)
{
    m_magButtons = {start, accept, cancel};
}

void CalibrationController::setMagText(const QString &text, const QString &kind)
{
    m_magText = text;
    m_magKind = kind;
    emit magChanged();
}

void CalibrationController::setMagFocus(int i)
{
    if (i == m_samples.focus() || i < 0 || i >= MagSamples::kCompasses) return;
    m_samples.setFocus(i);
    emit magFocusChanged();
    refreshCoverage();
    updateStats();
}

void CalibrationController::setMagAutosave(bool on)
{
    if (on == m_magAutosave) return;
    m_magAutosave = on;
    emit magAutosaveChanged();
}

void CalibrationController::onMagSample(int compass, float x, float y, float z)
{
    if (!(m_mag.running || m_compassShown)) return;
    if (m_samples.add(compass, x, y, z) && !m_fitTimer.isActive()) m_fitTimer.start();
    emit magViewChanged();
}

void CalibrationController::magClear()
{
    m_samples.clear();
    refreshCoverage();
    updateStats();
}

void CalibrationController::compassPageShown(bool shown)
{
    m_compassShown = shown;
    if (!shown) {
        m_statsTimer.stop();
        return;
    }
    requestParams(compassParams(), false);
    m_statsTimer.start();
    updateStats();
    emit magViewChanged();
}

// MAG_CAL_PROGRESS / REPORT ride on the EXTRA3 stream and the magnetometers on
// RAW_SENSORS, both of which can be slow or off: ask for them by message while
// a calibration runs, then hand them back.
void CalibrationController::raiseRates()
{
    const std::pair<uint32_t, double> want[] = {{191, 5}, {192, 2}, {27, 20}, {116, 20}, {129, 20}};
    for (const auto &[id, hz] : want) m_cmd->setMessageInterval(id, hz);
}

void CalibrationController::restoreRates()
{
    if (!linked()) return;
    for (uint32_t id : {191u, 192u, 116u, 129u}) m_cmd->resetMessageInterval(id);
    m_cmd->setMessageInterval(27, 10);   // what requestAllDataStreams asks for
}

void CalibrationController::magStart()
{
    if (!linked()) return setMagText(u"Not connected to a vehicle."_s, u"error"_s);
    const bool autosave = m_magAutosave;
    m_magFinish.stop();
    m_magEndedAt = 0;
    m_mag = Mag{};
    m_mag.starting = true;
    m_samples.clear();
    m_magReboot = false;
    setMagButtons(false, false, false);
    setMagText(u"Starting…"_s, u"running"_s);
    emit magViewChanged();

    raiseRates();
    m_cmd->startMagCal(0, true, autosave, 0, false, [this](bool got, const CommandSender::Ack &ack) {
        m_mag.starting = false;
        if (!got) {
            setMagButtons(true, false, false);
            restoreRates();
            return setMagText(noAnswer(MAV_CMD_DO_START_MAG_CAL), u"error"_s);
        }
        if (ack.result != 0) {
            setMagButtons(true, false, false);
            restoreRates();
            const QString said = !m_mag.said.isEmpty() ? u" %1."_s.arg(m_mag.said)
                                                       : u" Disarm, and check that a compass is enabled."_s;
            return setMagText(u"The autopilot refused to start: %1.%2"_s.arg(resultText(ack.result), said), u"error"_s);
        }
        if (m_mag.running) return;                       // progress came first and took over
        m_mag.running = true;
        setMagButtons(false, false, true);
        setMagText(u"Turn the vehicle slowly through every attitude…"_s, u"running"_s);
    });
}

void CalibrationController::onMagProgress(const MavMessage &mm)
{
    mavlink_mag_cal_progress_t d;
    mavlink_msg_mag_cal_progress_decode(&mm.msg, &d);
    const int id = d.compass_id;
    if (id < 0 || id >= 3) return;
    if (!m_mag.running) {
        if (!isMagRunning(d.cal_status)) return;
        if (wallMs() - m_magEndedAt < 4000) return;   // still under way when the cancel went out
        // Started elsewhere, or before this page was opened: follow it
        m_mag = Mag{};
        m_mag.running = true;
        setMagButtons(false, false, true);
    }
    if (d.cal_mask) m_mag.calMask = d.cal_mask;
    MagProgress &p = m_mag.progress[std::size_t(id)];
    p.has = true;
    p.status = d.cal_status;
    p.pct = d.completion_pct;
    p.attempt = d.attempt;
    std::copy(std::begin(d.completion_mask), std::end(d.completion_mask), p.mask.begin());
    if (id == m_samples.focus()) refreshCoverage();
    if (isMagRunning(d.cal_status)) m_magFinish.stop();   // a retry is under way
    updateMagInstruction();
    emit magChanged();
}

void CalibrationController::onMagReport(const MavMessage &mm)
{
    mavlink_mag_cal_report_t d;
    mavlink_msg_mag_cal_report_decode(&mm.msg, &d);
    const int id = d.compass_id;
    if (id < 0 || id >= 3 || !isMagFinal(d.cal_status)) return;
    MagReport &r = m_mag.reports[std::size_t(id)];
    r.has = true;
    r.status = d.cal_status;
    r.autosaved = d.autosaved != 0;
    r.fitness = d.fitness;
    r.ofsX = d.ofs_x;
    r.ofsY = d.ofs_y;
    r.ofsZ = d.ofs_z;
    r.scale = d.scale_factor;
    r.oldOrientation = d.old_orientation;
    r.newOrientation = d.new_orientation;
    if (d.cal_mask) m_mag.calMask = d.cal_mask;
    MagProgress &p = m_mag.progress[std::size_t(id)];
    const bool had = p.has;
    p.has = true;
    p.status = d.cal_status;
    p.pct = d.cal_status == 4 ? 100 : (had ? p.pct : 0);
    if (!had) p.attempt = 0;
    emit magChanged();
    // The autopilot repeats its reports until the result is accepted: started by the first one
    // (each of them starting it again kept the calibration running for ever, with no ACCEPT)
    if (m_mag.running && !m_magFinish.isActive()) m_magFinish.start();
}

/// The compasses in this calibration: cal_mask, or every one heard from
QList<int> CalibrationController::calibratingIds() const
{
    QList<int> ids;
    for (int i = 0; i < 3; ++i)
        if (m_mag.calMask ? (m_mag.calMask & (1 << i)) != 0 : m_mag.progress[std::size_t(i)].has) ids << i;
    return ids;
}

void CalibrationController::finishMag()
{
    m_mag.running = false;
    restoreRates();
    refreshCoverage();
    const QList<int> ids = calibratingIds();
    QList<int> ok, bad, unsaved;
    for (int i : ids) {
        const auto &p = m_mag.progress[std::size_t(i)];
        if (p.has && p.status == 4) {
            ok << i;
            if (!m_mag.reports[std::size_t(i)].autosaved) unsaved << i;
        } else {
            bad << i;
        }
    }
    setMagButtons(true, !unsaved.isEmpty(), false);
    if (!bad.isEmpty()) {
        QStringList why;
        for (int i : bad) {
            const auto &p = m_mag.progress[std::size_t(i)];
            QString hint = p.has ? magFailHint(p.status) : QString();
            if (hint.isEmpty()) hint = p.has ? magStatusName(p.status) : u"failed"_s;
            why << u"MAG %1: %2"_s.arg(i + 1).arg(hint);
        }
        setMagText(u"%1. Press START to try again%2."_s
                       .arg(why.join(". "_L1), ok.isEmpty() ? QString() : u", or ACCEPT to keep the compasses that passed"_s),
                   u"error"_s);
    } else if (!unsaved.isEmpty()) {
        setMagText(u"Good result. Press ACCEPT to save it."_s, u"ok"_s);
    } else {
        m_magReboot = true;
        setMagText(u"Calibration saved. Reboot the autopilot to use the new offsets."_s, u"ok"_s);
    }
    QTimer::singleShot(1500, this, [this] { requestParams(compassParams(), true); });
}

void CalibrationController::magAccept()
{
    int mask = 0;
    for (int i : calibratingIds())
        if (m_mag.progress[std::size_t(i)].has && m_mag.progress[std::size_t(i)].status == 4) mask |= 1 << i;
    if (!mask) return;
    m_cmd->acceptMagCal(mask, [this](bool got, const CommandSender::Ack &ack) {
        if (!got) return setMagText(noAnswer(MAV_CMD_DO_ACCEPT_MAG_CAL), u"error"_s);
        if (ack.result != 0) return setMagText(u"ACCEPT refused: %1."_s.arg(resultText(ack.result)), u"error"_s);
        setMagButtons(true, false, false);
        m_magReboot = true;
        setMagText(u"Calibration saved. Reboot the autopilot to use the new offsets."_s, u"ok"_s);
        QTimer::singleShot(800, this, [this] { requestParams(compassParams(), true); });
    });
}

void CalibrationController::magCancel()
{
    // Stop here regardless of the answer
    m_cmd->cancelMagCal(0, [this](bool, const CommandSender::Ack &) {
        m_magFinish.stop();
        m_magEndedAt = wallMs();
        m_mag.running = false;
        restoreRates();
        refreshCoverage();
        setMagButtons(true, false, false);
        setMagText(u"Cancelled. Nothing was saved."_s, QString());
    });
}

void CalibrationController::magReboot()
{
    DialogHub::confirm(u"Reboot the autopilot now?"_s, [this](bool ok) {
        if (!ok) return;
        m_cmd->rebootAutopilot();
        m_magReboot = false;
        setMagText(u"Rebooting… reconnects by itself on most links."_s, QString());
    });
}

void CalibrationController::refreshCoverage()
{
    const auto &p = m_mag.progress[std::size_t(m_samples.focus())];
    m_samples.setCoverage(m_mag.running && p.has && isMagRunning(p.status) ? std::optional<geogrid::Mask>(p.mask) : std::nullopt);
    emit magViewChanged();
}

void CalibrationController::updateMagInstruction()
{
    if (!m_mag.running) return;
    const MagProgress *p = &m_mag.progress[std::size_t(m_samples.focus())];
    if (!p->has) {
        p = nullptr;
        for (const auto &q : m_mag.progress)
            if (q.has) {
                p = &q;
                break;
            }
    }
    if (!p) return;
    if (p->status == 1) return setMagText(u"Waiting to start — keep the vehicle ready…"_s, u"running"_s);
    const int covered = m_samples.covered();
    const QString step = p->status == 3 ? u"Step 2 of 2: once more, every attitude. "_s : QString();
    const QString sphere = covered < 0 ? QString() : u"%1/80 directions covered. "_s.arg(covered);
    setMagText(step + sphere + u"Turn the vehicle until the white marker reaches the orange patch."_s, u"running"_s);
}

void CalibrationController::updateStats()
{
    if (!m_compassShown) return;
    const int focus = m_samples.focus();
    const int pts = m_samples.count(focus);
    QStringList parts{u"%1 pts"_s.arg(pts)};
    if (const auto f = m_samples.field(); f && *f > 0)
        parts << u"|B| %1 mG%2"_s.arg(std::lround(*f)).arg(m_samples.fitted() ? QString() : u" (no fit yet)"_s);
    if (m_samples.covered() >= 0) parts << u"%1/80"_s.arg(m_samples.covered());
    const QString s = pts ? parts.join(u" · "_s) : u"No samples"_s;
    if (s != m_magStats) {
        m_magStats = s;
        emit statsChanged();
    }
    updateMagInstruction();
}

QVariantList CalibrationController::magBars() const
{
    const QList<int> ids = calibratingIds();
    QVariantList out;
    for (int i = 0; i < 3; ++i) {
        const auto &p = m_mag.progress[std::size_t(i)];
        QString state;
        if (!p.has && !ids.contains(i)) state = u"idle"_s;
        else if (p.has && p.status == 4) state = u"ok"_s;
        else if (p.has && p.status >= 5) state = u"bad"_s;
        QString text = u"—"_s;
        if (p.has)
            text = u"%1% · %2%3"_s.arg(std::lround(p.pct)).arg(magStatusName(p.status),
                                                                         p.attempt > 1 ? u" · try %1"_s.arg(p.attempt) : QString());
        out << QVariantMap{{u"name"_s, u"MAG %1"_s.arg(i + 1)},
                           {u"color"_s, magColorList()[i]},
                           {u"pct"_s, p.has ? std::clamp(p.pct, 0.0, 100.0) : 0.0},
                           {u"state"_s, state},
                           {u"text"_s, text}};
    }
    return out;
}

QVariantList CalibrationController::magResults() const
{
    QVariantList out;
    for (int i = 0; i < 3; ++i) {
        const MagReport &r = m_mag.reports[std::size_t(i)];
        if (!r.has) continue;
        const bool ok = r.status == 4;
        const QString orient = r.newOrientation >= 0 && r.newOrientation != r.oldOrientation
                                   ? u"%1 → %2"_s.arg(rotationName(r.oldOrientation), rotationName(r.newOrientation))
                                   : rotationName(r.oldOrientation);
        const auto rnd = [](float v) { return std::isfinite(v) ? QString::number(std::lround(v)) : u"—"_s; };
        out << QVariantMap{{u"name"_s, u"MAG %1"_s.arg(i + 1)},
                           {u"color"_s, magColorList()[i]},
                           {u"ok"_s, ok},
                           {u"cells"_s, QStringList{magStatusName(r.status),
                                                                 std::isfinite(r.fitness) ? fixed(r.fitness, 1) : u"—"_s,
                                                                 QStringList{rnd(r.ofsX), rnd(r.ofsY), rnd(r.ofsZ)}.join(" / "_L1),
                                                                 std::isfinite(r.scale) && r.scale != 0 ? fixed(r.scale, 3) : u"—"_s,
                                                                 orient,
                                                                 r.autosaved ? u"saved"_s : ok ? u"not saved"_s : u"—"_s}}};
    }
    return out;
}

void CalibrationController::rebuildCompassRows()
{
    struct Slot {
        bool hasDev = false, hasExt = false, hasOrient = false;
        double dev = 0, ext = 0, orient = 0;
        QStringList ofs;
    };
    QList<Slot> slotList;
    for (const CompassSlot &s : compassSlots()) {
        Slot o;
        o.dev = param(s.dev, &o.hasDev);
        o.ext = param(s.ext, &o.hasExt);
        o.orient = param(s.orient, &o.hasOrient);
        for (const QString &p : s.ofs) {
            bool h = false;
            const double v = param(p, &h);
            o.ofs << (h ? QString::number(std::lround(v)) : u"—"_s);
        }
        slotList << o;
    }
    bool havePrio = false;
    for (const CompassPrio &p : compassPrios()) havePrio |= m_params->find(p.id) != nullptr;

    QVariantList rows;
    // Rows in priority order, as the autopilot uses them (older firmware: slot order)
    for (int i = 0; i < compassPrios().size(); ++i) {
        const CompassPrio &p = compassPrios()[i];
        const int devId = int(havePrio ? param(p.id) : slotList[i].dev);
        if (!devId) continue;
        const Slot *slot = nullptr;
        if (havePrio) {
            for (const Slot &s : slotList)
                if (s.dev && int(s.dev) == devId) {
                    slot = &s;
                    break;
                }
        } else {
            slot = &slotList[i];
        }
        bool hasUse = false;
        const double use = param(p.use, &hasUse);
        const int busType = devId & 0x7, bus = (devId >> 3) & 0x1f, address = (devId >> 8) & 0xff, type = (devId >> 16) & 0xff;
        QString where = busName(busType) + QString::number(bus);
        if (busType == 1) where += ' '_L1 + hex2(address);
        else if (busType == 3) where += u" node %1"_s.arg(address);
        else if (busType == 4) where = u"SITL"_s;
        rows << QVariantMap{{u"prio"_s, i + 1},
                            {u"device"_s, compassTypeName(type)},
                            {u"missing"_s, havePrio && !slot},
                            {u"bus"_s, where},
                            {u"use"_s, hasUse && use != 0},
                            {u"useKnown"_s, hasUse},
                            {u"useParam"_s, p.use},
                            {u"mount"_s, !slot || !slot->hasExt ? u"—"_s : slot->ext ? u"external"_s : u"internal"_s},
                            {u"orientation"_s, slot && slot->hasOrient ? rotationName(int(slot->orient)) : u"—"_s},
                            {u"offsets"_s, slot ? slot->ofs.join(" / "_L1) : u"—"_s}};
    }
    m_compassRows = rows;
    m_compassNote = rows.isEmpty() ? (linked() ? u"No compass parameters yet — READ FROM VEHICLE."_s : u"Not connected."_s)
                                   : QString();
}

void CalibrationController::rebuildFitOptions()
{
    QVariantList opts{QVariantMap{{u"value"_s, u"4"_s}, {u"label"_s, u"Very strict"_s}},
                      QVariantMap{{u"value"_s, u"8"_s}, {u"label"_s, u"Strict"_s}},
                      QVariantMap{{u"value"_s, u"16"_s}, {u"label"_s, u"Default"_s}},
                      QVariantMap{{u"value"_s, u"32"_s}, {u"label"_s, u"Relaxed"_s}}};
    bool has = false;
    const double v = param(u"COMPASS_CAL_FIT"_s, &has);
    if (has) {
        const QString s = QString::number(v);
        // A value off the list gets an entry of its own rather than being misread
        if (s != "4"_L1 && s != "8"_L1 && s != "16"_L1 && s != "32"_L1)
            opts << QVariantMap{{u"value"_s, s}, {u"label"_s, u"Custom (%1)"_s.arg(s)}};
        m_fitValue = s;
    }
    m_fitOptions = opts;
}

void CalibrationController::setMagFit(const QString &value)
{
    if (!linked()) return setMagText(u"Not connected to a vehicle."_s, u"error"_s);
    const double v = value.toDouble();
    m_fitValue = value;
    emit paramsChanged();
    m_params->write(u"COMPASS_CAL_FIT"_s, v, [this, value](bool ok, double) {
        if (ok) setMagText(u"COMPASS_CAL_FIT set to %1."_s.arg(value), QString());
        else setMagText(u"Could not set COMPASS_CAL_FIT: no confirmation from the autopilot"_s, u"error"_s);
        rebuildParamViews();
    });
}

void CalibrationController::setCompassUse(const QString &param, bool on)
{
    m_params->write(param, on ? 1 : 0, [this, param](bool ok, double) {
        if (!ok) DialogHub::toast(u"Could not set %1: no confirmation from the autopilot"_s.arg(param));
        rebuildParamViews();   // the checkbox goes back to what the vehicle has
    });
}

void CalibrationController::readCompassParams() { requestParams(compassParams(), true); }

} // namespace corv
