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

namespace corv {

CalibrationController *CalibrationController::s_instance = nullptr;

namespace {

constexpr int kAccelSuccess = 16777215;   // ACCELCAL_VEHICLE_POS_SUCCESS
constexpr int kAccelFailed = 16777216;    // ACCELCAL_VEHICLE_POS_FAILED

QString resultText(int result)
{
    switch (result) {
    case 1: return QStringLiteral("temporarily rejected — try again");
    case 2: return QStringLiteral("denied");
    case 3: return QStringLiteral("not supported by this firmware");
    case 4: return QStringLiteral("failed");
    case 5: return QStringLiteral("still in progress");
    case 6: return QStringLiteral("cancelled");
    default: return QStringLiteral("result %1").arg(result);
    }
}

QString noAnswer(int command)
{
    return QStringLiteral("No answer from the autopilot (No COMMAND_ACK for command %1 (timeout)).").arg(command);
}

// Autopilot texts worth repeating next to a calibration (the rest is chatter)
const QRegularExpression &calText()
{
    static const QRegularExpression re(QStringLiteral("cal|trim|baro|gyro|accel|compass|mag|disarm|level|sample|still|place|reboot|offset|fit"),
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
        for (const QString n : {QString(), QStringLiteral("2"), QStringLiteral("3")}) {
            AccelImu i;
            i.id = QStringLiteral("INS_ACC%1_ID").arg(n);
            for (const char *a : {"X", "Y", "Z"}) {
                i.ofs << QStringLiteral("INS_ACC%1OFFS_%2").arg(n, QLatin1String(a));
                i.scal << QStringLiteral("INS_ACC%1SCAL_%2").arg(n, QLatin1String(a));
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
        out << QStringLiteral("AHRS_TRIM_X") << QStringLiteral("AHRS_TRIM_Y");
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
        const QString n[3] = {QString(), QStringLiteral("2"), QStringLiteral("3")};
        const char *ext[3] = {"COMPASS_EXTERNAL", "COMPASS_EXTERN2", "COMPASS_EXTERN3"};
        for (int i = 0; i < 3; ++i) {
            CompassSlot s{QStringLiteral("COMPASS_DEV_ID%1").arg(n[i]), QLatin1String(ext[i]), QStringLiteral("COMPASS_ORIENT%1").arg(n[i]), {}};
            for (const char *a : {"X", "Y", "Z"}) s.ofs << QStringLiteral("COMPASS_OFS%1_%2").arg(n[i], QLatin1String(a));
            out << s;
        }
        return out;
    }();
    return l;
}
const QList<CompassPrio> &compassPrios()
{
    static const QList<CompassPrio> l = {{QStringLiteral("COMPASS_PRIO1_ID"), QStringLiteral("COMPASS_USE")},
                                         {QStringLiteral("COMPASS_PRIO2_ID"), QStringLiteral("COMPASS_USE2")},
                                         {QStringLiteral("COMPASS_PRIO3_ID"), QStringLiteral("COMPASS_USE3")}};
    return l;
}
const QStringList &compassParams()
{
    static const QStringList l = [] {
        QStringList out;
        for (const CompassSlot &s : compassSlots()) out << s.dev << s.ext << s.orient << s.ofs;
        for (const CompassPrio &p : compassPrios()) out << p.id << p.use;
        out << QStringLiteral("COMPASS_CAL_FIT");
        return out;
    }();
    return l;
}

// AP_HAL::Device bus types and AP_Compass_Backend::DevTypes
QString busName(int t)
{
    switch (t) {
    case 1: return QStringLiteral("I2C");
    case 2: return QStringLiteral("SPI");
    case 3: return QStringLiteral("DroneCAN");
    case 4: return QStringLiteral("SITL");
    case 5: return QStringLiteral("MSP");
    case 6: return QStringLiteral("Serial");
    case 7: return QStringLiteral("WSPI");
    default: return QStringLiteral("bus %1").arg(t);
    }
}
QString compassTypeName(int t)
{
    static const QHash<int, const char *> names = {
        {0x01, "HMC5883 (old)"}, {0x07, "HMC5883"}, {0x02, "LSM303D"}, {0x04, "AK8963"}, {0x05, "BMM150"}, {0x06, "LSM9DS1"},
        {0x08, "LIS3MDL"}, {0x09, "AK09916"}, {0x0A, "IST8310"}, {0x0B, "ICM20948"}, {0x0C, "MMC3416"}, {0x0D, "QMC5883L"},
        {0x0E, "MAG3110"}, {0x0F, "SITL"}, {0x10, "IST8308"}, {0x11, "RM3100"}, {0x12, "RM3100"}, {0x13, "MMC5983"},
        {0x14, "AK09918"}, {0x15, "AK09915"}, {0x16, "QMC5883P"}, {0x17, "BMM350"}, {0x18, "IIS2MDC"}, {0x1A, "AF9838"}};
    const auto hex = [](int n) { return QStringLiteral("0x%1").arg(n, 2, 16, QLatin1Char('0')).toUpper().replace(QLatin1String("0X"), QLatin1String("0x")); };
    return names.contains(t) ? QLatin1String(names.value(t)) : QStringLiteral("type %1").arg(hex(t));
}
QString hex2(int n) { return QStringLiteral("0x") + QStringLiteral("%1").arg(n, 2, 16, QLatin1Char('0')).toUpper(); }

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
    if (v < 0) return QStringLiteral("—");
    if (v >= 100) return v - 100 ? QStringLiteral("Custom %1").arg(v - 100) : QStringLiteral("Custom");
    return v < int(std::size(rot)) ? QLatin1String(rot[v]) : QStringLiteral("#%1").arg(v);
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
    case 5: return QStringLiteral("the samples do not fit a sphere — turn the vehicle more slowly through every attitude, away from metal");
    case 6: return QStringLiteral("the compass orientation does not match the autopilot — check COMPASS_ORIENT");
    case 7: return QStringLiteral("the field strength is out of range — metal or magnets nearby, or the wrong compass setup");
    case 8: return QStringLiteral("the offsets are too large (COMPASS_OFFS_MAX) — interference close to the compass");
    case 9: return QStringLiteral("the scale factors are out of range — interference or a damaged sensor");
    case 10: return QStringLiteral("the fit residuals are too high — try FITNESS Relaxed, or move away from interference");
    default: return {};
    }
}
bool isMagRunning(int s) { return s >= 1 && s <= 3; }
bool isMagFinal(int s) { return s >= 4; }

QString fixed(double v, int d) { return QString::number(v, 'f', d); }

} // namespace

const QStringList &CalibrationController::magColorList()
{
    static const QStringList c = {QStringLiteral("#00d2ff"), QStringLiteral("#ffa020"), QStringLiteral("#ff4fd8")};
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
            failAccel(QStringLiteral("The autopilot accepted but never asked for a position (firmware without MAVLink accelerometer calibration?)."));
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
            const QString running = m_results.value(it.key()).toMap().value(QStringLiteral("running")).toString();
            setResult(it.key(), running + QLatin1Char('\n') + text, QStringLiteral("running"), true);
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
        if (active && pos == m_accel.pos && m_accel.phase == Position) out << QStringLiteral("current");
        else if (active && pos == m_accel.pos && m_accel.phase == Sampling) out << QStringLiteral("sampling");
        else if (m_accel.done.contains(pos) && !(active && pos == m_accel.pos)) out << QStringLiteral("done");
        else out << QString();
    }
    return out;
}

QString CalibrationController::accelButton() const
{
    switch (m_accel.phase) {
    case Position: return QStringLiteral("CONTINUE (%1)").arg(QLatin1String(kPositions[m_accel.pos - 1].title));
    case Sampling: return QStringLiteral("SAMPLING…");
    case Starting:
    case Waiting: return QStringLiteral("STARTING…");
    default: return QStringLiteral("CALIBRATE ACCEL");
    }
}

bool CalibrationController::accelButtonEnabled() const { return !accelActive() || m_accel.phase == Position; }

void CalibrationController::setAccelText(const QString &text)
{
    m_accelText = text;
    m_accelKind = m_accel.phase == Done ? QStringLiteral("ok") : m_accel.phase == Failed ? QStringLiteral("error")
                  : accelActive() ? QStringLiteral("running") : QString();
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
        m_accelText = QStringLiteral("Not connected to a vehicle.");
        m_accelKind = QStringLiteral("error");
        emit accelChanged();
        return;
    }
    m_accel = Accel{};
    const int run = ++m_accelRun;
    m_accel.phase = Starting;
    m_accelLog.clear();
    const QString t = QStringLiteral("Measuring the gyros first — keep the vehicle still…");
    renderAccel(&t);
    m_cmd->calibrateAccel([this, run](bool got, const CommandSender::Ack &ack) {
        if (run != m_accelRun) return;
        if (!got) return failAccel(noAnswer(MAV_CMD_PREFLIGHT_CALIBRATION));
        if (m_accel.phase != Starting) return;          // the first position came first
        if (ack.result != 0) return failAccel(QStringLiteral("The autopilot refused to start: %1.").arg(resultText(ack.result)));
        m_accel.phase = Waiting;
        const QString w = QStringLiteral("Waiting for the autopilot to ask for the first position…");
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
    const QString t = QStringLiteral("Place the vehicle %1 and hold it still, then press CONTINUE.").arg(QLatin1String(kPositions[pos - 1].say));
    renderAccel(&t);
}

void CalibrationController::continueAccel()
{
    const int pos = m_accel.pos;
    const int run = m_accelRun;
    m_accel.phase = Sampling;
    m_accel.confirmedPos = pos;
    m_accel.confirmedAt = wallMs();
    const QString t = QStringLiteral("Sampling — hold still…");
    renderAccel(&t);
    m_cmd->sendAccelCalPosition(pos, [this, pos, run](bool got, const CommandSender::Ack &ack) {
        if (run != m_accelRun || m_accel.phase != Sampling || m_accel.pos != pos) return;
        const int result = got ? ack.result : -1;
        if (result == 0) return;                         // the next position, or the end, comes as a command
        m_accel.phase = Position;
        m_accel.confirmedPos = 0;
        const QString why = result == -1 ? QStringLiteral("no answer") : resultText(result);
        const QString w = QStringLiteral("The autopilot did not take position %1 (%2). Hold it still and press CONTINUE again.").arg(pos).arg(why);
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
        t = QStringLiteral("Calibration successful — offsets saved. Reboot the autopilot before arming.");
        QTimer::singleShot(1000, this, [this] { requestParams(accelParams(), true); });
    } else {
        m_accel.phase = Failed;
        const QString extra = !why.isEmpty() && !matches(why, "failed") ? QStringLiteral(" — ") + why : QString();
        t = QStringLiteral("Calibration FAILED%1. Hold the vehicle still and square in every position, then start again.").arg(extra);
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
    DialogHub::confirm(QStringLiteral("MAVLink cannot cancel an accelerometer calibration: the autopilot keeps waiting for the positions until it is rebooted.\n\nReboot the autopilot now?"),
                       [this](bool reboot) {
                           m_accelTimer.stop();
                           m_accel = Accel{};
                           ++m_accelRun;
                           m_accelEndedAt = wallMs();
                           if (reboot) m_cmd->rebootAutopilot();
                           const QString t = reboot ? QStringLiteral("Aborted — autopilot rebooting.")
                                                    : QStringLiteral("Aborted here — the autopilot is still waiting until it reboots.");
                           renderAccel(&t);
                       });
}

void CalibrationController::rebuildAccelTable()
{
    QVariantList rows;
    const auto fmt = [](bool has, double v, int d) { return has ? fixed(v, d) : QStringLiteral("—"); };
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
        rows << QVariantMap{{QStringLiteral("cells"), QStringList{QStringLiteral("IMU %1").arg(n + 1), ofs.join(QLatin1String(" / ")),
                                                                  scal.join(QLatin1String(" / ")),
                                                                  anyMissing ? QStringLiteral("not read") : untouched ? QStringLiteral("not calibrated") : QStringLiteral("calibrated")}},
                            {QStringLiteral("warn"), untouched}};
    }
    bool hx = false, hy = false;
    const double tx = param(QStringLiteral("AHRS_TRIM_X"), &hx), ty = param(QStringLiteral("AHRS_TRIM_Y"), &hy);
    const auto deg = [](bool h, double v) { return h ? fixed(v * 180 / M_PI, 2) + QStringLiteral("°") : QStringLiteral("—"); };
    rows << QVariantMap{{QStringLiteral("cells"), QStringList{QStringLiteral("LEVEL"), QStringLiteral("roll trim %1 · pitch trim %2").arg(deg(hx, tx), deg(hy, ty))}},
                        {QStringLiteral("span"), true}};
    m_accelTable = rows;
}

void CalibrationController::readAccelParams() { requestParams(accelParams(), true); }

void CalibrationController::accelPageShown() { requestParams(accelParams(), false); }

// ── One-command calibrations ─────────────────────────────────────────────────

void CalibrationController::setResult(const QString &id, const QString &text, const QString &kind, bool busy)
{
    QVariantMap r = m_results.value(id).toMap();
    r[QStringLiteral("text")] = text;
    r[QStringLiteral("kind")] = kind;
    r[QStringLiteral("busy")] = busy;
    m_results[id] = r;
    emit resultsChanged();
}

void CalibrationController::runOneShot(const QString &id, const AckCommand &command, const QString &running, const QString &done,
                                       std::function<void()> after)
{
    if (m_oneShotSaid.contains(id)) return;
    if (!linked()) {
        setResult(id, QStringLiteral("Not connected to a vehicle."), QStringLiteral("error"), false);
        return;
    }
    QVariantMap r = m_results.value(id).toMap();
    r[QStringLiteral("running")] = running;
    m_results[id] = r;
    m_oneShotSaid.insert(id, QString());
    setResult(id, running, QStringLiteral("running"), true);
    command([this, id, done, after](bool got, int result) {
        const QString said = m_oneShotSaid.take(id);
        if (got && result == 0) {
            static const QRegularExpression chatter(QStringLiteral("^(calibrating|updating)"), QRegularExpression::CaseInsensitiveOption);
            setResult(id, !said.isEmpty() && !chatter.match(said).hasMatch() ? done + QLatin1Char('\n') + said : done, QStringLiteral("ok"), false);
            if (after) after();
        } else if (got) {
            setResult(id, QStringLiteral("Refused: %1.%2").arg(resultText(result), said.isEmpty() ? QString() : QLatin1Char('\n') + said),
                      QStringLiteral("error"), false);
        } else {
            setResult(id, noAnswer(m_lastOneShotCommand.value(id)), QStringLiteral("error"), false);
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
    m_lastOneShotCommand[QStringLiteral("level")] = MAV_CMD_PREFLIGHT_CALIBRATION;
    runOneShot(QStringLiteral("level"), ackCommand([this](CommandSender::AckDone d) { m_cmd->calibrateLevel(std::move(d)); }),
               QStringLiteral("Levelling — keep the vehicle still…"), QStringLiteral("Level saved (AHRS_TRIM_X / Y)."),
               [this] { QTimer::singleShot(500, this, [this] { requestParams({QStringLiteral("AHRS_TRIM_X"), QStringLiteral("AHRS_TRIM_Y")}, true); }); });
}

void CalibrationController::calibrateSimple()
{
    m_lastOneShotCommand[QStringLiteral("simple")] = MAV_CMD_PREFLIGHT_CALIBRATION;
    runOneShot(QStringLiteral("simple"), ackCommand([this](CommandSender::AckDone d) { m_cmd->calibrateAccelSimple(std::move(d)); }),
               QStringLiteral("Calibrating — level and still…"), QStringLiteral("Accelerometers calibrated. Reboot the autopilot before arming."),
               [this] { QTimer::singleShot(500, this, [this] { requestParams(accelParams(), true); }); });
}

void CalibrationController::calibrateGyro()
{
    m_lastOneShotCommand[QStringLiteral("gyro")] = MAV_CMD_PREFLIGHT_CALIBRATION;
    runOneShot(QStringLiteral("gyro"), ackCommand([this](CommandSender::AckDone d) { m_cmd->calibrateGyro(std::move(d)); }),
               QStringLiteral("Measuring the gyros — keep the vehicle completely still…"), QStringLiteral("Gyro offsets updated."));
}

void CalibrationController::calibrateBaro()
{
    m_lastOneShotCommand[QStringLiteral("baro")] = MAV_CMD_PREFLIGHT_CALIBRATION;
    runOneShot(QStringLiteral("baro"), ackCommand([this](CommandSender::AckDone d) { m_cmd->calibrateBaro(std::move(d)); }),
               QStringLiteral("Sampling ground pressure…"), QStringLiteral("Barometer zeroed: the altitude now reads 0 here."));
}

void CalibrationController::calibrateFromHeading(const QString &yawText)
{
    bool ok = false;
    const double yaw = QLocale::c().toDouble(yawText.trimmed(), &ok);
    if (!ok || !std::isfinite(yaw) || yaw < 0 || yaw >= 360) {
        setResult(QStringLiteral("fixed"), QStringLiteral("Enter a heading from 0 to 359.9°."), QStringLiteral("error"), false);
        return;
    }
    m_lastOneShotCommand[QStringLiteral("fixed")] = MAV_CMD_FIXED_MAG_CAL_YAW;
    runOneShot(QStringLiteral("fixed"),
               ackCommand([this, yaw](CommandSender::AckDone d) { m_cmd->magCalFixedYaw(float(yaw), 0, 0, 0, std::move(d)); }),
               QStringLiteral("Calibrating from a %1° heading…").arg(QString::number(yaw)),
               QStringLiteral("Compass offsets set from the heading. Reboot the autopilot to use them."),
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
    if (!linked()) return setMagText(QStringLiteral("Not connected to a vehicle."), QStringLiteral("error"));
    const bool autosave = m_magAutosave;
    m_magFinish.stop();
    m_mag = Mag{};
    m_mag.starting = true;
    m_samples.clear();
    m_magReboot = false;
    setMagButtons(false, false, false);
    setMagText(QStringLiteral("Starting…"), QStringLiteral("running"));
    emit magViewChanged();

    raiseRates();
    m_cmd->startMagCal(0, true, autosave, 0, false, [this](bool got, const CommandSender::Ack &ack) {
        m_mag.starting = false;
        if (!got) {
            setMagButtons(true, false, false);
            restoreRates();
            return setMagText(noAnswer(MAV_CMD_DO_START_MAG_CAL), QStringLiteral("error"));
        }
        if (ack.result != 0) {
            setMagButtons(true, false, false);
            restoreRates();
            const QString said = !m_mag.said.isEmpty() ? QStringLiteral(" %1.").arg(m_mag.said)
                                                       : QStringLiteral(" Disarm, and check that a compass is enabled.");
            return setMagText(QStringLiteral("The autopilot refused to start: %1.%2").arg(resultText(ack.result), said), QStringLiteral("error"));
        }
        if (m_mag.running) return;                       // progress came first and took over
        m_mag.running = true;
        setMagButtons(false, false, true);
        setMagText(QStringLiteral("Turn the vehicle slowly through every attitude…"), QStringLiteral("running"));
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
    if (m_mag.running) m_magFinish.start();
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
            if (hint.isEmpty()) hint = p.has ? magStatusName(p.status) : QStringLiteral("failed");
            why << QStringLiteral("MAG %1: %2").arg(i + 1).arg(hint);
        }
        setMagText(QStringLiteral("%1. Press START to try again%2.")
                       .arg(why.join(QLatin1String(". ")), ok.isEmpty() ? QString() : QStringLiteral(", or ACCEPT to keep the compasses that passed")),
                   QStringLiteral("error"));
    } else if (!unsaved.isEmpty()) {
        setMagText(QStringLiteral("Good result. Press ACCEPT to save it."), QStringLiteral("ok"));
    } else {
        m_magReboot = true;
        setMagText(QStringLiteral("Calibration saved. Reboot the autopilot to use the new offsets."), QStringLiteral("ok"));
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
        if (!got) return setMagText(noAnswer(MAV_CMD_DO_ACCEPT_MAG_CAL), QStringLiteral("error"));
        if (ack.result != 0) return setMagText(QStringLiteral("ACCEPT refused: %1.").arg(resultText(ack.result)), QStringLiteral("error"));
        setMagButtons(true, false, false);
        m_magReboot = true;
        setMagText(QStringLiteral("Calibration saved. Reboot the autopilot to use the new offsets."), QStringLiteral("ok"));
        QTimer::singleShot(800, this, [this] { requestParams(compassParams(), true); });
    });
}

void CalibrationController::magCancel()
{
    // Stop here regardless of the answer
    m_cmd->cancelMagCal(0, [this](bool, const CommandSender::Ack &) {
        m_magFinish.stop();
        m_mag.running = false;
        restoreRates();
        refreshCoverage();
        setMagButtons(true, false, false);
        setMagText(QStringLiteral("Cancelled. Nothing was saved."), QString());
    });
}

void CalibrationController::magReboot()
{
    DialogHub::confirm(QStringLiteral("Reboot the autopilot now?"), [this](bool ok) {
        if (!ok) return;
        m_cmd->rebootAutopilot();
        m_magReboot = false;
        setMagText(QStringLiteral("Rebooting… reconnects by itself on most links."), QString());
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
    if (p->status == 1) return setMagText(QStringLiteral("Waiting to start — keep the vehicle ready…"), QStringLiteral("running"));
    const int covered = m_samples.covered();
    const QString step = p->status == 3 ? QStringLiteral("Step 2 of 2: once more, every attitude. ") : QString();
    const QString sphere = covered < 0 ? QString() : QStringLiteral("%1/80 directions covered. ").arg(covered);
    setMagText(step + sphere + QStringLiteral("Turn the vehicle until the white marker reaches the orange patch."), QStringLiteral("running"));
}

void CalibrationController::updateStats()
{
    if (!m_compassShown) return;
    const int focus = m_samples.focus();
    const int pts = m_samples.count(focus);
    QStringList parts{QStringLiteral("%1 pts").arg(pts)};
    if (const auto f = m_samples.field(); f && *f > 0)
        parts << QStringLiteral("|B| %1 mG%2").arg(std::lround(*f)).arg(m_samples.fitted() ? QString() : QStringLiteral(" (no fit yet)"));
    if (m_samples.covered() >= 0) parts << QStringLiteral("%1/80").arg(m_samples.covered());
    const QString s = pts ? parts.join(QStringLiteral(" · ")) : QStringLiteral("No samples");
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
        if (!p.has && !ids.contains(i)) state = QStringLiteral("idle");
        else if (p.has && p.status == 4) state = QStringLiteral("ok");
        else if (p.has && p.status >= 5) state = QStringLiteral("bad");
        QString text = QStringLiteral("—");
        if (p.has)
            text = QStringLiteral("%1% · %2%3").arg(std::lround(p.pct)).arg(magStatusName(p.status),
                                                                         p.attempt > 1 ? QStringLiteral(" · try %1").arg(p.attempt) : QString());
        out << QVariantMap{{QStringLiteral("name"), QStringLiteral("MAG %1").arg(i + 1)},
                           {QStringLiteral("color"), magColorList()[i]},
                           {QStringLiteral("pct"), p.has ? std::clamp(p.pct, 0.0, 100.0) : 0.0},
                           {QStringLiteral("state"), state},
                           {QStringLiteral("text"), text}};
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
                                   ? QStringLiteral("%1 → %2").arg(rotationName(r.oldOrientation), rotationName(r.newOrientation))
                                   : rotationName(r.oldOrientation);
        const auto rnd = [](float v) { return std::isfinite(v) ? QString::number(std::lround(v)) : QStringLiteral("—"); };
        out << QVariantMap{{QStringLiteral("name"), QStringLiteral("MAG %1").arg(i + 1)},
                           {QStringLiteral("color"), magColorList()[i]},
                           {QStringLiteral("ok"), ok},
                           {QStringLiteral("cells"), QStringList{magStatusName(r.status),
                                                                 std::isfinite(r.fitness) ? fixed(r.fitness, 1) : QStringLiteral("—"),
                                                                 QStringList{rnd(r.ofsX), rnd(r.ofsY), rnd(r.ofsZ)}.join(QLatin1String(" / ")),
                                                                 std::isfinite(r.scale) && r.scale != 0 ? fixed(r.scale, 3) : QStringLiteral("—"),
                                                                 orient,
                                                                 r.autosaved ? QStringLiteral("saved") : ok ? QStringLiteral("not saved") : QStringLiteral("—")}}};
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
            o.ofs << (h ? QString::number(std::lround(v)) : QStringLiteral("—"));
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
        if (busType == 1) where += QLatin1Char(' ') + hex2(address);
        else if (busType == 3) where += QStringLiteral(" node %1").arg(address);
        else if (busType == 4) where = QStringLiteral("SITL");
        rows << QVariantMap{{QStringLiteral("prio"), i + 1},
                            {QStringLiteral("device"), compassTypeName(type)},
                            {QStringLiteral("missing"), havePrio && !slot},
                            {QStringLiteral("bus"), where},
                            {QStringLiteral("use"), hasUse && use != 0},
                            {QStringLiteral("useKnown"), hasUse},
                            {QStringLiteral("useParam"), p.use},
                            {QStringLiteral("mount"), !slot || !slot->hasExt ? QStringLiteral("—") : slot->ext ? QStringLiteral("external") : QStringLiteral("internal")},
                            {QStringLiteral("orientation"), slot && slot->hasOrient ? rotationName(int(slot->orient)) : QStringLiteral("—")},
                            {QStringLiteral("offsets"), slot ? slot->ofs.join(QLatin1String(" / ")) : QStringLiteral("—")}};
    }
    m_compassRows = rows;
    m_compassNote = rows.isEmpty() ? (linked() ? QStringLiteral("No compass parameters yet — READ FROM VEHICLE.") : QStringLiteral("Not connected."))
                                   : QString();
}

void CalibrationController::rebuildFitOptions()
{
    QVariantList opts{QVariantMap{{QStringLiteral("value"), QStringLiteral("4")}, {QStringLiteral("label"), QStringLiteral("Very strict")}},
                      QVariantMap{{QStringLiteral("value"), QStringLiteral("8")}, {QStringLiteral("label"), QStringLiteral("Strict")}},
                      QVariantMap{{QStringLiteral("value"), QStringLiteral("16")}, {QStringLiteral("label"), QStringLiteral("Default")}},
                      QVariantMap{{QStringLiteral("value"), QStringLiteral("32")}, {QStringLiteral("label"), QStringLiteral("Relaxed")}}};
    bool has = false;
    const double v = param(QStringLiteral("COMPASS_CAL_FIT"), &has);
    if (has) {
        const QString s = QString::number(v);
        // A value off the list gets an entry of its own rather than being misread
        if (s != QLatin1String("4") && s != QLatin1String("8") && s != QLatin1String("16") && s != QLatin1String("32"))
            opts << QVariantMap{{QStringLiteral("value"), s}, {QStringLiteral("label"), QStringLiteral("Custom (%1)").arg(s)}};
        m_fitValue = s;
    }
    m_fitOptions = opts;
}

void CalibrationController::setMagFit(const QString &value)
{
    if (!linked()) return setMagText(QStringLiteral("Not connected to a vehicle."), QStringLiteral("error"));
    const double v = value.toDouble();
    m_fitValue = value;
    emit paramsChanged();
    m_params->write(QStringLiteral("COMPASS_CAL_FIT"), v, [this, value](bool ok, double) {
        if (ok) setMagText(QStringLiteral("COMPASS_CAL_FIT set to %1.").arg(value), QString());
        else setMagText(QStringLiteral("Could not set COMPASS_CAL_FIT: no confirmation from the autopilot"), QStringLiteral("error"));
        rebuildParamViews();
    });
}

void CalibrationController::setCompassUse(const QString &param, bool on)
{
    m_params->write(param, on ? 1 : 0, [this, param](bool ok, double) {
        if (!ok) DialogHub::toast(QStringLiteral("Could not set %1: no confirmation from the autopilot").arg(param));
        rebuildParamViews();   // the checkbox goes back to what the vehicle has
    });
}

void CalibrationController::readCompassParams() { requestParams(compassParams(), true); }

} // namespace corv
