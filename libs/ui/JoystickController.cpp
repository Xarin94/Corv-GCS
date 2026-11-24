#include "ui/JoystickController.h"

#include "core/Clock.h"
#include "core/DebugLog.h"
#include "core/Settings.h"
#include "gpu/SceneItem.h"
#include "ui/DialogHub.h"
#include "vehicle/CommandSender.h"
#include "vehicle/VehicleHub.h"
#include "vehicle/VehicleState.h"

#include <QGuiApplication>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include <algorithm>
#include <cmath>
#include <memory>

using namespace Qt::StringLiterals;

namespace corv {

using namespace joystick;

namespace {
const QString kConfigKey = u"datad-joystick-config"_s;
const char *const kDirections[4] = {"up", "down", "left", "right"};
constexpr qint64 kStaleMs = 500;      // no device data for this long: channels released
constexpr int kPollMs = 10;           // 100 Hz, fast enough for RC control
constexpr qint64 kPreviewMs = 40;     // the page needs 25 Hz at most

QJsonObject loadJson(const QString &key)
{
    QJsonObject o = Settings::instance().json(key).toObject();
    if (o.isEmpty()) {
        const QString raw = Settings::instance().value(key).toString();
        if (!raw.isEmpty()) o = QJsonDocument::fromJson(raw.toUtf8()).object();
    }
    return o;
}
} // namespace

JoystickController *JoystickController::s_instance = nullptr;

JoystickController::JoystickController(CommandSender *cmd, VehicleHub *hub, QObject *parent) : QObject(parent), m_cmd(cmd), m_hub(hub)
{
    s_instance = this;
    for (int i = 0; i < kMinInputs; ++i) m_axes.push_back(defaultAxis(i));
    for (int i = 0; i < 2; ++i) m_servoMap[std::size_t(i)] = servoFromJson(QJsonValue(), i);
    load();
    m_raw.assign(m_axes.size(), std::nullopt);
    m_pollTimer.setInterval(kPollMs);
    m_pollTimer.setTimerType(Qt::PreciseTimer);
    connect(&m_pollTimer, &QTimer::timeout, this, &JoystickController::poll);
    m_sendTimer.setTimerType(Qt::PreciseTimer);
    connect(&m_sendTimer, &QTimer::timeout, this, &JoystickController::send);
    // Devices plugged and unplugged: the list once a second, and the device we use picked up again
    m_scanTimer.setInterval(1000);
    connect(&m_scanTimer, &QTimer::timeout, this, &JoystickController::scan);
    if (Gamepad::available()) {
        m_scanTimer.start();
        QTimer::singleShot(0, this, &JoystickController::scan);
    }
}

JoystickController::~JoystickController()
{
    if (m_enabled) setEnabled(false);
}

// ── Devices ──────────────────────────────────────────────────────────────────

void JoystickController::scan()
{
    QVariantList list;
    for (const Gamepad::Device &d : m_pad.devices())
        list << QVariantMap{{u"id"_s, int(d.id)},
                            {u"label"_s, u"[%1] %2 (%3A/%4B)"_s.arg(d.id).arg(d.name).arg(d.axes).arg(d.buttons)}};
    if (list != m_deviceList) {
        m_deviceList = list;
        emit devicesChanged();
    }
    if (!m_pad.isOpen()) reselect();
}

bool JoystickController::reselect()
{
    if (m_pad.isOpen()) return true;
    const std::vector<Gamepad::Device> devs = m_pad.devices();
    const Gamepad::Device *pick = nullptr;
    for (const Gamepad::Device &d : devs)
        if (!m_padName.isEmpty() && d.name == m_padName) pick = &d;
    if (!pick && m_padName.isEmpty() && devs.size() == 1) pick = &devs[0];
    if (!pick) return false;
    select(int(pick->id));
    return m_pad.isOpen();
}

void JoystickController::select(int id)
{
    if (id < 0) {
        // No device: everything off
        m_servoButtons = false;
        m_servoStatus = u"Disabled"_s;
        setEnabled(false);
        m_pad.close();
        m_padName.clear();
        m_pollTimer.stop();
        resetButtons();
        save();
        emit stateChanged();
        emit configChanged();
        return;
    }
    if (m_pad.isOpen() && int(m_pad.device().id) != id) {
        if (m_enabled) releaseAll();
        m_channels.fill(kRelease);
        resetButtons();
    }
    if (!m_pad.open(unsigned(id))) {
        emit stateChanged();
        return;
    }
    m_padName = m_pad.device().name;
    // Rows for every input of this device, keeping the existing mappings
    const std::size_t count = std::size_t(std::max({kMinInputs, m_pad.device().axes, int(m_axes.size())}));
    for (std::size_t i = m_axes.size(); i < count; ++i) m_axes.push_back(defaultAxis(int(i)));
    m_raw.assign(m_axes.size(), std::nullopt);
    m_lastData = wallMs();
    m_pollTimer.start();   // preview and the look buttons work without the override
    save();
    emit stateChanged();
    emit configChanged();
}

// ── Override ─────────────────────────────────────────────────────────────────

void JoystickController::askEnable()
{
    if (m_enabled) return;
    if (!m_pad.isOpen() && !reselect()) {
        DialogHub::toast(u"No gamepad selected — press SCAN and pick one"_s);
        emit stateChanged();
        return;
    }
    DialogHub::confirm(u"RC override takes control of the vehicle: it may move. Continue?"_s, [this](bool ok) {
        if (ok) setEnabled(true);
        emit stateChanged();
    });
}

void JoystickController::setEnabled(bool on)
{
    if (on) {
        if (m_enabled) return;
        if (!m_pad.isOpen() && !reselect()) {
            emit stateChanged();
            return;
        }
        m_enabled = true;
        m_suspended = false;
        m_lastData = wallMs();   // grace period before the stale-data failsafe
        emit stateChanged();
        applyParameters();       // the sending starts once they are written
        return;
    }
    const bool was = m_enabled;
    m_enabled = false;
    m_suspended = false;
    m_rcActive = false;
    ++m_paramSession;
    m_sendTimer.stop();
    if (was) releaseAll();
    m_channels.fill(kRelease);
    if (!was) return;
    restoreParameters();
    emit stateChanged();
}

void JoystickController::applyParameters()
{
    // ArduPilot takes RC_CHANNELS_OVERRIDE only from SYSID_MYGCS, only with a non-zero
    // RC_OVERRIDE_TIME and without the ignore-override bit (2) of RC_OPTIONS; with no
    // receiver the RC throttle failsafe would trip. Each value is read first, to put back.
    const int session = ++m_paramSession;
    m_saved.clear();
    const auto s = m_hub->snapshot();
    if (!s->connected || s->connectionType.startsWith("msp"_L1)) {
        m_sendTimer.start(int(std::lround(1000.0 / m_sendRate)));
        return;
    }
    struct Step {
        QString id;
        std::function<std::optional<float>(float)> target;
        QString otherId = {};   // its name before or after 4.7
    };
    // MAV_GCS_SYSID from 4.7 on: the name of this version first, the other when it does not answer
    const QStringList version = s->firmwareVersion.section('-'_L1, 0, 0).split('.'_L1);
    const bool renamed = version.size() >= 2 && version[0].toInt() * 100 + version[1].toInt() >= 407;
    const QString gcsId[] = {u"SYSID_MYGCS"_s, u"MAV_GCS_SYSID"_s};
    auto steps = std::make_shared<std::vector<Step>>(std::vector<Step>{
        {gcsId[renamed], [](float v) { return v == 255 ? std::nullopt : std::optional<float>(255); }, gcsId[!renamed]},
        {u"RC_OVERRIDE_TIME"_s, [](float) { return std::optional<float>(3); }},
        {u"RC_OPTIONS"_s,
         [](float v) {
             const int cur = int(v), cleared = cur & ~2;
             return cleared != cur ? std::optional<float>(float(cleared)) : std::nullopt;
         }},
        {u"FS_THR_ENABLE"_s, [](float) { return std::optional<float>(0); }},
    });
    // The chain holds itself weakly; the request in flight keeps it alive
    auto next = std::make_shared<std::function<void(std::size_t)>>();
    std::weak_ptr<std::function<void(std::size_t)>> weak = next;
    *next = [this, session, steps, weak](std::size_t i) {
        if (session != m_paramSession || !m_enabled) return;   // switched off meanwhile
        if (i >= steps->size()) {
            m_sendTimer.start(int(std::lround(1000.0 / m_sendRate)));
            emit stateChanged();
            return;
        }
        const Step step = (*steps)[i];
        const auto next = weak.lock();
        if (!next) return;
        m_cmd->fetchParameter(step.id, 1500, 2, [this, session, step, steps, next, i](bool ok, float value, int type) {
            if (session != m_paramSession) return;
            if (!ok && !step.otherId.isEmpty()) {
                (*steps)[i].id = step.otherId;
                (*steps)[i].otherId.clear();
                (*next)(i);
                return;
            }
            if (!ok) {
                LOG_W("joystick") << "could not read" << step.id;
                (*next)(i + 1);
                return;
            }
            const std::optional<float> want = step.target(value);
            if (!want) {
                (*next)(i + 1);
                return;
            }
            m_saved.push_back({step.id, value, type});
            m_cmd->setParameter(step.id, *want, type, [step, next, i, value, want](bool ok, float) {
                if (ok) LOG_I("joystick") << step.id << value << "->" << *want;
                else LOG_W("joystick") << "could not set" << step.id;
                (*next)(i + 1);
            });
        });
    };
    (*next)(0);
}

void JoystickController::restoreParameters()
{
    for (const Saved &s : m_saved) {
        LOG_I("joystick") << "restored" << s.id << "to" << s.value;
        m_cmd->setParameter(s.id, s.value, s.type, [](bool, float) {});
    }
    m_saved.clear();
}

void JoystickController::setSendRate(int hz)
{
    m_sendRate = std::clamp(hz, 1, 50);   // 0 Hz would flood the link
    if (m_sendTimer.isActive()) m_sendTimer.start(int(std::lround(1000.0 / m_sendRate)));
    save();
    emit stateChanged();
}

void JoystickController::releaseAll()
{
    if (!m_hub->snapshot()->connected) return;
    std::array<uint16_t, kChannels> release{};
    m_cmd->sendRcChannelsOverride(release);
}

void JoystickController::suspend(const QString &why)
{
    m_suspended = true;
    m_rcActive = false;
    m_channels.fill(kRelease);
    releaseAll();
    LOG_W("joystick") << why << "- channels released, override suspended";
    emit stateChanged();
}

void JoystickController::send()
{
    const auto s = m_hub->snapshot();
    if (!m_enabled || !s->connected) {
        if (m_rcActive) {
            m_rcActive = false;
            emit stateChanged();
        }
        return;
    }
    // No device data for 500 ms: release to the receiver and wait, still enabled
    if (m_suspended || wallMs() - m_lastData > kStaleMs) {
        if (!m_suspended) suspend(u"no gamepad data"_s);
        return;
    }
    if (!m_rcActive) {
        m_rcActive = true;
        emit stateChanged();
    }
    m_cmd->sendRcChannelsOverride(m_channels);
}

// ── Reading ──────────────────────────────────────────────────────────────────

void JoystickController::poll()
{
    Gamepad::State st;
    if (!m_pad.isOpen() || !m_pad.read(st)) {
        // Unplugged: keep the name so the same device is picked up again
        m_pollTimer.stop();
        resetButtons();
        std::fill(m_raw.begin(), m_raw.end(), std::nullopt);
        if (m_enabled && !m_suspended) suspend(u"gamepad lost"_s);
        emit stateChanged();
        emit liveChanged();
        return;
    }
    const qint64 now = wallMs();
    m_lastData = now;
    if (m_suspended) {
        m_suspended = false;
        LOG_I("joystick") << "gamepad data resumed - override active again";
        emit stateChanged();
    }
    std::array<uint16_t, kChannels> ch{};
    for (std::size_t i = 0; i < m_axes.size(); ++i) {
        const std::optional<double> raw = readSource(m_axes[i], st.axes, st.buttons);
        m_raw[i] = raw;
        const std::optional<int> pwm = channelPwm(m_axes[i], raw);
        if (pwm && m_axes[i].channel >= 1 && m_axes[i].channel <= kChannels) ch[std::size_t(m_axes[i].channel - 1)] = uint16_t(*pwm);
    }
    m_channels = ch;
    pollButtons(st.buttons);
    if (now - m_lastPreview >= kPreviewMs) {
        m_lastPreview = now;
        emit liveChanged();
    }
}

void JoystickController::resetButtons()
{
    m_prevButtons.reset();
    m_buttonTarget.clear();
    m_pressed.clear();
    SceneItem::setGamepadLook(false, false, false, false);
}

void JoystickController::pollButtons(const std::vector<double> &buttons)
{
    std::vector<bool> down(buttons.size());
    m_pressed.clear();
    for (std::size_t i = 0; i < buttons.size(); ++i) {
        down[i] = buttons[i] > 0.5;
        if (down[i]) m_pressed.push_back(int(i));
    }
    // Buttons count only while the window is active
    if (QGuiApplication::applicationState() != Qt::ApplicationActive) {
        const std::vector<int> keep = m_pressed;
        resetButtons();
        m_pressed = keep;
        return;
    }
    auto held = [&](int b) { return b >= 0 && b < int(down.size()) && down[std::size_t(b)]; };
    SceneItem::setGamepadLook(held(m_viewMap[0]), held(m_viewMap[1]), held(m_viewMap[2]), held(m_viewMap[3]));

    // A press is an edge on the same vehicle: a button held across a reconnection or a
    // configuration change never becomes a new servo command
    const auto s = m_hub->snapshot();
    const QString target = s->connected ? u"%1:%2:%3"_s.arg(s->connectionType).arg(s->systemId).arg(s->componentId) : QString();
    if (m_prevButtons && !target.isEmpty() && target == m_buttonTarget && m_servoButtons) {
        for (const ServoConfig &cfg : m_servoMap) {
            const int b = cfg.button;
            if (!held(b) || b >= int(m_prevButtons->size()) || (*m_prevButtons)[std::size_t(b)]) continue;
            const int pwm = servoPwm(cfg);
            m_cmd->sendServoTest(cfg.servo, pwm);
            m_servoStatus = u"Sent S%1: %2% (%3 us)"_s.arg(cfg.servo).arg(cfg.percent).arg(pwm);
            emit stateChanged();
        }
    }
    m_prevButtons = down;
    m_buttonTarget = target;
}

// ── Configuration ────────────────────────────────────────────────────────────

void JoystickController::setInput(int index, const QVariantMap &patch)
{
    if (index < 0 || index >= int(m_axes.size())) return;
    QJsonObject o = toJson(m_axes[std::size_t(index)]);
    if (patch.contains(u"source"_s)) {
        const QString src = patch.value(u"source"_s).toString();
        o.insert(u"sourceType"_s, src.section(':'_L1, 0, 0));
        o.insert(u"sourceIndex"_s, src.section(':'_L1, 1).toInt());
    }
    for (const char *k : {"channel", "inverted", "deadzone", "expo"}) {
        const QString key = QLatin1String(k);
        if (patch.contains(key)) o.insert(key, QJsonValue::fromVariant(patch.value(key)));
    }
    m_axes[std::size_t(index)] = axisFromJson(o, index);
    save();
    emit configChanged();
}

void JoystickController::setServo(int index, const QVariantMap &patch)
{
    if (index < 0 || index >= int(m_servoMap.size())) return;
    QJsonObject o = toJson(m_servoMap[std::size_t(index)]);
    for (const char *k : {"button", "servo", "percent"}) {
        const QString key = QLatin1String(k);
        if (patch.contains(key)) o.insert(key, QJsonValue::fromVariant(patch.value(key)));
    }
    m_servoMap[std::size_t(index)] = servoFromJson(o, index);
    resetButtons();
    save();
    emit configChanged();
}

void JoystickController::setViewButton(const QString &direction, int button)
{
    for (int d = 0; d < 4; ++d)
        if (direction == QLatin1String(kDirections[d])) m_viewMap[std::size_t(d)] = button >= 0 && button < 256 ? button : -1;
    resetButtons();
    save();
    emit configChanged();
}

void JoystickController::setServoButtons(bool on)
{
    // Armed deliberately each session, never saved
    m_servoButtons = on;
    m_servoStatus = on ? u"Ready - release held buttons before commanding"_s : u"Disabled"_s;
    resetButtons();
    emit stateChanged();
}

void JoystickController::load()
{
    const QJsonObject c = loadJson(kConfigKey);
    if (c.isEmpty()) return;
    m_padName = c.value(u"gamepadId"_s).toString();
    if (c.value(u"sendRateHz"_s).isDouble()) m_sendRate = std::clamp(c.value(u"sendRateHz"_s).toInt(), 1, 50);
    const QJsonArray axes = c.value(u"axisMap"_s).toArray();
    if (!axes.isEmpty()) {
        m_axes.clear();
        const int n = std::max(kMinInputs, std::min(64, int(axes.size())));
        for (int i = 0; i < n; ++i) m_axes.push_back(i < axes.size() && !axes.at(i).isNull() ? axisFromJson(axes.at(i), i) : defaultAxis(i));
    }
    const QJsonArray servos = c.value(u"servoMap"_s).toArray();
    for (int i = 0; i < 2; ++i) m_servoMap[std::size_t(i)] = servoFromJson(servos.at(i), i);
    const QJsonObject view = c.value(u"viewMap"_s).toObject();
    for (int d = 0; d < 4; ++d) {
        const QJsonValue v = view.value(QLatin1String(kDirections[d]));
        m_viewMap[std::size_t(d)] = v.isDouble() && v.toInt() >= 0 && v.toInt() < 256 ? v.toInt() : -1;
    }
}

void JoystickController::save()
{
    QJsonArray axes, servos;
    for (const AxisConfig &a : m_axes) axes.append(toJson(a));
    for (const ServoConfig &s : m_servoMap) servos.append(toJson(s));
    QJsonObject view;
    for (int d = 0; d < 4; ++d) view.insert(QLatin1String(kDirections[d]), m_viewMap[std::size_t(d)]);
    Settings::instance().setJson(kConfigKey, QJsonObject{{u"gamepadIndex"_s, m_pad.isOpen() ? QJsonValue(int(m_pad.device().id)) : QJsonValue()},
                                                         {u"gamepadId"_s, m_padName},
                                                         {u"sendRateHz"_s, m_sendRate},
                                                         {u"axisMap"_s, axes},
                                                         {u"servoMap"_s, servos},
                                                         {u"viewMap"_s, view}});
}

// ── What the page shows ──────────────────────────────────────────────────────

QString JoystickController::status() const
{
    if (!Gamepad::available()) return u"Not available in this build (no SDL3)"_s;
    if (!m_enabled)
        return m_pad.isOpen() ? u"RC disabled - input preview / FPV available"_s : u"Disabled - No gamepad (press SCAN)"_s;
    if (m_suspended || !m_pad.isOpen())
        return !m_pad.isOpen() ? u"SUSPENDED - gamepad unplugged (channels released)"_s
                               : u"SUSPENDED - no gamepad data (channels released)"_s;
    if (m_rcActive) return u"SENDING (%1Hz)"_s.arg(m_sendRate);
    if (m_hub->snapshot()->connected) return u"Starting (%1Hz)"_s.arg(m_sendRate);
    return u"Active (%1Hz) - Waiting connection"_s.arg(m_sendRate);
}

QString JoystickController::statusKind() const
{
    if (!m_enabled) return {};
    if (m_suspended || !m_pad.isOpen()) return u"warning"_s;
    if (m_rcActive) return u"active"_s;
    return m_hub->snapshot()->connected ? u"warning"_s : QString();
}

QVariantList JoystickController::inputs() const
{
    QVariantList out;
    for (const AxisConfig &a : m_axes)
        out << QVariantMap{{u"source"_s, sourceKey(a)}, {u"channel"_s, a.channel}, {u"inverted"_s, a.inverted},
                           {u"deadzone"_s, a.deadzone}, {u"expo"_s, a.expo}};
    return out;
}

QVariantList JoystickController::sourceOptions() const
{
    const int axes = m_pad.isOpen() ? m_pad.device().axes : 0, buttons = m_pad.isOpen() ? m_pad.device().buttons : 0;
    QVariantList out{QVariantMap{{u"value"_s, u"none:0"_s}, {u"label"_s, u"Unmapped"_s}}};
    for (int i = 0; i < axes; ++i)
        out << QVariantMap{{u"value"_s, u"axis:%1"_s.arg(i)}, {u"label"_s, u"Axis %1"_s.arg(i)}};
    for (int i = 0; i < buttons; ++i)
        out << QVariantMap{{u"value"_s, u"button:%1"_s.arg(i)}, {u"label"_s, u"Button %1"_s.arg(i)}};
    // Sources a row uses that this device does not have
    QStringList extra;
    for (const AxisConfig &a : m_axes) {
        const bool missing = (a.source == AxisConfig::Source::Axis && a.sourceIndex >= axes) || (a.source == AxisConfig::Source::Button && a.sourceIndex >= buttons);
        if (!missing || extra.contains(sourceKey(a))) continue;
        extra << sourceKey(a);
        out << QVariantMap{{u"value"_s, sourceKey(a)},
                           {u"label"_s, u"%1 %2 (unavailable)"_s
                                                         .arg(a.source == AxisConfig::Source::Axis ? u"Axis"_s : u"Button"_s)
                                                         .arg(a.sourceIndex)}};
    }
    return out;
}

QVariantList JoystickController::buttonOptions() const
{
    const int buttons = m_pad.isOpen() ? m_pad.device().buttons : 0;
    QVariantList out{QVariantMap{{u"value"_s, u"-1"_s}, {u"label"_s, u"Unmapped"_s}}};
    for (int i = 0; i < buttons; ++i)
        out << QVariantMap{{u"value"_s, QString::number(i)}, {u"label"_s, u"Button %1"_s.arg(i)}};
    QList<int> used;
    for (const ServoConfig &s : m_servoMap) used << s.button;
    for (int b : m_viewMap) used << b;
    std::sort(used.begin(), used.end());
    used.erase(std::unique(used.begin(), used.end()), used.end());
    for (int b : used)
        if (b >= buttons)
            out << QVariantMap{{u"value"_s, QString::number(b)}, {u"label"_s, u"Button %1 (unavailable)"_s.arg(b)}};
    return out;
}

QVariantList JoystickController::servos() const
{
    QVariantList out;
    for (const ServoConfig &s : m_servoMap)
        out << QVariantMap{{u"button"_s, s.button}, {u"servo"_s, s.servo}, {u"percent"_s, s.percent}};
    return out;
}

QVariantMap JoystickController::view() const
{
    QVariantMap out;
    for (int d = 0; d < 4; ++d) out.insert(QLatin1String(kDirections[d]), m_viewMap[std::size_t(d)]);
    return out;
}

QVariantList JoystickController::raw() const
{
    QVariantList out;
    for (const auto &v : m_raw) out << (v ? QVariant(*v) : QVariant());
    return out;
}

QVariantList JoystickController::channels() const
{
    QVariantList out;
    for (uint16_t c : m_channels) out << int(c);
    return out;
}

QString JoystickController::pressed() const
{
    if (m_pressed.empty()) return u"—"_s;
    QStringList s;
    for (int b : m_pressed) s << QString::number(b);
    return s.join(u", "_s);
}

} // namespace corv
