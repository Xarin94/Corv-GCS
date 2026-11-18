#include "ui/SetupController.h"

#include "core/DebugLog.h"
#include "ui/DialogHub.h"
#include "vehicle/CommandSender.h"
#include "vehicle/FlightModes.h"
#include "vehicle/ParamStore.h"
#include "vehicle/VehicleHub.h"
#include "vehicle/VehicleState.h"

#include <QTimer>
#include <cmath>
#include <memory>

namespace corv {

SetupController *SetupController::s_instance = nullptr;

SetupController::SetupController(ParamStore *params, CommandSender *cmd, VehicleHub *hub, QObject *parent)
    : QObject(parent), m_params(params), m_cmd(cmd), m_hub(hub)
{
    s_instance = this;
    m_rcMin.fill(2000);
    m_rcMax.fill(1000);
    m_rcTrim.fill(1500);
    // Every PARAM_VALUE lands in ParamStore: the pages repaint, coalesced
    connect(m_params, &ParamStore::changed, this, [this] {
        if (m_revPending) return;
        m_revPending = true;
        QTimer::singleShot(100, this, [this] {
            m_revPending = false;
            ++m_paramRev;
            emit paramsChanged();
        });
    });
    m_live.setInterval(250);
    connect(&m_live, &QTimer::timeout, this, &SetupController::tick);
    m_live.start();
}

void SetupController::setPage(const QString &p)
{
    if (p == m_page) return;
    m_page = p;
    emit pageChanged();
    // Re-request the RC stream so data flows on this page (stream 3 = RC_CHANNELS at 4 Hz)
    if (p == QLatin1String("RADIO CAL") && m_hub->snapshot()->connected) m_cmd->requestDataStream(3, 4);
}

QVariant SetupController::param(const QString &name) const
{
    const ParamValue *p = m_params->find(name);
    return p ? QVariant(p->value) : QVariant();
}

void SetupController::setStatus(const QString &page, const QString &text, bool ok)
{
    m_status[page] = QVariantMap{{QStringLiteral("text"), text}, {QStringLiteral("ok"), ok}};
    emit statusChanged();
}

void SetupController::readParams(const QStringList &names, const QString &page)
{
    QStringList unique;
    for (const QString &n : names)
        if (!unique.contains(n)) unique << n;
    if (unique.isEmpty()) return;
    setStatus(page, QStringLiteral("Reading %1 parameters...").arg(unique.size()), false);
    m_params->enqueueFetch(unique);
    // Answers arrive asynchronously (one at a time, retried): report when the queue drains
    auto conn = std::make_shared<QMetaObject::Connection>();
    *conn = connect(m_params, &ParamStore::queueChanged, this, [this, unique, page, conn] {
        if (m_params->queueBusy()) return;
        disconnect(*conn);
        int missing = 0;
        for (const QString &n : unique)
            if (!m_params->find(n)) ++missing;
        if (missing) setStatus(page, QStringLiteral("%1/%2 read — %3 unsupported on this firmware").arg(unique.size() - missing).arg(unique.size()).arg(missing), false);
        else setStatus(page, QStringLiteral("%1 parameters read").arg(unique.size()), true);
    });
}

void SetupController::writeParams(const QVariantMap &values, const QString &page, const QString &doneText)
{
    QList<QPair<QString, double>> list;
    for (auto it = values.cbegin(); it != values.cend(); ++it) {
        bool ok = false;
        const double v = it.value().toDouble(&ok);
        if (ok && std::isfinite(v)) list.append({it.key(), v});
    }
    if (list.isEmpty()) {
        DialogHub::toast(QStringLiteral("Nothing to write"));
        return;
    }
    setStatus(page, QStringLiteral("Writing %1 parameters...").arg(list.size()), false);
    m_params->writeMany(list, [this, page, doneText](int written, int total) {
        setStatus(page, QStringLiteral("%1 parameters written").arg(written) + (written < total ? QStringLiteral(", %1 failed").arg(total - written) : QString()),
                  written == total);
        DialogHub::toast(doneText.arg(written));
    });
}

QString SetupController::vehicleClass() const
{
    const int t = m_hub->snapshot()->vehicleType;
    return t > 0 ? modes::vehicleTypeName(t) : QString();
}

/// The modes a FLTMODEn can take: the vehicle's own table once it is known,
/// else the copter list of index.html.
QVariantList SetupController::flightModeOptions() const
{
    const auto s = m_hub->snapshot();
    QVariantList out;
    if (s->vehicleType > 0) {
        for (const auto &[num, name] : modes::available(s->vehicleType, s->connectionType))
            out << QVariantMap{{QStringLiteral("value"), QString::number(num)}, {QStringLiteral("label"), name}};
        if (!out.isEmpty()) return out;
    }
    static const QList<QPair<int, const char *>> copter{{0, "STABILIZE"}, {2, "ALT_HOLD"}, {5, "LOITER"}, {3, "AUTO"}, {4, "GUIDED"},
                                                        {6, "RTL"},       {9, "LAND"},     {1, "ACRO"},   {16, "POSHOLD"}, {7, "CIRCLE"}};
    for (const auto &[num, name] : copter)
        out << QVariantMap{{QStringLiteral("value"), QString::number(num)}, {QStringLiteral("label"), QString::fromLatin1(name)}};
    return out;
}

// ── Live data (4 Hz) ────────────────────────────────────────────────────────

void SetupController::tick()
{
    const auto s = m_hub->snapshot();
    if (s->vehicleType != m_vehicleType) {
        m_vehicleType = s->vehicleType;
        emit vehicleChanged();
    }
    // RC_CHANNELS: min / max while calibrating (updateRcCalBars)
    for (int i = 0; i < 16; ++i) {
        const int v = s->rcChannels[std::size_t(i)];
        if (v == 0 || v == 65535) continue;   // no data or unused channel
        m_rcValue[std::size_t(i)] = v;
        if (m_rcCalibrating) {
            m_rcMin[std::size_t(i)] = std::min(m_rcMin[std::size_t(i)], v);
            m_rcMax[std::size_t(i)] = std::max(m_rcMax[std::size_t(i)], v);
        }
    }
    if (m_page != QLatin1String("RADIO CAL") && m_page != QLatin1String("VIBRATIONS") && m_page != QLatin1String("SERVO / RELAY")) return;
    m_vib = QVariantMap{{QStringLiteral("x"), s->vibX}, {QStringLiteral("y"), s->vibY}, {QStringLiteral("z"), s->vibZ},
                        {QStringLiteral("clip0"), s->vibClip0}, {QStringLiteral("clip1"), s->vibClip1}, {QStringLiteral("clip2"), s->vibClip2}};
    m_vibHistory.clear();
    for (const VibrationSample &v : s->vibHistory) m_vibHistory << QVariant(QVariantList{std::abs(v.x), std::abs(v.y), std::abs(v.z)});
    m_servos.clear();
    for (int i = 0; i < 16; ++i) m_servos << int(s->servoOutputs[std::size_t(i)]);
    emit liveChanged();
}

QVariantList SetupController::rc() const
{
    QVariantList out;
    for (std::size_t i = 0; i < 16; ++i)
        out << QVariantMap{{QStringLiteral("value"), m_rcValue[i]}, {QStringLiteral("min"), m_rcMin[i]}, {QStringLiteral("max"), m_rcMax[i]}};
    return out;
}

void SetupController::rcCalToggle()
{
    m_rcCalibrating = !m_rcCalibrating;
    if (m_rcCalibrating) {
        m_rcMin.fill(2000);
        m_rcMax.fill(1000);
        m_rcStatus = QStringLiteral("Calibrating... move all sticks to extremes");
    } else {
        // Capture trim at centre
        const auto s = m_hub->snapshot();
        for (std::size_t i = 0; i < 16; ++i) m_rcTrim[i] = s->rcChannels[i];
        m_rcStatus = QStringLiteral("Calibration stopped. Trim captured.");
    }
    emit rcCalChanged();
    emit liveChanged();
}

void SetupController::rcCalSave()
{
    if (m_rcCalibrating) {
        DialogHub::toast(QStringLiteral("Stop calibration first"));
        return;
    }
    QVariantMap values;
    for (int i = 0; i < 16; ++i) {
        const auto k = std::size_t(i);
        if (m_rcMin[k] < m_rcMax[k]) {
            LOG_I("rccal") << QStringLiteral("saving CH%1: MIN=%2 MAX=%3 TRIM=%4").arg(i + 1).arg(m_rcMin[k]).arg(m_rcMax[k]).arg(m_rcTrim[k]);
            values[QStringLiteral("RC%1_MIN").arg(i + 1)] = m_rcMin[k];
            values[QStringLiteral("RC%1_MAX").arg(i + 1)] = m_rcMax[k];
            values[QStringLiteral("RC%1_TRIM").arg(i + 1)] = m_rcTrim[k];
        }
    }
    if (values.isEmpty()) {
        DialogHub::toast(QStringLiteral("No channels calibrated. Run calibration first (START → move sticks → STOP)."));
        return;
    }
    writeParams(values, QStringLiteral("RADIO CAL"), QStringLiteral("Saved %1 RC calibration parameters"));
}

void SetupController::rcCalReset()
{
    m_rcMin.fill(2000);
    m_rcMax.fill(1000);
    m_rcTrim.fill(1500);
    m_rcCalibrating = false;
    emit rcCalChanged();
    emit liveChanged();
}

QVariantList SetupController::relays() const
{
    QVariantList out;
    for (bool on : m_relay) out << on;
    return out;
}

void SetupController::toggleRelay(int relay)
{
    if (relay < 0 || relay > 3) return;
    const bool on = m_relay[std::size_t(relay)];
    m_cmd->sendRelayToggle(relay, on ? 0 : 1);
    m_relay[std::size_t(relay)] = !on;
    emit relaysChanged();
}

void SetupController::servoTest(int servo, int pwm) { m_cmd->sendServoTest(servo, pwm); }

} // namespace corv
