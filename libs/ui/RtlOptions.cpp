#include "ui/RtlOptions.h"

#include "ui/DialogHub.h"
#include "vehicle/FlightModes.h"
#include "vehicle/ParamStore.h"
#include "vehicle/VehicleHub.h"

#include <cmath>
#include <memory>

using namespace Qt::StringLiterals;

namespace corv {

RtlOptions *RtlOptions::s_instance = nullptr;

RtlOptions::RtlOptions(ParamStore *params, VehicleHub *hub, QObject *parent) : QObject(parent), m_params(params), m_hub(hub)
{
    s_instance = this;
    m_fields = {{u"alt"_s, u"RTL ALT (m)"_s}, {u"final"_s, u"RTL ALT FINAL (m)"_s}, {u"speed"_s, u"RTL SPEED (m/s)"_s},
                {u"points"_s, u"SRTL POINTS"_s}, {u"accuracy"_s, u"SRTL ACCURACY (m)"_s}};
    // Another link may be another vehicle
    connect(hub, &VehicleHub::connectionChanged, this, &RtlOptions::clear);
    // The answers show as they come
    connect(params, &ParamStore::changed, this, [this] {
        if (m_reading) fill();
    });
}

std::vector<RtlOptions::Name> RtlOptions::namesOf(const QString &key) const
{
    const auto s = m_hub->snapshot();
    return names(key, modes::vehicleTypeName(s->vehicleType), s->firmwareVersion);
}

std::vector<RtlOptions::Name> RtlOptions::names(const QString &key, const QString &vehicle, const QString &version)
{
    // Of an old and a new name, the one of the version; both while it is unknown
    const QStringList v = version.section('-'_L1, 0, 0).split('.'_L1);
    const int number = v.size() >= 2 ? v[0].toInt() * 100 + v[1].toInt() : 0;
    auto renamed = [number](const QString &now, int since, const QString &before, double beforeScale) {
        if (!number) return std::vector<Name>{{now, 1}, {before, beforeScale}};
        return number >= since ? std::vector<Name>{{now, 1}} : std::vector<Name>{{before, beforeScale}};
    };
    if (key == "points"_L1 || key == "accuracy"_L1) {
        if (vehicle == "Plane"_L1 || vehicle == "Sub"_L1) return {};
        return {{key == "points"_L1 ? u"SRTL_POINTS"_s : u"SRTL_ACCURACY"_s, 1}};
    }
    if (vehicle == "Plane"_L1) return key == "alt"_L1 ? renamed(u"RTL_ALTITUDE"_s, 405, u"ALT_HOLD_RTL"_s, 100) : std::vector<Name>{};
    if (vehicle == "Rover"_L1) return key == "speed"_L1 ? std::vector<Name>{{u"RTL_SPEED"_s, 1}} : std::vector<Name>{};
    if (vehicle == "Sub"_L1) return {};
    if (key == "alt"_L1) return renamed(u"RTL_ALT_M"_s, 407, u"RTL_ALT"_s, 100);
    if (key == "final"_L1) return renamed(u"RTL_ALT_FINAL_M"_s, 407, u"RTL_ALT_FINAL"_s, 100);
    return renamed(u"RTL_SPEED_MS"_s, 407, u"RTL_SPEED"_s, 100);
}

void RtlOptions::clear()
{
    for (Field &f : m_fields) {
        f.param.clear();
        f.text.clear();
    }
    m_reading = false;
    emit changed();
}

void RtlOptions::read()
{
    const auto s = m_hub->snapshot();
    if (!s->connected) {
        DialogHub::toast(u"No vehicle connected"_s);
        return;
    }
    QStringList ask;
    for (const Field &f : m_fields) {
        const auto list = namesOf(f.key);
        bool known = false;
        for (const Name &n : list) known = known || m_params->find(n.param);
        if (!known)
            for (const Name &n : list) ask << n.param;
    }
    if (ask.isEmpty()) {
        fill();
        return;
    }
    m_reading = true;
    emit changed();
    m_params->enqueueFetch(ask);
    auto conn = std::make_shared<QMetaObject::Connection>();
    *conn = connect(m_params, &ParamStore::queueChanged, this, [this, conn] {
        if (m_params->queueBusy()) return;
        disconnect(*conn);
        m_reading = false;
        fill();
    });
}

void RtlOptions::fill()
{
    for (Field &f : m_fields) {
        f.param.clear();
        f.text.clear();
        for (const Name &n : namesOf(f.key)) {
            const ParamValue *p = m_params->find(n.param);
            if (!p) continue;
            f.param = n.param;
            f.scale = n.scale;
            f.text = QString::number(std::round(p->value / n.scale * 1e4) / 1e4, 'g', 10);
            break;
        }
    }
    emit changed();
}

void RtlOptions::write(const QVariantMap &texts)
{
    QList<QPair<QString, double>> list;
    for (const Field &f : m_fields) {
        if (f.param.isEmpty() || !texts.contains(f.key)) continue;
        const QString t = texts.value(f.key).toString().trimmed().replace(','_L1, '.'_L1);
        bool ok = false;
        const double v = t.toDouble(&ok);
        if (!ok || t == f.text) continue;
        list.append({f.param, v * f.scale});
    }
    if (list.isEmpty()) {
        DialogHub::toast(u"Nothing to write"_s);
        return;
    }
    m_params->writeMany(list, [this](int written, int total) {
        DialogHub::toast(written == total ? u"Written %1 RTL parameters"_s.arg(written) : u"%1 of %2 RTL parameters written"_s.arg(written).arg(total));
        fill();
    });
}

QVariantList RtlOptions::fields() const
{
    QVariantList out;
    for (const Field &f : m_fields)
        out << QVariantMap{{u"key"_s, f.key}, {u"label"_s, f.label}, {u"text"_s, f.text}, {u"param"_s, f.param}, {u"available"_s, !f.param.isEmpty()}};
    return out;
}

} // namespace corv
