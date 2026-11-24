// RtlOptions.h - GCS OPTIONS -> RTL OPTIONS (QML singleton "Rtl"): the parameters
// that decide how the vehicle comes home, in metres and m/s whatever the firmware
// stores. Their names and units depend on the vehicle and its version (a Copter's
// RTL_ALT in cm became RTL_ALT_M in metres with 4.7, a Plane has RTL_ALTITUDE):
// READ asks for the names of the vehicle's version (all of them while it is unknown)
// and the one that answers is used; a field the vehicle has no parameter for stays
// greyed. WRITE sends only what was changed.
// GUI thread.
#pragma once

#include <QObject>
#include <QQmlEngine>
#include <QVariantList>
#include <vector>

namespace corv {

class ParamStore;
class VehicleHub;

class RtlOptions : public QObject
{
    Q_OBJECT
    QML_NAMED_ELEMENT(Rtl)
    QML_SINGLETON
    /// [{key, label, text, param, available}] in the order of the panel.
    Q_PROPERTY(QVariantList fields READ fields NOTIFY changed)
    Q_PROPERTY(bool reading READ reading NOTIFY changed)

public:
    RtlOptions(ParamStore *params, VehicleHub *hub, QObject *parent);
    static RtlOptions *s_instance;
    static RtlOptions *create(QQmlEngine *, QJSEngine *) { QJSEngine::setObjectOwnership(s_instance, QJSEngine::CppOwnership); return s_instance; }

    Q_INVOKABLE void read();
    /// What the fields show now, by key: only the ones that changed are sent.
    Q_INVOKABLE void write(const QVariantMap &texts);

    QVariantList fields() const;
    bool reading() const { return m_reading; }

    struct Name {
        QString param;
        double scale = 1;   // value of the parameter per unit of the panel
    };
    /// The parameters a field may be, on a vehicle of this class (Copter, Plane, Rover, Sub) and version ("4.6.3").
    static std::vector<Name> names(const QString &key, const QString &vehicleClass, const QString &version);

signals:
    void changed();

private:
    struct Field {
        QString key, label;
        QString param;      // the one the vehicle has
        double scale = 1;
        QString text;       // as read from it
    };
    std::vector<Name> namesOf(const QString &key) const;
    void fill();
    void clear();

    ParamStore *m_params;
    VehicleHub *m_hub;
    std::vector<Field> m_fields;
    bool m_reading = false;
};

} // namespace corv
