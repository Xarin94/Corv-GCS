// ParamMeta.h - what ArduPilot documents about each parameter: display name,
// description, units, range, increment, the list of values and the bits of a
// bitmask (Mission Planner's ParameterMetaData).
//
// Source: ArduPilot's apm.pdef.json (Tools/autotest/param_metadata), one per
// firmware. The copies bundled in resources/params/<Firmware>.pdef.json.gz are
// generated from the ArduPilot tree the vehicles run (tools/gen-param-meta.py),
// completed with the parameters only the stable releases still have (groups
// "stable 4.6.3": SRn_*, ARMING_CHECK, WPNAV_*...); a newer file downloaded from autotest.ardupilot.org is saved to
// <data>/params/<Firmware>.pdef.json and takes precedence, entry by entry, over
// the bundled one (names only the bundle knows, e.g. those of a fork, stay).
//
// Lookup by vehicle class ('Plane', 'Copter', 'Rover', 'Sub' as ParamCatalog
// names them): the firmware of the class first, then the others, so a
// parameter of a library is described before the vehicle type is known.
// Tables are parsed on first use (30 ms each). GUI thread.
#pragma once

#include <QHash>
#include <QList>
#include <QPair>
#include <QString>
#include <QStringList>

namespace corv {

struct ParamInfo {
    QString displayName;
    QString description;
    QString units;
    QString user;                       // Standard, Advanced
    bool hasRange = false;
    double low = 0, high = 0;
    double increment = 0;               // 0: not documented
    QList<QPair<double, QString>> values;   // sorted by value
    QList<QPair<int, QString>> bits;        // sorted by bit
    bool rebootRequired = false;
    bool readOnly = false;
    QString lastRelease;                // the last stable release that has it; empty: current firmware

    bool isBitmask() const { return !bits.isEmpty(); }
    bool hasValues() const { return !values.isEmpty(); }
    /// The label of a listed value; empty when the value is not listed.
    QString valueLabel(double v) const;
    /// The names of the bits set ("bit N" for an undocumented one); "none" for 0.
    QString bitsLabel(qint64 v) const;
    bool outOfRange(double v) const { return hasRange && (v < low - 1e-9 || v > high + 1e-9); }
};

class ParamMeta
{
public:
    static ParamMeta &instance();

    /// 'Plane' -> ArduPlane, 'Copter' -> ArduCopter, 'Rover' -> Rover, 'Sub' -> ArduSub.
    static QString firmwareOf(const QString &vehicleClass);
    static QStringList firmwares();

    /// The entry of `name`, from the firmware of the class, else from another; nullptr if undocumented.
    const ParamInfo *find(const QString &name, const QString &vehicleClass);
    /// Entries of the firmware of the class, and where they come from.
    int count(const QString &vehicleClass);
    QString source(const QString &vehicleClass);

    /// A downloaded apm.pdef.json: parsed, saved under <data>/params, used from now on.
    bool installDownloaded(const QString &vehicleClass, const QByteArray &json, int *count = nullptr, QString *error = nullptr);
    /// Back to the bundled table.
    void removeDownloaded(const QString &vehicleClass);

    /// apm.pdef.json (grouped by prefix, "json" holds the version) into a flat table.
    static bool parse(const QByteArray &json, QHash<QString, ParamInfo> &out, QString *source = nullptr, QString *error = nullptr);
    /// The folder of the downloaded tables (tests point it elsewhere).
    static QString downloadDir();

private:
    ParamMeta() = default;
    struct Table {
        bool loaded = false;
        bool downloaded = false;
        QString source;
        QHash<QString, ParamInfo> params;
    };
    Table &table(const QString &firmware);
    void load(const QString &firmware, Table &t);

    QHash<QString, Table> m_tables;
};

} // namespace corv
