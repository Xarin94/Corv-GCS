// ParamCatalog.h - parameter names known before any download, descriptions,
// value formatting (port of js/ui/ParamCatalog.js and the description table
// of ParametersPageController.js).
//
// On a slow link (LoRa, 19200 baud SiK, satellite) a full parameter download
// is minutes of airtime, so the PARAMETERS page offers a catalog of names to
// read one at a time. It merges:
//   1. a built-in seed of commonly tuned ArduPilot parameters, per vehicle class;
//   2. every name ever received from a vehicle or seen in a .param file,
//      persisted under the Electron localStorage key corv.params.learnedNames;
//   3. what is loaded now.
// Names are never invented: a typed name can be read anyway and is learned
// when the autopilot answers. Favourites persist under corv.params.favorites.
#pragma once

#include <QSet>
#include <QString>
#include <QStringList>

namespace corv {

namespace paramdata {
struct Pair {
    const char *key;
    const char *value;
};
extern const char *const kCommon[];
extern const char *const kPlane[];
extern const char *const kCopter[];
extern const char *const kRover[];
extern const char *const kSub[];
extern const Pair kDescriptions[];
extern const Pair kGroupHints[];
} // namespace paramdata

class ParamCatalog
{
public:
    static ParamCatalog &instance();

    /// Remember names (upper-cased, 16 characters at most). Returns how many were new.
    int learn(const QStringList &names);
    int learnedCount() const { return int(m_learned.size()); }
    void clearLearned();

    bool isFavorite(const QString &name) const { return m_favorites.contains(name); }
    bool toggleFavorite(const QString &name);
    QStringList favorites() const;

    /// Built-ins of the vehicle class ('Plane', 'Copter', 'Rover', 'Sub'; Plane otherwise)
    /// + common + learned + `loaded`, sorted.
    QStringList catalog(const QString &vehicleClass, const QStringList &loaded) const;
    bool isBuiltin(const QString &name) const;

    /// SERVO1_FUNCTION -> SERVO, EK3_ENABLE -> EK, Q_A_RAT_RLL_P -> Q, FLTMODE1 -> FLTMODE
    static QString groupOf(const QString &name);
    static QStringList groups(const QStringList &names);
    /// Exact description, then SERVOn_FUNCTION-style pattern, then the group hint.
    static QString description(const QString &name);
    /// Integers without decimals (ArduPilot sends REAL32 even for integer parameters),
    /// floats with up to 6 decimals and no trailing zeros (formatParamValue).
    static QString formatValue(double value, int mavType);
    /// A name as MAVLink allows it: A-Z, 0-9, underscore, 16 characters.
    static QString sanitize(const QString &raw);

private:
    ParamCatalog();
    void save(const QString &key, const QSet<QString> &set) const;

    QSet<QString> m_learned;
    QSet<QString> m_favorites;
};

} // namespace corv
