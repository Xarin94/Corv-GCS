#include "vehicle/ParamCatalog.h"

#include "core/Settings.h"

#include <QHash>
#include <QJsonArray>
#include <QJsonDocument>
#include <QRegularExpression>
#include <algorithm>
#include <cmath>

using namespace Qt::StringLiterals;

namespace corv {

namespace {
const QString kLearnedKey = u"corv.params.learnedNames"_s;
const QString kFavKey = u"corv.params.favorites"_s;

/// The value as Electron left it (a JSON string of an array) or as this version writes it.
QSet<QString> loadSet(const QString &key)
{
    QSet<QString> out;
    QJsonArray arr;
    const QJsonValue j = Settings::instance().json(key);
    if (j.isArray()) arr = j.toArray();
    else if (j.isString()) arr = QJsonDocument::fromJson(j.toString().toUtf8()).array();
    for (const QJsonValue &v : arr)
        if (v.isString() && !v.toString().isEmpty()) out.insert(v.toString());
    return out;
}

const QSet<QString> &builtinSet(const char *const *list)
{
    static QHash<const void *, QSet<QString>> cache;
    auto it = cache.find(list);
    if (it == cache.end()) {
        QSet<QString> s;
        for (const char *const *p = list; *p; ++p) s.insert(QString::fromLatin1(*p));
        it = cache.insert(list, s);
    }
    return *it;
}

const char *const *vehicleList(const QString &vehicleClass)
{
    if (vehicleClass == "Copter"_L1) return paramdata::kCopter;
    if (vehicleClass == "Rover"_L1) return paramdata::kRover;
    if (vehicleClass == "Sub"_L1) return paramdata::kSub;
    return paramdata::kPlane;
}

const QHash<QString, QString> &table(const paramdata::Pair *pairs)
{
    static QHash<const void *, QHash<QString, QString>> cache;
    auto it = cache.find(pairs);
    if (it == cache.end()) {
        QHash<QString, QString> h;
        for (const paramdata::Pair *p = pairs; p->key; ++p) h.insert(QString::fromLatin1(p->key), QString::fromUtf8(p->value));
        it = cache.insert(pairs, h);
    }
    return *it;
}
} // namespace

ParamCatalog &ParamCatalog::instance()
{
    static ParamCatalog c;
    return c;
}

ParamCatalog::ParamCatalog() : m_learned(loadSet(kLearnedKey)), m_favorites(loadSet(kFavKey)) {}

void ParamCatalog::save(const QString &key, const QSet<QString> &set) const
{
    QStringList sorted(set.cbegin(), set.cend());
    sorted.sort();
    Settings::instance().setJson(key, QJsonArray::fromStringList(sorted));
}

int ParamCatalog::learn(const QStringList &names)
{
    int added = 0;
    for (const QString &raw : names) {
        const QString name = raw.trimmed().toUpper();
        if (name.isEmpty() || name.size() > 16) continue;
        if (!m_learned.contains(name)) {
            m_learned.insert(name);
            ++added;
        }
    }
    if (added) save(kLearnedKey, m_learned);
    return added;
}

bool ParamCatalog::toggleFavorite(const QString &name)
{
    if (!m_favorites.remove(name)) m_favorites.insert(name);
    save(kFavKey, m_favorites);
    return m_favorites.contains(name);
}

QStringList ParamCatalog::favorites() const
{
    QStringList l(m_favorites.cbegin(), m_favorites.cend());
    l.sort();
    return l;
}

QStringList ParamCatalog::catalog(const QString &vehicleClass, const QStringList &loaded) const
{
    QSet<QString> set = builtinSet(paramdata::kCommon);
    set.unite(builtinSet(vehicleList(vehicleClass)));
    set.unite(m_learned);
    for (const QString &n : loaded) set.insert(n);
    QStringList l(set.cbegin(), set.cend());
    l.sort();
    return l;
}

QString ParamCatalog::groupOf(const QString &name)
{
    const int i = name.indexOf('_'_L1);
    const QString head = i > 0 ? name.left(i) : name;
    QString stripped = head;
    while (!stripped.isEmpty() && stripped.back().isDigit()) stripped.chop(1);
    return stripped.size() >= 2 ? stripped : head;
}

QStringList ParamCatalog::groups(const QStringList &names)
{
    QSet<QString> set;
    for (const QString &n : names) set.insert(groupOf(n));
    QStringList l(set.cbegin(), set.cend());
    l.sort();
    return l;
}

QString ParamCatalog::description(const QString &name)
{
    const auto &desc = table(paramdata::kDescriptions);
    if (auto it = desc.constFind(name); it != desc.cend()) return *it;
    // Indexed parameters: SERVO5_FUNCTION reads like SERVO1_FUNCTION (first digit run -> n)
    static const QRegularExpression digits(u"\\d+"_s);
    const auto first = digits.match(name);
    if (first.hasMatch()) {
        QString base = name;
        base.replace(first.capturedStart(), first.capturedLength(), u"n"_s);
        for (auto it = desc.cbegin(); it != desc.cend(); ++it) {
            QString k = it.key();
            const auto m = digits.match(k);
            if (m.hasMatch()) k.replace(m.capturedStart(), m.capturedLength(), u"n"_s);
            if (k == base) return it.value();
        }
    }
    return table(paramdata::kGroupHints).value(name.section('_'_L1, 0, 0));
}

QString ParamCatalog::formatValue(double value, int mavType)
{
    if (!std::isfinite(value)) return QString::number(value);
    if (mavType >= 1 && mavType <= 6) return QString::number(std::llround(value));
    if (std::abs(value - std::round(value)) < 1e-4) return QString::number(std::llround(value));
    QString s = QString::number(value, 'f', 6);
    while (s.endsWith('0'_L1)) s.chop(1);
    if (s.endsWith('.'_L1)) s.chop(1);
    return s.isEmpty() ? u"0"_s : s;
}

QString ParamCatalog::sanitize(const QString &raw)
{
    QString out;
    for (QChar c : raw.toUpper())
        if ((c >= 'A'_L1 && c <= 'Z'_L1) || c.isDigit() || c == '_'_L1) out += c;
    return out.left(16);
}

} // namespace corv
