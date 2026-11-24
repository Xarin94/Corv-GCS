#include "vehicle/ParamMeta.h"

#include "core/DataRoot.h"
#include "core/DebugLog.h"
#include "core/Inflate.h"

#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QRegularExpression>
#include <QSaveFile>
#include <QXmlStreamReader>
#include <algorithm>
#include <cmath>

using namespace Qt::StringLiterals;

namespace corv {

namespace {
QString g_downloadDir;

double toNumber(const QJsonValue &v, bool *ok)
{
    if (v.isDouble()) {
        *ok = true;
        return v.toDouble();
    }
    return v.toString().trimmed().toDouble(ok);
}

ParamInfo parseEntry(const QJsonObject &o)
{
    ParamInfo p;
    p.displayName = o.value("DisplayName"_L1).toString().trimmed();
    p.description = o.value("Description"_L1).toString().simplified();
    p.units = o.value("Units"_L1).toString().trimmed();
    p.user = o.value("User"_L1).toString().trimmed();
    const QJsonObject range = o.value("Range"_L1).toObject();
    if (!range.isEmpty()) {
        bool okLo = false, okHi = false;
        p.low = toNumber(range.value("low"_L1), &okLo);
        p.high = toNumber(range.value("high"_L1), &okHi);
        p.hasRange = okLo && okHi && p.low <= p.high;
    }
    bool ok = false;
    const double inc = toNumber(o.value("Increment"_L1), &ok);
    if (ok && inc > 0 && std::isfinite(inc)) p.increment = inc;
    const QJsonObject values = o.value("Values"_L1).toObject();
    for (auto it = values.begin(); it != values.end(); ++it) {
        const double v = it.key().trimmed().toDouble(&ok);
        if (ok) p.values.append({v, it.value().toString().trimmed()});
    }
    std::sort(p.values.begin(), p.values.end(), [](const auto &a, const auto &b) { return a.first < b.first; });
    const QJsonObject bits = o.value("Bitmask"_L1).toObject();
    for (auto it = bits.begin(); it != bits.end(); ++it) {
        const int b = it.key().trimmed().toInt(&ok);
        if (ok && b >= 0 && b < 63) p.bits.append({b, it.value().toString().trimmed()});
    }
    std::sort(p.bits.begin(), p.bits.end(), [](const auto &a, const auto &b) { return a.first < b.first; });
    const auto flag = [&](const char *key) {
        const QJsonValue v = o.value(QLatin1String(key));
        return v.isBool() ? v.toBool() : v.toString().compare("True"_L1, Qt::CaseInsensitive) == 0;
    };
    p.rebootRequired = flag("RebootRequired");
    p.readOnly = flag("ReadOnly");
    return p;
}

} // namespace

// ── ParamInfo ────────────────────────────────────────────────────────────────

QString ParamInfo::valueLabel(double v) const
{
    for (const auto &[value, label] : values)
        if (std::abs(value - v) < 1e-6) return label;
    return {};
}

QString ParamInfo::bitsLabel(qint64 v) const
{
    if (v == 0) return u"none"_s;
    QStringList names;
    for (int b = 0; b < 63; ++b) {
        if (!(v & (qint64(1) << b))) continue;
        QString name;
        for (const auto &[bit, label] : bits)
            if (bit == b) name = label;
        names << (name.isEmpty() ? u"bit %1"_s.arg(b) : name);
    }
    return names.join(u", "_s);
}

// ── ParamMeta ────────────────────────────────────────────────────────────────

ParamMeta &ParamMeta::instance()
{
    static ParamMeta m;
    return m;
}

QString ParamMeta::firmwareOf(const QString &vehicleClass)
{
    if (vehicleClass == "Copter"_L1) return u"ArduCopter"_s;
    if (vehicleClass == "Rover"_L1) return u"Rover"_s;
    if (vehicleClass == "Sub"_L1) return u"ArduSub"_s;
    return u"ArduPlane"_s;
}

QStringList ParamMeta::firmwares() { return {u"ArduPlane"_s, u"ArduCopter"_s, u"Rover"_s, u"ArduSub"_s}; }

QString ParamMeta::downloadDir() { return g_downloadDir.isEmpty() ? DataRoot::dir(u"params"_s) : g_downloadDir; }

bool ParamMeta::parse(const QByteArray &json, QHash<QString, ParamInfo> &out, QString *source, QString *error)
{
    QJsonParseError pe;
    const QJsonDocument doc = QJsonDocument::fromJson(json, &pe);
    if (!doc.isObject()) {
        if (error) *error = pe.error != QJsonParseError::NoError ? pe.errorString() : u"not a JSON object"_s;
        return false;
    }
    const QJsonObject root = doc.object();
    int n = 0;
    for (auto g = root.begin(); g != root.end(); ++g) {
        if (g.key() == "json"_L1) {
            if (source) *source = g.value().toObject().value("source"_L1).toString();
            continue;
        }
        const QJsonObject group = g.value().toObject();
        const QString release = g.key().startsWith("stable "_L1) ? g.key().mid(7) : QString();
        for (auto it = group.begin(); it != group.end(); ++it) {
            if (!it.value().isObject()) continue;
            // Older files prefix the vehicle's own parameters ("ArduPlane:THR_MAX")
            QString name = it.key();
            if (const qsizetype colon = name.indexOf(':'_L1); colon >= 0) name = name.mid(colon + 1);
            ParamInfo info = parseEntry(it.value().toObject());
            info.lastRelease = release;
            out.insert(name, info);
            ++n;
        }
    }
    if (n == 0) {
        if (error) *error = u"no parameter in the file"_s;
        return false;
    }
    return true;
}

bool ParamMeta::parseXml(const QByteArray &xml, QHash<QString, ParamInfo> &out, QString *error)
{
    QXmlStreamReader r(xml);
    ParamInfo p;
    QString name;
    int n = 0;
    while (!r.atEnd()) {
        r.readNext();
        if (r.isStartElement()) {
            const auto tag = r.name();
            const auto a = r.attributes();
            if (tag == "param"_L1) {
                p = ParamInfo{};
                name = a.value("name"_L1).toString();
                if (const qsizetype colon = name.indexOf(':'_L1); colon >= 0) name = name.mid(colon + 1);
                p.displayName = a.value("humanName"_L1).toString().trimmed();
                p.description = a.value("documentation"_L1).toString().simplified();
                p.user = a.value("user"_L1).toString().trimmed();
            } else if (tag == "field"_L1) {
                const QString key = a.value("name"_L1).toString();
                const QString text = r.readElementText(QXmlStreamReader::IncludeChildElements).trimmed();
                bool ok = false;
                if (key == "Units"_L1) p.units = text;
                else if (key == "Increment"_L1) {
                    const double inc = text.toDouble(&ok);
                    if (ok && inc > 0 && std::isfinite(inc)) p.increment = inc;
                } else if (key == "Range"_L1) {
                    static const QRegularExpression space(u"\\s+"_s);
                    const QStringList ends = text.split(space, Qt::SkipEmptyParts);
                    bool okLo = false, okHi = false;
                    if (ends.size() == 2) {
                        p.low = ends[0].toDouble(&okLo);
                        p.high = ends[1].toDouble(&okHi);
                    }
                    p.hasRange = okLo && okHi && p.low <= p.high;
                } else if (key == "RebootRequired"_L1) p.rebootRequired = text.compare("True"_L1, Qt::CaseInsensitive) == 0;
                else if (key == "ReadOnly"_L1) p.readOnly = text.compare("True"_L1, Qt::CaseInsensitive) == 0;
            } else if (tag == "value"_L1) {
                bool ok = false;
                const double v = a.value("code"_L1).toString().trimmed().toDouble(&ok);
                const QString label = r.readElementText().trimmed();
                if (ok) p.values.append({v, label});
            } else if (tag == "bit"_L1) {
                bool ok = false;
                const int b = a.value("code"_L1).toString().trimmed().toInt(&ok);
                const QString label = r.readElementText().trimmed();
                if (ok && b >= 0 && b < 63) p.bits.append({b, label});
            }
        } else if (r.isEndElement() && r.name() == "param"_L1 && !name.isEmpty()) {
            std::sort(p.values.begin(), p.values.end(), [](const auto &x, const auto &y) { return x.first < y.first; });
            std::sort(p.bits.begin(), p.bits.end(), [](const auto &x, const auto &y) { return x.first < y.first; });
            out.insert(name, p);
            ++n;
        }
    }
    if (r.hasError() || n == 0) {
        if (error) *error = r.hasError() ? r.errorString() : u"no parameter in the file"_s;
        return false;
    }
    return true;
}

void ParamMeta::setVersion(const QString &vehicleClass, const QString &version)
{
    const QString fw = firmwareOf(vehicleClass);
    if (m_versions.value(fw) == version) return;
    m_versions.insert(fw, version);
    // Read again only where a table of that release exists, or existed
    m_tables.remove(fw);
}

void ParamMeta::load(const QString &firmware, Table &t)
{
    QElapsedTimer clock;
    clock.start();
    t.loaded = true;
    t.params.clear();
    t.downloaded = false;
    // The bundled table
    QFile bundled(u":/params/%1.pdef.json.gz"_s.arg(firmware));
    if (bundled.open(QIODevice::ReadOnly)) {
        const QByteArray json = gunzip(bundled.readAll(), 2 * 1024 * 1024);
        QString src, err;
        if (parse(json, t.params, &src, &err)) t.source = src.isEmpty() ? u"bundled"_s : u"bundled · "_s + src;
        else LOG_W("params") << "bundled metadata of" << firmware << "unreadable:" << err;
    }
    // A downloaded one wins, entry by entry
    QFile dl(downloadDir() + u"/%1.pdef.json"_s.arg(firmware));
    if (dl.open(QIODevice::ReadOnly)) {
        QString err;
        if (parse(dl.readAll(), t.params, nullptr, &err)) {
            t.downloaded = true;
            t.source = u"autotest.ardupilot.org · "_s + QFileInfo(dl).lastModified().toString(u"yyyy-MM-dd"_s);
        } else {
            LOG_W("params") << dl.fileName() << "unreadable:" << err;
        }
    }
    // The release the vehicle runs wins over both
    const QString version = m_versions.value(firmware);
    QFile rel(downloadDir() + u"/%1-%2.pdef.xml"_s.arg(firmware, version));
    if (!version.isEmpty() && rel.open(QIODevice::ReadOnly)) {
        QString err;
        if (parseXml(rel.readAll(), t.params, &err)) {
            t.downloaded = true;
            t.source = u"%1 · autotest.ardupilot.org · %2"_s.arg(version, QFileInfo(rel).lastModified().toString(u"yyyy-MM-dd"_s));
        } else {
            LOG_W("params") << rel.fileName() << "unreadable:" << err;
        }
    }
    LOG_I("params") << "metadata of" << firmware << ":" << t.params.size() << "parameters," << t.source << "," << clock.elapsed() << "ms";
}

ParamMeta::Table &ParamMeta::table(const QString &firmware)
{
    Table &t = m_tables[firmware];
    if (!t.loaded) load(firmware, t);
    return t;
}

const ParamInfo *ParamMeta::find(const QString &name, const QString &vehicleClass)
{
    const QString first = firmwareOf(vehicleClass);
    {
        const Table &t = table(first);
        if (auto it = t.params.constFind(name); it != t.params.cend()) return &*it;
    }
    for (const QString &fw : firmwares()) {
        if (fw == first) continue;
        const Table &t = table(fw);
        if (auto it = t.params.constFind(name); it != t.params.cend()) return &*it;
    }
    return nullptr;
}

int ParamMeta::count(const QString &vehicleClass) { return int(table(firmwareOf(vehicleClass)).params.size()); }
QString ParamMeta::source(const QString &vehicleClass) { return table(firmwareOf(vehicleClass)).source; }

bool ParamMeta::installDownloaded(const QString &vehicleClass, const QByteArray &data, int *count, QString *error, const QString &version)
{
    QHash<QString, ParamInfo> check;
    if (version.isEmpty() ? !parse(data, check, nullptr, error) : !parseXml(data, check, error)) return false;
    const QString fw = firmwareOf(vehicleClass);
    QSaveFile f(downloadDir() + (version.isEmpty() ? u"/%1.pdef.json"_s.arg(fw) : u"/%1-%2.pdef.xml"_s.arg(fw, version)));
    if (!f.open(QIODevice::WriteOnly) || f.write(data) != data.size() || !f.commit()) {
        if (error) *error = f.errorString();
        return false;
    }
    m_tables.remove(fw);
    if (count) *count = int(check.size());
    return true;
}

void ParamMeta::removeDownloaded(const QString &vehicleClass)
{
    const QString fw = firmwareOf(vehicleClass);
    QFile::remove(downloadDir() + u"/%1.pdef.json"_s.arg(fw));
    for (const QString &f : QDir(downloadDir()).entryList({u"%1-*.pdef.xml"_s.arg(fw)}, QDir::Files)) QFile::remove(downloadDir() + '/'_L1 + f);
    m_tables.remove(fw);
}

} // namespace corv
