#include "mission/MissionStore.h"

#include "core/DataRoot.h"
#include "core/DebugLog.h"
#include "mission/RouteGeo.h"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QRegularExpression>
#include <QSaveFile>
#include <algorithm>

namespace corv::mission {

namespace {

QString isoNow() { return QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs); }
QString iso(const QDateTime &t) { return t.toUTC().toString(Qt::ISODateWithMs); }

bool readMissionFile(const QString &path, QJsonObject &out, QString *error)
{
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
        if (error) *error = f.errorString();
        return false;
    }
    QJsonParseError pe;
    const QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &pe);
    if (!doc.isObject()) {
        if (error) *error = pe.errorString();
        return false;
    }
    out = doc.object();
    if (!out.value(QStringLiteral("items")).isArray()) {
        if (error) *error = QStringLiteral("not a mission file (no items array)");
        return false;
    }
    return true;
}

void summarize(const QJsonArray &items, MissionEntry &e)
{
    QList<LL> pts;
    for (const QJsonValue &v : items) {
        const QJsonObject o = v.toObject();
        const QJsonValue la = o.value(QStringLiteral("lat")), lo = o.value(QStringLiteral("lng"));
        if (la.isDouble() && lo.isDouble()) pts.append({la.toDouble(), lo.toDouble()});
    }
    e.waypoints = int(items.size());
    e.distanceM = std::round(pathLength(pts));
}

bool writeJson(const QString &path, const QJsonObject &obj, QString *error)
{
    // Temp file and rename: a crash mid-write must not destroy the mission being overwritten
    QSaveFile f(path);
    if (!f.open(QIODevice::WriteOnly)) {
        if (error) *error = f.errorString();
        return false;
    }
    f.write(QJsonDocument(obj).toJson(QJsonDocument::Indented));
    if (!f.commit()) {
        if (error) *error = f.errorString();
        return false;
    }
    return true;
}

} // namespace

QString MissionStore::slugify(const QString &name)
{
    static const QRegularExpression bad(QStringLiteral("[^\\w\\-. ]+"));
    static const QRegularExpression spaces(QStringLiteral("\\s+"));
    static const QRegularExpression edges(QStringLiteral("^[-.]+|[-.]+$"));
    QString base = name.trimmed();
    base.remove(bad);
    base.replace(spaces, QStringLiteral("-"));
    base.remove(edges);
    base = base.left(60);
    return base.isEmpty() ? QStringLiteral("mission-%1").arg(QDateTime::currentMSecsSinceEpoch()) : base;
}

StoreIndex MissionStore::rebuildIndex()
{
    StoreIndex idx;
    idx.root = DataRoot::root();
    idx.portable = !DataRoot::isFallback();
    const QDir missions(DataRoot::missionsDir());
    for (const QFileInfo &fi : missions.entryInfoList({QStringLiteral("*.json")}, QDir::Files)) {
        QJsonObject data;
        QString err;
        if (!readMissionFile(fi.absoluteFilePath(), data, &err)) {
            // A corrupt or unrelated .json in the folder must not break the whole index
            LOG_W("store") << "skipping" << fi.fileName() << ":" << err;
            continue;
        }
        MissionEntry e;
        e.id = fi.completeBaseName();
        e.file = QStringLiteral("missions/") + fi.fileName();
        e.name = data.value(QStringLiteral("name")).toString(e.id);
        if (e.name.isEmpty()) e.name = e.id;
        e.notes = data.value(QStringLiteral("notes")).toString();
        e.created = data.value(QStringLiteral("created")).toString(iso(fi.birthTime().isValid() ? fi.birthTime() : fi.lastModified()));
        e.modified = iso(fi.lastModified());
        summarize(data.value(QStringLiteral("items")).toArray(), e);
        idx.missions.append(e);
    }
    std::sort(idx.missions.begin(), idx.missions.end(), [](const MissionEntry &a, const MissionEntry &b) { return a.modified > b.modified; });

    const QDir logs(DataRoot::logsDir());
    for (const QFileInfo &fi : logs.entryInfoList(QDir::Files)) {
        LogEntry l;
        l.file = QStringLiteral("logs/") + fi.fileName();
        l.name = fi.fileName();
        l.type = fi.suffix().toLower();
        l.size = fi.size();
        l.modified = iso(fi.lastModified());
        idx.logs.append(l);
    }
    std::sort(idx.logs.begin(), idx.logs.end(), [](const LogEntry &a, const LogEntry &b) { return a.modified > b.modified; });

    QJsonArray jm, jl;
    for (const MissionEntry &e : idx.missions)
        jm.append(QJsonObject{{QStringLiteral("id"), e.id}, {QStringLiteral("file"), e.file}, {QStringLiteral("name"), e.name},
                              {QStringLiteral("notes"), e.notes}, {QStringLiteral("created"), e.created}, {QStringLiteral("modified"), e.modified},
                              {QStringLiteral("waypoints"), e.waypoints}, {QStringLiteral("distanceM"), e.distanceM}});
    for (const LogEntry &l : idx.logs)
        jl.append(QJsonObject{{QStringLiteral("file"), l.file}, {QStringLiteral("name"), l.name}, {QStringLiteral("type"), l.type},
                              {QStringLiteral("size"), l.size}, {QStringLiteral("modified"), l.modified}});
    const QJsonObject index{{QStringLiteral("version"), 1}, {QStringLiteral("updated"), isoNow()}, {QStringLiteral("root"), idx.root},
                            {QStringLiteral("portable"), idx.portable}, {QStringLiteral("missions"), jm}, {QStringLiteral("logs"), jl}};
    QString err;
    if (!writeJson(DataRoot::root() + QStringLiteral("/index.json"), index, &err)) LOG_W("store") << "could not write index.json:" << err;
    return idx;
}

MissionStore::Saved MissionStore::save(const QString &id, const QString &name, const QString &notes, const QJsonArray &items,
                                       const QJsonObject &route, int vehicleType, QString *error)
{
    if (items.isEmpty()) {
        if (error) *error = QStringLiteral("Mission is empty");
        return {};
    }
    const QString dir = DataRoot::missionsDir();
    QString targetId = id;
    QString created = isoNow();
    if (!targetId.isEmpty()) {
        QJsonObject old;
        if (readMissionFile(dir + QLatin1Char('/') + targetId + QStringLiteral(".json"), old, nullptr))
            created = old.value(QStringLiteral("created")).toString(created);
    } else {
        targetId = slugify(name);
        int k = 2;
        while (QFile::exists(dir + QLatin1Char('/') + targetId + QStringLiteral(".json"))) targetId = QStringLiteral("%1-%2").arg(slugify(name)).arg(k++);
    }
    const QJsonObject payload{
        {QStringLiteral("format"), QStringLiteral("corv-gcs-mission")},
        {QStringLiteral("version"), 2},
        {QStringLiteral("name"), name.isEmpty() ? targetId : name},
        {QStringLiteral("notes"), notes},
        {QStringLiteral("created"), created},
        {QStringLiteral("modified"), isoNow()},
        {QStringLiteral("vehicleType"), vehicleType},
        {QStringLiteral("meta"), QJsonObject{}},
        // v2: the editable route; `items` is the compiled list, for v1 readers and the index
        {QStringLiteral("route"), route.isEmpty() ? QJsonValue(QJsonValue::Null) : QJsonValue(route)},
        {QStringLiteral("items"), items},
    };
    const QString file = dir + QLatin1Char('/') + targetId + QStringLiteral(".json");
    if (!writeJson(file, payload, error)) return {};
    rebuildIndex();
    LOG_I("store") << QStringLiteral("mission saved: \"%1\" (%2 items, vehicle type %3) → %4").arg(payload.value(QStringLiteral("name")).toString()).arg(items.size()).arg(vehicleType).arg(file);
    return {targetId, file, payload.value(QStringLiteral("name")).toString()};
}

QJsonObject MissionStore::load(const QString &id, QString *error)
{
    const QString file = DataRoot::missionsDir() + QLatin1Char('/') + QFileInfo(id).fileName() + QStringLiteral(".json");
    if (!QFile::exists(file)) {
        if (error) *error = QStringLiteral("Mission \"%1\" not found").arg(id);
        return {};
    }
    QJsonObject data;
    if (!readMissionFile(file, data, error)) return {};
    data.insert(QStringLiteral("id"), id);
    LOG_I("store") << QStringLiteral("mission loaded: \"%1\" (%2 items, format v%3)")
                          .arg(data.value(QStringLiteral("name")).toString())
                          .arg(data.value(QStringLiteral("items")).toArray().size())
                          .arg(data.value(QStringLiteral("version")).toInt(1));
    return data;
}

bool MissionStore::remove(const QString &id)
{
    const QString file = DataRoot::missionsDir() + QLatin1Char('/') + QFileInfo(id).fileName() + QStringLiteral(".json");
    if (QFile::exists(file)) QFile::remove(file);
    LOG_I("store") << "mission deleted:" << id;
    rebuildIndex();
    return true;
}

bool MissionStore::rename(const QString &id, const QString &newName, QString *error)
{
    const QJsonObject data = load(id, error);
    if (data.isEmpty()) return false;
    // The Electron version drops the route on rename; keeping it loses nothing
    const Saved s = save(id, newName, data.value(QStringLiteral("notes")).toString(), data.value(QStringLiteral("items")).toArray(),
                         data.value(QStringLiteral("route")).toObject(), data.value(QStringLiteral("vehicleType")).toInt(), error);
    return !s.id.isEmpty();
}

} // namespace corv::mission
