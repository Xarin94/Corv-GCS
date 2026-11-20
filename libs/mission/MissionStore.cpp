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

using namespace Qt::StringLiterals;

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
    if (!out.value(u"items"_s).isArray()) {
        if (error) *error = u"not a mission file (no items array)"_s;
        return false;
    }
    return true;
}

void summarize(const QJsonArray &items, MissionEntry &e)
{
    QList<LL> pts;
    for (const QJsonValue &v : items) {
        const QJsonObject o = v.toObject();
        const QJsonValue la = o.value(u"lat"_s), lo = o.value(u"lng"_s);
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
    static const QRegularExpression bad(u"[^\\w\\-. ]+"_s);
    static const QRegularExpression spaces(u"\\s+"_s);
    static const QRegularExpression edges(u"^[-.]+|[-.]+$"_s);
    QString base = name.trimmed();
    base.remove(bad);
    base.replace(spaces, u"-"_s);
    base.remove(edges);
    base = base.left(60);
    return base.isEmpty() ? u"mission-%1"_s.arg(QDateTime::currentMSecsSinceEpoch()) : base;
}

StoreIndex MissionStore::rebuildIndex()
{
    StoreIndex idx;
    idx.root = DataRoot::root();
    idx.portable = !DataRoot::isFallback();
    const QDir missions(DataRoot::missionsDir());
    for (const QFileInfo &fi : missions.entryInfoList({u"*.json"_s}, QDir::Files)) {
        QJsonObject data;
        QString err;
        if (!readMissionFile(fi.absoluteFilePath(), data, &err)) {
            // A corrupt or unrelated .json in the folder must not break the whole index
            LOG_W("store") << "skipping" << fi.fileName() << ":" << err;
            continue;
        }
        MissionEntry e;
        e.id = fi.completeBaseName();
        e.file = u"missions/"_s + fi.fileName();
        e.name = data.value(u"name"_s).toString(e.id);
        if (e.name.isEmpty()) e.name = e.id;
        e.notes = data.value(u"notes"_s).toString();
        e.created = data.value(u"created"_s).toString(iso(fi.birthTime().isValid() ? fi.birthTime() : fi.lastModified()));
        e.modified = iso(fi.lastModified());
        summarize(data.value(u"items"_s).toArray(), e);
        idx.missions.append(e);
    }
    std::sort(idx.missions.begin(), idx.missions.end(), [](const MissionEntry &a, const MissionEntry &b) { return a.modified > b.modified; });

    const QDir logs(DataRoot::logsDir());
    for (const QFileInfo &fi : logs.entryInfoList(QDir::Files)) {
        LogEntry l;
        l.file = u"logs/"_s + fi.fileName();
        l.name = fi.fileName();
        l.type = fi.suffix().toLower();
        l.size = fi.size();
        l.modified = iso(fi.lastModified());
        idx.logs.append(l);
    }
    std::sort(idx.logs.begin(), idx.logs.end(), [](const LogEntry &a, const LogEntry &b) { return a.modified > b.modified; });

    QJsonArray jm, jl;
    for (const MissionEntry &e : idx.missions)
        jm.append(QJsonObject{{u"id"_s, e.id}, {u"file"_s, e.file}, {u"name"_s, e.name},
                              {u"notes"_s, e.notes}, {u"created"_s, e.created}, {u"modified"_s, e.modified},
                              {u"waypoints"_s, e.waypoints}, {u"distanceM"_s, e.distanceM}});
    for (const LogEntry &l : idx.logs)
        jl.append(QJsonObject{{u"file"_s, l.file}, {u"name"_s, l.name}, {u"type"_s, l.type},
                              {u"size"_s, l.size}, {u"modified"_s, l.modified}});
    const QJsonObject index{{u"version"_s, 1}, {u"updated"_s, isoNow()}, {u"root"_s, idx.root},
                            {u"portable"_s, idx.portable}, {u"missions"_s, jm}, {u"logs"_s, jl}};
    QString err;
    if (!writeJson(DataRoot::root() + u"/index.json"_s, index, &err)) LOG_W("store") << "could not write index.json:" << err;
    return idx;
}

MissionStore::Saved MissionStore::save(const QString &id, const QString &name, const QString &notes, const QJsonArray &items,
                                       const QJsonObject &route, int vehicleType, QString *error)
{
    if (items.isEmpty()) {
        if (error) *error = u"Mission is empty"_s;
        return {};
    }
    const QString dir = DataRoot::missionsDir();
    QString targetId = id;
    QString created = isoNow();
    if (!targetId.isEmpty()) {
        QJsonObject old;
        if (readMissionFile(dir + '/'_L1 + targetId + u".json"_s, old, nullptr))
            created = old.value(u"created"_s).toString(created);
    } else {
        targetId = slugify(name);
        int k = 2;
        while (QFile::exists(dir + '/'_L1 + targetId + u".json"_s)) targetId = u"%1-%2"_s.arg(slugify(name)).arg(k++);
    }
    const QJsonObject payload{
        {u"format"_s, u"corv-gcs-mission"_s},
        {u"version"_s, 2},
        {u"name"_s, name.isEmpty() ? targetId : name},
        {u"notes"_s, notes},
        {u"created"_s, created},
        {u"modified"_s, isoNow()},
        {u"vehicleType"_s, vehicleType},
        {u"meta"_s, QJsonObject{}},
        // v2: the editable route; `items` is the compiled list, for v1 readers and the index
        {u"route"_s, route.isEmpty() ? QJsonValue(QJsonValue::Null) : QJsonValue(route)},
        {u"items"_s, items},
    };
    const QString file = dir + '/'_L1 + targetId + u".json"_s;
    if (!writeJson(file, payload, error)) return {};
    rebuildIndex();
    LOG_I("store") << u"mission saved: \"%1\" (%2 items, vehicle type %3) → %4"_s.arg(payload.value(u"name"_s).toString()).arg(items.size()).arg(vehicleType).arg(file);
    return {targetId, file, payload.value(u"name"_s).toString()};
}

QJsonObject MissionStore::load(const QString &id, QString *error)
{
    const QString file = DataRoot::missionsDir() + '/'_L1 + QFileInfo(id).fileName() + u".json"_s;
    if (!QFile::exists(file)) {
        if (error) *error = u"Mission \"%1\" not found"_s.arg(id);
        return {};
    }
    QJsonObject data;
    if (!readMissionFile(file, data, error)) return {};
    data.insert(u"id"_s, id);
    LOG_I("store") << u"mission loaded: \"%1\" (%2 items, format v%3)"_s
                          .arg(data.value(u"name"_s).toString())
                          .arg(data.value(u"items"_s).toArray().size())
                          .arg(data.value(u"version"_s).toInt(1));
    return data;
}

bool MissionStore::remove(const QString &id)
{
    const QString file = DataRoot::missionsDir() + '/'_L1 + QFileInfo(id).fileName() + u".json"_s;
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
    const Saved s = save(id, newName, data.value(u"notes"_s).toString(), data.value(u"items"_s).toArray(),
                         data.value(u"route"_s).toObject(), data.value(u"vehicleType"_s).toInt(), error);
    return !s.id.isEmpty();
}

} // namespace corv::mission
