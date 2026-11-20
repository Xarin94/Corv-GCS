#include "core/Settings.h"

#include "core/DataRoot.h"
#include "core/DebugLog.h"

#include <QFile>
#include <QJsonDocument>
#include <QSaveFile>

using namespace Qt::StringLiterals;

namespace corv {

Settings &Settings::instance()
{
    static Settings s;
    return s;
}

void Settings::load(const QString &path)
{
    m_path = path.isEmpty() ? DataRoot::root() + u"/settings.json"_s : path;
    m_data = {};
    QFile f(m_path);
    if (f.open(QIODevice::ReadOnly)) {
        QJsonParseError err{};
        const auto doc = QJsonDocument::fromJson(f.readAll(), &err);
        if (doc.isObject()) m_data = doc.object();
        else LOG_W("settings") << "settings.json not readable:" << err.errorString();
    }
    m_saveTimer.setSingleShot(true);
    m_saveTimer.setInterval(500);
    QObject::connect(&m_saveTimer, &QTimer::timeout, this, &Settings::flush, Qt::UniqueConnection);
    m_loaded = true;
}

QVariant Settings::value(const QString &key, const QVariant &fallback) const
{
    const auto it = m_data.constFind(key);
    return it == m_data.constEnd() ? fallback : it->toVariant();
}

void Settings::setValue(const QString &key, const QVariant &value)
{
    setJson(key, QJsonValue::fromVariant(value));
}

void Settings::setJson(const QString &key, const QJsonValue &value)
{
    if (m_data.value(key) == value) return;
    m_data.insert(key, value);
    scheduleSave();
    emit changed(key);
}

void Settings::remove(const QString &key)
{
    if (!m_data.contains(key)) return;
    m_data.remove(key);
    scheduleSave();
    emit changed(key);
}

void Settings::scheduleSave()
{
    if (!m_loaded) load();
    m_saveTimer.start();
}

void Settings::flush()
{
    if (m_path.isEmpty()) return;
    m_saveTimer.stop();
    QSaveFile f(m_path);
    if (!f.open(QIODevice::WriteOnly)) {
        LOG_W("settings") << "cannot write" << m_path << f.errorString();
        return;
    }
    f.write(QJsonDocument(m_data).toJson(QJsonDocument::Indented));
    if (!f.commit()) LOG_W("settings") << "cannot save" << m_path << f.errorString();
}

} // namespace corv
