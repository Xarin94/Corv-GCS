// Settings.h - what the Electron version kept in localStorage, now a JSON file.
//
// <data root>/settings.json holds one flat object, keyed like the old
// localStorage keys (hud cells, joystick mapping, FPV, LiDAR, ROS, last home,
// radio profile, flight stack, language...). The last Electron release exports
// its localStorage into that file; this version reads it on first start.
// Writes are debounced and go through QSaveFile (temp file + rename).
#pragma once

#include <QJsonObject>
#include <QJsonValue>
#include <QObject>
#include <QTimer>
#include <QVariant>

namespace corv {

class Settings : public QObject
{
    Q_OBJECT
public:
    static Settings &instance();

    /// Load from a specific file (tests); the app uses <data root>/settings.json.
    void load(const QString &path = {});
    QString path() const { return m_path; }

    Q_INVOKABLE QVariant value(const QString &key, const QVariant &fallback = {}) const;
    Q_INVOKABLE void setValue(const QString &key, const QVariant &value);
    Q_INVOKABLE void remove(const QString &key);
    bool contains(const QString &key) const { return m_data.contains(key); }

    QJsonValue json(const QString &key) const { return m_data.value(key); }
    void setJson(const QString &key, const QJsonValue &value);

    /// Write now (quit path).
    void flush();

signals:
    void changed(const QString &key);

private:
    Settings() = default;
    void scheduleSave();

    QString m_path;
    QJsonObject m_data;
    QTimer m_saveTimer;
    bool m_loaded = false;
};

} // namespace corv
