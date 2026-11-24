// Language.h - the interface in English or Simplified Chinese (port of
// js/core/i18n.js), changed while running.
//
// The interface is written in English. A dictionary (resources/i18n/zh.json) is
// keyed on the English text exactly as it is shown, runs of whitespace collapsed:
// a translator without contexts gives qsTr() the entry of the text whatever the
// file it is in, and Theme.tr() the same for the texts the view models compose.
// What has no entry - telemetry, parameter and flight mode names, units - stays as
// it is. The HUD and the documentation stay English by design.
#pragma once

#include <QHash>
#include <QObject>
#include <QSet>
#include <QTranslator>

namespace corv {

class DictionaryTranslator : public QTranslator
{
    Q_OBJECT
public:
    using QTranslator::QTranslator;
    bool loadJson(const QString &file);
    QString translate(const char *context, const char *sourceText, const char *disambiguation, int n) const override;
    bool isEmpty() const override { return m_dict.isEmpty(); }
    int size() const { return int(m_dict.size()); }
    bool contains(const QString &english) const { return m_dict.contains(english); }

private:
    QHash<QString, QString> m_dict;
    mutable QSet<QString> m_missed;   // CORV_I18N_MISSES=file: the texts shown that have no entry
};

class Language : public QObject
{
    Q_OBJECT
public:
    static Language &instance();
    /// "en" or "zh"; anything else is English.
    void set(const QString &code);
    QString code() const { return m_code; }

signals:
    void changed();

private:
    Language() = default;
    QString m_code = QStringLiteral("en");
    DictionaryTranslator m_zh;
};

} // namespace corv
