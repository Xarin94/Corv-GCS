#include "ui/Language.h"

#include "core/DebugLog.h"

#include <QCoreApplication>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>

using namespace Qt::StringLiterals;

namespace corv {

bool DictionaryTranslator::loadJson(const QString &file)
{
    QFile f(file);
    if (!f.open(QIODevice::ReadOnly)) return false;
    QJsonParseError error;
    const QJsonObject o = QJsonDocument::fromJson(f.readAll(), &error).object();
    if (o.isEmpty()) {
        LOG_E("i18n") << file << error.errorString();
        return false;
    }
    m_dict.clear();
    m_dict.reserve(o.size());
    for (auto it = o.begin(); it != o.end(); ++it) m_dict.insert(it.key().simplified(), it.value().toString());
    return true;
}

QString DictionaryTranslator::translate(const char *, const char *sourceText, const char *, int) const
{
    if (!sourceText || !*sourceText) return {};
    const QString raw = QString::fromUtf8(sourceText);
    const QString key = raw.simplified();
    const auto it = m_dict.constFind(key);
    if (it == m_dict.cend()) {
        static const QString missesFile = qEnvironmentVariable("CORV_I18N_MISSES");
        if (!missesFile.isEmpty() && !m_missed.contains(key)) {
            m_missed.insert(key);
            QFile f(missesFile);
            if (f.open(QIODevice::Append)) f.write(QJsonDocument(QJsonObject{{u"t"_s, key}}).toJson(QJsonDocument::Compact) + '\n');
        }
        return {};
    }
    // The spaces around the text are layout, not text
    qsizetype lead = 0, trail = raw.size();
    while (lead < raw.size() && raw[lead].isSpace()) ++lead;
    while (trail > lead && raw[trail - 1].isSpace()) --trail;
    return raw.left(lead) + it.value() + raw.mid(trail);
}

Language &Language::instance()
{
    static Language l;
    return l;
}

void Language::set(const QString &code)
{
    const QString next = code == "zh"_L1 ? u"zh"_s : u"en"_s;
    if (next == m_code) return;
    QCoreApplication::removeTranslator(&m_zh);
    if (next == "zh"_L1) {
        if (m_zh.isEmpty() && !m_zh.loadJson(u":/i18n/zh.json"_s)) return;
        QCoreApplication::installTranslator(&m_zh);
    }
    m_code = next;
    LOG_I("i18n") << "language" << m_code;
    emit changed();
}

} // namespace corv
