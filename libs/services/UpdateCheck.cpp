#include "services/UpdateCheck.h"

#include <QJsonObject>
#include <QRegularExpression>
#include <QSysInfo>
#include <algorithm>

namespace corv::update {

QString Version::toString() const
{
    QString s = QStringLiteral("%1.%2.%3").arg(major).arg(minor).arg(patch);
    if (!pre.isEmpty()) s += QLatin1Char('-') + pre.join(QLatin1Char('.'));
    return s;
}

std::optional<Version> parseVersion(QString text)
{
    text = text.trimmed();
    if (text.startsWith(QLatin1Char('v')) || text.startsWith(QLatin1Char('V'))) text.remove(0, 1);
    text = text.section(QLatin1Char('+'), 0, 0);
    static const QRegularExpression re(QStringLiteral("^(\\d+)\\.(\\d+)(?:\\.(\\d+))?(?:-([0-9A-Za-z.-]+))?$"));
    const auto m = re.match(text);
    if (!m.hasMatch()) return std::nullopt;
    Version v;
    v.major = m.captured(1).toInt();
    v.minor = m.captured(2).toInt();
    v.patch = m.captured(3).isEmpty() ? 0 : m.captured(3).toInt();
    if (!m.captured(4).isEmpty()) v.pre = m.captured(4).split(QLatin1Char('.'), Qt::SkipEmptyParts);
    return v;
}

int compare(const Version &a, const Version &b)
{
    const int core[3][2] = {{a.major, b.major}, {a.minor, b.minor}, {a.patch, b.patch}};
    for (const auto &c : core)
        if (c[0] != c[1]) return c[0] < c[1] ? -1 : 1;
    // A final release outranks its pre-releases
    if (a.pre.isEmpty() || b.pre.isEmpty()) return a.pre.isEmpty() == b.pre.isEmpty() ? 0 : a.pre.isEmpty() ? 1 : -1;
    for (int i = 0; i < std::min(a.pre.size(), b.pre.size()); ++i) {
        bool na = false, nb = false;
        const qlonglong ia = a.pre[i].toLongLong(&na), ib = b.pre[i].toLongLong(&nb);
        if (na && nb) {
            if (ia != ib) return ia < ib ? -1 : 1;
            continue;
        }
        // Numeric identifiers rank below alphanumeric ones
        if (na != nb) return na ? -1 : 1;
        const int c = QString::compare(a.pre[i], b.pre[i]);
        if (c) return c < 0 ? -1 : 1;
    }
    if (a.pre.size() != b.pre.size()) return a.pre.size() < b.pre.size() ? -1 : 1;
    return 0;
}

Platform currentPlatform()
{
#if defined(Q_OS_WIN)
    return Platform::Windows;
#elif defined(Q_OS_MACOS)
    return Platform::MacOS;
#else
    return Platform::Linux;
#endif
}

QString currentArch()
{
    const QString a = QSysInfo::currentCpuArchitecture();
    return a == QLatin1String("arm64") || a == QLatin1String("aarch64") ? QStringLiteral("arm64") : QStringLiteral("x86_64");
}

std::optional<Asset> packageFor(const Release &r, Platform p, const QString &arch)
{
    // Processor names as they appear in file names
    const QStringList mine = arch == QLatin1String("arm64") ? QStringList{QStringLiteral("arm64"), QStringLiteral("aarch64")}
                                                            : QStringList{QStringLiteral("x86_64"), QStringLiteral("x64"), QStringLiteral("amd64")};
    const QStringList other = arch == QLatin1String("arm64") ? QStringList{QStringLiteral("x86_64"), QStringLiteral("x64"), QStringLiteral("amd64")}
                                                             : QStringList{QStringLiteral("arm64"), QStringLiteral("aarch64")};
    const auto has = [](const QString &name, const QStringList &tokens) {
        for (const QString &t : tokens)
            if (name.contains(t, Qt::CaseInsensitive)) return true;
        return false;
    };
    const auto ends = [](const QString &name, const char *ext) { return name.endsWith(QLatin1String(ext), Qt::CaseInsensitive); };
    std::optional<Asset> generic;
    for (const Asset &a : r.assets) {
        bool kind = false;
        switch (p) {
        case Platform::Windows: kind = ends(a.name, ".exe") && a.name.contains(QLatin1String("setup"), Qt::CaseInsensitive); break;
        case Platform::MacOS: kind = ends(a.name, ".dmg"); break;
        case Platform::Linux: kind = ends(a.name, ".AppImage"); break;
        }
        if (!kind || has(a.name, other)) continue;
        // One built for this processor (or universal) first, else one with no processor in its name
        if (has(a.name, mine) || a.name.contains(QLatin1String("universal"), Qt::CaseInsensitive)) return a;
        if (!generic) generic = a;
    }
    return generic;
}

std::optional<Release> newest(const QJsonArray &releases, const Version &current, bool includePre, Platform p,
                              const QString &arch)
{
    std::optional<Release> best;
    for (const QJsonValue &v : releases) {
        const QJsonObject o = v.toObject();
        if (o.value(QLatin1String("draft")).toBool()) continue;
        const bool pre = o.value(QLatin1String("prerelease")).toBool();
        const auto ver = parseVersion(o.value(QLatin1String("tag_name")).toString());
        if (!ver || (pre && !includePre) || (ver->isPrerelease() && !includePre)) continue;
        if (compare(*ver, current) <= 0 || (best && compare(*ver, best->version) <= 0)) continue;
        Release r;
        r.tag = o.value(QLatin1String("tag_name")).toString();
        r.name = o.value(QLatin1String("name")).toString();
        r.notes = o.value(QLatin1String("body")).toString();
        r.page = o.value(QLatin1String("html_url")).toString();
        r.version = *ver;
        r.prerelease = pre;
        for (const QJsonValue &av : o.value(QLatin1String("assets")).toArray()) {
            const QJsonObject a = av.toObject();
            Asset as;
            as.name = a.value(QLatin1String("name")).toString();
            as.url = a.value(QLatin1String("browser_download_url")).toString();
            as.size = qint64(a.value(QLatin1String("size")).toDouble());
            const QString digest = a.value(QLatin1String("digest")).toString();
            if (digest.startsWith(QLatin1String("sha256:"))) as.sha256 = digest.mid(7).toLower();
            r.assets << as;
        }
        if (!packageFor(r, p, arch)) continue;
        best = r;
    }
    return best;
}

} // namespace corv::update
