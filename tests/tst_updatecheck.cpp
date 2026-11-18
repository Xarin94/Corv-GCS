// UpdateCheck: semantic-version order, and the release and package picked
// from a GitHub release list for each system.
#include "services/UpdateCheck.h"

#include <QJsonDocument>
#include <QJsonObject>
#include <QTest>

using namespace corv;
using namespace corv::update;

namespace {

Version v(const char *s)
{
    const auto r = parseVersion(QString::fromLatin1(s));
    return r ? *r : Version{-1, -1, -1, {}};
}

QJsonObject asset(const QString &name, const QString &digest = {})
{
    QJsonObject a{{"name", name}, {"browser_download_url", "https://example.invalid/" + name}, {"size", 1000}};
    if (!digest.isEmpty()) a.insert("digest", digest);
    return a;
}

QJsonObject release(const QString &tag, bool pre, const QJsonArray &assets, bool draft = false)
{
    return {{"tag_name", tag}, {"name", "CORV GCS " + tag}, {"prerelease", pre}, {"draft", draft},
            {"body", "notes"}, {"html_url", "https://example.invalid/" + tag}, {"assets", assets}};
}

// The three packages the CI attaches, plus the Electron 1.8.8 ones
QJsonArray fullSet(const QString &label)
{
    return {asset("CORV-GCS-2.0.0-setup-x64.exe", "sha256:ABCDEF"), asset("CORV-GCS-2.0.0-windows-x64.zip"),
            asset("CORV-GCS-" + label + "-x86_64.AppImage"), asset("CORV-GCS-" + label + "-macos-universal.dmg")};
}

} // namespace

class TestUpdateCheck : public QObject
{
    Q_OBJECT
private slots:
    void parse()
    {
        QCOMPARE(v("v2.0.0-alpha.7").toString(), QStringLiteral("2.0.0-alpha.7"));
        QCOMPARE(v("1.8.8").toString(), QStringLiteral("1.8.8"));
        QCOMPARE(v("2.1").toString(), QStringLiteral("2.1.0"));
        QCOMPARE(v("2.0.0+build.5").toString(), QStringLiteral("2.0.0"));
        QVERIFY(!parseVersion(QStringLiteral("latest")));
        QVERIFY(!parseVersion(QStringLiteral("v2")));
    }

    void order_data()
    {
        QTest::addColumn<QString>("lower");
        QTest::addColumn<QString>("higher");
        QTest::addRow("patch") << "1.8.8" << "1.8.9";
        QTest::addRow("1.x to 2 alpha") << "1.8.8" << "2.0.0-alpha.1";
        QTest::addRow("alpha numeric") << "2.0.0-alpha.7" << "2.0.0-alpha.10";
        QTest::addRow("alpha to beta") << "2.0.0-alpha.9" << "2.0.0-beta.1";
        QTest::addRow("pre to final") << "2.0.0-rc.2" << "2.0.0";
        QTest::addRow("dev above alphas") << "2.0.0-alpha.99" << "2.0.0-dev";
        QTest::addRow("dev below final") << "2.0.0-dev" << "2.0.0";
        QTest::addRow("shorter pre first") << "2.0.0-alpha" << "2.0.0-alpha.1";
    }
    void order()
    {
        QFETCH(QString, lower);
        QFETCH(QString, higher);
        const auto a = parseVersion(lower), b = parseVersion(higher);
        QVERIFY(a && b);
        QCOMPARE(compare(*a, *b), -1);
        QCOMPARE(compare(*b, *a), 1);
        QCOMPARE(compare(*a, *a), 0);
    }

    void newestPicksPackage()
    {
        const QJsonArray list{release("v2.0.0-alpha.9", true, fullSet("2.0.0-alpha.9")),
                              release("v2.0.0-alpha.10", true, fullSet("2.0.0-alpha.10"), true),   // draft
                              release("v2.0.0-alpha.8", true, fullSet("2.0.0-alpha.8")),
                              release("v1.8.8", false, {asset("CORV.GCS.Setup.1.8.8.exe"), asset("CORV.GCS-1.8.8.AppImage")})};
        const auto r = newest(list, v("2.0.0-alpha.7"), true, Platform::Linux, "x86_64");
        QVERIFY(r);
        QCOMPARE(r->tag, QStringLiteral("v2.0.0-alpha.9"));
        QCOMPARE(packageFor(*r, Platform::Linux, "x86_64")->name, QStringLiteral("CORV-GCS-2.0.0-alpha.9-x86_64.AppImage"));
        const auto win = packageFor(*r, Platform::Windows, "x86_64");
        QCOMPARE(win->name, QStringLiteral("CORV-GCS-2.0.0-setup-x64.exe"));
        QCOMPARE(win->sha256, QStringLiteral("abcdef"));
        QCOMPARE(packageFor(*r, Platform::MacOS, "arm64")->name, QStringLiteral("CORV-GCS-2.0.0-alpha.9-macos-universal.dmg"));
        // Already on the newest, or past it
        QVERIFY(!newest(list, v("2.0.0-alpha.9"), true, Platform::Linux, "x86_64"));
        QVERIFY(!newest(list, v("2.0.0-dev"), true, Platform::Linux, "x86_64"));
        // A final release ignores pre-releases
        QVERIFY(!newest(list, v("1.8.8"), false, Platform::Windows, "x86_64"));
    }

    void noPackageNoUpdate()
    {
        // The newest has no macOS package yet: the one before it is offered
        QJsonArray winOnly{asset("CORV-GCS-2.0.0-setup-x64.exe")};
        const QJsonArray list{release("v2.0.0-alpha.9", true, winOnly), release("v2.0.0-alpha.8", true, fullSet("2.0.0-alpha.8"))};
        QCOMPARE(newest(list, v("2.0.0-alpha.7"), true, Platform::MacOS, "arm64")->tag, QStringLiteral("v2.0.0-alpha.8"));
        QCOMPARE(newest(list, v("2.0.0-alpha.7"), true, Platform::Windows, "x86_64")->tag, QStringLiteral("v2.0.0-alpha.9"));
    }

    void processorOfTheDmg()
    {
        // The Electron releases: one DMG per processor
        Release r;
        for (const char *n : {"CORV.GCS-1.8.8-arm64.dmg", "CORV.GCS-1.8.8-x64.dmg", "CORV.GCS-1.8.8.AppImage"}) r.assets << Asset{n, n, 1, {}};
        QCOMPARE(packageFor(r, Platform::MacOS, "arm64")->name, QStringLiteral("CORV.GCS-1.8.8-arm64.dmg"));
        QCOMPARE(packageFor(r, Platform::MacOS, "x86_64")->name, QStringLiteral("CORV.GCS-1.8.8-x64.dmg"));
        QCOMPARE(packageFor(r, Platform::Linux, "x86_64")->name, QStringLiteral("CORV.GCS-1.8.8.AppImage"));
        QVERIFY(!packageFor(r, Platform::Linux, "arm64") || packageFor(r, Platform::Linux, "arm64")->name == QStringLiteral("CORV.GCS-1.8.8.AppImage"));
        QVERIFY(!packageFor(r, Platform::Windows, "x86_64"));
    }
};

QTEST_GUILESS_MAIN(TestUpdateCheck)
#include "tst_updatecheck.moc"
