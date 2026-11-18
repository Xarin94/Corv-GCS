// CORV GCS 2.0 - Qt Quick application entry point.
//
//   corv-gcs [--connect tcp:HOST:PORT | udp:HOST:PORT | serial:PORT:BAUD]
//
// --connect opens a link at start (SITL tests), as the CONNECT button would.
#include "DebugServer.h"
#include "TestDriver.h"
#include "core/DataRoot.h"
#include "core/DebugLog.h"
#include "core/Settings.h"
#include "terrain/TileStore.h"
#include "ui/ReplayController.h"
#include "ui/GcsController.h"
#include "ui/PlanController.h"

#include <QCommandLineParser>
#include <QDir>
#include <QFile>
#include <QFontDatabase>
#include <QGuiApplication>
#include <QPalette>
#include <QStyleHints>
#include <QIcon>
#include <QQmlApplicationEngine>
#include <QQuickStyle>
#include <QQuickWindow>
#include <QTimer>
#include <memory>
#include <QtQml/qqmlextensionplugin.h>

Q_IMPORT_QML_PLUGIN(Corv_UiPlugin)
Q_IMPORT_QML_PLUGIN(Corv_GpuPlugin)

#ifdef Q_OS_WIN
// Laptops: ask the driver for the dedicated GPU (was --force_high_performance_gpu in Electron)
extern "C" {
__declspec(dllexport) unsigned long NvOptimusEnablement = 1;
__declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
}
#endif

using namespace corv;

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);
    QGuiApplication::setApplicationName(QStringLiteral(CORV_APP_NAME));
    QGuiApplication::setApplicationVersion(QStringLiteral(CORV_VERSION));
    QGuiApplication::setOrganizationName(QString());
    app.setWindowIcon(QIcon(QStringLiteral(":/icons/icon-256x256.png")));

    QCommandLineParser p;
    p.addHelpOption();
    p.addVersionOption();
    QCommandLineOption connectOpt(QStringLiteral("connect"), QStringLiteral("Open a link at start: tcp:HOST:PORT, udp:HOST:PORT, serial:PORT:BAUD"), QStringLiteral("link"));
    QCommandLineOption grabOpt(QStringLiteral("grab"), QStringLiteral("Save the window as an image after --grab-after ms (tests)"), QStringLiteral("file"));
    QCommandLineOption grabAfterOpt(QStringLiteral("grab-after"), QStringLiteral("Delay before --grab, ms"), QStringLiteral("ms"), QStringLiteral("10000"));
    QCommandLineOption quitAfterOpt(QStringLiteral("quit-after"), QStringLiteral("Quit after this many ms (tests)"), QStringLiteral("ms"));
    QCommandLineOption tabOpt(QStringLiteral("tab"), QStringLiteral("Open this tab at start: 0 FLIGHT DATA, 1 FLIGHT PLAN, 2 SETUP, 3 SYS CONFIG (tests)"), QStringLiteral("index"));
    QCommandLineOption importOpt(QStringLiteral("import"), QStringLiteral("Import a route file into FLIGHT PLAN at start (tests)"), QStringLiteral("file"));
    QCommandLineOption replayOpt(QStringLiteral("replay"), QStringLiteral("Load a .tlog or .bin into LOG REPLAY at start (tests)"), QStringLiteral("file"));
    QCommandLineOption grabsOpt(QStringLiteral("grab-script"), QStringLiteral("Tests: a file of lines 'ms tab N' / 'ms grab FILE' / 'ms select INDEX' / 'ms emit toggleSatellite' run in order"), QStringLiteral("file"));
    QCommandLineOption debugPortOpt(QStringLiteral("debug-port"), QStringLiteral("Tests: a JSON-RPC debug server on 127.0.0.1:PORT (set, get, call, stats, screenshot, script)"), QStringLiteral("port"));
    p.addOptions({connectOpt, grabOpt, grabAfterOpt, quitAfterOpt, tabOpt, importOpt, replayOpt, grabsOpt, debugPortOpt});
    p.process(app);

    DebugLog::init();
    Settings::instance().load();
    LOG_I("app") << "CORV GCS" << CORV_VERSION << "starting, data root" << DataRoot::root();

    // Fonts of the Electron UI (Rajdhani for labels, Roboto Mono for data)
    for (const QString &f : QDir(QStringLiteral(":/fonts")).entryList())
        if (QFontDatabase::addApplicationFont(QStringLiteral(":/fonts/") + f) < 0)
            LOG_D("app") << "font not loaded:" << f;

    QQuickStyle::setStyle(QStringLiteral("Basic"));
    // The interface is dark whatever the desktop theme: the Basic style draws what the
    // QML does not style (lists of selects, tooltips, menus, the non-native file
    // dialog) with the application palette, which otherwise follows the system and
    // gave black text on black lists on a dark Windows / KDE theme.
    QGuiApplication::styleHints()->setColorScheme(Qt::ColorScheme::Dark);
    {
        QPalette pal;
        const QColor bg(0x0a, 0x0f, 0x16), base(0x06, 0x0a, 0x10), text(0xff, 0xff, 0xff), dim(0x6e, 0x7f, 0x8d);
        const QColor button(0x14, 0x1e, 0x2d), accent(0x00, 0xd2, 0xff);
        for (QPalette::ColorGroup g : {QPalette::Active, QPalette::Inactive, QPalette::Disabled}) {
            const bool off = g == QPalette::Disabled;
            pal.setColor(g, QPalette::Window, bg);
            pal.setColor(g, QPalette::WindowText, off ? dim : text);
            pal.setColor(g, QPalette::Base, base);
            pal.setColor(g, QPalette::AlternateBase, bg);
            pal.setColor(g, QPalette::Text, off ? dim : text);
            pal.setColor(g, QPalette::PlaceholderText, dim);
            pal.setColor(g, QPalette::Button, button);
            pal.setColor(g, QPalette::ButtonText, off ? dim : text);
            pal.setColor(g, QPalette::BrightText, text);
            pal.setColor(g, QPalette::Highlight, QColor(0x00, 0x5a, 0x70));
            pal.setColor(g, QPalette::HighlightedText, text);
            pal.setColor(g, QPalette::ToolTipBase, QColor(0x10, 0x18, 0x22));
            pal.setColor(g, QPalette::ToolTipText, text);
            pal.setColor(g, QPalette::Link, accent);
            pal.setColor(g, QPalette::Light, QColor(0x2a, 0x36, 0x46));
            pal.setColor(g, QPalette::Midlight, QColor(0x1e, 0x28, 0x36));
            pal.setColor(g, QPalette::Mid, QColor(0x16, 0x1e, 0x2a));
            pal.setColor(g, QPalette::Dark, QColor(0x04, 0x08, 0x0e));
            pal.setColor(g, QPalette::Shadow, Qt::black);
        }
        QGuiApplication::setPalette(pal);
    }
    // The tile store belongs to the GUI thread: create it here, before a map paints on the render thread
    TileStore::instance();

    GcsController gcs(nullptr);
    QQmlApplicationEngine engine;
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreationFailed, &app, [] { QCoreApplication::exit(1); }, Qt::QueuedConnection);
    engine.loadFromModule("Corv.Ui", "Main");

    if (auto *w = qobject_cast<QQuickWindow *>(engine.rootObjects().value(0))) {
        const auto api = w->rendererInterface()->graphicsApi();
        static const QHash<int, QString> names = {{QSGRendererInterface::Direct3D11, QStringLiteral("Direct3D 11")},
                                                 {QSGRendererInterface::Direct3D12, QStringLiteral("Direct3D 12")},
                                                 {QSGRendererInterface::Vulkan, QStringLiteral("Vulkan")},
                                                 {QSGRendererInterface::Metal, QStringLiteral("Metal")},
                                                 {QSGRendererInterface::OpenGL, QStringLiteral("OpenGL")},
                                                 {QSGRendererInterface::Software, QStringLiteral("software")}};
        DebugLog::setHeader(QStringLiteral("Graphics"), names.value(api, QStringLiteral("other")));
    }

    if (p.isSet(connectOpt)) {
        const QStringList parts = p.value(connectOpt).split(QLatin1Char(':'));
        if (parts.size() == 3) {
            const QString t = parts[0].toUpper();
            QTimer::singleShot(300, &gcs, [&gcs, t, parts] {
                gcs.connectLink(t, parts[1], parts[2].toInt());
            });
        }
    }

    // Test hooks: an image of the window without touching the screen, and a timed quit
    if (p.isSet(grabOpt)) {
        const QString file = p.value(grabOpt);
        QTimer::singleShot(p.value(grabAfterOpt).toInt(), &app, [&engine, file] {
            if (auto *w = qobject_cast<QQuickWindow *>(engine.rootObjects().value(0))) {
                const bool ok = w->grabWindow().save(file);
                LOG_I("app") << "window grabbed to" << file << (ok ? "" : "(failed)");
            }
        });
    }
    if (p.isSet(quitAfterOpt)) QTimer::singleShot(p.value(quitAfterOpt).toInt(), &app, &QCoreApplication::quit);

    QObject *root = engine.rootObjects().value(0);
    QObject *header = root ? root->findChild<QObject *>(QStringLiteral("header")) : nullptr;
    if (p.isSet(tabOpt) && header) header->setProperty("currentTab", p.value(tabOpt).toInt());
    if (p.isSet(importOpt) && PlanController::s_instance)
        QTimer::singleShot(500, &app, [file = p.value(importOpt)] { PlanController::s_instance->importFile(QUrl::fromLocalFile(file)); });
    if (p.isSet(replayOpt) && ReplayController::s_instance)
        QTimer::singleShot(500, &app, [file = p.value(replayOpt)] { ReplayController::s_instance->openPath(file); });
    // Tests: a timed script of the debug commands (TestDriver), and the debug server
    TestDriver driver(engine);
    if (p.isSet(grabsOpt)) {
        QFile f(p.value(grabsOpt));
        if (f.open(QIODevice::ReadOnly)) {
            for (const QString &line : QString::fromUtf8(f.readAll()).split(QLatin1Char('\n'), Qt::SkipEmptyParts)) {
                const QStringList w = line.trimmed().split(QLatin1Char(' '), Qt::SkipEmptyParts);
                if (w.size() < 3) continue;
                const int at = w[0].toInt();
                const QString cmd = w[1], arg = w.mid(2).join(QLatin1Char(' '));
                QTimer::singleShot(at, &app, [&driver, cmd, arg] {
                    QString error;
                    driver.run(cmd, arg, &error);
                    if (!error.isEmpty()) LOG_W("app") << "script:" << cmd << arg << "-" << error;
                });
            }
        }
    }
    std::unique_ptr<DebugServer> debugServer;
    if (p.isSet(debugPortOpt)) {
        debugServer = std::make_unique<DebugServer>(driver);
        debugServer->listen(quint16(p.value(debugPortOpt).toUInt()));
    }

    const int rc = app.exec();
    Settings::instance().flush();
    DebugLog::shutdown();
    return rc;
}
