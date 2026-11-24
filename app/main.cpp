// CORV GCS - Qt Quick application entry point.
//
//   corv-gcs [--connect tcp:HOST:PORT | udp:HOST:PORT | serial:PORT:BAUD]
//
// --connect opens a link at start (SITL tests), as the CONNECT button would.
#include "DebugServer.h"
#include "TestDriver.h"
#include "core/DataRoot.h"
#include "core/DebugLog.h"
#include "core/ElectronSettings.h"
#include "core/Settings.h"
#include "terrain/TileStore.h"
#include "ui/ReplayController.h"
#include "ui/GcsController.h"
#include "ui/Language.h"
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

using namespace Qt::StringLiterals;

Q_IMPORT_QML_PLUGIN(Corv_UiPlugin)
Q_IMPORT_QML_PLUGIN(Corv_GpuPlugin)

#ifdef Q_OS_WIN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

// Laptops: ask the driver for the dedicated GPU (was --force_high_performance_gpu in Electron)
extern "C" {
__declspec(dllexport) unsigned long NvOptimusEnablement = 1;
__declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
}
#endif

using namespace corv;

namespace {
/// Windows slows a process down to save power (on battery, or with its window behind
/// another) and gives it 15.6 ms timers: the frames, which are paced by a timer, came
/// late and the telemetry with them. Not for a ground station in the field.
void keepFullSpeed()
{
#ifdef Q_OS_WIN
    PROCESS_POWER_THROTTLING_STATE state{};
    state.Version = PROCESS_POWER_THROTTLING_CURRENT_VERSION;
    state.ControlMask = PROCESS_POWER_THROTTLING_EXECUTION_SPEED;
    state.StateMask = 0;   // never throttled
    SetProcessInformation(GetCurrentProcess(), ProcessPowerThrottling, &state, sizeof state);
    // (Windows 11; refused as a whole by an older one, so asked apart)
    state.ControlMask = PROCESS_POWER_THROTTLING_IGNORE_TIMER_RESOLUTION;
    SetProcessInformation(GetCurrentProcess(), ProcessPowerThrottling, &state, sizeof state);
#endif
}
} // namespace

int main(int argc, char *argv[])
{
    keepFullSpeed();
    QGuiApplication app(argc, argv);
    QGuiApplication::setApplicationName(QStringLiteral(CORV_APP_NAME));
    QGuiApplication::setApplicationVersion(QStringLiteral(CORV_VERSION));
    QGuiApplication::setOrganizationName(QString());
    app.setWindowIcon(QIcon(u":/icons/icon-256x256.png"_s));

    QCommandLineParser p;
    p.addHelpOption();
    p.addVersionOption();
    QCommandLineOption connectOpt(u"connect"_s, u"Open a link at start: tcp:HOST:PORT, udp:HOST:PORT, serial:PORT:BAUD"_s, u"link"_s);
    QCommandLineOption grabOpt(u"grab"_s, u"Save the window as an image after --grab-after ms (tests)"_s, u"file"_s);
    QCommandLineOption grabAfterOpt(u"grab-after"_s, u"Delay before --grab, ms"_s, u"ms"_s, u"10000"_s);
    QCommandLineOption quitAfterOpt(u"quit-after"_s, u"Quit after this many ms (tests)"_s, u"ms"_s);
    QCommandLineOption tabOpt(u"tab"_s, u"Open this tab at start: 0 FLIGHT DATA, 1 FLIGHT PLAN, 2 SETUP, 3 SYS CONFIG (tests)"_s, u"index"_s);
    QCommandLineOption importOpt(u"import"_s, u"Import a route file into FLIGHT PLAN at start (tests)"_s, u"file"_s);
    QCommandLineOption replayOpt(u"replay"_s, u"Load a .tlog or .bin into LOG REPLAY at start (tests)"_s, u"file"_s);
    QCommandLineOption grabsOpt(u"grab-script"_s, u"Tests: a file of lines 'ms tab N' / 'ms grab FILE' / 'ms select INDEX' / 'ms emit toggleSatellite' run in order"_s, u"file"_s);
    QCommandLineOption debugPortOpt(u"debug-port"_s, u"Tests: a JSON-RPC debug server on 127.0.0.1:PORT (set, get, call, stats, screenshot, script)"_s, u"port"_s);
    p.addOptions({connectOpt, grabOpt, grabAfterOpt, quitAfterOpt, tabOpt, importOpt, replayOpt, grabsOpt, debugPortOpt});
    p.process(app);

    DebugLog::init();
    // The very first start: the settings of CORV GCS 1.x, if it was used on this computer
    const bool firstStart = qEnvironmentVariableIsEmpty("CORV_DATA_ROOT") && !QFile::exists(DataRoot::root() + u"/settings.json"_s);
    Settings::instance().load();
    int imported = 0;
    if (firstStart)
        if (const QString old = electronSettings::findStore(); !old.isEmpty()) imported = electronSettings::importInto(electronSettings::read(old));
    LOG_I("app") << "CORV GCS" << CORV_VERSION << "starting, data root" << DataRoot::root();

    // Fonts of the Electron UI (Rajdhani for labels, Roboto Mono for data)
    for (const QString &f : QDir(u":/fonts"_s).entryList())
        if (QFontDatabase::addApplicationFont(u":/fonts/"_s + f) < 0)
            LOG_D("app") << "font not loaded:" << f;

    QQuickStyle::setStyle(u"Basic"_s);
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

    // The interface's language as it was left (gcs-lang); SYS CONFIG changes it while running
    Language::instance().set(Settings::instance().value(u"gcs-lang"_s).toString());
    GcsController gcs(nullptr);
    QQmlApplicationEngine engine;
    QObject::connect(&Language::instance(), &Language::changed, &engine, &QQmlApplicationEngine::retranslate);
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreationFailed, &app, [] { QCoreApplication::exit(1); }, Qt::QueuedConnection);
    engine.loadFromModule("Corv.Ui", "Main");

    if (auto *w = qobject_cast<QQuickWindow *>(engine.rootObjects().value(0))) {
        const auto api = w->rendererInterface()->graphicsApi();
        static const QHash<int, QString> names = {{QSGRendererInterface::Direct3D11, u"Direct3D 11"_s},
                                                 {QSGRendererInterface::Direct3D12, u"Direct3D 12"_s},
                                                 {QSGRendererInterface::Vulkan, u"Vulkan"_s},
                                                 {QSGRendererInterface::Metal, u"Metal"_s},
                                                 {QSGRendererInterface::OpenGL, u"OpenGL"_s},
                                                 {QSGRendererInterface::Software, u"software"_s}};
        DebugLog::setHeader(u"Graphics"_s, names.value(api, u"other"_s));
    }

    if (imported)
        QTimer::singleShot(2000, &gcs, [&gcs, imported] { gcs.notify(u"Settings of CORV GCS 1.x imported (%1)"_s.arg(imported)); });

    if (p.isSet(connectOpt)) {
        const QStringList parts = p.value(connectOpt).split(':'_L1);
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
    QObject *header = root ? root->findChild<QObject *>(u"header"_s) : nullptr;
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
            for (const QString &line : QString::fromUtf8(f.readAll()).split('\n'_L1, Qt::SkipEmptyParts)) {
                const QStringList w = line.trimmed().split(' '_L1, Qt::SkipEmptyParts);
                if (w.size() < 3) continue;
                const int at = w[0].toInt();
                const QString cmd = w[1], arg = w.mid(2).join(' '_L1);
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
