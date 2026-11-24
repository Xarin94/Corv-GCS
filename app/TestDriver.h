// TestDriver.h - drives the running application for tests: the commands of
// --grab-script and of the debug server (DebugServer, --debug-port), the
// native replacement of the Chromium DevTools protocol the Electron tests used.
//
//   tab N                     open a tab of the header bar
//   emit SIGNAL               a view button of the header bar (toggleSatellite...)
//   call VM.method [ARGS]     a view model method; ARGS a string, a JSON object
//                             (one QVariantMap) or a JSON array (the arguments)
//   set OBJECT PROP VALUE     a property of a view model or of a named QML item
//   get OBJECT PROP           read one back
//   key / keyup ARROW         an arrow key held or released
//   size WxH                  resize the window
//   select N, popover ID, library open|close   FLIGHT PLAN helpers
//   grab FILE                 the window as an image
//   stats                     what the 3D views report (fps, chunks, draw calls...)
//
// The debug server reads more (tests/sitl): the vehicle snapshot ("state"), every
// property of an object ("props"), the rows of a model ("rows"), and what the operator
// was told since the start: toasts, HUD messages and questions ("events").
// GUI thread.
#pragma once

#include <QJsonObject>
#include <QJsonValue>
#include <QList>
#include <QObject>
#include <QString>

class QQmlApplicationEngine;
class QQuickWindow;

namespace corv {

class TestDriver
{
public:
    explicit TestDriver(QQmlApplicationEngine &engine);

    /// A script line's command and its argument text ("call", "Gcs.setMode FBWA").
    /// Returns the result (null when there is none); `error` is set when the command failed.
    QJsonValue run(const QString &cmd, const QString &arg, QString *error = nullptr);
    /// The same with JSON parameters (the debug server).
    QJsonValue runJson(const QString &method, const QJsonValue &params, QString *error);

    /// Keep what the operator is told (toasts, HUD messages, questions) for "events".
    void recordEvents();

    /// A view model by the name QML gives it: Gcs, Plan, Params...
    static QObject *viewModel(const QString &name);

private:
    QQuickWindow *window() const;
    QObject *header() const;
    QObject *object(const QString &name) const;
    QJsonValue call(const QString &target, const QJsonValue &args, QString *error);
    QJsonValue stats() const;
    QJsonValue state() const;
    QJsonValue props(const QString &name, QString *error) const;
    QJsonValue rows(const QString &name, const QString &property, int from, int max, QString *error) const;
    QJsonValue events(qint64 since) const;
    void addEvent(QJsonObject e);

    QQmlApplicationEngine &m_engine;
    QObject m_context;             // receiver of the recorded signals
    QList<QJsonObject> m_events;   // the last kMaxEvents, oldest first
    qint64 m_eventSeq = 0;
};

} // namespace corv
