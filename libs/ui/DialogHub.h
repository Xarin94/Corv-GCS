// DialogHub.h - the in-page alert / confirm of index.html for every view model
// ("DialogHub" QML singleton). Dialogs.qml shows what is asked here; the
// answer comes back to the callback given with the question, once.
// GUI thread.
#pragma once

#include <QHash>
#include <QObject>
#include <QQmlEngine>
#include <functional>

namespace corv {

class DialogHub : public QObject
{
    Q_OBJECT
    QML_NAMED_ELEMENT(DialogHub)
    QML_SINGLETON
public:
    explicit DialogHub(QObject *parent);
    static DialogHub *s_instance;
    static DialogHub *create(QQmlEngine *, QJSEngine *) { QJSEngine::setObjectOwnership(s_instance, QJSEngine::CppOwnership); return s_instance; }

    /// A toast at the top centre (alert() of the Electron version).
    static void toast(const QString &text);
    /// A modal OK / Cancel; `done(true)` on OK.
    static void confirm(const QString &message, std::function<void(bool ok)> done);
    /// A modal text prompt; `done(ok, text)`.
    static void prompt(const QString &message, const QString &value, std::function<void(bool ok, const QString &text)> done);

    Q_INVOKABLE void answer(int token, bool ok, const QString &text);

signals:
    void toastRequested(const QString &text);
    void confirmRequested(int token, const QString &message);
    void promptRequested(int token, const QString &message, const QString &value);

private:
    QHash<int, std::function<void(bool, const QString &)>> m_pending;
    int m_next = 1;
};

} // namespace corv
