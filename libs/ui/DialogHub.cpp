#include "ui/DialogHub.h"

#include "core/DebugLog.h"

namespace corv {

DialogHub *DialogHub::s_instance = nullptr;

DialogHub::DialogHub(QObject *parent) : QObject(parent) { s_instance = this; }

void DialogHub::toast(const QString &text)
{
    LOG_I("ui") << "toast:" << text;
    if (s_instance) emit s_instance->toastRequested(text);
}

void DialogHub::confirm(const QString &message, std::function<void(bool)> done)
{
    if (!s_instance) {
        if (done) done(false);
        return;
    }
    const int t = s_instance->m_next++;
    s_instance->m_pending.insert(t, [done](bool ok, const QString &) {
        if (done) done(ok);
    });
    emit s_instance->confirmRequested(t, message);
}

void DialogHub::prompt(const QString &message, const QString &value, std::function<void(bool, const QString &)> done)
{
    if (!s_instance) {
        if (done) done(false, {});
        return;
    }
    const int t = s_instance->m_next++;
    s_instance->m_pending.insert(t, std::move(done));
    emit s_instance->promptRequested(t, message, value);
}

void DialogHub::answer(int token, bool ok, const QString &text)
{
    auto fn = m_pending.take(token);
    if (fn) fn(ok, text);
}

} // namespace corv
