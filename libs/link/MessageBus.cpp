#include "link/MessageBus.h"

#include <QThread>

namespace corv {

int MessageBus::subscribe(uint32_t msgid, QObject *context, Handler h)
{
    QMutexLocker lock(&m_lock);
    const int token = m_next++;
    m_subs[msgid].append({token, context, std::move(h)});
    return token;
}

void MessageBus::unsubscribe(int token)
{
    QMutexLocker lock(&m_lock);
    for (auto it = m_subs.begin(); it != m_subs.end(); ++it) {
        auto &list = it.value();
        for (qsizetype i = 0; i < list.size(); ++i)
            if (list[i].token == token) {
                list.removeAt(i);
                return;
            }
    }
}

void MessageBus::publish(const MavMessage &m)
{
    QList<Sub> targets;
    {
        QMutexLocker lock(&m_lock);
        if (auto it = m_subs.find(m.id()); it != m_subs.end()) targets += it.value();
        if (auto it = m_subs.find(kAll); it != m_subs.end()) targets += it.value();
        // Drop subscriptions whose context is gone
        for (auto &list : m_subs) list.removeIf([](const Sub &s) { return s.ctx.isNull(); });
    }
    for (const Sub &s : targets) {
        QObject *ctx = s.ctx.data();
        if (!ctx) continue;
        if (ctx->thread() == QThread::currentThread()) {
            s.fn(m);
        } else {
            Handler fn = s.fn;
            QMetaObject::invokeMethod(ctx, [fn, m] { fn(m); }, Qt::QueuedConnection);
        }
    }
}

} // namespace corv
