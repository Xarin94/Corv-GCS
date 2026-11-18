// MessageBus.h - per-message subscriptions across threads (onMessage/offMessage of
// MAVLinkManager.js).
//
// The link thread publishes every MavMessage; a subscriber names a context
// QObject and its handler runs in that object's thread (queued from the link
// thread to the GUI thread). A subscription dies with its context.
#pragma once

#include "link/MavMessage.h"

#include <QHash>
#include <QMutex>
#include <QObject>
#include <QPointer>
#include <functional>

namespace corv {

class MessageBus : public QObject
{
    Q_OBJECT
public:
    using Handler = std::function<void(const MavMessage &)>;
    static constexpr uint32_t kAll = 0xFFFFFFFFu;

    explicit MessageBus(QObject *parent = nullptr) : QObject(parent) {}

    /// Handler for msgid (kAll = every message), run in context's thread. Returns a token.
    int subscribe(uint32_t msgid, QObject *context, Handler h);
    void unsubscribe(int token);

    /// Called by the link thread for every message.
    void publish(const MavMessage &m);

private:
    struct Sub {
        int token;
        QPointer<QObject> ctx;
        Handler fn;
    };
    QMutex m_lock;
    QHash<uint32_t, QList<Sub>> m_subs;
    int m_next = 1;
};

} // namespace corv
