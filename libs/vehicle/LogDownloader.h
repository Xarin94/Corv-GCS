// LogDownloader.h - the DataFlash logs on the autopilot's SD card, over MAVLink
// (Mission Planner's "Download DataFlash Log Via Mavlink").
//
// List: LOG_REQUEST_LIST (0..0xFFFF) answered by one LOG_ENTRY per log
// {id 1..num_logs, num_logs, last_log_num, time_utc, size}; one LOG_ENTRY with
// num_logs 0 when the card is empty. Entries lost on the way are asked again by
// range after `listTimeoutMs` of silence.
//
// Download, one log at a time: LOG_REQUEST_DATA {id, ofs 0, count 0xFFFFFFFF}
// makes ArduPilot stream the whole file as LOG_DATA {id, ofs, count <= 90,
// data[90]}, paced by the link's free space. Every 90-byte block received is
// written in place into a file mapped in memory (<name>.part, renamed when
// complete) and marked in a bitmap; when the stream has passed the end, or after
// `stallMs` of silence, the first run of missing blocks is asked for with one
// LOG_REQUEST_DATA {ofs, count}, and so on until nothing is missing. A request
// replaces the one before on the vehicle, so one is outstanding at a time.
// `retries` silences in a row without a new block end the download.
// LOG_REQUEST_END (sent by the owner when the queue is done, or on cancel) lets
// the vehicle log again; LOG_ERASE wipes the card.
//
// ArduPilot answers none of this while armed. GUI thread; the owner feeds every
// LOG_ENTRY / LOG_DATA of the vehicle to onMessage() and sends what `send` is given.
#pragma once

#include "link/Mavlink.h"

#include <QElapsedTimer>
#include <QFile>
#include <QList>
#include <QMap>
#include <QObject>
#include <QString>
#include <QTimer>
#include <functional>
#include <memory>
#include <vector>

namespace corv {

class LogDownloader : public QObject
{
    Q_OBJECT
public:
    using Send = std::function<void(const mavlink_message_t &)>;
    struct Entry {
        int id = 0;
        quint32 timeUtc = 0;    // seconds since 1970, 0 when the log has no GPS time
        quint32 size = 0;
    };
    static constexpr int kBlock = 90;   // LOG_DATA payload

    explicit LogDownloader(Send send, QObject *parent = nullptr);
    ~LogDownloader() override;

    void setTarget(int system, int component);
    int targetSystem() const { return m_sys; }
    void setListTimeout(int ms) { m_listTimeoutMs = ms; }
    void setStallTimeout(int ms) { m_stallMs = ms; }
    void setRetries(int n) { m_retries = n; }

    /// LOG_ENTRY and LOG_DATA; anything else is ignored.
    void onMessage(const mavlink_message_t &m);

    // ── List ──
    void requestList();
    bool listing() const { return m_listing; }
    /// By id.
    QList<Entry> entries() const { return m_entries.values(); }

    // ── Download ──
    /// Fetch log `id` of `size` bytes into `path` (via path + ".part"). False if busy or the file cannot be made.
    bool download(int id, quint32 size, const QString &path, QString *error = nullptr);
    bool downloading() const { return m_dl != nullptr; }
    int currentId() const;
    /// Stop the download in progress (the partial file is removed) and send LOG_REQUEST_END.
    void cancel();

    // ── Vehicle ──
    void sendRequestEnd();
    void sendErase();

signals:
    void entriesChanged();
    /// The list is complete (ok) or what arrived is all there will be (ok false, error says why).
    void listFinished(bool ok, const QString &error);
    void progress(int id, qint64 received, qint64 size, double bytesPerSecond);
    void finished(int id, bool ok, const QString &path, const QString &error);

private:
    struct Download;
    void sendList(int start, int end);
    void sendData(int id, quint32 ofs, quint32 count);
    void onListTimeout();
    void finishList(bool ok, const QString &error);
    void onEntry(const mavlink_log_entry_t &e);
    void onData(const mavlink_log_data_t &d);
    void onStall();
    void requestNextGap();
    void endDownload(bool ok, const QString &error);
    void emitProgress(bool force);

    Send m_send;
    int m_sys = 1, m_comp = 1;
    int m_listTimeoutMs = 1500, m_stallMs = 1500, m_retries = 8;

    bool m_listing = false;
    int m_listExpected = -1;
    int m_listTries = 0;
    QMap<int, Entry> m_entries;
    QTimer m_listTimer;

    std::unique_ptr<Download> m_dl;
    QTimer m_stallTimer;
};

} // namespace corv
