#include "vehicle/LogDownloader.h"

#include "core/DebugLog.h"

#include <QFileInfo>
#include <algorithm>
#include <cstring>

using namespace Qt::StringLiterals;

namespace corv {

namespace {
constexpr int kMaxRunBlocks = 4096;     // one gap request: at most 360 KB
constexpr int kProgressMs = 150;
} // namespace

struct LogDownloader::Download {
    int id = 0;
    quint32 size = 0;
    QString path;
    QFile file;
    uchar *map = nullptr;
    std::vector<bool> have;              // per 90-byte block
    qint64 blocks = 0, received = 0;     // blocks in the file, blocks received
    qint64 scan = 0;                     // every block before this one is present
    quint32 reqStart = 0, reqEnd = 0;    // the range asked for
    int stalls = 0;
    QElapsedTimer clock, lastProgress;
    qint64 rateBytes = 0;                // for the rate: bytes at rateMark
    qint64 rateMark = 0;
    double rate = 0;
};

LogDownloader::LogDownloader(Send send, QObject *parent) : QObject(parent), m_send(std::move(send))
{
    m_listTimer.setSingleShot(true);
    connect(&m_listTimer, &QTimer::timeout, this, &LogDownloader::onListTimeout);
    m_stallTimer.setSingleShot(true);
    connect(&m_stallTimer, &QTimer::timeout, this, &LogDownloader::onStall);
}

LogDownloader::~LogDownloader()
{
    if (m_dl) endDownload(false, u"closed"_s);
}

void LogDownloader::setTarget(int system, int component)
{
    m_sys = system;
    m_comp = component;
}


void LogDownloader::onMessage(const mavlink_message_t &m)
{
    if (m.msgid == MAVLINK_MSG_ID_LOG_ENTRY) {
        mavlink_log_entry_t e;
        mavlink_msg_log_entry_decode(&m, &e);
        onEntry(e);
    } else if (m.msgid == MAVLINK_MSG_ID_LOG_DATA) {
        mavlink_log_data_t d;
        mavlink_msg_log_data_decode(&m, &d);
        onData(d);
    }
}

// ── Sending ──────────────────────────────────────────────────────────────────

void LogDownloader::sendList(int start, int end)
{
    mavlink_message_t m;
    mavlink_msg_log_request_list_pack(mav::kGcsSystemId, mav::kGcsComponentId, &m, uint8_t(m_sys), uint8_t(m_comp), uint16_t(start), uint16_t(end));
    m_send(m);
}

void LogDownloader::sendData(int id, quint32 ofs, quint32 count)
{
    mavlink_message_t m;
    mavlink_msg_log_request_data_pack(mav::kGcsSystemId, mav::kGcsComponentId, &m, uint8_t(m_sys), uint8_t(m_comp), uint16_t(id), ofs, count);
    m_send(m);
}

void LogDownloader::sendRequestEnd()
{
    mavlink_message_t m;
    mavlink_msg_log_request_end_pack(mav::kGcsSystemId, mav::kGcsComponentId, &m, uint8_t(m_sys), uint8_t(m_comp));
    m_send(m);
}

void LogDownloader::sendErase()
{
    mavlink_message_t m;
    mavlink_msg_log_erase_pack(mav::kGcsSystemId, mav::kGcsComponentId, &m, uint8_t(m_sys), uint8_t(m_comp));
    m_send(m);
    m_entries.clear();
    emit entriesChanged();
}

// ── List ─────────────────────────────────────────────────────────────────────

void LogDownloader::requestList()
{
    if (m_dl) return;   // the vehicle serves one or the other
    m_entries.clear();
    m_listExpected = -1;
    m_listTries = 0;
    m_listing = true;
    emit entriesChanged();
    sendList(0, 0xFFFF);
    m_listTimer.start(m_listTimeoutMs * 2);   // the first answer can take a moment (card scan)
}

void LogDownloader::onEntry(const mavlink_log_entry_t &e)
{
    if (!m_listing) return;
    m_listExpected = e.num_logs;
    if (e.num_logs == 0) {
        finishList(true, {});
        return;
    }
    if (e.id >= 1) m_entries.insert(e.id, Entry{e.id, e.time_utc, e.size});
    emit entriesChanged();
    if (m_entries.size() >= m_listExpected) finishList(true, {});
    else m_listTimer.start(m_listTimeoutMs);
}

void LogDownloader::onListTimeout()
{
    if (!m_listing) return;
    if (++m_listTries > 3) {
        finishList(false, m_listExpected < 0 ? u"no answer from the vehicle (disarmed? logging enabled?)"_s
                                             : u"%1 of %2 entries received"_s.arg(m_entries.size()).arg(m_listExpected));
        return;
    }
    if (m_listExpected < 0) {
        sendList(0, 0xFFFF);
    } else {
        // The ids lost on the way: from the first missing to the last missing
        int first = -1, last = -1;
        for (int id = 1; id <= m_listExpected; ++id) {
            if (m_entries.contains(id)) continue;
            if (first < 0) first = id;
            last = id;
        }
        if (first < 0) {
            finishList(true, {});
            return;
        }
        sendList(first, last);
    }
    m_listTimer.start(m_listTimeoutMs);
}

void LogDownloader::finishList(bool ok, const QString &error)
{
    m_listTimer.stop();
    m_listing = false;
    LOG_I("logs") << "list:" << m_entries.size() << "of" << m_listExpected << (error.isEmpty() ? QString() : u"("_s + error + ')'_L1);
    emit entriesChanged();
    emit listFinished(ok, error);
}

// ── Download ─────────────────────────────────────────────────────────────────

bool LogDownloader::download(int id, quint32 size, const QString &path, QString *error)
{
    auto fail = [error](const QString &e) {
        if (error) *error = e;
        return false;
    };
    if (m_dl) return fail(u"a download is in progress"_s);
    if (m_listing) finishList(false, u"interrupted by a download"_s);
    auto d = std::make_unique<Download>();
    d->id = id;
    d->size = size;
    d->path = path;
    d->file.setFileName(path + u".part"_s);
    if (!d->file.open(QIODevice::ReadWrite | QIODevice::Truncate)) return fail(d->file.errorString());
    if (size > 0) {
        if (!d->file.resize(size)) return fail(d->file.errorString());
        d->map = d->file.map(0, size);
        if (!d->map) return fail(u"cannot map %1 bytes: %2"_s.arg(size).arg(d->file.errorString()));
    }
    d->blocks = (qint64(size) + kBlock - 1) / kBlock;
    d->have.assign(size_t(d->blocks), false);
    d->clock.start();
    d->lastProgress.start();
    m_dl = std::move(d);
    LOG_I("logs") << "downloading log" << id << "," << size << "B to" << path;
    if (size == 0) {
        endDownload(true, {});
        return true;
    }
    m_dl->reqStart = 0;
    m_dl->reqEnd = size;
    sendData(id, 0, 0xFFFFFFFFu);
    m_stallTimer.start(m_stallMs * 2);
    emitProgress(true);
    return true;
}

void LogDownloader::onData(const mavlink_log_data_t &d)
{
    if (!m_dl || d.id != m_dl->id) return;
    Download &dl = *m_dl;
    const quint32 ofs = d.ofs;
    quint32 count = std::min<quint32>(d.count, kBlock);
    // A short packet before the announced end: the log is shorter than LOG_ENTRY said
    if (count < kBlock && ofs < dl.size && ofs + count < dl.size) {
        LOG_W("logs") << "log" << dl.id << "ends at" << ofs + count << "instead of" << dl.size;
        dl.size = ofs + count;
        dl.blocks = (qint64(dl.size) + kBlock - 1) / kBlock;
        dl.have.resize(size_t(dl.blocks));
        dl.received = std::count(dl.have.begin(), dl.have.end(), true);
        dl.reqEnd = std::min(dl.reqEnd, dl.size);
    }
    if (ofs < dl.size && count > 0) {
        count = std::min(count, dl.size - ofs);
        std::memcpy(dl.map + ofs, d.data, count);
        // Gap requests start on block boundaries, so a packet is one block
        if (ofs % kBlock == 0) {
            const qint64 b = ofs / kBlock;
            if (b < dl.blocks && !dl.have[size_t(b)] && (count == kBlock || ofs + count == dl.size)) {
                dl.have[size_t(b)] = true;
                ++dl.received;
                dl.stalls = 0;
            }
        }
    }
    emitProgress(false);
    if (dl.received >= dl.blocks) {
        endDownload(true, {});
        return;
    }
    // The last packet of the range asked for (or the vehicle says the file ends there): the
    // next gap. Packets of an earlier request still on the way are kept but move nothing,
    // or each of them would ask again.
    const bool ofCurrent = ofs >= dl.reqStart && (ofs < dl.reqEnd || count == 0);
    if (ofCurrent && (ofs + count >= dl.reqEnd || d.count < kBlock)) {
        requestNextGap();
        return;
    }
    m_stallTimer.start(m_stallMs);
}

void LogDownloader::requestNextGap()
{
    Download &dl = *m_dl;
    while (dl.scan < dl.blocks && dl.have[size_t(dl.scan)]) ++dl.scan;
    if (dl.scan >= dl.blocks) {
        endDownload(true, {});
        return;
    }
    qint64 end = dl.scan;
    while (end < dl.blocks && !dl.have[size_t(end)] && end - dl.scan < kMaxRunBlocks) ++end;
    const quint32 ofs = quint32(dl.scan * kBlock);
    const quint32 stop = std::min<quint32>(quint32(end * kBlock), dl.size);
    dl.reqStart = ofs;
    dl.reqEnd = stop;
    sendData(dl.id, ofs, stop - ofs);
    m_stallTimer.start(m_stallMs);
}

void LogDownloader::onStall()
{
    if (!m_dl) return;
    if (++m_dl->stalls > m_retries) {
        endDownload(false, u"the vehicle stopped sending (%1 of %2 bytes)"_s.arg(m_dl->received * kBlock).arg(m_dl->size));
        return;
    }
    LOG_D("logs") << "log" << m_dl->id << "stalled, asking again from block" << m_dl->scan;
    requestNextGap();
}

void LogDownloader::emitProgress(bool force)
{
    Download &dl = *m_dl;
    if (!force && dl.lastProgress.elapsed() < kProgressMs) return;
    const qint64 now = dl.clock.elapsed();
    const qint64 bytes = std::min<qint64>(dl.received * kBlock, dl.size);
    if (now - dl.rateMark >= 1000 || dl.rate == 0) {
        if (now > dl.rateMark) {
            const double inst = double(bytes - dl.rateBytes) * 1000.0 / double(now - dl.rateMark);
            dl.rate = dl.rate == 0 ? inst : 0.6 * dl.rate + 0.4 * inst;
        }
        dl.rateMark = now;
        dl.rateBytes = bytes;
    }
    dl.lastProgress.restart();
    emit progress(dl.id, bytes, dl.size, dl.rate);
}

void LogDownloader::endDownload(bool ok, const QString &error)
{
    m_stallTimer.stop();
    std::unique_ptr<Download> dl = std::move(m_dl);
    if (dl->map) dl->file.unmap(dl->map);
    if (ok) dl->file.resize(dl->size);   // a log shorter than announced
    dl->file.close();
    QString err = error;
    if (ok) {
        QFile::remove(dl->path);
        if (!QFile::rename(dl->file.fileName(), dl->path)) {
            ok = false;
            err = u"cannot rename %1"_s.arg(dl->file.fileName());
        }
    }
    if (!ok) QFile::remove(dl->file.fileName());
    const double s = dl->clock.elapsed() / 1000.0;
    if (ok) LOG_I("logs") << "log" << dl->id << "saved," << dl->size << "B in" << s << "s," << (s > 0 ? dl->size / s / 1024.0 : 0) << "KB/s";
    else LOG_W("logs") << "log" << dl->id << "failed:" << err;
    if (ok) emit progress(dl->id, dl->size, dl->size, s > 0 ? dl->size / s : 0);
    emit finished(dl->id, ok, ok ? dl->path : QString(), err);
}

void LogDownloader::cancel()
{
    if (m_listing) finishList(false, u"cancelled"_s);
    if (m_dl) endDownload(false, u"cancelled"_s);
    sendRequestEnd();
}

} // namespace corv
