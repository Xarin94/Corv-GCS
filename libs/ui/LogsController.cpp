#include "ui/LogsController.h"

#include "core/DataRoot.h"
#include "core/DebugLog.h"
#include "core/Settings.h"
#include "link/LinkManager.h"
#include "link/MessageBus.h"
#include "ui/DialogHub.h"
#include "ui/ReplayController.h"
#include "vehicle/CommandSender.h"
#include "vehicle/LogDownloader.h"
#include "vehicle/VehicleHub.h"

#include <QDateTime>
#include <QDesktopServices>
#include <QDir>
#include <QFileInfo>
#include <cmath>

using namespace Qt::StringLiterals;

namespace corv {

LogsController *LogsController::s_instance = nullptr;

namespace {
const QString kFolderKey = u"logs-download-folder"_s;

QString durationText(double s)
{
    if (!std::isfinite(s) || s < 0) return u"--"_s;
    const int t = int(std::lround(s));
    return t >= 3600 ? u"%1:%2:%3"_s.arg(t / 3600).arg((t / 60) % 60, 2, 10, '0'_L1).arg(t % 60, 2, 10, '0'_L1)
                     : u"%1:%2"_s.arg(t / 60).arg(t % 60, 2, 10, '0'_L1);
}
} // namespace

// ── Model ────────────────────────────────────────────────────────────────────

QHash<int, QByteArray> LogListModel::roleNames() const
{
    return {{IdRole, "logId"}, {DateRole, "date"}, {SizeRole, "sizeText"}, {BytesRole, "bytes"}, {SelectedRole, "selected"},
            {StateRole, "logState"}, {ProgressRole, "progress"}, {PathRole, "path"}, {DetailRole, "detail"}};
}

QVariant LogListModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid() || index.row() >= m_rows.size()) return {};
    const Row &r = m_rows[index.row()];
    switch (role) {
    case IdRole: return r.id;
    case DateRole:
        return r.timeUtc ? QDateTime::fromSecsSinceEpoch(r.timeUtc).toLocalTime().toString(u"yyyy-MM-dd  HH:mm:ss"_s) : u"no date"_s;
    case SizeRole: return LogsController::sizeText(r.size);
    case BytesRole: return double(r.size);
    case SelectedRole: return r.selected;
    case StateRole: return r.state;
    case ProgressRole: return r.progress;
    case PathRole: return r.path;
    case DetailRole: return r.detail;
    default: return {};
    }
}

void LogListModel::reset(const QList<Row> &rows)
{
    beginResetModel();
    m_rows = rows;
    endResetModel();
}

LogListModel::Row *LogListModel::find(int id)
{
    for (Row &r : m_rows)
        if (r.id == id) return &r;
    return nullptr;
}

void LogListModel::touch(int id)
{
    for (int i = 0; i < m_rows.size(); ++i)
        if (m_rows[i].id == id) emit dataChanged(index(i), index(i));
}

void LogListModel::touchAll()
{
    if (!m_rows.isEmpty()) emit dataChanged(index(0), index(int(m_rows.size()) - 1));
}

// ── Controller ───────────────────────────────────────────────────────────────

LogsController::LogsController(LinkManager *link, MessageBus *bus, CommandSender *cmd, VehicleHub *hub, QObject *parent)
    : QObject(parent), m_link(link), m_bus(bus), m_cmd(cmd), m_hub(hub)
{
    s_instance = this;
    m_dl = new LogDownloader([this](const mavlink_message_t &m) { if (m_link) m_link->send(m); }, this);
    for (uint32_t id : {uint32_t(MAVLINK_MSG_ID_LOG_ENTRY), uint32_t(MAVLINK_MSG_ID_LOG_DATA)}) {
        m_bus->subscribe(id, this, [this](const MavMessage &mm) {
            if (mm.msg.sysid == m_dl->targetSystem()) m_dl->onMessage(mm.msg);
        });
    }
    connect(m_dl, &LogDownloader::entriesChanged, this, [this] {
        if (m_dl->listing()) setStatus(u"Reading the list of logs… %1"_s.arg(m_dl->entries().size()));
    });
    connect(m_dl, &LogDownloader::listFinished, this, [this](bool ok, const QString &error) {
        rebuildFromList();
        if (!ok && m_model.rows().isEmpty()) {
            setStatus(u"No list: "_s + error + (m_armed ? u" — ArduPilot does not serve logs while armed"_s : QString()), true);
        } else if (!ok) {
            setStatus(u"%1 logs listed (%2)"_s.arg(m_model.rows().size()).arg(error), true);
        } else {
            setStatus(m_model.rows().isEmpty() ? u"No logs on the vehicle"_s
                                               : u"%1 logs on the vehicle, %2"_s.arg(m_model.rows().size()).arg(totalSize()));
        }
    });
    connect(m_dl, &LogDownloader::progress, this, [this](int id, qint64 received, qint64 size, double rate) {
        LogListModel::Row *r = m_model.find(id);
        if (!r) return;
        r->progress = size > 0 ? double(received) / double(size) : 1.0;
        m_currentBytes = received;
        const double left = rate > 0 ? double(size - received) / rate : -1;
        r->detail = u"%1/s · %2 left"_s.arg(sizeText(rate), durationText(left));
        m_model.touch(id);
        const QString of = m_queueTotal > 1 ? u" · %1 of %2"_s.arg(m_queueDone + m_queueFailed + 1).arg(m_queueTotal) : QString();
        setStatus(u"Log %1 · %2 / %3 · %4%5"_s.arg(id).arg(sizeText(received), sizeText(size), r->detail, of));
    });
    connect(m_dl, &LogDownloader::finished, this, [this](int id, bool ok, const QString &path, const QString &error) {
        if (!ok && error == "cancelled"_L1) {
            // cancel() puts the rows back
            m_currentBytes = 0;
            return;
        }
        if (LogListModel::Row *r = m_model.find(id)) {
            r->state = ok ? u"saved"_s : u"failed"_s;
            r->path = path;
            r->progress = ok ? 1.0 : r->progress;
            r->detail = ok ? QFileInfo(path).fileName() : error;
            m_queueBytesDone += r->size;
            m_model.touch(id);
        }
        m_currentBytes = 0;
        if (ok) ++m_queueDone;
        else ++m_queueFailed;
        startNext();
    });
    // The link state for the buttons; a new vehicle is a new list
    m_linkPoll.setInterval(500);
    connect(&m_linkPoll, &QTimer::timeout, this, [this] {
        const auto snap = m_hub->snapshot();
        const bool ready = snap->connected;
        const bool armed = ready && snap->armed;
        if (ready != m_linkReady || armed != m_armed) {
            if (!ready && m_linkReady && downloading()) cancel();
            m_linkReady = ready;
            m_armed = armed;
            emit linkChanged();
        }
    });
    m_linkPoll.start();
    setStatus(u"Connect to a vehicle and press LIST LOGS"_s);
}

LogsController::~LogsController()
{
    if (s_instance == this) s_instance = nullptr;
}

bool LogsController::listing() const { return m_dl->listing(); }
bool LogsController::downloading() const { return m_dl->downloading() || !m_queue.isEmpty(); }

void LogsController::setStatus(const QString &text, bool error)
{
    m_status = text;
    m_statusError = error;
    emit stateChanged();
}

QString LogsController::sizeText(double bytes)
{
    if (bytes >= 1024.0 * 1024 * 1024) return u"%1 GB"_s.arg(bytes / (1024.0 * 1024 * 1024), 0, 'f', 2);
    if (bytes >= 1024.0 * 1024) return u"%1 MB"_s.arg(bytes / (1024.0 * 1024), 0, 'f', 1);
    if (bytes >= 1024.0) return u"%1 KB"_s.arg(bytes / 1024.0, 0, 'f', 0);
    return u"%1 B"_s.arg(qint64(bytes));
}

QString LogsController::fileName(int id, quint32 timeUtc, quint32 size)
{
    if (timeUtc == 0) return u"log%1 %2B.bin"_s.arg(id).arg(size);
    return QDateTime::fromSecsSinceEpoch(timeUtc).toLocalTime().toString(u"yyyy-MM-dd HH-mm-ss"_s) + u" log%1.bin"_s.arg(id);
}

QString LogsController::folder() const
{
    const QString f = Settings::instance().value(kFolderKey).toString();
    return f.isEmpty() ? DataRoot::logsDir() + u"/dataflash"_s : f;
}

void LogsController::setFolder(const QUrl &url)
{
    const QString f = url.isLocalFile() ? url.toLocalFile() : url.toString();
    if (f.isEmpty()) return;
    Settings::instance().setValue(kFolderKey, f);
    emit folderChanged();
    markSaved();
}

QUrl LogsController::folderUrl() const { return QUrl::fromLocalFile(folder()); }

void LogsController::openFolder()
{
    QDir().mkpath(folder());
    QDesktopServices::openUrl(QUrl::fromLocalFile(folder()));
}

QString LogsController::totalSize() const
{
    double total = 0;
    for (const auto &r : m_model.rows()) total += r.size;
    return sizeText(total);
}

int LogsController::selectedCount() const
{
    int n = 0;
    for (const auto &r : m_model.rows()) n += r.selected ? 1 : 0;
    return n;
}

QString LogsController::selectedSize() const
{
    double total = 0;
    for (const auto &r : m_model.rows())
        if (r.selected) total += r.size;
    return sizeText(total);
}

double LogsController::queueProgress() const
{
    if (m_queueTotal == 0 || m_queueBytes == 0) return 0;
    return std::min(1.0, double(m_queueBytesDone + m_currentBytes) / double(m_queueBytes));
}

void LogsController::refresh()
{
    if (!m_linkReady) {
        DialogHub::toast(u"Not connected"_s);
        return;
    }
    if (downloading()) {
        DialogHub::toast(u"A download is in progress"_s);
        return;
    }
    m_dl->setTarget(m_cmd->targetSystem(), m_cmd->targetComponent());
    m_model.reset({});
    emit selectionChanged();
    setStatus(u"Asking the vehicle for its logs…"_s);
    m_dl->requestList();
}

void LogsController::rebuildFromList()
{
    QList<LogListModel::Row> rows;
    const QList<LogDownloader::Entry> entries = m_dl->entries();
    // Newest first, as Mission Planner lists them
    for (auto it = entries.crbegin(); it != entries.crend(); ++it) {
        LogListModel::Row r;
        r.id = it->id;
        r.timeUtc = it->timeUtc;
        r.size = it->size;
        rows << r;
    }
    m_model.reset(rows);
    markSaved();
    emit selectionChanged();
}

void LogsController::markSaved()
{
    // A file of the same name and size in the folder: that log is already here
    const QString dir = folder();
    for (auto &r : m_model.rows()) {
        if (r.state == "downloading"_L1 || r.state == "queued"_L1) continue;
        const QString path = dir + '/'_L1 + fileName(r.id, r.timeUtc, r.size);
        const QFileInfo fi(path);
        if (fi.exists() && fi.size() == qint64(r.size)) {
            r.state = u"saved"_s;
            r.path = path;
            r.progress = 1;
            r.detail = fi.fileName();
        } else if (r.state == "saved"_L1) {
            r.state.clear();
            r.path.clear();
            r.progress = 0;
            r.detail.clear();
        }
    }
    m_model.touchAll();
}

void LogsController::toggle(int id)
{
    if (LogListModel::Row *r = m_model.find(id)) {
        r->selected = !r->selected;
        m_model.touch(id);
        emit selectionChanged();
    }
}

void LogsController::selectAll(bool on)
{
    for (auto &r : m_model.rows()) r.selected = on;
    m_model.touchAll();
    emit selectionChanged();
}

void LogsController::download(int id)
{
    if (downloading()) return;
    for (auto &r : m_model.rows()) r.selected = r.id == id;
    m_model.touchAll();
    emit selectionChanged();
    downloadSelected();
}

void LogsController::downloadSelected()
{
    if (!m_linkReady) {
        DialogHub::toast(u"Not connected"_s);
        return;
    }
    if (downloading()) return;
    m_queue.clear();
    m_queueBytes = m_queueBytesDone = m_currentBytes = 0;
    m_queueDone = m_queueFailed = 0;
    for (auto &r : m_model.rows()) {
        if (!r.selected) continue;
        m_queue << r.id;
        m_queueBytes += r.size;
        r.state = u"queued"_s;
        r.progress = 0;
        r.detail.clear();
    }
    m_model.touchAll();
    if (m_queue.isEmpty()) {
        DialogHub::toast(u"Tick the logs to download first"_s);
        return;
    }
    if (!QDir().mkpath(folder())) {
        setStatus(u"Cannot create "_s + folder(), true);
        m_queue.clear();
        markSaved();
        return;
    }
    m_queueTotal = int(m_queue.size());
    m_dl->setTarget(m_cmd->targetSystem(), m_cmd->targetComponent());
    LOG_I("logs") << "downloading" << m_queueTotal << "logs," << m_queueBytes << "B to" << folder();
    startNext();
}

void LogsController::startNext()
{
    while (!m_queue.isEmpty()) {
        const int id = m_queue.takeFirst();
        LogListModel::Row *r = m_model.find(id);
        if (!r) continue;
        const QString path = folder() + '/'_L1 + fileName(r->id, r->timeUtc, r->size);
        r->state = u"downloading"_s;
        r->progress = 0;
        m_model.touch(id);
        QString err;
        if (m_dl->download(r->id, r->size, path, &err)) {
            emit stateChanged();
            return;
        }
        r->state = u"failed"_s;
        r->detail = err;
        m_model.touch(id);
        ++m_queueFailed;
    }
    // The queue is done: the vehicle may log again
    m_dl->sendRequestEnd();
    const QString where = QDir::toNativeSeparators(folder());
    if (m_queueFailed == 0) setStatus(u"%1 logs downloaded to %2"_s.arg(m_queueDone).arg(where));
    else setStatus(u"%1 logs downloaded, %2 failed"_s.arg(m_queueDone).arg(m_queueFailed), true);
    DialogHub::toast(m_queueFailed == 0 ? u"%1 logs downloaded"_s.arg(m_queueDone)
                                        : u"%1 logs downloaded, %2 failed"_s.arg(m_queueDone).arg(m_queueFailed));
    m_queueTotal = 0;
    emit stateChanged();
}

void LogsController::cancel()
{
    m_queue.clear();
    m_queueTotal = 0;
    m_dl->cancel();
    for (auto &r : m_model.rows())
        if (r.state == "queued"_L1 || r.state == "downloading"_L1) {
            r.state.clear();
            r.progress = 0;
            r.detail.clear();
        }
    markSaved();
    setStatus(u"Cancelled"_s);
}

void LogsController::eraseAll()
{
    if (!m_linkReady) {
        DialogHub::toast(u"Not connected"_s);
        return;
    }
    if (downloading()) return;
    const QString what = m_model.rows().isEmpty() ? u"every log"_s : u"all %1 logs (%2)"_s.arg(m_model.rows().size()).arg(totalSize());
    DialogHub::confirm(u"Erase %1 from the vehicle's SD card? This cannot be undone."_s.arg(what), [this](bool ok) {
        if (!ok) return;
        m_dl->setTarget(m_cmd->targetSystem(), m_cmd->targetComponent());
        m_dl->sendErase();
        m_model.reset({});
        emit selectionChanged();
        setStatus(u"Erasing the logs…"_s);
        LOG_I("logs") << "LOG_ERASE sent";
        // The card takes a moment: list again
        QTimer::singleShot(2500, this, [this] { if (m_linkReady && !downloading()) refresh(); });
    });
}

void LogsController::replay(int id)
{
    const LogListModel::Row *r = m_model.find(id);
    if (!r || r->path.isEmpty()) return;
    ReplayController *rc = ReplayController::s_instance;
    if (!rc || !rc->sectionVisible()) {
        DialogHub::toast(u"Disconnect first: LOG REPLAY needs no live link"_s);
        return;
    }
    rc->openPath(r->path);
    DialogHub::toast(u"%1 opened in LOG REPLAY (FLIGHT DATA)"_s.arg(QFileInfo(r->path).fileName()));
}

} // namespace corv
