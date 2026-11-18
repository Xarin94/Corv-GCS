// LogsController.h - SETUP → LOG DOWNLOAD ("Logs" QML singleton): the DataFlash
// logs on the vehicle's SD card, listed, downloaded to a folder and opened in
// LOG REPLAY (Mission Planner's DataFlash Logs → Download DataFlash Log Via
// Mavlink). The protocol is LogDownloader's; this keeps the list, the queue
// of the logs ticked and the folder (settings key logs-download-folder,
// <data>/logs/dataflash by default).
//
// Files are named after the log's date as the vehicle reports it, local time
// ("2026-11-17 10-23-45 log12.bin"), "log12 1048576B.bin" when the log has no
// GPS time. A file of that name and size already in the folder marks the log as
// saved.
#pragma once

#include <QAbstractListModel>
#include <QObject>
#include <QQmlEngine>
#include <QTimer>
#include <QUrl>

namespace corv {

class CommandSender;
class LinkManager;
class LogDownloader;
class MessageBus;
class VehicleHub;

class LogListModel : public QAbstractListModel
{
    Q_OBJECT
public:
    enum Roles { IdRole = Qt::UserRole + 1, DateRole, SizeRole, BytesRole, SelectedRole, StateRole, ProgressRole, PathRole, DetailRole };
    struct Row {
        int id = 0;
        quint32 timeUtc = 0;
        quint32 size = 0;
        bool selected = false;
        QString state;          // "", queued, downloading, saved, failed
        double progress = 0;
        QString path;
        QString detail;         // the error of a failed download, the rate of the current one
    };
    explicit LogListModel(QObject *parent = nullptr) : QAbstractListModel(parent) {}
    int rowCount(const QModelIndex & = {}) const override { return int(m_rows.size()); }
    QVariant data(const QModelIndex &index, int role) const override;
    QHash<int, QByteArray> roleNames() const override;

    void reset(const QList<Row> &rows);
    QList<Row> &rows() { return m_rows; }
    const QList<Row> &rows() const { return m_rows; }
    Row *find(int id);
    void touch(int id);
    void touchAll();

private:
    QList<Row> m_rows;
};

class LogsController : public QObject
{
    Q_OBJECT
    QML_NAMED_ELEMENT(Logs)
    QML_SINGLETON
    Q_PROPERTY(QAbstractItemModel *entries READ entries CONSTANT)
    Q_PROPERTY(bool listing READ listing NOTIFY stateChanged)
    Q_PROPERTY(bool downloading READ downloading NOTIFY stateChanged)
    Q_PROPERTY(QString statusText READ statusText NOTIFY stateChanged)
    Q_PROPERTY(bool statusError READ statusError NOTIFY stateChanged)
    Q_PROPERTY(double queueProgress READ queueProgress NOTIFY stateChanged)
    Q_PROPERTY(int count READ count NOTIFY stateChanged)
    Q_PROPERTY(QString totalSize READ totalSize NOTIFY stateChanged)
    Q_PROPERTY(int selectedCount READ selectedCount NOTIFY selectionChanged)
    Q_PROPERTY(QString selectedSize READ selectedSize NOTIFY selectionChanged)
    Q_PROPERTY(QString folder READ folder NOTIFY folderChanged)
    Q_PROPERTY(QUrl folderUrl READ folderUrl NOTIFY folderChanged)
    Q_PROPERTY(bool linkReady READ linkReady NOTIFY linkChanged)
    Q_PROPERTY(bool armed READ armed NOTIFY linkChanged)

public:
    LogsController(LinkManager *link, MessageBus *bus, CommandSender *cmd, VehicleHub *hub, QObject *parent);
    ~LogsController() override;
    static LogsController *s_instance;
    static LogsController *create(QQmlEngine *, QJSEngine *) { QJSEngine::setObjectOwnership(s_instance, QJSEngine::CppOwnership); return s_instance; }

    QAbstractItemModel *entries() { return &m_model; }
    bool listing() const;
    bool downloading() const;
    QString statusText() const { return m_status; }
    bool statusError() const { return m_statusError; }
    double queueProgress() const;
    int count() const { return int(m_model.rows().size()); }
    QString totalSize() const;
    int selectedCount() const;
    QString selectedSize() const;
    QString folder() const;
    QUrl folderUrl() const;
    bool linkReady() const { return m_linkReady; }
    bool armed() const { return m_armed; }

    /// Ask the vehicle for its list of logs.
    Q_INVOKABLE void refresh();
    Q_INVOKABLE void toggle(int id);
    Q_INVOKABLE void selectAll(bool on);
    /// Download the logs ticked, one after the other.
    Q_INVOKABLE void downloadSelected();
    Q_INVOKABLE void download(int id);
    Q_INVOKABLE void cancel();
    /// LOG_ERASE, after a confirmation: every log on the card.
    Q_INVOKABLE void eraseAll();
    Q_INVOKABLE void setFolder(const QUrl &folder);
    Q_INVOKABLE void openFolder();
    /// Open a downloaded log in LOG REPLAY (needs no live link).
    Q_INVOKABLE void replay(int id);

    /// The file a log is saved as.
    static QString fileName(int id, quint32 timeUtc, quint32 size);
    /// 1.2 MB, 340 KB...
    static QString sizeText(double bytes);

signals:
    void stateChanged();
    void selectionChanged();
    void folderChanged();
    void linkChanged();

private:
    void setStatus(const QString &text, bool error = false);
    void rebuildFromList();
    void startNext();
    void markSaved();

    LinkManager *m_link;
    MessageBus *m_bus;
    CommandSender *m_cmd;
    VehicleHub *m_hub;
    LogDownloader *m_dl = nullptr;
    LogListModel m_model;
    QList<int> m_queue;
    int m_queueTotal = 0, m_queueDone = 0, m_queueFailed = 0;
    qint64 m_queueBytes = 0, m_queueBytesDone = 0, m_currentBytes = 0;
    QString m_status;
    bool m_statusError = false;
    QTimer m_linkPoll;
    bool m_linkReady = false, m_armed = false;
};

} // namespace corv
