// ParamStore.h - the vehicle's parameters as the GCS knows them (STATE.parameters
// and the logic of ParametersPageController.js / TabController.js).
//
// Every PARAM_VALUE lands here, whoever asked for it. READ ALL takes the whole
// list over MAVLink FTP (@PARAM/param.pck: seconds) and falls back to
// PARAM_REQUEST_LIST (one PARAM_VALUE per parameter: minutes on a slow radio)
// when the vehicle has no FTP, refuses the file or the transfer stalls. Single
// reads are queued and serialized (two PARAM_REQUEST_READs in flight on a lossy
// link mostly collide), each retried twice, 80 ms apart. Every name received is
// learned by the catalog (batched). Writes wait for the PARAM_VALUE echo.
// GUI thread.
#pragma once

#include <QHash>
#include <QObject>
#include <QSet>
#include <QStringList>
#include <QTimer>
#include <functional>
#include <map>
#include <memory>

namespace corv {

class CommandSender;
class LinkManager;
class MavFtpClient;
class MessageBus;
class VehicleHub;

struct ParamValue {
    double value = 0;
    int type = 9;      // MAV_PARAM_TYPE (ArduPilot reports REAL32 for everything)
    int index = -1;
};

class ParamStore : public QObject
{
    Q_OBJECT
public:
    ParamStore(LinkManager *link, MessageBus *bus, CommandSender *cmd, VehicleHub *hub, QObject *parent = nullptr);
    ~ParamStore() override;

    static ParamStore *s_instance;

    const std::map<QString, ParamValue> &all() const { return m_params; }
    const ParamValue *find(const QString &name) const;
    QStringList names() const;
    int total() const { return m_total; }
    int received() const { return int(m_params.size()); }

    // READ ALL
    bool reading() const { return m_reading; }
    QString readText() const { return m_readText; }
    double readFraction() const { return m_readFraction; }
    void readAll();
    void cancelRead();

    // Single reads (the catalog)
    void enqueueFetch(const QStringList &names);
    void setFetchTimeout(int ms) { m_fetchTimeoutMs = ms; }
    int fetchTimeout() const { return m_fetchTimeoutMs; }
    /// "", "pending", "ok", "fail"
    QString fetchState(const QString &name) const { return m_fetchState.value(name); }
    QString queueText() const;
    bool queueBusy() const { return m_queueBusy; }
    bool queueError() const { return !m_queueBusy && m_queueFailed > 0; }

    // Writes
    void write(const QString &name, double value, std::function<void(bool ok, double value)> done = {});
    void writeMany(const QList<QPair<QString, double>> &list, std::function<void(int written, int total)> done);
    /// NAME,VALUE per line, sorted (Mission Planner's .param format).
    bool saveFile(const QString &path, QString *error = nullptr) const;
    /// Every line written to the vehicle in turn; names are learned even if a write fails.
    void loadFile(const QString &path, std::function<void(int written, int total, const QString &error)> done);

signals:
    /// The list changed (values, entries); listeners coalesce.
    void changed();
    void readChanged();
    void queueChanged();

private:
    void onParamValue(const QString &name, double value, int type, int index, int count);
    void ingestPack(const QList<std::tuple<QString, double, int>> &params, int total);
    void startStream();
    void readPack(int tries);
    void streamTick();
    void setRead(bool reading, const QString &text, double fraction);
    void drainQueue();
    void flushLearned();
    bool isMavlink() const;

    LinkManager *m_link;
    MessageBus *m_bus;
    CommandSender *m_cmd;
    VehicleHub *m_hub;
    std::unique_ptr<MavFtpClient> m_ftp;
    std::map<QString, ParamValue> m_params;
    int m_total = 0;

    bool m_reading = false;
    bool m_streamRead = false;   // PARAM_REQUEST_LIST running
    QSet<int> m_streamSeen;      // places of the list that came
    QTimer m_streamTimer;        // what did not is asked again
    qint64 m_streamHeard = 0;
    int m_streamQuiet = 0;       // rounds in a row that brought nothing
    QString m_readText;
    double m_readFraction = 0;
    qint64 m_readStarted = 0;

    QStringList m_queue;
    QHash<QString, QString> m_fetchState;
    bool m_queueBusy = false;
    int m_queueDone = 0, m_queueFailed = 0;
    int m_fetchTimeoutMs = 4000;

    QStringList m_toLearn;
    QTimer m_learnTimer;
    QTimer m_changedTimer;   // coalesces bursts of PARAM_VALUE into one changed()
};

} // namespace corv
