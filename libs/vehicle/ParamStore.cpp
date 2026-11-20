#include "vehicle/ParamStore.h"

#include "core/Clock.h"
#include "core/DebugLog.h"
#include "link/LinkManager.h"
#include "link/MavFtp.h"
#include "link/Mavlink.h"
#include "link/MessageBus.h"
#include "vehicle/CommandSender.h"
#include "vehicle/ParamCatalog.h"
#include "vehicle/VehicleHub.h"

#include <QFile>
#include <QPointer>
#include <QRegularExpression>
#include <QSaveFile>
#include <cmath>
#include <cstring>

using namespace Qt::StringLiterals;

namespace corv {

ParamStore *ParamStore::s_instance = nullptr;

namespace {
constexpr int kInterRequestGapMs = 80;
const QString kParamFile = u"@PARAM/param.pck"_s;
} // namespace

ParamStore::ParamStore(LinkManager *link, MessageBus *bus, CommandSender *cmd, VehicleHub *hub, QObject *parent)
    : QObject(parent), m_link(link), m_bus(bus), m_cmd(cmd), m_hub(hub)
{
    s_instance = this;
    m_ftp = std::make_unique<MavFtpClient>([this](const QByteArray &payload, int sys, int comp) {
        mavlink_message_t m;
        mavlink_msg_file_transfer_protocol_pack(mav::kGcsSystemId, mav::kGcsComponentId, &m, 0, uint8_t(sys), uint8_t(comp),
                                                reinterpret_cast<const uint8_t *>(payload.constData()));
        if (m_link) m_link->send(m);
    });

    m_bus->subscribe(MAVLINK_MSG_ID_PARAM_VALUE, this, [this](const MavMessage &mm) {
        mavlink_param_value_t pv;
        mavlink_msg_param_value_decode(&mm.msg, &pv);
        onParamValue(mav::fieldString(pv.param_id, 16).trimmed(), pv.param_value, pv.param_type, pv.param_index, pv.param_count);
    });
    m_bus->subscribe(MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL, this, [this](const MavMessage &mm) {
        mavlink_file_transfer_protocol_t f;
        mavlink_msg_file_transfer_protocol_decode(&mm.msg, &f);
        m_ftp->onPacket(QByteArray(reinterpret_cast<const char *>(f.payload), int(sizeof f.payload)));
    });

    m_learnTimer.setSingleShot(true);
    m_learnTimer.setInterval(1000);
    connect(&m_learnTimer, &QTimer::timeout, this, &ParamStore::flushLearned);
    m_changedTimer.setSingleShot(true);
    m_changedTimer.setInterval(200);
    connect(&m_changedTimer, &QTimer::timeout, this, &ParamStore::changed);
}

ParamStore::~ParamStore()
{
    flushLearned();
    if (s_instance == this) s_instance = nullptr;
}

const ParamValue *ParamStore::find(const QString &name) const
{
    auto it = m_params.find(name);
    return it == m_params.end() ? nullptr : &it->second;
}

QStringList ParamStore::names() const
{
    QStringList l;
    l.reserve(qsizetype(m_params.size()));
    for (const auto &kv : m_params) l << kv.first;
    return l;
}

bool ParamStore::isMavlink() const
{
    return m_hub && m_hub->snapshot()->connectionType.startsWith("mavlink"_L1);
}

void ParamStore::flushLearned()
{
    if (m_toLearn.isEmpty()) return;
    ParamCatalog::instance().learn(m_toLearn);
    m_toLearn.clear();
}

void ParamStore::onParamValue(const QString &name, double value, int type, int index, int count)
{
    if (name.isEmpty()) return;
    m_params[name] = {value, type, index};
    m_total = count;
    // Every name the vehicle reports is remembered for the catalog of a later, offline session
    m_toLearn << name;
    if (!m_learnTimer.isActive()) m_learnTimer.start();
    if (m_streamRead) {
        const int got = received();
        setRead(got < m_total, u"%1/%2"_s.arg(got).arg(m_total), m_total ? double(got) / m_total : 0);
        if (got >= m_total) {
            m_streamRead = false;
            LOG_I("param") << "parameter list read one by one:" << got << "in" << (monoMs() - m_readStarted) << "ms";
        }
    }
    if (!m_changedTimer.isActive()) m_changedTimer.start();
}

void ParamStore::setRead(bool reading, const QString &text, double fraction)
{
    m_reading = reading;
    m_readText = text;
    m_readFraction = std::clamp(fraction, 0.0, 1.0);
    emit readChanged();
}

void ParamStore::ingestPack(const QList<std::tuple<QString, double, int>> &params, int total)
{
    // The file is the vehicle's whole list: it replaces what was loaded
    m_params.clear();
    int i = 0;
    QStringList names;
    for (const auto &[name, value, type] : params) {
        m_params[name] = {value, type, i++};
        names << name;
    }
    m_total = total ? total : int(params.size());
    ParamCatalog::instance().learn(names);
    emit changed();
}

void ParamStore::startStream()
{
    m_params.clear();
    m_total = 0;
    m_streamRead = true;
    setRead(true, u"0/?"_s, 0);
    emit changed();
    m_cmd->requestAllParameters();
}

void ParamStore::readAll()
{
    if (m_reading) return;
    m_readStarted = monoMs();
    if (!isMavlink()) {
        // MSP has no FTP; PARAM_REQUEST_LIST goes wherever the link takes it
        startStream();
        return;
    }
    setRead(true, u"FTP · opening…"_s, 0);
    MavFtpClient::Options o;
    o.targetSystem = m_cmd->targetSystem();
    o.targetComponent = m_cmd->targetComponent();
    QPointer<ParamStore> self(this);
    LOG_I("ftp") << "parameter download requested (sys" << o.targetSystem << ")";
    m_ftp->readFile(
        kParamFile, o,
        [self](const MavFtpClient::Progress &p) {
            if (!self) return;
            // Count the parameters decodable so far: the progress the operator sees
            const ftp::ParamPack part = ftp::decodeParamPack(self->m_ftp->partial());
            const QString kb = QString::number(p.bytes / 1024.0, 'f', 1);
            self->setRead(true, part.total ? u"%1/%2 · FTP %3 kB"_s.arg(part.count).arg(part.total).arg(kb) : u"FTP %1 kB"_s.arg(kb),
                          part.total ? double(part.count) / part.total : 0);
        },
        [self](bool ok, const QByteArray &data, const QString &code, const QString &text) {
            if (!self) return;
            if (ok) {
                const ftp::ParamPack pack = ftp::decodeParamPack(data);
                if (pack.ok && pack.complete) {
                    QList<std::tuple<QString, double, int>> list;
                    list.reserve(pack.params.size());
                    for (const auto &p : pack.params) list.append({p.name, p.value, p.type});
                    self->ingestPack(list, pack.total);
                    const double s = (monoMs() - self->m_readStarted) / 1000.0;
                    LOG_I("ftp") << "parameters:" << pack.count << "in" << data.size() << "B," << s << "s";
                    self->setRead(false, u"%1/%2 · FTP %3 s"_s.arg(pack.total).arg(pack.total).arg(s, 0, 'f', 1), 1);
                    return;
                }
                LOG_W("ftp") << "parameter file not decodable:" << (pack.error.isEmpty() ? u"%1/%2 entries"_s.arg(pack.count).arg(pack.total) : pack.error);
            } else {
                LOG_W("param") << "FTP download failed (" << code << "):" << text << "- reading one by one";
                if (code == "CANCELLED"_L1) {
                    self->setRead(false, text, 0);
                    return;
                }
            }
            // No FTP on this firmware, a refused file or a dead transfer: the old way works everywhere
            self->startStream();
            self->setRead(true, u"no FTP · one by one…"_s, 0);
        });
}

void ParamStore::cancelRead()
{
    if (m_ftp->busy()) m_ftp->cancel(u"cancelled by the operator"_s);
    if (m_streamRead) {
        m_streamRead = false;
        setRead(false, u"%1/%2 · stopped"_s.arg(received()).arg(m_total), m_total ? double(received()) / m_total : 0);
    }
}

// ── Single reads ─────────────────────────────────────────────────────────────

void ParamStore::enqueueFetch(const QStringList &names)
{
    int added = 0;
    for (const QString &raw : names) {
        const QString name = ParamCatalog::sanitize(raw);
        if (name.isEmpty() || m_fetchState.value(name) == "pending"_L1 || m_queue.contains(name)) continue;
        m_queue << name;
        m_fetchState[name] = u"pending"_s;
        ++added;
    }
    if (!added) return;
    emit queueChanged();
    if (!m_queueBusy) {
        m_queueBusy = true;
        m_queueDone = m_queueFailed = 0;
        drainQueue();
    }
}

void ParamStore::drainQueue()
{
    if (m_queue.isEmpty()) {
        m_queueBusy = false;
        emit queueChanged();
        return;
    }
    emit queueChanged();
    const QString name = m_queue.first();
    QPointer<ParamStore> self(this);
    m_cmd->fetchParameter(name, m_fetchTimeoutMs, 2, [self, name](bool ok, float, int) {
        if (!self) return;
        self->m_fetchState[name] = ok ? u"ok"_s : u"fail"_s;
        if (ok) {
            ++self->m_queueDone;
            ParamCatalog::instance().learn({name});
        } else {
            ++self->m_queueFailed;
        }
        self->m_queue.removeFirst();
        emit self->changed();
        emit self->queueChanged();
        QTimer::singleShot(self->m_queue.isEmpty() ? 0 : kInterRequestGapMs, self, &ParamStore::drainQueue);
    });
}

QString ParamStore::queueText() const
{
    if (m_queueBusy && !m_queue.isEmpty()) {
        const int queued = int(m_queue.size()) - 1;
        return u"Reading %1%2"_s.arg(m_queue.first(), queued > 0 ? u" — %1 queued"_s.arg(queued) : QString());
    }
    if (m_queueDone || m_queueFailed)
        return m_queueFailed ? u"%1 read, %2 no reply (raise the timeout)"_s.arg(m_queueDone).arg(m_queueFailed)
                             : u"%1 read"_s.arg(m_queueDone);
    return {};
}

// ── Writes ───────────────────────────────────────────────────────────────────

void ParamStore::write(const QString &name, double value, std::function<void(bool, double)> done)
{
    const ParamValue *p = find(name);
    const int type = p ? p->type : MAV_PARAM_TYPE_REAL32;
    QPointer<ParamStore> self(this);
    m_cmd->setParameter(name, float(value), type, [self, name, type, done](bool ok, float v) {
        if (self && ok) {
            auto &e = self->m_params[name];
            e.value = v;
            e.type = type;
            emit self->changed();
        }
        if (done) done(ok, v);
    });
}

void ParamStore::writeMany(const QList<QPair<QString, double>> &list, std::function<void(int, int)> done)
{
    struct Run {
        QList<QPair<QString, double>> list;
        int next = 0, written = 0;
        std::function<void(int, int)> done;
        std::function<void()> step;
    };
    auto run = std::make_shared<Run>();
    run->list = list;
    run->done = std::move(done);
    QPointer<ParamStore> self(this);
    std::weak_ptr<Run> weak = run;
    run->step = [self, weak] {
        auto r = weak.lock();
        if (!r || !self) return;
        if (r->next >= r->list.size()) {
            if (r->done) r->done(r->written, int(r->list.size()));
            r->step = nullptr;   // breaks the cycle: the run is over
            return;
        }
        const auto item = r->list[r->next++];
        self->write(item.first, item.second, [r](bool ok, double) {
            if (ok) ++r->written;
            else LOG_W("param") << "failed to write" << r->list[r->next - 1].first;
            if (r->step) r->step();
        });
    };
    run->step();
}

bool ParamStore::saveFile(const QString &path, QString *error) const
{
    QSaveFile f(path);
    if (!f.open(QIODevice::WriteOnly)) {
        if (error) *error = f.errorString();
        return false;
    }
    QStringList lines;
    for (const auto &[name, p] : m_params) lines << name + ','_L1 + ParamCatalog::formatValue(p.value, p.type);
    f.write(lines.join('\n'_L1).toUtf8());
    if (!f.commit()) {
        if (error) *error = f.errorString();
        return false;
    }
    return true;
}

void ParamStore::loadFile(const QString &path, std::function<void(int, int, const QString &)> done)
{
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
        if (done) done(0, 0, f.errorString());
        return;
    }
    QList<QPair<QString, double>> list;
    QStringList names;
    static const QRegularExpression sep(u"[,\\s]+"_s);
    for (const QString &raw : QString::fromUtf8(f.readAll()).split('\n'_L1)) {
        const QString line = raw.trimmed();
        if (line.isEmpty() || line.startsWith('#'_L1)) continue;
        const QStringList parts = line.split(sep, Qt::SkipEmptyParts);
        if (parts.isEmpty()) continue;
        names << parts[0];
        bool ok = false;
        const double v = parts.size() >= 2 ? parts[1].toDouble(&ok) : 0;
        if (ok && std::isfinite(v)) list.append({parts[0].trimmed().toUpper(), v});
    }
    // Learned even if a write fails: a .param dump of the same airframe seeds the catalog
    ParamCatalog::instance().learn(names);
    writeMany(list, [done](int written, int total) {
        if (done) done(written, total, {});
    });
}

} // namespace corv
