#include "ui/ParamsController.h"

#include "core/DebugLog.h"
#include "ui/DialogHub.h"
#include "vehicle/FlightModes.h"
#include "vehicle/ParamCatalog.h"
#include "vehicle/ParamMeta.h"
#include "vehicle/ParamStore.h"
#include "vehicle/VehicleHub.h"

#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QPointer>
#include <cmath>

namespace corv {

ParamsController *ParamsController::s_instance = nullptr;

namespace {
constexpr int kCatalogMaxRows = 400;   // rows listed before asking to refine
constexpr int kBulkConfirm = 40;

QString numberText(double v)
{
    return ParamCatalog::formatValue(v, 9);
}

QString rangeText(const ParamInfo &m)
{
    if (!m.hasRange) return {};
    QString t = numberText(m.low) + QStringLiteral(" – ") + numberText(m.high);
    if (!m.units.isEmpty()) t += QLatin1Char(' ') + m.units;
    return t;
}

QString kindOf(const ParamInfo *m)
{
    if (!m) return QStringLiteral("number");
    if (m->isBitmask()) return QStringLiteral("bitmask");
    if (m->hasValues()) return QStringLiteral("values");
    return QStringLiteral("number");
}

/// What the value means: the listed label, the bits set, else the documented range.
QString meaningOf(const ParamInfo *m, bool haveValue, double v)
{
    if (!m) return {};
    if (m->isBitmask()) return haveValue ? m->bitsLabel(std::llround(v)) : QString();
    if (m->hasValues()) {
        if (!haveValue) return {};
        const QString l = m->valueLabel(v);
        return l.isEmpty() ? QStringLiteral("(not a listed value)") : l;
    }
    return rangeText(*m);
}
} // namespace

// ── Table ────────────────────────────────────────────────────────────────────

ParamTableModel::ParamTableModel(ParamStore *store, QObject *parent) : QAbstractListModel(parent), m_store(store) {}

QHash<int, QByteArray> ParamTableModel::roleNames() const
{
    return {{NameRole, "name"}, {ValueRole, "value"}, {DescRole, "desc"}, {EditedRole, "edited"}, {EditTextRole, "editText"},
            {TitleRole, "title"}, {UnitsRole, "units"}, {KindRole, "kind"}, {MeaningRole, "meaning"}, {OptionsRole, "options"},
            {OutOfRangeRole, "outOfRange"}, {RebootRole, "reboot"}, {ReadOnlyRole, "readOnly"}};
}

bool ParamTableModel::currentValue(const QString &name, double *v) const
{
    if (auto it = m_edits.constFind(name); it != m_edits.cend()) {
        *v = *it;
        return true;
    }
    if (const ParamValue *p = m_store->find(name)) {
        *v = p->value;
        return true;
    }
    return false;
}

QVariant ParamTableModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid() || index.row() >= m_rows.size()) return {};
    const QString &name = m_rows[index.row()];
    const ParamValue *p = m_store->find(name);
    switch (role) {
    case NameRole: return name;
    case ValueRole: return p ? ParamCatalog::formatValue(p->value, p->type) : QString();
    case EditedRole: return m_edits.contains(name);
    case EditTextRole: return m_editText.value(name, p ? ParamCatalog::formatValue(p->value, p->type) : QString());
    default: break;
    }
    const ParamInfo *m = ParamMeta::instance().find(name, m_vehicleClass);
    double v = 0;
    const bool have = currentValue(name, &v);
    switch (role) {
    case DescRole: return m && !m->description.isEmpty() ? m->description : ParamCatalog::description(name);
    case TitleRole: return m ? m->displayName : QString();
    case UnitsRole: return m ? m->units : QString();
    case KindRole: return kindOf(m);
    case MeaningRole: return meaningOf(m, have, v);
    case OptionsRole: {
        // The listed values for a select: "value: label", and the current one when it is not listed
        QVariantList opts;
        if (!m || m->isBitmask() || !m->hasValues()) return opts;
        if (have && m->valueLabel(v).isEmpty())
            opts << QVariantMap{{QStringLiteral("value"), numberText(v)}, {QStringLiteral("label"), numberText(v) + QStringLiteral(": (not listed)")}};
        for (const auto &[value, label] : m->values)
            opts << QVariantMap{{QStringLiteral("value"), numberText(value)}, {QStringLiteral("label"), numberText(value) + QStringLiteral(": ") + label}};
        return opts;
    }
    case OutOfRangeRole: return m && have && m->outOfRange(v);
    case RebootRole: return m && m->rebootRequired;
    case ReadOnlyRole: return m && m->readOnly;
    default: return {};
    }
}

void ParamTableModel::setFilter(const QString &f)
{
    if (f == m_filter) return;
    m_filter = f;
    refresh();
}

void ParamTableModel::setVehicleClass(const QString &c)
{
    if (c == m_vehicleClass) return;
    m_vehicleClass = c;
    touchAll();
}

void ParamTableModel::touchAll()
{
    if (!m_rows.isEmpty()) emit dataChanged(index(0), index(int(m_rows.size()) - 1));
}

void ParamTableModel::refresh()
{
    // The filter matches the name, or the display name of the documentation ("battery", "throttle")
    QStringList rows;
    for (const auto &kv : m_store->all()) {
        bool match = m_filter.isEmpty() || kv.first.contains(m_filter);
        if (!match && m_filter.size() >= 3) {
            const ParamInfo *m = ParamMeta::instance().find(kv.first, m_vehicleClass);
            match = m && m->displayName.contains(m_filter, Qt::CaseInsensitive);
        }
        if (match) rows << kv.first;
    }
    // A value typed and then confirmed by the vehicle is no longer a change
    for (auto it = m_edits.begin(); it != m_edits.end();) {
        const ParamValue *p = m_store->find(it.key());
        if (p && std::abs(p->value - it.value()) < 1e-6) {
            m_editText.remove(it.key());
            it = m_edits.erase(it);
        } else {
            ++it;
        }
    }
    if (rows == m_rows) {
        if (!m_rows.isEmpty()) emit dataChanged(index(0), index(int(m_rows.size()) - 1));
        return;
    }
    beginResetModel();
    m_rows = rows;
    endResetModel();
}

bool ParamTableModel::setEdit(const QString &name, const QString &text)
{
    const ParamValue *p = m_store->find(name);
    const int type = p ? p->type : 9;
    bool ok = false;
    const double v = type >= 1 && type <= 6 ? double(text.trimmed().toLongLong(&ok)) : text.trimmed().toDouble(&ok);
    const qsizetype row = m_rows.indexOf(name);
    if (!ok || !std::isfinite(v)) return false;
    if (p && p->value != v) {
        m_edits[name] = v;
        m_editText[name] = text.trimmed();
    } else {
        m_edits.remove(name);
        m_editText.remove(name);
    }
    if (row >= 0) emit dataChanged(index(int(row)), index(int(row)));
    return m_edits.contains(name);
}

void ParamTableModel::clearEdits()
{
    m_edits.clear();
    m_editText.clear();
    if (!m_rows.isEmpty()) emit dataChanged(index(0), index(int(m_rows.size()) - 1));
}

// ── Catalog ──────────────────────────────────────────────────────────────────

ParamCatalogModel::ParamCatalogModel(ParamStore *store, QObject *parent) : QAbstractListModel(parent), m_store(store) {}

QHash<int, QByteArray> ParamCatalogModel::roleNames() const
{
    return {{NameRole, "name"}, {ValueRole, "value"}, {LoadedRole, "loaded"}, {CustomRole, "custom"}, {StateRole, "fetchState"}, {FavoriteRole, "favorite"}};
}

QVariant ParamCatalogModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid() || index.row() >= m_rows.size()) return {};
    const QString &name = m_rows[index.row()];
    const ParamValue *p = m_store->find(name);
    switch (role) {
    case NameRole: return name;
    case ValueRole: return p ? ParamCatalog::formatValue(p->value, p->type) : QString();
    case LoadedRole: return p != nullptr;
    case CustomRole: return name == m_custom;
    case StateRole: return m_store->fetchState(name);
    case FavoriteRole: return ParamCatalog::instance().isFavorite(name);
    default: return {};
    }
}

void ParamCatalogModel::rebuild(const QStringList &catalog, const QString &filter, const QString &group)
{
    QStringList visible;
    for (const QString &n : catalog)
        if ((group.isEmpty() || ParamCatalog::groupOf(n) == group) && (filter.isEmpty() || n.contains(filter))) visible << n;
    // A name the catalog has never seen is offered anyway: the autopilot is the authority
    QStringList rows;
    m_custom.clear();
    if (!filter.isEmpty() && !catalog.contains(filter)) {
        m_custom = filter;
        rows << filter;
    }
    rows += visible.mid(0, kCatalogMaxRows);
    m_visible = visible;
    beginResetModel();
    m_rows = rows;
    endResetModel();
}

void ParamCatalogModel::refreshValues()
{
    if (!m_rows.isEmpty()) emit dataChanged(index(0), index(int(m_rows.size()) - 1));
}

// ── Controller ───────────────────────────────────────────────────────────────

ParamsController::ParamsController(ParamStore *store, VehicleHub *hub, QObject *parent)
    : QObject(parent), m_store(store), m_hub(hub), m_table(store), m_catalog(store)
{
    s_instance = this;
    connect(m_store, &ParamStore::readChanged, this, &ParamsController::readChanged);
    connect(m_store, &ParamStore::queueChanged, this, [this] {
        m_catalog.refreshValues();
        emit queueChanged();
    });
    connect(m_store, &ParamStore::changed, this, [this] {
        m_table.refresh();
        // A newly learned name enters the catalog
        const int before = int(m_catalogNames.size());
        m_catalogNames = ParamCatalog::instance().catalog(vehicleClass(), m_store->names());
        if (m_catalogNames.size() != before) rebuildCatalog();
        else m_catalog.refreshValues();
        emit listChanged();
        ++m_metaRev;          // the detail strip shows the new value
        emit metaChanged();
    });
    // The link state for the buttons (READ ALL needs a vehicle)
    m_linkPoll.setInterval(500);
    connect(&m_linkPoll, &QTimer::timeout, this, [this] {
        const bool ready = linkReady();
        if (ready != m_linkReady) {
            m_linkReady = ready;
            emit linkChanged();
        }
        // The metadata follow the vehicle connected last (Plane before any)
        if (ready && m_hub->snapshot()->vehicleType > 0 && vehicleClass() != m_metaClass) {
            m_metaClass = vehicleClass();
            m_table.setVehicleClass(m_metaClass);
            ++m_metaRev;
            emit metaChanged();
        }
    });
    m_linkPoll.start();
    m_metaClass = QStringLiteral("Plane");
    m_table.setVehicleClass(m_metaClass);
    m_catalogNames = ParamCatalog::instance().catalog(vehicleClass(), m_store->names());
    rebuildCatalog();
}

QString ParamsController::vehicleClass() const { return m_hub ? modes::vehicleTypeName(m_hub->snapshot()->vehicleType) : QStringLiteral("Plane"); }

bool ParamsController::linkReady() const { return m_hub && m_hub->snapshot()->connected; }
bool ParamsController::reading() const { return m_store->reading(); }
QString ParamsController::readText() const { return m_store->readText(); }
double ParamsController::readFraction() const { return m_store->readFraction(); }
int ParamsController::loadedCount() const { return m_store->received(); }
QString ParamsController::queueText() const { return m_store->queueText(); }
bool ParamsController::queueBusy() const { return m_store->queueBusy(); }
bool ParamsController::queueError() const { return m_store->queueError(); }
int ParamsController::fetchTimeout() const { return m_store->fetchTimeout(); }

void ParamsController::setFetchTimeout(int ms)
{
    m_store->setFetchTimeout(ms);
    emit filtersChanged();
}

void ParamsController::setTableFilter(const QString &f)
{
    const QString v = f.trimmed().toUpper();
    if (v == m_tableFilter) return;
    m_tableFilter = v;
    m_table.setFilter(v);
    emit filtersChanged();
}

void ParamsController::setCatalogFilter(const QString &f)
{
    const QString v = ParamCatalog::sanitize(f);
    if (v == m_catalogFilter) return;
    m_catalogFilter = v;
    rebuildCatalog();
    emit filtersChanged();
}

void ParamsController::setCatalogGroup(const QString &g)
{
    if (g == m_catalogGroup) return;
    m_catalogGroup = g;
    rebuildCatalog();
    emit filtersChanged();
}

void ParamsController::rebuildCatalog()
{
    m_groups = ParamCatalog::groups(m_catalogNames);
    if (!m_catalogGroup.isEmpty() && !m_groups.contains(m_catalogGroup)) m_catalogGroup.clear();
    m_catalog.rebuild(m_catalogNames, m_catalogFilter, m_catalogGroup);
    emit catalogChanged();
}

QString ParamsController::catalogCount() const { return QStringLiteral("%1/%2").arg(m_catalog.visible().size()).arg(m_catalogNames.size()); }

void ParamsController::pageShown()
{
    m_catalogNames = ParamCatalog::instance().catalog(vehicleClass(), m_store->names());
    rebuildCatalog();
    m_table.refresh();
}

// ── Metadata ─────────────────────────────────────────────────────────────────

QString ParamsController::metaSource() const
{
    ParamMeta &meta = ParamMeta::instance();
    const int n = meta.count(m_metaClass);
    if (n == 0) return QStringLiteral("No parameter descriptions");
    return QStringLiteral("%1 descriptions · %2 · %3").arg(n).arg(ParamMeta::firmwareOf(m_metaClass), meta.source(m_metaClass));
}

QVariantMap ParamsController::meta(const QString &name) const
{
    QVariantMap out;
    out.insert(QStringLiteral("name"), name);
    const ParamInfo *m = ParamMeta::instance().find(name, m_metaClass);
    const ParamValue *p = m_store->find(name);
    const QHash<QString, double> edits = m_table.edits();
    const bool edited = edits.contains(name);
    const bool have = edited || p;
    const double cur = edited ? edits.value(name) : (p ? p->value : 0.0);
    out.insert(QStringLiteral("known"), m != nullptr);
    out.insert(QStringLiteral("loaded"), p != nullptr);
    out.insert(QStringLiteral("value"), p ? ParamCatalog::formatValue(p->value, p->type) : QString());
    out.insert(QStringLiteral("current"), have ? numberText(cur) : QString());
    out.insert(QStringLiteral("currentNumber"), cur);
    out.insert(QStringLiteral("edited"), edited);
    out.insert(QStringLiteral("kind"), kindOf(m));
    out.insert(QStringLiteral("meaning"), meaningOf(m, have, cur));
    if (!m) {
        out.insert(QStringLiteral("description"), ParamCatalog::description(name));
        return out;
    }
    out.insert(QStringLiteral("title"), m->displayName);
    out.insert(QStringLiteral("description"), m->description);
    out.insert(QStringLiteral("units"), m->units);
    out.insert(QStringLiteral("user"), m->user);
    out.insert(QStringLiteral("hasRange"), m->hasRange);
    out.insert(QStringLiteral("low"), m->low);
    out.insert(QStringLiteral("high"), m->high);
    out.insert(QStringLiteral("range"), rangeText(*m));
    out.insert(QStringLiteral("increment"), m->increment > 0 ? numberText(m->increment) : QString());
    out.insert(QStringLiteral("reboot"), m->rebootRequired);
    out.insert(QStringLiteral("readOnly"), m->readOnly);
    out.insert(QStringLiteral("outOfRange"), have && m->outOfRange(cur));
    QVariantList values, bits;
    for (const auto &[value, label] : m->values)
        values << QVariantMap{{QStringLiteral("value"), numberText(value)}, {QStringLiteral("label"), label}};
    for (const auto &[bit, label] : m->bits)
        bits << QVariantMap{{QStringLiteral("bit"), bit}, {QStringLiteral("label"), label}};
    out.insert(QStringLiteral("values"), values);
    out.insert(QStringLiteral("bits"), bits);
    return out;
}

QString ParamsController::bitsLabel(const QString &name, double value) const
{
    const ParamInfo *m = ParamMeta::instance().find(name, m_metaClass);
    return m ? m->bitsLabel(std::llround(value)) : QString();
}

void ParamsController::updateMeta()
{
    if (m_metaBusy) return;
    const QString fw = ParamMeta::firmwareOf(m_metaClass);
    const QUrl url(QStringLiteral("https://autotest.ardupilot.org/Parameters/%1/apm.pdef.json").arg(fw));
    if (!m_net) m_net = new QNetworkAccessManager(this);
    QNetworkRequest req(url);
    req.setTransferTimeout(60000);
    req.setAttribute(QNetworkRequest::RedirectPolicyAttribute, QNetworkRequest::NoLessSafeRedirectPolicy);
    m_metaBusy = true;
    emit metaChanged();
    DialogHub::toast(QStringLiteral("Downloading the %1 parameter descriptions…").arg(fw));
    LOG_I("params") << "metadata download" << url.toString();
    QNetworkReply *reply = m_net->get(req);
    const QString cls = m_metaClass;
    connect(reply, &QNetworkReply::finished, this, [this, reply, cls, fw] {
        reply->deleteLater();
        m_metaBusy = false;
        if (reply->error() != QNetworkReply::NoError) {
            LOG_W("params") << "metadata download failed:" << reply->errorString();
            DialogHub::toast(QStringLiteral("Download failed: ") + reply->errorString());
            emit metaChanged();
            return;
        }
        int n = 0;
        QString err;
        if (!ParamMeta::instance().installDownloaded(cls, reply->readAll(), &n, &err)) {
            DialogHub::toast(QStringLiteral("Parameter descriptions not usable: ") + err);
        } else {
            DialogHub::toast(QStringLiteral("%1 parameter descriptions updated (%2)").arg(fw).arg(n));
            m_table.refresh();
            m_table.touchAll();
        }
        ++m_metaRev;
        emit metaChanged();
    });
}

void ParamsController::resetMeta()
{
    ParamMeta::instance().removeDownloaded(m_metaClass);
    m_table.touchAll();
    ++m_metaRev;
    emit metaChanged();
    DialogHub::toast(QStringLiteral("Using the parameter descriptions bundled with the GCS"));
}

void ParamsController::readAll()
{
    if (!linkReady()) {
        DialogHub::toast(QStringLiteral("Not connected"));
        return;
    }
    m_store->readAll();
}

void ParamsController::cancelRead() { m_store->cancelRead(); }

void ParamsController::writeChanged()
{
    const QHash<QString, double> edits = m_table.edits();
    if (edits.isEmpty()) {
        DialogHub::toast(QStringLiteral("No parameters changed. Edit values first."));
        return;
    }
    QList<QPair<QString, double>> list;
    for (auto it = edits.cbegin(); it != edits.cend(); ++it) list.append({it.key(), it.value()});
    std::sort(list.begin(), list.end());
    m_store->writeMany(list, [this](int written, int total) {
        DialogHub::toast(QStringLiteral("Written %1/%2 parameters").arg(written).arg(total));
        m_table.refresh();
        emit listChanged();
    });
}

bool ParamsController::setEdit(const QString &name, const QString &text)
{
    const bool changed = m_table.setEdit(name, text);
    emit listChanged();
    ++m_metaRev;          // the detail strip shows the typed value
    emit metaChanged();
    return changed;
}

void ParamsController::saveFile(const QUrl &file)
{
    if (m_store->received() == 0) {
        DialogHub::toast(QStringLiteral("No parameters loaded. Read parameters first."));
        return;
    }
    QString err;
    if (m_store->saveFile(file.toLocalFile(), &err)) DialogHub::toast(QStringLiteral("Saved %1 parameters").arg(m_store->received()));
    else DialogHub::toast(QStringLiteral("Error: ") + err);
}

void ParamsController::loadFile(const QUrl &file)
{
    const QString path = file.toLocalFile();
    const QString name = file.fileName();
    m_store->loadFile(path, [name](int written, int, const QString &error) {
        if (!error.isEmpty()) DialogHub::toast(QStringLiteral("Error: ") + error);
        else DialogHub::toast(QStringLiteral("Loaded %1 parameters from %2").arg(written).arg(name));
    });
}

void ParamsController::fetch(const QString &name) { m_store->enqueueFetch({name}); }

void ParamsController::fetchTyped()
{
    const QString name = !m_catalogFilter.isEmpty() ? m_catalogFilter : m_catalog.visible().value(0);
    if (!name.isEmpty()) m_store->enqueueFetch({name});
}

void ParamsController::fetchListed()
{
    const QStringList names = m_catalog.visible();
    if (names.isEmpty()) return;
    if (names.size() > kBulkConfirm) {
        DialogHub::confirm(QStringLiteral("Read %1 parameters one by one? On a slow link this takes about %2 s.")
                               .arg(names.size())
                               .arg(int(std::ceil(names.size() * 0.6))),
                           [this, names](bool ok) {
                               if (ok) m_store->enqueueFetch(names);
                           });
        return;
    }
    m_store->enqueueFetch(names);
}

void ParamsController::fetchFavorites()
{
    const QStringList favs = ParamCatalog::instance().favorites();
    if (favs.isEmpty()) {
        DialogHub::toast(QStringLiteral("No starred parameters yet — click the ☆ next to a name to star it."));
        return;
    }
    m_store->enqueueFetch(favs);
}

void ParamsController::toggleFavorite(const QString &name)
{
    ParamCatalog::instance().toggleFavorite(name);
    m_catalog.refreshValues();
}

} // namespace corv
