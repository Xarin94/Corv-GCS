#include "ui/PlanController.h"

#include "core/DataRoot.h"
#include "core/DebugLog.h"
#include "core/Settings.h"
#include "gpu/SceneItem.h"
#include "link/LinkManager.h"
#include "ui/GcsController.h"
#include "ui/MapItem.h"
#include "mission/MissionStore.h"
#include "mission/MissionTransfer.h"
#include "mission/Platforms.h"
#include "mission/RouteGeo.h"
#include "terrain/Hgt.h"
#include "terrain/HgtDownloader.h"
#include "vehicle/CommandSender.h"
#include "vehicle/VehicleHub.h"

#include <QDateTime>
#include <QDesktopServices>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QLocale>
#include <QPointer>
#include <QRegularExpression>
#include <QSet>
#include <QUrl>
#include <QtConcurrent>
#include <cmath>

using namespace Qt::StringLiterals;

namespace corv {

using namespace mission;

PlanController *PlanController::s_instance = nullptr;

namespace {

const char *kLsLink = "corv.fp.link";
const char *kLsRadio = "corv.fp.radio";
constexpr int kMaxHistory = 100;
constexpr int kMaxIssueRows = 8;

QString fmtNumV(const QJsonValue &v) { return fmtNum(num(v)); }
QString fmtLL(const LL &p, int prec) { return u"%1, %2"_s.arg(p.lat, 0, 'f', prec).arg(p.lng, 0, 'f', prec); }

QString fmtArea(double m2)
{
    return m2 >= 1e6 ? u"%1 km²"_s.arg(m2 / 1e6, 0, 'f', 2) : u"%1 ha"_s.arg(m2 / 1e4, 0, 'f', 1);
}

/// A {lat, lng} object of the document, nothing when it is not set.
std::optional<LL> llOf(const QJsonValue &v)
{
    const QJsonObject o = v.toObject();
    if (o.isEmpty()) return std::nullopt;
    return LL{num(o.value(u"lat"_s)), num(o.value(u"lng"_s))};
}

QJsonObject llJson(const LL &p) { return {{u"lat"_s, p.lat}, {u"lng"_s, p.lng}}; }

/// Range of the coverage raster around the operator, m.
double coverageRange(const QJsonObject &radio)
{
    const double km = num(radio.value(u"coverageRange"_s));
    return (km ? km : 15) * 1000;
}

QString linkName(Link c)
{
    switch (c) {
    case Link::Good: return u"good"_s;
    case Link::Degraded: return u"degraded"_s;
    case Link::Marginal: return u"marginal"_s;
    default: return u"none"_s;
    }
}

QJsonArray fieldsOf(const QString &scope, const QJsonObject &seg, const QJsonObject &action)
{
    if (scope == "seg"_L1) return segmentDef(seg.value(u"type"_s).toString()).value(u"fields"_s).toArray();
    if (scope == "action"_L1) return actionDef(action.value(u"type"_s).toString()).value(u"fields"_s).toArray();
    if (scope == "route"_L1) return routeParamFields();
    if (scope == "camera"_L1) return cameraFields();
    if (scope == "gimbal"_L1) return gimbalFields();
    if (scope == "radio"_L1) return radioFields();
    return {};
}

bool hasWhenOn(const QJsonArray &fields, const QString &key)
{
    for (const QJsonValue &f : fields)
        if (f.toObject().value(u"when"_s).toObject().contains(key)) return true;
    return false;
}

/// The edited text / check / option as the document stores it; nothing when the text is not a number.
std::optional<QJsonValue> parseField(const QJsonObject &f, const QVariant &value)
{
    const QString type = f.value(u"type"_s).toString();
    if (type == "check"_L1) return QJsonValue(value.toBool());
    if (type == "select"_L1) return QJsonValue(value.toString());
    const QString text = value.toString().trimmed();
    if (text.isEmpty()) return f.value(u"nullable"_s).toBool() ? QJsonValue(QJsonValue::Null) : QJsonValue(0);
    bool ok = false;
    double d = QLocale::c().toDouble(text, &ok);
    if (!ok) d = text.toDouble(&ok);
    if (!ok || !std::isfinite(d)) return std::nullopt;
    if (f.contains(u"min"_s)) d = std::max(d, f.value(u"min"_s).toDouble());
    if (f.contains(u"max"_s)) d = std::min(d, f.value(u"max"_s).toDouble());
    return QJsonValue(d);
}

QString connectionLabel(const QString &type)
{
    static const QHash<QString, QString> labels{
        {u"mavlink-serial"_s, u"MAVLink Serial"_s}, {u"mavlink-udp"_s, u"MAVLink UDP"_s}, {u"mavlink-tcp"_s, u"MAVLink TCP"_s},
        {u"msp-serial"_s, u"MSP Serial (INAV / Betaflight)"_s}, {u"msp-tcp"_s, u"MSP TCP (INAV SITL)"_s}};
    return labels.value(type, type);
}

/// A mission file's route; files written before the route editor only carry compiled items.
QJsonObject routeOfFile(const QJsonObject &data)
{
    const QJsonObject r = data.value(u"route"_s).toObject();
    if (r.value(u"segments"_s).isArray()) return r;
    return itemsToRoute(itemsFromJson(data.value(u"items"_s).toArray()), data.value(u"name"_s).toString(), false);
}

RawItem rawOf(const CompiledItem &it)
{
    return {it.seq, it.command, it.frame, it.lat, it.lng, it.alt, it.param1, it.param2, it.param3, it.param4, it.isHome};
}

RawItem rawOf(int seq, const MissionItem &m)
{
    return {seq, m.command, m.frame, m.lat, m.lng, m.alt, m.param1, m.param2, m.param3, m.param4};
}

MissionItem missionItemOf(const RawItem &r)
{
    return {r.command, r.frame, float(r.param1), float(r.param2), float(r.param3), float(r.param4), r.lat, r.lng, float(r.alt)};
}

InavWp inavWpOf(const QVariantMap &m)
{
    auto i = [&m](const char *k) { return m.value(QLatin1String(k)).toInt(); };
    return {i("index"), i("action"), i("lat"), i("lon"), i("alt"), i("p1"), i("p2"), i("p3"), i("flag")};
}

} // namespace

// ── SegmentListModel ─────────────────────────────────────────────────────────

QVariant SegmentListModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid() || index.row() >= m_rows.size()) return {};
    const Row &r = m_rows[index.row()];
    switch (role) {
    case SegIdRole: return r.id;
    case IndexRole: return index.row();
    case TypeRole: return r.type;
    case LabelRole: return r.label;
    case ColorRole: return r.color;
    case SummaryRole: return r.summary;
    case WorstRole: return r.worst;
    case IssuesTextRole: return r.issuesText;
    case OpenRole: return r.open;
    }
    return {};
}

QHash<int, QByteArray> SegmentListModel::roleNames() const
{
    return {{SegIdRole, "segId"}, {IndexRole, "segIndex"}, {TypeRole, "segType"}, {LabelRole, "segLabel"}, {ColorRole, "segColor"},
            {SummaryRole, "summary"}, {WorstRole, "worst"}, {IssuesTextRole, "issuesText"}, {OpenRole, "open"}};
}

void SegmentListModel::setRows(const QList<Row> &rows)
{
    bool sameIds = rows.size() == m_rows.size();
    for (qsizetype i = 0; sameIds && i < rows.size(); ++i) sameIds = rows[i].id == m_rows[i].id;
    if (!sameIds) {
        beginResetModel();
        m_rows = rows;
        endResetModel();
        return;
    }
    // Same cards: update in place, the delegates (and a field being edited) stay
    m_rows = rows;
    if (!rows.isEmpty()) emit dataChanged(index(0), index(int(rows.size()) - 1));
}

// ── Construction ─────────────────────────────────────────────────────────────

PlanController::PlanController(VehicleHub *hub, CommandSender *cmd, QObject *parent)
    : QObject(parent), m_hub(hub), m_cmd(cmd)
{
    s_instance = this;
    // The ground radio outlives any one route: the last one used seeds new routes
    const QJsonObject savedRadio = Settings::instance().json(QLatin1String(kLsRadio)).toObject();
    if (!savedRadio.isEmpty()) rememberRadio(savedRadio);
    m_route = createRoute();
    m_baseline = m_route;
    m_showLink = Settings::instance().value(QLatin1String(kLsLink)).toString() == "1"_L1;

    for (QTimer *t : {&m_compileTimer, &m_terrainRetry, &m_linkTimer, &m_hintTimer}) t->setSingleShot(true);
    connect(&m_compileTimer, &QTimer::timeout, this, &PlanController::runCompile);
    connect(&m_terrainRetry, &QTimer::timeout, this, &PlanController::runCompile);
    connect(&m_linkTimer, &QTimer::timeout, this, &PlanController::runLink);
    connect(&m_hintTimer, &QTimer::timeout, this, [this] { if (m_tool == "select"_L1) setHint({}); });

    // Elevation arriving (read from disk or downloaded) changes the calculated route
    connect(&HgtStore::instance(), &HgtStore::tileLoaded, this, [this] { scheduleCompile(100); }, Qt::QueuedConnection);
    connect(&HgtDownloader::instance(), &HgtDownloader::tileDone, this, [this] { scheduleCompile(100); });

    m_pollTimer.setInterval(500);
    connect(&m_pollTimer, &QTimer::timeout, this, &PlanController::poll);
    m_pollTimer.start();
    scheduleCompile(0);
}

void PlanController::poll()
{
    // Twice a second: the vehicle marker, a home arriving, an empty plan following the vehicle
    QString kind;
    const auto t = takeoffPoint(&kind);
    if (t && kind != "custom"_L1
        && (kind != m_compiledHomeKind || !m_compiled.home || std::abs(m_compiled.home->lat - t->lat) > 1e-7
            || std::abs(m_compiled.home->lng - t->lng) > 1e-7)
        && !m_compileTimer.isActive() && !m_terrainRetry.isActive())
        scheduleCompile();
    if (!m_visible) return;
    // An empty plan goes where the vehicle is
    const bool live = t && (kind == "home"_L1 || kind == "vehicle"_L1);
    if (live && segs().isEmpty() && !llOf(params().value(u"home"_s)) && !(m_centeredAt && haversine(*m_centeredAt, *t) < 1000)) {
        emit centerRequested(t->lat, t->lng, 15);
        m_centeredAt = t;
    }
    emit mapChanged();
}

// ── Document helpers ─────────────────────────────────────────────────────────

QJsonArray PlanController::segs() const { return m_route.value(u"segments"_s).toArray(); }
void PlanController::setSegs(const QJsonArray &a) { m_route.insert(u"segments"_s, a); }
QJsonObject PlanController::params() const { return m_route.value(u"params"_s).toObject(); }
void PlanController::setParams(const QJsonObject &p) { m_route.insert(u"params"_s, p); }
int PlanController::segmentCount() const { return int(segs().size()); }
std::shared_ptr<const VehicleState> PlanController::snapshot() const { return m_hub ? m_hub->snapshot() : nullptr; }
int PlanController::vehicleType() const { const auto s = snapshot(); return s ? s->vehicleType : 0; }
bool PlanController::vehicleConnected() const { const auto s = snapshot(); return s && s->connected; }

int PlanController::segIndex(const QString &id) const
{
    const QJsonArray a = segs();
    for (qsizetype i = 0; i < a.size(); ++i)
        if (a[i].toObject().value(u"id"_s).toString() == id) return int(i);
    return -1;
}

QJsonObject PlanController::segment(const QString &id) const
{
    const int i = segIndex(id);
    return i < 0 ? QJsonObject() : segs()[i].toObject();
}

void PlanController::putSegment(const QJsonObject &seg)
{
    QJsonArray a = segs();
    const int i = segIndex(seg.value(u"id"_s).toString());
    if (i < 0) return;
    a[i] = seg;
    setSegs(a);
}

void PlanController::replaceRoute(const QJsonObject &r)
{
    m_route = normalizeRoute(r);
    emit routeChanged();
    emit formsChanged();
}

void PlanController::loadRoute(const QJsonObject &route, const QString &id, const QString &name, int fitMs)
{
    replaceRoute(route);
    m_currentId = id;
    m_currentName = name;
    m_selectedId.clear();
    resetHistory();
    applyHistory();
    emit libraryChanged();
    if (fitMs >= 0) QTimer::singleShot(fitMs, this, [this] { emit fitRequested(); });
}

QString PlanController::routeName() const
{
    const QString n = m_route.value(u"name"_s).toString();
    return n.isEmpty() ? m_currentName : n;
}

void PlanController::setRouteName(const QString &name)
{
    const QString t = name.trimmed();
    m_route.insert(u"name"_s, t.isEmpty() ? QJsonValue(QJsonValue::Null) : QJsonValue(t));
    commit(u"Rename route"_s);
}

// ── History (MissionHistory.js) ──────────────────────────────────────────────

bool PlanController::commit(const QString &label)
{
    if (m_route == m_baseline) return false;
    m_undo.append({m_baseline, label});
    if (m_undo.size() > kMaxHistory) m_undo.removeFirst();
    m_baseline = m_route;
    m_redo.clear();
    emit historyChanged();
    emit routeChanged();
    return true;
}

void PlanController::resetHistory()
{
    m_baseline = m_route;
    m_undo.clear();
    m_redo.clear();
    emit historyChanged();
}

QString PlanController::undoTip() const { return m_undo.isEmpty() ? u"Nothing to undo"_s : u"Undo: %1 (Ctrl+Z)"_s.arg(m_undo.last().label); }
QString PlanController::redoTip() const { return m_redo.isEmpty() ? u"Nothing to redo"_s : u"Redo: %1 (Ctrl+Y)"_s.arg(m_redo.last().label); }
void PlanController::undo() { stepHistory(m_undo, m_redo); }
void PlanController::redo() { stepHistory(m_redo, m_undo); }

void PlanController::stepHistory(QList<Step> &from, QList<Step> &to)
{
    if (from.isEmpty()) return;
    const Step e = from.takeLast();
    to.append({m_baseline, e.label});
    m_baseline = m_route = e.doc;
    emit historyChanged();
    applyHistory();
}

void PlanController::applyHistory()
{
    if (!m_selectedId.isEmpty() && segIndex(m_selectedId) < 0) m_selectedId.clear();
    emit selectionChanged();
    emit routeChanged();
    emit formsChanged();
    emit derivedChanged();
    refreshList();
    rebuildInspector();
    emit mapChanged();
    scheduleCompile(0);
}

// ── Where the plan stands ────────────────────────────────────────────────────

std::optional<LL> PlanController::vehicleFix() const
{
    const auto s = snapshot();
    if (!s || s->isDemo() || !s->connected || s->gpsFix < 2 || (!s->lat && !s->lon)) return std::nullopt;
    return LL{s->lat, s->lon};
}

std::optional<LL> PlanController::takeoffPoint(QString *kind) const
{
    auto found = [kind](const QString &k, std::optional<LL> p) {
        if (kind) *kind = k;
        return p;
    };
    if (const auto custom = llOf(params().value(u"home"_s))) return found(u"custom"_s, custom);
    // Where the vehicle is based: its home, else the vehicle on the link, else the last home seen
    if (const auto s = snapshot(); s && s->homeLat && s->homeLon) return found(u"home"_s, LL{*s->homeLat, *s->homeLon});
    if (const auto fix = vehicleFix()) {
        // A vehicle without a home is taken once: a moving vehicle does not recompile every tick
        if (m_compiled.home && (m_compiledHomeKind == "vehicle"_L1 || m_compiledHomeKind == "home"_L1))
            return found(m_compiledHomeKind, LL{m_compiled.home->lat, m_compiled.home->lng});
        return found(u"vehicle"_s, fix);
    }
    if (const auto last = llOf(Settings::instance().json(u"corv.lastHome"_s))) return found(u"last"_s, last);
    return found({}, std::nullopt);
}

std::optional<LL> PlanController::placedOperator() const { return llOf(params().value(u"operator"_s)); }

std::optional<LL> PlanController::operatorPoint() const
{
    if (const auto op = placedOperator()) return op;
    if (m_compiled.home) return LL{m_compiled.home->lat, m_compiled.home->lng};
    return std::nullopt;
}

TerrainFn PlanController::terrainFn() const
{
    return [](double lat, double lng) { return HgtStore::instance().elevation(lat, lng); };
}

// ── Compile ──────────────────────────────────────────────────────────────────

void PlanController::scheduleCompile(int delayMs)
{
    if (m_status != "busy"_L1) {
        m_status = u"busy"_s;
        emit compiledChanged();
    }
    m_compileTimer.start(delayMs);
}

bool PlanController::ensureRouteTerrain()
{
    // The 1° elevation tiles under every base point and leg must be parsed (or known missing)
    QSet<QPair<int, int>> tiles;
    auto add = [&](double lat, double lng) {
        if (std::isfinite(lat) && std::isfinite(lng)) tiles.insert({int(std::floor(lat)), int(std::floor(lng))});
    };
    QList<LL> pts;
    if (const auto t = takeoffPoint()) pts.append(*t);
    for (const QJsonValue &v : segs()) pts.append(pointsOf(v.toObject()));
    std::optional<LL> op = placedOperator();
    if (!op && !pts.isEmpty()) op = pts.first();
    if (op) {
        add(op->lat, op->lng);
        if (m_showLink) {
            // The coverage raster reaches `coverageRange` around the operator
            const double dLat = coverageRange(params().value(u"radio"_s).toObject()) / 111320;
            const double dLng = dLat / std::max(0.2, std::cos(op->lat * M_PI / 180));
            for (int a : {-1, 1}) for (int b : {-1, 1}) add(op->lat + a * dLat, op->lng + b * dLng);
        }
    }
    for (qsizetype i = 0; i < pts.size(); ++i) {
        add(pts[i].lat, pts[i].lng);
        if (i == 0) continue;
        const LL &a = pts[i - 1], &b = pts[i];
        const int steps = std::max(2, int(std::ceil(std::max(std::abs(b.lat - a.lat), std::abs(b.lng - a.lng)) * 2)));
        for (int s = 1; s < steps; ++s) add(a.lat + (b.lat - a.lat) * s / steps, a.lng + (b.lng - a.lng) * s / steps);
    }
    bool pending = false;
    HgtStore &store = HgtStore::instance();
    for (const auto &[lat, lng] : tiles) {
        if (store.get(lat, lng)) continue;
        if (!store.missing(lat, lng)) { pending = true; continue; }   // being read from disk
        // Not on disk: fetch it (once per session unless it failed)
        HgtDownloader::instance().ensure(lat, lng);
        pending |= HgtDownloader::instance().busy(lat, lng);
    }
    return !pending;
}

void PlanController::runCompile()
{
    if (!ensureRouteTerrain()) {
        m_terrainRetry.start(400);
        return;
    }
    QString kind;
    const auto takeoff = takeoffPoint(&kind);
    const auto s = snapshot();
    CompileContext ctx;
    ctx.terrain = terrainFn();
    ctx.home = takeoff;
    ctx.homeAlt = s && s->homeAlt ? *s->homeAlt : 0;
    ctx.vehicleType = s ? s->vehicleType : 0;
    ctx.platform = activePlatformId();
    ctx.itemLimit = m_inavLimit;
    m_compiled = compileRoute(m_route, ctx);
    m_compiledHomeKind = takeoff ? kind : QString();
    ++m_compileRev;
    publishMission3D();

    refreshCard();
    refreshList();
    refreshInspectorDetail();
    emit derivedChanged();
    emit mapChanged();
    scheduleLink(0);
}

// ── Route card ───────────────────────────────────────────────────────────────

QString PlanController::statusTitle() const
{
    static const QHash<QString, QString> t{{u"ok"_s, u"Route calculated"_s},
                                           {u"warn"_s, u"Route calculated with warnings"_s},
                                           {u"error"_s, u"Route has errors"_s},
                                           {u"busy"_s, u"Calculating…"_s},
                                           {u"empty"_s, u"Empty route"_s}};
    return t.value(m_status);
}

bool PlanController::uploadBlocked() const
{
    return std::any_of(m_compiled.issues.begin(), m_compiled.issues.end(), [](const Issue &i) { return i.level == "error"_L1; });
}

QList<Issue> PlanController::platformIssues() const
{
    const auto s = snapshot();
    if (!s || !s->connected) return {};
    const Platform &plat = platform(activePlatformId());
    const bool msp = s->connectionType.contains(u"MSP"_s, Qt::CaseInsensitive);
    if (plat.transport == "msp"_L1 && !msp)
        return {{u"warn"_s, u"%1 missions travel over MSP — the connected link speaks MAVLink"_s.arg(plat.label), {}}};
    if (plat.transport == "mavlink"_L1 && msp)
        return {{u"warn"_s, u"The connected link speaks MSP — an ArduPilot mission cannot be uploaded over it"_s, {}}};
    return {};
}

void PlanController::refreshCard()
{
    const RouteStats &st = m_compiled.stats;
    const bool has = !segs().isEmpty();
    int errors = 0, warns = 0;
    for (const Issue &i : m_compiled.issues) {
        errors += i.level == "error"_L1;
        warns += i.level == "warn"_L1;
    }
    m_status = !has ? u"empty"_s : errors ? u"error"_s : warns ? u"warn"_s : u"ok"_s;
    const QString none = u"--"_s;
    m_stats = {has ? fmtDist(st.lengthM) : none, has ? fmtTime(st.durationS) : none, has ? QString::number(st.waypoints) : none,
               has && st.maxAgl ? u"%1 m"_s.arg(qRound64(*st.maxAgl)) : none};
    m_chips = routeChips();
    m_issues = cardIssues(has);
    emit compiledChanged();
}

QVariantList PlanController::routeChips() const
{
    const RouteStats &st = m_compiled.stats;
    const QJsonObject P = params();
    const Platform &plat = platform(activePlatformId());
    const QJsonObject cam = P.value(u"camera"_s).toObject();
    QVariantList chips;
    auto chip = [&](const QString &t, const QString &cls = {}, const QString &title = {}) {
        chips.append(QVariantMap{{u"text"_s, t}, {u"cls"_s, cls}, {u"title"_s, title}});
    };
    if (plat.id != "ardupilot"_L1) chip(plat.label);
    if (m_compiled.inav) chip(u"%1 / %2 WP"_s.arg(m_compiled.inav->wps.size()).arg(m_inavLimit ? m_inavLimit : plat.itemLimit));
    const QString altMode = P.value(u"altMode"_s).toString();
    const QString am = altMode == "amsl"_L1 ? u"AMSL"_s : altMode == "rel"_L1 ? u"REL"_s : u"AGL"_s;
    chip(u"%1 %2 m"_s.arg(am, fmtNumV(P.value(u"defaultAlt"_s))));
    if (altMode == "agl"_L1 && truthy(P.value(u"terrainWaypoints"_s)))
        chip(u"terrain WPs ±%1 m"_s.arg(fmtNumV(P.value(u"aglTolerance"_s))));
    const double spd = num(P.value(u"defaultSpeed"_s));
    chip(spd > 0 ? u"%1 m/s"_s.arg(fmtNum(spd)) : u"vehicle speed"_s);
    chip(truthy(P.value(u"takeoff"_s)) ? u"T/O %1 m"_s.arg(qRound64(num(P.value(u"takeoffAlt"_s)))) : u"no take-off"_s);
    const QString end = P.value(u"endAction"_s).toString();
    chip(end == "land"_L1 ? u"LAND"_s : end == "none"_L1 ? u"end: nothing"_s : u"RTL"_s);
    if (P.value(u"turnType"_s).toString() == "spline"_L1) chip(u"spline"_s);
    if (!cam.value(u"name"_s).toString().isEmpty() && cam.value(u"preset"_s).toString() != "generic"_L1) chip(cam.value(u"name"_s).toString());
    if (st.photos) chip(u"%1 photos"_s.arg(st.photos));
    if (m_showLink && m_analysis) {
        const Link cls = m_analysis->worst ? m_analysis->worst->clazz : Link::Good;
        const QString text = cls == Link::Good ? u"LINK OK"_s : cls == Link::Unknown ? u"LINK ?"_s : u"LINK "_s + linkStyle(cls).shortName;
        chip(text, u"link-"_s + linkName(cls), summarizeLink(*m_analysis));
    }
    return chips;
}

QVariantList PlanController::cardIssues(bool hasSegments) const
{
    // One line per distinct problem; the same warning on forty waypoints reads "40 seg …"
    struct Group { QString level, text; QStringList segIds; };
    QList<Group> groups;
    auto add = [&](const Issue &i) {
        if (i.level == "info"_L1 && hasSegments) return;
        for (Group &g : groups)
            if (g.level == i.level && g.text == i.text) {
                if (!i.segId.isEmpty()) g.segIds << i.segId;
                return;
            }
        groups.append({i.level, i.text, i.segId.isEmpty() ? QStringList() : QStringList{i.segId}});
    };
    for (const Issue &i : m_compiled.issues) add(i);
    for (const Issue &i : platformIssues()) add(i);
    if (m_showLink && m_analysis) {
        auto along = [&](Link c, const QString &text) {
            if (const double len = m_analysis->lengths[int(c)]; len > 0) add({u"warn"_s, text.arg(fmtDist(len)), {}});
        };
        along(Link::None, u"Radio link lost along %1 of the route"_s);
        along(Link::Marginal, u"Radio link below the safety margin along %1"_s);
        along(Link::Unknown, u"No elevation data for the radio link along %1"_s);
    }
    auto row = [](const QString &level, const QString &text, const QString &where, const QString &segId) {
        return QVariantMap{{u"level"_s, level}, {u"text"_s, text}, {u"where"_s, where}, {u"segId"_s, segId}};
    };
    QVariantList out;
    for (qsizetype gi = 0; gi < groups.size() && gi < kMaxIssueRows; ++gi) {
        const Group &g = groups[gi];
        const QString first = g.segIds.value(0);
        const int idx = first.isEmpty() ? -1 : segIndex(first);
        const QString where = g.segIds.size() > 1 ? u"%1 seg"_s.arg(g.segIds.size()) : idx >= 0 ? QString::number(idx + 1) : QString();
        out.append(row(g.level, g.text, where, idx >= 0 ? first : QString()));
    }
    if (groups.size() > kMaxIssueRows) out.append(row(u"info"_s, u"… %1 more"_s.arg(groups.size() - kMaxIssueRows), {}, {}));
    return out;
}

void PlanController::issueClicked(const QString &segId)
{
    if (segId.isEmpty() || segIndex(segId) < 0) return;
    selectSegment(segId, true);
    emit scrollToSegment(segIndex(segId));
}

// ── Segment list & inspector ─────────────────────────────────────────────────

QString PlanController::segmentSummary(const QJsonObject &seg) const
{
    const QJsonObject sp = seg.value(u"params"_s).toObject();
    const QString type = seg.value(u"type"_s).toString();
    const auto stIt = m_compiled.segStats.constFind(seg.value(u"id"_s).toString());
    const SegStats *st = stIt == m_compiled.segStats.cend() ? nullptr : &*stIt;
    auto photos = [&](QStringList &parts) { if (st && st->photos) parts << u"%1 img"_s.arg(st->photos); };
    QStringList parts;
    if (type != "poi"_L1 && type != "landing"_L1) parts << u"%1 m"_s.arg(fmtNum(segmentAlt(seg, params())));
    if (num(sp.value(u"speed"_s)) > 0) parts << u"%1 m/s"_s.arg(fmtNumV(sp.value(u"speed"_s)));
    if (type == "circle"_L1) {
        parts << u"r %1 m"_s.arg(fmtNumV(sp.value(u"radius"_s))) << u"%1×"_s.arg(fmtNumV(sp.value(u"turns"_s)));
    } else if (type == "perimeter"_L1) {
        parts << u"%1 vtx"_s.arg(pointsOf(seg).size());
        if (st) parts << fmtDist(st->lengthM);
    } else if (type == "area"_L1) {
        if (st) {
            parts << fmtArea(st->area) << u"%1 lanes"_s.arg(st->lanes);
            if (st->gsd) parts << u"%1 cm/px"_s.arg(*st->gsd, 0, 'f', 1);
            photos(parts);
        }
    } else if (type == "corridor"_L1) {
        parts << u"%1 m wide"_s.arg(fmtNumV(sp.value(u"width"_s)));
        if (st) {
            parts << u"%1 pass%2"_s.arg(st->lanes).arg(st->lanes > 1 ? u"es"_s : QString());
            photos(parts);
        }
    } else if (type == "poi"_L1) {
        parts << (sp.value(u"mode"_s).toString() == "clear"_L1 ? u"ROI off"_s : u"look at, %1 m"_s.arg(fmtNumV(sp.value(u"alt"_s))));
    } else if (type == "landing"_L1) {
        parts << (truthy(sp.value(u"vtol"_s)) ? u"VTOL"_s : u"land"_s);
    }
    const qsizetype na = seg.value(u"actions"_s).toArray().size();
    if (na) parts << u"%1 action%2"_s.arg(na).arg(na > 1 ? u"s"_s : QString());
    return parts.join(u" · "_s);
}

void PlanController::refreshList()
{
    QHash<QString, QList<const Issue *>> bySeg;
    for (const Issue &i : m_compiled.issues) if (!i.segId.isEmpty()) bySeg[i.segId].append(&i);
    QList<SegmentListModel::Row> rows;
    for (const QJsonValue &v : segs()) {
        const QJsonObject seg = v.toObject();
        const QString id = seg.value(u"id"_s).toString(), type = seg.value(u"type"_s).toString();
        const QJsonObject def = segmentDef(type);
        QStringList texts;
        bool err = false;
        for (const Issue *i : bySeg.value(id)) {
            texts << i->text;
            err |= i->level == "error"_L1;
        }
        rows.append({id, type, def.value(u"label"_s).toString(), def.value(u"color"_s).toString(), segmentSummary(seg),
                     err ? u"error"_s : texts.isEmpty() ? QString() : u"warn"_s, texts.join('\n'_L1), id == m_selectedId});
    }
    m_segModel.setRows(rows);
}

QVariantList PlanController::fieldsFor(const QJsonArray &fields, const QJsonObject &values, const WhyFn &why) const
{
    QVariantList out;
    for (const QJsonValue &fv : fields) {
        const QJsonObject f = fv.toObject();
        if (!fieldVisible(f, values)) continue;
        const QString key = f.value(u"key"_s).toString();
        const QString type = f.value(u"type"_s).toString();
        const QJsonValue v = values.value(key);
        const QString reason = why ? why(f, QString()) : QString();
        QVariantMap m{{u"key"_s, key}, {u"label"_s, f.value(u"label"_s).toString()}, {u"unit"_s, f.value(u"unit"_s).toString()},
                      {u"type"_s, type}, {u"disabled"_s, !reason.isEmpty()},
                      {u"title"_s, reason.isEmpty() ? f.value(u"title"_s).toString() : reason}};
        if (type == "select"_L1) {
            QVariantList opts;
            int current = 0;
            const QJsonArray o = f.value(u"options"_s).toArray();
            const QString cur = v.isString() ? v.toString() : v.isDouble() ? fmtNum(v.toDouble()) : v.isBool() ? (v.toBool() ? u"true"_s : u"false"_s) : QString();
            for (qsizetype i = 0; i < o.size(); ++i) {
                const QJsonArray pair = o[i].toArray();
                const QString val = pair.at(0).toString();
                const QString optWhy = reason.isEmpty() && why ? why(f, val) : QString();
                if (val == cur) current = int(i);
                opts.append(QVariantMap{{u"value"_s, val}, {u"label"_s, pair.at(1).toString()}, {u"disabled"_s, !optWhy.isEmpty()}, {u"title"_s, optWhy}});
            }
            m.insert(u"options"_s, opts);
            m.insert(u"current"_s, current);
        } else if (type == "check"_L1) {
            m.insert(u"checked"_s, truthy(v));
        } else {
            const bool empty = v.isNull() || v.isUndefined() || (v.isString() && v.toString().isEmpty()) || (f.contains(u"zeroLabel"_s) && num(v) == 0);
            m.insert(u"value"_s, empty ? QString() : (v.isString() ? v.toString() : fmtNum(num(v))));
            const QString ph = f.value(u"placeholder"_s).toString();
            m.insert(u"placeholder"_s, ph.isEmpty() ? f.value(u"zeroLabel"_s).toString() : ph);
        }
        out.append(m);
    }
    return out;
}

void PlanController::rebuildInspector()
{
    m_inspFields.clear();
    m_inspActions.clear();
    const QJsonObject seg = segment(m_selectedId);
    if (!seg.isEmpty()) {
        const QString type = seg.value(u"type"_s).toString();
        QJsonArray fields;
        for (const QJsonValue &fv : segmentDef(type).value(u"fields"_s).toArray()) {
            QJsonObject f = fv.toObject();
            // Nullable altitude: the route default as placeholder
            if (f.value(u"key"_s).toString() == "alt"_L1 && f.value(u"nullable"_s).toBool()) {
                f.insert(u"placeholder"_s, fmtNumV(params().value(u"defaultAlt"_s)));
                f.insert(u"title"_s, u"Empty = route default altitude"_s);
            }
            fields.append(f);
        }
        m_inspFields = fieldsFor(fields, seg.value(u"params"_s).toObject(), [type](const QJsonObject &f, const QString &opt) {
            const QString key = type + '.'_L1 + f.value(u"key"_s).toString();
            return unsupportedReason(u"segField"_s, opt.isEmpty() ? key : key + '.'_L1 + opt);
        });
        // An action the flight stack cannot fly is kept, greyed out with the reason
        const QJsonArray actions = seg.value(u"actions"_s).toArray();
        const QString platLabel = platform(activePlatformId()).label;
        for (qsizetype i = 0; i < actions.size(); ++i) {
            const QJsonObject a = actions[i].toObject();
            QString at = a.value(u"type"_s).toString();
            if (!actionTypes().contains(at)) at = u"raw"_s;
            const QJsonObject ad = actionDef(at);
            const QString why = unsupportedReason(u"action"_s, a.value(u"type"_s).toString());
            m_inspActions.append(QVariantMap{
                {u"idx"_s, int(i)},
                {u"label"_s, ad.value(u"label"_s).toString()},
                {u"summary"_s, why.isEmpty() ? actionSummary(a) : u"not flown by %1"_s.arg(platLabel)},
                {u"unsupported"_s, !why.isEmpty()},
                {u"title"_s, why},
                {u"fields"_s, fieldsFor(ad.value(u"fields"_s).toArray(), a, [why](const QJsonObject &, const QString &) { return why; })},
            });
        }
    }
    emit inspectorChanged();
    refreshInspectorDetail();
}

void PlanController::refreshInspectorDetail()
{
    m_inspIssues.clear();
    m_inspDerived.clear();
    const QJsonObject seg = segment(m_selectedId);
    if (!seg.isEmpty()) {
        for (const Issue &i : m_compiled.issues)
            if (i.segId == m_selectedId) m_inspIssues.append(QVariantMap{{u"level"_s, i.level}, {u"text"_s, i.text}});
        const QString type = seg.value(u"type"_s).toString();
        if (type == "area"_L1 || type == "corridor"_L1) {
            const QJsonObject P = params();
            const SurveyGeometry g = surveyGeometry(seg, segmentAlt(seg, P), P.value(u"camera"_s).toObject());
            m_inspDerived << u"lane spacing <b>%1 m</b>"_s.arg(g.sideDistance, 0, 'f', 1);
            if (g.hasSensor) {
                m_inspDerived << u"swath <b>%1 m</b> at %2 m%3"_s.arg(g.sensor.swath, 0, 'f', 0).arg(g.sensorDistance, 0, 'f', 0)
                                     .arg(g.sensor.limitedByRange ? u" (range-limited)"_s : QString());
            } else {
                m_inspDerived << u"trigger every <b>%1 m</b>"_s.arg(g.triggerDistance, 0, 'f', 1);
                if (g.gsd) m_inspDerived << u"GSD <b>%1 cm/px</b>"_s.arg(*g.gsd, 0, 'f', 2);
                m_inspDerived << u"footprint <b>%1 × %2 m</b>"_s.arg(g.footprint.width, 0, 'f', 0).arg(g.footprint.height, 0, 'f', 0);
            }
        }
    }
    emit inspectorDetailChanged();
}

QString PlanController::inspectorCoords() const
{
    const QList<LL> pts = pointsOf(segment(m_selectedId));
    if (pts.size() == 1) return fmtLL(pts[0], 6);
    return u"%1 points · drag the handles on the map, right-click a vertex to insert or remove"_s.arg(pts.size());
}

bool PlanController::inspectorCanAct() const
{
    return segment(m_selectedId).value(u"type"_s).toString() != "poi"_L1;
}

QVariantList PlanController::addActionOptions() const
{
    auto option = [](const QString &value, const QString &label, const QString &why) {
        return QVariantMap{{u"value"_s, value}, {u"label"_s, label}, {u"disabled"_s, !why.isEmpty()}, {u"title"_s, why}};
    };
    QVariantList out{option({}, u"+ add…"_s, {})};
    for (const QString &k : actionOrder()) out.append(option(k, actionDef(k).value(u"label"_s).toString(), unsupportedReason(u"action"_s, k)));
    return out;
}

void PlanController::selectSegment(const QString &id, bool pan)
{
    m_selectedId = segIndex(id) >= 0 ? id : QString();
    refreshList();
    rebuildInspector();
    emit selectionChanged();
    emit mapChanged();
    if (!m_selectedId.isEmpty() && pan) {
        if (const auto c = centroid(pointsOf(segment(m_selectedId)))) emit panRequested(c->lat, c->lng);
    }
}

void PlanController::selectFromMap(const QString &id)
{
    selectSegment(id);
    emit scrollToSegment(segIndex(id));
}

void PlanController::toggleSegment(const QString &id, bool pan)
{
    selectSegment(m_selectedId == id && !pan ? QString() : id, pan);
}

void PlanController::addSegment(const QJsonObject &seg, const QString &label)
{
    // A landing must be the last thing: anything added after it goes before it
    QJsonArray a = segs();
    const bool lastLanding = !a.isEmpty() && a.last().toObject().value(u"type"_s).toString() == "landing"_L1
                             && seg.value(u"type"_s).toString() != "landing"_L1;
    a.insert(lastLanding ? a.size() - 1 : a.size(), seg);
    setSegs(a);
    commit(label);
    selectFromMap(seg.value(u"id"_s).toString());
    scheduleCompile();
}

void PlanController::deleteSegment(const QString &id)
{
    const int i = segIndex(id);
    if (i < 0) return;
    QJsonArray a = segs();
    a.removeAt(i);
    setSegs(a);
    if (m_selectedId == id) m_selectedId.clear();
    commit(u"Delete segment"_s);
    refreshList();
    rebuildInspector();
    emit selectionChanged();
    emit mapChanged();
    scheduleCompile();
}

void PlanController::deleteSelected()
{
    if (!m_selectedId.isEmpty()) deleteSegment(m_selectedId);
}

void PlanController::moveSegment(int from, int to)
{
    QJsonArray a = segs();
    if (from < 0 || to < 0 || from >= a.size() || to >= a.size() || from == to) return;
    const QJsonValue moved = a.takeAt(from);
    a.insert(to, moved);
    setSegs(a);
    commit(u"Reorder segments"_s);
    refreshList();
    scheduleCompile();
}

// ── Fields ───────────────────────────────────────────────────────────────────

void PlanController::setField(const QString &scope, const QString &segId, int actionIdx, const QString &key, const QVariant &value)
{
    const QJsonObject seg = segment(segId);
    QJsonObject action;
    if ((scope == "seg"_L1 || scope == "action"_L1) && seg.isEmpty()) return;
    if (scope == "action"_L1) {
        const QJsonArray acts = seg.value(u"actions"_s).toArray();
        if (actionIdx < 0 || actionIdx >= acts.size()) return;
        action = acts[actionIdx].toObject();
    }
    const QJsonArray fields = fieldsOf(scope, seg, action);
    QJsonObject f;
    for (const QJsonValue &fv : fields)
        if (fv.toObject().value(u"key"_s).toString() == key) { f = fv.toObject(); break; }
    if (f.isEmpty()) return;
    const std::optional<QJsonValue> v = parseField(f, value);
    if (!v) return;
    if (scope == "seg"_L1) setSegmentParam(seg, key, *v, hasWhenOn(fields, key));
    else if (scope == "action"_L1) setActionField(seg, actionIdx, action, key, *v);
    else if (scope == "route"_L1 || scope == "gimbal"_L1) setRouteParam(scope == "route"_L1, key, *v, hasWhenOn(fields, key));
    else if (scope == "camera"_L1 || scope == "radio"_L1) setProfileField(scope, key, *v);
}

void PlanController::setSegmentParam(QJsonObject seg, const QString &key, const QJsonValue &v, bool reshapes)
{
    QJsonObject sp = seg.value(u"params"_s).toObject();
    if (sp.value(key) == v) return;
    sp.insert(key, v);
    seg.insert(u"params"_s, sp);
    putSegment(seg);
    commit(u"Edit %1"_s.arg(segmentDef(seg.value(u"type"_s).toString()).value(u"label"_s).toString().toLower()));
    // Fields with `when` clauses may have appeared or disappeared
    if (reshapes) rebuildInspector();
    refreshList();
    refreshInspectorDetail();
    emit mapChanged();
    scheduleCompile();
}

void PlanController::setActionField(QJsonObject seg, int idx, QJsonObject action, const QString &key, const QJsonValue &v)
{
    if (action.value(key) == v) return;
    action.insert(key, v);
    QJsonArray acts = seg.value(u"actions"_s).toArray();
    acts[idx] = action;
    seg.insert(u"actions"_s, acts);
    putSegment(seg);
    commit(u"Edit action"_s);
    // The card's summary follows the edit, unless the action is greyed out
    if (idx < m_inspActions.size()) {
        QVariantMap m = m_inspActions[idx].toMap();
        if (!m.value(u"unsupported"_s).toBool()) {
            m.insert(u"summary"_s, actionSummary(action));
            m_inspActions[idx] = m;
            emit inspectorChanged();
        }
    }
    refreshList();
    scheduleCompile();
}

void PlanController::setRouteParam(bool route, const QString &key, const QJsonValue &v, bool reshapes)
{
    QJsonObject P = params();
    if (P.value(key) == v) return;
    P.insert(key, v);
    setParams(P);
    if (route) {
        commit(u"Route %1"_s.arg(key));
        // Dependent fields (AGL tolerance, take-off altitude) appear or vanish
        if (reshapes) emit formsChanged();
        if (key == "defaultAlt"_L1) rebuildInspector();   // the altitude placeholder
        refreshCard();
        refreshList();
    } else {
        commit(u"Gimbal defaults"_s);
    }
    emit derivedChanged();
    scheduleCompile();
}

void PlanController::setProfileField(const QString &scope, const QString &key, const QJsonValue &v)
{
    const bool camera = scope == "camera"_L1;
    QJsonObject P = params();
    QJsonObject prof = P.value(scope).toObject();
    if (prof.value(key) == v) return;
    prof.insert(key, v);
    // A preset fills the profile; hand-edited values (for the radio, those of the link budget) make it custom
    static const QSet<QString> radioSpec{u"freq"_s, u"txPower"_s, u"gainGround"_s, u"gainAir"_s, u"minRssi"_s};
    bool reshaped = true;
    if (key == "preset"_L1) {
        const QJsonObject preset = (camera ? cameraPresets() : radioPresets()).value(v.toString()).toObject();
        if (!preset.isEmpty() && v.toString() != "custom"_L1)
            for (auto it = preset.begin(); it != preset.end(); ++it) prof.insert(it.key(), it.value());
        else prof.insert(u"name"_s, u"Custom"_s);
    } else if ((camera || radioSpec.contains(key)) && prof.value(u"preset"_s).toString() != "custom"_L1) {
        prof.insert(u"preset"_s, u"custom"_s);
        prof.insert(u"name"_s, u"Custom"_s);
    } else {
        reshaped = false;
    }
    P.insert(scope, prof);
    setParams(P);
    if (camera) {
        commit(u"Camera profile"_s);
        emit formsChanged();
        emit derivedChanged();
        refreshCard();
        refreshInspectorDetail();
        scheduleCompile();
    } else {
        commit(u"Radio profile"_s);
        rememberRadio(prof);
        Settings::instance().setJson(QLatin1String(kLsRadio), prof);
        if (reshaped) emit formsChanged();
        emit derivedChanged();
        scheduleLink();
    }
}

void PlanController::addAction(const QString &segId, const QString &type)
{
    QJsonObject seg = segment(segId);
    if (seg.isEmpty() || type.isEmpty() || !actionTypes().contains(type)) return;
    if (const QString why = unsupportedReason(u"action"_s, type); !why.isEmpty()) {
        setHint(why, true);
        return;
    }
    QJsonArray acts = seg.value(u"actions"_s).toArray();
    acts.append(createAction(type));
    seg.insert(u"actions"_s, acts);
    putSegment(seg);
    commit(u"Add action"_s);
    rebuildInspector();
    refreshList();
    scheduleCompile();
}

void PlanController::removeAction(const QString &segId, int idx)
{
    QJsonObject seg = segment(segId);
    QJsonArray acts = seg.value(u"actions"_s).toArray();
    if (idx < 0 || idx >= acts.size()) return;
    acts.removeAt(idx);
    seg.insert(u"actions"_s, acts);
    putSegment(seg);
    commit(u"Remove action"_s);
    rebuildInspector();
    refreshList();
    scheduleCompile();
}

// ── Popover forms ────────────────────────────────────────────────────────────

QVariantList PlanController::routeForm() const
{
    // Route settings the active flight stack has a use for
    const QString plat = activePlatformId();
    QJsonArray fields;
    for (const QJsonValue &fv : routeParamFields()) {
        const QJsonArray pl = fv.toObject().value(u"platforms"_s).toArray();
        if (pl.isEmpty() || pl.contains(plat)) fields.append(fv);
    }
    return fieldsFor(fields, params(), [](const QJsonObject &f, const QString &opt) {
        const QString key = f.value(u"key"_s).toString();
        return unsupportedReason(u"routeParam"_s, opt.isEmpty() ? key : key + '.'_L1 + opt);
    });
}

QVariantList PlanController::cameraForm() const { return fieldsFor(cameraFields(), params().value(u"camera"_s).toObject(), {}); }
QVariantList PlanController::gimbalForm() const { return fieldsFor(gimbalFields(), params(), {}); }
QVariantList PlanController::radioForm() const { return fieldsFor(radioFields(), params().value(u"radio"_s).toObject(), {}); }

QStringList PlanController::cameraDerived() const
{
    const QJsonObject P = params();
    const QJsonObject cam = P.value(u"camera"_s).toObject();
    const Fov fov = cameraFov(cam);
    const double alt = num(P.value(u"defaultAlt"_s));
    const Footprint fp = cameraFootprint(std::max(1.0, alt ? alt : 100), cam);
    QStringList out{u"FOV <b>%1° × %2°</b>"_s.arg(fov.fovH, 0, 'f', 1).arg(fov.fovV, 0, 'f', 1),
                    u"at %1 m: footprint <b>%2 × %3 m</b>"_s.arg(fmtNum(alt)).arg(fp.width, 0, 'f', 0).arg(fp.height, 0, 'f', 0)};
    const double imgW = num(cam.value(u"imgW"_s));
    if (imgW > 0) out << u"GSD <b>%1 cm/px</b>"_s.arg(fp.width / imgW * 100, 0, 'f', 2);
    const double yaw = num(P.value(u"gimbalYaw"_s));
    out << u"pitch <b>%1°</b>%2"_s.arg(fmtNumV(P.value(u"gimbalPitch"_s)), yaw ? u" yaw <b>%1°</b>"_s.arg(fmtNum(yaw)) : QString());
    return out;
}

QStringList PlanController::radioDerived() const
{
    const QJsonObject P = params();
    const QJsonObject R = P.value(u"radio"_s).toObject();
    auto n = [&](const char *k) { return num(R.value(QLatin1String(k))); };
    const double budget = n("txPower") + n("gainGround") + n("gainAir") - n("losses") - n("minRssi");
    const double fs0 = freeSpaceRange(R, 0), fsM = freeSpaceRange(R, n("minMargin"));
    const QString mode = P.value(u"altMode"_s).toString();
    const QString alt = fmtNumV(P.value(u"defaultAlt"_s));
    const QString altText = mode == "amsl"_L1 ? u"%1 m AMSL"_s.arg(alt) : mode == "rel"_L1 ? u"%1 m above take-off"_s.arg(alt) : u"%1 m AGL"_s.arg(alt);
    QStringList parts{u"budget <b>%1 dB</b>"_s.arg(budget, 0, 'f', 0),
                      u"free space <b>%1</b> with margin · <b>%2</b> to sensitivity"_s.arg(fmtDist(fsM), fmtDist(fs0)),
                      u"overlay at <b>%1</b>"_s.arg(altText)};
    if (m_gs) parts << u"antenna <b>%1</b>"_s.arg(m_gs->antennaMsl ? u"%1 m AMSL"_s.arg(qRound64(*m_gs->antennaMsl)) : u"no terrain data"_s);
    if (m_showLink && m_analysis) {
        const auto &A = *m_analysis;
        const Link cls = A.worst ? A.worst->clazz : Link::Good;
        parts << u"route: <b><font color=\"%1\">%2</font></b>"_s.arg(linkStyle(cls == Link::Unknown ? Link::None : cls).color.name(), summarizeLink(A));
        if (A.worst && A.worst->clazz != Link::Good)
            parts << u"worst at <b>%1</b>: %2 dBm, margin <b>%3 dB</b>%4"_s.arg(fmtDist(A.worst->dist)).arg(A.worst->rssi, 0, 'f', 0)
                         .arg(A.worst->margin, 0, 'f', 0).arg(A.worst->loss > 0 ? u", terrain −%1 dB"_s.arg(A.worst->loss, 0, 'f', 0) : QString());
        parts << u"farthest <b>%1</b> from the antenna"_s.arg(fmtDist(A.maxGsDist));
    } else if (!m_showLink) {
        parts << u"enable the coverage overlay to analyse the route"_s;
    }
    return parts;
}

QString PlanController::homeDesc() const
{
    QString kind;
    const auto t = takeoffPoint(&kind);
    if (!t) return u"first segment (no vehicle)"_s;
    if (kind == "custom"_L1) return u"%1 (custom)"_s.arg(fmtLL(*t, 5));
    if (kind == "home"_L1) return u"vehicle home %1"_s.arg(fmtLL(*t, 5));
    if (kind == "vehicle"_L1) return u"vehicle position (no home yet)"_s;
    return u"last vehicle home %1 (no vehicle)"_s.arg(fmtLL(*t, 5));
}

QString PlanController::operatorDesc() const
{
    if (const auto op = placedOperator()) return fmtLL(*op, 5);
    return m_compiled.home ? u"at the take-off point"_s : u"at the take-off point (none yet)"_s;
}

bool PlanController::operatorPlaced() const { return placedOperator().has_value(); }

void PlanController::setHomeVehicle()
{
    if (const auto p = vehicleFix()) setHome(p->lat, p->lng);
    else emit toast(u"No vehicle position available"_s);
}

void PlanController::setPlacement(const QString &key, std::optional<LL> pos, const QString &label)
{
    QJsonObject P = params();
    P.insert(key, pos ? QJsonValue(llJson(*pos)) : QJsonValue(QJsonValue::Null));
    setParams(P);
    commit(label);
    emit derivedChanged();
    emit mapChanged();
}

void PlanController::setHome(double lat, double lng)
{
    setPlacement(u"home"_s, LL{lat, lng}, u"Set take-off point"_s);
    scheduleCompile();
}

void PlanController::resetHome()
{
    setPlacement(u"home"_s, std::nullopt, u"Reset take-off point"_s);
    scheduleCompile();
}

void PlanController::setOperator(std::optional<LL> pos)
{
    setPlacement(u"operator"_s, pos, pos ? u"Place operator"_s : u"Reset operator"_s);
    scheduleLink(0);
}

void PlanController::placeOperator()
{
    emit openPopover(QString());
    setTool(u"operator"_s);
}

void PlanController::resetOperator() { setOperator(std::nullopt); }
void PlanController::openRouteSettingsFromMap() { selectSegment({}); emit openPopover(u"route"_s); }
void PlanController::openRadioFromMap() { selectSegment({}); emit openPopover(u"radio"_s); }

// ── Tools & drawing ──────────────────────────────────────────────────────────

QVariantList PlanController::tools() const
{
    QVariantList out;
    auto add = [&](const QString &id, const QString &label, const QString &key, bool aux) {
        const QString why = segmentTypes().contains(id) ? unsupportedReason(u"segment"_s, id) : QString();
        out.append(QVariantMap{{u"id"_s, id}, {u"aux"_s, aux}, {u"disabled"_s, !why.isEmpty()},
                               {u"title"_s, why.isEmpty() ? u"%1 (%2)"_s.arg(label, key) : u"%1 — %2"_s.arg(label, why)}});
    };
    add(u"select"_s, u"Select / pan"_s, u"V"_s, false);
    for (const QString &t : segmentOrder()) {
        const QJsonObject d = segmentDef(t);
        add(t, d.value(u"label"_s).toString(), d.value(u"hotkey"_s).toString(), false);
    }
    add(u"operator"_s, u"Operator position (ground antenna)"_s, u"O"_s, true);
    return out;
}

void PlanController::setHint(const QString &text, bool flash)
{
    m_hint = text;
    emit hintChanged();
    if (flash) m_hintTimer.start(4000);
}

void PlanController::setTool(const QString &name)
{
    const QJsonObject def = segmentDef(name);
    if (const QString why = !def.isEmpty() ? unsupportedReason(u"segment"_s, name) : QString(); !why.isEmpty()) {
        setHint(u"%1: %2"_s.arg(def.value(u"label"_s).toString(), why), true);
        return;
    }
    if (m_draft) cancelDraft();
    m_tool = name;
    emit toolChanged();
    setHint(name == "operator"_L1 ? u"Click on the map where the operator and the ground antenna will be — Esc to cancel"_s
                                  : def.value(u"hint"_s).toString());
}

void PlanController::keyTool(const QString &key)
{
    const QString k = key.toUpper();
    if (k == "V"_L1) return setTool(u"select"_s);
    if (k == "O"_L1) return setTool(u"operator"_s);
    for (const QString &t : segmentOrder())
        if (segmentDef(t).value(u"hotkey"_s).toString() == k) return setTool(t);
}

void PlanController::escape()
{
    if (m_draft) { cancelDraft(); setTool(u"select"_s); }
    else if (m_tool != "select"_L1) setTool(u"select"_s);
    else if (!m_selectedId.isEmpty()) selectSegment({});
    emit openPopover(QString());
}

void PlanController::mapClick(double lat, double lng, bool nearFirstDraftPoint)
{
    if (m_tool == "select"_L1) { selectSegment({}); return; }
    if (m_tool == "operator"_L1) {
        setOperator(LL{lat, lng});
        setTool(u"select"_s);
        return;
    }
    const QJsonObject def = segmentDef(m_tool);
    if (def.isEmpty()) return;
    const QJsonValue pts = def.value(u"points"_s);
    if (pts.isDouble()) {
        addSegment(createSegment(m_tool, {{lat, lng}}), u"Add %1"_s.arg(def.value(u"label"_s).toString().toLower()));
        if (m_tool != "waypoint"_L1) setTool(u"select"_s);   // waypoints chain, the others are one-shot
        return;
    }
    // Polygon / polyline: accumulate vertices; a click on the first vertex closes a polygon
    if (pts.toString() == "poly"_L1 && m_draft && m_draft->points.size() >= 3 && nearFirstDraftPoint) finishDraft();
    else draftAddPoint(lat, lng);
}

void PlanController::draftAddPoint(double lat, double lng)
{
    if (!m_draft) m_draft = Draft{m_tool, {}};
    m_draft->points.append({lat, lng});
    emit mapChanged();
}

void PlanController::mapDoubleClick()
{
    if (m_draft) finishDraft();
}

void PlanController::finishDraft()
{
    if (!m_draft) return;
    const Draft d = *m_draft;
    const QJsonObject def = segmentDef(d.type);
    const int need = def.value(u"points"_s).toString() == "poly"_L1 ? 3 : 2;
    m_draft.reset();
    if (d.points.size() < need) { emit mapChanged(); return; }
    addSegment(createSegment(d.type, d.points), u"Add %1"_s.arg(def.value(u"label"_s).toString().toLower()));
    setTool(u"select"_s);
}

void PlanController::cancelDraft()
{
    m_draft.reset();
    emit mapChanged();
}

void PlanController::movePoint(const QString &segId, int idx, double lat, double lng)
{
    QJsonObject seg = segment(segId);
    QJsonArray pts = seg.value(u"points"_s).toArray();
    if (idx < 0 || idx >= pts.size()) return;
    pts[idx] = llJson({lat, lng});
    seg.insert(u"points"_s, pts);
    putSegment(seg);
    emit mapChanged();
}

void PlanController::insertPoint(const QString &segId, int afterIdx, double lat, double lng)
{
    QJsonObject seg = segment(segId);
    QJsonArray pts = seg.value(u"points"_s).toArray();
    pts.insert(afterIdx + 1, llJson({lat, lng}));
    seg.insert(u"points"_s, pts);
    putSegment(seg);
    emit mapChanged();
}

void PlanController::moveSegmentBy(const QString &segId, const QList<LL> &original, double dLat, double dLng)
{
    QJsonObject seg = segment(segId);
    QList<LL> moved;
    for (const LL &p : original) moved.append({p.lat + dLat, p.lng + dLng});
    seg.insert(u"points"_s, pointsToJson(moved));
    putSegment(seg);
    emit mapChanged();
}

void PlanController::setCircleRadius(const QString &segId, double radius)
{
    QJsonObject seg = segment(segId);
    QJsonObject sp = seg.value(u"params"_s).toObject();
    sp.insert(u"radius"_s, std::max(5.0, double(qRound64(radius))));
    seg.insert(u"params"_s, sp);
    putSegment(seg);
    emit mapChanged();
}

void PlanController::endGesture(const QString &label)
{
    commit(label);
    refreshList();
    if (label == "Change radius"_L1) rebuildInspector();
    refreshInspectorDetail();
    scheduleCompile();
}

void PlanController::setProfileHover(std::optional<LL> pt, std::optional<Link> losClass)
{
    m_profileHover = pt;
    m_profileHoverClass = losClass;
    emit mapChanged();
}

void PlanController::contextAction(const QString &act, double lat, double lng, const QString &segId, int pointIdx)
{
    QJsonObject seg = segment(segId);
    if (act == "finish"_L1) finishDraft();
    else if (act == "cancel"_L1) { cancelDraft(); setTool(u"select"_s); }
    else if (act == "select"_L1) selectFromMap(segId);
    else if (act == "delete"_L1) deleteSegment(segId);
    else if (act == "insert"_L1 || act == "delvtx"_L1) {
        QJsonArray pts = seg.value(u"points"_s).toArray();
        if (pointIdx < 0 || pointIdx >= pts.size()) return;
        if (act == "insert"_L1) {
            const QList<LL> ll = pointsOf(seg);
            const LL a = ll[pointIdx], b = ll[(pointIdx + 1) % ll.size()];
            insertPoint(segId, pointIdx, (a.lat + b.lat) / 2, (a.lng + b.lng) / 2);
        } else {
            pts.removeAt(pointIdx);
            seg.insert(u"points"_s, pts);
            putSegment(seg);
        }
        commit(act == "insert"_L1 ? u"Insert vertex"_s : u"Delete vertex"_s);
        selectSegment(segId);
        scheduleCompile();
    } else if (act == "wp"_L1) addSegment(createSegment(u"waypoint"_s, {{lat, lng}}), u"Add waypoint"_s);
    else if (act == "poi"_L1) addSegment(createSegment(u"poi"_s, {{lat, lng}}), u"Add POI"_s);
    else if (act == "home"_L1) setHome(lat, lng);
    else if (act == "operator"_L1) setOperator(LL{lat, lng});
    else if (act == "deselect"_L1) selectSegment({});
}

// ── Menu ─────────────────────────────────────────────────────────────────────

void PlanController::menuAction(const QString &act)
{
    if (act == "new"_L1) confirmReplace(u"Start a new route? Unsaved changes will be lost."_s, [this] { loadRoute(createRoute()); });
    else if (act == "clear"_L1) clearRoute();
    else if (act == "invert"_L1) invertRoute();
    else if (act == "flatten"_L1) flattenRoute();
    else if (act == "fit"_L1) emit fitRequested();
    else if (act == "export"_L1) {
        if (m_compiled.items.isEmpty()) emit toast(u"Nothing to export — the route is empty."_s);
        else emit saveFileRequested(exportName());
    }
}

void PlanController::clearRoute()
{
    if (segs().isEmpty()) return;
    confirm(u"Remove every segment from the route?"_s, [this] {
        setSegs({});
        m_route.insert(u"name"_s, QJsonValue::Null);
        m_selectedId.clear();
        commit(u"Clear route"_s);
        applyHistory();
    });
}

void PlanController::invertRoute()
{
    // The segments in reverse order, lines walked the other way; a landing stays last
    QJsonArray a = segs();
    std::optional<QJsonValue> landing;
    if (!a.isEmpty() && a.last().toObject().value(u"type"_s).toString() == "landing"_L1) landing = a.takeAt(a.size() - 1);
    QJsonArray rev;
    for (qsizetype i = a.size() - 1; i >= 0; --i) {
        QJsonObject s = a[i].toObject();
        const QString t = s.value(u"type"_s).toString();
        if (t == "perimeter"_L1 || t == "corridor"_L1) {
            const QJsonArray p = s.value(u"points"_s).toArray();
            QJsonArray pr;
            for (qsizetype k = p.size() - 1; k >= 0; --k) pr.append(p[k]);
            s.insert(u"points"_s, pr);
        }
        rev.append(s);
    }
    if (landing) rev.append(*landing);
    setSegs(rev);
    commit(u"Invert route"_s);
    refreshList();
    emit mapChanged();
    scheduleCompile();
}

void PlanController::flattenRoute()
{
    if (m_compiled.items.isEmpty()) return;
    confirm(u"Replace every segment with the calculated waypoints? Areas and corridors will no longer be editable as shapes."_s, [this] {
        QList<RawItem> raw;
        for (const CompiledItem &it : m_compiled.items) raw.append(rawOf(it));
        QJsonObject flat = itemsToRoute(raw, m_route.value(u"name"_s).toString(), false);
        QJsonObject P = params();
        const QJsonObject fp = flat.value(u"params"_s).toObject();
        P.insert(u"takeoff"_s, fp.value(u"takeoff"_s));
        P.insert(u"endAction"_s, fp.value(u"endAction"_s));
        P.insert(u"defaultSpeed"_s, fp.value(u"defaultSpeed"_s));
        P.insert(u"altMode"_s, u"agl"_s);
        flat.insert(u"params"_s, P);
        replaceRoute(flat);
        m_selectedId.clear();
        commit(u"Convert to waypoints"_s);
        applyHistory();
    });
}

QString PlanController::exportName() const
{
    QString n = routeName();
    if (n.isEmpty()) n = u"route"_s;
    static const QRegularExpression bad(u"[^\\w.-]+"_s);
    n.replace(bad, u"_"_s);
    return n + u".waypoints"_s;
}

void PlanController::exportFile(const QUrl &url)
{
    QFile f(url.toLocalFile());
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        emit toast(u"Export failed: %1"_s.arg(f.errorString()));
        return;
    }
    f.write(toWpl(uploadItems(m_compiled.items)).toUtf8());
    LOG_I("plan") << "exported" << m_compiled.items.size() << "items to" << f.fileName();
}

void PlanController::importFile(const QUrl &url)
{
    QFile f(url.toLocalFile());
    if (!f.open(QIODevice::ReadOnly)) {
        emit toast(u"Import failed: %1"_s.arg(f.errorString()));
        return;
    }
    const QString text = QString::fromUtf8(f.readAll());
    const QString base = QFileInfo(f.fileName()).completeBaseName();
    QJsonObject route;
    if (text.trimmed().startsWith('{'_L1)) {
        QJsonParseError pe;
        const QJsonObject data = QJsonDocument::fromJson(text.toUtf8(), &pe).object();
        if (data.isEmpty()) { emit toast(u"Import failed: %1"_s.arg(pe.errorString())); return; }
        route = routeOfFile(data);
    } else {
        QList<RawItem> items;
        QString err;
        if (!parseWpl(text, items, &err)) { emit toast(u"Import failed: %1"_s.arg(err)); return; }
        route = itemsToRoute(items, base, true);
    }
    if (route.value(u"name"_s).toString().isEmpty()) route.insert(u"name"_s, base);
    confirmReplace(u"Replace the current route with the imported one?"_s, [this, route] { loadRoute(route, {}, {}, 400); });
}

// ── Upload / read ────────────────────────────────────────────────────────────

bool PlanController::beginTransfer(QString &button, const QString &text)
{
    if (m_transferBusy) return false;
    m_transferBusy = true;
    button = text;
    emit buttonsChanged();
    return true;
}

void PlanController::flashButton(QString &which, const QString &text, int ms)
{
    which = text;
    emit buttonsChanged();
    const QString rest = &which == &m_uploadText ? u"UPLOAD"_s : &which == &m_readText ? u"READ"_s : u"SAVE"_s;
    QTimer::singleShot(ms, this, [this, &which, rest] {
        which = rest;
        m_transferBusy = false;
        emit buttonsChanged();
    });
}

void PlanController::readFailed(const QString &why, int ms)
{
    emit toast(why);
    flashButton(m_readText, u"READ"_s, ms);
}

void PlanController::readDone(const QJsonObject &route, int count)
{
    loadRoute(route, {}, {}, 400);
    flashButton(m_readText, u"READ %1"_s.arg(count), 2000);
}

void PlanController::upload()
{
    const bool msp = usesMspMissions();
    if (const QString problem = msp ? inavLinkProblem() : vehicleConnected() ? QString() : u"Not connected"_s; !problem.isEmpty()) {
        emit toast(problem);
        return;
    }
    if (segs().isEmpty()) { emit toast(u"The route is empty"_s); return; }
    if (msp && (!m_compiled.inav || m_compiled.inav->wps.isEmpty())) { emit toast(u"Nothing in this route can be uploaded to INAV"_s); return; }
    QStringList errors;
    for (const Issue &i : m_compiled.issues) if (i.level == "error"_L1) errors << u"• "_s + i.text;
    auto go = [this, msp] { msp ? doUploadInav() : doUpload(); };
    if (errors.isEmpty()) go();
    else confirm(u"The route has %1 error%2:\n%3\n\nUpload anyway?"_s.arg(errors.size()).arg(errors.size() > 1 ? u"s"_s : QString(), errors.join('\n'_L1)), go);
}

void PlanController::read()
{
    if (!vehicleConnected()) { emit toast(u"Not connected"_s); return; }
    const bool msp = usesMspMissions();
    if (const QString problem = msp ? inavLinkProblem() : QString(); !problem.isEmpty()) { emit toast(problem); return; }
    confirmReplace(u"Replace the current route with the mission stored on the vehicle?"_s, [this, msp] { msp ? doReadInav() : doRead(); });
}

void PlanController::doUpload()
{
    if (!beginTransfer(m_uploadText, u"UPLOADING…"_s)) return;
    QList<MissionItem> items;
    for (const RawItem &r : uploadItems(m_compiled.items)) items.append(missionItemOf(r));
    const int n = int(items.size());
    QPointer<PlanController> self(this);
    m_cmd->uploadMission(items, [self, n](bool ok, const QString &text) {
        if (!self) return;
        if (!ok) emit self->toast(u"Upload failed: %1"_s.arg(text));
        self->flashButton(self->m_uploadText, ok ? u"DONE (%1)"_s.arg(n) : u"UPLOAD"_s, 2000);
    });
}

void PlanController::doRead()
{
    if (!beginTransfer(m_readText, u"READING…"_s)) return;
    QPointer<PlanController> self(this);
    m_cmd->downloadMission(
        [self](int done, int total) {
            if (!self) return;
            self->m_readText = u"READING %1/%2"_s.arg(done).arg(total);
            emit self->buttonsChanged();
        },
        [self](bool ok, const QString &error, const QList<MissionItem> &items) {
            if (!self) return;
            if (!ok) { self->readFailed(u"Read failed: %1"_s.arg(error), 2000); return; }
            QList<RawItem> raw;
            for (qsizetype i = 0; i < items.size(); ++i) raw.append(rawOf(int(i), items[i]));
            const QJsonObject route = itemsToRoute(raw, u"From vehicle"_s, true);
            // ArduPilot always keeps its home at seq 0: with nothing after it there is no mission
            if (route.value(u"segments"_s).toArray().isEmpty()) self->readFailed(u"The vehicle has no mission stored (only its home position)."_s, 10);
            else self->readDone(route, int(items.size()));
        });
}

// ── Upload / read, INAV (InavTransfer.js) ────────────────────────────────────
// The waypoint records come from the compiler (RouteCompiler -> INAV); the MSP
// session on the link thread owns the transfer, one record in flight at a time.

QString PlanController::inavLinkProblem() const
{
    const auto s = snapshot();
    if (!s || !s->connected) return u"Not connected"_s;
    if (!s->connectionType.startsWith("msp"_L1))
        return u"%1 missions travel over MSP — connect the aircraft as MSP Serial or MSP TCP (SETUP → CONNECTION)"_s.arg(platform(activePlatformId()).label);
    return {};
}

LinkManager *PlanController::mspTransfer(const QString &op, QString &button, const QString &text, const MspProgress &progress, const MspDone &done)
{
    LinkManager *link = GcsController::s_instance ? GcsController::s_instance->link() : nullptr;
    if (!link || !beginTransfer(button, text)) return nullptr;
    auto pc = std::make_shared<QMetaObject::Connection>();
    auto dc = std::make_shared<QMetaObject::Connection>();
    *pc = connect(link, &LinkManager::mspMissionProgress, this, progress, Qt::QueuedConnection);
    *dc = connect(link, &LinkManager::mspMissionDone, this, [pc, dc, op, done](const QString &o, bool ok, const QString &error, const QVariantMap &r) {
        if (o != op) return;
        QObject::disconnect(*pc);
        QObject::disconnect(*dc);
        done(ok, error, r);
    }, Qt::QueuedConnection);
    return link;
}

void PlanController::doUploadInav()
{
    const auto progress = [this](const QString &phase, int d, int total) {
        m_uploadText = phase == "upload"_L1 ? u"UPLOADING %1/%2"_s.arg(d).arg(total) : u"UPLOADING…"_s;
        emit buttonsChanged();
    };
    const auto done = [this](bool ok, const QString &error, const QVariantMap &r) {
        // The board just said how many waypoints it holds: believe it
        if (const int maxWp = r.value(u"maxWaypoints"_s).toInt(); maxWp > 0 && maxWp != m_inavLimit) {
            m_inavLimit = maxWp;
            scheduleCompile(0);
        }
        if (!ok) emit toast(u"Upload failed: %1"_s.arg(error));
        flashButton(m_uploadText, ok ? u"DONE (%1)%2"_s.arg(r.value(u"count"_s).toInt()).arg(r.value(u"saved"_s).toBool() ? u" · SAVED"_s : QString())
                                     : u"UPLOAD"_s, 2000);
    };
    LinkManager *link = mspTransfer(u"upload"_s, m_uploadText, u"UPLOADING…"_s, progress, done);
    if (!link) return;
    QList<MspWaypoint> wps;
    for (const InavWp &w : m_compiled.inav->wps) wps << MspWaypoint{w.index, w.action, w.lat, w.lon, w.alt, w.p1, w.p2, w.p3, w.flag};
    link->mspMissionUpload(wps, params().value(u"inavSaveEeprom"_s).toBool());
}

void PlanController::doReadInav()
{
    const auto progress = [this](const QString &, int d, int total) {
        m_readText = u"READING %1/%2"_s.arg(d).arg(total);
        emit buttonsChanged();
    };
    const auto done = [this](bool ok, const QString &error, const QVariantMap &r) {
        if (!ok) { readFailed(u"Read failed: %1"_s.arg(error), 2000); return; }
        if (const int maxWp = r.value(u"maxWaypoints"_s).toInt(); maxWp > 0) m_inavLimit = maxWp;
        const QVariantList list = r.value(u"wps"_s).toList();
        if (list.isEmpty()) { readFailed(u"The flight controller has no mission loaded"_s, 10); return; }
        QList<InavWp> wps;
        for (const QVariant &v : list) wps << inavWpOf(v.toMap());
        readDone(itemsToRoute(inavToItems(wps), u"From INAV"_s, true), int(wps.size()));
    };
    if (LinkManager *link = mspTransfer(u"download"_s, m_readText, u"READING…"_s, progress, done)) link->mspMissionDownload();
}

// ── Library ──────────────────────────────────────────────────────────────────

void PlanController::save() { saveCurrent(false); }
void PlanController::saveAsNew() { saveCurrent(true); }

void PlanController::saveCurrent(bool forceNew)
{
    if (segs().isEmpty()) { emit toast(u"Nothing to save — the route is empty."_s); return; }
    if (!forceNew && !m_currentId.isEmpty()) { writeMission(m_currentId, m_currentName); return; }
    const QString suggestion = !m_currentName.isEmpty() ? m_currentName : u"Mission %1"_s.arg(QDateTime::currentDateTimeUtc().toString(u"yyyy-MM-dd HH:mm"_s));
    prompt(u"Mission name:"_s, suggestion, [this](const QString &name) { writeMission({}, name); });
}

void PlanController::writeMission(const QString &id, const QString &name)
{
    QString err;
    const MissionStore::Saved saved = MissionStore::save(id, name, QString(), serializeItems(m_compiled.items), m_route, vehicleType(), &err);
    if (saved.id.isEmpty()) { emit toast(u"Save failed: %1"_s.arg(err)); return; }
    m_currentId = saved.id;
    m_currentName = saved.name;
    m_route.insert(u"name"_s, saved.name);
    m_baseline.insert(u"name"_s, saved.name);
    emit routeChanged();
    flashButton(m_saveText, u"SAVED"_s, 1500);
    refreshLibrary();
}

void PlanController::openLibrary()
{
    refreshLibrary();
    emit libraryOpenChanged(true);
}

void PlanController::refreshLibrary()
{
    const StoreIndex idx = MissionStore::rebuildIndex();
    m_libRoot = idx.root;
    m_libPortable = idx.portable;
    m_libMissions.clear();
    m_libLogs.clear();
    auto when = [](const QString &iso) {
        const QDateTime t = QDateTime::fromString(iso, Qt::ISODateWithMs);
        return t.isValid() ? t.toLocalTime().toString(u"dd/MM/yyyy, HH:mm"_s) : QString();
    };
    for (const MissionEntry &e : idx.missions)
        m_libMissions.append(QVariantMap{{u"id"_s, e.id}, {u"name"_s, e.name}, {u"notes"_s, e.notes},
                                         {u"meta"_s, u"%1 WP · %2 · %3"_s.arg(e.waypoints).arg(e.distanceM ? fmtDist(e.distanceM) : u"—"_s, when(e.modified))}});
    auto size = [](qint64 b) {
        return b >= 1048576 ? u"%1 MB"_s.arg(b / 1048576.0, 0, 'f', 1) : b >= 1024 ? u"%1 kB"_s.arg(qRound64(b / 1024.0)) : u"%1 B"_s.arg(b);
    };
    constexpr int kMaxLogs = 50;
    for (qsizetype i = 0; i < idx.logs.size() && i < kMaxLogs; ++i)
        m_libLogs.append(QVariantMap{{u"name"_s, idx.logs[i].name}, {u"meta"_s, u"%1 · %2"_s.arg(size(idx.logs[i].size), when(idx.logs[i].modified))}});
    if (idx.logs.size() > kMaxLogs)
        m_libLogs.append(QVariantMap{{u"name"_s, QString()}, {u"meta"_s, u"… %1 more in the logs folder"_s.arg(idx.logs.size() - kMaxLogs)}});
    emit libraryChanged();
}

void PlanController::loadMission(const QString &id)
{
    QString err;
    const QJsonObject data = MissionStore::load(id, &err);
    if (data.isEmpty()) { emit toast(u"Mission library error: %1"_s.arg(err)); return; }
    QJsonObject route = routeOfFile(data);
    QString name = data.value(u"name"_s).toString(id);
    if (name.isEmpty()) name = id;
    route.insert(u"name"_s, name);
    loadRoute(route, id, name, 300);
    emit libraryOpenChanged(false);
}

void PlanController::libraryAction(const QString &act, const QString &id)
{
    QString name = id, notes;
    for (const QVariant &v : m_libMissions) {
        const QVariantMap m = v.toMap();
        if (m.value(u"id"_s).toString() == id) { name = m.value(u"name"_s).toString(); notes = m.value(u"notes"_s).toString(); }
    }
    auto failed = [this](const QString &err) { emit toast(u"Mission library error: %1"_s.arg(err)); };
    if (act == "load"_L1) {
        confirmReplace(u"Replace the current plan? Unsaved changes will be lost."_s, [this, id] { loadMission(id); });
    } else if (act == "overwrite"_L1) {
        if (segs().isEmpty()) { emit toast(u"The current plan is empty — nothing to write over it with."_s); return; }
        confirm(u"Overwrite \"%1\" with the current plan (%2 segments, %3 items)?"_s.arg(name).arg(segs().size()).arg(m_compiled.items.size()),
                [this, id, name, notes, failed] {
                    QString err;
                    if (MissionStore::save(id, name, notes, serializeItems(m_compiled.items), m_route, vehicleType(), &err).id.isEmpty()) return failed(err);
                    m_currentId = id;
                    m_currentName = name;
                    refreshLibrary();
                });
    } else if (act == "rename"_L1) {
        prompt(u"New mission name:"_s, name, [this, id, name, failed](const QString &newName) {
            if (newName == name) return;
            QString err;
            if (!MissionStore::rename(id, newName, &err)) return failed(err);
            if (m_currentId == id) { m_currentName = newName; emit routeChanged(); }
            refreshLibrary();
        });
    } else if (act == "delete"_L1) {
        confirm(u"Delete \"%1\" permanently?"_s.arg(name), [this, id] {
            MissionStore::remove(id);
            if (m_currentId == id) { m_currentId.clear(); m_currentName.clear(); emit routeChanged(); }
            refreshLibrary();
        });
    }
}

void PlanController::revealDataFolder()
{
    QDesktopServices::openUrl(QUrl::fromLocalFile(DataRoot::root()));
}

// ── Dialogs ──────────────────────────────────────────────────────────────────

void PlanController::confirm(const QString &message, std::function<void()> yes)
{
    const int t = m_nextToken++;
    m_pending.insert(t, [yes](bool ok, const QString &) { if (ok) yes(); });
    emit confirmRequested(t, message);
}

void PlanController::confirmReplace(const QString &message, std::function<void()> go)
{
    if (segs().isEmpty()) go();
    else confirm(message, std::move(go));
}

void PlanController::prompt(const QString &message, const QString &value, std::function<void(const QString &)> done)
{
    const int t = m_nextToken++;
    m_pending.insert(t, [done](bool ok, const QString &text) {
        const QString v = text.trimmed();
        if (ok && !v.isEmpty()) done(v);
    });
    emit promptRequested(t, message, value);
}

void PlanController::answer(int token, bool ok, const QString &text)
{
    auto fn = m_pending.take(token);
    LOG_I("ui") << "dialog" << token << "->" << (ok ? "OK" : "Cancel");
    if (fn) fn(ok, text);
}

void PlanController::pageShown(bool shown)
{
    m_visible = shown;
    if (shown) emit mapChanged();
}

// ── Flight stack ─────────────────────────────────────────────────────────────

QString PlanController::platformId() const { return activePlatformId(); }

void PlanController::setPlatformId(const QString &id)
{
    if (!setActivePlatform(id)) return;
    LOG_I("plan") << "flight stack:" << id;
    // Everything platform-dependent is re-derived: tools, fields, how the route compiles
    m_inavLimit = 0;
    emit openPopover(QString());
    if (m_tool != "select"_L1 && m_tool != "operator"_L1 && !unsupportedReason(u"segment"_s, m_tool).isEmpty()) setTool(u"select"_s);
    emit platformChanged();
    emit formsChanged();
    rebuildInspector();
    refreshList();
    scheduleCompile(0);
}

QString PlanController::platformHint() const { return platform(activePlatformId()).hint; }

QString PlanController::platformLink() const
{
    const Platform &p = platform(activePlatformId());
    return u"%1 · %2 baud"_s.arg(connectionLabel(p.connType)).arg(p.connBaud);
}

QString PlanController::platformLabel() const { return platform(activePlatformId()).label; }
QString PlanController::platformConnType() const { return platform(activePlatformId()).connType; }
QString PlanController::platformConnLabel() const { return connectionLabel(platform(activePlatformId()).connType); }
int PlanController::platformBaud() const { return platform(activePlatformId()).connBaud; }
bool PlanController::missionsSupported() const { return mission::missionsSupported(); }

QString PlanController::noMissionReason() const
{
    const Platform &p = platform(activePlatformId());
    if (p.missions) return {};
    return p.noMissionReason.isEmpty() ? u"%1 cannot fly missions"_s.arg(p.label) : p.noMissionReason;
}

QString PlanController::setupReason(const QString &section) const
{
    const QString why = unsupportedReason(u"setup"_s, section);
    return why.isEmpty() ? QString() : u"%1: %2"_s.arg(platform(activePlatformId()).label, why);
}

// ── Radio link ───────────────────────────────────────────────────────────────

void PlanController::setShowLink(bool v)
{
    m_showLink = v;
    Settings::instance().setValue(QLatin1String(kLsLink), v ? u"1"_s : u"0"_s);
    emit layersChanged();
    scheduleLink(0);
    refreshCard();
    emit derivedChanged();
    emit mapChanged();
}

void PlanController::scheduleLink(int delayMs) { m_linkTimer.start(delayMs); }

void PlanController::runLink()
{
    if (m_coverageCancel) m_coverageCancel->store(true);
    if (m_linkBusy) { m_linkBusy = false; emit linkChanged(); }
    const QJsonObject R = params().value(u"radio"_s).toObject();
    const auto pos = operatorPoint();
    const TerrainFn terrain = terrainFn();
    m_gs = pos ? std::optional<GroundStation>(mission::groundStation(R, *pos, terrain)) : std::nullopt;
    m_analysis.reset();
    m_worstEval.reset();
    const bool analyse = m_showLink && m_gs;
    if (analyse) {
        if (!m_gs->antennaMsl) {
            // No tile under the antenna yet: the load (or download) recompiles, then analyses again
            const int lat = int(std::floor(pos->lat)), lng = int(std::floor(pos->lng));
            HgtStore::instance().get(lat, lng);
            if (HgtStore::instance().missing(lat, lng)) HgtDownloader::instance().ensure(lat, lng);
        }
        if (m_compiled.navPath.size() >= 2) m_analysis = analyzeRoute(R, *m_gs, m_compiled.navPath, terrain, 40);
        if (m_analysis && m_analysis->worst) {
            const LinkSample &w = *m_analysis->worst;
            m_worstEval = evaluateLink(R, *m_gs, w.lat, w.lng, w.altMsl, terrain, 30, 600, true);
        }
    }
    emit derivedChanged();
    refreshCard();
    emit mapChanged();
    if (analyse) updateCoverage(R, terrain);
}

void PlanController::updateCoverage(const QJsonObject &R, const TerrainFn &terrain)
{
    // The coverage raster: only when something it depends on changed
    const QJsonObject P = params();
    const double alt = num(P.value(u"defaultAlt"_s));
    const QString mode = P.value(u"altMode"_s).toString();
    const double homeElev = m_compiled.home ? m_compiled.home->elev : m_gs->elev.value_or(0);
    const QString key = u"%1|%2|%3|%4|%5|%6|%7"_s.arg(QString::fromUtf8(QJsonDocument(R).toJson(QJsonDocument::Compact)))
                            .arg(m_gs->lat, 0, 'f', 7).arg(m_gs->lng, 0, 'f', 7)
                            .arg(m_gs->antennaMsl ? QString::number(*m_gs->antennaMsl) : u"null"_s)
                            .arg(mode).arg(alt).arg(homeElev);
    if (key == m_coverageKey && m_coverage) return;
    if (!m_gs->antennaMsl) { m_coverage.reset(); m_coverageKey.clear(); emit mapChanged(); return; }

    std::function<double(double)> altAt;
    if (mode == "amsl"_L1) altAt = [alt](double) { return alt; };
    else if (mode == "rel"_L1) altAt = [alt, homeElev](double) { return homeElev + alt; };
    else altAt = [alt](double t) { return t + alt; };
    const double range = coverageRange(R);
    auto cancel = std::make_shared<std::atomic<bool>>(false);
    m_coverageCancel = cancel;
    m_linkBusy = true;
    emit linkChanged();
    using Result = std::shared_ptr<const Coverage>;
    auto *w = new QFutureWatcher<Result>(this);
    connect(w, &QFutureWatcher<Result>::finished, this, [this, w, cancel, key] {
        w->deleteLater();
        if (cancel->load()) return;
        const Result cov = w->result();
        if (m_coverageCancel == cancel) {
            m_linkBusy = false;
            emit linkChanged();
        }
        if (!cov) return;
        m_coverage = cov;
        m_coverageKey = key;
        emit mapChanged();
    });
    w->setFuture(QtConcurrent::run([R, gs = *m_gs, altAt, terrain, range, cancel]() -> Result {
        auto c = computeCoverage(R, gs, altAt, terrain, range, cancel.get());
        return c ? std::make_shared<const Coverage>(std::move(*c)) : nullptr;
    }));
}

void PlanController::publishMission3D()
{
    // The located navigation items, in mission order, as the 3D view draws them
    // (buildMissionTrajectory3D of main.js): altitudes already resolved against
    // the terrain by the compiler, so the route in 3D and the profile agree
    static const QSet<int> nav = {16, 17, 18, 19, 21, 22, 31, 82, 84, 85};
    Mission3DInput mission;
    std::vector<MapItem::MissionPoint> onMap;
    for (qsizetype i = 0; i < m_compiled.items.size(); ++i) {
        const CompiledItem &it = m_compiled.items[i];
        const bool home = it.isHome || (i == 0 && it.frame == 0 && it.command == 16);
        if (it.command == MAV_CMD_NAV_RETURN_TO_LAUNCH) {
            mission.rtlSeq = it.seq;
            continue;
        }
        if (!it.lat && !it.lng) continue;
        onMap.push_back({{it.lat, it.lng}, home});
        if (it.command == MAV_CMD_DO_SET_ROI || it.command == MAV_CMD_DO_SET_ROI_LOCATION) {
            mission.pois.push_back({it.lat, it.lng, it.altMsl, it.terrain, it.seq});
            continue;
        }
        if (!nav.contains(it.command)) continue;
        mission.rtlSeq.reset();   // a return closes the route only when nothing is flown after it
        MissionPoint3D p;
        p.lat = it.lat;
        p.lon = it.lng;
        p.altMsl = it.altMsl;
        p.ground = it.terrain;
        p.seq = it.seq;
        p.derived = it.derived;
        if (home) p.kind = u"home"_s;
        else if (it.command == 22 || it.command == 84) p.kind = u"takeoff"_s;
        else if (it.command == 21 || it.command == 85 || it.landing) p.kind = u"land"_s;
        else if (it.command == 17 || it.command == 18 || it.command == 19 || it.command == 31) p.kind = u"loiter"_s;
        else p.kind = u"wp"_s;
        if (p.kind == "loiter"_L1) {
            const double r = it.command == 31 ? it.param2 : it.param3;   // LOITER_TO_ALT keeps its radius in param 2
            p.loiterRadius = it.loiter ? it.loiter->r : std::abs(r);
            p.turns = it.loiter ? it.loiter->turns : it.command == 18 ? std::max(1, int(std::lround(it.param1))) : 1;
            p.ccw = r < 0;
        }
        p.navLand = it.command == 21;
        p.alt = it.alt;
        p.hold = it.command == 16 || it.command == 82 ? it.param1 : 0;
        p.segId = it.segId;
        p.segType = it.segType;
        mission.points.push_back(p);
    }
    // (a mission of nothing but a home draws nothing)
    if (mission.points.empty()) mission = {};
    SceneItem::setMission(std::move(mission));
    MapItem::setMission(std::move(onMap));
}

} // namespace corv
