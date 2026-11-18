#include "ui/PlanController.h"

#include "core/DataRoot.h"
#include "core/DebugLog.h"
#include "core/Settings.h"
#include "gpu/SceneItem.h"
#include "link/LinkManager.h"
#include "ui/GcsController.h"
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

namespace corv {

using namespace mission;

PlanController *PlanController::s_instance = nullptr;

namespace {

const char *kLsLink = "corv.fp.link";
const char *kLsRadio = "corv.fp.radio";
constexpr int kMaxHistory = 100;

QString fmtNum(double v) { return QString::number(v, 'g', 12); }
QString fmtNumV(const QJsonValue &v) { return fmtNum(num(v)); }

QString fmtDist(double m)
{
    return m >= 1000 ? QStringLiteral("%1 km").arg(m / 1000, 0, 'f', m >= 10000 ? 0 : 1) : QStringLiteral("%1 m").arg(qRound64(m));
}

QString fmtTime(double s)
{
    if (!s) return QStringLiteral("--");
    const qint64 m = qRound64(s / 60);
    return m >= 60 ? QStringLiteral("%1h %2m").arg(m / 60).arg(m % 60, 2, 10, QLatin1Char('0')) : QStringLiteral("%1 min").arg(m);
}

QString fmtArea(double m2)
{
    return m2 >= 1e6 ? QStringLiteral("%1 km²").arg(m2 / 1e6, 0, 'f', 2) : QStringLiteral("%1 ha").arg(m2 / 1e4, 0, 'f', 1);
}

QString lower(const QString &s) { return s.toLower(); }

QString linkName(Link c)
{
    switch (c) {
    case Link::Good: return QStringLiteral("good");
    case Link::Degraded: return QStringLiteral("degraded");
    case Link::Marginal: return QStringLiteral("marginal");
    default: return QStringLiteral("none");
    }
}

QJsonArray fieldsOf(const QString &scope, const QJsonObject &seg, const QJsonObject &action)
{
    if (scope == QLatin1String("seg")) return segmentDef(seg.value(QStringLiteral("type")).toString()).value(QStringLiteral("fields")).toArray();
    if (scope == QLatin1String("action")) return actionDef(action.value(QStringLiteral("type")).toString()).value(QStringLiteral("fields")).toArray();
    if (scope == QLatin1String("route")) return routeParamFields();
    if (scope == QLatin1String("camera")) return cameraFields();
    if (scope == QLatin1String("gimbal")) return gimbalFields();
    if (scope == QLatin1String("radio")) return radioFields();
    return {};
}

bool hasWhenOn(const QJsonArray &fields, const QString &key)
{
    for (const QJsonValue &f : fields)
        if (f.toObject().value(QStringLiteral("when")).toObject().contains(key)) return true;
    return false;
}

QString connectionLabel(const QString &type)
{
    static const QHash<QString, QString> labels{
        {QStringLiteral("mavlink-serial"), QStringLiteral("MAVLink Serial")}, {QStringLiteral("mavlink-udp"), QStringLiteral("MAVLink UDP")},
        {QStringLiteral("mavlink-tcp"), QStringLiteral("MAVLink TCP")},
        {QStringLiteral("msp-serial"), QStringLiteral("MSP Serial (INAV / Betaflight)")}, {QStringLiteral("msp-tcp"), QStringLiteral("MSP TCP (INAV SITL)")}};
    return labels.value(type, type);
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
    m_showLink = Settings::instance().value(QLatin1String(kLsLink)).toString() == QLatin1String("1");

    m_compileTimer.setSingleShot(true);
    connect(&m_compileTimer, &QTimer::timeout, this, &PlanController::runCompile);
    m_terrainRetry.setSingleShot(true);
    connect(&m_terrainRetry, &QTimer::timeout, this, &PlanController::runCompile);
    m_linkTimer.setSingleShot(true);
    connect(&m_linkTimer, &QTimer::timeout, this, &PlanController::runLink);
    m_hintTimer.setSingleShot(true);
    connect(&m_hintTimer, &QTimer::timeout, this, [this] { if (m_tool == QLatin1String("select")) setHint({}); });

    // Elevation arriving (read from disk or downloaded) changes the calculated route
    connect(&HgtStore::instance(), &HgtStore::tileLoaded, this, [this] { scheduleCompile(100); }, Qt::QueuedConnection);
    connect(&HgtDownloader::instance(), &HgtDownloader::tileDone, this, [this] { scheduleCompile(100); });

    // Twice a second: the vehicle marker, a home arriving, an empty plan following the vehicle
    m_pollTimer.setInterval(500);
    connect(&m_pollTimer, &QTimer::timeout, this, [this] {
        QString kind;
        const auto t = takeoffPoint(&kind);
        if (t && kind != QLatin1String("custom")
            && (kind != m_compiledHomeKind || !m_compiled.home || std::abs(m_compiled.home->lat - t->lat) > 1e-7
                || std::abs(m_compiled.home->lng - t->lng) > 1e-7)
            && !m_compileTimer.isActive() && !m_terrainRetry.isActive())
            scheduleCompile();
        if (!m_visible) return;
        // An empty plan goes where the vehicle is
        if (segs().isEmpty() && params().value(QStringLiteral("home")).toObject().isEmpty()) {
            QString k2;
            const auto base = takeoffPoint(&k2);
            const bool live = base && (k2 == QLatin1String("home") || k2 == QLatin1String("vehicle"));
            if (live && !(m_centeredLive && m_centeredAt && haversine(*m_centeredAt, *base) < 1000)) {
                emit centerRequested(base->lat, base->lng, 15);
                m_centeredAt = base;
                m_centeredLive = true;
            }
        }
        emit mapChanged();
    });
    m_pollTimer.start();
    scheduleCompile(0);
}

// ── Document helpers ─────────────────────────────────────────────────────────

QJsonArray PlanController::segs() const { return m_route.value(QStringLiteral("segments")).toArray(); }
void PlanController::setSegs(const QJsonArray &a) { m_route.insert(QStringLiteral("segments"), a); }
QJsonObject PlanController::params() const { return m_route.value(QStringLiteral("params")).toObject(); }
void PlanController::setParams(const QJsonObject &p) { m_route.insert(QStringLiteral("params"), p); }
int PlanController::segmentCount() const { return int(segs().size()); }

int PlanController::segIndex(const QString &id) const
{
    const QJsonArray a = segs();
    for (qsizetype i = 0; i < a.size(); ++i)
        if (a[i].toObject().value(QStringLiteral("id")).toString() == id) return int(i);
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
    const int i = segIndex(seg.value(QStringLiteral("id")).toString());
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

QString PlanController::routeName() const
{
    const QString n = m_route.value(QStringLiteral("name")).toString();
    return n.isEmpty() ? m_currentName : n;
}

void PlanController::setRouteName(const QString &name)
{
    const QString t = name.trimmed();
    m_route.insert(QStringLiteral("name"), t.isEmpty() ? QJsonValue(QJsonValue::Null) : QJsonValue(t));
    commit(QStringLiteral("Rename route"));
}

void PlanController::syncName() { emit routeChanged(); }

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

QString PlanController::undoTip() const { return m_undo.isEmpty() ? QStringLiteral("Nothing to undo") : QStringLiteral("Undo: %1 (Ctrl+Z)").arg(m_undo.last().label); }
QString PlanController::redoTip() const { return m_redo.isEmpty() ? QStringLiteral("Nothing to redo") : QStringLiteral("Redo: %1 (Ctrl+Y)").arg(m_redo.last().label); }

void PlanController::undo()
{
    if (m_undo.isEmpty()) return;
    const Step e = m_undo.takeLast();
    m_redo.append({m_baseline, e.label});
    m_baseline = e.doc;
    m_route = m_baseline;
    emit historyChanged();
    applyHistory(true);
}

void PlanController::redo()
{
    if (m_redo.isEmpty()) return;
    const Step e = m_redo.takeLast();
    m_undo.append({m_baseline, e.label});
    m_baseline = e.doc;
    m_route = m_baseline;
    emit historyChanged();
    applyHistory(true);
}

void PlanController::applyHistory(bool ok)
{
    if (!ok) return;
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

std::optional<LL> PlanController::takeoffPoint(QString *kind) const
{
    const QJsonObject home = params().value(QStringLiteral("home")).toObject();
    if (!home.isEmpty()) {
        if (kind) *kind = QStringLiteral("custom");
        return LL{num(home.value(QStringLiteral("lat"))), num(home.value(QStringLiteral("lng")))};
    }
    // Where the vehicle is based: its home, else the vehicle on the link, else the last home seen
    const auto s = m_hub ? m_hub->snapshot() : nullptr;
    std::optional<LL> base;
    QString k;
    if (s && s->homeLat && s->homeLon) {
        base = LL{*s->homeLat, *s->homeLon};
        k = QStringLiteral("home");
    } else if (s && !s->isDemo() && s->connected && s->gpsFix >= 2 && (s->lat || s->lon)) {
        base = LL{s->lat, s->lon};
        k = QStringLiteral("vehicle");
    } else {
        const QJsonObject last = Settings::instance().json(QStringLiteral("corv.lastHome")).toObject();
        if (!last.isEmpty()) {
            base = LL{num(last.value(QStringLiteral("lat"))), num(last.value(QStringLiteral("lng")))};
            k = QStringLiteral("last");
        }
    }
    // A vehicle without a home is taken once: a moving vehicle does not recompile every tick
    if (k == QLatin1String("vehicle") && m_compiled.home
        && (m_compiledHomeKind == QLatin1String("vehicle") || m_compiledHomeKind == QLatin1String("home"))) {
        if (kind) *kind = m_compiledHomeKind;
        return LL{m_compiled.home->lat, m_compiled.home->lng};
    }
    if (kind) *kind = k;
    return base;
}

std::optional<LL> PlanController::operatorPoint() const
{
    const QJsonObject op = params().value(QStringLiteral("operator")).toObject();
    if (!op.isEmpty()) return LL{num(op.value(QStringLiteral("lat"))), num(op.value(QStringLiteral("lng")))};
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
    if (m_status != QLatin1String("busy")) {
        m_status = QStringLiteral("busy");
        emit compiledChanged();
    }
    m_compileTimer.start(delayMs);
}

bool PlanController::ensureRouteTerrain()
{
    // The 1° elevation tiles under every base point and leg must be parsed (or known missing)
    QSet<QPair<int, int>> seen;
    auto add = [&](double lat, double lng) {
        if (std::isfinite(lat) && std::isfinite(lng)) seen.insert({int(std::floor(lat)), int(std::floor(lng))});
    };
    QList<LL> pts;
    if (const auto t = takeoffPoint()) pts.append(*t);
    for (const QJsonValue &v : segs()) pts.append(pointsOf(v.toObject()));
    const QJsonObject P = params();
    const QJsonObject op = P.value(QStringLiteral("operator")).toObject();
    const std::optional<LL> opPos = !op.isEmpty() ? std::optional<LL>(LL{num(op.value(QStringLiteral("lat"))), num(op.value(QStringLiteral("lng")))})
                                    : pts.isEmpty() ? std::nullopt : std::optional<LL>(pts.first());
    if (opPos) {
        add(opPos->lat, opPos->lng);
        if (m_showLink) {
            // The coverage raster reaches `coverageRange` around the operator
            const double cr = num(P.value(QStringLiteral("radio")).toObject().value(QStringLiteral("coverageRange")));
            const double dLat = (cr ? cr : 15) * 1000 / 111320;
            const double dLng = dLat / std::max(0.2, std::cos(opPos->lat * M_PI / 180));
            for (int a : {-1, 1}) for (int b : {-1, 1}) add(opPos->lat + a * dLat, opPos->lng + b * dLng);
        }
    }
    for (qsizetype i = 0; i < pts.size(); ++i) {
        add(pts[i].lat, pts[i].lng);
        if (i > 0) {
            const LL &a = pts[i - 1], &b = pts[i];
            const int steps = std::max(2, int(std::ceil(std::max(std::abs(b.lat - a.lat), std::abs(b.lng - a.lng)) * 2)));
            for (int s = 1; s < steps; ++s) add(a.lat + (b.lat - a.lat) * s / steps, a.lng + (b.lng - a.lng) * s / steps);
        }
    }
    bool pending = false;
    HgtStore &store = HgtStore::instance();
    for (const auto &t : seen) {
        if (store.get(t.first, t.second)) continue;
        if (store.missing(t.first, t.second)) {
            // Not on disk: fetch it (once per session unless it failed)
            HgtDownloader::instance().ensure(t.first, t.second);
            if (HgtDownloader::instance().busy(t.first, t.second)) pending = true;
        } else {
            pending = true;   // being read from disk
        }
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
    CompileContext ctx;
    ctx.terrain = terrainFn();
    ctx.home = takeoff;
    const auto s = m_hub ? m_hub->snapshot() : nullptr;
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
    static const QHash<QString, QString> t{{QStringLiteral("ok"), QStringLiteral("Route calculated")},
                                           {QStringLiteral("warn"), QStringLiteral("Route calculated with warnings")},
                                           {QStringLiteral("error"), QStringLiteral("Route has errors")},
                                           {QStringLiteral("busy"), QStringLiteral("Calculating…")},
                                           {QStringLiteral("empty"), QStringLiteral("Empty route")}};
    return t.value(m_status);
}

bool PlanController::uploadBlocked() const
{
    for (const Issue &i : m_compiled.issues) if (i.level == QLatin1String("error")) return true;
    return false;
}

QVariantList PlanController::platformIssues() const
{
    const auto s = m_hub ? m_hub->snapshot() : nullptr;
    if (!s || !s->connected) return {};
    const Platform &plat = platform(activePlatformId());
    const bool msp = s->connectionType.contains(QStringLiteral("MSP"), Qt::CaseInsensitive);
    if (plat.transport == QLatin1String("msp") && !msp)
        return {QVariantMap{{QStringLiteral("level"), QStringLiteral("warn")}, {QStringLiteral("text"), QStringLiteral("%1 missions travel over MSP — the connected link speaks MAVLink").arg(plat.label)}}};
    if (plat.transport == QLatin1String("mavlink") && msp)
        return {QVariantMap{{QStringLiteral("level"), QStringLiteral("warn")}, {QStringLiteral("text"), QStringLiteral("The connected link speaks MSP — an ArduPilot mission cannot be uploaded over it")}}};
    return {};
}

void PlanController::refreshCard()
{
    const RouteStats &st = m_compiled.stats;
    const QJsonArray sg = segs();
    int errors = 0, warns = 0;
    for (const Issue &i : m_compiled.issues) {
        if (i.level == QLatin1String("error")) ++errors;
        else if (i.level == QLatin1String("warn")) ++warns;
    }
    m_status = sg.isEmpty() ? QStringLiteral("empty") : errors ? QStringLiteral("error") : warns ? QStringLiteral("warn") : QStringLiteral("ok");
    const bool has = !sg.isEmpty();
    m_stats = {has ? fmtDist(st.lengthM) : QStringLiteral("--"), has ? fmtTime(st.durationS) : QStringLiteral("--"),
               has ? QString::number(st.waypoints) : QStringLiteral("--"),
               has && st.maxAgl ? QStringLiteral("%1 m").arg(qRound64(*st.maxAgl)) : QStringLiteral("--")};

    const QJsonObject P = params();
    const Platform &plat = platform(activePlatformId());
    const QJsonObject cam = P.value(QStringLiteral("camera")).toObject();
    QVariantList chips;
    auto chip = [&](const QString &t, const QString &cls = {}, const QString &title = {}) {
        chips.append(QVariantMap{{QStringLiteral("text"), t}, {QStringLiteral("cls"), cls}, {QStringLiteral("title"), title}});
    };
    if (plat.id != QLatin1String("ardupilot")) chip(plat.label);
    if (m_compiled.inav) chip(QStringLiteral("%1 / %2 WP").arg(m_compiled.inav->wps.size()).arg(m_inavLimit ? m_inavLimit : plat.itemLimit));
    const QString altMode = P.value(QStringLiteral("altMode")).toString();
    const QString am = altMode == QLatin1String("amsl") ? QStringLiteral("AMSL") : altMode == QLatin1String("rel") ? QStringLiteral("REL") : QStringLiteral("AGL");
    chip(QStringLiteral("%1 %2 m").arg(am, fmtNumV(P.value(QStringLiteral("defaultAlt")))));
    if (altMode == QLatin1String("agl") && truthy(P.value(QStringLiteral("terrainWaypoints"))))
        chip(QStringLiteral("terrain WPs ±%1 m").arg(fmtNumV(P.value(QStringLiteral("aglTolerance")))));
    const double spd = num(P.value(QStringLiteral("defaultSpeed")));
    chip(spd > 0 ? QStringLiteral("%1 m/s").arg(fmtNum(spd)) : QStringLiteral("vehicle speed"));
    chip(truthy(P.value(QStringLiteral("takeoff"))) ? QStringLiteral("T/O %1 m").arg(qRound64(num(P.value(QStringLiteral("takeoffAlt"))))) : QStringLiteral("no take-off"));
    const QString end = P.value(QStringLiteral("endAction")).toString();
    chip(end == QLatin1String("land") ? QStringLiteral("LAND") : end == QLatin1String("none") ? QStringLiteral("end: nothing") : QStringLiteral("RTL"));
    if (P.value(QStringLiteral("turnType")).toString() == QLatin1String("spline")) chip(QStringLiteral("spline"));
    if (!cam.value(QStringLiteral("name")).toString().isEmpty() && cam.value(QStringLiteral("preset")).toString() != QLatin1String("generic"))
        chip(cam.value(QStringLiteral("name")).toString());
    if (st.photos) chip(QStringLiteral("%1 photos").arg(st.photos));
    if (m_showLink && m_analysis) {
        const Link cls = m_analysis->worst ? m_analysis->worst->clazz : Link::Good;
        const QString text = cls == Link::Good ? QStringLiteral("LINK OK") : cls == Link::Unknown ? QStringLiteral("LINK ?")
                                                                         : QStringLiteral("LINK ") + linkStyle(cls).shortName;
        chip(text, QStringLiteral("link-") + linkName(cls), summarizeLink(*m_analysis));
    }
    m_chips = chips;

    // One line per distinct problem; the same warning on forty waypoints reads "40 seg …"
    struct Group { QString level, text; QStringList segIds; };
    QList<Group> groups;
    auto addIssue = [&](const QString &level, const QString &text, const QString &segId) {
        if (level == QLatin1String("info") && !sg.isEmpty()) return;
        for (Group &g : groups)
            if (g.level == level && g.text == text) { if (!segId.isEmpty()) g.segIds << segId; return; }
        groups.append({level, text, segId.isEmpty() ? QStringList() : QStringList{segId}});
    };
    for (const Issue &i : m_compiled.issues) addIssue(i.level, i.text, i.segId);
    for (const QVariant &v : platformIssues()) addIssue(v.toMap().value(QStringLiteral("level")).toString(), v.toMap().value(QStringLiteral("text")).toString(), {});
    if (m_showLink && m_analysis) {
        const auto &L = m_analysis->lengths;
        if (L[int(Link::None)] > 0) addIssue(QStringLiteral("warn"), QStringLiteral("Radio link lost along %1 of the route").arg(fmtDist(L[int(Link::None)])), {});
        if (L[int(Link::Marginal)] > 0) addIssue(QStringLiteral("warn"), QStringLiteral("Radio link below the safety margin along %1").arg(fmtDist(L[int(Link::Marginal)])), {});
        if (L[int(Link::Unknown)] > 0) addIssue(QStringLiteral("warn"), QStringLiteral("No elevation data for the radio link along %1").arg(fmtDist(L[int(Link::Unknown)])), {});
    }
    QVariantList issues;
    for (qsizetype gi = 0; gi < groups.size() && gi < 8; ++gi) {
        const Group &g = groups[gi];
        const QString first = g.segIds.isEmpty() ? QString() : g.segIds.first();
        const int idx = first.isEmpty() ? -1 : segIndex(first);
        const QString where = g.segIds.size() > 1 ? QStringLiteral("%1 seg").arg(g.segIds.size()) : idx >= 0 ? QString::number(idx + 1) : QString();
        issues.append(QVariantMap{{QStringLiteral("level"), g.level}, {QStringLiteral("text"), g.text},
                                  {QStringLiteral("where"), where}, {QStringLiteral("segId"), idx >= 0 ? first : QString()}});
    }
    if (groups.size() > 8)
        issues.append(QVariantMap{{QStringLiteral("level"), QStringLiteral("info")}, {QStringLiteral("text"), QStringLiteral("… %1 more").arg(groups.size() - 8)},
                                  {QStringLiteral("where"), QString()}, {QStringLiteral("segId"), QString()}});
    m_issues = issues;
    emit compiledChanged();
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
    const QJsonObject P = params();
    const QJsonObject sp = seg.value(QStringLiteral("params")).toObject();
    const QString type = seg.value(QStringLiteral("type")).toString();
    const QString id = seg.value(QStringLiteral("id")).toString();
    const bool hasSt = m_compiled.segStats.contains(id);
    const SegStats st = m_compiled.segStats.value(id);
    QStringList parts;
    if (type != QLatin1String("poi") && type != QLatin1String("landing")) parts << QStringLiteral("%1 m").arg(fmtNum(segmentAlt(seg, P)));
    if (num(sp.value(QStringLiteral("speed"))) > 0) parts << QStringLiteral("%1 m/s").arg(fmtNumV(sp.value(QStringLiteral("speed"))));
    if (type == QLatin1String("circle")) {
        parts << QStringLiteral("r %1 m").arg(fmtNumV(sp.value(QStringLiteral("radius")))) << QStringLiteral("%1×").arg(fmtNumV(sp.value(QStringLiteral("turns"))));
    } else if (type == QLatin1String("perimeter")) {
        parts << QStringLiteral("%1 vtx").arg(pointsOf(seg).size());
        if (hasSt) parts << fmtDist(st.lengthM);
    } else if (type == QLatin1String("area")) {
        if (hasSt) {
            parts << fmtArea(st.area) << QStringLiteral("%1 lanes").arg(st.lanes);
            if (st.gsd) parts << QStringLiteral("%1 cm/px").arg(*st.gsd, 0, 'f', 1);
            if (st.photos) parts << QStringLiteral("%1 img").arg(st.photos);
        }
    } else if (type == QLatin1String("corridor")) {
        parts << QStringLiteral("%1 m wide").arg(fmtNumV(sp.value(QStringLiteral("width"))));
        if (hasSt) {
            parts << QStringLiteral("%1 pass%2").arg(st.lanes).arg(st.lanes > 1 ? QStringLiteral("es") : QString());
            if (st.photos) parts << QStringLiteral("%1 img").arg(st.photos);
        }
    } else if (type == QLatin1String("poi")) {
        parts << (sp.value(QStringLiteral("mode")).toString() == QLatin1String("clear") ? QStringLiteral("ROI off")
                                                                                    : QStringLiteral("look at, %1 m").arg(fmtNumV(sp.value(QStringLiteral("alt")))));
    } else if (type == QLatin1String("landing")) {
        parts << (truthy(sp.value(QStringLiteral("vtol"))) ? QStringLiteral("VTOL") : QStringLiteral("land"));
    }
    const qsizetype na = seg.value(QStringLiteral("actions")).toArray().size();
    if (na) parts << QStringLiteral("%1 action%2").arg(na).arg(na > 1 ? QStringLiteral("s") : QString());
    return parts.join(QStringLiteral(" · "));
}

void PlanController::refreshList()
{
    QHash<QString, QList<const Issue *>> bySeg;
    for (const Issue &i : m_compiled.issues) if (!i.segId.isEmpty()) bySeg[i.segId].append(&i);
    QList<SegmentListModel::Row> rows;
    for (const QJsonValue &v : segs()) {
        const QJsonObject seg = v.toObject();
        const QString id = seg.value(QStringLiteral("id")).toString();
        const QJsonObject def = segmentDef(seg.value(QStringLiteral("type")).toString());
        SegmentListModel::Row r;
        r.id = id;
        r.type = seg.value(QStringLiteral("type")).toString();
        r.label = def.value(QStringLiteral("label")).toString();
        r.color = def.value(QStringLiteral("color")).toString();
        r.summary = segmentSummary(seg);
        r.open = id == m_selectedId;
        QStringList texts;
        bool err = false;
        for (const Issue *i : bySeg.value(id)) { texts << i->text; if (i->level == QLatin1String("error")) err = true; }
        r.worst = err ? QStringLiteral("error") : texts.isEmpty() ? QString() : QStringLiteral("warn");
        r.issuesText = texts.join(QLatin1Char('\n'));
        rows.append(r);
    }
    m_segModel.setRows(rows);
}

QVariantList PlanController::fieldsFor(const QJsonArray &fields, const QJsonObject &values,
                                       const std::function<QString(const QJsonObject &, const QString &)> &why) const
{
    QVariantList out;
    for (const QJsonValue &fv : fields) {
        const QJsonObject f = fv.toObject();
        if (!fieldVisible(f, values)) continue;
        const QString key = f.value(QStringLiteral("key")).toString();
        const QString type = f.value(QStringLiteral("type")).toString();
        const QJsonValue v = values.value(key);
        const QString reason = why ? why(f, QString()) : QString();
        QVariantMap m;
        m.insert(QStringLiteral("key"), key);
        m.insert(QStringLiteral("label"), f.value(QStringLiteral("label")).toString());
        m.insert(QStringLiteral("unit"), f.value(QStringLiteral("unit")).toString());
        m.insert(QStringLiteral("type"), type);
        m.insert(QStringLiteral("disabled"), !reason.isEmpty());
        m.insert(QStringLiteral("title"), reason.isEmpty() ? f.value(QStringLiteral("title")).toString() : reason);
        if (type == QLatin1String("select")) {
            QVariantList opts;
            int current = 0;
            const QJsonArray o = f.value(QStringLiteral("options")).toArray();
            const QString cur = v.isString() ? v.toString() : v.isDouble() ? fmtNum(v.toDouble()) : v.isBool() ? (v.toBool() ? QStringLiteral("true") : QStringLiteral("false")) : QString();
            for (qsizetype i = 0; i < o.size(); ++i) {
                const QJsonArray pair = o[i].toArray();
                const QString val = pair.at(0).toString();
                const QString optWhy = reason.isEmpty() && why ? why(f, val) : QString();
                if (val == cur) current = int(i);
                opts.append(QVariantMap{{QStringLiteral("value"), val}, {QStringLiteral("label"), pair.at(1).toString()},
                                        {QStringLiteral("disabled"), !optWhy.isEmpty()}, {QStringLiteral("title"), optWhy}});
            }
            m.insert(QStringLiteral("options"), opts);
            m.insert(QStringLiteral("current"), current);
        } else if (type == QLatin1String("check")) {
            m.insert(QStringLiteral("checked"), truthy(v));
        } else {
            const bool hasZeroLabel = f.contains(QStringLiteral("zeroLabel"));
            const bool empty = v.isNull() || v.isUndefined() || (v.isString() && v.toString().isEmpty()) || (hasZeroLabel && num(v) == 0);
            m.insert(QStringLiteral("value"), empty ? QString() : (v.isString() ? v.toString() : fmtNum(num(v))));
            const QString ph = f.value(QStringLiteral("placeholder")).toString();
            m.insert(QStringLiteral("placeholder"), ph.isEmpty() ? f.value(QStringLiteral("zeroLabel")).toString() : ph);
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
        const QString type = seg.value(QStringLiteral("type")).toString();
        const QJsonObject P = params();
        QJsonArray fields;
        for (const QJsonValue &fv : segmentDef(type).value(QStringLiteral("fields")).toArray()) {
            QJsonObject f = fv.toObject();
            // Nullable altitude: the route default as placeholder
            if (f.value(QStringLiteral("key")).toString() == QLatin1String("alt") && f.value(QStringLiteral("nullable")).toBool()) {
                f.insert(QStringLiteral("placeholder"), fmtNumV(P.value(QStringLiteral("defaultAlt"))));
                f.insert(QStringLiteral("title"), QStringLiteral("Empty = route default altitude"));
            }
            fields.append(f);
        }
        m_inspFields = fieldsFor(fields, seg.value(QStringLiteral("params")).toObject(), [type](const QJsonObject &f, const QString &opt) {
            const QString key = f.value(QStringLiteral("key")).toString();
            return unsupportedReason(QStringLiteral("segField"), opt.isEmpty() ? type + QLatin1Char('.') + key : type + QLatin1Char('.') + key + QLatin1Char('.') + opt);
        });
        // An action the flight stack cannot fly is kept, greyed out with the reason
        const QJsonArray actions = seg.value(QStringLiteral("actions")).toArray();
        const QString platLabel = platform(activePlatformId()).label;
        for (qsizetype i = 0; i < actions.size(); ++i) {
            const QJsonObject a = actions[i].toObject();
            QString at = a.value(QStringLiteral("type")).toString();
            if (!actionTypes().contains(at)) at = QStringLiteral("raw");
            const QJsonObject ad = actionDef(at);
            const QString why = unsupportedReason(QStringLiteral("action"), a.value(QStringLiteral("type")).toString());
            m_inspActions.append(QVariantMap{
                {QStringLiteral("idx"), int(i)},
                {QStringLiteral("label"), ad.value(QStringLiteral("label")).toString()},
                {QStringLiteral("summary"), why.isEmpty() ? actionSummary(a) : QStringLiteral("not flown by %1").arg(platLabel)},
                {QStringLiteral("unsupported"), !why.isEmpty()},
                {QStringLiteral("title"), why},
                {QStringLiteral("fields"), fieldsFor(ad.value(QStringLiteral("fields")).toArray(), a, [why](const QJsonObject &, const QString &) { return why; })},
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
            if (i.segId == m_selectedId) m_inspIssues.append(QVariantMap{{QStringLiteral("level"), i.level}, {QStringLiteral("text"), i.text}});
        const QString type = seg.value(QStringLiteral("type")).toString();
        if (type == QLatin1String("area") || type == QLatin1String("corridor")) {
            const QJsonObject P = params();
            const SurveyGeometry g = surveyGeometry(seg, segmentAlt(seg, P), P.value(QStringLiteral("camera")).toObject());
            if (g.hasSensor) {
                m_inspDerived << QStringLiteral("lane spacing <b>%1 m</b>").arg(g.sideDistance, 0, 'f', 1)
                              << QStringLiteral("swath <b>%1 m</b> at %2 m%3").arg(g.sensor.swath, 0, 'f', 0).arg(g.sensorDistance, 0, 'f', 0)
                                     .arg(g.sensor.limitedByRange ? QStringLiteral(" (range-limited)") : QString());
            } else {
                m_inspDerived << QStringLiteral("lane spacing <b>%1 m</b>").arg(g.sideDistance, 0, 'f', 1)
                              << QStringLiteral("trigger every <b>%1 m</b>").arg(g.triggerDistance, 0, 'f', 1);
                if (g.gsd) m_inspDerived << QStringLiteral("GSD <b>%1 cm/px</b>").arg(*g.gsd, 0, 'f', 2);
                m_inspDerived << QStringLiteral("footprint <b>%1 × %2 m</b>").arg(g.footprint.width, 0, 'f', 0).arg(g.footprint.height, 0, 'f', 0);
            }
        }
    }
    emit inspectorDetailChanged();
}

QString PlanController::inspectorCoords() const
{
    const QJsonObject seg = segment(m_selectedId);
    const QList<LL> pts = pointsOf(seg);
    if (pts.size() == 1) return QStringLiteral("%1, %2").arg(pts[0].lat, 0, 'f', 6).arg(pts[0].lng, 0, 'f', 6);
    return QStringLiteral("%1 points · drag the handles on the map, right-click a vertex to insert or remove").arg(pts.size());
}

bool PlanController::inspectorCanAct() const
{
    return segment(m_selectedId).value(QStringLiteral("type")).toString() != QLatin1String("poi");
}

QVariantList PlanController::addActionOptions() const
{
    QVariantList out{QVariantMap{{QStringLiteral("value"), QString()}, {QStringLiteral("label"), QStringLiteral("+ add…")},
                                 {QStringLiteral("disabled"), false}, {QStringLiteral("title"), QString()}}};
    for (const QString &k : actionOrder()) {
        const QString why = unsupportedReason(QStringLiteral("action"), k);
        out.append(QVariantMap{{QStringLiteral("value"), k}, {QStringLiteral("label"), actionDef(k).value(QStringLiteral("label")).toString()},
                               {QStringLiteral("disabled"), !why.isEmpty()}, {QStringLiteral("title"), why}});
    }
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
    const bool lastLanding = !a.isEmpty() && a.last().toObject().value(QStringLiteral("type")).toString() == QLatin1String("landing")
                             && seg.value(QStringLiteral("type")).toString() != QLatin1String("landing");
    if (lastLanding) a.insert(a.size() - 1, seg);
    else a.append(seg);
    setSegs(a);
    commit(label);
    const QString id = seg.value(QStringLiteral("id")).toString();
    selectSegment(id);
    emit scrollToSegment(segIndex(id));
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
    commit(QStringLiteral("Delete segment"));
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
    commit(QStringLiteral("Reorder segments"));
    refreshList();
    scheduleCompile();
}

// ── Fields ───────────────────────────────────────────────────────────────────

void PlanController::setField(const QString &scope, const QString &segId, int actionIdx, const QString &key, const QVariant &value)
{
    QJsonObject seg = segment(segId);
    QJsonObject action;
    if (scope == QLatin1String("seg") || scope == QLatin1String("action")) {
        if (seg.isEmpty()) return;
        if (scope == QLatin1String("action")) {
            const QJsonArray acts = seg.value(QStringLiteral("actions")).toArray();
            if (actionIdx < 0 || actionIdx >= acts.size()) return;
            action = acts[actionIdx].toObject();
        }
    }
    const QJsonArray fields = fieldsOf(scope, seg, action);
    QJsonObject f;
    for (const QJsonValue &fv : fields) if (fv.toObject().value(QStringLiteral("key")).toString() == key) { f = fv.toObject(); break; }
    if (f.isEmpty()) return;

    const QString type = f.value(QStringLiteral("type")).toString();
    QJsonValue v;
    if (type == QLatin1String("check")) {
        v = value.toBool();
    } else if (type == QLatin1String("select")) {
        v = value.toString();
    } else {
        const QString text = value.toString().trimmed();
        if (text.isEmpty()) {
            v = f.value(QStringLiteral("nullable")).toBool() ? QJsonValue(QJsonValue::Null) : QJsonValue(0);
        } else {
            bool ok = false;
            double d = QLocale::c().toDouble(text, &ok);
            if (!ok) d = text.toDouble(&ok);
            if (!ok || !std::isfinite(d)) return;
            if (f.contains(QStringLiteral("min"))) d = std::max(d, f.value(QStringLiteral("min")).toDouble());
            if (f.contains(QStringLiteral("max"))) d = std::min(d, f.value(QStringLiteral("max")).toDouble());
            v = d;
        }
    }

    if (scope == QLatin1String("seg")) {
        QJsonObject sp = seg.value(QStringLiteral("params")).toObject();
        if (sp.value(key) == v) return;
        sp.insert(key, v);
        seg.insert(QStringLiteral("params"), sp);
        putSegment(seg);
        commit(QStringLiteral("Edit %1").arg(lower(segmentDef(seg.value(QStringLiteral("type")).toString()).value(QStringLiteral("label")).toString())));
        // Fields with `when` clauses may have appeared or disappeared
        if (hasWhenOn(fields, key)) rebuildInspector();
        refreshList();
        refreshInspectorDetail();
        emit mapChanged();
        scheduleCompile();
        return;
    }
    if (scope == QLatin1String("action")) {
        if (action.value(key) == v) return;
        action.insert(key, v);
        QJsonArray acts = seg.value(QStringLiteral("actions")).toArray();
        acts[actionIdx] = action;
        seg.insert(QStringLiteral("actions"), acts);
        putSegment(seg);
        commit(QStringLiteral("Edit action"));
        if (actionIdx < m_inspActions.size()) {
            QVariantMap m = m_inspActions[actionIdx].toMap();
            if (!m.value(QStringLiteral("unsupported")).toBool()) {
                m.insert(QStringLiteral("summary"), actionSummary(action));
                m_inspActions[actionIdx] = m;
                emit inspectorChanged();
            }
        }
        refreshList();
        scheduleCompile();
        return;
    }

    QJsonObject P = params();
    if (scope == QLatin1String("route") || scope == QLatin1String("gimbal")) {
        if (P.value(key) == v) return;
        P.insert(key, v);
        setParams(P);
        if (scope == QLatin1String("route")) {
            commit(QStringLiteral("Route %1").arg(key));
            // Dependent fields (AGL tolerance, take-off altitude) appear or vanish
            if (hasWhenOn(fields, key)) emit formsChanged();
            if (key == QLatin1String("defaultAlt")) rebuildInspector();   // the altitude placeholder
            refreshCard();
            refreshList();
        } else {
            commit(QStringLiteral("Gimbal defaults"));
        }
        emit derivedChanged();
        scheduleCompile();
        return;
    }
    if (scope == QLatin1String("camera")) {
        QJsonObject cam = P.value(QStringLiteral("camera")).toObject();
        if (cam.value(key) == v) return;
        cam.insert(key, v);
        if (key == QLatin1String("preset")) {
            const QJsonObject preset = cameraPresets().value(v.toString()).toObject();
            if (!preset.isEmpty() && v.toString() != QLatin1String("custom"))
                for (auto it = preset.begin(); it != preset.end(); ++it) cam.insert(it.key(), it.value());
            else cam.insert(QStringLiteral("name"), QStringLiteral("Custom"));
        } else if (cam.value(QStringLiteral("preset")).toString() != QLatin1String("custom")) {
            // Hand-edited values are a custom camera
            cam.insert(QStringLiteral("preset"), QStringLiteral("custom"));
            cam.insert(QStringLiteral("name"), QStringLiteral("Custom"));
        }
        P.insert(QStringLiteral("camera"), cam);
        setParams(P);
        commit(QStringLiteral("Camera profile"));
        emit formsChanged();
        emit derivedChanged();
        refreshCard();
        refreshInspectorDetail();
        scheduleCompile();
        return;
    }
    if (scope == QLatin1String("radio")) {
        QJsonObject radio = P.value(QStringLiteral("radio")).toObject();
        if (radio.value(key) == v) return;
        radio.insert(key, v);
        static const QSet<QString> spec{QStringLiteral("freq"), QStringLiteral("txPower"), QStringLiteral("gainGround"), QStringLiteral("gainAir"), QStringLiteral("minRssi")};
        bool restructure = false;
        if (key == QLatin1String("preset")) {
            const QJsonObject preset = radioPresets().value(v.toString()).toObject();
            if (!preset.isEmpty() && v.toString() != QLatin1String("custom"))
                for (auto it = preset.begin(); it != preset.end(); ++it) radio.insert(it.key(), it.value());
            else radio.insert(QStringLiteral("name"), QStringLiteral("Custom"));
            restructure = true;
        } else if (spec.contains(key) && radio.value(QStringLiteral("preset")).toString() != QLatin1String("custom")) {
            radio.insert(QStringLiteral("preset"), QStringLiteral("custom"));
            radio.insert(QStringLiteral("name"), QStringLiteral("Custom"));
            restructure = true;
        }
        P.insert(QStringLiteral("radio"), radio);
        setParams(P);
        commit(QStringLiteral("Radio profile"));
        rememberRadio(radio);
        Settings::instance().setJson(QLatin1String(kLsRadio), radio);
        if (restructure) emit formsChanged();
        emit derivedChanged();
        scheduleLink();
    }
}

void PlanController::addAction(const QString &segId, const QString &type)
{
    QJsonObject seg = segment(segId);
    if (seg.isEmpty() || type.isEmpty() || !actionTypes().contains(type)) return;
    const QString why = unsupportedReason(QStringLiteral("action"), type);
    if (!why.isEmpty()) { setHint(why, true); return; }
    QJsonArray acts = seg.value(QStringLiteral("actions")).toArray();
    acts.append(createAction(type));
    seg.insert(QStringLiteral("actions"), acts);
    putSegment(seg);
    commit(QStringLiteral("Add action"));
    rebuildInspector();
    refreshList();
    scheduleCompile();
}

void PlanController::removeAction(const QString &segId, int idx)
{
    QJsonObject seg = segment(segId);
    QJsonArray acts = seg.value(QStringLiteral("actions")).toArray();
    if (idx < 0 || idx >= acts.size()) return;
    acts.removeAt(idx);
    seg.insert(QStringLiteral("actions"), acts);
    putSegment(seg);
    commit(QStringLiteral("Remove action"));
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
        const QJsonArray pl = fv.toObject().value(QStringLiteral("platforms")).toArray();
        if (pl.isEmpty() || pl.contains(plat)) fields.append(fv);
    }
    return fieldsFor(fields, params(), [](const QJsonObject &f, const QString &opt) {
        const QString key = f.value(QStringLiteral("key")).toString();
        return unsupportedReason(QStringLiteral("routeParam"), opt.isEmpty() ? key : key + QLatin1Char('.') + opt);
    });
}

QVariantList PlanController::cameraForm() const { return fieldsFor(cameraFields(), params().value(QStringLiteral("camera")).toObject(), {}); }
QVariantList PlanController::gimbalForm() const { return fieldsFor(gimbalFields(), params(), {}); }
QVariantList PlanController::radioForm() const { return fieldsFor(radioFields(), params().value(QStringLiteral("radio")).toObject(), {}); }

QStringList PlanController::cameraDerived() const
{
    const QJsonObject P = params();
    const QJsonObject cam = P.value(QStringLiteral("camera")).toObject();
    const Fov fov = cameraFov(cam);
    const double alt = num(P.value(QStringLiteral("defaultAlt")));
    const Footprint fp = cameraFootprint(std::max(1.0, alt ? alt : 100), cam);
    QStringList out{QStringLiteral("FOV <b>%1° × %2°</b>").arg(fov.fovH, 0, 'f', 1).arg(fov.fovV, 0, 'f', 1),
                    QStringLiteral("at %1 m: footprint <b>%2 × %3 m</b>").arg(fmtNum(alt)).arg(fp.width, 0, 'f', 0).arg(fp.height, 0, 'f', 0)};
    const double imgW = num(cam.value(QStringLiteral("imgW")));
    if (imgW > 0) out << QStringLiteral("GSD <b>%1 cm/px</b>").arg(fp.width / imgW * 100, 0, 'f', 2);
    const double yaw = num(P.value(QStringLiteral("gimbalYaw")));
    out << QStringLiteral("pitch <b>%1°</b>%2").arg(fmtNumV(P.value(QStringLiteral("gimbalPitch"))),
                                                    yaw ? QStringLiteral(" yaw <b>%1°</b>").arg(fmtNum(yaw)) : QString());
    return out;
}

QStringList PlanController::radioDerived() const
{
    const QJsonObject P = params();
    const QJsonObject R = P.value(QStringLiteral("radio")).toObject();
    auto n = [&](const char *k) { return num(R.value(QLatin1String(k))); };
    const double budget = n("txPower") + n("gainGround") + n("gainAir") - n("losses") - n("minRssi");
    const double fs0 = freeSpaceRange(R, 0), fsM = freeSpaceRange(R, n("minMargin"));
    const QString mode = P.value(QStringLiteral("altMode")).toString();
    const QString alt = fmtNumV(P.value(QStringLiteral("defaultAlt")));
    const QString altText = mode == QLatin1String("amsl") ? QStringLiteral("%1 m AMSL").arg(alt)
                          : mode == QLatin1String("rel") ? QStringLiteral("%1 m above take-off").arg(alt) : QStringLiteral("%1 m AGL").arg(alt);
    QStringList parts{QStringLiteral("budget <b>%1 dB</b>").arg(budget, 0, 'f', 0),
                      QStringLiteral("free space <b>%1</b> with margin · <b>%2</b> to sensitivity").arg(fmtDist(fsM), fmtDist(fs0)),
                      QStringLiteral("overlay at <b>%1</b>").arg(altText)};
    if (m_gs) parts << QStringLiteral("antenna <b>%1</b>").arg(m_gs->antennaMsl ? QStringLiteral("%1 m AMSL").arg(qRound64(*m_gs->antennaMsl)) : QStringLiteral("no terrain data"));
    if (m_showLink && m_analysis) {
        const auto &A = *m_analysis;
        const Link cls = A.worst ? A.worst->clazz : Link::Good;
        parts << QStringLiteral("route: <b><font color=\"%1\">%2</font></b>").arg(linkStyle(cls == Link::Unknown ? Link::None : cls).color.name(), summarizeLink(A));
        if (A.worst && A.worst->clazz != Link::Good)
            parts << QStringLiteral("worst at <b>%1</b>: %2 dBm, margin <b>%3 dB</b>%4").arg(fmtDist(A.worst->dist)).arg(A.worst->rssi, 0, 'f', 0)
                         .arg(A.worst->margin, 0, 'f', 0).arg(A.worst->loss > 0 ? QStringLiteral(", terrain −%1 dB").arg(A.worst->loss, 0, 'f', 0) : QString());
        parts << QStringLiteral("farthest <b>%1</b> from the antenna").arg(fmtDist(A.maxGsDist));
    } else if (!m_showLink) {
        parts << QStringLiteral("enable the coverage overlay to analyse the route");
    }
    return parts;
}

QString PlanController::homeDesc() const
{
    QString kind;
    const auto t = takeoffPoint(&kind);
    if (!t) return QStringLiteral("first segment (no vehicle)");
    const QString at = QStringLiteral("%1, %2").arg(t->lat, 0, 'f', 5).arg(t->lng, 0, 'f', 5);
    if (kind == QLatin1String("custom")) return QStringLiteral("%1 (custom)").arg(at);
    if (kind == QLatin1String("home")) return QStringLiteral("vehicle home %1").arg(at);
    if (kind == QLatin1String("vehicle")) return QStringLiteral("vehicle position (no home yet)");
    return QStringLiteral("last vehicle home %1 (no vehicle)").arg(at);
}

QString PlanController::operatorDesc() const
{
    const QJsonObject op = params().value(QStringLiteral("operator")).toObject();
    if (!op.isEmpty()) return QStringLiteral("%1, %2").arg(num(op.value(QStringLiteral("lat"))), 0, 'f', 5).arg(num(op.value(QStringLiteral("lng"))), 0, 'f', 5);
    return m_compiled.home ? QStringLiteral("at the take-off point") : QStringLiteral("at the take-off point (none yet)");
}

bool PlanController::operatorPlaced() const { return !params().value(QStringLiteral("operator")).toObject().isEmpty(); }

void PlanController::setHomeVehicle()
{
    const auto s = m_hub ? m_hub->snapshot() : nullptr;
    if (!s || s->isDemo() || !s->connected || s->gpsFix < 2 || (!s->lat && !s->lon)) {
        emit toast(QStringLiteral("No vehicle position available"));
        return;
    }
    setHome(s->lat, s->lon);
}

void PlanController::setHome(double lat, double lng)
{
    QJsonObject P = params();
    P.insert(QStringLiteral("home"), QJsonObject{{QStringLiteral("lat"), lat}, {QStringLiteral("lng"), lng}});
    setParams(P);
    commit(QStringLiteral("Set take-off point"));
    emit derivedChanged();
    emit mapChanged();
    scheduleCompile();
}

void PlanController::resetHome()
{
    QJsonObject P = params();
    P.insert(QStringLiteral("home"), QJsonValue::Null);
    setParams(P);
    commit(QStringLiteral("Reset take-off point"));
    emit derivedChanged();
    emit mapChanged();
    scheduleCompile();
}

void PlanController::setOperator(std::optional<LL> pos)
{
    QJsonObject P = params();
    P.insert(QStringLiteral("operator"), pos ? QJsonValue(QJsonObject{{QStringLiteral("lat"), pos->lat}, {QStringLiteral("lng"), pos->lng}})
                                             : QJsonValue(QJsonValue::Null));
    setParams(P);
    commit(pos ? QStringLiteral("Place operator") : QStringLiteral("Reset operator"));
    emit derivedChanged();
    emit mapChanged();
    scheduleLink(0);
}

void PlanController::placeOperator()
{
    emit openPopover(QString());
    setTool(QStringLiteral("operator"));
}

void PlanController::resetOperator() { setOperator(std::nullopt); }
void PlanController::openRouteSettingsFromMap() { selectSegment({}); emit openPopover(QStringLiteral("route")); }
void PlanController::openRadioFromMap() { selectSegment({}); emit openPopover(QStringLiteral("radio")); }

// ── Tools & drawing ──────────────────────────────────────────────────────────

QVariantList PlanController::tools() const
{
    QVariantList out;
    auto add = [&](const QString &id, const QString &label, const QString &key, bool aux) {
        const QString why = segmentTypes().contains(id) ? unsupportedReason(QStringLiteral("segment"), id) : QString();
        out.append(QVariantMap{{QStringLiteral("id"), id}, {QStringLiteral("aux"), aux}, {QStringLiteral("disabled"), !why.isEmpty()},
                               {QStringLiteral("title"), why.isEmpty() ? QStringLiteral("%1 (%2)").arg(label, key) : QStringLiteral("%1 — %2").arg(label, why)}});
    };
    add(QStringLiteral("select"), QStringLiteral("Select / pan"), QStringLiteral("V"), false);
    for (const QString &t : segmentOrder()) {
        const QJsonObject d = segmentDef(t);
        add(t, d.value(QStringLiteral("label")).toString(), d.value(QStringLiteral("hotkey")).toString(), false);
    }
    add(QStringLiteral("operator"), QStringLiteral("Operator position (ground antenna)"), QStringLiteral("O"), true);
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
    const QString why = !def.isEmpty() ? unsupportedReason(QStringLiteral("segment"), name) : QString();
    if (!why.isEmpty()) {
        setHint(QStringLiteral("%1: %2").arg(def.value(QStringLiteral("label")).toString(), why), true);
        return;
    }
    if (m_draft) cancelDraft();
    m_tool = name;
    emit toolChanged();
    if (name == QLatin1String("operator")) setHint(QStringLiteral("Click on the map where the operator and the ground antenna will be — Esc to cancel"));
    else setHint(def.value(QStringLiteral("hint")).toString());
}

void PlanController::keyTool(const QString &key)
{
    const QString k = key.toUpper();
    if (k == QLatin1String("V")) { setTool(QStringLiteral("select")); return; }
    if (k == QLatin1String("O")) { setTool(QStringLiteral("operator")); return; }
    for (const QString &t : segmentOrder())
        if (segmentDef(t).value(QStringLiteral("hotkey")).toString() == k) { setTool(t); return; }
}

void PlanController::escape()
{
    if (m_draft) { cancelDraft(); setTool(QStringLiteral("select")); }
    else if (m_tool != QLatin1String("select")) setTool(QStringLiteral("select"));
    else if (!m_selectedId.isEmpty()) selectSegment({});
    emit openPopover(QString());
}

void PlanController::mapClick(double lat, double lng, bool nearFirstDraftPoint)
{
    if (m_tool == QLatin1String("select")) { selectSegment({}); return; }
    if (m_tool == QLatin1String("operator")) {
        setOperator(LL{lat, lng});
        setTool(QStringLiteral("select"));
        return;
    }
    const QJsonObject def = segmentDef(m_tool);
    if (def.isEmpty()) return;
    const QJsonValue pts = def.value(QStringLiteral("points"));
    if (pts.isDouble()) {
        addSegment(createSegment(m_tool, {{lat, lng}}), QStringLiteral("Add %1").arg(lower(def.value(QStringLiteral("label")).toString())));
        if (m_tool != QLatin1String("waypoint")) setTool(QStringLiteral("select"));   // waypoints chain, the others are one-shot
        return;
    }
    // Polygon / polyline: accumulate vertices; a click on the first vertex closes a polygon
    if (pts.toString() == QLatin1String("poly") && m_draft && m_draft->points.size() >= 3 && nearFirstDraftPoint) {
        finishDraft();
        return;
    }
    draftAddPoint(lat, lng);
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
    const int need = def.value(QStringLiteral("points")).toString() == QLatin1String("poly") ? 3 : 2;
    m_draft.reset();
    if (d.points.size() < need) { emit mapChanged(); return; }
    addSegment(createSegment(d.type, d.points), QStringLiteral("Add %1").arg(lower(def.value(QStringLiteral("label")).toString())));
    setTool(QStringLiteral("select"));
}

void PlanController::cancelDraft()
{
    m_draft.reset();
    emit mapChanged();
}

void PlanController::movePoint(const QString &segId, int idx, double lat, double lng)
{
    QJsonObject seg = segment(segId);
    QJsonArray pts = seg.value(QStringLiteral("points")).toArray();
    if (idx < 0 || idx >= pts.size()) return;
    pts[idx] = QJsonObject{{QStringLiteral("lat"), lat}, {QStringLiteral("lng"), lng}};
    seg.insert(QStringLiteral("points"), pts);
    putSegment(seg);
    emit mapChanged();
}

void PlanController::insertPoint(const QString &segId, int afterIdx, double lat, double lng)
{
    QJsonObject seg = segment(segId);
    QJsonArray pts = seg.value(QStringLiteral("points")).toArray();
    pts.insert(afterIdx + 1, QJsonObject{{QStringLiteral("lat"), lat}, {QStringLiteral("lng"), lng}});
    seg.insert(QStringLiteral("points"), pts);
    putSegment(seg);
    emit mapChanged();
}

void PlanController::moveSegmentBy(const QString &segId, const QList<LL> &original, double dLat, double dLng)
{
    QJsonObject seg = segment(segId);
    QList<LL> moved;
    for (const LL &p : original) moved.append({p.lat + dLat, p.lng + dLng});
    seg.insert(QStringLiteral("points"), pointsToJson(moved));
    putSegment(seg);
    emit mapChanged();
}

void PlanController::setCircleRadius(const QString &segId, double radius)
{
    QJsonObject seg = segment(segId);
    QJsonObject sp = seg.value(QStringLiteral("params")).toObject();
    sp.insert(QStringLiteral("radius"), std::max(5.0, double(qRound64(radius))));
    seg.insert(QStringLiteral("params"), sp);
    putSegment(seg);
    emit mapChanged();
}

void PlanController::endGesture(const QString &label)
{
    commit(label);
    refreshList();
    if (label == QLatin1String("Change radius")) rebuildInspector();
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
    if (act == QLatin1String("finish")) finishDraft();
    else if (act == QLatin1String("cancel")) { cancelDraft(); setTool(QStringLiteral("select")); }
    else if (act == QLatin1String("select")) { selectSegment(segId); emit scrollToSegment(segIndex(segId)); }
    else if (act == QLatin1String("delete")) deleteSegment(segId);
    else if (act == QLatin1String("insert")) {
        const QList<LL> pts = pointsOf(seg);
        if (pointIdx < 0 || pointIdx >= pts.size()) return;
        const LL a = pts[pointIdx], b = pts[(pointIdx + 1) % pts.size()];
        insertPoint(segId, pointIdx, (a.lat + b.lat) / 2, (a.lng + b.lng) / 2);
        commit(QStringLiteral("Insert vertex"));
        selectSegment(segId);
        scheduleCompile();
    } else if (act == QLatin1String("delvtx")) {
        QJsonArray pts = seg.value(QStringLiteral("points")).toArray();
        if (pointIdx < 0 || pointIdx >= pts.size()) return;
        pts.removeAt(pointIdx);
        seg.insert(QStringLiteral("points"), pts);
        putSegment(seg);
        commit(QStringLiteral("Delete vertex"));
        selectSegment(segId);
        scheduleCompile();
    } else if (act == QLatin1String("wp")) addSegment(createSegment(QStringLiteral("waypoint"), {{lat, lng}}), QStringLiteral("Add waypoint"));
    else if (act == QLatin1String("poi")) addSegment(createSegment(QStringLiteral("poi"), {{lat, lng}}), QStringLiteral("Add POI"));
    else if (act == QLatin1String("home")) setHome(lat, lng);
    else if (act == QLatin1String("operator")) setOperator(LL{lat, lng});
    else if (act == QLatin1String("deselect")) selectSegment({});
}

// ── Menu ─────────────────────────────────────────────────────────────────────

void PlanController::menuAction(const QString &act)
{
    if (act == QLatin1String("new")) {
        auto go = [this] {
            replaceRoute(createRoute());
            m_currentId.clear();
            m_currentName.clear();
            m_selectedId.clear();
            resetHistory();
            applyHistory(true);
            emit libraryChanged();
        };
        if (!segs().isEmpty()) confirm(QStringLiteral("Start a new route? Unsaved changes will be lost."), go);
        else go();
    } else if (act == QLatin1String("clear")) {
        if (segs().isEmpty()) return;
        confirm(QStringLiteral("Remove every segment from the route?"), [this] {
            setSegs({});
            m_route.insert(QStringLiteral("name"), QJsonValue::Null);
            m_selectedId.clear();
            commit(QStringLiteral("Clear route"));
            applyHistory(true);
        });
    } else if (act == QLatin1String("invert")) {
        QJsonArray a = segs();
        std::optional<QJsonValue> landing;
        if (!a.isEmpty() && a.last().toObject().value(QStringLiteral("type")).toString() == QLatin1String("landing")) landing = a.takeAt(a.size() - 1);
        QJsonArray rev;
        for (qsizetype i = a.size() - 1; i >= 0; --i) {
            QJsonObject s = a[i].toObject();
            const QString t = s.value(QStringLiteral("type")).toString();
            if (t == QLatin1String("perimeter") || t == QLatin1String("corridor")) {
                QJsonArray p = s.value(QStringLiteral("points")).toArray(), pr;
                for (qsizetype k = p.size() - 1; k >= 0; --k) pr.append(p[k]);
                s.insert(QStringLiteral("points"), pr);
            }
            rev.append(s);
        }
        if (landing) rev.append(*landing);
        setSegs(rev);
        commit(QStringLiteral("Invert route"));
        refreshList();
        emit mapChanged();
        scheduleCompile();
    } else if (act == QLatin1String("flatten")) {
        if (m_compiled.items.isEmpty()) return;
        confirm(QStringLiteral("Replace every segment with the calculated waypoints? Areas and corridors will no longer be editable as shapes."), [this] {
            QList<RawItem> raw;
            for (const CompiledItem &it : m_compiled.items) {
                RawItem r;
                r.seq = it.seq; r.command = it.command; r.frame = it.frame; r.lat = it.lat; r.lng = it.lng; r.alt = it.alt;
                r.param1 = it.param1; r.param2 = it.param2; r.param3 = it.param3; r.param4 = it.param4; r.isHome = it.isHome;
                raw.append(r);
            }
            QJsonObject flat = itemsToRoute(raw, m_route.value(QStringLiteral("name")).toString(), false);
            QJsonObject P = params();
            const QJsonObject fp = flat.value(QStringLiteral("params")).toObject();
            P.insert(QStringLiteral("takeoff"), fp.value(QStringLiteral("takeoff")));
            P.insert(QStringLiteral("endAction"), fp.value(QStringLiteral("endAction")));
            P.insert(QStringLiteral("altMode"), QStringLiteral("agl"));
            flat.insert(QStringLiteral("params"), P);
            replaceRoute(flat);
            m_selectedId.clear();
            commit(QStringLiteral("Convert to waypoints"));
            applyHistory(true);
        });
    } else if (act == QLatin1String("fit")) {
        emit fitRequested();
    } else if (act == QLatin1String("export")) {
        if (m_compiled.items.isEmpty()) { emit toast(QStringLiteral("Nothing to export — the route is empty.")); return; }
        emit saveFileRequested(exportName());
    }
}

QString PlanController::exportName() const
{
    QString n = routeName();
    if (n.isEmpty()) n = QStringLiteral("route");
    static const QRegularExpression bad(QStringLiteral("[^\\w.-]+"));
    n.replace(bad, QStringLiteral("_"));
    return n + QStringLiteral(".waypoints");
}

void PlanController::exportFile(const QUrl &url)
{
    QFile f(url.toLocalFile());
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        emit toast(QStringLiteral("Export failed: %1").arg(f.errorString()));
        return;
    }
    f.write(toWpl(uploadItems(m_compiled.items)).toUtf8());
    LOG_I("plan") << "exported" << m_compiled.items.size() << "items to" << f.fileName();
}

void PlanController::importFile(const QUrl &url)
{
    QFile f(url.toLocalFile());
    if (!f.open(QIODevice::ReadOnly)) {
        emit toast(QStringLiteral("Import failed: %1").arg(f.errorString()));
        return;
    }
    const QString text = QString::fromUtf8(f.readAll());
    const QString base = QFileInfo(f.fileName()).completeBaseName();
    QJsonObject route;
    if (text.trimmed().startsWith(QLatin1Char('{'))) {
        QJsonParseError pe;
        const QJsonObject data = QJsonDocument::fromJson(text.toUtf8(), &pe).object();
        if (data.isEmpty()) { emit toast(QStringLiteral("Import failed: %1").arg(pe.errorString())); return; }
        const QJsonObject r = data.value(QStringLiteral("route")).toObject();
        route = r.value(QStringLiteral("segments")).isArray() ? r
                                                               : itemsToRoute(itemsFromJson(data.value(QStringLiteral("items")).toArray()), data.value(QStringLiteral("name")).toString(), false);
    } else {
        QList<RawItem> items;
        QString err;
        if (!parseWpl(text, items, &err)) { emit toast(QStringLiteral("Import failed: %1").arg(err)); return; }
        route = itemsToRoute(items, base, true);
    }
    auto go = [this, route, base]() mutable {
        if (route.value(QStringLiteral("name")).toString().isEmpty()) route.insert(QStringLiteral("name"), base);
        replaceRoute(route);
        m_currentId.clear();
        m_currentName.clear();
        m_selectedId.clear();
        resetHistory();
        applyHistory(true);
        emit libraryChanged();
        QTimer::singleShot(400, this, [this] { emit fitRequested(); });
    };
    if (!segs().isEmpty()) confirm(QStringLiteral("Replace the current route with the imported one?"), go);
    else go();
}

// ── Upload / read ────────────────────────────────────────────────────────────

void PlanController::flashButton(QString &which, const QString &text, int ms)
{
    which = text;
    emit buttonsChanged();
    QString *target = &which;
    const QString rest = &which == &m_uploadText ? QStringLiteral("UPLOAD") : &which == &m_readText ? QStringLiteral("READ") : QStringLiteral("SAVE");
    QTimer::singleShot(ms, this, [this, target, rest] {
        *target = rest;
        m_transferBusy = false;
        emit buttonsChanged();
    });
}

void PlanController::upload()
{
    const auto s = m_hub ? m_hub->snapshot() : nullptr;
    if (usesMspMissions()) {
        uploadInav();
        return;
    }
    if (!s || !s->connected) { emit toast(QStringLiteral("Not connected")); return; }
    if (segs().isEmpty()) { emit toast(QStringLiteral("The route is empty")); return; }
    QStringList errors;
    for (const Issue &i : m_compiled.issues) if (i.level == QLatin1String("error")) errors << QStringLiteral("• ") + i.text;
    if (!errors.isEmpty()) {
        confirm(QStringLiteral("The route has %1 error%2:\n%3\n\nUpload anyway?").arg(errors.size()).arg(errors.size() > 1 ? QStringLiteral("s") : QString(), errors.join(QLatin1Char('\n'))),
                [this] { doUpload(); });
        return;
    }
    doUpload();
}

void PlanController::doUpload()
{
    if (m_transferBusy) return;
    m_transferBusy = true;
    m_uploadText = QStringLiteral("UPLOADING…");
    emit buttonsChanged();
    QList<MissionItem> items;
    for (const RawItem &r : uploadItems(m_compiled.items)) {
        MissionItem it;
        it.command = r.command;
        it.frame = r.frame;
        it.param1 = float(r.param1); it.param2 = float(r.param2); it.param3 = float(r.param3); it.param4 = float(r.param4);
        it.lat = r.lat; it.lng = r.lng; it.alt = float(r.alt);
        items.append(it);
    }
    const int n = int(items.size());
    QPointer<PlanController> self(this);
    m_cmd->uploadMission(items, [self, n](bool ok, const QString &text) {
        if (!self) return;
        if (ok) self->flashButton(self->m_uploadText, QStringLiteral("DONE (%1)").arg(n), 2000);
        else {
            emit self->toast(QStringLiteral("Upload failed: %1").arg(text));
            self->flashButton(self->m_uploadText, QStringLiteral("UPLOAD"), 2000);
        }
    });
}

void PlanController::read()
{
    const auto s = m_hub ? m_hub->snapshot() : nullptr;
    if (!s || !s->connected) { emit toast(QStringLiteral("Not connected")); return; }
    if (usesMspMissions()) {
        if (QString problem = inavLinkProblem(); !problem.isEmpty()) { emit toast(problem); return; }
        if (!segs().isEmpty()) confirm(QStringLiteral("Replace the current route with the mission stored on the vehicle?"), [this] { doReadInav(); });
        else doReadInav();
        return;
    }
    if (!segs().isEmpty()) confirm(QStringLiteral("Replace the current route with the mission stored on the vehicle?"), [this] { doRead(); });
    else doRead();
}

void PlanController::doRead()
{
    if (m_transferBusy) return;
    m_transferBusy = true;
    m_readText = QStringLiteral("READING…");
    emit buttonsChanged();
    QPointer<PlanController> self(this);
    m_cmd->downloadMission(
        [self](int done, int total) {
            if (!self) return;
            self->m_readText = QStringLiteral("READING %1/%2").arg(done).arg(total);
            emit self->buttonsChanged();
        },
        [self](bool ok, const QString &error, const QList<MissionItem> &items) {
            if (!self) return;
            if (!ok) {
                emit self->toast(QStringLiteral("Read failed: %1").arg(error));
                self->flashButton(self->m_readText, QStringLiteral("READ"), 2000);
                return;
            }
            QList<RawItem> raw;
            for (qsizetype i = 0; i < items.size(); ++i) {
                const MissionItem &m = items[i];
                RawItem r;
                r.seq = int(i);
                r.command = m.command; r.frame = m.frame;
                r.lat = m.lat; r.lng = m.lng; r.alt = m.alt;
                r.param1 = m.param1; r.param2 = m.param2; r.param3 = m.param3; r.param4 = m.param4;
                raw.append(r);
            }
            const QJsonObject route = itemsToRoute(raw, QStringLiteral("From vehicle"), true);
            // ArduPilot always keeps its home at seq 0: with nothing after it there is no mission
            if (route.value(QStringLiteral("segments")).toArray().isEmpty()) {
                emit self->toast(QStringLiteral("The vehicle has no mission stored (only its home position)."));
                self->flashButton(self->m_readText, QStringLiteral("READ"), 10);
                return;
            }
            self->replaceRoute(route);
            self->m_currentId.clear();
            self->m_currentName.clear();
            self->m_selectedId.clear();
            self->resetHistory();
            self->applyHistory(true);
            emit self->libraryChanged();
            QTimer::singleShot(400, self, [self] { if (self) emit self->fitRequested(); });
            self->flashButton(self->m_readText, QStringLiteral("READ %1").arg(items.size()), 2000);
        });
}

// ── Upload / read, INAV (InavTransfer.js) ────────────────────────────────────
// The waypoint records come from the compiler (RouteCompiler -> INAV); the MSP
// session on the link thread owns the transfer, one record in flight at a time.

QString PlanController::inavLinkProblem() const
{
    const auto s = m_hub ? m_hub->snapshot() : nullptr;
    if (!s || !s->connected) return QStringLiteral("Not connected");
    if (!s->connectionType.startsWith(QLatin1String("msp")))
        return QStringLiteral("%1 missions travel over MSP — connect the aircraft as MSP Serial or MSP TCP (SETUP → CONNECTION)").arg(platform(activePlatformId()).label);
    return {};
}

void PlanController::uploadInav()
{
    if (QString problem = inavLinkProblem(); !problem.isEmpty()) { emit toast(problem); return; }
    if (segs().isEmpty()) { emit toast(QStringLiteral("The route is empty")); return; }
    if (!m_compiled.inav || m_compiled.inav->wps.isEmpty()) { emit toast(QStringLiteral("Nothing in this route can be uploaded to INAV")); return; }
    QStringList errors;
    for (const Issue &i : m_compiled.issues) if (i.level == QLatin1String("error")) errors << QStringLiteral("• ") + i.text;
    if (!errors.isEmpty()) {
        confirm(QStringLiteral("The route has %1 error%2:\n%3\n\nUpload anyway?").arg(errors.size()).arg(errors.size() > 1 ? QStringLiteral("s") : QString(), errors.join(QLatin1Char('\n'))),
                [this] { doUploadInav(); });
        return;
    }
    doUploadInav();
}

void PlanController::doUploadInav()
{
    LinkManager *link = GcsController::s_instance ? GcsController::s_instance->link() : nullptr;
    if (!link || m_transferBusy) return;
    m_transferBusy = true;
    m_uploadText = QStringLiteral("UPLOADING…");
    emit buttonsChanged();
    QList<MspWaypoint> wps;
    for (const InavWp &w : m_compiled.inav->wps) {
        MspWaypoint m;
        m.index = w.index; m.action = w.action; m.lat = w.lat; m.lon = w.lon; m.alt = w.alt;
        m.p1 = w.p1; m.p2 = w.p2; m.p3 = w.p3; m.flag = w.flag;
        wps << m;
    }
    const bool save = m_route.value(QStringLiteral("params")).toObject().value(QStringLiteral("inavSaveEeprom")).toBool();
    auto progress = std::make_shared<QMetaObject::Connection>();
    auto done = std::make_shared<QMetaObject::Connection>();
    *progress = connect(link, &LinkManager::mspMissionProgress, this, [this](const QString &phase, int d, int total) {
        m_uploadText = phase == QLatin1String("upload") ? QStringLiteral("UPLOADING %1/%2").arg(d).arg(total) : QStringLiteral("UPLOADING…");
        emit buttonsChanged();
    }, Qt::QueuedConnection);
    *done = connect(link, &LinkManager::mspMissionDone, this, [this, progress, done](const QString &op, bool ok, const QString &error, const QVariantMap &r) {
        if (op != QLatin1String("upload")) return;
        disconnect(*progress);
        disconnect(*done);
        const int maxWp = r.value(QStringLiteral("maxWaypoints")).toInt();
        // The board just said how many waypoints it holds: believe it
        if (maxWp > 0 && maxWp != m_inavLimit) {
            m_inavLimit = maxWp;
            scheduleCompile(0);
        }
        if (ok) {
            flashButton(m_uploadText, QStringLiteral("DONE (%1)%2").arg(r.value(QStringLiteral("count")).toInt())
                                          .arg(r.value(QStringLiteral("saved")).toBool() ? QStringLiteral(" · SAVED") : QString()), 2000);
        } else {
            emit toast(QStringLiteral("Upload failed: %1").arg(error));
            flashButton(m_uploadText, QStringLiteral("UPLOAD"), 2000);
        }
    }, Qt::QueuedConnection);
    link->mspMissionUpload(wps, save);
}

void PlanController::doReadInav()
{
    LinkManager *link = GcsController::s_instance ? GcsController::s_instance->link() : nullptr;
    if (!link || m_transferBusy) return;
    m_transferBusy = true;
    m_readText = QStringLiteral("READING…");
    emit buttonsChanged();
    auto progress = std::make_shared<QMetaObject::Connection>();
    auto done = std::make_shared<QMetaObject::Connection>();
    *progress = connect(link, &LinkManager::mspMissionProgress, this, [this](const QString &, int d, int total) {
        m_readText = QStringLiteral("READING %1/%2").arg(d).arg(total);
        emit buttonsChanged();
    }, Qt::QueuedConnection);
    *done = connect(link, &LinkManager::mspMissionDone, this, [this, progress, done](const QString &op, bool ok, const QString &error, const QVariantMap &r) {
        if (op != QLatin1String("download")) return;
        disconnect(*progress);
        disconnect(*done);
        if (!ok) {
            emit toast(QStringLiteral("Read failed: %1").arg(error));
            flashButton(m_readText, QStringLiteral("READ"), 2000);
            return;
        }
        if (const int maxWp = r.value(QStringLiteral("maxWaypoints")).toInt(); maxWp > 0) m_inavLimit = maxWp;
        const QVariantList list = r.value(QStringLiteral("wps")).toList();
        if (list.isEmpty()) {
            emit toast(QStringLiteral("The flight controller has no mission loaded"));
            flashButton(m_readText, QStringLiteral("READ"), 10);
            return;
        }
        QList<InavWp> wps;
        for (const QVariant &v : list) {
            const QVariantMap m = v.toMap();
            InavWp w;
            w.index = m.value(QStringLiteral("index")).toInt(); w.action = m.value(QStringLiteral("action")).toInt();
            w.lat = m.value(QStringLiteral("lat")).toInt(); w.lon = m.value(QStringLiteral("lon")).toInt(); w.alt = m.value(QStringLiteral("alt")).toInt();
            w.p1 = m.value(QStringLiteral("p1")).toInt(); w.p2 = m.value(QStringLiteral("p2")).toInt(); w.p3 = m.value(QStringLiteral("p3")).toInt();
            w.flag = m.value(QStringLiteral("flag")).toInt();
            wps << w;
        }
        replaceRoute(itemsToRoute(inavToItems(wps), QStringLiteral("From INAV"), true));
        m_currentId.clear();
        m_currentName.clear();
        m_selectedId.clear();
        resetHistory();
        applyHistory(true);
        emit libraryChanged();
        scheduleCompile(0);
        QTimer::singleShot(400, this, [this] { emit fitRequested(); });
        flashButton(m_readText, QStringLiteral("READ %1").arg(wps.size()), 2000);
    }, Qt::QueuedConnection);
    link->mspMissionDownload();
}

// ── Library ──────────────────────────────────────────────────────────────────

void PlanController::save() { saveCurrent(false); }
void PlanController::saveAsNew() { saveCurrent(true); }

void PlanController::saveCurrent(bool forceNew)
{
    if (segs().isEmpty()) { emit toast(QStringLiteral("Nothing to save — the route is empty.")); return; }
    const QString id = forceNew ? QString() : m_currentId;
    if (!id.isEmpty()) { writeMission(id, m_currentName); return; }
    const QString suggestion = !m_currentName.isEmpty() ? m_currentName
                                                        : QStringLiteral("Mission %1").arg(QDateTime::currentDateTimeUtc().toString(QStringLiteral("yyyy-MM-dd HH:mm")));
    prompt(QStringLiteral("Mission name:"), suggestion, [this](const QString &name) { writeMission({}, name); });
}

void PlanController::writeMission(const QString &id, const QString &name)
{
    QString err;
    const auto s = m_hub ? m_hub->snapshot() : nullptr;
    const MissionStore::Saved saved = MissionStore::save(id, name, QString(), serializeItems(m_compiled.items), m_route, s ? s->vehicleType : 0, &err);
    if (saved.id.isEmpty()) { emit toast(QStringLiteral("Save failed: %1").arg(err)); return; }
    m_currentId = saved.id;
    m_currentName = saved.name;
    m_route.insert(QStringLiteral("name"), saved.name);
    m_baseline.insert(QStringLiteral("name"), saved.name);
    emit routeChanged();
    flashButton(m_saveText, QStringLiteral("SAVED"), 1500);
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
        return t.isValid() ? t.toLocalTime().toString(QStringLiteral("dd/MM/yyyy, HH:mm")) : QString();
    };
    for (const MissionEntry &e : idx.missions)
        m_libMissions.append(QVariantMap{{QStringLiteral("id"), e.id}, {QStringLiteral("name"), e.name}, {QStringLiteral("notes"), e.notes},
                                         {QStringLiteral("meta"), QStringLiteral("%1 WP · %2 · %3").arg(e.waypoints)
                                                                      .arg(e.distanceM ? fmtDist(e.distanceM) : QStringLiteral("—"), when(e.modified))}});
    auto size = [](qint64 b) {
        return b >= 1048576 ? QStringLiteral("%1 MB").arg(b / 1048576.0, 0, 'f', 1) : b >= 1024 ? QStringLiteral("%1 kB").arg(qRound64(b / 1024.0)) : QStringLiteral("%1 B").arg(b);
    };
    for (qsizetype i = 0; i < idx.logs.size() && i < 50; ++i)
        m_libLogs.append(QVariantMap{{QStringLiteral("name"), idx.logs[i].name}, {QStringLiteral("meta"), QStringLiteral("%1 · %2").arg(size(idx.logs[i].size), when(idx.logs[i].modified))}});
    if (idx.logs.size() > 50)
        m_libLogs.append(QVariantMap{{QStringLiteral("name"), QString()}, {QStringLiteral("meta"), QStringLiteral("… %1 more in the logs folder").arg(idx.logs.size() - 50)}});
    emit libraryChanged();
}

void PlanController::loadMission(const QString &id)
{
    QString err;
    const QJsonObject data = MissionStore::load(id, &err);
    if (data.isEmpty()) { emit toast(QStringLiteral("Mission library error: %1").arg(err)); return; }
    // Files written before the route editor only carry compiled items: rebuild an editable route
    const QJsonObject r = data.value(QStringLiteral("route")).toObject();
    QJsonObject route = r.value(QStringLiteral("segments")).isArray()
                            ? r
                            : itemsToRoute(itemsFromJson(data.value(QStringLiteral("items")).toArray()), data.value(QStringLiteral("name")).toString(), false);
    const QString name = data.value(QStringLiteral("name")).toString(id);
    route.insert(QStringLiteral("name"), name.isEmpty() ? id : name);
    replaceRoute(route);
    m_currentId = id;
    m_currentName = name.isEmpty() ? id : name;
    m_selectedId.clear();
    resetHistory();
    applyHistory(true);
    emit libraryChanged();
    emit libraryOpenChanged(false);
    QTimer::singleShot(300, this, [this] { emit fitRequested(); });
}

void PlanController::libraryAction(const QString &act, const QString &id)
{
    QString name = id;
    QString notes;
    for (const QVariant &v : m_libMissions) {
        const QVariantMap m = v.toMap();
        if (m.value(QStringLiteral("id")).toString() == id) { name = m.value(QStringLiteral("name")).toString(); notes = m.value(QStringLiteral("notes")).toString(); }
    }
    if (act == QLatin1String("load")) {
        if (!segs().isEmpty()) confirm(QStringLiteral("Replace the current plan? Unsaved changes will be lost."), [this, id] { loadMission(id); });
        else loadMission(id);
    } else if (act == QLatin1String("overwrite")) {
        if (segs().isEmpty()) { emit toast(QStringLiteral("The current plan is empty — nothing to write over it with.")); return; }
        confirm(QStringLiteral("Overwrite \"%1\" with the current plan (%2 segments, %3 items)?").arg(name).arg(segs().size()).arg(m_compiled.items.size()),
                [this, id, name, notes] {
                    QString err;
                    const auto s = m_hub ? m_hub->snapshot() : nullptr;
                    const MissionStore::Saved saved = MissionStore::save(id, name, notes, serializeItems(m_compiled.items), m_route, s ? s->vehicleType : 0, &err);
                    if (saved.id.isEmpty()) { emit toast(QStringLiteral("Mission library error: %1").arg(err)); return; }
                    m_currentId = id;
                    m_currentName = name;
                    refreshLibrary();
                });
    } else if (act == QLatin1String("rename")) {
        prompt(QStringLiteral("New mission name:"), name, [this, id, name](const QString &newName) {
            if (newName == name) return;
            QString err;
            if (!MissionStore::rename(id, newName, &err)) { emit toast(QStringLiteral("Mission library error: %1").arg(err)); return; }
            if (m_currentId == id) { m_currentName = newName; emit routeChanged(); }
            refreshLibrary();
        });
    } else if (act == QLatin1String("delete")) {
        confirm(QStringLiteral("Delete \"%1\" permanently?").arg(name), [this, id] {
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
    if (m_tool != QLatin1String("select") && m_tool != QLatin1String("operator") && !unsupportedReason(QStringLiteral("segment"), m_tool).isEmpty())
        setTool(QStringLiteral("select"));
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
    return QStringLiteral("%1 · %2 baud").arg(connectionLabel(p.connType)).arg(p.connBaud);
}

bool PlanController::missionsSupported() const { return mission::missionsSupported(activePlatformId()); }

// ── Radio link ───────────────────────────────────────────────────────────────

void PlanController::setShowLink(bool v)
{
    m_showLink = v;
    Settings::instance().setValue(QLatin1String(kLsLink), v ? QStringLiteral("1") : QStringLiteral("0"));
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
    const QJsonObject P = params();
    const QJsonObject R = P.value(QStringLiteral("radio")).toObject();
    const auto pos = operatorPoint();
    const TerrainFn terrain = terrainFn();
    m_gs = pos ? std::optional<GroundStation>(mission::groundStation(R, *pos, terrain)) : std::nullopt;
    m_analysis.reset();
    m_worstEval.reset();

    if (!m_showLink || !m_gs) {
        emit derivedChanged();
        refreshCard();
        emit mapChanged();
        return;
    }
    if (!m_gs->antennaMsl) {
        // No tile under the antenna yet: the load (or download) recompiles, then analyses again
        HgtStore::instance().get(int(std::floor(pos->lat)), int(std::floor(pos->lng)));
        if (HgtStore::instance().missing(int(std::floor(pos->lat)), int(std::floor(pos->lng))))
            HgtDownloader::instance().ensure(int(std::floor(pos->lat)), int(std::floor(pos->lng)));
    }
    if (m_compiled.navPath.size() >= 2) m_analysis = analyzeRoute(R, *m_gs, m_compiled.navPath, terrain, 40);
    if (m_analysis && m_analysis->worst) {
        const LinkSample &w = *m_analysis->worst;
        m_worstEval = evaluateLink(R, *m_gs, w.lat, w.lng, w.altMsl, terrain, 30, 600, true);
    }
    emit derivedChanged();
    refreshCard();
    emit mapChanged();

    // Coverage raster: only when something it depends on changed
    const double homeElevVal = m_compiled.home ? m_compiled.home->elev : (m_gs->elev ? *m_gs->elev : 0);
    const QString key = QStringLiteral("%1|%2|%3|%4|%5|%6|%7|%8|%9")
                            .arg(QString::fromUtf8(QJsonDocument(R).toJson(QJsonDocument::Compact)))
                            .arg(m_gs->lat, 0, 'f', 7).arg(m_gs->lng, 0, 'f', 7)
                            .arg(m_gs->antennaMsl ? QString::number(*m_gs->antennaMsl) : QStringLiteral("null"))
                            .arg(P.value(QStringLiteral("altMode")).toString())
                            .arg(num(P.value(QStringLiteral("defaultAlt"))))
                            .arg(homeElevVal).arg(0).arg(0);
    if (key == m_coverageKey && m_coverage) return;
    if (!m_gs->antennaMsl) { m_coverage.reset(); m_coverageKey.clear(); emit mapChanged(); return; }

    const double alt = num(P.value(QStringLiteral("defaultAlt")));
    const QString mode = P.value(QStringLiteral("altMode")).toString();
    std::function<double(double)> altAt;
    if (mode == QLatin1String("amsl")) altAt = [alt](double) { return alt; };
    else if (mode == QLatin1String("rel")) altAt = [alt, homeElevVal](double) { return homeElevVal + alt; };
    else altAt = [alt](double t) { return t + alt; };
    const double cr = num(R.value(QStringLiteral("coverageRange")));
    const double range = (cr ? cr : 15) * 1000;
    auto cancel = std::make_shared<std::atomic<bool>>(false);
    m_coverageCancel = cancel;
    m_linkBusy = true;
    emit linkChanged();
    const GroundStation gs = *m_gs;
    auto *w = new QFutureWatcher<std::shared_ptr<const Coverage>>(this);
    connect(w, &QFutureWatcher<std::shared_ptr<const Coverage>>::finished, this, [this, w, cancel, key] {
        w->deleteLater();
        if (cancel->load()) return;
        const auto cov = w->result();
        if (m_coverageCancel == cancel) {
            m_linkBusy = false;
            emit linkChanged();
        }
        if (!cov) return;
        m_coverage = cov;
        m_coverageKey = key;
        emit mapChanged();
    });
    w->setFuture(QtConcurrent::run([R, gs, altAt, terrain, range, cancel]() -> std::shared_ptr<const Coverage> {
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
    std::vector<MissionPoint3D> pts;
    std::vector<QString> segOf;
    int index = 0;
    for (const CompiledItem &it : m_compiled.items) {
        const int i = index++;
        if (!nav.contains(it.command) || (!it.lat && !it.lng)) continue;
        MissionPoint3D p;
        p.lat = it.lat;
        p.lon = it.lng;
        p.altMsl = it.altMsl;
        p.ground = it.terrain;
        p.seq = it.seq;
        p.derived = it.derived;
        if (it.isHome || (i == 0 && it.frame == 0 && it.command == 16)) p.kind = QStringLiteral("home");
        else if (it.command == 22 || it.command == 84) p.kind = QStringLiteral("takeoff");
        else if (it.command == 21 || it.command == 85 || it.landing) p.kind = QStringLiteral("land");
        else if (it.command == 17 || it.command == 18 || it.command == 19 || it.command == 31) p.kind = QStringLiteral("loiter");
        else p.kind = QStringLiteral("wp");
        if (p.kind == QLatin1String("loiter"))
            p.loiterRadius = it.loiter ? it.loiter->r : std::abs(it.command == 31 ? it.param2 : it.param3);
        p.alt = it.alt;
        p.hold = (it.command == 16 || it.command == 19) ? it.param1 : 0;
        p.turns = it.command == 18 ? int(it.param1) : 0;
        if (!it.segId.isEmpty()) p.segType = it.segType;
        segOf.push_back(it.segId);
        pts.push_back(p);
    }
    // Inside a pattern only its ends are marked
    for (std::size_t i = 0; i < pts.size(); ++i) {
        if (segOf[i].isEmpty() || pts[i].derived) continue;
        auto same = [&](std::size_t j) { return segOf[j] == segOf[i] && !pts[j].derived; };
        bool before = false, after = false;
        for (std::size_t j = 0; j < i && !before; ++j) before = same(j);
        for (std::size_t j = i + 1; j < pts.size() && !after; ++j) after = same(j);
        pts[i].segPos = !before ? 1 : !after ? 3 : 2;
    }
    SceneItem::setMission(std::move(pts));
}

} // namespace corv
