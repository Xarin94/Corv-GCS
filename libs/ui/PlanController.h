// PlanController.h - the Flight Plan page (port of js/ui/FlightPlanController.js,
// MissionHistory.js and MissionLibrary.js), the "Plan" QML singleton.
//
// Route editor: pick a tool from the palette on the map, draw the figure, tune
// it in the inspector under the segment's card. The route is recalculated a
// moment after every edit: the calculated waypoints, the route card statistics
// and the warnings that gate UPLOAD come from RouteCompiler; this class edits
// the document and feeds the page. PlanMapItem and ProfileItem draw from it.
//
// The document is JSON (RouteModel.h). Undo/redo keeps whole snapshots: a
// route is a few hundred points at most, cheaper than an inverse-command log
// and impossible to drift out of sync with the document.
#pragma once

#include "mission/RadioLink.h"
#include "mission/RouteCompiler.h"

#include <QAbstractListModel>
#include <QFutureWatcher>
#include <QJsonObject>
#include <QObject>
#include <QQmlEngine>
#include <QTimer>
#include <QVariantList>
#include <atomic>
#include <functional>
#include <memory>

namespace corv {

class CommandSender;
class VehicleHub;

/// The segment cards: one row per segment, kept across edits so a field being typed in keeps its focus.
class SegmentListModel : public QAbstractListModel
{
    Q_OBJECT
public:
    enum Roles { SegIdRole = Qt::UserRole + 1, IndexRole, TypeRole, LabelRole, ColorRole, SummaryRole, WorstRole, IssuesTextRole, OpenRole };
    struct Row { QString id, type, label, color, summary, worst, issuesText; bool open = false; };
    using QAbstractListModel::QAbstractListModel;
    int rowCount(const QModelIndex &parent = {}) const override { return parent.isValid() ? 0 : int(m_rows.size()); }
    QVariant data(const QModelIndex &index, int role) const override;
    QHash<int, QByteArray> roleNames() const override;
    void setRows(const QList<Row> &rows);

private:
    QList<Row> m_rows;
};

class PlanController : public QObject
{
    Q_OBJECT
    QML_NAMED_ELEMENT(Plan)
    QML_SINGLETON

    // Route card
    Q_PROPERTY(QString routeName READ routeName NOTIFY routeChanged)
    Q_PROPERTY(QString status READ status NOTIFY compiledChanged)
    Q_PROPERTY(QString statusTitle READ statusTitle NOTIFY compiledChanged)
    Q_PROPERTY(QStringList stats READ stats NOTIFY compiledChanged)
    Q_PROPERTY(QVariantList chips READ chips NOTIFY compiledChanged)
    Q_PROPERTY(QVariantList issues READ issues NOTIFY compiledChanged)
    Q_PROPERTY(bool uploadBlocked READ uploadBlocked NOTIFY compiledChanged)
    Q_PROPERTY(QString uploadText READ uploadText NOTIFY buttonsChanged)
    Q_PROPERTY(QString readText READ readText NOTIFY buttonsChanged)
    Q_PROPERTY(QString saveText READ saveText NOTIFY buttonsChanged)
    Q_PROPERTY(bool transferBusy READ transferBusy NOTIFY buttonsChanged)
    // Segment list and inspector
    Q_PROPERTY(QObject *segmentModel READ segmentModel CONSTANT)
    Q_PROPERTY(int segmentCount READ segmentCount NOTIFY routeChanged)
    Q_PROPERTY(QString selectedId READ selectedId NOTIFY selectionChanged)
    Q_PROPERTY(QVariantList inspectorFields READ inspectorFields NOTIFY inspectorChanged)
    Q_PROPERTY(QVariantList inspectorActions READ inspectorActions NOTIFY inspectorChanged)
    Q_PROPERTY(QVariantList addActionOptions READ addActionOptions NOTIFY inspectorChanged)
    Q_PROPERTY(bool inspectorCanAct READ inspectorCanAct NOTIFY inspectorChanged)
    Q_PROPERTY(QStringList inspectorDerived READ inspectorDerived NOTIFY inspectorDetailChanged)
    Q_PROPERTY(QVariantList inspectorIssues READ inspectorIssues NOTIFY inspectorDetailChanged)
    Q_PROPERTY(QString inspectorCoords READ inspectorCoords NOTIFY inspectorDetailChanged)
    Q_PROPERTY(bool canUndo READ canUndo NOTIFY historyChanged)
    Q_PROPERTY(bool canRedo READ canRedo NOTIFY historyChanged)
    Q_PROPERTY(QString undoTip READ undoTip NOTIFY historyChanged)
    Q_PROPERTY(QString redoTip READ redoTip NOTIFY historyChanged)
    // Map chrome
    Q_PROPERTY(QString tool READ tool NOTIFY toolChanged)
    Q_PROPERTY(bool drafting READ drafting NOTIFY mapChanged)
    Q_PROPERTY(QString hint READ hint NOTIFY hintChanged)
    Q_PROPERTY(QVariantList tools READ tools NOTIFY platformChanged)
    Q_PROPERTY(bool showSat READ showSat WRITE setShowSat NOTIFY layersChanged)
    Q_PROPERTY(bool showCalc READ showCalc WRITE setShowCalc NOTIFY layersChanged)
    Q_PROPERTY(bool showCam READ showCam WRITE setShowCam NOTIFY layersChanged)
    Q_PROPERTY(bool showLink READ showLink WRITE setShowLink NOTIFY layersChanged)
    Q_PROPERTY(bool linkBusy READ linkBusy NOTIFY linkChanged)
    // Popovers
    Q_PROPERTY(QVariantList routeForm READ routeForm NOTIFY formsChanged)
    Q_PROPERTY(QVariantList cameraForm READ cameraForm NOTIFY formsChanged)
    Q_PROPERTY(QVariantList gimbalForm READ gimbalForm NOTIFY formsChanged)
    Q_PROPERTY(QVariantList radioForm READ radioForm NOTIFY formsChanged)
    Q_PROPERTY(QStringList cameraDerived READ cameraDerived NOTIFY derivedChanged)
    Q_PROPERTY(QStringList radioDerived READ radioDerived NOTIFY derivedChanged)
    Q_PROPERTY(QString homeDesc READ homeDesc NOTIFY derivedChanged)
    Q_PROPERTY(QString operatorDesc READ operatorDesc NOTIFY derivedChanged)
    Q_PROPERTY(bool operatorPlaced READ operatorPlaced NOTIFY derivedChanged)
    // Flight stack (SYS CONFIG)
    Q_PROPERTY(QString platformId READ platformId WRITE setPlatformId NOTIFY platformChanged)
    Q_PROPERTY(QString platformHint READ platformHint NOTIFY platformChanged)
    Q_PROPERTY(QString platformLink READ platformLink NOTIFY platformChanged)
    Q_PROPERTY(bool missionsSupported READ missionsSupported NOTIFY platformChanged)
    // Library
    Q_PROPERTY(QVariantList libraryMissions READ libraryMissions NOTIFY libraryChanged)
    Q_PROPERTY(QVariantList libraryLogs READ libraryLogs NOTIFY libraryChanged)
    Q_PROPERTY(QString libraryRoot READ libraryRoot NOTIFY libraryChanged)
    Q_PROPERTY(bool libraryPortable READ libraryPortable NOTIFY libraryChanged)
    Q_PROPERTY(QString currentMissionId READ currentMissionId NOTIFY libraryChanged)

public:
    PlanController(VehicleHub *hub, CommandSender *cmd, QObject *parent);
    static PlanController *s_instance;
    static PlanController *create(QQmlEngine *, QJSEngine *) { QJSEngine::setObjectOwnership(s_instance, QJSEngine::CppOwnership); return s_instance; }

    // ── QML: route card ──
    QString routeName() const;
    QString status() const { return m_status; }
    QString statusTitle() const;
    QStringList stats() const { return m_stats; }
    QVariantList chips() const { return m_chips; }
    QVariantList issues() const { return m_issues; }
    bool uploadBlocked() const;
    QString uploadText() const { return m_uploadText; }
    QString readText() const { return m_readText; }
    QString saveText() const { return m_saveText; }
    bool transferBusy() const { return m_transferBusy; }

    QObject *segmentModel() { return &m_segModel; }
    int segmentCount() const;
    QString selectedId() const { return m_selectedId; }
    QVariantList inspectorFields() const { return m_inspFields; }
    QVariantList inspectorActions() const { return m_inspActions; }
    QVariantList addActionOptions() const;
    bool inspectorCanAct() const;
    QStringList inspectorDerived() const { return m_inspDerived; }
    QVariantList inspectorIssues() const { return m_inspIssues; }
    QString inspectorCoords() const;
    bool canUndo() const { return !m_undo.isEmpty(); }
    bool canRedo() const { return !m_redo.isEmpty(); }
    QString undoTip() const;
    QString redoTip() const;

    QString tool() const { return m_tool; }
    bool drafting() const { return m_draft.has_value(); }
    QString hint() const { return m_hint; }
    QVariantList tools() const;
    bool showSat() const { return m_showSat; }
    void setShowSat(bool v) { m_showSat = v; emit layersChanged(); emit mapChanged(); }
    bool showCalc() const { return m_showCalc; }
    void setShowCalc(bool v) { m_showCalc = v; emit layersChanged(); emit mapChanged(); }
    bool showCam() const { return m_showCam; }
    void setShowCam(bool v) { m_showCam = v; emit layersChanged(); emit mapChanged(); }
    bool showLink() const { return m_showLink; }
    void setShowLink(bool v);
    bool linkBusy() const { return m_linkBusy; }

    QVariantList routeForm() const;
    QVariantList cameraForm() const;
    QVariantList gimbalForm() const;
    QVariantList radioForm() const;
    QStringList cameraDerived() const;
    QStringList radioDerived() const;
    QString homeDesc() const;
    QString operatorDesc() const;
    bool operatorPlaced() const;

    QString platformId() const;
    void setPlatformId(const QString &id);
    QString platformHint() const;
    QString platformLink() const;
    bool missionsSupported() const;

    QVariantList libraryMissions() const { return m_libMissions; }
    QVariantList libraryLogs() const { return m_libLogs; }
    QString libraryRoot() const { return m_libRoot; }
    bool libraryPortable() const { return m_libPortable; }
    QString currentMissionId() const { return m_currentId; }

    // ── QML: actions ──
    Q_INVOKABLE void setRouteName(const QString &name);
    Q_INVOKABLE void setTool(const QString &name);
    Q_INVOKABLE void escape();
    Q_INVOKABLE void keyTool(const QString &key);
    Q_INVOKABLE void selectSegment(const QString &id, bool pan = false);
    /// A click on a figure or base point: select it and bring its card into view.
    Q_INVOKABLE void selectFromMap(const QString &id);
    Q_INVOKABLE void toggleSegment(const QString &id, bool pan = false);
    Q_INVOKABLE void deleteSegment(const QString &id);
    Q_INVOKABLE void deleteSelected();
    Q_INVOKABLE void moveSegment(int from, int to);
    /// scope: seg | action | route | camera | gimbal | radio. value: text for numbers and selects, bool for checks.
    Q_INVOKABLE void setField(const QString &scope, const QString &segId, int actionIdx, const QString &key, const QVariant &value);
    Q_INVOKABLE void addAction(const QString &segId, const QString &type);
    Q_INVOKABLE void removeAction(const QString &segId, int idx);
    Q_INVOKABLE void undo();
    Q_INVOKABLE void redo();
    Q_INVOKABLE void upload();
    Q_INVOKABLE void read();
    Q_INVOKABLE void save();
    Q_INVOKABLE void saveAsNew();
    Q_INVOKABLE void openLibrary();
    Q_INVOKABLE void refreshLibrary();
    Q_INVOKABLE void libraryAction(const QString &act, const QString &id);
    Q_INVOKABLE void revealDataFolder();
    Q_INVOKABLE void menuAction(const QString &act);
    Q_INVOKABLE void importFile(const QUrl &url);
    Q_INVOKABLE void exportFile(const QUrl &url);
    Q_INVOKABLE QString exportName() const;
    Q_INVOKABLE void setHomeVehicle();
    Q_INVOKABLE void resetHome();
    Q_INVOKABLE void placeOperator();
    Q_INVOKABLE void resetOperator();
    Q_INVOKABLE void contextAction(const QString &act, double lat, double lng, const QString &segId, int pointIdx);
    Q_INVOKABLE void issueClicked(const QString &segId);
    /// Answer to a confirmRequested / promptRequested.
    Q_INVOKABLE void answer(int token, bool ok, const QString &text = {});
    Q_INVOKABLE void pageShown(bool shown);

    // ── PlanMapItem / ProfileItem ──
    const QJsonObject &route() const { return m_route; }
    const mission::CompiledRoute &compiled() const { return m_compiled; }
    int compileRevision() const { return m_compileRev; }
    struct Draft { QString type; QList<mission::LL> points; };
    const std::optional<Draft> &draft() const { return m_draft; }
    std::optional<mission::LL> takeoffPoint(QString *kind = nullptr) const;
    std::optional<mission::LL> operatorPoint() const;
    const std::optional<mission::GroundStation> &groundStation() const { return m_gs; }
    const std::optional<mission::LinkAnalysis> &linkAnalysis() const { return m_analysis; }
    const std::optional<mission::LinkEval> &worstEval() const { return m_worstEval; }
    const std::shared_ptr<const mission::Coverage> &coverage() const { return m_coverage; }
    mission::TerrainFn terrainFn() const;
    VehicleHub *hub() const { return m_hub; }

    // Map editing (no history step until the gesture ends)
    void mapClick(double lat, double lng, bool nearFirstDraftPoint);
    void mapDoubleClick();
    void draftAddPoint(double lat, double lng);
    Q_INVOKABLE void finishDraft();
    void cancelDraft();
    void movePoint(const QString &segId, int idx, double lat, double lng);
    void insertPoint(const QString &segId, int afterIdx, double lat, double lng);
    void moveSegmentBy(const QString &segId, const QList<mission::LL> &original, double dLat, double dLng);
    void setCircleRadius(const QString &segId, double radius);
    void endGesture(const QString &label);
    void setHome(double lat, double lng);
    void setOperator(std::optional<mission::LL> pos);
    void openRouteSettingsFromMap();
    void openRadioFromMap();

    // Hover point of the profile, drawn on the map
    void setProfileHover(std::optional<mission::LL> pt, std::optional<mission::Link> losClass);
    const std::optional<mission::LL> &profileHover() const { return m_profileHover; }
    const std::optional<mission::Link> &profileHoverClass() const { return m_profileHoverClass; }

signals:
    void routeChanged();
    void compiledChanged();
    void buttonsChanged();
    void selectionChanged();
    void inspectorChanged();
    void inspectorDetailChanged();
    void historyChanged();
    void toolChanged();
    void hintChanged();
    void layersChanged();
    void linkChanged();
    void formsChanged();
    void derivedChanged();
    void platformChanged();
    void libraryChanged();
    /// Something on the map moved: PlanMapItem repaints.
    void mapChanged();
    void fitRequested();
    void panRequested(double lat, double lng);
    void centerRequested(double lat, double lng, int zoom);
    void scrollToSegment(int index);
    void openPopover(const QString &which);    // route | camera | radio | ""
    void libraryOpenChanged(bool open);
    void toast(const QString &text);
    void confirmRequested(int token, const QString &message);
    void promptRequested(int token, const QString &message, const QString &value);
    void saveFileRequested(const QString &suggestedName);

private:
    // Document
    QJsonArray segs() const;
    void setSegs(const QJsonArray &a);
    int segIndex(const QString &id) const;
    QJsonObject segment(const QString &id) const;
    void putSegment(const QJsonObject &seg);
    QJsonObject params() const;
    void setParams(const QJsonObject &p);
    void replaceRoute(const QJsonObject &r);
    void addSegment(const QJsonObject &seg, const QString &label);

    // History
    bool commit(const QString &label);
    void resetHistory();
    void applyHistory(bool ok);

    // Compile
    void scheduleCompile(int delayMs = 250);
    void runCompile();
    void publishMission3D();
    // INAV missions over MSP
    QString inavLinkProblem() const;
    void uploadInav();
    void doUploadInav();
    void doReadInav();
    bool ensureRouteTerrain();
    void refreshCard();
    void refreshList();
    void rebuildInspector();
    void refreshInspectorDetail();
    QString segmentSummary(const QJsonObject &seg) const;
    QVariantList fieldsFor(const QJsonArray &fields, const QJsonObject &values, const std::function<QString(const QJsonObject &, const QString &)> &why) const;

    // Link
    void scheduleLink(int delayMs = 150);
    void runLink();
    QVariantList platformIssues() const;

    // Upload / download
    void doUpload();
    void doRead();
    void flashButton(QString &which, const QString &text, int ms);

    // Library
    void saveCurrent(bool forceNew);
    void writeMission(const QString &id, const QString &name);
    void loadMission(const QString &id);

    // Dialogs
    void confirm(const QString &message, std::function<void()> yes);
    void prompt(const QString &message, const QString &value, std::function<void(const QString &)> done);
    void setHint(const QString &text, bool flash = false);
    void syncName();

    VehicleHub *m_hub;
    CommandSender *m_cmd;
    QJsonObject m_route;
    mission::CompiledRoute m_compiled;
    QString m_compiledHomeKind;
    int m_compileRev = 0;
    QTimer m_compileTimer, m_linkTimer, m_pollTimer, m_terrainRetry;
    bool m_visible = false;

    // Card
    QString m_status = QStringLiteral("empty");
    QStringList m_stats{QStringLiteral("--"), QStringLiteral("--"), QStringLiteral("--"), QStringLiteral("--")};
    QVariantList m_chips, m_issues;
    QString m_uploadText = QStringLiteral("UPLOAD"), m_readText = QStringLiteral("READ"), m_saveText = QStringLiteral("SAVE");
    bool m_transferBusy = false;

    // List / inspector
    SegmentListModel m_segModel;
    QString m_selectedId;
    QVariantList m_inspFields, m_inspActions, m_inspIssues;
    QStringList m_inspDerived;

    // History
    struct Step { QJsonObject doc; QString label; };
    QJsonObject m_baseline;
    QList<Step> m_undo, m_redo;

    // Tools
    QString m_tool = QStringLiteral("select");
    QString m_hint;
    QTimer m_hintTimer;
    std::optional<Draft> m_draft;
    bool m_showSat = true, m_showCalc = true, m_showCam = true, m_showLink = false;

    // Link
    std::optional<mission::GroundStation> m_gs;
    std::optional<mission::LinkAnalysis> m_analysis;
    std::optional<mission::LinkEval> m_worstEval;
    std::shared_ptr<const mission::Coverage> m_coverage;
    QString m_coverageKey;
    std::shared_ptr<std::atomic<bool>> m_coverageCancel;
    bool m_linkBusy = false;
    std::optional<mission::LL> m_profileHover;
    std::optional<mission::Link> m_profileHoverClass;

    // Library
    QString m_currentId, m_currentName;
    QVariantList m_libMissions, m_libLogs;
    QString m_libRoot;
    bool m_libPortable = true;

    // Dialogs
    int m_nextToken = 1;
    QHash<int, std::function<void(bool, const QString &)>> m_pending;
    int m_inavLimit = 0;
    std::optional<mission::LL> m_centeredAt;
    bool m_centeredLive = false;
};

} // namespace corv
