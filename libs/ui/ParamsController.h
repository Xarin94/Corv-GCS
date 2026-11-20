// ParamsController.h - SETUP → PARAMETERS ("Params" QML singleton): the port of
// ParametersPageController.js and of the parameter table of TabController.js.
//
// Left, the catalog: names known before any download (ParamCatalog), filtered by
// text and group; a click reads that one parameter, Enter reads the typed name,
// READ LISTED / READ ★ queue many. Right, the table of what is loaded, filtered
// by text, edited in place: a changed value turns amber and WRITE CHANGED sends
// the changed ones. READ ALL goes through ParamStore (FTP, then one by one).
// The lists are views over ParamStore, rebuilt when it changes (coalesced).
#pragma once

#include <QAbstractListModel>
#include <QHash>
#include <QObject>
#include <QQmlEngine>
#include <QTimer>
#include <QUrl>
#include <QVariantMap>

class QNetworkAccessManager;

namespace corv {

class ParamStore;
class VehicleHub;

class ParamTableModel : public QAbstractListModel
{
    Q_OBJECT
public:
    enum Roles {
        NameRole = Qt::UserRole + 1, ValueRole, DescRole, EditedRole, EditTextRole,
        TitleRole, UnitsRole, KindRole, MeaningRole, OptionsRole, OutOfRangeRole, RebootRole, ReadOnlyRole
    };
    explicit ParamTableModel(ParamStore *store, QObject *parent = nullptr);
    int rowCount(const QModelIndex & = {}) const override { return int(m_rows.size()); }
    QVariant data(const QModelIndex &index, int role) const override;
    QHash<int, QByteArray> roleNames() const override;

    void setFilter(const QString &f);
    /// The metadata follow the vehicle class ('Plane', 'Copter'...).
    void setVehicleClass(const QString &c);
    void refresh();
    /// Every row again (the metadata were replaced).
    void touchAll();
    /// The text typed in a row: true when it parses and differs from the vehicle's value.
    bool setEdit(const QString &name, const QString &text);
    QHash<QString, double> edits() const { return m_edits; }

private:
    /// The value shown: the one typed, else the vehicle's.
    bool currentValue(const QString &name, double *v) const;

    ParamStore *m_store;
    QString m_filter;
    QString m_vehicleClass = QStringLiteral("Plane");
    QStringList m_rows;
    QHash<QString, double> m_edits;
    QHash<QString, QString> m_editText;
};

class ParamCatalogModel : public QAbstractListModel
{
    Q_OBJECT
public:
    enum Roles { NameRole = Qt::UserRole + 1, ValueRole, LoadedRole, CustomRole, StateRole, FavoriteRole };
    explicit ParamCatalogModel(ParamStore *store, QObject *parent = nullptr);
    int rowCount(const QModelIndex & = {}) const override { return int(m_rows.size()); }
    QVariant data(const QModelIndex &index, int role) const override;
    QHash<int, QByteArray> roleNames() const override;

    /// Rows: the typed name first when the catalog does not know it, then at most 400 matches.
    void rebuild(const QStringList &catalog, const QString &filter, const QString &group);
    void refreshValues();
    QStringList visible() const { return m_visible; }

private:
    ParamStore *m_store;
    QStringList m_rows;
    QStringList m_visible;
    QString m_custom;
};

class ParamsController : public QObject
{
    Q_OBJECT
    QML_NAMED_ELEMENT(Params)
    QML_SINGLETON
    Q_PROPERTY(QAbstractItemModel *table READ table CONSTANT)
    Q_PROPERTY(QAbstractItemModel *catalog READ catalog CONSTANT)
    Q_PROPERTY(bool reading READ reading NOTIFY readChanged)
    Q_PROPERTY(QString readText READ readText NOTIFY readChanged)
    Q_PROPERTY(double readFraction READ readFraction NOTIFY readChanged)
    Q_PROPERTY(int loadedCount READ loadedCount NOTIFY listChanged)
    Q_PROPERTY(int changedCount READ changedCount NOTIFY listChanged)
    Q_PROPERTY(QString tableFilter READ tableFilter WRITE setTableFilter NOTIFY filtersChanged)
    Q_PROPERTY(QString catalogFilter READ catalogFilter WRITE setCatalogFilter NOTIFY filtersChanged)
    Q_PROPERTY(QString catalogGroup READ catalogGroup WRITE setCatalogGroup NOTIFY filtersChanged)
    Q_PROPERTY(QStringList groups READ groups NOTIFY catalogChanged)
    Q_PROPERTY(QString catalogCount READ catalogCount NOTIFY catalogChanged)
    Q_PROPERTY(QString queueText READ queueText NOTIFY queueChanged)
    Q_PROPERTY(bool queueBusy READ queueBusy NOTIFY queueChanged)
    Q_PROPERTY(bool queueError READ queueError NOTIFY queueChanged)
    Q_PROPERTY(int fetchTimeout READ fetchTimeout WRITE setFetchTimeout NOTIFY filtersChanged)
    Q_PROPERTY(bool linkReady READ linkReady NOTIFY linkChanged)
    Q_PROPERTY(QString metaSource READ metaSource NOTIFY metaChanged)
    Q_PROPERTY(bool metaBusy READ metaBusy NOTIFY metaChanged)
    Q_PROPERTY(int metaRev READ metaRev NOTIFY metaChanged)

public:
    ParamsController(ParamStore *store, VehicleHub *hub, QObject *parent);
    static ParamsController *s_instance;
    static ParamsController *create(QQmlEngine *, QJSEngine *) { QJSEngine::setObjectOwnership(s_instance, QJSEngine::CppOwnership); return s_instance; }

    QAbstractItemModel *table() { return &m_table; }
    QAbstractItemModel *catalog() { return &m_catalog; }
    bool reading() const;
    QString readText() const;
    double readFraction() const;
    int loadedCount() const;
    int changedCount() const { return int(m_table.edits().size()); }
    QString tableFilter() const { return m_tableFilter; }
    void setTableFilter(const QString &f);
    QString catalogFilter() const { return m_catalogFilter; }
    void setCatalogFilter(const QString &f);
    QString catalogGroup() const { return m_catalogGroup; }
    void setCatalogGroup(const QString &g);
    QStringList groups() const { return m_groups; }
    QString catalogCount() const;
    QString queueText() const;
    bool queueBusy() const;
    bool queueError() const;
    int fetchTimeout() const;
    void setFetchTimeout(int ms);
    bool linkReady() const;
    QString metaSource() const;
    bool metaBusy() const { return m_metaBusy; }
    int metaRev() const { return m_metaRev; }

    /// What ArduPilot documents about a parameter, for the detail strip and the editors:
    /// {name, known, title, description, units, user, hasRange, low, high, increment,
    ///  values: [{value, label}], bits: [{bit, label}], reboot, readOnly,
    ///  loaded, value (the vehicle's), current (typed or the vehicle's), edited, meaning, outOfRange}
    Q_INVOKABLE QVariantMap meta(const QString &name) const;
    /// The names of the bits set in `value` for a bitmask parameter.
    Q_INVOKABLE QString bitsLabel(const QString &name, double value) const;
    /// Newer metadata from autotest.ardupilot.org for the vehicle class (Mission Planner's
    /// "update parameter metadata"); BUNDLED goes back to the copy shipped with the GCS.
    Q_INVOKABLE void updateMeta();
    Q_INVOKABLE void resetMeta();

    Q_INVOKABLE void readAll();
    Q_INVOKABLE void cancelRead();
    Q_INVOKABLE void writeChanged();
    Q_INVOKABLE bool setEdit(const QString &name, const QString &text);
    Q_INVOKABLE void saveFile(const QUrl &file);
    Q_INVOKABLE void loadFile(const QUrl &file);
    /// Catalog: read one name (a click), the typed name or the first match (Enter)
    Q_INVOKABLE void fetch(const QString &name);
    Q_INVOKABLE void fetchTyped();
    Q_INVOKABLE void fetchListed();
    Q_INVOKABLE void fetchFavorites();
    Q_INVOKABLE void toggleFavorite(const QString &name);
    /// The page was opened: the catalog follows the vehicle class.
    Q_INVOKABLE void pageShown();

signals:
    void readChanged();
    void listChanged();
    void filtersChanged();
    void catalogChanged();
    void queueChanged();
    void linkChanged();
    void metaChanged();

private:
    void rebuildCatalog();
    QString vehicleClass() const;

    ParamStore *m_store;
    VehicleHub *m_hub;
    ParamTableModel m_table;
    ParamCatalogModel m_catalog;
    QString m_tableFilter, m_catalogFilter, m_catalogGroup;
    QStringList m_catalogNames, m_groups;
    QTimer m_linkPoll;
    bool m_linkReady = false;
    QString m_metaClass;
    bool m_metaBusy = false;
    int m_metaRev = 0;
    QNetworkAccessManager *m_net = nullptr;
};

} // namespace corv
