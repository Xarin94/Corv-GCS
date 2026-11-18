// LteController.h - SETUP → COMMS → CELLULAR LINK (CellularLinkController.js;
// QML singleton "Lte") and the LTE cell readouts of the flight screen
// (updateCellRows of UIController.js).
//
// Connects to one LTEtelem module through a relay: the operator gives the
// module ID and its AES-256 key, the relay address is entered once. The relay
// knows no keys, so a wrong key is not refused: the module just stays
// offline, and the key fingerprint shown then is the one to compare with
// e2e_key_fp on the module. The link itself is LteTransport on the link thread
// and feeds the normal MAVLink pipeline.
//
// Module ID and relay settings persist under 'lte-link-config'. The key is
// never written there: with REMEMBER KEY it is kept by the OS keychain
// (DPAPI on Windows, the Secret Service through libsecret on Linux; none on
// macOS yet, where it is typed every time).
//
// The cell map settings ('lte-cell-map': show, allOperators, table) drive the
// cell sites of the maps and the cell rows of the position panel (table = GCS
// OPTIONS → LTE OVERLAY).
//
// Cell sites (CellTowerLayer.js): while CORV_CELL_INFO arrives (last message
// under 10 s old) the sites within 100 km come from the cell database (CellDb)
// and are fetched again when the vehicle has moved 10 km from the last query
// centre, when the serving network changes or the settings change. The ones
// drawn, the serving site and the others within 10 km, are picked again every
// 250 m and at every cell change and published to the mini-map and the 3D view
// (CellOverlay). A serving cell missing from the sites is asked again with its
// ECI (found under another operator's code when the cell is shared), then
// looked up online and added as a site of its own.
// GUI thread.
#pragma once

#include "core/Geo.h"
#include "scene/CellOverlay.h"

#include <QObject>
#include <QQmlEngine>
#include <QTimer>
#include <QUrl>
#include <QVariantMap>
#include <memory>
#include <optional>

namespace corv {

class CellDb;
class CommandSender;
class ConnectionManager;
class LinkManager;
class VehicleHub;

class LteController : public QObject
{
    Q_OBJECT
    QML_NAMED_ELEMENT(Lte)
    QML_SINGLETON

    Q_PROPERTY(QVariantMap saved READ saved NOTIFY savedChanged)       // moduleId, host, port, certSha256, rememberKey
    Q_PROPERTY(bool keychain READ keychain CONSTANT)
    Q_PROPERTY(QString savedKey READ savedKey NOTIFY savedKeyChanged)
    Q_PROPERTY(bool available READ available CONSTANT)                 // crypto backend in this build
    Q_PROPERTY(bool connecting READ connecting NOTIFY statusChanged)
    Q_PROPERTY(bool online READ online NOTIFY statusChanged)
    Q_PROPERTY(QVariantMap status READ status NOTIFY statusChanged)     // the texts of the STATUS panel
    Q_PROPERTY(QVariantMap cell READ cell NOTIFY cellChanged)           // the CELL panel (1 Hz)
    Q_PROPERTY(QVariantMap cellRows READ cellRows NOTIFY cellRowsChanged)   // flight screen rows (4 Hz)
    Q_PROPERTY(QVariantMap cellMap READ cellMap NOTIFY cellMapChanged)  // show, allOperators, table
    Q_PROPERTY(QVariantMap db READ db NOTIFY dbChanged)                 // the texts of the cell database rows, online, hasToken, busy
    Q_PROPERTY(QStringList countries READ countries CONSTANT)

public:
    LteController(LinkManager *link, ConnectionManager *conn, CommandSender *cmd, VehicleHub *hub, QObject *parent = nullptr);
    ~LteController() override;

    static LteController *s_instance;
    static LteController *create(QQmlEngine *, QJSEngine *) { QJSEngine::setObjectOwnership(s_instance, QJSEngine::CppOwnership); return s_instance; }

    /// CONNECT: validates, remembers the settings (and the key with REMEMBER KEY), connects.
    Q_INVOKABLE void connectModule(const QVariantMap &form);
    Q_INVOKABLE void disconnectModule();
    /// The fingerprint of the relay certificate seen last ("" before any connection); saved as the pin.
    Q_INVOKABLE QString pinCertificate();
    Q_INVOKABLE void setCellMap(const QString &name, bool on);

    // Cell database (LTE CELL MAP panel)
    Q_INVOKABLE void dbRefresh();
    Q_INVOKABLE void dbSetOnline(bool on);
    Q_INVOKABLE void dbSaveToken(const QString &token);
    Q_INVOKABLE void dbDownloadOpenCellid(const QString &country);
    Q_INVOKABLE void dbDownloadCopy(const QString &country);
    Q_INVOKABLE void dbImport(const QUrl &file);
    Q_INVOKABLE void dbClear();

    QVariantMap saved() const;
    bool keychain() const;
    QString savedKey() const { return m_savedKey; }
    bool available() const;
    bool connecting() const { return m_connecting; }
    bool online() const { return m_online; }
    QVariantMap status() const { return m_status; }
    QVariantMap cell() const { return m_cell; }
    QVariantMap cellRows() const { return m_cellRows; }
    QVariantMap cellMap() const;
    QVariantMap db() const { return m_dbView; }
    QStringList countries() const;

    /// "1870.0 MHz" from the downlink EARFCN, "B3" when unknown, else fallback (lte-bands.js).
    static QString frequencyText(unsigned earfcn, int band, const QString &fallback = QStringLiteral("---"));
    static QString operatorName(int mcc, int mnc);
    static QString operatorShort(int mcc, int mnc);

signals:
    void savedChanged();
    void savedKeyChanged();
    void statusChanged();
    void cellChanged();
    void cellRowsChanged();
    void cellMapChanged();
    void dbChanged();

private:
    void onLteStatus(const QVariantMap &st);
    void renderCell();
    void updateCellRows();
    void saveConfig(const QVariantMap &cfg);
    // Cell sites (CellTowerLayer.js)
    void updateCellMap();
    void queryCells(const LatLon &centre);
    void findServing();
    void lookupServing();
    void pickSites(const std::optional<LatLon> &vehicle);
    void publishOverlay();
    void hideOverlay();
    QColor linkColor() const;
    void renderDbStatus(const QVariantMap &st);

    LinkManager *m_link;
    ConnectionManager *m_conn;
    CommandSender *m_cmd;
    VehicleHub *m_hub;
    QString m_savedKey;
    bool m_connecting = false;
    bool m_online = false;
    QVariantMap m_status;
    QString m_lastFingerprint;
    QString m_lastModuleSession;
    QVariantMap m_cell, m_cellRows;
    QTimer m_cellTimer, m_rowsTimer;
    qint64 m_rowsAt = 0;

    std::unique_ptr<CellDb> m_db;
    QVariantMap m_dbView;
    bool m_dbBusy = false;
    std::vector<CellSite> m_sites;            // last query result
    int m_sitesVersion = 0, m_filterVersion = -1;
    int m_serving = -1, m_filterServing = -2;   // index into m_sites
    unsigned m_servingEci = 0, m_requeriedEci = 0, m_lookupEci = 0;
    std::optional<LatLon> m_filterCentre, m_queryCentre;
    QString m_queryKey;
    bool m_querying = false, m_lookingUp = false, m_shown = false;
    qint64 m_queryRetryAt = 0;
    QString m_queryError, m_dbDate;
    int m_dbMissing = 0;
    std::vector<CellSite> m_drawn;            // published: the serving site and those within 10 km
    int m_drawnServing = -1;
    QColor m_linkColor;
    int m_overlayVersion = 0;
    std::optional<double> m_servingKm;        // vehicle to the serving mast
};

} // namespace corv
