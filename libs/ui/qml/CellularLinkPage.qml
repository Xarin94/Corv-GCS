// SETUP → COMMS → CELLULAR LINK (#subtab-lte-link, CellularLinkController.js):
// the LTEtelem module (ID, AES-256 key, REMEMBER KEY), the relay, the link
// status and the LTE cell the module reports, its mast from the cell database and
// the database itself. The logic is in LteController (singleton Lte) and CellDb.
import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Dialogs

Flickable {
    id: page
    contentWidth: width
    contentHeight: flow.implicitHeight + 48
    clip: true
    ScrollBar.vertical: ScrollBar { }

    readonly property var saved: Lte.saved
    readonly property var db: Lte.db
    Component.onCompleted: Lte.dbRefresh()
    // A wide value row: label above the text when it does not fit beside it
    component InfoRow: Column {
        property string label
        property string value: "--"
        property color valueColor: Theme.textMain
        width: parent ? parent.width : 320
        spacing: 2
        CfgLabel { text: Theme.tr(parent.label) }
        Text {
            width: parent.width
            wrapMode: Text.Wrap
            text: Theme.tr(parent.value)
            color: parent.valueColor
            font.family: Theme.fontData
            font.pixelSize: 11
        }
    }

    Flow {
        id: flow
        x: 24; y: 24
        width: page.width - 48
        spacing: 21

        SetupPanel {
            title: "CELLULAR MODULE"
            CfgRow { label: "MODULE ID"; CfgInput { id: moduleIn; width: 200; placeholderText: "CRV-001"; text: page.saved.moduleId || "" } }
            CfgRow {
                label: "AES-256 KEY"
                CfgInput {
                    id: keyIn
                    width: 150
                    placeholderText: qsTr("64 hex characters")
                    echoMode: showBtn.shown ? TextInput.Normal : TextInput.Password
                    text: Lte.savedKey
                }
                GcsButton {
                    id: showBtn
                    property bool shown: false
                    width: 44; height: 26
                    text: shown ? "HIDE" : "SHOW"
                    tip: "Show / hide the key"
                    onClicked: shown = !shown
                }
            }
            CfgCheck {
                id: rememberChk
                text: "REMEMBER KEY ON THIS COMPUTER"
                tip: "Stored encrypted by the operating system keychain, never in plain text"
                enabled: Lte.keychain
                checked: Lte.keychain && !!page.saved.rememberKey
                opacity: enabled ? 1 : 0.5
            }
            CfgNote {
                text: Theme.tr(Lte.keychain ? "Telemetry and commands are encrypted end to end with this key: the relay cannot read or alter them."
                                   : "No OS keychain available: the key must be entered every time. Telemetry and commands are encrypted end to end with this key.")
            }
            CfgNote { visible: !Lte.available; color: "#ff3333"; text: qsTr("This build has no crypto backend (OpenSSL): the cellular link is not available.") }
            Row {
                spacing: 6
                GcsButton {
                    width: 157; height: 29; primary: true
                    text: "CONNECT"
                    blocked: Lte.connecting
                    onClicked: if (!Lte.connecting) Lte.connectModule({
                        moduleId: moduleIn.text, keyHex: keyIn.text, host: hostIn.text, port: parseInt(portIn.text) || 5765,
                        certSha256: certIn.text, rememberKey: rememberChk.checked
                    })
                }
                GcsButton { width: 157; height: 29; text: "DISCONNECT"; onClicked: Lte.disconnectModule() }
            }
        }

        SetupPanel {
            title: "RELAY"
            CfgRow { label: "HOST"; CfgInput { id: hostIn; width: 200; placeholderText: "relay.example.com"; text: page.saved.host || "" } }
            CfgRow { label: "PORT"; CfgInput { id: portIn; width: 80; numeric: true; text: String(page.saved.port || 5765) } }
            CfgRow {
                label: "CERT SHA-256"
                CfgInput { id: certIn; width: 150; placeholderText: qsTr("Optional: pin the relay certificate"); text: page.saved.certSha256 || "" }
                GcsButton {
                    width: 44; height: 26
                    text: "PIN"
                    tip: "Pin the certificate the relay presented on the last connection"
                    onClicked: { const fp = Lte.pinCertificate(); if (fp) certIn.text = fp }
                }
            }
            CfgNote { text: qsTr("Relay settings stay on this computer only.") }
        }

        SetupPanel {
            title: "LINK STATUS"
            InfoRow { label: "STATE"; value: Lte.status.state || "Not connected"; valueColor: Lte.status.stateColor || Theme.textMain }
            InfoRow { label: "MODULE"; value: Lte.status.air || "--" }
            InfoRow { label: "MODULE SESSION"; value: Lte.status.session || "--" }
            InfoRow { label: "RELAY TLS"; value: Lte.status.tls || "--" }
            InfoRow { label: "RECORDS IN / OUT"; value: Lte.status.records || "--" }
            InfoRow { label: "REJECTED"; value: Lte.status.rejected || "--" }
            InfoRow { label: "TRAFFIC"; value: Lte.status.traffic || "--" }
            InfoRow { label: "LAST DATA"; value: Lte.status.age || "--" }
            InfoRow { label: "RECONNECTS"; value: Lte.status.reconnects || "--" }
            InfoRow { label: "LAST ERROR"; value: Lte.status.error || "--" }
        }

        SetupPanel {
            title: "LTE CELL MAP"
            InfoRow { label: "SERVING CELL"; value: Lte.cell.serving || "--" }
            InfoRow { label: "SIGNAL"; value: Lte.cell.signal || "--" }
            InfoRow { label: "NEIGHBOURS"; value: Lte.cell.neighbours || "--" }
            InfoRow { label: "CELL CHANGES"; value: Lte.cell.changes || "--" }
            InfoRow { label: "MAST"; value: Lte.cell.mast || "--" }
            CfgCheck {
                text: "SHOW CELL SITES ON THE MAP"
                tip: "Cell sites within 100 km and a dashed line to the serving cell, on the 2D map"
                checked: !!Lte.cellMap.show
                onToggled: (c) => Lte.setCellMap("show", c)
            }
            CfgCheck {
                text: "ALL OPERATORS"
                tip: "Off: only the network of the serving cell"
                checked: !!Lte.cellMap.allOperators
                onToggled: (c) => Lte.setCellMap("allOperators", c)
            }
            CfgCheck {
                text: "CELL ROWS ON THE FLIGHT SCREEN (LTE OVERLAY)"
                checked: !!Lte.cellMap.table
                onToggled: (c) => Lte.setCellMap("table", c)
            }
            InfoRow { label: "BUILT-IN"; value: page.db.bundle || "--" }
            InfoRow { label: "ADDED"; value: page.db.files || "--" }
            InfoRow { label: "DOWNLOADED"; value: page.db.status || "--" }
            InfoRow { label: "ONLINE LOOKUPS"; value: page.db.lookups || "--" }
            CfgCheck {
                text: "LOOK UP MISSING CELLS ONLINE"
                tip: "A serving cell in no database is asked to beaconDB (and to the OpenCelliD API with a token): only its identifiers are sent"
                checked: page.db.online !== false
                onToggled: (c) => Lte.dbSetOnline(c)
            }
            CfgRow {
                label: "OPENCELLID TOKEN"
                CfgInput {
                    id: tokenIn
                    width: 150
                    echoMode: TextInput.Password
                    placeholderText: Theme.tr(page.db.hasToken ? "saved (type a new one to replace it)" : "free at opencellid.org")
                }
                GcsButton {
                    width: 44; height: 26
                    text: "SAVE"
                    tip: "Keep the token: used for the online lookup and DOWNLOAD FROM OPENCELLID"
                    onClicked: { Lte.dbSaveToken(tokenIn.text); tokenIn.text = "" }
                }
            }
            CfgRow {
                label: "COUNTRY"
                CfgSelect { id: countrySel; width: 150; options: Lte.countries; value: Lte.countries[0] }
            }
            Row {
                spacing: 6
                GcsButton {
                    width: 157; height: 26
                    text: "DOWNLOAD FROM OPENCELLID"
                    tip: "Today's OpenCelliD file of the country (needs the token; twice a day at most)"
                    blocked: !!page.db.busy
                    onClicked: if (!page.db.busy) Lte.dbDownloadOpenCellid(countrySel.currentText)
                }
                GcsButton {
                    width: 157; height: 26
                    text: "DOWNLOAD 2024 COPY"
                    tip: "The whole country from the June 2024 OpenCelliD copy, for use without internet"
                    blocked: !!page.db.busy
                    onClicked: if (!page.db.busy) Lte.dbDownloadCopy(countrySel.currentText)
                }
            }
            Row {
                spacing: 6
                GcsButton {
                    width: 157; height: 26
                    text: "IMPORT CSV"
                    tip: "Add a cell CSV in the OpenCelliD or Mozilla MLS layout (.csv or .csv.gz)"
                    blocked: !!page.db.busy
                    onClicked: if (!page.db.busy) importDialog.open()
                }
                GcsButton {
                    width: 157; height: 26
                    text: "CLEAR"
                    tip: "Delete downloaded, imported and looked-up cells (the built-in ones stay)"
                    onClicked: Lte.dbClear()
                }
            }
            CfgNote {
                textFormat: Text.StyledText
                text: "Italy, San Marino and Vatican City are <b>built in</b> and work offline: OpenCelliD of the installer's date, the 2024 OpenCelliD copy and the final Mozilla Location Service export. Elsewhere cells within 100 km of the vehicle are downloaded by themselves from the 2024 OpenCelliD copy. A file DOWNLOADED FROM OPENCELLID or imported takes precedence. A serving cell in none of them is looked up online and kept. Positions are estimates from crowd measurements, not surveyed masts: they can be hundreds of metres off. Data: <b>OpenCelliD</b> (CC BY-SA 4.0), Mozilla Location Service and beaconDB (public domain)."
            }
        }
    }

    FileDialog {
        id: importDialog
        title: "Import LTE cells (OpenCelliD or MLS CSV)"
        nameFilters: ["Cell CSV (*.csv *.gz)", "All files (*)"]
        onAccepted: Lte.dbImport(selectedFile)
    }
}
