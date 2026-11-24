// SETUP → COMMS → RTK / GPS INJECT (#subtab-rtk-gps, initRTKTab of
// TabController.js): the correction source (NTRIP caster or serial base
// station), the RTCM stream statistics and the vehicle's RTK state. The logic
// is in RtkController (singleton Rtk).
import QtQuick
import QtQuick.Controls.Basic

Flickable {
    id: page
    contentWidth: width
    contentHeight: flow.implicitHeight + 48
    clip: true
    ScrollBar.vertical: ScrollBar { }

    readonly property var saved: Rtk.saved
    property string source: saved.source || "ntrip"
    property string ggaMode: saved.ggaMode || "vehicle"
    property var ports: []
    Component.onCompleted: ports = Gcs.serialPorts()   // auto-scan on the first visit

    function ntripConfig() {
        return {
            host: hostIn.text.trim(), port: parseInt(portIn.text) || 2101, mountpoint: mountIn.text.trim(),
            username: userIn.text, password: passIn.text, tls: tlsChk.checked, ggaMode: page.ggaMode,
            lat: parseFloat(latIn.text), lon: parseFloat(lonIn.text), alt: parseFloat(altIn.text) || 0,
            ggaInterval: parseInt(ggaInt.value) || 10
        }
    }

    Flow {
        id: flow
        x: 24; y: 24
        width: page.width - 48
        spacing: 21

        SetupPanel {
            title: "CORRECTION SOURCE"
            CfgRow {
                label: "SOURCE"
                CfgSelect {
                    width: 200
                    options: [{ value: "ntrip", label: "NTRIP Caster (Internet)" }, { value: "serial", label: "Serial Base Station" }]
                    value: page.source
                    onPicked: (v) => { page.source = v; Rtk.setSourceChoice(v) }
                }
            }
            CfgValue {
                width: parent.width
                horizontalAlignment: Text.AlignLeft
                text: Theme.tr(Rtk.connStatus)
                color: Rtk.connStatusActive ? Theme.accentCyan : Theme.textMain
            }
        }

        SetupPanel {
            title: "NTRIP CASTER"
            visible: page.source === "ntrip"
            CfgRow { label: "HOST"; CfgInput { id: hostIn; width: 200; placeholderText: "caster.example.com"; text: page.saved.host || "" } }
            CfgRow { label: "PORT"; CfgInput { id: portIn; width: 80; numeric: true; text: String(page.saved.port || 2101) } }
            CfgRow {
                label: "MOUNTPOINT"
                CfgInput { id: mountIn; width: 140; placeholderText: "MOUNTPOINT"; text: page.saved.mountpoint || "" }
                GcsButton {
                    width: 54; height: 26
                    text: Rtk.listing ? "..." : "LIST"
                    tip: "Fetch caster sourcetable"
                    onClicked: if (!Rtk.listing) Rtk.fetchSourcetable(page.ntripConfig())
                }
            }
            CfgRow {
                label: "SOURCETABLE"
                visible: Rtk.mounts.length > 0
                CfgSelect {
                    width: 200
                    options: [{ value: "", label: "-- " + Rtk.mounts.length + " mountpoints --" }].concat(Rtk.mounts)
                    value: ""
                    // Picking from the sourcetable fills the mountpoint field
                    onPicked: (v) => { if (v) mountIn.text = v }
                }
            }
            CfgRow { label: "USERNAME"; CfgInput { id: userIn; width: 200; text: page.saved.username || "" } }
            CfgRow { label: "PASSWORD"; CfgInput { id: passIn; width: 200; echoMode: TextInput.Password } }
            CfgCheck { id: tlsChk; text: "USE TLS"; checked: !!page.saved.tls }
            CfgRow {
                label: "GGA UPLOAD (VRS)"
                CfgSelect {
                    width: 160
                    options: [{ value: "off", label: "Off" }, { value: "vehicle", label: "Vehicle position" }, { value: "manual", label: "Manual position" }]
                    value: page.ggaMode
                    onPicked: (v) => page.ggaMode = v
                }
            }
            CfgRow { label: "LAT"; visible: page.ggaMode === "manual"; CfgInput { id: latIn; width: 140; numeric: true; placeholderText: "45.0000000"; text: page.saved.lat !== undefined && page.saved.lat !== null ? String(page.saved.lat) : "" } }
            CfgRow { label: "LON"; visible: page.ggaMode === "manual"; CfgInput { id: lonIn; width: 140; numeric: true; placeholderText: "9.0000000"; text: page.saved.lon !== undefined && page.saved.lon !== null ? String(page.saved.lon) : "" } }
            CfgRow { label: "ALT (M)"; visible: page.ggaMode === "manual"; CfgInput { id: altIn; width: 80; numeric: true; text: String(page.saved.alt || 0) } }
            CfgRow {
                label: "GGA INTERVAL (S)"
                CfgSelect { id: ggaInt; width: 80; options: ["1", "5", "10", "30"]; value: String(page.saved.ggaInterval || 10); onPicked: (v) => value = v }
            }
            Row {
                spacing: 6
                GcsButton { width: 157; height: 29; primary: true; text: "CONNECT"; blocked: Rtk.connecting; onClicked: if (!Rtk.connecting) Rtk.ntripConnect(page.ntripConfig(), page.source) }
                GcsButton { width: 157; height: 29; text: "DISCONNECT"; onClicked: Rtk.ntripDisconnect() }
            }
        }

        SetupPanel {
            title: "SERIAL BASE STATION"
            visible: page.source === "serial"
            CfgRow {
                label: "SERIAL PORT"
                CfgSelect {
                    id: portSel
                    width: 170
                    options: [{ value: "", label: "Select port..." }].concat(page.ports.map(p => ({ value: p.path, label: p.label })))
                    onPicked: (v) => value = v
                }
                GcsButton { width: 26; height: 26; text: "↺"; tip: "Scan"; onClicked: page.ports = Gcs.serialPorts() }
            }
            CfgRow {
                label: "BAUD RATE"
                CfgSelect { id: baudSel; width: 120; options: ["9600", "38400", "57600", "115200", "230400", "460800"]; value: "115200"; onPicked: (v) => value = v }
            }
            Row {
                spacing: 6
                GcsButton { width: 157; height: 29; primary: true; text: "CONNECT"; onClicked: Rtk.connectSerial(portSel.value, parseInt(baudSel.value) || 115200) }
                GcsButton { width: 157; height: 29; text: "DISCONNECT"; onClicked: Rtk.disconnectSerial() }
            }
        }

        SetupPanel {
            title: "RTK STREAM STATUS"
            CfgRow { label: "STATUS"; CfgValue { text: Theme.tr(Rtk.streamStatus); color: Rtk.streaming ? "#00ff7f" : Theme.textMain } }
            CfgRow { label: "SOURCE"; CfgValue { text: Theme.tr(Rtk.activeSource) } }
            CfgRow { label: "MSG RATE"; CfgValue { text: Rtk.msgPerSec + " msg/s" } }
            CfgRow { label: "TOTAL MSGS"; CfgValue { text: String(Rtk.msgCount) } }
            CfgRow { label: "BYTES RX"; CfgValue { text: Rtk.bytesText } }
            CfgRow { label: "GGA SENT"; CfgValue { text: Rtk.ggaSent } }
            CfgRow { label: "RECONNECTS"; CfgValue { text: Rtk.reconnects } }
            Text { text: qsTr("RTCM MESSAGE TYPES"); color: Theme.accentCyan; font.family: Theme.fontUi; font.pixelSize: 12; font.weight: Font.Bold; font.letterSpacing: 2; topPadding: 8 }
            Column {
                width: parent.width
                Text {
                    visible: Rtk.msgTypes.length === 0
                    text: qsTr("No messages yet"); opacity: 0.5
                    color: Theme.textDim; font.family: Theme.fontData; font.pixelSize: 11
                }
                Repeater {
                    model: Rtk.msgTypes
                    delegate: Row {
                        required property var modelData
                        spacing: 8
                        leftPadding: 4
                        Text { width: 40; text: parent.modelData.id; color: "#00d2ff"; font.family: Theme.fontData; font.pixelSize: 11 }
                        Text { text: Theme.tr(parent.modelData.name); color: Theme.textDim; font.family: Theme.fontData; font.pixelSize: 11 }
                    }
                }
            }
        }

        SetupPanel {
            title: "DRONE RTK STATUS"
            CfgRow { label: "GPS FIX"; CfgValue { text: Theme.tr(Rtk.droneFix); color: Rtk.droneFixColor || Theme.textMain; font.weight: Font.Bold } }
            CfgRow { label: "SATELLITES"; CfgValue { text: Rtk.droneSat } }
            CfgRow { label: "HDOP"; CfgValue { text: Rtk.droneHdop } }
            CfgRow { label: "BASELINE"; CfgValue { text: Rtk.droneBaseline } }
            CfgRow { label: "ACCURACY"; CfgValue { text: Rtk.droneAccuracy } }
            CfgRow { label: "IAR HYPOTHESES"; CfgValue { text: Rtk.droneIar } }
        }
    }
}
