// SETUP → COMMS → TELEMETRY FORWARD (#subtab-tel-forward, initTelForwardTab
// of TabController.js): add an output (UDP client / server, serial; MAVLink
// passthrough or LTM), the active outputs with their counters, the data being
// forwarded. The logic is in TelForwardController (singleton TelForward).
import QtQuick
import QtQuick.Controls.Basic

Flickable {
    id: page
    contentWidth: width
    contentHeight: flow.implicitHeight + 48
    clip: true
    ScrollBar.vertical: ScrollBar { }

    readonly property var saved: TelForward.saved
    property string type: saved.type || "udp-client"
    property string protocol: saved.protocol || "mavlink"
    property var ports: []
    onTypeChanged: if (type === "serial" && ports.length === 0) ports = Gcs.serialPorts()

    Flow {
        id: flow
        x: 24; y: 24
        width: page.width - 48
        spacing: 21

        SetupPanel {
            title: "ADD OUTPUT"
            CfgRow {
                label: "OUTPUT TYPE"
                CfgSelect {
                    width: 200
                    options: [{ value: "udp-client", label: "UDP Client (send to host)" }, { value: "udp-server", label: "UDP Server (listen)" },
                              { value: "serial", label: "Serial Port" }]
                    value: page.type
                    onPicked: (v) => page.type = v
                }
            }
            CfgRow {
                label: "SERIAL PORT"
                visible: page.type === "serial"
                CfgSelect {
                    id: portSel
                    width: 170
                    options: [{ value: "", label: "Select port..." }].concat(page.ports.map(p => ({ value: p.path, label: p.label })))
                    onPicked: (v) => value = v
                }
                GcsButton { width: 26; height: 26; text: "↺"; tip: "Scan ports"; onClicked: page.ports = Gcs.serialPorts() }
            }
            CfgRow {
                label: "BAUD RATE"
                visible: page.type === "serial"
                CfgSelect { id: baudSel; width: 120; options: ["2400", "4800", "9600", "19200", "38400", "57600", "115200"]; value: String(page.saved.baudRate || 57600); onPicked: (v) => value = v }
            }
            CfgRow { label: "HOST"; visible: page.type === "udp-client"; CfgInput { id: hostIn; width: 200; text: page.saved.host || "127.0.0.1" } }
            CfgRow { label: "PORT"; visible: page.type === "udp-client"; CfgInput { id: portIn; width: 80; numeric: true; text: String(page.saved.port || 14550) } }
            CfgRow { label: "LISTEN PORT"; visible: page.type === "udp-server"; CfgInput { id: listenIn; width: 80; numeric: true; text: String(page.saved.listenPort || 14551) } }
            CfgRow {
                label: "PROTOCOL"
                CfgSelect {
                    width: 200
                    options: [{ value: "mavlink", label: "MAVLink Passthrough" }, { value: "ltm", label: "LTM (Lightweight Telemetry)" }]
                    value: page.protocol
                    onPicked: (v) => page.protocol = v
                }
            }
            // Write access only makes sense for MAVLink passthrough
            CfgCheck {
                id: rwChk
                visible: page.protocol === "mavlink"
                text: "WRITE ACCESS (BIDIRECTIONAL)"
            }
            GcsButton {
                width: parent.width; height: 29
                primary: true
                text: "ADD OUTPUT"
                onClicked: TelForward.addOutput({
                    type: page.type, protocol: page.protocol, writeAccess: rwChk.checked,
                    portPath: portSel.value, baudRate: parseInt(baudSel.value) || 57600,
                    host: hostIn.text.trim(), port: parseInt(portIn.text) || 0, listenPort: parseInt(listenIn.text) || 0
                })
            }
            CfgValue {
                width: parent.width
                horizontalAlignment: Text.AlignLeft
                text: TelForward.addStatus
                color: TelForward.addError ? "#ff3333" : Theme.accentCyan
                visible: text.length > 0
            }
        }

        SetupPanel {
            title: "ACTIVE OUTPUTS"
            Text {
                visible: TelForward.outputs.length === 0
                text: "No outputs configured"
                opacity: 0.5
                color: Theme.textDim; font.family: Theme.fontUi; font.pixelSize: 11
            }
            Repeater {
                model: TelForward.outputs
                delegate: Column {
                    required property var modelData
                    width: parent.width
                    spacing: 3
                    opacity: modelData.connected ? 1 : 0.6
                    Item {
                        width: parent.width
                        height: 24
                        Text {
                            anchors.verticalCenter: parent.verticalCenter
                            width: parent.width - 110
                            elide: Text.ElideRight
                            text: parent.parent.modelData.label
                            color: Theme.textMain; font.family: Theme.fontData; font.pixelSize: 11
                        }
                        Rectangle {
                            // Write access: packets from this endpoint are injected into the vehicle link
                            visible: parent.parent.modelData.rw
                            x: parent.width - 104
                            anchors.verticalCenter: parent.verticalCenter
                            width: 24; height: 15; radius: 2
                            color: "transparent"; border.color: Theme.accentActive; border.width: Theme.hairline
                            Text { anchors.centerIn: parent; text: "RW"; color: Theme.accentActive; font.family: Theme.fontUi; font.pixelSize: 9; font.weight: Font.Bold }
                        }
                        GcsButton {
                            anchors.right: parent.right
                            anchors.verticalCenter: parent.verticalCenter
                            width: 70; height: 22
                            text: "REMOVE"
                            onClicked: TelForward.removeOutput(parent.parent.modelData.id)
                        }
                    }
                    Text {
                        width: parent.width
                        wrapMode: Text.Wrap
                        text: parent.modelData.stats
                        color: Theme.textDim; font.family: Theme.fontData; font.pixelSize: 10
                    }
                    Rectangle { width: parent.width; height: 1; color: Theme.borderSubtle }
                }
            }
        }

        SetupPanel {
            title: "CURRENT DATA"
            CfgRow { label: "LAT"; CfgValue { text: TelForward.current.lat || "---" } }
            CfgRow { label: "LON"; CfgValue { text: TelForward.current.lon || "---" } }
            CfgRow { label: "ALT"; CfgValue { text: TelForward.current.alt || "---" } }
            CfgRow { label: "HEADING"; CfgValue { text: TelForward.current.heading || "---" } }
            CfgRow { label: "GROUNDSPEED"; CfgValue { text: TelForward.current.gs || "---" } }
        }
    }
}
