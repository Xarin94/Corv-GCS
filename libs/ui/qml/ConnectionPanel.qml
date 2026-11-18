// SETUP → CONNECTION (#subtab-connection, TabController.js): connection type,
// the fields of that type, CONNECT / DISCONNECT. Settings are remembered.
import QtQuick
import QtQuick.Controls.Basic

SetupPanel {
    id: panel
    title: "CONNECTION"

    readonly property var types: [
        ["mavlink-serial", "MAVLink Serial"], ["mavlink-udp", "MAVLink UDP"], ["mavlink-tcp", "MAVLink TCP"],
        ["msp-serial", "MSP Serial (INAV / Betaflight)"], ["msp-tcp", "MSP TCP (INAV SITL)"]
    ]
    readonly property string type: types[typeBox.currentIndex][0]
    readonly property bool serial: type === "mavlink-serial" || type === "msp-serial"
    property var ports: []

    component CfgCombo: DarkCombo {
        width: 320
        height: 29
        itemHeight: 26
    }
    component CfgInput: TextField {
        width: 320
        height: 29
        leftPadding: 12
        color: Theme.textMain
        font.family: Theme.fontData
        font.pixelSize: Theme.fsMd
        background: Rectangle { radius: 3; color: Theme.inputBg; border.color: Theme.borderInput; border.width: Theme.hairline }
    }
    component GcsBtn: Rectangle {
        id: gb
        property string text
        property bool connect: false
        signal clicked()
        width: 320
        height: 29
        radius: 3
        color: connect ? Qt.rgba(0, 180 / 255, 0, gm.containsMouse ? 0.3 : 0.2) : (gm.containsMouse ? Theme.gcsButtonHover : Theme.gcsButton)
        border.width: Theme.hairline
        border.color: connect ? Theme.accentGreen : Theme.borderInput
        Text {
            anchors.centerIn: parent
            text: gb.text
            color: gb.connect ? Theme.accentGreen : Theme.textMain
            font.family: Theme.fontUi
            font.pixelSize: Theme.fsMd
            font.weight: Font.DemiBold
            font.letterSpacing: 0.8
        }
        MouseArea { id: gm; anchors.fill: parent; hoverEnabled: true; onClicked: gb.clicked() }
    }

    Column {
        spacing: 3
        CfgLabel { text: "CONNECTION TYPE" }
        CfgCombo {
            id: typeBox
            model: panel.types.map(t => t[1])
            currentIndex: Math.max(0, panel.types.findIndex(t => t[0] === Gcs.setting("conn-type", "mavlink-serial")))
            onActivated: {
                Gcs.setSetting("conn-type", panel.type)
                if (panel.serial) panel.ports = Gcs.serialPorts()
            }
        }
        Text {
            width: 320
            wrapMode: Text.Wrap
            text: "ArduPilot: MAVLink Serial at 57600 baud (pre-selected — change it if your setup differs)."
            color: Theme.textDim
            font.family: Theme.fontUi
            font.pixelSize: Theme.fsSm
            lineHeight: 1.3
            topPadding: 3
        }
    }

    // Serial fields
    Column {
        visible: panel.serial
        spacing: 3
        CfgLabel { text: "SERIAL PORT" }
        CfgCombo {
            id: portBox
            model: panel.ports.length ? panel.ports.map(p => p.label) : ["-- Scan ports --"]
        }
        GcsBtn { text: "SCAN"; onClicked: panel.ports = Gcs.serialPorts() }
    }
    Column {
        visible: panel.serial
        spacing: 3
        CfgLabel { text: "BAUD RATE" }
        CfgCombo {
            id: baudBox
            model: ["9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600"]
            currentIndex: panel.type === "msp-serial" ? 4 : 3
        }
    }

    // Network fields
    Column {
        visible: panel.type === "mavlink-udp"
        spacing: 3
        CfgLabel { text: "UDP HOST" }
        CfgInput { id: udpHost; text: Gcs.setting("udp-host", "127.0.0.1") }
        CfgLabel { text: "UDP PORT"; topPadding: 7 }
        CfgInput { id: udpPort; text: Gcs.setting("udp-port", "14550") }
    }
    Column {
        visible: panel.type === "mavlink-tcp" || panel.type === "msp-tcp"
        spacing: 3
        CfgLabel { text: "TCP HOST" }
        CfgInput { id: tcpHost; text: Gcs.setting("tcp-host", "127.0.0.1"); placeholderText: "Simulator IP (e.g. 192.168.x.x)" }
        CfgLabel { text: "TCP PORT"; topPadding: 7 }
        CfgInput { id: tcpPort; text: Gcs.setting("tcp-port", "5760") }
    }

    Column {
        spacing: 0
        GcsBtn {
            text: Gcs.connectionState === "CONNECTING" ? "CONNECTING..." : "CONNECT"
            connect: true
            onClicked: {
                const t = panel.type
                if (t === "mavlink-udp") {
                    Gcs.setSetting("udp-host", udpHost.text); Gcs.setSetting("udp-port", udpPort.text)
                    Gcs.connectType(t, udpHost.text, parseInt(udpPort.text))
                } else if (t === "mavlink-tcp" || t === "msp-tcp") {
                    Gcs.setSetting("tcp-host", tcpHost.text); Gcs.setSetting("tcp-port", tcpPort.text)
                    Gcs.connectType(t, tcpHost.text, parseInt(tcpPort.text))
                } else {
                    const p = panel.ports[portBox.currentIndex]
                    Gcs.connectType(t, p ? p.path : "", parseInt(baudBox.currentText))
                }
            }
        }
        GcsBtn { text: "DISCONNECT"; onClicked: Gcs.disconnectLink() }
        Text {
            visible: Gcs.lastError.length > 0
            width: 320
            topPadding: 6
            wrapMode: Text.Wrap
            text: "Connection failed: " + Gcs.lastError
            color: Theme.statusRed
            font.family: Theme.fontUi
            font.pixelSize: Theme.fsSm
        }
    }
}
