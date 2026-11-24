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
    // SYS CONFIG → FLIGHT STACK pre-sets the link; a live one is never touched
    function suggest() {
        if (Vehicle.connected) return
        const i = types.findIndex(t => t[0] === Plan.platformConnType)
        if (i >= 0) { typeBox.currentIndex = i; Gcs.setSetting("conn-type", types[i][0]) }
        const b = baudBox.model.indexOf(String(Plan.platformBaud))
        if (b >= 0) baudBox.currentIndex = b
    }
    Connections { target: Plan; function onPlatformChanged() { panel.suggest() } }

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
            text: Theme.tr(gb.text)
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
        CfgLabel { text: qsTr("CONNECTION TYPE") }
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
            text: Vehicle.connected ? Plan.platformLabel + qsTr(" selected — disconnect to change the link.")
                                    : Plan.platformLabel + ": " + Theme.tr(Plan.platformConnLabel) + qsTr(" at ") + Plan.platformBaud
                                      + qsTr(" baud (pre-selected — change it if your setup differs).")
            color: Theme.textDim
            font.family: Theme.fontUi
            font.pixelSize: Theme.fsSm
            lineHeight: 1.3
            topPadding: 3
        }
    }

    // MSP is request / response: the poll rate is the telemetry rate
    Column {
        objectName: "mspProfile"
        visible: panel.type === "msp-serial" || panel.type === "msp-tcp"
        spacing: 3
        CfgLabel { text: qsTr("POLL RATE") }
        CfgCombo {
            readonly property var profiles: ["normal", "slow"]
            model: [Theme.tr("Normal (USB / 115200)"), Theme.tr("Slow link (long-range radio)")]
            currentIndex: Math.max(0, profiles.indexOf(Gcs.setting("msp-profile", "normal")))
            onActivated: Gcs.setSetting("msp-profile", profiles[currentIndex])
            HoverTip { text: "MSP is request/response — the poll rate is the telemetry rate" }
        }
    }

    // Serial fields
    Column {
        visible: panel.serial
        spacing: 3
        CfgLabel { text: qsTr("SERIAL PORT") }
        CfgCombo {
            id: portBox
            model: panel.ports.length ? panel.ports.map(p => p.label) : ["-- Scan ports --"]
        }
        GcsBtn { text: "SCAN"; onClicked: panel.ports = Gcs.serialPorts() }
    }
    Column {
        visible: panel.serial
        spacing: 3
        CfgLabel { text: qsTr("BAUD RATE") }
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
        CfgLabel { text: qsTr("UDP HOST") }
        CfgInput { id: udpHost; text: Gcs.setting("udp-host", "127.0.0.1") }
        CfgLabel { text: qsTr("UDP PORT"); topPadding: 7 }
        CfgInput { id: udpPort; text: Gcs.setting("udp-port", "14550") }
    }
    Column {
        visible: panel.type === "mavlink-tcp" || panel.type === "msp-tcp"
        spacing: 3
        CfgLabel { text: qsTr("TCP HOST") }
        CfgInput { id: tcpHost; text: Gcs.setting("tcp-host", "127.0.0.1"); placeholderText: qsTr("Simulator IP (e.g. 192.168.x.x)") }
        CfgLabel { text: qsTr("TCP PORT"); topPadding: 7 }
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
            text: qsTr("Connection failed: ") + Gcs.lastError
            color: Theme.statusRed
            font.family: Theme.fontUi
            font.pixelSize: Theme.fsSm
        }
    }
}
