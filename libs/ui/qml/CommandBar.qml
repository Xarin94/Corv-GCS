// GCS command bar (#gcs-command-bar, CommandBarController.js): three zones.
// Status: heartbeat, link state, signal, battery, GPS, flight time.
// Control: flight mode, pre-arm dot, ARM / TAKEOFF / RTL / AUTO / LAND.
// Info: mission speed with SET.
import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts

Rectangle {
    id: bar
    height: Theme.cmdBarH
    color: Theme.glassToolbar
    Rectangle { anchors.top: parent.top; width: parent.width; height: Theme.hairline; color: Theme.borderAccent }

    readonly property bool linked: Vehicle.connected
    readonly property bool canCommand: Vehicle.connected
    property real flash: 0
    /// ARM / DISARM pressed: Main.qml asks first, with the FORCE variant (ArmDialog)
    signal armRequested()
    Timer { interval: 500; running: true; repeat: true; onTriggered: bar.flash = 1 - bar.flash }

    component Section: Item {
        default property alias content: row.data
        property bool divider: true
        implicitWidth: row.implicitWidth + 14
        height: 33
        Row {
            id: row
            x: 7
            height: parent.height
            spacing: 4
        }
        Rectangle {
            visible: parent.divider
            anchors.right: parent.right
            width: Theme.hairline
            height: parent.height
            color: Theme.borderSubtle
        }
    }
    component Val: Text {
        anchors.verticalCenter: parent ? parent.verticalCenter : undefined
        font.family: Theme.fontData
        font.pixelSize: Theme.fsSm
        font.letterSpacing: 0.04 * Theme.fsSm
        color: Theme.textMain
    }
    component Ico: SvgIcon {
        anchors.verticalCenter: parent ? parent.verticalCenter : undefined
        width: 22
        height: 22
        color: Theme.textDim
    }

    // ── Status zone ──
    Row {
        id: statusZone
        x: 12
        y: 7
        Section {
            Rectangle {
                anchors.verticalCenter: parent.verticalCenter
                width: 8; height: 8; radius: 4
                color: !bar.linked ? Theme.statusRed : Vehicle.heartbeatAlive ? Theme.accentGreen : (bar.flash ? Theme.accentOrange : "transparent")
            }
            Val {
                text: bar.linked ? (Vehicle.heartbeatAlive ? Vehicle.linkType + " " + Vehicle.linkKbps.toFixed(1) + "k" : "NO HEARTBEAT") : "DISCONNECTED"
                color: bar.linked ? (Vehicle.heartbeatAlive ? Theme.accentGreen : Theme.accentOrange) : Theme.statusRed
                font.pixelSize: Theme.fsXs
                font.letterSpacing: 0.08 * Theme.fsXs
            }
            Ico { parts: Icons.signal; color: bar.linked ? Theme.accentCyan : Theme.textDim }
            Val { text: Vehicle.rssi >= 0 ? Vehicle.rssi : "--" }
        }
        Section {
            Ico { parts: Icons.battery; color: Vehicle.batteryRemaining < 0 ? Theme.textDim : Vehicle.batteryRemaining < 20 ? Theme.statusRed : Vehicle.batteryRemaining < 40 ? Theme.accentOrange : Theme.accentGreen }
            Val { text: Vehicle.batteryVoltage.toFixed(1) + "V" }
            Rectangle {
                anchors.verticalCenter: parent.verticalCenter
                width: 45; height: 10; radius: 2
                color: Qt.rgba(1, 1, 1, 0.08)
                border.color: Qt.rgba(1, 1, 1, 0.15)
                border.width: Theme.hairline
                Rectangle {
                    x: 1; y: 1
                    height: parent.height - 2
                    width: Vehicle.batteryRemaining > 0 ? (parent.width - 2) * Vehicle.batteryRemaining / 100 : 0
                    radius: 1
                    color: Vehicle.batteryRemaining < 20 ? Theme.statusRed : Vehicle.batteryRemaining < 40 ? Theme.accentOrange : Theme.accentGreen
                }
            }
            Val { text: Vehicle.batteryRemaining >= 0 ? Vehicle.batteryRemaining + "%" : "--%" }
        }
        Section {
            Ico { parts: Icons.gps; color: Vehicle.gpsStale || !bar.linked ? Theme.textDim : Vehicle.gpsFix >= 3 ? Theme.accentGreen : Vehicle.gpsFix >= 2 ? Theme.accentOrange : Theme.statusRed }
            Val {
                text: !bar.linked || Vehicle.gpsStale ? "No GPS" : Vehicle.gpsFixName
                color: !bar.linked || Vehicle.gpsStale || Vehicle.gpsFix < 2 ? "#ff3333" : Vehicle.gpsFix < 3 ? Theme.accentOrange : Theme.accentGreen
            }
            Val { text: Vehicle.gpsNumSat + " sat"; color: Theme.textDim }
        }
        Section {
            divider: false
            Ico { parts: Icons.clock }
            Val {
                text: Vehicle.flightTime.substring(3)
                color: Vehicle.armed ? Theme.textMain : Theme.textDim
                font.pixelSize: Theme.fsXs
                font.letterSpacing: 0.08 * Theme.fsXs
            }
        }
    }

    // ── Control zone ──
    Item {
        id: controlZone
        x: statusZone.x + statusZone.width + 10
        width: infoZone.x - 10 - x
        height: parent.height
        Rectangle { width: Theme.hairline; height: parent.height; color: Theme.borderAccentMid }
        Rectangle { anchors.right: parent.right; width: Theme.hairline; height: parent.height; color: Theme.borderAccentMid }

        Row {
            x: 8
            y: 7
            height: 33
            spacing: 0
            Section {
                DarkCombo {
                    id: modeBox
                    anchors.verticalCenter: parent.verticalCenter
                    width: 102
                    height: 22
                    enabled: bar.canCommand
                    model: bar.canCommand ? Vehicle.availableModes : []
                    displayText: bar.linked ? Vehicle.flightMode : "--"
                    onActivated: (i) => Gcs.setMode(model[i])
                    font.family: Theme.fontUi
                    font.pixelSize: Theme.fsSm
                    font.weight: Font.Bold
                    itemHeight: 22
                    popupMinWidth: 130
                    popupMaxHeight: 420
                    contentItem: Text {
                        leftPadding: 8
                        text: modeBox.displayText
                        font: modeBox.font
                        color: Theme.modeColor(Vehicle.modeClass)
                        verticalAlignment: Text.AlignVCenter
                        elide: Text.ElideRight
                    }
                    indicator: SvgIcon {
                        x: modeBox.width - width - 6
                        anchors.verticalCenter: parent.verticalCenter
                        width: 12; height: 12
                        parts: [{ d: "M5 9l7 7 7-7", stroke: 2.4, cap: "round" }]
                        color: Theme.modeColor(Vehicle.modeClass)
                    }
                    background: Rectangle {
                        radius: 3
                        color: Theme.cmdSelect
                        border.width: Theme.hairline
                        border.color: Qt.rgba(Theme.modeColor(Vehicle.modeClass).r, Theme.modeColor(Vehicle.modeClass).g, Theme.modeColor(Vehicle.modeClass).b, 0.4)
                    }
                }
            }
            Section {
                divider: false
                Rectangle {
                    anchors.verticalCenter: parent.verticalCenter
                    width: 7; height: 7; radius: 3.5
                    color: !bar.linked ? Theme.textDim : (Vehicle.sensorsOk ? Theme.accentGreen : Theme.accentOrange)
                }
                Item { width: 4; height: 1 }
                CmdButton {
                    anchors.verticalCenter: parent.verticalCenter
                    text: Vehicle.armed ? "DISARM" : "ARM"
                    accent: Vehicle.armed ? Theme.accentRed : Theme.accentGreen
                    bold: true
                    tooltip: "ARM / DISARM"
                    onClicked: if (bar.canCommand) bar.armRequested()
                }
                CmdButton { anchors.verticalCenter: parent.verticalCenter; text: "TAKEOFF"; tooltip: "Takeoff"; onClicked: if (bar.canCommand) Gcs.takeoff(10) }
                CmdButton { anchors.verticalCenter: parent.verticalCenter; text: "RTL"; accent: Theme.accentActive; tooltip: "Return to Launch"; onClicked: if (bar.canCommand) Gcs.returnToLaunch() }
                CmdButton { anchors.verticalCenter: parent.verticalCenter; text: "AUTO"; accent: Theme.accentGreen; tooltip: "Start AUTO mission"; onClicked: if (bar.canCommand) Gcs.setMode("AUTO") }
                CmdButton { anchors.verticalCenter: parent.verticalCenter; text: "LAND"; accent: Theme.accentRed; tooltip: "Emergency Land"; onClicked: if (bar.canCommand) Gcs.land() }
            }
        }
    }

    // ── Info zone ──
    Row {
        id: infoZone
        anchors.right: parent.right
        anchors.rightMargin: 11
        y: 7
        height: 33
        spacing: 4
        Text {
            anchors.verticalCenter: parent.verticalCenter
            text: "SPD"
            color: Theme.textDim
            font.family: Theme.fontUi
            font.pixelSize: Theme.fs2xs
            font.weight: Font.DemiBold
            font.letterSpacing: 0.1 * Theme.fs2xs
        }
        TextField {
            id: speed
            anchors.verticalCenter: parent.verticalCenter
            width: 48
            height: 18
            padding: 0
            leftPadding: 4
            rightPadding: 4
            horizontalAlignment: Text.AlignHCenter
            text: "10"
            color: Theme.textMain
            font.family: Theme.fontData
            font.pixelSize: Theme.fsXs
            validator: DoubleValidator { bottom: 0; top: 30 }
            background: Rectangle {
                radius: 2
                color: Qt.rgba(1, 1, 1, 0.08)
                border.color: Qt.rgba(1, 1, 1, 0.15)
                border.width: Theme.hairline
            }
        }
        Text {
            anchors.verticalCenter: parent.verticalCenter
            text: "m/s"
            color: Theme.textDim
            font.family: Theme.fontUi
            font.pixelSize: Theme.fs2xs
            font.weight: Font.DemiBold
            font.letterSpacing: 0.1 * Theme.fs2xs
        }
        CmdButton {
            anchors.verticalCenter: parent.verticalCenter
            text: "SET"
            implicitHeight: 16
            fontSize: Theme.fs2xs
            implicitWidth: 26
            tooltip: "Send speed command"
            onClicked: if (bar.canCommand) Gcs.setMissionSpeed(parseFloat(speed.text))
        }
    }
}
