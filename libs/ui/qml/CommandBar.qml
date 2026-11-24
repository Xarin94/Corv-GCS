// GCS command bar (#gcs-command-bar, CommandBarController.js): three zones.
// Status: heartbeat, link state, signal, battery, GPS, flight time, target.
// Control: flight mode, pre-arm dot, ARM / TAKEOFF / RTL / AUTO / FBWA or POSHOLD / LAND.
// Info: mission speed with SET, distance home and mission progress.
// As the window narrows the least needed readouts leave the bar one at a time.
import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts

Rectangle {
    id: bar
    height: Theme.cmdBarH
    color: Theme.glassToolbar
    Rectangle { anchors.top: parent.top; width: parent.width; height: Theme.hairline; color: Theme.borderAccent }

    readonly property bool linked: Vehicle.connected
    /// The state shown is a vehicle's: connected, or a log's in LOG REPLAY
    readonly property bool hasState: Vehicle.connected || Vehicle.mode === "REPLAY"
    readonly property bool hasGps: hasState && !Vehicle.gpsStale
    readonly property bool canCommand: Vehicle.connected
    readonly property bool lte: Vehicle.connectionType === "mavlink-lte"
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
    // Five bars, lit by the signal's level (updateSignalBars)
    component SignalBars: Item {
        id: sig
        property int bars: -1
        readonly property color tone: bars < 0 ? Theme.textDim : bars >= 4 ? Theme.accentGreen : bars >= 2 ? Theme.accentOrange : Theme.accentRed
        anchors.verticalCenter: parent ? parent.verticalCenter : undefined
        width: 22
        height: 22
        Repeater {
            model: 5
            delegate: Rectangle {
                required property int index
                readonly property bool on: index < sig.bars
                x: (0.6 + 4.8 * index) * 22 / 24
                width: 3.6 * 22 / 24
                height: (4 + 4 * index) * 22 / 24
                y: 22 * 22 / 24 - height
                radius: 0.6
                color: on ? sig.tone : Theme.textDim
                opacity: on ? 1 : 0.3
                Behavior on color { ColorAnimation { duration: 300 } }
                Behavior on opacity { NumberAnimation { duration: 300 } }
            }
        }
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
                HoverTip { text: "Heartbeat" }
            }
            Val {
                text: bar.linked ? (Vehicle.heartbeatAlive ? Vehicle.linkType + " " + (Vehicle.linkKbps >= 100 ? Math.round(Vehicle.linkKbps) : Vehicle.linkKbps.toFixed(1)) + (Theme.winW > 1300 ? " kbps" : "k") : qsTr("NO HEARTBEAT")) : qsTr("DISCONNECTED")
                color: bar.linked ? (Vehicle.heartbeatAlive ? Theme.accentGreen : Theme.accentOrange) : Theme.statusRed
                font.pixelSize: Theme.fsXs
                font.letterSpacing: 0.08 * Theme.fsXs
            }
            SignalBars { bars: bar.linked ? Vehicle.signalBars : -1 }
            // The radio's RSSI (the LTE module reports its cell's on the same scale: dBm), or the link quality
            Val {
                text: !bar.linked ? "--" : Vehicle.rssi >= 0 ? (bar.lte ? Math.round(Vehicle.rssiDbm) + " dBm" : Vehicle.rssi)
                                         : Vehicle.linkQuality > 0 ? Math.round(Vehicle.linkQuality) + "%" : "--"
            }
            Val { visible: bar.linked && Vehicle.rssi >= 0 && Vehicle.remRssi >= 0; text: "R:" + Vehicle.remRssi; color: Theme.textDim }
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
            // RTK fixed green, RTK float yellow, 3D / DGPS cyan, less red
            readonly property color fixColor: !bar.hasGps || Vehicle.gpsFix < 3 ? "#ff3333" : Vehicle.gpsFix === 6 ? "#00ff7f" : Vehicle.gpsFix === 5 ? "#ffcc00" : "#00d2ff"
            id: gpsSection
            Ico { parts: Icons.gps; color: bar.hasGps ? gpsSection.fixColor : Theme.textDim }
            Val { text: Theme.tr(!bar.hasGps ? "No GPS" : Vehicle.gpsFixName); color: gpsSection.fixColor }
            Val { text: (bar.hasGps ? Vehicle.gpsNumSat : 0) + qsTr(" sat"); color: Theme.textDim }
            Val { visible: bar.hasGps && Vehicle.gpsHdop < 99 && Theme.winW > 1400; text: "HDOP " + Vehicle.gpsHdop.toFixed(1); color: Theme.textDim }
        }
        Section {
            divider: Target.active
            Ico { parts: Icons.clock }
            Val {
                text: Vehicle.flightTime.substring(3)
                color: Vehicle.armed ? Theme.accentCyan : Theme.textDim
                font.pixelSize: Theme.fsXs
                font.letterSpacing: 0.08 * Theme.fsXs
            }
        }
        // Target (GCS OPTIONS): the way to it from the nose, and how far
        Section {
            objectName: "cmdTarget"
            divider: false
            visible: Target.active
            SvgIcon {
                anchors.verticalCenter: parent.verticalCenter
                width: 20
                height: 20
                parts: [{ d: "M12 2 L18 18 L12 14 L6 18 Z", fill: true }]
                color: "#ff4444"
                rotation: Target.relativeBearing
            }
            Val { text: Target.distance >= 1000 ? (Target.distance / 1000).toFixed(1) + " km" : Math.round(Target.distance) + " m" }
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
                    HoverTip { text: "Flight mode" }
                    model: bar.canCommand ? Vehicle.availableModes : []
                    displayText: bar.hasState ? Vehicle.flightMode : "--"
                    onActivated: (i) => Gcs.setMode(model[i])
                    font.family: Theme.fontUi
                    font.pixelSize: Theme.fsSm
                    font.weight: Font.Bold
                    itemHeight: 22
                    popupMinWidth: 130
                    popupMaxHeight: 420
                    contentItem: Text {
                        leftPadding: 8
                        text: Theme.tr(modeBox.displayText)
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
                    color: !bar.linked || Vehicle.preArm === 0 ? Theme.textDim : Vehicle.preArm === 1 ? Theme.accentGreen : Theme.accentRed
                    opacity: bar.linked && Vehicle.preArm === 2 && bar.flash ? 0.4 : 1
                    Behavior on opacity { NumberAnimation { duration: 400 } }
                    HoverTip {
                        text: !bar.linked || Vehicle.preArm === 0 ? "Pre-arm: Unknown" : Vehicle.preArm === 1 ? "Pre-arm: OK"
                              : Vehicle.preArmText.length > 0 ? Vehicle.preArmText : "Pre-arm: Check failed"
                    }
                }
                Item { width: 4; height: 1 }
                CmdButton {
                    anchors.verticalCenter: parent.verticalCenter
                    text: Vehicle.armed ? "DISARM" : "ARM"
                    accent: Vehicle.armed ? Theme.accentRed : Theme.accentGreen
                    bold: true
                    tooltip: Vehicle.armed ? "DISARM the vehicle (currently ARMED)" : "ARM the vehicle (currently DISARMED)"
                    onClicked: if (bar.canCommand) bar.armRequested()
                }
                CmdButton { anchors.verticalCenter: parent.verticalCenter; text: "TAKEOFF"; tooltip: "Takeoff"; onClicked: if (bar.canCommand) Gcs.askTakeoff() }
                CmdButton { anchors.verticalCenter: parent.verticalCenter; text: "RTL"; accent: Theme.accentActive; tooltip: "Return to Launch"; onClicked: if (bar.canCommand) Gcs.returnToLaunch() }
                CmdButton { anchors.verticalCenter: parent.verticalCenter; text: "AUTO"; accent: Theme.accentGreen; tooltip: "Start AUTO mission"; onClicked: if (bar.canCommand) Gcs.setMode("AUTO") }
                CmdButton { objectName: "cmdFbwa"; visible: bar.canCommand && Vehicle.vehicleTypeName === "Plane"; anchors.verticalCenter: parent.verticalCenter; text: "FBWA"; tooltip: "Fly By Wire A"; onClicked: if (bar.canCommand) Gcs.setMode("FBWA") }
                CmdButton { objectName: "cmdPoshold"; visible: bar.canCommand && Vehicle.vehicleTypeName === "Copter"; anchors.verticalCenter: parent.verticalCenter; text: "POSHOLD"; tooltip: "Position Hold"; onClicked: if (bar.canCommand) Gcs.setMode("POSHOLD") }
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
        // SPD ... SET, the "m/s" saying SET when it went out
        function sendSpeed() {
            const v = parseFloat(speed.text)
            if (!bar.canCommand || !(v > 0)) return
            Gcs.setMissionSpeed(v)
            speedSent.restart()
        }
        Timer { id: speedSent; interval: 1500 }
        Text {
            anchors.verticalCenter: parent.verticalCenter
            text: qsTr("SPD")
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
            onAccepted: infoZone.sendSpeed()
            HoverTip { text: "Mission speed (m/s)" }
            background: Rectangle {
                radius: 2
                color: Qt.rgba(1, 1, 1, 0.08)
                border.color: Qt.rgba(1, 1, 1, 0.15)
                border.width: Theme.hairline
            }
        }
        Text {
            anchors.verticalCenter: parent.verticalCenter
            text: speedSent.running ? Theme.tr("SET") : "m/s"
            color: speedSent.running ? "#00ff88" : Theme.textDim
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
            onClicked: infoZone.sendSpeed()
        }
        // Distance home and mission progress, one over the other (.cmd-nav)
        Item { visible: navGrid.visible; width: 6; height: 1 }
        Rectangle { visible: navGrid.visible; anchors.verticalCenter: parent.verticalCenter; width: Theme.hairline; height: 33; color: Theme.borderSubtle }
        Item { visible: navGrid.visible; width: 4; height: 1 }
        Grid {
            id: navGrid
            objectName: "cmdNav"
            visible: Theme.winW > 1300
            anchors.verticalCenter: parent.verticalCenter
            columns: 2
            columnSpacing: Theme.clamp(4, 0.45 * Theme.vw, 10)
            rowSpacing: 1
            component NavLbl: Text {
                color: Theme.textDim
                font.family: Theme.fontUi
                font.pixelSize: Theme.fs2xs
                font.weight: Font.DemiBold
                font.letterSpacing: 0.1 * Theme.fs2xs
            }
            component NavVal: Text {
                width: 52
                horizontalAlignment: Text.AlignRight
                color: Theme.textNav
                font.family: Theme.fontData
                font.pixelSize: Theme.fsXs
            }
            NavLbl { text: qsTr("Home"); HoverTip { text: "Straight-line distance to home" } }
            NavVal {
                text: !bar.hasState || !Vehicle.homeSet ? "--" : Vehicle.homeDistance >= 1000 ? (Vehicle.homeDistance / 1000).toFixed(2) + "km" : Math.round(Vehicle.homeDistance) + "m"
            }
            NavLbl { text: qsTr("WP"); HoverTip { text: "Active waypoint / mission length" } }
            NavVal { text: bar.hasState && Vehicle.missionCount > 0 ? Vehicle.missionSeq + "/" + Vehicle.missionCount : "--" }
        }
    }
}
