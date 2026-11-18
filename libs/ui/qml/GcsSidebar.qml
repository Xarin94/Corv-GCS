// GCS sidebar (gcs-sidebar.css, GCSSidebarController.js): 240 px on the right,
// collapsible sections. Opened from the GCS OPTIONS tab on the right edge.
import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Dialogs
import Corv.Ui

Rectangle {
    id: side
    width: 241
    color: Theme.glassToolbar
    signal clearTrail()
    Rectangle { width: Theme.hairline; height: parent.height; color: Theme.borderAccent }

    component SectionHeader: Item {
        id: sh
        property string title
        property bool open: true
        width: parent.width
        height: 30
        Text {
            x: 14
            anchors.verticalCenter: parent.verticalCenter
            text: sh.title
            color: Theme.accentCyan
            font.family: Theme.fontUi
            font.pixelSize: Theme.fsMd
            font.weight: Font.Bold
            font.letterSpacing: 1.5
        }
        Text {
            anchors.right: parent.right
            anchors.rightMargin: 15
            anchors.verticalCenter: parent.verticalCenter
            text: "▼"
            rotation: sh.open ? 0 : -90
            color: Theme.textDim
            font.pixelSize: 8
        }
        MouseArea { anchors.fill: parent; onClicked: sh.open = !sh.open }
    }
    component Section: Column {
        id: sec
        property string title
        default property alias body: inner.data
        width: parent ? parent.width : 230
        SectionHeader { id: hdr; title: sec.title }
        Column {
            id: inner
            visible: hdr.open
            x: 12
            width: parent.width - 24
            topPadding: 6
            bottomPadding: 6
            spacing: 6
        }
        Rectangle { width: parent.width; height: Theme.hairline; color: Theme.borderSection }
    }
    component GcsButton: Rectangle {
        id: gb
        property string text
        property color accent: "white"
        signal clicked()
        width: parent ? parent.width : 206
        height: 29
        radius: 3
        color: gm.containsMouse ? Theme.gcsButtonHover : Theme.gcsButton
        border.width: Theme.hairline
        border.color: Theme.borderInput
        Text {
            anchors.centerIn: parent
            text: gb.text
            color: gb.accent
            font.family: Theme.fontUi
            font.pixelSize: Theme.fsMd
            font.weight: Font.DemiBold
            font.letterSpacing: 0.8
        }
        MouseArea { id: gm; anchors.fill: parent; hoverEnabled: true; onClicked: gb.clicked() }
    }
    component Kv: Item {
        property string k
        property string v
        width: parent ? parent.width : 206
        height: 17
        Text { text: parent.k; color: Theme.textDim; font.family: Theme.fontUi; font.pixelSize: Theme.fsSm; font.letterSpacing: 1; anchors.verticalCenter: parent.verticalCenter }
        Text { anchors.right: parent.right; text: parent.v; color: Theme.textMain; font.family: Theme.fontData; font.pixelSize: Theme.fsSm; anchors.verticalCenter: parent.verticalCenter }
    }
    component Check: Item {
        id: ck
        property string text
        property bool checked: false
        signal toggled(bool on)
        width: parent ? parent.width : 206
        height: 27
        Rectangle {
            x: 4
            anchors.verticalCenter: parent.verticalCenter
            width: 13; height: 13; radius: 2
            color: ck.checked ? "#2f8fd8" : "white"
            border.color: ck.checked ? "#2f8fd8" : "#767676"
            border.width: 1
            Text { anchors.centerIn: parent; visible: ck.checked; text: "✓"; color: "white"; font.pixelSize: 11; font.bold: true }
        }
        Text {
            x: 26
            anchors.verticalCenter: parent.verticalCenter
            text: ck.text
            color: Theme.textMain
            font.family: Theme.fontUi
            font.pixelSize: Theme.fsMd
        }
        MouseArea { anchors.fill: parent; onClicked: { ck.checked = !ck.checked; ck.toggled(ck.checked) } }
    }
    component Field: Item {
        property string k
        property alias text: f.text
        width: parent ? parent.width : 206
        height: 21
        Text { text: parent.k; color: Theme.textDim; font.family: Theme.fontUi; font.pixelSize: Theme.fsSm; anchors.verticalCenter: parent.verticalCenter }
        TextField {
            id: f
            anchors.right: parent.right
            width: 70; height: 21
            padding: 2; rightPadding: 6
            horizontalAlignment: Text.AlignRight
            color: Theme.textMain
            font.family: Theme.fontData
            font.pixelSize: Theme.fsMd
            background: Rectangle { radius: 3; color: Theme.inputBg; border.color: Theme.borderInput; border.width: Theme.hairline }
        }
    }

    Flickable {
        anchors.fill: parent
        anchors.leftMargin: 1
        contentHeight: content.implicitHeight
        clip: true
        ScrollBar.vertical: ScrollBar { width: 6 }
        Column {
            id: content
            width: 230
            topPadding: 8

            Section {
                title: "VEHICLE"
                Kv { k: "TYPE"; v: Vehicle.connected ? Vehicle.vehicleTypeName : "--" }
                Kv { k: "SYS ID"; v: Vehicle.systemId }
                Kv { k: "FW"; v: "--" }
            }
            Section {
                title: "ACTIONS"
                GcsButton { text: "SET HOME"; onClicked: Gcs.setHomeHere() }
                GcsButton { text: "REBOOT FC"; onClicked: Gcs.rebootAutopilot() }
            }
            Section {
                title: "MISSION"
                GcsButton { text: "READ MISSION" }
                Row {
                    spacing: 6
                    Text { text: "WPT:"; color: Theme.textDim; font.family: Theme.fontUi; font.pixelSize: Theme.fsSm }
                    Text { text: Vehicle.missionSeq + "/" + Vehicle.missionCount; color: Theme.textMain; font.family: Theme.fontData; font.pixelSize: Theme.fsMd }
                }
            }
            Section {
                title: "GCS OPTIONS"
                Check { text: "MUTE GCS OUTPUT"; checked: Gcs.gcsMuted; onToggled: (on) => Gcs.gcsMuted = on }
                // Same setting as SYS CONFIG → ADS-B (toggleADSB, adsb-enabled)
                Check {
                    text: "SHOW ADS-B TRAFFIC"
                    checked: SysConfig.adsbEnabled
                    onToggled: (on) => SysConfig.adsbEnabled = on
                }
                // Same setting as SYS CONFIG → ROTOR LOAD (window.toggleRotorLoad)
                Check {
                    id: rotorChk
                    text: "SHOW ROTOR LOAD"
                    checked: SysConfig.jsonField("rotor-load-cfg", "enabled", true)
                    onToggled: (on) => SysConfig.setJsonField("rotor-load-cfg", "enabled", on)
                    Connections {
                        target: SysConfig
                        function onJsonFieldChanged(key) {
                            if (key === "rotor-load-cfg") rotorChk.checked = SysConfig.jsonField("rotor-load-cfg", "enabled", true)
                        }
                    }
                }
                // The cell rows of the position panel (lte-cell-map.table)
                Check {
                    text: "LTE OVERLAY"
                    checked: !!Lte.cellMap.table
                    onToggled: (on) => Lte.setCellMap("table", on)
                }
                Check {
                    text: "RESTRICTED ZONES"
                    checked: GeoZones.enabled
                    onToggled: (on) => GeoZones.enabled = on
                }
                GcsButton { text: "CLEAR TRAIL"; onClicked: side.clearTrail() }
            }
            Section {
                title: "RTL OPTIONS"
                Field { k: "RTL ALT (m)" }
                Field { k: "RTL ALT FINAL (m)" }
                Field { k: "RTL SPEED (m/s)" }
                Text { text: "SMART RTL"; color: Theme.accentCyan; font.family: Theme.fontUi; font.pixelSize: Theme.fsSm; topPadding: 6 }
                Field { k: "SRTL POINTS" }
                Field { k: "SRTL ACCURACY (m)" }
                GcsButton { text: "READ FROM VEHICLE" }
                GcsButton { text: "WRITE TO VEHICLE" }
            }
            Section {
                title: "TARGET"
                Field { k: "LAT" }
                Field { k: "LON" }
                GcsButton { text: "SET TARGET" }
                GcsButton { text: "CLEAR" }
            }
            Section {
                title: "MESSAGE LOG"
                Rectangle {
                    width: parent.width
                    height: 180
                    color: Qt.rgba(5 / 255, 8 / 255, 12 / 255, 0.9)
                    border.color: Theme.borderSection
                    border.width: Theme.hairline
                    radius: 3
                    Flickable {
                        anchors.fill: parent
                        anchors.margins: 5
                        contentHeight: log.implicitHeight
                        clip: true
                        Text {
                            id: log
                            width: parent.width
                            text: Gcs.messageLog
                            wrapMode: Text.Wrap
                            color: Theme.textMain
                            font.family: Theme.fontData
                            font.pixelSize: Theme.fsSm
                        }
                    }
                }
            }
            // Shown only while no live link is up (LogReplayController.js)
            Section {
                title: "LOG REPLAY"
                visible: Replay.sectionVisible
                GcsButton {
                    text: Replay.loading ? "LOADING..." : "OPEN LOG FILE..."
                    enabled: Replay.openEnabled
                    opacity: enabled ? 1 : 0.4
                    onClicked: replayDialog.open()
                }
                Rectangle {
                    visible: Replay.loaded
                    width: parent.width
                    height: info.implicitHeight + 12
                    radius: 3
                    color: Qt.rgba(1, 1, 1, 0.03)
                    border.color: Theme.borderSubtle
                    border.width: Theme.hairline
                    Column {
                        id: info
                        x: 8; y: 6
                        width: parent.width - 16
                        spacing: 4
                        Repeater {
                            model: [["FILE", Replay.fileName], ["FORMAT", Replay.format], ["DUR", Replay.durationText], ["MSGS", String(Replay.messages)]]
                            delegate: Item {
                                required property var modelData
                                width: info.width
                                height: 14
                                Text { text: parent.modelData[0]; color: Theme.textDim; font.family: Theme.fontData; font.pixelSize: 10; font.weight: Font.DemiBold; font.letterSpacing: 1 }
                                Text {
                                    anchors.right: parent.right
                                    width: Math.min(implicitWidth, 130)
                                    horizontalAlignment: Text.AlignRight
                                    elide: Text.ElideMiddle
                                    text: parent.modelData[1]
                                    color: Theme.textMain
                                    font.family: Theme.fontData; font.pixelSize: 10
                                }
                            }
                        }
                        GcsButton { text: "UNLOAD"; onClicked: Replay.unload() }
                    }
                }
                Text {
                    visible: !Replay.loaded
                    width: parent.width
                    wrapMode: Text.Wrap
                    textFormat: Text.StyledText
                    text: "Load an ArduPilot <b>.tlog</b> or <b>.bin</b> to replay a past flight."
                    color: Theme.textDim
                    font.family: Theme.fontUi
                    font.pixelSize: 10
                    lineHeight: 1.3
                }
            }
        }
    }

    FileDialog {
        id: replayDialog
        title: "Open ArduPilot log"
        currentFolder: Replay.logsFolder
        nameFilters: ["ArduPilot logs (*.tlog *.bin)", "All files (*)"]
        onAccepted: Replay.open(selectedFile)
    }
}
