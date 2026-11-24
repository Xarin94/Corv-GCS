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
            text: Theme.tr(sh.title)
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
            text: Theme.tr(gb.text)
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
        Text { text: Theme.tr(parent.k); color: Theme.textDim; font.family: Theme.fontUi; font.pixelSize: Theme.fsSm; font.letterSpacing: 1; anchors.verticalCenter: parent.verticalCenter }
        Text { anchors.right: parent.right; text: Theme.tr(parent.v); color: Theme.textMain; font.family: Theme.fontData; font.pixelSize: Theme.fsSm; anchors.verticalCenter: parent.verticalCenter }
    }
    component Check: Item {
        id: ck
        property string text
        property bool checked: false
        property string tip: ""
        signal toggled(bool on)
        width: parent ? parent.width : 206
        height: 27
        HoverTip { text: ck.tip }
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
            text: Theme.tr(ck.text)
            color: Theme.textMain
            font.family: Theme.fontUi
            font.pixelSize: Theme.fsMd
        }
        MouseArea { anchors.fill: parent; onClicked: { ck.checked = !ck.checked; ck.toggled(ck.checked) } }
    }
    component Field: Item {
        id: fld
        property string k
        property alias text: f.text
        property alias placeholder: f.placeholderText
        property string tip: ""
        property real fieldWidth: 70
        width: parent ? parent.width : 206
        height: 21
        opacity: enabled ? 1 : 0.4
        Text { text: Theme.tr(parent.k); color: Theme.textDim; font.family: Theme.fontUi; font.pixelSize: Theme.fsSm; anchors.verticalCenter: parent.verticalCenter }
        TextField {
            id: f
            anchors.right: parent.right
            width: fld.fieldWidth; height: 21
            selectByMouse: true
            placeholderTextColor: Theme.textDim
            HoverHandler { id: fh }
            ToolTip.visible: fh.hovered && fld.tip.length > 0
            ToolTip.delay: 600
            ToolTip.text: Theme.tr(fld.tip)
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
                Kv { k: "SYS ID"; v: Vehicle.connected ? Vehicle.systemId : "--" }
                Kv { k: "FW"; v: Vehicle.connected && Vehicle.firmwareVersion.length > 0 ? Vehicle.firmwareVersion : "--" }
            }
            Section {
                title: "ACTIONS"
                GcsButton { text: "SET HOME"; onClicked: Gcs.setHomeHere() }
                GcsButton { text: "REBOOT FC"; enabled: !Vehicle.armed; opacity: enabled ? 1 : 0.3; onClicked: Gcs.rebootAutopilot() }
            }
            Section {
                title: "MISSION"
                // The vehicle's mission into FLIGHT PLAN, and so on the maps and in 3D (a route there is asked about first)
                GcsButton { text: Plan.readText === "READ" ? "READ MISSION" : Plan.readText; enabled: Vehicle.connected; opacity: enabled ? 1 : 0.4; onClicked: Plan.read() }
                Row {
                    spacing: 6
                    Text { text: qsTr("WPT:"); color: Theme.textDim; font.family: Theme.fontUi; font.pixelSize: Theme.fsSm }
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
                    tip: "LTE cell rows in the position panel, on the cellular link"
                    checked: !!Lte.cellMap.table
                    onToggled: (on) => Lte.setCellMap("table", on)
                }
                Check {
                    text: "RESTRICTED ZONES"
                    tip: "UAS geographical zones within 50 km of the vehicle and the mission, on the maps and the 3D terrain, with height warnings (120 m in Europe, 400 ft in the US)"
                    checked: GeoZones.enabled
                    onToggled: (on) => GeoZones.enabled = on
                }
                GcsButton { text: "CLEAR TRAIL"; onClicked: { side.clearTrail(); Gcs.notify("Trail cleared") } }
            }
            Section {
                title: "RTL OPTIONS"
                // A field the vehicle has no parameter for stays greyed; the tip names the parameter
                Repeater {
                    id: rtlFields
                    model: Rtl.fields
                    delegate: Column {
                        id: rf
                        required property var modelData
                        required property int index
                        readonly property string key: modelData.key
                        property alias text: rtlField.text
                        width: parent ? parent.width : 206
                        spacing: 6
                        Text { visible: rf.index === 3; text: qsTr("SMART RTL"); color: Theme.accentCyan; font.family: Theme.fontUi; font.pixelSize: Theme.fsSm; topPadding: 6 }
                        Field { id: rtlField; k: rf.modelData.label; text: rf.modelData.text; tip: rf.modelData.param; enabled: rf.modelData.available }
                    }
                }
                GcsButton { text: Rtl.reading ? "READING..." : "READ FROM VEHICLE"; enabled: Vehicle.connected && !Rtl.reading; opacity: enabled ? 1 : 0.4; onClicked: Rtl.read() }
                GcsButton {
                    text: "WRITE TO VEHICLE"
                    enabled: Vehicle.connected
                    opacity: enabled ? 1 : 0.4
                    onClicked: {
                        const v = {}
                        for (let i = 0; i < rtlFields.count; ++i) v[rtlFields.itemAt(i).key] = rtlFields.itemAt(i).text
                        Rtl.write(v)
                    }
                }
            }
            Section {
                title: "TARGET"
                Field { id: targetLat; k: "LAT"; fieldWidth: 110; placeholder: "47.2603" }
                Field { id: targetLon; k: "LON"; fieldWidth: 110; placeholder: "11.3439" }
                GcsButton { text: "SET TARGET"; onClicked: Target.set(targetLat.text, targetLon.text) }
                GcsButton { text: "CLEAR"; enabled: Target.active; opacity: enabled ? 1 : 0.4; onClicked: Target.clear() }
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
                    // Selectable; it follows the newest message unless it was scrolled up to read
                    Flickable {
                        id: logView
                        anchors.fill: parent
                        anchors.margins: 5
                        contentWidth: width
                        contentHeight: log.implicitHeight
                        clip: true
                        boundsBehavior: Flickable.StopAtBounds
                        property bool atEnd: true
                        onMovementEnded: atEnd = contentY >= contentHeight - height - 4
                        onContentHeightChanged: if (atEnd) contentY = Math.max(0, contentHeight - height)
                        onHeightChanged: if (atEnd) contentY = Math.max(0, contentHeight - height)
                        ScrollBar.vertical: ScrollBar { width: 6; onPressedChanged: if (!pressed) logView.atEnd = logView.contentY >= logView.contentHeight - logView.height - 4 }
                        TextEdit {
                            id: log
                            objectName: "messageLog"
                            width: logView.width - 8
                            text: Gcs.messageLog
                            readOnly: true
                            selectByMouse: true
                            selectionColor: Theme.accentCyan
                            selectedTextColor: "black"
                            wrapMode: TextEdit.Wrap
                            textFormat: TextEdit.PlainText
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
                                Text { text: Theme.tr(parent.modelData[0]); color: Theme.textDim; font.family: Theme.fontData; font.pixelSize: 10; font.weight: Font.DemiBold; font.letterSpacing: 1 }
                                Text {
                                    anchors.right: parent.right
                                    width: Math.min(implicitWidth, 130)
                                    horizontalAlignment: Text.AlignRight
                                    elide: Text.ElideMiddle
                                    text: Theme.tr(parent.modelData[1])
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
