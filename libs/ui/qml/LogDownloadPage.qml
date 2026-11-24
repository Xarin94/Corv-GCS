// SETUP → TOOLS → LOG DOWNLOAD: the DataFlash logs on the vehicle's SD card
// (Mission Planner's DataFlash Logs → Download DataFlash Log Via Mavlink):
// list, tick, download to a folder, open in LOG REPLAY, erase the card.
// Logic in LogsController ("Logs").
import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Dialogs
import Corv.Ui

Rectangle {
    id: page
    radius: 6
    color: Theme.glassPanel
    border.color: Theme.borderPanel
    border.width: Theme.hairline

    // A check that only reports the click: the model holds the state
    component Tick: Rectangle {
        property bool on: false
        property bool partial: false
        signal clicked()
        width: 14; height: 14; radius: 2
        color: on || partial ? "#75cffa" : Qt.rgba(1, 1, 1, 0.06)
        border.color: on || partial ? "#75cffa" : "#8a96a0"
        border.width: on || partial ? 0 : 1
        SvgIcon {
            anchors.centerIn: parent
            width: 12; height: 12
            visible: parent.on || parent.partial
            parts: [{ d: parent.on ? "M5 12.5l4.2 4.2L19 7" : "M6 12h12", stroke: 3, cap: "round" }]
            color: "#3b3b3b"
        }
        MouseArea { anchors.fill: parent; anchors.margins: -6; cursorShape: Qt.PointingHandCursor; onClicked: parent.clicked() }
    }

    Row {
        x: 17; y: 17
        spacing: 12
        Text {
            id: pageTitle
            text: qsTr("LOG DOWNLOAD")
            color: Theme.accentCyan
            font.family: Theme.fontUi; font.pixelSize: 12; font.weight: Font.Bold; font.letterSpacing: 2
        }
        Text {
            anchors.baseline: pageTitle.baseline
            text: qsTr("DataFlash logs on the vehicle's SD card · MAVLink LOG_REQUEST_DATA")
            color: Theme.textDim
            font.family: Theme.fontUi; font.pixelSize: 11
        }
    }
    Rectangle { x: 17; y: 43; width: parent.width - 34; height: Theme.hairline; color: Theme.borderTab }

    Item {
        id: body
        x: 17; y: 55
        width: parent.width - 34
        height: parent.height - 72

        Flow {
            id: toolbar
            width: parent.width
            spacing: 10
            GcsButton {
                height: 30
                text: Logs.listing ? "LISTING…" : "LIST LOGS"
                enabled: Logs.linkReady && !Logs.downloading && !Logs.listing
                tip: "Ask the vehicle for the logs on its SD card"
                onClicked: Logs.refresh()
            }
            GcsButton {
                height: 30
                primary: true
                text: Logs.selectedCount > 0 ? "DOWNLOAD SELECTED (" + Logs.selectedCount + " · " + Logs.selectedSize + ")" : "DOWNLOAD SELECTED"
                enabled: Logs.linkReady && !Logs.downloading && Logs.selectedCount > 0
                onClicked: Logs.downloadSelected()
            }
            GcsButton {
                height: 30
                text: "CANCEL"
                visible: Logs.downloading || Logs.listing
                onClicked: Logs.cancel()
            }
            GcsButton {
                height: 30
                text: "ERASE ALL LOGS"
                enabled: Logs.linkReady && !Logs.downloading && !Logs.listing
                frame: Qt.rgba(1, 68 / 255, 68 / 255, 0.45)
                tip: "Wipe every log from the vehicle's SD card (asks first)"
                onClicked: Logs.eraseAll()
            }
            Item { width: 12; height: 30 }
            Row {
                height: 30
                spacing: 8
                Text {
                    anchors.verticalCenter: parent.verticalCenter
                    text: qsTr("FOLDER")
                    color: Theme.textDim
                    font.family: Theme.fontUi; font.pixelSize: 10; font.weight: Font.Bold; font.letterSpacing: 1.5
                }
                Text {
                    anchors.verticalCenter: parent.verticalCenter
                    width: Math.min(implicitWidth, Math.max(160, body.width - 900))
                    text: Logs.folder
                    elide: Text.ElideMiddle
                    color: Theme.textMutedBright
                    font.family: Theme.fontData; font.pixelSize: 10
                }
                GcsButton { anchors.verticalCenter: parent.verticalCenter; height: 26; text: "CHANGE"; onClicked: folderDialog.open() }
                GcsButton { anchors.verticalCenter: parent.verticalCenter; height: 26; text: "OPEN"; onClicked: Logs.openFolder() }
            }
        }

        // Status: what is happening, the progress of the queue, a warning when armed
        Column {
            id: status
            y: toolbar.height + 10
            width: parent.width
            spacing: 6
            Row {
                spacing: 10
                height: 18
                Rectangle {
                    visible: Logs.downloading
                    anchors.verticalCenter: parent.verticalCenter
                    width: 160; height: 6; radius: 3
                    color: Qt.rgba(1, 1, 1, 0.08)
                    Rectangle { width: parent.width * Logs.queueProgress; height: parent.height; radius: 3; color: Theme.accentCyan }
                }
                Text {
                    anchors.verticalCenter: parent.verticalCenter
                    text: Theme.tr(Logs.statusText)
                    color: Logs.statusError ? "#ff8844" : Logs.downloading || Logs.listing ? Theme.accentCyan : Theme.textMuted
                    font.family: Theme.fontData; font.pixelSize: 11
                }
            }
            Text {
                visible: Logs.armed
                text: qsTr("Vehicle armed: ArduPilot lists and sends logs only when disarmed.")
                color: Theme.accentOrange
                font.family: Theme.fontUi; font.pixelSize: 12; font.weight: Font.DemiBold
            }
        }

        Rectangle {
            id: frame
            y: status.y + status.height + 10
            width: parent.width
            height: parent.height - y
            color: "transparent"
            border.color: Qt.rgba(1, 1, 1, 0.06)
            border.width: Theme.hairline
            radius: 4
            clip: true

            readonly property real tickW: 40
            readonly property real idW: 70
            readonly property real dateW: 200
            readonly property real sizeW: 110
            readonly property real actionsW: 210
            readonly property real stateW: width - tickW - idW - dateW - sizeW - actionsW

            Rectangle {
                id: head
                width: parent.width; height: 30
                color: Theme.glassToolbar
                Tick {
                    x: 14
                    anchors.verticalCenter: parent.verticalCenter
                    on: Logs.count > 0 && Logs.selectedCount === Logs.count
                    partial: Logs.selectedCount > 0 && Logs.selectedCount < Logs.count
                    enabled: !Logs.downloading
                    onClicked: Logs.selectAll(Logs.selectedCount < Logs.count)
                }
                Row {
                    x: frame.tickW
                    height: parent.height
                    Repeater {
                        model: [["#", frame.idW], ["DATE", frame.dateW], ["SIZE", frame.sizeW], ["STATUS", frame.stateW]]
                        delegate: Text {
                            required property var modelData
                            width: modelData[1]; height: 30
                            leftPadding: 8
                            verticalAlignment: Text.AlignVCenter
                            text: Theme.tr(modelData[0])
                            color: Theme.accentCyan
                            font.family: Theme.fontUi; font.pixelSize: 10; font.weight: Font.Bold; font.letterSpacing: 1.5
                        }
                    }
                }
            }

            ListView {
                id: list
                y: head.height
                width: parent.width
                height: parent.height - head.height
                clip: true
                model: Logs.entries
                boundsBehavior: Flickable.StopAtBounds
                ScrollBar.vertical: ScrollBar { }
                delegate: Rectangle {
                    id: row
                    required property int logId
                    required property string date
                    required property string sizeText
                    required property bool selected
                    required property string logState
                    required property double progress
                    required property string path
                    required property string detail
                    width: list.width
                    height: 32
                    color: selected ? Qt.rgba(0, 210 / 255, 1, 0.07) : (hover.hovered ? Qt.rgba(0, 210 / 255, 1, 0.04) : "transparent")
                    HoverHandler { id: hover }
                    MouseArea { anchors.fill: parent; enabled: !Logs.downloading; onClicked: Logs.toggle(row.logId) }
                    Rectangle { anchors.bottom: parent.bottom; width: parent.width; height: Theme.hairline; color: Qt.rgba(1, 1, 1, 0.04) }
                    Tick {
                        x: 14
                        anchors.verticalCenter: parent.verticalCenter
                        on: row.selected
                        enabled: !Logs.downloading
                        onClicked: Logs.toggle(row.logId)
                    }
                    Text {
                        x: frame.tickW; width: frame.idW; height: parent.height
                        leftPadding: 8
                        verticalAlignment: Text.AlignVCenter
                        text: row.logId
                        color: Theme.textMain
                        font.family: Theme.fontData; font.pixelSize: 11
                    }
                    Text {
                        x: frame.tickW + frame.idW; width: frame.dateW; height: parent.height
                        leftPadding: 8
                        verticalAlignment: Text.AlignVCenter
                        text: row.date
                        color: row.date === "no date" ? Theme.textDim : Theme.textMain
                        font.family: Theme.fontData; font.pixelSize: 11
                    }
                    Text {
                        x: frame.tickW + frame.idW + frame.dateW; width: frame.sizeW; height: parent.height
                        leftPadding: 8
                        verticalAlignment: Text.AlignVCenter
                        text: row.sizeText
                        color: Theme.accentCyan
                        font.family: Theme.fontData; font.pixelSize: 11
                    }
                    Row {
                        x: frame.tickW + frame.idW + frame.dateW + frame.sizeW + 8
                        width: frame.stateW - 16
                        height: parent.height
                        spacing: 10
                        Rectangle {
                            visible: row.logState === "downloading" || row.logState === "queued"
                            anchors.verticalCenter: parent.verticalCenter
                            width: 120; height: 6; radius: 3
                            color: Qt.rgba(1, 1, 1, 0.08)
                            Rectangle { width: parent.width * row.progress; height: parent.height; radius: 3; color: Theme.accentCyan }
                        }
                        Text {
                            anchors.verticalCenter: parent.verticalCenter
                            text: row.logState === "saved" ? qsTr("SAVED") : row.logState === "failed" ? qsTr("FAILED") : row.logState === "queued" ? qsTr("QUEUED")
                                : row.logState === "downloading" ? Math.round(row.progress * 100) + " %" : ""
                            color: row.logState === "saved" ? Theme.accentGreen : row.logState === "failed" ? "#ff6666" : Theme.accentCyan
                            font.family: Theme.fontUi; font.pixelSize: 11; font.weight: Font.Bold; font.letterSpacing: 1
                        }
                        Text {
                            anchors.verticalCenter: parent.verticalCenter
                            width: parent.width - x
                            text: Theme.tr(row.detail)
                            elide: Text.ElideMiddle
                            color: row.logState === "failed" ? "#ff8888" : Theme.textDim
                            font.family: Theme.fontData; font.pixelSize: 10
                        }
                    }
                    Row {
                        anchors.right: parent.right; anchors.rightMargin: 10
                        anchors.verticalCenter: parent.verticalCenter
                        spacing: 6
                        GcsButton {
                            height: 24
                            text: row.logState === "saved" ? "AGAIN" : "DOWNLOAD"
                            visible: row.logState !== "downloading" && row.logState !== "queued"
                            enabled: Logs.linkReady && !Logs.downloading
                            onClicked: Logs.download(row.logId)
                        }
                        GcsButton {
                            height: 24
                            text: "REPLAY"
                            visible: row.logState === "saved"
                            enabled: Replay.sectionVisible
                            tip: Replay.sectionVisible ? "Open in LOG REPLAY (FLIGHT DATA)" : "Disconnect first: LOG REPLAY needs no live link"
                            onClicked: Logs.replay(row.logId)
                        }
                    }
                }
                Text {
                    visible: list.count === 0
                    anchors.centerIn: parent
                    horizontalAlignment: Text.AlignHCenter
                    text: Theme.tr(Logs.listing ? "Reading the list…"
                        : Logs.linkReady ? "LIST LOGS asks the vehicle for the logs on its SD card.\nArduPilot answers only while disarmed."
                        : "Connect to a vehicle to list its logs.")
                    color: Theme.textDim; font.family: Theme.fontUi; font.pixelSize: 12
                }
            }
        }
    }

    FolderDialog {
        id: folderDialog
        title: "Folder for the downloaded logs"
        currentFolder: Logs.folderUrl
        onAccepted: Logs.setFolder(selectedFolder)
    }
}
