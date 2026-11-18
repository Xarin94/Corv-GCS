// MISSION LIBRARY (#mission-library-page, the .params-page chrome): saved
// missions with LOAD / OVERWRITE / REN / DEL, the data folder, the flight logs.
import QtQuick
import QtQuick.Controls.Basic
import Corv.Ui

Rectangle {
    id: lib
    visible: false
    color: Qt.rgba(4 / 255, 8 / 255, 14 / 255, 0.98)
    MouseArea { anchors.fill: parent; acceptedButtons: Qt.AllButtons; onWheel: (w) => w.accepted = true }

    Connections {
        target: Plan
        function onLibraryOpenChanged(open) { lib.visible = open }
    }
    Shortcut { sequence: "Escape"; enabled: lib.visible; onActivated: lib.visible = false }

    component SmallBtn: Rectangle {
        id: sb
        property string text
        property bool danger: false
        property string tip
        signal clicked()
        width: sbText.implicitWidth + 16
        height: sbText.implicitHeight + 8
        radius: 3
        color: Theme.inputBg
        border.color: sh.hovered ? (danger ? "#ff4444" : Theme.accentCyan) : Theme.borderInput
        border.width: Theme.hairline
        Text {
            id: sbText
            anchors.centerIn: parent
            text: sb.text
            color: sh.hovered ? (sb.danger ? "#ff4444" : Theme.accentCyan) : Theme.textMain
            font.family: Theme.fontUi; font.pixelSize: 9; font.letterSpacing: 0.5
        }
        HoverHandler { id: sh; cursorShape: Qt.PointingHandCursor }
        TapHandler { onTapped: sb.clicked() }
        ToolTip.visible: sh.hovered && tip.length > 0
        ToolTip.text: tip
        ToolTip.delay: 600
    }

    // Header
    Item {
        id: header
        width: parent.width
        height: 64
        Text {
            x: 24
            anchors.verticalCenter: parent.verticalCenter
            text: "MISSION LIBRARY"
            color: Theme.accentCyan
            font.family: Theme.fontUi; font.pixelSize: 14; font.weight: Font.Bold; font.letterSpacing: 3
        }
        Rectangle {
            x: parent.width - 24 - width
            anchors.verticalCenter: parent.verticalCenter
            width: 36; height: 36; radius: 4
            color: "transparent"
            border.color: closeHover.hovered ? Theme.accentCyan : Qt.rgba(1, 1, 1, 0.15)
            border.width: Theme.hairline
            Text { anchors.centerIn: parent; text: "×"; color: closeHover.hovered ? Theme.accentCyan : Theme.textMain; font.pixelSize: 22 }
            HoverHandler { id: closeHover; cursorShape: Qt.PointingHandCursor }
            TapHandler { onTapped: lib.visible = false }
            ToolTip.visible: closeHover.hovered
            ToolTip.text: "Close"
        }
        Rectangle { anchors.bottom: parent.bottom; width: parent.width; height: Theme.hairline; color: Theme.borderAccent }
    }

    // Body: at most 1240 px, centred
    Item {
        id: body
        y: header.height + 16
        width: Math.min(1240, parent.width) - 48
        x: (parent.width - Math.min(1240, parent.width)) / 2 + 24
        height: parent.height - header.height - 32

        Row {
            id: toolbar
            spacing: 8
            GcsButton { height: 27; text: "SAVE AS NEW"; tip: "Save the current plan as a new mission"; onClicked: Plan.saveAsNew() }
            GcsButton { height: 27; text: "OPEN FOLDER"; tip: "Open the data folder"; onClicked: Plan.revealDataFolder() }
            Text {
                anchors.verticalCenter: parent.verticalCenter
                width: body.width - x
                elide: Text.ElideMiddle
                text: Plan.libraryRoot
                color: Plan.libraryPortable ? Theme.textDim : "#ffb300"
                font.family: Theme.fontData; font.pixelSize: 10
                HoverHandler { id: rootHover }
                ToolTip.visible: rootHover.hovered
                ToolTip.text: Plan.libraryPortable ? "Stored inside the installation folder (portable)"
                                                   : "Installation folder is not writable — stored in the user data folder instead"
            }
        }

        Text {
            id: missionsTitle
            y: toolbar.height + 10
            text: "SAVED MISSIONS"
            color: Theme.accentCyan
            font.family: Theme.fontUi; font.pixelSize: 10; font.weight: Font.Bold; font.letterSpacing: 1.5
        }
        Rectangle {
            id: missionsBox
            y: missionsTitle.y + missionsTitle.height + 6
            width: body.width
            height: (body.height - y - 40) * 0.66
            radius: 4
            color: "transparent"
            border.color: Qt.rgba(1, 1, 1, 0.06)
            border.width: Theme.hairline
            clip: true
            Text {
                visible: Plan.libraryMissions.length === 0
                anchors.horizontalCenter: parent.horizontalCenter
                y: 16
                horizontalAlignment: Text.AlignHCenter
                text: "No saved missions yet.\nBuild a plan and press SAVE."
                color: Theme.textDim
                font.family: Theme.fontUi; font.pixelSize: 10
                lineHeight: 1.6
            }
            ListView {
                anchors.fill: parent
                anchors.margins: 1
                model: Plan.libraryMissions
                boundsBehavior: Flickable.StopAtBounds
                ScrollBar.vertical: ScrollBar { }
                delegate: Rectangle {
                    id: row
                    required property var modelData
                    readonly property bool current: modelData.id === Plan.currentMissionId
                    width: ListView.view.width
                    height: Math.max(mainCol.implicitHeight, 24) + 14
                    color: current ? Qt.rgba(0, 210 / 255, 1, 0.09) : rowHover.hovered ? Qt.rgba(0, 210 / 255, 1, 0.05) : "transparent"
                    HoverHandler { id: rowHover }
                    Rectangle { visible: row.current; width: 2; height: parent.height; color: Theme.accentCyan }
                    Rectangle { anchors.bottom: parent.bottom; width: parent.width; height: Theme.hairline; color: Qt.rgba(1, 1, 1, 0.04) }
                    Column {
                        id: mainCol
                        x: 10
                        anchors.verticalCenter: parent.verticalCenter
                        width: actions.x - 20
                        spacing: 2
                        Text { width: parent.width; elide: Text.ElideRight; text: row.modelData.name; color: Theme.textMain; font.family: Theme.fontData; font.pixelSize: 12 }
                        Text { text: row.modelData.meta; color: Theme.textDim; font.family: Theme.fontUi; font.pixelSize: 10 }
                        Text { visible: row.modelData.notes.length > 0; text: row.modelData.notes; color: Theme.textDim; font.family: Theme.fontUi; font.pixelSize: 10 }
                    }
                    Row {
                        id: actions
                        x: parent.width - width - 10
                        anchors.verticalCenter: parent.verticalCenter
                        spacing: 5
                        SmallBtn { text: "LOAD"; tip: "Replace the current plan with this one"; onClicked: Plan.libraryAction("load", row.modelData.id) }
                        SmallBtn { text: "OVERWRITE"; tip: "Overwrite this mission with the current plan"; onClicked: Plan.libraryAction("overwrite", row.modelData.id) }
                        SmallBtn { text: "REN"; tip: "Rename"; onClicked: Plan.libraryAction("rename", row.modelData.id) }
                        SmallBtn { text: "DEL"; danger: true; tip: "Delete"; onClicked: Plan.libraryAction("delete", row.modelData.id) }
                    }
                }
            }
        }

        Text {
            id: logsTitle
            y: missionsBox.y + missionsBox.height + 10
            text: "FLIGHT LOGS"
            color: Theme.accentCyan
            font.family: Theme.fontUi; font.pixelSize: 10; font.weight: Font.Bold; font.letterSpacing: 1.5
        }
        Rectangle {
            y: logsTitle.y + logsTitle.height + 6
            width: body.width
            height: body.height - y
            radius: 4
            color: "transparent"
            border.color: Qt.rgba(1, 1, 1, 0.06)
            border.width: Theme.hairline
            clip: true
            Text {
                visible: Plan.libraryLogs.length === 0
                anchors.horizontalCenter: parent.horizontalCenter
                y: 16
                text: "No flight logs recorded yet."
                color: Theme.textDim
                font.family: Theme.fontUi; font.pixelSize: 10
            }
            ListView {
                anchors.fill: parent
                anchors.margins: 1
                model: Plan.libraryLogs
                boundsBehavior: Flickable.StopAtBounds
                ScrollBar.vertical: ScrollBar { }
                delegate: Item {
                    required property var modelData
                    width: ListView.view.width
                    height: 30
                    Rectangle { anchors.bottom: parent.bottom; width: parent.width; height: Theme.hairline; color: Qt.rgba(1, 1, 1, 0.04) }
                    Text { x: 10; anchors.verticalCenter: parent.verticalCenter; text: parent.modelData.name; color: Theme.textMain; font.family: Theme.fontData; font.pixelSize: 12 }
                    Text { anchors.right: parent.right; anchors.rightMargin: 10; anchors.verticalCenter: parent.verticalCenter; text: parent.modelData.meta; color: Theme.textDim; font.family: Theme.fontUi; font.pixelSize: 10 }
                }
            }
        }
    }
}
