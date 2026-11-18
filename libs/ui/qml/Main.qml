// Main window, laid out as index.html: global header (60 px) and tab bar
// (32 px) on top, the tab content, the GCS command bar (46 px) at the bottom,
// the custom title bar appearing over the top 40 px under the mouse.
import QtQuick
import QtQuick.Controls.Basic
import Corv.Ui

Window {
    id: win
    width: 1600
    height: 920
    minimumWidth: 1100
    minimumHeight: 700
    visible: true
    flags: Qt.Window | Qt.FramelessWindowHint
    color: Theme.bg
    title: "CORV GCS v" + Gcs.version

    onWidthChanged: Theme.winW = width
    onHeightChanged: Theme.winH = height
    Component.onCompleted: { Theme.winW = width; Theme.winH = height }

    HeaderBar {
        id: header
        objectName: "header"
        width: parent.width
        fps: flightView.scene.fps
        satellite: !flightView.scene.schematic
        sunlight: flightView.scene.sunlight
        horizonLock: flightView.hud.pitchLocked
        firstPerson: !flightView.scene.thirdPerson
        trajectory: flightView.scene.trajectory
        onToggleCamera: flightView.scene.thirdPerson = !flightView.scene.thirdPerson
        onToggleTrajectory: flightView.scene.trajectory = !flightView.scene.trajectory
        onToggleSatellite: flightView.scene.schematic = !flightView.scene.schematic
        onToggleSunlight: flightView.scene.sunlight = !flightView.scene.sunlight
        onToggleHorizonLock: flightView.hud.pitchLocked = !flightView.hud.pitchLocked
        onToggleTheme: flightView.scene.lightTheme = !flightView.scene.lightTheme
        onOpenUpdate: updateDialog.open = true
    }

    Item {
        id: content
        y: 93
        width: parent.width
        height: parent.height - 93 - Theme.cmdBarH

        FlightDataView {
            id: flightView
            objectName: "flightView"
            anchors.fill: parent
            visible: header.currentTab === 0
            windowHeight: win.height
            windowTop: content.y
        }
        FlightPlanPage {
            anchors.fill: parent
            visible: header.currentTab === 1
            shown: visible && !library.visible
        }
        SetupPage {
            objectName: "setupPage"
            anchors.fill: parent
            visible: header.currentTab === 2
        }
        SysConfigPage {
            anchors.fill: parent
            visible: header.currentTab === 3
            scene: flightView.scene
        }
    }

    CommandBar {
        id: commandBar
        anchors.bottom: parent.bottom
        width: parent.width
        // DISARM in the air is normally refused: the dialog says so when the vehicle looks airborne
        onArmRequested: armDialog.show(Vehicle.armed, (Vehicle.homeSet && Vehicle.altRel > 2) || Vehicle.groundspeed > 1.5 || Vehicle.airspeed > 1.5,
                                       Vehicle.sensorsOk)
    }
    // LOG REPLAY timeline: bottom-right, above the command bar, while a log is loaded
    ReplayTimeline {
        objectName: "replayTimeline"
        visible: Replay.loaded
        anchors.right: parent.right
        anchors.bottom: commandBar.top
        z: 50
    }

    // Full-window pages and the in-page dialogs, over everything but the title bar
    MissionLibrary {
        id: library
        anchors.fill: parent
        z: 90
    }
    Dialogs {
        z: 95
    }
    UpdateDialog {
        id: updateDialog
        objectName: "updateDialog"
        z: 97
    }
    ArmDialog {
        id: armDialog
        objectName: "armDialog"
        anchors.fill: parent
        z: 96
    }

    TitleBar {
        window: win
        width: parent.width
        z: 100
    }

    // Window edges: resize like a framed window
    MouseArea { z: 101; anchors.right: parent.right; anchors.bottom: parent.bottom; width: 8; height: 8; cursorShape: Qt.SizeFDiagCursor; onPressed: win.startSystemResize(Qt.RightEdge | Qt.BottomEdge) }
    MouseArea { z: 101; anchors.right: parent.right; y: 40; width: 3; height: parent.height - 48; cursorShape: Qt.SizeHorCursor; onPressed: win.startSystemResize(Qt.RightEdge) }
    MouseArea { z: 101; anchors.left: parent.left; y: 40; width: 3; height: parent.height - 48; cursorShape: Qt.SizeHorCursor; onPressed: win.startSystemResize(Qt.LeftEdge) }
    MouseArea { z: 101; anchors.bottom: parent.bottom; x: 8; width: parent.width - 16; height: 3; cursorShape: Qt.SizeVerCursor; onPressed: win.startSystemResize(Qt.BottomEdge) }
}
