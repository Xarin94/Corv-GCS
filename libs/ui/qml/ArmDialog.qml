// The ARM / DISARM confirmation of the command bar (showConfirmDialog of
// CommandBarController.js): the question, what the normal command does, what
// the force variant does (param2 = 21196) and three buttons: CANCEL, ARM /
// DISARM, FORCE ARM / FORCE DISARM. Enter confirms, Escape or a click outside
// cancels.
import QtQuick

Rectangle {
    id: dlg
    property bool open: false
    property bool disarm: false
    property bool flying: false
    property bool prearmOk: true

    function show(isDisarm, isFlying, prearm) {
        disarm = isDisarm
        flying = isFlying
        prearmOk = prearm
        open = true
        okBtn.forceActiveFocus()
    }
    function close(choice) {
        if (!open) return
        open = false
        if (choice === "confirm") disarm ? Gcs.disarm(false) : Gcs.arm(false)
        else if (choice === "force") disarm ? Gcs.disarm(true) : Gcs.arm(true)
    }

    visible: open
    color: Qt.rgba(0, 0, 0, 0.6)
    MouseArea { anchors.fill: parent; onPressed: dlg.close(null) }   // a click outside cancels

    Rectangle {
        anchors.centerIn: parent
        width: 420
        height: col.implicitHeight + 32
        radius: 6
        color: "#1a1f2e"
        border.color: Theme.accentCyan
        border.width: 1
        MouseArea { anchors.fill: parent }   // clicks on the box stay in it

        Column {
            id: col
            x: 20; y: 16
            width: parent.width - 40
            spacing: 8
            Text {
                width: parent.width
                wrapMode: Text.Wrap
                text: dlg.disarm ? "DISARM the vehicle?" : "ARM the vehicle?"
                color: Theme.textMain
                font.family: Theme.fontUi; font.pixelSize: 14; font.weight: Font.DemiBold
            }
            Text {
                width: parent.width
                wrapMode: Text.Wrap
                lineHeight: 1.2
                text: dlg.disarm
                      ? "Sends MAV_CMD_COMPONENT_ARM_DISARM (400) with param1=0. The autopilot refuses a normal disarm while the vehicle is flying or not landed."
                        + (dlg.flying ? " The vehicle currently looks airborne — a normal disarm will most likely be rejected." : "")
                      : "Ensure the area is clear. Sends MAV_CMD_COMPONENT_ARM_DISARM (400) with param1=1. The autopilot runs all pre-arm checks (GPS, EKF, compass, battery, safety switch) and refuses if any fails."
                        + (dlg.prearmOk ? "" : " Pre-arm checks are currently NOT passing.")
                color: "#cccccc"
                font.family: Theme.fontUi; font.pixelSize: 12
            }
            Text {
                width: parent.width
                wrapMode: Text.Wrap
                lineHeight: 1.2
                text: dlg.disarm
                      ? "FORCE DISARM sends param2=21196: it bypasses the disarm checks and cuts the motors immediately, in flight too. The vehicle will fall."
                      : "FORCE ARM sends param2=21196: it skips every pre-arm check. The props can spin with a bad EKF, no GPS fix or a failing sensor. Use only if you know exactly which check is failing and why."
                color: "#ff9f43"
                font.family: Theme.fontUi; font.pixelSize: 11
            }
            Item { width: 1; height: 6 }
            Row {
                anchors.right: parent.right
                spacing: 8
                component DlgButton: Rectangle {
                    property alias text: t.text
                    property color ink: "white"
                    signal clicked()
                    width: t.implicitWidth + 28
                    height: 26
                    radius: 3
                    border.width: activeFocus ? 1 : 0
                    border.color: "white"
                    Text { id: t; anchors.centerIn: parent; color: parent.ink; font.family: Theme.fontUi; font.pixelSize: 11; font.weight: Font.DemiBold; font.letterSpacing: 0.5 }
                    MouseArea { anchors.fill: parent; cursorShape: Qt.PointingHandCursor; onClicked: parent.clicked() }
                }
                DlgButton { text: "CANCEL"; color: "#333333"; ink: "#cccccc"; onClicked: dlg.close(null) }
                DlgButton { id: okBtn; text: dlg.disarm ? "DISARM" : "ARM"; color: Theme.accentCyan; ink: "black"; onClicked: dlg.close("confirm") }
                DlgButton { text: dlg.disarm ? "FORCE DISARM" : "FORCE ARM"; color: "#c0392b"; onClicked: dlg.close("force") }
            }
        }
    }
    Keys.onEscapePressed: close(null)
    Keys.onReturnPressed: close("confirm")
    Keys.onEnterPressed: close("confirm")
    focus: open
}
