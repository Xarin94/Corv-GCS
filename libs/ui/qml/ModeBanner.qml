// Mode change banner (#mode-change-banner, showModeBanner of
// CommandBarController.js, .mode-banner of animations.css): the new flight
// mode in the mode category's colour, 80 px below the top of the flight view,
// shown for 2.5 s on every change after the first one.
import QtQuick
import Corv.Ui

Item {
    id: root
    width: frame.width
    height: frame.height
    visible: false

    // lastDisplayedMode of CommandBarController.js: the first mode seen sets it
    // without a banner
    property string lastDisplayedMode: ""
    readonly property string mode: Vehicle.flightMode
    onModeChanged: {
        if (mode && mode !== "UNKNOWN" && mode !== lastDisplayedMode) {
            if (lastDisplayedMode !== "")   // skip initial set
                show(mode, Vehicle.modeClass)
            lastDisplayedMode = mode
        }
    }

    function show(name, category) {
        label.text = name
        frame.accent = Theme.modeColor(category)
        anim.restart()
    }

    Rectangle {
        id: frame
        property color accent: Theme.accentCyan
        width: label.implicitWidth + 56     // padding 8 px 28 px
        height: label.implicitHeight + 16
        radius: 4
        color: Qt.rgba(0, 0, 0, 0.85)       // --bg-overlay
        border.width: 1
        border.color: Qt.rgba(accent.r, accent.g, accent.b, 0.4)

        Text {
            id: label
            anchors.centerIn: parent
            color: frame.accent
            font.family: Theme.fontUi
            font.pixelSize: 16
            font.weight: Font.Bold
            font.letterSpacing: 3
            font.capitalization: Font.AllUppercase
        }
    }

    // mode-banner-in, 2.5 s ease: 0 % hidden 20 px up, 15 % in place, 85 % still,
    // 100 % hidden 10 px up; then .hidden
    SequentialAnimation {
        id: anim
        PropertyAction { target: root; property: "visible"; value: true }
        ParallelAnimation {
            NumberAnimation { target: root; property: "opacity"; from: 0; to: 1; duration: 375; easing.type: Easing.OutCubic }
            NumberAnimation { target: frame; property: "y"; from: -20; to: 0; duration: 375; easing.type: Easing.OutCubic }
        }
        PauseAnimation { duration: 1750 }
        ParallelAnimation {
            NumberAnimation { target: root; property: "opacity"; to: 0; duration: 375; easing.type: Easing.InOutQuad }
            NumberAnimation { target: frame; property: "y"; to: -10; duration: 375; easing.type: Easing.InOutQuad }
        }
        PropertyAction { target: root; property: "visible"; value: false }
    }
}
