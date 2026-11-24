// .gcs-btn of the calibration pages (.cal-actions): 29 px, Rajdhani 11 px
// semibold. primary: the green .cal-primary of a button that starts a
// procedure; armed: orange and pulsing while the vehicle waits for the operator
// (CONTINUE). Disabled: 40 % opacity.
import QtQuick
import QtQuick.Controls.Basic

Rectangle {
    id: b
    property string text: ""
    property bool primary: false
    property bool armed: false
    property string tip: ""
    signal clicked()

    readonly property color tone: armed ? Theme.accentOrange : primary ? Theme.accentGreen : Theme.textMain
    implicitWidth: label.implicitWidth + 22
    implicitHeight: 29
    radius: 3
    opacity: enabled ? 1 : 0.4
    color: hover.hovered && enabled ? Theme.gcsButtonHover : Theme.gcsButton
    border.width: Theme.hairline
    border.color: !enabled ? Theme.borderInput
                : armed ? Theme.accentOrange
                : primary ? Theme.accentGreen
                : hover.hovered ? Theme.accentCyan : Theme.borderInput

    // The pulse of .cal-primary.is-armed: a 3 px orange glow, 1.4 s
    Rectangle {
        anchors.fill: parent
        anchors.margins: -3
        radius: 6
        color: "transparent"
        border.width: 3
        border.color: Qt.rgba(1, 170 / 255, 0, 0.18)
        visible: b.armed && b.enabled
        SequentialAnimation on opacity {
            running: b.armed && b.enabled
            loops: Animation.Infinite
            NumberAnimation { from: 0; to: 1; duration: 700; easing.type: Easing.InOutSine }
            NumberAnimation { from: 1; to: 0; duration: 700; easing.type: Easing.InOutSine }
        }
    }
    Text {
        id: label
        anchors.centerIn: parent
        text: Theme.tr(b.text)
        color: b.enabled ? b.tone : Theme.textMain
        font.family: Theme.fontUi
        font.pixelSize: 11
        font.weight: Font.DemiBold
        font.letterSpacing: 0.8
    }
    HoverHandler { id: hover; cursorShape: b.enabled ? Qt.PointingHandCursor : Qt.ForbiddenCursor }
    TapHandler { enabled: b.enabled; onTapped: b.clicked() }
    ToolTip.visible: tip.length > 0 && hover.hovered
    ToolTip.text: Theme.tr(tip)
    ToolTip.delay: 600
}
