// .cmd-btn of the command bar: dark fill, coloured text and outline per role.
import QtQuick
import QtQuick.Controls.Basic

Rectangle {
    id: b
    property string text
    property color accent: "white"            // white = neutral button
    property bool neutral: Qt.colorEqual(accent, "white")
    property bool bold: false
    property real fontSize: Theme.fsXs
    property string tooltip: ""
    signal clicked()

    implicitWidth: lbl.implicitWidth + 2.75 * fontSize * 2
    implicitHeight: 21
    radius: 3
    opacity: enabled ? 1 : 0.4
    color: mouse.containsMouse && enabled ? Theme.cmdButtonHover : Theme.cmdButton
    border.width: Theme.hairline
    border.color: neutral ? Theme.borderInput : accent

    Text {
        id: lbl
        anchors.centerIn: parent
        text: Theme.tr(b.text)
        color: b.accent
        font.family: Theme.fontUi
        font.pixelSize: b.fontSize
        font.weight: b.bold ? Font.Bold : Font.DemiBold
        font.letterSpacing: 0.09 * b.fontSize
    }
    MouseArea {
        id: mouse
        anchors.fill: parent
        hoverEnabled: true
        enabled: b.enabled
        onClicked: b.clicked()
    }
    ToolTip.visible: tooltip.length > 0 && mouse.containsMouse
    ToolTip.delay: 600
    ToolTip.text: Theme.tr(tooltip)
}
