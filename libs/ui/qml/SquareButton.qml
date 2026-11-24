// .ui-btn.square of the tab bar: 30x30, black, cyan outline; orange when active.
import QtQuick
import QtQuick.Controls.Basic

Rectangle {
    id: b
    property var icon: null
    property string label: ""
    property bool active: false
    property color accent: Theme.accentActive   // while active
    property string tooltip: ""
    signal clicked()

    readonly property color fg: active ? accent : Theme.accentCyan
    width: 30
    height: 30
    color: active ? Qt.rgba(accent.r, accent.g, accent.b, 0.2) : (mouse.containsMouse ? Qt.rgba(0, 40 / 255, 60 / 255, 0.9) : Theme.squareBg)
    border.color: fg
    border.width: Theme.hairline

    SvgIcon {
        visible: b.icon !== null
        anchors.centerIn: parent
        width: 18
        height: 18
        parts: b.icon ? b.icon : []
        color: b.fg
    }
    Text {
        visible: b.label.length > 0
        anchors.centerIn: parent
        text: Theme.tr(b.label)
        color: b.fg
        font.family: Theme.fontUi
        font.pixelSize: Theme.fsMd
        font.weight: Font.DemiBold
        font.letterSpacing: 1
    }
    MouseArea {
        id: mouse
        anchors.fill: parent
        hoverEnabled: true
        onClicked: b.clicked()
    }
    ToolTip.visible: tooltip.length > 0 && mouse.containsMouse
    ToolTip.delay: 600
    ToolTip.text: Theme.tr(tooltip)
}
