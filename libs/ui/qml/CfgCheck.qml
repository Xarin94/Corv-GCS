// .cfg-check: a 14 px checkbox (accent-color cyan as Chromium draws it on a
// dark page) with a short label beside it, 11 px Rajdhani semibold.
import QtQuick

Item {
    id: c
    property bool checked: false
    property string text: ""
    property real fontSize: 11
    property string tip: ""
    signal toggled(bool checked)

    implicitHeight: Math.max(26, label.implicitHeight)
    implicitWidth: 22 + label.implicitWidth

    Rectangle {
        id: box
        width: 14; height: 14
        anchors.verticalCenter: parent.verticalCenter
        radius: 2
        color: c.checked ? "#75cffa" : Qt.rgba(1, 1, 1, 0.06)
        border.color: c.checked ? "#75cffa" : "#8a96a0"
        border.width: c.checked ? 0 : 1
        SvgIcon {
            anchors.centerIn: parent
            width: 12; height: 12
            visible: c.checked
            parts: [{ d: "M5 12.5l4.2 4.2L19 7", stroke: 3, cap: "round" }]
            color: "#3b3b3b"
        }
    }
    Text {
        id: label
        x: 22
        width: c.width - 22
        anchors.verticalCenter: parent.verticalCenter
        text: Theme.tr(c.text)
        wrapMode: Text.Wrap
        color: Theme.textMain
        font.family: Theme.fontUi
        font.pointSize: (c.fontSize) * 0.75
        font.weight: Font.DemiBold
        font.letterSpacing: 1
        lineHeight: 1.3
    }
    MouseArea {
        anchors.fill: parent
        cursorShape: Qt.PointingHandCursor
        onClicked: { c.checked = !c.checked; c.toggled(c.checked) }
    }
    HoverTip { text: c.tip }
}
