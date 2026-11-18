// .cfg-slider-wrap: a 2 px track with a square thumb, the live value beside it
// (11 px Roboto Mono, at least 38 px, right-aligned).
import QtQuick
import QtQuick.Controls.Basic

Item {
    id: s
    property real from: 0
    property real to: 1
    property real stepSize: 0.01
    property real value: 0
    property string valueText: value.toFixed(2)
    signal moved(real value)

    implicitHeight: 14
    implicitWidth: 185

    Slider {
        id: sl
        x: 0
        width: s.width - valueLabel.width - 8
        anchors.verticalCenter: parent.verticalCenter
        height: 14
        from: s.from
        to: s.to
        stepSize: s.stepSize
        value: s.value
        padding: 0
        onMoved: { s.value = value; s.moved(value) }
        background: Rectangle {
            x: 0
            y: (sl.height - height) / 2
            width: sl.width
            height: 2
            color: "#333333"
        }
        handle: Rectangle {
            x: sl.visualPosition * (sl.width - width)
            y: (sl.height - height) / 2
            width: 12
            height: 12
            radius: 2
            color: "#75cffa"
        }
    }
    Text {
        id: valueLabel
        anchors.right: parent.right
        anchors.verticalCenter: parent.verticalCenter
        width: Math.max(38, implicitWidth)
        horizontalAlignment: Text.AlignRight
        text: s.valueText
        color: Theme.textMain
        font.family: Theme.fontData
        font.pixelSize: 11
    }
}
