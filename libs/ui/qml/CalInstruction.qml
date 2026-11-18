// .cal-instruction: what to do now, in one line, with the state on the left
// border (running orange, ok green, error red).
import QtQuick

Rectangle {
    id: box
    property string text: ""
    property string kind: ""
    width: parent ? parent.width : 320
    height: label.implicitHeight + 16
    radius: 2
    color: Theme.inputBg
    Rectangle {
        width: 3
        height: parent.height
        radius: 1
        color: box.kind === "running" ? Theme.accentOrange : box.kind === "ok" ? Theme.accentGreen
             : box.kind === "error" ? Theme.accentRed : Theme.borderSubtle
    }
    Text {
        id: label
        x: 13
        y: 8
        width: parent.width - 23
        text: box.text
        wrapMode: Text.Wrap
        color: box.kind === "error" ? "#ff7070" : Theme.textMain
        font.family: Theme.fontUi
        font.pixelSize: 13
        font.weight: Font.DemiBold
        lineHeight: 1.15
    }
}
