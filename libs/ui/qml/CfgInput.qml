// .failsafe-input: the dark text / number field of the configuration pages
// (11 px Roboto Mono, 26 px tall, padding 0 8). `committed` fires on Enter or
// when the field loses focus with a changed text (the DOM "change" event).
import QtQuick
import QtQuick.Controls.Basic

TextField {
    id: f
    property real fontSize: 11
    property bool numeric: false
    property color frame: Theme.borderInput
    property string committedText: ""
    signal committed(string text)

    implicitHeight: 26
    implicitWidth: 96
    leftPadding: 8
    rightPadding: 8
    topPadding: 0
    bottomPadding: 0
    verticalAlignment: TextInput.AlignVCenter
    color: enabled ? Theme.textMain : Theme.textDim
    placeholderTextColor: Theme.textDim
    selectionColor: Qt.rgba(0, 210 / 255, 1, 0.35)
    selectedTextColor: Theme.textMain
    font.family: Theme.fontData
    font.pointSize: (fontSize) * 0.75
    selectByMouse: true
    inputMethodHints: numeric ? Qt.ImhFormattedNumbersOnly : Qt.ImhNone
    background: Rectangle {
        radius: 3
        color: Theme.inputBg
        border.color: f.activeFocus ? Theme.borderAccentMid : f.frame
        border.width: Theme.hairline
    }
    Component.onCompleted: committedText = text
    onTextChanged: if (!activeFocus) committedText = text
    // Number fields select their text on focus, as the Electron version did
    onActiveFocusChanged: {
        if (activeFocus && numeric) selectAll()
        if (!activeFocus && text !== committedText) { committedText = text; committed(text) }
    }
    onAccepted: { committedText = text; committed(text); focus = false }
}
