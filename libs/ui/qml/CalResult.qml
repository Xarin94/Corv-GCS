// .cal-result: the answer of a one-command calibration under its button.
// result: {text, kind, busy} of Calibration.results.
import QtQuick

Text {
    property var result: ({})
    width: parent ? parent.width : 320
    text: Theme.tr(result && result.text ? result.text : "")
    visible: text.length > 0
    wrapMode: Text.Wrap
    color: !result ? Theme.textDim : result.kind === "running" ? Theme.accentOrange
         : result.kind === "ok" ? Theme.accentGreen : result.kind === "error" ? "#ff7070" : Theme.textDim
    font.family: Theme.fontUi
    font.pixelSize: 11
    lineHeight: 1.15
}
