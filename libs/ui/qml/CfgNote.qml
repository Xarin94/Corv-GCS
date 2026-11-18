// .cfg-note: a line of plain explanation (10 px Rajdhani, dim).
import QtQuick

Text {
    width: parent ? parent.width : 320
    wrapMode: Text.Wrap
    color: Theme.textDim
    font.family: Theme.fontUi
    font.pixelSize: 10
    lineHeight: 1.3
}
