// Text drawn straight on the 3D image: a hard 1 px black outline, no glow
// (--text-outline / --text-outline-strong of variables.css).
import QtQuick

Text {
    style: Text.Outline
    styleColor: Qt.rgba(0, 0, 0, 0.92)
    renderType: Text.QtRendering
}
