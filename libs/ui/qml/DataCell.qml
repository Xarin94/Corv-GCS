// .data-cell of the tapes: label over the value column (right-aligned), the
// unit in a fixed gutter beside the value, baseline-aligned. Primary cells use
// --fs-2xl in white, the others --fs-xl in --hud-text-sub with a hairline on top.
import QtQuick

Item {
    id: cell
    property string label
    property string value
    property string unit
    property bool primary: false
    property color valueColor: primary ? Theme.hudText : Theme.hudTextSub
    property bool topBorder: !primary

    readonly property real pad: Theme.clamp(2, 0.6 * Theme.vh, 8)
    readonly property real unitW: Theme.fsSm * 3.1
    implicitHeight: pad * 2 + lbl.implicitHeight + val.implicitHeight * 0.92

    Rectangle {
        visible: cell.topBorder
        width: parent.width
        height: Theme.hairline
        color: Theme.borderSubtle
    }

    OutlinedText {
        id: lbl
        y: cell.pad
        width: parent.width - cell.unitW - 0.4 * Theme.fsSm
        horizontalAlignment: Text.AlignRight
        text: Theme.tr(cell.label).toUpperCase()
        color: Theme.hudTextLbl
        font.family: Theme.fontUi
        font.pixelSize: Theme.fsSm
        font.weight: Font.DemiBold
        font.letterSpacing: 0.09 * Theme.fsSm
    }
    OutlinedText {
        id: val
        anchors.top: lbl.bottom
        anchors.topMargin: -0.08 * font.pixelSize
        width: lbl.width
        horizontalAlignment: Text.AlignRight
        text: cell.value
        color: cell.valueColor
        font.family: Theme.fontData
        font.pixelSize: cell.primary ? Theme.fs2xl : Theme.fsXl
        font.features: { "tnum": 1 }
        lineHeight: 1.05
    }
    OutlinedText {
        anchors.left: val.right
        anchors.leftMargin: 0.4 * Theme.fsSm
        anchors.baseline: val.baseline
        width: cell.unitW
        text: cell.unit
        color: Theme.hudTextLbl
        font.family: Theme.fontUi
        font.pixelSize: Theme.fsSm
        font.weight: Font.DemiBold
        font.letterSpacing: 0.06 * Theme.fsSm
    }
}
