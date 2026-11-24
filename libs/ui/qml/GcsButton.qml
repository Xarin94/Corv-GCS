// .gcs-btn: dark button with a hairline border, 10 px Rajdhani semibold.
// primary: the green UPLOAD style (.fp-btn-primary); blocked: red text when the
// route has errors; dashed: the .file-upload-btn of SELECT HGT FOLDER.
import QtQuick
import QtQuick.Controls.Basic

Rectangle {
    id: b
    property string text: ""
    property real fontSize: 10
    property real spacing: 1
    property bool primary: false
    property bool blocked: false
    property bool dashed: false
    property string tip: ""
    property color frame: primary ? (blocked ? Qt.rgba(1, 60 / 255, 60 / 255, 0.4) : (hover.hovered ? Theme.accentGreen : Qt.rgba(68 / 255, 1, 68 / 255, 0.35)))
                                  : Theme.borderInput
    signal clicked()

    implicitHeight: 24
    implicitWidth: label.implicitWidth + 16
    radius: 3
    opacity: enabled ? 1 : 0.5
    color: dashed ? Qt.rgba(26 / 255, 26 / 255, 26 / 255, 0.9)
         : primary && hover.hovered ? Qt.rgba(68 / 255, 1, 68 / 255, 0.12)
         : hover.hovered && enabled ? Theme.gcsButtonHover : Theme.gcsButton
    border.color: frame
    border.width: dashed ? 0 : Theme.hairline

    // Dashed frame for the folder button
    Canvas {
        anchors.fill: parent
        visible: b.dashed
        onPaint: {
            const ctx = getContext("2d")
            ctx.clearRect(0, 0, width, height)
            ctx.strokeStyle = "rgba(255,255,255,0.15)"
            ctx.lineWidth = 0.67
            ctx.setLineDash([3, 2])
            ctx.beginPath()
            ctx.roundedRect(0.5, 0.5, width - 1, height - 1, 3, 3)
            ctx.stroke()
        }
    }
    Text {
        id: label
        anchors.centerIn: parent
        text: Theme.tr(b.text)
        color: b.dashed ? Theme.textDim : b.primary ? (b.blocked ? "#ff8080" : Theme.accentGreen) : Theme.textMain
        font.family: Theme.fontUi
        font.pointSize: (b.fontSize) * 0.75
        font.weight: Font.DemiBold
        font.letterSpacing: b.spacing
    }
    HoverHandler { id: hover; cursorShape: b.enabled ? Qt.PointingHandCursor : Qt.ArrowCursor }
    // The press stays here: with the default policy a map under the button got it too
    TapHandler { gesturePolicy: TapHandler.ReleaseWithinBounds; enabled: b.enabled; onTapped: b.clicked() }
    ToolTip.visible: tip.length > 0 && hover.hovered
    ToolTip.text: Theme.tr(tip)
    ToolTip.delay: 600
}
