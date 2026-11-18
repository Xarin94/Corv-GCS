// An icon from the Electron SVGs (24x24 viewBox), drawn with Qt Quick Shapes so
// it takes the button's colour like `currentColor`.
// parts: [{ d: "...", fill: true|false, stroke: width (0 = none), cap: "round", opacity: 1 }]
import QtQuick
import QtQuick.Shapes

Item {
    id: icon
    property var parts: []
    property color color: "white"
    property real viewBox: 24
    implicitWidth: 18
    implicitHeight: 18

    Repeater {
        model: icon.parts
        delegate: Shape {
            required property var modelData
            width: icon.viewBox
            height: icon.viewBox
            preferredRendererType: Shape.CurveRenderer
            antialiasing: true
            transform: Scale { xScale: icon.width / icon.viewBox; yScale: icon.height / icon.viewBox }
            readonly property real alpha: modelData.opacity !== undefined ? modelData.opacity : 1
            ShapePath {
                fillColor: modelData.fill ? Qt.rgba(icon.color.r, icon.color.g, icon.color.b, alpha) : "transparent"
                strokeColor: modelData.stroke ? Qt.rgba(icon.color.r, icon.color.g, icon.color.b, alpha) : "transparent"
                strokeWidth: modelData.stroke ? modelData.stroke : -1
                capStyle: modelData.cap === "round" ? ShapePath.RoundCap : ShapePath.FlatCap
                joinStyle: modelData.cap === "round" || modelData.join === "round" ? ShapePath.RoundJoin : ShapePath.MiterJoin
                // stroke-dasharray, in viewBox units like the SVG
                strokeStyle: modelData.dash ? ShapePath.DashLine : ShapePath.SolidLine
                dashPattern: modelData.dash ? modelData.dash.map(v => v / (modelData.stroke || 1)) : [4, 2]
                PathSvg { path: modelData.d }
            }
        }
    }
}
