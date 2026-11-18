// The vehicle in one of the six accelerometer positions (positionGlyph of
// CalibrationController.js): a side view, nose to the right, or a view from
// behind with the navigation lights (red left, green right), on a dashed ground
// line. viewBox 100 × 80, scaled to the width.
import QtQuick

Canvas {
    id: g
    property string view: "side"     // "side" or "rear"
    property real rot: 0             // degrees
    property bool flip: false        // upside down
    property real lift: 5            // sits the drawing on the ground line
    property color ink: Theme.textDim
    property color dimInk: Theme.textDim
    height: width * 0.8
    onInkChanged: requestPaint()
    onWidthChanged: requestPaint()

    onPaint: {
        const ctx = getContext("2d")
        ctx.reset()
        const k = width / 100
        ctx.scale(k, k)
        const ground = 70
        ctx.lineWidth = 1.2
        ctx.strokeStyle = g.dimInk
        ctx.setLineDash([3, 3])
        ctx.beginPath(); ctx.moveTo(8, ground); ctx.lineTo(92, ground); ctx.stroke()
        ctx.setLineDash([])

        ctx.translate(50, ground - g.lift)
        if (g.flip) ctx.scale(1, -1)
        else ctx.rotate(g.rot * Math.PI / 180)
        ctx.lineWidth = 1.4
        ctx.strokeStyle = g.ink
        ctx.fillStyle = Qt.rgba(127 / 255, 127 / 255, 127 / 255, 0.15)
        const rr = (x, y, w, h, r) => { ctx.beginPath(); ctx.roundedRect(x, y, w, h, r, r); ctx.fill(); ctx.stroke() }
        const dot = (x, y, r, c) => { ctx.beginPath(); ctx.fillStyle = c; ctx.arc(x, y, r, 0, 2 * Math.PI); ctx.fill() }
        const mast = (x, y0) => { ctx.beginPath(); ctx.moveTo(x, y0); ctx.lineTo(x, -11); ctx.stroke(); dot(x, -13, 2.5, g.ink) }
        if (g.view === "side") {
            rr(-24, -4, 46, 8, 2)
            ctx.beginPath(); ctx.fillStyle = Theme.accentCyan
            ctx.moveTo(22, -4.5); ctx.lineTo(31, 0); ctx.lineTo(22, 4.5); ctx.closePath(); ctx.fill()
            mast(-6, -4)
        } else {
            rr(-21, -3.5, 42, 7, 2)
            mast(0, -3.5)
            dot(-21, 0, 3, "#ff4d4d")
            dot(21, 0, 3, "#2fd27a")
        }
    }
}
