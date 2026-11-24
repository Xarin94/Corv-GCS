// SETUP → VIBRATIONS (#subtab-vibration, initVibrationDisplay): the X / Y / Z
// vibration history with the 30 and 60 m/s² lines, current values, clipping.
import QtQuick
import QtQuick.Controls.Basic

Flickable {
    id: page
    contentHeight: col.implicitHeight + 48
    clip: true
    Column {
        id: col
        x: 24; y: 24
        spacing: 21
        SetupPanel {
            title: "VIBRATION LEVELS"
            width: Math.min(860, page.width - 48)
            Canvas {
                id: chart
                width: parent.width
                height: 200
                property var hist: SetupCfg.vibHistory
                onHistChanged: requestPaint()
                onPaint: {
                    const ctx = getContext("2d")
                    const w = width, h = height
                    ctx.clearRect(0, 0, w, h)
                    const maxVal = 80
                    const yAt = (v) => h - (v / maxVal) * h
                    ctx.setLineDash([4, 4])
                    ctx.strokeStyle = "rgba(255,255,0,0.2)"
                    ctx.beginPath(); ctx.moveTo(0, yAt(30)); ctx.lineTo(w, yAt(30)); ctx.stroke()
                    ctx.strokeStyle = "rgba(255,0,0,0.2)"
                    ctx.beginPath(); ctx.moveTo(0, yAt(60)); ctx.lineTo(w, yAt(60)); ctx.stroke()
                    ctx.setLineDash([])
                    if (hist.length < 2) return
                    const colors = ["#ff4444", "#44ff44", "#4488ff"]
                    const step = w / (hist.length - 1)
                    for (let k = 0; k < 3; ++k) {
                        ctx.strokeStyle = colors[k]
                        ctx.lineWidth = 1.5
                        ctx.beginPath()
                        for (let i = 0; i < hist.length; ++i) {
                            const x = i * step, y = yAt(hist[i][k])
                            if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y)
                        }
                        ctx.stroke()
                    }
                }
            }
            Row {
                spacing: 24
                Repeater {
                    model: [["X", "#ff4444", "x"], ["Y", "#44ff44", "y"], ["Z", "#4488ff", "z"]]
                    delegate: Row {
                        required property var modelData
                        spacing: 6
                        Rectangle { width: 10; height: 10; radius: 2; color: parent.modelData[1]; anchors.verticalCenter: parent.verticalCenter }
                        Text { text: parent.modelData[0] + ": " + Number(SetupCfg.vib[parent.modelData[2]] || 0).toFixed(1); color: Theme.textMain; font.family: Theme.fontData; font.pixelSize: 12 }
                    }
                }
                Text {
                    text: qsTr("Clipping: ") + (SetupCfg.vib.clip0 || 0) + " / " + (SetupCfg.vib.clip1 || 0) + " / " + (SetupCfg.vib.clip2 || 0)
                    color: Theme.textMain; font.family: Theme.fontData; font.pixelSize: 12
                }
            }
            CfgNote { text: "Threshold: <30 m/s² = good, 30-60 = marginal, >60 = high" }
        }
    }
}
