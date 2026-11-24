// .gz-legend (GeoZoneLayer.js): the key of the UAS zones on the FLIGHT PLAN
// map while they are on: banded no-fly and conditional swatches and the ramp
// of the height that may be flown.
import QtQuick
import Corv.Ui

Rectangle {
    id: lg
    width: col.implicitWidth + 16
    height: col.implicitHeight + 12
    radius: 3
    color: Qt.rgba(0, 0, 0, 0.72)
    border.color: Theme.borderAccentMid
    border.width: Theme.hairline

    component Banded: Canvas {
        property color tone
        width: 18; height: 10
        onPaint: {
            const c = getContext("2d")
            c.fillStyle = tone
            c.fillRect(0, 0, width, height)
            c.strokeStyle = "black"
            c.lineWidth = 2.5
            for (let x = -height; x < width; x += 6) {
                c.beginPath(); c.moveTo(x, height); c.lineTo(x + height, 0); c.stroke()
            }
        }
    }
    component Lbl: Text {
        color: Theme.textMutedBright
        font.family: Theme.fontUi
        font.pixelSize: Theme.fsXs
        anchors.verticalCenter: parent ? parent.verticalCenter : undefined
    }

    Column {
        id: col
        x: 8
        y: 6
        spacing: 4
        Text { text: qsTr("UAS ZONES"); color: Theme.textDim; font.family: Theme.fontUi; font.pixelSize: Theme.fsXs; font.weight: Font.DemiBold; font.letterSpacing: 1 }
        Row { spacing: 6; Banded { tone: "#e8231d" } Lbl { text: qsTr("No-fly / authorisation") } }
        Row { spacing: 6; Banded { tone: "#b060ff" } Lbl { text: qsTr("Conditions apply") } }
        Row {
            spacing: 6
            Row {
                Repeater {
                    model: [[15, "#ff541a"], [30, "#ff8c1a"], [60, "#ffc41a"], [90, "#e2e81e"], [120, "#8cde3c"]]
                    delegate: Rectangle {
                        required property var modelData
                        width: 22; height: 12
                        color: modelData[1]
                        Text { anchors.centerIn: parent; text: Theme.tr(parent.modelData[0]); color: "black"; font.family: Theme.fontData; font.pixelSize: 8 }
                    }
                }
            }
            Lbl { text: qsTr("max m AGL") }
        }
    }
}
