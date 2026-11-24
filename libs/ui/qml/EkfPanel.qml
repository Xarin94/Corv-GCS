// EKF STATUS bar (#ekf-panel, EkfStatusPanel.js): the estimator's source and
// flags, then three charts of the last 20 s sharing the height. Logic and
// drawing in EkfController ("Ekf") and EkfChart.
import QtQuick
import QtQuick.Layouts
import Corv.Ui

LeftBar {
    id: bar
    title: "EKF STATUS"
    window: "LAST 20 s"

    // .ekf-flag: ✓ good, ✕ critical, ▲ warning, • info
    component Flag: Rectangle {
        property string label
        property string level
        implicitWidth: ft.implicitWidth + 10
        implicitHeight: ft.implicitHeight + 2
        radius: 3
        color: level === "critical" ? Qt.rgba(208 / 255, 59 / 255, 59 / 255, 0.15) : "transparent"
        border.width: Theme.hairline + 0.3
        border.color: level === "good" ? Qt.rgba(12 / 255, 163 / 255, 12 / 255, 0.7) : level === "warning" ? "#fab219"
                    : level === "critical" ? "#d03b3b" : Qt.rgba(1, 1, 1, 0.12)
        Text {
            id: ft
            anchors.centerIn: parent
            textFormat: Text.StyledText
            text: (parent.level === "good" ? "<font color=\"#0ca30c\">✓</font>" : parent.level === "critical" ? "✕" : parent.level === "warning" ? "▲" : "•")
                  + " " + parent.label
            color: parent.level === "warning" ? "#fab219" : parent.level === "critical" ? "#ff8a8a" : parent.level === "info" ? "#c3c2b7" : "#f0efec"
            font.family: Theme.fontUi; font.pixelSize: Theme.fsXs; font.weight: Font.DemiBold; font.letterSpacing: 0.06 * Theme.fsXs
        }
    }

    Text {
        Layout.fillWidth: true
        text: Theme.tr(Ekf.sourceText)
        elide: Text.ElideRight
        color: "#c3c2b7"
        font.family: Theme.fontUi; font.pixelSize: Theme.fsXs; font.weight: Font.DemiBold; font.letterSpacing: 0.12 * Theme.fsXs
    }
    Flow {
        Layout.fillWidth: true
        spacing: 4
        visible: Ekf.flags.length > 0
        Repeater {
            model: Ekf.flags
            delegate: Flag { required property var modelData; label: modelData.label; level: modelData.state }
        }
    }

    Repeater {
        model: 3
        delegate: ColumnLayout {
            id: card
            required property int index
            readonly property var info: Ekf.cards[index] || ({ title: "", unit: "", chips: [], legend: [] })
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredHeight: 1
            spacing: 2
            Rectangle { Layout.fillWidth: true; height: Theme.hairline; color: Qt.rgba(1, 1, 1, 0.10) }
            // .ekf-card-head: the title, the chips on the right
            RowLayout {
                Layout.fillWidth: true
                Layout.topMargin: 3
                spacing: 8
                Text {
                    text: Theme.tr(card.info.title)
                    color: "#f0efec"
                    font.family: Theme.fontUi; font.pixelSize: Theme.fsXs; font.weight: Font.Bold; font.letterSpacing: 0.12 * Theme.fsXs
                }
                Item { Layout.fillWidth: true }
                Repeater {
                    model: card.info.chips
                    // .ekf-chip: the key dim, the value in the data face; ▲ and amber / red on a warning
                    delegate: Row {
                        required property var modelData
                        readonly property string tone: modelData.status === "warning" ? "#fab219" : modelData.status === "critical" ? "#ff8a8a" : ""
                        spacing: 3
                        Text {
                            text: (parent.tone ? "▲ " : "") + parent.modelData.k
                            color: parent.tone || "#898781"
                            font.family: Theme.fontUi; font.pixelSize: Theme.fsXs; font.letterSpacing: 0.06 * Theme.fsXs
                        }
                        Text {
                            text: parent.modelData.v
                            color: parent.tone || "#f0efec"
                            font.family: Theme.fontData; font.pixelSize: Theme.fsXs
                        }
                    }
                }
            }
            Text {
                Layout.fillWidth: true
                text: card.info.unit
                elide: Text.ElideRight
                color: "#898781"
                font.family: Theme.fontUi; font.pixelSize: Theme.clamp(9, 0.6 * Theme.vw, 12); font.letterSpacing: 0.04 * 10
            }
            EkfChart {
                card: card.index
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.minimumHeight: 60
            }
            Flow {
                Layout.fillWidth: true
                Layout.bottomMargin: 4
                spacing: 12
                Repeater {
                    model: card.info.legend
                    delegate: Row {
                        required property var modelData
                        spacing: 4
                        Rectangle { anchors.verticalCenter: parent.verticalCenter; width: 10; height: 2; radius: 1; color: parent.modelData.color }
                        Text {
                            text: Theme.tr(parent.modelData.name)
                            color: "#c3c2b7"
                            font.family: Theme.fontUi; font.pixelSize: Theme.fsXs; font.weight: Font.DemiBold; font.letterSpacing: 0.06 * Theme.fsXs
                        }
                        Text {
                            width: Math.max(implicitWidth, 3.5 * Theme.fsXs)
                            text: Theme.tr(parent.modelData.value)
                            color: "#f0efec"
                            font.family: Theme.fontData; font.pixelSize: Theme.fsXs
                        }
                    }
                }
            }
        }
    }
}
