// The update dialog, opened by UPDATE in the header (Updates view model): the
// version found, its release notes, what INSTALL does on this system, the
// download progress, and LATER / SKIP THIS VERSION / RELEASE PAGE. Escape or
// a click outside closes it; a download carries on in the background.
import QtQuick
import QtQuick.Controls.Basic
import Corv.Ui

Rectangle {
    id: dlg
    property bool open: false
    readonly property bool busy: Updates.state === "downloading" || Updates.state === "installing"

    visible: open
    anchors.fill: parent
    color: Qt.rgba(0, 0, 0, 0.55)
    MouseArea { anchors.fill: parent; onClicked: dlg.open = false }
    Keys.onEscapePressed: dlg.open = false
    onOpenChanged: if (open) forceActiveFocus()

    component DlgButton: Rectangle {
        id: b
        property string text
        property bool primary: false
        property bool enabledState: true
        signal clicked()
        width: bt.implicitWidth + 32
        height: bt.implicitHeight + 12
        radius: 4
        opacity: enabledState ? 1 : 0.4
        color: primary ? Theme.accentCyan : "#2a2a3e"
        border.color: primary ? Theme.accentCyan : "#555555"
        border.width: 1
        Text {
            id: bt
            anchors.centerIn: parent
            text: Theme.tr(b.text)
            color: b.primary ? "#000000" : "#cccccc"
            font.family: Theme.fontUi
            font.pixelSize: 12
            font.bold: b.primary
            font.letterSpacing: 1
        }
        TapHandler { enabled: b.enabledState; onTapped: b.clicked() }
        HoverHandler { cursorShape: b.enabledState ? Qt.PointingHandCursor : Qt.ArrowCursor }
    }

    Rectangle {
        id: box
        anchors.centerIn: parent
        width: Math.min(560, parent.width - 48)
        height: col.implicitHeight + 40
        radius: 8
        color: "#1a1a2e"
        border.color: "#555555"
        border.width: 1
        MouseArea { anchors.fill: parent }   // clicks inside stay inside

        Column {
            id: col
            x: 24
            y: 20
            width: box.width - 48
            spacing: 10

            Text {
                text: Theme.tr(Updates.state === "ready" ? "UPDATE DOWNLOADED" : "UPDATE AVAILABLE")
                color: Theme.accentCyan
                font.family: Theme.fontUi
                font.pixelSize: 18
                font.weight: Font.Bold
                font.letterSpacing: 2
            }
            Text {
                width: parent.width
                wrapMode: Text.Wrap
                text: "CORV GCS " + Updates.latest + qsTr("   ·   installed ") + Updates.version
                      + (Updates.packageSize > 0 ? "   ·   " + (Updates.packageSize / 1048576).toFixed(0) + " MB" : "")
                color: "#e0e0e0"
                font.family: Theme.fontData
                font.pixelSize: 13
            }
            // Release notes, as GitHub shows them (Markdown)
            Rectangle {
                width: parent.width
                height: Math.min(260, notes.implicitHeight + 20)
                visible: Updates.notes.length > 0
                radius: 4
                color: "#12121f"
                border.color: "#333344"
                border.width: 1
                Flickable {
                    id: flick
                    anchors.fill: parent
                    anchors.margins: 10
                    clip: true
                    contentHeight: notes.implicitHeight
                    boundsBehavior: Flickable.StopAtBounds
                    ScrollBar.vertical: ScrollBar { policy: flick.contentHeight > flick.height ? ScrollBar.AlwaysOn : ScrollBar.AlwaysOff }
                    Text {
                        id: notes
                        width: flick.width - 12
                        wrapMode: Text.Wrap
                        textFormat: Text.MarkdownText
                        text: Updates.notes
                        color: "#c8c8d0"
                        linkColor: Theme.accentCyan
                        font.family: Theme.fontData
                        font.pixelSize: 12
                        onLinkActivated: (link) => Qt.openUrlExternally(link)
                    }
                }
            }
            Text {
                width: parent.width
                wrapMode: Text.Wrap
                text: Theme.tr(Updates.installText)
                color: Theme.textDim
                font.family: Theme.fontData
                font.pixelSize: 12
            }
            // Download progress
            Rectangle {
                visible: Updates.state === "downloading"
                width: parent.width
                height: 6
                radius: 3
                color: "#2a2a3e"
                Rectangle {
                    width: parent.width * Updates.progress
                    height: parent.height
                    radius: 3
                    color: Theme.accentCyan
                }
            }
            Text {
                visible: Updates.message.length > 0 && Updates.state !== "available"
                width: parent.width
                wrapMode: Text.Wrap
                text: Updates.message + (Updates.state === "downloading" ? "  " + (Updates.progress * 100).toFixed(0) + " %" : "")
                color: Updates.state === "error" ? "#ff6060" : "#e0e0e0"
                font.family: Theme.fontData
                font.pixelSize: 12
            }
            Item { width: 1; height: 4 }
            Item {
                width: parent.width
                height: buttons.height
                DlgButton {
                    anchors.left: parent.left
                    text: "RELEASE PAGE"
                    onClicked: Updates.openPage()
                }
                Row {
                    id: buttons
                    anchors.right: parent.right
                    spacing: 10
                    DlgButton {
                        visible: !dlg.busy && Updates.state !== "ready"
                        text: "SKIP THIS VERSION"
                        onClicked: { Updates.skip(); dlg.open = false }
                    }
                    DlgButton {
                        text: dlg.busy ? "HIDE" : "LATER"
                        onClicked: dlg.open = false
                    }
                    DlgButton {
                        visible: Updates.state === "downloading"
                        text: "CANCEL"
                        onClicked: Updates.cancel()
                    }
                    DlgButton {
                        visible: !dlg.busy && Updates.state !== "ready"
                        primary: true
                        text: "INSTALL"
                        onClicked: Updates.install()
                    }
                }
            }
        }
    }
}
