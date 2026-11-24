// The in-page alert / confirm / prompt of index.html and MissionLibrary.js:
// toasts at the top centre (4 s, click to dismiss), a modal confirm (Enter =
// OK, Escape = Cancel, a click outside cancels) and a modal text prompt.
// Fed by Plan (confirmRequested, promptRequested, toast) and SysConfig (toast).
import QtQuick
import QtQuick.Controls.Basic
import Corv.Ui

Item {
    id: host
    objectName: "dialogs"   // the test commands answer a question with close(true)
    anchors.fill: parent

    property int token: -1
    property string mode: ""        // confirm | prompt
    property string message: ""

    function toast(text) { toasts.append({ text: text }) }

    Connections {
        target: Plan
        function onToast(text) { host.toast(text) }
        function onConfirmRequested(t, msg) { host.hubQuestion = false; host.token = t; host.message = msg; host.mode = "confirm"; okBtn.forceActiveFocus() }
        function onPromptRequested(t, msg, value) { host.hubQuestion = false; host.token = t; host.message = msg; host.mode = "prompt"; input.text = value; input.forceActiveFocus(); input.selectAll() }
    }
    Connections {
        target: SysConfig
        function onToast(text) { host.toast(text) }
    }
    // Every other view model asks through DialogHub
    property bool hubQuestion: false
    Connections {
        target: DialogHub
        function onToastRequested(text) { host.toast(text) }
        function onConfirmRequested(t, msg) { host.hubQuestion = true; host.token = t; host.message = msg; host.mode = "confirm"; okBtn.forceActiveFocus() }
        function onPromptRequested(t, msg, value) { host.hubQuestion = true; host.token = t; host.message = msg; host.mode = "prompt"; input.text = value; input.forceActiveFocus(); input.selectAll() }
    }

    function close(ok) {
        const t = token
        const text = input.text
        const hub = hubQuestion
        mode = ""
        token = -1
        hubQuestion = false
        if (hub) DialogHub.answer(t, ok, text)
        else Plan.answer(t, ok, text)
    }

    // ── Toasts ──
    ListModel { id: toasts }
    Column {
        anchors.horizontalCenter: parent.horizontalCenter
        y: 48
        spacing: 8
        z: 2
        Repeater {
            model: toasts
            delegate: Rectangle {
                id: t
                required property string text
                required property int index
                anchors.horizontalCenter: parent.horizontalCenter
                width: Math.min(500, tt.implicitWidth + 40)
                height: tt.implicitHeight + 20
                radius: 6
                color: "#1a1a2e"
                border.color: "#444444"
                border.width: 1
                opacity: 0
                Component.onCompleted: opacity = 1
                Behavior on opacity { NumberAnimation { duration: 200 } }
                Text {
                    id: tt
                    anchors.centerIn: parent
                    width: Math.min(460, implicitWidth)
                    horizontalAlignment: Text.AlignHCenter
                    wrapMode: Text.Wrap
                    text: Theme.tr(t.text)
                    color: "#e0e0e0"
                    font.family: Theme.fontUi
                    font.pixelSize: 13
                }
                Timer { interval: 4000; running: true; onTriggered: t.dismiss() }
                function dismiss() { opacity = 0; removeTimer.start() }
                Timer { id: removeTimer; interval: 200; onTriggered: { for (let i = 0; i < toasts.count; ++i) if (toasts.get(i).text === t.text) { toasts.remove(i); break } } }
                TapHandler { onTapped: t.dismiss() }
            }
        }
    }

    // ── Modal ──
    Rectangle {
        visible: host.mode.length > 0
        anchors.fill: parent
        color: Qt.rgba(0, 0, 0, 0.55)
        z: 1
        MouseArea { anchors.fill: parent; onClicked: host.close(false) }
        Keys.onEscapePressed: host.close(false)

        Rectangle {
            id: box
            anchors.centerIn: parent
            width: host.mode === "prompt" ? Math.max(320, Math.min(440, msg.implicitWidth + 48)) : Math.max(260, Math.min(420, msg.implicitWidth + 56))
            height: col.implicitHeight + 40
            radius: 8
            color: "#1a1a2e"
            border.color: "#555555"
            border.width: 1
            MouseArea { anchors.fill: parent }   // clicks inside stay inside
            Column {
                id: col
                x: host.mode === "prompt" ? 24 : 28
                y: 20
                width: box.width - 2 * x
                Text {
                    id: msg
                    width: parent.width
                    wrapMode: Text.Wrap
                    horizontalAlignment: host.mode === "prompt" ? Text.AlignLeft : Text.AlignHCenter
                    text: Theme.tr(host.message)
                    color: "#e0e0e0"
                    font.family: Theme.fontData
                    font.pixelSize: 13
                    lineHeight: host.mode === "prompt" ? 1 : 1.5
                }
                Item { width: 1; height: host.mode === "prompt" ? 12 : 18 }
                TextField {
                    id: input
                    objectName: "dialogInput"
                    visible: host.mode === "prompt"
                    width: parent.width
                    height: visible ? 32 : 0
                    leftPadding: 10
                    color: Theme.textMain
                    font.family: Theme.fontData
                    font.pixelSize: 13
                    selectByMouse: true
                    background: Rectangle { radius: 4; color: Theme.inputBg; border.color: input.activeFocus ? Theme.accentCyan : Theme.borderInput; border.width: 1 }
                    onAccepted: host.close(true)
                    Keys.onShortcutOverride: (e) => e.accepted = (e.key === Qt.Key_Escape)
                    Keys.onEscapePressed: host.close(false)
                }
                Item { width: 1; height: host.mode === "prompt" ? 16 : 0 }
                Row {
                    spacing: 12
                    anchors.horizontalCenter: host.mode === "prompt" ? undefined : parent.horizontalCenter
                    anchors.right: host.mode === "prompt" ? parent.right : undefined
                    Rectangle {
                        width: cancelText.implicitWidth + 44
                        height: cancelText.implicitHeight + 12
                        radius: 4
                        color: "#2a2a3e"
                        border.color: "#555555"
                        border.width: 1
                        Text { id: cancelText; anchors.centerIn: parent; text: qsTr("Cancel"); color: "#cccccc"; font.pixelSize: 12 }
                        TapHandler { onTapped: host.close(false) }
                        HoverHandler { cursorShape: Qt.PointingHandCursor }
                    }
                    Rectangle {
                        id: okBtn
                        width: okText.implicitWidth + 44
                        height: okText.implicitHeight + 12
                        radius: 4
                        color: Theme.accentCyan
                        focus: host.mode === "confirm"
                        Keys.onShortcutOverride: (e) => e.accepted = (e.key === Qt.Key_Escape || e.key === Qt.Key_Return || e.key === Qt.Key_Enter)
                        Keys.onReturnPressed: host.close(true)
                        Keys.onEnterPressed: host.close(true)
                        Keys.onEscapePressed: host.close(false)
                        Text { id: okText; anchors.centerIn: parent; text: qsTr("OK"); color: "#000000"; font.pixelSize: 12; font.bold: true }
                        TapHandler { onTapped: host.close(true) }
                        HoverHandler { cursorShape: Qt.PointingHandCursor }
                    }
                }
            }
        }
    }
}
