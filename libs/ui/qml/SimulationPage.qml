// SETUP → TOOLS → SIMULATION (#subtab-simulation, initSimulationTab): the
// ArduPilot SITL launcher and a manual UDP connection to a SITL elsewhere.
import QtQuick
import QtQuick.Controls.Basic

Flickable {
    id: page
    contentHeight: flow.implicitHeight + 48
    clip: true
    property string vehicle: "copter"
    function suggest() {
        const h = Sitl.suggestHome(vehicle, parseFloat(latIn.text), parseFloat(lonIn.text))
        latIn.text = String(h.lat)
        lonIn.text = String(h.lon)
    }
    Component.onCompleted: suggest()
    Flow {
        id: flow
        x: 24; y: 24
        width: page.width - 48
        spacing: 21
        SetupPanel {
            title: "SITL LAUNCHER (ArduPilot)"
            width: 420
            CfgRow {
                label: "VEHICLE TYPE"
                CfgSelect { width: 260; options: Sitl.vehicles; value: page.vehicle; onPicked: (v) => { page.vehicle = v; page.suggest() } }
            }
            CfgRow {
                label: "VERSION"
                CfgSelect {
                    id: versionSel
                    width: 160
                    options: [{ value: "stable", label: "Stable" }, { value: "beta", label: "Beta" }, { value: "latest", label: "Latest (Dev)" }]
                    value: "stable"
                    onPicked: (v) => value = v
                }
            }
            CfgRow {
                label: "HOME POSITION"
                CfgInput { id: latIn; width: 100; placeholderText: "Lat"; text: "47.2603" }
                CfgInput { id: lonIn; width: 100; placeholderText: "Lon"; text: "11.3439" }
            }
            CfgRow { label: "SPEED MULTIPLIER"; CfgInput { id: speedIn; width: 60; numeric: true; text: "1" } }
            Row {
                spacing: 6
                GcsButton {
                    width: 120; height: 29
                    text: Sitl.downloadText
                    blocked: Sitl.busy
                    onClicked: if (!Sitl.busy) Sitl.download(page.vehicle, versionSel.value)
                }
                GcsButton {
                    width: 160; height: 29
                    primary: true
                    text: Sitl.busy ? "STARTING..." : "LAUNCH & CONNECT"
                    blocked: Sitl.busy
                    onClicked: if (!Sitl.busy) Sitl.launch(page.vehicle, versionSel.value, parseFloat(latIn.text), parseFloat(lonIn.text), parseInt(speedIn.text) || 1)
                }
                GcsButton { width: 90; height: 29; text: "STOP"; onClicked: Sitl.stop() }
            }
            CfgValue { width: parent.width; horizontalAlignment: Text.AlignLeft; wrapMode: Text.Wrap; text: Sitl.status }
        }
        SetupPanel {
            title: "SITL MANUAL CONNECTION"
            CfgRow { label: "HOST"; CfgInput { id: hostIn; width: 160; text: "127.0.0.1" } }
            CfgRow { label: "PORT"; CfgInput { id: portIn; width: 80; numeric: true; text: "14550" } }
            GcsButton { width: parent.width; height: 29; text: "CONNECT UDP"; onClicked: Sitl.connectUdp(hostIn.text, parseInt(portIn.text) || 14550) }
        }
    }
}
