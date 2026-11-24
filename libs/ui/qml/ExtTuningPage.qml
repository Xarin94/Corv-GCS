// SETUP → EXT TUNING (#subtab-ext-tuning): sliders for a few gains and speeds;
// a slider may drive several parameters (RATE ROLL/PITCH sets both P gains).
import QtQuick
import QtQuick.Controls.Basic

Flickable {
    id: page
    contentHeight: col.implicitHeight + 48
    clip: true
    // [label, parameters, min, max, step, default]
    readonly property var sliders: [
        ["RATE ROLL/PITCH", ["ATC_RAT_RLL_P", "ATC_RAT_PIT_P"], 0.01, 0.5, 0.005, 0.135],
        ["RATE YAW", ["ATC_RAT_YAW_P"], 0.01, 0.5, 0.005, 0.18],
        ["LOITER SPEED (cm/s)", ["LOIT_SPEED"], 100, 2000, 50, 500],
        ["WP SPEED (cm/s)", ["WPNAV_SPEED"], 100, 2000, 50, 500],
        ["RTL ALT (cm)", ["RTL_ALT"], 0, 8000, 100, 1500],
        ["PILOT SPEED UP (cm/s)", ["PILOT_SPEED_UP"], 50, 500, 10, 250]
    ]
    Column {
        id: col
        x: 24; y: 24
        spacing: 21
        SetupPanel {
            title: "EXTENDED TUNING"
            width: 520
            Repeater {
                id: rows
                model: page.sliders
                delegate: CfgRow {
                    id: r
                    required property var modelData
                    readonly property var current: SetupCfg.paramRev >= 0 ? SetupCfg.param(modelData[1][0]) : undefined
                    property real value: modelData[5]
                    onCurrentChanged: if (current !== undefined) value = current
                    label: modelData[0]
                    CfgSlider {
                        width: 300
                        from: r.modelData[2]; to: r.modelData[3]; stepSize: r.modelData[4]
                        value: r.value
                        valueText: r.modelData[4] < 1 ? r.value.toFixed(3) : String(Math.round(r.value))
                        onMoved: (v) => r.value = v
                    }
                }
            }
            Row {
                spacing: 6
                GcsButton {
                    width: 200; height: 29
                    text: "READ FROM VEHICLE"
                    onClicked: SetupCfg.readParams([].concat(...page.sliders.map(s => s[1])), "EXT TUNING")
                }
                GcsButton {
                    width: 200; height: 29
                    primary: true
                    text: "WRITE TO VEHICLE"
                    onClicked: {
                        const v = {}
                        for (let i = 0; i < rows.count; ++i) for (const p of page.sliders[i][1]) v[p] = rows.itemAt(i).value
                        SetupCfg.writeParams(v, "EXT TUNING", "Written %1 tuning parameters")
                    }
                }
            }
            CfgNote { text: Theme.tr(SetupCfg.status["EXT TUNING"] ? SetupCfg.status["EXT TUNING"].text : "") }
        }
    }
}
