// SETUP → FAILSAFE (#subtab-failsafe, initFailsafe): battery low / critical
// voltages and actions, RC throttle and GCS failsafes (FS_MAP).
import QtQuick
import QtQuick.Controls.Basic

Flickable {
    id: page
    contentHeight: flow.implicitHeight + 48
    clip: true
    readonly property var battActions: [{ value: "0", label: "Disabled" }, { value: "1", label: "Land" }, { value: "2", label: "RTL" },
                                        { value: "3", label: "SmartRTL or RTL" }, { value: "4", label: "SmartRTL or Land" }, { value: "5", label: "Terminate" }]
    readonly property var fsActions: [{ value: "0", label: "Disabled" }, { value: "1", label: "Enabled - Always RTL" },
                                      { value: "2", label: "Enabled - Continue Auto" }, { value: "3", label: "Enabled - Always Land" }]
    readonly property var names: ["BATT_LOW_VOLT", "BATT_CRT_VOLT", "BATT_FS_LOW_ACT", "BATT_FS_CRT_ACT", "FS_THR_ENABLE", "FS_THR_VALUE", "FS_GCS_ENABLE"]
    Column {
        id: flow
        x: 24; y: 24
        spacing: 21
        Row {
            spacing: 21
            SetupPanel {
                title: "BATTERY FAILSAFE"
                ParamField { id: lowV; label: "LOW VOLTAGE"; param: "BATT_LOW_VOLT"; decimals: 2; fallback: "10.5" }
                ParamField { id: crtV; label: "CRITICAL VOLTAGE"; param: "BATT_CRT_VOLT"; decimals: 2; fallback: "10.0" }
                ParamSelect { id: lowAct; label: "LOW ACTION"; param: "BATT_FS_LOW_ACT"; fallback: "2"; options: page.battActions }
                ParamSelect { id: crtAct; label: "CRITICAL ACTION"; param: "BATT_FS_CRT_ACT"; fallback: "1"; options: page.battActions }
            }
            SetupPanel {
                title: "RC / GCS FAILSAFE"
                ParamSelect { id: thrEn; label: "RC FAILSAFE"; param: "FS_THR_ENABLE"; fallback: "1"; options: page.fsActions }
                ParamField { id: thrVal; label: "RC PWM THRESHOLD"; param: "FS_THR_VALUE"; decimals: 0; fallback: "975" }
                ParamSelect { id: gcsEn; label: "GCS FAILSAFE"; param: "FS_GCS_ENABLE"; fallback: "1"; options: page.fsActions }
            }
        }
        Row {
            spacing: 6
            GcsButton { width: 220; height: 29; text: "READ FAILSAFE PARAMS"; onClicked: SetupCfg.readParams(page.names, "FAILSAFE") }
            GcsButton {
                width: 220; height: 29
                primary: true
                text: "WRITE FAILSAFE PARAMS"
                onClicked: SetupCfg.writeParams({
                    BATT_LOW_VOLT: lowV.text, BATT_CRT_VOLT: crtV.text, BATT_FS_LOW_ACT: lowAct.value, BATT_FS_CRT_ACT: crtAct.value,
                    FS_THR_ENABLE: thrEn.value, FS_THR_VALUE: thrVal.text, FS_GCS_ENABLE: gcsEn.value
                }, "FAILSAFE", "Written %1 failsafe parameters")
            }
        }
        CfgNote { width: 460; text: SetupCfg.status["FAILSAFE"] ? SetupCfg.status["FAILSAFE"].text : "" }
    }
}
