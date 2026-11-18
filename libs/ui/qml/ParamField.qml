// A number field bound to one vehicle parameter (.pid-input / .failsafe-input
// with data-param): shows the value in ParamStore ("--" until read) and keeps
// what the operator types until the parameter is read again.
import QtQuick

CfgRow {
    id: pf
    property string param
    property int decimals: 4
    property real fieldWidth: 100
    property string fallback: ""             // shown before any read (the value= of index.html)
    readonly property string text: input.text
    readonly property var current: SetupCfg.paramRev >= 0 ? SetupCfg.param(param) : undefined
    function refresh() { if (!input.activeFocus) input.text = current === undefined ? fallback : Number(current).toFixed(decimals) }
    onCurrentChanged: refresh()
    Component.onCompleted: refresh()
    CfgInput {
        id: input
        width: pf.fieldWidth
        numeric: true
        placeholderText: "--"
    }
}
