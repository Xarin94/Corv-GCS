// A select bound to one vehicle parameter (FLTMODEn, FS_*_ACT...): the option
// of the value in ParamStore, else the default of index.html.
import QtQuick

CfgRow {
    id: ps
    property string param
    property var options: []
    property string fallback: ""
    property real fieldWidth: 200
    readonly property string value: sel.value
    readonly property var current: SetupCfg.paramRev >= 0 ? SetupCfg.param(param) : undefined
    onCurrentChanged: if (current !== undefined) sel.value = String(Math.round(current))
    Component.onCompleted: sel.value = current !== undefined ? String(Math.round(current)) : fallback
    CfgSelect {
        id: sel
        width: ps.fieldWidth
        options: ps.options
        onPicked: (v) => value = v
    }
}
