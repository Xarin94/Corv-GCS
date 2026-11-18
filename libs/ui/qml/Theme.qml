// Design tokens of css/variables.css (dark theme), with the fluid sizes of the
// Electron interface: the same clamp(min, min(vw, vh), max) formulas, fed by
// the window size (Main.qml sets winW / winH).
pragma Singleton
import QtQuick

QtObject {
    id: t
    // Window size in logical px (CSS px of the Electron version at the same scale)
    property real winW: 1600
    property real winH: 900
    readonly property real vw: winW / 100
    readonly property real vh: winH / 100
    function clamp(lo, v, hi) { return Math.max(lo, Math.min(hi, v)) }

    // --fs-* type scale
    readonly property real fs2xs: clamp(8, Math.min(0.55 * vw, 1.05 * vh), 12)
    readonly property real fsXs: clamp(9, Math.min(0.65 * vw, 1.25 * vh), 14)
    readonly property real fsSm: clamp(10, Math.min(0.76 * vw, 1.45 * vh), 16)
    readonly property real fsMd: clamp(11, Math.min(0.88 * vw, 1.70 * vh), 19)
    readonly property real fsLg: clamp(13, Math.min(1.10 * vw, 2.10 * vh), 24)
    readonly property real fsXl: clamp(15, Math.min(1.50 * vw, 2.90 * vh), 32)
    readonly property real fs2xl: clamp(24, Math.min(2.60 * vw, 5.00 * vh), 56)

    // Layout
    readonly property real gutter: clamp(10, vw, 22)
    readonly property real chromeTop: 92          // header 60 + tab bar 32
    readonly property real cmdBarH: 46
    readonly property real minimapW: clamp(200, 15 * vw, 340)
    readonly property real minimapH: clamp(140, 10.4 * vw, 236)
    readonly property real annunW: clamp(84, 6.2 * vw, 130)
    readonly property real ekfBarW: clamp(300, 21 * vw, 440)    // EKF STATUS / FFT bar on the left
    readonly property real hairline: 0.6667       // 1 device pixel at 1.5x, as the 1px CSS borders

    // Colours
    readonly property color bg: "#080c12"
    readonly property color accentCyan: "#00d2ff"
    readonly property color accentRed: "#ff0000"
    readonly property color accentActive: "#ff9900"
    readonly property color accentGreen: "#44ff44"
    readonly property color accentOrange: "#ffaa00"
    readonly property color modeManual: "#ffcc00"
    readonly property color hudGreen: "#00ff7f"
    readonly property color statusRed: "#ff4444"

    readonly property color textMain: "#ffffff"
    readonly property color textDim: "#6e7f8d"
    readonly property color textMuted: "#889999"
    readonly property color textNav: "#88aacc"
    readonly property color textMutedBright: "#cfd8dd"
    readonly property color hudText: "#ffffff"
    readonly property color hudTextSub: "#dde5ea"
    readonly property color hudTextLbl: "#d8e4eb"
    readonly property color groupAmber: "#ffbf00"

    readonly property color glassPanel: Qt.rgba(10 / 255, 15 / 255, 20 / 255, 0.95)
    readonly property color glassToolbar: Qt.rgba(8 / 255, 12 / 255, 18 / 255, 0.97)
    readonly property color toggleBg: Qt.rgba(8 / 255, 12 / 255, 18 / 255, 0.95)
    readonly property color toggleBgHover: Qt.rgba(20 / 255, 30 / 255, 45 / 255, 0.95)
    readonly property color cmdButton: Qt.rgba(30 / 255, 40 / 255, 55 / 255, 0.9)
    readonly property color cmdButtonHover: Qt.rgba(50 / 255, 65 / 255, 85 / 255, 0.9)
    readonly property color cmdSelect: Qt.rgba(20 / 255, 30 / 255, 40 / 255, 0.9)
    readonly property color gcsButton: Qt.rgba(20 / 255, 30 / 255, 45 / 255, 0.9)
    readonly property color gcsButtonHover: Qt.rgba(35 / 255, 50 / 255, 70 / 255, 0.9)
    readonly property color inputBg: Qt.rgba(10 / 255, 15 / 255, 22 / 255, 0.9)
    readonly property color squareBg: Qt.rgba(0, 0, 0, 0.9)

    readonly property color borderAccent: Qt.rgba(0, 210 / 255, 1, 0.15)
    readonly property color borderAccentMid: Qt.rgba(0, 210 / 255, 1, 0.4)
    readonly property color borderTab: Qt.rgba(0, 210 / 255, 1, 0.1)
    readonly property color borderPanel: Qt.rgba(0, 210 / 255, 1, 0.12)
    readonly property color borderSubtle: Qt.rgba(1, 1, 1, 0.08)
    readonly property color borderInput: Qt.rgba(1, 1, 1, 0.1)
    readonly property color borderSection: Qt.rgba(1, 1, 1, 0.06)
    // Kept for the earlier components
    readonly property color border: borderAccentMid
    readonly property color borderDim: borderSubtle
    readonly property color inputBgDark: inputBg

    // Rajdhani for the interface, Roboto Mono for the data (vendor/fonts)
    readonly property string fontUi: "Rajdhani"
    readonly property string fontData: "Roboto Mono"

    function modeColor(cls) {
        switch (cls) {
        case "manual": return modeManual
        case "assisted": return accentCyan
        case "auto": return accentGreen
        case "emergency": return accentActive
        }
        return accentCyan
    }
}
