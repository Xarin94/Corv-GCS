// The SVG icons of html/index.html, as path data (circles and rects written as paths).
pragma Singleton
import QtQuick

QtObject {
    function circle(cx, cy, r) {
        return "M" + (cx - r) + " " + cy + "a" + r + " " + r + " 0 1 0 " + (2 * r) + " 0a" + r + " " + r + " 0 1 0 " + (-2 * r) + " 0z"
    }
    function rrect(x, y, w, h, r) {
        return "M" + (x + r) + " " + y + "h" + (w - 2 * r) + "a" + r + " " + r + " 0 0 1 " + r + " " + r + "v" + (h - 2 * r)
            + "a" + r + " " + r + " 0 0 1 " + (-r) + " " + r + "h" + (-(w - 2 * r)) + "a" + r + " " + r + " 0 0 1 " + (-r) + " " + (-r)
            + "v" + (-(h - 2 * r)) + "a" + r + " " + r + " 0 0 1 " + r + " " + (-r) + "z"
    }

    // Tab bar buttons
    readonly property var fpv: [
        { d: rrect(2, 5, 20, 14, 2), stroke: 2 },
        { d: circle(12, 12, 3), stroke: 2 },
        { d: circle(12, 12, 1), fill: true }
    ]
    readonly property var satellite: [{ d: "M3 6.5L9 4l6 2 6-2v13l-6 2-6-2-6 2v-13z", fill: true }]
    readonly property var sun: [
        { d: circle(12, 12, 5), fill: true },
        { d: "M12 2v2M12 20v2M4.22 4.22l1.42 1.42M18.36 18.36l1.42 1.42M2 12h2M20 12h2M4.22 19.78l1.42-1.42M18.36 5.64l1.42-1.42", stroke: 2, cap: "round" }
    ]
    readonly property var tilt: [
        { d: "M2 12H22", stroke: 2 },
        { d: "M9 12l3-4 3 4", stroke: 2, cap: "round" }
    ]
    readonly property var trajectory: [{ d: "M12 3v6M8 5l4-2 4 2M7 21c1-6 4-9 5-12M17 21c-1-6-4-9-5-12", stroke: 2, cap: "round" }]
    readonly property var theme: [
        { d: circle(12, 12, 8), stroke: 2 },
        { d: "M12 4a8 8 0 0 1 0 16V4z", fill: true }
    ]

    // Command bar
    readonly property var signal: [
        { d: rrect(0.6, 18, 3.6, 4, 0.6), fill: true },
        { d: rrect(5.4, 14, 3.6, 8, 0.6), fill: true },
        { d: rrect(10.2, 10, 3.6, 12, 0.6), fill: true },
        { d: rrect(15, 6, 3.6, 16, 0.6), fill: true },
        { d: rrect(19.8, 2, 3.6, 20, 0.6), fill: true }
    ]
    readonly property var battery: [
        { d: rrect(2, 6, 18, 12, 2), stroke: 2 },
        { d: "M22 10v4", stroke: 2, cap: "round" }
    ]
    readonly property var gps: [
        { d: circle(12, 12, 3), fill: true },
        { d: "M12 2v3M12 19v3M2 12h3M19 12h3", stroke: 2, cap: "round" },
        { d: circle(12, 12, 8), stroke: 1.5, opacity: 0.5 }
    ]
    readonly property var clock: [
        { d: circle(12, 12, 9), stroke: 2 },
        { d: "M12 7v5l3 3", stroke: 2, cap: "round" }
    ]

    // Title bar
    readonly property var minimize: [{ d: "M0 5h12v2H0z", fill: true }]
    readonly property var maximize: [{ d: "M1 1h10v10H1z", stroke: 2 }]
    readonly property var close: [{ d: "M1 1L11 11M11 1L1 11", stroke: 2 }]
}
