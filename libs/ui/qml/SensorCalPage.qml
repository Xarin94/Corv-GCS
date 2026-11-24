// SETUP → GYRO / BARO (#subtab-sensor-cal): one command each, its COMMAND_ACK
// the result. Logic in CalibrationController ("Calibration").
import QtQuick
import Corv.Ui

Item {
    id: page
    Row {
        y: 24
        anchors.horizontalCenter: parent.horizontalCenter
        spacing: 20
        SetupPanel {
            title: "GYROSCOPES"
            CfgNote { text: qsTr("Measures the gyro offsets. Keep the vehicle completely still for a few seconds. ArduPilot also does this at every boot unless INS_GYR_CAL is 0.") }
            CalButton {
                width: parent.width
                text: "CALIBRATE GYROS"
                enabled: !(Calibration.results.gyro && Calibration.results.gyro.busy)
                onClicked: Calibration.calibrateGyro()
            }
            CalResult { result: Calibration.results.gyro }
        }
        SetupPanel {
            title: "BAROMETER"
            CfgNote { text: qsTr("Takes the current pressure as ground level: the altitude reads zero here. Vehicle still, out of the wind and the prop wash.") }
            CalButton {
                width: parent.width
                text: "CALIBRATE BARO"
                enabled: !(Calibration.results.baro && Calibration.results.baro.busy)
                onClicked: Calibration.calibrateBaro()
            }
            CalResult { result: Calibration.results.baro }
        }
    }
}
