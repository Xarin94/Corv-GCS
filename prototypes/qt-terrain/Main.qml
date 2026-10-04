import QtQuick
import QtQuick3D

Item {
    width: 1280
    height: 720
    View3D {
        anchors.fill: parent
        environment: SceneEnvironment {
            backgroundMode: SceneEnvironment.Color
            clearColor: "#102333"
            antialiasingMode: SceneEnvironment.MSAA
            antialiasingQuality: SceneEnvironment.High
        }
        PerspectiveCamera {
            id: camera
            position: Qt.vector3d(0, 32000, 68000)
            eulerRotation.x: -25
            clipNear: 100
            clipFar: 300000
            fieldOfView: 60
        }
        CustomCamera {
            id: packetCamera
            position: packetCameraPosition
            rotation: packetCameraRotation
            projection: packetProjection
        }
        camera: packetMode ? packetCamera : camera
        DirectionalLight { eulerRotation: Qt.vector3d(-45, -30, 0); brightness: 1.5 }
        Node {
            id: terrainNode
            Model {
                visible: !packetMode
                geometry: terrainGeometry
                materials: PrincipledMaterial {
                    baseColor: "#6fa477"
                    roughness: 1
                    cullMode: Material.NoCulling
                }
            }
            Repeater3D {
                model: geometryList
                Model {
                    required property var modelData
                    geometry: modelData
                    materials: PrincipledMaterial { baseColor: "#6fa477"; roughness: 1; cullMode: Material.NoCulling }
                }
            }
            Repeater3D {
                model: pointGeometryList
                Model {
                    required property var modelData
                    geometry: modelData
                    materials: DefaultMaterial { diffuseColor: "#ff6677"; lighting: DefaultMaterial.NoLighting }
                }
            }
            NumberAnimation on eulerRotation.y {
                from: -12; to: 12; duration: 10000
                loops: Animation.Infinite
                running: !packetMode
                easing.type: Easing.InOutSine
            }
        }
    }
    Rectangle {
        anchors.left: parent.left; anchors.top: parent.top
        anchors.margins: 18
        width: 470; height: 106
        color: "#c020303c"; radius: 6
        Text {
            anchors.fill: parent; anchors.margins: 12
            color: "white"; font.pixelSize: 16
            text: "CORV · Qt Quick 3D terrain prototype\n" +
                  "HGT: " + tileName + " · " + triangleCount + " triangles\n" +
                  "Geometry/backend check · GCS feature parity pending"
        }
    }
    Rectangle { anchors.centerIn: parent; width: 40; height: 2; color: "#d0ffffff" }
    Rectangle { anchors.centerIn: parent; width: 2; height: 40; color: "#d0ffffff" }
}
