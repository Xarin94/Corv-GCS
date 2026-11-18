// ModelData.h - a vehicle model as the renderer draws it (the GLTFLoader part of
// js/main.js loadModel), without GPU resources.
//
// The models in resources/models are glTF binaries generated at true scale by
// scripts/gen-models.js: triangles with POSITION and NORMAL and one base colour
// per material, no textures. The loader flattens every node of the default
// scene into one triangle list of (position, normal, colour) and applies what
// Electron did to the loaded scene: a quarter turn about Y (the files have the
// nose on +X, the vehicle frame looks down -Z), the model scale, and the
// bounding box centre moved to the origin, the pivot of the attitude.
#pragma once

#include <QByteArray>
#include <QString>
#include <memory>
#include <vector>

namespace corv {

struct ModelData {
    QString name;
    /// 9 floats per vertex: x y z, nx ny nz, r g b; 3 vertices per triangle.
    std::vector<float> vertices;
    float radius = 0;   // of the bounding sphere around the pivot, metres

    int vertexCount() const { return int(vertices.size() / 9); }

    /// Parse a .glb. Null with *error set when it is not one this loader reads.
    static std::shared_ptr<ModelData> fromGlb(const QByteArray &glb, float scale, const QString &name, QString *error = nullptr);
    /// The placeholder of createPlaceholderModel(): fuselage, nose, wing, tail and fin boxes.
    static std::shared_ptr<ModelData> placeholder();
};

/// Model file for a MAV_TYPE (MODEL_BY_MAV_TYPE of js/main.js); plane.glb by default.
QString modelForVehicleType(int mavType);

} // namespace corv
