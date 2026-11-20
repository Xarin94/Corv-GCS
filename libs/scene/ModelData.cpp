#include "scene/ModelData.h"

#include <QHash>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMatrix4x4>
#include <QQuaternion>
#include <QVector3D>
#include <QtEndian>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

using namespace Qt::StringLiterals;

namespace corv {

namespace {

constexpr quint32 kGlbMagic = 0x46546C67;   // "glTF"
constexpr quint32 kChunkJson = 0x4E4F534A;
constexpr quint32 kChunkBin = 0x004E4942;

struct Accessor {
    const char *data = nullptr;
    int count = 0;
    int components = 0;
    int componentType = 0;
    int stride = 0;
};

int componentCount(const QString &type)
{
    if (type == "SCALAR"_L1) return 1;
    if (type == "VEC2"_L1) return 2;
    if (type == "VEC3"_L1) return 3;
    if (type == "VEC4"_L1) return 4;
    if (type == "MAT4"_L1) return 16;
    return 0;
}

int componentSize(int t)
{
    switch (t) {
    case 5120: case 5121: return 1;
    case 5122: case 5123: return 2;
    case 5125: case 5126: return 4;
    default: return 0;
    }
}

class Glb
{
public:
    QJsonObject json;
    QByteArray bin;
    QString error;

    bool accessor(int index, Accessor *out) const
    {
        const QJsonArray accessors = json.value(u"accessors"_s).toArray();
        if (index < 0 || index >= accessors.size()) return false;
        const QJsonObject a = accessors[index].toObject();
        const int viewIndex = a.value(u"bufferView"_s).toInt(-1);
        const QJsonArray views = json.value(u"bufferViews"_s).toArray();
        if (viewIndex < 0 || viewIndex >= views.size()) return false;
        const QJsonObject v = views[viewIndex].toObject();
        if (v.value(u"buffer"_s).toInt(0) != 0) return false;
        out->count = a.value(u"count"_s).toInt();
        out->components = componentCount(a.value(u"type"_s).toString());
        out->componentType = a.value(u"componentType"_s).toInt();
        const int elem = out->components * componentSize(out->componentType);
        if (!elem) return false;
        out->stride = v.value(u"byteStride"_s).toInt(elem);
        const qint64 offset = qint64(v.value(u"byteOffset"_s).toInt(0)) + a.value(u"byteOffset"_s).toInt(0);
        const qint64 need = offset + qint64(out->stride) * std::max(0, out->count - 1) + elem;
        if (offset < 0 || need > bin.size()) return false;
        out->data = bin.constData() + offset;
        return true;
    }
};

float readFloat(const Accessor &a, int i, int c)
{
    const char *p = a.data + qint64(i) * a.stride;
    switch (a.componentType) {
    case 5126: {
        float f;
        std::memcpy(&f, p + c * 4, 4);
        return qFromLittleEndian(f);
    }
    case 5121: return float(quint8(p[c])) / 255.f;
    case 5123: return float(qFromLittleEndian<quint16>(p + c * 2)) / 65535.f;
    default: return 0;
    }
}

quint32 readIndex(const Accessor &a, int i)
{
    const char *p = a.data + qint64(i) * a.stride;
    switch (a.componentType) {
    case 5121: return quint8(p[0]);
    case 5123: return qFromLittleEndian<quint16>(p);
    case 5125: return qFromLittleEndian<quint32>(p);
    default: return 0;
    }
}

QMatrix4x4 nodeMatrix(const QJsonObject &n)
{
    QMatrix4x4 m;
    const QJsonArray mat = n.value(u"matrix"_s).toArray();
    if (mat.size() == 16) {
        float v[16];
        for (int i = 0; i < 16; ++i) v[i] = float(mat[i].toDouble());
        // glTF is column major, QMatrix4x4(const float*) takes row major
        return QMatrix4x4(v).transposed();
    }
    const QJsonArray t = n.value(u"translation"_s).toArray();
    if (t.size() == 3) m.translate(float(t[0].toDouble()), float(t[1].toDouble()), float(t[2].toDouble()));
    const QJsonArray r = n.value(u"rotation"_s).toArray();
    if (r.size() == 4) m.rotate(QQuaternion(float(r[3].toDouble()), float(r[0].toDouble()), float(r[1].toDouble()), float(r[2].toDouble())));
    const QJsonArray s = n.value(u"scale"_s).toArray();
    if (s.size() == 3) m.scale(float(s[0].toDouble()), float(s[1].toDouble()), float(s[2].toDouble()));
    return m;
}

void pushVertex(std::vector<float> &out, const QVector3D &p, const QVector3D &n, const QVector3D &c)
{
    out.insert(out.end(), {p.x(), p.y(), p.z(), n.x(), n.y(), n.z(), c.x(), c.y(), c.z()});
}

void addPrimitive(const Glb &g, const QJsonObject &prim, const QMatrix4x4 &m, std::vector<float> &out)
{
    const int mode = prim.value(u"mode"_s).toInt(4);
    if (mode != 4) return;   // triangles only
    const QJsonObject attrs = prim.value(u"attributes"_s).toObject();
    Accessor pos, nrm, idx;
    if (!g.accessor(attrs.value(u"POSITION"_s).toInt(-1), &pos) || pos.components != 3) return;
    const bool hasNormals = g.accessor(attrs.value(u"NORMAL"_s).toInt(-1), &nrm) && nrm.components == 3 && nrm.count == pos.count;
    const bool indexed = prim.contains(u"indices"_s) && g.accessor(prim.value(u"indices"_s).toInt(-1), &idx);

    QVector3D colour(0.8f, 0.8f, 0.8f);
    const QJsonArray materials = g.json.value(u"materials"_s).toArray();
    const int mi = prim.value(u"material"_s).toInt(-1);
    if (mi >= 0 && mi < materials.size()) {
        const QJsonArray f = materials[mi].toObject().value(u"pbrMetallicRoughness"_s).toObject()
                                 .value(u"baseColorFactor"_s).toArray();
        if (f.size() >= 3) colour = QVector3D(float(f[0].toDouble()), float(f[1].toDouble()), float(f[2].toDouble()));
    }
    const QMatrix3x3 nm = m.normalMatrix();
    auto vertexAt = [&](quint32 i, QVector3D *p, QVector3D *n) {
        *p = m.map(QVector3D(readFloat(pos, int(i), 0), readFloat(pos, int(i), 1), readFloat(pos, int(i), 2)));
        if (hasNormals) {
            const float x = readFloat(nrm, int(i), 0), y = readFloat(nrm, int(i), 1), z = readFloat(nrm, int(i), 2);
            *n = QVector3D(nm(0, 0) * x + nm(0, 1) * y + nm(0, 2) * z, nm(1, 0) * x + nm(1, 1) * y + nm(1, 2) * z,
                           nm(2, 0) * x + nm(2, 1) * y + nm(2, 2) * z).normalized();
        }
    };
    const int count = indexed ? idx.count : pos.count;
    for (int t = 0; t + 2 < count; t += 3) {
        QVector3D p[3], n[3];
        for (int k = 0; k < 3; ++k) {
            const quint32 vi = indexed ? readIndex(idx, t + k) : quint32(t + k);
            if (vi >= quint32(pos.count)) return;
            vertexAt(vi, &p[k], &n[k]);
        }
        if (!hasNormals) n[0] = n[1] = n[2] = QVector3D::normal(p[0], p[1], p[2]);
        for (int k = 0; k < 3; ++k) pushVertex(out, p[k], n[k], colour);
    }
}

void addNode(const Glb &g, int index, const QMatrix4x4 &parent, std::vector<float> &out, int depth)
{
    const QJsonArray nodes = g.json.value(u"nodes"_s).toArray();
    if (index < 0 || index >= nodes.size() || depth > 32) return;
    const QJsonObject n = nodes[index].toObject();
    const QMatrix4x4 m = parent * nodeMatrix(n);
    const QJsonArray meshes = g.json.value(u"meshes"_s).toArray();
    const int mesh = n.value(u"mesh"_s).toInt(-1);
    if (mesh >= 0 && mesh < meshes.size())
        for (const QJsonValue &p : meshes[mesh].toObject().value(u"primitives"_s).toArray()) addPrimitive(g, p.toObject(), m, out);
    for (const QJsonValue &c : n.value(u"children"_s).toArray()) addNode(g, c.toInt(-1), m, out, depth + 1);
}

/// Pivot on the bounding box centre (the recentre of loadModel) and the bounding radius.
void finish(ModelData &d)
{
    if (d.vertices.empty()) return;
    float lo[3] = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
    float hi[3] = {-lo[0], -lo[1], -lo[2]};
    for (std::size_t i = 0; i < d.vertices.size(); i += 9)
        for (int k = 0; k < 3; ++k) {
            lo[k] = std::min(lo[k], d.vertices[i + std::size_t(k)]);
            hi[k] = std::max(hi[k], d.vertices[i + std::size_t(k)]);
        }
    float r2 = 0;
    for (std::size_t i = 0; i < d.vertices.size(); i += 9) {
        float s = 0;
        for (int k = 0; k < 3; ++k) {
            d.vertices[i + std::size_t(k)] -= 0.5f * (lo[k] + hi[k]);
            s += d.vertices[i + std::size_t(k)] * d.vertices[i + std::size_t(k)];
        }
        r2 = std::max(r2, s);
    }
    d.radius = std::sqrt(r2);
}

} // namespace

std::shared_ptr<ModelData> ModelData::fromGlb(const QByteArray &bytes, float scale, const QString &name, QString *error)
{
    auto fail = [error](const QString &e) -> std::shared_ptr<ModelData> {
        if (error) *error = e;
        return nullptr;
    };
    if (bytes.size() < 20 || qFromLittleEndian<quint32>(bytes.constData()) != kGlbMagic) return fail(u"not a glTF binary"_s);
    if (qFromLittleEndian<quint32>(bytes.constData() + 4) != 2) return fail(u"glTF version is not 2"_s);
    Glb g;
    qint64 off = 12;
    while (off + 8 <= bytes.size()) {
        const quint32 len = qFromLittleEndian<quint32>(bytes.constData() + off);
        const quint32 type = qFromLittleEndian<quint32>(bytes.constData() + off + 4);
        if (off + 8 + qint64(len) > bytes.size()) return fail(u"truncated chunk"_s);
        const QByteArray chunk = bytes.mid(off + 8, len);
        if (type == kChunkJson) g.json = QJsonDocument::fromJson(chunk).object();
        else if (type == kChunkBin && g.bin.isEmpty()) g.bin = chunk;
        off += 8 + len;
    }
    if (g.json.isEmpty()) return fail(u"no JSON chunk"_s);

    // Electron: model.rotation.y = PI / 2 and the scale, then recentred
    QMatrix4x4 root;
    root.rotate(90.f, 0, 1, 0);
    root.scale(scale);
    auto d = std::make_shared<ModelData>();
    d->name = name;
    const QJsonArray scenes = g.json.value(u"scenes"_s).toArray();
    const int sceneIndex = g.json.value(u"scene"_s).toInt(0);
    if (sceneIndex >= 0 && sceneIndex < scenes.size()) {
        for (const QJsonValue &n : scenes[sceneIndex].toObject().value(u"nodes"_s).toArray()) addNode(g, n.toInt(-1), root, d->vertices, 0);
    } else {
        const int nodeCount = int(g.json.value(u"nodes"_s).toArray().size());
        for (int i = 0; i < nodeCount; ++i) addNode(g, i, root, d->vertices, 0);
    }
    if (d->vertices.empty()) return fail(u"no triangles"_s);
    finish(*d);
    return d;
}

std::shared_ptr<ModelData> ModelData::placeholder()
{
    auto d = std::make_shared<ModelData>();
    d->name = u"placeholder"_s;
    // Boxes in the files' frame (nose on +X), then the same quarter turn as a model
    QMatrix4x4 root;
    root.rotate(90.f, 0, 1, 0);
    auto box = [&](QVector3D c, QVector3D size, QVector3D colour) {
        const QVector3D h = size / 2;
        static const int faces[6][4] = {{0, 1, 3, 2}, {4, 6, 7, 5}, {0, 4, 5, 1}, {2, 3, 7, 6}, {0, 2, 6, 4}, {1, 5, 7, 3}};
        QVector3D v[8];
        for (int i = 0; i < 8; ++i)
            v[i] = root.map(c + QVector3D(i & 4 ? h.x() : -h.x(), i & 2 ? h.y() : -h.y(), i & 1 ? h.z() : -h.z()));
        for (const auto &f : faces) {
            const QVector3D n = QVector3D::normal(v[f[0]], v[f[1]], v[f[2]]);
            for (const int k : {0, 1, 2, 0, 2, 3}) pushVertex(d->vertices, v[f[k]], n, colour);
        }
    };
    const QVector3D body(0.87f, 0.87f, 0.87f), dark(0.13f, 0.13f, 0.13f);
    box({0, 0, 0}, {10, 2.6f, 2.6f}, body);       // fuselage
    box({6.3f, 0, 0}, {3, 1.6f, 1.6f}, body);      // nose
    box({-0.5f, 0, 0}, {2.2f, 0.25f, 10}, dark);   // wing
    box({-5, 0.2f, 0}, {1.2f, 0.2f, 3}, dark);     // tail
    box({-5.3f, 0.9f, 0}, {1.0f, 1.6f, 0.2f}, dark); // fin
    finish(*d);
    return d;
}

QString modelForVehicleType(int t)
{
    static const QHash<int, const char *> map = {
        {1, "plane.glb"}, {2, "quadcopter.glb"}, {3, "helicopter.glb"}, {4, "helicopter.glb"}, {5, "antenna-tracker.glb"},
        {7, "airship.glb"}, {8, "airship.glb"}, {10, "rover.glb"}, {11, "boat.glb"}, {12, "rov.glb"},
        {13, "hexacopter.glb"}, {14, "octocopter.glb"}, {15, "tricopter.glb"}, {16, "flying-wing.glb"},
        {19, "quadplane-vtol.glb"}, {20, "quadplane-vtol.glb"}, {21, "quadplane-vtol.glb"}, {22, "quadplane-vtol.glb"},
        {23, "quadplane-vtol.glb"}, {24, "quadplane-vtol.glb"}, {25, "quadplane-vtol.glb"}, {27, "octocopter.glb"},
        {29, "octocopter.glb"}};
    return QString::fromLatin1(map.value(t, "plane.glb"));
}

} // namespace corv
