"""Native Qt Quick 3D HGT experiment, independent of the Electron application.

Mesh preparation happens before opening the view and is excluded from frame
measurements. This is a backend feasibility check, not a GCS speed comparison.
"""
import argparse
import array
import json
import math
import os
from pathlib import Path
import re
import sys
import threading
import time
sys.path.insert(0, str(Path(__file__).resolve().parent))
from render_packet import read_packet, terrain_mesh, point_positions


def build_mesh(filename, step):
    raw = Path(filename).read_bytes()
    size = math.isqrt(len(raw) // 2)
    if size not in (1201, 3601) or len(raw) != size * size * 2:
        raise ValueError("Expected a 1201 or 3601 signed big-endian HGT grid")
    if step <= 0 or (size - 1) % step:
        raise ValueError("--step must divide the tile cell count (1200 or 3600)")
    heights = array.array("h")
    heights.frombytes(raw)
    if sys.byteorder == "little":
        heights.byteswap()
    match = re.match(r"([NS])(\d{1,2})", Path(filename).name.upper())
    latitude = int(match[2]) * (-1 if match[1] == "S" else 1) if match else 45
    span_z = 111320.0
    span_x = span_z * math.cos(math.radians(latitude + 0.5))
    width = (size - 1) // step + 1
    dx, dz = span_x / (width - 1), span_z / (width - 1)
    vertices = array.array("f")
    indices = array.array("I")
    min_h, max_h = math.inf, -math.inf

    def height(row, col):
        value = heights[max(0, min(size - 1, row)) * size + max(0, min(size - 1, col))]
        return value if value > -12000 else 0

    for row in range(width):
        for col in range(width):
            r, c = row * step, col * step
            h = height(r, c)
            # Approximate normals from neighbouring samples in world metres.
            nx = -(height(r, c + step) - height(r, c - step)) / (2 * dx)
            nz = -(height(r + step, c) - height(r - step, c)) / (2 * dz)
            norm = math.sqrt(nx * nx + 1 + nz * nz)
            vertices.extend((col * dx - span_x / 2, h, row * dz - span_z / 2,
                             nx / norm, 1 / norm, nz / norm))
            min_h, max_h = min(min_h, h), max(max_h, h)
    for row in range(width - 1):
        for col in range(width - 1):
            a = row * width + col
            indices.extend((a, a + width, a + 1, a + 1, a + width, a + width + 1))
    return vertices.tobytes(), indices.tobytes(), (span_x, span_z, min_h, max_h), len(indices) // 3, size


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument("--hgt")
    input_group.add_argument("--packet", help="CRVG snapshot exported by the GCS, using its LOD and camera")
    parser.add_argument("--step", type=int, default=20)
    parser.add_argument("--seconds", type=float, default=15)
    parser.add_argument("--output", default="qt-terrain-result.json")
    parser.add_argument("--smoke", action="store_true", help="Transparent GPU window for functional checks; no performance claims")
    parser.add_argument("--offscreen", action="store_true", help="Headless QML check; platform may use software without 3D")
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")
    if args.offscreen:
        os.environ["QT_QPA_PLATFORM"] = "offscreen"
    os.environ.setdefault("QSG_RENDER_LOOP", "threaded")
    os.environ.setdefault("QSG_INFO", "1")
    from PySide6 import __version__ as qt_version
    from PySide6.QtCore import QByteArray, QTimer, Qt, QUrl, qVersion
    from PySide6.QtGui import QGuiApplication, QVector3D, QMatrix4x4, QQuaternion
    from PySide6.QtQuick import QQuickView
    from PySide6.QtQuick3D import QQuick3DGeometry

    start = time.perf_counter()
    document = read_packet(args.packet) if args.packet else None
    meshes = [terrain_mesh(r) for r in document["terrain"]] if document else []
    if document:
        triangles = sum(m[3] for m in meshes)
        vertex_data = index_data = b""
        bounds, size = (0,0,0,0), None
    else:
        vertex_data, index_data, bounds, triangles, size = build_mesh(args.hgt, args.step)
    prepare_ms = (time.perf_counter() - start) * 1000
    app = QGuiApplication(sys.argv[:1])
    view = QQuickView()
    if args.smoke and not args.offscreen:
        view.setFlags(Qt.WindowType.Tool | Qt.WindowType.FramelessWindowHint | Qt.WindowType.WindowDoesNotAcceptFocus)
        view.setOpacity(0)
    geometry = QQuick3DGeometry()
    geometry.setStride(24)
    geometry.setPrimitiveType(QQuick3DGeometry.PrimitiveType.Triangles)
    geometry.addAttribute(QQuick3DGeometry.Attribute.Semantic.PositionSemantic, 0,
                          QQuick3DGeometry.Attribute.ComponentType.F32Type)
    geometry.addAttribute(QQuick3DGeometry.Attribute.Semantic.NormalSemantic, 12,
                          QQuick3DGeometry.Attribute.ComponentType.F32Type)
    geometry.addAttribute(QQuick3DGeometry.Attribute.Semantic.IndexSemantic, 0,
                          QQuick3DGeometry.Attribute.ComponentType.U32Type)
    geometry.setVertexData(QByteArray(vertex_data))
    geometry.setIndexData(QByteArray(index_data))
    sx, sz, lo, hi = bounds
    geometry.setBounds(QVector3D(-sx / 2, lo, -sz / 2), QVector3D(sx / 2, hi, sz / 2))
    context = view.rootContext()
    context.setContextProperty("terrainGeometry", geometry)
    context.setContextProperty("tileName", Path(args.packet or args.hgt).name)
    context.setContextProperty("triangleCount", triangles)
    geometries, point_geometries = [], []
    for vertices, indices, box, _ in meshes:
        g = QQuick3DGeometry()
        g.setStride(24)
        g.setPrimitiveType(QQuick3DGeometry.PrimitiveType.Triangles)
        for semantic, offset, kind in [(QQuick3DGeometry.Attribute.Semantic.PositionSemantic, 0, QQuick3DGeometry.Attribute.ComponentType.F32Type),
                                      (QQuick3DGeometry.Attribute.Semantic.NormalSemantic, 12, QQuick3DGeometry.Attribute.ComponentType.F32Type),
                                      (QQuick3DGeometry.Attribute.Semantic.IndexSemantic, 0, QQuick3DGeometry.Attribute.ComponentType.U32Type)]:
            g.addAttribute(semantic, offset, kind)
        g.setVertexData(QByteArray(vertices)); g.setIndexData(QByteArray(indices))
        g.setBounds(QVector3D(*box[:3]), QVector3D(*box[3:]))
        geometries.append(g)
    for cloud in document["pointClouds"] if document else []:
        positions = point_positions(cloud, document["timeSeconds"])
        if not positions:
            continue
        g = QQuick3DGeometry()
        g.setStride(12); g.setPrimitiveType(QQuick3DGeometry.PrimitiveType.Points)
        g.addAttribute(QQuick3DGeometry.Attribute.Semantic.PositionSemantic, 0, QQuick3DGeometry.Attribute.ComponentType.F32Type)
        g.setVertexData(QByteArray(positions.tobytes()))
        g.setBounds(QVector3D(*(min(positions[k::3]) for k in range(3))), QVector3D(*(max(positions[k::3]) for k in range(3))))
        point_geometries.append(g)
    context.setContextProperty("packetMode", document is not None)
    context.setContextProperty("geometryList", geometries)
    context.setContextProperty("pointGeometryList", point_geometries)
    cam = document["camera"] if document else None
    context.setContextProperty("packetCameraPosition", QVector3D(*cam["position"]) if cam else QVector3D())
    context.setContextProperty("packetCameraRotation", QQuaternion(cam["quaternion"][3], *cam["quaternion"][:3]) if cam else QQuaternion())
    # Protocol matrices are column-major; QMatrix4x4's constructor uses rows.
    context.setContextProperty("packetProjection", QMatrix4x4(*(cam["projection"][c * 4 + r] for r in range(4) for c in range(4))) if cam else QMatrix4x4())
    view.setResizeMode(QQuickView.ResizeMode.SizeRootObjectToView)
    view.resize(int(document["camera"]["viewport"][0]), int(document["camera"]["viewport"][1])) if document else view.resize(1280, 720)
    view.setSource(QUrl.fromLocalFile(str(Path(__file__).with_name("Main.qml").resolve())))
    if view.status() == QQuickView.Status.Error:
        raise RuntimeError("QML failed: " + "; ".join(e.toString() for e in view.errors()))
    intervals, threads = [], set()
    stamp = [None]
    lock = threading.Lock()
    begun = time.perf_counter()

    def on_frame():
        now = time.perf_counter()
        with lock:
            threads.add(threading.get_ident())
            if stamp[0] is not None and now - begun > 2:
                intervals.append((now - stamp[0]) * 1000)
            stamp[0] = now

    view.frameSwapped.connect(on_frame, Qt.ConnectionType.DirectConnection)
    view.show()

    def finish():
        output = Path(args.output).resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        image = view.grabWindow()
        screenshot = output.with_suffix(".png")
        screenshot_saved = not image.isNull() and image.save(str(screenshot))
        with lock:
            frames = sorted(intervals)
            frame_threads = list(threads)
        interface = view.rendererInterface()
        graphics_api = str(interface.graphicsApi())
        probe = image.pixelColor(image.width() // 2, int(image.height() * .8)) if not image.isNull() else None
        terrain_rendered = screenshot_saved and "Software" not in graphics_api and probe is not None and probe.green() > probe.red() and probe.green() > probe.blue()
        result = {
            "prototype": "Qt Quick 3D render contract consumer" if document else "Qt Quick 3D native HGT mesh; not complete GCS",
            "mode": "offscreen QML check" if args.offscreen else ("transparent GPU functional smoke" if args.smoke else "visible feasibility experiment"),
            "qt": qVersion(), "pyside": qt_version,
            "graphicsApi": graphics_api,
            "requestedRenderLoop": os.environ["QSG_RENDER_LOOP"],
            "guiThread": threading.get_ident(), "frameSwappedThreads": frame_threads,
            "hgt": Path(args.hgt).name if args.hgt else None, "gridSize": size, "step": args.step if args.hgt else None,
            "renderPacket": Path(args.packet).name if args.packet else None,
            "packetTerrainChunks": len(geometries), "packetPointClouds": len(point_geometries),
            "triangles": triangles, "prepareMsExcluded": round(prepare_ms, 2),
            "vertexBytes": len(vertex_data) + sum(len(m[0]) for m in meshes), "indexBytes": len(index_data) + sum(len(m[1]) for m in meshes),
            "viewport": [view.width(), view.height()], "dpr": view.devicePixelRatio(),
            "screenshotSaved": screenshot_saved, "measuredIntervals": len(frames),
            "terrainRendered": terrain_rendered,
            "frameIntervalP95Ms": None if args.smoke or args.offscreen or not frames else frames[math.ceil(len(frames) * .95) - 1],
            "comparisonWithElectronValid": False,
            "limitations": ["Static GCS snapshot, no ongoing streaming" if document else "Single decimated HGT mesh; no GCS chunk LOD/streaming",
                            "Simplified materials; satellite imagery, outlines, video and telemetry UI pending",
                            "Point buffers consumed; point style and fading are simplified" if document else "No LiDAR",
                            "Python geometry preprocessing excluded; production conversion needs a native worker"]
        }
        output.write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(json.dumps(result, indent=2))
        app.exit(0 if screenshot_saved and (args.offscreen or terrain_rendered) else 1)

    QTimer.singleShot(int(args.seconds * 1000), finish)
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
