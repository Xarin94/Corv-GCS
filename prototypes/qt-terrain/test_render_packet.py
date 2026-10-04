"""Protocol checks use Python + Node, without PySide6 or archived audit files."""
import array
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from render_packet import read_packet, terrain_mesh, point_positions


class RenderPacketTests(unittest.TestCase):
    def test_javascript_snapshot(self):
        root = Path(__file__).resolve().parents[2]
        with tempfile.TemporaryDirectory(prefix="corv-render-contract-") as directory:
            filename = Path(directory) / "fixture.crvg"
            subprocess.run(["node", str(root / "scripts/test-render-architecture.js"),
                            "--packet-out", str(filename)], check=True, capture_output=True)
            world = read_packet(filename)
        self.assertGreater(len(world["terrain"]), 1)
        self.assertEqual(len(world["camera"]["projection"]), 16)
        self.assertGreater(world["camera"]["viewport"][2], 0)
        for chunk in world["terrain"]:
            self.assertEqual(chunk["heights"].typecode, "h")
            self.assertEqual(len(chunk["heights"]), chunk["width"] ** 2)
        self.assertTrue(world["pointClouds"])
        for cloud in world["pointClouds"]:
            points = point_positions(cloud, world["timeSeconds"])
            self.assertEqual(len(points) % 3, 0)

    def test_heightfield_normals_edges_and_winding(self):
        # A sloping plane with unequal X/Z spacing exercises one-sided edges.
        heights = array.array("h", [-20, -16, -12, -10, -6, -2, 0, 4, 8])
        vertices, indices, bounds, triangles = terrain_mesh({
            "width": 3, "heights": heights, "grid": [100, 200, 2, 5], "minH": -20, "maxH": 8
        })
        v, ix = array.array("f"), array.array("I")
        v.frombytes(vertices); ix.frombytes(indices)
        self.assertEqual(triangles, 8)
        self.assertEqual(bounds, (100, -20, 200, 104, 8, 210))
        expected = (-2 / 3, 1 / 3, -2 / 3)
        for i in range(9):
            for actual, wanted in zip(v[i * 6 + 3:i * 6 + 6], expected):
                self.assertAlmostEqual(actual, wanted, places=6)
        a, b, c = (v[j * 6:j * 6 + 3] for j in ix[:3])
        # Cross product Y is positive: triangle faces agree with the normal.
        self.assertGreater((b[2] - a[2]) * (c[0] - a[0]) - (b[0] - a[0]) * (c[2] - a[2]), 0)
        with self.assertRaises(ValueError):
            terrain_mesh({"width": 3, "heights": array.array("h", [1]), "grid": [0, 0, 1, 1]})

    def test_point_transforms_and_ttl(self):
        cloud = {
            "visible": True,
            "mapTransform": {"position": [10, 20, 30], "scale": [2, 3, 4]},
            "chunks": [{"count": 1, "positions": array.array("f", [1, 2, 3])}],
            "liveTransform": {"position": [100, 200, 300], "quaternion": [0, math.sqrt(.5), 0, math.sqrt(.5)]},
            "live": {"count": 2, "positions": array.array("f", [1, 0, 0, 0, 1, 0]), "birth": [9, 1], "ttl": 3}
        }
        points = point_positions(cloud, 10)
        self.assertEqual(list(points[:3]), [12, 26, 42])
        self.assertEqual(list(points[3:]), [100, 200, 299])
        cloud["visible"] = False
        self.assertEqual(len(point_positions(cloud, 10)), 0)

    def test_invalid_packets(self):
        manifest = {"data": {"schemaVersion": 1, "axes": "east-up-south", "units": "metres"}, "attachments": []}

        def packet(value, version=1):
            encoded = json.dumps(value).encode("utf-8")
            padding = (-16 - len(encoded)) % 8
            return struct.pack("<4sIII", b"CRVG", version, len(encoded), 0) + encoded + bytes(padding)

        cases = [packet(manifest)[:-1], packet(manifest, 2)]
        manifest["data"]["badReference"] = {"$buffer": 0}
        cases.append(packet(manifest))
        manifest["data"].pop("badReference")
        manifest["attachments"] = [{"type": "i16", "offset": 1, "byteLength": 2}]
        cases.append(packet(manifest))
        with tempfile.TemporaryDirectory(prefix="corv-render-contract-") as directory:
            filename = Path(directory) / "invalid.crvg"
            for raw in cases:
                with self.subTest(length=len(raw)):
                    filename.write_bytes(raw)
                    with self.assertRaises(ValueError):
                        read_packet(filename)


if __name__ == "__main__":
    unittest.main()
