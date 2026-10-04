"""CRVG v1 reader and mesh conversion, also testable without Qt installed."""
import array
import json
import math
from pathlib import Path
import struct
import sys

TYPES = {"i16": "h", "u8": "B", "u16": "H", "u32": "I", "f32": "f", "f64": "d"}


def read_packet(filename):
    raw = Path(filename).read_bytes()
    if len(raw) < 16 or len(raw) > 512 * 1024 * 1024:
        raise ValueError("Invalid render packet length")
    magic, version, manifest_length, payload_length = struct.unpack_from("<4sIII", raw)
    if magic != b"CRVG" or version != 1:
        raise ValueError("Unsupported render packet")
    payload_offset = (16 + manifest_length + 7) // 8 * 8
    if manifest_length > 16 * 1024 * 1024 or payload_offset + payload_length != len(raw):
        raise ValueError("Truncated render packet")
    manifest = json.loads(raw[16:16 + manifest_length])
    buffers = []
    for attachment in manifest["attachments"]:
        kind = TYPES.get(attachment["type"])
        offset, length = attachment["offset"], attachment["byteLength"]
        if kind is None or not isinstance(offset, int) or not isinstance(length, int) or offset < 0 or length < 0 or offset % 8 or offset + length > payload_length:
            raise ValueError("Invalid render attachment")
        values = array.array(kind)
        if length % values.itemsize:
            raise ValueError("Invalid render attachment alignment")
        values.frombytes(raw[payload_offset + offset:payload_offset + offset + length])
        if sys.byteorder != "little" and values.itemsize > 1:
            values.byteswap()
        buffers.append(values)

    def unpack(value):
        if isinstance(value, dict) and "$buffer" in value:
            index = value["$buffer"]
            if not isinstance(index, int) or not 0 <= index < len(buffers):
                raise ValueError("Missing render buffer")
            return buffers[index]
        if isinstance(value, list):
            return [unpack(v) for v in value]
        if isinstance(value, dict):
            return {k: unpack(v) for k, v in value.items()}
        return value

    result = unpack(manifest["data"])
    if result["schemaVersion"] != 1 or result["axes"] != "east-up-south" or result["units"] != "metres":
        raise ValueError("Unsupported render coordinates")
    return result


def terrain_mesh(record):
    width, heights = record["width"], record["heights"]
    if not isinstance(width, int) or width < 2 or len(heights) != width * width:
        raise ValueError("Invalid heightfield dimensions")
    x, z, dx, dz = record["grid"]
    vertices, indices = array.array("f"), array.array("I")
    for row in range(width):
        for col in range(width):
            east, west = min(col + 1, width - 1), max(col - 1, 0)
            south, north = min(row + 1, width - 1), max(row - 1, 0)
            ex, sz = (east - west) * dx, (south - north) * dz
            ey = heights[row * width + east] - heights[row * width + west]
            sy = heights[south * width + col] - heights[north * width + col]
            nx, ny, nz = -sz * ey, sz * ex, -sy * ex
            length = math.sqrt(nx * nx + ny * ny + nz * nz)
            vertices.extend((x + col * dx, heights[row * width + col], z + row * dz,
                             nx / length, ny / length, nz / length))
    for row in range(width - 1):
        for col in range(width - 1):
            a, b = row * width + col, (row + 1) * width + col
            indices.extend((a, b, a + 1, b, b + 1, a + 1))
    bounds = (x, record["minH"], z, x + (width - 1) * dx, record["maxH"], z + (width - 1) * dz)
    return vertices.tobytes(), indices.tobytes(), bounds, len(indices) // 3


def point_positions(cloud, snapshot_time):
    result = array.array("f")
    if not cloud["visible"]:
        return result
    transform = cloud["mapTransform"]
    for block in cloud["chunks"]:
        positions = block["positions"]
        if len(positions) != block["count"] * 3:
            raise ValueError("Invalid point block dimensions")
        for i in range(0, len(positions), 3):
            result.extend(positions[i + k] * transform["scale"][k] + transform["position"][k] for k in range(3))
    live, transform = cloud["live"], cloud["liveTransform"]
    qx, qy, qz, qw = transform["quaternion"]
    if len(live["positions"]) != live["count"] * 3 or len(live["birth"]) != live["count"]:
        raise ValueError("Invalid live point dimensions")
    for i in range(live["count"]):
        if snapshot_time - live["birth"][i] > live["ttl"]:
            continue
        x, y, z = live["positions"][i * 3:i * 3 + 3]
        tx, ty, tz = 2 * (qy * z - qz * y), 2 * (qz * x - qx * z), 2 * (qx * y - qy * x)
        result.extend((x + qw * tx + qy * tz - qz * ty + transform["position"][0],
                       y + qw * ty + qz * tx - qx * tz + transform["position"][1],
                       z + qw * tz + qx * ty - qy * tx + transform["position"][2]))
    return result
