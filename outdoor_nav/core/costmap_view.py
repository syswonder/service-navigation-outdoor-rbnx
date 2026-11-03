"""Read-only OccupancyGrid snapshots for the web, compressed as palette PNG."""

import base64
from io import BytesIO
import math

import numpy as np
from PIL import Image


def grid_snapshot(message, age_s, robot=None, path=(), theme=None):
    info = message.info
    width, height = int(info.width), int(info.height)
    if not (
        0 < width <= 10000
        and 0 < height <= 10000
        and width * height <= 16000000
        and math.isfinite(info.resolution)
        and info.resolution > 0
        and len(message.data) == width * height
    ):
        raise ValueError("Invalid costmap dimensions or resolution")
    # ROS OccupancyGrid values, NOT the raw 0..255 costmap values: unknown=-1,
    # free=0, inflated=1..98, inscribed=99, lethal=100.
    values = np.asarray(message.data, dtype=np.int16)
    if np.any((values < -1) | (values > 100)):
        raise ValueError("Invalid OccupancyGrid cell value")
    pixels = np.flipud((values % 256).astype(np.uint8).reshape(height, width))
    image = Image.frombytes("P", (width, height), pixels.tobytes())
    palette = [0] * 768
    palette[:3] = [233, 239, 233]
    for i in range(1, 99):
        palette[3 * i : 3 * i + 3] = [255, round(225 - 140 * i / 98), 100]
    palette[297:303] = [239, 55, 69, 126, 12, 30]
    palette[765:768] = [78, 88, 98]
    if theme == "global":
        for i in range(1, 101):
            palette[3 * i : 3 * i + 3] = [110 + round(i * 0.9), 65, 200]
    elif theme == "local":
        for i in range(1, 101):
            palette[3 * i : 3 * i + 3] = [20, 220 - round(i * 0.7), 255]
        transparency = bytearray([210] * 256)
        transparency[0] = transparency[255] = 0
        image.info["transparency"] = bytes(transparency)
    image.putpalette(palette)
    output = BytesIO()
    image.save(output, format="PNG", optimize=True)
    q = info.origin.orientation
    origin_yaw = math.atan2(
        2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
    )
    return {
        "schema_version": 1,
        "frame_id": message.header.frame_id,
        "width": width,
        "height": height,
        "resolution": float(info.resolution),
        "origin": {
            "x": info.origin.position.x,
            "y": info.origin.position.y,
            "yaw": origin_yaw,
        },
        "stamp_s": message.header.stamp.sec + message.header.stamp.nanosec / 1e9,
        "age_s": max(0.0, age_s),
        "stale": age_s >= 3.0,
        "image": "data:image/png;base64,"
        + base64.b64encode(output.getvalue()).decode(),
        "robot": robot,
        "path": list(path),
    }
