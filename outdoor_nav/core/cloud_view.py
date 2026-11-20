"""Downsampled lidar cloud snapshots for the 3D page view; read-only.

Duck-typed over a PointCloud2-shaped message so the module stays
importable (and testable) without ROS.
"""

import numpy as np

# sensor_msgs.PointField.FLOAT32
_FLOAT32 = 7


def parse_xyz(msg):
    """Return the x/y/z FLOAT32 columns of a PointCloud2 as an (N, 3) array."""
    offsets, datatypes = {}, {}
    for field in msg.fields:
        if field.name in ("x", "y", "z"):
            offsets[field.name], datatypes[field.name] = field.offset, field.datatype
    if len(offsets) != 3 or msg.is_bigendian:
        raise ValueError("cloud needs little-endian x/y/z fields")
    if any(kind != _FLOAT32 for kind in datatypes.values()):
        raise ValueError("cloud x/y/z must be FLOAT32")
    count = msg.width * msg.height
    if count == 0 or msg.point_step <= 0 or len(msg.data) < count * msg.point_step:
        raise ValueError("empty cloud")
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(count, msg.point_step)
    return np.stack([
        raw[:, offsets[name]:offsets[name] + 4].copy().view(np.float32).reshape(-1)
        for name in ("x", "y", "z")
    ], axis=1)


def voxel_downsample(xyz, size, cap):
    """Keep one point per `size` voxel, then cap with a random draw."""
    if len(xyz) == 0:
        return xyz
    keys = np.floor(xyz / size).astype(np.int64)
    _, first = np.unique(keys, axis=0, return_index=True)
    points = xyz[np.sort(first)]
    if len(points) > cap:
        points = points[np.random.default_rng().choice(len(points), cap, replace=False)]
    return points
