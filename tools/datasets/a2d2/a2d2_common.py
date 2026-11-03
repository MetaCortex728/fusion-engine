"""Shared helpers for the A2D2 validation tools.

A2D2 describes every sensor by a "view": an origin and the x- and y-axes of
the sensor frame, all in vehicle coordinates. With z = x cross y, the matrix
R = [x y z] (axes as columns) maps sensor-frame directions into the vehicle
frame, so

    p_vehicle = R @ p_view + origin        i.e. T_vehicle_view = (R, origin)

convert.py confirms this numerically on the data (see convention_check()).

Transforms are named T_target_source and stored as 4x4 numpy arrays.
"""

import json
import struct

import numpy as np

LIDAR_NAMES = ["front_center", "front_left", "front_right", "rear_left", "rear_right"]

# The camera "view" frame is x forward, y left, z up. The engine's camera
# frame is the optical frame: x right, y down, z forward, i.e.
# Q = (-y, -z, x). T_optical_view:
R_OPTICAL_VIEW = np.array([[0.0, -1.0, 0.0],
                           [0.0, 0.0, -1.0],
                           [1.0, 0.0, 0.0]])


def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def rotation_from_view(view):
    x = np.asarray(view["x-axis"], dtype=float)
    y = np.asarray(view["y-axis"], dtype=float)
    x = x / np.linalg.norm(x)
    y = y / np.linalg.norm(y)
    z = np.cross(x, y)
    return np.column_stack([x, y, z])


def T_vehicle_view(view, transpose=False):
    """4x4 T_vehicle_<sensor>. `transpose` builds the rejected alternative
    (axes as rows), used only by the convention check."""
    R = rotation_from_view(view)
    T = np.eye(4)
    T[:3, :3] = R.T if transpose else R
    T[:3, 3] = np.asarray(view["origin"], dtype=float)
    return T


def inverse(T):
    Ti = np.eye(4)
    R = T[:3, :3]
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ T[:3, 3]
    return Ti


def apply(T, points):
    return points @ T[:3, :3].T + T[:3, 3]


def T_optical_view():
    T = np.eye(4)
    T[:3, :3] = R_OPTICAL_VIEW
    return T


def png_size(path):
    """(width, height) from a PNG's IHDR chunk."""
    with open(path, "rb") as f:
        head = f.read(24)
    if head[:8] != b"\x89PNG\r\n\x1a\n" or head[12:16] != b"IHDR":
        raise ValueError(f"{path}: not a PNG file")
    return struct.unpack(">II", head[16:24])
