"""Shared helpers for the LAENTIEC (RGBDT dataset) thermal check."""

import os
import re

import numpy as np

WIDTH = 640
HEIGHT = 480

# Centi-kelvin: raw = round((T + 273.15) * 100), T = raw * 0.01 - 273.15.
SCALE = 0.01
OFFSET = -273.15
KELVIN = 273.15

STEM = re.compile(r"^(\d+)\.csv$")


def list_thermal_csvs(thermal_dir):
    """Sorted (timestamp_ms, path) for every <digits>.csv in thermal_dir.

    The file name is the capture time in milliseconds. Other files (the
    dataset's PNG previews) are ignored."""
    out = []
    for name in os.listdir(thermal_dir):
        m = STEM.match(name)
        if m:
            out.append((int(m.group(1)), os.path.join(thermal_dir, name)))
    out.sort()
    return out


def read_temperature_csv(path):
    """Optris temperature matrix in degrees Celsius, float64, HEIGHT x WIDTH."""
    t = np.loadtxt(path, delimiter=",", dtype=np.float64, ndmin=2)
    if t.shape != (HEIGHT, WIDTH):
        raise ValueError(f"{path}: expected {HEIGHT}x{WIDTH} values, got "
                         f"{t.shape[0]}x{t.shape[1]}")
    return t
