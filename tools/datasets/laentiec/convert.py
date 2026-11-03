#!/usr/bin/env python3
"""Convert the RGBDT LAENTIEC thermal CSVs into a thermal-only sequence.

    python3 -I convert.py --data /data/extracted/RAW-RGBDT-Dataset/thermal \
        --out /tmp/laentiec_work

Reads (never writes) every <timestamp_ms>.csv in --data: Optris PI 640i
temperature matrices, degrees Celsius per pixel, 640x480. Before writing
anything it checks that every value is finite and representable as a 16-bit
centi-kelvin value, round((T + 273.15) * 100) in [0, 65535], and reports the
minimum and maximum temperatures.

Writes, below --out (keep it outside the repository; it holds data derived
from the dataset):

    sequence/sequence.yaml           manifest (LiDAR + thermal camera)
    sequence/thermal.yaml            thermal config, encoding: linear,
                                     scale 0.01, offset -273.15
    sequence/thermal.csv             timestamp_ns,path
    sequence/thermal/NNNNNN.png      16-bit greyscale PNG, centi-kelvin
    sequence/lidar.csv               one empty scan per thermal frame
    sequence/lidar/NNNNNN.bin        empty (0 points)
    frames.csv                       frame,timestamp_ns,csv,png
    range.json                       min/max temperature, grid and range checks

The sequence format requires a LiDAR, so each thermal frame gets an empty
scan with the same timestamp. The thermal intrinsics and extrinsics are
placeholders: there is no calibration between the Optris and anything else
in this dataset, and this check does not exercise projection.
"""

import argparse
import json
import os
import struct
import sys
import zlib

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from laentiec_common import (HEIGHT, KELVIN, OFFSET, SCALE, WIDTH,  # noqa: E402
                             list_thermal_csvs, read_temperature_csv)


def write_png16(path, raw):
    """16-bit greyscale PNG (colour type 0, bit depth 16), no filtering."""
    h, w = raw.shape
    be = raw.astype(">u2")
    rows = np.zeros((h, 1 + 2 * w), dtype=np.uint8)
    rows[:, 1:] = be.view(np.uint8).reshape(h, 2 * w)

    def chunk(tag, data):
        c = struct.pack(">I", len(data)) + tag + data
        return c + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 16, 0, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(rows.tobytes(), 6))
    png += chunk(b"IEND", b"")
    with open(path, "wb") as f:
        f.write(png)


THERMAL_YAML = """\
# Optris PI 640i of the RGBDT dataset (LAENTIEC), converted by
# tools/datasets/laentiec/convert.py. Pixels hold temperature in centi-kelvin.
# Intrinsics and extrinsics are placeholders: the dataset has no calibration
# for this camera, and the thermal check does not use them.
name: thermal
resolution: [{w}, {h}]
intrinsics:
  K: [[500.0, 0.0, {cx}], [0.0, 500.0, {cy}], [0.0, 0.0, 1.0]]
distortion: [0.0, 0.0, 0.0, 0.0, 0.0]
extrinsics:
  source: lidar
  rotation: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
  translation: [0.0, 0.0, 0.0]
time_offset_ns: 0
thermal:
  encoding: linear
  # T [degrees C] = raw * scale + offset
  linear: {{scale: {scale!r}, offset: {offset!r}}}
"""

MANIFEST = """\
format_version: 1
lidar:
  name: lidar
  list: lidar.csv
cameras:
  thermal:
    config: thermal.yaml
    list: thermal.csv
    tau_ns: 0
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--data", required=True, help="directory with the thermal *.csv files")
    ap.add_argument("--out", required=True, help="work directory (outside the repository)")
    args = ap.parse_args()

    files = list_thermal_csvs(args.data)
    if not files:
        sys.exit(f"no <timestamp>.csv files in {args.data}")
    stamps = [t for t, _ in files]
    if len(set(stamps)) != len(stamps):
        sys.exit("duplicate timestamps in the file names")

    # Pass 1: read and check everything before writing.
    raws = []
    per_file = []
    tmin, tmax = np.inf, -np.inf
    max_grid_dev = 0.0
    max_ck_dev = 0.0
    for t_ms, path in files:
        t = read_temperature_csv(path)
        if not np.all(np.isfinite(t)):
            sys.exit(f"{path}: non-finite values")
        ck = (t + KELVIN) * 100.0
        r = np.round(ck)
        if r.min() < 0 or r.max() > 65535:
            sys.exit(f"{path}: temperatures {t.min()}..{t.max()} C are outside the 16-bit "
                     f"centi-kelvin range -273.15..382.2 C")
        max_ck_dev = max(max_ck_dev, float(np.abs(ck - r).max()))
        max_grid_dev = max(max_grid_dev, float(np.abs(t * 10.0 - np.round(t * 10.0)).max()))
        tmin = min(tmin, float(t.min()))
        tmax = max(tmax, float(t.max()))
        raws.append(r.astype(np.uint16))
        per_file.append({"timestamp_ms": t_ms, "csv": path,
                         "min_c": float(t.min()), "max_c": float(t.max())})
        print(f"\rread {len(raws)}/{len(files)}", end="", file=sys.stderr)
    print(file=sys.stderr)

    seq = os.path.join(args.out, "sequence")
    os.makedirs(os.path.join(seq, "thermal"), exist_ok=True)
    os.makedirs(os.path.join(seq, "lidar"), exist_ok=True)
    thermal_rows, lidar_rows, frame_rows = [], [], []
    for j, ((t_ms, path), raw) in enumerate(zip(files, raws)):
        t_ns = t_ms * 1_000_000
        png = f"thermal/{j:06d}.png"
        bin_ = f"lidar/{j:06d}.bin"
        write_png16(os.path.join(seq, png), raw)
        open(os.path.join(seq, bin_), "wb").close()
        thermal_rows.append(f"{t_ns},{png}")
        lidar_rows.append(f"{t_ns},{bin_}")
        frame_rows.append(f"{j},{t_ns},{path},{os.path.join(seq, png)}")
    for name, rows in (("thermal.csv", thermal_rows), ("lidar.csv", lidar_rows)):
        with open(os.path.join(seq, name), "w") as f:
            f.write("timestamp_ns,path\n" + "\n".join(rows) + "\n")
    with open(os.path.join(seq, "thermal.yaml"), "w") as f:
        f.write(THERMAL_YAML.format(w=WIDTH, h=HEIGHT, cx=(WIDTH - 1) / 2, cy=(HEIGHT - 1) / 2,
                                    scale=SCALE, offset=OFFSET))
    with open(os.path.join(seq, "sequence.yaml"), "w") as f:
        f.write(MANIFEST)
    with open(os.path.join(args.out, "frames.csv"), "w") as f:
        f.write("frame,timestamp_ns,csv,png\n" + "\n".join(frame_rows) + "\n")

    all_raw = np.stack(raws)
    summary = {
        "frames": len(files),
        "pixels": int(all_raw.size),
        "min_c": tmin,
        "max_c": tmax,
        "min_raw_ck": int(all_raw.min()),
        "max_raw_ck": int(all_raw.max()),
        "representable_range_c": [0 * SCALE + OFFSET, 65535 * SCALE + OFFSET],
        "max_deviation_from_0.1C_grid_c": max_grid_dev / 10.0,
        "max_centikelvin_rounding": max_ck_dev,
        "per_file": per_file,
    }
    with open(os.path.join(args.out, "range.json"), "w") as f:
        json.dump(summary, f, indent=1)
    print(f"{len(files)} frames; temperatures {tmin:.2f} .. {tmax:.2f} C; "
          f"centi-kelvin {summary['min_raw_ck']} .. {summary['max_raw_ck']} "
          f"(all within 0..65535); max distance of (T + 273.15) * 100 from an integer: "
          f"{max_ck_dev:.3g}")
    print(f"sequence: {os.path.join(seq, 'sequence.yaml')}")


if __name__ == "__main__":
    main()
