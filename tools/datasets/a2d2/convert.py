#!/usr/bin/env python3
"""Convert an A2D2 front-centre camera slice into fusion-engine sequences.

    python3 convert.py --data /data --drive 20180810_150607 --out /tmp/a2d2_work \
        [--config cams_lidars.json] [--first 60 --last 109]

Reads (never writes) the A2D2 files:
    <data>/cams_lidars.json
    <data>/<drive>/camera/cam_front_center/*.json, *.png
    <data>/<drive>/lidar/cam_front_center/*.npz

and writes, below --out (keep it outside the repository; it holds data
derived from A2D2):

    frames.csv                       frame,timestamp_ns,image,npz
    transforms.json                  every extrinsic used, 4x4, T_target_source
    convention_check.json            numerical check of the view convention
    direct/                          one sequence: camera-frame points,
                                     identity extrinsics
    chain/<lidar>/                   one sequence per LiDAR: points in that
                                     LiDAR's own frame, extrinsics
                                     T_camera_lidar = T_camera_vehicle @ T_vehicle_lidar
    reference/NNNNNN.csv             per point: A2D2 row, col, depth, lidar_id,
                                     and where the point went in the chain scans

The RGB list files point at the original PNGs by absolute path; images are
not copied. The engine's sequence format has one LiDAR per sequence, hence
one chain sequence per LiDAR.
"""

import argparse
import glob
import json
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from a2d2_common import (LIDAR_NAMES, T_optical_view, T_vehicle_view, apply,  # noqa: E402
                         inverse, load_json, png_size)

CAMERA = "front_center"
CAMERA_DIR = "cam_front_center"


def fmt(v):
    return repr(float(v))


def yaml_rigid(T, indent):
    pad = " " * indent
    R = T[:3, :3]
    rows = ", ".join("[" + ", ".join(fmt(x) for x in r) + "]" for r in R)
    return (f"{pad}rotation: [{rows}]\n"
            f"{pad}translation: [{', '.join(fmt(x) for x in T[:3, 3])}]\n")


def write_camera_config(path, K, width, height, source, T_camera_source):
    with open(path, "w", encoding="utf-8") as f:
        f.write("# A2D2 cam_front_center, undistorted images: CamMatrix, no distortion.\n")
        f.write("name: rgb\n")
        f.write(f"resolution: [{width}, {height}]\n")
        f.write("intrinsics:\n")
        f.write("  K: [" + ", ".join("[" + ", ".join(fmt(x) for x in r) + "]" for r in K) + "]\n")
        f.write("distortion: [0.0, 0.0, 0.0, 0.0, 0.0]\n")
        f.write("extrinsics:\n")
        f.write(f"  source: {source}\n")
        f.write(yaml_rigid(T_camera_source, 2))
        f.write("time_offset_ns: 0\n")


def write_manifest(path, lidar_name, T_body_lidar=None):
    with open(path, "w", encoding="utf-8") as f:
        f.write("format_version: 1\n")
        f.write("lidar:\n")
        f.write(f"  name: {lidar_name}\n")
        f.write("  list: lidar.csv\n")
        if T_body_lidar is not None:
            # Body = A2D2 vehicle frame. Informational: there is no trajectory,
            # so the engine does not use it for compensation.
            f.write("  T_body_lidar:\n")
            f.write(yaml_rigid(T_body_lidar, 4))
        f.write("cameras:\n")
        f.write("  rgb:\n")
        f.write("    config: rgb.yaml\n")
        f.write("    list: rgb.csv\n")
        # LiDAR scans carry the camera's own timestamp, so the association
        # is exact; a small threshold makes any mismatch visible.
        f.write("    tau_ns: 1000000\n")


def write_list(path, entries):
    with open(path, "w", encoding="utf-8") as f:
        f.write("timestamp_ns,path\n")
        for t, p in entries:
            f.write(f"{t},{p}\n")


def write_bin(path, points, intensity):
    rec = np.empty((len(points), 4), dtype="<f4")
    rec[:, :3] = points
    rec[:, 3] = intensity
    if not np.all(np.isfinite(rec)):
        raise ValueError(f"{path}: non-finite value")
    rec.tofile(path)


def wrap(a):
    return np.angle(np.exp(1j * a))


def convention_check(per_lidar_points, per_lidar_azimuth, T_vehicle_cam_by_conv,
                     T_vehicle_lidar_by_conv):
    """For each LiDAR and both candidate conventions, move the camera-frame
    points into the LiDAR frame and compare the LiDAR azimuth with A2D2's
    `azimuth` attribute. A Velodyne reports azimuth clockwise from the
    sensor's y-axis: azimuth = pi/2 - atan2(y, x). A wrong rotation cannot
    reproduce this for every LiDAR at once. The number of distinct
    elevation angles (0.1 deg bins) shows the laser rings: a 16-beam
    sensor gives a few dozen bins in its true frame and hundreds otherwise."""
    out = {}
    for conv in ("axes_as_columns", "axes_as_rows"):
        out[conv] = {}
        T_vc = T_vehicle_cam_by_conv[conv]
        for name in LIDAR_NAMES:
            pts = np.concatenate(per_lidar_points[name]) if per_lidar_points[name] else np.zeros((0, 3))
            az = np.concatenate(per_lidar_azimuth[name]) if per_lidar_azimuth[name] else np.zeros(0)
            if len(pts) == 0:
                out[conv][name] = {"points": 0}
                continue
            T_lc = inverse(T_vehicle_lidar_by_conv[conv][name]) @ T_vc
            pl = apply(T_lc, pts)
            resid = np.abs(wrap(az - (np.pi / 2 - np.arctan2(pl[:, 1], pl[:, 0]))))
            el = np.degrees(np.arctan2(pl[:, 2], np.hypot(pl[:, 0], pl[:, 1])))
            out[conv][name] = {
                "points": int(len(pts)),
                "azimuth_residual_median_deg": float(np.degrees(np.median(resid))),
                "azimuth_residual_p99_deg": float(np.degrees(np.percentile(resid, 99))),
                "elevation_bins_0p1deg": int(len(np.unique(np.round(el, 1)))),
            }
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--data", default="/data", help="A2D2 root with cams_lidars.json")
    ap.add_argument("--config", help="cams_lidars.json (default <data>/cams_lidars.json)")
    ap.add_argument("--drive", default="20180810_150607")
    ap.add_argument("--first", type=int, help="first frame number (file name suffix) to use")
    ap.add_argument("--last", type=int, help="last frame number (inclusive) to use")
    ap.add_argument("--out", required=True, help="work directory (outside the repository)")
    args = ap.parse_args()

    cfg = load_json(args.config or os.path.join(args.data, "cams_lidars.json"))
    cam = cfg["cameras"][CAMERA]
    K = np.asarray(cam["CamMatrix"], dtype=float)

    cam_dir = os.path.join(args.data, args.drive, "camera", CAMERA_DIR)
    lidar_dir = os.path.join(args.data, args.drive, "lidar", CAMERA_DIR)
    frames = []
    for path in sorted(glob.glob(os.path.join(cam_dir, "*.json"))):
        m = re.search(r"_(\d+)\.json$", path)
        number = int(m.group(1)) if m else None
        if args.first is not None and (number is None or number < args.first):
            continue
        if args.last is not None and (number is None or number > args.last):
            continue
        frames.append(path)
    if not frames:
        sys.exit(f"convert.py: no camera JSON files in {cam_dir}")

    # Extrinsics. "view" = A2D2 sensor frame; "camera" = optical frame.
    T_v_view = T_vehicle_view(cam["view"])
    T_camera_vehicle = T_optical_view() @ inverse(T_v_view)
    T_vehicle_lidar = {n: T_vehicle_view(cfg["lidars"][n]["view"]) for n in LIDAR_NAMES}
    T_camera_lidar = {n: T_camera_vehicle @ T_vehicle_lidar[n] for n in LIDAR_NAMES}
    # T_lidar_view: camera view frame (where A2D2 stores the points) to LiDAR.
    T_lidar_view = {n: inverse(T_vehicle_lidar[n]) @ T_v_view for n in LIDAR_NAMES}

    out = os.path.abspath(args.out)
    for d in ["direct/lidar", "reference"] + [f"chain/{n}/lidar" for n in LIDAR_NAMES]:
        os.makedirs(os.path.join(out, d), exist_ok=True)

    width = height = None
    rows_frames = []
    direct_lidar, rgb_list = [], []
    chain_lidar = {n: [] for n in LIDAR_NAMES}
    conv_pts = {n: [] for n in LIDAR_NAMES}
    conv_az = {n: [] for n in LIDAR_NAMES}
    reproj_err = []

    for i, jpath in enumerate(frames):
        meta = load_json(jpath)
        if meta.get("cam_name") != CAMERA:
            sys.exit(f"convert.py: {jpath}: cam_name {meta.get('cam_name')!r}, expected {CAMERA!r}")
        ids = {int(k): v for k, v in meta["lidar_ids"].items()}
        if any(ids.get(k) != n for k, n in enumerate(LIDAR_NAMES)):
            sys.exit(f"convert.py: {jpath}: unexpected lidar_ids {meta['lidar_ids']}")
        img = os.path.join(cam_dir, meta["image_png"])
        npz = os.path.join(lidar_dir, meta["pcld_npz"])
        w, h = png_size(img)
        if width is None:
            width, height = w, h
            if [w, h] != list(cam["Resolution"]):
                print(f"convert.py: note: image size {w}x{h} differs from Resolution {cam['Resolution']}")
        elif (w, h) != (width, height):
            sys.exit(f"convert.py: {img}: size {w}x{h} differs from the first image")
        t_ns = int(meta["cam_tstamp"]) * 1000  # microseconds -> nanoseconds

        z = np.load(npz)
        p_view = np.asarray(z["pcloud_points"], dtype=float)
        lid = np.asarray(z["pcloud_attr.lidar_id"]).astype(int)
        row = np.asarray(z["pcloud_attr.row"], dtype=float)
        col = np.asarray(z["pcloud_attr.col"], dtype=float)
        depth = np.asarray(z["pcloud_attr.depth"], dtype=float)
        refl = np.asarray(z["pcloud_attr.reflectance"], dtype=float)
        az = np.asarray(z["pcloud_attr.azimuth"], dtype=float)
        if not np.all(np.isin(lid, range(len(LIDAR_NAMES)))):
            sys.exit(f"convert.py: {npz}: unknown lidar_id")

        # Sanity check of the stored projection (established on one frame).
        q = p_view @ T_optical_view()[:3, :3].T
        uv = (q @ K.T)[:, :2] / q[:, 2:3]
        reproj_err.append(np.hypot(uv[:, 0] - col, uv[:, 1] - row))

        name = f"{i:06d}"
        # Direct: optical-frame points, identity extrinsics.
        write_bin(os.path.join(out, "direct/lidar", name + ".bin"), q, refl)
        direct_lidar.append((t_ns, f"lidar/{name}.bin"))
        rgb_list.append((t_ns, img))

        # Chain: each LiDAR's points in its own frame, original order kept.
        chain_index = np.full(len(p_view), -1, dtype=int)
        for k, n in enumerate(LIDAR_NAMES):
            sel = np.flatnonzero(lid == k)
            chain_index[sel] = np.arange(len(sel))
            write_bin(os.path.join(out, "chain", n, "lidar", name + ".bin"),
                      apply(T_lidar_view[n], p_view[sel]), refl[sel])
            chain_lidar[n].append((t_ns, f"lidar/{name}.bin"))
            conv_pts[n].append(p_view[sel])
            conv_az[n].append(az[sel])

        with open(os.path.join(out, "reference", name + ".csv"), "w", encoding="utf-8") as f:
            f.write("point,lidar_id,chain_index,row,col,depth\n")
            for j in range(len(p_view)):
                f.write(f"{j},{lid[j]},{chain_index[j]},{fmt(row[j])},{fmt(col[j])},{fmt(depth[j])}\n")
        rows_frames.append((i, t_ns, img, npz, len(p_view)))

    write_camera_config(os.path.join(out, "direct/rgb.yaml"), K, width, height,
                        "camera_points", np.eye(4))
    write_manifest(os.path.join(out, "direct/sequence.yaml"), "camera_points")
    write_list(os.path.join(out, "direct/lidar.csv"), direct_lidar)
    write_list(os.path.join(out, "direct/rgb.csv"), rgb_list)
    for n in LIDAR_NAMES:
        d = os.path.join(out, "chain", n)
        lname = f"lidar_{n}"
        write_camera_config(os.path.join(d, "rgb.yaml"), K, width, height, lname, T_camera_lidar[n])
        write_manifest(os.path.join(d, "sequence.yaml"), lname, T_vehicle_lidar[n])
        write_list(os.path.join(d, "lidar.csv"), chain_lidar[n])
        write_list(os.path.join(d, "rgb.csv"), rgb_list)

    with open(os.path.join(out, "frames.csv"), "w", encoding="utf-8") as f:
        f.write("frame,timestamp_ns,image,npz,points\n")
        for r in rows_frames:
            f.write(",".join(str(x) for x in r) + "\n")

    with open(os.path.join(out, "transforms.json"), "w", encoding="utf-8") as f:
        json.dump({
            "convention": "T_target_source, p_target = T[:3,:3] @ p_source + T[:3,3]",
            "T_vehicle_cameraview": T_v_view.tolist(),
            "T_camera_vehicle": T_camera_vehicle.tolist(),
            "T_vehicle_lidar": {n: T_vehicle_lidar[n].tolist() for n in LIDAR_NAMES},
            "T_camera_lidar": {n: T_camera_lidar[n].tolist() for n in LIDAR_NAMES},
        }, f, indent=1)

    convs = {
        "axes_as_columns": (T_v_view, {n: T_vehicle_lidar[n] for n in LIDAR_NAMES}),
        "axes_as_rows": (T_vehicle_view(cam["view"], transpose=True),
                         {n: T_vehicle_view(cfg["lidars"][n]["view"], transpose=True)
                          for n in LIDAR_NAMES}),
    }
    check = convention_check(conv_pts, conv_az, {c: v[0] for c, v in convs.items()},
                             {c: v[1] for c, v in convs.items()})
    e = np.concatenate(reproj_err)
    check["stored_projection_vs_CamMatrix_px"] = {"median": float(np.median(e)),
                                                  "max": float(e.max())}
    check["image_size"] = [width, height]
    check["Resolution_json"] = list(cam["Resolution"])
    with open(os.path.join(out, "convention_check.json"), "w", encoding="utf-8") as f:
        json.dump(check, f, indent=1)

    print(f"convert.py: {len(frames)} frames, {sum(r[4] for r in rows_frames)} points -> {out}")
    for n in LIDAR_NAMES:
        a, b = check["axes_as_columns"][n], check["axes_as_rows"][n]
        if a.get("points"):
            print(f"  {n:12s} azimuth residual median {a['azimuth_residual_median_deg']:.3f} deg "
                  f"(axes as rows: {b['azimuth_residual_median_deg']:.1f} deg), "
                  f"elevation bins {a['elevation_bins_0p1deg']} (rows: {b['elevation_bins_0p1deg']})")
    print(f"  stored col/row vs CamMatrix projection: median {np.median(e):.3g} px, max {e.max():.3g} px")


if __name__ == "__main__":
    main()
