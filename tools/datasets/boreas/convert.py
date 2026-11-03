#!/usr/bin/env python3
"""Convert a Boreas slice into a fusion-engine sequence.

    python3 convert.py --data /data/boreas-2021-03-02-13-38 --out /tmp/boreas_work

Reads (never writes) the Boreas files:
    <data>/lidar/<t_us>.bin           float32 x, y, z, intensity, laser_id, t
    <data>/camera/<t_us>.png          rectified images, mid-exposure time
    <data>/calib/P_camera.txt, T_camera_lidar.txt, T_applanix_lidar.txt
    <data>/applanix/gps_post_process.csv, lidar_poses.csv, camera_poses.csv

and writes, below --out (keep it outside the repository; it holds data
derived from Boreas):

    sequence/sequence.yaml            manifest: LiDAR (body = LiDAR frame,
                                      T_body_lidar = identity), rgb camera,
                                      trajectory, tau_ns = half the LiDAR period
    sequence/rgb.yaml                 K = P_camera[:, :3], no distortion,
                                      extrinsics T_camera_lidar
    sequence/lidar.csv, rgb.csv       list files (rgb.csv points at the
                                      original PNGs by absolute path)
    sequence/lidar/NNNNNN.bin         x, y, z, intensity (float32)
    sequence/trajectory.csv           T_world_lidar at 200 Hz in a local ENU
                                      frame: gps_post_process composed with
                                      T_applanix_lidar
    point_times/NNNNNN.f32            per-point time (s, relative to the scan
                                      time = file name), same order as the .bin
    laser_id/NNNNNN.u8                per-point laser (ring) number
    scans.csv                         index, timestamp_ns, points, t min/max
    enu_origin.json                   the ENU offset subtracted
    convention_check.json             numeric checks of the conventions
"""

import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from boreas_common import (C_enu_sensor, interp_poses, load_matrix, png_size,  # noqa: E402
                           read_boreas_lidar, read_pose_csv, rot_angle_deg, rot_to_quat,
                           stamp_files, stats)

MARGIN_NS = 1_000_000_000  # trajectory extends 1 s beyond the sensor data


def fmt(v):
    return repr(float(v))


def yaml_rigid(T, indent):
    pad = " " * indent
    rows = ", ".join("[" + ", ".join(fmt(x) for x in r) + "]" for r in T[:3, :3])
    return (f"{pad}rotation: [{rows}]\n"
            f"{pad}translation: [{', '.join(fmt(x) for x in T[:3, 3])}]\n")


def write_camera_config(path, K, width, height, T_camera_lidar, note):
    with open(path, "w", encoding="utf-8") as f:
        f.write(f"# {note}\n")
        f.write("name: rgb\n")
        f.write(f"resolution: [{width}, {height}]\n")
        f.write("intrinsics:\n")
        f.write("  K: [" + ", ".join("[" + ", ".join(fmt(x) for x in r) + "]" for r in K) + "]\n")
        f.write("distortion: [0.0, 0.0, 0.0, 0.0, 0.0]\n")
        f.write("extrinsics:\n")
        f.write("  source: lidar\n")
        f.write(yaml_rigid(T_camera_lidar, 2))
        f.write("time_offset_ns: 0\n")


def write_manifest(path, lidar_list, rgb_config, rgb_list, tau_ns, trajectory=None):
    with open(path, "w", encoding="utf-8") as f:
        f.write("format_version: 1\n")
        if trajectory:
            f.write(f"trajectory: {trajectory}\n")
        f.write("lidar:\n  name: lidar\n")
        f.write(f"  list: {lidar_list}\n")
        f.write("  T_body_lidar:\n")
        f.write(yaml_rigid(np.eye(4), 4))
        f.write("cameras:\n  rgb:\n")
        f.write(f"    config: {rgb_config}\n")
        f.write(f"    list: {rgb_list}\n")
        f.write(f"    tau_ns: {int(tau_ns)}\n")


def write_list(path, rows):
    with open(path, "w", encoding="utf-8") as f:
        f.write("timestamp_ns,path\n")
        for t, p in rows:
            f.write(f"{int(t)},{p}\n")


def write_trajectory(path, t_ns, pos, R):
    q = rot_to_quat(R)
    with open(path, "w", encoding="utf-8") as f:
        f.write("timestamp_ns,x,y,z,qw,qx,qy,qz\n")
        for i in range(len(t_ns)):
            f.write(f"{int(t_ns[i])},{fmt(pos[i, 0])},{fmt(pos[i, 1])},{fmt(pos[i, 2])},"
                    f"{fmt(q[i, 0])},{fmt(q[i, 1])},{fmt(q[i, 2])},{fmt(q[i, 3])}\n")


def pose_arrays(d):
    pos = np.stack([d["easting"], d["northing"], d["altitude"]], 1)
    return pos, C_enu_sensor(d["roll"], d["pitch"], d["heading"])


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    data = os.path.abspath(args.data)
    out = os.path.abspath(args.out)
    seq_dir = os.path.join(out, "sequence")
    for d in ("sequence/lidar", "point_times", "laser_id"):
        os.makedirs(os.path.join(out, d), exist_ok=True)

    P = load_matrix(os.path.join(data, "calib/P_camera.txt"))
    T_camera_lidar = load_matrix(os.path.join(data, "calib/T_camera_lidar.txt"))
    T_applanix_lidar = load_matrix(os.path.join(data, "calib/T_applanix_lidar.txt"))
    # P_camera.txt is stored as 4x4 (last row 0 0 0 1); the projection is the
    # top 3x4 with a zero 4th column (no stereo baseline term).
    P = P[:3]
    if np.any(P[:, 3] != 0):
        raise SystemExit("P_camera has a non-zero 4th column; this converter assumes none")
    K = P[:, :3]

    lidar = stamp_files(os.path.join(data, "lidar"), ".bin")
    camera = stamp_files(os.path.join(data, "camera"), ".png")
    lidar_ns = np.array([t * 1000 for t, _ in lidar], dtype=np.int64)
    camera_ns = np.array([t * 1000 for t, _ in camera], dtype=np.int64)
    width, height = png_size(camera[0][1])
    sizes = {png_size(p) for _, p in camera}
    if len(sizes) != 1:
        raise SystemExit(f"camera images differ in size: {sizes}")

    # --- trajectory: T_enu_applanix (200 Hz) composed with T_applanix_lidar.
    gps = read_pose_csv(os.path.join(data, "applanix/gps_post_process.csv"), "s")
    t0 = min(lidar_ns[0], camera_ns[0]) - MARGIN_NS
    t1 = max(lidar_ns[-1], camera_ns[-1]) + MARGIN_NS
    keep = (gps["t_ns"] >= t0) & (gps["t_ns"] <= t1)
    g_t = gps["t_ns"][keep]
    g_pos, g_R = pose_arrays({k: v[keep] for k, v in gps.items() if k != "t_ns"})
    if np.any(np.diff(g_t) <= 0):
        raise SystemExit("gps_post_process times are not strictly increasing")
    R_wl = g_R @ T_applanix_lidar[:3, :3]
    p_wl = g_pos + g_R @ T_applanix_lidar[:3, 3]
    origin = np.round(p_wl[0], 3)
    p_local = p_wl - origin
    write_trajectory(os.path.join(seq_dir, "trajectory.csv"), g_t, p_local, R_wl)
    with open(os.path.join(out, "enu_origin.json"), "w", encoding="utf-8") as f:
        json.dump({"subtracted_from_boreas_enu_m": origin.tolist(),
                   "note": "trajectory.csv positions = Boreas ENU position - this offset"}, f,
                  indent=2)

    # --- convention checks -------------------------------------------------
    check = {}
    lp = read_pose_csv(os.path.join(data, "applanix/lidar_poses.csv"), "us")
    cp = read_pose_csv(os.path.join(data, "applanix/camera_poses.csv"), "us")
    # lidar_poses at the scan times, against the composition interpolated there
    sel = np.isin(lp["t_ns"], lidar_ns)
    l_pos, l_R = pose_arrays({k: v[sel] for k, v in lp.items() if k != "t_ns"})
    pi, Ri = interp_poses(g_t, p_wl, R_wl, lp["t_ns"][sel])
    check["lidar_file_times_in_lidar_poses"] = f"{int(sel.sum())} of {len(lidar_ns)}"
    check["lidar_poses_vs_gps_composed_T_applanix_lidar"] = {
        "position_error_m": stats(np.linalg.norm(pi - l_pos, axis=1)),
        "rotation_error_deg": stats(rot_angle_deg(Ri, l_R)),
    }
    # the same with the transposed angle convention, for contrast
    gT = np.transpose(g_R, (0, 2, 1))
    lT = np.transpose(l_R, (0, 2, 1))
    _, RiT = interp_poses(g_t, p_wl, gT @ T_applanix_lidar[:3, :3], lp["t_ns"][sel])
    check["lidar_poses_transposed_convention_rotation_error_deg"] = stats(rot_angle_deg(RiT, lT))
    # camera_poses: T_enu_camera = T_enu_lidar * T_camera_lidar^-1
    sel_c = np.isin(cp["t_ns"], camera_ns)
    c_pos, c_R = pose_arrays({k: v[sel_c] for k, v in cp.items() if k != "t_ns"})
    pi, Ri = interp_poses(g_t, p_wl, R_wl, cp["t_ns"][sel_c])
    T_lc = np.linalg.inv(T_camera_lidar)
    pc = pi + Ri @ T_lc[:3, 3]
    Rc = Ri @ T_lc[:3, :3]
    check["camera_file_times_in_camera_poses"] = f"{int(sel_c.sum())} of {len(camera_ns)}"
    check["camera_poses_vs_lidar_traj_composed_inv_T_camera_lidar"] = {
        "position_error_m": stats(np.linalg.norm(pc - c_pos, axis=1)),
        "rotation_error_deg": stats(rot_angle_deg(Rc, c_R)),
    }
    # the wrong direction (T_camera_lidar used as T_lidar_camera), for contrast
    pc_w = pi + Ri @ T_camera_lidar[:3, 3]
    check["camera_poses_if_T_camera_lidar_were_T_lidar_camera"] = {
        "position_error_m": stats(np.linalg.norm(pc_w - c_pos, axis=1)),
        "rotation_error_deg": stats(rot_angle_deg(Ri @ T_camera_lidar[:3, :3], c_R)),
    }

    # --- LiDAR scans -------------------------------------------------------
    lidar_rows, scan_rows = [], []
    t_min, t_max = [], []
    for i, (t_us, path) in enumerate(lidar):
        pts = read_boreas_lidar(path)
        if not np.all(np.isfinite(pts)):
            raise SystemExit(f"{path}: non-finite values")
        name = f"{i:06d}"
        pts[:, :4].astype("<f4").tofile(os.path.join(seq_dir, "lidar", name + ".bin"))
        pts[:, 5].astype("<f4").tofile(os.path.join(out, "point_times", name + ".f32"))
        lid = pts[:, 4]
        if np.any(lid != np.round(lid)) or lid.min() < 0 or lid.max() > 255:
            raise SystemExit(f"{path}: laser_id is not a small integer")
        lid.astype(np.uint8).tofile(os.path.join(out, "laser_id", name + ".u8"))
        lidar_rows.append((t_us * 1000, f"lidar/{name}.bin"))
        t_min.append(float(pts[:, 5].min()))
        t_max.append(float(pts[:, 5].max()))
        scan_rows.append((i, t_us * 1000, len(pts), t_min[-1], t_max[-1]))
    write_list(os.path.join(seq_dir, "lidar.csv"), lidar_rows)
    write_list(os.path.join(seq_dir, "rgb.csv"), [(t * 1000, p) for t, p in camera])
    with open(os.path.join(out, "scans.csv"), "w", encoding="utf-8") as f:
        f.write("index,timestamp_ns,points,t_min_s,t_max_s\n")
        for r in scan_rows:
            f.write(",".join(str(x) for x in r) + "\n")

    period = np.diff(lidar_ns)
    tau_ns = int(np.median(period)) // 2
    check["point_time"] = {
        "t_min_s": stats(t_min), "t_max_s": stats(t_max),
        "lidar_period_ms": stats(period / 1e6),
        "camera_period_ms": stats(np.diff(camera_ns) / 1e6),
        "note": "t spans about one rotation centred on 0: relative to the file time, "
                "which is the middle of the sweep",
    }

    write_camera_config(os.path.join(seq_dir, "rgb.yaml"), K, width, height, T_camera_lidar,
                        "Boreas camera, rectified images: K = P_camera[:, :3], no distortion; "
                        "extrinsics T_camera_lidar.txt")
    write_manifest(os.path.join(seq_dir, "sequence.yaml"), "lidar.csv", "rgb.yaml", "rgb.csv",
                   tau_ns, "trajectory.csv")
    check["tau_ns"] = tau_ns
    check["trajectory"] = {"poses": int(len(g_t)), "start_ns": int(g_t[0]), "end_ns": int(g_t[-1]),
                           "period_ms": stats(np.diff(g_t) / 1e6)}
    with open(os.path.join(out, "convention_check.json"), "w", encoding="utf-8") as f:
        json.dump(check, f, indent=2)
    print(json.dumps(check, indent=2))
    print(f"wrote {len(lidar)} scans, {len(camera)} images, {len(g_t)} poses -> {out}")


if __name__ == "__main__":
    main()
