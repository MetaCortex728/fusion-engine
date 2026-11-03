#!/usr/bin/env python3
"""Run the Boreas checks on a converted slice and write the results.

    python3 compare.py --data /data/boreas-2021-03-02-13-38 --work /tmp/boreas_work \
        --fuse-cli build/default/apps/fuse_cli --probe build/default/apps/fusion_probe \
        --results /results/boreas [--jobs 12]

Needs the output of convert.py in --work. Everything the engine computes
(association, trajectory interpolation, scan-level compensation, projection)
comes from fuse_cli and fusion_probe; this script prepares their inputs and
measures their outputs.

  item 3  time association      fuse_cli on the converted sequence
  item 4  pose interpolation    fusion_probe interpolate on every 20th pose
  item 2  calibration sweep     fuse_cli on depth-discontinuity scans with
                                perturbed T_camera_lidar, slow frames
  item 5  compensation          fuse_cli on the same scans with and without
                                trajectory, all frames
  item 6  per-point times       measured from the Boreas per-point t

Edge alignment metric: LiDAR depth-discontinuity points (foreground side of
a range jump between neighbouring returns of one laser) are projected by the
engine; image edges are non-maximum-suppressed Sobel maxima of the Gaussian-
smoothed luminance above the image's 90th gradient percentile; the metric is
the mean distance (px) from the projected points to the nearest edge pixel,
each distance capped at CAP_PX.

Writes to --results: association.csv, interpolation.csv,
interpolation_bins.csv, calibration_sweep.csv, compensation.csv,
compensation_bins.csv, point_times.csv, summary.json, report.md, overlays.
"""

import argparse
import json
import multiprocessing as mp
import os
import shutil
import subprocess
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from boreas_common import (axis_angle, nearest_rank, quat_to_rot, read_pose_csv,  # noqa: E402
                           rot_angle_deg, stats, write_png_rgb)

CAP_PX = 50            # distance cap of the edge metric
EDGE_SIGMA = 2.0       # Gaussian smoothing before Sobel, px
EDGE_PERCENTILE = 90   # gradient-magnitude threshold, percentile over the image
DISC_MIN_JUMP_M = 1.0  # range jump for a depth discontinuity: max(1 m, 10 % of range)
DISC_REL_JUMP = 0.10
DISC_MAX_GAP_DEG = 0.6  # neighbouring returns of one laser must be this close in azimuth
DISC_RANGE_M = (3.0, 60.0)
SLOW_MPS = 1.0
KEEP_EVERY = 20        # 200 Hz -> 10 Hz for the interpolation check

ROT_STEPS_DEG = [-1.0, -0.5, -0.25, 0.25, 0.5, 1.0]
TRANS_STEPS_M = [-0.10, -0.05, 0.05, 0.10]
AXES = {"x": [1, 0, 0], "y": [0, 1, 0], "z": [0, 0, 1]}


# --- small helpers -----------------------------------------------------------

def run(cmd):
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        raise SystemExit(f"command failed ({r.returncode}): {' '.join(cmd)}\n{r.stdout}\n{r.stderr}")
    return r.stdout


def read_list(path):
    rows = []
    with open(path, encoding="utf-8") as f:
        f.readline()
        for line in f:
            if line.strip():
                t, p = line.rstrip("\n").split(",", 1)
                rows.append((int(t), p))
    return rows


def write_list(path, rows):
    with open(path, "w", encoding="utf-8") as f:
        f.write("timestamp_ns,path\n")
        for t, p in rows:
            f.write(f"{int(t)},{p}\n")


def read_csv_dicts(path):
    with open(path, encoding="utf-8") as f:
        head = f.readline().strip().split(",")
        return [dict(zip(head, line.rstrip("\n").split(","))) for line in f if line.strip()]


def write_csv(path, header, rows):
    with open(path, "w", encoding="utf-8") as f:
        f.write(",".join(header) + "\n")
        for r in rows:
            f.write(",".join(fmt(x) for x in r) + "\n")


def fmt(x):
    if isinstance(x, (bool, np.bool_)):
        return "1" if x else "0"
    if isinstance(x, (int, np.integer)):
        return str(int(x))
    if isinstance(x, (float, np.floating)):
        return "" if not np.isfinite(x) else f"{float(x):.6g}"
    return str(x)


def yaml_rigid(T, indent):
    pad = " " * indent
    rows = ", ".join("[" + ", ".join(repr(float(x)) for x in r) + "]" for r in T[:3, :3])
    return (f"{pad}rotation: [{rows}]\n"
            f"{pad}translation: [{', '.join(repr(float(x)) for x in T[:3, 3])}]\n")


def read_camera_yaml(path):
    """K, resolution and T_camera_lidar from the converter's rgb.yaml (its own
    flow-style output; not a general YAML reader)."""
    vals = {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            s = line.strip()
            if ":" in s and not s.startswith("#"):
                k, v = s.split(":", 1)
                vals[k.strip()] = v.strip()
    K = np.array(json.loads(vals["K"]))
    w, h = json.loads(vals["resolution"])
    T = np.eye(4)
    T[:3, :3] = np.array(json.loads(vals["rotation"]))
    T[:3, 3] = np.array(json.loads(vals["translation"]))
    return K, int(w), int(h), T


def write_camera_yaml(path, K, w, h, T):
    with open(path, "w", encoding="utf-8") as f:
        f.write("name: rgb\n")
        f.write(f"resolution: [{w}, {h}]\n")
        f.write("intrinsics:\n  K: [" +
                ", ".join("[" + ", ".join(repr(float(x)) for x in r) + "]" for r in K) + "]\n")
        f.write("distortion: [0.0, 0.0, 0.0, 0.0, 0.0]\n")
        f.write("extrinsics:\n  source: lidar\n")
        f.write(yaml_rigid(T, 2))
        f.write("time_offset_ns: 0\n")


def write_manifest(path, lidar_list, rgb_yaml, rgb_list, tau_ns, trajectory=None):
    with open(path, "w", encoding="utf-8") as f:
        f.write("format_version: 1\n")
        if trajectory:
            f.write(f"trajectory: {trajectory}\n")
        f.write(f"lidar:\n  name: lidar\n  list: {lidar_list}\n  T_body_lidar:\n")
        f.write(yaml_rigid(np.eye(4), 4))
        f.write(f"cameras:\n  rgb:\n    config: {rgb_yaml}\n    list: {rgb_list}\n"
                f"    tau_ns: {int(tau_ns)}\n")


def read_tau(manifest):
    with open(manifest, encoding="utf-8") as f:
        for line in f:
            if line.strip().startswith("tau_ns:"):
                return int(line.split(":")[1])
    raise SystemExit(f"{manifest}: no tau_ns")


# --- motion state from the 200 Hz solution ----------------------------------

class Motion:
    def __init__(self, data):
        g = read_pose_csv(os.path.join(data, "applanix/gps_post_process.csv"), "s")
        self.t = g["t_ns"]
        self.speed = np.sqrt(g["vel_east"] ** 2 + g["vel_north"] ** 2 + g["vel_up"] ** 2)
        self.yaw_rate = np.abs(g["angvel_z"])

    def at(self, t_ns):
        t = np.asarray(t_ns, dtype=np.int64)
        x = (t - self.t[0]).astype(float)
        xs = (self.t - self.t[0]).astype(float)
        return np.interp(x, xs, self.speed), np.interp(x, xs, self.yaw_rate)

    def max_over(self, t0_ns, t1_ns):
        """Max speed over [t0, t1] (both arrays)."""
        out = np.empty(len(t0_ns))
        for k, (a, b) in enumerate(zip(t0_ns, t1_ns)):
            lo, hi = min(a, b), max(a, b)
            m = (self.t >= lo) & (self.t <= hi)
            out[k] = max(self.speed[m].max() if m.any() else 0.0, *self.at([lo, hi])[0])
        return out


# --- item 3: association ------------------------------------------------------

def check_association(work, fuse_cli, results):
    manifest = os.path.join(work, "sequence/sequence.yaml")
    out = os.path.join(work, "run_full")
    run([fuse_cli, manifest, "--out", out, "--frames", "1"])
    rows = read_csv_dicts(os.path.join(out, "association_rgb.csv"))
    shutil.copy(os.path.join(out, "association_rgb.csv"), os.path.join(results, "association.csv"))
    dt = np.array([int(r["delta_t_ns"]) for r in rows])
    acc = np.array([r["accepted"] == "1" for r in rows])
    tau = read_tau(manifest)
    images = read_list(os.path.join(work, "sequence/rgb.csv"))
    t_img = np.array([t for t, _ in images])
    t_match = np.array([int(r["t_match_ns"]) for r in rows])
    used = np.unique(t_match[acc])
    res = {
        "scans": int(len(rows)), "images": int(len(images)), "tau_ns": tau,
        "accepted": int(acc.sum()), "rejected": int((~acc).sum()),
        "match_rate": float(acc.mean()),
        "abs_delta_t_ms": stats(np.abs(dt[acc]) / 1e6),
        "delta_t_ms_signed": {"min": float(dt.min() / 1e6), "median": float(np.median(dt) / 1e6),
                              "max": float(dt.max() / 1e6)},
        "distinct_images_used": int(len(used)),
        "images_never_matched": int(len(t_img) - len(np.intersect1d(t_img, used))),
        "images_matched_twice": int(np.sum(np.unique(t_match[acc], return_counts=True)[1] > 1)),
    }
    return res, rows


# --- item 4: pose interpolation ----------------------------------------------

def check_interpolation(work, probe, motion, results):
    src = os.path.join(work, "sequence/trajectory.csv")
    with open(src, encoding="utf-8") as f:
        header = f.readline()
        lines = [ln for ln in f if ln.strip()]
    d = os.path.join(work, "interp")
    os.makedirs(d, exist_ok=True)
    kept = list(range(0, len(lines), KEEP_EVERY))
    with open(os.path.join(d, "trajectory_10hz.csv"), "w", encoding="utf-8") as f:
        f.write(header)
        f.writelines(lines[k] for k in kept)
    held = [k for k in range(kept[0], kept[-1] + 1) if k % KEEP_EVERY != 0]
    truth = np.array([[float(x) for x in lines[k].split(",")[1:]] for k in held])
    t_held = np.array([int(lines[k].split(",")[0]) for k in held], dtype=np.int64)
    with open(os.path.join(d, "times.csv"), "w", encoding="utf-8") as f:
        f.write("timestamp_ns\n")
        f.writelines(f"{t}\n" for t in t_held)
    run([probe, "interpolate", os.path.join(d, "trajectory_10hz.csv"), "--times",
         os.path.join(d, "times.csv"), "--out", os.path.join(d, "interpolated.csv")])
    est = np.loadtxt(os.path.join(d, "interpolated.csv"), delimiter=",", skiprows=1,
                     dtype=np.float64, usecols=range(1, 8))
    t_est = np.loadtxt(os.path.join(d, "interpolated.csv"), delimiter=",", skiprows=1,
                       dtype=np.int64, usecols=0)
    if not np.array_equal(t_est, t_held):
        raise SystemExit("interpolated times differ from the requested times")
    pos_err = np.linalg.norm(est[:, :3] - truth[:, :3], axis=1)
    rot_err = rot_angle_deg(quat_to_rot(truth[:, 3:]), quat_to_rot(est[:, 3:]))
    speed, yaw = motion.at(t_held)
    write_csv(os.path.join(results, "interpolation.csv"),
              ["timestamp_ns", "pos_err_m", "rot_err_deg", "speed_mps", "yaw_rate_radps"],
              zip(t_held, pos_err, rot_err, speed, yaw))
    bins = []
    for name, val, edges in (("speed_mps", speed, [0, 1, 3, 6, 9, np.inf]),
                             ("yaw_rate_radps", yaw, [0, 0.1, 0.3, 0.5, np.inf])):
        for lo, hi in zip(edges[:-1], edges[1:]):
            m = (val >= lo) & (val < hi)
            if not m.any():
                continue
            sp, sr = stats(pos_err[m]), stats(rot_err[m])
            bins.append([name, lo, hi, int(m.sum()), sp["median"], sp["p95"], sp["max"],
                         sr["median"], sr["p95"], sr["max"]])
    write_csv(os.path.join(results, "interpolation_bins.csv"),
              ["bin_by", "lo", "hi", "n", "pos_median_m", "pos_p95_m", "pos_max_m",
               "rot_median_deg", "rot_p95_deg", "rot_max_deg"], bins)
    # the worst case: which motion produces it
    worst = int(np.argmax(pos_err))
    return {"kept_poses": len(kept), "held_out": len(held),
            "position_error_m": stats(pos_err), "rotation_error_deg": stats(rot_err),
            "worst_position": {"t_ns": int(t_held[worst]), "speed_mps": float(speed[worst]),
                               "yaw_rate_radps": float(yaw[worst])},
            "bins": bins}


# --- depth discontinuities -------------------------------------------------

def discontinuities(pts):
    """Indices of foreground depth-discontinuity points: per laser, returns
    in firing order (per-point time), neighbours closer than
    DISC_MAX_GAP_DEG in azimuth, and a range jump of
    max(DISC_MIN_JUMP_M, DISC_REL_JUMP * r_near)."""
    lid = pts[:, 4].astype(np.int64)
    order = np.lexsort((pts[:, 5], lid))
    p = pts[order].astype(np.float64)
    r = np.linalg.norm(p[:, :3], axis=1)
    az = np.degrees(np.arctan2(p[:, 1], p[:, 0]))
    same = lid[order][1:] == lid[order][:-1]
    gap = np.abs((az[1:] - az[:-1] + 180) % 360 - 180)
    adj = same & (gap <= DISC_MAX_GAP_DEG)
    ra, rb = r[:-1], r[1:]
    a_fg = adj & (rb - ra >= np.maximum(DISC_MIN_JUMP_M, DISC_REL_JUMP * ra))
    b_fg = adj & (ra - rb >= np.maximum(DISC_MIN_JUMP_M, DISC_REL_JUMP * rb))
    mask = np.zeros(len(p), bool)
    mask[:-1] |= a_fg
    mask[1:] |= b_fg
    mask &= (r >= DISC_RANGE_M[0]) & (r <= DISC_RANGE_M[1])
    return np.sort(order[mask])


def build_edge_scans(work, T_cl):
    """Writes edges/lidar/NNNNNN.bin with the discontinuity points in front of
    the camera (z_camera > 0; the engine decides visibility), plus their
    indices and per-point times."""
    d = os.path.join(work, "edges")
    for s in ("lidar", "index", "point_times"):
        os.makedirs(os.path.join(d, s), exist_ok=True)
    scans = read_list(os.path.join(work, "sequence/lidar.csv"))
    rows = []
    counts = []
    for i, (t, rel) in enumerate(scans):
        name = f"{i:06d}"
        xyzi = np.fromfile(os.path.join(work, "sequence", rel), dtype="<f4").reshape(-1, 4)
        tt = np.fromfile(os.path.join(work, "point_times", name + ".f32"), dtype="<f4")
        lid = np.fromfile(os.path.join(work, "laser_id", name + ".u8"), dtype=np.uint8)
        pts = np.column_stack([xyzi, lid.astype(np.float32), tt])
        idx = discontinuities(pts)
        zc = (T_cl[2, :3] @ xyzi[idx, :3].T.astype(np.float64)) + T_cl[2, 3]
        idx = idx[zc > 0]
        xyzi[idx].astype("<f4").tofile(os.path.join(d, "lidar", name + ".bin"))
        idx.astype("<u4").tofile(os.path.join(d, "index", name + ".u32"))
        tt[idx].astype("<f4").tofile(os.path.join(d, "point_times", name + ".f32"))
        rows.append((t, os.path.join(d, "lidar", name + ".bin")))
        counts.append(len(idx))
    write_list(os.path.join(d, "lidar.csv"), rows)
    return stats(counts)


def run_variant(work, fuse_cli, name, lidar_list, rgb_yaml, trajectory, frames=None):
    """Runs fuse_cli on a manifest in edges/<name>/ and returns per-frame
    (u, v) arrays (NaN where not visible) and the summary rows."""
    d = os.path.join(work, "edges", name)
    os.makedirs(d, exist_ok=True)
    tau = read_tau(os.path.join(work, "sequence/sequence.yaml"))
    manifest = os.path.join(d, "sequence.yaml")
    write_manifest(manifest, lidar_list, rgb_yaml, os.path.join(work, "sequence/rgb.csv"), tau,
                   trajectory)
    out = os.path.join(d, "out")
    if os.path.isdir(out):
        shutil.rmtree(out)
    cmd = [fuse_cli, manifest, "--out", out]
    if frames:
        cmd += ["--frames", str(frames)]
    run(cmd)
    summary = read_csv_dicts(os.path.join(out, "summary.csv"))
    uv = []
    for k in range(len(summary)):
        path = os.path.join(out, f"frame_{k:06d}.csv")
        with open(path, encoding="utf-8") as f:
            head = f.readline().strip().split(",")
            cu, cv = head.index("rgb_u"), head.index("rgb_v")
            a = []
            for line in f:
                s = line.split(",")
                a.append((float(s[cu]) if s[cu] else np.nan, float(s[cv]) if s[cv] else np.nan))
        uv.append(np.array(a, dtype=np.float64).reshape(-1, 2))
        os.remove(path)  # per-point CSVs are large; (u, v) is kept in memory
    return uv, summary


# --- image edges and the distance transform -----------------------------------

def gaussian_blur(img, sigma):
    r = int(np.ceil(3 * sigma))
    x = np.arange(-r, r + 1)
    k = np.exp(-0.5 * (x / sigma) ** 2)
    k /= k.sum()
    p = np.pad(img, ((0, 0), (r, r)), mode="edge")
    tmp = sum(k[i] * p[:, i:i + img.shape[1]] for i in range(2 * r + 1))
    p = np.pad(tmp, ((r, r), (0, 0)), mode="edge")
    return sum(k[i] * p[i:i + img.shape[0], :] for i in range(2 * r + 1))


def edge_map(gray):
    g = gaussian_blur(gray.astype(np.float32), EDGE_SIGMA)
    p = np.pad(g, 1, mode="edge")
    gx = ((p[:-2, 2:] + 2 * p[1:-1, 2:] + p[2:, 2:]) - (p[:-2, :-2] + 2 * p[1:-1, :-2] + p[2:, :-2]))
    gy = ((p[2:, :-2] + 2 * p[2:, 1:-1] + p[2:, 2:]) - (p[:-2, :-2] + 2 * p[:-2, 1:-1] + p[:-2, 2:]))
    mag = np.hypot(gx, gy)
    ang = (np.degrees(np.arctan2(gy, gx)) + 180.0) % 180.0
    q = (np.round(ang / 45.0).astype(np.int64)) % 4  # 0: horizontal gradient, 1: 45, 2: 90, 3: 135
    m = np.pad(mag, 1, mode="constant")
    h, w = mag.shape
    keep = np.zeros_like(mag, dtype=bool)
    for k, (dy, dx) in enumerate([(0, 1), (1, 1), (1, 0), (1, -1)]):
        a = m[1 + dy:1 + dy + h, 1 + dx:1 + dx + w]
        b = m[1 - dy:1 - dy + h, 1 - dx:1 - dx + w]
        keep |= (q == k) & (mag >= a) & (mag >= b)
    thr = np.percentile(mag, EDGE_PERCENTILE)
    return keep & (mag > thr)


def distance_transform(edges, cap):
    """Euclidean distance (px) to the nearest edge pixel, exact up to `cap`
    and clamped to `cap` beyond it."""
    h, w = edges.shape
    big = float(cap + 1)
    col = np.full((h, w), big, np.float32)
    run_ = np.full(w, big, np.float32)
    for y in range(h):  # downward
        run_ = np.where(edges[y], 0.0, np.minimum(run_ + 1, big))
        col[y] = run_
    run_ = np.full(w, big, np.float32)
    for y in range(h - 1, -1, -1):  # upward
        run_ = np.where(edges[y], 0.0, np.minimum(run_ + 1, big))
        col[y] = np.minimum(col[y], run_)
    g2 = col * col
    d2 = g2.copy()
    for dx in range(1, cap + 1):
        sh = np.full_like(g2, big * big)
        sh[:, dx:] = g2[:, :-dx]
        np.minimum(d2, sh + dx * dx, out=d2)
        sh = np.full_like(g2, big * big)
        sh[:, :-dx] = g2[:, dx:]
        np.minimum(d2, sh + dx * dx, out=d2)
    return np.minimum(np.sqrt(d2), cap)


# --- metric evaluation in worker processes -----------------------------------

_JOB = {}


def _evaluate_image(j):
    """For camera image j: metric for every (scan, variant) projected into it."""
    W, H, gray_dir = _JOB["w"], _JOB["h"], _JOB["gray_dir"]
    gray = np.fromfile(os.path.join(gray_dir, f"gray_{j:06d}.u8"), dtype=np.uint8).reshape(H, W)
    edges = edge_map(gray)
    dt = distance_transform(edges, CAP_PX)
    out = []
    for (i, variant) in _JOB["by_image"][j]:
        uv = _JOB["uv"][variant][i]
        vis = np.isfinite(uv[:, 0])
        c = np.clip(np.round(uv[vis, 0]).astype(np.int64), 0, W - 1)
        r = np.clip(np.round(uv[vis, 1]).astype(np.int64), 0, H - 1)
        d = dt[r, c]
        out.append((i, variant, int(vis.sum()), float(d.mean()) if len(d) else np.nan,
                    float(np.median(d)) if len(d) else np.nan, int(np.sum(d >= CAP_PX))))
    return j, float(edges.mean()), out


def evaluate(work, probe, uv_by_variant, assoc_by_variant, W, H, jobs):
    """assoc_by_variant[v][i] = image index or None."""
    by_image = {}
    for v, assoc in assoc_by_variant.items():
        for i, j in enumerate(assoc):
            if j is not None:
                by_image.setdefault(j, []).append((i, v))
    gray_dir = os.path.join(work, "gray")
    need = sorted(j for j in by_image
                  if not os.path.exists(os.path.join(gray_dir, f"gray_{j:06d}.u8")))
    if need:
        run([probe, "gray", os.path.join(work, "sequence/sequence.yaml"), "--out", gray_dir,
             "--indices", ",".join(str(j) for j in need)])
    _JOB.update(w=W, h=H, gray_dir=gray_dir, by_image=by_image, uv=uv_by_variant)
    res, density = {}, {}
    with mp.get_context("fork").Pool(jobs) as pool:
        for j, dens, out in pool.imap_unordered(_evaluate_image, sorted(by_image)):
            density[j] = dens
            for (i, v, n, mean, med, capped) in out:
                res[(v, i)] = {"image": j, "n": n, "mean_px": mean, "median_px": med,
                               "capped": capped}
    return res, density


# --- overlays ---------------------------------------------------------------

def overlay(work, results, name, j, layers, W, H, crop=None):
    gray = np.fromfile(os.path.join(work, "gray", f"gray_{j:06d}.u8"),
                       dtype=np.uint8).reshape(H, W)
    img = np.repeat((gray * 0.7).astype(np.uint8)[:, :, None], 3, axis=2)
    for uv, color in layers:
        vis = np.isfinite(uv[:, 0])
        c = np.round(uv[vis, 0]).astype(int)
        r = np.round(uv[vis, 1]).astype(int)
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                cc, rr = c + dc, r + dr
                ok = (cc >= 0) & (cc < W) & (rr >= 0) & (rr < H)
                img[rr[ok], cc[ok]] = color
    if crop:
        x0, y0, x1, y1 = crop
        img = img[y0:y1, x0:x1]
    write_png_rgb(os.path.join(results, name), img)


# --- main -----------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", required=True)
    ap.add_argument("--work", required=True)
    ap.add_argument("--fuse-cli", required=True)
    ap.add_argument("--probe", required=True)
    ap.add_argument("--results", required=True)
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    args = ap.parse_args()
    work = os.path.abspath(args.work)
    results = os.path.abspath(args.results)
    os.makedirs(results, exist_ok=True)
    fuse_cli, probe = os.path.abspath(args.fuse_cli), os.path.abspath(args.probe)

    motion = Motion(args.data)
    K, W, H, T_cl = read_camera_yaml(os.path.join(work, "sequence/rgb.yaml"))
    scans = read_list(os.path.join(work, "sequence/lidar.csv"))
    t_scan = np.array([t for t, _ in scans], dtype=np.int64)
    images = read_list(os.path.join(work, "sequence/rgb.csv"))
    t_img = np.array([t for t, _ in images], dtype=np.int64)
    summary = {"slice": os.path.basename(os.path.normpath(args.data)),
               "scans": len(scans), "images": len(images)}
    with open(os.path.join(work, "convention_check.json"), encoding="utf-8") as f:
        summary["conventions"] = json.load(f)

    print("item 3: association")
    summary["association"], assoc_rows = check_association(work, fuse_cli, results)
    img_index = {t: k for k, t in enumerate(t_img)}

    print("item 4: pose interpolation")
    summary["interpolation"] = check_interpolation(work, probe, motion, results)

    print("discontinuity scans")
    summary["discontinuity_points_per_scan"] = build_edge_scans(work, T_cl)
    edges_list = os.path.join(work, "edges/lidar.csv")
    rgb_yaml = os.path.join(work, "sequence/rgb.yaml")
    traj = os.path.join(work, "sequence/trajectory.csv")

    speed_scan, yaw_scan = motion.at(t_scan)
    delta_t = np.array([int(r["delta_t_ns"]) for r in assoc_rows], dtype=np.int64)
    accepted = np.array([r["accepted"] == "1" for r in assoc_rows])
    t_cam = t_scan + delta_t
    slow_speed = motion.max_over(t_scan, t_cam)
    slow = accepted & (slow_speed < SLOW_MPS)
    n_first_slow = int(np.argmin(slow)) if not slow.all() else len(slow)
    slow_frames = np.nonzero(slow)[0]

    print("item 5: with and without compensation (engine)")
    uv, assoc_v, summ = {}, {}, {}
    for name, trj in (("nocomp", None), ("comp", traj)):
        uv[name], summ[name] = run_variant(work, fuse_cli, name, edges_list, rgb_yaml, trj)
        assoc_v[name] = [int(r["rgb_index"]) if r["rgb_accepted"] == "1" else None
                         for r in summ[name]]
    comp_flag = np.array([r["rgb_compensated"] == "1" for r in summ["comp"]])
    nocomp_flag = np.array([r["rgb_compensated"] == "1" for r in summ["nocomp"]])

    # diagnostic: scan time moved to the median per-point time of the scan's
    # visible discontinuity points (the time the camera-facing sector was
    # measured). Same engine compensation, different scan time.
    t_fov = np.zeros(len(scans), dtype=np.int64)
    fov_rel_ms = np.full(len(scans), np.nan)
    for i in range(len(scans)):
        tt = np.fromfile(os.path.join(work, "edges/point_times", f"{i:06d}.f32"), dtype="<f4")
        vis = np.isfinite(uv["nocomp"][i][:, 0])
        rel = float(np.median(tt[vis])) if vis.any() else 0.0
        fov_rel_ms[i] = rel * 1e3
        t_fov[i] = t_scan[i] + int(round(rel * 1e9))
    if np.any(np.diff(t_fov) <= 0):
        raise SystemExit("sector times are not increasing")
    fov_list = os.path.join(work, "edges/lidar_fovtime.csv")
    write_list(fov_list, [(t, p) for t, (_, p) in zip(t_fov, read_list(edges_list))])
    uv["fovtime"], summ["fovtime"] = run_variant(work, fuse_cli, "fovtime", fov_list, rgb_yaml,
                                                 traj)
    assoc_v["fovtime"] = [int(r["rgb_index"]) if r["rgb_accepted"] == "1" else None
                          for r in summ["fovtime"]]

    print(f"item 2: calibration sweep on {len(slow_frames)} slow frames")
    cal_variants = [("given", np.eye(4))]
    for ax, vec in AXES.items():
        for s in ROT_STEPS_DEG:
            D = np.eye(4)
            D[:3, :3] = axis_angle(vec, s)
            cal_variants.append((f"rot_{ax}_{s:+g}deg", D))
        for s in TRANS_STEPS_M:
            D = np.eye(4)
            D[:3, 3] = np.array(vec) * s
            cal_variants.append((f"trans_{ax}_{s * 100:+g}cm", D))
    n_frames_cal = int(slow_frames.max()) + 1 if len(slow_frames) else 0
    cal_dir = os.path.join(work, "edges/calib")
    os.makedirs(cal_dir, exist_ok=True)
    for name, D in cal_variants:
        y = os.path.join(cal_dir, f"{name}.yaml")
        write_camera_yaml(y, K, W, H, D @ T_cl)
        key = "cal:" + name
        uv[key], summ[key] = run_variant(work, fuse_cli, "cal_" + name, edges_list, y, None,
                                         frames=n_frames_cal)
        full = [None] * len(scans)
        for i in slow_frames:
            r = summ[key][i]
            full[i] = int(r["rgb_index"]) if r["rgb_accepted"] == "1" else None
        assoc_v[key] = full
        uv[key] = uv[key] + [np.zeros((0, 2))] * (len(scans) - len(uv[key]))

    print("evaluating edge alignment")
    metric, density = evaluate(work, probe, uv, assoc_v, W, H, args.jobs)
    summary["edge_density"] = stats(list(density.values()))

    # --- item 2 results --------------------------------------------------
    cal_rows = []
    base = np.array([metric[("cal:given", i)]["mean_px"] for i in slow_frames])
    cal_tab = {}
    for name, _ in cal_variants:
        v = np.array([metric[("cal:" + name, i)]["mean_px"] for i in slow_frames])
        nvis = np.array([metric[("cal:" + name, i)]["n"] for i in slow_frames])
        better = int(np.sum(v < base))
        cal_tab[name] = float(np.mean(v))
        cal_rows.append([name, len(v), float(np.mean(v)), float(np.mean(v) - np.mean(base)),
                         better, float(np.mean(nvis))])
    write_csv(os.path.join(results, "calibration_sweep.csv"),
              ["variant", "frames", "mean_px", "minus_given_px", "frames_better_than_given",
               "mean_points"], cal_rows)
    # per axis: is the given calibration the minimum, and where does a
    # parabola through the sweep put the minimum?
    axis_fit = {}
    for kind, steps, unit, fmt_ in (("rot", ROT_STEPS_DEG, "deg", lambda s: f"{s:+g}deg"),
                                    ("trans", TRANS_STEPS_M, "m",
                                     lambda s: f"{s * 100:+g}cm")):
        for ax in AXES:
            xs = [0.0] + list(steps)
            ys = [cal_tab["given"]] + [cal_tab[f"{kind}_{ax}_{fmt_(s)}"] for s in steps]
            order = np.argsort(xs)
            xs, ys = np.array(xs)[order], np.array(ys)[order]
            a, b, _c = np.polyfit(xs, ys, 2)
            vertex = -b / (2 * a) if a > 0 else float("nan")
            k0 = int(np.nonzero(xs == 0)[0][0])
            axis_fit[f"{kind}_{ax}"] = {
                "unit": unit, "steps": xs.tolist(), "mean_px": ys.tolist(),
                "given_is_minimum": bool(np.argmin(ys) == k0),
                "given_below_both_smallest_steps": bool(ys[k0] < ys[k0 - 1] and ys[k0] < ys[k0 + 1]),
                "parabola_vertex": float(vertex),
            }
    summary["calibration"] = {
        "frames": [int(x) for x in slow_frames], "max_speed_mps": float(slow_speed[slow].max()),
        "compensation": "off (vehicle below 1 m/s between scan and image times)",
        "given_mean_px": cal_tab["given"], "axes": axis_fit,
    }

    # --- item 5 results --------------------------------------------------
    comp_rows = []
    for i in range(len(scans)):
        if not accepted[i]:
            continue
        a, b, c = (metric.get(("nocomp", i)), metric.get(("comp", i)), metric.get(("fovtime", i)))
        same_img = assoc_v["fovtime"][i] == assoc_v["nocomp"][i]
        t_c = t_scan[i] + delta_t[i]
        comp_rows.append([
            i, int(delta_t[i]), float(speed_scan[i]), float(yaw_scan[i]),
            abs(delta_t[i]) * 1e-9 * speed_scan[i], fov_rel_ms[i],
            (t_c - t_fov[i]) / 1e6, bool(comp_flag[i]), a["n"], a["mean_px"], b["mean_px"],
            a["mean_px"] - b["mean_px"], c["mean_px"] if (c and same_img) else np.nan,
            (a["mean_px"] - c["mean_px"]) if (c and same_img) else np.nan])
    head = ["scan", "delta_t_ns", "speed_mps", "yaw_rate_radps", "abs_dt_times_speed_m",
            "sector_time_rel_scan_ms", "camera_minus_sector_ms", "compensated", "points",
            "nocomp_mean_px", "comp_mean_px", "improvement_px", "sector_comp_mean_px",
            "sector_improvement_px"]
    write_csv(os.path.join(results, "compensation.csv"), head, comp_rows)
    C = np.array([[float(x) for x in r] for r in comp_rows])
    col = {h: k for k, h in enumerate(head)}
    bins = []
    for name, key, edges in (("abs_dt_times_speed_m", "abs_dt_times_speed_m",
                              [0, 0.05, 0.1, 0.2, 0.3, np.inf]),
                             ("yaw_rate_radps", "yaw_rate_radps", [0, 0.1, 0.3, 0.5, np.inf])):
        val = C[:, col[key]]
        for lo, hi in zip(edges[:-1], edges[1:]):
            m = (val >= lo) & (val < hi)
            if not m.any():
                continue
            imp, simp = C[m, col["improvement_px"]], C[m, col["sector_improvement_px"]]
            simp = simp[np.isfinite(simp)]
            bins.append([name, lo, hi, int(m.sum()), float(np.mean(C[m, col["nocomp_mean_px"]])),
                         float(np.mean(C[m, col["comp_mean_px"]])), float(np.median(imp)),
                         float(np.mean(imp > 0)),
                         float(np.median(simp)) if len(simp) else np.nan,
                         float(np.mean(simp > 0)) if len(simp) else np.nan])
    write_csv(os.path.join(results, "compensation_bins.csv"),
              ["bin_by", "lo", "hi", "frames", "nocomp_mean_px", "comp_mean_px",
               "median_improvement_px", "frac_improved", "sector_median_improvement_px",
               "sector_frac_improved"], bins)
    moving = C[:, col["speed_mps"]] >= SLOW_MPS
    sec_ok = np.isfinite(C[:, col["sector_comp_mean_px"]])
    dt_ms = C[:, col["delta_t_ns"]] / 1e6
    fov_ms = C[:, col["sector_time_rel_scan_ms"]]
    summary["compensation"] = {
        "frames": int(len(C)), "compensated_by_engine": int(C[:, col["compensated"]].sum()),
        "nocomp_run_compensated": int(nocomp_flag.sum()),
        "moving_frames": int(moving.sum()),
        "moving_mean_px": {"nocomp": float(np.mean(C[moving, col["nocomp_mean_px"]])),
                           "comp": float(np.mean(C[moving, col["comp_mean_px"]])),
                           "sector_time_comp": float(np.mean(
                               C[moving & sec_ok, col["sector_comp_mean_px"]]))},
        "moving_frac_improved_by_comp": float(np.mean(C[moving, col["improvement_px"]] > 0)),
        "sector_time_same_image": int(sec_ok.sum()),
        "delta_t_vs_sector_time": {
            "corr": float(np.corrcoef(dt_ms, fov_ms)[0, 1]),
            "camera_minus_sector_ms": stats(np.abs(C[:, col["camera_minus_sector_ms"]])),
            "abs_delta_t_ms": stats(np.abs(dt_ms)),
        },
        "bins": bins,
    }

    # --- item 6: per-point time spread -----------------------------------
    pt_rows = []
    for i in range(len(scans)):
        tt = np.fromfile(os.path.join(work, "point_times", f"{i:06d}.f32"), dtype="<f4")
        xyz = np.fromfile(os.path.join(work, "sequence", scans[i][1]),
                          dtype="<f4").reshape(-1, 4)[:, :3].astype(np.float64)
        pc = xyz @ T_cl[:3, :3].T + T_cl[:3, 3]
        front = pc[:, 2] > 0
        uvw = pc[front] @ K.T
        u, v = uvw[:, 0] / uvw[:, 2], uvw[:, 1] / uvw[:, 2]
        inside = (u >= -0.5) & (u < W - 0.5) & (v >= -0.5) & (v < H - 0.5)
        tf = tt[front][inside]
        span = float(tt.max() - tt.min())
        fspan = float(tf.max() - tf.min()) if len(tf) else np.nan
        pt_rows.append([i, len(tt), span * 1e3, len(tf), fspan * 1e3,
                        float(np.median(tf)) * 1e3 if len(tf) else np.nan, speed_scan[i],
                        yaw_scan[i], span * speed_scan[i], fspan * speed_scan[i],
                        np.degrees(span * yaw_scan[i]), np.degrees(fspan * yaw_scan[i])])
    ph = ["scan", "points", "span_ms", "camera_points", "camera_span_ms",
          "camera_median_t_ms", "speed_mps", "yaw_rate_radps", "span_displacement_m",
          "camera_span_displacement_m", "span_rotation_deg", "camera_span_rotation_deg"]
    write_csv(os.path.join(results, "point_times.csv"), ph, pt_rows)
    Pm = np.array(pt_rows, dtype=float)
    pc_ = {h: k for k, h in enumerate(ph)}
    summary["point_times"] = {k: stats(Pm[:, pc_[k]]) for k in ph[2:] if k not in (
        "camera_points", "speed_mps", "yaw_rate_radps")}
    summary["point_times"]["camera_median_t_vs_delta_t_corr"] = float(np.corrcoef(
        Pm[accepted, pc_["camera_median_t_ms"]], delta_t[accepted] / 1e6)[0, 1])

    # --- overlays ----------------------------------------------------------
    ov = []
    red, green, cyan = (255, 40, 40), (40, 255, 40), (40, 200, 255)
    score = C[:, col["abs_dt_times_speed_m"]]
    picks = [int(C[k, col["scan"]]) for k in np.argsort(-score)[:3]]
    if len(slow_frames):
        picks.append(int(slow_frames[len(slow_frames) // 2]))
    for i in picks:
        j = assoc_v["nocomp"][i]
        layers = [(uv["nocomp"][i], red), (uv["comp"][i], green)]
        if assoc_v["fovtime"][i] == j:
            layers.append((uv["fovtime"][i], cyan))
        name = f"overlay_scan{i:04d}_image{j:04d}.png"
        overlay(work, results, name, j, layers, W, H)
        ov.append(name)
    summary["overlays"] = {"files": ov, "colours": "red: no compensation, green: engine "
                           "scan-level compensation, cyan: compensation from the sector time"}

    with open(os.path.join(results, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, default=float)
    write_report(results, summary, cal_rows)
    print(f"results -> {results}")


def write_report(results, s, cal_rows):
    a, ip, cal, cp, pt = (s["association"], s["interpolation"], s["calibration"],
                          s["compensation"], s["point_times"])
    L = []
    L.append(f"# Boreas checks: {s['slice']}\n")
    L.append(f"{s['scans']} LiDAR scans, {s['images']} camera images.\n")
    cc = s["conventions"]
    L.append("## Conventions\n")
    L.append(f"- lidar_poses vs gps_post_process composed with T_applanix_lidar: position "
             f"max {cc['lidar_poses_vs_gps_composed_T_applanix_lidar']['position_error_m']['max']:.2e} m, "
             f"rotation max {cc['lidar_poses_vs_gps_composed_T_applanix_lidar']['rotation_error_deg']['max']:.2e} deg "
             f"(transposed angle convention: median "
             f"{cc['lidar_poses_transposed_convention_rotation_error_deg']['median']:.1f} deg)")
    k = "camera_poses_vs_lidar_traj_composed_inv_T_camera_lidar"
    L.append(f"- camera_poses vs that trajectory composed with T_camera_lidar^-1: position max "
             f"{cc[k]['position_error_m']['max']:.2e} m, rotation max "
             f"{cc[k]['rotation_error_deg']['max']:.2e} deg\n")
    L.append("## Time association (item 3)\n")
    L.append(f"tau = {a['tau_ns'] / 1e6:.4f} ms. Accepted {a['accepted']} of {a['scans']} "
             f"({100 * a['match_rate']:.1f}%), rejected {a['rejected']}. |delta_t|: median "
             f"{a['abs_delta_t_ms']['median']:.2f} ms, p95 {a['abs_delta_t_ms']['p95']:.2f} ms, max "
             f"{a['abs_delta_t_ms']['max']:.2f} ms. {a['distinct_images_used']} distinct images "
             f"used, {a['images_never_matched']} never matched.\n")
    L.append("## Pose interpolation (item 4)\n")
    L.append(f"{ip['kept_poses']} kept poses (10 Hz), {ip['held_out']} held out.\n")
    L.append("| | median | p95 | max |\n|---|---|---|---|")
    pe, re_ = ip["position_error_m"], ip["rotation_error_deg"]
    L.append(f"| position (mm) | {pe['median'] * 1e3:.3f} | {pe['p95'] * 1e3:.3f} | {pe['max'] * 1e3:.3f} |")
    L.append(f"| rotation (deg) | {re_['median']:.5f} | {re_['p95']:.5f} | {re_['max']:.5f} |\n")
    L.append("| bin | n | pos median mm | pos p95 mm | pos max mm | rot median deg | rot p95 deg | rot max deg |")
    L.append("|---|---|---|---|---|---|---|---|")
    for b in ip["bins"]:
        L.append(f"| {b[0]} [{b[1]:g}, {b[2]:g}) | {b[3]} | {b[4] * 1e3:.3f} | {b[5] * 1e3:.3f} | "
                 f"{b[6] * 1e3:.3f} | {b[7]:.5f} | {b[8]:.5f} | {b[9]:.5f} |")
    L.append("")
    L.append("## Calibration sweep (item 2)\n")
    L.append(f"{len(cal['frames'])} frames below 1 m/s (max {cal['max_speed_mps']:.3f} m/s), "
             f"compensation {cal['compensation']}. Mean edge distance with the given calibration: "
             f"{cal['given_mean_px']:.3f} px.\n")
    L.append("| axis | steps | mean px | given is minimum | parabola vertex |\n|---|---|---|---|---|")
    for ax, f in cal["axes"].items():
        L.append(f"| {ax} | {', '.join(f'{x:g}' for x in f['steps'])} {f['unit']} | "
                 f"{', '.join(f'{y:.3f}' for y in f['mean_px'])} | {f['given_is_minimum']} | "
                 f"{f['parabola_vertex']:.3g} {f['unit']} |")
    L.append("")
    L.append("## Scan motion compensation (item 5)\n")
    m = cp["moving_mean_px"]
    L.append(f"{cp['frames']} associated frames, {cp['compensated_by_engine']} compensated by the "
             f"engine. Moving frames ({cp['moving_frames']}): mean edge distance no compensation "
             f"{m['nocomp']:.3f} px, engine compensation {m['comp']:.3f} px, compensation from the "
             f"sector time {m['sector_time_comp']:.3f} px. Compensation improved "
             f"{100 * cp['moving_frac_improved_by_comp']:.1f}% of moving frames.\n")
    d = cp["delta_t_vs_sector_time"]
    L.append(f"delta_t vs the median per-point time of the camera-facing sector: correlation "
             f"{d['corr']:.4f}; |camera time - sector time| median "
             f"{d['camera_minus_sector_ms']['median']:.2f} ms, max "
             f"{d['camera_minus_sector_ms']['max']:.2f} ms (|delta_t| median "
             f"{d['abs_delta_t_ms']['median']:.2f} ms).\n")
    L.append("| bin | frames | nocomp px | comp px | median improvement px | improved | sector median improvement px | sector improved |")
    L.append("|---|---|---|---|---|---|---|---|")
    for b in cp["bins"]:
        L.append(f"| {b[0]} [{b[1]:g}, {b[2]:g}) | {b[3]} | {b[4]:.3f} | {b[5]:.3f} | {b[6]:+.3f} | "
                 f"{100 * b[7]:.0f}% | {b[8]:+.3f} | {100 * b[9]:.0f}% |")
    L.append("")
    L.append("## Per-point time spread (item 6)\n")
    for key in ("span_ms", "camera_span_ms", "span_displacement_m", "camera_span_displacement_m",
                "span_rotation_deg", "camera_span_rotation_deg"):
        v = pt[key]
        L.append(f"- {key}: median {v['median']:.4g}, p95 {v['p95']:.4g}, max {v['max']:.4g}")
    L.append(f"- correlation of the camera sector's median point time with delta_t: "
             f"{pt['camera_median_t_vs_delta_t_corr']:.4f}\n")
    L.append(f"Overlays: {', '.join(s['overlays']['files'])} ({s['overlays']['colours']}).\n")
    with open(os.path.join(results, "report.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(L))


if __name__ == "__main__":
    main()
