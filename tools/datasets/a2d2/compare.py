#!/usr/bin/env python3
"""Run fuse_cli on the converted A2D2 sequences and compare with A2D2.

    python3 compare.py --work /tmp/a2d2_work --fuse-cli build/default/apps/fuse_cli \\
        --results /results/a2d2

--work is the directory written by convert.py. fuse_cli runs on the direct
sequence and on each per-LiDAR chain sequence; its per-point CSVs keep the
scan's point order, so every engine point is matched to its A2D2 reference
row by index (through `chain_index` for the chain scans).

Writes to --results (statistics only, no points or images):
    per_frame_lidar.csv   per variant, frame and lidar_id (and "all")
    chain_vs_direct.csv   per frame and lidar_id: chain minus direct
    summary.json          totals, convention check, latency
    report.md             short human-readable report
"""

import argparse
import csv
import json
import math
import os
import subprocess
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from a2d2_common import LIDAR_NAMES, load_json  # noqa: E402

VARIANTS = ("direct", "chain")


# --- reading -------------------------------------------------------------------

def read_engine_csv(path):
    """Columns of fuse_cli's frame CSV: u, v, depth (NaN when not visible),
    nearest (bool), the float32 point (x, y, z) in the scan's frame."""
    u, v, d, near, xyz = [], [], [], [], []
    with open(path, newline="", encoding="utf-8") as f:
        r = csv.reader(f)
        head = next(r)
        iu, iv, idp, inr = (head.index(c) for c in ("rgb_u", "rgb_v", "rgb_depth", "rgb_nearest"))
        for row in r:
            xyz.append((float(row[1]), float(row[2]), float(row[3])))
            if row[iu] == "":
                u.append(math.nan)
                v.append(math.nan)
                d.append(math.nan)
                near.append(False)
            else:
                u.append(float(row[iu]))
                v.append(float(row[iv]))
                d.append(float(row[idp]))
                near.append(row[inr] == "1")
    return (np.array(u), np.array(v), np.array(d), np.array(near, dtype=bool),
            np.array(xyz, dtype=float).reshape(-1, 3))


def read_reference(path):
    a = np.genfromtxt(path, delimiter=",", names=True)
    a = np.atleast_1d(a)
    return {k: a[k] for k in a.dtype.names}


# --- engine model --------------------------------------------------------------

def zbuffer_nearest(u, v, depth, width, height):
    """The engine's z-buffer rule applied to given pixels: per pixel the
    smallest depth wins, the lowest point index on a tie. NaN = not visible."""
    vis = np.flatnonzero(~np.isnan(u))
    col = np.floor(u[vis] + 0.5).astype(np.int64)
    row = np.floor(v[vis] + 0.5).astype(np.int64)
    pix = row * width + col
    order = np.lexsort((vis, depth[vis], pix))
    first = np.ones(len(order), dtype=bool)
    first[1:] = pix[order][1:] != pix[order][:-1]
    out = np.zeros(len(u), dtype=bool)
    out[vis[order[first]]] = True
    return out


def project(K, p):
    with np.errstate(divide="ignore", invalid="ignore"):
        uv = (p @ K.T)[:, :2] / p[:, 2:3]
    return uv[:, 0], uv[:, 1], p[:, 2]


# --- statistics ----------------------------------------------------------------

def stats(ref, eng, sel, width, height):
    """Engine vs A2D2 over the points in `sel`."""
    inside = ((ref["col"] >= -0.5) & (ref["col"] < width - 0.5) &
              (ref["row"] >= -0.5) & (ref["row"] < height - 0.5) & (ref["depth"] > 0))
    vis = ~np.isnan(eng["u"])
    both = sel & vis & inside
    px = np.hypot(eng["u"][both] - ref["col"][both], eng["v"][both] - ref["row"][both])
    dz = np.abs(eng["depth"][both] - ref["depth"][both])
    s = {
        "count": int(sel.sum()),
        "a2d2_inside": int((sel & inside).sum()),
        "engine_visible": int((sel & vis).sum()),
        "compared": int(both.sum()),
        "not_visible_but_a2d2_inside": int((sel & ~vis & inside).sum()),
        "visible_but_a2d2_outside": int((sel & vis & ~inside).sum()),
        "zbuffer_hidden": int((sel & vis & ~eng["nearest"]).sum()),
    }
    if len(px):
        s.update({
            "px_err_median": float(np.median(px)),
            "px_err_max": float(px.max()),
            "frac_within_0p5px": float(np.mean(px <= 0.5)),
            "depth_err_median_m": float(np.median(dz)),
            "depth_err_max_m": float(dz.max()),
        })
    else:
        s.update({k: math.nan for k in ("px_err_median", "px_err_max", "frac_within_0p5px",
                                        "depth_err_median_m", "depth_err_max_m")})
    if "merged_nearest" in eng:
        s["zbuffer_hidden_merged"] = int((sel & vis & ~eng["merged_nearest"]).sum())
    return s, px, dz


STAT_COLS = ["count", "a2d2_inside", "engine_visible", "compared", "not_visible_but_a2d2_inside",
             "visible_but_a2d2_outside", "zbuffer_hidden", "zbuffer_hidden_merged",
             "px_err_median", "px_err_max", "frac_within_0p5px", "depth_err_median_m",
             "depth_err_max_m"]


def fmt(x):
    if isinstance(x, float):
        return "" if math.isnan(x) else repr(x)
    return str(x)


# --- main ----------------------------------------------------------------------

def run_fuse_cli(fuse_cli, manifest, out_dir):
    r = subprocess.run([fuse_cli, manifest, "--out", out_dir], capture_output=True, text=True)
    if r.returncode != 0:
        sys.exit(f"compare.py: fuse_cli failed on {manifest}:\n{r.stdout}{r.stderr}")
    return load_json(os.path.join(out_dir, "summary.json"))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--work", required=True, help="directory written by convert.py")
    ap.add_argument("--fuse-cli", required=True, help="path to the fuse_cli executable")
    ap.add_argument("--results", required=True, help="output directory for statistics")
    ap.add_argument("--run-dir", help="fuse_cli output directory (default <work>/run)")
    args = ap.parse_args()

    work = os.path.abspath(args.work)
    run_dir = os.path.abspath(args.run_dir or os.path.join(work, "run"))
    os.makedirs(args.results, exist_ok=True)

    # Run the engine.
    latency = {"direct": run_fuse_cli(args.fuse_cli, os.path.join(work, "direct/sequence.yaml"),
                                      os.path.join(run_dir, "direct"))}
    for n in LIDAR_NAMES:
        latency[f"chain/{n}"] = run_fuse_cli(
            args.fuse_cli, os.path.join(work, "chain", n, "sequence.yaml"),
            os.path.join(run_dir, "chain", n))

    tf = load_json(os.path.join(work, "transforms.json"))
    conv = load_json(os.path.join(work, "convention_check.json"))
    width, height = conv["image_size"]
    # Intrinsics as the engine read them (identical in every config).
    K = None
    with open(os.path.join(work, "direct/rgb.yaml"), encoding="utf-8") as f:
        for line in f:
            if line.strip().startswith("K:"):
                K = np.array(json.loads(line.split(":", 1)[1]), dtype=float)
    T_cl = {n: np.array(tf["T_camera_lidar"][n]) for n in LIDAR_NAMES}

    with open(os.path.join(work, "frames.csv"), encoding="utf-8") as f:
        frames = [int(r["frame"]) for r in csv.DictReader(f)]

    groups = [(str(k), LIDAR_NAMES[k], k) for k in range(len(LIDAR_NAMES))] + [("all", "all", None)]
    per_rows, cvd_rows = [], []
    pooled = {(var, g[0]): {"px": [], "dz": [], "s": []} for var in VARIANTS for g in groups}
    model_err = {var: 0.0 for var in VARIANTS}
    cvd_tot = {"px_max": 0.0, "depth_max_m": 0.0, "visibility_mismatch": 0,
               "nearest_mismatch_direct_vs_merged": 0}

    for i in frames:
        name = f"{i:06d}"
        ref = read_reference(os.path.join(work, "reference", name + ".csv"))
        lid = ref["lidar_id"].astype(int)
        cidx = ref["chain_index"].astype(int)
        n_pts = len(lid)

        eng = {}
        u, v, d, near, xyz = read_engine_csv(os.path.join(run_dir, "direct", f"frame_{name}.csv"))
        eng["direct"] = {"u": u, "v": v, "depth": d, "nearest": near}
        mu, mv, md = project(K, xyz)  # float64 model of the engine on the float32 points
        vis = ~np.isnan(u)
        if vis.any():
            model_err["direct"] = max(model_err["direct"], float(np.max(
                np.hypot(u[vis] - mu[vis], v[vis] - mv[vis]))))

        cu, cv, cd = (np.full(n_pts, np.nan) for _ in range(3))
        cn = np.zeros(n_pts, dtype=bool)
        for k, n in enumerate(LIDAR_NAMES):
            sel = np.flatnonzero(lid == k)
            path = os.path.join(run_dir, "chain", n, f"frame_{name}.csv")
            u, v, d, near, xyz = read_engine_csv(path)
            if len(u) != len(sel) or not np.array_equal(cidx[sel], np.arange(len(sel))):
                sys.exit(f"compare.py: {path}: {len(u)} points, reference has {len(sel)}")
            cu[sel], cv[sel], cd[sel], cn[sel] = u, v, d, near
            pc = xyz @ T_cl[n][:3, :3].T + T_cl[n][:3, 3]
            mu, mv, _ = project(K, pc)
            vis = ~np.isnan(u)
            if vis.any():
                model_err["chain"] = max(model_err["chain"], float(np.max(
                    np.hypot(u[vis] - mu[vis], v[vis] - mv[vis]))))
        eng["chain"] = {"u": cu, "v": cv, "depth": cd, "nearest": cn,
                        "merged_nearest": zbuffer_nearest(cu, cv, cd, width, height)}

        for var in VARIANTS:
            for gid, gname, k in groups:
                sel = np.ones(n_pts, dtype=bool) if k is None else (lid == k)
                s, px, dz = stats(ref, eng[var], sel, width, height)
                per_rows.append([var, i, gid, gname] + [fmt(s.get(c, math.nan)) for c in STAT_COLS])
                p = pooled[(var, gid)]
                p["px"].append(px)
                p["dz"].append(dz)
                p["s"].append(s)

        # Chain minus direct.
        dvis = ~np.isnan(eng["direct"]["u"])
        cvis = ~np.isnan(cu)
        for gid, gname, k in groups:
            sel = np.ones(n_pts, dtype=bool) if k is None else (lid == k)
            both = sel & dvis & cvis
            dpx = np.hypot(cu[both] - eng["direct"]["u"][both], cv[both] - eng["direct"]["v"][both])
            ddz = np.abs(cd[both] - eng["direct"]["depth"][both])
            vm = int((sel & (dvis != cvis)).sum())
            nm = int((sel & (eng["direct"]["nearest"] != eng["chain"]["merged_nearest"])).sum())
            row = [i, gid, gname, int(both.sum()),
                   fmt(float(dpx.max()) if len(dpx) else math.nan),
                   fmt(float(np.median(dpx)) if len(dpx) else math.nan),
                   fmt(float(ddz.max()) if len(ddz) else math.nan), vm, nm]
            cvd_rows.append(row)
            if k is None:
                cvd_tot["px_max"] = max(cvd_tot["px_max"], float(dpx.max()) if len(dpx) else 0.0)
                cvd_tot["depth_max_m"] = max(cvd_tot["depth_max_m"],
                                             float(ddz.max()) if len(ddz) else 0.0)
                cvd_tot["visibility_mismatch"] += vm
                cvd_tot["nearest_mismatch_direct_vs_merged"] += nm

    with open(os.path.join(args.results, "per_frame_lidar.csv"), "w", newline="",
              encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["variant", "frame", "lidar_id", "lidar"] + STAT_COLS)
        w.writerows(per_rows)
    with open(os.path.join(args.results, "chain_vs_direct.csv"), "w", newline="",
              encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["frame", "lidar_id", "lidar", "compared", "px_diff_max", "px_diff_median",
                    "depth_diff_max_m", "visibility_mismatch", "nearest_mismatch_direct_vs_merged"])
        w.writerows(cvd_rows)

    # Totals over all frames.
    totals = {}
    for var in VARIANTS:
        totals[var] = {}
        for gid, gname, _ in groups:
            p = pooled[(var, gid)]
            px = np.concatenate(p["px"])
            dz = np.concatenate(p["dz"])
            t = {c: sum(s[c] for s in p["s"]) for c in STAT_COLS
                 if c in p["s"][0] and not c.startswith(("px_", "frac_", "depth_"))}
            if len(px):
                t.update({"px_err_median": float(np.median(px)), "px_err_max": float(px.max()),
                          "frac_within_0p5px": float(np.mean(px <= 0.5)),
                          "depth_err_median_m": float(np.median(dz)),
                          "depth_err_max_m": float(dz.max())})
            totals[var][gname] = t

    summary = {
        "dataset": "A2D2 (Audi Autonomous Driving Dataset), CC BY-ND 4.0, statistics only",
        "frames": len(frames),
        "image_size": [width, height],
        "totals": totals,
        "chain_vs_direct": cvd_tot,
        "engine_vs_float64_model_of_float32_points_px_max": model_err,
        "convention_check": conv,
        "fuse_cli_process_ms_median": {k: v["process_ms"]["median"] for k, v in latency.items()},
    }
    with open(os.path.join(args.results, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=1)
    write_report(os.path.join(args.results, "report.md"), summary)
    print(f"compare.py: {len(frames)} frames -> {args.results}")
    for var in VARIANTS:
        a = totals[var]["all"]
        print(f"  {var:6s}: {a['compared']} of {a['count']} points compared, median "
              f"{a['px_err_median']:.3g} px, max {a['px_err_max']:.3g} px, "
              f"{100 * a['frac_within_0p5px']:.3f}% within 0.5 px, depth max "
              f"{a['depth_err_max_m']:.3g} m")
    print(f"  chain vs direct: max {cvd_tot['px_max']:.3g} px, {cvd_tot['depth_max_m']:.3g} m")


def write_report(path, s):
    L = []
    L.append("# A2D2 check: projection and the multi-LiDAR chain\n")
    L.append(f"Data: A2D2 (Audi Autonomous Driving Dataset), CC BY-ND 4.0. {s['frames']} frames "
             f"of cam_front_center, image {s['image_size'][0]}x{s['image_size'][1]}. "
             "Statistics only; no points or images are reproduced.\n")
    L.append("Pixel error: distance between the engine's (u, v) and A2D2's (col, row). "
             "Depth error: |engine depth - A2D2 depth|. Over points visible to the engine and "
             "inside the image according to A2D2.\n")
    for var in VARIANTS:
        L.append(f"## {var}\n")
        L.append("| lidar | points | compared | median px | max px | within 0.5 px | "
                 "median depth m | max depth m | not visible, A2D2 inside | "
                 "visible, A2D2 outside | z-buffer hidden |")
        L.append("|---|---|---|---|---|---|---|---|---|---|---|")
        for name, t in s["totals"][var].items():
            hidden = str(t["zbuffer_hidden"])
            if "zbuffer_hidden_merged" in t:
                hidden += f" per LiDAR / {t['zbuffer_hidden_merged']} merged"
            L.append(f"| {name} | {t['count']} | {t['compared']} | {t['px_err_median']:.3g} | "
                     f"{t['px_err_max']:.3g} | {100 * t['frac_within_0p5px']:.4f}% | "
                     f"{t['depth_err_median_m']:.3g} | {t['depth_err_max_m']:.3g} | "
                     f"{t['not_visible_but_a2d2_inside']} | {t['visible_but_a2d2_outside']} | "
                     f"{hidden} |")
        L.append("")
    c = s["chain_vs_direct"]
    m = s["engine_vs_float64_model_of_float32_points_px_max"]
    L.append("## Chain vs direct\n")
    L.append(f"- Largest pixel difference over all points: {c['px_max']:.3g} px; largest depth "
             f"difference: {c['depth_max_m']:.3g} m.")
    L.append(f"- Points visible in one variant only: {c['visibility_mismatch']}.")
    L.append(f"- Points whose z-buffer flag differs between the direct run and the chain runs "
             f"merged by the engine's rule: {c['nearest_mismatch_direct_vs_merged']}.")
    L.append(f"- Engine vs a float64 projection of the same float32 points: max "
             f"{m['direct']:.3g} px (direct), {m['chain']:.3g} px (chain). The remaining error "
             "against A2D2 is the float32 storage of the points in the sequence format.\n")
    L.append("## Conventions\n")
    cc = s["convention_check"]
    L.append(f"- A2D2's stored col/row vs a CamMatrix projection of Q = (-y, -z, x): median "
             f"{cc['stored_projection_vs_CamMatrix_px']['median']:.3g} px, max "
             f"{cc['stored_projection_vs_CamMatrix_px']['max']:.3g} px.")
    L.append("- View convention (axes as columns, p_vehicle = R p + origin) vs LiDAR azimuth "
             "attribute, median residual in degrees (axes as rows in brackets):")
    for n in LIDAR_NAMES:
        a, b = cc["axes_as_columns"][n], cc["axes_as_rows"][n]
        if a.get("points"):
            L.append(f"  - {n}: {a['azimuth_residual_median_deg']:.3f} "
                     f"({b['azimuth_residual_median_deg']:.1f})")
    L.append("")
    L.append("## Notes\n")
    L.append("- A2D2 keeps every point, including several landing on one pixel. The engine "
             "projects all of them too, but only the nearest per pixel enters the depth map "
             "(`nearest`); \"z-buffer hidden\" counts the others. They still have a projection "
             "and are compared.")
    L.append("- In the chain variant each LiDAR is its own sequence, so the engine's z-buffer "
             "runs per LiDAR. \"merged\" applies the same rule to the engine's chain output "
             "across all LiDARs.")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(L) + "\n")


if __name__ == "__main__":
    main()
