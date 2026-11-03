#!/usr/bin/env python3
"""Compare the engine's Celsius and renders with the original LAENTIEC CSVs.

    python3 -I compare.py --work /tmp/laentiec_work \
        --thermal-export build/default/apps/thermal_export --results /results/laentiec

Runs thermal_export on the sequence written by convert.py. thermal_export
reads each 16-bit centi-kelvin PNG through fusion_io, converts every pixel
with the camera config's linear encoding, writes the result as float64, and
renders it with render_display() using a per-image window from the 1st to the
99th percentile. This script then re-reads the original CSVs (frames.csv
names them) and compares, per pixel and per image:

    error = engine Celsius - CSV Celsius

It writes to --results:

    per_frame.csv     per frame: CSV and engine min/max/mean, max |error|,
                      window from numpy and from the engine, pixels below and
                      above the window, black and white display pixels
                      (engine, and expected from the CSV), pixels in the
                      bottom and top tenth of the window, flags and notes
    error_hist.csv    histogram of |error| over all pixels
    summary.json      totals
    report.md         the same as short tables
    render_*.png      engine renders for visual review: coldest and warmest
                      scene, narrowest and widest window, most saturated,
                      median, every flagged frame, and the two frames with
                      the most pixels squeezed into a tenth of the window

Renders are images derived from the dataset (CC BY 4.0): publish them only
with the attribution in report.md, and never add them to the repository.
"""

import argparse
import csv
import json
import os
import shutil
import subprocess
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from laentiec_common import HEIGHT, WIDTH, read_temperature_csv  # noqa: E402

P_LO, P_HI = 1.0, 99.0
# |error| histogram edges in degrees Celsius; exact zeros are counted apart.
EDGES = [0.0, 1e-15, 1e-14, 1e-13, 1e-12, 1e-9, 1e-6, 1e-3, 5e-3, 1e-2, np.inf]
# Flags for "clips or saturates unexpectedly".
MAX_SATURATED = 0.05   # fraction of pure black or pure white display pixels
MIN_WINDOW_C = 1.0     # window narrower than this is flagged
# Not a fault, but reported: a percentile window on a bimodal scene (cold sky,
# warm ground) squeezes one mode into the top or bottom tenth of the window,
# where the hot colour map is nearly white or nearly black.
MAX_TENTH = 0.25

ATTRIBUTION = (
    "Thermal data: RGBDT dataset, LAENTIEC experimental area, University of "
    "Málaga, Zenodo record 8032971 (https://zenodo.org/records/8032971), "
    "licensed CC BY 4.0 (https://creativecommons.org/licenses/by/4.0/). "
    "The renders are derived from it: converted to centi-kelvin and rendered "
    "with fusion-engine's window/level and hot colour map.")


def nearest_rank(sorted_values, p):
    n = sorted_values.size
    rank = int(np.ceil(p / 100.0 * n))
    return float(sorted_values[min(max(rank, 1), n) - 1])


def window_value(t, lo, hi):
    """window_level() of thermal.hpp for the window [lo, hi]."""
    width = max(hi - lo, 1e-6)
    low = 0.5 * (lo + hi) - width / 2.0
    return np.clip((t - low) / width, 0.0, 1.0)


def expected_black_white(t, lo, hi):
    """Pure-black and pure-white pixel counts of the hot colour map for
    temperatures t with the window [lo, hi], following thermal.hpp."""
    v = window_value(t, lo, hi)
    x = 3.0 * v
    ch = lambda a: np.floor(255.0 * np.clip(a, 0.0, 1.0) + 0.5)  # noqa: E731 (lround, a >= 0)
    r, g, b = ch(x), ch(x - 1.0), ch(x - 2.0)
    black = int(np.count_nonzero((r == 0) & (g == 0) & (b == 0)))
    white = int(np.count_nonzero((r == 255) & (g == 255) & (b == 255)))
    return black, white


def read_rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def fmt(x, digits=3):
    if isinstance(x, float):
        return f"{x:.{digits}g}"
    return str(x)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--work", required=True, help="convert.py output directory")
    ap.add_argument("--thermal-export", required=True, help="path to the thermal_export binary")
    ap.add_argument("--results", required=True, help="output directory for the report")
    args = ap.parse_args()

    manifest = os.path.join(args.work, "sequence", "sequence.yaml")
    engine_dir = os.path.join(args.work, "engine")
    r = subprocess.run([args.thermal_export, manifest, "--out", engine_dir,
                        "--percentiles", f"{P_LO:g},{P_HI:g}"], capture_output=True, text=True)
    if r.returncode != 0:
        sys.exit(f"thermal_export failed ({r.returncode}):\n{r.stdout}{r.stderr}")
    print(r.stdout.strip())

    frames = read_rows(os.path.join(args.work, "frames.csv"))
    engine_stats = read_rows(os.path.join(engine_dir, "thermal_stats.csv"))
    if len(engine_stats) != len(frames):
        sys.exit(f"thermal_export processed {len(engine_stats)} frames, expected {len(frames)}")

    os.makedirs(args.results, exist_ok=True)
    hist = np.zeros(len(EDGES) - 1, dtype=np.int64)
    zeros = 0
    signed_min, signed_max = np.inf, -np.inf
    total = 0
    rows = []
    for fr, es in zip(frames, engine_stats):
        j = int(fr["frame"])
        assert int(es["frame"]) == j and int(es["timestamp_ns"]) == int(fr["timestamp_ns"])
        t = read_temperature_csv(fr["csv"])
        eng = np.fromfile(os.path.join(engine_dir, f"celsius_{j:06d}.f64"), dtype="<f8")
        if eng.size != WIDTH * HEIGHT:
            sys.exit(f"frame {j}: engine wrote {eng.size} values")
        eng = eng.reshape(HEIGHT, WIDTH)
        if not np.all(np.isfinite(eng)):
            sys.exit(f"frame {j}: engine has non-convertible pixels")
        err = eng - t
        a = np.abs(err).ravel()
        zeros += int(np.count_nonzero(a == 0.0))
        hist += np.histogram(a[a > 0.0], bins=EDGES)[0]
        signed_min = min(signed_min, float(err.min()))
        signed_max = max(signed_max, float(err.max()))
        total += a.size

        s = np.sort(t.ravel())
        lo, hi = nearest_rank(s, P_LO), nearest_rank(s, P_HI)
        e_lo, e_hi = float(es["window_lo_c"]), float(es["window_hi_c"])
        below = int(np.count_nonzero(t < lo))
        above = int(np.count_nonzero(t > hi))
        exp_black, exp_white = expected_black_white(t, lo, hi)
        v = window_value(t, lo, hi)
        bottom_tenth = int(np.count_nonzero(v <= 0.1))
        top_tenth = int(np.count_nonzero(v >= 0.9))
        n = t.size
        row = {
            "frame": j,
            "timestamp_ns": int(fr["timestamp_ns"]),
            "csv_min_c": float(t.min()), "csv_max_c": float(t.max()),
            "csv_mean_c": float(t.mean()),
            "engine_min_c": float(es["min_c"]), "engine_max_c": float(es["max_c"]),
            "engine_mean_c": float(es["mean_c"]),
            "max_abs_error_c": float(a.max()),
            "window_lo_c": lo, "window_hi_c": hi,
            "engine_window_lo_c": e_lo, "engine_window_hi_c": e_hi,
            "below_window": below, "above_window": above,
            "engine_below_window": int(es["below_window"]),
            "engine_above_window": int(es["above_window"]),
            "display_black": int(es["display_black"]),
            "display_white": int(es["display_white"]),
            "expected_black": exp_black, "expected_white": exp_white,
            "bottom_tenth": bottom_tenth, "top_tenth": top_tenth,
            "convertible": int(es["convertible"]),
        }
        flags = []
        if row["convertible"] != n:
            flags.append("unconvertible")
        if hi - lo < MIN_WINDOW_C:
            flags.append("narrow_window")
        if below > 0.01 * n or above > 0.01 * n:
            flags.append("clip_over_1pct")
        if row["display_black"] > MAX_SATURATED * n:
            flags.append("black_over_5pct")
        if row["display_white"] > MAX_SATURATED * n:
            flags.append("white_over_5pct")
        if (row["display_black"], row["display_white"]) != (exp_black, exp_white):
            flags.append("render_mismatch")
        if abs(e_lo - lo) > 0.005 or abs(e_hi - hi) > 0.005:
            flags.append("window_mismatch")
        row["flags"] = ";".join(flags)
        notes = []
        if bottom_tenth > MAX_TENTH * n:
            notes.append("bottom_tenth_over_25pct")
        if top_tenth > MAX_TENTH * n:
            notes.append("top_tenth_over_25pct")
        row["notes"] = ";".join(notes)
        rows.append(row)
        print(f"\rcompared {len(rows)}/{len(frames)}", end="", file=sys.stderr)
    print(file=sys.stderr)

    with open(os.path.join(args.results, "per_frame.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for row in rows:
            w.writerow({k: (repr(v) if isinstance(v, float) else v) for k, v in row.items()})
    with open(os.path.join(args.results, "error_hist.csv"), "w") as f:
        f.write("abs_error_from_c,abs_error_to_c,pixels\n")
        f.write(f"0,0,{zeros}\n")
        for k in range(len(EDGES) - 1):
            f.write(f"{EDGES[k]!r},{EDGES[k + 1]!r},{int(hist[k])}\n")

    def col(name):
        return np.array([row[name] for row in rows], dtype=np.float64)

    n_px = WIDTH * HEIGHT
    flagged, noted = {}, {}
    for row in rows:
        for fl in filter(None, row["flags"].split(";")):
            flagged.setdefault(fl, []).append(row["frame"])
        for nt in filter(None, row["notes"].split(";")):
            noted.setdefault(nt, []).append(row["frame"])
    summary = {
        "frames": len(rows),
        "pixels": total,
        "temperature_min_c": float(col("csv_min_c").min()),
        "temperature_max_c": float(col("csv_max_c").max()),
        "max_abs_error_c": float(col("max_abs_error_c").max()),
        "signed_error_range_c": [signed_min, signed_max],
        "exact_zero_error_pixels": zeros,
        "max_abs_diff_min_c": float(np.abs(col("engine_min_c") - col("csv_min_c")).max()),
        "max_abs_diff_max_c": float(np.abs(col("engine_max_c") - col("csv_max_c")).max()),
        "max_abs_diff_mean_c": float(np.abs(col("engine_mean_c") - col("csv_mean_c")).max()),
        "max_abs_diff_window_lo_c": float(
            np.abs(col("engine_window_lo_c") - col("window_lo_c")).max()),
        "max_abs_diff_window_hi_c": float(
            np.abs(col("engine_window_hi_c") - col("window_hi_c")).max()),
        "window_width_c": {"min": float((col("window_hi_c") - col("window_lo_c")).min()),
                           "median": float(np.median(col("window_hi_c") - col("window_lo_c"))),
                           "max": float((col("window_hi_c") - col("window_lo_c")).max())},
        "below_window_fraction_max": float(col("below_window").max() / n_px),
        "above_window_fraction_max": float(col("above_window").max() / n_px),
        "display_black_fraction": {"median": float(np.median(col("display_black")) / n_px),
                                   "max": float(col("display_black").max() / n_px)},
        "display_white_fraction": {"median": float(np.median(col("display_white")) / n_px),
                                   "max": float(col("display_white").max() / n_px)},
        "engine_below_equals_numpy": bool(np.all(col("engine_below_window") ==
                                                 col("below_window"))),
        "engine_above_equals_numpy": bool(np.all(col("engine_above_window") ==
                                                 col("above_window"))),
        "bottom_tenth_fraction": {"median": float(np.median(col("bottom_tenth")) / n_px),
                                  "max": float(col("bottom_tenth").max() / n_px)},
        "top_tenth_fraction": {"median": float(np.median(col("top_tenth")) / n_px),
                               "max": float(col("top_tenth").max() / n_px)},
        "flagged": {k: v for k, v in sorted(flagged.items())},
        "noted": {k: v for k, v in sorted(noted.items())},
        "error_hist": {"zero": zeros,
                       "bins": [[EDGES[k], EDGES[k + 1], int(hist[k])]
                                for k in range(len(EDGES) - 1)]},
    }

    # A few renders for visual review: coldest and warmest scene, narrowest
    # and widest window, most saturated, and the median-mean frame.
    by = lambda key: sorted(rows, key=key)  # noqa: E731
    picks = []
    for cand in (by(lambda r: r["csv_mean_c"])[0], by(lambda r: r["csv_mean_c"])[-1],
                 by(lambda r: r["window_hi_c"] - r["window_lo_c"])[0],
                 by(lambda r: r["window_hi_c"] - r["window_lo_c"])[-1],
                 by(lambda r: r["display_black"] + r["display_white"])[-1],
                 by(lambda r: r["csv_mean_c"])[len(rows) // 2]):
        if cand["frame"] not in [p["frame"] for p in picks]:
            picks.append(cand)
    # Flagged frames are always kept; then the frames with the most pixels in
    # the top or bottom tenth of the window.
    for row in [r for r in rows if r["flags"]]:
        if row["frame"] not in [p["frame"] for p in picks]:
            picks.append(row)
    squeezed = by(lambda r: -max(r["top_tenth"], r["bottom_tenth"]))
    for row in [r for r in squeezed if r["notes"]][:2]:
        if row["frame"] not in [p["frame"] for p in picks]:
            picks.append(row)
    kept = []
    for p in picks:
        name = f"render_{p['frame']:06d}_{p['timestamp_ns'] // 1_000_000}.png"
        shutil.copyfile(os.path.join(engine_dir, f"render_{p['frame']:06d}.png"),
                        os.path.join(args.results, name))
        kept.append({"file": name, "frame": p["frame"],
                     "window_c": [p["window_lo_c"], p["window_hi_c"]],
                     "min_c": p["csv_min_c"], "max_c": p["csv_max_c"],
                     "mean_c": p["csv_mean_c"],
                     "black": p["display_black"], "white": p["display_white"],
                     "bottom_tenth": p["bottom_tenth"], "top_tenth": p["top_tenth"],
                     "flags": p["flags"], "notes": p["notes"]})
    summary["renders"] = kept

    with open(os.path.join(args.results, "summary.json"), "w") as f:
        json.dump(summary, f, indent=1)
    write_report(os.path.join(args.results, "report.md"), summary, n_px)
    print(json.dumps({k: summary[k] for k in (
        "frames", "pixels", "temperature_min_c", "temperature_max_c", "max_abs_error_c",
        "max_abs_diff_min_c", "max_abs_diff_max_c", "max_abs_diff_mean_c", "flagged",
        "bottom_tenth_fraction", "top_tenth_fraction")},
        indent=1))


def write_report(path, s, n_px):
    L = []
    L.append("# LAENTIEC radiometric thermal check\n")
    L.append(ATTRIBUTION + "\n")
    L.append(f"- Frames: {s['frames']}, pixels: {s['pixels']:,}")
    L.append(f"- Temperatures: {s['temperature_min_c']:.1f} to {s['temperature_max_c']:.1f} C")
    L.append(f"- Max |engine - CSV|: {fmt(s['max_abs_error_c'])} C "
             f"(signed {fmt(s['signed_error_range_c'][0])} .. "
             f"{fmt(s['signed_error_range_c'][1])}); exactly zero for "
             f"{s['exact_zero_error_pixels']:,} pixels")
    L.append(f"- Per-image min / max / mean, max difference: {fmt(s['max_abs_diff_min_c'])} / "
             f"{fmt(s['max_abs_diff_max_c'])} / {fmt(s['max_abs_diff_mean_c'])} C")
    L.append(f"- Window (1st-99th percentile) width: min {s['window_width_c']['min']:.1f}, "
             f"median {s['window_width_c']['median']:.1f}, max "
             f"{s['window_width_c']['max']:.1f} C; engine vs numpy window max difference "
             f"{fmt(s['max_abs_diff_window_lo_c'])} / {fmt(s['max_abs_diff_window_hi_c'])} C")
    L.append(f"- Below / above the window, worst image: "
             f"{100 * s['below_window_fraction_max']:.2f}% / "
             f"{100 * s['above_window_fraction_max']:.2f}%")
    L.append(f"- Pure black display pixels: median "
             f"{100 * s['display_black_fraction']['median']:.2f}%, max "
             f"{100 * s['display_black_fraction']['max']:.2f}%; pure white: median "
             f"{100 * s['display_white_fraction']['median']:.2f}%, max "
             f"{100 * s['display_white_fraction']['max']:.2f}%")
    L.append(f"- Pixels in the bottom tenth of the window: median "
             f"{100 * s['bottom_tenth_fraction']['median']:.1f}%, max "
             f"{100 * s['bottom_tenth_fraction']['max']:.1f}%; top tenth: median "
             f"{100 * s['top_tenth_fraction']['median']:.1f}%, max "
             f"{100 * s['top_tenth_fraction']['max']:.1f}%")
    flagged = s["flagged"]
    L.append(f"- Flagged frames: {', '.join(f'{k}: {v}' for k, v in flagged.items()) or 'none'}")
    for k, v in s["noted"].items():
        L.append(f"- Note {k}: {len(v)} of {s['frames']} frames (listed in summary.json)")
    L.append("\n## |error| histogram\n")
    L.append("| from (C) | to (C) | pixels |")
    L.append("|---|---|---|")
    L.append(f"| 0 | 0 | {s['error_hist']['zero']:,} |")
    for lo, hi, c in s["error_hist"]["bins"]:
        L.append(f"| {lo:g} | {hi:g} | {c:,} |")
    L.append("\n## Renders\n")
    L.append("| file | window (C) | min / mean / max (C) | black | white | bottom / top tenth "
             "| flags, notes |")
    L.append("|---|---|---|---|---|---|---|")
    for r in s["renders"]:
        L.append(f"| {r['file']} | {r['window_c'][0]:.1f} .. {r['window_c'][1]:.1f} | "
                 f"{r['min_c']:.1f} / {r['mean_c']:.1f} / {r['max_c']:.1f} | "
                 f"{100 * r['black'] / n_px:.2f}% | {100 * r['white'] / n_px:.2f}% | "
                 f"{100 * r['bottom_tenth'] / n_px:.1f}% / {100 * r['top_tenth'] / n_px:.1f}% | "
                 f"{';'.join(filter(None, [r['flags'], r['notes']])) or '-'} |")
    with open(path, "w") as f:
        f.write("\n".join(L) + "\n")


if __name__ == "__main__":
    main()
