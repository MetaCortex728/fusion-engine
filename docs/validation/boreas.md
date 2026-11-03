# Real-data check 3: real calibration, time association, pose interpolation and motion compensation on Boreas

This check runs the engine on a moving vehicle with real timing. It covers
the parts that A2D2 and LAENTIEC could not: projection with a calibration
the engine did not derive itself, association of LiDAR scans with camera
images by timestamp, interpolation of a real trajectory, and scan-level
motion compensation. It also measures how far apart the per-point times
within a scan are, to size the per-point deskew item on the roadmap.

Tooling: [`tools/datasets/boreas/`](../../tools/datasets/boreas/). It is
Python 3 with numpy only, pinned in `requirements.txt`, and it sits outside
the C++ libraries. All engine results come from `fuse_cli` and from
`fusion_probe` (`apps/fusion_probe/`), a small tool that exposes
`Trajectory::interpolate` and the engine's RGB reader to scripts.

## The slice

`boreas-2021-03-02-13-38`, LiDAR files `1614710324826904.bin` to
`1614710386946045.bin` (600 scans, 62 s) and the 622 camera images in the
same interval. The Applanix CSVs cover the whole drive and were used whole.
The vehicle moves at 6.4 m/s on average (max 9.2 m/s), with yaw rates up
to 0.74 rad/s. It is below 1 m/s for the first 64 scans.

## Conventions, confirmed against the data

`convert.py` checks each convention numerically before it writes anything.

- **Pose angles and frames.** `gps_post_process.csv` (200 Hz, Applanix
  frame) composed with `T_applanix_lidar` and interpolated to each scan time
  equals `lidar_poses.csv` to within 3.8e-5 m and 2.6e-3° (600 scans). With
  the transposed rotation convention the error is 85°.
- **`T_camera_lidar` direction.** The LiDAR trajectory composed with
  `T_camera_lidar⁻¹` equals `camera_poses.csv` to within 2.7e-5 m and
  1.4e-3° (622 images). Read as `T_lidar_camera` it would be 0.91 m and 163°
  off.
- **File names are the pose times.** Every LiDAR and camera file time
  appears in `lidar_poses.csv` and `camera_poses.csv` respectively.
- **Per-point time `t`.** It is in seconds relative to the file time and
  spans one rotation centred on it: median minimum −51.76 ms, median
  maximum +51.76 ms. **The LiDAR file time is the middle of the sweep**,
  not its start.
- **Projection.** `P_camera` has a zero fourth column, so `K = P_camera[:, :3]`
  with no distortion (the images are rectified).

The trajectory in the sequence is `T_world_lidar` (body = LiDAR frame) at
200 Hz in a local ENU frame, `timestamp_ns,x,y,z,qw,qx,qy,qz`.

## Results

Run on 2026-11-03 with a `default` (GCC, RelWithDebInfo) build over the
whole slice. Full numbers are in `/results/boreas/report.md` and
`summary.json`, outside the repository.

### Time association

`tau_ns` = 51,879,500 (half the median LiDAR period of 103.76 ms).

| scans | accepted | rejected | \|Δt\| median | p95 | max |
|-------|----------|----------|---------------|-----|-----|
| 600 | 600 (100%) | 0 | 24.51 ms | 46.79 ms | 49.98 ms |

Each scan got a different image; 22 images were never matched (the camera
runs slightly faster than 10 Hz).

### Pose interpolation

The 200 Hz poses are thinned to every 20th (10 Hz, 642 poses).
`fusion_probe interpolate` evaluates the engine's `Trajectory` at the 12,179
held-out times, and the result is compared with the held-out poses.

| | median | p95 | max |
|---|---|---|---|
| position | 0.83 mm | 3.05 mm | 5.17 mm |
| rotation | 0.019° | 0.098° | 0.49° |

Position error grows with yaw rate (median 0.57 mm below 0.1 rad/s,
2.5 mm above 0.5 rad/s); the largest rotation errors occur at 6–9 m/s.
Even the worst position error is about an order of magnitude below the
2–4 cm RMS accuracy of the post-processed solution itself, so 10 Hz
interpolation adds no error that matters next to the reference.

### Projection with the given calibration

Metric: LiDAR depth-discontinuity points (the near side of a range jump
of at least max(1 m, 10%) between neighbouring returns of one laser) are
projected by the engine. Image edges are non-maximum-suppressed Sobel maxima
above the 90th gradient percentile of the smoothed luminance. The metric is
the mean distance from projected points to the nearest edge pixel, each
capped at 50 px. It uses the 64 scans below 1 m/s (max 0.94 m/s), without
compensation.

The given calibration scores 5.04 px. Rotations of ±0.25°, ±0.5°, ±1° and
translations of ±5 cm, ±10 cm about each axis:

| axis | −1° / −10 cm | −0.5° / −5 cm | −0.25° | given | +0.25° | +0.5° / +5 cm | +1° / +10 cm |
|------|------|------|------|------|------|------|------|
| rot x | 5.17 | 5.02 | 5.02 | 5.04 | 5.25 | 5.46 | 5.92 |
| rot y | 7.17 | 6.62 | 6.01 | 5.04 | **4.75** | 5.54 | 5.97 |
| rot z | 5.15 | 5.07 | 5.05 | 5.04 | 5.07 | 5.10 | 5.14 |
| trans x | 6.51 | 5.85 | | 5.04 | | **4.59** | 5.03 |
| trans y | 5.29 | 5.16 | | 5.04 | | 5.00 | 4.96 |
| trans z | 5.52 | 5.24 | | 5.04 | | 4.92 | 4.87 |

The given calibration is a local minimum of this metric only for rotation
about z (vertex −0.02°). Per axis:

- **rot y, trans x:** clear U-shapes with the minimum about 0.3° and 5 cm
  from the given value (parabola vertices +0.31°, +0.057 m), 0.3–0.4 px
  lower than the given calibration.
- **rot x:** flat for negative steps (5.02 px at −0.25° and −0.5°), rising
  for positive ones; vertex −0.43°.
- **trans y, trans z:** decrease monotonically across the sweep (by 0.08
  and 0.17 px at +10 cm); the minimum, if there is one, lies beyond 10 cm.

The rotations are within about 0.5° of this metric's optimum, and the
largest pixel gains from any perturbation are below 0.5 px against a 5 px
baseline. But the check does not show the given calibration to be the
optimum, and trans y and z are not bracketed. Whether the offsets are in
the calibration or a bias of the metric (edges and depth discontinuities
need not coincide exactly, and taking the near side of each jump shifts
points one way) is not settled here.

### Scan-level motion compensation: why it does not help on Boreas

For every associated pair, `fuse_cli` projects the same discontinuity
points without a trajectory and with it (scan moved from the LiDAR time to
the camera time). All 600 scans were compensated by the engine.

| moving frames (> 1 m/s) | no compensation | engine compensation | compensation from the sector time |
|---|---|---|---|
| 536 | 10.63 px | 10.67 px | 10.62 px |

Engine compensation improves 48% of the moving frames, i.e. it is
neutral to slightly worse, at every `|Δt| × speed` and yaw rate bin
(median change between −0.26 and +0.29 px). This contradicts the
expectation, and the cause is in the rig's timing, not in the engine:

- The camera is triggered when the LiDAR sweep passes the camera. The
  median per-point time of the camera-facing discontinuity points
  correlates with `Δt` at 0.80 (0.98 for all camera-facing points), and
  the camera time is a median 4.2 ms from that sector time, while
  `|Δt|` to the scan time is a median 24.5 ms.
- The sweep is not cut at a fixed azimuth: relative to the file time,
  the camera sector is swept anywhere from −40 ms to +35 ms (5th–95th
  percentile), and that offset follows `Δt`. So the points the camera sees
  were captured close to the camera's own time already, whatever `Δt` is.
  Moving the whole scan by `Δt` applies motion those points never went
  through.
- As a control, `compare.py` also runs the engine with each scan stamped
  at its camera sector's median point time. That compensation is almost
  a no-op (10.62 px), as expected when there is almost nothing to correct.

The scan-level compensation itself is exercised end to end: the engine
selects the trajectory poses, moves the scan and reprojects it. But this
data cannot show that it improves alignment, because the right reference
time for the visible points is not the scan time. On a rig like this,
scan-level compensation should use the camera sector's time, or per-point
deskew.

### Per-point time spread (measurement only)

| | median | p95 | max |
|---|---|---|---|
| per-point time span, whole scan | 103.5 ms | 103.8 ms | 104.0 ms |
| span, points in the camera's view | 25.6 ms | 103.5 ms | 103.8 ms |
| vehicle displacement over the whole-scan span | 0.78 m | 0.93 m | 0.95 m |
| displacement over the camera-view span | 0.21 m | 0.88 m | 0.95 m |
| rotation over the whole-scan span | 0.28° | 3.19° | 4.39° |
| rotation over the camera-view span | 0.10° | 1.43° | 3.90° |

The camera-view span is usually about a quarter of the sweep, but reaches
the whole sweep when the sweep's start and end fall inside the camera's
view (29% of scans): the visible points then come from both ends of the
sweep, about 100 ms apart.
At these speeds one sweep is close to 1 m of travel and up to 4.4° of
heading, which is what per-point deskew would correct. Per-point deskew is
not implemented in this issue.

## What it proves

- The Boreas conventions above, against the dataset's own per-sensor poses,
  to micrometres and millidegrees.
- The engine's association rule on real, jittered timestamps: every scan
  gets one image within `tau`, none twice.
- `Trajectory::interpolate` on a real 200 Hz trajectory thinned to 10 Hz:
  millimetre position error, well below the reference's accuracy.
- The engine projects with a real `T_camera_lidar` and `P_camera` to within
  about 5 px of image edges (slow frames), and no perturbation in the sweep
  improves that by more than 0.5 px.
- Scan-level compensation runs end to end on every moving frame, and the
  measured reason why it does not improve alignment here.

## What it does not prove

- **Calibration accuracy.** The edge metric is a proxy. It does not find
  the given calibration optimal on every axis, and it cannot tell
  calibration error from metric bias.
- **That compensation improves alignment.** On this rig it cannot (see
  above). The pose arithmetic of compensation is checked on synthetic data
  in the unit tests.
- **Interpolation against ground truth.** The held-out poses come from the
  same smooth post-processed solution; the check measures interpolation
  error relative to it, not error relative to the true motion.
- **Lens distortion.** The images are rectified, so no distortion model is
  exercised.
- **Per-point deskew.** Measured, not implemented.

## Licence and attribution

Boreas (Burnett et al., "Boreas: A Multi-Season Autonomous Driving Dataset",
2022, University of Toronto, <https://www.boreas.utias.utoronto.ca>) is
listed under [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) in its
[AWS Open Data registry entry](https://registry.opendata.aws/boreas/). That
licence has not yet been confirmed on the project's own site, so the
overlay images stay in `/results/boreas/` and are not published. This
repository contains only code and the statistics above, no Boreas points or
images. The converted sequence and engine output contain Boreas data; keep
them outside the repository. The conventions were established with the help
of the pyboreas documentation (BSD-3-Clause,
<https://github.com/utiasASRL/pyboreas>); no pyboreas code is used.

## How to reproduce

**1. Data.** The sequence is in the public bucket
`s3://boreas/boreas-2021-03-02-13-38/` (no credentials needed). The check
used a slice that had already been extracted to `/data`; the commands below
were not run as part of it.

```sh
D=~/boreas/boreas-2021-03-02-13-38
aws s3 sync --no-sign-request s3://boreas/boreas-2021-03-02-13-38/applanix $D/applanix
aws s3 sync --no-sign-request s3://boreas/boreas-2021-03-02-13-38/calib $D/calib
# lidar/ and camera/: only files from 1614710324826904 to 1614710386946045 (µs)
for s in lidar camera; do
  aws s3 ls --no-sign-request s3://boreas/boreas-2021-03-02-13-38/$s/ |
    awk '{print $4}' | awk -F. '$1 >= 1614710324826904 && $1 <= 1614710386947043' |
    while read f; do aws s3 cp --no-sign-request \
      s3://boreas/boreas-2021-03-02-13-38/$s/$f $D/$s/$f; done
done
```

**2. Build** `fuse_cli` and `fusion_probe`:

```sh
cmake --preset default && cmake --build --preset default
```

**3. Convert and compare** (in a virtual environment outside the
repository):

```sh
python3 -m venv ~/boreas-venv
~/boreas-venv/bin/pip install -r tools/datasets/boreas/requirements.txt
~/boreas-venv/bin/python -I tools/datasets/boreas/convert.py \
    --data $D --out /tmp/boreas_work
~/boreas-venv/bin/python -I tools/datasets/boreas/compare.py \
    --data $D --work /tmp/boreas_work \
    --fuse-cli build/default/apps/fuse_cli --probe build/default/apps/fusion_probe \
    --results /results/boreas
```

`compare.py` writes `association.csv`, `interpolation.csv`,
`interpolation_bins.csv`, `calibration_sweep.csv`, `compensation.csv`,
`compensation_bins.csv`, `point_times.csv`, `summary.json`, `report.md` and
four overlays (red: no compensation, green: engine compensation, cyan:
compensation from the sector time).
