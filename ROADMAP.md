# Roadmap

Planned work in priority order. Estimates are rough: focused working time
including tests and documentation, not calendar time. Nothing here is
implemented yet.

## 1. Per-point deskew, validated on Boreas (about 6 h)

**Goal.** Move each LiDAR point by its own capture time within the sweep,
not the whole scan by one time. The [Boreas check](docs/validation/boreas.md)
showed this is needed: one sweep spans about 104 ms, up to 0.95 m and 4.4°
of vehicle motion. Scan-level compensation did not improve alignment there
(10.63 → 10.67 px), because the camera fires as the LiDAR sweeps past it.

**Done when.**
- The sequence format has an optional per-point time (a new format version
  or a sibling file), read and validated by `fusion_io`, with format
  version 1 still read unchanged.
- `build_aligned_frame()` deskews each point to the camera's time when
  per-point times and a trajectory are present, and says so in the output.
- Synthetic ground truth: a rotating-LiDAR sweep on the moving rig, where
  deskewed points match ray-cast depth to the same tolerance as the
  existing compensation test.
- The Boreas converter writes per-point times, and the Boreas page reports
  the edge distance on the 536 moving frames with deskew next to the
  existing columns, whatever the result.

## 2. Several LiDARs in one sequence and one `AlignedFrame` (about 4 h)

**Goal.** Fuse all LiDARs of a rig in the engine. Today the format holds one
LiDAR per sequence, so the [A2D2 check](docs/validation/a2d2.md) composes
`T_camera_vehicle · T_vehicle_lidar` in the converter, runs one sequence per
LiDAR and merges the z-buffers in Python.

**Done when.**
- The manifest lists several LiDARs, each with its list file, time offset
  and `T_body_lidar`; cameras give extrinsics to the body or to a named
  LiDAR.
- One `AlignedFrame` holds points from every LiDAR with a LiDAR id, and the
  z-buffer runs across all of them.
- Synthetic test with two LiDARs on the rig; the A2D2 check runs as one
  sequence and reproduces today's merged visibility and `nearest` flags.

## 3. ZOD adapter and validation (about 4 h, once access is granted)

**Goal.** A fourth real-data check on the Zenseact Open Dataset, once
Zenseact grants access. Its licence is CC BY-SA, which needs attribution.

**Done when.**
- `tools/datasets/zod/` converts a sequence into the sequence format, checks
  the dataset's conventions numerically before writing, and keeps all data
  outside the repository.
- `docs/validation/zod.md` states what it proves and what not, with the
  CC BY-SA attribution, and the README table gets a row.

## 4. 3D→thermal projection with real calibration on HEARTH (about 4 h, once released)

**Goal.** Projection into a thermal camera with a real LiDAR–thermal
calibration. LAENTIEC checked the thermal encoding but had no calibration,
so projection into a real thermal camera is still unchecked.

**Done when.**
- A converter for HEARTH (once it is released, and subject to its licence)
  and a check of projected depth discontinuities against thermal image
  edges, with the same honesty about what the metric can show as the Boreas
  calibration sweep.
- Planck conversion is exercised on real raw counts if the dataset provides
  them.

## 5. Calibration check tool (about 3 h)

**Goal.** Turn the edge-alignment sweep from the Boreas check into a
reusable command.

**Done when.**
- A `fusion_calib_check` (or `fuse_cli` subcommand) takes a sequence, a set
  of frames and perturbation steps, and writes a report: the edge-distance
  metric for the given calibration and each rotation and translation step,
  with parabola vertices per axis.
- It reproduces the Boreas sweep numbers from the C++ tool, and a synthetic
  test with a known calibration error finds that error.

## 6. Interactive desktop viewer (ImGui) with live re-projection (about 6 h)

**Goal.** Complement the static web viewer with a desktop viewer that
re-projects live while the user changes a time offset or an extrinsic, to
see misalignment directly.

**Done when.**
- A separate optional target (ImGui, outside the core and `fusion_io`, off
  by default) loads a sequence, shows the projection over each camera
  image, and has sliders for time offset and extrinsic perturbation that
  re-run `build_aligned_frame()` on change.
- The core library gains no GUI dependency, and CI builds the core without
  it.

## 7. Performance: parallel projection and a Boreas benchmark (about 3 h)

**Goal.** Measured, repeatable latency numbers and a faster projection on
multi-core embedded targets.

**Done when.**
- Projection and the z-buffer run in parallel across points (standard
  library threads only, deterministic output identical to the serial
  path).
- A benchmark on the Boreas slice reports median and p95 `process_ms` for
  serial and parallel runs, with the machine and build stated, on the
  Boreas page or a new benchmark page.
