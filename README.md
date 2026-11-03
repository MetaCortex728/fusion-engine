# fusion-engine

fusion-engine is a small C++17 library and command-line tool that aligns
LiDAR, RGB camera and thermal camera data to one reference time and one pixel
grid. For each LiDAR scan it picks the matching camera images by timestamp,
moves the scan to each camera's capture time along the vehicle trajectory,
projects the points into the distorted pinhole cameras with a z-buffer, reads
the thermal value under each point and converts it to °C, and returns one
aligned frame. It needs only the C++ standard library, Eigen and yaml-cpp (no
ROS, no GUI toolkit). It is for developers of perception pipelines on embedded
platforms who want multi-sensor alignment without a large dependency tree.
It has been checked against synthetic ground truth and on three real datasets
(A2D2, LAENTIEC and Boreas; see [Validation](#validation)).

> **Status: early-stage.** No release yet, no stable API.

![The fuse_cli web viewer on the synthetic sequence: the RGB image of scan 2
with LiDAR points coloured by their thermal-camera temperature, 18 °C road
and a 45 °C box, and the side panel with association, point counts and
latency](docs/images/viewer-synthetic.png)

*The `fuse_cli --viewer` page for scan 2 of the synthetic sequence (rendered
test scene, no dataset imagery), RGB view with points coloured by the
temperature the thermal camera reads for them: the road at 18 °C, the warm
box ahead at 45 °C, grey where the thermal camera has no value. Captured
with headless Chromium.*

## Pipeline at a glance

```
 sequence on disk (docs/sequence-format.md)
   LiDAR scans · RGB frames · thermal frames (16-bit raw) · trajectory
        │
        ▼
 1. time association      per camera: nearest image to each scan time,
                           accepted if |Δt| <= tau, with per-sensor clock offsets
        ▼
 2. motion compensation   scan moved from t_lidar to t_camera along the
                           interpolated trajectory (scan-level, when covered)
        ▼
 3. projection            distorted pinhole camera, z-buffer per pixel
        ▼
 4. thermal lookup        raw value under each point, °C (Planck or linear)
        ▼
 AlignedFrame             per point and camera: pixel, depth, visibility,
                           raw thermal value and °C; per camera: Δt, accepted,
                           compensated
        ▼
 fuse_cli outputs         per-point CSV, summary with latency, overlays,
                           web viewer
```

## Quick start

No dataset is needed. The test build includes a generator for a synthetic
five-scan sequence (LiDAR, RGB, thermal, trajectory) rendered from the test
scene.

With Docker (Ubuntu 22.04 toolchain, see `Dockerfile` and `compose.yaml`):

```sh
docker compose build
docker compose run --rm dev
# inside the container, in /work:
cmake --preset default && cmake --build --preset default && ctest --preset default
build/default/tests/make_synthetic_sequence out/synthetic
build/default/apps/fuse_cli out/synthetic/sequence.yaml --out out/fused --viewer
```

Then open `out/fused/viewer/index.html` in any browser on the host (the
repository, including `out/`, is mounted into the container). The page works
from `file://`; it needs no server and makes no network requests.

Without Docker you need CMake 3.21 or newer, Ninja, GCC or Clang with C++17,
Eigen 3.4 and yaml-cpp. On Ubuntu:

```sh
sudo apt-get install cmake ninja-build g++ clang libeigen3-dev libyaml-cpp-dev
```

and then the same commands as inside the container.

## The web viewer

`fuse_cli ... --viewer` writes `viewer/index.html` into the output directory:
a static page with inline CSS and JavaScript and no external libraries. The
run's data except the per-point arrays is embedded in the page as JSON
(`<script type="application/json" id="fusion-data">`; nanosecond timestamps
are strings there, because epoch nanoseconds do not fit exactly in a
JavaScript number). The points of the k-th frame are in
`points/frame_NNNNNN.js` (NNNNNN = k), a script calling
`fusionPoints(k, {"rgb": …, "thermal": …})`; the page loads it with a
`<script>` element when it shows the frame (and the frames either side), so
opening a long run does not parse every frame's points, and there is no
`fetch`, which `file://` pages cannot use. The camera images it
shows are written next to it: `rgb_NNNNNN.png` is the RGB frame and
`thermal_NNNNNN.png` the thermal frame rendered in °C with the hot colour
map, both numbered by the camera's image index. Keep the `viewer/`
directory together when copying it.

- **Per frame:** the RGB or rendered thermal image with the projected LiDAR
  points drawn at their pixel, coloured by depth (red near, blue far) or by
  the point's thermal value from the thermal camera (grey where it has none).
- **Controls:** frame slider and previous/next (arrow keys and Home/End work
  too), RGB/thermal, colour by depth or thermal, point size, colour range
  (min/max, with "Auto" for the run's range), opacity, and whether to show
  points hidden by the z-buffer (drawn as outlines).
- **Side panel:** per camera the association `Δt`, accepted and compensated
  flags and the number of projected points (hidden and embedded counts on
  hover); the scan's point count and processing, read and write times; the
  run's median and p95 processing and read latency.
- **Layout:** desktop and narrow widths, light and dark (follows the
  system setting).
- **Large runs:** at most 20000 points per camera and frame are embedded
  (`--viewer-max-points N` changes the cap). When a camera sees m > N
  points, they are taken in scan order and the k-th kept point
  (k = 0 … N − 1) is the one at rank `floor(k · m / N)`. The selection
  depends only on m and N, so a run always embeds the same points. The page
  states the cap and marks frames that were subsampled. The page is written
  frame by frame, so memory use does not grow with the run length.
- **Size:** about 34–36 bytes per embedded point. At the default cap a
  frame with 20000 points in each of two cameras is a 1.4 MB points file,
  so 100 such frames are about 140 MB on disk; the browser reads only the
  page (about 0.5 KB per frame plus 20 KB) and the points files of the shown
  frame and its neighbours. In headless Chromium the synthetic run's page
  loads in about 15–30 ms and a 1.4 MB frame is loaded and drawn in about
  80 ms.

The tests run `fuse_cli --viewer` on the synthetic sequence, parse the
embedded JSON and the points files and compare every frame, flag, count and point with the CSV
outputs. The page itself has been loaded in headless Chromium (light, dark,
390 px wide) with no script errors and no network requests.

## Validation

| Check | Data | Headline result | Page |
|-------|------|-----------------|------|
| Synthetic ground truth | test scene, moving rig, five-scan sequence (unit tests) | After compensation 964 of 994 projected points (all 493 box points) match ray-cast depth to 1e-9 relative, against 0 of 493 box points before. In every accepted image of the sequence ≥ 96% of points match to 1e-9 relative depth. | [below](#synthetic-test-scene) |
| A2D2 | 50 frames, 866,142 points, 5 LiDARs | Engine pixel equals A2D2's to ≤ 1.1e-4 px and depth to ≤ 3.8e-6 m (float32 storage). The five-LiDAR extrinsic chain agrees with the direct projection to 1.9e-4 px. | [a2d2.md](docs/validation/a2d2.md) |
| RGBDT LAENTIEC | 312 thermal frames, 95.8 M pixels | Linear (centi-kelvin) thermal encoding reproduces every pixel to ≤ 3e-6 °C over −24.8 °C to 29.8 °C. Rendering matches an independent reference. | [laentiec.md](docs/validation/laentiec.md) |
| Boreas | 600 scans, 62 s, moving vehicle | All 600 scans associated (median \|Δt\| 24.5 ms). Pose interpolation error median 0.83 mm / 0.019°. Projection with the given calibration is 5.04 px from image edges. Scan-level compensation runs but does not help (10.63 → 10.67 px), because the camera fires as the LiDAR sweeps past it. | [boreas.md](docs/validation/boreas.md) |

Each page says what the check proves and what it does not. No dataset files
or images derived from them are in this repository.

## Using the library

| Header                   | Provides |
|--------------------------|----------|
| `fusion/config.hpp`      | `load_sensor_config()` / `parse_sensor_config()` read a per-sensor YAML file into `SensorConfig`: name, resolution, intrinsics `K`, distortion `[k1, k2, p1, p2, k3]`, extrinsics `T_<sensor>_<source>`, and `time_offset_ns`. Thermal cameras add an optional `thermal` section (`encoding: planck` with `R`, `B`, `F`, `O`, or `encoding: linear` with `scale` and `offset`; see [thermal encoding](docs/sequence-format.md#thermal-encoding)). Problems throw `ConfigError` naming the file and field. |
| `fusion/geometry.hpp`    | `CameraModel`: pinhole with radial-tangential distortion, `distort()` / `undistort()`, `project()`. `project_point(camera, T_camera_lidar, p_lidar)` returns the sub-pixel position, pixel index and z-depth. Points behind the camera, outside the image or beyond the radius where the distortion stops being monotonic are rejected. |
| `fusion/depth_map.hpp`   | `SparseDepthMap` keeping the nearest depth per pixel, and `project_to_depth_map()`. |
| `fusion/time.hpp`        | Integer nanoseconds (`TimeNs`). `associate()` pairs each reference time with the other sensor's nearest measurement (earlier one on a tie), accepted if `|t_j - t_ref| <= tau`, with `delta_t_ns = t_match_ns - t_ref_ns`. `default_tau_ns()` is half the slower sensor's period. |
| `fusion/pose.hpp`        | `Trajectory` of `StampedPose`s (`T_world_body`) with linear position and shortest-arc slerp interpolation. No extrapolation. |
| `fusion/lidar.hpp`       | Scan-level motion compensation: `compensate_scan()` moves a scan from `t_l` to `t_c` with `p_L(t_c) = T_body_lidar^-1 · T_world_body(t_c)^-1 · T_world_body(t_l) · T_body_lidar · p_L`. |
| `fusion/aligned_frame.hpp` | `build_aligned_frame()`: association, compensation and projection for one scan into each camera, with per-point pixel, depth, z-buffer flag and, for thermal, the raw value and °C. `write_aligned_frame_csv()` writes it. |
| `fusion/thermal.hpp`     | `raw_to_celsius()` (Planck or linear), `window_level()`, `hot_colormap()`, `render_display()`, `project_to_thermal()`. |
| `fusion/sequence.hpp`    | `fusion_io` library: reads and writes the [sequence format](docs/sequence-format.md) with errors that name the file and line. |
| `fusion/image_io.hpp`    | `fusion_io` library: 8-bit RGB and 16-bit thermal PNG via LodePNG; raw thermal values round-trip bit-exactly. |

Conventions: transforms are named `T_target_source` (maps points from
`source` into `target`). Time is integer nanoseconds. Pixel centres sit at
integer coordinates, so the image spans `[-0.5, width - 0.5) x [-0.5,
height - 0.5)`. Depth is z in the camera's optical frame (x right, y down,
z forward). [`configs/`](configs/) holds annotated example configs for the
synthetic cameras.

```cpp
#include <fusion/aligned_frame.hpp>
#include <fusion/config.hpp>

const fusion::SensorConfig rgb = fusion::load_sensor_config("configs/rgb_camera.yaml");
fusion::FrameInputs in;
in.scan = &scan;                         // fusion::LidarScan at t_l
in.trajectory = &trajectory;             // optional, T_world_body
in.T_body_lidar = T_body_lidar;
in.rgb = fusion::CameraInput{&rgb, association, nullptr};  // from fusion::associate()
const fusion::AlignedFrame frame = fusion::build_aligned_frame(in);
for (const auto& p : frame.rgb->points) {
    if (p.projection) { /* p.projection->pixel, p.projection->depth, p.nearest */ }
}
```

## Tools

### `fuse_cli`

```sh
fuse_cli <sequence.yaml> --out <dir> [--frames N] [--overlay] [--viewer] [--viewer-max-points N]
```

| Output | Contents |
|--------|----------|
| `frame_NNNNNN.csv` | one row per scan point: `point,x,y,z,intensity`, then per camera `u,v,depth,nearest` (thermal adds `raw` and `celsius`); empty when the point is not visible or the image was rejected |
| `summary.csv` | one row per frame: per camera the matched image index and time, `delta_t_ns`, `accepted`, `compensated`, projected points; then `read_ms`, `process_ms`, `write_ms` |
| `summary.json` | median and p95 (nearest rank) of `process_ms`, `read_ms` and `write_ms` |
| `association_rgb.csv`, `association_thermal.csv` | the time association per camera |
| `overlay_rgb_NNNNNN.png`, `overlay_thermal_NNNNNN.png` | with `--overlay`: depth points (red near, blue far) over the RGB image and the rendered thermal image |
| `viewer/index.html`, `viewer/points/` and the images | with `--viewer`: the [web viewer](#the-web-viewer) |

`process_ms` is measured with `std::chrono::steady_clock` around
`build_aligned_frame()` only (compensation, projection, z-buffer), not file
reading or writing. With `--viewer` the RGB frames are read too (counted in
`read_ms`) and the page and its images are written (counted in
`write_ms`). On the synthetic sequence (about 6,000 points per scan) a
`default` build took about 1 ms per frame on one development machine; that
is an example run, not a benchmark. Exit codes: 0 success, 1 a sequence or
processing error (the message names the file), 2 bad arguments.

### `thermal_export` and `fusion_probe`

`thermal_export` converts every thermal frame of a sequence to °C
(float64) and renders it with a per-image percentile window; the
[LAENTIEC check](docs/validation/laentiec.md) uses it. `fusion_probe`
exposes trajectory interpolation and the RGB reader to validation scripts;
the [Boreas check](docs/validation/boreas.md) uses it.

```sh
build/default/apps/thermal_export out/synthetic/sequence.yaml --out out/thermal
build/default/apps/fusion_probe interpolate trajectory.csv --times times.csv --out poses.csv
build/default/apps/fusion_probe gray out/synthetic/sequence.yaml --out out/gray --indices 0,5
```

Dataset converters for the validation checks live in
[`tools/datasets/`](tools/datasets/) (Python 3 with numpy, outside the C++
libraries).

## Sequence format

Adapters convert datasets into one on-disk format: a YAML manifest, camera
configs, `timestamp_ns,path` list files, float32 LiDAR `.bin` scans, PNG
images (16-bit raw for thermal) and an optional trajectory CSV. See
[docs/sequence-format.md](docs/sequence-format.md).

## Build and test

```sh
cmake --preset default && cmake --build --preset default && ctest --preset default
```

| Preset    | Compiler | Notes                                   |
|-----------|----------|-----------------------------------------|
| `default` | GCC      | RelWithDebInfo                          |
| `clang`   | Clang    | RelWithDebInfo                          |
| `asan`    | GCC      | Debug with AddressSanitizer and UBSan   |

`ci-default`, `ci-clang` and `ci-asan` do the same with warnings as errors.
CI runs all three on x64, and `ci-default` on arm64 for pull requests. The ASan presets use GCC
because Clang 14's ASan runtime crashes at startup in about one run in four
under newer kernels with larger address-space randomization (seen on WSL2
with a 6.18 kernel).

The `dev` container also mounts a dataset folder read-only at `/data`
(default `./data`, or set `FUSION_DATA_DIR`) and `./out` at `/out`.
Datasets are never committed.

### Synthetic test scene

`tests/support/` holds a deterministic scene used as ground truth: boxes on a
ground plane with a raw thermal value per surface, virtual RGB and thermal
pinhole cameras, and a virtual spinning LiDAR. A moving rig drives at 8 m/s
with a 0.6 rad/s yaw rate; the camera captures 33 ms after the scan. After
compensation all 493 projected box points and 964 of 994 projected points
overall agree with the ray-cast depth to 1e-9 relative (the rest are
occluded from the camera), with a median error below 1e-15 m. Without it,
none of the box points agree (median error 0.26 m).

`tests/support/synthetic_sequence.hpp` turns the rig into a sequence: five
10 Hz scans, 20 Hz RGB, 10 Hz thermal with one dropped frame, clock offsets,
and a trajectory every 40 ms. Scan 0 has no RGB image and scan 3 no thermal
image within `tau`, so both rejection paths run. Each thermal surface has
an apparent temperature (road 18 °C, walls and blocks 12–26 °C, a warm
vehicle-sized box 45 °C, sky −15 °C) converted to a raw count with the
inverse of the camera's Planck model, so the thermal camera reads them back
to within 0.01 °C. The Planck constants (R 380747, B 1428, F 1,
O −88.539) are illustrative, not a specific camera's calibration, but fit an
uncooled LWIR camera: B corresponds to a 10.1 µm effective wavelength and
there are about 49 counts per kelvin at 20 °C. The tests check that the
readers return the generator's values exactly, that association matches a
brute-force search, that at least 96% of the projected points in each
accepted image match ray-cast depth (1e-9 relative in memory, 1e-5 from the
float32 files), that every malformed input gives its own error, and that
`fuse_cli` and its viewer data agree with the per-frame CSVs.

## Design rules

- The core library (`fusion`) depends only on the C++17 standard library,
  Eigen and yaml-cpp, found on the system. `fusion_io` adds the vendored
  LodePNG for PNG. No ROS, GUI or dataset dependencies.
- No x86-only intrinsics; the code builds on arm64.
- Time is integer nanoseconds; transforms are `T_target_source`.
- Tests use synthetic data only.

## Licence and third-party notices

Licensed under the Apache License, Version 2.0; see [LICENSE](LICENSE).
Vendored code (LodePNG, zlib licence; doctest, MIT, tests only) is listed in
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md). The datasets used for
validation keep their own licences; their pages give the attribution, and
none of their data is redistributed here.

## Status

Milestones M0 to M5 are complete: projection, thermal, time and pose, aligned
frames with the sequence format and `fuse_cli`, three real-data checks, and the
web viewer. CI builds and tests every push on x64 (GCC, Clang, and GCC with
AddressSanitizer and UndefinedBehaviorSanitizer) and on arm64 (GCC). Planned work is in [ROADMAP.md](ROADMAP.md).

## Ownership and contributors

This is an independent personal project by Han
([MetaCortex728](https://github.com/MetaCortex728)). It is not owned by, or
developed for, any employer, client, university or other organization, and no
one else currently works on the repository.

## Project history

Developed from November 2026. Earlier design notes are not published. The
git history records each step: projection, thermal, time and pose, aligned
frames and the sequence format, the three real-data checks, and the viewer.
The work was planned and reviewed by Han with Anthropic's Claude as a coding
assistant.

## Roadmap

Planned work, in priority order, is in [ROADMAP.md](ROADMAP.md). Next up is
per-point deskew, which the Boreas check showed is needed.
