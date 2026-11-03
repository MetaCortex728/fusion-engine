# fusion-engine

> **Status: early-stage.** The library can load a per-sensor YAML config,
> project LiDAR points into a distorted pinhole camera, build a sparse depth
> map, convert thermal images to degrees Celsius (raw counts with Planck
> constants, or radiometric temperature with a linear scale such as
> centi-kelvin), render them for display,
> and project LiDAR points into a thermal camera. It can associate sensor
> timestamps, interpolate a body trajectory, and move a whole LiDAR scan to a
> camera's capture time. It builds an aligned frame per LiDAR scan, reads
> sequences in its own [on-disk format](docs/sequence-format.md), and
> `fuse_cli` runs a whole sequence. Besides synthetic data, its projection
> and multi-LiDAR extrinsics have been checked on real sensor geometry from
> A2D2 ([validation](docs/validation/a2d2.md)), and the linear thermal
> encoding and rendering on real temperature fields from RGBDT LAENTIEC
> ([validation](docs/validation/laentiec.md)). On a moving vehicle from
> Boreas ([validation](docs/validation/boreas.md)), time association,
> trajectory interpolation and projection with a real calibration work;
> scan-level compensation runs but does not improve alignment there,
> because the camera fires as the LiDAR sweeps past it. There is no release.

## What it does

fusion-engine is planned as a lightweight C++17 library that aligns LiDAR,
camera, and thermal measurements to one reference time and one pixel grid,
and outputs an aligned frame.

## Who it is for

Developers building perception pipelines on embedded platforms who need
multi-sensor alignment without pulling in ROS or a large dependency tree.

## Built with

- C++17
- [Eigen](https://eigen.tuxfamily.org/) for linear algebra
- [yaml-cpp](https://github.com/jbeder/yaml-cpp) for configuration files
- [LodePNG](https://github.com/lvandeve/lodepng) (zlib licence, vendored) for
  PNG images, in the `fusion_io` library only

No ROS dependency. See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) for
the vendored code.

## Design rules

- Dependencies stay minimal. The core library (`fusion`) needs only Eigen
  and yaml-cpp, both found on the system and never vendored. Eigen types
  appear in the public headers, and yaml-cpp is used only inside the config
  loader. The sequence I/O library (`fusion_io`) adds the vendored LodePNG
  for PNG images. Nothing else is vendored except doctest for the tests.
- No x86-only intrinsics, so the code stays portable to embedded targets.
- Time is represented as integer nanoseconds.
- Transforms are named `T_target_source`: the transform that maps a point
  from the `source` frame into the `target` frame.

## How to run

You need CMake 3.21 or newer, Ninja, GCC or Clang with C++17 support,
Eigen 3.4, and yaml-cpp. On Ubuntu:

```sh
sudo apt-get install cmake ninja-build g++ clang libeigen3-dev libyaml-cpp-dev
```

### What exists

| Header                   | Provides |
|--------------------------|----------|
| `fusion/config.hpp`      | `load_sensor_config()` / `parse_sensor_config()` read a per-sensor YAML file into `SensorConfig`: name, resolution, intrinsics `K`, distortion `[k1, k2, p1, p2, k3]`, extrinsics `T_<sensor>_<source>`, and `time_offset_ns`. Thermal cameras add an optional `thermal` section, which loads into `SensorConfig::thermal` (a `ThermalCalibration`). `encoding: planck` (the default) takes `thermal.planck` with the constants `R`, `B`, `F`, `O`. `encoding: linear` takes `thermal.linear` with `scale` and `offset` (see [thermal encoding](docs/sequence-format.md#thermal-encoding)). Problems throw `ConfigError` naming the file and field, e.g. a missing field, a wrong shape, or a rotation that is not invertible or not orthonormal. |
| `fusion/geometry.hpp`    | `CameraModel`: pinhole with radial-tangential distortion, `distort()` / `undistort()`, and `project()`. `project_point(camera, T_camera_lidar, p_lidar)` returns the sub-pixel position, pixel index, and z-depth. Only points with z > 0 that land inside the image are kept. Points beyond the radius where radial distortion stops being monotonic are also rejected, so they cannot fold back into the image. |
| `fusion/depth_map.hpp`   | `SparseDepthMap` over the camera's pixel grid, keeping the nearest depth per pixel, and `project_to_depth_map()` to fill one from a point cloud. |
| `fusion/time.hpp`        | Times are `int64_t` nanoseconds (`TimeNs`). `to_reference_time()` applies a sensor's `time_offset_ns` (`t_reference = t_sensor + time_offset_ns`). `associate()` pairs each reference time (the LiDAR's, by default) with the other sensor's nearest measurement, takes the earlier one on an exact tie, and accepts it only if `|t_j - t_ref| <= tau`. An overload takes sensor-clock stamps plus both offsets. Every reference time gets a result, rejected ones included, with `delta_t_ns = t_match_ns - t_ref_ns`. `default_tau_ns()` gives half the slower sensor's period. `write_association_csv()` writes `t_ref_ns,t_match_ns,delta_t_ns,accepted`. |
| `fusion/pose.hpp`        | `StampedPose` (time, `Eigen::Vector3d` position, `Eigen::Quaterniond` orientation) and `Trajectory`, which holds poses with strictly increasing times and rejects unsorted or duplicate times. `interpolate(t)` and `T_world_body(t)` use `alpha = (t - t_i) / (t_{i+1} - t_i)`, linear position, and slerp along the shortest arc (`slerp_shortest()`). Times outside the trajectory throw `std::out_of_range`; there is no extrapolation. |
| `fusion/lidar.hpp`       | Scan-level motion compensation. `compensate_scan()` moves a scan taken at `t_l` to a camera time `t_c`: `p_L(t_c) = T_body_lidar^-1 * T_world_body(t_c)^-1 * T_world_body(t_l) * T_body_lidar * p_L`. The points stay in the LiDAR frame, so projection with `T_camera_lidar` applies unchanged. `scan_motion_transform()` returns that transform. |
| `fusion/aligned_frame.hpp` | `build_aligned_frame()` builds the fused result for one LiDAR scan at `t_l`. For each camera input (RGB, thermal; each optional) it takes the `Association` for `t_l` and records its `delta_t_ns` and `accepted` flag. For an accepted camera, the scan is compensated to the camera's time when a trajectory covers both times (`compensated` says whether it was), then projected with `project_point()` into a `SparseDepthMap`. Per point and camera, the frame holds the pixel and depth, or "not visible", whether the point is the nearest at its pixel, and for thermal the raw count and its temperature in °C, converted with the camera's configured encoding. The frame copies everything it needs; inputs are only read. `write_aligned_frame_csv()` writes the per-point output. |
| `fusion/sequence.hpp`    | `fusion_io` library. Reads the [sequence format](docs/sequence-format.md): `load_sequence()` validates the manifest, camera configs, list files and trajectory, and checks that every listed file exists. It also provides readers for LiDAR `.bin` scans, RGB and thermal frames (with an image size check against the config) and the trajectory CSV, plus writers for adapters. `associate_camera()` and `camera_tau_ns()` associate a camera's frames with the scans. Problems throw `SequenceError` naming the file (and line). |
| `fusion/image_io.hpp`    | `fusion_io` library. 8-bit RGB PNG and 16-bit greyscale thermal PNG reading and writing via LodePNG. Raw thermal counts round-trip bit-exactly. |
| `fusion/thermal.hpp`     | `RawThermalImage` holds raw `uint16_t` values unchanged. `raw_to_celsius()` has three overloads. With `PlanckParams` it applies `T = B / ln(R / (S - O) + F) - 273.15` and throws `std::domain_error` when the result is undefined: S equal to O, a logarithm argument that is not positive or equals 1, or a result at or below absolute zero. With `LinearParams` it applies `T = raw * scale + offset` (scale 0.01 and offset -273.15 for centi-kelvin) and rejects a non-finite or zero scale and a non-finite offset. With a `ThermalCalibration` it uses whichever encoding the config selected. `window_level()` computes `clamp((T - (L - W / 2)) / W, 0, 1)` and rejects W <= 0. `hot_colormap()` maps that value to 8-bit RGB, black through red and yellow to white. `render_display()` builds a separate display image with Planck constants or a `ThermalCalibration`, and only reads the raw image. `project_to_thermal()` projects LiDAR points with `project_point()`. It returns the thermal `SparseDepthMap` and, for every projected point, its pixel, its depth, and the raw count at that pixel. |

Conventions: pixel centres sit at integer coordinates, so the image spans
`[-0.5, width - 0.5) x [-0.5, height - 0.5)`. Depth is the z coordinate in
the camera's optical frame (x right, y down, z forward).

[`configs/rgb_camera.yaml`](configs/rgb_camera.yaml) and
[`configs/thermal_camera.yaml`](configs/thermal_camera.yaml) are annotated
example configs. They describe the synthetic RGB and thermal cameras from the
test scene, not real hardware. The thermal config uses the example Planck
constants R = 380747, B = 1428, F = 1, O = -88.539.

```cpp
#include <fusion/config.hpp>
#include <fusion/depth_map.hpp>

const fusion::SensorConfig cfg = fusion::load_sensor_config("configs/rgb_camera.yaml");
const fusion::CameraModel camera = fusion::make_camera_model(cfg);
// cfg.source_frame == "lidar", so cfg.T_sensor_source is T_camera_lidar.
const fusion::SparseDepthMap depth =
    fusion::project_to_depth_map(camera, cfg.T_sensor_source, points_lidar);
for (const auto& e : depth.entries()) { /* e.col, e.row, e.depth */ }
```

```cpp
#include <fusion/config.hpp>
#include <fusion/thermal.hpp>

const fusion::SensorConfig th = fusion::load_sensor_config("configs/thermal_camera.yaml");
const fusion::PlanckParams& planck = th.thermal->planck;
const fusion::RawThermalImage raw(th.width, th.height, counts);  // counts: std::vector<uint16_t>
const double celsius = fusion::raw_to_celsius(raw.at(10, 20), planck);
const fusion::DisplayImage display =
    fusion::render_display(raw, planck, fusion::WindowLevel{/*level*/ 30.0, /*width*/ 20.0});
const fusion::ThermalProjection tp = fusion::project_to_thermal(
    fusion::make_camera_model(th), th.T_sensor_source, points_lidar, raw);
for (const auto& s : tp.samples) { /* s.point_index, s.projection, s.raw */ }
```

```cpp
#include <fusion/lidar.hpp>
#include <fusion/time.hpp>

// LiDAR stamps are the reference; the camera config's time_offset_ns moves
// camera stamps onto the reference clock.
const fusion::TimeNs tau = fusion::default_tau_ns(lidar_period_ns, camera_period_ns);
const auto matches = fusion::associate(lidar_stamps, lidar_offset_ns,
                                       camera_stamps, camera_cfg.time_offset_ns, tau);
fusion::write_association_csv("association.csv", matches);

const fusion::Trajectory trajectory(poses);  // std::vector<fusion::StampedPose>, T_world_body
for (const auto& m : matches) {
    if (!m.accepted) continue;
    const auto points_at_camera_time = fusion::compensate_scan(
        scan_points, trajectory, T_body_lidar, m.t_ref_ns, m.t_match_ns);
    // project points_at_camera_time with camera_cfg.T_sensor_source (T_camera_lidar)
}
```

### Run `fuse_cli` on the synthetic sequence

`fuse_cli` runs every LiDAR scan of a sequence through the engine:

```sh
fuse_cli <sequence.yaml> --out <dir> [--frames N] [--overlay]
```

No dataset is needed to try it. The test build includes
`make_synthetic_sequence`, which writes a five-scan sequence (LiDAR, RGB,
thermal, trajectory) rendered from the synthetic test scene:

```sh
cmake --preset default && cmake --build --preset default
build/default/tests/make_synthetic_sequence out/synthetic
build/default/apps/fuse_cli out/synthetic/sequence.yaml --out out/fused --overlay
```

It writes to `out/fused`:

| File | Contents |
|------|----------|
| `frame_NNNNNN.csv` | one row per scan point: `point,x,y,z,intensity`, then per camera `u,v,depth,nearest` (thermal adds `raw` and `celsius`; `celsius` is empty without a `thermal` section in the config); empty when the point is not visible or the image was rejected |
| `overlay_rgb_NNNNNN.png`, `overlay_thermal_NNNNNN.png` | with `--overlay`: depth points (red near, blue far) drawn over the RGB image and over the thermal image, rendered in °C with the configured encoding and the hot colormap |
| `summary.csv` | one row per frame: per camera the matched image index and time, `delta_t_ns`, `accepted`, `compensated`, number of projected points; then `read_ms`, `process_ms`, `write_ms` |
| `summary.json` | median and p95 of `process_ms`, `read_ms` and `write_ms` |
| `association_rgb.csv`, `association_thermal.csv` | the time association per camera (`write_association_csv()`) |

Processing time is measured with `std::chrono::steady_clock` around
`build_aligned_frame()` only. That covers compensation, projection and the
z-buffer, but not file reading (`read_ms`) or output writing (`write_ms`).
The median and p95 (nearest rank) are printed at the end. On the synthetic
sequence (about 6,000 points per scan) a `default` build took about 1 ms per
frame on one development machine. Treat that as an example run, not a
benchmark.

Exit codes: 0 on success, 1 for a sequence or processing error (the message
names the file), 2 for bad arguments.

### Export thermal frames with `thermal_export`

`thermal_export` converts every thermal frame of a sequence to °C with the
camera's configured encoding. For each frame it writes the values as
little-endian float64 (`celsius_NNNNNN.f64`) and a render with a per-image
percentile window (`render_NNNNNN.png`, 1st to 99th percentile by default).
`thermal_stats.csv` gets one row per frame: min, max, mean, the window,
pixels outside it, and pure black and white display pixels. The
[LAENTIEC check](docs/validation/laentiec.md) uses it.

```sh
build/default/apps/thermal_export out/synthetic/sequence.yaml --out out/thermal
```

### Query the engine with `fusion_probe`

`fusion_probe` exposes two engine functions to validation scripts:
`interpolate` evaluates a trajectory CSV at the nanosecond times listed in
a file, and `gray` writes the RGB frames of a sequence, read with the same
reader as `fuse_cli`, as raw 8-bit luminance. The
[Boreas check](docs/validation/boreas.md) uses it.

```sh
build/default/apps/fusion_probe interpolate trajectory.csv --times times.csv --out poses.csv
build/default/apps/fusion_probe gray out/synthetic/sequence.yaml --out out/gray --indices 0,5
```

### Build and test

Each preset builds into `build/<preset>`:

```sh
cmake --preset default && cmake --build --preset default && ctest --preset default
```

| Preset    | Compiler | Notes                                   |
|-----------|----------|-----------------------------------------|
| `default` | GCC      | RelWithDebInfo                          |
| `clang`   | Clang    | RelWithDebInfo                          |
| `asan`    | GCC      | Debug with AddressSanitizer and UBSan   |

The `ci-default`, `ci-clang`, and `ci-asan` presets do the same but treat
warnings as errors.

The ASan presets use GCC because Clang 14's ASan runtime crashes at startup
in about one run in four under newer kernels with larger address-space
randomization (seen on WSL2 with a 6.18 kernel), while GCC 11's does not.

### Synthetic test scene

`tests/support/` holds a deterministic synthetic scene used as ground truth
by the tests: axis-aligned boxes on a ground plane, two virtual pinhole
cameras (RGB and thermal) at different poses, and a virtual spinning LiDAR.
Each surface (the ground and each box) carries a raw thermal value, and the
virtual thermal camera renders a synthetic raw image from them.
Any randomness takes an explicit seed. This code is test-only and is not
part of the library's API. It uses Eigen types (`Eigen::Vector3d`,
`Eigen::Isometry3d`) for points and transforms. The tests project scene
points from the virtual LiDAR into the virtual cameras and compare the result
against ray-cast depth. For the thermal camera they also check that each
LiDAR point reads its surface's raw value. Points within 2 pixels of a
surface boundary in the rendered image are left out of that check.

The scene also has a moving rig (`make_moving_rig()`): a body moving at a
constant 8 m/s with a constant 0.6 rad/s yaw rate, carrying a LiDAR, the
RGB camera and the thermal camera, all rigidly mounted. The LiDAR takes an instantaneous scan at
`t_l`, and the camera captures at `t_c = t_l + 33 ms`. Ground truth is the
scene ray-cast from the camera's true pose at `t_c`. The compensation test
samples the body's trajectory every 40 ms, compensates the scan to `t_c`, and
compares projected depth with ray-cast depth. After compensation all 493
projected box points and 964 of 994 projected points overall agree within
1e-9 relative depth, the same tolerance as the static RGB test; the rest are
occluded from the camera. The median depth error is below 1e-15 m. Without
compensation, none of the 493 box points agree and the median depth error
is 0.26 m on box points and 0.25 m over all points. Ground points are
less affected because the planar motion keeps the ground plane in place.

`tests/support/synthetic_sequence.hpp` turns the moving rig into a sequence:
five 10 Hz scans, 20 Hz RGB images, 10 Hz thermal images with one frame
dropped, sensor clock offsets, and the trajectory sampled every 40 ms. Scan 0
has no RGB image within its 25 ms `tau_ns`, and scan 3 has no thermal image
within the default 50 ms, so both rejection paths are exercised. The tests
check that:

- every reader returns the generator's values exactly (float32 points,
  bit-exact raw thermal counts and RGB pixels, trajectory poses);
- the association flags and `delta_t`s match a brute-force search;
- the aligned frames built from the in-memory (double-precision) scans agree
  with the ray-cast depth of each camera at its true pose at the image's time.
  In each of the 8 accepted camera images, at least 96% of the projected
  points (the lowest is 810 of 838) agree within 1e-9 relative depth, with a
  median error below 2e-15 m, and no point lands where the camera sees
  farther. 2943 visible thermal points away from surface edges read their
  surface's raw value;
- built from the files, the same points agree within 1e-5 relative depth,
  with a median error of about 2e-7 m. That limit comes from float32
  storage of the points;
- each malformed input gives its own error, and `fuse_cli` runs end to end
  with a summary that matches the per-frame CSVs.

### Roadmap

- Per-point deskew: moving each LiDAR point by its own capture time within
  the sweep. This needs per-point timestamps, which MS2 does not provide, so
  only scan-level compensation exists for now. Boreas has them: one sweep
  spans about 104 ms, up to 0.95 m and 4.4° of vehicle motion
  ([docs/validation/boreas.md](docs/validation/boreas.md)).
- Dataset adapters that convert into the [sequence format](docs/sequence-format.md).
  The first three are validation tooling: A2D2 in `tools/datasets/a2d2/`
  ([docs/validation/a2d2.md](docs/validation/a2d2.md)) and RGBDT LAENTIEC
  thermal in `tools/datasets/laentiec/`
  ([docs/validation/laentiec.md](docs/validation/laentiec.md)), and Boreas
  in `tools/datasets/boreas/`
  ([docs/validation/boreas.md](docs/validation/boreas.md)).
- Several LiDARs in one sequence. The format holds one LiDAR per sequence
  today, so the A2D2 chain check runs one sequence per LiDAR.

### Development container

The `Dockerfile` (Ubuntu 22.04) and `compose.yaml` provide the same toolchain:

```sh
docker compose run --rm dev
# inside the container, in /work:
cmake --preset default && cmake --build --preset default && ctest --preset default
```

The `dev` service mounts the repository at `/work`, a dataset folder at
`/data` (read-only), and `./out` at `/out`. The dataset folder defaults to
`./data`; point it elsewhere with `FUSION_DATA_DIR`:

```sh
FUSION_DATA_DIR=/path/to/datasets docker compose run --rm dev
```

Datasets are never committed to this repository.

## License

Licensed under the Apache License, Version 2.0. See [LICENSE](LICENSE).
