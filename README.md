# fusion-engine

> **Status: early-stage.** The library can load a per-sensor YAML config,
> project LiDAR points into a distorted pinhole camera, build a sparse depth
> map, convert raw thermal counts to degrees Celsius, render them for display,
> and project LiDAR points into a thermal camera. It can associate sensor
> timestamps, interpolate a body trajectory, and move a whole LiDAR scan to a
> camera's capture time. Aligned frames and dataset adapters do not exist
> yet. There is no release.

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

No ROS dependency.

## Design rules

- Dependencies stay minimal: Eigen and yaml-cpp, both found on the system
  and never vendored. Eigen types appear in the public headers; yaml-cpp is
  used only inside the config loader.
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
| `fusion/config.hpp`      | `load_sensor_config()` / `parse_sensor_config()` read a per-sensor YAML file into `SensorConfig`: name, resolution, intrinsics `K`, distortion `[k1, k2, p1, p2, k3]`, extrinsics `T_<sensor>_<source>`, and `time_offset_ns`. Thermal cameras add an optional `thermal.planck` section with the constants `R`, `B`, `F`, `O`, which loads into `SensorConfig::thermal`. Problems throw `ConfigError` naming the file and field, e.g. a missing field, a wrong shape, or a rotation that is not invertible or not orthonormal. |
| `fusion/geometry.hpp`    | `CameraModel`: pinhole with radial-tangential distortion, `distort()` / `undistort()`, and `project()`. `project_point(camera, T_camera_lidar, p_lidar)` returns the sub-pixel position, pixel index, and z-depth. Only points with z > 0 that land inside the image are kept. Points beyond the radius where radial distortion stops being monotonic are also rejected, so they cannot fold back into the image. |
| `fusion/depth_map.hpp`   | `SparseDepthMap` over the camera's pixel grid, keeping the nearest depth per pixel, and `project_to_depth_map()` to fill one from a point cloud. |
| `fusion/time.hpp`        | Times are `int64_t` nanoseconds (`TimeNs`). `to_reference_time()` applies a sensor's `time_offset_ns` (`t_reference = t_sensor + time_offset_ns`). `associate()` pairs each reference time (the LiDAR's, by default) with the other sensor's nearest measurement, takes the earlier one on an exact tie, and accepts it only if `|t_j - t_ref| <= tau`. An overload takes sensor-clock stamps plus both offsets. Every reference time gets a result, rejected ones included, with `delta_t_ns = t_match_ns - t_ref_ns`. `default_tau_ns()` gives half the slower sensor's period. `write_association_csv()` writes `t_ref_ns,t_match_ns,delta_t_ns,accepted`. |
| `fusion/pose.hpp`        | `StampedPose` (time, `Eigen::Vector3d` position, `Eigen::Quaterniond` orientation) and `Trajectory`, which holds poses with strictly increasing times and rejects unsorted or duplicate times. `interpolate(t)` and `T_world_body(t)` use `alpha = (t - t_i) / (t_{i+1} - t_i)`, linear position, and slerp along the shortest arc (`slerp_shortest()`). Times outside the trajectory throw `std::out_of_range`; there is no extrapolation. |
| `fusion/lidar.hpp`       | Scan-level motion compensation. `compensate_scan()` moves a scan taken at `t_l` to a camera time `t_c`: `p_L(t_c) = T_body_lidar^-1 * T_world_body(t_c)^-1 * T_world_body(t_l) * T_body_lidar * p_L`. The points stay in the LiDAR frame, so projection with `T_camera_lidar` applies unchanged. `scan_motion_transform()` returns that transform. |
| `fusion/thermal.hpp`     | `RawThermalImage` holds raw `uint16_t` counts unchanged. `raw_to_celsius()` applies `T = B / ln(R / (S - O) + F) - 273.15` and throws `std::domain_error` when the result is undefined: S equal to O, a logarithm argument that is not positive or equals 1, or a result at or below absolute zero. `window_level()` computes `clamp((T - (L - W / 2)) / W, 0, 1)` and rejects W <= 0. `hot_colormap()` maps that value to 8-bit RGB, black through red and yellow to white. `render_display()` builds a separate display image and only reads the raw image. `project_to_thermal()` projects LiDAR points with `project_point()`. It returns the thermal `SparseDepthMap` and, for every projected point, its pixel, its depth, and the raw count at that pixel. |

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
constant 8 m/s with a constant 0.6 rad/s yaw rate, carrying a LiDAR and the
RGB camera, both rigidly mounted. The LiDAR takes an instantaneous scan at
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

### Roadmap

- Per-point deskew: moving each LiDAR point by its own capture time within
  the sweep. This needs per-point timestamps, which MS2 does not provide, so
  only scan-level compensation exists for now.
- Aligned frames and dataset adapters.

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
