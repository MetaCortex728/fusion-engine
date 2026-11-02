# fusion-engine

> **Status: early-stage.** The library can load a per-sensor YAML config,
> project LiDAR points into a distorted pinhole camera, build a sparse depth
> map, convert raw thermal counts to degrees Celsius, render them for display,
> and project LiDAR points into a thermal camera. Time alignment, aligned
> frames, and dataset adapters do not exist yet. There is no release.

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

### Build and test

Each preset builds into `build/<preset>`:

```sh
cmake --preset default && cmake --build --preset default && ctest --preset default
```

| Preset    | Compiler | Notes                                   |
|-----------|----------|-----------------------------------------|
| `default` | GCC      | RelWithDebInfo                          |
| `clang`   | Clang    | RelWithDebInfo                          |
| `asan`    | Clang    | Debug with AddressSanitizer and UBSan   |

The `ci-default`, `ci-clang`, and `ci-asan` presets do the same but treat
warnings as errors.

In some environments, binaries built with Clang 14 ASan sometimes segfault at
startup, before any test runs. This was seen on WSL2 with a 6.18 kernel, where
an empty ASan program crashes the same way. If `asan` or `ci-asan` fails with
`SEGFAULT` and prints no doctest output, rerun it.

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
