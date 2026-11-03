# Using the library and tools

## Using the library

| Header                   | Provides |
|--------------------------|----------|
| `fusion/config.hpp`      | `load_sensor_config()` / `parse_sensor_config()` read a per-sensor YAML file into `SensorConfig`: name, resolution, intrinsics `K`, distortion `[k1, k2, p1, p2, k3]`, extrinsics `T_<sensor>_<source>`, and `time_offset_ns`. Thermal cameras add an optional `thermal` section (`encoding: planck` with `R`, `B`, `F`, `O`, or `encoding: linear` with `scale` and `offset`; see [thermal encoding](sequence-format.md#thermal-encoding)). Problems throw `ConfigError` naming the file and field. |
| `fusion/geometry.hpp`    | `CameraModel`: pinhole with radial-tangential distortion, `distort()` / `undistort()`, `project()`. `project_point(camera, T_camera_lidar, p_lidar)` returns the sub-pixel position, pixel index and z-depth. Points behind the camera, outside the image or beyond the radius where the distortion stops being monotonic are rejected. |
| `fusion/depth_map.hpp`   | `SparseDepthMap` keeping the nearest depth per pixel, and `project_to_depth_map()`. |
| `fusion/time.hpp`        | Integer nanoseconds (`TimeNs`). `associate()` pairs each reference time with the other sensor's nearest measurement (earlier one on a tie), accepted if `|t_j - t_ref| <= tau`, with `delta_t_ns = t_match_ns - t_ref_ns`. `default_tau_ns()` is half the slower sensor's period. |
| `fusion/pose.hpp`        | `Trajectory` of `StampedPose`s (`T_world_body`) with linear position and shortest-arc slerp interpolation. No extrapolation. |
| `fusion/lidar.hpp`       | Scan-level motion compensation: `compensate_scan()` moves a scan from `t_l` to `t_c` with `p_L(t_c) = T_body_lidar^-1 · T_world_body(t_c)^-1 · T_world_body(t_l) · T_body_lidar · p_L`. |
| `fusion/aligned_frame.hpp` | `build_aligned_frame()`: association, compensation and projection for one scan into each camera, with per-point pixel, depth, z-buffer flag and, for thermal, the raw value and °C. `write_aligned_frame_csv()` writes it. |
| `fusion/thermal.hpp`     | `raw_to_celsius()` (Planck or linear), `window_level()`, `hot_colormap()`, `render_display()`, `project_to_thermal()`. |
| `fusion/sequence.hpp`    | `fusion_io` library: reads and writes the [sequence format](sequence-format.md) with errors that name the file and line. |
| `fusion/image_io.hpp`    | `fusion_io` library: 8-bit RGB and 16-bit thermal PNG via LodePNG; raw thermal values round-trip bit-exactly. |

Conventions: transforms are named `T_target_source` (maps points from
`source` into `target`). Time is integer nanoseconds. Pixel centres sit at
integer coordinates, so the image spans `[-0.5, width - 0.5) x [-0.5,
height - 0.5)`. Depth is z in the camera's optical frame (x right, y down,
z forward). [`configs/`](../configs/) holds annotated example configs for the
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
| `viewer/index.html`, `viewer/points/` and the images | with `--viewer`: the [web viewer](viewer.md) |

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
[LAENTIEC check](validation/laentiec.md) uses it. `fusion_probe`
exposes trajectory interpolation and the RGB reader to validation scripts;
the [Boreas check](validation/boreas.md) uses it.

```sh
build/default/apps/thermal_export out/synthetic/sequence.yaml --out out/thermal
build/default/apps/fusion_probe interpolate trajectory.csv --times times.csv --out poses.csv
build/default/apps/fusion_probe gray out/synthetic/sequence.yaml --out out/gray --indices 0,5
```

Dataset converters for the validation checks live in
[`tools/datasets/`](../tools/datasets/) (Python 3 with numpy, outside the C++
libraries).

## Sequence format

Adapters convert datasets into one on-disk format: a YAML manifest, camera
configs, `timestamp_ns,path` list files, float32 LiDAR `.bin` scans, PNG
images (16-bit raw for thermal) and an optional trajectory CSV. See
[sequence-format.md](sequence-format.md).
