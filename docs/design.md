# Design

## What it does

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
(A2D2, LAENTIEC and Boreas; see [Validation](validation/README.md)).

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

## Design rules

- The core library (`fusion`) depends only on the C++17 standard library,
  Eigen and yaml-cpp, found on the system. `fusion_io` adds the vendored
  LodePNG for PNG. No ROS, GUI or dataset dependencies.
- No x86-only intrinsics; the code builds on arm64.
- Time is integer nanoseconds; transforms are `T_target_source`.
- Tests use synthetic data only.

