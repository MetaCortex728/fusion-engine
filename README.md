# fusion-engine

> **Status: early-stage.** This repository currently holds only project
> documentation and the license. There is no source code, build system, or
> release yet. Nothing described as a goal below is implemented today.

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
- A YAML parser for configuration (not yet chosen)

No ROS dependency.

## Design rules

- Dependencies stay minimal: Eigen and a YAML parser.
- No x86-only intrinsics, so the code stays portable to embedded targets.
- Time is represented as integer nanoseconds.
- Transforms are named `T_target_source`: the transform that maps a point
  from the `source` frame into the `target` frame.

## How to run

There is nothing to build or run yet. Build instructions will arrive together
with the build system.

## License

Licensed under the Apache License, Version 2.0. See [LICENSE](LICENSE).
