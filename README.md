# fusion-engine

> **Status: early-stage.** This repository holds the build system, a
> development container, and a placeholder library that only reports its
> version. There is no release yet. Nothing described as a goal below is
> implemented today.

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

The library currently provides only `fusion::version()`. You need CMake 3.21
or newer, Ninja, and GCC or Clang with C++17 support.

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
