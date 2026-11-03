# Building and testing

## Requirements

Without Docker you need CMake 3.21 or newer, Ninja, GCC or Clang with C++17,
Eigen 3.4 and yaml-cpp. On Ubuntu:

```sh
sudo apt-get install cmake ninja-build g++ clang libeigen3-dev libyaml-cpp-dev
```

and then the same commands as inside the container (see the [quick start](../README.md#quick-start)).

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
