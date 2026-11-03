# fusion-engine

fusion-engine is a small C++17 library and command-line tool that aligns
LiDAR, RGB camera and thermal camera data to one reference time and one pixel
grid, and returns one aligned frame per LiDAR scan. It needs only the C++
standard library, Eigen and yaml-cpp (no ROS, no GUI toolkit). It is for
developers of perception pipelines on embedded platforms who want
multi-sensor alignment without a large dependency tree. How it works:
[docs/design.md](docs/design.md).

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
Eigen 3.4 and yaml-cpp; see [docs/building.md](docs/building.md) for the
packages, presets, tests and the synthetic test scene.

## Validation

| Check | Headline result | Page |
|-------|-----------------|------|
| Synthetic ground truth | After compensation all 493 box points match ray-cast depth to 1e-9 relative (0 before) | [building.md](docs/building.md#synthetic-test-scene) |
| A2D2 | Pixel equals A2D2's to ≤ 1.1e-4 px, depth to ≤ 3.8e-6 m | [a2d2.md](docs/validation/a2d2.md) |
| RGBDT LAENTIEC | Every thermal pixel reproduced to ≤ 3e-6 °C | [laentiec.md](docs/validation/laentiec.md) |
| Boreas | All 600 scans associated; scan-level compensation does not help | [boreas.md](docs/validation/boreas.md) |

The full table is in [docs/validation/README.md](docs/validation/README.md).
No dataset files or images derived from them are in this repository.

## Documentation

- [docs/design.md](docs/design.md): what the engine does, the pipeline at a
  glance, and the design rules.
- [docs/usage.md](docs/usage.md): the library headers and conventions, the
  `fuse_cli`, `thermal_export` and `fusion_probe` tools, and the sequence
  format summary.
- [docs/viewer.md](docs/viewer.md): the `fuse_cli --viewer` web viewer.
- [docs/building.md](docs/building.md): requirements, presets, CI, the dev
  container and the synthetic test scene.
- [docs/sequence-format.md](docs/sequence-format.md): the on-disk sequence
  format.

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
