# CLAUDE.md

## Build and test

Presets: `default` (GCC), `clang`, `asan` (Clang, ASan + UBSan), and
`ci-default`, `ci-clang`, `ci-asan` (same, warnings as errors). Build dir is
`build/<preset>`.

```sh
cmake --preset ci-default && cmake --build --preset ci-default && ctest --preset ci-default
cmake --preset ci-clang && cmake --build --preset ci-clang && ctest --preset ci-clang
cmake --preset ci-asan && cmake --build --preset ci-asan && ctest --preset ci-asan
```

## Layout

- `include/fusion/` public headers, `src/` core library (`fusion` static lib).
- `tests/` doctest unit tests, run through CTest.
- `third_party/doctest/` vendored doctest header.
- `.github/workflows/ci.yml` CI; `Dockerfile` and `compose.yaml` dev container.

## Rules

- Transforms are named `T_target_source` (maps points from `source` into `target`).
- Time is integer nanoseconds.
- No x86-only intrinsics; code must build on arm64.
- Core library depends only on the C++17 standard library, Eigen, and a YAML
  parser. No ROS, GUI, or dataset dependencies.
- Never commit datasets or images derived from datasets.
- Tests use synthetic data only.
- One change per commit, prefixed `feat:`, `fix:`, `test:`, `docs:`, or `build:`.
