# Third-party notices

fusion-engine vendors the following third-party code under `third_party/`.
Each component keeps its own licence file next to the code.

| Component | Version | Licence | Location | Used by |
|-----------|---------|---------|----------|---------|
| [LodePNG](https://github.com/lvandeve/lodepng) by Lode Vandevenne | commit `7b2893e35891c0c85bee5595caafaf3c2044b409` (`LODEPNG_VERSION_STRING` 20261102) | zlib | `third_party/lodepng/` (`LICENSE`) | `fusion_io` library: PNG reading and writing |
| [doctest](https://github.com/doctest/doctest) by Viktor Kirilov | 2.5.3 | MIT | `third_party/doctest/` (`LICENSE.txt`) | unit tests only |

The LodePNG files `lodepng.h` and `lodepng.cpp` are unmodified copies of the
upstream files at the commit above.

Eigen and yaml-cpp are found on the system at build time and are not
vendored.
