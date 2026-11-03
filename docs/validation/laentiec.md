# Real-data check 2: radiometric thermal input on RGBDT LAENTIEC

This check runs the engine's linear thermal encoding on real temperature
fields. The RGBDT dataset's Optris PI 640i delivers temperature directly, in
degrees Celsius per pixel. The converter stores each frame as a 16-bit PNG in
centi-kelvin, the way many radiometric cameras export temperature. The engine
reads the PNGs back with `encoding: linear` (`scale: 0.01`,
`offset: -273.15`). Its Celsius value for every pixel is then compared with
the original CSV, and every frame is rendered with the engine's
window/level and hot colour map.

Tooling: [`tools/datasets/laentiec/`](../../tools/datasets/laentiec/) (Python 3
with numpy only, pinned in `requirements.txt`) and the `thermal_export` tool
in [`apps/thermal_export/`](../../apps/thermal_export/main.cpp).

## What it proves

- **The linear encoding, end to end.** The sensor config's `thermal`
  section with `encoding: linear` is parsed, and 16-bit PNGs are read
  through `fusion_io` with their values unchanged. `T = raw * 0.01 - 273.15`
  reproduces all 95,846,400 pixels of 312 real frames to within 3e-6 °C.
  That residual comes from the CSV, not from the engine (see below).
- **Range.** The scenes cover −24.8 °C to 29.8 °C. 240 of the 312 frames
  contain temperatures below 0 °C and 5 go below −20 °C. Centi-kelvin
  values 24,835 to 30,295 are used, far from both ends of the 16-bit range
  (−273.15 °C to 382.2 °C).
- **Statistics are preserved.** Per image, the engine's minimum, maximum and
  mean agree with the CSV's to within 3e-6 °C, 5.7e-14 °C and 1.9e-8 °C.
- **Rendering on real temperature fields.** `render_display()` with the
  linear encoding and a per-image window gives, in every frame, exactly as
  many pure black and pure white pixels as an independent numpy version of
  the window/level and hot colour map. Nothing is clipped beyond what the
  window implies. No pixel fails to convert.

## What it does not prove

- **Planck conversion of raw counts.** The Optris data is already
  temperature, so `encoding: planck` is not used here. It is covered only by
  the unit tests.
- **Geometry.** The dataset has no calibration between the thermal camera
  and the RealSense RGB-D camera. The sequence uses placeholder intrinsics
  and extrinsics, and every LiDAR scan is empty. So projection, the depth
  map, extrinsics and time alignment are not exercised.
- **Radiometric accuracy of the camera.** The check compares the engine with
  the camera's own temperature output. Whether that output is right
  (emissivity, ambient compensation) is outside its scope.
- **Temperatures above 29.8 °C**, or near the ends of the 16-bit range on
  real data. Those cases are covered only by the unit tests (raw 0 and
  65535).

## Results (312 frames, 95,846,400 pixels)

Run on 2026-11-03 with a `default` (GCC, RelWithDebInfo) build of
`thermal_export`. Error is `engine °C − CSV °C`.

| measure | value |
|---------|-------|
| temperature range | −24.8 °C to 29.8 °C |
| max \|error\| | 3.0e-06 °C (bound from centi-kelvin rounding: 0.005 °C) |
| per-image min / max / mean, max difference | 3.0e-06 / 5.7e-14 / 1.9e-08 °C |
| engine window vs numpy window, max difference | 3.0e-06 °C (low end) / 5.7e-14 °C (high end) |
| non-convertible pixels | 0 |

Histogram of |error| over all pixels:

| \|error\| (°C) | pixels |
|----------------|--------|
| exactly 0 | 494,949 |
| 1e-14 to 1e-13 | 93,530,284 |
| 1e-6 to 1e-3 | 1,821,167 |
| anything else | 0 |

- **Where the residual comes from.** The Optris temperatures have a
  resolution of 0.1 °C. Most values are written with one decimal and map to
  a whole number of centi-kelvin with no rounding loss. Converting back
  costs one floating-point rounding, under 1e-13 °C. 1,821,167 pixels (1.9%), all
  between −1 °C and 1 °C, are written in the CSV with float32 noise, for
  example `0.900002` or `-0.199997`. Rounding to centi-kelvin snaps them
  back onto the 0.1 °C grid, so the engine returns 0.9 and differs from the
  CSV by up to 3e-6 °C. The same snapping explains the 3e-6 °C difference
  in the minimum: frame 51's coldest pixel is `-0.300003` in the CSV and
  −0.3 °C in the engine. No error lies between 1e-3 °C and the 0.005 °C rounding
  bound.
- **Rendering.** Each frame was rendered with the window from the 1st to the
  99th percentile of its own temperatures (nearest rank). Window widths are
  2.8 °C to 33.2 °C (median 15.7 °C). At most 1.00% of the pixels lie below
  the window and at most 1.00% above it. Pure black pixels make up a median
  of 1.06% (at most 1.55%), pure white a median of 1.24% (at most 2.16%).
  Values equal to the window's ends are included, so these can exceed 1%.
  No frame was flagged for clipping, a narrow window (< 1 °C), or a
  disagreement with the numpy reference.
- **Expected but worth knowing:** percentile windows compress bimodal
  scenes. Many frames show cold sky (down to −24.8 °C) above ground at
  10 °C to 20 °C. The sky's tail stretches the window downwards, so the
  ground lands in the top tenth of the window, where the hot colour map is
  pale yellow to white. In 223 of 312 frames more than 25% of the pixels lie
  in the top tenth (median 39.6%, at most 78.2%). Nothing is clipped there,
  and the engine renders what the window asks for. For display, a window
  fitted to the region of interest works better on these scenes than a
  whole-image percentile window.
- **`fuse_cli`** also runs on the converted sequence: 312 frames, each
  thermal image associated with `delta_t = 0`, and no points, because the
  scans are empty.

The renders kept for review (in `/results/laentiec/`, not in the repository)
are: the coldest and warmest scene, the narrowest and widest window, the most
saturated frame, the median frame, and the two frames with the most pixels in
the top tenth.

## Licence and attribution

Thermal data: RGBDT dataset, LAENTIEC experimental area, University of
Málaga, [Zenodo record 8032971](https://zenodo.org/records/8032971),
licensed [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/). Results
and images derived from it may be published with this attribution. The
renders were made by converting the temperatures to centi-kelvin and
rendering them with fusion-engine's window/level and hot colour map.

This repository contains only code and the statistics above. Converted
sequences, Celsius exports and renders stay outside it, as the project's
rules require.

## How to reproduce

**1. Data.** Download the record's archive (about 1.5 GB) from
<https://zenodo.org/records/8032971> and extract it, for example to
`~/rgbdt`. The thermal frames are in `RAW-RGBDT-Dataset/thermal/`:
`<timestamp_ms>.csv` (640×480, °C) next to the dataset's own `.png`
previews, which are not used. The check used a copy that had already been
extracted to `/data/extracted`. The download step itself was not run as part
of it.

**2. Build `thermal_export`.**

```sh
cmake --preset default && cmake --build --preset default
```

**3. Convert and compare** (in a virtual environment outside the
repository):

```sh
python3 -m venv ~/laentiec-venv
~/laentiec-venv/bin/pip install -r tools/datasets/laentiec/requirements.txt
~/laentiec-venv/bin/python -I tools/datasets/laentiec/convert.py \
    --data ~/rgbdt/RAW-RGBDT-Dataset/thermal --out /tmp/laentiec_work
~/laentiec-venv/bin/python -I tools/datasets/laentiec/compare.py \
    --work /tmp/laentiec_work --thermal-export build/default/apps/thermal_export \
    --results /results/laentiec
```

`convert.py` first reads every CSV and checks that each value is finite and
that `round((T + 273.15) * 100)` fits in 0 to 65535. It stops before
writing anything if not, and prints the minimum and maximum temperatures. It
then writes a sequence with one 16-bit PNG per frame, a thermal config with
`encoding: linear`, and an empty LiDAR scan per frame. The sequence format
needs a LiDAR, and the scans share the thermal timestamps (the file names,
in ms). It also writes `frames.csv` (frame, timestamp, source CSV, PNG) and
`range.json` (range and grid checks).

`compare.py` runs `thermal_export` with a 1st–99th percentile window. Then it
re-reads the original CSVs and writes:

| File | Contents |
|------|----------|
| `per_frame.csv` | per frame: CSV and engine min/max/mean, max \|error\|, window from numpy and from the engine, pixels below and above it, pure black and white display pixels (engine and numpy), pixels in the bottom and top tenth, flags and notes |
| `error_hist.csv` | histogram of \|error\| |
| `summary.json` | totals, flagged and noted frames, the renders kept |
| `report.md` | the same as short tables, with the attribution |
| `render_*.png` | renders for visual review |

`thermal_export` works with any sequence that has a thermal camera with a
`thermal` section. For each frame it writes the engine's Celsius values as
little-endian float64 (`celsius_NNNNNN.f64`), a render (`render_NNNNNN.png`)
and a row of `thermal_stats.csv`:

```sh
thermal_export <sequence.yaml> --out <dir> [--frames N] [--percentiles LO,HI] \
    [--no-celsius] [--no-render]
```
