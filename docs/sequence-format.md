# Sequence format

A sequence is a directory that `fuse_cli` and the `fusion_io` library can read
without knowing which dataset it came from. Dataset adapters only convert
into this format; they never need to change the engine.

```
my_sequence/
├── sequence.yaml        manifest
├── rgb.yaml             camera configs (format: fusion/config.hpp)
├── thermal.yaml
├── trajectory.csv       optional body trajectory
├── lidar.csv            one list file per sensor
├── rgb.csv
├── thermal.csv
├── lidar/000000.bin     data files, anywhere below the manifest
├── rgb/000000.png
└── thermal/000000.png
```

The file and directory names above are only conventions. The manifest names
every file it uses.

## Manifest: `sequence.yaml`

```yaml
format_version: 1                # required, must be 1
trajectory: trajectory.csv       # optional
lidar:                           # required
  name: lidar                    # optional, default "lidar"
  list: lidar.csv                # required
  time_offset_ns: 0              # optional, default 0
  T_body_lidar:                  # required when a trajectory is given
    rotation: [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    translation: [0, 0, 1.8]     # metres
cameras:                         # optional, and so is each camera
  rgb:
    config: rgb.yaml             # required
    list: rgb.csv                # required
    tau_ns: 25000000             # optional association threshold
  thermal:
    config: thermal.yaml
    list: thermal.csv
```

- Paths are relative to the directory that holds the manifest. Absolute
  paths also work.
- Unknown fields are errors, so a typo cannot silently drop a setting.
- `lidar.name` is the LiDAR frame's name. Each camera config's
  `extrinsics.source` must equal it, because the engine uses the camera's
  extrinsics as `T_camera_lidar`.
- `T_body_lidar` maps points from the LiDAR frame into the body frame of the
  trajectory. The rotation must be orthonormal with determinant +1 (within
  1e-5, the same check as the sensor config), and it is re-orthonormalized
  when read.
- `tau_ns` is the camera's association threshold: an image is accepted for a
  scan only if `|t_image - t_scan| <= tau_ns` on the reference clock. Without
  it the default is half the slower sensor's median period
  (`default_tau_ns()`), which needs at least two scans and two images.

## Clocks

All timestamps are integer nanoseconds. LiDAR and camera list files hold
times on each sensor's own clock. The sensor's `time_offset_ns`, from the
manifest for the LiDAR and from the camera config for a camera, moves them
onto the reference clock:

```
t_reference = t_sensor + time_offset_ns
```

Trajectory timestamps are on the reference clock.

## List files: `timestamp_ns,path`

```
timestamp_ns,path
1707000000,lidar/000000.bin
1807000000,lidar/000001.bin
```

- The first line must be exactly `timestamp_ns,path`.
- Each row is a decimal integer timestamp, a comma, and a path. Everything
  after the first comma is the path, so paths may contain commas. Quoting
  is not supported.
- Timestamps must increase strictly. Unsorted or duplicate times are errors.
- Every listed file must exist, and the list must not be empty.
- Blank lines are ignored and Windows line endings are accepted.

## LiDAR scans: `.bin`

Each point is one 16-byte record of four little-endian IEEE-754 float32
values: `x, y, z, intensity`. Coordinates are metres in the LiDAR frame.
There is no header. The file size must be a multiple of 16 bytes, and every
value must be finite. Adapters should drop points with no return rather than
write NaN.

The scan has one time, the list file's timestamp (scan-level). Per-point
times are not part of format version 1.

## Images

- **RGB**: 8-bit PNG, normally RGB. Greyscale, palette, and alpha variants
  are accepted and converted to RGB. 16-bit RGB images are rejected.
- **Thermal**: 16-bit greyscale PNG holding the camera's raw values
  unchanged. Any other colour type or bit depth is rejected, so values are
  never rescaled on the way in. The camera config's `thermal` section says
  what the values mean (see below).
- The image size must equal the `resolution` in the camera's config. This is
  checked when the image is read.

### Thermal encoding

The thermal camera config (format: `fusion/config.hpp`) has an optional
`thermal` section that converts raw values to degrees Celsius. It is used
for rendering and for the per-point `thermal_celsius` output. There are two
encodings:

```yaml
thermal:
  encoding: planck               # optional, the default
  planck: {R: 380747, B: 1428, F: 1, O: -88.539}
```

`planck` is for raw radiometric counts S:
`T = B / ln(R / (S - O) + F) - 273.15`, with finite constants, R > 0 and
B > 0.

```yaml
thermal:
  encoding: linear
  linear: {scale: 0.01, offset: -273.15}
```

`linear` is for cameras that deliver temperature directly:
`T = raw * scale + offset`, with finite values and `scale != 0`. The
example is centi-kelvin, which many radiometric exports use (Optris, FLIR).
Raw 0 is −273.15 °C and 65535 is 382.2 °C. To store temperatures in
centi-kelvin, write `round((T + 273.15) * 100)` and check that it fits in
0 to 65535. `tools/datasets/laentiec/convert.py` is an example.

Only the selected encoding's section may be present. An unknown encoding,
a missing or non-finite parameter, or a zero scale is a `ConfigError` that
names the field, e.g. `thermal.linear.scale: must not be zero`. Without a
`thermal` section the raw values are still read and written, but nothing is
converted.

## Trajectory: `timestamp_ns,x,y,z,qw,qx,qy,qz`

```
timestamp_ns,x,y,z,qw,qx,qy,qz
1607000000,-4.141788597280218,-0.1179,0,0.9941844405466216,0,0,-0.10769075250457227
```

Each row is a pose `T_world_body`: the position of the body origin in the
world frame, and the orientation as a unit quaternion (Hamilton convention,
scalar first). Quaternions are normalized when read and must not be zero.
Times must increase strictly. Between rows the engine interpolates positions
linearly and orientations with shortest-arc slerp. It never extrapolates: a
camera whose time falls outside the trajectory gets an uncompensated
projection, and the output marks it with `compensated = 0`.

Write numbers with full precision. The library writer uses the shortest text
that reads back to the same double.

## Errors

`load_sequence()` reads the manifest, the camera configs, every list file and
the trajectory, and checks that every listed file exists. Data files are read
one frame at a time. Every problem throws `fusion::SequenceError`. The
message starts with the file, and with the line number for list and
trajectory rows. Examples:

| Problem                         | Message (abridged) |
|---------------------------------|--------------------|
| missing data file               | `lidar.csv:3: missing file .../lidar/000001.bin` |
| missing config or list          | `sequence.yaml: cameras.rgb.config: missing file .../rgb.yaml` |
| unsorted timestamps             | `lidar.csv:3: unsorted timestamps: 1706999999 after 1707000000` |
| duplicate timestamps            | `lidar.csv:3: duplicate timestamp 1707000000` |
| wrong LiDAR record size         | `lidar/000000.bin: size <n> bytes is not a multiple of the 16-byte record (float32 x,y,z,intensity)` |
| image size differs from config  | `rgb/000000.png: image size 3x2 does not match the sensor config 'rgb' resolution 640x480` |
| wrong PNG type for thermal      | `thermal/000000.png: expected a 16-bit greyscale PNG of raw thermal counts, ...` |
| bad header                      | `rgb.csv:1: expected the header 'timestamp_ns,path'` |
| unknown manifest field          | `sequence.yaml: trajectroy: unknown field (line 3)` |

## Writing a sequence from code

`fusion/sequence.hpp` and `fusion/image_io.hpp` provide `write_list_csv()`,
`write_trajectory_csv()`, `write_lidar_bin()`, `write_rgb_png()` and
`write_thermal_png()`. The synthetic generator in
`tests/support/synthetic_sequence.cpp` is a complete example: it writes the
manifest and camera configs as text and every data file with these
functions.
