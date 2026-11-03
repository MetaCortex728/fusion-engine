# The web viewer

`fuse_cli ... --viewer` writes `viewer/index.html` into the output directory:
a static page with inline CSS and JavaScript and no external libraries. The
run's data except the per-point arrays is embedded in the page as JSON
(`<script type="application/json" id="fusion-data">`; nanosecond timestamps
are strings there, because epoch nanoseconds do not fit exactly in a
JavaScript number). The points of the k-th frame are in
`points/frame_NNNNNN.js` (NNNNNN = k), a script calling
`fusionPoints(k, {"rgb": …, "thermal": …})`; the page loads it with a
`<script>` element when it shows the frame (and the frames either side), so
opening a long run does not parse every frame's points, and there is no
`fetch`, which `file://` pages cannot use. The camera images it
shows are written next to it: `rgb_NNNNNN.png` is the RGB frame and
`thermal_NNNNNN.png` the thermal frame rendered in °C with the hot colour
map, both numbered by the camera's image index. Keep the `viewer/`
directory together when copying it.

- **Per frame:** the RGB or rendered thermal image with the projected LiDAR
  points drawn at their pixel, coloured by depth (red near, blue far) or by
  the point's thermal value from the thermal camera (grey where it has none).
- **Controls:** frame slider and previous/next (arrow keys and Home/End work
  too), RGB/thermal, colour by depth or thermal, point size, colour range
  (min/max, with "Auto" for the run's range), opacity, and whether to show
  points hidden by the z-buffer (drawn as outlines).
- **Side panel:** per camera the association `Δt`, accepted and compensated
  flags and the number of projected points (hidden and embedded counts on
  hover); the scan's point count and processing, read and write times; the
  run's median and p95 processing and read latency.
- **Layout:** desktop and narrow widths, light and dark (follows the
  system setting).
- **Large runs:** at most 20000 points per camera and frame are embedded
  (`--viewer-max-points N` changes the cap). When a camera sees m > N
  points, they are taken in scan order and the k-th kept point
  (k = 0 … N − 1) is the one at rank `floor(k · m / N)`. The selection
  depends only on m and N, so a run always embeds the same points. The page
  states the cap and marks frames that were subsampled. The page is written
  frame by frame, so memory use does not grow with the run length.
- **Size:** about 34–36 bytes per embedded point. At the default cap a
  frame with 20000 points in each of two cameras is a 1.4 MB points file,
  so 100 such frames are about 140 MB on disk; the browser reads only the
  page (about 0.5 KB per frame plus 20 KB) and the points files of the shown
  frame and its neighbours. In headless Chromium the synthetic run's page
  loads in about 15–30 ms and a 1.4 MB frame is loaded and drawn in about
  80 ms.

The tests run `fuse_cli --viewer` on the synthetic sequence, parse the
embedded JSON and the points files and compare every frame, flag, count and point with the CSV
outputs. The page itself has been loaded in headless Chromium (light, dark,
390 px wide) with no script errors and no network requests.
