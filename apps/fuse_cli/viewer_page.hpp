// The viewer page around the embedded JSON: kViewerPageHead ends inside the
// <script type="application/json" id="fusion-data"> element, kViewerPageTail
// closes it and holds the page's script. Everything is inline; the page makes
// no network requests and uses no external libraries.
#pragma once

namespace fuse_cli {

inline constexpr const char* kViewerPageHead = R"HTML(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>fusion-engine viewer</title>
<style>
:root {
  color-scheme: light dark;
  --bg: #f6f7f9; --fg: #1d2129; --muted: #5b6270; --panel: #ffffff;
  --border: #d6dae1; --accent: #1f6feb; --stage: #e4e7ec; --warn: #9a6700;
  --ok: #1a7f37; --bad: #cf222e;
}
@media (prefers-color-scheme: dark) {
  :root {
    --bg: #111418; --fg: #e6e8eb; --muted: #9aa3ae; --panel: #1a1e24;
    --border: #2f3640; --accent: #58a6ff; --stage: #0b0d10; --warn: #d29922;
    --ok: #3fb950; --bad: #f85149;
  }
}
* { box-sizing: border-box; }
body {
  margin: 0; background: var(--bg); color: var(--fg);
  font: 14px/1.45 system-ui, -apple-system, "Segoe UI", Roboto, sans-serif;
}
header { padding: 12px 16px; border-bottom: 1px solid var(--border); }
header h1 { margin: 0; font-size: 17px; font-weight: 600; }
header p { margin: 2px 0 0; color: var(--muted); word-break: break-all; }
.layout { display: flex; gap: 16px; padding: 16px; align-items: flex-start; }
.stage { flex: 1 1 auto; min-width: 0; }
.panel {
  flex: 0 0 340px; background: var(--panel); border: 1px solid var(--border);
  border-radius: 8px; padding: 12px 14px;
}
.nav { display: flex; gap: 8px; align-items: center; margin-bottom: 8px; }
.nav input[type=range] { flex: 1 1 auto; min-width: 80px; }
.nav output { min-width: 9ch; text-align: right; font-variant-numeric: tabular-nums; }
button {
  font: inherit; color: var(--fg); background: var(--panel);
  border: 1px solid var(--border); border-radius: 6px; padding: 4px 10px; cursor: pointer;
}
button:hover { border-color: var(--accent); }
button:focus-visible, input:focus-visible { outline: 2px solid var(--accent); outline-offset: 1px; }
.canvas-wrap {
  position: relative; background: var(--stage); border: 1px solid var(--border);
  border-radius: 8px; overflow: hidden; line-height: 0;
}
canvas { display: block; width: 100%; height: auto; image-rendering: pixelated; }
#message {
  position: absolute; inset: 0; display: flex; align-items: center; justify-content: center;
  line-height: 1.4; color: var(--muted); padding: 16px; text-align: center;
}
#message[hidden] { display: none; }
.legend { display: flex; align-items: center; gap: 8px; margin-top: 8px; color: var(--muted); }
#legend-bar { flex: 1 1 auto; height: 10px; border-radius: 5px; border: 1px solid var(--border); }
.legend span { font-variant-numeric: tabular-nums; white-space: nowrap; }
.hint { color: var(--muted); font-size: 12px; margin: 6px 0 0; }
h2 { font-size: 13px; margin: 14px 0 6px; text-transform: uppercase; letter-spacing: .04em; color: var(--muted); }
h2:first-child { margin-top: 0; }
.row { display: flex; align-items: center; gap: 8px; margin: 6px 0; flex-wrap: wrap; }
.row > label:first-child { flex: 0 0 96px; color: var(--muted); }
.row input[type=range] { flex: 1 1 100px; }
.row input[type=number] {
  width: 4.6em; font: inherit; color: var(--fg); background: var(--bg);
  border: 1px solid var(--border); border-radius: 4px; padding: 2px 4px;
}
.seg { display: inline-flex; gap: 10px; }
table { width: 100%; border-collapse: collapse; font-variant-numeric: tabular-nums; }
th, td { text-align: left; padding: 3px 4px; border-bottom: 1px solid var(--border); }
th { color: var(--muted); font-weight: 500; }
td.num, th.num { text-align: right; }
.yes { color: var(--ok); } .no { color: var(--bad); }
.note { color: var(--warn); font-size: 12px; margin: 6px 0 0; }
dl { display: grid; grid-template-columns: auto 1fr; gap: 2px 10px; margin: 0; }
dt { color: var(--muted); } dd { margin: 0; text-align: right; font-variant-numeric: tabular-nums; }
@media (max-width: 860px) {
  .layout { flex-direction: column; padding: 10px; }
  .panel { flex: none; width: 100%; }
}
</style>
</head>
<body>
<header>
  <h1>fusion-engine viewer</h1>
  <p id="sequence"></p>
</header>
<div class="layout">
  <main class="stage">
    <div class="nav">
      <button id="prev" type="button" aria-label="Previous frame">&#9664;</button>
      <input id="frame" type="range" min="0" max="0" value="0" step="1" aria-label="Frame">
      <button id="next" type="button" aria-label="Next frame">&#9654;</button>
      <output id="frame-label" for="frame"></output>
    </div>
    <div class="canvas-wrap">
      <canvas id="view" width="640" height="480"></canvas>
      <div id="message" hidden></div>
    </div>
    <div class="legend">
      <span id="legend-lo"></span><div id="legend-bar"></div><span id="legend-hi"></span>
    </div>
    <p class="hint">Arrow keys change frames. Points are drawn at their projected pixel (pixel centres at integer coordinates), far points first. Hidden points (behind a nearer point at the same pixel) are drawn as outlines.</p>
  </main>
  <aside class="panel">
    <h2>View</h2>
    <div class="row"><label>Camera</label>
      <span class="seg">
        <label><input type="radio" name="camera" value="rgb" id="cam-rgb"> RGB</label>
        <label><input type="radio" name="camera" value="thermal" id="cam-thermal"> Thermal</label>
      </span></div>
    <div class="row"><label>Colour by</label>
      <span class="seg">
        <label><input type="radio" name="color" value="depth" id="color-depth" checked> Depth</label>
        <label><input type="radio" name="color" value="thermal" id="color-thermal"> Thermal</label>
      </span></div>
    <div class="row"><label for="size">Point size</label>
      <input id="size" type="range" min="1" max="12" step="1" value="4"><output id="size-out"></output></div>
    <div class="row"><label for="opacity">Opacity</label>
      <input id="opacity" type="range" min="0.05" max="1" step="0.05" value="0.9"><output id="opacity-out"></output></div>
    <div class="row"><label for="range-min" id="range-label">Depth (m)</label>
      <input id="range-min" type="number" step="0.1" aria-label="Colour range minimum">
      <input id="range-max" type="number" step="0.1" aria-label="Colour range maximum">
      <button id="range-auto" type="button">Auto</button></div>
    <div class="row"><label for="hidden">Hidden points</label>
      <label><input id="hidden" type="checkbox"> show z-buffered points</label></div>

    <h2>Frame</h2>
    <dl id="frame-info"></dl>
    <table>
      <thead><tr><th>Camera</th><th class="num">&Delta;t (ms)</th><th>Accepted</th><th>Comp.</th><th class="num">Points</th></tr></thead>
      <tbody id="camera-rows"></tbody>
    </table>
    <p class="note" id="subsample-note" hidden></p>

    <h2>Run</h2>
    <dl id="run-info"></dl>
    <p class="hint" id="cap-note"></p>
  </aside>
</div>
<script type="application/json" id="fusion-data">)HTML";

inline constexpr const char* kViewerPageTail = R"HTML(</script>
<script>
(function () {
  'use strict';
  var data = JSON.parse(document.getElementById('fusion-data').textContent);
  var frames = data.frames;
  var run = data.run;
  function $(id) { return document.getElementById(id); }

  // Colour maps. Depth: red (near) through yellow, green and cyan to blue
  // (far), the same as fuse_cli's overlays. Thermal: the hot map from dark red
  // through yellow to white.
  function depthColor(t) {
    var h = 4 * t, k = Math.min(Math.floor(h), 3), f = h - k, c = Math.round(255 * f), d = 255 - c;
    if (k === 0) return [255, c, 0];
    if (k === 1) return [d, 255, 0];
    if (k === 2) return [0, 255, c];
    return [0, d, 255];
  }
  function hotColor(t) {
    var x = 0.2 + 0.8 * t;
    return [Math.round(255 * Math.min(1, 3 * x)), Math.round(255 * Math.min(1, Math.max(0, 3 * x - 1))),
            Math.round(255 * Math.min(1, Math.max(0, 3 * x - 2)))];
  }
  function lut(fn) {
    var out = [];
    for (var i = 0; i < 256; i++) { var c = fn(i / 255); out.push('rgb(' + c[0] + ',' + c[1] + ',' + c[2] + ')'); }
    return out;
  }
  var LUT = { depth: lut(depthColor), thermal: lut(hotColor) };
  var NO_VALUE = 'rgb(150,150,150)';

  function niceRange(r, step) {
    if (!r) return [0, 1];
    var lo = Math.floor(r[0] / step) * step, hi = Math.ceil(r[1] / step) * step;
    if (hi <= lo) hi = lo + step;
    return [Number(lo.toFixed(2)), Number(hi.toFixed(2))];
  }
  var autoRange = { depth: niceRange(run.depth_range, 0.1), thermal: niceRange(run.thermal_range, 0.1) };

  var hasRgb = !!data.cameras.rgb, hasThermal = !!data.cameras.thermal;
  var state = {
    frame: 0,
    camera: hasRgb ? 'rgb' : 'thermal',
    color: 'depth',
    size: 4,
    opacity: 0.9,
    showHidden: false,
    range: { depth: autoRange.depth.slice(), thermal: autoRange.thermal.slice() }
  };

  var canvas = $('view'), ctx = canvas.getContext('2d');
  var images = {};
  function image(src) {
    if (!images[src]) {
      var img = new Image();
      img.onload = function () { draw(); };
      img.onerror = function () { img.failed = true; draw(); };
      img.src = src;
      images[src] = img;
    }
    return images[src];
  }

  function fmt(x, d) { return x === null || x === undefined ? '–' : Number(x).toFixed(d); }
  function el(tag, text, cls) {
    var e = document.createElement(tag);
    if (text !== undefined) e.textContent = text;
    if (cls) e.className = cls;
    return e;
  }
  function fillDl(dl, pairs) {
    dl.textContent = '';
    pairs.forEach(function (p) { dl.appendChild(el('dt', p[0])); dl.appendChild(el('dd', p[1])); });
  }
  function unit() { return state.color === 'depth' ? 'm' : data.thermal_unit; }

  function drawPoints(cam) {
    var p = cam.points, n = p.u.length, order = [];
    for (var i = 0; i < n; i++) if (p.nearest[i] || state.showHidden) order.push(i);
    order.sort(function (a, b) { return p.depth[b] - p.depth[a]; });
    var values = state.color === 'depth' ? p.depth : p.thermal, table = LUT[state.color];
    var lo = state.range[state.color][0], hi = state.range[state.color][1], span = hi - lo;
    var s = state.size, half = s / 2;
    ctx.globalAlpha = state.opacity;
    ctx.lineWidth = Math.max(1, s / 4);
    for (var k = 0; k < order.length; k++) {
      var j = order[k], v = values[j], color = NO_VALUE;
      if (v !== null) {
        var t = span > 0 ? (v - lo) / span : 0;
        color = table[Math.round(255 * Math.min(1, Math.max(0, t)))];
      }
      var x = p.u[j] + 0.5 - half, y = p.v[j] + 0.5 - half;
      if (p.nearest[j]) {
        ctx.fillStyle = color;
        ctx.fillRect(x, y, s, s);
        if (s >= 5) {
          // A thin dark edge keeps points readable on images of similar colour.
          ctx.lineWidth = 1;
          ctx.strokeStyle = 'rgba(0,0,0,0.55)';
          ctx.strokeRect(x + 0.5, y + 0.5, s - 1, s - 1);
          ctx.lineWidth = Math.max(1, s / 4);
        }
      } else {
        ctx.strokeStyle = color;
        ctx.strokeRect(x, y, s, s);
      }
    }
    ctx.globalAlpha = 1;
  }

  function draw() {
    var f = frames[state.frame];
    var dims = data.cameras[state.camera];
    var cam = f ? f[state.camera] : null;
    if (!dims) {
      $('message').textContent = 'This sequence has no cameras.';
      $('message').hidden = false;
      return;
    }
    if (canvas.width !== dims.width) canvas.width = dims.width;
    if (canvas.height !== dims.height) canvas.height = dims.height;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    var msg = $('message');
    if (!cam || !cam.accepted) {
      msg.textContent = 'No ' + (state.camera === 'rgb' ? 'RGB' : 'thermal') + ' image was accepted for this scan' +
        (cam ? ' (Δt ' + fmt(cam.delta_t_ns / 1e6, 2) + ' ms is outside the association threshold).' : '.');
      msg.hidden = false;
      return;
    }
    msg.hidden = true;
    if (cam.image) {
      var img = image(cam.image);
      if (img.complete && img.naturalWidth > 0) ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
      else if (img.failed) { msg.textContent = 'Image ' + cam.image + ' could not be loaded.'; msg.hidden = false; }
    }
    drawPoints(cam);
  }

  function updateLegend() {
    var stops = [];
    for (var i = 0; i <= 8; i++) stops.push(LUT[state.color][Math.round(255 * i / 8)]);
    $('legend-bar').style.background = 'linear-gradient(to right,' + stops.join(',') + ')';
    var r = state.range[state.color];
    $('legend-lo').textContent = r[0] + ' ' + unit();
    $('legend-hi').textContent = r[1] + ' ' + unit();
    $('range-label').textContent = (state.color === 'depth' ? 'Depth' : 'Thermal') + ' (' + unit() + ')';
    $('range-min').value = r[0];
    $('range-max').value = r[1];
  }

  function updatePanel() {
    var f = frames[state.frame];
    $('frame-label').textContent = (state.frame + 1) + ' / ' + frames.length;
    $('frame').value = state.frame;
    var ms = run.frame_ms;
    fillDl($('frame-info'), [
      ['Scan', '#' + f.index],
      ['t_ref (ns)', String(f.t_ref_ns)],
      ['Scan points', String(f.points)],
      ['Processing', fmt(ms.process[state.frame], 3) + ' ms'],
      ['Read / write', fmt(ms.read[state.frame], 2) + ' / ' + fmt(ms.write[state.frame], 2) + ' ms']
    ]);
    var rows = $('camera-rows');
    rows.textContent = '';
    var notes = [];
    ['rgb', 'thermal'].forEach(function (key) {
      var c = f[key];
      if (!c) return;
      var tr = document.createElement('tr');
      tr.appendChild(el('td', key === 'rgb' ? 'RGB' : 'Thermal'));
      var dt = el('td', fmt(c.delta_t_ns / 1e6, 2), 'num');
      dt.title = c.delta_t_ns + ' ns, image #' + c.match_index;
      tr.appendChild(dt);
      tr.appendChild(el('td', c.accepted ? 'yes' : 'no', c.accepted ? 'yes' : 'no'));
      tr.appendChild(el('td', c.accepted ? (c.compensated ? 'yes' : 'no') : '–'));
      var pts = el('td', String(c.projected), 'num');
      pts.title = c.hidden + ' hidden by the z-buffer; ' + c.embedded + ' embedded in this page';
      tr.appendChild(pts);
      rows.appendChild(tr);
      if (c.subsampled) notes.push((key === 'rgb' ? 'RGB' : 'Thermal') + ': showing ' + c.embedded + ' of ' + c.projected + ' points');
      else if (c.hidden) notes.push((key === 'rgb' ? 'RGB' : 'Thermal') + ': ' + c.hidden + ' of ' + c.projected + ' points hidden by the z-buffer');
    });
    var note = $('subsample-note');
    note.textContent = notes.join('. ') + (notes.length ? '.' : '');
    note.hidden = notes.length === 0;
  }

  function setFrame(i) {
    state.frame = Math.max(0, Math.min(frames.length - 1, i));
    updatePanel();
    draw();
  }

  function init() {
    $('sequence').textContent = data.sequence;
    $('frame').max = Math.max(0, frames.length - 1);
    $('cam-rgb').disabled = !hasRgb;
    $('cam-thermal').disabled = !hasThermal;
    $('color-thermal').disabled = !hasThermal;
    $('cam-' + state.camera).checked = true;
    fillDl($('run-info'), [
      ['Frames', String(run.frames)],
      ['Processing median', fmt(run.process_ms.median, 3) + ' ms'],
      ['Processing p95', fmt(run.process_ms.p95, 3) + ' ms'],
      ['Reading median / p95', fmt(run.read_ms.median, 2) + ' / ' + fmt(run.read_ms.p95, 2) + ' ms']
    ]);
    var anySub = frames.some(function (f) {
      return (f.rgb && f.rgb.subsampled) || (f.thermal && f.thermal.subsampled);
    });
    $('cap-note').textContent = 'Processing time covers build_aligned_frame() only (compensation, projection, z-buffer). ' +
      'At most ' + data.max_points_per_camera + ' points per camera and frame are embedded; when a camera sees m more ' +
      'than that, the k-th point kept is the one at rank floor(k·m/' + data.max_points_per_camera +
      ') among its visible points in scan order. ' + (anySub ? 'Some frames of this run are subsampled.' : 'No frame of this run is subsampled.');

    $('prev').addEventListener('click', function () { setFrame(state.frame - 1); });
    $('next').addEventListener('click', function () { setFrame(state.frame + 1); });
    $('frame').addEventListener('input', function (e) { setFrame(Number(e.target.value)); });
    document.querySelectorAll('input[name=camera]').forEach(function (r) {
      r.addEventListener('change', function () { state.camera = r.value; draw(); });
    });
    document.querySelectorAll('input[name=color]').forEach(function (r) {
      r.addEventListener('change', function () { state.color = r.value; updateLegend(); draw(); });
    });
    $('size').addEventListener('input', function (e) { state.size = Number(e.target.value); $('size-out').textContent = state.size + ' px'; draw(); });
    $('opacity').addEventListener('input', function (e) { state.opacity = Number(e.target.value); $('opacity-out').textContent = state.opacity.toFixed(2); draw(); });
    $('hidden').addEventListener('change', function (e) { state.showHidden = e.target.checked; draw(); });
    function rangeChanged() {
      var lo = Number($('range-min').value), hi = Number($('range-max').value);
      if (isFinite(lo) && isFinite(hi) && hi > lo) {
        state.range[state.color] = [lo, hi];
        updateLegend();
        draw();
      }
    }
    $('range-min').addEventListener('change', rangeChanged);
    $('range-max').addEventListener('change', rangeChanged);
    $('range-auto').addEventListener('click', function () {
      state.range[state.color] = autoRange[state.color].slice(); updateLegend(); draw();
    });
    document.addEventListener('keydown', function (e) {
      // Leave the keys to controls that use them (sliders, number fields,
      // radio groups); everywhere else they change frames.
      var t = e.target, tag = t && t.tagName;
      if (tag === 'SELECT' || tag === 'TEXTAREA') return;
      if (tag === 'INPUT' && t.type !== 'checkbox' && t.type !== 'button') return;
      if (e.altKey || e.ctrlKey || e.metaKey) return;
      var step = { ArrowRight: 1, ArrowDown: 1, ArrowLeft: -1, ArrowUp: -1 }[e.key];
      if (step) { setFrame(state.frame + step); e.preventDefault(); }
      else if (e.key === 'Home') { setFrame(0); e.preventDefault(); }
      else if (e.key === 'End') { setFrame(frames.length - 1); e.preventDefault(); }
    });

    $('size-out').textContent = state.size + ' px';
    $('opacity-out').textContent = state.opacity.toFixed(2);
    updateLegend();
    if (frames.length === 0) { $('message').textContent = 'This run has no frames.'; $('message').hidden = false; return; }
    setFrame(0);
  }
  init();
})();
</script>
</body>
</html>
)HTML";

}  // namespace fuse_cli
