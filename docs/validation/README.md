# Validation

| Check | Data | Headline result | Page |
|-------|------|-----------------|------|
| Synthetic ground truth | test scene, moving rig, five-scan sequence (unit tests) | After compensation 964 of 994 projected points (all 493 box points) match ray-cast depth to 1e-9 relative, against 0 of 493 box points before. In every accepted image of the sequence ≥ 96% of points match to 1e-9 relative depth. | [building.md](../building.md#synthetic-test-scene) |
| A2D2 | 50 frames, 866,142 points, 5 LiDARs | Engine pixel equals A2D2's to ≤ 1.1e-4 px and depth to ≤ 3.8e-6 m (float32 storage). The five-LiDAR extrinsic chain agrees with the direct projection to 1.9e-4 px. | [a2d2.md](a2d2.md) |
| RGBDT LAENTIEC | 312 thermal frames, 95.8 M pixels | Linear (centi-kelvin) thermal encoding reproduces every pixel to ≤ 3e-6 °C over −24.8 °C to 29.8 °C. Rendering matches an independent reference. | [laentiec.md](laentiec.md) |
| Boreas | 600 scans, 62 s, moving vehicle | All 600 scans associated (median \|Δt\| 24.5 ms). Pose interpolation error median 0.83 mm / 0.019°. Projection with the given calibration is 5.04 px from image edges. Scan-level compensation runs but does not help (10.63 → 10.67 px), because the camera fires as the LiDAR sweeps past it. | [boreas.md](boreas.md) |

Each page says what the check proves and what it does not. No dataset files
or images derived from them are in this repository.
