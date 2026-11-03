#include "synthetic_sequence.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <fusion/sequence.hpp>

namespace fusion::testing {

namespace {

namespace fs = std::filesystem;

constexpr std::int64_t kMs = 1'000'000;
constexpr double kMaxDistance = 100.0;
constexpr std::uint16_t kThermalBackground = 8000;

std::string num(double v) {
    char buf[64];
    const auto r = std::to_chars(buf, buf + sizeof(buf), v);
    return std::string(buf, r.ptr);
}

std::string numbered(std::size_t i, const char* ext) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%06zu%s", i, ext);
    return buf;
}

// Nearest stamp by brute force, earlier one on a tie.
ExpectedMatch nearest(const std::vector<std::int64_t>& stamps, std::int64_t t, std::int64_t tau) {
    ExpectedMatch best;
    bool have = false;
    for (std::size_t j = 0; j < stamps.size(); ++j) {
        const std::int64_t d = stamps[j] - t;
        if (!have || std::llabs(d) < std::llabs(best.delta_t_ns)) {
            best = {j, stamps[j], d, false};
            have = true;
        }
    }
    best.accepted = std::llabs(best.delta_t_ns) <= tau;
    return best;
}

fusion::SensorConfig camera_config(const std::string& name, const PinholeIntrinsics& K,
                                   const Rigid3& T_camera_lidar, std::int64_t offset_ns) {
    fusion::SensorConfig c;
    c.name = name;
    c.width = K.width;
    c.height = K.height;
    c.K << K.fx, 0.0, K.cx, 0.0, K.fy, K.cy, 0.0, 0.0, 1.0;
    c.source_frame = "lidar";
    c.T_sensor_source = T_camera_lidar;
    c.time_offset_ns = offset_ns;
    return c;
}

// Flat colour per surface, shaded by the angle to a fixed light, and a sky
// colour where nothing is hit.
fusion::RgbImage render_rgb(const Scene& scene, const PinholeCamera& cam) {
    static const fusion::Rgb8 palette[] = {
        {120, 110, 100}, {200, 60, 50}, {60, 160, 70}, {70, 90, 200}, {220, 190, 60}};
    const Vec3 light = Vec3(0.3, 0.5, 1.0).normalized();
    fusion::RgbImage img{cam.K.width, cam.K.height, {}};
    img.pixels.reserve(static_cast<std::size_t>(cam.K.width) * static_cast<std::size_t>(cam.K.height));
    for (int row = 0; row < cam.K.height; ++row) {
        for (int col = 0; col < cam.K.width; ++col) {
            const Ray ray = cam.pixel_ray_world(col, row);
            const auto hit = scene.raycast(ray.origin, ray.direction, kMaxDistance);
            if (!hit) {
                img.pixels.push_back({150, 190, 230});
                continue;
            }
            const fusion::Rgb8 base = palette[static_cast<std::size_t>(hit->surface) % 5];
            const double shade = 0.55 + 0.45 * std::abs(hit->normal.dot(light));
            const auto s = [&](std::uint8_t v) {
                return static_cast<std::uint8_t>(std::lround(shade * v));
            };
            img.pixels.push_back({s(base.r), s(base.g), s(base.b)});
        }
    }
    return img;
}

std::string matrix_yaml(const Eigen::Matrix3d& m) {
    std::string s = "[";
    for (int r = 0; r < 3; ++r) {
        s += r ? ", [" : "[";
        for (int c = 0; c < 3; ++c) s += (c ? ", " : "") + num(m(r, c));
        s += "]";
    }
    return s + "]";
}

std::string vector_yaml(const Eigen::Vector3d& v) {
    return "[" + num(v.x()) + ", " + num(v.y()) + ", " + num(v.z()) + "]";
}

void write_file(const std::string& path, const std::string& text) {
    std::ofstream out(path, std::ios::binary);
    out << text;
    out.flush();
    if (!out) throw std::runtime_error("cannot write " + path);
}

std::string config_yaml(const fusion::SensorConfig& c) {
    std::ostringstream y;
    y << "# Synthetic " << c.name << " camera of the generated test sequence.\n"
      << "name: " << c.name << "\n"
      << "resolution: [" << c.width << ", " << c.height << "]\n"
      << "intrinsics:\n  K: " << matrix_yaml(c.K) << "\n"
      << "distortion: [0, 0, 0, 0, 0]\n"
      << "extrinsics:\n  source: " << c.source_frame << "\n"
      << "  rotation: " << matrix_yaml(c.T_sensor_source.linear()) << "\n"
      << "  translation: " << vector_yaml(c.T_sensor_source.translation()) << "\n"
      << "time_offset_ns: " << c.time_offset_ns << "\n";
    if (c.thermal) {
        const fusion::PlanckParams& p = c.thermal->planck;
        y << "thermal:\n  planck: {R: " << num(p.R) << ", B: " << num(p.B) << ", F: " << num(p.F)
          << ", O: " << num(p.O) << "}\n";
    }
    return y.str();
}

}  // namespace

SyntheticSequence make_synthetic_sequence(int scans) {
    if (scans < 1 || scans > 5) {
        throw std::invalid_argument("make_synthetic_sequence: scans must be in [1, 5]");
    }
    const auto n = static_cast<std::size_t>(scans);
    SyntheticSequence s;
    s.scene = Scene::reference();
    s.rig = make_moving_rig();
    const MovingRig& rig = s.rig;

    std::vector<std::int64_t> t_l;
    for (std::size_t k = 0; k < n; ++k) {
        t_l.push_back(rig.t_lidar_ns - 100 * kMs * static_cast<std::int64_t>(n - 1 - k));
    }

    // LiDAR scans with a deterministic intensity per surface and range.
    for (std::size_t k = 0; k < n; ++k) {
        fusion::LidarScan out;
        out.t_ns = t_l[k];
        std::vector<int> surfaces;
        for (const LidarPoint& lp : scan(s.scene, rig.lidar_at(t_l[k]))) {
            out.points.push_back(lp.p_lidar);
            out.intensity.push_back(
                static_cast<float>((10.0 + 15.0 * lp.surface) / (1.0 + 0.05 * lp.range)));
            surfaces.push_back(lp.surface);
        }
        s.scans.push_back(std::move(out));
        s.scan_surfaces.push_back(std::move(surfaces));
    }

    // Cameras.
    s.rgb.config = camera_config("rgb", rig.camera_K, rig.T_camera_lidar(), -2 * kMs);
    s.thermal.config = camera_config("thermal", rig.thermal_K, rig.T_thermal_lidar(), 5 * kMs);
    s.thermal.config.thermal = fusion::ThermalCalibration{{380747.0, 1428.0, 1.0, -88.539}};
    for (std::size_t j = 0; j < 2 * n; ++j) {
        s.rgb.stamps_ref_ns.push_back(t_l[0] + 33 * kMs + 50 * kMs * static_cast<std::int64_t>(j));
    }
    for (std::size_t k = 0; k < n; ++k) {
        if (k == 3) continue;  // dropped frame
        s.thermal.stamps_ref_ns.push_back(t_l[k] + 21 * kMs);
    }
    s.rgb.tau_ns = 25 * kMs;
    s.rgb.tau_in_manifest = true;
    s.thermal.tau_ns = 50 * kMs;  // default: half the 100 ms median period
    s.thermal.tau_in_manifest = false;
    for (SyntheticCamera* cam : {&s.rgb, &s.thermal}) {
        for (std::int64_t t : cam->stamps_ref_ns) {
            cam->stamps_sensor_ns.push_back(t - cam->config.time_offset_ns);
        }
        for (std::int64_t t : t_l) cam->expected.push_back(nearest(cam->stamps_ref_ns, t, cam->tau_ns));
    }

    for (std::int64_t t : s.rgb.stamps_ref_ns) s.rgb_images.push_back(render_rgb(s.scene, rig.camera_at(t)));
    for (std::int64_t t : s.thermal.stamps_ref_ns) {
        const PinholeCamera cam = rig.thermal_at(t);
        s.thermal_images.emplace_back(cam.K.width, cam.K.height,
                                      cam.render_raw(s.scene, kMaxDistance, kThermalBackground));
    }

    // Trajectory every 40 ms, covering every scan and image time.
    for (std::int64_t t = t_l.front() - 100 * kMs; t <= t_l.back() + 100 * kMs; t += 40 * kMs) {
        const Rigid3 T = rig.body.T_world_body(t);
        s.poses.push_back({t, T.translation(), Eigen::Quaterniond(T.linear())});
    }
    return s;
}

std::string write_synthetic_sequence(const SyntheticSequence& s, const std::string& dir) {
    const fs::path root(dir);
    for (const char* sub : {"lidar", "rgb", "thermal"}) fs::create_directories(root / sub);

    std::vector<fusion::StampedPath> lidar, rgb, thermal;
    for (std::size_t k = 0; k < s.scans.size(); ++k) {
        const std::string rel = "lidar/" + numbered(k, ".bin");
        fusion::write_lidar_bin((root / rel).string(), s.scans[k].points, s.scans[k].intensity);
        lidar.push_back({s.scans[k].t_ns - s.lidar_time_offset_ns, rel});
    }
    for (std::size_t j = 0; j < s.rgb_images.size(); ++j) {
        const std::string rel = "rgb/" + numbered(j, ".png");
        fusion::write_rgb_png((root / rel).string(), s.rgb_images[j]);
        rgb.push_back({s.rgb.stamps_sensor_ns[j], rel});
    }
    for (std::size_t j = 0; j < s.thermal_images.size(); ++j) {
        const std::string rel = "thermal/" + numbered(j, ".png");
        fusion::write_thermal_png((root / rel).string(), s.thermal_images[j]);
        thermal.push_back({s.thermal.stamps_sensor_ns[j], rel});
    }
    fusion::write_list_csv((root / "lidar.csv").string(), lidar);
    fusion::write_list_csv((root / "rgb.csv").string(), rgb);
    fusion::write_list_csv((root / "thermal.csv").string(), thermal);
    fusion::write_trajectory_csv((root / "trajectory.csv").string(), s.poses);
    write_file((root / "rgb.yaml").string(), config_yaml(s.rgb.config));
    write_file((root / "thermal.yaml").string(), config_yaml(s.thermal.config));

    std::ostringstream m;
    m << "# Synthetic sequence generated from the moving test rig. Not a dataset.\n"
      << "format_version: 1\n"
      << "trajectory: trajectory.csv\n"
      << "lidar:\n"
      << "  name: lidar\n"
      << "  list: lidar.csv\n"
      << "  time_offset_ns: " << s.lidar_time_offset_ns << "\n"
      << "  T_body_lidar:\n"
      << "    rotation: " << matrix_yaml(s.rig.T_body_lidar.linear()) << "\n"
      << "    translation: " << vector_yaml(s.rig.T_body_lidar.translation()) << "\n"
      << "cameras:\n"
      << "  rgb:\n    config: rgb.yaml\n    list: rgb.csv\n";
    if (s.rgb.tau_in_manifest) m << "    tau_ns: " << s.rgb.tau_ns << "\n";
    m << "  thermal:\n    config: thermal.yaml\n    list: thermal.csv\n";
    if (s.thermal.tau_in_manifest) m << "    tau_ns: " << s.thermal.tau_ns << "\n";
    const std::string manifest = (root / "sequence.yaml").string();
    write_file(manifest, m.str());
    return manifest;
}

}  // namespace fusion::testing
