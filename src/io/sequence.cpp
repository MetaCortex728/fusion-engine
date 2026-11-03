#include "fusion/sequence.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>
#include <system_error>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace fusion {

namespace fs = std::filesystem;

namespace {

constexpr double kOrthonormalTol = 1e-5;  // same as the sensor config loader

[[noreturn]] void fail(const std::string& where, const std::string& what) {
    throw SequenceError(where + ": " + what);
}

std::string at_line(const std::string& path, std::size_t line) {
    return path + ":" + std::to_string(line);
}

std::string resolve(const std::string& base_dir, const std::string& p) {
    const fs::path path(p);
    if (path.is_absolute() || base_dir.empty()) return path.lexically_normal().string();
    return (fs::path(base_dir) / path).lexically_normal().string();
}

void require_file(const std::string& where, const std::string& path) {
    std::error_code ec;
    if (!fs::is_regular_file(path, ec)) fail(where, "missing file " + path);
}

// Reads a text file into lines, without trailing '\r'.
std::vector<std::string> read_lines(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) fail(path, "cannot open file (missing file?)");
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        lines.push_back(line);
    }
    if (in.bad()) fail(path, "cannot read file");
    return lines;
}

std::vector<std::string> split(const std::string& s, char sep) {
    std::vector<std::string> out;
    std::size_t start = 0;
    while (true) {
        const std::size_t pos = s.find(sep, start);
        out.push_back(s.substr(start, pos - start));
        if (pos == std::string::npos) break;
        start = pos + 1;
    }
    return out;
}

bool parse_int(const std::string& s, TimeNs& out) {
    const char* first = s.data();
    const char* last = s.data() + s.size();
    const auto [ptr, ec] = std::from_chars(first, last, out);
    return ec == std::errc() && ptr == last && first != last;
}

bool parse_double(const std::string& s, double& out) {
    const char* first = s.data();
    const char* last = s.data() + s.size();
    const auto [ptr, ec] = std::from_chars(first, last, out);
    return ec == std::errc() && ptr == last && first != last && std::isfinite(out);
}

TimeNs parse_stamp(const std::string& where, const std::string& s) {
    TimeNs t = 0;
    if (!parse_int(s, t)) fail(where, "timestamp_ns '" + s + "' is not an integer");
    return t;
}

void check_increasing(const std::string& where, TimeNs prev, TimeNs t) {
    if (t == prev) fail(where, "duplicate timestamp " + std::to_string(t));
    if (t < prev) {
        fail(where, "unsorted timestamps: " + std::to_string(t) + " after " + std::to_string(prev));
    }
}

template <typename Writer>
void write_text(const std::string& path, Writer&& writer) {
    std::ofstream out(path, std::ios::binary);
    if (!out) fail(path, "cannot open file for writing");
    writer(out);
    out.flush();
    if (!out) fail(path, "cannot write file");
}

template <typename T>
void put(std::ostream& out, T value) {
    char buf[64];
    const auto r = std::to_chars(buf, buf + sizeof(buf), value);
    if (r.ec != std::errc()) throw std::runtime_error("number format");
    out.write(buf, r.ptr - buf);
}

TimeNs median_period(const std::vector<StampedPath>& s) {
    std::vector<TimeNs> d;
    for (std::size_t i = 1; i < s.size(); ++i) d.push_back(s[i].t_ns - s[i - 1].t_ns);
    std::nth_element(d.begin(), d.begin() + static_cast<std::ptrdiff_t>((d.size() - 1) / 2),
                     d.end());
    return d[(d.size() - 1) / 2];
}

std::vector<TimeNs> stamps_of(const std::vector<StampedPath>& s) {
    std::vector<TimeNs> out;
    out.reserve(s.size());
    for (const StampedPath& e : s) out.push_back(e.t_ns);
    return out;
}

// --- manifest ----------------------------------------------------------------

class Manifest {
public:
    explicit Manifest(std::string path) : path_(std::move(path)) {}

    [[noreturn]] void fail_at(const std::string& field, const std::string& what,
                              const YAML::Node* node = nullptr) const {
        std::string msg = field.empty() ? what : field + ": " + what;
        if (node != nullptr && node->IsDefined() && !node->Mark().is_null()) {
            msg += " (line " + std::to_string(node->Mark().line + 1) + ")";
        }
        fail(path_, msg);
    }

    void only_keys(const YAML::Node& map, const std::string& field,
                   const std::set<std::string>& allowed) const {
        if (!map.IsMap()) fail_at(field, "expected a map", &map);
        for (const auto& kv : map) {
            const std::string key = kv.first.as<std::string>();
            if (allowed.count(key) == 0) {
                fail_at(field.empty() ? key : field + "." + key, "unknown field", &kv.first);
            }
        }
    }

    // Field `key` of a map; nullopt if it is absent or null.
    std::optional<YAML::Node> optional(const YAML::Node& map, const std::string& key) const {
        const YAML::Node node = map[key];
        if (!node.IsDefined() || node.IsNull()) return std::nullopt;
        return node;
    }

    YAML::Node required(const YAML::Node& map, const std::string& field,
                        const std::string& key) const {
        auto node = optional(map, key);
        if (!node) fail_at(field.empty() ? key : field + "." + key, "missing required field");
        return *node;
    }

    std::string text(const YAML::Node& node, const std::string& field) const {
        if (!node.IsScalar() || node.Scalar().empty()) fail_at(field, "expected a string", &node);
        return node.Scalar();
    }

    TimeNs integer(const YAML::Node& node, const std::string& field) const {
        TimeNs v = 0;
        if (!node.IsScalar() || !parse_int(node.Scalar(), v)) {
            fail_at(field, "expected an integer", &node);
        }
        return v;
    }

    double number(const YAML::Node& node, const std::string& field) const {
        double v = 0.0;
        if (!node.IsScalar() || !parse_double(node.Scalar(), v)) {
            fail_at(field, "expected a finite number", &node);
        }
        return v;
    }

    Eigen::Isometry3d rigid(const YAML::Node& node, const std::string& field) const {
        only_keys(node, field, {"rotation", "translation"});
        const YAML::Node r = required(node, field, "rotation");
        const YAML::Node t = required(node, field, "translation");
        Eigen::Matrix3d R;
        if (!r.IsSequence() || r.size() != 3) fail_at(field + ".rotation", "expected 3 rows", &r);
        for (std::size_t i = 0; i < 3; ++i) {
            if (!r[i].IsSequence() || r[i].size() != 3) {
                fail_at(field + ".rotation", "expected 3 rows of 3 numbers", &r);
            }
            for (std::size_t j = 0; j < 3; ++j) {
                R(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) =
                    number(r[i][j], field + ".rotation");
            }
        }
        const double err = (R.transpose() * R - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff();
        if (err > kOrthonormalTol || R.determinant() <= 0.0) {
            fail_at(field + ".rotation", "not a rotation matrix", &r);
        }
        if (!t.IsSequence() || t.size() != 3) fail_at(field + ".translation", "expected 3 numbers", &t);
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear() = Eigen::Quaterniond(R).normalized().toRotationMatrix();
        T.translation() = Eigen::Vector3d(number(t[0], field + ".translation"),
                                          number(t[1], field + ".translation"),
                                          number(t[2], field + ".translation"));
        return T;
    }

    const std::string& path() const { return path_; }

private:
    std::string path_;
};

CameraStream load_camera(const Manifest& m, const YAML::Node& node, const std::string& field,
                         const std::string& base_dir, const std::string& lidar_name) {
    m.only_keys(node, field, {"config", "list", "tau_ns"});
    CameraStream cam;
    cam.config_path = resolve(base_dir, m.text(m.required(node, field, "config"), field + ".config"));
    require_file(m.path() + ": " + field + ".config", cam.config_path);
    try {
        cam.config = load_sensor_config(cam.config_path);
    } catch (const ConfigError& e) {
        throw SequenceError(e.what());
    }
    if (cam.config.source_frame != lidar_name) {
        fail(cam.config_path, "extrinsics.source is '" + cam.config.source_frame +
                                  "', expected the LiDAR frame '" + lidar_name + "'");
    }
    cam.list_path = resolve(base_dir, m.text(m.required(node, field, "list"), field + ".list"));
    require_file(m.path() + ": " + field + ".list", cam.list_path);
    cam.frames = read_list_csv(cam.list_path, base_dir);
    if (const auto tau = m.optional(node, "tau_ns")) {
        cam.tau_ns = m.integer(*tau, field + ".tau_ns");
        if (*cam.tau_ns < 0) m.fail_at(field + ".tau_ns", "must not be negative", &*tau);
    }
    return cam;
}

}  // namespace

// --- list files and trajectory ------------------------------------------------

std::vector<StampedPath> read_list_csv(const std::string& path, const std::string& base_dir) {
    const std::vector<std::string> lines = read_lines(path);
    if (lines.empty() || lines[0] != "timestamp_ns,path") {
        fail(at_line(path, 1), "expected the header 'timestamp_ns,path'");
    }
    std::vector<StampedPath> out;
    for (std::size_t i = 1; i < lines.size(); ++i) {
        const std::string& line = lines[i];
        if (line.empty()) continue;
        const std::string where = at_line(path, i + 1);
        const std::size_t comma = line.find(',');
        if (comma == std::string::npos || comma + 1 == line.size()) {
            fail(where, "expected 'timestamp_ns,path'");
        }
        StampedPath e;
        e.t_ns = parse_stamp(where, line.substr(0, comma));
        if (!out.empty()) check_increasing(where, out.back().t_ns, e.t_ns);
        e.path = resolve(base_dir, line.substr(comma + 1));
        require_file(where, e.path);
        out.push_back(std::move(e));
    }
    if (out.empty()) fail(path, "lists no files");
    return out;
}

Trajectory read_trajectory_csv(const std::string& path) {
    const std::vector<std::string> lines = read_lines(path);
    if (lines.empty() || lines[0] != "timestamp_ns,x,y,z,qw,qx,qy,qz") {
        fail(at_line(path, 1), "expected the header 'timestamp_ns,x,y,z,qw,qx,qy,qz'");
    }
    std::vector<StampedPose> poses;
    for (std::size_t i = 1; i < lines.size(); ++i) {
        if (lines[i].empty()) continue;
        const std::string where = at_line(path, i + 1);
        const std::vector<std::string> f = split(lines[i], ',');
        if (f.size() != 8) {
            fail(where, "expected 8 fields, got " + std::to_string(f.size()));
        }
        StampedPose p;
        p.t_ns = parse_stamp(where, f[0]);
        if (!poses.empty()) check_increasing(where, poses.back().t_ns, p.t_ns);
        std::array<double, 7> v{};
        for (std::size_t k = 0; k < 7; ++k) {
            if (!parse_double(f[k + 1], v[k])) {
                fail(where, "field " + std::to_string(k + 2) + " '" + f[k + 1] +
                                "' is not a finite number");
            }
        }
        p.position = {v[0], v[1], v[2]};
        p.orientation = Eigen::Quaterniond(v[3], v[4], v[5], v[6]);
        if (p.orientation.norm() < 1e-9) fail(where, "quaternion has zero norm");
        poses.push_back(p);
    }
    if (poses.empty()) fail(path, "lists no poses");
    try {
        return Trajectory(std::move(poses));
    } catch (const std::exception& e) {
        fail(path, e.what());
    }
}

void write_list_csv(const std::string& path, const std::vector<StampedPath>& entries) {
    write_text(path, [&](std::ostream& out) {
        out << "timestamp_ns,path\n";
        for (const StampedPath& e : entries) out << e.t_ns << ',' << e.path << '\n';
    });
}

void write_trajectory_csv(const std::string& path, const std::vector<StampedPose>& poses) {
    write_text(path, [&](std::ostream& out) {
        out << "timestamp_ns,x,y,z,qw,qx,qy,qz\n";
        for (const StampedPose& p : poses) {
            out << p.t_ns;
            const double v[7] = {p.position.x(),    p.position.y(),    p.position.z(),
                                 p.orientation.w(), p.orientation.x(), p.orientation.y(),
                                 p.orientation.z()};
            for (double x : v) {
                out << ',';
                put(out, x);
            }
            out << '\n';
        }
    });
}

// --- LiDAR ----------------------------------------------------------------------

namespace {

constexpr std::size_t kRecordBytes = 16;

float le_float(const unsigned char* b) {
    const std::uint32_t u = static_cast<std::uint32_t>(b[0]) |
                            (static_cast<std::uint32_t>(b[1]) << 8) |
                            (static_cast<std::uint32_t>(b[2]) << 16) |
                            (static_cast<std::uint32_t>(b[3]) << 24);
    float f = 0.0f;
    std::memcpy(&f, &u, sizeof(f));
    return f;
}

void put_le_float(unsigned char* b, float f) {
    std::uint32_t u = 0;
    std::memcpy(&u, &f, sizeof(u));
    for (int k = 0; k < 4; ++k) b[k] = static_cast<unsigned char>((u >> (8 * k)) & 0xFF);
}

}  // namespace

LidarScan read_lidar_bin(const std::string& path, TimeNs t_ns) {
    std::ifstream in(path, std::ios::binary);
    if (!in) fail(path, "cannot open file (missing file?)");
    std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(in)),
                                     std::istreambuf_iterator<char>());
    if (in.bad()) fail(path, "cannot read file");
    if (bytes.size() % kRecordBytes != 0) {
        fail(path, "size " + std::to_string(bytes.size()) +
                       " bytes is not a multiple of the 16-byte record (float32 x,y,z,intensity)");
    }
    const std::size_t n = bytes.size() / kRecordBytes;
    LidarScan scan;
    scan.t_ns = t_ns;
    scan.points.reserve(n);
    scan.intensity.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        const unsigned char* r = bytes.data() + i * kRecordBytes;
        const float x = le_float(r), y = le_float(r + 4), z = le_float(r + 8), v = le_float(r + 12);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(v)) {
            fail(path, "record " + std::to_string(i) + " holds a non-finite value");
        }
        scan.points.emplace_back(x, y, z);
        scan.intensity.push_back(v);
    }
    return scan;
}

void write_lidar_bin(const std::string& path, const std::vector<Eigen::Vector3d>& points,
                     const std::vector<float>& intensity) {
    if (points.size() != intensity.size()) {
        throw std::invalid_argument("write_lidar_bin: point and intensity counts differ");
    }
    std::vector<unsigned char> bytes(points.size() * kRecordBytes);
    for (std::size_t i = 0; i < points.size(); ++i) {
        unsigned char* r = bytes.data() + i * kRecordBytes;
        for (int k = 0; k < 3; ++k) put_le_float(r + 4 * k, static_cast<float>(points[i][k]));
        put_le_float(r + 12, intensity[i]);
    }
    write_text(path, [&](std::ostream& out) {
        out.write(reinterpret_cast<const char*>(bytes.data()),
                  static_cast<std::streamsize>(bytes.size()));
    });
}

// --- images -------------------------------------------------------------------

namespace {

void check_size(const std::string& path, int w, int h, const SensorConfig& config) {
    if (w != config.width || h != config.height) {
        fail(path, "image size " + std::to_string(w) + "x" + std::to_string(h) +
                       " does not match the sensor config '" + config.name + "' resolution " +
                       std::to_string(config.width) + "x" + std::to_string(config.height));
    }
}

}  // namespace

RgbImage read_rgb_frame(const std::string& path, const SensorConfig& config) {
    RgbImage img;
    try {
        img = read_rgb_png(path);
    } catch (const ImageError& e) {
        throw SequenceError(e.what());
    }
    check_size(path, img.width, img.height, config);
    return img;
}

RawThermalImage read_thermal_frame(const std::string& path, const SensorConfig& config) {
    try {
        RawThermalImage img = read_thermal_png(path);
        check_size(path, img.width(), img.height(), config);
        return img;
    } catch (const ImageError& e) {
        throw SequenceError(e.what());
    }
}

// --- sequence -------------------------------------------------------------------

Sequence load_sequence(const std::string& manifest_path) {
    const Manifest m(manifest_path);
    require_file(manifest_path, manifest_path);
    YAML::Node root;
    try {
        root = YAML::LoadFile(manifest_path);
    } catch (const YAML::Exception& e) {
        fail(manifest_path, std::string("invalid YAML: ") + e.what());
    }
    m.only_keys(root, "", {"format_version", "trajectory", "lidar", "cameras"});
    const YAML::Node version = m.required(root, "", "format_version");
    if (m.integer(version, "format_version") != 1) {
        m.fail_at("format_version", "unsupported version '" + version.Scalar() + "', expected 1",
                  &version);
    }
    const std::string base_dir = fs::path(manifest_path).parent_path().string();

    Sequence seq;
    seq.manifest_path = manifest_path;

    const YAML::Node lidar = m.required(root, "", "lidar");
    m.only_keys(lidar, "lidar", {"name", "list", "time_offset_ns", "T_body_lidar"});
    if (const auto n = m.optional(lidar, "name")) {
        seq.lidar_name = m.text(*n, "lidar.name");
    }
    seq.lidar_list_path = resolve(base_dir, m.text(m.required(lidar, "lidar", "list"), "lidar.list"));
    require_file(manifest_path + ": lidar.list", seq.lidar_list_path);
    seq.lidar = read_list_csv(seq.lidar_list_path, base_dir);
    if (const auto off = m.optional(lidar, "time_offset_ns")) {
        seq.lidar_time_offset_ns = m.integer(*off, "lidar.time_offset_ns");
    }
    const auto mount = m.optional(lidar, "T_body_lidar");
    if (mount) seq.T_body_lidar = m.rigid(*mount, "lidar.T_body_lidar");

    if (const auto traj = m.optional(root, "trajectory")) {
        seq.trajectory_path = resolve(base_dir, m.text(*traj, "trajectory"));
        require_file(manifest_path + ": trajectory", seq.trajectory_path);
        seq.trajectory = read_trajectory_csv(seq.trajectory_path);
        if (!mount) m.fail_at("lidar.T_body_lidar", "required when a trajectory is given");
    }

    if (const auto cams = m.optional(root, "cameras")) {
        m.only_keys(*cams, "cameras", {"rgb", "thermal"});
        if (const auto rgb = m.optional(*cams, "rgb")) {
            seq.rgb = load_camera(m, *rgb, "cameras.rgb", base_dir, seq.lidar_name);
        }
        if (const auto th = m.optional(*cams, "thermal")) {
            seq.thermal = load_camera(m, *th, "cameras.thermal", base_dir, seq.lidar_name);
        }
    }
    return seq;
}

LidarScan load_lidar_scan(const Sequence& sequence, std::size_t index) {
    const StampedPath& e = sequence.lidar.at(index);
    const std::vector<TimeNs> t = to_reference_time({e.t_ns}, sequence.lidar_time_offset_ns);
    return read_lidar_bin(e.path, t[0]);
}

TimeNs camera_tau_ns(const Sequence& sequence, const CameraStream& camera) {
    if (camera.tau_ns) return *camera.tau_ns;
    if (sequence.lidar.size() < 2 || camera.frames.size() < 2) {
        fail(sequence.manifest_path, "camera '" + camera.config.name +
                                         "': set tau_ns; the default needs at least two LiDAR "
                                         "scans and two camera frames");
    }
    return default_tau_ns(median_period(sequence.lidar), median_period(camera.frames));
}

std::vector<Association> associate_camera(const Sequence& sequence, const CameraStream& camera) {
    return associate(stamps_of(sequence.lidar), sequence.lidar_time_offset_ns,
                     stamps_of(camera.frames), camera.config.time_offset_ns,
                     camera_tau_ns(sequence, camera));
}

}  // namespace fusion
