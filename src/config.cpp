#include "fusion/config.hpp"

#include <charconv>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <system_error>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace fusion {

namespace {

// Tolerance on max |R^T R - I| for an accepted rotation. Loose enough for
// matrices written with six decimals; the accepted rotation is then
// re-orthonormalized.
constexpr double kOrthonormalTol = 1e-5;
// |det R| below this counts as not invertible.
constexpr double kSingularDet = 1e-9;

class Parser {
public:
    explicit Parser(std::string origin) : origin_(std::move(origin)) {}

    [[noreturn]] void fail(const std::string& field, const std::string& what,
                           const YAML::Node* at = nullptr) const {
        std::string msg = origin_ + ": ";
        if (!field.empty()) msg += field + ": ";
        msg += what;
        if (at != nullptr && at->IsDefined() && !at->Mark().is_null()) {
            msg += " (line " + std::to_string(at->Mark().line + 1) + ")";
        }
        throw ConfigError(msg);
    }

    // Child `key` of a map node; error if absent or empty.
    YAML::Node child(const YAML::Node& parent, const std::string& parent_path,
                     const std::string& key) const {
        const std::string path = parent_path.empty() ? key : parent_path + "." + key;
        if (!parent.IsMap()) fail(parent_path, "expected a map with field '" + key + "'", &parent);
        const YAML::Node node = parent[key];
        if (!node.IsDefined()) fail(path, "missing required field");
        if (node.IsNull()) fail(path, "missing value", &node);
        return node;
    }

    std::string text(const YAML::Node& node, const std::string& path) const {
        if (!node.IsScalar()) fail(path, "expected a string", &node);
        const std::string s = node.Scalar();
        if (s.empty()) fail(path, "must not be empty", &node);
        return s;
    }

    double number(const YAML::Node& node, const std::string& path) const {
        if (!node.IsScalar()) fail(path, "expected a number", &node);
        double value = 0.0;
        if (!YAML::convert<double>::decode(node, value)) {
            fail(path, "expected a number, got '" + node.Scalar() + "'", &node);
        }
        if (!std::isfinite(value)) fail(path, "must be finite", &node);
        return value;
    }

    // Decimal integer only: no fraction, exponent, or hex/octal prefixes.
    std::int64_t integer(const YAML::Node& node, const std::string& path) const {
        if (!node.IsScalar()) fail(path, "expected an integer", &node);
        const std::string& s = node.Scalar();
        const char* first = s.data();
        const char* last = s.data() + s.size();
        if (first != last && *first == '+') ++first;
        std::int64_t value = 0;
        const auto [ptr, ec] = std::from_chars(first, last, value);
        if (ec == std::errc::result_out_of_range) {
            fail(path, "integer '" + s + "' is out of range", &node);
        }
        if (ec != std::errc() || ptr != last || first == last) {
            fail(path, "expected an integer, got '" + s + "'", &node);
        }
        return value;
    }

    std::vector<double> numbers(const YAML::Node& node, const std::string& path,
                                std::size_t n) const {
        const std::string want = "expected a list of " + std::to_string(n) + " numbers";
        if (!node.IsSequence()) fail(path, want, &node);
        if (node.size() != n) {
            fail(path, want + ", got " + std::to_string(node.size()), &node);
        }
        std::vector<double> out;
        for (std::size_t i = 0; i < n; ++i) {
            out.push_back(number(node[i], path + "[" + std::to_string(i) + "]"));
        }
        return out;
    }

    Eigen::Matrix3d matrix3(const YAML::Node& node, const std::string& path) const {
        const std::string want = "expected a 3x3 matrix as 3 rows of 3 numbers";
        if (!node.IsSequence()) fail(path, want, &node);
        if (node.size() != 3) fail(path, want + ", got " + std::to_string(node.size()) + " rows", &node);
        Eigen::Matrix3d m;
        for (std::size_t r = 0; r < 3; ++r) {
            const YAML::Node row = node[r];
            const std::string row_path = path + "[" + std::to_string(r) + "]";
            if (!row.IsSequence() || row.size() != 3) {
                fail(row_path,
                     want + ", row " + std::to_string(r) + " has " +
                         (row.IsSequence() ? std::to_string(row.size()) : std::string("no")) +
                         " entries",
                     &row);
            }
            for (std::size_t c = 0; c < 3; ++c) {
                m(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) =
                    number(row[c], row_path + "[" + std::to_string(c) + "]");
            }
        }
        return m;
    }

private:
    std::string origin_;
};

std::string fmt(double v) {
    std::ostringstream os;
    os.precision(6);
    os << v;
    return os.str();
}

SensorConfig parse(const YAML::Node& root, const Parser& p) {
    if (!root.IsMap()) p.fail("", "expected a YAML map at the top level", &root);
    SensorConfig cfg;

    cfg.name = p.text(p.child(root, "", "name"), "name");

    {
        const YAML::Node res = p.child(root, "", "resolution");
        if (!res.IsSequence() || res.size() != 2) {
            p.fail("resolution", "expected [width, height]", &res);
        }
        const std::int64_t w = p.integer(res[0], "resolution[0]");
        const std::int64_t h = p.integer(res[1], "resolution[1]");
        constexpr std::int64_t kMax = 1 << 20;
        if (w <= 0 || h <= 0 || w > kMax || h > kMax) {
            p.fail("resolution", "width and height must be in [1, " + std::to_string(kMax) + "]",
                   &res);
        }
        cfg.width = static_cast<int>(w);
        cfg.height = static_cast<int>(h);
    }

    {
        const YAML::Node intr = p.child(root, "", "intrinsics");
        const YAML::Node k_node = p.child(intr, "intrinsics", "K");
        const Eigen::Matrix3d K = p.matrix3(k_node, "intrinsics.K");
        if (!(K(0, 0) > 0.0) || !(K(1, 1) > 0.0)) {
            p.fail("intrinsics.K", "fx = K[0][0] and fy = K[1][1] must be positive", &k_node);
        }
        if (K(1, 0) != 0.0 || K(2, 0) != 0.0 || K(2, 1) != 0.0 || K(2, 2) != 1.0) {
            p.fail("intrinsics.K", "expected the form [[fx, s, cx], [0, fy, cy], [0, 0, 1]]",
                   &k_node);
        }
        cfg.K = K;
    }

    {
        const auto d = p.numbers(p.child(root, "", "distortion"), "distortion", 5);
        cfg.distortion = {d[0], d[1], d[2], d[3], d[4]};
    }

    {
        const YAML::Node ext = p.child(root, "", "extrinsics");
        cfg.source_frame = p.text(p.child(ext, "extrinsics", "source"), "extrinsics.source");
        const YAML::Node r_node = p.child(ext, "extrinsics", "rotation");
        const Eigen::Matrix3d R = p.matrix3(r_node, "extrinsics.rotation");
        const double det = R.determinant();
        if (!(std::abs(det) >= kSingularDet)) {
            p.fail("extrinsics.rotation", "matrix is not invertible (determinant " + fmt(det) + ")",
                   &r_node);
        }
        if (det < 0.0) {
            p.fail("extrinsics.rotation",
                   "determinant is " + fmt(det) + "; a reflection is not a rotation", &r_node);
        }
        const double ortho_err =
            (R.transpose() * R - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff();
        if (ortho_err > kOrthonormalTol) {
            p.fail("extrinsics.rotation",
                   "matrix is not orthonormal (max |R^T R - I| = " + fmt(ortho_err) + ")",
                   &r_node);
        }
        const auto t = p.numbers(p.child(ext, "extrinsics", "translation"),
                                 "extrinsics.translation", 3);
        cfg.T_sensor_source.linear() = Eigen::Quaterniond(R).normalized().toRotationMatrix();
        cfg.T_sensor_source.translation() = Eigen::Vector3d(t[0], t[1], t[2]);
    }

    cfg.time_offset_ns = p.integer(p.child(root, "", "time_offset_ns"), "time_offset_ns");
    return cfg;
}

}  // namespace

SensorConfig parse_sensor_config(const std::string& yaml_text, const std::string& origin) {
    const Parser p(origin);
    YAML::Node root;
    try {
        root = YAML::Load(yaml_text);
    } catch (const YAML::Exception& e) {
        p.fail("", std::string("invalid YAML: ") + e.what());
    }
    return parse(root, p);
}

SensorConfig load_sensor_config(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw ConfigError(path + ": cannot open file");
    std::ostringstream buf;
    buf << in.rdbuf();
    if (in.bad()) throw ConfigError(path + ": cannot read file");
    return parse_sensor_config(buf.str(), path);
}

CameraModel make_camera_model(const SensorConfig& config) {
    return CameraModel(config.width, config.height, config.K, config.distortion);
}

}  // namespace fusion
