#include <doctest.h>

#include <initializer_list>
#include <string>

#include <fusion/config.hpp>
#include <fusion/geometry.hpp>

#include "support/sensors.hpp"

using fusion::ConfigError;
using fusion::SensorConfig;

namespace {

const std::string kValid = R"(name: thermal
resolution: [320, 256]
intrinsics:
  K: [[320.0, 0.0, 159.5], [0.0, 321.0, 127.5], [0.0, 0.0, 1.0]]
distortion: [-0.1, 0.02, 0.001, -0.002, 0.0005]
extrinsics:
  source: lidar
  rotation: [[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]]
  translation: [0.1, -0.6, 0.05]
time_offset_ns: -2500000
)";

// kValid with one exact line replaced (or removed if `to` is empty).
std::string with(const std::string& from, const std::string& to) {
    std::string s = kValid;
    const auto pos = s.find(from + "\n");
    REQUIRE_MESSAGE(pos != std::string::npos, "line not in base config: ", from);
    s.replace(pos, from.size() + 1, to.empty() ? std::string() : to + "\n");
    return s;
}

// Parsing must fail with a ConfigError whose message contains every fragment.
void check_error(const std::string& yaml, std::initializer_list<const char*> fragments) {
    std::string message;
    try {
        fusion::parse_sensor_config(yaml, "test.yaml");
    } catch (const ConfigError& e) {
        message = e.what();
    }
    CAPTURE(message);
    REQUIRE_MESSAGE(!message.empty(), "expected a ConfigError");
    CHECK(message.rfind("test.yaml: ", 0) == 0);
    for (const char* f : fragments) {
        CHECK_MESSAGE(message.find(f) != std::string::npos, "missing fragment: ", f);
    }
}

}  // namespace

TEST_CASE("valid config parses every field") {
    const SensorConfig cfg = fusion::parse_sensor_config(kValid);
    CHECK(cfg.name == "thermal");
    CHECK(cfg.width == 320);
    CHECK(cfg.height == 256);
    CHECK(cfg.K(0, 0) == 320.0);
    CHECK(cfg.K(1, 1) == 321.0);
    CHECK(cfg.K(0, 2) == 159.5);
    CHECK(cfg.K(1, 2) == 127.5);
    CHECK(cfg.distortion.k1 == -0.1);
    CHECK(cfg.distortion.k2 == 0.02);
    CHECK(cfg.distortion.p1 == 0.001);
    CHECK(cfg.distortion.p2 == -0.002);
    CHECK(cfg.distortion.k3 == 0.0005);
    CHECK(cfg.source_frame == "lidar");
    // Lidar +x (forward) maps to camera +z (forward).
    const Eigen::Vector3d p = cfg.T_sensor_source * Eigen::Vector3d(1.0, 0.0, 0.0);
    CHECK((p - Eigen::Vector3d(0.1, -0.6, 1.05)).norm() < 1e-15);
    CHECK(cfg.time_offset_ns == -2500000);

    const fusion::CameraModel cam = fusion::make_camera_model(cfg);
    CHECK(cam.width() == 320);
    CHECK(cam.distortion().k3 == 0.0005);
}

TEST_CASE("example config matches the synthetic RGB camera and reference LiDAR") {
    const SensorConfig cfg = fusion::load_sensor_config(FUSION_SOURCE_DIR "/configs/rgb_camera.yaml");
    const fusion::testing::PinholeCamera rgb = fusion::testing::make_rgb_camera();
    const fusion::testing::LidarConfig lidar = fusion::testing::make_reference_lidar();
    CHECK(cfg.name == "rgb");
    CHECK(cfg.width == rgb.K.width);
    CHECK(cfg.height == rgb.K.height);
    CHECK(cfg.K(0, 0) == rgb.K.fx);
    CHECK(cfg.K(1, 1) == rgb.K.fy);
    CHECK(cfg.K(0, 2) == rgb.K.cx);
    CHECK(cfg.K(1, 2) == rgb.K.cy);
    CHECK(cfg.distortion.is_zero());
    CHECK(cfg.source_frame == "lidar");
    CHECK(cfg.time_offset_ns == 0);
    CHECK_FALSE(cfg.thermal.has_value());
    const Eigen::Isometry3d T_camera_lidar = rgb.T_world_camera.inverse() * lidar.T_world_lidar;
    CHECK((cfg.T_sensor_source.matrix() - T_camera_lidar.matrix()).cwiseAbs().maxCoeff() < 1e-12);
}

TEST_CASE("example thermal config matches the synthetic thermal camera and reference LiDAR") {
    const SensorConfig cfg =
        fusion::load_sensor_config(FUSION_SOURCE_DIR "/configs/thermal_camera.yaml");
    const fusion::testing::PinholeCamera thermal = fusion::testing::make_thermal_camera();
    const fusion::testing::LidarConfig lidar = fusion::testing::make_reference_lidar();
    CHECK(cfg.name == "thermal");
    CHECK(cfg.width == thermal.K.width);
    CHECK(cfg.height == thermal.K.height);
    CHECK(cfg.K(0, 0) == thermal.K.fx);
    CHECK(cfg.K(1, 1) == thermal.K.fy);
    CHECK(cfg.K(0, 2) == thermal.K.cx);
    CHECK(cfg.K(1, 2) == thermal.K.cy);
    CHECK(cfg.distortion.is_zero());
    CHECK(cfg.source_frame == "lidar");
    CHECK(cfg.time_offset_ns == 0);
    const Eigen::Isometry3d T_thermal_lidar =
        thermal.T_world_camera.inverse() * lidar.T_world_lidar;
    CHECK((cfg.T_sensor_source.matrix() - T_thermal_lidar.matrix()).cwiseAbs().maxCoeff() < 1e-12);
    REQUIRE(cfg.thermal.has_value());
    CHECK(cfg.thermal->planck.R == 380747.0);
    CHECK(cfg.thermal->planck.B == 1428.0);
    CHECK(cfg.thermal->planck.F == 1.0);
    CHECK(cfg.thermal->planck.O == -88.539);
}

TEST_CASE("thermal section errors are reported by field") {
    const std::string base = kValid + "thermal:\n  planck: {R: 380747, B: 1428, F: 1, O: -88.539}\n";
    const SensorConfig cfg = fusion::parse_sensor_config(base);
    REQUIRE(cfg.thermal.has_value());
    CHECK(cfg.thermal->planck.O == -88.539);
    CHECK_FALSE(fusion::parse_sensor_config(kValid).thermal.has_value());

    check_error(kValid + "thermal:\n", {"thermal: missing value"});
    check_error(kValid + "thermal:\n  other: 1\n", {"thermal.planck: missing required field"});
    check_error(kValid + "thermal:\n  planck: {R: 380747, B: 1428, F: 1}\n",
                {"thermal.planck.O: missing required field"});
    check_error(kValid + "thermal:\n  planck: {R: 380747, B: abc, F: 1, O: 0}\n",
                {"thermal.planck.B: expected a number"});
    check_error(kValid + "thermal:\n  planck: {R: 0, B: 1428, F: 1, O: 0}\n",
                {"thermal.planck.R: must be positive"});
    check_error(kValid + "thermal:\n  planck: {R: 380747, B: -1, F: 1, O: 0}\n",
                {"thermal.planck.B: must be positive"});
    check_error(kValid + "thermal:\n  planck: {R: 380747, B: 1428, F: .inf, O: 0}\n",
                {"thermal.planck.F: must be finite"});
}

TEST_CASE("thermal encoding: planck by default, or linear with scale and offset") {
    const SensorConfig def = fusion::parse_sensor_config(
        kValid + "thermal:\n  planck: {R: 380747, B: 1428, F: 1, O: -88.539}\n");
    REQUIRE(def.thermal.has_value());
    CHECK(def.thermal->encoding == fusion::ThermalEncoding::planck);

    const SensorConfig planck = fusion::parse_sensor_config(
        kValid + "thermal:\n  encoding: planck\n  planck: {R: 380747, B: 1428, F: 1, O: -88.539}\n");
    REQUIRE(planck.thermal.has_value());
    CHECK(planck.thermal->encoding == fusion::ThermalEncoding::planck);
    CHECK(planck.thermal->planck.R == 380747.0);

    const SensorConfig linear = fusion::parse_sensor_config(
        kValid + "thermal:\n  encoding: linear\n  linear: {scale: 0.01, offset: -273.15}\n");
    REQUIRE(linear.thermal.has_value());
    CHECK(linear.thermal->encoding == fusion::ThermalEncoding::linear);
    CHECK(linear.thermal->linear.scale == 0.01);
    CHECK(linear.thermal->linear.offset == -273.15);
    CHECK(fusion::raw_to_celsius(0, *linear.thermal) == -273.15);

    const SensorConfig negative = fusion::parse_sensor_config(
        kValid + "thermal:\n  encoding: linear\n  linear: {scale: -0.1, offset: 0}\n");
    CHECK(negative.thermal->linear.scale == -0.1);
}

TEST_CASE("thermal encoding errors are reported by field") {
    const std::string th = kValid + "thermal:\n";
    check_error(th + "  encoding: celsius\n  linear: {scale: 1, offset: 0}\n",
                {"thermal.encoding: unknown encoding 'celsius'"});
    check_error(th + "  encoding: linear\n", {"thermal.linear: missing required field"});
    check_error(th + "  encoding: linear\n  linear: {offset: -273.15}\n",
                {"thermal.linear.scale: missing required field"});
    check_error(th + "  encoding: linear\n  linear: {scale: 0.01}\n",
                {"thermal.linear.offset: missing required field"});
    check_error(th + "  encoding: linear\n  linear: {scale: 0, offset: -273.15}\n",
                {"thermal.linear.scale: must not be zero"});
    check_error(th + "  encoding: linear\n  linear: {scale: .nan, offset: -273.15}\n",
                {"thermal.linear.scale: must be finite"});
    check_error(th + "  encoding: linear\n  linear: {scale: 0.01, offset: .inf}\n",
                {"thermal.linear.offset: must be finite"});
    check_error(th + "  encoding: linear\n  linear: {scale: abc, offset: 0}\n",
                {"thermal.linear.scale: expected a number"});
    check_error(th + "  encoding: linear\n  linear: {scale: 0.01, offset: -273.15}\n"
                     "  planck: {R: 380747, B: 1428, F: 1, O: -88.539}\n",
                {"thermal.planck: only allowed with encoding 'planck'"});
    check_error(th + "  planck: {R: 380747, B: 1428, F: 1, O: -88.539}\n"
                     "  linear: {scale: 0.01, offset: -273.15}\n",
                {"thermal.linear: only allowed with encoding 'linear'"});
    check_error(th + "  encoding: [linear]\n  linear: {scale: 0.01, offset: -273.15}\n",
                {"thermal.encoding: expected a string"});
}

TEST_CASE("slightly non-orthonormal rotation is accepted and re-orthonormalized") {
    // cos(30 deg) and sin(30 deg) written with six decimals.
    const SensorConfig cfg = fusion::parse_sensor_config(with(
        "  rotation: [[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]]",
        "  rotation: [[0.866025, -0.5, 0.0], [0.5, 0.866025, 0.0], [0.0, 0.0, 1.0]]"));
    const Eigen::Matrix3d R = cfg.T_sensor_source.linear();
    CHECK((R.transpose() * R - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() < 1e-14);
    CHECK(R(0, 0) == doctest::Approx(0.8660254037844386).epsilon(1e-6));
}

TEST_CASE("missing fields are reported by name") {
    check_error(with("name: thermal", ""), {"name: missing required field"});
    check_error(with("resolution: [320, 256]", ""), {"resolution: missing required field"});
    check_error(with("  K: [[320.0, 0.0, 159.5], [0.0, 321.0, 127.5], [0.0, 0.0, 1.0]]",
                     "  focal: 1.0"),
                {"intrinsics.K: missing required field"});
    check_error(with("distortion: [-0.1, 0.02, 0.001, -0.002, 0.0005]", ""),
                {"distortion: missing required field"});
    check_error(with("  source: lidar", ""), {"extrinsics.source: missing required field"});
    check_error(with("  rotation: [[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]]", ""),
                {"extrinsics.rotation: missing required field"});
    check_error(with("  translation: [0.1, -0.6, 0.05]", ""),
                {"extrinsics.translation: missing required field"});
    check_error(with("time_offset_ns: -2500000", ""), {"time_offset_ns: missing required field"});
    check_error(with("time_offset_ns: -2500000", "time_offset_ns:"),
                {"time_offset_ns: missing value"});
}

TEST_CASE("wrong shapes are reported with the expected and actual size") {
    check_error(with("resolution: [320, 256]", "resolution: [320, 256, 3]"),
                {"resolution: expected [width, height]"});
    check_error(with("  K: [[320.0, 0.0, 159.5], [0.0, 321.0, 127.5], [0.0, 0.0, 1.0]]",
                     "  K: [[320.0, 0.0, 159.5], [0.0, 321.0, 127.5]]"),
                {"intrinsics.K: expected a 3x3 matrix", "got 2 rows"});
    check_error(with("  K: [[320.0, 0.0, 159.5], [0.0, 321.0, 127.5], [0.0, 0.0, 1.0]]",
                     "  K: [[320.0, 0.0, 159.5], [0.0, 321.0], [0.0, 0.0, 1.0]]"),
                {"intrinsics.K[1]: expected a 3x3 matrix", "row 1 has 2 entries"});
    check_error(with("  K: [[320.0, 0.0, 159.5], [0.0, 321.0, 127.5], [0.0, 0.0, 1.0]]",
                     "  K: [320.0, 321.0, 159.5, 127.5]"),
                {"intrinsics.K: expected a 3x3 matrix", "got 4 rows"});
    check_error(with("distortion: [-0.1, 0.02, 0.001, -0.002, 0.0005]",
                     "distortion: [-0.1, 0.02, 0.001, -0.002]"),
                {"distortion: expected a list of 5 numbers, got 4"});
    check_error(with("  translation: [0.1, -0.6, 0.05]", "  translation: [0.1, -0.6]"),
                {"extrinsics.translation: expected a list of 3 numbers, got 2"});
    check_error(with("  rotation: [[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]]",
                     "  rotation: 1.0"),
                {"extrinsics.rotation: expected a 3x3 matrix"});
    check_error(with("  source: lidar", "  source: [lidar]"),
                {"extrinsics.source: expected a string"});
}

TEST_CASE("non-invertible and non-rotation matrices are rejected") {
    const std::string base_rot =
        "  rotation: [[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]]";
    check_error(with(base_rot, "  rotation: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 1.0, 0.0]]"),
                {"extrinsics.rotation: matrix is not invertible", "determinant 0"});
    check_error(with(base_rot, "  rotation: [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]"),
                {"extrinsics.rotation: matrix is not invertible"});
    check_error(with(base_rot, "  rotation: [[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]"),
                {"extrinsics.rotation: determinant is -1", "reflection"});
    check_error(with(base_rot, "  rotation: [[2.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]"),
                {"extrinsics.rotation: matrix is not orthonormal"});
}

TEST_CASE("invalid values are rejected") {
    check_error(with("time_offset_ns: -2500000", "time_offset_ns: 1.5"),
                {"time_offset_ns: expected an integer, got '1.5'"});
    check_error(with("time_offset_ns: -2500000", "time_offset_ns: 1e9"),
                {"time_offset_ns: expected an integer"});
    check_error(with("time_offset_ns: -2500000", "time_offset_ns: 0x10"),
                {"time_offset_ns: expected an integer"});
    check_error(with("time_offset_ns: -2500000", "time_offset_ns: 99999999999999999999"),
                {"time_offset_ns: integer", "out of range"});
    check_error(with("resolution: [320, 256]", "resolution: [320, 0]"),
                {"resolution: width and height must be in"});
    check_error(with("resolution: [320, 256]", "resolution: [320.5, 256]"),
                {"resolution[0]: expected an integer"});
    check_error(with("distortion: [-0.1, 0.02, 0.001, -0.002, 0.0005]",
                     "distortion: [-0.1, abc, 0.001, -0.002, 0.0005]"),
                {"distortion[1]: expected a number, got 'abc'"});
    check_error(with("distortion: [-0.1, 0.02, 0.001, -0.002, 0.0005]",
                     "distortion: [-0.1, .nan, 0.001, -0.002, 0.0005]"),
                {"distortion[1]: must be finite"});
    check_error(with("  K: [[320.0, 0.0, 159.5], [0.0, 321.0, 127.5], [0.0, 0.0, 1.0]]",
                     "  K: [[-320.0, 0.0, 159.5], [0.0, 321.0, 127.5], [0.0, 0.0, 1.0]]"),
                {"intrinsics.K: fx", "must be positive"});
    check_error(with("  K: [[320.0, 0.0, 159.5], [0.0, 321.0, 127.5], [0.0, 0.0, 1.0]]",
                     "  K: [[320.0, 0.0, 159.5], [0.0, 321.0, 127.5], [0.0, 0.0, 2.0]]"),
                {"intrinsics.K: expected the form"});
    check_error(with("name: thermal", "name: \"\""), {"name: must not be empty"});
}

TEST_CASE("unreadable files and malformed YAML are reported") {
    check_error("name: [unclosed", {"invalid YAML"});
    check_error("- just\n- a list\n", {"expected a YAML map at the top level"});
    check_error("", {"expected a YAML map at the top level"});

    std::string message;
    try {
        fusion::load_sensor_config("/nonexistent/fusion/sensor.yaml");
    } catch (const ConfigError& e) {
        message = e.what();
    }
    CHECK(message == "/nonexistent/fusion/sensor.yaml: cannot open file");
}
