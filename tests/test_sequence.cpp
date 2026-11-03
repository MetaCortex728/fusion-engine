// Sequence format: round trip through the generator and the readers, and a
// specific error for each kind of malformed input.
#include <doctest.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <fusion/image_io.hpp>
#include <fusion/sequence.hpp>

#include "support/synthetic_sequence.hpp"

using namespace fusion::testing;
namespace fs = std::filesystem;

namespace {

std::string out_dir(const std::string& name) {
    const std::string dir = std::string(FUSION_TEST_OUTPUT_DIR) + "/sequence_" + name;
    fs::remove_all(dir);
    return dir;
}

std::string read_text(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    std::ostringstream s;
    s << in.rdbuf();
    return s.str();
}

void write_text(const std::string& path, const std::string& text) {
    std::ofstream out(path, std::ios::binary);
    out << text;
}

void replace_in(const std::string& path, const std::string& from, const std::string& to) {
    std::string s = read_text(path);
    const auto pos = s.find(from);
    REQUIRE_MESSAGE(pos != std::string::npos, "'" << from << "' not in " << path);
    s.replace(pos, from.size(), to);
    write_text(path, s);
}

const SyntheticSequence& small_sequence() {
    static const SyntheticSequence s = make_synthetic_sequence(2);
    return s;
}

// Writes a fresh copy of the small sequence and returns its manifest path.
std::string fresh(const std::string& name) {
    return write_synthetic_sequence(small_sequence(), out_dir(name));
}

// Message of the SequenceError thrown by f; fails the test if none is.
template <typename F>
std::string error_of(F&& f) {
    try {
        f();
    } catch (const fusion::SequenceError& e) {
        return e.what();
    }
    FAIL("expected a SequenceError");
    return {};
}

bool has(const std::string& s, const std::string& part) { return s.find(part) != std::string::npos; }

}  // namespace

TEST_CASE("sequence round trip: every reader returns the generator's values") {
    const SyntheticSequence s = make_synthetic_sequence(5);
    const std::string manifest = write_synthetic_sequence(s, out_dir("round_trip"));
    const fusion::Sequence seq = fusion::load_sequence(manifest);

    // LiDAR: stamps, times on the reference clock, exact float32 points and
    // intensities.
    REQUIRE(seq.lidar.size() == s.scans.size());
    CHECK(seq.lidar_name == "lidar");
    CHECK(seq.lidar_time_offset_ns == s.lidar_time_offset_ns);
    CHECK((seq.T_body_lidar.matrix() - s.rig.T_body_lidar.matrix()).cwiseAbs().maxCoeff() <= 1e-15);
    for (std::size_t k = 0; k < s.scans.size(); ++k) {
        CAPTURE(k);
        const fusion::LidarScan got = fusion::load_lidar_scan(seq, k);
        const fusion::LidarScan& want = s.scans[k];
        CHECK(got.t_ns == want.t_ns);
        REQUIRE(got.points.size() == want.points.size());
        REQUIRE(got.intensity.size() == want.intensity.size());
        std::size_t mismatches = 0;
        for (std::size_t i = 0; i < got.points.size(); ++i) {
            for (int c = 0; c < 3; ++c) {
                // The reader widens float32 to double exactly.
                const double v = got.points[i][c];
                if (v != static_cast<double>(as_float32(want.points[i][c])) ||
                    static_cast<float>(v) != as_float32(want.points[i][c])) {
                    ++mismatches;
                }
            }
            if (got.intensity[i] != want.intensity[i]) ++mismatches;
        }
        CHECK(mismatches == 0);
    }

    // Cameras: configs, stamps, tau, and bit-exact images.
    REQUIRE(seq.rgb);
    REQUIRE(seq.thermal);
    for (const auto& pair : {std::pair{&*seq.rgb, &s.rgb}, {&*seq.thermal, &s.thermal}}) {
        const fusion::CameraStream* got = pair.first;
        const SyntheticCamera* want = pair.second;
        CAPTURE(want->config.name);
        CHECK(got->config.name == want->config.name);
        CHECK(got->config.width == want->config.width);
        CHECK(got->config.height == want->config.height);
        CHECK(got->config.K == want->config.K);
        CHECK(got->config.time_offset_ns == want->config.time_offset_ns);
        CHECK((got->config.T_sensor_source.matrix() - want->config.T_sensor_source.matrix())
                  .cwiseAbs()
                  .maxCoeff() <= 1e-15);
        REQUIRE(got->frames.size() == want->stamps_sensor_ns.size());
        for (std::size_t j = 0; j < got->frames.size(); ++j) {
            CHECK(got->frames[j].t_ns == want->stamps_sensor_ns[j]);
        }
        CHECK(got->tau_ns.has_value() == want->tau_in_manifest);
        CHECK(fusion::camera_tau_ns(seq, *got) == want->tau_ns);
    }
    REQUIRE(seq.thermal->config.thermal);
    CHECK(seq.thermal->config.thermal->planck.O == s.thermal.config.thermal->planck.O);
    for (std::size_t j = 0; j < s.rgb_images.size(); ++j) {
        const fusion::RgbImage img = fusion::read_rgb_frame(seq.rgb->frames[j].path, seq.rgb->config);
        CHECK(img.width == s.rgb_images[j].width);
        CHECK(img.height == s.rgb_images[j].height);
        CHECK(img.pixels == s.rgb_images[j].pixels);
    }
    for (std::size_t j = 0; j < s.thermal_images.size(); ++j) {
        const fusion::RawThermalImage img =
            fusion::read_thermal_frame(seq.thermal->frames[j].path, seq.thermal->config);
        CHECK(img.values() == s.thermal_images[j].values());
    }

    // Trajectory: exactly the generator's poses (after the same normalization).
    REQUIRE(seq.trajectory);
    const fusion::Trajectory want(s.poses);
    REQUIRE(seq.trajectory->size() == want.size());
    for (std::size_t i = 0; i < want.size(); ++i) {
        const auto& a = seq.trajectory->poses()[i];
        const auto& b = want.poses()[i];
        CHECK(a.t_ns == b.t_ns);
        CHECK(a.position == b.position);
        CHECK(a.orientation.coeffs() == b.orientation.coeffs());
    }
}

TEST_CASE("16-bit PNG keeps every raw value, including ones a codec could shrink") {
    const std::string dir = out_dir("png16");
    fs::create_directories(dir);
    // All values have equal high and low bytes, which an encoder choosing
    // the smallest format could store as 8 bits.
    std::vector<std::uint16_t> same_bytes;
    for (int i = 0; i < 12; ++i) same_bytes.push_back(static_cast<std::uint16_t>(0x0101 * (i * 20)));
    const fusion::RawThermalImage a(4, 3, same_bytes);
    fusion::write_thermal_png(dir + "/a.png", a);
    CHECK(fusion::read_thermal_png(dir + "/a.png").values() == a.values());

    const fusion::RawThermalImage b(3, 2, {0, 1, 255, 256, 65534, 65535});
    fusion::write_thermal_png(dir + "/b.png", b);
    CHECK(fusion::read_thermal_png(dir + "/b.png").values() == b.values());

    // A two-colour RGB image stays 8-bit RGB and reads back exactly.
    fusion::RgbImage rgb{2, 2, {{0, 0, 0}, {255, 255, 255}, {0, 0, 0}, {255, 255, 255}}};
    fusion::write_rgb_png(dir + "/c.png", rgb);
    CHECK(fusion::read_rgb_png(dir + "/c.png").pixels == rgb.pixels);
    // But it is not a thermal image.
    CHECK_THROWS_WITH_AS(fusion::read_thermal_png(dir + "/c.png"),
                         doctest::Contains("expected a 16-bit greyscale PNG"), fusion::ImageError);
    CHECK_THROWS_WITH_AS(fusion::read_rgb_png(dir + "/a.png"),
                         doctest::Contains("expected an 8-bit PNG"), fusion::ImageError);
}

TEST_CASE("sequence errors: missing files") {
    SUBCASE("manifest") {
        const std::string e = error_of([] { fusion::load_sequence(out_dir("nope") + "/sequence.yaml"); });
        CHECK(has(e, "sequence.yaml"));
        CHECK(has(e, "missing file"));
    }
    SUBCASE("listed LiDAR scan") {
        const std::string m = fresh("missing_scan");
        fs::remove(fs::path(m).parent_path() / "lidar/000001.bin");
        const std::string e = error_of([&] { fusion::load_sequence(m); });
        CHECK(has(e, "lidar.csv:3"));
        CHECK(has(e, "missing file"));
        CHECK(has(e, "000001.bin"));
    }
    SUBCASE("listed image") {
        const std::string m = fresh("missing_image");
        fs::remove(fs::path(m).parent_path() / "thermal/000000.png");
        const std::string e = error_of([&] { fusion::load_sequence(m); });
        CHECK(has(e, "thermal.csv:2"));
        CHECK(has(e, "missing file"));
    }
    SUBCASE("camera config") {
        const std::string m = fresh("missing_config");
        fs::remove(fs::path(m).parent_path() / "rgb.yaml");
        const std::string e = error_of([&] { fusion::load_sequence(m); });
        CHECK(has(e, "cameras.rgb.config"));
        CHECK(has(e, "missing file"));
    }
    SUBCASE("trajectory") {
        const std::string m = fresh("missing_trajectory");
        fs::remove(fs::path(m).parent_path() / "trajectory.csv");
        const std::string e = error_of([&] { fusion::load_sequence(m); });
        CHECK(has(e, "trajectory"));
        CHECK(has(e, "missing file"));
    }
    SUBCASE("list file") {
        const std::string m = fresh("missing_list");
        fs::remove(fs::path(m).parent_path() / "lidar.csv");
        const std::string e = error_of([&] { fusion::load_sequence(m); });
        CHECK(has(e, "lidar.list"));
        CHECK(has(e, "missing file"));
    }
}

TEST_CASE("sequence errors: unsorted or duplicate timestamps") {
    const SyntheticSequence& s = small_sequence();
    const std::string t0 = std::to_string(s.scans[0].t_ns);
    const std::string t1 = std::to_string(s.scans[1].t_ns);
    SUBCASE("unsorted list") {
        const std::string m = fresh("unsorted_list");
        const std::string list = (fs::path(m).parent_path() / "lidar.csv").string();
        replace_in(list, t1 + ",", std::to_string(s.scans[0].t_ns - 1) + ",");
        const std::string e = error_of([&] { fusion::load_sequence(m); });
        CHECK(has(e, "lidar.csv:3"));
        CHECK(has(e, "unsorted timestamps"));
    }
    SUBCASE("duplicate in list") {
        const std::string m = fresh("duplicate_list");
        const std::string list = (fs::path(m).parent_path() / "lidar.csv").string();
        replace_in(list, t1 + ",", t0 + ",");
        const std::string e = error_of([&] { fusion::load_sequence(m); });
        CHECK(has(e, "lidar.csv:3"));
        CHECK(has(e, "duplicate timestamp " + t0));
    }
    SUBCASE("unsorted trajectory") {
        const std::string m = fresh("unsorted_traj");
        const std::string traj = (fs::path(m).parent_path() / "trajectory.csv").string();
        replace_in(traj, std::to_string(s.poses[2].t_ns) + ",", std::to_string(s.poses[0].t_ns - 5) + ",");
        const std::string e = error_of([&] { fusion::load_sequence(m); });
        CHECK(has(e, "trajectory.csv:4"));
        CHECK(has(e, "unsorted timestamps"));
    }
    SUBCASE("duplicate in trajectory") {
        const std::string m = fresh("duplicate_traj");
        const std::string traj = (fs::path(m).parent_path() / "trajectory.csv").string();
        replace_in(traj, std::to_string(s.poses[2].t_ns) + ",", std::to_string(s.poses[1].t_ns) + ",");
        const std::string e = error_of([&] { fusion::load_sequence(m); });
        CHECK(has(e, "trajectory.csv:4"));
        CHECK(has(e, "duplicate timestamp"));
    }
}

TEST_CASE("sequence errors: wrong LiDAR record size") {
    const std::string m = fresh("record_size");
    const std::string bin = (fs::path(m).parent_path() / "lidar/000000.bin").string();
    {
        std::ofstream out(bin, std::ios::binary | std::ios::app);
        out.write("abc", 3);
    }
    const fusion::Sequence seq = fusion::load_sequence(m);
    const std::string e = error_of([&] { fusion::load_lidar_scan(seq, 0); });
    CHECK(has(e, "000000.bin"));
    CHECK(has(e, "not a multiple of the 16-byte record"));
}

TEST_CASE("sequence errors: image size does not match the sensor config") {
    const std::string m = fresh("image_size");
    const fs::path root = fs::path(m).parent_path();
    fusion::write_rgb_png((root / "rgb/000000.png").string(), fusion::RgbImage{3, 2, std::vector<fusion::Rgb8>(6)});
    fusion::write_thermal_png((root / "thermal/000000.png").string(), fusion::RawThermalImage(5, 4));
    const fusion::Sequence seq = fusion::load_sequence(m);

    std::string e = error_of([&] { fusion::read_rgb_frame(seq.rgb->frames[0].path, seq.rgb->config); });
    CHECK(has(e, "rgb/000000.png"));
    CHECK(has(e, "image size 3x2 does not match the sensor config 'rgb' resolution 640x480"));
    e = error_of([&] { fusion::read_thermal_frame(seq.thermal->frames[0].path, seq.thermal->config); });
    CHECK(has(e, "image size 5x4 does not match the sensor config 'thermal' resolution 320x256"));

    // An 8-bit image where a 16-bit thermal image is expected.
    fusion::write_rgb_png((root / "thermal/000000.png").string(),
                          fusion::RgbImage{320, 256, std::vector<fusion::Rgb8>(320 * 256)});
    e = error_of([&] { fusion::read_thermal_frame(seq.thermal->frames[0].path, seq.thermal->config); });
    CHECK(has(e, "expected a 16-bit greyscale PNG"));
    // Not a PNG at all.
    write_text((root / "rgb/000000.png").string(), "not a png");
    e = error_of([&] { fusion::read_rgb_frame(seq.rgb->frames[0].path, seq.rgb->config); });
    CHECK(has(e, "not a valid PNG"));
}

TEST_CASE("sequence errors: malformed manifest and list files") {
    SUBCASE("list header") {
        const std::string m = fresh("list_header");
        replace_in((fs::path(m).parent_path() / "rgb.csv").string(), "timestamp_ns,path", "t,file");
        const std::string e = error_of([&] { fusion::load_sequence(m); });
        CHECK(has(e, "rgb.csv:1"));
        CHECK(has(e, "expected the header 'timestamp_ns,path'"));
    }
    SUBCASE("non-integer timestamp") {
        const std::string m = fresh("list_stamp");
        const std::string t = std::to_string(small_sequence().scans[0].t_ns);
        replace_in((fs::path(m).parent_path() / "lidar.csv").string(), t + ",", t + ".5,");
        const std::string e = error_of([&] { fusion::load_sequence(m); });
        CHECK(has(e, "lidar.csv:2"));
        CHECK(has(e, "is not an integer"));
    }
    SUBCASE("trajectory row") {
        const std::string m = fresh("traj_row");
        const std::string traj = (fs::path(m).parent_path() / "trajectory.csv").string();
        write_text(traj, read_text(traj) + "1,2,3\n");
        const std::string e = error_of([&] { fusion::load_sequence(m); });
        CHECK(has(e, "trajectory.csv"));
        CHECK(has(e, "expected 8 fields, got 3"));
    }
    SUBCASE("unknown field") {
        const std::string m = fresh("unknown_field");
        replace_in(m, "trajectory: trajectory.csv", "trajectroy: trajectory.csv");
        const std::string e = error_of([&] { fusion::load_sequence(m); });
        CHECK(has(e, "trajectroy: unknown field"));
    }
    SUBCASE("format version") {
        const std::string m = fresh("version");
        replace_in(m, "format_version: 1", "format_version: 2");
        const std::string e = error_of([&] { fusion::load_sequence(m); });
        CHECK(has(e, "format_version: unsupported version '2'"));
    }
    SUBCASE("trajectory without T_body_lidar") {
        const std::string m = fresh("no_mount");
        std::string text = read_text(m);
        const auto a = text.find("  T_body_lidar:");
        const auto b = text.find("cameras:");
        text.erase(a, b - a);
        write_text(m, text);
        const std::string e = error_of([&] { fusion::load_sequence(m); });
        CHECK(has(e, "lidar.T_body_lidar: required when a trajectory is given"));
    }
    SUBCASE("camera extrinsics not relative to the LiDAR") {
        const std::string m = fresh("source");
        replace_in((fs::path(m).parent_path() / "rgb.yaml").string(), "source: lidar", "source: body");
        const std::string e = error_of([&] { fusion::load_sequence(m); });
        CHECK(has(e, "rgb.yaml"));
        CHECK(has(e, "expected the LiDAR frame 'lidar'"));
    }
    SUBCASE("bad camera config") {
        const std::string m = fresh("bad_config");
        replace_in((fs::path(m).parent_path() / "rgb.yaml").string(), "resolution: [640, 480]",
                   "resolution: [640]");
        const std::string e = error_of([&] { fusion::load_sequence(m); });
        CHECK(has(e, "rgb.yaml: resolution"));
    }
    SUBCASE("default tau needs two stamps") {
        const std::string dir = out_dir("one_scan");
        const std::string m = write_synthetic_sequence(make_synthetic_sequence(1), dir);
        const fusion::Sequence seq = fusion::load_sequence(m);
        CHECK(fusion::camera_tau_ns(seq, *seq.rgb) == 25'000'000);  // set in the manifest
        const std::string e = error_of([&] { fusion::camera_tau_ns(seq, *seq.thermal); });
        CHECK(has(e, "set tau_ns"));
    }
}
