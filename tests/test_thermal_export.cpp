// thermal_export end to end on the generated synthetic sequence, with the
// Planck and the linear encoding.
#include <doctest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include <fusion/image_io.hpp>
#include <fusion/sequence.hpp>
#include <fusion/thermal.hpp>

#include "support/synthetic_sequence.hpp"

using namespace fusion::testing;
namespace fs = std::filesystem;

namespace {

std::string quote(const std::string& s) { return "\"" + s + "\""; }

int run_export(const std::string& args, const std::string& log) {
    const std::string cmd =
        quote(THERMAL_EXPORT_PATH) + " " + args + " > " + quote(log) + " 2>&1";
    return std::system(cmd.c_str());
}

std::vector<double> read_f64(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    const std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(in)),
                                           std::istreambuf_iterator<char>());
    REQUIRE(bytes.size() % 8 == 0);
    std::vector<double> out(bytes.size() / 8);
    for (std::size_t i = 0; i < out.size(); ++i) {
        std::uint64_t u = 0;
        for (std::size_t k = 0; k < 8; ++k) u |= std::uint64_t{bytes[i * 8 + k]} << (8 * k);
        std::memcpy(&out[i], &u, 8);
    }
    return out;
}

std::vector<std::string> lines(const std::string& path) {
    std::ifstream in(path);
    std::vector<std::string> out;
    for (std::string l; std::getline(in, l);) out.push_back(l);
    return out;
}

std::string frame_file(const std::string& prefix, std::size_t i, const std::string& ext) {
    std::string n = std::to_string(i);
    return prefix + std::string(6 - n.size(), '0') + n + ext;
}

double nearest_rank(std::vector<double> v, double p) {
    std::sort(v.begin(), v.end());
    const auto rank = static_cast<std::size_t>(std::ceil(p / 100.0 * static_cast<double>(v.size())));
    return v[std::min(std::max<std::size_t>(rank, 1), v.size()) - 1];
}

void check_export(const SyntheticSequence& s, const std::string& name) {
    const std::string root = std::string(FUSION_TEST_OUTPUT_DIR) + "/thermal_export_" + name;
    fs::remove_all(root);
    const std::string manifest = write_synthetic_sequence(s, root + "/seq");
    const std::string out = root + "/out";
    REQUIRE(run_export(quote(manifest) + " --out " + quote(out) + " --percentiles 5,95",
                       root + "/log.txt") == 0);
    const fusion::Sequence seq = fusion::load_sequence(manifest);
    REQUIRE(seq.thermal);
    const fusion::ThermalCalibration& cal = *seq.thermal->config.thermal;

    const auto stats = lines(out + "/thermal_stats.csv");
    REQUIRE(stats.size() == seq.thermal->frames.size() + 1);
    CHECK(stats[0].rfind("frame,timestamp_ns,width,height,convertible,", 0) == 0);
    for (std::size_t j = 0; j < seq.thermal->frames.size(); ++j) {
        CAPTURE(j);
        const fusion::RawThermalImage raw =
            fusion::read_thermal_frame(seq.thermal->frames[j].path, seq.thermal->config);
        const std::vector<double> c = read_f64(out + "/" + frame_file("celsius_", j, ".f64"));
        REQUIRE(c.size() == raw.values().size());
        for (std::size_t i = 0; i < c.size(); ++i) {
            CHECK(c[i] == fusion::raw_to_celsius(raw.values()[i], cal));
        }
        // The render is render_display() with the 5th-95th percentile window.
        const double lo = nearest_rank(c, 5.0), hi = nearest_rank(c, 95.0);
        const fusion::DisplayImage want =
            fusion::render_display(raw, cal, fusion::WindowLevel{0.5 * (lo + hi), std::max(hi - lo, 1e-6)});
        const fusion::RgbImage got =
            fusion::read_rgb_png(out + "/" + frame_file("render_", j, ".png"));
        REQUIRE(got.pixels.size() == want.pixels.size());
        CHECK(std::equal(got.pixels.begin(), got.pixels.end(), want.pixels.begin()));
        CHECK(stats[j + 1].rfind(std::to_string(j) + "," +
                                     std::to_string(seq.thermal->frames[j].t_ns) + ",",
                                 0) == 0);
    }
}

}  // namespace

TEST_CASE("thermal_export converts and renders with the configured encoding") {
    SyntheticSequence s = make_synthetic_sequence(2);
    SUBCASE("planck") { check_export(s, "planck"); }
    SUBCASE("linear") {
        s.thermal.config.thermal = fusion::ThermalCalibration::from_linear({0.01, -273.15});
        check_export(s, "linear");
    }
}

TEST_CASE("thermal_export rejects bad arguments") {
    const std::string log = std::string(FUSION_TEST_OUTPUT_DIR) + "/thermal_export_args.txt";
    CHECK(run_export("", log) != 0);
    CHECK(run_export("x.yaml --out o --percentiles 99,1", log) != 0);
    CHECK(run_export("x.yaml --out o --percentiles 1", log) != 0);
    CHECK(run_export("--help", log) == 0);
    CHECK(run_export(quote(std::string(FUSION_TEST_OUTPUT_DIR) + "/missing.yaml") + " --out " +
                         quote(std::string(FUSION_TEST_OUTPUT_DIR) + "/thermal_export_missing"),
                     log) != 0);
}
