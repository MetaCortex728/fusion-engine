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
#include <set>
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

TEST_CASE("synthetic thermal scene has realistic apparent temperatures") {
    // Expected counts and read-back temperatures computed separately with
    // Python's decimal module at 50 significant digits from
    // S = R / (exp(B / (T + 273.15)) - F) + O, rounded, and
    // T = B / ln(R / (S - O) + F) - 273.15.
    struct Case {
        double celsius;
        std::uint16_t raw;
        double back;
    };
    const Case surfaces[] = {
        {18.0, 2755, 18.009618623534798807},  // ground (road)
        {45.0, 4239, 45.000775571147893681},  // warm box ahead
        {22.0, 2951, 21.990069179437776924},  // wall
        {12.0, 2474, 12.002635736574103780},  // low block
        {26.0, 3156, 25.992338607353652018},  // floating block
    };
    const Case sky{-15.0, 1425, -14.999260924897754969};

    REQUIRE(std::size(kSyntheticSurfaceCelsius) == std::size(surfaces));
    const SyntheticSequence s = make_synthetic_sequence(2);
    REQUIRE(s.thermal.config.thermal.has_value());
    const fusion::ThermalCalibration& cal = *s.thermal.config.thermal;
    CHECK(cal.encoding == fusion::ThermalEncoding::planck);
    for (std::size_t i = 0; i < std::size(surfaces); ++i) {
        CAPTURE(i);
        const Case& c = surfaces[i];
        CHECK(kSyntheticSurfaceCelsius[i] == c.celsius);
        CHECK(celsius_to_raw(c.celsius, kSyntheticPlanck) == c.raw);
        CHECK(s.scene.raw_of(static_cast<int>(i)) == c.raw);
        CHECK(std::abs(fusion::raw_to_celsius(c.raw, cal) - c.back) <= 1e-9);
        CHECK(std::abs(fusion::raw_to_celsius(c.raw, cal) - c.celsius) < 0.01);
    }
    CHECK(kSyntheticSkyCelsius == sky.celsius);
    CHECK(celsius_to_raw(sky.celsius, kSyntheticPlanck) == sky.raw);
    CHECK(std::abs(fusion::raw_to_celsius(sky.raw, cal) - sky.back) <= 1e-9);

    // Every pixel of every thermal frame is one of these counts: road and
    // walls 10-30 °C, the warm box 35-60 °C, the sky -15 °C.
    std::set<std::uint16_t> seen;
    for (const fusion::RawThermalImage& img : s.thermal_images) {
        seen.insert(img.values().begin(), img.values().end());
    }
    for (const std::uint16_t v : seen) {
        CAPTURE(v);
        const bool known = v == sky.raw || std::any_of(std::begin(surfaces), std::end(surfaces),
                                                       [&](const Case& c) { return c.raw == v; });
        CHECK(known);
    }
    CHECK(seen.count(2755) == 1);  // the thermal camera sees the road,
    CHECK(seen.count(4239) == 1);  // the warm box ahead
    CHECK(seen.count(1425) == 1);  // and the sky

    // The inverse rejects counts outside 16 bits and invalid constants.
    CHECK_THROWS_AS(celsius_to_raw(-150.0, kSyntheticPlanck), std::domain_error);
    CHECK_THROWS_AS(celsius_to_raw(600.0, kSyntheticPlanck), std::domain_error);
    CHECK_THROWS_AS(celsius_to_raw(20.0, fusion::PlanckParams{0.0, 1428.0, 1.0, 0.0}),
                    std::invalid_argument);
}
