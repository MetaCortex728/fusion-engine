// fuse_cli end to end on the generated synthetic sequence.
#include <doctest.h>

#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <fusion/image_io.hpp>

#include "support/synthetic_sequence.hpp"

using namespace fusion::testing;
namespace fs = std::filesystem;

namespace {

std::string quote(const std::string& s) { return "\"" + s + "\""; }

// Runs fuse_cli with `args`, output to `log`; returns std::system's status
// (0 for exit code 0).
int run_cli(const std::string& args, const std::string& log) {
    const std::string cmd = quote(FUSE_CLI_PATH) + " " + args + " > " + quote(log) + " 2>&1";
    return std::system(cmd.c_str());
}

std::vector<std::vector<std::string>> read_csv(const std::string& path) {
    std::ifstream in(path);
    std::vector<std::vector<std::string>> rows;
    std::string line;
    while (std::getline(in, line)) {
        std::vector<std::string> fields;
        std::size_t start = 0;
        while (true) {
            const std::size_t pos = line.find(',', start);
            fields.push_back(line.substr(start, pos - start));
            if (pos == std::string::npos) break;
            start = pos + 1;
        }
        rows.push_back(fields);
    }
    return rows;
}

std::size_t column(const std::vector<std::string>& header, const std::string& name) {
    const auto it = std::find(header.begin(), header.end(), name);
    REQUIRE_MESSAGE(it != header.end(), "no column " << name);
    return static_cast<std::size_t>(it - header.begin());
}

std::string read_text(const std::string& path) {
    std::ifstream in(path);
    std::ostringstream s;
    s << in.rdbuf();
    return s.str();
}

// The number after `"key": {"median": ` style fields in summary.json.
double json_number(const std::string& json, const std::string& object, const std::string& key) {
    const auto o = json.find("\"" + object + "\"");
    REQUIRE(o != std::string::npos);
    const auto k = json.find("\"" + key + "\":", o);
    REQUIRE(k != std::string::npos);
    return std::stod(json.substr(k + key.size() + 3));
}

std::string frame_file(const std::string& prefix, std::size_t i, const std::string& ext) {
    std::string n = std::to_string(i);
    return prefix + std::string(6 - n.size(), '0') + n + ext;
}

}  // namespace

TEST_CASE("fuse_cli runs the synthetic sequence end to end") {
    const std::string root = std::string(FUSION_TEST_OUTPUT_DIR) + "/fuse_cli";
    fs::remove_all(root);
    const SyntheticSequence s = make_synthetic_sequence(5);
    const std::string manifest = write_synthetic_sequence(s, root + "/sequence");
    const std::string out = root + "/out";

    const int status = run_cli(quote(manifest) + " --out " + quote(out) + " --overlay", root + "/log.txt");
    INFO(read_text(root + "/log.txt"));
    REQUIRE(status == 0);
    const std::string log = read_text(root + "/log.txt");
    CHECK(log.find("5 frames") != std::string::npos);
    CHECK(log.find("median") != std::string::npos);
    CHECK(log.find("p95") != std::string::npos);

    // Files.
    for (const char* f : {"summary.csv", "summary.json", "association_rgb.csv",
                          "association_thermal.csv"}) {
        CHECK_MESSAGE(fs::is_regular_file(out + "/" + f), f);
    }
    for (std::size_t k = 0; k < 5; ++k) {
        CAPTURE(k);
        CHECK(fs::is_regular_file(out + "/" + frame_file("frame_", k, ".csv")));
        const std::string rgb = out + "/" + frame_file("overlay_rgb_", k, ".png");
        const std::string th = out + "/" + frame_file("overlay_thermal_", k, ".png");
        CHECK(fs::exists(rgb) == s.rgb.expected[k].accepted);
        CHECK(fs::exists(th) == s.thermal.expected[k].accepted);
        if (fs::exists(rgb)) CHECK(fusion::read_rgb_png(rgb).width == 640);
        if (fs::exists(th)) CHECK(fusion::read_rgb_png(th).width == 320);
    }

    // Summary rows against the generator and the per-frame CSVs.
    const auto summary = read_csv(out + "/summary.csv");
    REQUIRE(summary.size() == 6);
    const auto& h = summary[0];
    std::vector<double> process_ms;
    for (std::size_t k = 0; k < 5; ++k) {
        CAPTURE(k);
        const auto& r = summary[k + 1];
        REQUIRE(r.size() == h.size());
        CHECK(r[column(h, "frame")] == std::to_string(k));
        CHECK(r[column(h, "t_ref_ns")] == std::to_string(s.scans[k].t_ns));
        for (const auto& named : {std::pair{std::string("rgb"), &s.rgb},
                                  {std::string("thermal"), &s.thermal}}) {
            const std::string& name = named.first;
            const SyntheticCamera* cam = named.second;
            CAPTURE(name);
            const ExpectedMatch& e = cam->expected[k];
            CHECK(r[column(h, name + "_index")] == std::to_string(e.index));
            CHECK(r[column(h, name + "_t_ns")] == std::to_string(e.t_ns));
            CHECK(r[column(h, name + "_delta_t_ns")] == std::to_string(e.delta_t_ns));
            CHECK(r[column(h, name + "_accepted")] == (e.accepted ? "1" : "0"));
            CHECK(r[column(h, name + "_compensated")] == (e.accepted ? "1" : "0"));
        }

        const auto frame = read_csv(out + "/" + frame_file("frame_", k, ".csv"));
        REQUIRE(frame.size() > 1);
        const auto& fh = frame[0];
        CHECK(r[column(h, "points")] == std::to_string(frame.size() - 1));
        CHECK(frame.size() - 1 == s.scans[k].points.size());
        std::size_t rgb_visible = 0, thermal_visible = 0;
        for (std::size_t i = 1; i < frame.size(); ++i) {
            REQUIRE(frame[i].size() == fh.size());
            if (!frame[i][column(fh, "rgb_u")].empty()) ++rgb_visible;
            if (!frame[i][column(fh, "thermal_u")].empty()) {
                ++thermal_visible;
                CHECK_FALSE(frame[i][column(fh, "thermal_raw")].empty());
                CHECK_FALSE(frame[i][column(fh, "thermal_celsius")].empty());
            }
        }
        CHECK(r[column(h, "rgb_projected")] == std::to_string(rgb_visible));
        CHECK(r[column(h, "thermal_projected")] == std::to_string(thermal_visible));
        CHECK((rgb_visible > 0) == s.rgb.expected[k].accepted);
        CHECK((thermal_visible > 0) == s.thermal.expected[k].accepted);

        const double p = std::stod(r[column(h, "process_ms")]);
        CHECK(p >= 0.0);
        CHECK(std::stod(r[column(h, "read_ms")]) >= 0.0);
        process_ms.push_back(p);
    }

    // summary.json agrees with the summary rows: median of 5 is the third
    // smallest, nearest-rank p95 of 5 is the largest.
    const std::string json = read_text(out + "/summary.json");
    CHECK(json.find("\"frames\": 5") != std::string::npos);
    std::sort(process_ms.begin(), process_ms.end());
    CHECK(json_number(json, "process_ms", "median") == doctest::Approx(process_ms[2]).epsilon(1e-9));
    CHECK(json_number(json, "process_ms", "p95") == doctest::Approx(process_ms[4]).epsilon(1e-9));
    CHECK(json_number(json, "read_ms", "p95") >= json_number(json, "read_ms", "median"));

    // The association CSV has one row per scan.
    CHECK(read_csv(out + "/association_thermal.csv").size() == 6);
}

TEST_CASE("fuse_cli --frames limits the run and bad input fails") {
    const std::string root = std::string(FUSION_TEST_OUTPUT_DIR) + "/fuse_cli_frames";
    fs::remove_all(root);
    const std::string manifest =
        write_synthetic_sequence(make_synthetic_sequence(3), root + "/sequence");
    const std::string out = root + "/out";

    REQUIRE(run_cli(quote(manifest) + " --out " + quote(out) + " --frames 2", root + "/log.txt") == 0);
    CHECK(fs::exists(out + "/frame_000001.csv"));
    CHECK_FALSE(fs::exists(out + "/frame_000002.csv"));
    CHECK_FALSE(fs::exists(out + "/overlay_rgb_000001.png"));  // no --overlay
    CHECK(read_csv(out + "/summary.csv").size() == 3);

    CHECK(run_cli("--out " + quote(out), root + "/log2.txt") != 0);  // no manifest
    CHECK(read_text(root + "/log2.txt").find("usage:") != std::string::npos);
    CHECK(run_cli(quote(manifest) + " --out " + quote(out) + " --frames 0", root + "/log3.txt") != 0);
    CHECK(run_cli(quote(root + "/none.yaml") + " --out " + quote(out), root + "/log4.txt") != 0);
    CHECK(read_text(root + "/log4.txt").find("missing file") != std::string::npos);
    CHECK(run_cli("--help", root + "/log5.txt") == 0);
}
