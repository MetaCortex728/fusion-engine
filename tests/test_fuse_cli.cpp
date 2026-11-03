// fuse_cli end to end on the generated synthetic sequence.
#include <doctest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <fusion/image_io.hpp>

#include "support/mini_json.hpp"
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

namespace {

// The JSON embedded in viewer/index.html, between the opening
// <script type="application/json" id="fusion-data"> and the next </script>.
std::string embedded_json(const std::string& html) {
    const std::string open = "<script type=\"application/json\" id=\"fusion-data\">";
    const auto a = html.find(open);
    REQUIRE(a != std::string::npos);
    const auto b = html.find("</script>", a);
    REQUIRE(b != std::string::npos);
    return html.substr(a + open.size(), b - a - open.size());
}

// The page's embedded data with each frame's points file merged in as
// frames[k][camera]["points"]. Checks that points file k is the call
// fusionPoints(k, {...}); and holds an entry for each camera of the frame.
fusion::testing::json::Value load_viewer_data(const std::string& viewer_dir) {
    namespace json = fusion::testing::json;
    json::Value data = json::parse(embedded_json(read_text(viewer_dir + "/index.html")));
    std::vector<json::Value>& frames = data.object.at("frames").array;
    for (std::size_t k = 0; k < frames.size(); ++k) {
        CAPTURE(k);
        const std::string name = frames[k]["points_file"].string;
        char expected[48];
        std::snprintf(expected, sizeof(expected), "points/frame_%06zu.js", k);
        CHECK(name == expected);
        const std::string js = read_text(viewer_dir + "/" + name);
        const std::string open = "fusionPoints(" + std::to_string(k) + ",";
        const std::string close = ");\n";
        REQUIRE(js.size() > open.size() + close.size());
        REQUIRE(js.compare(0, open.size(), open) == 0);
        REQUIRE(js.compare(js.size() - close.size(), close.size(), close) == 0);
        const json::Value points =
            json::parse(js.substr(open.size(), js.size() - open.size() - close.size()));
        for (const char* cam : {"rgb", "thermal"}) {
            json::Value& c = frames[k].object.at(cam);
            CHECK_FALSE(c.has("points"));
            CHECK(points[cam].is_null() == c.is_null());
            if (!c.is_null()) c.object["points"] = points[cam];
        }
    }
    return data;
}

// Point indices of the rows of a frame CSV visible in `camera`, in scan order.
std::vector<std::size_t> visible_points(const std::vector<std::vector<std::string>>& frame,
                                        const std::string& camera) {
    std::vector<std::size_t> v;
    const std::size_t u = column(frame[0], camera + "_u");
    for (std::size_t r = 1; r < frame.size(); ++r) {
        if (!frame[r][u].empty()) v.push_back(r - 1);
    }
    return v;
}

}  // namespace

TEST_CASE("fuse_cli --viewer writes a self-contained page whose data matches the run") {
    namespace json = fusion::testing::json;
    const std::string root = std::string(FUSION_TEST_OUTPUT_DIR) + "/fuse_cli_viewer";
    fs::remove_all(root);
    const SyntheticSequence s = make_synthetic_sequence(5);
    const std::string manifest = write_synthetic_sequence(s, root + "/sequence");
    const std::string out = root + "/out";

    const int status = run_cli(quote(manifest) + " --out " + quote(out) + " --viewer", root + "/log.txt");
    INFO(read_text(root + "/log.txt"));
    REQUIRE(status == 0);
    CHECK(read_text(root + "/log.txt").find("viewer/index.html") != std::string::npos);

    const std::string page_path = out + "/viewer/index.html";
    REQUIRE(fs::is_regular_file(page_path));
    const std::string html = read_text(page_path);

    // No external scripts, styles or URLs: the only files the page loads are
    // its own points scripts and images next to it.
    CHECK(html.find("<script src") == std::string::npos);
    CHECK(html.find("<link") == std::string::npos);
    CHECK(html.find("http://") == std::string::npos);
    CHECK(html.find("https://") == std::string::npos);
    CHECK(html.find("fetch(") == std::string::npos);
    CHECK(html.find("XMLHttpRequest") == std::string::npos);

    const json::Value data = load_viewer_data(out + "/viewer");
    CHECK(data["format"].number == 2);
    CHECK(data["sequence"].string == manifest);
    CHECK(data["cameras"]["rgb"]["width"].number == 640);
    CHECK(data["cameras"]["thermal"]["width"].number == 320);
    CHECK(data["max_points_per_camera"].number == 20000);
    const json::Value& frames = data["frames"];
    REQUIRE(frames.size() == 5);

    const auto summary = read_csv(out + "/summary.csv");
    const auto& h = summary[0];
    for (std::size_t k = 0; k < 5; ++k) {
        CAPTURE(k);
        const json::Value& f = frames[k];
        CHECK(f["index"].number == static_cast<double>(k));
        CHECK(f["t_ref_ns"].string == std::to_string(s.scans[k].t_ns));
        CHECK(f["points"].number == static_cast<double>(s.scans[k].points.size()));
        const auto frame = read_csv(out + "/" + frame_file("frame_", k, ".csv"));
        for (const auto& named : {std::pair{std::string("rgb"), &s.rgb},
                                  {std::string("thermal"), &s.thermal}}) {
            const std::string& name = named.first;
            CAPTURE(name);
            const ExpectedMatch& e = named.second->expected[k];
            const json::Value& c = f[name];
            CHECK(c["accepted"].boolean == e.accepted);
            CHECK(c["compensated"].boolean == e.accepted);
            CHECK(static_cast<long long>(c["delta_t_ns"].number) == e.delta_t_ns);
            CHECK(c["match_index"].number == static_cast<double>(e.index));
            CHECK(c["t_match_ns"].string == std::to_string(e.t_ns));
            CHECK(std::to_string(static_cast<long long>(c["projected"].number)) ==
                  summary[k + 1][column(h, name + "_projected")]);

            // Every visible point is embedded (fewer than the cap), with the
            // CSV's pixel, depth and nearest flag at the page's precision.
            const std::vector<std::size_t> visible = visible_points(frame, name);
            const json::Value& p = c["points"];
            CHECK(c["embedded"].number == static_cast<double>(visible.size()));
            CHECK_FALSE(c["subsampled"].boolean);
            REQUIRE(p["i"].size() == visible.size());
            for (const char* key : {"u", "v", "depth", "nearest", "thermal"}) {
                CHECK(p[key].size() == visible.size());
            }
            std::size_t hidden = 0;
            const auto& fh = frame[0];
            for (std::size_t j = 0; j < visible.size(); ++j) {
                const std::size_t i = visible[j];
                REQUIRE(p["i"][j].number == static_cast<double>(i));
                const auto& row = frame[i + 1];
                CHECK(std::abs(p["u"][j].number - std::stod(row[column(fh, name + "_u")])) <= 0.0051);
                CHECK(std::abs(p["v"][j].number - std::stod(row[column(fh, name + "_v")])) <= 0.0051);
                CHECK(std::abs(p["depth"][j].number - std::stod(row[column(fh, name + "_depth")])) <=
                      0.00051);
                const std::string nearest = row[column(fh, name + "_nearest")];
                CHECK((p["nearest"][j].number == 1) == (nearest == "1"));
                if (nearest != "1") ++hidden;
                // The thermal value is the point's °C in the thermal camera.
                const std::string celsius = row[column(fh, "thermal_celsius")];
                CHECK(p["thermal"][j].is_null() == celsius.empty());
                if (!celsius.empty()) {
                    CHECK(std::abs(p["thermal"][j].number - std::stod(celsius)) <= 0.0051);
                }
            }
            CHECK(c["hidden"].number == static_cast<double>(hidden));

            // The image the page shows sits next to it.
            if (e.accepted) {
                REQUIRE(c["image"].type == json::Value::Type::String);
                const std::string img = out + "/viewer/" + c["image"].string;
                REQUIRE(fs::is_regular_file(img));
                CHECK(fusion::read_rgb_png(img).width == (name == "rgb" ? 640 : 320));
            } else {
                CHECK(c["image"].is_null());
                CHECK(p["i"].size() == 0);
            }
        }
    }

    // Run statistics agree with summary.json and the per-frame timings.
    const std::string summary_json = read_text(out + "/summary.json");
    const json::Value& run = data["run"];
    CHECK(run["frames"].number == 5);
    CHECK(run["process_ms"]["median"].number ==
          doctest::Approx(json_number(summary_json, "process_ms", "median")).epsilon(1e-9));
    CHECK(run["process_ms"]["p95"].number ==
          doctest::Approx(json_number(summary_json, "process_ms", "p95")).epsilon(1e-9));
    REQUIRE(run["frame_ms"]["process"].size() == 5);
    for (std::size_t k = 0; k < 5; ++k) {
        CHECK(run["frame_ms"]["process"][k].number ==
              doctest::Approx(std::stod(summary[k + 1][column(h, "process_ms")])).epsilon(1e-9));
    }
    REQUIRE(run["depth_range"].size() == 2);
    CHECK(run["depth_range"][0].number > 0.0);
    CHECK(run["depth_range"][1].number > run["depth_range"][0].number);
    REQUIRE(run["thermal_range"].size() == 2);
    CHECK(data["thermal_unit"].string == "\xC2\xB0" "C");
}

TEST_CASE("fuse_cli --viewer-max-points subsamples deterministically") {
    namespace json = fusion::testing::json;
    const std::string root = std::string(FUSION_TEST_OUTPUT_DIR) + "/fuse_cli_viewer_cap";
    fs::remove_all(root);
    const std::string manifest =
        write_synthetic_sequence(make_synthetic_sequence(3), root + "/sequence");
    const std::string out = root + "/out";
    const std::size_t cap = 100;

    REQUIRE(run_cli(quote(manifest) + " --out " + quote(out) + " --viewer --viewer-max-points " +
                        std::to_string(cap),
                    root + "/log.txt") == 0);
    const json::Value data = load_viewer_data(out + "/viewer");
    CHECK(data["max_points_per_camera"].number == static_cast<double>(cap));
    REQUIRE(data["frames"].size() == 3);
    std::size_t subsampled = 0;
    for (std::size_t k = 0; k < 3; ++k) {
        CAPTURE(k);
        const auto frame = read_csv(out + "/" + frame_file("frame_", k, ".csv"));
        for (const std::string name : {"rgb", "thermal"}) {
            CAPTURE(name);
            const json::Value& c = data["frames"][k][name];
            const std::vector<std::size_t> visible = visible_points(frame, name);
            const std::size_t m = visible.size();
            CHECK(c["projected"].number == static_cast<double>(m));
            const std::size_t n = std::min(m, cap);
            REQUIRE(c["points"]["i"].size() == n);
            CHECK(c["embedded"].number == static_cast<double>(n));
            CHECK(c["subsampled"].boolean == (m > cap));
            if (m > cap) ++subsampled;
            for (std::size_t j = 0; j < n; ++j) {
                const std::size_t rank = m > cap ? j * m / cap : j;
                CHECK(c["points"]["i"][j].number == static_cast<double>(visible[rank]));
            }
        }
    }
    CHECK(subsampled > 0);  // the synthetic cameras see several hundred points

    // Same input, same page data.
    REQUIRE(run_cli(quote(manifest) + " --out " + quote(root + "/out2") +
                        " --viewer --viewer-max-points " + std::to_string(cap),
                    root + "/log2.txt") == 0);
    const json::Value again = load_viewer_data(root + "/out2/viewer");
    for (std::size_t k = 0; k < 3; ++k) {
        for (const std::string name : {"rgb", "thermal"}) {
            const auto& a = data["frames"][k][name]["points"]["i"].array;
            const auto& b = again["frames"][k][name]["points"]["i"].array;
            REQUIRE(a.size() == b.size());
            for (std::size_t j = 0; j < a.size(); ++j) CHECK(a[j].number == b[j].number);
        }
    }

    CHECK(run_cli(quote(manifest) + " --out " + quote(out) + " --viewer-max-points 0",
                  root + "/log3.txt") != 0);
    CHECK(read_text(root + "/log3.txt").find("--viewer-max-points") != std::string::npos);
}

TEST_CASE("the test JSON reader rejects invalid JSON") {
    namespace json = fusion::testing::json;
    CHECK(json::parse("{\"a\": [1, -2.5e3, true, null, \"x\\u00b0\"]}")["a"][1].number == -2500);
    CHECK_THROWS(json::parse("{\"a\": 1,}"));
    CHECK_THROWS(json::parse("[1 2]"));
    CHECK_THROWS(json::parse("[NaN]"));
    CHECK_THROWS(json::parse("{\"a\": 1} x"));
    CHECK_THROWS(json::parse("[01]"));
}
