// fusion_probe end to end on the generated synthetic sequence: trajectory
// interpolation through the engine and grey export of the RGB frames.
#include <doctest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include <fusion/pose.hpp>
#include <fusion/sequence.hpp>

#include "support/synthetic_sequence.hpp"

using namespace fusion::testing;
namespace fs = std::filesystem;

namespace {

std::string quote(const std::string& s) { return "\"" + s + "\""; }

int run_probe(const std::string& args, const std::string& log) {
    const std::string cmd = quote(FUSION_PROBE_PATH) + " " + args + " > " + quote(log) + " 2>&1";
    return std::system(cmd.c_str());
}

std::vector<std::string> split(const std::string& line) {
    std::vector<std::string> out;
    std::stringstream ss(line);
    for (std::string f; std::getline(ss, f, ',');) out.push_back(f);
    return out;
}

}  // namespace

TEST_CASE("fusion_probe interpolate reproduces Trajectory::interpolate") {
    const SyntheticSequence s = make_synthetic_sequence();
    const std::string root = std::string(FUSION_TEST_OUTPUT_DIR) + "/fusion_probe_interp";
    fs::remove_all(root);
    const std::string manifest = write_synthetic_sequence(s, root + "/seq");
    const fusion::Sequence seq = fusion::load_sequence(manifest);
    REQUIRE(seq.trajectory);
    const fusion::Trajectory& traj = *seq.trajectory;

    std::vector<fusion::TimeNs> times;
    for (int k = 0; k <= 20; ++k) {
        times.push_back(traj.start_ns() + (traj.end_ns() - traj.start_ns()) * k / 20 + (k % 3));
    }
    times.back() = traj.end_ns();
    {
        std::ofstream t(root + "/times.csv");
        t << "timestamp_ns\n";
        for (auto v : times) t << v << "\n";
    }
    REQUIRE(run_probe("interpolate " + quote(seq.trajectory_path) + " --times " +
                          quote(root + "/times.csv") + " --out " + quote(root + "/poses.csv"),
                      root + "/log.txt") == 0);

    std::ifstream in(root + "/poses.csv");
    std::string line;
    REQUIRE(std::getline(in, line));
    CHECK(line == "timestamp_ns,x,y,z,qw,qx,qy,qz");
    std::size_t n = 0;
    while (std::getline(in, line)) {
        const auto f = split(line);
        REQUIRE(f.size() == 8);
        REQUIRE(n < times.size());
        CHECK(std::stoll(f[0]) == times[n]);
        const fusion::StampedPose p = traj.interpolate(times[n]);
        // Shortest round-trip text, so the values read back exactly.
        CHECK(std::stod(f[1]) == p.position.x());
        CHECK(std::stod(f[2]) == p.position.y());
        CHECK(std::stod(f[3]) == p.position.z());
        CHECK(std::stod(f[4]) == p.orientation.w());
        CHECK(std::stod(f[5]) == p.orientation.x());
        CHECK(std::stod(f[6]) == p.orientation.y());
        CHECK(std::stod(f[7]) == p.orientation.z());
        ++n;
    }
    CHECK(n == times.size());

    // A time outside the trajectory is an error (no extrapolation).
    {
        std::ofstream t(root + "/outside.csv");
        t << traj.end_ns() + 1 << "\n";
    }
    CHECK(run_probe("interpolate " + quote(seq.trajectory_path) + " --times " +
                        quote(root + "/outside.csv") + " --out " + quote(root + "/o.csv"),
                    root + "/log2.txt") != 0);
}

TEST_CASE("fusion_probe gray writes the luminance of the RGB frames") {
    const SyntheticSequence s = make_synthetic_sequence();
    const std::string root = std::string(FUSION_TEST_OUTPUT_DIR) + "/fusion_probe_gray";
    fs::remove_all(root);
    const std::string manifest = write_synthetic_sequence(s, root + "/seq");
    REQUIRE(run_probe("gray " + quote(manifest) + " --out " + quote(root + "/out") +
                          " --indices 0,2",
                      root + "/log.txt") == 0);
    const fusion::Sequence seq = fusion::load_sequence(manifest);
    for (std::size_t i : {std::size_t{0}, std::size_t{2}}) {
        const fusion::RgbImage img = fusion::read_rgb_frame(seq.rgb->frames[i].path, seq.rgb->config);
        std::ifstream f(root + "/out/gray_00000" + std::to_string(i) + ".u8", std::ios::binary);
        const std::vector<unsigned char> y((std::istreambuf_iterator<char>(f)),
                                           std::istreambuf_iterator<char>());
        REQUIRE(y.size() == img.pixels.size());
        std::size_t bad = 0;
        for (std::size_t k = 0; k < y.size(); ++k) {
            const auto& p = img.pixels[k];
            const long expect = std::lround(0.299 * p.r + 0.587 * p.g + 0.114 * p.b);
            if (static_cast<long>(y[k]) != expect) ++bad;
        }
        CHECK(bad == 0);
    }
    CHECK_FALSE(fs::exists(root + "/out/gray_000001.u8"));
    CHECK(run_probe("gray " + quote(manifest) + " --out " + quote(root + "/out2") +
                        " --indices 999",
                    root + "/log2.txt") != 0);
}
