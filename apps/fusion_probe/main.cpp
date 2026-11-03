// fusion_probe: small engine queries for validation tooling.
//
//   fusion_probe interpolate <trajectory.csv> --times <times.csv> --out <poses.csv>
//       Reads a trajectory (sequence format), builds fusion::Trajectory and
//       writes Trajectory::interpolate(t) for every time in <times.csv> (one
//       integer nanosecond time per line, an optional header line
//       `timestamp_ns`). Output: timestamp_ns,x,y,z,qw,qx,qy,qz.
//
//   fusion_probe gray <sequence.yaml> --out <dir> [--indices i,j,...]
//       Reads RGB frames of a sequence with the same reader fuse_cli uses
//       (read_rgb_frame, size checked against the camera config) and writes
//       each as 8-bit luminance, row-major without header:
//       gray_NNNNNN.u8, Y = round(0.299 R + 0.587 G + 0.114 B).
//       NNNNNN is the frame's index in the RGB list. Default: all frames.
//
// Exit codes: 0 success, 1 a processing error, 2 bad arguments.
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include <fusion/pose.hpp>
#include <fusion/sequence.hpp>

namespace fs = std::filesystem;

namespace {

const char* kUsage =
    "usage: fusion_probe interpolate <trajectory.csv> --times <times.csv> --out <poses.csv>\n"
    "       fusion_probe gray <sequence.yaml> --out <dir> [--indices i,j,...]\n";

struct UsageError {
    std::string what;
};

std::string num(double v) {
    char buf[64];
    const auto r = std::to_chars(buf, buf + sizeof(buf), v);
    return std::string(buf, r.ptr);
}

template <typename T>
T parse_int(const std::string& s, const std::string& what) {
    T v{};
    const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), v);
    if (ec != std::errc() || ptr != s.data() + s.size()) {
        throw std::runtime_error(what + ": not an integer: '" + s + "'");
    }
    return v;
}

std::string trim(std::string s) {
    while (!s.empty() && (s.back() == '\r' || s.back() == ' ')) s.pop_back();
    return s;
}

int run_interpolate(const std::vector<std::string>& args) {
    std::string traj, times, out;
    for (std::size_t i = 0; i < args.size(); ++i) {
        if (args[i] == "--times" && i + 1 < args.size()) {
            times = args[++i];
        } else if (args[i] == "--out" && i + 1 < args.size()) {
            out = args[++i];
        } else if (!args[i].empty() && args[i][0] != '-' && traj.empty()) {
            traj = args[i];
        } else {
            throw UsageError{"unexpected argument " + args[i]};
        }
    }
    if (traj.empty() || times.empty() || out.empty()) {
        throw UsageError{"interpolate needs <trajectory.csv>, --times and --out"};
    }
    const fusion::Trajectory trajectory = fusion::read_trajectory_csv(traj);

    std::ifstream in(times);
    if (!in) throw std::runtime_error("cannot read " + times);
    std::ofstream o(out);
    o << "timestamp_ns,x,y,z,qw,qx,qy,qz\n";
    std::string line;
    std::size_t lineno = 0, n = 0;
    while (std::getline(in, line)) {
        ++lineno;
        line = trim(line);
        if (line.empty() || (lineno == 1 && line == "timestamp_ns")) continue;
        const auto t = parse_int<fusion::TimeNs>(line, times + ":" + std::to_string(lineno));
        const fusion::StampedPose p = trajectory.interpolate(t);
        const Eigen::Quaterniond& q = p.orientation;
        o << t << ',' << num(p.position.x()) << ',' << num(p.position.y()) << ','
          << num(p.position.z()) << ',' << num(q.w()) << ',' << num(q.x()) << ',' << num(q.y())
          << ',' << num(q.z()) << '\n';
        ++n;
    }
    o.flush();
    if (!o) throw std::runtime_error("cannot write " + out);
    std::cout << "fusion_probe: " << n << " poses interpolated from " << trajectory.size()
              << " -> " << out << "\n";
    return 0;
}

int run_gray(const std::vector<std::string>& args) {
    std::string manifest, out, indices;
    for (std::size_t i = 0; i < args.size(); ++i) {
        if (args[i] == "--out" && i + 1 < args.size()) {
            out = args[++i];
        } else if (args[i] == "--indices" && i + 1 < args.size()) {
            indices = args[++i];
        } else if (!args[i].empty() && args[i][0] != '-' && manifest.empty()) {
            manifest = args[i];
        } else {
            throw UsageError{"unexpected argument " + args[i]};
        }
    }
    if (manifest.empty() || out.empty()) throw UsageError{"gray needs <sequence.yaml> and --out"};
    const fusion::Sequence seq = fusion::load_sequence(manifest);
    if (!seq.rgb) throw std::runtime_error(manifest + ": no rgb camera");

    std::vector<std::size_t> which;
    if (indices.empty()) {
        for (std::size_t i = 0; i < seq.rgb->frames.size(); ++i) which.push_back(i);
    } else {
        std::stringstream ss(indices);
        std::string tok;
        while (std::getline(ss, tok, ',')) {
            const auto i = parse_int<std::size_t>(trim(tok), "--indices");
            if (i >= seq.rgb->frames.size()) {
                throw std::runtime_error("--indices: " + tok + " is beyond the RGB list");
            }
            which.push_back(i);
        }
    }
    std::error_code ec;
    fs::create_directories(out, ec);
    if (ec || !fs::is_directory(out)) throw std::runtime_error("cannot create " + out);

    for (std::size_t i : which) {
        const fusion::RgbImage img = fusion::read_rgb_frame(seq.rgb->frames[i].path, seq.rgb->config);
        std::vector<std::uint8_t> y(img.pixels.size());
        for (std::size_t k = 0; k < img.pixels.size(); ++k) {
            const auto& p = img.pixels[k];
            y[k] = static_cast<std::uint8_t>(std::lround(0.299 * p.r + 0.587 * p.g + 0.114 * p.b));
        }
        char name[32];
        std::snprintf(name, sizeof(name), "gray_%06zu.u8", i);
        const std::string path = (fs::path(out) / name).string();
        std::ofstream f(path, std::ios::binary);
        f.write(reinterpret_cast<const char*>(y.data()), static_cast<std::streamsize>(y.size()));
        f.flush();
        if (!f) throw std::runtime_error("cannot write " + path);
    }
    std::cout << "fusion_probe: " << which.size() << " grey images (" << seq.rgb->config.width
              << "x" << seq.rgb->config.height << ") -> " << out << "\n";
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << kUsage;
        return 2;
    }
    const std::string cmd = argv[1];
    if (cmd == "-h" || cmd == "--help") {
        std::cout << kUsage;
        return 0;
    }
    const std::vector<std::string> args(argv + 2, argv + argc);
    try {
        if (cmd == "interpolate") return run_interpolate(args);
        if (cmd == "gray") return run_gray(args);
        throw UsageError{"unknown command " + cmd};
    } catch (const UsageError& e) {
        std::cerr << "fusion_probe: " << e.what << "\n\n" << kUsage;
        return 2;
    } catch (const std::exception& e) {
        std::cerr << "fusion_probe: " << e.what() << "\n";
        return 1;
    }
}
