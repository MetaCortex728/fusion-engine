// Writes the synthetic test sequence (see tests/support/synthetic_sequence.hpp)
// in the on-disk sequence format, so fuse_cli can run without any dataset.
//
//   make_synthetic_sequence <dir> [--scans N]     (1 <= N <= 5, default 5)
#include <charconv>
#include <exception>
#include <iostream>
#include <string>
#include <system_error>

#include "support/synthetic_sequence.hpp"

int main(int argc, char** argv) {
    std::string dir;
    int scans = 5;
    bool ok = true;
    for (int i = 1; i < argc && ok; ++i) {
        const std::string arg = argv[i];
        if (arg == "--scans" && i + 1 < argc) {
            const std::string v = argv[++i];
            const auto [ptr, ec] = std::from_chars(v.data(), v.data() + v.size(), scans);
            ok = ec == std::errc() && ptr == v.data() + v.size();
        } else if (dir.empty() && !arg.empty() && arg[0] != '-') {
            dir = arg;
        } else {
            ok = false;
        }
    }
    if (!ok || dir.empty()) {
        std::cerr << "usage: make_synthetic_sequence <dir> [--scans N]   (1 <= N <= 5)\n";
        return 2;
    }
    try {
        const auto seq = fusion::testing::make_synthetic_sequence(scans);
        std::cout << fusion::testing::write_synthetic_sequence(seq, dir) << "\n";
    } catch (const std::exception& e) {
        std::cerr << "make_synthetic_sequence: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
