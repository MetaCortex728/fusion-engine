// Static web viewer for a fuse_cli run.
//
// ViewerWriter writes <out>/viewer/index.html: one self-contained page (inline
// CSS and JavaScript, no network requests, no external libraries) with the
// run's data embedded as JSON in a <script type="application/json"
// id="fusion-data"> element, so it works when opened from file://. The camera
// images it shows are written next to it (rgb_NNNNNN.png, thermal_NNNNNN.png,
// numbered by the camera's image index) and referenced by relative path.
//
// Timestamps (t_ref_ns, t_match_ns) are embedded as decimal strings, because
// epoch nanoseconds do not fit exactly in a JavaScript number.
//
// The page is written while the run progresses, one frame at a time, so a
// long run does not hold every frame's points in memory.
//
// Per camera and frame at most `max_points` projected points are embedded.
// When a camera sees m > max_points points, the points are taken in scan
// order and the k-th kept point (k = 0 .. max_points - 1) is the one with
// rank floor(k * m / max_points). The selection depends only on m and
// max_points, so the same run always embeds the same points.
#pragma once

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <fusion/aligned_frame.hpp>
#include <fusion/image_io.hpp>
#include <fusion/sequence.hpp>

namespace fuse_cli {

/// Median and nearest-rank p95.
struct Stats {
    double median = 0.0;
    double p95 = 0.0;
};

/// The default cap on embedded points per camera and frame.
constexpr std::size_t kDefaultViewerMaxPoints = 3000;

/// Ranks (0-based, among m items) kept by the deterministic subsampling:
/// all of them when m <= max_points, else floor(k * m / max_points) for
/// k = 0 .. max_points - 1. Strictly increasing.
std::vector<std::size_t> subsample_ranks(std::size_t m, std::size_t max_points);

class ViewerWriter {
public:
    /// Creates <out_dir>/viewer and starts index.html. Throws
    /// std::runtime_error if it cannot.
    ViewerWriter(const std::string& out_dir, const fusion::Sequence& seq,
                 const std::string& manifest, std::size_t max_points);

    /// Appends one frame. `rgb` and `thermal_display` are the images the
    /// frame's accepted cameras show (the RGB frame and the rendered thermal
    /// frame); each is written once per camera image index.
    void add_frame(std::size_t index, const fusion::AlignedFrame& frame,
                   const fusion::RgbImage* rgb, const fusion::RgbImage* thermal_display);

    /// Writes the run's statistics and per-frame timings and closes the page.
    void finish(const std::vector<double>& read_ms, const std::vector<double>& process_ms,
                const std::vector<double>& write_ms, const Stats& read, const Stats& process,
                const Stats& write);

    std::string index_path() const { return index_path_; }

private:
    void write_camera(const char* key, const std::optional<fusion::CameraFrame>& cam,
                      const fusion::AlignedFrame& frame, const fusion::RgbImage* image);

    std::string dir_;
    std::string index_path_;
    std::ofstream out_;
    std::size_t max_points_;
    std::size_t frames_ = 0;
    bool thermal_celsius_ = false;  ///< thermal values are °C, else raw counts
    std::set<std::size_t> written_rgb_, written_thermal_;
    double depth_lo_, depth_hi_, thermal_lo_, thermal_hi_;
};

}  // namespace fuse_cli
