// Timestamps and nearest-neighbour time association between sensors.
//
// All times are integer nanoseconds (std::int64_t). There are no
// floating-point seconds in this interface.
//
// Every sensor stamps its measurements on its own clock. Its config's
// `time_offset_ns` moves those stamps onto the reference clock:
//   t_reference = t_sensor + time_offset_ns
// Association runs on reference-clock times.
#pragma once

#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <string>
#include <vector>

namespace fusion {

/// Time in integer nanoseconds.
using TimeNs = std::int64_t;

/// Applies a sensor's time offset: t_reference = t_sensor + time_offset_ns
/// for every stamp. Throws std::overflow_error if a result does not fit in
/// TimeNs.
std::vector<TimeNs> to_reference_time(const std::vector<TimeNs>& t_sensor_ns,
                                      TimeNs time_offset_ns);

/// Default association threshold: half the slower sensor's period, i.e.
/// max(period_a_ns, period_b_ns) / 2, rounded down to whole nanoseconds.
/// Throws std::invalid_argument unless both periods are positive.
TimeNs default_tau_ns(TimeNs period_a_ns, TimeNs period_b_ns);

/// The nearest measurement of another sensor for one reference time.
struct Association {
    TimeNs t_ref_ns = 0;      ///< reference time (reference clock)
    std::size_t match_index = 0;  ///< index of the nearest measurement
    TimeNs t_match_ns = 0;    ///< its time (reference clock)
    TimeNs delta_t_ns = 0;    ///< t_match_ns - t_ref_ns
    bool accepted = false;    ///< |delta_t_ns| <= tau
};

/// For each reference time t_ref, picks the measurement j of `other_ns` that
/// minimizes |t_j - t_ref| and accepts it only if |t_j - t_ref| <= tau_ns.
/// On an exact tie the earlier measurement (the smaller time, or the
/// smaller index for equal times) wins.
///
/// Both inputs are reference-clock times and must be sorted in
/// non-decreasing order. The result has one entry per reference time, in
/// order, including rejected candidates, so their delta_t stays visible.
/// It is empty if either input is empty: with no measurements there is no
/// candidate.
///
/// Throws std::invalid_argument for a negative tau or unsorted input, and
/// std::overflow_error if a time difference does not fit in TimeNs.
std::vector<Association> associate(const std::vector<TimeNs>& reference_ns,
                                   const std::vector<TimeNs>& other_ns, TimeNs tau_ns);

/// Same, for sensor-clock stamps: applies each sensor's time offset with
/// to_reference_time() first, then associates. Pass each sensor's
/// SensorConfig::time_offset_ns. The results hold reference-clock times.
std::vector<Association> associate(const std::vector<TimeNs>& reference_sensor_ns,
                                   TimeNs reference_offset_ns,
                                   const std::vector<TimeNs>& other_sensor_ns,
                                   TimeNs other_offset_ns, TimeNs tau_ns);

/// Writes the results as CSV with the header
///   t_ref_ns,t_match_ns,delta_t_ns,accepted
/// and one row per association; `accepted` is 1 or 0.
void write_association_csv(std::ostream& out, const std::vector<Association>& results);

/// Same, to a file. Throws std::runtime_error if it cannot be written.
void write_association_csv(const std::string& path, const std::vector<Association>& results);

}  // namespace fusion
