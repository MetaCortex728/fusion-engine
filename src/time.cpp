#include "fusion/time.hpp"

#include <algorithm>
#include <fstream>
#include <limits>
#include <ostream>
#include <stdexcept>

namespace fusion {

namespace {

constexpr TimeNs kMin = std::numeric_limits<TimeNs>::min();
constexpr TimeNs kMax = std::numeric_limits<TimeNs>::max();

TimeNs checked_add(TimeNs a, TimeNs b) {
    if ((b > 0 && a > kMax - b) || (b < 0 && a < kMin - b)) {
        throw std::overflow_error("fusion: time does not fit in int64 nanoseconds");
    }
    return a + b;
}

TimeNs checked_sub(TimeNs a, TimeNs b) {
    if ((b < 0 && a > kMax + b) || (b > 0 && a < kMin + b)) {
        throw std::overflow_error("fusion: time difference does not fit in int64 nanoseconds");
    }
    return a - b;
}

// |d| without overflow for d == INT64_MIN.
std::uint64_t magnitude(TimeNs d) {
    return d < 0 ? std::uint64_t{0} - static_cast<std::uint64_t>(d)
                 : static_cast<std::uint64_t>(d);
}

void require_sorted(const std::vector<TimeNs>& t, const char* what) {
    if (!std::is_sorted(t.begin(), t.end())) {
        throw std::invalid_argument(std::string("fusion::associate: ") + what +
                                    " times are not sorted");
    }
}

}  // namespace

std::vector<TimeNs> to_reference_time(const std::vector<TimeNs>& t_sensor_ns,
                                      TimeNs time_offset_ns) {
    std::vector<TimeNs> out;
    out.reserve(t_sensor_ns.size());
    for (const TimeNs t : t_sensor_ns) out.push_back(checked_add(t, time_offset_ns));
    return out;
}

TimeNs default_tau_ns(TimeNs period_a_ns, TimeNs period_b_ns) {
    if (period_a_ns <= 0 || period_b_ns <= 0) {
        throw std::invalid_argument("fusion::default_tau_ns: periods must be positive");
    }
    return std::max(period_a_ns, period_b_ns) / 2;
}

std::vector<Association> associate(const std::vector<TimeNs>& reference_ns,
                                   const std::vector<TimeNs>& other_ns, TimeNs tau_ns) {
    if (tau_ns < 0) throw std::invalid_argument("fusion::associate: tau must not be negative");
    require_sorted(reference_ns, "reference");
    require_sorted(other_ns, "other sensor");

    std::vector<Association> out;
    if (reference_ns.empty() || other_ns.empty()) return out;
    out.reserve(reference_ns.size());
    const std::uint64_t tau = static_cast<std::uint64_t>(tau_ns);

    for (const TimeNs t_ref : reference_ns) {
        // First measurement at or after t_ref; the nearest is it or the one
        // before. For equal times, lower_bound lands on the first of them.
        const auto after = std::lower_bound(other_ns.begin(), other_ns.end(), t_ref);
        std::size_t best = 0;
        if (after == other_ns.end()) {
            best = other_ns.size() - 1;
            // Move to the first of several equal last times.
            while (best > 0 && other_ns[best - 1] == other_ns[best]) --best;
        } else if (after == other_ns.begin()) {
            best = 0;
        } else {
            const std::size_t i_after = static_cast<std::size_t>(after - other_ns.begin());
            std::size_t i_before = i_after - 1;
            while (i_before > 0 && other_ns[i_before - 1] == other_ns[i_before]) --i_before;
            const std::uint64_t d_before = magnitude(checked_sub(other_ns[i_before], t_ref));
            const std::uint64_t d_after = magnitude(checked_sub(other_ns[i_after], t_ref));
            // On a tie the earlier measurement wins.
            best = d_before <= d_after ? i_before : i_after;
        }
        Association a;
        a.t_ref_ns = t_ref;
        a.match_index = best;
        a.t_match_ns = other_ns[best];
        a.delta_t_ns = checked_sub(a.t_match_ns, t_ref);
        a.accepted = magnitude(a.delta_t_ns) <= tau;
        out.push_back(a);
    }
    return out;
}

std::vector<Association> associate(const std::vector<TimeNs>& reference_sensor_ns,
                                   TimeNs reference_offset_ns,
                                   const std::vector<TimeNs>& other_sensor_ns,
                                   TimeNs other_offset_ns, TimeNs tau_ns) {
    return associate(to_reference_time(reference_sensor_ns, reference_offset_ns),
                     to_reference_time(other_sensor_ns, other_offset_ns), tau_ns);
}

void write_association_csv(std::ostream& out, const std::vector<Association>& results) {
    out << "t_ref_ns,t_match_ns,delta_t_ns,accepted\n";
    for (const Association& a : results) {
        out << a.t_ref_ns << ',' << a.t_match_ns << ',' << a.delta_t_ns << ','
            << (a.accepted ? 1 : 0) << '\n';
    }
}

void write_association_csv(const std::string& path, const std::vector<Association>& results) {
    std::ofstream file(path);
    if (!file) throw std::runtime_error("fusion::write_association_csv: cannot open " + path);
    write_association_csv(file, results);
    file.flush();
    if (!file) throw std::runtime_error("fusion::write_association_csv: cannot write " + path);
}

}  // namespace fusion
