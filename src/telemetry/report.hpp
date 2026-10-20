#pragma once

#include <chrono>
#include <cstdint>
#include <iosfwd>

#include "platform/rt_thread.hpp"
#include "telemetry/latency_histogram.hpp"

namespace ptc::telemetry {

// Scalar results gathered by the main thread after all worker threads have
// been joined. Later milestones add fields (buffer overruns, wafers, ...).
struct RunMetrics {
  std::chrono::nanoseconds period{};
  std::chrono::nanoseconds runtime{};
  std::uint64_t ticks = 0;
  std::uint64_t missed_deadlines = 0;
  platform::RtStatus rt{};
  int mlock_errno = 0;
};

// Histograms are passed by const reference: the report borrows them, the
// ControlLoop owns them.
void print_report(std::ostream& os, const RunMetrics& m, const LatencyHistogram& wake_lateness,
                  const LatencyHistogram& exec_time);

}  // namespace ptc::telemetry
