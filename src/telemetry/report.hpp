#pragma once

#include <chrono>
#include <cstdint>
#include <iosfwd>
#include <string>

#include "platform/rt_thread.hpp"
#include "telemetry/latency_histogram.hpp"
#include "telemetry/metrics.hpp"

namespace ptc::telemetry {

struct LoopSummary {
  std::chrono::nanoseconds period{};
  std::uint64_t ticks = 0;
  std::uint64_t missed_deadlines = 0;
  const LatencyHistogram* wake_lateness = nullptr;  // borrowed from the PeriodicLoop
  const LatencyHistogram* exec_time = nullptr;
  platform::RtStatus rt{};
};

// Everything gathered by the main thread AFTER every worker thread has been
// joined. All pointers are non-owning borrows of objects main still owns.
struct ReportData {
  std::chrono::nanoseconds runtime{};
  LoopSummary control;
  LoopSummary sensor;
  std::uint64_t samples_published = 0;
  const ControllerMetrics* ctl = nullptr;
  std::uint64_t log_lines = 0;
  std::uint64_t log_dropped = 0;
  std::string log_path;
  int mlock_errno = 0;
  FaultType injected = FaultType::None;
};

void print_report(std::ostream& os, const ReportData& d);

}  // namespace ptc::telemetry
