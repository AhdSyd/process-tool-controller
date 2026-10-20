#include "telemetry/report.hpp"

#include <iomanip>
#include <ostream>
#include <string>

namespace ptc::telemetry {
namespace {

double us(std::int64_t ns) { return static_cast<double>(ns) / 1000.0; }

void print_hist_row(std::ostream& os, const char* label, const LatencyHistogram& h) {
  os << "  " << std::left << std::setw(16) << label << std::right << std::fixed
     << std::setprecision(1)                                       //
     << std::setw(9) << us(h.percentile_ns(50.0))                  //
     << std::setw(9) << us(h.percentile_ns(99.0))                  //
     << std::setw(9) << us(h.percentile_ns(99.9))                  //
     << std::setw(10) << us(h.max_ns())                            //
     << std::setw(9) << h.mean_ns() / 1000.0 << '\n';
}

}  // namespace

void print_report(std::ostream& os, const RunMetrics& m, const LatencyHistogram& wake_lateness,
                  const LatencyHistogram& exec_time) {
  const double runtime_s = std::chrono::duration<double>(m.runtime).count();
  const double period_us = std::chrono::duration<double, std::micro>(m.period).count();
  const double achieved_hz = runtime_s > 0 ? static_cast<double>(m.ticks) / runtime_s : 0.0;
  const double miss_pct =
      m.ticks ? 100.0 * static_cast<double>(m.missed_deadlines) / static_cast<double>(m.ticks) : 0.0;

  os << "\n================ RUN REPORT ================\n"
     << std::fixed << std::setprecision(3)
     << "  runtime            " << runtime_s << " s\n"
     << std::setprecision(1)
     << "  tick period        " << period_us << " us (target " << 1e6 / period_us << " Hz)\n"
     << "  ticks executed     " << m.ticks << "  (" << achieved_hz << " Hz achieved)\n"
     << "  missed deadlines   " << m.missed_deadlines << std::setprecision(4) << "  ("
     << miss_pct << " % of ticks)\n"
     << "\n  timing (us)            p50      p99    p99.9       max     mean\n";
  print_hist_row(os, "wake lateness", wake_lateness);
  print_hist_row(os, "tick exec time", exec_time);
  if (wake_lateness.overflow_count() > 0) {
    os << "  note: " << wake_lateness.overflow_count() << " wakeups were >= "
       << LatencyHistogram::kMaxTrackedUs / 1000 << " ms late\n";
  }

  os << "\n  scheduling\n"
     << "    SCHED_FIFO       "
     << (m.rt.fifo_granted ? "granted, priority " + std::to_string(m.rt.fifo_priority)
         : m.rt.fifo_errno == 0
             ? std::string("not requested")
             : std::string("not granted: ") + platform::describe_errno(m.rt.fifo_errno))
     << '\n'
     << "    timer slack      " << (m.rt.timer_slack_set ? "1 ns" : "default (~50 us)") << '\n'
     << "    cpu pinning      "
     << (m.rt.pinned_cpu >= 0 ? "cpu " + std::to_string(m.rt.pinned_cpu) : std::string("none"))
     << '\n'
     << "    mlockall         " << platform::describe_errno(m.mlock_errno) << '\n'
     << "\n  Soft real-time on a general-purpose OS: these are measured\n"
     << "  values for this run on this machine, not guaranteed bounds.\n"
     << "============================================\n";
}

}  // namespace ptc::telemetry
