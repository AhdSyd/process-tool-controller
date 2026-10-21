#include "telemetry/report.hpp"

#include <iomanip>
#include <ostream>
#include <string>

namespace ptc::telemetry {
namespace {

double us(std::int64_t ns) { return static_cast<double>(ns) / 1000.0; }
double ms(std::int64_t ns) { return static_cast<double>(ns) / 1e6; }

void hist_row(std::ostream& os, const char* label, const LatencyHistogram& h) {
  os << "    " << std::left << std::setw(22) << label << std::right << std::fixed << std::setprecision(1)
     << std::setw(9) << us(h.percentile_ns(50.0)) << std::setw(9) << us(h.percentile_ns(99.0))
     << std::setw(9) << us(h.percentile_ns(99.9)) << std::setw(10) << us(h.max_ns()) << '\n';
}

std::string rt_text(const platform::RtStatus& rt) {
  if (rt.rt_granted) {
    std::string s = std::string(rt.policy) + " granted";
    if (rt.rt_priority > 0) s += ", priority " + std::to_string(rt.rt_priority);
    if (rt.pinned_cpu >= 0) s += ", cpu " + std::to_string(rt.pinned_cpu);
    return s;
  }
  if (rt.rt_errno == 0) return "normal scheduling (realtime not requested)";
  return std::string(rt.policy) + " NOT granted: " + platform::describe_errno(rt.rt_errno);
}

void loop_block(std::ostream& os, const char* name, const LoopSummary& l, double runtime_s) {
  const double period_us = std::chrono::duration<double, std::micro>(l.period).count();
  const double hz = runtime_s > 0 ? static_cast<double>(l.ticks) / runtime_s : 0.0;
  const double miss_pct =
      l.ticks ? 100.0 * static_cast<double>(l.missed_deadlines) / static_cast<double>(l.ticks) : 0.0;
  os << "  " << name << " thread  (" << std::fixed << std::setprecision(0) << 1e6 / period_us
     << " Hz target, " << rt_text(l.rt) << ")\n"
     << "    ticks " << l.ticks << " (" << std::setprecision(1) << hz << " Hz achieved), missed deadlines "
     << l.missed_deadlines << " (" << std::setprecision(3) << miss_pct << " %)\n"
     << "    timing (us)                 p50      p99    p99.9       max\n";
  hist_row(os, "wake lateness (jitter)", *l.wake_lateness);
  hist_row(os, "tick execution time", *l.exec_time);
  if (l.wake_lateness->overflow_count() > 0) {
    os << "    note: " << l.wake_lateness->overflow_count() << " wakeups were >= "
       << LatencyHistogram::kMaxTrackedUs / 1000 << " ms late\n";
  }
}

}  // namespace

void print_report(std::ostream& os, const ReportData& d) {
  const ControllerMetrics& c = *d.ctl;
  const double runtime_s = std::chrono::duration<double>(d.runtime).count();

  os << "\n======================== RUN REPORT ========================\n"
     << std::fixed << std::setprecision(3) << "  total runtime        " << runtime_s << " s\n";
  if (d.injected != FaultType::None) os << "  fault injected       " << fault_name(d.injected) << '\n';
  os << '\n';

  loop_block(os, "control", d.control, runtime_s);
  os << '\n';
  loop_block(os, "sensor ", d.sensor, runtime_s);

  os << "\n  sensor data path\n"
     << "    samples published    " << d.samples_published << '\n'
     << "    samples consumed     " << c.samples_consumed << '\n'
     << "    buffer overruns      " << c.buffer_overruns << "  (samples overwritten before the control thread read them)\n"
     << "    ticks w/o new sample " << c.ticks_without_new_sample << '\n';

  os << "\n  process\n"
     << "    wafers processed     " << c.wafers_processed << '\n';
  if (c.wafers_processed > 0) {
    os << std::setprecision(2) << "    mean cycle time      " << static_cast<double>(c.total_cycle_ns) / 1e9 / c.wafers_processed
       << " s/wafer  (" << std::setprecision(0) << 3600.0 * 1e9 * c.wafers_processed / static_cast<double>(c.total_cycle_ns)
       << " wafers/hour)\n"
       << std::setprecision(2) << "    max |T - setpoint|   " << c.process_max_dev_c << " C during Process (spec: 2.00)\n"
       << "    max overshoot        " << c.rampup_overshoot_c << " C\n";
  }
  os << "    aborts / faults      " << c.aborts << " / " << c.faults << '\n';

  static const char* kInterlockNames[] = {"OverTemperature", "CriticalOverTemperature", "OverPressureHeaterOn",
                                          "SlitValveOpenUnderVacuum", "StaleSensorData"};
  bool any_trip = false;
  for (std::size_t i = 0; i < 5; ++i) any_trip |= c.interlock_trips[i] > 0;
  if (any_trip) {
    os << "    interlock trips     ";
    for (std::size_t i = 0; i < 5; ++i)
      if (c.interlock_trips[i]) os << ' ' << kInterlockNames[i] << " x" << c.interlock_trips[i];
    os << '\n';
  }

  if (c.safety_event_count > 0) {
    os << "\n  safety events      reason                    from      detect  heater-off   to safe\n";
    for (std::uint32_t i = 0; i < c.safety_event_count; ++i) {
      const SafetyEvent& e = c.safety_events[i];
      os << "    " << std::left << std::setw(15)
         << (e.escalated ? "abort->FAULT" : e.fault ? "FAULT" : "abort") << std::setw(26) << reason_name(e.reason)
         << std::setw(9) << state_name(e.from) << std::right << std::setprecision(2) << std::setw(6)
         << ms(e.detect_latency_ns) << "ms" << std::setw(8) << us(e.heater_off_latency_ns) << "us";
      if (e.safe_ns >= 0) os << std::setw(8) << std::setprecision(1) << ms(e.safe_ns) << "ms";
      else os << "   (not reached)";
      os << '\n';
    }
    os << "    detect = age of the sample that showed the problem when the control tick acted on it\n";
  }

  os << "\n  logging              " << d.log_lines << " lines -> " << d.log_path << ", " << d.log_dropped
     << " dropped\n"
     << "  mlockall             " << platform::describe_errno(d.mlock_errno) << '\n'
     << "\n  Soft real-time on a general-purpose OS: these are measured values for\n"
     << "  this run on this machine, not guaranteed bounds.\n"
     << "============================================================\n";
}

}  // namespace ptc::telemetry
