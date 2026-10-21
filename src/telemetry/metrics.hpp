#pragma once

#include <array>
#include <cstdint>

#include "common/types.hpp"
#include "telemetry/latency_histogram.hpp"

namespace ptc::telemetry {

// One abort or fault, start to finish.
struct SafetyEvent {
  Reason reason = Reason::None;
  State from = State::Idle;              // state the recipe was in
  bool fault = false;                    // went (or escalated) to Fault
  bool escalated = false;                // started as Abort, escalated to Fault
  std::int64_t trip_ns = 0;              // tick on which it was detected
  std::int64_t detect_latency_ns = 0;    // trip tick - timestamp of the sample that showed it
  std::int64_t heater_off_latency_ns = 0;  // trip tick -> heater command 0 (same tick by design)
  std::int64_t safe_ns = -1;             // trip -> safe state (vented Idle, or de-energised Fault); -1 = not reached
};

// Written only by the control thread; read by main only after the control
// thread has been joined. Fixed-size: nothing here allocates on the hot path.
struct ControllerMetrics {
  std::uint64_t samples_consumed = 0;
  std::uint64_t buffer_overruns = 0;  // samples lost to ring overwrite (FR-6)
  std::uint64_t ticks_without_new_sample = 0;

  std::uint32_t wafers_processed = 0;
  std::uint32_t aborts = 0;
  std::uint32_t faults = 0;
  std::array<std::uint32_t, 8> interlock_trips{};  // indexed by safety::Interlock

  // Temperature control quality, measured on the sensor values the controller saw.
  double process_max_dev_c = 0.0;     // max |T - setpoint| while in Process
  double rampup_overshoot_c = 0.0;    // max (T - setpoint) in RampUp + Process
  std::int64_t last_cycle_ns = 0;
  std::int64_t total_cycle_ns = 0;

  static constexpr std::size_t kMaxSafetyEvents = 32;
  std::array<SafetyEvent, kMaxSafetyEvents> safety_events{};
  std::uint32_t safety_event_count = 0;
};

}  // namespace ptc::telemetry
