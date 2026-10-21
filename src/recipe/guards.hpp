#pragma once

// Transition guards (FR-2). Every guard is a pure function of its arguments:
// no globals, no clock reads, no side effects. That is what makes each one
// unit-testable in isolation with a hand-built SensorSample.
//
// The one guard that needs history ("in band for 500 consecutive ms") is
// split into a pure accumulator (update_in_band) and a pure check
// (settled): the history is an explicit argument, not hidden state.

#include <cmath>
#include <cstdint>

#include "common/types.hpp"

namespace ptc::recipe::guards {

// PumpDown -> RampUp: chamber pressure strictly below target (1e-3 Torr).
constexpr bool pumpdown_complete(const SensorSample& s, double target_torr) noexcept {
  return s.pressure_torr < target_torr;
}

constexpr bool in_band(const SensorSample& s, double setpoint_c, double band_c) noexcept {
  const double e = s.temperature_c - setpoint_c;
  return e <= band_c && e >= -band_c;
}

// Returns how long temperature has been continuously inside the band,
// including this tick. Any excursion resets it to zero ("consecutive").
constexpr std::int64_t update_in_band(std::int64_t in_band_ns, const SensorSample& s,
                                      double setpoint_c, double band_c, std::int64_t dt_ns) noexcept {
  return in_band(s, setpoint_c, band_c) ? in_band_ns + dt_ns : 0;
}

// RampUp -> Process: in band for at least `required_ns` consecutively.
constexpr bool settled(std::int64_t in_band_ns, std::int64_t required_ns) noexcept {
  return in_band_ns >= required_ns;
}

// RampDown -> Vent: cool enough to handle the wafer.
constexpr bool cooled(const SensorSample& s, double unload_temp_c) noexcept {
  return s.temperature_c < unload_temp_c;
}

// Vent -> Unload and Aborting -> Idle: at (near) atmosphere.
constexpr bool vented(const SensorSample& s, double vent_target_torr) noexcept {
  return s.pressure_torr >= vent_target_torr;
}

constexpr bool valve_fully_open(const SensorSample& s) noexcept { return s.valve_position >= 0.98; }
constexpr bool valve_fully_closed(const SensorSample& s) noexcept { return s.valve_position <= 0.02; }

}  // namespace ptc::recipe::guards
