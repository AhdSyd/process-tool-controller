#pragma once

#include <cstdint>

#include "common/types.hpp"

namespace ptc::safety {

// Interlocks (FR-4). Evaluated EVERY tick, before the recipe runs, and
// deliberately blind to which recipe state we are in: a rule like "never
// above 450 C" must hold whether the recipe thinks it is heating or not. If
// interlocks were written inside recipe states, a recipe bug (wrong state,
// missed transition) could silently disable the protection. Independence is
// the point.
enum class Interlock : std::uint8_t {
  OverTemperature,           // T > 450 C                      -> abort
  CriticalOverTemperature,   // T > 500 C (abort did not work)  -> fault
  OverPressureHeaterOn,      // P > 10 Torr with heater on      -> abort
  SlitValveOpenUnderVacuum,  // valve open with P < 1 Torr      -> abort
  StaleSensorData,           // no new sample for > 10 ms       -> fault
  kCount
};
constexpr std::size_t kInterlockCount = static_cast<std::size_t>(Interlock::kCount);

// Abort = controlled shutdown (heater off, vent, back to Idle).
// Fault = immediate de-energise of everything, latched until reset. Used when
// the controlled path is itself unsafe or unverifiable.
enum class Severity : std::uint8_t { Abort, Fault };

constexpr Severity severity(Interlock i) noexcept {
  switch (i) {
    case Interlock::CriticalOverTemperature:
    // Stale data is a Fault, not an Abort: the vent sequence's exit condition
    // is a pressure reading, and we no longer have trustworthy readings.
    case Interlock::StaleSensorData: return Severity::Fault;
    default: return Severity::Abort;
  }
}

constexpr Reason reason_for(Interlock i) noexcept {
  switch (i) {
    case Interlock::OverTemperature: return Reason::OverTemperature;
    case Interlock::CriticalOverTemperature: return Reason::CriticalOverTemperature;
    case Interlock::OverPressureHeaterOn: return Reason::OverPressureHeaterOn;
    case Interlock::SlitValveOpenUnderVacuum: return Reason::SlitValveOpenUnderVacuum;
    case Interlock::StaleSensorData: return Reason::StaleSensorData;
    case Interlock::kCount: break;
  }
  return Reason::None;
}

constexpr const char* interlock_name(Interlock i) noexcept { return reason_name(reason_for(i)); }

struct InterlockLimits {
  double max_temp_c = 450.0;
  double critical_temp_c = 500.0;
  double max_pressure_heater_on_torr = 10.0;
  double valve_min_pressure_torr = 1.0;
  double valve_open_threshold = 0.05;   // > 5 % open counts as "open"
  std::int64_t stale_ns = 10 * kNsPerMs;
  int stale_debounce_ticks = 2;         // must be stale on N consecutive ticks

  // Hysteresis: a tripped interlock only CLEARS once the value is back past
  // the limit by this margin. Without it, sensor noise around a limit makes
  // the interlock chatter trip/clear/trip, spamming the log and re-triggering
  // actions.
  double temp_hysteresis_c = 5.0;
  double pressure_hysteresis_frac = 0.2;

  // The limits a tripped interlock must get back inside of to clear.
  InterlockLimits clear_limits() const noexcept {
    InterlockLimits c = *this;
    c.max_temp_c -= temp_hysteresis_c;
    c.critical_temp_c -= temp_hysteresis_c;
    c.max_pressure_heater_on_torr *= 1.0 - pressure_hysteresis_frac;
    c.valve_min_pressure_torr *= 1.0 + pressure_hysteresis_frac;
    return c;
  }
};

// Small fixed-size set of tripped interlocks (a bitmask).
class TripSet {
 public:
  constexpr void set(Interlock i) noexcept { bits_ |= bit(i); }
  constexpr bool test(Interlock i) const noexcept { return bits_ & bit(i); }
  constexpr bool any() const noexcept { return bits_ != 0; }
  constexpr std::uint32_t bits() const noexcept { return bits_; }
  constexpr TripSet newly_set_vs(TripSet prev) const noexcept { return TripSet(bits_ & ~prev.bits_); }
  constexpr bool operator==(TripSet o) const noexcept { return bits_ == o.bits_; }

  // Highest severity first; ties broken by enum order.
  constexpr Interlock most_severe() const noexcept {
    for (std::size_t i = 0; i < kInterlockCount; ++i) {
      const auto il = static_cast<Interlock>(i);
      if (test(il) && severity(il) == Severity::Fault) return il;
    }
    for (std::size_t i = 0; i < kInterlockCount; ++i) {
      const auto il = static_cast<Interlock>(i);
      if (test(il)) return il;
    }
    return Interlock::kCount;
  }

  constexpr TripSet() = default;

 private:
  constexpr explicit TripSet(std::uint32_t b) : bits_(b) {}
  static constexpr std::uint32_t bit(Interlock i) noexcept { return 1u << static_cast<unsigned>(i); }
  std::uint32_t bits_ = 0;
};

// ---- Pure predicates: one per interlock, each unit-tested in isolation ----
constexpr bool over_temperature(const SensorSample& s, const InterlockLimits& l) noexcept {
  return s.temperature_c > l.max_temp_c;
}
constexpr bool critical_over_temperature(const SensorSample& s, const InterlockLimits& l) noexcept {
  return s.temperature_c > l.critical_temp_c;
}
constexpr bool over_pressure_heater_on(const SensorSample& s, bool heater_on,
                                       const InterlockLimits& l) noexcept {
  return heater_on && s.pressure_torr > l.max_pressure_heater_on_torr;
}
constexpr bool slit_valve_open_under_vacuum(const SensorSample& s, const InterlockLimits& l) noexcept {
  return s.valve_position > l.valve_open_threshold && s.pressure_torr < l.valve_min_pressure_torr;
}
// `latest` is null if no sample has ever been received.
constexpr bool sensor_data_stale(const SensorSample* latest, std::int64_t now_ns,
                                 const InterlockLimits& l) noexcept {
  return latest == nullptr || now_ns - latest->t_ns > l.stale_ns;
}

struct InterlockInput {
  const SensorSample* latest;  // null if no sample yet
  std::int64_t now_ns;
  bool heater_on;              // heater commanded on (duty > 0, contactor closed)
};

// Pure: evaluates every predicate. No debouncing.
TripSet evaluate(const InterlockInput& in, const InterlockLimits& l) noexcept;

// Stateful wrapper around the pure evaluate():
//   * Hysteresis: an active trip stays active until the value is back inside
//     the clear limits (see InterlockLimits::clear_limits).
//   * Stale-data debounce: a single late control tick (the OS delayed us, not
//     the sensor) must not fault the tool; a sensor that has really stopped
//     stays stale on the next tick too and trips 1 ms later.
class InterlockMonitor {
 public:
  explicit InterlockMonitor(const InterlockLimits& l = {}) : limits_(l), clear_(l.clear_limits()) {}
  TripSet update(const InterlockInput& in) noexcept;
  const InterlockLimits& limits() const noexcept { return limits_; }

 private:
  InterlockLimits limits_;
  InterlockLimits clear_;
  TripSet active_{};
  int stale_count_ = 0;
};

// ---- Abort sequencing decision (FR-5) ----
struct Action {
  enum class Kind : std::uint8_t { None, Abort, Fault } kind = Kind::None;
  Reason reason = Reason::None;
};

// Given the current state and what is tripped, what must happen NOW?
//   * Fault-severity trip  -> Fault, from ANY state, including Aborting.
//                            This is what makes the abort path interruptible.
//   * Abort-severity trip  -> Aborting, unless already Aborting or Fault
//                            (re-starting an abort that is in progress would
//                            only reset its timeout).
//   * In Fault             -> nothing more to do: already the safest state.
Action decide(State current, TripSet trips) noexcept;

}  // namespace ptc::safety
