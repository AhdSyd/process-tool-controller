#pragma once

// Vocabulary types shared by every module. Plain data only: no behaviour, no
// allocation, trivially copyable so they can cross thread boundaries through
// the lock-free structures in io/ and telemetry/.

#include <chrono>
#include <cstdint>
#include <type_traits>

namespace ptc {

using Clock = std::chrono::steady_clock;

constexpr std::int64_t kNsPerUs = 1'000;
constexpr std::int64_t kNsPerMs = 1'000'000;
constexpr std::int64_t kNsPerSec = 1'000'000'000;

// All timestamps in the program are steady_clock nanoseconds since its epoch.
// Monotonic: never jumps when the wall clock is changed (NTP, DST, user).
inline std::int64_t to_ns(Clock::time_point t) noexcept {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
}
inline std::int64_t now_ns() noexcept { return to_ns(Clock::now()); }

// ---------------------------------------------------------------------------
// One reading of every chamber sensor, taken at the same instant.
// ---------------------------------------------------------------------------
struct SensorSample {
  std::int64_t t_ns = 0;      // when the sensor thread took the reading
  std::uint64_t seq = 0;      // monotonically increasing sample number
  double temperature_c = 0;   // chamber/susceptor temperature
  double pressure_torr = 0;   // chamber pressure
  double valve_position = 0;  // slit valve: 0 = fully closed, 1 = fully open
};
static_assert(std::is_trivially_copyable_v<SensorSample>);

// ---------------------------------------------------------------------------
// Everything the controller can command. Written by the control thread,
// applied by the plant (simulated hardware).
// ---------------------------------------------------------------------------
struct ActuatorCommand {
  double heater = 0.0;            // heater duty cycle 0..1 (drives an SSR)
  bool heater_contactor = false;  // main power to the heater; independent of the
                                  // SSR, opened only in Fault (last line of defence)
  bool pump = false;              // roughing/turbo pump
  bool vent = false;              // vent valve (lets N2 in to reach atmosphere)
  bool valve_open = false;        // slit valve command
};

// ---------------------------------------------------------------------------
// Recipe states (FR-1). Fault and Aborting are reachable from any state.
// ---------------------------------------------------------------------------
enum class State : std::uint8_t {
  Idle,
  Load,
  PumpDown,
  RampUp,
  Process,
  RampDown,
  Vent,
  Unload,
  Aborting,
  Fault,
  kCount
};

constexpr const char* state_name(State s) noexcept {
  switch (s) {
    case State::Idle: return "Idle";
    case State::Load: return "Load";
    case State::PumpDown: return "PumpDown";
    case State::RampUp: return "RampUp";
    case State::Process: return "Process";
    case State::RampDown: return "RampDown";
    case State::Vent: return "Vent";
    case State::Unload: return "Unload";
    case State::Aborting: return "Aborting";
    case State::Fault: return "Fault";
    case State::kCount: break;
  }
  return "?";
}

// Why the machine went to Aborting or Fault. Logged as a reason code.
enum class Reason : std::uint8_t {
  None,
  OperatorAbort,
  StateTimeout,
  AbortTimeout,
  OverTemperature,
  CriticalOverTemperature,
  OverPressureHeaterOn,
  SlitValveOpenUnderVacuum,
  StaleSensorData,
  kCount
};

constexpr const char* reason_name(Reason r) noexcept {
  switch (r) {
    case Reason::None: return "None";
    case Reason::OperatorAbort: return "OperatorAbort";
    case Reason::StateTimeout: return "StateTimeout";
    case Reason::AbortTimeout: return "AbortTimeout";
    case Reason::OverTemperature: return "OverTemperature";
    case Reason::CriticalOverTemperature: return "CriticalOverTemperature";
    case Reason::OverPressureHeaterOn: return "OverPressureHeaterOn";
    case Reason::SlitValveOpenUnderVacuum: return "SlitValveOpenUnderVacuum";
    case Reason::StaleSensorData: return "StaleSensorData";
    case Reason::kCount: break;
  }
  return "?";
}

// Simulated hardware faults for FR-10.
enum class FaultType : std::uint8_t {
  None,
  SensorDropout,       // sensor thread stops publishing (cable, ADC hang)
  TemperatureRunaway,  // heater SSR welded on: ignores the duty command
  StuckValve,          // slit valve actuator fails open
  VacuumLeak,          // chamber seal fails: pressure rises toward atmosphere
};

constexpr const char* fault_name(FaultType f) noexcept {
  switch (f) {
    case FaultType::None: return "none";
    case FaultType::SensorDropout: return "sensor-dropout";
    case FaultType::TemperatureRunaway: return "temp-runaway";
    case FaultType::StuckValve: return "stuck-valve";
    case FaultType::VacuumLeak: return "vacuum-leak";
  }
  return "?";
}

}  // namespace ptc
