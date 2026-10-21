#pragma once

#include <cstdint>
#include <random>

#include "common/types.hpp"

namespace ptc::sim {

// Physical constants of the simulated chamber. Values are chosen so a full
// wafer cycle takes ~13 s: slow enough to be realistic in shape, fast enough
// to run many cycles in a test.
struct PlantParams {
  // Thermal: first-order lag. Steady state at duty u is ambient + gain * u.
  double ambient_c = 25.0;
  double heater_gain_c = 600.0;  // full power settles at 625 C (above every limit)
  double thermal_tau_s = 2.5;

  // Vacuum: pump drives pressure exponentially toward base pressure; vent and
  // leaks drive it toward atmosphere. Combined as one linear ODE, solved exactly.
  double atmosphere_torr = 760.0;
  double base_torr = 1e-5;
  double pump_tau_s = 0.15;  // 760 Torr -> 1e-3 Torr in ~2 s
  double vent_tau_s = 0.30;
  double leak_tau_s = 2.0;   // leak vs pump settles near 57 Torr

  // Slit valve: constant-speed travel.
  double valve_travel_s = 0.30;

  // Sensor noise.
  double temp_noise_c = 0.15;        // 1 sigma, additive
  double pressure_noise_frac = 0.01; // 1 sigma, multiplicative (gauges are % of reading)

  std::uint64_t seed = 12345;  // deterministic noise: runs are reproducible
};

struct PlantState {
  double temperature_c;
  double pressure_torr;
  double valve_position;
  double heater_effective;  // duty actually reaching the heater element
};

// The "hardware". Owned and stepped by exactly one thread (the sensor thread
// in the real program, the test harness in tests). Not thread-safe by design.
class Plant {
 public:
  explicit Plant(const PlantParams& p = {});

  // Advance physics by dt seconds under command `cmd`. Uses exact solutions of
  // the linear ODEs, so it is stable for any dt (important when the sensor
  // thread itself gets descheduled and has to integrate a long gap).
  void step(double dt_s, const ActuatorCommand& cmd) noexcept;

  // Read all sensors (adds noise). Does not change physical state.
  SensorSample sample(std::int64_t t_ns, std::uint64_t seq) noexcept;

  void set_fault(FaultType f) noexcept { fault_ = f; }
  FaultType fault() const noexcept { return fault_; }
  const PlantState& state() const noexcept { return state_; }
  const PlantParams& params() const noexcept { return params_; }

  // Test hook: put the plant in an arbitrary physical state.
  void set_state(const PlantState& s) noexcept { state_ = s; }

 private:
  PlantParams params_;
  PlantState state_;
  FaultType fault_ = FaultType::None;
  std::mt19937_64 rng_;
  std::normal_distribution<double> unit_normal_{0.0, 1.0};
};

}  // namespace ptc::sim
