#include "sim/plant.hpp"

#include <algorithm>
#include <cmath>

namespace ptc::sim {

Plant::Plant(const PlantParams& p)
    : params_(p),
      state_{p.ambient_c, p.atmosphere_torr, 0.0, 0.0},
      rng_(p.seed) {}

void Plant::step(double dt_s, const ActuatorCommand& cmd) noexcept {
  if (dt_s <= 0.0) return;
  const PlantParams& p = params_;

  // ---- Heater -------------------------------------------------------------
  // The contactor is upstream of the SSR. A welded SSR (runaway fault) ignores
  // the duty command and conducts fully, but opening the contactor still cuts
  // power. That layering is why Fault opens the contactor.
  double duty = std::clamp(cmd.heater, 0.0, 1.0);
  if (fault_ == FaultType::TemperatureRunaway) duty = 1.0;
  if (!cmd.heater_contactor) duty = 0.0;
  state_.heater_effective = duty;

  // dT/dt = (T_eq - T) / tau, T_eq = ambient + gain * duty  -> exact solution
  const double t_eq = p.ambient_c + p.heater_gain_c * duty;
  state_.temperature_c = t_eq + (state_.temperature_c - t_eq) * std::exp(-dt_s / p.thermal_tau_s);

  // ---- Pressure -----------------------------------------------------------
  // dP/dt = a - b P, sum of first-order "pulls" toward base or atmosphere.
  double a = 0.0, b = 0.0;
  if (cmd.pump) { a += p.base_torr / p.pump_tau_s;       b += 1.0 / p.pump_tau_s; }
  if (cmd.vent) { a += p.atmosphere_torr / p.vent_tau_s; b += 1.0 / p.vent_tau_s; }
  if (fault_ == FaultType::VacuumLeak) {
    a += p.atmosphere_torr / p.leak_tau_s;
    b += 1.0 / p.leak_tau_s;
  }
  if (b > 0.0) {
    const double p_eq = a / b;
    state_.pressure_torr = p_eq + (state_.pressure_torr - p_eq) * std::exp(-b * dt_s);
  }  // else: sealed chamber, pressure holds

  // ---- Slit valve ---------------------------------------------------------
  const bool want_open = cmd.valve_open || fault_ == FaultType::StuckValve;
  const double target = want_open ? 1.0 : 0.0;
  const double max_move = dt_s / p.valve_travel_s;
  const double delta = std::clamp(target - state_.valve_position, -max_move, max_move);
  state_.valve_position = std::clamp(state_.valve_position + delta, 0.0, 1.0);
}

SensorSample Plant::sample(std::int64_t t_ns, std::uint64_t seq) noexcept {
  SensorSample s;
  s.t_ns = t_ns;
  s.seq = seq;
  s.temperature_c = state_.temperature_c + params_.temp_noise_c * unit_normal_(rng_);
  s.pressure_torr =
      std::max(0.0, state_.pressure_torr * (1.0 + params_.pressure_noise_frac * unit_normal_(rng_)));
  s.valve_position = state_.valve_position;  // limit-switch/encoder: treat as exact
  return s;
}

}  // namespace ptc::sim
