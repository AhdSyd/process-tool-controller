#include "control/pid.hpp"

#include <algorithm>

namespace ptc::control {

void Pid::reset() noexcept {
  integral_ = 0.0;
  d_state_ = 0.0;
  prev_meas_ = 0.0;
  primed_ = false;
}

double Pid::update(double setpoint, double measurement, double dt_s, double feedforward) noexcept {
  if (dt_s <= 0.0) dt_s = 1e-3;
  const double error = setpoint - measurement;

  // Derivative on measurement, low-pass filtered.
  double raw_d = 0.0;
  if (primed_) raw_d = -(measurement - prev_meas_) / dt_s;
  prev_meas_ = measurement;
  primed_ = true;
  const double alpha = dt_s / (g_.d_filter_s + dt_s);
  d_state_ += alpha * (raw_d - d_state_);
  const double d_term = g_.kd * d_state_;

  const double p_term = g_.kp * error;
  const double unsat = feedforward + p_term + integral_ + d_term;

  // Conditional integration: only integrate if it would not deepen saturation.
  const bool saturated_high = unsat >= g_.out_max && error > 0.0;
  const bool saturated_low = unsat <= g_.out_min && error < 0.0;
  if (!saturated_high && !saturated_low) {
    integral_ += g_.ki * error * dt_s;
    integral_ = std::clamp(integral_, g_.out_min - g_.out_max, g_.out_max - g_.out_min);
  }

  return std::clamp(feedforward + p_term + integral_ + d_term, g_.out_min, g_.out_max);
}

}  // namespace ptc::control
