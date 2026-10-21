#pragma once

namespace ptc::control {

struct PidGains {
  double kp = 0.05;        // duty per degC of error
  double ki = 0.02;        // duty per degC*s
  double kd = 0.001;       // duty per degC/s (on measurement)
  double d_filter_s = 0.02;  // first-order low-pass on the derivative term
  double out_min = 0.0;
  double out_max = 1.0;
};

// PID with static feedforward, output clamping, and anti-windup (FR-7).
//
//   u = clamp(ff + Kp*e + I + D)
//
// Anti-windup (conditional integration / "clamping"): the integrator is NOT
// advanced when the output is saturated AND the error would push it further
// into saturation. During a ramp the heater sits at 100 % for ~2 s with a large
// error; a naive integrator would accumulate a huge value there and then cause
// a big overshoot while it "unwinds". Freezing it removes that. The integral is
// additionally clamped to the output range as a backstop.
//
// Derivative on measurement (not on error): a setpoint step then produces no
// derivative "kick". It is low-pass filtered because differentiating a noisy
// sensor at 1 kHz amplifies the noise enormously.
//
// Feedforward: the caller supplies the duty a model predicts is needed at the
// setpoint ((sp - ambient) / gain). The PID only trims model error, so the
// integrator does not have to "discover" the operating point from zero.
//
// Pure arithmetic: noexcept, no allocation, safe on the control thread.
class Pid {
 public:
  explicit Pid(const PidGains& g = {}) : g_(g) {}

  double update(double setpoint, double measurement, double dt_s, double feedforward = 0.0) noexcept;
  void reset() noexcept;

  double integral() const noexcept { return integral_; }
  const PidGains& gains() const noexcept { return g_; }

 private:
  PidGains g_;
  double integral_ = 0.0;
  double d_state_ = 0.0;
  double prev_meas_ = 0.0;
  bool primed_ = false;
};

}  // namespace ptc::control
