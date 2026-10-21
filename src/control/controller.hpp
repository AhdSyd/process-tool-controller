#pragma once

#include <atomic>
#include <cstdint>

#include "common/types.hpp"
#include "control/periodic_loop.hpp"
#include "control/pid.hpp"
#include "io/actuator_port.hpp"
#include "io/sensor_task.hpp"
#include "recipe/recipe_machine.hpp"
#include "safety/interlocks.hpp"
#include "telemetry/log_record.hpp"
#include "telemetry/metrics.hpp"

namespace ptc::control {

// Operator commands, UI thread -> control thread. Each field is an
// independent request; the control thread consumes it with exchange().
// Relaxed ordering: no field guards any other data.
struct CommandMailbox {
  std::atomic<std::uint32_t> wafers{0};  // wafers to add to the queue
  std::atomic<bool> abort{false};
  std::atomic<bool> reset{false};
};

// Live status, control thread -> UI thread (and the sensor thread reads
// `state` to arm test faults). Written every tick.
//
// `wafers_accepted` is stored LAST with memory_order_release and read FIRST by
// the UI with acquire. That makes it a publication point: if the UI sees
// wafers_accepted == N, it is guaranteed to also see the state/pending values
// from that tick or later, so "accepted all, Idle, nothing pending" can never
// be a stale mix from before the wafers were picked up.
struct StatusBoard {
  std::atomic<std::uint8_t> state{static_cast<std::uint8_t>(State::Idle)};
  std::atomic<std::uint8_t> reason{0};
  std::atomic<double> temperature_c{0};
  std::atomic<double> pressure_torr{0};
  std::atomic<double> valve_position{0};
  std::atomic<double> heater{0};
  std::atomic<std::uint32_t> wafers_done{0};
  std::atomic<std::uint32_t> wafers_pending{0};
  std::atomic<std::uint64_t> overruns{0};
  std::atomic<std::uint32_t> wafers_accepted{0};  // publication point, see above
};

struct ControllerConfig {
  recipe::RecipeParams recipe{};
  safety::InterlockLimits limits{};
  PidGains pid{};
  // Feedforward model. Deliberately a little different from the simulated
  // plant (gain 600): a real model is never exact, and the integrator's job is
  // to absorb that difference.
  double ff_ambient_c = 25.0;
  double ff_gain_c = 570.0;
};

// The control thread's tick. Each tick, in this fixed order:
//   1. drain the sensor ring -> latest sample (+ count overruns)
//   2. pick up operator commands
//   3. evaluate interlocks (independent of recipe state)
//   4. arbitrate: safety action > operator abort > normal recipe progression
//   5. compute outputs (PID) and write the actuator command
//   6. log transitions/trips (enqueue only), update metrics and status
// Safety is evaluated before the recipe and its decision wins, so an
// interlock trip turns the heater off in the SAME tick it is detected.
class Controller final : public TickTask {
 public:
  Controller(const ControllerConfig& cfg, io::SensorRing& ring, io::ActuatorPort& port,
             CommandMailbox& mailbox, StatusBoard& status, telemetry::LogQueue& log);

  void on_tick(const TickContext& ctx) noexcept override { step(to_ns(ctx.woke_at)); }

  // One control step at time `now_ns`. Public so tests can drive the
  // controller deterministically with simulated time, no threads.
  void step(std::int64_t now_ns) noexcept;

  // Optional clock used only to measure heater-off latency inside a tick.
  // Null (tests) = report latency as 0, i.e. "same tick".
  void set_latency_clock(std::int64_t (*clock)()) noexcept { clock_ = clock; }

  // Read only after the control thread has been joined (or from the thread
  // driving step() in tests).
  const telemetry::ControllerMetrics& metrics() const noexcept { return m_; }
  const recipe::RecipeMachine& machine() const noexcept { return machine_; }
  const ActuatorCommand& last_command() const noexcept { return last_cmd_; }
  safety::TripSet active_trips() const noexcept { return prev_trips_; }

 private:
  void log_trip_edges(safety::TripSet trips, std::int64_t now) noexcept;
  void on_transition(const recipe::Transition& tr, std::int64_t now, std::int64_t write_ns) noexcept;
  void update_quality(double t_meas) noexcept;
  void publish_status(double t_meas) noexcept;
  void log(const telemetry::LogRecord& r) noexcept { log_.try_push(r); }
  telemetry::LogRecord record(telemetry::LogEvent ev, std::int64_t now) const noexcept;

  const ControllerConfig cfg_;
  io::SensorRing& ring_;
  io::ActuatorPort& port_;
  CommandMailbox& mailbox_;
  StatusBoard& status_;
  telemetry::LogQueue& log_;

  recipe::RecipeMachine machine_;
  safety::InterlockMonitor monitor_;
  Pid pid_;

  SensorSample latest_{};
  bool have_sample_ = false;
  std::int64_t prev_now_ = 0;
  safety::TripSet prev_trips_{};
  ActuatorCommand last_cmd_{};
  std::uint32_t accepted_ = 0;
  std::int64_t (*clock_)() = nullptr;
  int open_event_ = -1;  // index of the safety event still waiting to reach safe

  telemetry::ControllerMetrics m_{};
};

}  // namespace ptc::control
