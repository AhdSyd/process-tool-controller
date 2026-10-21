#include "io/sensor_task.hpp"

namespace ptc::io {

SensorTask::SensorTask(sim::Plant& plant, SensorRing& ring, const ActuatorPort& actuators,
                       const std::atomic<std::uint8_t>& recipe_state, telemetry::LogQueue& log,
                       FaultPlan fault, std::int64_t nominal_period_ns)
    : plant_(plant),
      ring_(ring),
      actuators_(actuators),
      recipe_state_(recipe_state),
      log_(log),
      fault_(fault),
      nominal_period_ns_(nominal_period_ns) {}

void SensorTask::on_tick(const control::TickContext& ctx) noexcept {
  const std::int64_t now = to_ns(ctx.woke_at);
  // Integrate the REAL elapsed time, not the nominal period: if this thread
  // was descheduled for 20 ms, the chamber still lived through 20 ms.
  const std::int64_t dt_ns = last_ns_ ? now - last_ns_ : nominal_period_ns_;
  last_ns_ = now;

  plant_.step(static_cast<double>(dt_ns) * 1e-9, actuators_.read());
  maybe_inject(now);

  if (!dropout_) ring_.push(plant_.sample(now, seq_++));
}

void SensorTask::maybe_inject(std::int64_t now) noexcept {
  if (fault_.type == FaultType::None || injected_at_ns_ != 0) return;

  // Relaxed: we only need "some recent value" of the recipe state to decide
  // when to arm a test fault; nothing else is synchronised through it.
  const auto state = static_cast<State>(recipe_state_.load(std::memory_order_relaxed));
  if (trigger_seen_ns_ == 0) {
    if (state == fault_.trigger_state) trigger_seen_ns_ = now;
    return;
  }
  if (now - trigger_seen_ns_ < fault_.delay_ns) return;

  injected_at_ns_ = now;
  if (fault_.type == FaultType::SensorDropout) dropout_ = true;
  else plant_.set_fault(fault_.type);

  telemetry::LogRecord r;
  r.t_ns = now;
  r.event = telemetry::LogEvent::FaultInjected;
  r.a = static_cast<std::uint8_t>(fault_.type);
  log_.try_push(r);
}

}  // namespace ptc::io
