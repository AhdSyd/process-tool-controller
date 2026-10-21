#pragma once

#include <atomic>
#include <cstdint>

#include "common/types.hpp"
#include "control/periodic_loop.hpp"
#include "io/actuator_port.hpp"
#include "io/seqlock_ring.hpp"
#include "sim/plant.hpp"
#include "telemetry/log_record.hpp"

namespace ptc::io {

// 64 slots at 5 kHz = 12.8 ms of history: the control thread can be descheduled
// for ~12 ticks before any sample is lost.
using SensorRing = SeqlockRing<SensorSample, 64>;

// When to inject a simulated hardware fault (FR-10). Faults are armed by
// recipe state rather than wall time so tests are deterministic: e.g.
// "runaway 500 ms after Process starts" always hits the same physical moment.
struct FaultPlan {
  FaultType type = FaultType::None;
  State trigger_state = State::Process;
  std::int64_t delay_ns = 500 * kNsPerMs;
};

// The sensor thread's tick (5 kHz). In this simulation the sensor thread also
// plays the role of the physical world: each tick it
//   1. reads the latest actuator command,
//   2. advances the plant physics by the real elapsed time,
//   3. samples the sensors and publishes into the ring buffer.
// Real hardware would replace 1-2 with "the world happens" and 3 with an ADC read.
class SensorTask final : public control::TickTask {
 public:
  SensorTask(sim::Plant& plant, SensorRing& ring, const ActuatorPort& actuators,
             const std::atomic<std::uint8_t>& recipe_state, telemetry::LogQueue& log,
             FaultPlan fault, std::int64_t nominal_period_ns);

  void on_tick(const control::TickContext& ctx) noexcept override;

  // Valid after the owning loop is joined.
  std::uint64_t samples_published() const noexcept { return seq_; }
  std::int64_t fault_injected_at_ns() const noexcept { return injected_at_ns_; }

 private:
  void maybe_inject(std::int64_t now) noexcept;

  sim::Plant& plant_;
  SensorRing& ring_;
  const ActuatorPort& actuators_;
  const std::atomic<std::uint8_t>& recipe_state_;
  telemetry::LogQueue& log_;
  const FaultPlan fault_;
  const std::int64_t nominal_period_ns_;

  std::int64_t last_ns_ = 0;
  std::uint64_t seq_ = 0;
  std::int64_t trigger_seen_ns_ = 0;
  std::int64_t injected_at_ns_ = 0;
  bool dropout_ = false;
};

}  // namespace ptc::io
