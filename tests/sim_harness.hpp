#pragma once

// Deterministic, single-threaded closed loop: plant + ring + controller
// stepped in lockstep with SIMULATED time. Same production classes as the
// real program; only the threads and the clock are replaced. A 15-second
// recipe runs in well under a second and gives identical results every time.

#include <functional>
#include <memory>

#include "control/controller.hpp"
#include "io/sensor_task.hpp"
#include "sim/plant.hpp"

namespace ptc::test {

class SimHarness {
 public:
  static constexpr std::int64_t kSensorPeriodNs = 200 * kNsPerUs;  // 5 kHz
  static constexpr std::int64_t kControlPeriodNs = kNsPerMs;       // 1 kHz

  explicit SimHarness(const control::ControllerConfig& cfg = {}, const sim::PlantParams& pp = {})
      : plant(pp), ctl(cfg, ring, port, mailbox, status, log) {}

  // Advance one control period: 5 sensor ticks then 1 control tick.
  void tick() {
    for (int i = 0; i < 5; ++i) {
      now += kSensorPeriodNs;
      plant.step(static_cast<double>(kSensorPeriodNs) * 1e-9, port.read());
      if (!sensor_dropout) ring.push(plant.sample(now, seq++));
    }
    if (control_stall_ticks > 0) {  // simulate a descheduled control thread
      --control_stall_ticks;
      return;
    }
    ctl.step(now);
    telemetry::LogRecord r;
    while (log.try_pop(r)) on_log(r);
  }

  void run_ms(std::int64_t ms) {
    for (std::int64_t i = 0; i < ms; ++i) tick();
  }

  // Run until pred() is true or max_ms of simulated time pass. Returns pred().
  bool run_until(const std::function<bool()>& pred, std::int64_t max_ms) {
    for (std::int64_t i = 0; i < max_ms; ++i) {
      if (pred()) return true;
      tick();
    }
    return pred();
  }

  bool run_until_state(State s, std::int64_t max_ms) {
    return run_until([&] { return ctl.machine().state() == s; }, max_ms);
  }

  void start_wafers(std::uint32_t n) { mailbox.wafers.fetch_add(n); }
  State state() const { return ctl.machine().state(); }
  double seconds() const { return static_cast<double>(now - t0) / 1e9; }

  sim::Plant plant;
  io::SensorRing ring;
  io::ActuatorPort port;
  control::CommandMailbox mailbox;
  control::StatusBoard status;
  telemetry::LogQueue log;
  control::Controller ctl;

  std::function<void(const telemetry::LogRecord&)> on_log = [](const telemetry::LogRecord&) {};
  bool sensor_dropout = false;
  int control_stall_ticks = 0;
  static constexpr std::int64_t t0 = 1'000 * kNsPerSec;
  std::int64_t now = t0;
  std::uint64_t seq = 0;
};

inline std::unique_ptr<SimHarness> make_harness(const control::ControllerConfig& cfg = {},
                                                const sim::PlantParams& pp = {}) {
  auto h = std::make_unique<SimHarness>(cfg, pp);
  h->run_ms(5);  // prime: sensors flowing, controller has a sample
  return h;
}

}  // namespace ptc::test
