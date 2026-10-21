#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <thread>

#include "common/types.hpp"
#include "platform/rt_thread.hpp"
#include "telemetry/latency_histogram.hpp"

namespace ptc::control {

using ptc::Clock;

// Passed to the per-tick work. Plain data, built on the stack every tick.
struct TickContext {
  std::uint64_t tick_index;   // 0-based count of executed ticks
  Clock::time_point deadline; // when this tick was scheduled to start
  Clock::time_point woke_at;  // when it actually started
};

// Work done every tick. Implementations must obey the real-time rules: no
// blocking, no allocation, no I/O, no exceptions (hence noexcept).
class TickTask {
 public:
  virtual ~TickTask() = default;
  virtual void on_tick(const TickContext& ctx) noexcept = 0;
};

struct PeriodicLoopConfig {
  std::chrono::nanoseconds period = std::chrono::milliseconds(1);
  platform::RtRequest rt{};
  const char* thread_name = "ptc-periodic";  // <= 15 chars
};

// Owns one thread that runs a TickTask at a fixed period. Used twice:
//   control thread = PeriodicLoop(1 ms,   Controller)
//   sensor thread  = PeriodicLoop(200 us, SensorTask)
//
// RAII: the destructor stops and joins, so a PeriodicLoop can never leak a
// running thread, even on an exception path.
//
// Timing model:
//   * Deadlines are ABSOLUTE: deadline[n] = t0 + n * period. Each tick sleeps
//     until its deadline with sleep_until, so a late wakeup or a slow tick
//     never shifts the schedule (no cumulative drift).
//   * "Wake lateness" = actual wake time - deadline. That is the jitter metric.
//   * "Missed deadline" = the tick's work finished after the NEXT tick's
//     deadline, so the next tick could not start on time.
//   * On a miss the loop SKIPS the deadlines that already passed rather than
//     running them back-to-back. Catching up in a burst would run several
//     control updates within microseconds on the same stale sensor data,
//     which is worse than one honest, counted gap.
class PeriodicLoop {
 public:
  PeriodicLoop(PeriodicLoopConfig cfg, TickTask& task);
  ~PeriodicLoop();

  PeriodicLoop(const PeriodicLoop&) = delete;
  PeriodicLoop& operator=(const PeriodicLoop&) = delete;
  PeriodicLoop(PeriodicLoop&&) = delete;
  PeriodicLoop& operator=(PeriodicLoop&&) = delete;

  // Starts the thread and blocks until it has applied its RT settings.
  // Returns what the OS actually granted. Call at most once.
  platform::RtStatus start();

  // Idempotent. Requests stop and joins the thread.
  void stop() noexcept;

  bool started() const noexcept { return thread_.joinable(); }

  // Live, lock-free counters for other threads. Each field is individually
  // atomic; the set is not a consistent snapshot (fine for a status line).
  struct LiveStats {
    std::uint64_t ticks;
    std::uint64_t missed_deadlines;
    std::int64_t max_lateness_ns;
  };
  LiveStats live() const noexcept;

  // Full results. Only valid after stop() has returned: the join is the
  // happens-before edge that makes the loop thread's plain (non-atomic)
  // writes to these histograms visible to the caller.
  const telemetry::LatencyHistogram& wake_lateness() const noexcept { return wake_lateness_; }
  const telemetry::LatencyHistogram& exec_time() const noexcept { return exec_time_; }
  std::chrono::nanoseconds run_duration() const noexcept { return run_end_ - run_start_; }
  std::chrono::nanoseconds period() const noexcept { return cfg_.period; }

 private:
  void run(std::promise<platform::RtStatus>* setup_done) noexcept;

  const PeriodicLoopConfig cfg_;
  TickTask& task_;  // non-owning: the task outlives the loop

  // --- shared with other threads (atomics only) ---
  std::atomic<bool> stop_requested_{false};
  std::atomic<std::uint64_t> ticks_{0};
  std::atomic<std::uint64_t> missed_{0};
  std::atomic<std::int64_t> max_lateness_ns_{0};

  // --- written only by the loop thread; read only after join ---
  telemetry::LatencyHistogram wake_lateness_;
  telemetry::LatencyHistogram exec_time_;
  Clock::time_point run_start_{};
  Clock::time_point run_end_{};

  std::thread thread_;  // last member: constructed after everything it touches
};

}  // namespace ptc::control
