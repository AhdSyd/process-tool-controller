#include "control/control_loop.hpp"

#include <stdexcept>

namespace ptc::control {

ControlLoop::ControlLoop(ControlLoopConfig cfg, TickTask& task) : cfg_(cfg), task_(task) {
  if (cfg_.period <= std::chrono::nanoseconds::zero()) {
    throw std::invalid_argument("control period must be positive");
  }
}

ControlLoop::~ControlLoop() { stop(); }

platform::RtStatus ControlLoop::start() {
  if (thread_.joinable()) throw std::logic_error("ControlLoop::start called twice");

  // The promise's shared state is allocated here, on the calling thread,
  // before the control loop exists. Nothing on the tick path allocates.
  std::promise<platform::RtStatus> setup_done;
  auto granted = setup_done.get_future();
  thread_ = std::thread(&ControlLoop::run, this, &setup_done);
  return granted.get();  // `setup_done` outlives the wait, so the pointer is safe
}

void ControlLoop::stop() noexcept {
  // Memory ordering: relaxed is sufficient. The flag carries no payload; it
  // only has to become visible eventually, and the loop polls it every tick.
  // Everything the caller later reads (histograms, timestamps) is synchronised
  // by join(), not by this flag.
  stop_requested_.store(true, std::memory_order_relaxed);
  if (thread_.joinable()) thread_.join();
}

ControlLoop::LiveStats ControlLoop::live() const noexcept {
  // Relaxed: these are independent monitoring counters with a single writer.
  // No other data is published through them, so no ordering is required.
  return LiveStats{ticks_.load(std::memory_order_relaxed),
                   missed_.load(std::memory_order_relaxed),
                   max_lateness_ns_.load(std::memory_order_relaxed)};
}

void ControlLoop::run(std::promise<platform::RtStatus>* setup_done) noexcept {
  const platform::RtStatus rt = platform::configure_current_thread(cfg_.rt, "ptc-control");
  setup_done->set_value(rt);  // after this line `setup_done` may dangle; never touch it again

  // ---------------------------------------------------------------------
  // Hot loop. Rules: no allocation, no locks, no I/O, no exceptions.
  // Everything used below is a member (preallocated) or a stack scalar.
  // ---------------------------------------------------------------------
  const Clock::duration period = std::chrono::duration_cast<Clock::duration>(cfg_.period);
  run_start_ = Clock::now();
  Clock::time_point deadline = run_start_ + period;
  std::uint64_t tick_index = 0;
  std::uint64_t missed = 0;
  std::int64_t max_late = 0;

  while (!stop_requested_.load(std::memory_order_relaxed)) {
    // Absolute deadline: the sleep target is fixed by the schedule, not by
    // when we happened to finish the previous tick.
    std::this_thread::sleep_until(deadline);
    const Clock::time_point woke = Clock::now();

    const std::int64_t late_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(woke - deadline).count();
    wake_lateness_.record(late_ns);

    task_.on_tick(TickContext{tick_index, deadline, woke});

    const Clock::time_point done = Clock::now();
    exec_time_.record(std::chrono::duration_cast<std::chrono::nanoseconds>(done - woke).count());

    ++tick_index;
    ticks_.store(tick_index, std::memory_order_relaxed);
    if (late_ns > max_late) {
      max_late = late_ns;
      max_lateness_ns_.store(max_late, std::memory_order_relaxed);
    }

    deadline += period;
    if (done >= deadline) {
      // We finished after the next tick should already have started. Count
      // every deadline that has passed and skip to the first future one,
      // staying on the original phase grid (t0 + n*period).
      const auto skipped = static_cast<std::uint64_t>((done - deadline) / period) + 1u;
      missed += skipped;
      deadline += period * static_cast<Clock::rep>(skipped);
      missed_.store(missed, std::memory_order_relaxed);
    }
  }

  run_end_ = Clock::now();
}

}  // namespace ptc::control
