#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <thread>

#include "control/control_loop.hpp"

using namespace std::chrono_literals;
using ptc::control::Clock;
using ptc::control::ControlLoop;
using ptc::control::ControlLoopConfig;
using ptc::control::TickContext;
using ptc::control::TickTask;

namespace {

ControlLoopConfig test_config(std::chrono::nanoseconds period) {
  ControlLoopConfig cfg;
  cfg.period = period;
  cfg.rt.want_fifo = false;  // tests must not depend on RT privileges
  return cfg;
}

// Records the first N deadlines into a fixed array (no allocation per tick).
class RecordingTask final : public TickTask {
 public:
  static constexpr std::size_t kCap = 4096;
  void on_tick(const TickContext& ctx) noexcept override {
    if (n_ < kCap) deadlines_[n_++] = ctx.deadline;
    if (busy_ > Clock::duration::zero()) {
      const auto until = Clock::now() + busy_;
      while (Clock::now() < until) {
      }  // simulate an overrunning tick without sleeping
    }
  }
  void set_busy(Clock::duration d) { busy_ = d; }  // set before start() only
  std::size_t n_ = 0;
  std::array<Clock::time_point, kCap> deadlines_{};

 private:
  Clock::duration busy_{};
};

}  // namespace

TEST(ControlLoop, TicksAtRoughlyTheConfiguredRate) {
  RecordingTask task;
  ControlLoop loop(test_config(1ms), task);
  loop.start();
  std::this_thread::sleep_for(300ms);
  loop.stop();
  const auto ticks = loop.wake_lateness().count();
  // Generous bounds: CI machines and sanitizers are noisy. This is a sanity
  // check on the schedule, not a jitter claim.
  EXPECT_GT(ticks, 200u);
  EXPECT_LE(ticks, 310u);
  EXPECT_EQ(loop.exec_time().count(), ticks);
  EXPECT_EQ(loop.wake_lateness().negative_count(), 0u) << "woke before an absolute deadline";
}

TEST(ControlLoop, DeadlinesStayOnTheAbsoluteGrid) {
  RecordingTask task;
  ControlLoop loop(test_config(1ms), task);
  loop.start();
  std::this_thread::sleep_for(100ms);
  loop.stop();
  ASSERT_GT(task.n_, 10u);
  const auto t0 = task.deadlines_[0];
  for (std::size_t i = 1; i < task.n_; ++i) {
    const auto offset = task.deadlines_[i] - t0;
    EXPECT_EQ(offset % 1ms, Clock::duration::zero()) << "tick " << i << " drifted off the grid";
    EXPECT_GT(task.deadlines_[i], task.deadlines_[i - 1]);
  }
}

TEST(ControlLoop, OverrunsAreCountedAndSkippedNotBurst) {
  RecordingTask task;
  task.set_busy(2500us);  // every tick takes 2.5 periods
  ControlLoop loop(test_config(1ms), task);
  loop.start();
  std::this_thread::sleep_for(200ms);
  loop.stop();

  const auto ticks = loop.wake_lateness().count();
  const auto missed = loop.live().missed_deadlines;
  ASSERT_GT(ticks, 5u);
  // Each 2.5 ms tick swallows the next two deadlines: ~2 misses per tick.
  EXPECT_GE(missed, 2 * (ticks - 1));
  // Skipping keeps executed ticks far below 200 (bursting would approach it).
  EXPECT_LT(ticks, 100u);
  // Even after skipping, every executed deadline is still on the 1 ms grid.
  for (std::size_t i = 1; i < task.n_; ++i) {
    EXPECT_EQ((task.deadlines_[i] - task.deadlines_[0]) % 1ms, Clock::duration::zero());
  }
}

TEST(ControlLoop, StopIsIdempotentAndDestructorJoins) {
  RecordingTask task;
  {
    ControlLoop loop(test_config(1ms), task);
    loop.start();
    std::this_thread::sleep_for(20ms);
    loop.stop();
    loop.stop();  // second stop is a no-op
  }
  {
    ControlLoop loop(test_config(1ms), task);
    loop.start();
    std::this_thread::sleep_for(20ms);
  }  // destructor must stop and join; a leaked joinable thread would terminate()
  SUCCEED();
}

TEST(ControlLoop, RejectsNonPositivePeriod) {
  RecordingTask task;
  EXPECT_THROW(ControlLoop(test_config(0ns), task), std::invalid_argument);
}

TEST(ControlLoop, StartTwiceThrows) {
  RecordingTask task;
  ControlLoop loop(test_config(1ms), task);
  loop.start();
  EXPECT_THROW(loop.start(), std::logic_error);
}
