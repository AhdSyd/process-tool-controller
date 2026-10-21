#pragma once

#include <cstdint>

namespace ptc::platform {

// What the OS actually granted the calling thread. Every field is best-effort:
// without privileges the realtime request fails, and the program must report
// that rather than pretend otherwise (NFR-2).
struct RtStatus {
  bool rt_granted = false;     // Linux: SCHED_FIFO accepted. macOS: time-constraint policy accepted
  int rt_priority = 0;         // Linux FIFO priority in effect (0 on macOS / not granted)
  int rt_errno = 0;            // error from the request, if it failed
  bool timer_slack_set = false;  // Linux only: PR_SET_TIMERSLACK to 1 ns
  int pinned_cpu = -1;         // Linux only: CPU the thread is pinned to, -1 = not pinned
  const char* policy = "none"; // human-readable name of what was requested
};

struct RtRequest {
  bool want_rt = true;
  int fifo_priority = 80;        // Linux SCHED_FIFO 1..99
  int pin_cpu = -1;              // Linux: -1 = leave affinity alone
  std::int64_t period_ns = 1'000'000;  // macOS time-constraint policy needs the period
};

// Apply realtime settings to the CALLING thread. Call once, from inside the
// thread, before its loop. Never throws.
RtStatus configure_current_thread(const RtRequest& req, const char* name) noexcept;

// Try to lock all current and future pages into RAM so the control path never
// takes a major page fault. Returns 0 on success, otherwise an errno value.
int lock_process_memory() noexcept;

const char* describe_errno(int err) noexcept;

}  // namespace ptc::platform
