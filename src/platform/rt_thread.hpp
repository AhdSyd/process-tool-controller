#pragma once

namespace ptc::platform {

// What the OS actually granted the calling thread. Every field is best-effort:
// on a desktop or container without CAP_SYS_NICE the realtime request fails,
// and the program must report that rather than pretend otherwise (NFR-2).
struct RtStatus {
  bool fifo_granted = false;       // SCHED_FIFO accepted
  int fifo_priority = 0;           // priority actually in effect (0 if not FIFO)
  int fifo_errno = 0;              // errno from pthread_setschedparam, if it failed
  bool timer_slack_set = false;    // PR_SET_TIMERSLACK to 1 ns accepted
  int pinned_cpu = -1;             // CPU the thread is pinned to, -1 = not pinned
};

struct RtRequest {
  bool want_fifo = true;
  int fifo_priority = 80;  // 1..99; below kernel IRQ threads (50 by default) is
                           // sometimes preferred, but 80 is a common app choice
  int pin_cpu = -1;        // -1 = leave affinity alone
};

// Apply realtime settings to the CALLING thread. Call this once, from inside
// the control thread, before entering the loop. Never throws.
RtStatus configure_current_thread(const RtRequest& req, const char* name) noexcept;

// Try to lock all current and future pages into RAM so the control path never
// takes a major page fault. Returns 0 on success, otherwise errno.
int lock_process_memory() noexcept;

const char* describe_errno(int err) noexcept;

}  // namespace ptc::platform
