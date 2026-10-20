#include "platform/rt_thread.hpp"

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/prctl.h>

#include <cerrno>
#include <cstring>

namespace ptc::platform {

RtStatus configure_current_thread(const RtRequest& req, const char* name) noexcept {
  RtStatus st;
  const pthread_t self = pthread_self();

  if (name != nullptr) {
    pthread_setname_np(self, name);  // max 15 chars; purely diagnostic (top -H, gdb)
  }

  if (req.pin_cpu >= 0) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(req.pin_cpu, &set);
    if (pthread_setaffinity_np(self, sizeof(set), &set) == 0) st.pinned_cpu = req.pin_cpu;
  }

  // Timer slack: by default Linux may delay a normal thread's timer expiry by
  // up to 50 us so it can coalesce wakeups and save power. That slack shows up
  // directly as tick lateness. 1 ns is the minimum; 0 would mean "reset to the
  // default", which is a classic trap.
  st.timer_slack_set = (prctl(PR_SET_TIMERSLACK, 1UL, 0UL, 0UL, 0UL) == 0);

  if (req.want_fifo) {
    sched_param sp{};
    sp.sched_priority = req.fifo_priority;
    const int rc = pthread_setschedparam(self, SCHED_FIFO, &sp);
    if (rc == 0) {
      st.fifo_granted = true;
      st.fifo_priority = req.fifo_priority;
    } else {
      st.fifo_errno = rc;  // pthread_* return the error, they do not set errno
    }
  }
  return st;
}

int lock_process_memory() noexcept {
  return mlockall(MCL_CURRENT | MCL_FUTURE) == 0 ? 0 : errno;
}

const char* describe_errno(int err) noexcept {
  switch (err) {
    case -1: return "not requested";
    case 0: return "ok";
    case EPERM: return "EPERM (needs CAP_SYS_NICE / rtprio limit)";
    case ENOMEM: return "ENOMEM (RLIMIT_MEMLOCK too low)";
    case EINVAL: return "EINVAL";
    default: return std::strerror(err);
  }
}

}  // namespace ptc::platform
