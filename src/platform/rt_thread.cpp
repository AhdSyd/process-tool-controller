#include "platform/rt_thread.hpp"

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>

#include <algorithm>
#include <cerrno>
#include <cstring>

#if defined(__linux__)
#include <sys/prctl.h>
#elif defined(__APPLE__)
#include <mach/mach.h>
#include <mach/mach_time.h>
#include <mach/thread_policy.h>
#endif

namespace ptc::platform {

#if defined(__linux__)

RtStatus configure_current_thread(const RtRequest& req, const char* name) noexcept {
  RtStatus st;
  const pthread_t self = pthread_self();
  if (name != nullptr) pthread_setname_np(self, name);  // max 15 chars; shows in top -H, gdb

  if (req.pin_cpu >= 0) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(req.pin_cpu, &set);
    if (pthread_setaffinity_np(self, sizeof(set), &set) == 0) st.pinned_cpu = req.pin_cpu;
  }

  // Timer slack: by default Linux may fire a normal thread's timer up to 50 us
  // late so it can batch wakeups. That shows up directly as tick lateness.
  // 1 ns is the minimum; 0 would mean "reset to default", a classic trap.
  st.timer_slack_set = (prctl(PR_SET_TIMERSLACK, 1UL, 0UL, 0UL, 0UL) == 0);

  if (req.want_rt) {
    st.policy = "SCHED_FIFO";
    sched_param sp{};
    sp.sched_priority = req.fifo_priority;
    const int rc = pthread_setschedparam(self, SCHED_FIFO, &sp);
    if (rc == 0) {
      st.rt_granted = true;
      st.rt_priority = req.fifo_priority;
    } else {
      st.rt_errno = rc;  // pthread_* return the error; they do not set errno
    }
  }
  return st;
}

// MCL_CURRENT only, called once every thread and buffer exists. MCL_FUTURE
// would also lock later mappings, but if RLIMIT_MEMLOCK is finite it turns a
// later malloc/mmap (even on the non-RT UI thread) into a hard failure. The
// real-time threads allocate nothing after start-up, so locking what exists
// at that point covers everything they will ever touch.
int lock_process_memory() noexcept { return mlockall(MCL_CURRENT) == 0 ? 0 : errno; }

#elif defined(__APPLE__)

// macOS has no usable SCHED_FIFO for applications. Its realtime mechanism is
// the Mach "time constraint" policy (what Core Audio threads use): the thread
// declares its period and its CPU budget per period, and the scheduler treats
// it as realtime. No root required.
RtStatus configure_current_thread(const RtRequest& req, const char* name) noexcept {
  RtStatus st;
  if (name != nullptr) pthread_setname_np(name);  // macOS: names the calling thread only

  if (req.want_rt) {
    st.policy = "mach time-constraint";
    mach_timebase_info_data_t tb;
    mach_timebase_info(&tb);
    auto to_abs = [&](std::int64_t ns) -> std::uint32_t {
      const auto v = static_cast<std::uint64_t>(ns) * tb.denom / tb.numer;
      return static_cast<std::uint32_t>(std::min<std::uint64_t>(v, 0xFFFFFFFFu));
    };
    const std::int64_t period = std::max<std::int64_t>(req.period_ns, 100'000);
    thread_time_constraint_policy_data_t pol;
    pol.period = to_abs(period);
    pol.computation = to_abs(std::max<std::int64_t>(50'000, period / 4));  // budget per period
    pol.constraint = to_abs(period);                                       // must finish within
    pol.preemptible = TRUE;
    const kern_return_t kr =
        thread_policy_set(pthread_mach_thread_np(pthread_self()), THREAD_TIME_CONSTRAINT_POLICY,
                          reinterpret_cast<thread_policy_t>(&pol), THREAD_TIME_CONSTRAINT_POLICY_COUNT);
    if (kr == KERN_SUCCESS) st.rt_granted = true;
    else st.rt_errno = EPERM;
  }
  return st;
}

// macOS does not support mlockall for user processes.
int lock_process_memory() noexcept { return ENOTSUP; }

#else
#error "Unsupported platform: Linux and macOS only"
#endif

const char* describe_errno(int err) noexcept {
  switch (err) {
    case -1: return "not requested";
    case 0: return "ok";
    case EPERM: return "EPERM (needs privileges: run with sudo or CAP_SYS_NICE)";
    case ENOMEM: return "ENOMEM (RLIMIT_MEMLOCK too low)";
    case ENOTSUP: return "not supported on this OS";
    default: return std::strerror(err);
  }
}

}  // namespace ptc::platform
