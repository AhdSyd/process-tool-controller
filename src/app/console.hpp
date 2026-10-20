#pragma once

#include <signal.h>

#include <chrono>
#include <string>

namespace ptc::app {

// Line-oriented stdin reader for the main/UI thread that can be woken by a
// signal. The whole process runs with SIGINT/SIGTERM BLOCKED; the only place
// they are unblocked is inside ppoll() here, atomically, via the mask passed
// in. That guarantees:
//   * the handler always runs on the UI thread, never on the control thread
//     (so the control thread is never interrupted mid-tick by a handler), and
//   * there is no window where a signal arrives between "check flag" and
//     "go to sleep" and gets lost (the classic pselect/ppoll race fix).
class Console {
 public:
  enum class Event { Line, Timeout, Interrupted };

  explicit Console(const sigset_t& wait_mask) : wait_mask_(wait_mask) {}

  // Waits up to `timeout` for a complete line. Returns Line with `out` set,
  // Timeout, or Interrupted (a signal arrived). After stdin hits EOF it keeps
  // waiting on signals/timeouts only, so headless runs work with </dev/null.
  Event wait(std::string& out, std::chrono::milliseconds timeout);

  bool stdin_open() const noexcept { return stdin_open_; }

 private:
  bool take_line(std::string& out);

  sigset_t wait_mask_;
  std::string pending_;
  bool stdin_open_ = true;
};

}  // namespace ptc::app
