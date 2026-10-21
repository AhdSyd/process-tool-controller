#pragma once

#include <chrono>
#include <string>

namespace ptc::app {

// Line-oriented stdin reader for the main/UI thread that a signal can wake.
//
// Uses the classic self-pipe trick, which is portable (Linux and macOS):
// the signal handler writes one byte into a pipe; select() waits on both
// stdin and the pipe's read end. A signal therefore always wakes the UI
// thread promptly, with no window where it can be lost between "check flag"
// and "go to sleep".
//
// The process blocks SIGINT/SIGTERM in every thread except main, so the
// handler only ever runs on the UI thread and never interrupts the control
// thread mid-tick.
class Console {
 public:
  enum class Event { Line, Timeout, Signal };

  // `wake_fd` is the read end of the self-pipe.
  explicit Console(int wake_fd) : wake_fd_(wake_fd) {}

  // Waits up to `timeout` for a complete line, a signal, or nothing. After
  // stdin hits EOF it keeps waiting on signals/timeouts only, so headless
  // runs work with </dev/null.
  Event wait(std::string& out, std::chrono::milliseconds timeout);

 private:
  bool take_line(std::string& out);

  int wake_fd_;
  std::string pending_;
  bool stdin_open_ = true;
};

}  // namespace ptc::app
