// Process Tool Controller -- entry point and main/UI thread.
//
// Milestone 1: control thread ticking at a fixed period, jitter measurement,
// clean shutdown on `quit`, Ctrl-C (SIGINT), SIGTERM, or --duration expiry.

#include <pthread.h>
#include <signal.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <optional>
#include <string>

#include "app/console.hpp"
#include "control/control_loop.hpp"
#include "platform/rt_thread.hpp"
#include "telemetry/report.hpp"

namespace {

using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Signal plumbing. The handler does the only async-signal-safe thing we need:
// set a lock-free atomic flag. All real shutdown work happens on the UI thread.
// ---------------------------------------------------------------------------
std::atomic<bool> g_signalled{false};
static_assert(std::atomic<bool>::is_always_lock_free,
              "signal handler requires a lock-free atomic");

extern "C" void on_signal(int) { g_signalled.store(true, std::memory_order_relaxed); }

// M1 has no plant yet. The tick does nothing; we are measuring the loop itself.
class IdleTask final : public ptc::control::TickTask {
 public:
  void on_tick(const ptc::control::TickContext&) noexcept override {}
};

struct Options {
  double duration_s = 0.0;  // 0 = run until quit / Ctrl-C
  long period_us = 1000;
  bool want_fifo = true;
  int fifo_priority = 80;
  int pin_cpu = -1;
  bool mlock = true;
};

void usage(const char* argv0) {
  std::cerr << "usage: " << argv0 << " [options]\n"
            << "  --duration <sec>   run headless for this long, then report (default: interactive)\n"
            << "  --period-us <us>   control tick period (default 1000)\n"
            << "  --cpu <n>          pin the control thread to CPU n\n"
            << "  --priority <1-99>  SCHED_FIFO priority to request (default 80)\n"
            << "  --no-rt            do not request SCHED_FIFO\n"
            << "  --no-mlock         do not lock process memory\n";
}

std::optional<Options> parse(int argc, char** argv) {
  Options o;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> const char* { return i + 1 < argc ? argv[++i] : nullptr; };
    const char* v = nullptr;
    if (a == "--duration" && (v = next())) o.duration_s = std::atof(v);
    else if (a == "--period-us" && (v = next())) o.period_us = std::atol(v);
    else if (a == "--cpu" && (v = next())) o.pin_cpu = std::atoi(v);
    else if (a == "--priority" && (v = next())) o.fifo_priority = std::atoi(v);
    else if (a == "--no-rt") o.want_fifo = false;
    else if (a == "--no-mlock") o.mlock = false;
    else return std::nullopt;
  }
  if (o.period_us <= 0 || o.duration_s < 0 || o.fifo_priority < 1 || o.fifo_priority > 99)
    return std::nullopt;
  return o;
}

void print_status(const ptc::control::ControlLoop& loop) {
  const auto s = loop.live();
  std::cout << "ticks=" << s.ticks << "  missed=" << s.missed_deadlines
            << "  max_lateness=" << static_cast<double>(s.max_lateness_ns) / 1000.0 << " us"
            << std::endl;
}

}  // namespace

int main(int argc, char** argv) {
  const auto opts = parse(argc, argv);
  if (!opts) {
    usage(argv[0]);
    return 2;
  }

  // 1. Block SIGINT/SIGTERM BEFORE creating any thread. Threads inherit the
  //    mask, so the control thread can never be chosen to run the handler.
  sigset_t shutdown_signals, original_mask;
  sigemptyset(&shutdown_signals);
  sigaddset(&shutdown_signals, SIGINT);
  sigaddset(&shutdown_signals, SIGTERM);
  pthread_sigmask(SIG_BLOCK, &shutdown_signals, &original_mask);

  struct sigaction sa {};
  sa.sa_handler = on_signal;
  sigemptyset(&sa.sa_mask);
  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);

  // 2. Lock memory (best effort) so the control path never page-faults to disk.
  const int mlock_err = opts->mlock ? ptc::platform::lock_process_memory() : -1;

  // 3. Build and start the control loop. Ownership: main owns the task and
  //    the loop; the loop holds a non-owning reference to the task, and the
  //    task is declared first so it is destroyed last.
  IdleTask task;
  ptc::control::ControlLoopConfig cfg;
  cfg.period = std::chrono::microseconds(opts->period_us);
  cfg.rt.want_fifo = opts->want_fifo;
  cfg.rt.fifo_priority = opts->fifo_priority;
  cfg.rt.pin_cpu = opts->pin_cpu;
  auto loop = std::make_unique<ptc::control::ControlLoop>(cfg, task);
  const ptc::platform::RtStatus rt = loop->start();

  std::cout << "ptc: control loop running at " << opts->period_us << " us period"
            << (rt.fifo_granted ? " [SCHED_FIFO]" : " [SCHED_OTHER - no RT priority]") << '\n';
  if (opts->duration_s > 0) std::cout << "ptc: headless run for " << opts->duration_s << " s\n";
  else std::cout << "ptc: commands: status | quit   (Ctrl-C also quits)\n";
  std::cout.flush();

  // 4. UI loop. Signals are unblocked only inside Console::wait (ppoll).
  ptc::app::Console console(original_mask);
  const auto t_end = std::chrono::steady_clock::now() +
                     std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                         std::chrono::duration<double>(opts->duration_s));
  std::string line;
  bool quit = false;
  while (!quit && !g_signalled.load(std::memory_order_relaxed)) {
    if (opts->duration_s > 0 && std::chrono::steady_clock::now() >= t_end) break;

    switch (console.wait(line, 200ms)) {
      case ptc::app::Console::Event::Line:
        if (line == "quit" || line == "exit" || line == "q") quit = true;
        else if (line == "status" || line == "s") print_status(*loop);
        else if (line == "help" || line == "?") std::cout << "commands: status | quit\n";
        else if (!line.empty()) std::cout << "unknown command: " << line << " (try 'help')\n";
        break;
      case ptc::app::Console::Event::Interrupted:
      case ptc::app::Console::Event::Timeout:
        break;
    }
  }
  if (g_signalled.load(std::memory_order_relaxed)) std::cout << "\nptc: signal received\n";
  std::cout << "ptc: shutting down\n";

  // 5. Orderly shutdown: stop + join the control thread, THEN read its results.
  loop->stop();

  ptc::telemetry::RunMetrics m;
  m.period = loop->period();
  m.runtime = loop->run_duration();
  m.ticks = loop->wake_lateness().count();
  m.missed_deadlines = loop->live().missed_deadlines;
  m.rt = rt;
  m.mlock_errno = mlock_err;
  ptc::telemetry::print_report(std::cout, m, loop->wake_lateness(), loop->exec_time());
  return 0;
}
