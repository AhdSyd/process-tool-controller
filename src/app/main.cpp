// Process Tool Controller: entry point and main/UI thread.
//
// Threads (exactly four):
//   1. control  - PeriodicLoop(1 ms)   running Controller   (highest priority)
//   2. sensor   - PeriodicLoop(200 us) running SensorTask   (plant + 5 kHz sampling)
//   3. logging  - Logger: drains lock-free queues, writes the log file
//   4. main/UI  - this thread: console commands, signals, final report

#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>

#include "app/console.hpp"
#include "control/controller.hpp"
#include "control/periodic_loop.hpp"
#include "io/sensor_task.hpp"
#include "platform/rt_thread.hpp"
#include "sim/plant.hpp"
#include "telemetry/logger.hpp"
#include "telemetry/report.hpp"

namespace {

using namespace std::chrono_literals;
using ptc::State;

// ---------------------------------------------------------------------------
// Signals: the handler does only async-signal-safe things: bump a lock-free
// counter and write one byte into the self-pipe to wake the UI thread.
// ---------------------------------------------------------------------------
std::atomic<int> g_signals{0};
int g_wake_write_fd = -1;
static_assert(std::atomic<int>::is_always_lock_free, "signal handler requires a lock-free atomic");

extern "C" void on_signal(int) {
  const int saved = errno;
  g_signals.fetch_add(1, std::memory_order_relaxed);
  const char byte = 's';
  [[maybe_unused]] const ssize_t n = write(g_wake_write_fd, &byte, 1);
  errno = saved;
}

struct Options {
  std::uint32_t wafers = 0;  // >0: headless, run N wafers then exit
  double duration_s = 0.0;   // >0: headless, stop after this long
  long period_us = 1000;
  bool want_rt = true;
  int priority = 80;
  int cpu = -1;
  bool mlock = true;
  bool quiet = false;
  std::string log_path = "ptc_run.log";
  ptc::io::FaultPlan fault{};
  double process_time_s = 3.0;
  std::uint64_t seed = 12345;
};

void usage(const char* argv0) {
  std::cerr
      << "usage: " << argv0 << " [options]\n"
      << "  (no options)           interactive console: start [n] | abort | reset | status | quit\n"
      << "  --wafers <n>           headless: process n wafers, print report, exit\n"
      << "  --duration <sec>       headless: stop after this long (combine with --wafers to cap)\n"
      << "  --fault <name>         inject a simulated fault: sensor-dropout | temp-runaway |\n"
      << "                         stuck-valve | vacuum-leak\n"
      << "  --fault-state <state>  recipe state that arms the fault (default Process)\n"
      << "  --fault-delay-ms <ms>  delay after entering that state (default 500)\n"
      << "  --process-time <sec>   Process step duration (default 3)\n"
      << "  --log <path>           log file (default ptc_run.log)\n"
      << "  --quiet                do not echo events to the console\n"
      << "  --period-us <us>       control tick period (default 1000)\n"
      << "  --priority <1-99>      Linux SCHED_FIFO priority for the control thread (default 80)\n"
      << "  --cpu <n>              Linux: pin the control thread to CPU n\n"
      << "  --no-rt                do not request realtime scheduling\n"
      << "  --no-mlock             do not lock process memory\n"
      << "  --seed <n>             sensor-noise seed (default 12345)\n";
}

std::optional<ptc::FaultType> parse_fault(const std::string& s) {
  for (auto f : {ptc::FaultType::SensorDropout, ptc::FaultType::TemperatureRunaway,
                 ptc::FaultType::StuckValve, ptc::FaultType::VacuumLeak}) {
    if (s == ptc::fault_name(f)) return f;
  }
  return std::nullopt;
}

std::optional<State> parse_state(const std::string& s) {
  for (std::size_t i = 0; i < static_cast<std::size_t>(State::kCount); ++i) {
    if (s == ptc::state_name(static_cast<State>(i))) return static_cast<State>(i);
  }
  return std::nullopt;
}

std::optional<Options> parse(int argc, char** argv) {
  Options o;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::optional<std::string> {
      if (i + 1 < argc) return std::string(argv[++i]);
      return std::nullopt;
    };
    std::optional<std::string> v;
    if (a == "--wafers" && (v = next())) o.wafers = static_cast<std::uint32_t>(std::stoul(*v));
    else if (a == "--duration" && (v = next())) o.duration_s = std::stod(*v);
    else if (a == "--period-us" && (v = next())) o.period_us = std::stol(*v);
    else if (a == "--priority" && (v = next())) o.priority = std::stoi(*v);
    else if (a == "--cpu" && (v = next())) o.cpu = std::stoi(*v);
    else if (a == "--log" && (v = next())) o.log_path = *v;
    else if (a == "--process-time" && (v = next())) o.process_time_s = std::stod(*v);
    else if (a == "--seed" && (v = next())) o.seed = std::stoull(*v);
    else if (a == "--fault-delay-ms" && (v = next())) o.fault.delay_ns = std::stoll(*v) * ptc::kNsPerMs;
    else if (a == "--fault" && (v = next())) {
      const auto f = parse_fault(*v);
      if (!f) return std::nullopt;
      o.fault.type = *f;
    } else if (a == "--fault-state" && (v = next())) {
      const auto s = parse_state(*v);
      if (!s) return std::nullopt;
      o.fault.trigger_state = *s;
    }
    else if (a == "--no-rt") o.want_rt = false;
    else if (a == "--no-mlock") o.mlock = false;
    else if (a == "--quiet") o.quiet = true;
    else return std::nullopt;
  }
  if (o.period_us <= 0 || o.duration_s < 0 || o.priority < 1 || o.priority > 99 || o.process_time_s <= 0)
    return std::nullopt;
  return o;
}

void print_status(const ptc::control::StatusBoard& s, const ptc::control::PeriodicLoop& loop) {
  constexpr auto rlx = std::memory_order_relaxed;
  const auto st = static_cast<State>(s.state.load(rlx));
  const auto live = loop.live();
  std::ostringstream os;
  os << std::fixed << "state=" << ptc::state_name(st);
  if (st == State::Aborting || st == State::Fault)
    os << "(" << ptc::reason_name(static_cast<ptc::Reason>(s.reason.load(rlx))) << ")";
  os << std::setprecision(2) << "  T=" << s.temperature_c.load(rlx) << "C" << std::scientific
     << std::setprecision(2) << "  P=" << s.pressure_torr.load(rlx) << "Torr" << std::fixed
     << std::setprecision(2) << "  valve=" << s.valve_position.load(rlx) << std::setprecision(0)
     << "  heater=" << 100.0 * s.heater.load(rlx) << "%"
     << "  wafers=" << s.wafers_done.load(rlx) << " done/" << s.wafers_pending.load(rlx) << " queued"
     << "  ticks=" << live.ticks << " missed=" << live.missed_deadlines
     << "  overruns=" << s.overruns.load(rlx) << '\n';
  std::cout << os.str() << std::flush;
}

void log_command(ptc::telemetry::LogQueue& q, ptc::telemetry::CommandCode c, std::uint32_t arg) {
  ptc::telemetry::LogRecord r;
  r.t_ns = ptc::now_ns();
  r.event = ptc::telemetry::LogEvent::Command;
  r.a = static_cast<std::uint8_t>(c);
  r.u = arg;
  q.try_push(r);
}

}  // namespace

int main(int argc, char** argv) {
  std::optional<Options> opts;
  try {
    opts = parse(argc, argv);
  } catch (const std::exception&) {
    opts.reset();
  }
  if (!opts) {
    usage(argv[0]);
    return 2;
  }
  const bool headless = opts->wafers > 0 || opts->duration_s > 0;

  // --- 1. Signals: self-pipe + block in every thread we are about to create ---
  int pipe_fds[2];
  if (pipe(pipe_fds) != 0) {
    std::perror("pipe");
    return 1;
  }
  for (int fd : pipe_fds) {
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);
    fcntl(fd, F_SETFD, FD_CLOEXEC);
  }
  g_wake_write_fd = pipe_fds[1];

  sigset_t shutdown_signals, original_mask;
  sigemptyset(&shutdown_signals);
  sigaddset(&shutdown_signals, SIGINT);
  sigaddset(&shutdown_signals, SIGTERM);
  pthread_sigmask(SIG_BLOCK, &shutdown_signals, &original_mask);  // inherited by new threads
  struct sigaction sa{};
  sa.sa_handler = on_signal;
  sigemptyset(&sa.sa_mask);
  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);

  // --- 2. Build the system. ALL heap allocation for the run happens here, ---
  // before any real-time thread starts. Ownership is a straight line: main
  // owns everything through unique_ptr; components hold references to their
  // collaborators. Declaration order = construction order; destruction runs
  // in reverse, so the loops (threads) are always joined before the objects
  // they reference are destroyed.
  const std::int64_t t0 = ptc::now_ns();

  ptc::sim::PlantParams plant_params;
  plant_params.seed = opts->seed;
  ptc::control::ControllerConfig ctl_cfg;
  ctl_cfg.recipe.process_time_ns = static_cast<std::int64_t>(opts->process_time_s * 1e9);

  std::unique_ptr<ptc::telemetry::Logger> logger;
  try {
    logger = std::make_unique<ptc::telemetry::Logger>(opts->log_path, t0, !opts->quiet);
  } catch (const std::exception& e) {
    std::cerr << "ptc: " << e.what() << '\n';
    return 1;
  }
  auto plant = std::make_unique<ptc::sim::Plant>(plant_params);
  auto ring = std::make_unique<ptc::io::SensorRing>();
  auto port = std::make_unique<ptc::io::ActuatorPort>();
  auto mailbox = std::make_unique<ptc::control::CommandMailbox>();
  auto status = std::make_unique<ptc::control::StatusBoard>();

  constexpr std::int64_t kSensorPeriodNs = 200 * ptc::kNsPerUs;
  auto sensor_task = std::make_unique<ptc::io::SensorTask>(*plant, *ring, *port, status->state,
                                                           logger->sensor_queue(), opts->fault, kSensorPeriodNs);
  auto controller = std::make_unique<ptc::control::Controller>(ctl_cfg, *ring, *port, *mailbox, *status,
                                                               logger->control_queue());
  controller->set_latency_clock(&ptc::now_ns);

  ptc::control::PeriodicLoopConfig sensor_cfg;
  sensor_cfg.period = std::chrono::nanoseconds(kSensorPeriodNs);
  sensor_cfg.rt.want_rt = opts->want_rt;
  sensor_cfg.rt.fifo_priority = std::max(1, opts->priority - 10);  // below control
  sensor_cfg.thread_name = "ptc-sensor";
  auto sensor_loop = std::make_unique<ptc::control::PeriodicLoop>(sensor_cfg, *sensor_task);

  ptc::control::PeriodicLoopConfig control_cfg;
  control_cfg.period = std::chrono::microseconds(opts->period_us);
  control_cfg.rt.want_rt = opts->want_rt;
  control_cfg.rt.fifo_priority = opts->priority;
  control_cfg.rt.pin_cpu = opts->cpu;
  control_cfg.thread_name = "ptc-control";
  auto control_loop = std::make_unique<ptc::control::PeriodicLoop>(control_cfg, *controller);

  // --- 3. Start threads: logger, then sensors, then control. ---------------
  logger->start();
  {
    ptc::telemetry::LogRecord r;
    r.t_ns = t0;
    r.event = ptc::telemetry::LogEvent::RunStart;
    r.x = static_cast<double>(opts->period_us);
    logger->ui_queue().try_push(r);
  }
  const auto sensor_rt = sensor_loop->start();
  // Do not start control until data is flowing; otherwise its first tick
  // would (correctly) trip the stale-data interlock.
  for (int i = 0; i < 1000 && ring->published() == 0; ++i) std::this_thread::sleep_for(1ms);
  const auto control_rt = control_loop->start();

  const int mlock_err = opts->mlock ? ptc::platform::lock_process_memory() : -1;
  pthread_sigmask(SIG_SETMASK, &original_mask, nullptr);  // only main receives signals

  std::cout << "ptc: control " << opts->period_us << " us, sensors 200 us"
            << (control_rt.rt_granted ? std::string(" [") + control_rt.policy + "]"
                                      : std::string(" [no realtime priority]"))
            << ", log -> " << opts->log_path << '\n';
  if (opts->fault.type != ptc::FaultType::None) {
    std::cout << "ptc: will inject '" << ptc::fault_name(opts->fault.type) << "' "
              << opts->fault.delay_ns / ptc::kNsPerMs << " ms into " << ptc::state_name(opts->fault.trigger_state)
              << '\n';
  }
  if (!headless) std::cout << "ptc: commands: start [n] | abort | reset | status | quit   (Ctrl-C quits)\n";
  std::cout.flush();

  if (opts->wafers > 0) {
    mailbox->wafers.fetch_add(opts->wafers, std::memory_order_relaxed);
    log_command(logger->ui_queue(), ptc::telemetry::CommandCode::Start, opts->wafers);
  }

  // --- 4. UI loop -----------------------------------------------------------
  ptc::app::Console console(pipe_fds[0]);
  const auto t_start = std::chrono::steady_clock::now();
  const auto deadline = t_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                      std::chrono::duration<double>(opts->duration_s));
  std::uint32_t requested = opts->wafers;
  std::string line;
  bool quit = false;

  auto run_finished = [&] {
    // Acquire on the publication point first; see StatusBoard.
    if (status->wafers_accepted.load(std::memory_order_acquire) != requested) return false;
    const auto st = static_cast<State>(status->state.load(std::memory_order_relaxed));
    return (st == State::Idle || st == State::Fault) &&
           status->wafers_pending.load(std::memory_order_relaxed) == 0;
  };

  while (!quit && g_signals.load(std::memory_order_relaxed) == 0) {
    if (opts->duration_s > 0 && std::chrono::steady_clock::now() >= deadline) break;
    if (opts->wafers > 0 && run_finished()) break;

    if (console.wait(line, 50ms) != ptc::app::Console::Event::Line) continue;
    std::istringstream in(line);
    std::string cmd;
    in >> cmd;
    if (cmd == "quit" || cmd == "exit" || cmd == "q") {
      quit = true;
    } else if (cmd == "start") {
      std::uint32_t n = 1;
      in >> n;
      if (n == 0) n = 1;
      requested += n;
      mailbox->wafers.fetch_add(n, std::memory_order_relaxed);
      log_command(logger->ui_queue(), ptc::telemetry::CommandCode::Start, n);
      std::cout << "queued " << n << " wafer(s)\n";
    } else if (cmd == "abort") {
      mailbox->abort.store(true, std::memory_order_relaxed);
      log_command(logger->ui_queue(), ptc::telemetry::CommandCode::Abort, 0);
    } else if (cmd == "reset") {
      mailbox->reset.store(true, std::memory_order_relaxed);
      log_command(logger->ui_queue(), ptc::telemetry::CommandCode::Reset, 0);
    } else if (cmd == "status" || cmd == "s") {
      print_status(*status, *control_loop);
    } else if (cmd == "help" || cmd == "?") {
      std::cout << "commands: start [n] | abort | reset | status | quit\n";
    } else if (!cmd.empty()) {
      std::cout << "unknown command: " << cmd << " (try 'help')\n";
    }
  }
  if (g_signals.load(std::memory_order_relaxed) > 0) std::cout << "\nptc: signal received\n";

  // --- 5. Leave the chamber safe before stopping the software ---------------
  {
    const auto st = static_cast<State>(status->state.load(std::memory_order_relaxed));
    if (st != State::Idle && st != State::Fault) {
      std::cout << "ptc: recipe active (" << ptc::state_name(st)
                << "), aborting to a safe state before exit (Ctrl-C again to force)\n";
      const int signals_before = g_signals.load(std::memory_order_relaxed);
      mailbox->abort.store(true, std::memory_order_relaxed);
      log_command(logger->ui_queue(), ptc::telemetry::CommandCode::Abort, 0);
      const auto give_up = std::chrono::steady_clock::now() + 7s;
      while (std::chrono::steady_clock::now() < give_up &&
             g_signals.load(std::memory_order_relaxed) == signals_before) {
        const auto s = static_cast<State>(status->state.load(std::memory_order_relaxed));
        if (s == State::Idle || s == State::Fault) break;
        console.wait(line, 20ms);
      }
    }
  }
  std::cout << "ptc: shutting down\n";

  // --- 6. Orderly shutdown: producers first, logger last (so its final drain
  // sees everything), then read results. Each stop() joins, which is the
  // happens-before edge that makes the thread's plain writes visible here.
  control_loop->stop();
  sensor_loop->stop();
  log_command(logger->ui_queue(), ptc::telemetry::CommandCode::Quit, 0);
  logger->stop();

  ptc::telemetry::ReportData d;
  d.runtime = std::chrono::steady_clock::now() - t_start;
  d.control = {control_loop->period(), control_loop->wake_lateness().count(),
               control_loop->live().missed_deadlines, &control_loop->wake_lateness(),
               &control_loop->exec_time(), control_rt};
  d.sensor = {sensor_loop->period(), sensor_loop->wake_lateness().count(), sensor_loop->live().missed_deadlines,
              &sensor_loop->wake_lateness(), &sensor_loop->exec_time(), sensor_rt};
  d.samples_published = sensor_task->samples_published();
  d.ctl = &controller->metrics();
  d.log_lines = logger->lines_written();
  d.log_dropped = logger->dropped();
  d.log_path = logger->path();
  d.mlock_errno = mlock_err;
  d.injected = opts->fault.type;
  ptc::telemetry::print_report(std::cout, d);

  close(pipe_fds[0]);
  close(pipe_fds[1]);
  return 0;
}
