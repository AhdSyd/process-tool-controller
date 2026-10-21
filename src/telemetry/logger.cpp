#include "telemetry/logger.hpp"

#include <chrono>
#include <cstdio>
#include <iostream>
#include <stdexcept>

namespace ptc::telemetry {

namespace {

const char* command_name(std::uint8_t c) {
  switch (static_cast<CommandCode>(c)) {
    case CommandCode::Start: return "start";
    case CommandCode::Abort: return "abort";
    case CommandCode::Reset: return "reset";
    case CommandCode::Quit: return "quit";
  }
  return "?";
}

bool echo_worthy(LogEvent e) {
  switch (e) {
    case LogEvent::StateChange:
    case LogEvent::InterlockTrip:
    case LogEvent::FaultInjected:
    case LogEvent::WaferComplete:
    case LogEvent::SafeReached:
    case LogEvent::BufferOverrun: return true;
    default: return false;
  }
}

}  // namespace

std::size_t format_record(const LogRecord& r, std::int64_t t0_ns, const char* source, char* buf,
                          std::size_t cap) noexcept {
  const double t = static_cast<double>(r.t_ns - t0_ns) / 1e9;
  int n = 0;
  switch (r.event) {
    case LogEvent::RunStart:
      n = std::snprintf(buf, cap, "[%11.6f] %-4s RUN        start, control period %.0f us", t, source, r.x);
      break;
    case LogEvent::StateChange:
      n = std::snprintf(buf, cap, "[%11.6f] %-4s STATE      %-8s -> %-8s %s%s%s T=%.2fC P=%.3gTorr valve=%.2f", t,
                        source, state_name(static_cast<State>(r.a)), state_name(static_cast<State>(r.b)),
                        r.c ? "(" : "", r.c ? reason_name(static_cast<Reason>(r.c)) : "", r.c ? ")" : "",
                        r.x, r.y, r.z);
      break;
    case LogEvent::InterlockTrip:
      n = std::snprintf(buf, cap, "[%11.6f] %-4s INTERLOCK  TRIP  %-24s T=%.2fC P=%.3gTorr valve=%.2f", t,
                        source, reason_name(static_cast<Reason>(r.c)), r.x, r.y, r.z);
      break;
    case LogEvent::InterlockClear:
      n = std::snprintf(buf, cap, "[%11.6f] %-4s INTERLOCK  clear %s", t, source,
                        reason_name(static_cast<Reason>(r.c)));
      break;
    case LogEvent::WaferComplete:
      n = std::snprintf(buf, cap, "[%11.6f] %-4s WAFER      #%u complete, cycle %.3f s", t, source, r.u, r.x);
      break;
    case LogEvent::BufferOverrun:
      n = std::snprintf(buf, cap, "[%11.6f] %-4s OVERRUN    %u sensor samples lost (control thread late)", t,
                        source, r.u);
      break;
    case LogEvent::Command:
      n = std::snprintf(buf, cap, "[%11.6f] %-4s COMMAND    %s %u", t, source, command_name(r.a), r.u);
      break;
    case LogEvent::FaultInjected:
      n = std::snprintf(buf, cap, "[%11.6f] %-4s INJECT     simulated fault: %s", t, source,
                        fault_name(static_cast<FaultType>(r.a)));
      break;
    case LogEvent::SafeReached:
      n = std::snprintf(buf, cap, "[%11.6f] %-4s SAFE       safe state reached %.1f ms after %s trip", t, source,
                        r.x, reason_name(static_cast<Reason>(r.c)));
      break;
  }
  if (n < 0) return 0;
  return static_cast<std::size_t>(n) < cap ? static_cast<std::size_t>(n) : cap - 1;
}

Logger::Logger(const std::string& path, std::int64_t t0_ns, bool echo_to_stdout)
    : path_(path), out_(path, std::ios::out | std::ios::trunc), t0_ns_(t0_ns), echo_(echo_to_stdout) {
  if (!out_) throw std::runtime_error("cannot open log file: " + path);
}

Logger::~Logger() { stop(); }

void Logger::start() {
  if (thread_.joinable()) throw std::logic_error("Logger::start called twice");
  thread_ = std::thread(&Logger::run, this);
}

void Logger::stop() noexcept {
  stop_.store(true, std::memory_order_relaxed);
  if (thread_.joinable()) thread_.join();
}

void Logger::write(const LogRecord& r, const char* source) noexcept {
  char line[256];
  const std::size_t n = format_record(r, t0_ns_, source, line, sizeof(line));
  out_.write(line, static_cast<std::streamsize>(n));
  out_.put('\n');
  ++lines_;
  if (echo_ && echo_worthy(r.event)) {
    line[n] = '\n';
    std::cout.write(line, static_cast<std::streamsize>(n + 1));  // one write per line
    std::cout.flush();
  }
}

std::size_t Logger::drain_all() noexcept {
  std::size_t count = 0;
  LogRecord r;
  while (control_q_.try_pop(r)) { write(r, "ctl"); ++count; }
  while (sensor_q_.try_pop(r)) { write(r, "sen"); ++count; }
  while (ui_q_.try_pop(r)) { write(r, "ui"); ++count; }
  return count;
}

void Logger::run() noexcept {
  using namespace std::chrono_literals;
  while (!stop_.load(std::memory_order_relaxed)) {
    if (drain_all() == 0) std::this_thread::sleep_for(5ms);
  }
  // Producers have been stopped before the logger (shutdown order), so this
  // final drain sees everything that was ever enqueued.
  drain_all();
  out_.flush();
}

}  // namespace ptc::telemetry
