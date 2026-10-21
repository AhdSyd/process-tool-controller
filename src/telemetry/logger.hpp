#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>
#include <thread>

#include "telemetry/log_record.hpp"

namespace ptc::telemetry {

// Formats one record as a text line (no trailing newline). Pure; returns the
// number of characters written. Runs only on the logging thread (and tests).
std::size_t format_record(const LogRecord& r, std::int64_t t0_ns, const char* source, char* buf,
                          std::size_t cap) noexcept;

// The logging thread (FR-8). Owns one SPSC queue per producing thread, so
// every queue keeps exactly one producer and nothing on the logging path ever
// takes a lock. It wakes every few milliseconds, drains all queues, formats
// and writes to disk.
//
// Why no condition variable to wake it: notifying one would mean the control
// thread touching a mutex the logger also holds. If the logger were
// descheduled while holding it, the control thread would block behind a
// low-priority thread (priority inversion). Polling every 5 ms costs nothing
// and keeps the control thread entirely lock-free.
class Logger {
 public:
  // Throws std::runtime_error if the file cannot be opened.
  Logger(const std::string& path, std::int64_t t0_ns, bool echo_to_stdout);
  ~Logger();

  Logger(const Logger&) = delete;
  Logger& operator=(const Logger&) = delete;

  LogQueue& control_queue() noexcept { return control_q_; }
  LogQueue& sensor_queue() noexcept { return sensor_q_; }
  LogQueue& ui_queue() noexcept { return ui_q_; }

  void start();
  void stop() noexcept;  // idempotent; final drain + flush, then join

  // Valid after stop().
  std::uint64_t lines_written() const noexcept { return lines_; }
  std::uint64_t dropped() const noexcept {
    return control_q_.dropped() + sensor_q_.dropped() + ui_q_.dropped();
  }
  const std::string& path() const noexcept { return path_; }

 private:
  void run() noexcept;
  std::size_t drain_all() noexcept;
  void write(const LogRecord& r, const char* source) noexcept;

  std::string path_;
  std::ofstream out_;  // RAII: closed (and flushed) when the Logger is destroyed
  const std::int64_t t0_ns_;
  const bool echo_;
  std::uint64_t lines_ = 0;

  LogQueue control_q_;
  LogQueue sensor_q_;
  LogQueue ui_q_;

  std::atomic<bool> stop_{false};
  std::thread thread_;
};

}  // namespace ptc::telemetry
