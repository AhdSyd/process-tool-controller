#pragma once

#include <cstdint>

#include "common/types.hpp"
#include "telemetry/spsc_queue.hpp"

namespace ptc::telemetry {

// What happened. The producer stores codes and numbers only; turning them
// into text is the logging thread's job (FR-8: the control thread never
// formats strings).
enum class LogEvent : std::uint8_t {
  RunStart,        // x = tick period us
  StateChange,     // a = from, b = to, c = reason; x,y,z = T, P, valve
  InterlockTrip,   // a = interlock id, c = reason; x,y,z = T, P, valve
  InterlockClear,  // a = interlock id
  WaferComplete,   // u = wafers done, x = cycle time s
  BufferOverrun,   // u = samples lost this tick
  Command,         // a = CommandCode, u = argument
  FaultInjected,   // a = FaultType
  SafeReached,     // c = reason, x = trip-to-safe ms
};

enum class CommandCode : std::uint8_t { Start, Abort, Reset, Quit };

// 40 bytes, trivially copyable, fixed size: copying one into a preallocated
// queue slot is the entire cost of logging on the control thread.
struct LogRecord {
  std::int64_t t_ns = 0;
  LogEvent event = LogEvent::RunStart;
  std::uint8_t a = 0, b = 0, c = 0;
  std::uint32_t u = 0;
  double x = 0, y = 0, z = 0;
};

// One queue per producing thread keeps every queue single-producer.
using LogQueue = SpscQueue<LogRecord, 4096>;

}  // namespace ptc::telemetry
