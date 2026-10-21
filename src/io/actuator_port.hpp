#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>

#include "common/types.hpp"

namespace ptc::io {

// The actuator command interface: control thread writes, plant reads.
//
// The whole command is packed into ONE 64-bit atomic word, so the plant can
// never observe a half-updated command (e.g. "vent on" from this tick with
// "pump on" from the last). One word also means one lock-free store per tick.
//
// Memory ordering: relaxed. The word is self-contained: it is the data, not a
// flag guarding other data, so there is nothing else whose visibility needs
// ordering against it. Atomicity alone gives "latest complete command".
//
// Heater duty is quantised to 16 bits (resolution 1.5e-5), far finer than any
// real SSR/PWM output stage.
class ActuatorPort {
 public:
  void write(const ActuatorCommand& c) noexcept { word_.store(pack(c), std::memory_order_relaxed); }
  ActuatorCommand read() const noexcept { return unpack(word_.load(std::memory_order_relaxed)); }

  static std::uint64_t pack(const ActuatorCommand& c) noexcept {
    const double h = std::clamp(c.heater, 0.0, 1.0);
    auto duty = static_cast<std::uint64_t>(std::lround(h * 65535.0));
    return duty | (std::uint64_t{c.heater_contactor} << 16) | (std::uint64_t{c.pump} << 17) |
           (std::uint64_t{c.vent} << 18) | (std::uint64_t{c.valve_open} << 19);
  }

  static ActuatorCommand unpack(std::uint64_t w) noexcept {
    ActuatorCommand c;
    c.heater = static_cast<double>(w & 0xFFFFu) / 65535.0;
    c.heater_contactor = (w >> 16) & 1u;
    c.pump = (w >> 17) & 1u;
    c.vent = (w >> 18) & 1u;
    c.valve_open = (w >> 19) & 1u;
    return c;
  }

 private:
  std::atomic<std::uint64_t> word_{0};  // all-zero = everything off: fail-safe default
};

}  // namespace ptc::io
