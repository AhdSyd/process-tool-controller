#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace ptc::telemetry {

// Bounded lock-free single-producer / single-consumer FIFO (Lamport queue).
//
// Used for logging: the control thread is the producer, the logging thread
// the consumer. Unlike the sensor ring, a log must not lose OLD entries, so
// when full the producer drops the NEW entry and counts it. try_push never
// blocks, which is the whole point: a slow disk can make the log incomplete
// (visibly, via dropped()), but it can never make the control loop late.
//
// Synchronisation:
//   producer: write slot, then head_.store(release)  -> publishes the slot
//   consumer: head_.load(acquire), read slot, then tail_.store(release)
//             -> hands the slot back to the producer
// Each index has exactly one writer. Each side caches the other's index and
// only re-reads it when it appears full/empty, keeping cache-line traffic low.
template <typename T, std::size_t N>
class SpscQueue {
  static_assert(std::is_trivially_copyable_v<T>);
  static_assert(N >= 2 && (N & (N - 1)) == 0, "capacity must be a power of two");

 public:
  static constexpr std::size_t kCapacity = N;

  // Producer only.
  bool try_push(const T& v) noexcept {
    const std::uint64_t head = head_.load(std::memory_order_relaxed);
    if (head - tail_cache_ >= N) {
      tail_cache_ = tail_.load(std::memory_order_acquire);
      if (head - tail_cache_ >= N) {
        dropped_.fetch_add(1, std::memory_order_relaxed);
        return false;
      }
    }
    buf_[head & (N - 1)] = v;
    head_.store(head + 1, std::memory_order_release);
    return true;
  }

  // Consumer only.
  bool try_pop(T& out) noexcept {
    const std::uint64_t tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_cache_) {
      head_cache_ = head_.load(std::memory_order_acquire);
      if (tail == head_cache_) return false;
    }
    out = buf_[tail & (N - 1)];
    tail_.store(tail + 1, std::memory_order_release);
    return true;
  }

  // Any thread: monitoring counter, no ordering implied.
  std::uint64_t dropped() const noexcept { return dropped_.load(std::memory_order_relaxed); }

 private:
  std::array<T, N> buf_{};
  alignas(64) std::atomic<std::uint64_t> head_{0};
  alignas(64) std::uint64_t tail_cache_ = 0;  // producer-private
  alignas(64) std::atomic<std::uint64_t> tail_{0};
  alignas(64) std::uint64_t head_cache_ = 0;  // consumer-private
  alignas(64) std::atomic<std::uint64_t> dropped_{0};
};

}  // namespace ptc::telemetry
