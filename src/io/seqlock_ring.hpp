#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace ptc::io {

// Single-producer / single-consumer OVERWRITING ring buffer (FR-6).
//
// Why overwrite instead of block or drop-newest:
//   The producer is a sensor. It must never wait for the consumer (a real ADC
//   keeps converting whether or not software is listening), and when the
//   consumer falls behind, the NEWEST data is the most valuable data for
//   control. So on overrun the producer overwrites the oldest slot and the
//   consumer detects, after the fact, exactly how many samples it lost.
//   Nothing is dropped silently: every lost sample is counted.
//
// How it is safe without locks:
//   Each slot carries its own sequence counter (a per-slot seqlock). The
//   producer marks the slot "being written" (odd), writes the payload, then
//   marks it "holds sample s" (even, 2s+2). The consumer reads the counter,
//   copies the payload, and re-reads the counter; if either check fails the
//   producer lapped it mid-read and the sample is counted as lost.
//
//   The payload itself is stored as std::atomic<uint64_t> words, so a torn
//   read is detected-and-discarded rather than being a C++ data race
//   (undefined behaviour).
//
// Memory ordering (Boehm, "Can seqlocks get along with programming language
// memory models?", 2012, fence-free variant):
//   writer: seq=odd (relaxed); payload words (RELEASE); seq=even (release)
//   reader: seq (acquire); payload words (ACQUIRE); seq again (relaxed)
//   If the reader sees ANY payload word from a newer write, its acquire load
//   synchronises with that release store, so the writer's earlier "odd" store
//   happens-before the reader's re-check, which therefore cannot still see the
//   old even value: the torn read is always caught. No standalone fences are
//   used, which also means ThreadSanitizer can verify every edge (it does not
//   model atomic_thread_fence). On x86 these orderings compile to plain moves.
//
// Neither side ever blocks, allocates, or makes a system call.
template <typename T, std::size_t N>
class SeqlockRing {
  static_assert(std::is_trivially_copyable_v<T>, "payload is copied as raw words");
  static_assert(N >= 2 && (N & (N - 1)) == 0, "capacity must be a power of two");
  static constexpr std::size_t kWords = (sizeof(T) + 7) / 8;

 public:
  static constexpr std::size_t kCapacity = N;

  struct DrainResult {
    std::uint64_t delivered = 0;  // samples handed to the callback
    std::uint64_t lost = 0;       // samples overwritten before they were read
  };

  // ---- producer side (exactly one thread) ---------------------------------
  void push(const T& value) noexcept {
    const std::uint64_t s = next_seq_;  // producer-private, no atomic needed
    Slot& slot = slots_[s & (N - 1)];

    std::uint64_t words[kWords] = {};
    std::memcpy(words, &value, sizeof(T));

    slot.seq.store(2 * s + 1, std::memory_order_relaxed);  // odd: write in progress
    // Release on each payload word: anyone who reads this word also sees the
    // odd marker above (see class comment).
    for (std::size_t i = 0; i < kWords; ++i) slot.words[i].store(words[i], std::memory_order_release);
    slot.seq.store(2 * s + 2, std::memory_order_release);  // even: sample s complete

    next_seq_ = s + 1;
    head_.store(s + 1, std::memory_order_release);  // publish: s+1 samples exist
  }

  // ---- consumer side (exactly one thread) ---------------------------------
  // Calls f(const T&) for every sample not yet consumed, oldest first.
  template <typename F>
  DrainResult drain(F&& f) noexcept {
    DrainResult r;
    const std::uint64_t head = head_.load(std::memory_order_acquire);

    // Producer lapped us: everything older than the last N samples is gone.
    if (head - tail_ > N) {
      r.lost += head - N - tail_;
      tail_ = head - N;
    }

    for (std::uint64_t s = tail_; s < head; ++s) {
      T out;
      if (read_slot(s, out)) {
        f(static_cast<const T&>(out));
        ++r.delivered;
      } else {
        ++r.lost;  // overwritten while we were reading it
      }
    }
    tail_ = head;
    return r;
  }

  // Total samples ever pushed (approximate from the consumer's view).
  std::uint64_t published() const noexcept { return head_.load(std::memory_order_relaxed); }

 private:
  struct alignas(64) Slot {  // one cache line per slot: no false sharing
    std::atomic<std::uint64_t> seq{0};
    std::array<std::atomic<std::uint64_t>, kWords> words{};
  };

  bool read_slot(std::uint64_t s, T& out) const noexcept {
    const Slot& slot = slots_[s & (N - 1)];
    const std::uint64_t expected = 2 * s + 2;
    if (slot.seq.load(std::memory_order_acquire) != expected) return false;

    std::uint64_t words[kWords];
    for (std::size_t i = 0; i < kWords; ++i) words[i] = slot.words[i].load(std::memory_order_acquire);
    // Re-check: if the producer started overwriting this slot, we either see
    // its odd marker or a later value here, never `expected`.
    if (slot.seq.load(std::memory_order_relaxed) != expected) return false;

    std::memcpy(&out, words, sizeof(T));
    return true;
  }

  std::array<Slot, N> slots_{};
  alignas(64) std::atomic<std::uint64_t> head_{0};  // written by producer only
  alignas(64) std::uint64_t next_seq_ = 0;          // producer-private
  alignas(64) std::uint64_t tail_ = 0;              // consumer-private
};

}  // namespace ptc::io
