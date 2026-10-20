#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace ptc::telemetry {

// Fixed-resolution latency histogram for real-time measurement.
//
// Design constraints (NFR-3): record() is called from the control thread every
// tick, so it must never allocate, lock, or do anything with unbounded cost.
// All storage is a fixed std::array inside the object; the object itself is
// constructed before the control thread starts.
//
// Resolution: 1 microsecond buckets from 0 to kMaxTrackedUs-1. Anything at or
// above that lands in an overflow bucket, but the exact maximum is still kept,
// so "max" is always exact and percentiles in the overflow region are reported
// conservatively as the exact max.
//
// Percentiles are reported as the UPPER edge of the bucket that contains the
// requested rank, i.e. "at least p% of samples were <= this value". This
// errs on the pessimistic side by at most 1 us, which is the honest choice
// for a latency report.
//
// Not thread-safe by design: exactly one thread writes. Readers must only call
// the query functions after a happens-before edge with the writer (in this
// project: after std::thread::join on the control thread).
class LatencyHistogram {
 public:
  static constexpr std::size_t kMaxTrackedUs = 10'000;  // 10 ms = 10 tick periods

  LatencyHistogram() noexcept;

  // Record one sample in nanoseconds. Negative values are clamped to zero and
  // counted separately (they should not occur with a monotonic clock, so a
  // nonzero count indicates a measurement bug worth investigating).
  void record(std::int64_t ns) noexcept;

  void reset() noexcept;

  std::uint64_t count() const noexcept { return count_; }
  std::uint64_t negative_count() const noexcept { return negative_; }
  std::uint64_t overflow_count() const noexcept { return buckets_[kMaxTrackedUs]; }
  std::int64_t max_ns() const noexcept { return max_ns_; }
  std::int64_t min_ns() const noexcept { return count_ ? min_ns_ : 0; }
  double mean_ns() const noexcept;

  // p in [0, 100]. Returns nanoseconds (bucket upper edge, or exact max).
  std::int64_t percentile_ns(double p) const noexcept;

 private:
  std::array<std::uint64_t, kMaxTrackedUs + 1> buckets_{};  // +1 = overflow
  std::uint64_t count_ = 0;
  std::uint64_t negative_ = 0;
  std::int64_t max_ns_ = 0;
  std::int64_t min_ns_ = 0;
  long double sum_ns_ = 0;
};

}  // namespace ptc::telemetry
