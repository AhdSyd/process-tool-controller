#include "telemetry/latency_histogram.hpp"

#include <algorithm>
#include <cmath>

namespace ptc::telemetry {

LatencyHistogram::LatencyHistogram() noexcept { reset(); }

void LatencyHistogram::reset() noexcept {
  buckets_.fill(0);
  count_ = 0;
  negative_ = 0;
  max_ns_ = 0;
  min_ns_ = 0;
  sum_ns_ = 0;
}

void LatencyHistogram::record(std::int64_t ns) noexcept {
  if (ns < 0) {
    ++negative_;
    ns = 0;
  }
  const std::uint64_t us = static_cast<std::uint64_t>(ns) / 1000u;
  const std::size_t idx = us < kMaxTrackedUs ? static_cast<std::size_t>(us) : kMaxTrackedUs;
  ++buckets_[idx];

  if (count_ == 0) {
    max_ns_ = min_ns_ = ns;
  } else {
    max_ns_ = std::max(max_ns_, ns);
    min_ns_ = std::min(min_ns_, ns);
  }
  ++count_;
  sum_ns_ += static_cast<long double>(ns);
}

double LatencyHistogram::mean_ns() const noexcept {
  return count_ ? static_cast<double>(sum_ns_ / static_cast<long double>(count_)) : 0.0;
}

std::int64_t LatencyHistogram::percentile_ns(double p) const noexcept {
  if (count_ == 0) return 0;
  p = std::clamp(p, 0.0, 100.0);

  // Nearest-rank method: the smallest sample such that at least p% of all
  // samples are <= it. Rank is 1-based and always at least 1.
  // Multiply before dividing and shave a relative epsilon before ceil():
  // (99.9 / 100) * 1000 evaluates to 999.0000000000001 in binary floating
  // point, which would push the rank one sample too far.
  const double exact = p * static_cast<double>(count_) / 100.0;
  auto rank = static_cast<std::uint64_t>(std::ceil(exact - 1e-9 * std::max(1.0, exact)));
  rank = std::clamp<std::uint64_t>(rank, 1, count_);

  std::uint64_t cumulative = 0;
  for (std::size_t i = 0; i < kMaxTrackedUs; ++i) {
    cumulative += buckets_[i];
    if (cumulative >= rank) {
      const auto upper_edge = static_cast<std::int64_t>(i + 1) * 1000;
      return std::min(upper_edge, max_ns_);  // never report above the true max
    }
  }
  return max_ns_;  // rank falls in the overflow bucket
}

}  // namespace ptc::telemetry
