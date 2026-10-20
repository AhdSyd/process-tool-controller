#include <gtest/gtest.h>

#include <memory>

#include "telemetry/latency_histogram.hpp"

using ptc::telemetry::LatencyHistogram;

namespace {
// 80 KB of buckets: keep test instances off the stack.
std::unique_ptr<LatencyHistogram> make() { return std::make_unique<LatencyHistogram>(); }
}  // namespace

TEST(LatencyHistogram, EmptyReportsZero) {
  auto h = make();
  EXPECT_EQ(h->count(), 0u);
  EXPECT_EQ(h->percentile_ns(99.0), 0);
  EXPECT_EQ(h->max_ns(), 0);
  EXPECT_EQ(h->min_ns(), 0);
  EXPECT_DOUBLE_EQ(h->mean_ns(), 0.0);
}

TEST(LatencyHistogram, SingleSampleIsEveryPercentile) {
  auto h = make();
  h->record(12'345);  // 12.345 us -> bucket [12,13) us, but capped at true max
  EXPECT_EQ(h->percentile_ns(0), 12'345);
  EXPECT_EQ(h->percentile_ns(50), 12'345);
  EXPECT_EQ(h->percentile_ns(100), 12'345);
  EXPECT_EQ(h->max_ns(), 12'345);
  EXPECT_EQ(h->min_ns(), 12'345);
}

TEST(LatencyHistogram, NearestRankOnUniformData) {
  auto h = make();
  // Samples at 0.5, 1.5, ..., 999.5 us: exactly one per 1 us bucket.
  for (int i = 0; i < 1000; ++i) h->record(i * 1000 + 500);
  ASSERT_EQ(h->count(), 1000u);
  // Nearest rank: p50 -> 500th sample (499.5 us) -> bucket upper edge 500 us.
  EXPECT_EQ(h->percentile_ns(50), 500'000);
  EXPECT_EQ(h->percentile_ns(99), 990'000);
  EXPECT_EQ(h->percentile_ns(99.9), 999'000);
  EXPECT_EQ(h->percentile_ns(100), 999'500);  // capped at exact max
  EXPECT_NEAR(h->mean_ns(), 500'000.0, 1.0);
}

TEST(LatencyHistogram, PercentileIsConservativeUpperEdge) {
  auto h = make();
  for (int i = 0; i < 100; ++i) h->record(i < 99 ? 2'100 : 7'900);
  // p99 lands in the [2,3) us bucket -> reported as 3 us, never under-reported.
  EXPECT_EQ(h->percentile_ns(99), 3'000);
  EXPECT_EQ(h->percentile_ns(99.9), 7'900);
}

TEST(LatencyHistogram, OverflowKeepsExactMax) {
  auto h = make();
  for (int i = 0; i < 9; ++i) h->record(1'000);
  h->record(25'000'000);  // 25 ms, beyond the tracked range
  EXPECT_EQ(h->overflow_count(), 1u);
  EXPECT_EQ(h->max_ns(), 25'000'000);
  EXPECT_EQ(h->percentile_ns(100), 25'000'000);
  EXPECT_EQ(h->percentile_ns(90), 2'000);
}

TEST(LatencyHistogram, NegativeSamplesAreClampedAndCounted) {
  auto h = make();
  h->record(-500);
  EXPECT_EQ(h->negative_count(), 1u);
  EXPECT_EQ(h->min_ns(), 0);
  EXPECT_EQ(h->count(), 1u);
}

TEST(LatencyHistogram, ResetClearsEverything) {
  auto h = make();
  h->record(5'000);
  h->record(-1);
  h->reset();
  EXPECT_EQ(h->count(), 0u);
  EXPECT_EQ(h->negative_count(), 0u);
  EXPECT_EQ(h->percentile_ns(50), 0);
}
